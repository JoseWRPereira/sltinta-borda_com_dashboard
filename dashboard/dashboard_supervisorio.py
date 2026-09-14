"""
=============================================================================
SISTEMA SUPERVISORIO - CNC - SIDERURGICA - Baralds Steelworks
=============================================================================
Dashboard em Python (Tkinter + Matplotlib) para monitoramento de vibracao,
temperatura e rotacao, com tres fontes de dados selecionaveis:

  1) Aquisicao via MODBUS RTU serial (Arduino Uno atuando como ESCRAVO)
  2) Reproducao de um arquivo CSV previamente exportado
  3) Geracao de dados aleatorios (simulacao), com media e desvio padrao
     configuraveis por variavel — util para testar a interface sem hardware

Este dashboard implementa um CLIENTE (MESTRE) MODBUS RTU do zero, sobre a
mesma porta serial usada para o link fisico com o Arduino. Sao suportadas
as 8 funcoes classicas do protocolo:

  0x01  Read Coils                    (leitura de bits R/W)
  0x02  Read Discrete Inputs          (leitura de bits somente-leitura)
  0x03  Read Holding Registers        (leitura de registradores R/W)
  0x04  Read Input Registers          (leitura de registradores somente-leitura)
  0x05  Write Single Coil             (escrita de 1 bit)
  0x06  Write Single Register         (escrita de 1 registrador)
  0x0F  Write Multiple Coils          (escrita de N bits)
  0x10  Write Multiple Registers      (escrita de N registradores)

Todas estao implementadas e disponiveis na classe ModbusRTUMaster.

MAPA DE MEMORIA MODBUS (deve copiar o mapa do escravo):

  COILS (bits R/W) - FC 01/05/0F
    0: Saida digital 1 (reservada p/ expansao)
    1: Saida digital 2 (reservada p/ expansao)

  DISCRETE INPUTS (bits, somente leitura) - FC 02
    0: Status da temperatura do motor
    1: Status do nivel do lubrificante

  INPUT REGISTERS (16 bits, somente leitura) - FC 04
    0: Vibracao     x100 (mm/s * 100)
    1: Temperatura  x100 (°C * 100)
    2: Rotacao      x10  (RPM * 10)

  HOLDING REGISTERS (16 bits, R/W) - FC 03/06/10
    0: Limite ALERTA  vibracao     x100
    1: Limite CRITICO vibracao     x100
    2: Limite ALERTA  temperatura  x100
    3: Limite CRITICO temperatura  x100
    4: Limite ALERTA  rotacao      x10
    5: Limite CRITICO rotacao      x10

Sinais digitais:
  - Status de temperatura do Motor
  - Alarme de nivel de lubrificante

Funcionalidades:
  - Valor atual, media movel e historico grafico de cada variavel analogica
  - Indicadores luminosos (LED) para os sinais digitais
  - Alarmes visuais quando os limites configurados sao ultrapassados
  - Menu "Fonte de Dados" para alternar entre Modbus Serial / CSV / Aleatorio
  - Sincronizacao dos limites de alarme com o Arduino via Modbus (FC10)
  - Teste manual das coils (saidas digitais) via Modbus (FC01/05/0F)
  - Exportacao do historico para CSV
  - Cada fonte roda em thread separada (nao trava a interface grafica)

Dependencias:
  pip install pyserial matplotlib

Uso:
  python dashboard_supervisorio.py
=============================================================================
"""

import csv
import random
import struct
import threading
import queue
import time
from collections import deque
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import serial
    import serial.tools.list_ports
    SERIAL_DISPONIVEL = True
except ImportError:
    SERIAL_DISPONIVEL = False


# ============================== CONFIGURACOES ===============================

MODBUS_BAUDRATE = 9600            # baud classico do padrao Modbus RTU
MODBUS_SLAVE_ID_PADRAO = 1
MODBUS_TIMEOUT_S = 1.0

TAMANHO_HISTORICO = 300          # numero de pontos exibidos no grafico
JANELA_MEDIA_MOVEL = 10          # amostras usadas na media movel
INTERVALO_MS = 300               # periodo padrao de amostragem (Modbus/CSV/simulacao)

LIMITES = {
    "vibracao":    {"alerta": 7.0,  "critico": 12.0},   # mm/s
    "temperatura": {"alerta": 450.0, "critico": 520.0},  # °C
    "rotacao":     {"alerta": 2600.0, "critico": 2850.0},  # RPM
}

# Enderecos e fatores de escala dos Holding Registers de limite de alarme.
# A ORDEM desta lista e o formato esperado pelo firmware do Arduino (FC10
# escreve os 6 registradores de uma vez, comecando no endereco 0).
MAPA_LIMITES_HOLDING = [
    ("vibracao", "alerta", 100),
    ("vibracao", "critico", 100),
    ("temperatura", "alerta", 100),
    ("temperatura", "critico", 100),
    ("rotacao", "alerta", 10),
    ("rotacao", "critico", 10),
]

# Enderecos Modbus fixos usados pelo poll principal (devem bater com o .ino)
END_INPUT_REGISTERS = 0      # vibracao, temperatura, rotacao (3 registradores)
QTD_INPUT_REGISTERS = 3
ESCALA_INPUT_REGISTERS = [100.0, 100.0, 10.0]   # vibracao, temperatura, rotacao

END_DISCRETE_INPUTS = 0      # temp_motor, nivel_lubri (2 bits)
QTD_DISCRETE_INPUTS = 2

END_HOLDING_LIMITES = 0      # 6 registradores de limite (ver MAPA_LIMITES_HOLDING)
QTD_HOLDING_LIMITES = 6

END_COILS = 0                # 2 coils reservadas p/ expansao (saidas digitais)
QTD_COILS = 2

# Parametros default da fonte "dados aleatorios" (media e desvio padrao)
PARAMS_SIMULACAO_PADRAO = {
    "vibracao":    {"media": 4.0,   "desvio": 1.2},
    "temperatura": {"media": 320.0, "desvio": 25.0},
    "rotacao":     {"media": 1800.0, "desvio": 80.0},
    "prob_temp_motor_ok": 0.95,     # probabilidade de temp_motor = 1 (acesa) a cada ciclo
    "prob_nivel_ok": 0.97,     # probabilidade de nivel do lubrificante = 1 (OK) a cada ciclo
}

COR_OK = "#1b998b"
COR_ALERTA = "#f4a259"
COR_CRITICO = "#e63946"
COR_FUNDO = "#101820"
COR_PAINEL = "#182634"
COR_TEXTO = "#e8f1f2"


# =============================== NUCLEO DE DADOS =============================

class CanalDados:
    """Mantem o historico e a media movel de uma variavel analogica."""

    def __init__(self, nome, unidade, limites):
        self.nome = nome
        self.unidade = unidade
        self.limites = limites
        self.historico = deque(maxlen=TAMANHO_HISTORICO)
        self.tempos = deque(maxlen=TAMANHO_HISTORICO)
        self.janela_movel = deque(maxlen=JANELA_MEDIA_MOVEL)
        self.valor_atual = 0.0
        self.media_movel = 0.0

    def adicionar(self, valor, t):
        self.valor_atual = valor
        self.janela_movel.append(valor)
        self.media_movel = sum(self.janela_movel) / len(self.janela_movel)
        self.historico.append(valor)
        self.tempos.append(t)

    def status(self):
        if self.valor_atual >= self.limites["critico"]:
            return "CRITICO", COR_CRITICO
        if self.valor_atual >= self.limites["alerta"]:
            return "ALERTA", COR_ALERTA
        return "NORMAL", COR_OK


# ============================ CLIENTE (MESTRE) MODBUS RTU =====================
#
# Implementacao "from scratch" do protocolo Modbus RTU sobre pyserial, sem
# depender de bibliotecas de terceiros. Cobre as 8 funcoes classicas de
# leitura/escrita de bits e registradores, unica e multipla.

MODBUS_EXCECOES = {
    0x01: "Funcao ilegal",
    0x02: "Endereco de dado ilegal",
    0x03: "Valor de dado ilegal",
    0x04: "Falha no dispositivo escravo",
}


class ModbusError(Exception):
    """Classe base para erros de comunicacao Modbus."""


class ModbusTimeoutError(ModbusError):
    pass


class ModbusCRCError(ModbusError):
    pass


class ModbusExceptionError(ModbusError):
    def __init__(self, function_code, exception_code):
        self.function_code = function_code
        self.exception_code = exception_code
        descricao = MODBUS_EXCECOES.get(exception_code, "desconhecida")
        super().__init__(
            f"Excecao Modbus na funcao 0x{function_code:02X}: "
            f"codigo {exception_code} ({descricao})"
        )


class ModbusRTUMaster:
    """
    Cliente (mestre) Modbus RTU. Recebe uma conexao pyserial ja aberta e
    monta/interpreta os quadros manualmente (endereco, funcao, dados, CRC16).

    Thread-safe: um lock interno serializa pedidos vindos de threads
    diferentes (ex: a thread de polling e a GUI escrevendo limites ao
    mesmo tempo), garantindo que nunca haja duas transacoes Modbus
    concorrentes no mesmo barramento serial.
    """

    def __init__(self, conexao_serial, timeout=MODBUS_TIMEOUT_S):
        self.ser = conexao_serial
        self.timeout = timeout
        self._lock = threading.Lock()

    # ---------------------- CRC16 Modbus ----------------------
    @staticmethod
    def _crc16(dados: bytes) -> int:
        crc = 0xFFFF
        for byte in dados:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    # ---------------------- Envio / recepcao de quadro ----------------------
    def _enviar(self, slave_id, function_code, pdu: bytes):
        quadro = bytes([slave_id, function_code]) + pdu
        crc = self._crc16(quadro)
        quadro_completo = quadro + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        self.ser.reset_input_buffer()
        self.ser.write(quadro_completo)

    def _ler_bytes(self, n):
        self.ser.timeout = self.timeout
        dados = self.ser.read(n)
        return dados if len(dados) == n else None

    def _verificar_crc(self, quadro: bytes):
        corpo, crc_recebido = quadro[:-2], quadro[-2:]
        crc_calculado = self._crc16(corpo)
        crc_recebido_int = crc_recebido[0] | (crc_recebido[1] << 8)
        if crc_calculado != crc_recebido_int:
            raise ModbusCRCError("CRC invalido na resposta do escravo.")

    def _receber_resposta(self, com_byte_count: bool):
        """
        Le a resposta do escravo. Se com_byte_count=True, espera o formato
        variavel das leituras (addr+func+byteCount+dados+crc). Caso
        contrario, espera o formato fixo das escritas (addr+func+4 bytes+crc).
        Retorna os bytes de dados uteis (sem endereco/funcao/crc).
        """
        cabecalho = self._ler_bytes(2)
        if cabecalho is None:
            raise ModbusTimeoutError("Sem resposta do escravo (timeout).")
        _endereco, func_resp = cabecalho[0], cabecalho[1]

        if func_resp & 0x80:
            resto = self._ler_bytes(3)  # codigo de excecao + CRC(2)
            if resto is None:
                raise ModbusTimeoutError("Resposta de excecao incompleta.")
            quadro_completo = cabecalho + resto
            self._verificar_crc(quadro_completo)
            raise ModbusExceptionError(func_resp & 0x7F, resto[0])

        if com_byte_count:
            byte_count_b = self._ler_bytes(1)
            if byte_count_b is None:
                raise ModbusTimeoutError("Resposta incompleta (byte count).")
            byte_count = byte_count_b[0]
            dados = self._ler_bytes(byte_count)
            crc_bytes = self._ler_bytes(2)
            if dados is None or crc_bytes is None:
                raise ModbusTimeoutError("Resposta incompleta (dados/CRC).")
            quadro_completo = cabecalho + byte_count_b + dados + crc_bytes
            self._verificar_crc(quadro_completo)
            return dados
        else:
            resto = self._ler_bytes(6)  # 4 bytes de dados + CRC(2)
            if resto is None:
                raise ModbusTimeoutError("Resposta incompleta (escrita).")
            quadro_completo = cabecalho + resto
            self._verificar_crc(quadro_completo)
            return resto[:-2]

    @staticmethod
    def _bytes_para_bits(dados: bytes, quantidade: int):
        bits = []
        for i in range(quantidade):
            byte_idx, bit_idx = divmod(i, 8)
            bits.append(bool(dados[byte_idx] & (1 << bit_idx)))
        return bits

    # ============================================================
    # FC 0x01 - Read Coils
    # ============================================================
    def read_coils(self, slave_id, address, count):
        with self._lock:
            pdu = struct.pack(">HH", address, count)
            self._enviar(slave_id, 0x01, pdu)
            dados = self._receber_resposta(com_byte_count=True)
        return self._bytes_para_bits(dados, count)

    # ============================================================
    # FC 0x02 - Read Discrete Inputs
    # ============================================================
    def read_discrete_inputs(self, slave_id, address, count):
        with self._lock:
            pdu = struct.pack(">HH", address, count)
            self._enviar(slave_id, 0x02, pdu)
            dados = self._receber_resposta(com_byte_count=True)
        return self._bytes_para_bits(dados, count)

    # ============================================================
    # FC 0x03 - Read Holding Registers
    # ============================================================
    def read_holding_registers(self, slave_id, address, count):
        with self._lock:
            pdu = struct.pack(">HH", address, count)
            self._enviar(slave_id, 0x03, pdu)
            dados = self._receber_resposta(com_byte_count=True)
        return list(struct.unpack(f">{count}H", dados))

    # ============================================================
    # FC 0x04 - Read Input Registers
    # ============================================================
    def read_input_registers(self, slave_id, address, count):
        with self._lock:
            pdu = struct.pack(">HH", address, count)
            self._enviar(slave_id, 0x04, pdu)
            dados = self._receber_resposta(com_byte_count=True)
        return list(struct.unpack(f">{count}H", dados))

    # ============================================================
    # FC 0x05 - Write Single Coil
    # ============================================================
    def write_single_coil(self, slave_id, address, valor: bool):
        with self._lock:
            valor_modbus = 0xFF00 if valor else 0x0000
            pdu = struct.pack(">HH", address, valor_modbus)
            self._enviar(slave_id, 0x05, pdu)
            self._receber_resposta(com_byte_count=False)
        return True

    # ============================================================
    # FC 0x06 - Write Single Register
    # ============================================================
    def write_single_register(self, slave_id, address, valor: int):
        with self._lock:
            pdu = struct.pack(">HH", address, valor & 0xFFFF)
            self._enviar(slave_id, 0x06, pdu)
            self._receber_resposta(com_byte_count=False)
        return True

    # ============================================================
    # FC 0x0F - Write Multiple Coils
    # ============================================================
    def write_multiple_coils(self, slave_id, address, valores):
        with self._lock:
            quantidade = len(valores)
            byte_count = (quantidade + 7) // 8
            bytes_dados = bytearray(byte_count)
            for i, v in enumerate(valores):
                if v:
                    bytes_dados[i // 8] |= (1 << (i % 8))
            pdu = struct.pack(">HHB", address, quantidade, byte_count) + bytes(bytes_dados)
            self._enviar(slave_id, 0x0F, pdu)
            self._receber_resposta(com_byte_count=False)
        return True

    # ============================================================
    # FC 0x10 - Write Multiple Registers
    # ============================================================
    def write_multiple_registers(self, slave_id, address, valores):
        with self._lock:
            quantidade = len(valores)
            pdu = struct.pack(">HHB", address, quantidade, quantidade * 2)
            for v in valores:
                pdu += struct.pack(">H", v & 0xFFFF)
            self._enviar(slave_id, 0x10, pdu)
            self._receber_resposta(com_byte_count=False)
        return True


# ============================ FONTES DE DADOS =================================
#
# As tres classes abaixo compartilham a mesma interface: rodam em thread propria
# e colocam tuplas ("dados", {vibracao, temperatura, rotacao, temp_motor, nivel_lubri})
# ou ("erro"/"info", mensagem) na fila da GUI. Isso permite trocar a fonte sem
# alterar nenhum codigo do dashboard/graficos.

class FonteDadosBase(threading.Thread):
    """Classe base: controla o ciclo de parada comum a todas as fontes."""

    def __init__(self, fila_saida):
        super().__init__(daemon=True)
        self.fila_saida = fila_saida
        self._rodando = threading.Event()
        self._rodando.set()

    def parar(self):
        self._rodando.clear()


class LeitorModbusRTU(FonteDadosBase):
    """
    Fonte de dados real: le periodicamente o Arduino (escravo Modbus RTU)
    via Input Registers (medidas) e Discrete Inputs (sinais digitais).

    Expoe self.mestre (ModbusRTUMaster) publicamente para que a GUI possa
    fazer leituras/escritas avulsas (ex: sincronizar limites via Holding
    Registers, testar coils) na mesma conexao, de forma thread-safe.
    """

    def __init__(self, porta, baudrate, slave_id, fila_saida, intervalo_ms=INTERVALO_MS):
        super().__init__(fila_saida)
        self.porta = porta
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.intervalo_ms = intervalo_ms
        self.conexao = None
        self.mestre = None

    def run(self):
        try:
            self.conexao = serial.Serial(self.porta, self.baudrate, timeout=MODBUS_TIMEOUT_S)
            time.sleep(2)  # aguarda reset do Arduino ao abrir a porta
            self.mestre = ModbusRTUMaster(self.conexao, timeout=MODBUS_TIMEOUT_S)
        except Exception as e:
            self.fila_saida.put(("erro", f"Falha ao abrir porta serial: {e}"))
            return

        while self._rodando.is_set():
            try:
                brutos = self.mestre.read_input_registers(
                    self.slave_id, END_INPUT_REGISTERS, QTD_INPUT_REGISTERS
                )
                bits = self.mestre.read_discrete_inputs(
                    self.slave_id, END_DISCRETE_INPUTS, QTD_DISCRETE_INPUTS
                )
                dados = {
                    "vibracao": brutos[0] / ESCALA_INPUT_REGISTERS[0],
                    "temperatura": brutos[1] / ESCALA_INPUT_REGISTERS[1],
                    "rotacao": brutos[2] / ESCALA_INPUT_REGISTERS[2],
                    "temp_motor": bits[0],
                    "nivel_lubri": bits[1],
                }
                self.fila_saida.put(("dados", dados))
            except ModbusError as e:
                self.fila_saida.put(("erro", f"Modbus: {e}"))
            except Exception as e:
                self.fila_saida.put(("erro", f"Erro na comunicacao serial: {e}"))
                break

            time.sleep(self.intervalo_ms / 1000.0)

    def parar(self):
        super().parar()
        if self.conexao and self.conexao.is_open:
            self.conexao.close()


class LeitorCSV(FonteDadosBase):
    """
    Reproduz um historico previamente exportado (mesmo formato gerado pela
    opcao Exportar historico do menu Arquivo):
      timestamp, vibracao_mms, temperatura_c, rotacao_rpm, temp_motor, nivel_lubri

    O arquivo e lido inteiramente na memoria e reproduzido em loop, respeitando
    um intervalo fixo entre linhas (nao usa o timestamp gravado, para permitir
    reproduzir mais rapido ou mais devagar via 'intervalo_ms').
    """

    def __init__(self, caminho_csv, fila_saida, intervalo_ms=INTERVALO_MS, repetir=True):
        super().__init__(fila_saida)
        self.caminho_csv = caminho_csv
        self.intervalo_ms = intervalo_ms
        self.repetir = repetir
        self.linhas = []

    def _carregar(self):
        with open(self.caminho_csv, "r", encoding="utf-8") as f:
            leitor = csv.DictReader(f)
            for linha in leitor:
                try:
                    self.linhas.append({
                        "vibracao": float(linha["vibracao_mms"]),
                        "temperatura": float(linha["temperatura_c"]),
                        "rotacao": float(linha["rotacao_rpm"]),
                        "temp_motor": bool(int(linha["temp_motor"])),
                        "nivel_lubri": bool(int(linha["nivel_lubri"])),
                    })
                except (KeyError, ValueError):
                    continue  # ignora linhas mal formatadas

    def run(self):
        try:
            self._carregar()
        except Exception as e:
            self.fila_saida.put(("erro", f"Falha ao ler CSV: {e}"))
            return

        if not self.linhas:
            self.fila_saida.put(("erro", "Arquivo CSV vazio ou em formato invalido."))
            return

        while self._rodando.is_set():
            for linha in self.linhas:
                if not self._rodando.is_set():
                    break
                self.fila_saida.put(("dados", linha))
                time.sleep(self.intervalo_ms / 1000.0)
            if not self.repetir:
                break
        self.fila_saida.put(("info", "Reproducao do CSV finalizada."))


class GeradorAleatorio(FonteDadosBase):
    """
    Gera dados simulados usando distribuicao normal (media e desvio padrao
    configuraveis por variavel), util para testar o dashboard sem hardware.
    """

    def __init__(self, fila_saida, parametros, intervalo_ms=INTERVALO_MS):
        super().__init__(fila_saida)
        self.parametros = parametros
        self.intervalo_ms = intervalo_ms

    def run(self):
        while self._rodando.is_set():
            p = self.parametros
            vibracao = max(0.0, random.gauss(p["vibracao"]["media"], p["vibracao"]["desvio"]))
            temperatura = max(0.0, random.gauss(p["temperatura"]["media"], p["temperatura"]["desvio"]))
            rotacao = max(0.0, random.gauss(p["rotacao"]["media"], p["rotacao"]["desvio"]))
            temp_motor = random.random() < p["prob_temp_motor_ok"]
            nivel_lubri = random.random() < p["prob_nivel_ok"]

            self.fila_saida.put(("dados", {
                "vibracao": vibracao,
                "temperatura": temperatura,
                "rotacao": rotacao,
                "temp_motor": temp_motor,
                "nivel_lubri": nivel_lubri,
            }))
            time.sleep(self.intervalo_ms / 1000.0)


# ================================ APLICACAO ===================================

class DashboardSupervisorio(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Sistema Supervisorio - Baraldis Steelworks")
        self.geometry("1180x720")
        self.configure(bg=COR_FUNDO)
        self.minsize(1000, 650)

        self.fila = queue.Queue()
        self.fonte_atual = None                 # thread da fonte de dados ativa
        self.descricao_fonte = tk.StringVar(value="Nenhuma fonte ativa")
        self.tempo_inicio = time.time()

        self.params_simulacao = {
            "vibracao": dict(PARAMS_SIMULACAO_PADRAO["vibracao"]),
            "temperatura": dict(PARAMS_SIMULACAO_PADRAO["temperatura"]),
            "rotacao": dict(PARAMS_SIMULACAO_PADRAO["rotacao"]),
            "prob_temp_motor_ok": PARAMS_SIMULACAO_PADRAO["prob_temp_motor_ok"],
            "prob_nivel_ok": PARAMS_SIMULACAO_PADRAO["prob_nivel_ok"],
        }

        self.canais = {
            "vibracao": CanalDados("Vibracao", "mm/s", LIMITES["vibracao"]),
            "temperatura": CanalDados("Temperatura", "°C", LIMITES["temperatura"]),
            "rotacao": CanalDados("Rotacao", "RPM", LIMITES["rotacao"]),
        }
        self.temp_motor_ok = False
        self.nivel_ok = False
        self.registro_completo = []  # para exportacao CSV

        self._montar_menu()
        self._montar_layout()
        self._atualizar_gui()

    # ------------------------------ MENU ------------------------------
    def _montar_menu(self):
        menubar = tk.Menu(self)

        # ---- Menu Arquivo ----
        menu_arquivo = tk.Menu(menubar, tearoff=0)
        menu_arquivo.add_command(label="Exportar historico (CSV)", command=self._exportar_csv)
        menu_arquivo.add_separator()
        menu_arquivo.add_command(label="Sair", command=self._fechar)
        menubar.add_cascade(label="Arquivo", menu=menu_arquivo)

        # ---- Menu Fonte de Dados ----
        menu_fonte = tk.Menu(menubar, tearoff=0)
        menu_fonte.add_command(label="Modbus RTU (Arduino)...", command=self._dialogo_fonte_modbus)
        menu_fonte.add_command(label="Arquivo CSV...", command=self._dialogo_fonte_csv)
        menu_fonte.add_command(label="Dados aleatorios (simulacao)...", command=self._dialogo_fonte_aleatoria)
        menu_fonte.add_separator()
        menu_fonte.add_command(label="Parar fonte atual", command=self._parar_fonte)
        menubar.add_cascade(label="Fonte de Dados", menu=menu_fonte)

        # ---- Menu Configuracoes ----
        menu_config = tk.Menu(menubar, tearoff=0)
        menu_config.add_command(label="Definir limites de alarme...", command=self._dialogo_limites)
        menu_config.add_command(label="Testar saidas digitais (Coils Modbus)...", command=self._dialogo_coils)
        menubar.add_cascade(label="Configuracoes", menu=menu_config)

        # ---- Menu Ajuda ----
        menu_ajuda = tk.Menu(menubar, tearoff=0)
        menu_ajuda.add_command(label="Sobre", command=self._sobre)
        menubar.add_cascade(label="Ajuda", menu=menu_ajuda)

        self.config(menu=menubar)

    # ---------------------- Selecao de fonte: Modbus RTU (Serial) ----------------------
    def _dialogo_fonte_modbus(self):
        if not SERIAL_DISPONIVEL:
            messagebox.showerror("Erro", "pyserial nao esta instalado.\nExecute: pip install pyserial")
            return

        portas = [p.device for p in serial.tools.list_ports.comports()]
        if not portas:
            messagebox.showwarning("Aviso", "Nenhuma porta serial encontrada.")
            return

        janela = tk.Toplevel(self)
        janela.title("Conectar - Modbus RTU (Arduino)")
        janela.configure(bg=COR_PAINEL)
        janela.geometry("320x230")
        janela.resizable(False, False)

        tk.Label(janela, text="Porta serial:", bg=COR_PAINEL, fg=COR_TEXTO).pack(pady=(12, 2))
        combo = ttk.Combobox(janela, values=portas, state="readonly")
        combo.pack()
        if portas:
            combo.current(0)

        tk.Label(janela, text="Baudrate:", bg=COR_PAINEL, fg=COR_TEXTO).pack(pady=(10, 2))
        e_baud = tk.Entry(janela, width=10)
        e_baud.insert(0, str(MODBUS_BAUDRATE))
        e_baud.pack()

        tk.Label(janela, text="ID do escravo Modbus:", bg=COR_PAINEL, fg=COR_TEXTO).pack(pady=(10, 2))
        e_slave = tk.Entry(janela, width=10)
        e_slave.insert(0, str(MODBUS_SLAVE_ID_PADRAO))
        e_slave.pack()

        def confirmar():
            porta = combo.get()
            if not porta:
                return
            try:
                baud = int(e_baud.get())
                slave_id = int(e_slave.get())
            except ValueError:
                messagebox.showerror("Erro", "Baudrate e ID do escravo devem ser numeros inteiros.")
                return
            janela.destroy()
            self._parar_fonte()
            fonte = LeitorModbusRTU(porta, baud, slave_id, self.fila)
            self._iniciar_fonte(fonte, f"Modbus RTU: {porta} @ {baud} (escravo {slave_id})")

        tk.Button(janela, text="Conectar", command=confirmar).pack(pady=16)

    # ---------------------- Selecao de fonte: CSV ----------------------
    def _dialogo_fonte_csv(self):
        caminho = filedialog.askopenfilename(
            title="Selecionar arquivo CSV",
            filetypes=[("CSV", "*.csv")],
        )
        if not caminho:
            return

        intervalo = simpledialog.askinteger(
            "Intervalo de reproducao",
            "Intervalo entre amostras (ms):",
            initialvalue=INTERVALO_MS, minvalue=10, maxvalue=10000,
        )
        if intervalo is None:
            intervalo = INTERVALO_MS

        repetir = messagebox.askyesno("Repetir", "Reproduzir o arquivo em loop continuo?")

        self._parar_fonte()
        fonte = LeitorCSV(caminho, self.fila, intervalo_ms=intervalo, repetir=repetir)
        nome_arquivo = caminho.split("/")[-1].split("\\")[-1]
        self._iniciar_fonte(fonte, f"CSV: {nome_arquivo}")

    # ------------------- Selecao de fonte: Dados aleatorios -------------------
    def _dialogo_fonte_aleatoria(self):
        janela = tk.Toplevel(self)
        janela.title("Simulacao - Dados Aleatorios")
        janela.configure(bg=COR_PAINEL)
        janela.geometry("420x420")
        janela.resizable(False, False)

        entradas = {}

        def linha_variavel(pai, chave, titulo, unidade, linha_idx):
            tk.Label(pai, text=f"{titulo} ({unidade})", bg=COR_PAINEL, fg=COR_TEXTO,
                     font=("Segoe UI", 10, "bold")).grid(row=linha_idx, column=0, columnspan=2,
                                                          sticky="w", padx=10, pady=(10, 2))
            tk.Label(pai, text="Media:", bg=COR_PAINEL, fg="#8fa3b3").grid(
                row=linha_idx + 1, column=0, sticky="e", padx=(10, 4))
            e_media = tk.Entry(pai, width=10)
            e_media.insert(0, str(self.params_simulacao[chave]["media"]))
            e_media.grid(row=linha_idx + 1, column=1, sticky="w")

            tk.Label(pai, text="Desvio padrao:", bg=COR_PAINEL, fg="#8fa3b3").grid(
                row=linha_idx + 2, column=0, sticky="e", padx=(10, 4))
            e_desvio = tk.Entry(pai, width=10)
            e_desvio.insert(0, str(self.params_simulacao[chave]["desvio"]))
            e_desvio.grid(row=linha_idx + 2, column=1, sticky="w")

            entradas[chave] = {"media": e_media, "desvio": e_desvio}

        linha_variavel(janela, "vibracao", "Vibracao", "mm/s", 0)
        linha_variavel(janela, "temperatura", "Temperatura", "°C", 3)
        linha_variavel(janela, "rotacao", "Rotacao", "RPM", 6)

        tk.Label(janela, text="Probabilidade de temp. Motor (0-1):", bg=COR_PAINEL, fg="#8fa3b3").grid(
            row=9, column=0, sticky="e", padx=(10, 4), pady=(14, 2))
        e_temp_motor = tk.Entry(janela, width=10)
        e_temp_motor.insert(0, str(self.params_simulacao["prob_temp_motor_ok"]))
        e_temp_motor.grid(row=9, column=1, sticky="w", pady=(14, 2))

        tk.Label(janela, text="Probabilidade de nivel OK (0-1):", bg=COR_PAINEL, fg="#8fa3b3").grid(
            row=10, column=0, sticky="e", padx=(10, 4), pady=2)
        e_nivel = tk.Entry(janela, width=10)
        e_nivel.insert(0, str(self.params_simulacao["prob_nivel_ok"]))
        e_nivel.grid(row=10, column=1, sticky="w", pady=2)

        tk.Label(janela, text="Intervalo entre amostras (ms):", bg=COR_PAINEL, fg="#8fa3b3").grid(
            row=11, column=0, sticky="e", padx=(10, 4), pady=2)
        e_intervalo = tk.Entry(janela, width=10)
        e_intervalo.insert(0, str(INTERVALO_MS))
        e_intervalo.grid(row=11, column=1, sticky="w", pady=2)

        def iniciar():
            try:
                for chave in ["vibracao", "temperatura", "rotacao"]:
                    self.params_simulacao[chave]["media"] = float(entradas[chave]["media"].get())
                    self.params_simulacao[chave]["desvio"] = float(entradas[chave]["desvio"].get())
                self.params_simulacao["prob_temp_motor_ok"] = min(1.0, max(0.0, float(e_temp_motor.get())))
                self.params_simulacao["prob_nivel_ok"] = min(1.0, max(0.0, float(e_nivel.get())))
                intervalo = max(10, int(e_intervalo.get()))
            except ValueError:
                messagebox.showerror("Erro", "Valores invalidos. Use numeros (ex: 4.0, 1.2).")
                return

            janela.destroy()
            self._parar_fonte()
            fonte = GeradorAleatorio(self.fila, self.params_simulacao, intervalo_ms=intervalo)
            self._iniciar_fonte(fonte, "Simulacao (dados aleatorios)")

        tk.Button(janela, text="Iniciar simulacao", command=iniciar).grid(
            row=12, column=0, columnspan=2, pady=20)

    # ---------------------- Controle generico de fonte ----------------------
    def _iniciar_fonte(self, fonte_thread, descricao):
        self.fonte_atual = fonte_thread
        self.fonte_atual.start()
        self.descricao_fonte.set(descricao)
        self.lbl_status_conexao.config(text=descricao, fg=COR_OK)
        self.lbl_status_geral.config(text=f"Fonte ativa: {descricao}")

    def _parar_fonte(self):
        if self.fonte_atual:
            self.fonte_atual.parar()
            self.fonte_atual = None
        self.descricao_fonte.set("Nenhuma fonte ativa")
        if hasattr(self, "lbl_status_conexao"):
            self.lbl_status_conexao.config(text=self.descricao_fonte.get(), fg=COR_TEXTO)

    def _fonte_modbus_ativa(self):
        """Retorna a fonte Modbus ativa (com mestre pronto) ou None."""
        if isinstance(self.fonte_atual, LeitorModbusRTU) and self.fonte_atual.mestre is not None:
            return self.fonte_atual
        return None

    def _exportar_csv(self):
        if not self.registro_completo:
            messagebox.showinfo("Exportar", "Ainda nao ha dados para exportar.")
            return
        caminho = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
            initialfile="historico.csv",
        )
        if not caminho:
            return
        with open(caminho, "w", newline="", encoding="utf-8") as f:
            escritor = csv.writer(f)
            escritor.writerow(["timestamp", "vibracao_mms", "temperatura_c", "rotacao_rpm", "temp_motor", "nivel_lubri"])
            escritor.writerows(self.registro_completo)
        messagebox.showinfo("Exportar", f"Historico exportado para:\n{caminho}")

    def _dialogo_limites(self):
        canal = simpledialog.askstring(
            "Configurar limites",
            "Qual variavel? (vibracao / temperatura / rotacao)"
        )
        if canal not in LIMITES:
            if canal is not None:
                messagebox.showerror("Erro", "Variavel invalida.")
            return
        alerta = simpledialog.askfloat("Limite de alerta", f"Novo limite de ALERTA para {canal}:",
                                        initialvalue=LIMITES[canal]["alerta"])
        critico = simpledialog.askfloat("Limite critico", f"Novo limite CRITICO para {canal}:",
                                         initialvalue=LIMITES[canal]["critico"])
        if alerta is not None and critico is not None:
            LIMITES[canal]["alerta"] = alerta
            LIMITES[canal]["critico"] = critico
            self.canais[canal].limites = LIMITES[canal]
            messagebox.showinfo("Configuracoes", f"Limites de {canal} atualizados.")

            fonte_modbus = self._fonte_modbus_ativa()
            if fonte_modbus and messagebox.askyesno(
                "Sincronizar com Arduino",
                "Enviar todos os limites atuais para os Holding Registers do "
                "Arduino agora, via Modbus (Write Multiple Registers)?"
            ):
                self._sincronizar_limites_modbus(fonte_modbus)

    def _sincronizar_limites_modbus(self, fonte_modbus):
        try:
            valores = []
            for variavel, tipo_limite, escala in MAPA_LIMITES_HOLDING:
                valores.append(int(round(LIMITES[variavel][tipo_limite] * escala)))
            fonte_modbus.mestre.write_multiple_registers(
                fonte_modbus.slave_id, END_HOLDING_LIMITES, valores
            )
            self.lbl_status_geral.config(text="Limites sincronizados com o Arduino via Modbus (FC10).")
        except ModbusError as e:
            messagebox.showerror("Erro Modbus", f"Falha ao sincronizar limites: {e}")

    def _dialogo_coils(self):
        fonte_modbus = self._fonte_modbus_ativa()
        if not fonte_modbus:
            messagebox.showinfo(
                "Coils Modbus",
                "Conecte-se a uma fonte Modbus RTU (Arduino) para testar as coils."
            )
            return

        janela = tk.Toplevel(self)
        janela.title("Testar saidas digitais (Coils)")
        janela.configure(bg=COR_PAINEL)
        janela.geometry("340x260")
        janela.resizable(False, False)

        tk.Label(janela, text="Coils reservadas para expansao futura\n"
                              "(ex: sirene, valvula, rele de saida)",
                 bg=COR_PAINEL, fg="#8fa3b3", justify="center").pack(pady=(14, 8))

        var_coil0 = tk.BooleanVar()
        var_coil1 = tk.BooleanVar()
        chk0 = tk.Checkbutton(janela, text="Coil 0 - Saida digital 1", variable=var_coil0,
                               bg=COR_PAINEL, fg=COR_TEXTO, selectcolor=COR_FUNDO)
        chk1 = tk.Checkbutton(janela, text="Coil 1 - Saida digital 2", variable=var_coil1,
                               bg=COR_PAINEL, fg=COR_TEXTO, selectcolor=COR_FUNDO)
        chk0.pack(anchor="w", padx=30, pady=4)
        chk1.pack(anchor="w", padx=30, pady=4)

        lbl_resultado = tk.Label(janela, text="", bg=COR_PAINEL, fg=COR_TEXTO, wraplength=300)
        lbl_resultado.pack(pady=(8, 4))

        def ler_estado():
            try:
                estados = fonte_modbus.mestre.read_coils(
                    fonte_modbus.slave_id, END_COILS, QTD_COILS
                )
                var_coil0.set(estados[0])
                var_coil1.set(estados[1])
                lbl_resultado.config(text=f"Estado lido (FC01): {estados}", fg=COR_OK)
            except ModbusError as e:
                lbl_resultado.config(text=f"Erro: {e}", fg=COR_CRITICO)

        def escrever_coil_unico(indice, variavel):
            try:
                fonte_modbus.mestre.write_single_coil(
                    fonte_modbus.slave_id, END_COILS + indice, variavel.get()
                )
                lbl_resultado.config(text=f"Coil {indice} escrita via FC05.", fg=COR_OK)
            except ModbusError as e:
                lbl_resultado.config(text=f"Erro: {e}", fg=COR_CRITICO)

        def escrever_ambas():
            try:
                fonte_modbus.mestre.write_multiple_coils(
                    fonte_modbus.slave_id, END_COILS, [var_coil0.get(), var_coil1.get()]
                )
                lbl_resultado.config(text="Coils 0 e 1 escritas juntas via FC0F.", fg=COR_OK)
            except ModbusError as e:
                lbl_resultado.config(text=f"Erro: {e}", fg=COR_CRITICO)

        frame_botoes = tk.Frame(janela, bg=COR_PAINEL)
        frame_botoes.pack(pady=6)
        tk.Button(frame_botoes, text="Ler estado (FC01)", command=ler_estado).grid(row=0, column=0, padx=4, pady=4)
        tk.Button(frame_botoes, text="Aplicar Coil 0 (FC05)",
                  command=lambda: escrever_coil_unico(0, var_coil0)).grid(row=1, column=0, padx=4, pady=4)
        tk.Button(frame_botoes, text="Aplicar Coil 1 (FC05)",
                  command=lambda: escrever_coil_unico(1, var_coil1)).grid(row=1, column=1, padx=4, pady=4)
        tk.Button(frame_botoes, text="Aplicar ambas (FC0F)", command=escrever_ambas).grid(
            row=2, column=0, columnspan=2, pady=(8, 0))

    def _sobre(self):
        messagebox.showinfo(
            "Sobre",
            "Baraldis Steelworks - Supervisório\n"
            "Monitoramento de vibracao, temperatura e rotacao.\n"
            "Protocolo: Modbus RTU serial (Arduino = escravo, Dashboard = mestre).\n"
            "Fontes de dados: Modbus RTU, arquivo CSV ou simulacao aleatoria.\n\n"
            "Desenvolvido para fins didaticos\n"
            "Prof. José W. R. Pereira\n"
            "IFSP Salto\n"
        )

    # ------------------------------ LAYOUT ------------------------------
    def _montar_layout(self):
        topo = tk.Frame(self, bg=COR_FUNDO)
        topo.pack(fill="x", padx=12, pady=(10, 4))

        tk.Label(topo, text="SISTEMA SUPERVISÓRIO — Baraldis Steelworks - CNC 101",
                 bg=COR_FUNDO, fg=COR_TEXTO, font=("Segoe UI", 16, "bold")).pack(side="left")

        self.lbl_status_conexao = tk.Label(topo, text=self.descricao_fonte.get(),
                                            bg=COR_FUNDO, fg=COR_TEXTO, font=("Segoe UI", 10))
        self.lbl_status_conexao.pack(side="right")

        # Painel de cartoes com valor atual + media movel
        painel_cartoes = tk.Frame(self, bg=COR_FUNDO)
        painel_cartoes.pack(fill="x", padx=12, pady=6)

        self.widgets_cartao = {}
        for chave in ["vibracao", "temperatura", "rotacao"]:
            cartao = self._criar_cartao(painel_cartoes, self.canais[chave])
            cartao.pack(side="left", expand=True, fill="both", padx=6)

        # Painel de sinais digitais (LEDs)
        painel_digital = tk.Frame(self, bg=COR_PAINEL, height=60)
        painel_digital.pack(fill="x", padx=12, pady=6)

        self.led_temp_motor = self._criar_led(painel_digital, "Temperatura do Motor")
        self.led_nivel = self._criar_led(painel_digital, "Nivel de Lubrificante")

        # Graficos de historico
        painel_graficos = tk.Frame(self, bg=COR_FUNDO)
        painel_graficos.pack(fill="both", expand=True, padx=12, pady=(4, 10))

        self.fig = Figure(figsize=(11, 5), dpi=100, facecolor=COR_FUNDO)
        self.eixos = {}
        nomes = [("vibracao", "Vibracao (mm/s)"), ("temperatura", "Temperatura (°C)"), ("rotacao", "Rotacao (RPM)")]
        for i, (chave, titulo) in enumerate(nomes, start=1):
            ax = self.fig.add_subplot(1, 3, i)
            ax.set_title(titulo, color=COR_TEXTO, fontsize=10)
            ax.set_facecolor("#0b1119")
            ax.tick_params(colors=COR_TEXTO, labelsize=8)
            for spine in ax.spines.values():
                spine.set_color("#3a4a5a")
            self.eixos[chave] = ax

        self.fig.tight_layout(pad=2.0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=painel_graficos)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Barra de status inferior
        self.lbl_status_geral = tk.Label(self, text="Selecione uma fonte de dados no menu 'Fonte de Dados'.",
                                          bg=COR_FUNDO, fg=COR_TEXTO, anchor="w", font=("Segoe UI", 9))
        self.lbl_status_geral.pack(fill="x", padx=14, pady=(0, 8))

    def _criar_cartao(self, pai, canal):
        frame = tk.Frame(pai, bg=COR_PAINEL, bd=0, relief="flat")
        tk.Label(frame, text=canal.nome.upper(), bg=COR_PAINEL, fg="#8fa3b3",
                 font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=14, pady=(10, 0))

        lbl_valor = tk.Label(frame, text="--", bg=COR_PAINEL, fg=COR_TEXTO, font=("Segoe UI", 30, "bold"))
        lbl_valor.pack(anchor="w", padx=14)

        lbl_unidade_media = tk.Label(frame, text=f"Media movel: -- {canal.unidade}",
                                      bg=COR_PAINEL, fg="#8fa3b3", font=("Segoe UI", 10))
        lbl_unidade_media.pack(anchor="w", padx=14, pady=(0, 6))

        lbl_status = tk.Label(frame, text="NORMAL", bg=COR_OK, fg="#08110d",
                               font=("Segoe UI", 9, "bold"), width=14)
        lbl_status.pack(anchor="w", padx=14, pady=(0, 12))

        self.widgets_cartao[canal.nome.lower()] = {
            "valor": lbl_valor,
            "media": lbl_unidade_media,
            "status": lbl_status,
        }
        return frame

    def _criar_led(self, pai, titulo):
        frame = tk.Frame(pai, bg=COR_PAINEL)
        frame.pack(side="left", padx=24, pady=10)
        canvas = tk.Canvas(frame, width=22, height=22, bg=COR_PAINEL, highlightthickness=0)
        circulo = canvas.create_oval(3, 3, 19, 19, fill="#555555", outline="")
        canvas.pack(side="left", padx=(0, 8))
        tk.Label(frame, text=titulo, bg=COR_PAINEL, fg=COR_TEXTO, font=("Segoe UI", 10)).pack(side="left")
        return {"canvas": canvas, "circulo": circulo}

    # --------------------------- LOOP DE ATUALIZACAO ---------------------------
    def _atualizar_gui(self):
        houve_dado_novo = False

        while not self.fila.empty():
            tipo, conteudo = self.fila.get_nowait()
            if tipo == "erro":
                self.lbl_status_geral.config(text=f"Erro na fonte de dados: {conteudo}")
            elif tipo == "info":
                self.lbl_status_geral.config(text=str(conteudo))
            elif tipo == "dados":
                t = time.time() - self.tempo_inicio
                self.canais["vibracao"].adicionar(conteudo["vibracao"], t)
                self.canais["temperatura"].adicionar(conteudo["temperatura"], t)
                self.canais["rotacao"].adicionar(conteudo["rotacao"], t)
                self.temp_motor_ok = conteudo["temp_motor"]
                self.nivel_ok = conteudo["nivel_lubri"]

                self.registro_completo.append([
                    datetime.now().isoformat(timespec="seconds"),
                    conteudo["vibracao"], conteudo["temperatura"], conteudo["rotacao"],
                    int(conteudo["temp_motor"]), int(conteudo["nivel_lubri"]),
                ])
                houve_dado_novo = True

        if houve_dado_novo:
            self._atualizar_cartoes()
            self._atualizar_leds()
            self._atualizar_graficos()
            self.lbl_status_geral.config(text=f"Fonte ativa: {self.descricao_fonte.get()} — recebendo dados.")

        self.after(200, self._atualizar_gui)

    def _atualizar_cartoes(self):
        for chave, canal in self.canais.items():
            w = self.widgets_cartao[chave]
            w["valor"].config(text=f"{canal.valor_atual:.1f} {canal.unidade}")
            w["media"].config(text=f"Media movel: {canal.media_movel:.1f} {canal.unidade}")
            texto_status, cor = canal.status()
            w["status"].config(text=texto_status, bg=cor)

    def _atualizar_leds(self):
        cor_temp_motor = COR_OK if self.temp_motor_ok else COR_CRITICO
        cor_nivel = COR_OK if self.nivel_ok else COR_CRITICO
        self.led_temp_motor["canvas"].itemconfig(self.led_temp_motor["circulo"], fill=cor_temp_motor)
        self.led_nivel["canvas"].itemconfig(self.led_nivel["circulo"], fill=cor_nivel)

    def _atualizar_graficos(self):
        for chave, canal in self.canais.items():
            ax = self.eixos[chave]
            ax.clear()
            ax.set_title(f"{canal.nome} ({canal.unidade})", color=COR_TEXTO, fontsize=10)
            ax.set_facecolor("#0b1119")
            ax.tick_params(colors=COR_TEXTO, labelsize=8)
            for spine in ax.spines.values():
                spine.set_color("#3a4a5a")

            if canal.historico:
                ax.plot(list(canal.tempos), list(canal.historico), color="#4cc9f0", linewidth=1.2, label="Instantaneo")
                media_serie = self._media_movel_serie(list(canal.historico), JANELA_MEDIA_MOVEL)
                ax.plot(list(canal.tempos), media_serie, color="#f4a259", linewidth=1.4, label="Media movel")
                ax.axhline(canal.limites["alerta"], color=COR_ALERTA, linestyle="--", linewidth=0.8)
                ax.axhline(canal.limites["critico"], color=COR_CRITICO, linestyle="--", linewidth=0.8)
                ax.legend(fontsize=7, facecolor=COR_PAINEL, labelcolor=COR_TEXTO, loc="upper left")

        self.canvas.draw_idle()

    @staticmethod
    def _media_movel_serie(dados, janela):
        resultado = []
        buffer_local = deque(maxlen=janela)
        for v in dados:
            buffer_local.append(v)
            resultado.append(sum(buffer_local) / len(buffer_local))
        return resultado

    def _fechar(self):
        self._parar_fonte()
        self.destroy()


if __name__ == "__main__":
    if not SERIAL_DISPONIVEL:
        print("AVISO: pyserial nao encontrado (fonte Modbus RTU ficara indisponivel). "
              "Instale com: pip install pyserial")
    app = DashboardSupervisorio()
    app.protocol("WM_DELETE_WINDOW", app._fechar)
    app.mainloop()
