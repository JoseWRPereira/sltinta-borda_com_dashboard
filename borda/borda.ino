/*
  =========================================================================
  AQUISICAO DE SINAIS - CNC - Siderurgica - Baraldis Steelworks
  Arduino Uno - SERVIDOR (ESCRAVO) MODBUS RTU via porta serial
  =========================================================================
	Código gerado com auxílio de Claude IA para dar suporte às 
	atividades da disciplina de Inteligência Artificial (SLTINTA)
	ministrada pelo prof. José W. R. Pereira no IFSP-Salto
  =========================================================================

  Este firmware implementa o protocolo Modbus RTU do zero (sem biblioteca
  externa), respondendo como escravo no endereco MODBUS_SLAVE_ID.

  Funcoes Modbus implementadas:
    0x01  Read Coils                    (leitura de bits R/W)
    0x02  Read Discrete Inputs          (leitura de bits somente-leitura)
    0x03  Read Holding Registers        (leitura de registradores R/W)
    0x04  Read Input Registers          (leitura de registradores somente-leitura)
    0x05  Write Single Coil             (escrita de 1 bit)
    0x06  Write Single Register         (escrita de 1 registrador)
    0x0F  Write Multiple Coils          (escrita de N bits)
    0x10  Write Multiple Registers      (escrita de N registradores)

  Qualquer outra funcao, endereco fora da faixa ou tamanho de quadro
  invalido gera uma resposta de excecao Modbus padrao.

  -------------------------------------------------------------------------
  MAPA DE MEMORIA MODBUS (mesmo mapa a ser usado no cliente)
  -------------------------------------------------------------------------

  COILS (bits, leitura e escrita) - FC 01 / 05 / 0F
    Endereco 0: Saida digital 1 (reservada p/ expansao - ex: sirene/alarme) -> pino D4
    Endereco 1: Saida digital 2 (reservada p/ expansao - ex: valvula/rele)  -> pino D5

  DISCRETE INPUTS (bits, somente leitura) - FC 02
    Endereco 0: Status da temperatura do motor   (pino D2, 1 = chama acesa)
    Endereco 1: Status do nivel do lubrificante  (pino D3, 1 = nivel OK)

  INPUT REGISTERS (registradores de 16 bits, somente leitura) - FC 04
    Endereco 0: Vibracao     x100  (mm/s * 100)   -> ex: 432   = 4.32 mm/s
    Endereco 1: Temperatura  x100  (°C * 100)     -> ex: 18765 = 187.65 °C
    Endereco 2: Rotacao      x10   (RPM * 10)     -> ex: 14502 = 1450.2 RPM

  HOLDING REGISTERS (registradores de 16 bits, leitura e escrita) - FC 03/06/10
    Endereco 0: Limite ALERTA  vibracao     x100
    Endereco 1: Limite CRITICO vibracao     x100
    Endereco 2: Limite ALERTA  temperatura  x100
    Endereco 3: Limite CRITICO temperatura  x100
    Endereco 4: Limite ALERTA  rotacao      x10
    Endereco 5: Limite CRITICO rotacao      x10
  
  Fatores de escala (x100 / x10) sao necessarios porque registradores Modbus
  sao inteiros de 16 bits sem sinal (0-65535) e nao suportam ponto flutuante.
  =========================================================================
*/

// ---------------- Configuracao Modbus ----------------
const uint8_t  MODBUS_SLAVE_ID = 1;
const long     MODBUS_BAUDRATE = 9600;   

// ---------------- Pinagem dos sensores ----------------
const uint8_t PIN_VIBRACAO    = A0;
const uint8_t PIN_TEMPERATURA = A1;
const uint8_t PIN_ROTACAO     = A2;
const uint8_t PIN_TEMP_MOTOR  = 2;   // digital -> Discrete Input 0
const uint8_t PIN_NIVEL_LUBRI = 3;   // digital -> Discrete Input 1

// ---------------- Parametros de calibracao ----------------
const float ADC_REF_V = 5.0;
const int   ADC_RES   = 1023;

const float CAL_VIB_MIN_MMS = 0.0,   CAL_VIB_MAX_MMS = 20.0;
const float CAL_TEMP_MIN_C  = 0.0,   CAL_TEMP_MAX_C  = 600.0;
const float CAL_RPM_MIN     = 0.0,   CAL_RPM_MAX     = 3000.0;

const uint8_t N_AMOSTRAS = 8;   // media simples por leitura, reduz ruido do ADC

// ================== MAPA DE MEMORIA MODBUS (variaveis globais) ==================

#define NUM_COILS 2
bool coils[NUM_COILS] = { false, false };
const uint8_t PINOS_SAIDA[NUM_COILS] = { 4, 5 };  // reservados p/ expansao futura

#define NUM_DISCRETE_INPUTS 2
bool entradasDiscretas[NUM_DISCRETE_INPUTS] = { false, false };

#define NUM_INPUT_REGISTERS 3
uint16_t registrosEntrada[NUM_INPUT_REGISTERS] = { 0, 0, 0 };

#define NUM_HOLDING_REGISTERS 6
// Valores iniciais equivalem aos limites padrao do dashboard:
// vibracao 7.0/12.0 mm/s, temperatura 450/520 °C, rotacao 2600/2850 RPM
uint16_t registrosHolding[NUM_HOLDING_REGISTERS] = { 700, 1200, 45000, 52000, 26000, 28500 };

// ================== RECEPCAO DE QUADRO MODBUS (nao bloqueante) ==================

#define MAX_FRAME 32
uint8_t bufferRecepcao[MAX_FRAME];
uint8_t idxBuffer = 0;
unsigned long ultimoByteRecebido = 0;
// Intervalo de silencio considerado fim de quadro. Em um link serial USB
// ponto-a-ponto (sem RS-485 multiponto) esta folga de alguns ms e mais
// robusta que tentar cravar exatamente os 3.5 tempos de caractere do
// padrao Modbus classico, que sofreria com o jitter da porta USB-CDC.
const unsigned long TIMEOUT_ENTRE_BYTES_US = 3000;

// ============================== CRC16 MODBUS ====================================

uint16_t calcularCRC16(const uint8_t *dados, uint8_t tamanho) 
{
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < tamanho; pos++) 
  {
    crc ^= (uint16_t)dados[pos];
    for (uint8_t i = 8; i != 0; i--) 
    {
      if (crc & 0x0001) 
      {
        crc >>= 1;
        crc ^= 0xA001;
      } 
      else 
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ============================ ENVIO DE RESPOSTAS =================================

// Monta e envia um quadro calculando o CRC a partir dos dados fornecidos
// (usado para respostas construidas do zero: leituras e escritas multiplas).
void enviarQuadro(uint8_t *dadosSemCRC, uint8_t tamanho) 
{
  uint16_t crc = calcularCRC16(dadosSemCRC, tamanho);
  Serial.write(dadosSemCRC, tamanho);
  Serial.write((uint8_t)(crc & 0xFF));
  Serial.write((uint8_t)((crc >> 8) & 0xFF));
}

// Envia bytes ja prontos, incluindo CRC (usado para "ecoar" o pedido em
// FC05/FC06, cujo formato de resposta bem-sucedida e identico ao pedido).
void enviarBruto(uint8_t *dados, uint8_t tamanho) 
{
  Serial.write(dados, tamanho);
}

void enviarExcecao(uint8_t funcao, uint8_t codigoExcecao) 
{
  // Codigos padrao Modbus:
  // 0x01 Funcao ilegal | 0x02 Endereco de dado ilegal | 0x03 Valor de dado ilegal
  uint8_t resposta[3];
  resposta[0] = MODBUS_SLAVE_ID;
  resposta[1] = funcao | 0x80;
  resposta[2] = codigoExcecao;
  enviarQuadro(resposta, 3);
}

// ======================= TRATAMENTO DAS FUNCOES MODBUS ===========================

// FC 0x01 (Read Coils) e FC 0x02 (Read Discrete Inputs) compartilham a mesma
// logica: le N bits a partir de um endereco inicial de um mapa de bits.
void tratarLeituraBits( uint8_t *quadro, uint8_t tamanho, bool *mapaBits,
                        uint8_t numBitsDisponiveis, uint8_t funcao) 
{
  if (tamanho != 8) { enviarExcecao(funcao, 0x03); return; }

  uint16_t enderecoInicial = (quadro[2] << 8) | quadro[3];
  uint16_t quantidade      = (quadro[4] << 8) | quadro[5];

  if (quantidade == 0 || quantidade > 16) { enviarExcecao(funcao, 0x03); return; }
  if ((uint32_t)enderecoInicial + quantidade > numBitsDisponiveis) 
  {
    enviarExcecao(funcao, 0x02);
    return;
  }

  uint8_t byteCount = (quantidade + 7) / 8;
  uint8_t resposta[3 + 2];  // byteCount cabe em no maximo 2 bytes aqui (ate 16 bits)
  resposta[0] = MODBUS_SLAVE_ID;
  resposta[1] = funcao;
  resposta[2] = byteCount;
  resposta[3] = 0;
  resposta[4] = 0;

  for (uint16_t i = 0; i < quantidade; i++) 
  {
    if (mapaBits[enderecoInicial + i]) 
    {
      resposta[3 + (i / 8)] |= (1 << (i % 8));
    }
  }

  enviarQuadro(resposta, 3 + byteCount);
}

// FC 0x03 (Read Holding Registers) e FC 0x04 (Read Input Registers)
void tratarLeituraRegistros( uint8_t *quadro, uint8_t tamanho, uint16_t *mapaRegistros,
                             uint8_t numRegistrosDisponiveis, uint8_t funcao) 
{
  if (tamanho != 8) { enviarExcecao(funcao, 0x03); return; }

  uint16_t enderecoInicial = (quadro[2] << 8) | quadro[3];
  uint16_t quantidade      = (quadro[4] << 8) | quadro[5];

  if (quantidade == 0 || quantidade > 16) { enviarExcecao(funcao, 0x03); return; }
  if ((uint32_t)enderecoInicial + quantidade > numRegistrosDisponiveis) 
  {
    enviarExcecao(funcao, 0x02);
    return;
  }

  uint8_t byteCount = quantidade * 2;
  uint8_t resposta[3 + 32];
  resposta[0] = MODBUS_SLAVE_ID;
  resposta[1] = funcao;
  resposta[2] = byteCount;

  for (uint16_t i = 0; i < quantidade; i++) 
  {
    uint16_t valor = mapaRegistros[enderecoInicial + i];
    resposta[3 + i * 2]     = (valor >> 8) & 0xFF;
    resposta[3 + i * 2 + 1] = valor & 0xFF;
  }

  enviarQuadro(resposta, 3 + byteCount);
}

// FC 0x05 (Write Single Coil)
void tratarEscritaBitUnico(uint8_t *quadro, uint8_t tamanho) 
{
  if (tamanho != 8) { enviarExcecao(0x05, 0x03); return; }

  uint16_t endereco = (quadro[2] << 8) | quadro[3];
  uint16_t valor     = (quadro[4] << 8) | quadro[5];

  if (endereco >= NUM_COILS) { enviarExcecao(0x05, 0x02); return; }
  if (valor != 0x0000 && valor != 0xFF00) { enviarExcecao(0x05, 0x03); return; }

  coils[endereco] = (valor == 0xFF00);
  digitalWrite(PINOS_SAIDA[endereco], coils[endereco] ? HIGH : LOW);

  // Resposta bem-sucedida = eco exato do pedido (ja validado por CRC)
  enviarBruto(quadro, tamanho);
}

// FC 0x06 (Write Single Register)
void tratarEscritaRegistroUnico(uint8_t *quadro, uint8_t tamanho) 
{
  if (tamanho != 8) { enviarExcecao(0x06, 0x03); return; }

  uint16_t endereco = (quadro[2] << 8) | quadro[3];
  uint16_t valor     = (quadro[4] << 8) | quadro[5];

  if (endereco >= NUM_HOLDING_REGISTERS) { enviarExcecao(0x06, 0x02); return; }

  registrosHolding[endereco] = valor;
  enviarBruto(quadro, tamanho);
}

// FC 0x0F (Write Multiple Coils)
void tratarEscritaBitsMultiplos(uint8_t *quadro, uint8_t tamanho) 
{
  if (tamanho < 9) { enviarExcecao(0x0F, 0x03); return; }

  uint16_t enderecoInicial = (quadro[2] << 8) | quadro[3];
  uint16_t quantidade      = (quadro[4] << 8) | quadro[5];
  uint8_t  byteCount       = quadro[6];

  if (quantidade == 0 || quantidade > 16) { enviarExcecao(0x0F, 0x03); return; }
  if (byteCount != (quantidade + 7) / 8)  { enviarExcecao(0x0F, 0x03); return; }
  if (tamanho != 9 + byteCount)           { enviarExcecao(0x0F, 0x03); return; }
  if ((uint32_t)enderecoInicial + quantidade > NUM_COILS) 
  {
    enviarExcecao(0x0F, 0x02);
    return;
  }

  for (uint16_t i = 0; i < quantidade; i++) 
  {
    bool bit = (quadro[7 + (i / 8)] >> (i % 8)) & 0x01;
    coils[enderecoInicial + i] = bit;
    digitalWrite(PINOS_SAIDA[enderecoInicial + i], bit ? HIGH : LOW);
  }

  // Resposta: endereco inicial + quantidade escrita (sem os dados)
  uint8_t resposta[6];
  resposta[0] = MODBUS_SLAVE_ID;
  resposta[1] = 0x0F;
  resposta[2] = quadro[2];
  resposta[3] = quadro[3];
  resposta[4] = quadro[4];
  resposta[5] = quadro[5];
  enviarQuadro(resposta, 6);
}

// FC 0x10 (Write Multiple Registers)
void tratarEscritaRegistrosMultiplos(uint8_t *quadro, uint8_t tamanho) 
{
  if (tamanho < 9) { enviarExcecao(0x10, 0x03); return; }

  uint16_t enderecoInicial = (quadro[2] << 8) | quadro[3];
  uint16_t quantidade      = (quadro[4] << 8) | quadro[5];
  uint8_t  byteCount       = quadro[6];

  if (quantidade == 0 || quantidade > 16)  { enviarExcecao(0x10, 0x03); return; }
  if (byteCount != quantidade * 2)         { enviarExcecao(0x10, 0x03); return; }
  if (tamanho != 9 + byteCount)            { enviarExcecao(0x10, 0x03); return; }
  if ((uint32_t)enderecoInicial + quantidade > NUM_HOLDING_REGISTERS) 
  {
    enviarExcecao(0x10, 0x02);
    return;
  }

  for (uint16_t i = 0; i < quantidade; i++) 
  {
    uint16_t valor = (quadro[7 + i * 2] << 8) | quadro[7 + i * 2 + 1];
    registrosHolding[enderecoInicial + i] = valor;
  }

  uint8_t resposta[6];
  resposta[0] = MODBUS_SLAVE_ID;
  resposta[1] = 0x10;
  resposta[2] = quadro[2];
  resposta[3] = quadro[3];
  resposta[4] = quadro[4];
  resposta[5] = quadro[5];
  enviarQuadro(resposta, 6);
}

// ============================ DESPACHO DO QUADRO =================================

void processarFrame(uint8_t *quadro, uint8_t tamanho) 
{
  if (tamanho < 4) 
  	return;  // quadro minimo invalido (endereco+funcao+crc), ignora

  uint16_t crcRecebido   = quadro[tamanho - 2] | (quadro[tamanho - 1] << 8);
  uint16_t crcCalculado  = calcularCRC16(quadro, tamanho - 2);
  if (crcRecebido != crcCalculado) 
  	return;  // CRC invalido: descarta silenciosamente

  uint8_t endereco = quadro[0];
  if (endereco != MODBUS_SLAVE_ID) 
  	return;  // quadro nao e para este escravo

  uint8_t funcao = quadro[1];
  switch (funcao) 
  {
    case 0x01: tratarLeituraBits(quadro, tamanho, coils, NUM_COILS, funcao); 					break;
    case 0x02: tratarLeituraBits(quadro, tamanho, entradasDiscretas, NUM_DISCRETE_INPUTS, funcao); 		break;
    case 0x03: tratarLeituraRegistros(quadro, tamanho, registrosHolding, NUM_HOLDING_REGISTERS, funcao); 	break;
    case 0x04: tratarLeituraRegistros(quadro, tamanho, registrosEntrada, NUM_INPUT_REGISTERS, funcao); 		break;
    case 0x05: tratarEscritaBitUnico(quadro, tamanho); 								break;
    case 0x06: tratarEscritaRegistroUnico(quadro, tamanho); 							break;
    case 0x0F: tratarEscritaBitsMultiplos(quadro, tamanho); 							break;
    case 0x10: tratarEscritaRegistrosMultiplos(quadro, tamanho); 						break;
    default:   enviarExcecao(funcao, 0x01); 									break;  // funcao ilegal/nao suportada
  }
}

// ========================== AQUISICAO DOS SENSORES ================================

float lerMediaAnalogica(uint8_t pin) 
{
  long soma = 0;
  for (uint8_t i = 0; i < N_AMOSTRAS; i++) 
  {
    soma += analogRead(pin);
    delayMicroseconds(200);
  }
  return (float)soma / N_AMOSTRAS;
}

float escalar(float leituraADC, float minEng, float maxEng) 
{
  float tensao = (leituraADC / ADC_RES) * ADC_REF_V;
  float proporcao = tensao / ADC_REF_V;
  return minEng + proporcao * (maxEng - minEng);		// Revisar 
}

// Atualiza o mapa de Input Registers (medidas) e Discrete Inputs (sinais digitais).
// Esta e a unica funcao que "liga" o mundo fisico (sensores) ao mapa de memoria
// Modbus consultado pelo mestre.
void atualizarRegistros() 
{
  float adcVib  = lerMediaAnalogica(PIN_VIBRACAO);
  float adcTemp = lerMediaAnalogica(PIN_TEMPERATURA);
  float adcRpm  = lerMediaAnalogica(PIN_ROTACAO);

  float vibracao    = escalar(adcVib,  CAL_VIB_MIN_MMS, CAL_VIB_MAX_MMS);
  float temperatura = escalar(adcTemp, CAL_TEMP_MIN_C,  CAL_TEMP_MAX_C);
  float rotacao     = escalar(adcRpm,  CAL_RPM_MIN,     CAL_RPM_MAX);

  registrosEntrada[0] = (uint16_t)constrain(vibracao * 100.0,    0, 65535);
  registrosEntrada[1] = (uint16_t)constrain(temperatura * 100.0, 0, 65535);
  registrosEntrada[2] = (uint16_t)constrain(rotacao * 10.0,      0, 65535);

  entradasDiscretas[0] = !digitalRead(PIN_TEMP_MOTOR);   // 1 = temperatura excessiva detectada
  entradasDiscretas[1] = !digitalRead(PIN_NIVEL_LUBRI);  // 1 = nivel OK
}

// ================================== SETUP / LOOP ====================================

unsigned long ultimaAmostra = 0;
const unsigned long INTERVALO_AMOSTRAGEM_MS = 300;

void setup() 
{
  Serial.begin(MODBUS_BAUDRATE);

  pinMode(PIN_TEMP_MOTOR, INPUT_PULLUP);
  pinMode(PIN_NIVEL_LUBRI, INPUT_PULLUP);

  for (uint8_t i = 0; i < NUM_COILS; i++) 
  {
    pinMode(PINOS_SAIDA[i], OUTPUT);
    digitalWrite(PINOS_SAIDA[i], LOW);
  }

  atualizarRegistros();  // primeira leitura, antes do primeiro poll do mestre
}

void loop() 
{
  // ---- 1) Recepcao Modbus, nao bloqueante ----
  while (Serial.available()) 
  {
    if (idxBuffer < MAX_FRAME) 
    {
      bufferRecepcao[idxBuffer++] = Serial.read();
    } else 
    {
      Serial.read();  // descarta excedente (quadro maior que o esperado)
    }
    ultimoByteRecebido = micros();
  }
  if (idxBuffer > 0 && (micros() - ultimoByteRecebido) > TIMEOUT_ENTRE_BYTES_US) 
  {
    processarFrame(bufferRecepcao, idxBuffer);
    idxBuffer = 0;
  }

  // ---- 2) Amostragem periodica dos sensores ----
  unsigned long agora = millis();
  if (agora - ultimaAmostra >= INTERVALO_AMOSTRAGEM_MS) 
  {
    ultimaAmostra = agora;
    atualizarRegistros();
  }
}
