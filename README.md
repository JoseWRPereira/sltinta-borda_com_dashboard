# Baraldis Steelworks - CNC101

  
Projeto de aquisição, tratamento e exibição de dados de máquina industrial 
para situação de aprendizagem na disciplina de Inteligência Artificial.


A infraestrutura do projeto consiste em:

```
   +-------------+                              +--------------+
   |             |                              |  Dashboard   |
   | Arduino Uno +--------| Modbus RTU |--------+ Supervisório |
   |        .ino |                              |          .py |
   +------+------+                              +--------------+
          |
          |
    +-----+-----+
    |  Sensores |
    +-----------+
```


=========================================================================
  Arduino Uno 		 - SERVIDOR (Escravo) MODBUS RTU via porta serial
  Dashboard Superviśorio - CLIENTE  (Mestre)  MODBUS RTU via porta serial
=========================================================================

---

## Árvore do diretório

```
---sltinta-borda_com_dashboard/
      |
      +---borda/
      |	     |
      |	     +---borda.ino
      |	     +---borda.ino.standard.hex
      |	     +---borda.ino.with_bootloader.standard.hex
      |	     
      +---dashboard/
      |	     |
      |	     +---dashboard_supervisorio.py
      |	     +---requirements.txt
      |
      +---README.md
          
```


---

Instruções para execução do `Dashboard Supervisório`:
```bash
# Entre no diretório `dashboard`
cd dashboard

# Criando a virtual environment (venv)
python -m venv venv

# Acessando a venv
source venv/bin/activate

# Instalando requirements
(venv) pip install -r requirements.txt

# Executando dashboard_supervisorio.py
(venv) python dashboard_supervisorio.py

# Criando arquivo executável
(venv) pyinstaller --onefile --noconsole --collect-all customtkinter dashboard_supervisorio.py
```

O arquivo executável é criado no diretório `dist`.


