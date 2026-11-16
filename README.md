# 🔭 Telescópio Automatizado — Sistema de Orientação por Azimute e Altitude

Sistema de orientação e posicionamento automatizado para telescópio, desenvolvido com Arduino, sensores inerciais e magnetômetro. O projeto utiliza fusão de sensores para determinar a orientação do equipamento e controlar motores de passo responsáveis pelos movimentos de azimute e altitude.

O desenvolvimento integra conceitos de **Engenharia de Computação, Sistemas Embarcados, Eletrônica, Instrumentação e Processamento de Dados**.

## 📸 Demonstração

O projeto foi construído e apresentado em um evento de tecnologia, com funcionamento do sistema de posicionamento.

<!-- Adicione aqui uma foto ou vídeo do telescópio em funcionamento -->

## 🎯 Objetivo do Projeto

Desenvolver um sistema capaz de determinar a orientação espacial de um telescópio e realizar movimentos automatizados a partir de posições e trajetórias definidas.

Para isso, foram integrados sensores de orientação, processamento embarcado e motores de passo, permitindo controlar os movimentos horizontais (azimute) e verticais (altitude).

## ⚙️ Tecnologias e Componentes

### Hardware

| Componente | Função |
|---|---|
| Arduino Uno R3 | Unidade de processamento e controle |
| MPU-6050 | Acelerômetro e giroscópio |
| QMC5883L | Magnetômetro para determinação do azimute |
| Motores de passo NEMA 17 | Movimentação do telescópio |
| CNC Shield / Drivers | Acionamento dos motores |
| Estrutura mecânica | Suporte e movimentação do telescópio |

### Software

- **C++:** programação do firmware e da interface.
- **Arduino:** desenvolvimento do sistema embarcado.
- **Qt 6:** desenvolvimento da interface gráfica.
- **CMake:** configuração e compilação da interface.
- **Biblioteca AccelStepper:** controle dos motores de passo.
- **Protocolo I2C:** comunicação com os sensores.
- **Comunicação Serial:** integração entre Arduino e interface gráfica.

## 🧠 Funcionamento do Sistema

O funcionamento do projeto é dividido em diferentes etapas.

### 1. Leitura dos sensores

O sistema utiliza dois sensores principais:

- **MPU-6050:** fornece dados do acelerômetro e giroscópio, utilizados para determinar os ângulos de inclinação (Pitch e Roll).
- **QMC5883L:** fornece os dados magnéticos utilizados para calcular a orientação horizontal (Azimute).

A comunicação entre os sensores e o Arduino é realizada por meio do barramento I2C, utilizando o modo I2C Bypass do MPU-6050 para acesso ao magnetômetro.

### 2. Processamento e calibração

Os dados coletados passam por processos de tratamento para melhorar a estabilidade e a precisão das medições.

Entre as técnicas utilizadas estão:

- **Filtro Complementar:** combina os dados do acelerômetro e giroscópio para estimar Pitch e Roll.
- **Filtro Passa-Baixa:** reduz oscilações nas leituras do magnetômetro.
- **Hard-Iron Calibration:** corrige deslocamentos nos valores magnéticos.
- **Soft-Iron Calibration:** corrige diferenças de escala entre os eixos magnéticos.
- **Tilt Compensation:** utiliza os ângulos de inclinação para compensar os efeitos da posição do telescópio sobre a leitura magnética.

### 3. Determinação da orientação

A partir dos dados processados, o sistema calcula:

- **Azimute:** orientação horizontal do telescópio.
- **Pitch:** inclinação vertical.
- **Roll:** inclinação lateral.

Essas informações são utilizadas como referência para o posicionamento do equipamento.

### 4. Controle dos motores

O controle dos movimentos é realizado por dois motores de passo:

- **Motor X:** responsável pelo movimento de azimute.
- **Motor Z:** responsável pelo movimento de altitude.

A biblioteca AccelStepper permite controlar os motores utilizando sinais STEP e DIR, além de configurar velocidade e aceleração.

O firmware também implementa rotinas de inicialização, busca do norte magnético, nivelamento da altitude e execução de trajetórias entre posições definidas.

## 🖥️ Interface Gráfica

O projeto possui uma interface desenvolvida em C++ com Qt 6, responsável pela comunicação serial com o Arduino.

Entre os recursos implementados estão:

- Inicialização e calibração do sistema.
- Busca do norte magnético.
- Nivelamento da altitude.
- Configuração da velocidade dos motores.
- Definição de posições e trajetórias.
- Monitoramento das informações recebidas pela comunicação serial.
- Comando de parada dos motores.

## 📁 Organização do Repositório

```text
telescopio-azimute-arduino/
│
├── firmware/
│   └── telescopio_azimute.ino
│
├── interface-qt/
│   ├── main.cpp
│   ├── mainwindow.cpp
│   ├── mainwindow.h
│   ├── mainwindow.ui
│   └── CMakeLists.txt
│
├── testes-sensores/
│   └── Códigos de testes e calibração
│
└── README.md
```

## 🚀 Instalação e Execução

### Firmware — Arduino

1. Instale a Arduino IDE.
2. Instale a biblioteca [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/).
3. Abra o arquivo `firmware/telescopio_azimute.ino`.
4. Selecione a placa e a porta serial correspondentes ao Arduino.
5. Faça o upload do firmware.

A biblioteca Wire utilizada na comunicação I2C já faz parte do ambiente Arduino.

### Interface Gráfica — Qt

Requisitos:

- Qt 6.
- Componentes Qt Core, Widgets e SerialPort.
- CMake 3.19 ou superior.
- Compilador C++ compatível.

Na pasta `interface-qt`, execute:

```bash
cmake -S . -B build
cmake --build build --config Release
```

Abra o executável gerado após a compilação.

**Importante:** a porta serial está configurada inicialmente como `COM10` no código da interface. Altere essa configuração em `mainwindow.cpp` conforme a porta utilizada pelo Arduino.

## 🔌 Comunicação Serial

A comunicação entre a interface gráfica e o firmware utiliza comandos enviados pela porta serial, com velocidade de 9600 baud.

| Comando | Função |
|---|---|
| `CALIBRAR` | Executa a sequência de inicialização |
| `N` | Busca o norte magnético |
| `I` | Executa o nivelamento da altitude |
| `P` | Solicita a parada dos motores |
| `V` + valor | Define a velocidade dos motores |
| `Alvo:` | Envia os parâmetros de uma trajetória |

## 📚 Aprendizados e Conceitos Aplicados

O desenvolvimento deste projeto proporcionou a aplicação prática de diferentes conceitos da Engenharia de Computação:

- Programação de sistemas embarcados.
- Comunicação entre dispositivos via I2C e Serial.
- Aquisição e processamento de dados de sensores.
- Fusão de sensores e filtros digitais.
- Calibração de instrumentos.
- Controle de motores de passo.
- Desenvolvimento de interfaces gráficas.
- Integração entre hardware e software.

## 👥 Equipe

Projeto acadêmico desenvolvido em equipe, envolvendo a integração de hardware, software e estrutura mecânica.

<!-- Adicione aqui os nomes dos integrantes e orientadores -->

## 📌 Observações

O valor de declinação magnética está configurado inicialmente como zero no firmware. Portanto, o azimute apresentado corresponde à referência magnética, sem correção para o norte geográfico.

Este repositório reúne o firmware principal, a interface gráfica e os códigos desenvolvidos durante as etapas de experimentação e calibração dos sensores.

---

**Projeto acadêmico de Engenharia de Computação.**
