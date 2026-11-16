# Localizador de Estrelas: Azimute com Compensação de Tilt

Este sistema utiliza a fusão dos sensores **MPU-6050** (Acelerômetro/Giroscópio) e **QMC5883L** (Magnetômetro) para fornecer o azimute preciso de um telescópio, mesmo quando o tubo está inclinado.

## 🧠 Diferenciais Técnicos
- **Filtro Complementar:** Combina a estabilidade do acelerômetro com a agilidade do giroscópio para calcular Pitch e Roll.
- **Compensação de Inclinação (Tilt Compensation):** Utiliza trigonometria esférica para projetar os vetores magnéticos no plano horizontal.
- **Calibração Avançada:** Implementação de correções de Hard-Iron (desvio) e Soft-Iron (distorção de escala) nos três eixos (X, Y, Z).



## 🛠️ Esquema de Ligação
- SDA/SCL do MPU-6050 conectados ao Arduino.
- QMC5883L conectado ao barramento I2C auxiliar do MPU-6050 (Bypass Mode ativo).

## 📊 Saída de Dados
O sistema fornece em tempo real:
- **Pitch:** Inclinação vertical (Altitude).
- **Roll:** Inclinação lateral.
- **Azimute:** Direção horizontal corrigida.
