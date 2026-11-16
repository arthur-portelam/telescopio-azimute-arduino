#include <Wire.h> 

#include <math.h> 

  

// --- CONSTANTES TÉCNICAS  --- 

const int MPU_ADDR = 0x68; 

const float SENS_ACCEL = 16384.0; // Escala +/- 2g 

const float SENS_GYRO  = 131.0;   // Escala +/- 250 °/s 

const float RAD_TO_DEG = 57.2957; 

const float ALPHA      = 0.98; 

  

//---VARIAVEIS PARA CALIBRAÇÃO--- 

float offsetGX = 0, offsetGY = 0, offsetGZ = 0; 

float offsetAX =0, offsetAY =0,offsetAZ =0; 

 

// --- VARIÁVEIS DE ESTADO --- 

float pitch_filtrado = 0.0, roll_filtrado = 0.0; 

unsigned long tempo_ultimo_ciclo = 0; 

  

//---INICIALIZAÇÃO--- 

void setup() { 

    Serial.begin(115200); 

    Wire.begin(); 

    Wire.setWireTimeout(3000, true); // Camada de Segurança 

  

   Wire.beginTransmission(MPU_ADDR); 

    Wire.write(0x6B);  

    Wire.write(0);     

    Wire.endTransmission(); 

// Configuração explícita de +/- 250°/s e +/- 2g  

    escreverRegistro(0x1B, 0x00);  

    escreverRegistro(0x1C, 0x00);  

 

    Serial.println("Calibrando... Mantenha o sensor parado.");  

    calibrarSensor(2000);  

} 

  

void loop() { 

    // 1. BURST READ (COLETA DE DADOS) 

    Wire.beginTransmission(MPU_ADDR); 

    Wire.write(0x3B); 

    Wire.endTransmission(false); 

    Wire.requestFrom(MPU_ADDR, 14); 

  

    // 2. CRIAÇÃO DAS VARIAVEIS  

    int16_t rawAX = (Wire.read() << 8) | Wire.read(); 

    int16_t rawAY = (Wire.read() << 8) | Wire.read(); 

    int16_t rawAZ = (Wire.read() << 8) | Wire.read(); 

    Wire.read(); Wire.read(); // Ignora Temperatura 

    int16_t rawGX = (Wire.read() << 8) | Wire.read(); 

    int16_t rawGY = (Wire.read() << 8) | Wire.read(); 

    int16_t rawGZ = (Wire.read() << 8) | Wire.read(); 

 

    //CONVERSÃO DOS DADOS   

    float accX = (rawAX / SENS_ACCEL) - offsetAX; 

    float accY = (rawAY / SENS_ACCEL) - offsetAY; 

    float accZ = (rawAZ / SENS_ACCEL) - offsetAZ; 

    float taxaGiroX = (rawGX / SENS_GYRO) - offsetGX; 

    float taxaGiroY = (rawGY / SENS_GYRO) - offsetGY; 

    float taxaGiroZ = (rawGZ / SENS_GYRO) - offsetGZ; 

  

    // 3. DETERMINAÇÃO TEMPORAL (CÁLCULO DE PRECISÃO) 

    unsigned long tempo_atual = micros(); 

    float dt = (tempo_atual - tempo_ultimo_ciclo) / 1000000.0; 

    tempo_ultimo_ciclo = tempo_atual; 

    if (dt <= 0 || dt > 0.1) dt = 0.01; // Bloco de Validação 

  

    // 4. ORIENTAÇÃO ESPACIAL 

    float pitch_acc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 

    float roll_acc = atan2(accY, accZ) * RAD_TO_DEG; 

 

  

    // 5. FUSÃO SENSORIAL (FILTRO COMPLEMENTAR) 

 	float ang_atgX = pitch_filtrado + (taxaGiroX * dt); 

    float ang_atgY = roll_filtrado + (taxaGiroY * dt); 

 

    pitch_filtrado = (ALPHA * ang_atgX) + ((1.0 - ALPHA) * pitch_acc); 

    roll_filtrado = (ALPHA * ang_atgY) + ((1.0 - ALPHA) * roll_acc); 

 
	static unsigned long timerSerial = 0;
	if (millis() - timerSerial > 100) {
	    Serial.print("Angulo Estabilizado: "); 
    	Serial.println(pitch_filtrado); 
    	Serial.println(roll_filtrado); 	
	]
} 

 

void calibrarSensor(int amostras){ 

long somaGX= 0, somaGY= 0, somaGZ= 0; 

long somaAX= 0, somaAY= 0, somaAZ= 0; 

for (int i=0; i<amostras; i++){ 

        Wire.beginTransmission(MPU_ADDR); 

        Wire.write(0x3B);  

        Wire.endTransmission(false); 

        Wire.requestFrom(MPU_ADDR, 14); 

         

        int16_t aX = (Wire.read() << 8) | Wire.read(); 

        int16_t aY = (Wire.read() << 8) | Wire.read(); 

        int16_t aZ = (Wire.read() << 8) | Wire.read(); 

        Wire.read(); Wire.read(); // temperatura 

        int16_t gX = (Wire.read() << 8) | Wire.read(); 

        int16_t gY = (Wire.read() << 8) | Wire.read(); 

        int16_t gZ = (Wire.read() << 8) | Wire.read(); 

         

        somaGX += gX; 

        somaGY += gY; 

        somaGZ += gZ; 

         

        somaAX += aX; 

        somaAY += aY; 

        somaAZ += aZ; 

         

delay(2);  

} 

    offsetGX = (somaGX / (float)amostras) / SENS_GYRO; 

    offsetGY = (somaGY / (float)amostras) / SENS_GYRO; 

    offsetGZ = (somaGZ / (float)amostras) / SENS_GYRO; 

 

    offsetAX = (somaAX / (float)amostras) / SENS_ACCEL; 

    offsetAY = (somaAY / (float)amostras) / SENS_ACCEL; 

    offsetAZ = ((somaAZ / (float)amostras) / SENS_ACCEL) - 1.0; 

} 

void escreverRegistro(byte reg, byte val) { 

        Wire.beginTransmission(MPU_ADDR);  

        Wire.write(reg);  

        Wire.write(val);  

        Wire.endTransmission();  

} 
