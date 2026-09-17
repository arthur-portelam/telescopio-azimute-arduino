#include <Wire.h>
#include <math.h>

// ===== PARÂMETROS DOS MOTORES
// Motor X
const int stepPinX = 2;
const int dirPinX = 5;
// Motor Z
const int stepPinZ = 4;
const int dirPinZ = 7;
// Enable compartilhado (CNC Shield)
const int enablePin = 8;
int stepDelayX = 2000; // velocidade do motor X (ajustável via Qt em RPM)
bool pararSolicitado = false;
String bufferSerial = "";


// ===== PARÂMETROS DE CALIBRAÇÃO / CONSTANTES
// ==========================================================
const int QMC_ADDR = 0x0D;             // Endereço I2C do chip QMC5883L           
const int MPU_ADDR = 0x68;             // Endereço I2C do chip MPU-6050

const float ALPHA_QMC = 0.95; // Filtro Passa-Baixa QMC
const float ALPHA_MPU = 0.98; // Filtro Complementar
const float SENS_ACCEL = 16384.0; // Escala +/- 2g 
const float SENS_GYRO  = 131.0;   // Escala +/- 250 °/s 
const float declinacao = 0.0;        // Declinação magnética da cidade -23.0, se deixar em 0 mostra a magnética
const float DEADBAND = 1.0;     //Margem de erro tolerada em graus 
const float TOLERANCIA_AZIMUTE = 1.5; // Margem de erro aceitável (em graus)
const float TOLERANCIA_PITCH = 2.0; // Margem de erro para altitude

float azimute_alvo = 90.0; // "posição x" externa
float pitch_alvo = 0.0;  // Inclinação alvo
float pitch_estavel = 0.0;    

// Variáveis para rastrear os extremos do QMC
int16_t mx_min = 32767, mx_max = -32768;
int16_t my_min = 32767, my_max = -32768;
int16_t mz_min = 32767, mz_max = -32768;

//Variaveis de calibração do QMC
float offset_x = 0;
float offset_y = 0;
float offset_z = 0;
float scale_x = 1.0;
float scale_y = 1.0;
float scale_z = 1.0;

//---VARIAVEIS PARA CALIBRAÇÃO DO MPU--- 
float offsetGX = -2.828792;
float offsetGY = 0.603373;
float offsetGZ = -0.428433;
float offsetAX = -0.017621;
float offsetAY = 0.010805;
float offsetAZ = -0.033458;  

// --- VARIÁVEIS DE ESTADO --- 
float azimute_filtrado = 0.0;
float azimute_estavel = 0.0;
float magX_filtrado = 0.0;
float magY_filtrado = 0.0;
float magZ_filtrado = 0.0;
float pitch_filtrado = 0.0;
float roll_filtrado = 0.0; 
unsigned long tempo_ultimo_ciclo = 0; 
unsigned long tempo_ultima_impressao = 0;


// --- PROTÓTIPOS ---
void lerQMC(int16_t &mx, int16_t &my, int16_t &mz);
void lerMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
float diferencaAngular(float alvo, float atual);
void navegarParaAlvo();
void navegarParaAltitude();
void calibrarMPU(int amostras);
void calibrarQMC();
void lerComandoSerial();
void processarComando(String cmd);
void pararMotores();
bool checarInterrupcao();
void darPassosX(int quantidade, bool direita);
void darPassosXZDiferentes(int passosX, int passosZ, bool direitaX, bool direitaZ);
void executarSequencia();
void buscarNorte();

void setup(){
  // --- Configuração dos Motores ---
  pinMode(stepPinX, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinZ, OUTPUT);
  pinMode(dirPinZ, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // motores habilitados

  // --- Configuração dos Sensores ---
  Wire.begin();
  Serial.begin(9600);

  // --- 1. Destravando a porta do MPU-6050 ---
  Wire.beginTransmission(MPU_ADDR); // LIGA O MPU
  Wire.write(0x6B); 
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x37); 
  Wire.write(0x02);       // Ativa o I2C Bypass Mode (Abre a porta da bússola)
  Wire.endTransmission();

  delay(100); // Dá um tempo para o hardware estabilizar

  // --- 2. CONFIGURANDO A BÚSSOLA (QMC5883L) ---
  Wire.beginTransmission(QMC_ADDR); 
  Wire.write(0x0B); 
  Wire.write(0x01); 
  Wire.endTransmission(); 

  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x09); 
  Wire.write(0x1D); // Configura OSR: 512, RNG: 8G, ODR: 200Hz, Modo Contínuo
  Wire.endTransmission();

  delay(500);
  // Serial.println("=== CALIBRACAO INICIANDO DO MPU ===");
  // calibrarMPU(2000); // <-- COMENTADO POIS VOCÊ VAI USAR OS OFFSETS FIXOS

  tempo_ultimo_ciclo = micros();
}

void loop() {
  // --- 1. Ouve os comandos do Qt Creator em paralelo ---
  lerComandoSerial();

  // --- 2. Lógica de Leitura dos Sensores ---
  unsigned long tempo_atual = micros();
  float dt = (tempo_atual - tempo_ultimo_ciclo) / 1000000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.01;
  tempo_ultimo_ciclo = tempo_atual;

  float ax, ay, az, gx, gy, gz;
  lerMPU(ax, ay, az, gx, gy, gz);

  float pitch_accel = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG; 
  float roll_accel = atan2(ay, az) * RAD_TO_DEG; 

  pitch_filtrado = ALPHA_MPU * (pitch_filtrado + gy * dt) + (1.0 - ALPHA_MPU) * pitch_accel;
  roll_filtrado  = ALPHA_MPU * (roll_filtrado + gx * dt)  + (1.0 - ALPHA_MPU) * roll_accel;

  float pitch_rad = pitch_filtrado * DEG_TO_RAD;
  float roll_rad = roll_filtrado * DEG_TO_RAD;

	int16_t rawMX,rawMY,rawMZ;
	lerQMC (rawMX,rawMY,rawMZ);
	
  //Calibração de Hard-Iron
  float magX_calibrado = rawMX - offset_x;
  float magY_calibrado = rawMY - offset_y;
  float magZ_calibrado = rawMZ - offset_z;
	
  //Calibração de Soft-Iron
  magX_calibrado *= scale_x;
  magY_calibrado *= scale_y;
  magZ_calibrado *= scale_z;

	// --- O FILTRO PASSA-BAIXA ---
  magX_filtrado  = (magX_filtrado  * ALPHA_QMC) + (magX_calibrado * (1.0 - ALPHA_QMC));
  magY_filtrado  = (magY_filtrado  * ALPHA_QMC) + (magY_calibrado * (1.0 - ALPHA_QMC));
  magZ_filtrado  = (magZ_filtrado  * ALPHA_QMC) + (magZ_calibrado * (1.0 - ALPHA_QMC));

  //TILT COMPENSATION
  float Xh = magX_filtrado * cos(pitch_rad) + magY_filtrado * sin(roll_rad) * sin(pitch_rad) - magZ_filtrado * cos(roll_rad) * sin(pitch_rad);
  float Yh = magY_filtrado * cos(roll_rad) + magZ_filtrado * sin(roll_rad);

  float azimute = atan2(Yh,Xh) * RAD_TO_DEG;
  azimute += declinacao; 

  if (azimute < 0) azimute += 360;
  if (azimute >= 360) azimute -= 360;

  //Filtros de deadband (zona morta)
  float erro_pitch_estabilidade = pitch_filtrado - pitch_estavel;
  if (abs(erro_pitch_estabilidade) > 0.5 ) {
    pitch_estavel = pitch_filtrado;
  }
  float erro_azimute = diferencaAngular(azimute, azimute_estavel);
  if(abs(erro_azimute)> DEADBAND){
    azimute_estavel = azimute;
  }

  navegarParaAlvo();
  navegarParaAltitude();

  // Impressão da Telemetria (Ajustado para 500ms para ler melhor)
  if (millis() - tempo_ultima_impressao >= 500) {
    float erro_direcao = diferencaAngular(azimute_alvo, azimute_estavel);
    float erro_pitch = calcularErroPitch(pitch_alvo, pitch_estavel);

    Serial.print("Az: "); Serial.print(azimute_estavel, 1);
    Serial.print(" | Alvo Az: "); Serial.print(azimute_alvo, 1);
    Serial.print(" | Erro Az: "); Serial.print(erro_direcao, 1);
    Serial.print(" || Pitch: "); Serial.print(pitch_estavel, 1);
    Serial.print(" | Pitch Alvo: "); Serial.print(pitch_alvo, 1);
    Serial.print(" | Erro Pitch: "); Serial.print(erro_pitch, 1);
    Serial.print(" | Roll: "); Serial.println(roll_filtrado, 1);

    tempo_ultima_impressao = millis();
  }
}

// ===== FUNÇÕES DE COMUNICAÇÃO E MOTOR
// ==========================================================

void lerComandoSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processarComando(bufferSerial);
      bufferSerial = "";
    } else {
      bufferSerial += c;
    }
  }
}

void processarComando(String cmd) {
  cmd.trim();
  if (cmd == "S") {
    executarSequencia();
  } else if (cmd == "P") { // Parada de emergência
    pararMotores();
  } else if (cmd.startsWith("V")) {
    int rpm = cmd.substring(1).toInt();
    if (rpm > 0) {
      stepDelayX = 75000 / rpm;
    }
  } else if (cmd.startsWith("A")) { // Recebe alvo do Azimute
    azimute_alvo = cmd.substring(1).toFloat();
  } else if (cmd.startsWith("P") && cmd.length() > 1) { // Recebe alvo do Pitch
    pitch_alvo = cmd.substring(1).toFloat();
  } else if (cmd == "N"){
    buscarNorte();
  }
}

void pararMotores() {
  digitalWrite(enablePin, HIGH); // desabilita todos os eixos
}

// Verifica se chegou um comando "P" no meio do movimento
bool checarInterrupcao() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      if (bufferSerial == "P") {
        pararSolicitado = true;
        bufferSerial = "";
        return true;
      }
      bufferSerial = "";
    } else {
      bufferSerial += c;
    }
  }
  return false;
}

// Move o motor X e calibra o QMC simultaneamente
void darPassosX_Calibrando(int quantidade, bool direita) {
  digitalWrite(dirPinX, direita ? HIGH : LOW);
  for (int i = 0; i < quantidade; i++) {
    if (checarInterrupcao()) return;
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(stepDelayX);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(stepDelayX);

    // A cada 100 passos, e feita uma leitura do magnetômetro
    if (i % 100 == 0) {
        int16_t x, y, z;
        lerQMC(x, y, z);
        if (x < mx_min) mx_min = x;
        if (x > mx_max) mx_max = x;
        if (y < my_min) my_min = y;
        if (y > my_max) my_max = y;
        if (z < mz_min) mz_min = z;
        if (z > mz_max) mz_max = z;
    }
  }
}

// Move X e Z simultaneamente e calibra o QMC
void darPassosXZ_Calibrando(int passosX, int passosZ, bool direitaX, bool direitaZ) {
  digitalWrite(dirPinX, direitaX ? HIGH : LOW);
  digitalWrite(dirPinZ, direitaZ ? HIGH : LOW);
  
  int maxPassos = max(passosX, passosZ);
  
  for (int i = 0; i < maxPassos; i++) {
    if (checarInterrupcao()) return;
    
    if (i < passosX) digitalWrite(stepPinX, HIGH);
    if (i < passosZ) digitalWrite(stepPinZ, HIGH);
    delayMicroseconds(stepDelayX);
    
    if (i < passosX) digitalWrite(stepPinX, LOW);
    if (i < passosZ) digitalWrite(stepPinZ, LOW);
    delayMicroseconds(stepDelayX); //velocidade

    // Calibração embutida (Lê o sensor a cada 100 passos)
    if (i % 100 == 0) {
        int16_t x, y, z;
        lerQMC(x, y, z);
        if (x < mx_min) mx_min = x;
        if (x > mx_max) mx_max = x;
        if (y < my_min) my_min = y;
        if (y > my_max) my_max = y;
        if (z < mz_min) mz_min = z;
        if (z > mz_max) mz_max = z;
    }
  }
}

void executarSequencia() {
  pararSolicitado = false;
  digitalWrite(enablePin, LOW);

  Serial.println("\n=== INICIANDO CALIBRACAO MECANICA (15s) ===");

  // 1. Zera a memória do QMC
  mx_min = 32767; mx_max = -32768;
  my_min = 32767; my_max = -32768;
  mz_min = 32767; mz_max = -32768;

  // 2. Os motores oscilam em 3D durante 15 segundos
  unsigned long inicio = millis();
  while (millis() - inicio < 15000) {

    darPassosXZ_Calibrando(17000, 1500, false, false); 
    if (pararSolicitado) { pararMotores(); return; }
    
    darPassosXZ_Calibrando(17000, 1500, true, true);  
    if (pararSolicitado) { pararMotores(); return; }
    
    darPassosXZ_Calibrando(17000, 1500, true, false); 
    if (pararSolicitado) { pararMotores(); return; }
    
    darPassosXZ_Calibrando(17000, 1500, false, true);  
    if (pararSolicitado) { pararMotores(); return; }
  }

  // 3. Calcula os offsets 
  offset_x = (mx_max + mx_min) / 2.0;
  offset_y = (my_max + my_min) / 2.0;
  offset_z = (mz_max + mz_min) / 2.0;
  
  float range_x = (mx_max - mx_min) / 2.0;
  float range_y = (my_max - my_min) / 2.0;
  float range_z = (mz_max - mz_min) / 2.0;
  float range_medio = (range_x + range_y + range_z) / 3.0;
  
  scale_x = range_medio / range_x;
  scale_y = range_medio / range_y;
  scale_z = range_medio / range_z;


  Serial.println("\n=== RESULTADOS DA CALIBRACAO ===");
  Serial.print("Offset X (Hard-Iron): "); Serial.println(offset_x);
  Serial.print("Offset Y (Hard-Iron): "); Serial.println(offset_y);
  Serial.print("Offset Z (Hard-Iron): "); Serial.println(offset_z);
  
  Serial.print("Escala X (Soft-Iron): "); Serial.println(scale_x);
  Serial.print("Escala Y (Soft-Iron): "); Serial.println(scale_y);
  Serial.print("Escala Z (Soft-Iron): "); Serial.println(scale_z);
  Serial.println("================================\n");

  Serial.println("\n=== CALIBRACAO FINALIZADA COM SUCESSO ===");

  tempo_ultimo_ciclo = micros();
}


// ===== FUNÇÕES DOS SENSORES

void lerMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz){
    Wire.beginTransmission(MPU_ADDR); 
    Wire.write(0x3B); 
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_ADDR, 14); 

    int16_t raw_ax = (Wire.read() << 8) | Wire.read(); 
    int16_t raw_ay = (Wire.read() << 8) | Wire.read(); 
    int16_t raw_az = (Wire.read() << 8) | Wire.read(); 

    Wire.read(); Wire.read(); // Ignora Temperatura 

    int16_t raw_gx = (Wire.read() << 8) | Wire.read(); 
    int16_t raw_gy = (Wire.read() << 8) | Wire.read(); 
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();   

    ax = (raw_ax / SENS_ACCEL) - offsetAX; 
    ay = (raw_ay / SENS_ACCEL) - offsetAY; 
    az = (raw_az / SENS_ACCEL) - offsetAZ; 
    gx = (raw_gx / SENS_GYRO) - offsetGX; 
    gy = (raw_gy / SENS_GYRO) - offsetGY; 
    gz = (raw_gz / SENS_GYRO) - offsetGZ;
}

void calibrarMPU(int amostras){
  long somaGX=0, somaGY=0, somaGZ=0;
  long somaAX=0, somaAY=0, somaAZ=0;

  for (int i=0; i<amostras; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14);

    int16_t aX = (Wire.read()<<8)|Wire.read();
    int16_t aY = (Wire.read()<<8)|Wire.read();
    int16_t aZ = (Wire.read()<<8)|Wire.read();
    Wire.read(); Wire.read();
    int16_t gX = (Wire.read()<<8)|Wire.read();
    int16_t gY = (Wire.read()<<8)|Wire.read();
    int16_t gZ = (Wire.read()<<8)|Wire.read();

    somaGX += gX; somaGY += gY; somaGZ += gZ;
    somaAX += aX; somaAY += aY; somaAZ += aZ;
    delay(2);
  }

  offsetGX = (somaGX/(float)amostras)/SENS_GYRO;
  offsetGY = (somaGY/(float)amostras)/SENS_GYRO;
  offsetGZ = (somaGZ/(float)amostras)/SENS_GYRO;
  offsetAX = (somaAX/(float)amostras)/SENS_ACCEL;
  offsetAY = (somaAY/(float)amostras)/SENS_ACCEL;
  offsetAZ = ((somaAZ/(float)amostras)/SENS_ACCEL) - 1.0;
}

void lerQMC(int16_t &mx, int16_t &my, int16_t &mz){
  Wire.beginTransmission(QMC_ADDR);  
  Wire.write(0x00);  
  Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 6); 

  if(Wire.available() >= 6) {
      uint8_t x_lsb = Wire.read();
      uint8_t x_msb = Wire.read();
      mx = x_lsb | (x_msb << 8);  

      uint8_t y_lsb = Wire.read();
      uint8_t y_msb = Wire.read();
      my = y_lsb | (y_msb << 8);  

      uint8_t z_lsb = Wire.read();
      uint8_t z_msb = Wire.read();
      mz= z_lsb | (z_msb << 8);
  } else {   
    mx = 0; my = 0; mz = 0;
  }
}

void calibrarQMC(){
  unsigned long star = millis();

  while(millis() - star < 15000) { 
    int16_t x, y, z;
    lerQMC(x, y, z);

    if (x < mx_min) mx_min = x;
    if (x > mx_max) mx_max = x;
    if (y < my_min) my_min = y;
    if (y > my_max) my_max = y;
    if (z < mz_min) mz_min = z;
    if (z > mz_max) mz_max = z;

    Serial.print(".");
    delay(100);
  }
  
  offset_x = (mx_max + mx_min) / 2.0;
  offset_y = (my_max + my_min) / 2.0; 
  offset_z = (mz_max + mz_min) / 2.0;
  
  float range_x = (mx_max - mx_min) / 2.0;
  float range_y = (my_max - my_min) / 2.0;
  float range_z = (mz_max - mz_min) / 2.0;
  float range_medio = (range_x + range_y + range_z) / 3.0;
  
  scale_x = range_medio / range_x;
  scale_y = range_medio / range_y;
  scale_z = range_medio / range_z;
}

float diferencaAngular(float alvo, float atual) {
  float diff = alvo - atual;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

void navegarParaAlvo() {
  float erro = diferencaAngular(azimute_alvo, azimute_estavel);
  if (abs(erro) <= TOLERANCIA_AZIMUTE) {
    // Serial.println("ALINHADO");
  } else if (erro > 0) {
    // Serial.println("VIRAR DIREITA");
  } else {
    // Serial.println("VIRAR ESQUERDA");
  }
}

float calcularErroPitch(float alvo, float atual) {
    return alvo - atual;
}

void navegarParaAltitude() {
    float erro_p = calcularErroPitch(pitch_alvo, pitch_estavel);
    if (abs(erro_p) <= TOLERANCIA_PITCH) {
       // Serial.println("ALTITUDE OK");
    } 
    else if (erro_p > 0) {
        //Serial.println("SUBIR (Pitch Up)");
    } 
    else {
        //Serial.println("DESCER (Pitch Down)");
    }
}
void buscarNorte() {
  Serial.println("\n=== BUSCANDO O NORTE MAGNÉTICO ===");
  digitalWrite(enablePin, LOW); // Energiza os motores

  while (true) {
    if (checarInterrupcao()) { 
      pararMotores(); 
      return; 
    }

    //leitura do MPU
    float ax, ay, az, gx, gy, gz;
    lerMPU(ax, ay, az, gx, gy, gz);
    float pitch_rad = atan2(ax, sqrt(ay * ay + az * az)); 
    float roll_rad  = atan2(ay, az); 

    //Aplicando a calibração do QMC
    int16_t rawMX, rawMY, rawMZ;
    lerQMC(rawMX, rawMY, rawMZ);
    float mx = (rawMX - offset_x) * scale_x;
    float my = (rawMY - offset_y) * scale_y;
    float mz = (rawMZ - offset_z) * scale_z;

    //Tilt Compensation
    float Xh = mx * cos(pitch_rad) + my * sin(roll_rad) * sin(pitch_rad) - mz * cos(roll_rad) * sin(pitch_rad);
    float Yh = my * cos(roll_rad) + mz * sin(roll_rad);

    float azimute = atan2(Yh, Xh) * RAD_TO_DEG + declinacao;
    if (azimute < 0) azimute += 360;
    if (azimute >= 360) azimute -= 360;

    //Calcula o erro em relação ao Norte (Alvo = 0.0)
    float erro = diferencaAngular(0.0, azimute);
    int passos = (abs(erro) < 10.0) ? 2 : 10; 


    //Verifica se chegou ao alvo(norte)
    if (abs(erro) <= TOLERANCIA_AZIMUTE) {
      Serial.println(">>> NORTE ENCONTRADO COM SUCESSO! <<<");
      pararMotores();
      azimute_estavel = azimute; // Sincroniza com a variável do loop
      break;
    }

    //Decide o sentido do giro pelo caminho mais curto
    bool girarDireita = (erro < 0); 
    digitalWrite(dirPinX, girarDireita ? HIGH : LOW);

    //Dá 10 passos no motor X
    for (int i = 0; i < passos; i++) {
       digitalWrite(stepPinX, HIGH);
       delayMicroseconds(stepDelayX);
       digitalWrite(stepPinX, LOW);
       delayMicroseconds(stepDelayX);
    }
  }
}
