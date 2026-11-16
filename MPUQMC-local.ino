#include <Wire.h>
#include <math.h>
#include <AccelStepper.h>

// ===== PARÂMETROS DOS MOTORES
// Motor X
const int stepPinX = 2;
const int dirPinX = 5;
// Motor Z
const int stepPinZ = 4;
const int dirPinZ = 7;
// Enable compartilhado (CNC Shield)
const int enablePin = 8;
float velocidade_motor = 1000.0;
const float PASSOS_POR_GRAU_X = 40000.0 / 360.0;
const float PASSOS_POR_GRAU_Z = 29.70;

AccelStepper motorX(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper motorZ(AccelStepper::DRIVER, stepPinZ, dirPinZ);

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
const float TOLERANCIA_PITCH = 1.5; // Margem de erro para altitude

float azimute_alvo = 90.0; // "posição x" externa
float pitch_alvo = 0.0;  // Inclinação alvo
float pitch_estavel = 0.0;    
long posicaoAtualX = 0;
long posicaoAtualZ = 0;

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

float dir_ini = 0;
float dir_fin = 0;
float elv_ini = 0;
float elv_fin = 0;
float elv_max = 0;
float tempo_trajeto = 0;




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
void darPassosXZ_Calibrando(int passosX, int passosZ, bool direitaX, bool baixoZ);
void executarSequencia();
void buscarNorte();
float atualizarSensores();
void teste_um(String cmd);
void teste_dois(String cmd);

void setup(){
  bufferSerial.reserve(32);
  // --- Configuração dos Motores ---
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  // Configuração do Motor do Azimute (X)
  motorX.setMaxSpeed(1000.0);      // Velocidade máxima de ~9 graus por segundo
  motorX.setAcceleration(500.0);   // Rampa de aceleração suave (leva 2s para chegar no máximo)

  // Configuração do Motor da Altitude (Z)
  // O motor Z lida com o peso do tubo contra a gravidade.
  motorZ.setMaxSpeed(1000.0);
  motorZ.setAcceleration(500.0);

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
  
  lerComandoSerial();
  executandoMaquinaDeEstado();
  float azimute = atualizarSensores();

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

    Serial.print(F("Az: ")); Serial.print(azimute_estavel, 1);
    Serial.print(F(" | Alvo Az: ")); Serial.print(azimute_alvo, 1);
    Serial.print(F(" | Erro Az: ")); Serial.print(erro_direcao, 1);
    Serial.print(F(" || Pitch: ")); Serial.print(pitch_estavel, 1);
    Serial.print(F(" | Pitch Alvo: ")); Serial.print(pitch_alvo, 1);
    Serial.print(F(" | Erro Pitch: ")); Serial.print(erro_pitch, 1);
    Serial.print(F(" | Roll: ")); Serial.println(roll_filtrado, 1);

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


void pararMotores() {
  pararSolicitado = true;

  motorX.moveTo(motorX.currentPosition());
  motorZ.moveTo(motorZ.currentPosition());

  digitalWrite(enablePin, HIGH); 

}

// Verifica se chegou um comando "P" no meio do movimento
bool checarInterrupcao() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      bufferSerial.trim();
      if (bufferSerial == "P") {
        pararMotores(); 
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

// Move X e Z simultaneamente e calibra o QMC
void darPassosXZ_Calibrando(int passosX, int passosZ, bool direitaX, bool baixoZ) {

  motorX.move(direitaX ? passosX : -passosX);
  motorZ.move(baixoZ ? passosZ : -passosZ);

  unsigned long ultimoTempoSensor = 0;

  while( motorX.distanceToGo() != 0 || motorZ.distanceToGo() != 0 ){

    if (checarInterrupcao()) return;

    motorX.run();
    motorZ.run();

    if(millis() - ultimoTempoSensor >= 50 ){
      ultimoTempoSensor = millis();

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

  Serial.println(F("\n=== INICIANDO CALIBRACAO MECANICA (15s) ==="));

  // 1. Zera a memória do QMC
  mx_min = 32767; mx_max = -32768;
  my_min = 32767; my_max = -32768;
  mz_min = 32767; mz_max = -32768;

  // 2. Os motores oscilam em 3D durante 15 segundos
  unsigned long inicio = millis();
  while (millis() - inicio < 15000) {

    darPassosXZ_Calibrando(17000, 900, false, false); 
    if (pararSolicitado) { pararMotores(); return; }
    
    darPassosXZ_Calibrando(17000, 900, true, true);  
    if (pararSolicitado) { pararMotores(); return; }
    
    darPassosXZ_Calibrando(17000, 900, true, false); 
    if (pararSolicitado) { pararMotores(); return; }
    
    darPassosXZ_Calibrando(17000, 900, false, true);  
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


  Serial.println(F("\n=== RESULTADOS DA CALIBRACAO ==="));
  Serial.print(F("Offset X (Hard-Iron): ")); Serial.println(offset_x);
  Serial.print(F("Offset Y (Hard-Iron): ")); Serial.println(offset_y);
  Serial.print(F("Offset Z (Hard-Iron): ")); Serial.println(offset_z);
  
  Serial.print(F("Escala X (Soft-Iron): ")); Serial.println(scale_x);
  Serial.print(F("Escala Y (Soft-Iron): ")); Serial.println(scale_y);
  Serial.print(F("Escala Z (Soft-Iron): ")); Serial.println(scale_z);
  Serial.println(F("================================\n"));

  Serial.println(F("\n=== CALIBRACAO FINALIZADA COM SUCESSO ==="));

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
// Lê o azimute em repouso sem o atraso do filtro de 95%
float obterAzimuteInstantaneo() {
  // 1. Lê inclinação estática para compensar o tilt da bússola
  float ax, ay, az, gx, gy, gz;
  lerMPU(ax, ay, az, gx, gy, gz);
  float pitch_rad = atan2(ax, sqrt(ay * ay + az * az)); 
  float roll_rad  = atan2(ay, az); 

  // 2. Média de 5 leituras rápidas da bússola
  long somaX = 0, somaY = 0, somaZ = 0;
  for (int i = 0; i < 5; i++) {
    int16_t mx, my, mz;
    lerQMC(mx, my, mz);
    somaX += mx;
    somaY += my;
    somaZ += mz;
    delay(3);
  }

  // 3. Calibração Hard-Iron e Soft-Iron
  float mx_cal = ((somaX / 5.0) - offset_x) * scale_x;
  float my_cal = ((somaY / 5.0) - offset_y) * scale_y;
  float mz_cal = ((somaZ / 5.0) - offset_z) * scale_z;

  // 4. Tilt Compensation
  float Xh = mx_cal * cos(pitch_rad) + my_cal * sin(roll_rad) * sin(pitch_rad) - mz_cal * cos(roll_rad) * sin(pitch_rad);
  float Yh = my_cal * cos(roll_rad) + mz_cal * sin(roll_rad);

  az = atan2(Yh, Xh) * RAD_TO_DEG + declinacao;
  if (az < 0) az += 360;
  if (az >= 360) az -= 360;

  // Sincroniza os filtros do sistema para não dar salto quando voltar ao loop
  magX_filtrado = mx_cal;
  magY_filtrado = my_cal;
  magZ_filtrado = mz_cal;
  azimute_filtrado = az;
  azimute_estavel = az;

  return az;
}

void buscarNorte() {
  Serial.println(F("\n=== BUSCANDO O NORTE MAGNÉTICO ==="));
  digitalWrite(enablePin, LOW); // Energiza os motores

  while (true) {
    if (checarInterrupcao()) { 
      pararMotores(); 
      return; 
    }

    // Lê o azimute real e instantâneo
    float azimute = obterAzimuteInstantaneo();

    // Calcula o erro em relação ao Norte (Alvo = 0.0)
    float erro = diferencaAngular(0.0, azimute);
    float erroAbs = abs(erro);

    Serial.print(F("Azimute Real: ")); Serial.print(azimute, 1);
    Serial.print(F("° | Erro: ")); Serial.print(erro, 1);
    Serial.println(F("°"));

    // Verifica se chegou ao Norte dentro da tolerância
    if (erroAbs <= TOLERANCIA_AZIMUTE) {
      Serial.println(F(">>> NORTE ENCONTRADO COM SUCESSO! <<<"));
     
      motorX.setCurrentPosition(0);
      azimute_estavel = azimute;
      break;
    }

    // Calcula passos proporcionais (60% da distância restante):
    long passos = (long)(erroAbs * PASSOS_POR_GRAU_X * 0.6);

    // Limites de segurança:
    if (passos < 20) passos = 20;       // Mínimo de 20 passos (~0.18°) para ajuste fino
    if (passos > 1500) passos = 1500;   // Máximo de 1500 passos (~13.5°) por salto

    // Sentido do giro:
    // erro > 0: Norte está à direita (horário) -> passos negativos (-)
    // erro < 0: Norte está à esquerda (anti-horário) -> passos positivos (+)
    motorX.move(erro > 0 ? -passos : passos);

    while (motorX.distanceToGo() != 0) {
      if (checarInterrupcao()) {
        pararMotores();
        return;
      }
      motorX.run();
    }

    delay(40); // Pausa para a vibração mecânica cessar antes da leitura
  }
}

void nivelar_pitch() {
  Serial.println(F("\n=== NIVELANDO ALTITUDE (PITCH 0°) ==="));
  digitalWrite(enablePin, LOW); 

  while (true) {
    if (checarInterrupcao()) { 
      pararMotores(); 
      return; 
    }

    float somaPitch = 0;
    for (int k = 0; k < 5; k++) {
      float ax, ay, az, gx, gy, gz;
      lerMPU(ax, ay, az, gx, gy, gz);
      somaPitch += atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
      delay(2);
    }
    float pitch_atual = somaPitch / 5.0;

    float erro = calcularErroPitch(0.0, pitch_atual);

    Serial.print("Pitch Real: ");
    Serial.print(pitch_atual, 1);
    Serial.print("° | Erro: ");
    Serial.println(erro, 1);

    if (abs(erro) <= TOLERANCIA_PITCH) {
      Serial.println(F(">>> Nivelamento de Altitude Concluído com Sucesso! <<<"));

      motorZ.setCurrentPosition(0);
      pitch_filtrado = pitch_atual;
      pitch_estavel = pitch_atual;
      tempo_ultimo_ciclo = micros();
      break;
    }

    // Se pitch > 0 (apontado para cima), erro < 0 -> precisa DESCER.
    bool descerZ = (erro < 0.0);

    //int passos_para_pitch0 = (abs(erro) * PASSOS_POR_GRAU_Z);
    int passos = (abs(erro) < 5.0) ? 2 : 8;
    //motorZ.move(descerZ ? passos_para_pitch0 : -passos_para_pitch0);
    motorZ.move(descerZ ? passos : -passos);

    while (motorZ.distanceToGo() !=0){
      if (checarInterrupcao()){
        pararMotores(); 
        return;
      }
      motorZ.run();
    }
    delay(30); 
  }
}

float atualizarSensores(){
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

  azimute_filtrado = azimute;
  return azimute;
}

enum class estadoT{
      ESPERANDO,
      INICIALIZACAO,
      RASTREAMENTO,
      INDO_PARA_ALVO,
};

estadoT estadoAtual = estadoT::ESPERANDO;

void executandoMaquinaDeEstado(){
  switch (estadoAtual)
  {
    case estadoT::ESPERANDO:
      break;

    case estadoT::INICIALIZACAO:
      executarSequencia();
      if (pararSolicitado) { estadoAtual = estadoT::ESPERANDO; break; }

      nivelar_pitch();
      if (pararSolicitado) { estadoAtual = estadoT::ESPERANDO; break; }

      buscarNorte();
      if (pararSolicitado) { estadoAtual = estadoT::ESPERANDO; break; }

      posicaoAtualX = 0;
      posicaoAtualZ = 0;

      Serial.println("Calibracao finalizada e Ponto inicial identificado.");
      estadoAtual = estadoT::ESPERANDO;
      break;

    case estadoT::RASTREAMENTO:
      //logica de rastreamento
      break;

    case estadoT::INDO_PARA_ALVO:
      inicio_do_alvo();
      if (pararSolicitado) { estadoAtual = estadoT::ESPERANDO; 
      break; } 

      final_do_alvo();
      Serial.println("viagem concluida!");
      estadoAtual = estadoT::ESPERANDO;
      break;
  }
}

void processarComando(String cmd) {
  cmd.trim();
    if (cmd == "CALIBRAR") {
      estadoAtual = estadoT::INICIALIZACAO;
    } else if (cmd == "P") { // Parada de emergência
      pararMotores();
    } else if (cmd.startsWith("V")) {
      int rpm = cmd.substring(1).toInt();
      if (rpm > 0) {
        velocidade_motor = (rpm * 6.67);
        motorX.setMaxSpeed(velocidade_motor);
        motorZ.setMaxSpeed(velocidade_motor);
      }
    } //else if (cmd.startsWith("A")) { // Recebe alvo do Azimute
      //azimute_alvo = cmd.substring(1).toFloat();}  
      else if (cmd.startsWith("P") && cmd.length() > 1) { // Recebe alvo do Pitch
      pitch_alvo = cmd.substring(1).toFloat();
    } else if (cmd == "N"){
      buscarNorte();
    } else if (cmd == "RASTREAR"){
      estadoAtual = estadoT::RASTREAMENTO;
    } else if (cmd == "I"){
    nivelar_pitch();
    } else if ( cmd.startsWith("Alvo:")){
      alvo(cmd);
    } else if ( cmd.startsWith("DEMO1")){
      teste_um(cmd);
    }
    else if ( cmd.startsWith("DEMO2")){
      teste_dois(cmd);
    }
}

void alvo(String cmd){
    cmd.trim();
    pararSolicitado = false;
    
    if(cmd.startsWith("Alvo:")){
        String texto = cmd.substring(5);
        float dados[6];
        
        for(int i=0; i<6; i++){
            int virgula = texto.indexOf(',');
            
            if (virgula !=-1){
                dados[i] = texto.substring(0, virgula).toFloat();
                texto = texto.substring(virgula + 1);
            } else {
                dados[i] = texto.toFloat();
            }
        }
        
      dir_ini = dados[0];
      dir_fin = dados[1];
      elv_ini = dados[2];
      elv_fin = dados[3];
      elv_max = dados[4];
      tempo_trajeto = dados[5];

      Serial.println(F(">> Alvo recebido com sucesso!"));
      Serial.print(F("Direcao Inicial: ")); Serial.println(dir_ini);
      Serial.print(F("Direcao Final:   ")); Serial.println(dir_fin);
      Serial.print(F("Tempo(s):        ")); Serial.println(tempo_trajeto);
      estadoAtual = estadoT::INDO_PARA_ALVO;

    }
}
void teste_um(String cmd){
    cmd.trim();
    pararSolicitado = false;
  
      dir_ini = 20;
      dir_fin = 50;
      elv_ini = 15;
      elv_fin = 15;
      elv_max = 30;
      tempo_trajeto = 7;

      Serial.println(F(">> Alvo recebido com sucesso!"));
      Serial.print(F("Direcao Inicial: ")); Serial.println(dir_ini);
      Serial.print(F("Direcao Final:   ")); Serial.println(dir_fin);
      Serial.print(F("Tempo(s):        ")); Serial.println(tempo_trajeto);
      estadoAtual = estadoT::INDO_PARA_ALVO;
}
void teste_dois(String cmd){
    cmd.trim();
    pararSolicitado = false;
  
      dir_ini = 340;
      dir_fin = 20;
      elv_ini = 15;
      elv_fin = 15;
      elv_max = 40;
      tempo_trajeto = 12;

      Serial.println(F(">> Alvo recebido com sucesso!"));
      Serial.print(F("Direcao Inicial: ")); Serial.println(dir_ini);
      Serial.print(F("Direcao Final:   ")); Serial.println(dir_fin);
      Serial.print(F("Tempo(s):        ")); Serial.println(tempo_trajeto);
      estadoAtual = estadoT::INDO_PARA_ALVO;
}

void navegarParaAlvo() {
  float erro = diferencaAngular(azimute_alvo, azimute_estavel);
}

void inicio_do_alvo(){
  digitalWrite(enablePin, LOW);
  long alvo_passos_x =  calcularPassosMaisCurtoX(dir_ini);
  motorX.moveTo(alvo_passos_x);
  motorZ.moveTo(-elv_ini * PASSOS_POR_GRAU_Z);

  while(motorX.distanceToGo() != 0 || motorZ.distanceToGo() != 0){
    if (checarInterrupcao()){
      motorX.stop();
      motorZ.stop();
      pararMotores();
      return;
    }
    motorX.run();
    motorZ.run();
  }
}

void final_do_alvo(){
  digitalWrite(enablePin, LOW);
  
  //Proteção de segurança para o tempo
  if (tempo_trajeto <= 0) tempo_trajeto = 10.0; // Garante que não divide por zero
  float tempo_fase = tempo_trajeto / 2.0;

  //Calcula a metade do caminho em Azimute 
  float diff_azimute = diferencaAngular(dir_fin, dir_ini);
  float  metade_delta_ang = diff_azimute / 2.0;

  //passos em x para cade etapa
  long delta_passos_fase_x = -round(metade_delta_ang * PASSOS_POR_GRAU_X);

  //velocidade de x
  float dist_x_grau = abs(metade_delta_ang);
  long passos_dist_x = dist_x_grau * PASSOS_POR_GRAU_X; 
  float velocidade_x = (passos_dist_x > 0) ? (passos_dist_x / tempo_fase) : velocidade_motor;

  //subida até elv_max e metade do azimute
  float dist_z_subida = abs(elv_max - elv_ini);
  long passos_z = dist_z_subida * PASSOS_POR_GRAU_X;
  long passos_z_subida = dist_z_subida * PASSOS_POR_GRAU_Z;
  //velocidade de z
  float velocidade_z_s = (passos_z_subida > 0) ? (passos_z_subida / tempo_fase) : velocidade_motor;

  motorX.setMaxSpeed(velocidade_x);
  motorZ.setMaxSpeed(velocidade_z_s);

  //o sinal negativo em X é para girar no sentido correto
  motorX.move(delta_passos_fase_x);
  motorZ.moveTo(-elv_max * PASSOS_POR_GRAU_Z);

  Serial.println(F(">> Iniciando Fase 1: Subida ao ápice..."));

  while(motorX.distanceToGo() != 0 || motorZ.distanceToGo() != 0){
    if (checarInterrupcao()){
      motorX.stop();
      motorZ.stop();
      pararMotores();
      return;
    }
    motorX.run();
    motorZ.run();
  }

  //Descida de elv_max até elv_fin e chegada no azimute final
  float dist_z_descida = abs(elv_fin - elv_max);
  long passos_z_descida = dist_z_descida * PASSOS_POR_GRAU_Z;

  float velocidade_z_descida = (passos_z_descida > 0) ? (passos_z_descida / tempo_fase) : velocidade_motor;
  motorZ.setMaxSpeed(velocidade_z_descida);

  motorX.move(delta_passos_fase_x);
  motorZ.moveTo(-elv_fin * PASSOS_POR_GRAU_Z);

  Serial.println(F(">> Iniciando Fase 2: Descida ao destino final..."));

  while(motorX.distanceToGo() != 0 || motorZ.distanceToGo() != 0){
    if (checarInterrupcao()){
      motorX.stop();
      motorZ.stop();
      pararMotores();
      return;
    }
    motorX.run();
    motorZ.run();
  }

  motorX.setMaxSpeed(velocidade_motor);
  motorZ.setMaxSpeed(velocidade_motor);
  Serial.println(F(">> Alvo concluído com sucesso!"));
}
//para onde o motor está apontando
float obterAzimuteMotor(){
  float angulo = -(float)motorX.currentPosition() / PASSOS_POR_GRAU_X;
  angulo= fmod(angulo, 360.0);
  if(angulo < 0) angulo += 360.0;
  return angulo;
}

//calcula a coordenada de passos mais curta no círculo (-180 a +180)
long calcularPassosMaisCurtoX(float azimute_alvo){
  float atual= obterAzimuteMotor();
  float menor_erro= diferencaAngular(azimute_alvo, atual); // retorna o menor caminho
  long delta_passos= -round(menor_erro * PASSOS_POR_GRAU_X); // o menor caminho convertido em passos
  return motorX.currentPosition() + delta_passos; // possição atual mais a distancia que deve ser percorrida
}


