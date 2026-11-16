#include <Wire.h>
#include <math.h>



const int QMC_ADDR = 0x0D;             // Endereço I2C do chip QMC5883L           
const int MPU_ADDR = 0x68;             // Endereço I2C do chip MPU-6050


// ===== PARÂMETROS DE CALIBRAÇÃO / CONSTANTES =====
const float ALPHA_QMC = 0.95; // Filtro Passa-Baixa QMC
const float ALPHA_MPU = 0.98; // Filtro Complementar
const float SENS_ACCEL = 16384.0; // Escala +/- 2g 
const float SENS_GYRO  = 131.0;   // Escala +/- 250 °/s 
//const float RAD_TO_DEG = 57.2957;
//const float DEG_TO_RAD = 0.01745329;
const float declinacao = 0.0;        // Declinação magnética da cidade
const float DEADBAND = 1.0;     //Margem de erro tolerada em graus 
const float TOLERANCIA_AZIMUTE = 3.0; // Margem de erro aceitável (em graus)
const float TOLERANCIA_PITCH = 2.0; // Margem de erro para altitude
const float azimute_alvo = 90.0; // "posição x" externa
float pitch_alvo = 0.0;  // Inclinação alvo
float pitch_estavel = 0.0;    
float altura_alvo = 20.0; 
float altura_atual = 20.0;
float distancia_horizontal = 300.0;

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


//---VARIAVEIS PARA CALOIBRAÇÃO DO MPU--- 

float offsetGX = 0;
float offsetGY = 0;
float offsetGZ = 0; 
float offsetAX =0;
float offsetAY =0;
float offsetAZ =0;  

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
bool calculo_inicial_impresso = false;

// --- PROTÓTIPOS ---
void lerQMC(int16_t &mx, int16_t &my, int16_t &mz);
void lerMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
float diferencaAngular(float alvo, float atual);
void calcularPitchAlvo();
void navegarParaAlvo();
void navegarParaAltitude();
void calibrarMPU(int amostras);
void calibrarQMC();




void setup(){
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
  // Configurando o Set/Reset Period
  Wire.beginTransmission(QMC_ADDR); 
  Wire.write(0x0B); 
  Wire.write(0x01); 
  Wire.endTransmission(); 

  // Configurando o Control Register 1
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x09); 
  Wire.write(0x1D); // Configura OSR: 512, RNG: 8G, ODR: 200Hz, Modo Contínuo
  Wire.endTransmission();

  delay(500);

  Serial.println("=== CALIBRACAO INICIANDO DO MPU ===");
  calibrarMPU(2000);

  

  Serial.println("=== CALIBRACAO INICIANDO DO QMC===");
  Serial.println("Gire o sensor em todas as direcoes!");
  calibrarQMC();

  Serial.println("=== CALIBRACAO FINALIZADA ===");
  tempo_ultimo_ciclo = micros();
  
}

void loop() {
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
	magY_calibrado	*= scale_y;
  magZ_calibrado *= scale_z;

	// --- O FILTRO PASSA-BAIXA ---
  magX_filtrado  = (magX_filtrado  * ALPHA_QMC) + (magX_calibrado * (1.0 - ALPHA_QMC));
	magY_filtrado  = (magY_filtrado  * ALPHA_QMC) + (magY_calibrado * (1.0 - ALPHA_QMC));
  magZ_filtrado  = (magZ_filtrado  * ALPHA_QMC) + (magZ_calibrado * (1.0 - ALPHA_QMC));


  //TILT COMPENSATION
  float Xh = magX_filtrado * cos(pitch_rad) + magY_filtrado * sin(roll_rad) * sin(pitch_rad) - magZ_filtrado * cos(roll_rad) * sin(pitch_rad);
  float Yh = magY_filtrado * cos(roll_rad) + magZ_filtrado * sin(roll_rad);

	float azimute = atan2(Yh,Xh) * RAD_TO_DEG;//Calculo do ângulo já com a transformação em graus
	//azimute += declinacao; //Aplicando a declinação magnética de vitoria da conquista

	//Ajuste de quadrante: converte ângulos negativos para 0 até 360
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

//Inlcinação alvo
  calcularPitchAlvo();

  navegarParaAlvo();
  navegarParaAltitude();

  if (!calculo_inicial_impresso) {

    float erro_direcao = diferencaAngular(azimute_alvo, azimute_estavel);
    float erro_pitch = calcularErroPitch(pitch_alvo, pitch_estavel);

    Serial.println("====== POSICAO INICIAL IDENTIFICADA ======");

    Serial.print("Azimute atual: ");
    Serial.print(azimute_estavel, 1);

    Serial.print(" | Alvo Az: ");
    Serial.print(azimute_alvo, 1);

    Serial.print(" | Erro Az: ");
    Serial.print(erro_direcao, 1);


    Serial.print(" || Pitch atual: ");
    Serial.print(pitch_estavel, 1);

    Serial.print(" | Pitch Alvo: ");
    Serial.print(pitch_alvo, 1);

    Serial.print(" | Erro Pitch: ");
    Serial.print(erro_pitch, 1);

    calculo_inicial_impresso = true;
}

}


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

  Serial.println("Calibrando MPU...");

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

  Serial.println("Calibracao concluida!");

}

 // LEITURA DO QMC
void lerQMC(int16_t &mx, int16_t &my, int16_t &mz){
	
// Leitura em rajada de 6 bytes começando do 0x00  
  Wire.beginTransmission(QMC_ADDR);  
  Wire.write(0x00);  
  Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 6); // solicita os 6 bytes 


	if(Wire.available() >= 6) {
// Lendo e garantindo a ordem: PRIMEIRO é o Low (LSB) e o SEGUNDO é o High (MSB)
    	uint8_t x_lsb = Wire.read();
    	uint8_t x_msb = Wire.read();
    	mx = x_lsb | (x_msb << 8);  

    	uint8_t y_lsb = Wire.read();
    	uint8_t y_msb = Wire.read();
    	my = y_lsb | (y_msb << 8);  

	    uint8_t z_lsb = Wire.read();
	    uint8_t z_msb = Wire.read();
	    mz= z_lsb | (z_msb << 8);
	
  }
  else {   
    mx = 0;
    my = 0;
    mz = 0;
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
  
	// --- cálculo offset ---
  offset_x = (mx_max + mx_min) / 2.0;
  offset_y = (my_max + my_min) / 2.0; 
  offset_z = (mz_max + mz_min) / 2.0;
  
  // --- cálculo escala ---
  float range_x = (mx_max - mx_min) / 2.0;
  float range_y = (my_max - my_min) / 2.0;
  float range_z = (mz_max - mz_min) / 2.0;

  float range_medio = (range_x + range_y + range_z) / 3.0;
  
  scale_x = range_medio / range_x;
  scale_y = range_medio / range_y;
  scale_z = range_medio / range_z;

  
  Serial.println("\nOffsets e escalas calculados:");
  Serial.print("offset_x: "); Serial.println(offset_x);
  Serial.print("offset_y: "); Serial.println(offset_y);
  Serial.print("scale_x: "); Serial.println(scale_x);
  Serial.print("scale_y: "); Serial.println(scale_y);
}

//calculo para caminho mais curto
float diferencaAngular(float alvo, float atual) {
  float diff = alvo - atual;
//se a diferença do alvo para o o estado atual for maior que 180, ele deve subitrair a diferença  com 360, ele subtrai pois o caminho mais curto é o contrario do ciruculo trigonometrico (sentido horario)
  while (diff > 180) diff -= 360;
//se a diferença do alvo para o o estado atual for menor que 180, ele deve somar a diferença  com 360, ele soma pois o caminho mais curto (sentido horario)
  while (diff < -180) diff += 360;
  return diff;
}

void navegarParaAlvo() {
  float erro = diferencaAngular(azimute_alvo, azimute_estavel);
  static int ultima_acao = 0; 
  int acao_atual = 0;

  if (abs(erro) <= TOLERANCIA_AZIMUTE) {
    acao_atual = 1; // ALINHADO
  } else if (erro > 0) {
    acao_atual = 2; // VIRAR DIREITA
  } else {
    acao_atual = 3; // VIRAR ESQUERDA
  }

  if (acao_atual != ultima_acao) {
    Serial.print("----> ACAO: ");
    
    if (acao_atual == 1) {
      Serial.println("ALINHADO");
    } else if (acao_atual == 2) {
      Serial.println("VIRAR DIREITA");
    } else if (acao_atual == 3) {
      Serial.println("VIRAR ESQUERDA");
    }
    ultima_acao = acao_atual;
  }
}

float calcularErroPitch(float alvo, float atual) {
    return alvo - atual;
}

void navegarParaAltitude() {
  float erro_p = calcularErroPitch(pitch_alvo, pitch_estavel);
  static int ultima_acao_pitch = 0; 
  int acao_atual_pitch = 0;

  if (abs(erro_p) <= TOLERANCIA_PITCH) {
    acao_atual_pitch = 1; // ALTITUDE OK
  } 
  else if (erro_p > 0) {
    acao_atual_pitch = 2; // SUBIR (Pitch Up)
  } 
  else {
    acao_atual_pitch = 3; // DESCER (Pitch Down)
  }

  if (acao_atual_pitch != ultima_acao_pitch) {
    Serial.print("----> PITCH: ");
    if (acao_atual_pitch == 1) {
      Serial.println("[ ALTITUDE OK ]");
    } 
    else if (acao_atual_pitch == 2) {
      Serial.println("[ SUBIR ]");
    } 
    else if (acao_atual_pitch == 3) {
      Serial.println("[ DESCER ]");
    }
    ultima_acao_pitch = acao_atual_pitch;
  }
}

void calcularPitchAlvo() {
    float delta_altura = altura_alvo - altura_atual;
    pitch_alvo = atan2(delta_altura, distancia_horizontal) * RAD_TO_DEG;
}
