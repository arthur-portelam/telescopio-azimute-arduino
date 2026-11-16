#include <Wire.h>
#include <MPU6050_light.h>
#include <QMC5883LCompass.h>

MPU6050 mpu(Wire);
QMC5883LCompass compass;

// CONSTANTES
const float ALPHA_QMC = 0.95;
const float DECLINACAO = -23.0;
const float DEADBAND = 1.0;
const float TOLERANCIA_PITCH = 2.0;
const float TOLERANCIA_ALVO = 3.0; 

float azimute_alvo = 90.0;
float pitch_alvo = 0.0;           
float pitch_estavel = 0.0;  
float altura_alvo = 100.0;
float altura_atual = 20.0;
float distancia_horizontal = 300.0;     

// FILTROS
float magX_filtrado = 0;
float magY_filtrado = 0;
float magZ_filtrado = 0;
float azimute_estavel = 0;
unsigned long tempo_ultima_impressao = 0;

// HARD / SOFT IRON
int mx_min = 32767;
int mx_max = -32768;
int my_min = 32767;
int my_max = -32768;
int mz_min = 32767;
int mz_max = -32768;

float offset_x = 0;
float offset_y = 0;
float offset_z = 0;
float scale_x = 1.0;
float scale_y = 1.0;
float scale_z = 1.0;

// PROTÓTIPOS
void calibrarQMC();
float calcularErroPitch(float alvo, float atual);
float diferencaAngular(float alvo, float atual);
void navegarParaAlvo();
void navegarParaAltitude();
void calcularPitchAlvo();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print("Status MPU6050: ");
  Serial.println(status);
  delay(1000);

  Serial.println("Calibrando MPU...");
  mpu.calcOffsets(true, true);
  Serial.println("MPU calibrado!");

  compass.init();
  Serial.println("QMC iniciado!");
  delay(1000);

  calibrarQMC();
  Serial.println("Calibracao concluida!");
}



void loop() {
  mpu.update();
  float pitch = mpu.getAngleY();
  float roll = mpu.getAngleX();
  float pitch_rad = radians(pitch);
  float roll_rad = radians(roll);

  compass.read();
  int mx_raw = compass.getX();
  int my_raw = compass.getY();
  int mz_raw = compass.getZ();

  float mx = (mx_raw - offset_x);
  float my = (my_raw - offset_y);
  float mz = (mz_raw - offset_z);

  mx *= scale_x;
  my *= scale_y;
  mz *= scale_z;

  magX_filtrado = (magX_filtrado * ALPHA_QMC) + (mx * (1.0 - ALPHA_QMC));
  magY_filtrado = (magY_filtrado * ALPHA_QMC) + (my * (1.0 - ALPHA_QMC));
  magZ_filtrado = (magZ_filtrado * ALPHA_QMC) + (mz * (1.0 - ALPHA_QMC));

  float Xh = magX_filtrado * cos(pitch_rad) + magY_filtrado * sin(roll_rad) * sin(pitch_rad) - magZ_filtrado * cos(roll_rad) * sin(pitch_rad);
  float Yh = magY_filtrado * cos(roll_rad) + magZ_filtrado * sin(roll_rad);

  float azimute = atan2(Yh, Xh) * 180.0 / PI;
  azimute += DECLINACAO;

  if (azimute < 0) azimute += 360;
  if (azimute >= 360) azimute -= 360;

//Filtros de deadband (zona morta)
  float erro_pitch_estabilidade = pitch - pitch_estavel;
  if (abs(erro_pitch_estabilidade) > 0.5) {
    pitch_estavel = pitch;
  }
  float erro_azimute = diferencaAngular(azimute, azimute_estavel);
  if (abs(erro_azimute) > DEADBAND) {
    azimute_estavel = azimute;
  }

  calcularPitchAlvo();

//Chamada de navegações
  navegarParaAlvo();
  navegarParaAltitude();

//Imprimir
if (millis() - tempo_ultima_impressao >= 100) {

    float erro_direcao = diferencaAngular(azimute_alvo, azimute_estavel);
    float erro_pitch = calcularErroPitch(pitch_alvo, pitch_estavel);

    Serial.print("Az: ");
    Serial.print(azimute_estavel, 1);

    Serial.print(" | Alvo Az: ");
    Serial.print(azimute_alvo, 1);

    Serial.print(" | Erro Az: ");
    Serial.print(erro_direcao, 1);

    Serial.print(" || Pitch: ");
    Serial.print(pitch_estavel, 1);

    Serial.print(" | Pitch Alvo: ");
    Serial.print(pitch_alvo, 1);

    Serial.print(" | Erro Pitch: ");
    Serial.print(erro_pitch, 1);

    Serial.print(" | Roll: ");
    Serial.println(roll, 1);

    tempo_ultima_impressao = millis();
}
}




void calibrarQMC() {
  Serial.println("\nCALIBRACAO QMC5883L - Gire o sensor!");
  unsigned long inicio = millis();
  while (millis() - inicio < 15000) {
    compass.read();
    int mx = compass.getX();
    int my = compass.getY();
    int mz = compass.getZ();
    if (mx < mx_min) mx_min = mx;
    if (mx > mx_max) mx_max = mx;
    if (my < my_min) my_min = my;
    if (my > my_max) my_max = my;
    if (mz < mz_min) mz_min = mz;
    if (mz > mz_max) mz_max = mz;
    Serial.print(".");
    delay(100);
  }
  offset_x = (mx_max + mx_min) / 2.0;
  offset_y = (my_max + my_min) / 2.0;
  offset_z = (mz_max + mz_min) / 2.0;
  float range_x = (mx_max - mx_min) / 2.0;
  float range_y = (my_max - my_min) / 2.0;
  float range_z = (mz_max - mz_min) / 2.0;
  float media = (range_x + range_y + range_z) / 3.0;
  scale_x = media / range_x;
  scale_y = media / range_y;
  scale_z = media / range_z;
}

float diferencaAngular(float alvo, float atual) {
  float diff = alvo - atual;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

void navegarParaAlvo() {
  float erro = diferencaAngular(azimute_alvo, azimute_estavel);
  if (abs(erro) <= TOLERANCIA_ALVO) {
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
        //Serial.println("ALTITUDE OK");
    } 
    else if (erro_p > 0) {
        //Serial.println("SUBIR (Pitch Up)");
    } 
    else {
       // Serial.println("DESCER (Pitch Down)");
    }
}

void calcularPitchAlvo() {
    float delta_altura = altura_alvo - altura_atual;
    pitch_alvo = atan2(delta_altura, distancia_horizontal) * RAD_TO_DEG;
}
