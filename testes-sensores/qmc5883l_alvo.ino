#include <Wire.h>
#include <math.h>

const int QMC_ADDR = 0x0D;              // Endereço I2C do chip QMC5883L
const float declinacao = -23.0;         // Declinação magnética

// --- VARIÁVEIS DO FILTRO PASSA-BAIXA ---
const float ALPHA = 0.90; // 90% de inércia (memória) e 10% de leitura nova
float eixo_x_filtrado = 0.0;
float eixo_y_filtrado = 0.0;

// Variáveis para rastrear os extremos
int16_t x_min = 32767, x_max = -32768;
int16_t y_min = 32767, y_max = -32768;

//variaveis do offset
float offset_x = 0;
float offset_y = 0;

float anguloAlvo = 0.0;


void lerSensor(int16_t &rx, int16_t &ry, int16_t &rz);


void calibrar();


void setup(){
  Wire.begin();
  Serial.begin(9600);

  // --- 1. estravando a porta do MPU-6050 ---
  Wire.beginTransmission(0x68); // Acorda o MPU
  Wire.write(0x6B); 
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // Desabilita os comandos do MPU
  Wire.write(0x6A); 
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // Ativa o I2C Bypass Mode (Abre a porta da bússola)
  Wire.write(0x37); 
  Wire.write(0x02);
  Wire.endTransmission();

  delay(100); // Dá um tempo para o hardware estabilizar

  // --- 2. CONFIGURANDO A BÚSSOLA (QMC5883L) ---
  // Configurando o Set/Reset Period
  Wire.beginTransmission(0x0D); 
  Wire.write(0x0B); 
  Wire.write(0x01); 
  Wire.endTransmission(); 

  // Configurando o Control Register 1
  Wire.beginTransmission(0x0D);
  Wire.write(0x09); 
  Wire.write(0x1D); // Configura OSR: 512, RNG: 8G, ODR: 200Hz, Modo Contínuo
  Wire.endTransmission();

  Serial.println("=== CALIBRACAO INICIANDO ===");
  Serial.println("Gire o sensor em todas as direcoes!");

  calibrar();

  Serial.println("=== CALIBRACAO FINALIZADA ===");
  
}

void loop() {
	
	int16_t x,y,z;
	lerSensor (x,y,z);
	
	// Auto calibração continmua
	  if (x < x_min) x_min = x;
    if (x > x_max) x_max = x;
    if (y < y_min) y_min = y;
    if (y > y_max) y_max = y;
    
  	offset_x = (x_max + x_min) / 2.0;
  	offset_y = (y_max + y_min) / 2.0;
	
	float x_calibrado = x - offset_x;
	float y_calibrado = y - offset_y; 


	// --- O FILTRO PASSA-BAIXA ---
  	eixo_x_filtrado = (eixo_x_filtrado * ALPHA) + (x_calibrado * (1.0 - ALPHA));
	eixo_y_filtrado = (eixo_y_filtrado * ALPHA) + (y_calibrado * (1.0 - ALPHA));



	float azimute = atan2(eixo_y_filtrado,eixo_x_filtrado) * 180.0/PI;//Calculo do ângulo já com a transformação em graus

  azimute += declinacao; //Aplicando a declinação magnética de vitoria da conquista

	//Ajuste de quadrante: converte ângulos negativos para 0 até 360
  if (azimute < 0) azimute += 360;
  if (azimute >= 360) azimute -= 360;
    
    // 1. LER DADOS DO COMPUTADOR
  if (Serial.available() > 0) {
      anguloAlvo = Serial.parseFloat(); // Atualiza o alvo global
      Serial.print("\n>>> NOVO ALVO DEFINIDO: ");
      Serial.print(anguloAlvo);
      Serial.println(" graus <<<\n");
  }

// 2. CALCULAR QUANTO FALTA (Diferença)
// Lógica para encontrar o caminho mais curto (evita girar 350° se pode girar 10°)
float erro = anguloAlvo - azimute;

if (erro > 180) erro -= 360;
if (erro < -180) erro += 360;

float erroMotor = abs(erro);

// 3. MOSTRAR NO MONITOR SERIAL
Serial.print("Azimute Atual: ");
Serial.print(azimute);
Serial.print(" | Alvo: ");
Serial.print(anguloAlvo);
Serial.print(" | FALTA: ");
Serial.print(erroMotor);
Serial.println("°");

// 4. LÓGICA DE MOVIMENTAÇÃO
if (abs(erro) > 1.0) { // Se a diferença for maior que 1 grau
    // moverMotor(anguloAlvo, azimute); // Sua função de motor
} else {
    Serial.println(" [ NO ALVO! ] ");
}

  Serial.print("Direção do Telescópio (Azimute): ");
	Serial.println(azimute);

  delay(100);//ajustar

}



 // LEITURA DO SENSOR
void lerSensor(int16_t &rx, int16_t &ry, int16_t &rz){
	
// Leitura em rajada de 6 bytes começando do 0x00  
  Wire.beginTransmission(QMC_ADDR);  
  Wire.write(0x00);  
  Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 6); // solicita os 6 bytes 


	if(Wire.available() >= 6) {
// Lendo e garantindo a ordem: PRIMEIRO é o Low (LSB) e o SEGUNDO é o High (MSB)
    	uint8_t x_lsb = Wire.read();
    	uint8_t x_msb = Wire.read();
    	rx = x_lsb | (x_msb << 8);  

    	uint8_t y_lsb = Wire.read();
    	uint8_t y_msb = Wire.read();
    	ry = y_lsb | (y_msb << 8);  

	    uint8_t z_lsb = Wire.read();
	    uint8_t z_msb = Wire.read();
	    rz= z_lsb | (z_msb << 8);
	
  }
}
void calibrar(){
  unsigned long tempo = millis();

  while(millis() - tempo < 15000) { // travar o arduino por 15 segunddos para apenas ler a bússola
    int16_t x, y, z;
    lerSensor(x, y, z);

    if (x < x_min) x_min = x;
    if (x > x_max) x_max = x;

    if (y < y_min) y_min = y;
    if (y > y_max) y_max = y;

    Serial.print(".");
    delay(100);
  }
    
  offset_x = (x_max + x_min) / 2.0;
  offset_y = (y_max + y_min) / 2.0; 
  
  // "Partida a frio" do filtro: 
  // Inicializamos o filtro com o valor já calibrado para evitar que 
  // o azimute comece em 0 e vá subindo lentamente no início.
  int16_t ix, iy, iz;
  lerSensor(ix, iy, iz);
  eixo_x_filtrado = ix - offset_x;
  eixo_y_filtrado = iy - offset_y;
}
