//////////////////// Importando Biliotecas ////////////////////
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>

//////////////////// Definições ////////////////////
#define PASCALS_NIVEL_DO_MAR 	1013.25
#define I2C_BMP280 				0x76
#define SETPOINT_CALIBRADO 		0
// Pinos:
#define PIN_PITCH 	10
#define PIN_YAW		9
// Cinemática:
#define MARGEM_DE_SEGURANCA_LIFTOFF 7500
#define MARGEM_DE_SEGURANCA_APOGEU 	10
#define MARGEM_DE_SEGURANCA_POUSO 	20

//////////////////// Variáveis e Constantes ////////////////////
float ax, ay, az, gx, gy, gz;
float dt=0, last_dt=0;
float teta=0, psi=0, phi=0;
float yaw=0, pitch=0, roll=0;
float a11, a12, a13, a21, a22, a23, a31, a32, a33; // elementos da matriz de rotação
float Df[3] = {1,0,0}; // vetor direção (fixo no eixo X da base F)
float Di[3] = {0,0,0};
// Variáveis para calculos de cinemática:
float valor_aceleracao_inicial = 0;
float *ac_eixo_principal;
float ultima_aceleracao_eixo_principal;
float primeira_altura, altura, ultima_altura, altura_maxima = 0.0f;
bool deixarDeVerificarApogeu = false;

//////////////////// Structs e Objetos ////////////////////
struct PID
{
  float setPoint;
  float erro;
  float valor;
  float last_valor;
  float KP, KI, KD;
  float P, I, D;
  float ultimoDeltaT;
};

PID pid_alpha;
PID pid_beta;
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Servo motor_pitch;
Servo motor_yaw;

//////////////////// Setup e Loop ////////////////////
void setup(){
	iniciaMPU();
	iniciaBMP();

  	motor_pitch.attach(PIN_PITCH);
  	motor_yaw.attach(PIN_YAW);
  	motor_pitch.write(90);
  	motor_yaw.write(90);

  	delay(1000);
  	iniciaPID();

  	iniciaCinematica();
}

void loop(){
	updateMPU();
	updateCinematica();

	if(!verificaBurnout()) doTVC();

	if(!verificaLancamentoTerminado()){
    	dados_gyro += ("GX: " + (String) gx + ",    GY: " + gy + ",    GZ: " + gz + ";/n");
    	dados_acce_eixo_principal += (((String) *ac_eixo_principal) + ";/n");
  	}

  	if(deixarDeVerificarApogeu && verificaLancamentoTerminado()){
    	writeMicroSD("Altura_maxima.txt", (String) altura_maxima);
    	writeMicroSD("Dados_gyro.txt", dados_gyro);
    	writeMicroSD("dados_acce_eixo_principal.txt", dados_acce_eixo_principal);
  	}
}

//////////////////// BMP280 ////////////////////
void iniciaBMP(){
	while(!bmp.begin(I2C_BMP280));
}

float readAltitude(){
	return bmp.readAltitude(PASCALS_NIVEL_DO_MAR);
}

//////////////////// MPU6050 ////////////////////
void iniciaMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while(true);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  updateMPU();
}

void updateMPU(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x + 0.02; // somando correcao
  gy = g.gyro.y + 0.06;
  gz = g.gyro.z + 0.01;
}

//////////////////// Cinemática ////////////////////
void iniciaCinematica(){
	primeira_altura = readAltitude();
	descobreEixoPrincipal();
	while(verificaLancamentoIniciado() == false);
}

void updateCinematica(){
	ultima_altura = altura;
	altura = readAltitude();
	atualizaAlturaMaxima();
}

void atualizaAlturaMaxima(){
	if(altura >= ultima_altura - MARGEM_DE_SEGURANCA_APOGEU) altura_maxima = altura-primeira_altura;
}

void descobreEixoPrincipal(){
	updateMPU();
	ac_eixo_principal = (ax/lsb >= 0.9) ? &ax : ((ay/lsb >= 0.9) ? &ay : ((az/lsb >= 0.9) ? &az : &ax));
	valor_aceleracao_inicial = *ac_eixo_principal;
}

bool verificaLancamentoIniciado(){
  if(*ac_eixo_principal > valor_aceleracao_inicial + MARGEM_DE_SEGURANCA_LIFTOFF ) return true;
    return false;
}

bool verificaApogeu(){
  if(getAltitude() < ultima_altura - MARGEM_DE_SEGURANCA_APOGEU) return true;
  return false;
}

bool verificaLancamentoTerminado(){
  if((altura - primeira_altura) <= MARGEM_DE_SEGURANCA_POUSO){
    delay(5000);
    return true;
  }
  return false;
}

bool verificaBurnout(){
  if(*ac_eixo_principal < (ultima_aceleracao_eixo_principal + MARGEM_DE_SEGURANCA_LIFTOFF)) return true;
  ultima_aceleracao_eixo_principal = *ac_eixo_principal;
  return false;
}

//////////////////// Cálculos das Rotações ////////////////////
void integraGyro(float dt){
  teta+=gx*dt;
  psi+=gy*dt;
  phi+=gz*dt;
}

void calculaMatrizDeRotacao(){
  float sinTeta = sin(teta);
  float cosTeta = cos(teta);
  float sinPsi = sin(psi);
  float cosPsi = cos(psi);
  float sinPhi = sin(phi);
  float cosPhi = cos(phi);
  a11= cosPsi*cosPhi;
  a12= cosPsi*sinPhi;
  a13= -sinPsi;
  a21= -sinTeta*sinPsi*cosPhi-sinPhi*cosTeta;
  a22= -sinTeta*sinPsi*sinPhi+cosTeta*cosPhi;
  a23= -sinTeta*cosPsi;
  a31= cosTeta*sinPsi*cosPhi-sinPhi*sinTeta;
  a32= cosTeta*sinPsi*sinPhi+sinTeta*cosPhi;
  a33= cosTeta*cosPsi;
}

void calculaVetorDirecaoNaBaseInercial(){
  Di[0] = a11*Df[0];
  Di[1] = a21*Df[0];
  Di[2] = a31*Df[0];
}

float calculaNormaVetorDirecaoDaBaseInercial(){
  return sqrt(Di[0]*Di[0] + Di[1]*Di[1] + Di[2]*Di[2]);
}

//////////////////// TVC ////////////////////
void updateTime(){
	dt = (millis() - last_dt)/1000.0f;
  	last_dt = millis();
}

void doTVC(){
	updateTime();
	integraGyro();
  	calculaMatrizDeRotacao();
  	calculaVetorDirecaoNaBaseInercial();
  	float norma = calculaNormaVetorDirecaoDaBaseInercial(); // deve ser igual a 1
  	float alpha = acos(Di[1]/norma);
  	float beta = acos(Di[2]/norma);
  	alpha*=57.296; // converter de rad para graus
  	beta*=57.296;
  	setValor(&pid_alpha, alpha);
  	alpha = getResultPid(&pid_alpha);
  	setValor(&pid_beta, beta);
  	beta = getResultPid(&pid_beta);
  	motor_pitch.write(abs(alpha));
  	motor_yaw.write(abs(beta));
}

//////////////////// PID ////////////////////
void setKvalues(struct PID *pid, float kp, float ki, float kd){
  pid->KP = kp;
  pid->KI = ki;
  pid->KD = kd;
}

void setSetPoint(struct PID *pid, float setPoint){
  pid->setPoint = setPoint;
}

void setValor(struct PID *pid, float valor){
  pid->valor = valor;
}

float getResultPid(struct PID *pid){
  pid->erro = pid->setPoint - pid->valor;
  float deltaT = (millis() - pid->ultimoDeltaT)/1000.0;
  pid->ultimoDeltaT = millis();
  pid->P = pid->erro*pid->KP;
  pid->I+=(pid->erro*pid->KI)*deltaT;
  pid->D = (pid->last_valor - pid->valor)*(pid->KD/deltaT);
  pid->last_valor = pid->valor;
  return pid->P + pid->I + pid->D;
}

void iniciaPID(){
  setKvalues(&pid_alpha, 1, 0, 0);
  setSetPoint(&pid_alpha, SETPOINT_CALIBRADO);

  setKvalues(&pid_beta, 1, 0, 0);
  setSetPoint(&pid_beta, SETPOINT_CALIBRADO);
}

//////////////////// Cartão de Memória ////////////////////
void iniciaSD(){}

void writeMicroSD(String arquivo, String texto){}