// OBS: adicionar servo para o paraquedas

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

#define I2C_BMP280 0x76
#define PASCALS_NIVEL_DO_MAR 1013.25

#define MARGEM_DE_SEGURANCA_LIFTOFF 7500
#define MARGEM_DE_SEGURANCA_APOGEU 10
#define MARGEM_DE_SEGURANCA_POUSO 20

#define PIN_SERVO_PITCH 10
#define PIN_SERVO_YAW	9
#define LED_ERRO		5

#define KP 0
#define KI 0
#define KD 0
#define SETPOINT_CALIBRADO 0

const int MPU = 0x68;

Servo motor_pitch;
Servo motor_yaw;
Adafruit_BMP280 bmp;
PID pid_pitch(KP, KI, KD);
PID pid_yaw(KP, KI, KD);

int16_t ax, ay, az;
int16_t gx, gy, gz;
const float lsb = 16384.00;

int16_t valor_aceleracao_inicial = 0;
int16_t *ac_eixo_principal;
int16_t ultima_aceleracao_eixo_principal;

float primeira_altura, altura, ultima_altura, altura_maxima = 0.0f;

float q[4] = {1, 0, 0, 0}; // quaternion
float angulos_de_euler[3]; // roll, pitch, yaw
float dt;

float tempo_ultimo_updateTVC = 0.0f;

bool deixarDeVerificarApogeu = false;

String dados_gyro = "";
String dados_acce_eixo_principal = "Aceleracao Eixo Principal/n";

void setup(){
	pinMode(LED_ERRO, OUTPUT);

	iniciaSD();

	motor_pitch.attach(PIN_SERVO_PITCH);
	motor_pitch.write(90);
	motor_yaw.attach(PIN_SERVO_YAW);
	motor_yaw.write(90);

	startTVC();

	iniciaMPU();
	iniciaBMP();

	primeira_altura = getAltitude();

	descobreEixoPrincipal();

	while(verificaLancamentoIniciado() == false);
}

void loop(){
	ultima_altura = altura;
	altura = getAltitude();

	AtualizaAlturaMaxima();

	if(!verificaBurnout()) updateTVC();

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

//////////////////// BMP ////////////////////
void iniciaBMP(){
	t0 = millis();
	while(!bmp.begin(I2C_BMP280)){
		if((millis() - t0) >= 5000){
			digitalWrite(LED_ERRO, HIGH);
			while(false);
		}
	}
}

float getAltitude(){
	return bmp.readAltitude(PASCALS_NIVEL_DO_MAR);
}

//////////////////// MPU ////////////////////
void iniciaMPU(){
	Wire.begin();
	Wire.beginTransmission(MPU);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);
}

void updateMPU(){
  	Wire.beginTransmission(MPU);
  	Wire.write(0x3B);  
  	Wire.endTransmission(false);
  	Wire.requestFrom(MPU,14,true);

  	ax = Wire.read()<<8|Wire.read();      
  	ay = Wire.read()<<8|Wire.read(); 
  	az = Wire.read()<<8|Wire.read(); 
  	int16_t Tmp = Wire.read()<<8|Wire.read(); 
  	gx = Wire.read()<<8|Wire.read(); 
  	gy = Wire.read()<<8|Wire.read(); 
  	gz = Wire.read()<<8|Wire.read();
}

//////////////////// CINEMÁTICA ////////////////////
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

void AtualizaAlturaMaxima(){
  if(altura >= ultima_altura - MARGEM_DE_SEGURANCA_APOGEU) altura_maxima = altura-primeira_altura;
}

//////////////////// QUATERNIONS e ANGULOS DE EULER ////////////////////
void calculaAngulosDeEuler(){
	// derivar e integrar de uma vez só
  q[0] += (0.5f)*(q[1]*getGx() - q[2]*getGy() - q[3]*getGz())*dt;
  q[1] += (0.5f)*(q[0]*getGx() + q[2]*getGz() - q[3]*getGy())*dt;
  q[2] += (0.5f)*(q[0]*getGy() - q[1]*getGz() + q[3]*getGx())*dt;
  q[3] += (0.5f)*(q[0]*getGz() + q[1]*getGy() - q[2]*getGx())*dt;

  // converter para angulos de euler
  angulos_de_euler[0] = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]*q[1] + q[2]*q[2]));
  angulos_de_euler[1] = asin(2*(q[0]*q[2] - q[3]*q[1]));
  angulos_de_euler[2] = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
}

//////////////////// PID ////////////////////
// A classe PID foi escrita por Ivan Seidel
class PID{
public:
  
  double error;
  double sample;
  double lastSample;
  double kP, kI, kD;      
  double P, I, D;
  double pid;
  
  double setPoint;
  long lastProcess;
  
  PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  
  void addNewSample(double _sample){
    sample = _sample;
  }
  
  void setSetPoint(double _setPoint){
    setPoint = _setPoint;
  }
  
  double process(){
    // Implementação P ID
    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    
    //P
    P = error * kP;
    
    //I
    I = I + (error * kI) * deltaTime;
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
    // Soma tudo
    pid = P + I + D;
    
    return pid;
  }
};

//////////////////// TVC ////////////////////
void startTVC(){
	pid_pitch.setSetPoint(SETPOINT_CALIBRADO);
  	pid_yaw.setSetPoint(SETPOINT_CALIBRADO);
}

void updateTVC(){
	dt = (millis() - tempo_ultimo_updateTVC)/1000.0f;
	tempo_ultimo_updateTVC = millis();
	
	float PITCH = angulos_de_euler[1];
	float YAW = angulos_de_euler[2];

	pid_pitch.addNewSample(map(pitch, 0, 2*PI, 0, 100));
	pid_yaw.addNewSample(map(yaw, 0, 2*PI, 0, 100));

	motor_pitch.write(pid_pitch.process());
	motor_yaw.write(pid_yaw.process());
}

//////////////////// MICRO SD CARD ////////////////////
void iniciaSD(){
	/*
	if(!SD.begin(PINO_MICRO_SD)){
    	digitalWrite(LED_ERRO, HIGH); Serial.println("Erro ao iniciar o módulo micro SD");
  	}
	*/
}

void writeMicroSD(String arquivo, String texto){
  //File dataFile = SD.open(arquivo, FILE_WRITE);
  //if(dataFile){
//    dataFile.println(texto);
    //dataFile.close();
  //}
  //else digitalWrite(LED_ERRO, HIGH);
}