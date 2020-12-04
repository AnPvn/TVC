// OBS: adicionar servo para o paraquedas

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
    //Serial.println(P);
    
    //I
    I = I + (error * kI) * deltaTime;
    //Serial.println(I);
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    //Serial.println(D);
    
    // Soma tudo
    pid = P + I + D;
    
    return pid;
  }
};

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

#define KP 0.1f
#define KI 0.0f
#define KD 0
const int SETPOINT_CALIBRADO = 0;

const int MPU = 0x68;

Servo motor_pitch;
Servo motor_yaw;
Adafruit_BMP280 bmp;
PID pid_pitch(KP, KI, KD);
PID pid_yaw(KP, KI, KD);

int16_t ax, ay, az;
int16_t gx, gy, gz;
const float lsb_acc = 16384.00;
const float lsb_gyro = 131.00;

int16_t valor_aceleracao_inicial = 0;
int16_t *ac_eixo_principal;
int16_t ultima_aceleracao_eixo_principal;

float primeira_altura, altura, ultima_altura, altura_maxima = 0.0f;

float q[4] = {1, 0, 0, 0}; // quaternion
float angulos_de_euler[3]; // roll, pitch, yaw
float dt;
float norma;

float tempo_ultimo_updateTVC = 0.0f;

bool deixarDeVerificarApogeu = false;

String dados_gyro = "";
String dados_acce_eixo_principal = "Aceleracao Eixo Principal/n";

void setup(){
	pinMode(LED_ERRO, OUTPUT);

  //Serial.println("Iniciando SD...");
	//iniciaSD();

	motor_pitch.attach(PIN_SERVO_PITCH);
	motor_pitch.write(90);
	motor_yaw.attach(PIN_SERVO_YAW);
	motor_yaw.write(90);
delay(1000);
  Serial.println("Iniciando TVC...");
	startTVC();

  Serial.println("Iniciando MPU...");
	iniciaMPU();
  Serial.println("MPU iniciado.");
  //Serial.println("Iniciando BMP...");
	//iniciaBMP();

	//primeira_altura = getAltitude();

	descobreEixoPrincipal();

  //Serial.println("Esperando o início do lançamento...");
	//while(verificaLancamentoIniciado() == false);
  //Serial.println("And we have LIFT OOOOOOFF!!");
}

void loop(){
  updateMPU();
	//ultima_altura = altura;
	//altura = getAltitude();

  //Serial.println("Atualizando altura máxima...");
	//AtualizaAlturaMaxima();

	//if(!verificaBurnout()){
    Serial.println("Updating TVC...");
	  updateTVC();
	//}

	/*
  if(!verificaLancamentoTerminado()){
    dados_gyro += ("GX: " + (String) gx + ",    GY: " + gy + ",    GZ: " + gz + ";/n");
    dados_acce_eixo_principal += (((String) *ac_eixo_principal) + ";/n");
  }
 	
 	if(deixarDeVerificarApogeu && verificaLancamentoTerminado()){
    Serial.println("Lançamento terminado.");
    writeMicroSD("Altura_maxima.txt", (String) altura_maxima);
    writeMicroSD("Dados_gyro.txt", dados_gyro);
    writeMicroSD("dados_acce_eixo_principal.txt", dados_acce_eixo_principal);
    while(true);
  }
  */
}

//////////////////// BMP ////////////////////
void iniciaBMP(){
	float t0 = millis();
	while(!bmp.begin(I2C_BMP280)){
		if((millis() - t0) >= 5000){
			digitalWrite(LED_ERRO, HIGH);
			while(true);
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
    gx = gx/lsb_gyro;
    gy = gy/lsb_gyro;
    gz = gz/lsb_gyro;
    
    /*Serial.println(gx);
    Serial.println(gy);
    Serial.println(gz);*/
}

//////////////////// CINEMÁTICA ////////////////////
void descobreEixoPrincipal(){
	updateMPU();
	ac_eixo_principal = (ax/lsb_acc >= 0.9) ? &ax : ((ay/lsb_acc >= 0.9) ? &ay : ((az/lsb_acc >= 0.9) ? &az : &ax));
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
  q[0] += (0.5f)*(-q[1]*gx - q[2]*gy - q[3]*gz)*dt;
  q[1] += (0.5f)*(q[0]*gx + q[2]*gz - q[3]*gy)*dt;
  q[2] += (0.5f)*(q[0]*gy - q[1]*gz + q[3]*gx)*dt;
  q[3] += (0.5f)*(q[0]*gz + q[1]*gy - q[2]*gx)*dt;

  norma = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0] /= norma;
  q[1] /= norma;
  q[2] /= norma;
  q[3] /= norma;

  // converter para angulos de euler
  //angulos_de_euler[0] = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]*q[1] + q[2]*q[2]));
  //angulos_de_euler[1] = asin(2*(q[0]*q[2] - q[3]*q[1]));
  //angulos_de_euler[2] = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
  
  angulos_de_euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
  angulos_de_euler[1] = (-1)/(sin(2*q[1]*q[3] + 2*q[0]*q[2]));
  angulos_de_euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
}

//////////////////// TVC ////////////////////
void startTVC(){
  pid_pitch.setSetPoint(SETPOINT_CALIBRADO);
  pid_yaw.setSetPoint(SETPOINT_CALIBRADO);
}

void updateTVC(){
	dt = (millis() - tempo_ultimo_updateTVC)/1000.0f;
	tempo_ultimo_updateTVC = millis();

  calculaAngulosDeEuler();
  
	float PITCH = angulos_de_euler[1];
	float YAW = angulos_de_euler[2];

  PITCH = PITCH * 57296 / 1000;
  YAW = YAW * 57296 / 1000;

	pid_pitch.addNewSample(map(PITCH, 0, 2*PI, 0, 100));
	pid_yaw.addNewSample(map(YAW, 0, 2*PI, 0, 100));

  //Serial.println(pid_pitch.process());
  //Serial.println(pid_yaw.process());
	//motor_pitch.write(pid_pitch.process());
	//motor_yaw.write(pid_yaw.process());
  //float angulo = pid_yaw.process();
  //Serial.println(angulo);
  //motor_pitch.write(angulo);
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
