#include <Servo.h>

#define PIN_SERVO_PITCH 10
#define PIN_SERVO_YAW 9
#define IGNITOR_PARAQUEDAS 8
#define BUZZER 7
#define LED_ERRO 6
#define LED_DADOS_SALVOS_CARTAO_SD 5
#define PINO_MICRO_SD 4

Servo motor_pitch;
Servo motor_yaw;

bool deixarDeVerificarApogeu = false;

String dados_gyro = "";
String dados_acce_eixo_principal = "Aceleracao Eixo Principal/n";

void setup(){
	Serial.begin(9600);

	pinMode(BUZZER, OUTPUT);
	pinMode(LED_ERRO, OUTPUT);
	pinMode(LED_DADOS_SALVOS_CARTAO_SD, OUTPUT);
	pinMode(IGNITOR_PARAQUEDAS, OUTPUT);

	iniciaSD();

	motor_pitch.attach(PIN_SERVO_PITCH);
	motor_yaw.attach(PIN_SERVO_YAW);

	iniciaMPU();
	iniciaBMP();

	primeira_altura = getAltitude();

	descobreEixoPrincipal();
	
	while(verificaLancamentoIniciado() == false);

}

void loop(){
	updateMPU();
	ultima_altura = altura;
	altura = getAltitude();

	AtualizaAlturaMaxima()

	if(!verificaBurnout()) updateTVC();


	if(!deixarDeVerificarApogeu && verificaApogeu()){ 
		digitalWrite(IGNITOR_PARAQUEDAS, HIGH);
		deixarDeVerificarApogeu = true;
	}

	if(!verificaLancamentoTerminado()){
		dados_gyro += ("GX: " + gx + ",    GY: " + gy + ",    GZ: " + gz + ";/n");
		dados_acce_eixo_principal += (*ac_eixo_principal + ";/n");
	}


	if(deixarDeVerificarApogeu && verificaLancamentoTerminado()){
		writeMicroSD("Altura_maxima.txt", (String) altura_maxima);
		writeMicroSD("Dados_gyro.txt", dados_gyro);
		writeMicroSD("dados_acce_eixo_principal.txt", dados_acce_eixo_principal);
		delay(2000);
		digitalWrite(LED_DADOS_SALVOS_CARTAO_SD, HIGH);
	}

}