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
bool deixarDeSalvarDados = false;

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

  startTVC();

  iniciaMPU();
  iniciaBMP();

  setPrimeiraAltura(getAltitude());

  descobreEixoPrincipal();
  
  for(int i = 0; i < 10; i++){
  	digitalWrite(BUZZER, HIGH);
  	delay(100);
  	digitalWrite(BUZZER, LOW);
  	delay(900);
  }

  digitalWrite(BUZZER, HIGH);

  while(verificaLancamentoIniciado() == false);

  digitalWrite(BUZZER, LOW);

}

void loop(){
  
  setUltimaAltura(getAltura()); // getAltura != getAltitude
  setAltura(getAltitude()); 

  AtualizaAlturaMaxima();

  if(!verificaBurnout()) updateTVC(motor_pitch, motor_yaw);


  if(!deixarDeVerificarApogeu && verificaApogeu()){ 
    digitalWrite(IGNITOR_PARAQUEDAS, HIGH);
    deixarDeVerificarApogeu = true;
  }

  if(!verificaLancamentoTerminado()){
    dados_gyro += ("GX: " + (String) getGx() + ",    GY: " + getGy() + ",    GZ: " + getGz() + ";/n");
    dados_acce_eixo_principal += (((String) getAcEixoPrincipal()) + ";/n");
  }


  if(deixarDeVerificarApogeu && verificaLancamentoTerminado() && !deixarDeSalvarDados){
  	digitalWrite(BUZZER, HIGH);
    writeMicroSD("Altura_maxima.txt", (String) getAlturaMaxima());
    writeMicroSD("Dados_gyro.txt", dados_gyro);
    writeMicroSD("dados_acce_eixo_principal.txt", dados_acce_eixo_principal);
    delay(2000);
    digitalWrite(LED_DADOS_SALVOS_CARTAO_SD, HIGH);
    digitalWrite(BUZZER, LOW);
    deixarDeSalvarDados = true;
  }

}