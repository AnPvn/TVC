#define KP 0
#define KI 0
#define KD 0

#define SETPOINT_CALIBRADO 0

PID pid_pitch(KP, KI, KD);
PID pid_yaw(KP, KI, KD);

float tempo_ultimo_updateTVC = 0.0f;

void startTVC(){
  pid_pitch.setSetPoint(SETPOINT_CALIBRADO);
  pid_yaw.setSetPoint(SETPOINT_CALIBRADO);
}

void updateTVC(Servo motor_pitch, Servo motor_yaw){

  setDt((millis() - tempo_ultimo_updateTVC)/1000.0f);
  tempo_ultimo_updateTVC = millis();
  float pitch = getAnguloDeEuler(1);
  float yaw = getAnguloDeEuler(2);

  pid_pitch.addNewSample(map(pitch, 0, 2*PI, 0, 100));
  pid_yaw.addNewSample(map(yaw, 0, 2*PI, 0, 100));

  motor_pitch.write(pid_pitch.process());
  motor_yaw.write(pid_yaw.process());
}
