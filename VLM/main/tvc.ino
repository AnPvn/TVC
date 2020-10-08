#define KP 0
#define KI 0
#define KD 0

#define SETPOINT_CALIBRADO 0

PID pid_pitch(KP, KI, KD);
PID pid_yaw(KP, KI, KD);

pid_pitch.setSetPoint(SETPOINT_CALIBRADO);

void updateTVC(){

  float angulos[3] = getAngulosDeEuler();
  pitch = angulos[1];
  yaw = angulos[2];

  pid_pitch.addNewSample(map(pitch, 0, 2*PI, 0, 100));
  pid_yaw.addNewSample(map(yaw, 0, 2*PI, 0, 100));

  motor_pitch.write(pid_pitch.process());
  motor_yaw.write(pid_yaw.process());
}
