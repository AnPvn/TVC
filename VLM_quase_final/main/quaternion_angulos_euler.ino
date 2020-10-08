float q[4] = {1, 0, 0, 0}; // quaternion
float angulos_de_euler[3]; // roll, pitch, yaw

float dt;

float setDt(float deltatime){
  dt = deltatime;
}

float calculaAngulosDeEuler(){
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

float getAnguloDeEuler(int index){
  return angulos_de_euler[index];
}
