float q[4] = {1, 0, 0, 0}; // quaternion

float *getAngulosDeEuler(float dt){
  // derivar e integrar de uma vez só
  q[0] += (0.5f)*(q[1]*gx - q[2]*gy - q[3]*gz)*dt;
  q[1] += (0.5f)*(q[0]*gx + q[2]*gz - q[3]*gy)*dt;
  q[2] += (0.5f)*(q[0]*gy - q[1]*gz + q[3]*gx)*dt;
  q[3] += (0.5f)*(q[0]*gz + q[1]*gy - q[2]*gx)*dt;

  // converter para angulos de euler
  float angulos_de_euler[3]; // roll, pitch, yaw
  angulos_de_euler[0] = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]*q[1] + q[2]*q[2]));
  angulos_de_euler[1] = asin(2*(q[0]*q[2] - q[3]*q[1]));
  angulos_de_euler[2] = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));

  return angulos_de_euler;
}
