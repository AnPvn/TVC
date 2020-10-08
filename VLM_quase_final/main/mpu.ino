#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_ACCELGYRO

MPU6050 mpu;

void iniciaMPU(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    if(mpu.testConnection() == false){
      digitalWrite(LED_ERRO, HIGH);
      while(true);
    }
}

void updateMPU(){
  int16_t AX, AY, AZ, GX, GY, GZ;
  mpu.getMotion6(&AX, &AY, &AZ, &GX, &GY, &GZ);
  setAx(AX); setAy(AY); setAz(AZ);
  setGx(GX); setGy(GY); setGz(GZ);
}
