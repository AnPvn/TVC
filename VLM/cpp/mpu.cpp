#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_ACCELGYRO

MPU6050 mpu;

int16_t ax, ay, az; // mpu
int16_t gx, gy, gz; // mpu
const float lsb = 16384.00; // mpu

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
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}