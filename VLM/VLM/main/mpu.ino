#include <Wire.h>

const int MPU = 0x68;
 
void iniciaMPU()
{
  Serial.println("Iniciando MPU...");
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
   
  //Inicializa o acelerômetro
  Wire.write(0); 
  Wire.endTransmission(true);
}
 
void updateMPU()
{

  int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  
  //Solicita os dados do sensor
  Wire.requestFrom(MPU,14,true);  
  
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX=Wire.read()<<8|Wire.read();      
  AcY=Wire.read()<<8|Wire.read(); 
  AcZ=Wire.read()<<8|Wire.read(); 
  Tmp=Wire.read()<<8|Wire.read(); 
  GyX=Wire.read()<<8|Wire.read(); 
  GyY=Wire.read()<<8|Wire.read(); 
  GyZ=Wire.read()<<8|Wire.read(); 

  Serial.println("Lendo dados do MPU...");
  Serial.println(GyX);
  Serial.println(GyY);
  Serial.println(GyZ);
 
  setAx(AcX); setAy(AcY); setAz(AcZ);
  setGx(GyX); setGy(GyY); setGz(GyZ);
}



/*#include "I2Cdev.h"
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
}*/
