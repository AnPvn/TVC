#include <Wire.h>         
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define I2C_BMP280 0x76

#define PASCALS_NIVEL_DO_MAR 1013.25

Adafruit_BMP280 bmp;

void iniciaBMP(){
	while(!bmp.begin(I2C_BMP280));
}

float getAltitude(){
	bmp.readAltitude(PASCALS_NIVEL_DO_MAR);
}