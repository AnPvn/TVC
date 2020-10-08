#include <SPI.h>
#include <SD.h>

bool iniciaSD(){
	if(!SD.begin(PINO_MICRO_SD)){
		digitalWrite(LED_ERRO, HIGH); Serial.println("Erro ao iniciar o módulo micro SD");
	}
}

void writeMicroSD(String arquivo, String texto){
	File dataFile = SD.open(arquivo, FILE_WRITE);
	if(dataFile){
		dataFile.println(texto);
		dataFile.close();
	}
	else digitalWrite(LED_ERRO, HIGH);
}