#include <OneWire.h>
#include <DallasTemperature.h>
#include "GravityTDS.h"

#define OneWireBus 4
OneWire onewire(OneWireBus); //objeto para pin onewire 
DallasTemperature DS18B20(&onewire);

#define TDS_SENSOR_PIN 27
GravityTDS gravityTds;
float temperature = 25,tdsValue = 0, tdstotal = 0;

void setup() {
 Serial.begin(115200);
 DS18B20.begin();
 
 gravityTds.setPin(TDS_SENSOR_PIN);
 gravityTds.setAref(3.3);  //reference voltage on ADC, default 5.0V on Arduino UNO
 gravityTds.setAdcRange(4096);  //1024 for 10bit ADC;4096 for 12bit ADC
 gravityTds.begin();  //initialization
 
}

void loop() {
  DS18B20.requestTemperatures(); 
  float temperatureC = DS18B20.getTempCByIndex(0);
  Serial.println(temperatureC);
  tdsValue = analogRead(TDS_SENSOR_PIN);
  tdstotal = tdsValue * 0.3502;
  Serial.println(tdstotal);

  //temperature = readTemperature();  //add your temperature sensor and read it
    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate 
    tdsValue = gravityTds.getTdsValue();  // then get the value
    Serial.print(tdsValue);
    Serial.println("ppm");
    delay(1500);
}
