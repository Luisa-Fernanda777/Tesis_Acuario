#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_ESP_PH_WITH_ADC.h> 
#include "GravityTDS.hpp"

DFRobot_ESP_PH_WITH_ADC ph;
#define OneWireBus 4
#define ESPADC 4096.0   //the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 //the esp voltage supply value
#define PH_PIN 35    //the esp gpio data pin number

OneWire onewire(OneWireBus); //objeto para pin onewire 
DallasTemperature DS18B20(&onewire);

float temperature = 25,tds_Value = 0, tdstotal = 0,  phValue, voltage_ph;

void setup() {
 Serial.begin(115200);
 DS18B20.begin();
 TDS_Begin();
}

void loop() {
  DS18B20.requestTemperatures(); 
  float temperatureC = DS18B20.getTempCByIndex(0);
  Serial.println(temperatureC);
  
  TDS_Update(temperatureC);
  tds_Value = TDS_Get_Value();
  Serial.println(tds_Value);

  static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
        voltage_ph = analogRead(PH_PIN)/4096.0*3300;  // read the voltage
        phValue = ph.readPH(voltage_ph,temperatureC);  // convert voltage to pH with temperature compensation
        Serial.print("temperature:");
        Serial.print(temperatureC,1);
        Serial.print("^C  pH:");
        Serial.println(phValue,2);
    }
    ph.calibration(voltage_ph,temperatureC);           // calibration process by Serail CMD
    delay(1500);
}
