//Definicion de librerias incluidas
#include <Arduino.h>
#include "DFRobot_ESP_PH_WITH_ADC.h"
#include "EEPROM.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

//Definicion de los pines a usar para el sensor TDS y las muestras por SCOUNT
#define TdsSensorPin 27
#define VREF 3.3 // analog reference voltage(Volt) of the ADC
#define SCOUNT  5 // 5 muestras

//Declaracion de variables para medicion de ph

DFRobot_ESP_PH_WITH_ADC ph;
#define ESPADC 4096.0   //the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 //the esp voltage supply value
#define PH_PIN 35		//the esp gpio data pin number
float voltage, phValue, temperature = 25;

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;

//Declaracion variables TDS y turbidez
float tds = 0;
float tdsValue = 0;
float temptds = 25; // current temperature for compensation
float turbidez = 0;

//Declaracion variables controlador on/off
int calentador = 2;
int led_calentador =18;


void setup() {
  
  Serial.begin(115200);
  sensors.begin();

  //sensor de ph
  EEPROM.begin(32);//needed to permit storage of calibration value in eeprom
	ph.begin();

  /*pinMode(TdsSensorPin,INPUT);
  Start the DS18B20 sensor*/
  pinMode(calentador,OUTPUT);// configuracion del pin de control del calentador como salida
  pinMode(led_calentador,OUTPUT);
}


// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void loop() {

  // MEDICION DE PH 

  static unsigned long timepoint = millis();
	if (millis() - timepoint > 1000U) //time interval: 1s
	{
		timepoint = millis();
		
		voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
		Serial.print("voltage:");
		Serial.println(voltage, 4);
		
		//temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
		Serial.print("temperature:");
		Serial.print(temperature, 1);
		Serial.println("^C");

		phValue = ph.readPH(voltage, temperature); // convert voltage to pH with temperature compensation
		Serial.print("pH:");
		Serial.println(phValue, 4);
	}
	ph.calibration(voltage, temperature); // calibration process by Serail CMD

  //MEDICION DE TEMPERATURA

  //Aignacion de valor digital en temperature C o F
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  //float temperatureF = sensors.getTempFByIndex(0);

  
  //VISUALIZACION DE DATOS
  
  //Impresion de valores TDS y turbidez
  Serial.print("TDS:");
  Serial.print(tdsValue,0);
  Serial.println(" PPM");
  Serial.print("TURBIDEZ:");
  Serial.print(turbidez,0);
  Serial.println(" NTU");

  //Impresion de valores sensor DS18B20
  Serial.print("TEMPERATURA:");
  Serial.print(temperatureC);
  Serial.println(" ºC");
  //Serial.print(temperatureF);
  //Serial.println("ºF");

  //CONTROLADOR ON/OFF
   if (temperatureC >28.5){
    digitalWrite(calentador,LOW);
    digitalWrite(led_calentador,LOW);
    Serial.println("calentador Apagado");
   }

   if (temperatureC <= 28.5){
    digitalWrite(calentador,HIGH);
    digitalWrite(led_calentador,HIGH);
    Serial.println("calentador Encendido");
   }
  
  delay(2000);
}