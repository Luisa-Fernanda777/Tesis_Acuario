#include "GravityTDS.hpp"


float tdsValue;

void TDS_Begin(void){
    pinMode(PIN_TDS,INPUT);
}
void TDS_Update(float temp){
    int analogValue;
    float voltage, ecValue, ecValue25;
    analogValue = analogRead(PIN_TDS);
    voltage = (float)(analogValue)/(Adc_Range*AREF);
    ecValue=(133.42*voltage*voltage*voltage) - (255.86*voltage*voltage) + (857.39*voltage)*kValue;
    ecValue25  =  ecValue / (1.0+0.02*(temp-25.0));  //temperature compensation
    tdsValue = ecValue25 * TdsFactor;
  }
float TDS_Get_Value(void){
    return tdsValue;
}
