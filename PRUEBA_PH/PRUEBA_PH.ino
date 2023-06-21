float acid_voltage = 1999.5, neutral_voltage = 1490.0;
float voltage_ph,  ph_Value, temp = 25.0;

#define PH_PIN 35

void setup() {
 Serial.begin(115200);
}

void loop() {
  static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        voltage_ph = analogRead(PH_PIN)/4096.0*3300;
        float m =(7.0 - 4.0)/( (neutral_voltage - 1500)/3.0 -(acid_voltage - 1500)/3.0);
        float b = 7.0 - m*((neutral_voltage - 1500)/3.0);
        ph_Value = m * (voltage_ph - 1500)/3.0+ b;
          Serial.println(ph_Value);
        Serial.println(voltage_ph);
    }
      
}
