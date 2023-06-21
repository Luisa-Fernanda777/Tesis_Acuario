#ifndef GRAVITY_TDS_H
#define GRAVITY_TDS_H

#include "Arduino.h"

#define PIN_TDS    27      // Seleccionar pin de la ESP para ADC
#define Temp       25.0    // Seleccionar la temperatura a trabajar
#define TdsFactor  0.5
#define Adc_Range  4096.0
#define AREF       3.3
#define kValue     1.0

void TDS_Begin(void);
void TDS_Update(float tem);
float TDS_Get_Value(void);

#endif
