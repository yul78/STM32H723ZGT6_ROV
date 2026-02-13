#ifndef __ALLADC_H
#define __ALLADC_H

#include "adc.h"

extern float voltage_value[2];

uint8_t Water_Check(void);
void WaterADC_Init(void);

#endif
