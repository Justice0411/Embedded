#ifndef _ADC_H_
#define _ADC_H_
#include "stm32f10x.h"                  // Device header

void adc_1_init();

uint16_t adc_1_get_value(void);

#endif