#ifndef _DELAY_H_
#define _DELAY_H_

#include "stm32f10x.h" 

void delay_init(void);
void delay_ms(uint32_t t);
void delay_us(uint32_t t);

#endif