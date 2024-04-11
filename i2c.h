#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f10x.h"

#define LCD_I2C_DEVICE_ADDRESS      0x4E // Ð?a ch? c?a LCD I2C

// Các hàm public
void lcd_init(void);
void lcd_clear(void);
void lcd_puts(char *string);
void lcd_gotoxy(uint8_t row, uint8_t col);

#endif /* LCD_I2C_H */
