#include "lcd_i2c.h"
#include "delay.h"


static GPIO_InitTypeDef GPIO_InitStructure;
static I2C_InitTypeDef I2C_InitStructure;

void I2C_LCD_Configuration(void);

void I2C_LCD_Configuration(void)
{
    // cap clock cho ngoai vi va I2C
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);							// su dung kenh I2C2 cua STM32
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // cau hinh chan SDA va SCL
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;						//PB10 - SCL, PB11 - SDA
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // cau hinh I2C
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0; // 
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;       
    I2C_InitStructure.I2C_ClockSpeed = 100000;										
    I2C_Init(I2C2, &I2C_InitStructure);
    // cho phep bo I2C hoat dong
    I2C_Cmd(I2C2, ENABLE);

}

void lcd_Write_byte(char data)
{
    /* Send START condition */
    I2C_GenerateSTART(I2C_Chanel, ENABLE);
    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C_Chanel, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send PCF8574A address for write */
    I2C_Send7bitAddress(I2C_Chanel, PCF8574A_Address, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C_Chanel, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /* Send the byte to be written */
    I2C_SendData(I2C_Chanel, data);
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_Chanel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /* Send STOP condition */
    I2C_GenerateSTOP(I2C_Chanel, ENABLE);		
}
void lcd_init (void)
{
	delay_init();
	I2C_LCD_Configuration();
	/* Set 4-bits interface */
	lcd_Control_Write(0x33);		 
	delay_ms(10);
	lcd_Control_Write(0x32);
	delay_ms(50);
	/* Start to set LCD function */
	lcd_Control_Write(0x28);
		delay_ms(50);	
	/* clear LCD */
	lcd_Control_Write(0x01);
		delay_ms(50);
	/* wait 60ms */
	
	/* set entry mode */
	lcd_Control_Write(0x06);	delay_ms(50);;
	/* set display to on */	
	lcd_Control_Write(0x0C);	delay_ms(50);;
	/* move cursor to home and set data address to 0 */
	lcd_Control_Write(0x02);	delay_ms(50);
}
void lcd_Data_Write(char data)
{
	char data_u, data_l;
	uint8_t data_t[4],i=0;
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x0d;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0d;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	for(i = 0;i<4;i++)
	{
	     lcd_Write_byte(data_t[i]);
		delay_us(100);
	}

}
void lcd_Control_Write(char data)
{
    char data_u, data_l;
	uint8_t data_t[4],i=0;
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x04;  //en=1, rs=0
	data_t[1] = data_u;  //en=0, rs=0
	data_t[2] = data_l|0x04;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	for(i = 0;i<4;i++)
	{
	     lcd_Write_byte(data_t[i]);
			delay_us(100);
	}

}
void lcd_send_string (char *str)
{
	while (*str) lcd_Data_Write (*str++);
}
void Delete_LCD(void)
{
	lcd_Control_Write(0x01);
	delay_ms(50);
}

void lcd_gotoxy(uint8_t x,uint8_t y)
{
	unsigned char Address;
	if(y == 0) 
		Address = 0x80 + x;
	if(y == 1)
		Address = 0xc0 + x;
	lcd_Control_Write(Address);
}

