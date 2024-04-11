#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "lcd_i2c.h"
#include <stdio.h>
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "adc.h"

#define LIGHT_SENSOR_PIN        GPIO_Pin_0  //pin A0
#define ADC_CHANNEL             ADC_Channel_0
#define THRESHOLD               100
#define I2C_DEVICE_ADDRESS      0x50

#define LED_GPIO_PORT           GPIOA
#define LED_GPIO_PIN            GPIO_Pin_8  // Example pin for the LED

#define BUTTON1_GPIO_PORT       GPIOB
#define BUTTON1_GPIO_PIN        GPIO_Pin_0  // Example pin for Button 1
#define BUTTON2_GPIO_PORT       GPIOB
#define BUTTON2_GPIO_PIN        GPIO_Pin_1  // Example pin for Button 2

xSemaphoreHandle xMutex;

typedef enum {
    AUTOMATIC_MODE,
    MANUAL_MODE
} ControlMode;

ControlMode control_mode = AUTOMATIC_MODE;

float map(float value, float from_low, float from_high, float to_low, float to_high);

uint16_t luxvalue = 0; // Global variable to store lux value
//int button1_press_count = 0; // Bi?n d?m s? l?n nh?n nút nh?n 1
ControlMode previous_control_mode = AUTOMATIC_MODE; //Bien luu tr?ng thái tru?c dó c?a ch? d?

void vTask_ReadLightSensor(void *pvParameters);
void vTask_ControlLED(void *pvParameters);
void vTask_CheckButtons(void *pvParameters);

int main(void) {
    SystemInit();
    SystemCoreClockUpdate();
    xMutex = xSemaphoreCreateMutex();
    lcd_init();      
    adc_1_init();

    // Create tasks
    xTaskCreate(vTask_ReadLightSensor, "ReadLight", 128, NULL, 1, NULL);
    xTaskCreate(vTask_ControlLED, "ControlLED", 128, NULL, 1, NULL);
    xTaskCreate(vTask_CheckButtons, "CheckButtons", 128, NULL, 1, NULL);
    
    // Start the scheduler
    vTaskStartScheduler();
    
    return 0;
}

void vTask_ReadLightSensor(void *pvParameters) {
    char tmp[50];
    while (1) {
        // Read data from light sensor via ADC
        xSemaphoreTake(xMutex, portMAX_DELAY);
        lcd_gotoxy(0, 0);
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        luxvalue = map(ADC_GetConversionValue(ADC1),0,4095,0,170);
        sprintf(tmp,"Light:%d", luxvalue);
        lcd_send_string(tmp);
              
        xSemaphoreGive(xMutex);  
        vTaskDelay(600);
    }  
}

void GPIO_config() {
    // Configure LED GPIO
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStruct.GPIO_Pin = LED_GPIO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    // Configure Button 1 GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStruct.GPIO_Pin = BUTTON1_GPIO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;  // Input Pull-up
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUTTON1_GPIO_PORT, &GPIO_InitStruct);

    // Configure Button 2 GPIO
    GPIO_InitStruct.GPIO_Pin = BUTTON2_GPIO_PIN;
    GPIO_Init(BUTTON2_GPIO_PORT, &GPIO_InitStruct);
}

void vTask_ControlLED(void *pvParameters) {
    GPIO_config();
    while (1) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            if (control_mode == AUTOMATIC_MODE) {
                if (luxvalue > THRESHOLD) {
                    GPIO_SetBits(LED_GPIO_PORT , LED_GPIO_PIN );
                } else {
                    GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
                }
				
            }
        }  
        xSemaphoreGive(xMutex);       
        vTaskDelay(100); // Delay before re-checking
    }
}
void vTask_CheckButtons(void *pvParameters) {
    portTickType xLastWakeTime  ;
    const portTickType xButtonCheckPeriod = 20 / portTICK_RATE_MS; // Ki?m tra m?i 50ms
    GPIO_config(); // C?u hình GPIO cho các nút nh?n

    xLastWakeTime = xTaskGetTickCount();

while (1) {
    // Kiem tra trang thái cua nút nhan 1
    if (GPIO_ReadInputDataBit(BUTTON1_GPIO_PORT, BUTTON1_GPIO_PIN) == 0) {
        // Ð?o ngu?c giá tri cua control_mode
        if (control_mode == MANUAL_MODE) {
            control_mode = AUTOMATIC_MODE;
        } else {
            control_mode = MANUAL_MODE;
        }
        // Th?c hi?n các hành d?ng tùy thu?c vào control_mode t?i dây
        // Ví d?: Hi?n th? tr?ng thái m?i c?a control_mode, ho?c th?c hi?n các hành d?ng khác
				if (control_mode != previous_control_mode) {
                char tmp[15];
                sprintf(tmp,"Mode:%s", (control_mode == AUTOMATIC_MODE) ? "AUTO" : "MANUAL");
                lcd_gotoxy(0,1);
                lcd_send_string(tmp);
            }

            // C?p nh?t ch? d? tru?c dó v?i ch? d? hi?n t?i
            previous_control_mode = control_mode;
        }

        // Các cài d?t khác
				// Ki?m tra tr?ng thái c?a nút nh?n 2
        if (GPIO_ReadInputDataBit(BUTTON2_GPIO_PORT, BUTTON2_GPIO_PIN) == 0) {
            if (control_mode == MANUAL_MODE) {
                if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                    // Toggle tr?ng thái c?a LED khi ? ch? d? th? công
                    if (GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN) == Bit_RESET) {
                        GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
                    } else {
                        GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
                    }
                    xSemaphoreGive(xMutex);
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xButtonCheckPeriod);
    }
}

float map(float value, float from_low, float from_high, float to_low, float to_high) {
    float from_range = from_high - from_low;
    float to_range = to_high - to_low;
    float scaled_value = (value - from_low) / from_range;
    float mapped_value = to_low + (scaled_value * to_range);

    if(mapped_value < to_low) {
        return to_low;
    } else if(mapped_value > to_high) {
        return to_high;
    } else {
        return mapped_value;
    }
}
//void vTask_CheckButtons(void *pvParameters) {
//    portTickType xLastWakeTime;
//    const portTickType xButtonCheckPeriod = 50 / portTICK_RATE_MS; // Check every 50ms

//    GPIO_config(); // Configure GPIO for buttons

//    xLastWakeTime = xTaskGetTickCount();
//    while (1) {
//        // Check Button 1 state
//        if (GPIO_ReadInputDataBit(BUTTON1_GPIO_PORT, BUTTON1_GPIO_PIN) == 0) {
//            if(control_mode == MANUAL_MODE){
//							control_mode = AUTOMATIC_MODE;					    
//			}
//   }
//        // Check Button 2 state
//        if (GPIO_ReadInputDataBit(BUTTON2_GPIO_PORT, BUTTON2_GPIO_PIN) == 0) {
//            if (control_mode == MANUAL_MODE) {
//                if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
//                    // Toggle LED state manually
//                    if (GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN) == Bit_RESET) {
//                        GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
//                    } else {
//                        GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
//                    }
//                    xSemaphoreGive(xMutex);
//                }
//            }
//        }

//        vTaskDelayUntil(&xLastWakeTime, xButtonCheckPeriod);
//    }
//	}
//int button1_press_count = 0; // Bi?n d?m s? l?n nh?n nút nh?n 1
//ControlMode previous_control_mode = AUTOMATIC_MODE; Bi?n luu tr?ng thái tru?c dó c?a ch? d?
