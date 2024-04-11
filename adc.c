#include "adc.h"
void adc_1_init(){
	 ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Che do analog input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	

	
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Cho phép che do quét
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // Cho phép che do chuyen doi liên tuc
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
	
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}
// Ðoc giá tri tu ADC
uint16_t adc_1_get_value(void) {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // B?t d?u chuy?n d?i ADC

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); // Cho cho den khi quá trình chuyen doi hoàn tat

    return ADC_GetConversionValue(ADC1); // Tra ve giá tri dã chuyen doi
}
	
	
