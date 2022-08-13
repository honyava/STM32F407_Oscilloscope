#include "main.h"
#include <stdint.h>

#define Convert_to_mV 0.80586f  // 3300mV/4095

extern volatile uint8_t flag_button;
extern volatile uint8_t flag_ADC1;
extern volatile uint8_t flag_DMA_ADC1_2;
extern volatile uint8_t flag_DMA_ADC3;
extern volatile uint16_t ADC1_data;
extern uint16_t ADC1_data_avg[10];
extern uint32_t volatile BUFF_ADC1_2[50];
uint16_t BUFF_ADC2[50] = {0};
extern uint16_t volatile BUFF_ADC3[BUFF_SIZE];
const uint16_t V_25 = 760; //mV
const float Avg_slope = 2.5f; //mV
float volatile ADC1_data_conv = 0;
float volatile Temp_adc = 0;
uint8_t j = 0;

int main(void)
{
	Sys_Clock_168MHz();
	SysTick_Init();
	GPIOA6_Init();
	GPIOA7_Init();
  GPIOB1_Init();
	TIM3_Init();
	TIM2_Init();
	Button_K1_Init();
	DMA_Init();
	ADC3_Init();
	ADC1_2_Dual_Init();
  
  lcdBacklightOn();
  FSMC_Init();
  initLCD();
  lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);
  lcdFillRGB(COLOR_WHITE);
  lcdFillCircle(200,120,10,COLOR_RED);
  //lcdTest();
	//ADC1_Init();

	while(1)
	{
		if(flag_DMA_ADC1_2 == 1)
		{
      flag_DMA_ADC1_2 = 0;
      //ADC1_data = (uint16_t)BUFF_ADC1_2;
			for(uint8_t j = 0; j < 50; j++)
			{
				ADC1_data += (uint16_t)BUFF_ADC1_2[j]; // search sum
        BUFF_ADC2[j] = (BUFF_ADC1_2[j]&0xFFFF0000) >> 16;
			}
			ADC1_data = ADC1_data/50; // search average
			ADC1_data_conv = ADC1_data*Convert_to_mV;
			Temp_adc = ((ADC1_data_conv - V_25)/Avg_slope) + 25; // Temperature by STM32
		}
    ////////////////////For independet ADC1 + button
//		if (flag_button == 1 && flag_ADC1 == 1)
//		{
//			for(uint8_t j = 0; j < 10; j++)
//			{
//				ADC1_data += ADC1_data_avg[j]; // search sum
//			}
//			ADC1_data = ADC1_data/10; // search average
//			ADC1_data_conv = ADC1_data*Convert_to_mV;
//			Temp_adc = ((ADC1_data_conv - V_25)/Avg_slope) + 25; // Temperature by STM32
//			flag_button = 0;
//			flag_ADC1 = 0;		
//		}
		
		//GPIOA->BSRR = GPIO_BSRR_BS6; //set
		//Delay(500);
		//GPIOA->BSRR = GPIO_BSRR_BR6; //reset
		//Delay(500);
		
		
	}
}
