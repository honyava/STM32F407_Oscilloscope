#include "main.h"
#include <stdint.h>
#include "arm_math.h"
//#include "arm_const_structs.h"
#define FFT_SIZE 256

q15_t fft_Dbuff[BUFF_SIZE*2] = {0};


extern volatile uint8_t flag_button;
extern volatile uint8_t flag_ADC1;
extern volatile uint8_t flag_DMA_ADC1_2;
extern volatile uint8_t flag_DMA_ADC3;
extern volatile uint8_t flag_lcd_update;
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

float T = 1.0/Fs; // Sampling time
float t_del = 0.0; // time of 1 division
uint16_t volatile Amplitude = 0;

arm_rfft_instance_q15 S;
uint16_t k;
uint8_t status;

int main(void)
{
	Sys_Clock_168MHz();
	SysTick_Init();
	GPIOA6_Init();
	GPIOA7_Init();
  GPIOB1_Init();
  lcdBacklightOn();
  FSMC_Init();
  lcdInit();
  lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE_MIRROR);
  lcdFillRGB(COLOR_BLACK);
  lcdGrid(20, 20, COLOR_WHITE);
  //lcdFillCircle(30,30,10,COLOR_WHITE);
  
  TIM4_Init();
	TIM3_Init();
	TIM2_Init();
	Button_K1_Init();
	DMA_Init();
	ADC3_Init();
	ADC1_2_Dual_Init();
  float a = 1.2;
  lcdSetCursor(75, 5);
  lcdPrintf("%0.2f Hz",a); // frequance
  lcdSetCursor(215, 5);
  lcdPrintf("%0.2f V ",a); // Amplitude
  lcdSetCursor(65, 225);
  t_del = (T*35/280)*1000; // ms
  lcdPrintf("%0.3f ms ",t_del); // Time
	//ADC1_Init();
  status = arm_rfft_init_q15(&S, BUFF_SIZE, 0, 1);
  
	while(1)
	{
    
    if(flag_DMA_ADC3 == 1 && flag_lcd_update == 1)
    {
      if(status == ARM_MATH_SUCCESS)
      {
        //Delay(10);
        arm_rfft_q15(&S,(q15_t*)BUFF_ADC3, fft_Dbuff); //
        arm_cmplx_mag_q15(fft_Dbuff, (q15_t*)BUFF_ADC3, FFT_SIZE); //
        lcdSetCursor(215, 5);
        Amplitude = fft_Dbuff[0]*Convert_to_mV;
        lcdPrintf("%d mV", Amplitude); // Amplitude
      }
      
      flag_DMA_ADC3 = 0;
      flag_lcd_update = 0;
    }
    
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
