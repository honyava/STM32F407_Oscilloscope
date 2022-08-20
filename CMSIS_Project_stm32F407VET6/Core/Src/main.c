#include "main.h"
#include <stdint.h>
#include "arm_math.h"
//#include "arm_const_structs.h"
//#define FFT_SIZE 256
#define size_x  280U   // 35*8
#define size_y  200U   // 25*8

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
extern uint16_t BUFF_ADC3[BUFF_SIZE];
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
uint16_t maxValue_fft = 0;
uint32_t maxIndex_fft = 0;
uint16_t value_min, value_max;

uint16_t buff_x[size_x] = {0}; // 280 element in a row for 0x in grid
uint16_t buff_y[size_y] = {0}; // 200 element in a row for 0y in grid
uint16_t x_scale = 0;
uint16_t y_scale = 0;

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
  uint32_t frequance = 0;
  //lcdSetCursor(75, 5);
  //lcdPrintf("%0.2d Hz",frequance); // frequance
  lcdSetCursor(65, 225);
  t_del = (T*35/280)*1000; // ms
  lcdPrintf("%0.3f ms ",t_del); // Time
	//ADC1_Init();
  status = arm_rfft_init_q15(&S, BUFF_SIZE, 0, 1);
  
	while(1)
	{
    lcdGrid(20, 20, COLOR_WHITE);
    
    if(flag_DMA_ADC3 == 1 && flag_lcd_update == 1 && flag_button == 1)
    {
      if(status == ARM_MATH_SUCCESS)
      {
        //lcdDisplayOn();
        arm_rfft_q15(&S,(q15_t*)BUFF_ADC3, fft_Dbuff); // fft
        arm_cmplx_mag_q15(fft_Dbuff, (q15_t*)BUFF_ADC3, BUFF_SIZE); //magnitude for fft
        arm_max_q15(fft_Dbuff, BUFF_SIZE, (q15_t*)&maxValue_fft, &maxIndex_fft); // search max value
        Amplitude = maxValue_fft*Convert_to_mV; // convert max value in mV
        frequance = maxIndex_fft*10000/BUFF_SIZE; // Fd/BUFF_SIZE = sample on 1 count
        //lcdFillRect(0, 0, 320, 15, COLOR_BLACK); // Clear area lcd
        lcdSetCursor(215, 5);
        lcdPrintf("%d", Amplitude); // Amplitude
        lcdSetCursor(245, 5);
        lcdPrintf("mV");
        lcdSetCursor(75, 5);
        lcdPrintf("%d",frequance); // frequance
        lcdSetCursor(105, 5);
        lcdPrintf("Hz");
      }
      flag_DMA_ADC3 = 0;
      flag_lcd_update = 0;
    }
    else if(flag_DMA_ADC3 == 1 && flag_lcd_update == 1 && flag_button == 0)
    {
      min_max_elems(BUFF_ADC3, BUFF_SIZE, &value_min, &value_max);
      y_scale = (value_max - value_min)/(size_y - 1);
      for(uint16_t i = 0; i < BUFF_SIZE; i++)
      {
        BUFF_ADC3[i] = BUFF_ADC3[i]*y_scale; //Make values from ADC3 for format of display
      }
      //lcdDisplayOff();
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
