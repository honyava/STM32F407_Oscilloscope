
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"
#include "core_cm4.h"
#include "lcd_ili9341.h"
#include "ili9341.h"

void Sys_Clock_168MHz(void);
void SysTick_Handler(void);
void Delay(uint32_t ms);
void SysTick_Init(void);
uint32_t Get_Tick(void);
  
void GPIOA6_Init(void); 
void GPIOA7_Init(void);
void GPIOB1_Init(void); //LCD_BL

void TIM3_Init(void);
void TIM3_IRQHandler(void);
void TIM2_Init(void);
void TIM2_IRQHandler(void);
void Button_K1_Init(void);
void EXTI3_IRQHandler(void);

void ADC1_Init(void);
void ADC3_Init(void);
void ADC1_2_Dual_Init(void);
void DMA_Init(void);
void ADC_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void FSMC_Init(void);


#ifdef __cplusplus
}
#endif

#endif
