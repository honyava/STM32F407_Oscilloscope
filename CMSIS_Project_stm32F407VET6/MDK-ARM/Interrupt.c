#include "main.h"

volatile uint32_t SysTimer_ms = 0;
volatile uint32_t Delay_ms = 0;
volatile uint8_t flag_tim3 = 0;
volatile uint8_t flag_tim2 = 0;
volatile uint8_t flag_button = 0;
volatile uint8_t flag_ADC1 = 0;
volatile uint8_t flag_DMA_ADC1_2 = 0;
volatile uint8_t flag_DMA_ADC3 = 0;
volatile uint16_t ADC1_data = 0;
uint16_t ADC1_data_avg[10] = {0};
volatile uint8_t ADC1_Counter = 0;
uint32_t volatile BUFF_ADC1_2[50] = {0};
uint16_t volatile BUFF_ADC3[BUFF_SIZE] = {0};
uint16_t volatile BUFF_ADC1[50] = {0};

//////////////////////////My func for Interrupt
void Delay(uint32_t ms)
{
	Delay_ms = ms;
	while(Delay_ms != 0);
}
//////////////////////////
void SysTick_Handler(void)
{
	SysTimer_ms++;
	if(Delay_ms > 0) Delay_ms--;
}
uint32_t Get_Tick(void)
{
  return SysTimer_ms;
}
void TIM2_IRQHandler(void)
{
	if(READ_BIT(TIM2->SR, TIM_SR_UIF)) // check the flag of interrupt
	{
		TIM2->SR &= ~ TIM_SR_UIF; // Resetting the flag of interrupt
		if(flag_tim2 == 0)
		{
			GPIOA->BSRR = GPIO_BSRR_BR7;
			flag_tim2 = 1;
		}
		else
		{
			GPIOA->BSRR = GPIO_BSRR_BS7;
			flag_tim2 = 0;
		}		
	}
}

void TIM3_IRQHandler(void)
{
	if(READ_BIT(TIM3->SR, TIM_SR_UIF)) // check the flag of interrupt
	{
		TIM3->SR &= ~ TIM_SR_UIF; // Resetting the flag of interrupt
		if(flag_tim3 == 0)
		{
			GPIOA->BSRR = GPIO_BSRR_BR6;
			flag_tim3 = 1;
		}
		else
		{
			GPIOA->BSRR = GPIO_BSRR_BS6;
			flag_tim3 = 0;
		}
	}
}

void EXTI3_IRQHandler(void)
{
	EXTI->PR = (1 << EXTI_PR_PR3_Pos);
	flag_button = 1;
}
void ADC_IRQHandler(void)
{
	if(ADC3->SR & ADC_SR_EOC)
  {
		ADC3->SR = 0;
		if(ADC1_Counter == 10)
		{
			flag_ADC1 = 1;
			ADC1_Counter = 0;
			ADC1_data = 0;
		}		
		ADC1_data_avg[ADC1_Counter] = ADC3->DR;
		ADC1_Counter++;
	}
}

void DMA2_Stream0_IRQHandler(void) // for ADC1_2 (dual)
{
	if(READ_BIT(DMA2->LISR, DMA_LISR_TCIF0))
	{
		DMA2->LIFCR = DMA_LIFCR_CTCIF0;
		ADC1_data = 0;
		flag_DMA_ADC1_2 = 1;
	}
}

void DMA2_Stream1_IRQHandler(void) // for ADC3
{
	if(READ_BIT(DMA2->LISR, DMA_LISR_TCIF1))
	{
		DMA2->LIFCR = DMA_LIFCR_CTCIF1;
		ADC1_data = 0;
		flag_DMA_ADC3 = 1;
	}
}


