#include "main.h"


extern volatile uint16_t ADC1_data;
extern uint32_t volatile BUFF_ADC1_2[50];
extern uint16_t volatile BUFF_ADC3[BUFF_SIZE];
extern uint16_t volatile BUFF_ADC1[50];

void Sys_Clock_168MHz(void)
{
	CLEAR_REG(RCC->CR);
	CLEAR_REG(RCC->CFGR);
	CLEAR_REG(RCC->PLLCFGR);
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS); // for flash memory
	RCC->CR |= (1 << RCC_CR_HSION_Pos); //HSI on
	while(READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RESET);
	RCC->CR |= (1 << RCC_CR_HSEON_Pos); //HSE on
	while(READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET);
	RCC->CR &= ~(1 << RCC_CR_HSEBYP_Pos); //Bypass off
	
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); // Chose PLL for System Clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL); // Chose PLL for System Clock switch status
	SET_BIT(RCC->CFGR,RCC_CFGR_HPRE_DIV1); //AHB prescaler = 1
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV4); // APB1 prescaler = 4
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV2); // APB2 prescaler = 2
	RCC->CFGR &= ~(1 << RCC_CFGR_RTCPRE_Pos); //HSE division fo RTC = no clock
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, 4 << RCC_PLLCFGR_PLLM_Pos); // division 4
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, 168 << RCC_PLLCFGR_PLLN_Pos); // mult 168
	MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ, 4 << RCC_PLLCFGR_PLLQ_Pos); // division 4
	RCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLP_Pos); // division 2	
	SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);

	RCC->CR |= (1 << RCC_CR_PLLON_Pos); // Start PLL
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN); // Clock for GPIOH (Crystall Resonator)
	while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET);
}

void SysTick_Init(void)
{
	SysTick->CTRL &= ~(1 << SysTick_CTRL_ENABLE_Pos); // off
	SysTick->CTRL |= (1 << SysTick_CTRL_TICKINT_Pos);
	SysTick->CTRL |= (1 << SysTick_CTRL_CLKSOURCE_Pos); //168 MHz
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 167999UL << SysTick_LOAD_RELOAD_Pos); // set on 1 ms
	MODIFY_REG(SysTick->LOAD, SysTick_VAL_CURRENT_Msk, 167999UL << SysTick_VAL_CURRENT_Pos); // start count from 167999 to 0
	SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos); //on
}

void GPIOA6_Init(void)
{
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED6_Msk, 0x10 << GPIO_OSPEEDR_OSPEED6_Pos); // High speed
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE6_Msk, 0x01 << GPIO_MODER_MODE6_Pos); //Output PA6, Push-Pull
	//CLEAR_REG(GPIOA->OTYPER);
}

void GPIOA7_Init(void)
{
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED7_Msk, 0x10 << GPIO_OSPEEDR_OSPEED7_Pos); // High speed
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE7_Msk, 0x01 << GPIO_MODER_MODE7_Pos); //Output PA7, Push-Pull
	//CLEAR_REG(GPIOA->OTYPER);
}

void GPIOB1_Init(void)
{
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN); // for LCD
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED1_Msk, 0x10 << GPIO_OSPEEDR_OSPEED1_Pos); // High speed
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE1_Msk, 0x01 << GPIO_MODER_MODE1_Pos); //Output PB1, Push-Pull  
}

void Button_K1_Init(void)
{
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);
	MODIFY_REG(GPIOE->OSPEEDR, GPIO_OSPEEDR_OSPEED6_Msk, 0x10 << GPIO_OSPEEDR_OSPEED6_Pos); // High speed
	GPIOE->MODER &= ~(1 << GPIO_MODER_MODE3_Pos); // Input PE3
	GPIOE->PUPDR |= (0x01 << GPIO_PUPDR_PUPD3_Pos); // Pull-Up
	CLEAR_REG(GPIOE->OTYPER);	
	
	//EXTI3 for PE3
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN); //Clock for SYSCF
	MODIFY_REG(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI3, 4 << SYSCFG_EXTICR1_EXTI3_Pos); // EXTI3 For PE3 
	EXTI->IMR |= (1 << EXTI_IMR_MR3_Pos); // Enable interrupt
	EXTI->RTSR |= (1 << EXTI_RTSR_TR3_Pos); // Rasing trigger
	NVIC_EnableIRQ(EXTI3_IRQn);
	
}
void TIM2_Init(void)
{
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN); //clock to TIM2 (84 MHz)
	TIM2->SMCR &= ~ TIM_SMCR_SMS; 
	CLEAR_REG(TIM2->CR1);
	TIM2->PSC = 84;
	TIM2->ARR = 1000; //1000 Hz
	TIM2->DIER |= TIM_DIER_UIE; //interrupt on
	TIM2->CR1 &= ~TIM_CR1_DIR_Msk; // straight count
	
	MODIFY_REG(TIM2->CR2, TIM_CR2_MMS, 2 << TIM_CR2_MMS_Pos); // Update Event for ADC3
	SET_BIT(TIM2->CR1, TIM_CR1_CEN_Msk); // TIM2 enable
	//NVIC_SetPriority(1, TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);	
}
void TIM3_Init(void)
{
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN); //clock to TIM3 (84 MHz)
	TIM3->SMCR &= ~ TIM_SMCR_SMS; 
	CLEAR_REG(TIM3->CR1);
	TIM3->PSC = 8400;
	TIM3->ARR = 100; //100 Hz
	TIM3->DIER |= TIM_DIER_UIE; //interrupt on
	TIM3->CR1 &= ~TIM_CR1_DIR_Msk; // straight count
	
	MODIFY_REG(TIM3->CR2, TIM_CR2_MMS, 2 << TIM_CR2_MMS_Pos); // Update Event for ADC1
	SET_BIT(TIM3->CR1, TIM_CR1_CEN_Msk); // TIM3 enable
//	NVIC_SetPriority(1, TIM3_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);
}

void ADC1_Init(void) // for Temperature
{
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); // GPIOA Clock
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_ADC1EN); // Clock for ADC1
	SET_BIT(ADC1->CR1, ADC_CR1_EOCIE); // Enable interrupt fo EOC
	CLEAR_BIT(ADC1->CR1, ADC_CR1_RES_0);
	CLEAR_BIT(ADC1->CR1, ADC_CR1_RES_1); // Resolution 12 bit
	SET_BIT(ADC1->CR2, ADC_CR2_ADON); // A/D Converter ON 
	SET_BIT(ADC->CCR, ADC_CCR_TSVREFE); // Choose temperatute sensor
	ADC1->SQR3 = 0x10; // 16 channel for first conversation
	CLEAR_BIT(ADC1->CR2, ADC_CR2_ALIGN); // Right alignment
	MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, 8 << ADC_CR2_EXTSEL_Pos); // Externel event Timer 3 TRGO
	MODIFY_REG(ADC1->CR2, ADC_CR2_EXTEN, 1 << ADC_CR2_EXTEN_Pos); //External TRG enable, Rising edge
	SET_BIT(ADC1->CR2, ADC_CR2_DMA); // Enable DMA
	ADC1->CR2 |= ADC_CR2_DDS;  // For enable DMA+ADC(independet)
	SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); //  Starts conversion of regular channels
	MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP16, 1 << ADC_SMPR1_SMP16_Pos); // Channel 16, 15 cycles for conversation
	// Init DMA
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR; // Adress of data
  DMA2_Stream0->M0AR =(uint32_t)&BUFF_ADC1[0]; // Adress of buffer
	SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TCIE); // Interrupt enable, complete transfer
	CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_DIR); // perifheral to memmory
	SET_BIT(DMA2_Stream0->CR, DMA_SxCR_CIRC); // circual mode disabled
	DMA2_Stream0->CR |= (1 << DMA_SxCR_PL_Pos); // Set medium priority level
	SET_BIT(DMA2_Stream0->CR, DMA_SxCR_MINC); // incrementing memmory addres
	CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_PINC); // disabled incrementing perephiral addres
	DMA2_Stream0->CR |= (1 << DMA_SxCR_PSIZE_Pos); //periphiral data size 16b (half-word)
	DMA2_Stream0->CR |= (1 << DMA_SxCR_MSIZE_Pos); //memmory data size 16b (half-word)
	CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_CHSEL); // enable channel 0
	DMA2_Stream0->NDTR |= (50 << DMA_SxNDT_Pos);
	SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN); // Enable DMA
	
	//NVIC_EnableIRQ(ADC_IRQn);
}
  
void ADC1_2_Dual_Init(void)
{
	////////////////   Settings for ADC1
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); // GPIOA Clock
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_ADC1EN); // Clock for ADC1
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_ADC2EN); // Clock for ADC2
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_ADC3EN); // Clock for ADC3
	/////////Multi-mode
	ADC->CCR |= ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2; //Regular simultaneous mode only
	ADC->CCR |= ADC_CCR_DMA_1; //  DMA mode 2 enabled (2 / 3 half-words by pairs - 2&1 then 1&3 then 3&2)
	ADC->CCR |= (1 << ADC_CCR_DDS_Pos); // Enable DMA
	ADC->CCR &= ~(1 << ADC_CCR_DELAY_Pos); // delay 5 cycles
  //////// Settings ADC1
	CLEAR_BIT(ADC1->CR1, ADC_CR1_RES_0);
	CLEAR_BIT(ADC1->CR1, ADC_CR1_RES_1); // Resolution 12 bit
	SET_BIT(ADC1->CR2, ADC_CR2_ADON); // A/D Converter ON 
	SET_BIT(ADC->CCR, ADC_CCR_TSVREFE); // Choose temperatute sensor
	ADC1->SQR3 = 0x10; // 16 channel for first conversation
	CLEAR_BIT(ADC1->CR2, ADC_CR2_ALIGN); // Right alignment
	MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, 8 << ADC_CR2_EXTSEL_Pos); // Externel event Timer 3 TRGO
	MODIFY_REG(ADC1->CR2, ADC_CR2_EXTEN, 1 << ADC_CR2_EXTEN_Pos); //External TRG enable, Rising edge
	MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP16, 1 << ADC_SMPR1_SMP16_Pos); // Channel 16, 15 cycles for conversation	
	////////////////   Settings for PA0
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED0_Msk, 0x10 << GPIO_OSPEEDR_OSPEED1_Pos); // High speed
	GPIOA->MODER |= (3 << GPIO_MODER_MODE1_Pos); // Analog mode PA1
	CLEAR_REG(GPIOA->OTYPER);	
	/////////// Setting ADC2
	CLEAR_BIT(ADC2->CR1, ADC_CR1_RES_0);
	CLEAR_BIT(ADC2->CR1, ADC_CR1_RES_1); // Resolution 12 bit
	SET_BIT(ADC2->CR2, ADC_CR2_ADON); // A/D Converter ON 
	ADC2->SQR3 = 1; // 1 channel for first conversation
	ADC2->CR2 |=  ADC_CR2_EXTEN;
	ADC2->CR2 |=  ADC_CR2_EXTSEL;
	CLEAR_BIT(ADC2->CR2, ADC_CR2_ALIGN); // Right alignment
	MODIFY_REG(ADC2->SMPR2, ADC_SMPR2_SMP0, 1 << ADC_SMPR2_SMP0_Pos); // Channel 0, 15 cycles for conversation
  ////////////// Dual Start
	SET_BIT(ADC1->CR2, ADC_CR2_SWSTART); //  Starts conversion of regular channels
	// Init DMA
	DMA2_Stream0->PAR = (uint32_t)&ADC->CDR; // Adress of data
  DMA2_Stream0->M0AR =(uint32_t)&BUFF_ADC1_2[0]; // Adress of buffer
	SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TCIE); // Interrupt enable, complete transfer
	CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_DIR); // perifheral to memmory
	SET_BIT(DMA2_Stream0->CR, DMA_SxCR_CIRC); // circual mode enable
	DMA2_Stream0->CR &= ~(1 << DMA_SxCR_PL_Pos); // Set low priority level
	SET_BIT(DMA2_Stream0->CR, DMA_SxCR_MINC); // incrementing memmory addres
	CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_PINC); // disabled incrementing perephiral addres
	DMA2_Stream0->CR |= (2 << DMA_SxCR_PSIZE_Pos); //periphiral data size 16b (half-word)
	DMA2_Stream0->CR |= (2 << DMA_SxCR_MSIZE_Pos); //memmory data size 16b (half-word)
	CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_CHSEL); // enable channel 0
	DMA2_Stream0->NDTR |= (50 << DMA_SxNDT_Pos);
	SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN); // Enable DMA
}

void ADC3_Init(void) // for Temperature
{
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN); // GPIOA Clock
	SET_BIT(RCC->APB2ENR,RCC_APB2ENR_ADC3EN); // Clock for ADC3
	SET_BIT(ADC3->CR1, ADC_CR1_EOCIE); // Enable interrupt fo EOC
	CLEAR_BIT(ADC3->CR1, ADC_CR1_RES_0);
	CLEAR_BIT(ADC3->CR1, ADC_CR1_RES_1); // Resolution 12 bit
	SET_BIT(ADC3->CR2, ADC_CR2_ADON); // A/D Converter ON 
	//SET_BIT(ADC->CCR, ADC_CCR_TSVREFE); // Choose temperatute sensor
	ADC1->SQR3 = 1; // 1 channel for first conversation
	CLEAR_BIT(ADC3->CR2, ADC_CR2_ALIGN); // Right alignment
	MODIFY_REG(ADC3->CR2, ADC_CR2_EXTSEL, 6 << ADC_CR2_EXTSEL_Pos); // Externel event Timer 2 TRGO
	MODIFY_REG(ADC3->CR2, ADC_CR2_EXTEN, 1 << ADC_CR2_EXTEN_Pos); //External TRG enable, Rising edge
	SET_BIT(ADC3->CR2, ADC_CR2_DMA); // Enable DMA
	ADC3->CR2 |= ADC_CR2_DDS;  // For enable DMA+ADC(independet)
	MODIFY_REG(ADC3->SMPR1, ADC_SMPR2_SMP1, 1 << ADC_SMPR2_SMP1_Pos); // Channel 16, 15 cycles for conversation
	SET_BIT(ADC3->CR2, ADC_CR2_SWSTART); //  Starts conversion of regular channels
	// Init DMA
	DMA2_Stream1->PAR = (uint32_t)&ADC3->DR; // Adress of data
  DMA2_Stream1->M0AR =(uint32_t)&BUFF_ADC3[0]; // Adress of buffer
	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_TCIE); // Interrupt enable, complete transfer
	CLEAR_BIT(DMA2_Stream1->CR, DMA_SxCR_DIR); // perifheral to memmory
	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_CIRC); // circual mode disabled
	DMA2_Stream1->CR |= (1 << DMA_SxCR_PL_Pos); // Set medium priority level
	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_MINC); // incrementing memmory addres
	CLEAR_BIT(DMA2_Stream1->CR, DMA_SxCR_PINC); // disabled incrementing perephiral addres
	DMA2_Stream1->CR |= (1 << DMA_SxCR_PSIZE_Pos); //periphiral data size 16b (half-word)
	DMA2_Stream1->CR |= (1 << DMA_SxCR_MSIZE_Pos); //memmory data size 16b (half-word)
	MODIFY_REG(DMA2_Stream1->CR,DMA_SxCR_CHSEL , 2 << DMA_SxCR_CHSEL_Pos); // enable channel 2
	DMA2_Stream1->NDTR |= (BUFF_SIZE << DMA_SxNDT_Pos);
	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_EN); // Enable DMA
}

void DMA_Init(void)
{
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN); // Clock for DMA2 (for ADC) 
//	NVIC_SetPriority(1, DMA2_Stream0_IRQn);
//	NVIC_SetPriority(0, DMA2_Stream1_IRQn);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn); // Enable interrupt for ADC1_2 (Dual)
	NVIC_EnableIRQ(DMA2_Stream1_IRQn); // Enable interrupt for ADC3
}


void FSMC_Init(void)
{
  
}

	
