#include"stm32f767xx.h"

uint16_t status_word[14];
uint16_t generation_parametrs[27];
uint16_t adc_buffer[60];
uint16_t adc_value[6];
uint8_t analog_errors = 0;
uint16_t digital_errors = 0;
uint8_t flags = 0;
uint16_t timer_ready_state = 0;
uint32_t emitter_1_timer = 0;
uint32_t emitter_2_timer = 0;
uint8_t timer_calls = 0;

void init_all()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM3EN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN;

	SysTick_Config(1600000);

	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB | SYSCFG_EXTICR1_EXTI1_PB | SYSCFG_EXTICR1_EXTI2_PB;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB | SYSCFG_EXTICR2_EXTI5_PB;
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PB | SYSCFG_EXTICR3_EXTI9_PB | SYSCFG_EXTICR3_EXTI10_PB | SYSCFG_EXTICR3_EXTI11_PB;
	EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR2 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR8 | EXTI_IMR_MR9 | EXTI_IMR_MR10 | EXTI_IMR_MR11;
	EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1 | EXTI_RTSR_TR2 | EXTI_RTSR_TR4 | EXTI_RTSR_TR5 | EXTI_RTSR_TR8 | EXTI_RTSR_TR9 | EXTI_RTSR_TR10 | EXTI_RTSR_TR11;
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	__enable_irq();

	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	GPIOB->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
	GPIOC->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_0;
	GPIOC->AFR[0] |= GPIO_AFRL_AFRL7_1;
	GPIOC->AFR[1] |= GPIO_AFRH_AFRH0_1;
	TIM3->ARR = 200;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
	TIM3->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E;
	TIM3->CR1 |= TIM_CR1_CEN;

	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7;
	GPIOC->MODER |= GPIO_MODER_MODER1 | GPIO_MODER_MODER2;
	ADC1->SQR1 |= ADC_SQR1_L_1 | ADC_SQR1_L_2;
	ADC1->SQR3 |= ADC_SQR3_SQ2_0 | ADC_SQR3_SQ3_2 | ADC_SQR3_SQ3_1 | ADC_SQR3_SQ4_2 | ADC_SQR3_SQ4_1 | ADC_SQR3_SQ4_0 | ADC_SQR3_SQ5_3 | ADC_SQR3_SQ5_1 | ADC_SQR3_SQ5_0 | ADC_SQR3_SQ6_3 | ADC_SQR3_SQ6_2;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT | ADC_CR2_ADON;

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream0->CR  |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC;
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
//	DMA2_Stream0->M0AR = (uint32_t)&adc_buffer;
//	DMA2_Stream0->NDTR = 70;
	DMA2_Stream0->M0AR = (uint32_t)&adc_value;
	DMA2_Stream0->NDTR = 7;
   	DMA2_Stream0->CR |= DMA_SxCR_EN;

	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL2_2 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_0  | GPIO_AFRL_AFRL3_2 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_0;
	USART2->CR1 |= USART_CR1_OVER8;
	USART2->BRR = 0x08b;
	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;
	DAC->DHR12R1 = 0;
	DAC->DHR12R2 = 0;
	DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;

	//обработка бт-педали
}

void SysTick_Handler(void)
{
	++timer_calls;
	if(flags &= 0x1 != 0 && timer_calls > 4)
	{
		++timer_ready_state;
		if(flags &= 0x2 != 0)
		{
			ADC1->CR2 &= ~ADC_CR2_ADON;
			flags &= ~0x2;
		}
		else
		{
			ADC1->CR2 |= ADC_CR2_ADON;
			flags |= 0x2;
		}
		timer_calls = 0;
	}
	if(flags &= 0x4 != 0)
		++emitter_1_timer;
	if(flags &= 0x8 != 0)
		++emitter_1_timer;
}

void transmit_value(void *ad, int length)
{
	for(int i = length - 1; i >= 0; i--)
	{
		while((USART2->ISR & USART_ISR_TXE) == 0);
		USART2->TDR = *((char*)ad + i);
	}
}

void read_value(void *ad, int length)
{
	for(int i = length - 1; i >= 0; i--)
	{
		while((USART2->ISR & USART_ISR_RXNE) == 0);
		*((char*)ad + i) = USART2->RDR;
	}
}

void processing_adc_value()
{
	int max_adc_value[6], min_adc_value[6];
	for(int i = 0; i < 6; ++i)
		adc_value[i] = 0;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	for(int i = 1; i < 6; ++i)
		max_adc_value[i] = min_adc_value[i] = adc_buffer[i];
	for(int i = 1; i < 6; ++i)
		for(int j = 0; j < 10; ++j)
		{
			if(adc_buffer[i+j*6] > max_adc_value[i])
				max_adc_value[i] = adc_buffer[i+j*6];
			if(adc_buffer[i+j*6] < min_adc_value[i])
				min_adc_value[i] = adc_buffer[i+j*6];
		}
	if(max_adc_value != min_adc_value)
	{
		for(int i = 0; i < 6; ++i)
			for(int j = 0; j < 10; ++j)
				adc_value[i] += adc_buffer[i+j*6]/10;
	}
	else
	{
		for(int i = 0; i < 6; ++i)
			for(int j = 0; j < 10; ++j)
				adc_value[i] += adc_buffer[i+j*6]/8 - max_adc_value[i]/8 - min_adc_value[i]/8;
	}
	for(int i = 0; i < 6; ++i)
		adc_value[i] &= 0xfff8;
}

void analog_emergency_situations_check()
{
	ADC1->CR2 |= ADC_CR2_SWSTART;
	//processing_adc_value();
	if(adc_value[0] >= 2000)
		analog_errors |= 0x1;
//	if(adc_value[1] >= 2000)
//		analog_errors |= 0x2;
	if(adc_value[2] >= 2000)
		analog_errors |= 0x4;
/*	if(adc_value[3] >= 2000)
		analog_errors |= 0x8;
	if(adc_value[4] >= 1500)
	{
		GPIOA->BSRR |= GPIO_BSRR_BS_8;
		if(adc_value[4] >= 2000)
			analog_errors |= 0x10;
		if(adc_value[4] <= 1700)
			analog_errors &= ~0x10;
	}
*/	if(adc_value[5] >= 2000)
	{
		GPIOA->BSRR |= GPIO_BSRR_BS_8;
		if(adc_value[5] >= 3000)
			analog_errors |= 0x10;
		if(adc_value[5] <= 2500)
			analog_errors &= ~0x10;
	}
	if(/*adc_value[4] <= 1300 && */adc_value[5] <= 1300)
		GPIOA->BSRR |= GPIO_BSRR_BR_8;
}

void connection_check()
{
	while((USART2->ISR & USART_ISR_RXNE) == 0);
	while(USART2->RDR != 0xf0);
	while((USART2->ISR & USART_ISR_RXNE) == 0);
	while(USART2->RDR != 0xf1);
	while ((USART2->ISR & USART_ISR_TXE)==0);
	USART2->TDR = 0xe0;
	while ((USART2->ISR & USART_ISR_TXE)==0);
	USART2->TDR = 0xf1;
}

void getting_status_word()
{
	while ((USART2->ISR & USART_ISR_RXNE )==0);
	while(USART2->RDR != 0xf1);
	for(int i = 0; i < 14; ++i)
		read_value(&status_word[i],2);
}

void send_ll_state()
{
	analog_emergency_situations_check();
	if(analog_errors == 0 && digital_errors == 0)
	{
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xe1;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0x0e;
	}
	if(analog_errors != 0)
	{
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xe1;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0x1e;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = analog_errors;
		analog_errors = 0;
	}
	if(digital_errors != 0)
	{
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xe1;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0x2e;
		transmit_value(&digital_errors,2);
		digital_errors = 0;
	}
}

void ready_state()
{
	while(timer_ready_state <= 14400 && USART2->RDR != 0xfa && USART2->RDR != 0xfb)
	{
		analog_emergency_situations_check();
		if(analog_errors != 0)
		{
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0xe5;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0x1e;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = analog_errors;
		}
		if(digital_errors != 0)
		{
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0xe5;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0x2e;
			transmit_value(&digital_errors,2);
		}
		flags |= 0x4;
		GPIOB->BSRR |= GPIO_BSRR_BS_14;
	}
}

void transition_to_ready()
{
	while ((USART2->ISR & USART_ISR_RXNE ) == 0);
	while(USART2->RDR != 0xf3);
	for(int i = 0; i < 27; ++i)
		read_value(&generation_parametrs[i],2);
	analog_emergency_situations_check();
	if(analog_errors == 0 && digital_errors == 0)
	{
		//считывание радиометки
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xe4;
		//отправка кода метки
		begin:
		while ((USART2->ISR & USART_ISR_RXNE) == 0);
		GPIOB->BSRR |= GPIO_BSRR_BS_12;
		if(USART2->RDR == 0xf7)
		{
			//считывание радиометки
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0xe4;
			//отправка кода метки
			goto begin;
		}
		if(USART2->RDR == 0xf8)
			ready_state();
	}
	if(analog_errors != 0)
	{
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xe3;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0x1e;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = analog_errors;
	}
	if(digital_errors != 0)
	{
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xe3;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0x2e;
		transmit_value(&digital_errors,2);
	}
}

void input_generation_parameters_state()
{
	begin:
	TIM3->CCR2 = status_word[10];
	TIM3->CCR3 = status_word[11];
	while ((USART2->ISR & USART_ISR_RXNE) == 0)
	{
		analog_emergency_situations_check();
		if(analog_errors != 0)
		{
			while ((USART2->ISR & USART_ISR_TXE) == 0);
			USART2->TDR = 0xe2;
			while ((USART2->ISR & USART_ISR_TXE) == 0);
			USART2->TDR = 0x1e;
			while ((USART2->ISR & USART_ISR_TXE) == 0);
			USART2->TDR = analog_errors;
			analog_errors = 0;
		}
		if(digital_errors != 0)
		{
			while ((USART2->ISR & USART_ISR_TXE) == 0);
			USART2->TDR = 0xe2;
			while ((USART2->ISR & USART_ISR_TXE) == 0);
			USART2->TDR = 0x2e;
			transmit_value(&digital_errors,2);
			digital_errors = 0;
		}
		for(int i = 0; i <= 100000; ++i);
	}
	if(USART2->RDR == 0xf3)
	{
		for(int i = 0; i < 27; ++i)
			read_value(&generation_parametrs[i],2);
		goto begin;
	}
	if(USART2->RDR == 0xf4)
	{
		while ((USART2->ISR & USART_ISR_RXNE ) == 0);
		while(USART2->RDR != 0xf5);
		for(int i = 0; i < 14; ++i)
			read_value(&status_word[i],2);
		goto begin;
	}
	if(USART2->RDR == 0xf2)
	{
		transition_to_ready();
		goto begin;
	}
}

void chanel_1_generation()
{
	int voltage_1 = 0;
	if(generation_parametrs[25] < generation_parametrs[1] && generation_parametrs[25] > generation_parametrs[0])
		voltage_1 = generation_parametrs[4] + (generation_parametrs[25] - generation_parametrs[0]) * (generation_parametrs[5] - generation_parametrs[4]) / (generation_parametrs[1] - generation_parametrs[0]);
	if(generation_parametrs[25] < generation_parametrs[2] && generation_parametrs[25] > generation_parametrs[1])
		voltage_1 = generation_parametrs[5] + (generation_parametrs[25] - generation_parametrs[1]) * (generation_parametrs[6] - generation_parametrs[5]) / (generation_parametrs[2] - generation_parametrs[1]);
	if(generation_parametrs[25] < generation_parametrs[3] && generation_parametrs[25] > generation_parametrs[2])
		voltage_1 = generation_parametrs[6] + (generation_parametrs[25] - generation_parametrs[2]) * (generation_parametrs[7] - generation_parametrs[6]) / (generation_parametrs[3] - generation_parametrs[2]);
	flags |= 0x4;
	GPIOB->BSRR |= GPIO_BSRR_BS_14;
	GPIOB->BSRR |= GPIO_BSRR_BS_15;
	while(GPIOB->BSRR &= 0x8 == 0  && USART2->RDR != 0xfa)
	{
		DAC->DHR12R1 = voltage_1;
		GPIOB->BSRR |= GPIO_BSRR_BS_12;
		analog_emergency_situations_check();
		if(analog_errors != 0)
		{
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0xe6;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0x1e;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = analog_errors;
		}
		if(digital_errors != 0)
		{
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0xe6;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0x2e;
			transmit_value(&digital_errors,2);
		}
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xe7;
	}
	GPIOB->BSRR |= GPIO_BSRR_BR_12;
	DAC->DHR12R1 = 0;
	flags &= ~ 0x4;
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
	GPIOB->BSRR |= GPIO_BSRR_BR_15;
}

void chanel_2_generation()
{
	int voltage_2 = 0;
	if(generation_parametrs[26] < generation_parametrs[9] && generation_parametrs[26] > generation_parametrs[8])
		voltage_2 = generation_parametrs[12] + (generation_parametrs[25] - generation_parametrs[8]) * (generation_parametrs[13] - generation_parametrs[12]) / (generation_parametrs[9] - generation_parametrs[8]);
	if(generation_parametrs[26] < generation_parametrs[10] && generation_parametrs[26] > generation_parametrs[9])
		voltage_2 = generation_parametrs[13] + (generation_parametrs[25] - generation_parametrs[9]) * (generation_parametrs[14] - generation_parametrs[13]) / (generation_parametrs[10] - generation_parametrs[9]);
	if(generation_parametrs[26] < generation_parametrs[11] && generation_parametrs[26] > generation_parametrs[10])
		voltage_2 = generation_parametrs[14] + (generation_parametrs[25] - generation_parametrs[10]) * (generation_parametrs[15] - generation_parametrs[14]) / (generation_parametrs[11] - generation_parametrs[10]);
	flags |= 0x8;
	while(GPIOB->BSRR &= 0x10 == 0 && USART2->RDR != 0xfa)
	{
		DAC->DHR12R2 = voltage_2;
		GPIOB->BSRR |= GPIO_BSRR_BS_13;
		analog_emergency_situations_check();
		if(analog_errors != 0)
		{
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0xe6;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0x1e;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = analog_errors;
		}
		if(digital_errors != 0)
		{
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0xe6;
			while ((USART2->ISR & USART_ISR_TXE)==0);
			USART2->TDR = 0x2e;
			transmit_value(&digital_errors,2);
		}
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xe8;
	}
	GPIOB->BSRR |= GPIO_BSRR_BR_13;
	DAC->DHR12R2 = 0;
	flags &= ~ 0x8;
}

void EXTI0_IRQHandler()
{
	for(int i = 0; i <= 1000; ++i);
	if((flags &= 0x1) == 0)
		digital_errors |= 0x1;
	EXTI->PR |= EXTI_PR_PR0;
}

void EXTI1_IRQHandler()
{
	for(int i = 0; i <= 1000; ++i);
	if((flags &= 0x2) == 0)
		digital_errors |= 0x2;
	EXTI->PR |= EXTI_PR_PR1;
}

void EXTI2_IRQHandler()
{
	for(int i = 0; i <= 1000; ++i);
	digital_errors |= 0x4;
	EXTI->PR |= EXTI_PR_PR2;
}

void EXTI4_IRQHandler()
{
	for(int i = 0; i <= 1000; ++i);
	if((flags &= 0x4) == 0)
		digital_errors |= 0x8;
	else
		chanel_1_generation();
	EXTI->PR |= EXTI_PR_PR4;
}

void EXTI9_5_IRQHandler()
{
	for(int i = 0; i <= 1000; ++i);
	if (EXTI->PR & (1<<5))
	{
		if((flags &= 0x4) == 0)
			digital_errors |= 0x10;
		else
			chanel_1_generation();
		EXTI->PR |= EXTI_PR_PR5;
	}
	if (EXTI->PR & (1<<8))
	{
		digital_errors |= 0x20;
		EXTI->PR |= EXTI_PR_PR8;
	}
	if (EXTI->PR & (1<<9))
	{
		digital_errors |= 0x40;
		EXTI->PR |= EXTI_PR_PR9;
	}
}

void EXTI15_10_IRQHandler()
{
	for(int i = 0; i <= 1000; ++i);
	if (EXTI->PR & (1<<10))
	{
		digital_errors |= 0x80;
		EXTI->PR |= EXTI_PR_PR10;
	}
	if (EXTI->PR & (1<<11))
	{
		digital_errors |= 0x100;
		EXTI->PR |= EXTI_PR_PR11;
	}
}

int main(void)
{
	init_all();
  	connection_check();
	getting_status_word();
	send_ll_state();
	while(1)
	{
		input_generation_parameters_state();
	}
}

