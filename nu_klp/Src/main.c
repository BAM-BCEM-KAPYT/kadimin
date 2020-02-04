#include"stm32f767xx.h"

uint16_t adc_buffer[70];
uint16_t adc_value[7];
uint8_t analog_errors = 0;
uint16_t digital_errors = 0;
uint8_t flag_errors = 0;
uint16_t flags = 0;

void EXTI0_IRQHandler()
{
	digital_errors |= 0x1;
	EXTI->PR |= EXTI_PR_PR0;
}

void EXTI1_IRQHandler()
{
	digital_errors |= 0x2;
	EXTI->PR |= EXTI_PR_PR1;
}

void EXTI2_IRQHandler()
{
	digital_errors |= 0x4;
	EXTI->PR |= EXTI_PR_PR2;
}

void EXTI4_IRQHandler()
{
	if(flags &= 0x8000 != 0)
		digital_errors |= 0x8;
	EXTI->PR |= EXTI_PR_PR4;
}

void EXTI9_5_IRQHandler()
{
	EXTI->PR |= EXTI_PR_PR5;
}

void EXTI15_10_IRQHandler()
{
	EXTI->PR |= EXTI_PR_PR10;
}

void __init_all()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_DACEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN;

	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB | SYSCFG_EXTICR1_EXTI1_PB | SYSCFG_EXTICR1_EXTI2_PB;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB | SYSCFG_EXTICR2_EXTI5_PB;
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PB | SYSCFG_EXTICR3_EXTI9_PB | SYSCFG_EXTICR3_EXTI10_PB | SYSCFG_EXTICR3_EXTI11_PB;
	EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR2 | EXTI_IMR_MR4 | EXTI_IMR_MR5 | EXTI_IMR_MR8 | EXTI_IMR_MR9 | EXTI_IMR_MR10 | EXTI_IMR_MR11;
	EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1 | EXTI_RTSR_TR2 | EXTI_RTSR_TR4 | EXTI_RTSR_TR5 | EXTI_RTSR_TR8 | EXTI_RTSR_TR9 | EXTI_RTSR_TR10 | EXTI_RTSR_TR11;
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	__enable_irq();

	GPIOA->MODER |= GPIO_MODER_MODER8_0;
	GPIOB->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
	GPIOC->MODER |= GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;

	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7;
	GPIOC->MODER |= GPIO_MODER_MODER1 | GPIO_MODER_MODER2;
	ADC1->SQR1 |= ADC_SQR1_L_1 | ADC_SQR1_L_2;
	ADC1->SQR3 |= ADC_SQR3_SQ2_0 | ADC_SQR3_SQ3_2 | ADC_SQR3_SQ3_1 | ADC_SQR3_SQ4_2 | ADC_SQR3_SQ4_1 | ADC_SQR3_SQ4_0 | ADC_SQR3_SQ6_3 | ADC_SQR3_SQ6_1 | ADC_SQR3_SQ6_0;
	ADC1->SQR2 |= ADC_SQR2_SQ7_3 | ADC_SQR2_SQ7_2;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT | ADC_CR2_ADON;

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream0->CR  |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC;
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)&adc_buffer;
	DMA2_Stream0->NDTR = 70;
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

	GPIOA->BSRR |= GPIO_BSRR_BR_8;
	GPIOB->BSRR |= GPIO_BSRR_BR_12 | GPIO_BSRR_BR_13 | GPIO_BSRR_BR_14  | GPIO_BSRR_BR_15;
	GPIOC->BSRR |= GPIO_BSRR_BR_7  | GPIO_BSRR_BR_8 | GPIO_BSRR_BR_9;
}

void transmit_value(void *ad, int length)
{
	for(int i = length - 1; i >= 0; i--)
	{
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = *((char*)ad + i);
	}
}

void processing_adc_value()
{
	for(int i = 0; i < 7; ++i)
		adc_value[i] = 0;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	for(int i = 0; i <= 1000; i++);
	for(int i = 0; i < 70; ++i)
		adc_buffer[i] &= 0xfff8;
	int max_adc_value[7], min_adc_value[7];
	for(int i = 1; i < 7; ++i)
		max_adc_value[i] = min_adc_value[i] = adc_buffer[i];
	for(int i = 1; i < 7; ++i)
		for(int j = 0; j < 10; ++j)
		{
			if(adc_buffer[i+j*7] > max_adc_value[i])
				max_adc_value[i] = adc_buffer[i+j*7];
			if(adc_buffer[i+j*7] < min_adc_value[i])
				min_adc_value[i] = adc_buffer[i+j*7];
		}
	if(max_adc_value != min_adc_value)
	{
		for(int i = 0; i < 7; ++i)
			for(int j = 0; j < 10; ++j)
				adc_value[i] += adc_buffer[i+j*7]/10;
	}
	else
	{
		for(int i = 0; i < 7; ++i)
			for(int j = 0; j < 10; ++j)
				adc_value[i] += adc_buffer[i+j*7]/10;
	}
	for(int i = 0; i < 7; ++i)
		adc_value[i] &= 0xfff8;
}

void emergency_situations_check()
{
	processing_adc_value();
	if(adc_value[0] >= 2000)
		analog_errors |= 0x1;
	if(adc_value[2] >= 2000)
		analog_errors |= 0x4;
	if(flags &= 0x1 != 0)
		flag_errors |= 0x80;
	if(flags &= 0x2 != 0)
		flag_errors |= 0x100;
}

void connection_check()
{
	while(USART2->RDR != 0xfa);
	//while(USART2->RDR != 0x01);
	if(errors == 0)
	{
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xfb;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0x02;
	}
	else
	{
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xfb;
		while ((USART2->ISR & USART_ISR_TXE)==0);
		USART2->TDR = 0xff;
	//	transmit_value(&errors,2);
	}
}

void standby_state()
{
	flags |= 0x8000;
	while(1)
	{
		processing_adc_value();
		if(adc_value[0] >= 2000)
			analog_errors |= 0x1;
		if(adc_value[1] >= 2000)
			analog_errors |= 0x2;
		if(adc_value[2] >= 2000)
			analog_errors |= 0x4;
		if(adc_value[3] >= 2000)
			analog_errors |= 0x8;
		if(adc_value[4] >= 2000)
			analog_errors |= 0x10;
		if(adc_value[6] >= 2000)
			analog_errors |= 0x40;
	}

}

int main(void)
{
	__init_all();
	emergency_situations_check();
	connection_check();
	standby_state();
}
