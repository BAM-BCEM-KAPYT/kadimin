#include"stm32f407xx.h"
#include<math.h>

uint8_t bt_buffer[7];
uint8_t sbros_bt = 0;
uint8_t sbros_bt_2 = 0;
uint8_t adc_check_timer = 0;
uint8_t black_box_erase = 0;
uint8_t black_box_flags = 0;
uint8_t control_flags = 0;
uint8_t status_word[78];
uint8_t pilot_blinking = 0;
uint8_t duty_cycle_set_1 = 0;
uint8_t duty_cycle_set_2 = 0;
uint8_t rfid_buffer[33];
uint8_t crc_buffer = 0;
uint8_t indicator_blinking = 0;
uint8_t command_buffer[11];
uint8_t crc = 0;
uint8_t crc_summ = 0;
uint8_t black_box[5];
uint8_t emitter_1_timer_to_send = 0;
uint8_t emitter_2_timer_to_send = 0;
uint16_t bt_power = 0;
uint16_t betta = 3435;
uint16_t power_1_to_send = 0;
uint16_t power_2_to_send = 0;
uint16_t power_1 = 0;
uint16_t power_2 = 0;
uint16_t power_1_cor = 0;
uint16_t power_2_cor = 0;
uint16_t emitter_1_timer = 0;
uint16_t emitter_2_timer = 0;
uint16_t generation_parametrs[7];
uint16_t adc_buffer[60];
uint16_t adc_value[6];
uint16_t analog_errors = 0;
uint16_t digital_errors = 0;
uint16_t rfid_timer = 0;
uint16_t impulse_counter = 0;
uint32_t flags = 0;
uint32_t black_box_time[2];
double resistance = 0;
double temperature = 0;

void init_all()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN | RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_I2C2EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_ADC1EN;

	GPIOA->MODER |= GPIO_MODER_MODER2_0;
	GPIOB->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
	GPIOD->MODER |= GPIO_MODER_MODER7_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER13_0;

	SysTick_Config(1600000);

	GPIOA->MODER |= GPIO_MODER_MODER15_1;
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH7_0;
	TIM2->PSC = 400;
	TIM2->ARR = 200;
	TIM2->CCR1 = 0;
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->CR1 |= TIM_CR1_CEN;

	GPIOD->MODER |= GPIO_MODER_MODER12_1;
	GPIOD->AFR[1] |= GPIO_AFRH_AFRH4_1;
	TIM4->PSC = 400;
	TIM4->ARR = 200;
	TIM4->CCR1 = 0;
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CR1 |= TIM_CR1_CEN;

	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7;
	GPIOC->MODER |= GPIO_MODER_MODER1 | GPIO_MODER_MODER2;
	ADC1->SQR1 |= ADC_SQR1_L_1 | ADC_SQR1_L_2;
	ADC1->SQR3 |= ADC_SQR3_SQ2_0 | ADC_SQR3_SQ3_2 | ADC_SQR3_SQ3_1 | ADC_SQR3_SQ4_2 | ADC_SQR3_SQ4_1 | ADC_SQR3_SQ4_0 | ADC_SQR3_SQ5_3 | ADC_SQR3_SQ5_1 | ADC_SQR3_SQ5_0 | ADC_SQR3_SQ6_3 | ADC_SQR3_SQ6_2;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON;

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream0->CR  |= DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC;
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)&adc_buffer;
	DMA2_Stream0->NDTR = 60;
 	DMA2_Stream0->CR |= DMA_SxCR_EN;

	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL6_2 | GPIO_AFRL_AFRL6_1 | GPIO_AFRL_AFRL6_0  | GPIO_AFRL_AFRL7_2 | GPIO_AFRL_AFRL7_1 | GPIO_AFRL_AFRL7_0;
	USART1->CR1 |= USART_CR1_OVER8;
	USART1->BRR = 0x116;

	GPIOD->MODER |= GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1;
	GPIOD->AFR[0] |= GPIO_AFRL_AFRL5_2 | GPIO_AFRL_AFRL5_1 | GPIO_AFRL_AFRL5_0  | GPIO_AFRL_AFRL6_2 | GPIO_AFRL_AFRL6_1 | GPIO_AFRL_AFRL6_0;
	USART2->CR1 |= USART_CR1_OVER8;
	USART2->BRR = 0x22C;
	NVIC_EnableIRQ (USART2_IRQn);
	USART2->CR1  |= USART_CR1_RXNEIE; 
	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

	GPIOD->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOD->AFR[1] |= GPIO_AFRH_AFRH0_2 | GPIO_AFRH_AFRH0_1 | GPIO_AFRH_AFRH0_0 | GPIO_AFRH_AFRH1_2 | GPIO_AFRH_AFRH1_1 | GPIO_AFRH_AFRH1_0;
	USART3->CR1 |= USART_CR1_OVER8;
	USART3->BRR = 0x116;
	USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;
	DAC->DHR12R1 = 0;
	DAC->DHR12R2 = 0;
	DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;

	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11;
	GPIOB->AFR[1] |= (0x04<<2*4);
	GPIOB->AFR[1] |= (0x04<<3*4);
	I2C2->CR1 &= ~I2C_CR1_SMBUS;
	I2C2->CR2 &= ~I2C_CR2_FREQ;
	I2C2->CR2 |= 42;
	I2C2->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY);
	I2C2->CCR |= 208;
	I2C2->TRISE = 42;
	I2C2->CR1 |= I2C_CR1_PE;
}

void flash_write(uint8_t reg_addr1, uint8_t reg_addr2, uint8_t data)
{
  I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 & I2C_SR1_SB));
	(void) I2C2->SR1;
	I2C2->DR = 0xA0;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR1;
	(void) I2C2->SR2;
	I2C2->DR = reg_addr1;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	(void) I2C2->SR1;
	I2C2->DR = reg_addr2;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	(void) I2C2->SR1;
	I2C2->DR = data;
	while(!(I2C2->SR1 & I2C_SR1_BTF));
	I2C2->CR1 |= I2C_CR1_STOP;
}

uint8_t flash_read(uint8_t reg_addr1, uint8_t reg_addr2)
{
	uint8_t data;
	I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 & I2C_SR1_SB));
	(void) I2C2->SR1;
	I2C2->DR = 0xA0;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR1;
	(void) I2C2->SR2;
	I2C2->DR = reg_addr1;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	(void) I2C2->SR1;
	I2C2->DR = reg_addr2;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->CR1 |= I2C_CR1_STOP;
	I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 & I2C_SR1_SB));
	(void) I2C2->SR1;
	I2C2->DR = 0xA1;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	(void) I2C2->SR1;
	(void) I2C2->SR2;
	I2C2->CR1 &= ~I2C_CR1_ACK;
	while(!(I2C2->SR1 & I2C_SR1_RXNE));
	data = I2C2->DR;
	I2C2->CR1 |= I2C_CR1_STOP;
	return data;
}

void transmit_value(void *ad, int length, int usart)
{
	if(usart == 1)
	{
		for(int i = length - 1; i >= 0; --i)
		{
			while((USART1->SR & USART_SR_TXE) == 0);
			USART1->DR = *((char*)ad + i);
			crc_buffer += *((char*)ad + i);
		}
	}
	if(usart == 2)
	{
		for(int i = length - 1; i >= 0; --i)
		{
			while((USART2->SR & USART_SR_TXE) == 0);
			USART2->DR = *((char*)ad + i);
			crc_buffer += *((char*)ad + i);
		}
	}
	if(usart == 3)
	{
		for(int i = length - 1; i >= 0; --i)
		{
			while((USART3->SR & USART_SR_TXE) == 0);
			USART3->DR = *((char*)ad + i);
			crc_buffer += *((char*)ad + i);
		}
	}
}

void read_value(void *ad, int length)
{
	for(int i = length - 1; i >= 0; --i)
	{
		while((USART3->SR & USART_SR_RXNE) == 0);
		*((char*)ad + i) = USART3->DR;
		crc_summ += USART3->DR;
	}
}

void set_flag(int flag_number, int value)
{
	switch(flag_number)
	{
		case 1:
		{
			if(value != 0)
				status_word[10] |= 0x1;
			else
				status_word[10] &= ~0x1;
			break;
		}
		case 2:
		{
			if(value != 0)
				status_word[10] |= 0x2;
			else
				status_word[10] &= ~0x2;
			break;
		}
		case 3:
		{
			if(value != 0)
				status_word[10] |= 0x4;
			else
				status_word[10] &= ~0x4;
			break;
		}
		case 4:
		{
			if(value != 0)
				status_word[10] |= 0x8;
			else
				status_word[10] &= ~0x8;
			break;
		}
		case 5:
		{
			if(value != 0)
				status_word[10] |= 0x10;
			else
				status_word[10] &= ~0x10;
			break;
		}
		case 6:
		{
			if(value != 0)
				status_word[10] |= 0x20;
			else
				status_word[10] &= ~0x20;
			break;
		}
		case 7:
		{
			if(value != 0)
				status_word[10] |= 0x40;
			else
				status_word[10] &= ~0x40;
			break;
		}
		case 8:
		{
			if(value != 0)
				status_word[10] |= 0x80;
			else
				status_word[10] &= ~0x80;
			break;
		}
		case 9:
		{
			if(value != 0)
				status_word[11] |= 0x1;
			else
				status_word[11] &= ~0x1;
			break;
		}
		case 10:
		{
			if(value != 0)
				status_word[11] |= 0x2;
			else
				status_word[11] &= ~0x2;
			break;
		}
		case 11:
		{
			if(value != 0)
				status_word[11] |= 0x4;
			else
				status_word[11] &= ~0x4;
			break;
		}
	}
	flash_write(0, 10, status_word[10]);
	for(int i = 0; i <= 20000; ++i);
	flash_write(0, 11, status_word[11]);
	for(int i = 0; i <= 20000; ++i);
}

void clear_command_buffer()
{
	for(int i = 0; i < 11; ++i)
		command_buffer[i] = 0;
}

void USART2_IRQHandler(void)
{
	if((USART2->SR & USART_SR_RXNE) != 0)
	{
		for(int i = 0; i < 7; ++i)
		{
			while((USART2->SR & USART_SR_RXNE) == 0);
			bt_buffer[i] = USART2->DR;
		}
	}
	clear_command_buffer();
}
void read_command(int length)
{
	
	crc_summ = 0;
	for(int i = 0; i < length; ++i)
	{
		while((USART3->SR & USART_SR_RXNE) == 0);
		command_buffer[i] = USART3->DR;
		crc_summ +=command_buffer[i];
	}
	while((USART3->SR & USART_SR_RXNE) == 0);
	crc = USART3->DR;
	if(crc_summ != crc)
		flags |= 0x1;
	crc_summ = 0;
}
void SysTick_Handler(void)
{
	if((flags & 0x2000) != 0)
	{
		++indicator_blinking;
		if(indicator_blinking > 1 && (flags & 0x4000) == 0)
		{
			GPIOB->BSRR |= GPIO_BSRR_BS_13;
			control_flags |= 0x2;
			GPIOB->BSRR |= GPIO_BSRR_BR_14;
			control_flags &= ~0x4;
			flags |= 0x4000;
			indicator_blinking = 0;
		}
		if(indicator_blinking > 4 && (flags & 0x4000) != 0)
		{
			GPIOB->BSRR |= GPIO_BSRR_BR_13;
			control_flags &= ~0x2;
			GPIOB->BSRR |= GPIO_BSRR_BS_14;
			control_flags |= 0x4;
			flags &= ~0x4000;
			indicator_blinking = 0;
		}
	}
	if((flags & 0x80000) != 0)
	{
		++sbros_bt;
		GPIOD->BSRR |= GPIO_BSRR_BR_7;
		if(sbros_bt > 60)
		{
			sbros_bt = 0;
			flags &= ~0x80000;
			GPIOD->BSRR |= GPIO_BSRR_BS_7;
		}
	}
	if((flags & 0x100000) != 0)
	{
		++sbros_bt_2;
		GPIOD->BSRR |= GPIO_BSRR_BR_7;
		if(sbros_bt_2 > 10)
		{
			sbros_bt_2 = 0;
			flags &= ~0x100000;
			GPIOD->BSRR |= GPIO_BSRR_BS_7;
		}
	}
	if((flags & 0x8) != 0 || (flags & 0x10) != 0)
		++adc_check_timer;
	if((flags & 0x8) != 0)
	{
		if((flags & 0x8000) == 0)
		{
			if(adc_value[4] >= 10 && adc_value[4] <= 15)
				power_1_cor = (int)(generation_parametrs[5] * status_word[48] / 100);
			if(adc_value[4] > 15 && adc_value[4] <= 20)
				power_1_cor = (int)(generation_parametrs[5] * status_word[49] / 100);
			if(adc_value[4] > 20 && adc_value[4] <= 30)
				power_1_cor = (int)(generation_parametrs[5] * status_word[50] / 100);
			if(adc_value[4] >= 30 && adc_value[4] <= 40)
				power_1_cor = (int)(generation_parametrs[5] * status_word[51] / 100);
			
			if(power_1_cor * 10 >= (status_word[30] * 256 + status_word[31]) && power_1_cor * 10 <= (status_word[34] * 256 + status_word[35]))
				power_1 = (int)((((status_word[34] * 256 + status_word[35]) * (status_word[28] * 256 + status_word[29]) - (status_word[30] * 256 + status_word[31]) * (status_word[32] * 256 + status_word[33])) - ((status_word[28] * 256 + status_word[29]) - (status_word[32] * 256 + status_word[33])) * power_1_cor * 10) / ((status_word[34] * 256 + status_word[35]) - (status_word[30] * 256 + status_word[31])));
			if(power_1_cor * 10 > (status_word[34] * 256 + status_word[35]) && power_1_cor * 10 <= (status_word[38] * 256 + status_word[39]))
				power_1 = (int)((((status_word[38] * 256 + status_word[39]) * (status_word[32] * 256 + status_word[33]) - (status_word[34] * 256 + status_word[35]) * (status_word[36] * 256 + status_word[37])) - ((status_word[32] * 256 + status_word[33]) - (status_word[36] * 256 + status_word[37])) * power_1_cor * 10) / ((status_word[38] * 256 + status_word[39]) - (status_word[34] * 256 + status_word[35])));
			if(power_1_cor * 10 > (status_word[38] * 256 + status_word[39]) && power_1_cor * 10 <= (status_word[42] * 256 + status_word[43]))
				power_1 = (int)((((status_word[42] * 256 + status_word[43]) * (status_word[36] * 256 + status_word[37]) - (status_word[38] * 256 + status_word[39]) * (status_word[40] * 256 + status_word[41])) - ((status_word[36] * 256 + status_word[37]) - (status_word[40] * 256 + status_word[41])) * power_1_cor * 10) / ((status_word[42] * 256 + status_word[43]) - (status_word[38] * 256 + status_word[39])));
		
			power_1 = (int)(power_1 * 12.41);
			DAC->DHR12R1 = power_1;
		}
		++emitter_1_timer_to_send;
		++emitter_1_timer;
	}
	if((flags & 0x10) != 0)
	{
		if((flags & 0x8000) == 0)
		{
			if(adc_value[5] >= 10 && adc_value[5] <= 15)
				power_2_cor = (int)(generation_parametrs[4] * status_word[44] / 100);
			if(adc_value[5] > 15 && adc_value[5] <= 20)
				power_2_cor = (int)(generation_parametrs[4] * status_word[45] / 100);
			if(adc_value[5] > 20 && adc_value[5] <= 30)
				power_2_cor = (int)(generation_parametrs[4] * status_word[46] / 100);
			if(adc_value[5] > 30 && adc_value[5] <= 40)
				power_2_cor = (int)(generation_parametrs[4] * status_word[47] / 100);
			
			if(power_2_cor * 10 >= (status_word[14] * 256 + status_word[15]) && power_2_cor * 10 <= (status_word[18] * 256 + status_word[19]))
				power_2 = (int)((((status_word[18] * 256 + status_word[19]) * (status_word[12] * 256 + status_word[13]) - (status_word[14] * 256 + status_word[15]) * (status_word[16] * 256 + status_word[17])) - ((status_word[12] * 256 + status_word[13]) - (status_word[16] * 256 + status_word[17])) * power_2_cor * 10) / ((status_word[18] * 256 + status_word[19]) - (status_word[14] * 256 + status_word[15])));
			if(power_2_cor * 10 > (status_word[18] * 256 + status_word[19]) && power_2_cor * 10 <= (status_word[22] * 256 + status_word[23]))
				power_2 = (int)((((status_word[22] * 256 + status_word[23]) * (status_word[16] * 256 + status_word[17]) - (status_word[18] * 256 + status_word[19]) * (status_word[20] * 256 + status_word[21])) - ((status_word[16] * 256 + status_word[17]) - (status_word[20] * 256 + status_word[21])) * power_2_cor * 10) / ((status_word[22] * 256 + status_word[23]) - (status_word[18] * 256 + status_word[19])));
			if(power_2_cor * 10 > (status_word[22] * 256 + status_word[23]) && power_2_cor * 10 <= (status_word[26] * 256 + status_word[27]))
				power_2 = (int)((((status_word[26] * 256 + status_word[27]) * (status_word[20] * 256 + status_word[21]) - (status_word[22] * 256 + status_word[23]) * (status_word[24] * 256 + status_word[25])) - ((status_word[20] * 256 + status_word[21]) - (status_word[24] * 256 + status_word[25])) * power_2_cor * 10) / ((status_word[26] * 256 + status_word[27]) - (status_word[22] * 256 + status_word[23])));
			
			power_2 = (int)(power_2 * 12.41);
			DAC->DHR12R2 = power_2;
		}
		++emitter_2_timer_to_send;
		++emitter_2_timer;
	}
	if((flags & 0x40) != 0)
	{
		++pilot_blinking;
		if(pilot_blinking >= 5 && (flags & 0x80) == 0)
		{
			TIM2->CCR1 = duty_cycle_set_1;
			TIM4->CCR1 = duty_cycle_set_2;
			flags |= 0x80;
			pilot_blinking = 0;
		}
		if(pilot_blinking >= 5 && (flags & 0x80) != 0)
		{
			TIM2->CCR1 = 0;
			TIM4->CCR1 = 0;
			flags &= ~0x80;
			pilot_blinking = 0;
		}
	}
	if((flags & 0x8) != 0 && (generation_parametrs[6] & 0x1) != 0)
	{
		if((flags & 0x200) != 0 && impulse_counter >= generation_parametrs[3])
		{
			GPIOB->BSRR |= GPIO_BSRR_BR_15;
			flags &= ~0x200;
			impulse_counter = 0;
		}
		if((flags & 0x200) == 0 && impulse_counter >= generation_parametrs[1])
		{
			GPIOB->BSRR |= GPIO_BSRR_BS_15;
			flags |= 0x200;
			impulse_counter = 0;
		}		
		++impulse_counter;
	}
  if((flags & 0x10) != 0 && (generation_parametrs[6] & 0x100) != 0)
	{
		if((flags & 0x200) != 0 && impulse_counter >= generation_parametrs[2])
		{
			GPIOD->BSRR |= GPIO_BSRR_BR_10;
			flags &= ~0x200;
			impulse_counter = 0;
		}
		if((flags & 0x200) == 0 && impulse_counter >= generation_parametrs[0])
		{
			GPIOD->BSRR |= GPIO_BSRR_BS_10;
			flags |= 0x200;
			impulse_counter = 0;
		}	
		++impulse_counter;
	}
}

void set_duty_cycle(int duty_cycle)
{
	switch(duty_cycle)
	{
		case 0:
		{
			TIM2->CCR1 = 0;
			TIM4->CCR1 = 0;
			duty_cycle_set_1 = 0;
			duty_cycle_set_2 = 0;
			break;
		}
		case 25:
		{
			TIM2->CCR1 = status_word[67] * 2;
			TIM4->CCR1 = status_word[66] * 2;
			duty_cycle_set_1 = status_word[67] * 2;
			duty_cycle_set_2 = status_word[66] * 2;
			
			break;
		}
		case 50:
		{
			TIM2->CCR1 = status_word[69] * 2;
			TIM4->CCR1 = status_word[68] * 2;
			duty_cycle_set_1 = status_word[69] * 2;
			duty_cycle_set_2 = status_word[68] * 2;
			break;
		}
		case 75:
		{
			TIM2->CCR1 = status_word[71] * 2;
			TIM4->CCR1 = status_word[70] * 2;
			duty_cycle_set_1 = status_word[71] * 2;
			duty_cycle_set_2 = status_word[70] * 2;
			break;
		}
		case 100:
		{
			TIM2->CCR1 = status_word[73] * 2;
			TIM4->CCR1 = status_word[72] * 2;
			duty_cycle_set_1 = status_word[73] * 2;
			duty_cycle_set_2 = status_word[72] * 2;
			break;
		}
	}
}

void processing_adc_value()
{
	for(int i = 0; i < 10; ++i)
	{
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
		DMA2_Stream0->M0AR = (uint32_t)&adc_buffer[i * 6];
	  DMA2_Stream0->CR |= DMA_SxCR_EN;
		ADC1->CR2 |= ADC_CR2_SWSTART;
		for(int j = 0; j < 1000; ++j);
	}
	for(int i = 0; i < 6; ++i)
		adc_value[i] = 0;
	for(int i = 0; i < 6; ++i)
		for(int j = 0; j < 10; ++j)
			adc_value[i] += adc_buffer[i + j * 6];
	for(int i = 0; i < 6; ++i)
		adc_value[i] = (int)(adc_value[i] / 10);

	for(int i = 0; i < 4; ++i)
		adc_value[i] = (int)(adc_value[i] / 7.33);

	if((adc_value[0] / 10) >= status_word[1] && (flags & 0x8) == 0 && (flags & 0x10) == 0)
		analog_errors |= 0x1;
	if((adc_value[1] / 10) >= status_word[0] && (flags & 0x8) == 0 && (flags & 0x10) == 0)
		analog_errors |= 0x2;
	if((adc_value[2] / 10) >= status_word[1] && (flags & 0x8) == 0 && (flags & 0x10) == 0)
		analog_errors |= 0x1000;
	if((adc_value[3] / 10) >= status_word[0] && (flags & 0x8) == 0 && (flags & 0x10) == 0)
		analog_errors |= 0x2000;

	if((adc_value[0] / 10) >= status_word[3] && ((flags & 0x8) != 0 || (flags & 0x10) != 0))
		analog_errors |= 0x1;
	if((adc_value[1] / 10) >= status_word[2] && ((flags & 0x8) != 0 || (flags & 0x10) != 0))
		analog_errors |= 0x2;
	if((adc_value[2] / 10) >= status_word[3] && ((flags & 0x8) != 0 || (flags & 0x10) != 0))
		analog_errors |= 0x1000;
	if((adc_value[3] / 10) >= status_word[2] && ((flags & 0x8) != 0 || (flags & 0x10) != 0))
		analog_errors |= 0x2000;

	adc_value[1] = (int)((adc_value[1] * (status_word[58] * 256 + status_word[59]) + (status_word[56] * 256 + status_word[57]) * 100) / 1000);
	adc_value[0] = (int)((adc_value[0] * (status_word[62] * 256 + status_word[63]) + (status_word[60] * 256 + status_word[61]) * 100) / 1000);
	adc_value[3] = (int)((adc_value[3] * (status_word[52] * 256 + status_word[53])) / 1000);
	adc_value[2] = (int)((adc_value[2] * (status_word[54] * 256 + status_word[55])) / 1000);

	adc_value[4] = (int)(adc_value[4] * 0.825);
	resistance = 10000 * adc_value[4] / (4096 - adc_value[4]);
	temperature = (int)((298.15 * betta) / (betta * 1.0 + 298.15 * log(resistance / 10000.0)) - 273.15);
	if(temperature < 0)
		temperature = 0;
	adc_value[4] = (int)(temperature);

	adc_value[5] = (int)(adc_value[5] * 0.825);
	resistance = 10000 * adc_value[5] / (4096 - adc_value[5]);
	temperature = (int)((298.15 * betta) / (betta * 1.0 + 298.15 * log(resistance / 10000.0)) - 273.15);
	if(temperature < 0)
		temperature = 0;
	adc_value[5] = (int)(temperature);
}

void analog_emergency_situations_check()
{
	processing_adc_value();

	if(adc_value[4] < 10)
		analog_errors |= 0x400;
	if(adc_value[4] >= 10)
		analog_errors &= ~0x400;
	if(adc_value[5] < 10)
		analog_errors |= 0x800;
	if(adc_value[5] >= 10)
		analog_errors &= ~0x800;

	if((adc_value[4] >= status_word[4 || adc_value[4] >= status_word[6]] || adc_value[4] >= status_word[5]) && (flags & 0x2) == 0)
	{
		GPIOD->BSRR |= GPIO_BSRR_BS_13;
		control_flags |= 0x8;
		if(adc_value[4] >= status_word[6])
		{
			flags |= 0x10000;
			if((black_box_flags & 0x10) == 0)
			{
				++black_box[0];
				flash_write(0, 78, black_box[0]);
				for(int k = 0; k <= 20000; ++k);
				black_box_flags |= 0x10;
			}
		}
		if(adc_value[4] >= status_word[5])
		{
			flags |= 0x10000;
			analog_errors |= 0x100;
		}
	}
	if(adc_value[4] <= status_word[7])
	{
		analog_errors &= ~0x140;
		flags &= ~0x10000;
	}
	if((adc_value[5] >= status_word[4] || adc_value[5] >= status_word[6] || adc_value[5] >= status_word[5]) && (flags & 0x2) == 0)
	{
		GPIOD->BSRR |= GPIO_BSRR_BS_13;
		control_flags |= 0x8;
		if(adc_value[5] >= status_word[6])
		{
			flags |= 0x20000;
			if((black_box_flags & 0x10) == 0)
			{
				++black_box[0];
				flash_write(0, 78, black_box[0]);
				for(int k = 0; k <= 20000; ++k);
				black_box_flags |= 0x10;
			}
		}
		if(adc_value[5] >= status_word[5])
		{
			analog_errors |= 0x200;
			flags |= 0x20000;
		}
	}
	if(adc_value[5] <= status_word[7])
	{
		analog_errors &= ~0x280;
		flags &= ~0x20000;
	}
	if(adc_value[4] <= (status_word[4] - 2) && adc_value[5] <= status_word[4] - 2)
	{
		GPIOD->BSRR |= GPIO_BSRR_BR_13;
		control_flags &= ~0x8;
	}
}

void errors_check()
{
	analog_emergency_situations_check();
	
	if((status_word[10] & 0x4) != 0)
		analog_errors |= 0x10;
	if((status_word[10] & 0x8) != 0)
		analog_errors |= 0x20;
	if((flags & 0x8) == 0 && (flags & 0x10) == 0)
	{
		if((flags & 0x20000) != 0)
			analog_errors |= 0x80;
		if((flags & 0x10000) != 0)
			analog_errors |= 0x40;
	}
	
	if((flags & 0x1) != 0)
	{
//		digital_errors |= 0x800;
		flags &= ~0x1;
	}

	if((GPIOD->IDR & 0x4000) == 0 && (flags & 0x8) == 0 && (flags & 0x10) == 0 && (flags & 0x40) == 0)
		digital_errors |= 0x8;
	if((GPIOD->IDR & 0x8000) == 0 && (flags & 0x8) == 0 && (flags & 0x10) == 0 && (flags & 0x40) == 0)
		digital_errors |= 0x10;
	if((GPIOC->IDR & 0x40) != 0 && (status_word[10] & 0x20) == 0)
		digital_errors |= 0x2;
	if((GPIOC->IDR & 0x80) 	!= 0 && (status_word[10] & 0x10) == 0)
		digital_errors |= 0x1;
	if((GPIOC->IDR & 0x100) == 0)
	{
		digital_errors |= 0x40;
		if((black_box_flags & 0x1) == 0)
		{
			++black_box[2];
			flash_write(0, 80, black_box[2]);
			for(int k = 0; k <= 20000; ++k);
			black_box_flags |= 0x1;
		}
	}
	if((GPIOC->IDR & 0x200) == 0)
	{
		digital_errors |= 0x100;
		if((black_box_flags & 0x2) == 0)
		{
			++black_box[4];
			flash_write(0, 82, black_box[4]);
			for(int k = 0; k <= 20000; ++k);
			black_box_flags |= 0x2;
		}
	}
	if((GPIOA->IDR & 0x400) == 0)
	{
		digital_errors |= 0x20;
		if((black_box_flags & 0x4) == 0)
		{
			++black_box[1];
			flash_write(0, 79,black_box[1]);
			for(int k = 0; k <= 20000; ++k);
			black_box_flags |= 0x4;
		}
	}
	if((GPIOA->IDR & 0x800) == 0)
	{
		digital_errors |= 0x80;
		if((black_box_flags & 0x8) == 0)
		{
			++black_box[3];
			flash_write(0, 81,black_box[3]);
			for(int k = 0; k <= 20000; ++k);
			black_box_flags |= 0x8;
		}
	}
	if((GPIOA->IDR & 0x1000) == 0)
		digital_errors |= 0x4;
}

void error_send()
{
	errors_check();
	if(analog_errors == 0 && digital_errors == 0)
	{
		flags &= ~0x2000;
		GPIOB->BSRR |= GPIO_BSRR_BR_14;
		if((flags & 0x40) != 0)
			GPIOB->BSRR |= GPIO_BSRR_BS_13;
	}
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x05;
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0xf1;
	crc_buffer = 0xf6 + adc_value[4] + adc_value[5];
	transmit_value(&analog_errors, 2, 3);
	transmit_value(&digital_errors,2, 3);
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = adc_value[4];
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = adc_value[5];
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = crc_buffer;
	crc_buffer = 0;
	digital_errors = 0;
	analog_errors = 0;
	if((status_word[10] & 0x1) != 0)
		analog_errors |= 0x4;
	if((status_word[10] & 0x2) != 0)
		analog_errors |= 0x8;
	flags &= ~0x200000;
}

void connection_check()
{
	errors_check();
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x05;
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0xF0;
	if(digital_errors == 0 && analog_errors == 0)
	{
		while ((USART3->SR & USART_SR_TXE)==0);
		USART3->DR = 0x0A;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0xFF;
	}
	if(digital_errors != 0 || analog_errors != 0)
	{
		while ((USART3->SR & USART_SR_TXE)==0);
		USART3->DR = 0x0C;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x01;
		analog_errors = 0;
		digital_errors = 0;
	}
	clear_command_buffer();
}

void connection_check_in_ready()
{
	if((flags & 0x200000) == 0 && analog_errors != 0)
	{
		status_word[10] &= ~0x1;
		status_word[10] &= ~0x2;
		set_flag(1, 0);
		set_flag(2, 0);
		analog_errors &= ~0x4;
		analog_errors &= ~0x8;
	}
	if((flags & 0x8000) == 0)
	{
		errors_check();		
		if(adc_check_timer > 20)
				adc_check_timer = 21;
		if((status_word[11] & 0x1) == 0 && ((flags & 0x8) != 0 || (flags & 0x10) != 0) && adc_check_timer >= 10 && adc_check_timer <= 20)
		{
			if((generation_parametrs[4] * 100) / status_word[64] > 40 && (flags & 0x10) != 0)
			{
				if((status_word[10] & 0x80) == 0)
				{
					if((adc_value[3] * 100) / generation_parametrs[4] >= 120 || (adc_value[3] * 100) / generation_parametrs[4] <= 80)
					{
						analog_errors |= 0x20;
						status_word[10] |= 0x8;
						set_flag(4, 1);
						status_word[10] |= 0x2;
						set_flag(2, 1);
					}		
					if((adc_value[3] * 100) / generation_parametrs[4] >= 115 || (adc_value[3] * 100) / generation_parametrs[4] <= 85)
					{
						status_word[10] |= 0x2;
						set_flag(2, 1);
						flags |= 0x200000;
					}
				}
				else
				{
					if((adc_value[1] * 100) / generation_parametrs[4] >= 120 || (adc_value[1] * 100) / generation_parametrs[4] <= 80)
					{
						analog_errors |= 0x20;
						status_word[10] |= 0x8;
						set_flag(4, 1);
						status_word[10] |= 0x2;
						set_flag(2, 1);
					}		
					if((adc_value[1] * 100) / generation_parametrs[4] >= 115 || (adc_value[1] * 100) / generation_parametrs[4] <= 85)
					{
						status_word[10] |= 0x2;
						set_flag(2, 1);
						flags |= 0x200000;
					}
				}
			}
			if((generation_parametrs[5] * 100) / status_word[65] > 40 && (flags & 0x8) != 0)
			{
				if((status_word[10] & 0x80) == 0)
				{
					if((adc_value[2] * 100) / generation_parametrs[5] >= 120 || (adc_value[2] * 100) / generation_parametrs[5] <= 80)
					{
						analog_errors |= 0x10;
						status_word[10] |= 0x4;
						set_flag(3, 1);
						status_word[10] |= 0x1;
						set_flag(1, 1);
					}
					if((adc_value[2] * 100) / generation_parametrs[5] >= 115 || (adc_value[2] * 100) / generation_parametrs[5] <= 85)
					{
						status_word[10] |= 0x1;
						set_flag(1, 1);
						flags |= 0x200000;
					}
				}
				else
				{
					if((adc_value[0] * 100) / generation_parametrs[5] >= 120 || (adc_value[0] * 100) / generation_parametrs[5] <= 80)
					{
						analog_errors |= 0x10;
						status_word[10] |= 0x4;
						set_flag(3, 1);
						status_word[10] |= 0x1;
						set_flag(1, 1);
					}
					if((adc_value[0] * 100) / generation_parametrs[5] >= 115 || (adc_value[0] * 100) / generation_parametrs[5] <= 85)
					{
						status_word[10] |= 0x1;
						set_flag(1, 1);
						flags |= 0x200000;
					}
				}
			}
		}
	}
	
	if((status_word[10] & 0x4) != 0)
		analog_errors |= 0x10;
	if((status_word[10] & 0x8) != 0)
		analog_errors |= 0x20;		
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x05;
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0xf8;
	crc_buffer = 0x05 + 0xf8;
	if((flags & 0x8) == 0 && (flags & 0x10) == 0 && analog_errors == 0 && digital_errors == 0)
	{
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x00;
	}
	if((flags & 0x8) != 0 && analog_errors == 0 && digital_errors == 0)
	{
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x0A;
		crc_buffer += 0x0A;
	}
	if((flags & 0x10) != 0 && analog_errors == 0 && digital_errors == 0)
	{
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x0B;
		crc_buffer += 0x0B;
	}
	if(analog_errors != 0 || digital_errors != 0)
	{
		while ((USART3->SR & USART_SR_TXE) == 0);
		crc_buffer += 0x0C;
		USART3->DR = 0x0C;
	}
	if(((flags & 0x8) != 0 || (flags & 0x10) != 0) && analog_errors == 0 && digital_errors == 0)
	{
		if((flags & 0x8) != 0 && analog_errors == 0 && digital_errors == 0)
		{
			while ((USART3->SR & USART_SR_TXE) == 0);
			USART3->DR = 0;
			transmit_value(&emitter_1_timer, 1, 3);
			if((generation_parametrs[6] & 0x1) == 0)
			{
				if((status_word[10] & 0x80) == 0)
					transmit_value(&adc_value[2], 2, 3);
				if((status_word[10] & 0x80) != 0)
					transmit_value(&adc_value[0], 2, 3);
			}
			else
			{
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0;
			}
		}
		if((flags & 0x10) != 0 && analog_errors == 0 && digital_errors == 0)
		{
			while ((USART3->SR & USART_SR_TXE) == 0);
			USART3->DR = 0;
			transmit_value(&emitter_2_timer, 1, 3);
			if((generation_parametrs[6] & 0x100) == 0)
			{
				if((status_word[10] & 0x80) == 0)
					transmit_value(&adc_value[3], 2, 3);
				if((status_word[10] & 0x80) != 0)
					transmit_value(&adc_value[1], 2, 3);
			}
			else
			{
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0;
			}
		}
	}
	else
		for(int i = 0; i < 4; ++i)
		{
			while ((USART3->SR & USART_SR_TXE) == 0);
			USART3->DR = 0;
		}

	for(int i = 0; i < 10; ++i)
	{
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
		DMA2_Stream0->M0AR = (uint32_t)&adc_buffer[i * 6];
		DMA2_Stream0->CR |= DMA_SxCR_EN;
		ADC1->CR2 |= ADC_CR2_SWSTART;
		for(int j = 0; j < 1000; ++j);
	}
	for(int i = 0; i < 6; ++i)
		adc_value[i] = 0;
	for(int i = 0; i < 6; ++i)
		for(int j = 0; j < 10; ++j)
			adc_value[i] += adc_buffer[i+j*6];
	for(int i = 0; i < 6; ++i)
		adc_value[i] = (int)(adc_value[i] / 10);

	for(int i = 0; i < 4; ++i)
		adc_value[i] = (int)(adc_value[i] / 7.33);

	adc_value[4] = (int)(adc_value[4] * 0.825);
	resistance = 10000 * adc_value[4] / (4096 - adc_value[4]);
	temperature = (int)((298.15 * betta) / (betta * 1.0 + 298.15 * log(resistance / 10000.0)) - 273.15);
	if(temperature < 0)
		temperature = 0;
	adc_value[4] = (int)(temperature);

	adc_value[5] = (int)(adc_value[5] * 0.825);
	resistance = 10000 * adc_value[5] / (4096 - adc_value[5]);
	temperature = (int)((298.15 * betta) / (betta * 1.0 + 298.15 * log(resistance / 10000.0)) - 273.15);
	if(temperature < 0)
		temperature = 0;
	adc_value[5] = (int)(temperature);

	for(int i = 0; i < 6; ++i)
		transmit_value(&adc_value[i], 2, 3);

	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0;
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0;
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0;
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0;
	
	while ((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = crc_buffer;
	clear_command_buffer();
	crc_buffer = 0;
	emitter_1_timer_to_send = 0;
	emitter_2_timer_to_send = 0;
}

void chanel_1_generation()
{
	flags &= ~0x40;
	TIM2->CCR1 = duty_cycle_set_1;
	TIM4->CCR1 = duty_cycle_set_2;	
	if((flags & 0x8000) != 0)
	{
		power_1 = (int)(generation_parametrs[5] * 124.1);
		DAC->DHR12R1 = power_1;
	}
	
	GPIOB->BSRR |= GPIO_BSRR_BS_13;
	GPIOB->BSRR |= GPIO_BSRR_BS_14;
	flags |= 0x8;
	GPIOB->BSRR |= GPIO_BSRR_BS_15;

	while((GPIOD->IDR & 0x4000) == 0 && (flags & 0x800) == 0)
	{
		if(emitter_1_timer >= 600 && (status_word[10] & 0x40) != 0)
			flags |= 0x800;
		if((USART3->SR & USART_SR_RXNE) != 0)
		{
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[0] = USART3->DR;
			crc_summ += command_buffer[0];
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[1] = USART3->DR;
			crc_summ += command_buffer[1];
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf8)
			{
				connection_check_in_ready();
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf1)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				error_send();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe7)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 1)
				{
					GPIOB->BSRR |= GPIO_BSRR_BS_14 | GPIO_BSRR_BR_13;
					DAC->DHR12R1 = 0;
					DAC->DHR12R2 = 0;
					GPIOB->BSRR |= GPIO_BSRR_BR_15;
					GPIOD->BSRR |= GPIO_BSRR_BR_10;
					flags |= 0x800;
				}
				if(command_buffer[2] == 2)
				{
					flags |= 0x2000;
					DAC->DHR12R1 = 0;
					DAC->DHR12R2 = 0;
					GPIOB->BSRR |= GPIO_BSRR_BR_15;
					GPIOD->BSRR |= GPIO_BSRR_BR_10;
					flags |= 0x800;
				}
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe7;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xec;
			}
			clear_command_buffer();
		}
	}
	power_1 = 0;
	DAC->DHR12R1 = 0;
	GPIOB->BSRR |= GPIO_BSRR_BR_15;
	flags &= ~ 0x8;
	flags &= ~ 0x80;
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
	if((status_word[10] & 0x1) != 0)
		analog_errors |= 0x4;
	if((flags & 0x10000) != 0)
		analog_errors |= 0x40;

	black_box_time[0] += emitter_1_timer;
	for(int k = 0; k <= 20000; ++k);
	flash_write(0, 83, *(((char*)&black_box_time[0]) + 0));
	for(int k = 0; k <= 20000; ++k);
	flash_write(0, 84, *(((char*)&black_box_time[0]) + 1));
	for(int k = 0; k <= 20000; ++k);
	flash_write(0, 85, *(((char*)&black_box_time[0]) + 2));
	for(int k = 0; k <= 20000; ++k);
	flash_write(0, 86, *(((char*)&black_box_time[0]) + 3));
	emitter_1_timer = 0;
	
	flags |= 0x40;
	
	adc_check_timer = 0;
}

void chanel_2_generation()
{
	flags &= ~0x40;
	TIM2->CCR1 = duty_cycle_set_1;
	TIM4->CCR1 = duty_cycle_set_2;
	if((flags & 0x8000) != 0)
	{
		power_2 = (int)(generation_parametrs[4] * 124.1);
		DAC->DHR12R2 = power_2;
	}
	
	GPIOB->BSRR |= GPIO_BSRR_BS_13;
	GPIOB->BSRR |= GPIO_BSRR_BS_14;
	flags |= 0x10;
	GPIOD->BSRR |= GPIO_BSRR_BS_10;

	while((GPIOD->IDR & 0x8000) == 0 && (flags & 0x800) == 0)
	{
		if(emitter_1_timer >= 600 && (status_word[10] & 0x40) != 0)
			flags |= 0x800;
		if((USART3->SR & USART_SR_RXNE) != 0)
		{
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[0] = USART3->DR;
			crc_summ += command_buffer[0];
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[1] = USART3->DR;
			crc_summ += command_buffer[1];
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf8)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				connection_check_in_ready();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf1)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				error_send();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe7)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 1)
				{
					GPIOB->BSRR |= GPIO_BSRR_BS_14 | GPIO_BSRR_BR_13;
					DAC->DHR12R1 = 0;
					DAC->DHR12R2 = 0;
					GPIOB->BSRR |= GPIO_BSRR_BR_15;
					GPIOD->BSRR |= GPIO_BSRR_BR_10;
					flags |= 0x800;
				}
				if(command_buffer[2] == 2)
				{
					flags |= 0x2000;
					DAC->DHR12R1 = 0;
					DAC->DHR12R2 = 0;
					GPIOB->BSRR |= GPIO_BSRR_BR_15;
					GPIOD->BSRR |= GPIO_BSRR_BR_10;
					flags |= 0x800;
				}
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe7;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xec;
			}
			clear_command_buffer();
		}
	}

	power_2 = 0;
	DAC->DHR12R2 = 0;
	GPIOD->BSRR |= GPIO_BSRR_BR_10;
	flags &= ~ 0x10;
	flags &= ~ 0x80;
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
	if((status_word[10] & 0x2) != 0)
		analog_errors |= 0x8;
	if((flags & 0x20000) != 0)
		analog_errors |= 0x80;

	black_box_time[1] += emitter_2_timer;
	for(int k = 0; k <= 20000; ++k);
	flash_write(0, 87, *(((char*)&black_box_time[1]) + 0));
	for(int k = 0; k <= 20000; ++k);
	flash_write(0, 88, *(((char*)&black_box_time[1]) + 1));
	for(int k = 0; k <= 20000; ++k);
	flash_write(0, 89, *(((char*)&black_box_time[1]) + 2));
	for(int k = 0; k <= 20000; ++k);
	flash_write(0, 90, *(((char*)&black_box_time[1]) + 3));
	emitter_2_timer = 0;
	
	flags |= 0x40;
	
	adc_check_timer = 0;
}

void ready_state()
{
	if((flags & 0x8000) == 0)
	{
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x05;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0xf6;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0xfb;
	}
	GPIOB->BSRR |= GPIO_BSRR_BS_13;
	flags |= 0x40;
	flags &= ~0x800;
	flags &= ~0x20;
	flags &= ~0x2000;
	flags &= ~0x40000;
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
	analog_errors = 0;
	digital_errors = 0;
	while((flags & 0x20) == 0 && (flags & 0x100) == 0)
	{
		if(((GPIOD->IDR & 0x4000) == 0 || (GPIOD->IDR & 0x8000) == 0) && (flags & 0x200000) == 0 && analog_errors != 0)
		{
			status_word[10] &= ~0x1;
			status_word[10] &= ~0x2;
			set_flag(1, 0);
			set_flag(2, 0);
			analog_errors &= ~0x4;
			analog_errors &= ~0x8;
			flags |= 0x200000;
		}
		if((GPIOD->IDR & 0x8000) != 0 && (GPIOD->IDR & 0x4000) != 0)
			flags &= ~0x800;
		if((USART3->SR & USART_SR_RXNE) != 0)
		{
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[0] = USART3->DR;
			crc_summ +=command_buffer[0];
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[1] = USART3->DR;
			crc_summ += command_buffer[1];
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf1)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				error_send();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe7)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 1)
				{
					GPIOB->BSRR |= GPIO_BSRR_BS_14 | GPIO_BSRR_BR_13;
				}
				if(command_buffer[2] == 2)
				{
					flags |= 0x2000;
				}
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe7;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xec;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf4)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				set_duty_cycle(command_buffer[2]);
				clear_command_buffer();
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf4;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf9;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf7)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				flags |= 0x20;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf7;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xfc;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf8)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				connection_check_in_ready();
				if((GPIOD->IDR & 0x4000) == 0 && (flags & 0x800) == 0)
					chanel_1_generation();
				if((GPIOD->IDR & 0x8000) == 0 && (flags & 0x800) == 0)
					chanel_2_generation();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf6)
			{
				for(int i = 0; i < 7; ++i)
				{
					read_value(&generation_parametrs[i], 2);
				}
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf6;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xfb;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xee)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 0x02)
					flags |= 0x100;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xee;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x02;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf5;
			}
			crc_buffer = 0;
			clear_command_buffer();
		}
	}
	flags &= ~0x4;
	flags &= ~0x20;
	flags &= ~0x40;
	flags &= ~0x80;
	flags &= ~0x1000;
	flags &= ~0x8000;
	GPIOB->BSRR |= GPIO_BSRR_BR_13;
	TIM2->CCR1 = duty_cycle_set_1;
	TIM4->CCR1 = duty_cycle_set_2;
}

void service_menu_control()
{
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x05;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0xfa;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x01;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x00;
	while((flags & 0x100) == 0)
	{
		if((USART3->SR & USART_SR_RXNE) != 0)
		{
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[0] = USART3->DR;
			crc_summ += command_buffer[0];
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[1] = USART3->DR;
			crc_summ += command_buffer[1];
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xfb)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				while((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xfb;
				crc_buffer = 0;
				for(int i = 0; i < 10; ++i)
				{
					DMA2_Stream0->CR &= ~DMA_SxCR_EN;
					DMA2_Stream0->M0AR = (uint32_t)&adc_buffer[i * 6];
					DMA2_Stream0->CR |= DMA_SxCR_EN;
					ADC1->CR2 |= ADC_CR2_SWSTART;
					for(int j = 0; j < 1000; ++j);
				}
				for(int i = 0; i < 6; ++i)
					adc_value[i] = 0;
				for(int i = 0; i < 6; ++i)
					for(int j = 0; j < 10; ++j)
						adc_value[i] += adc_buffer[i+j*6];
				for(int i = 0; i < 6; ++i)
					adc_value[i] = (int)(adc_value[i] / 10);

				adc_value[4] = (int)(adc_value[4] * 0.825);
				resistance = 10000 * adc_value[4] / (4096 - adc_value[4]);
				temperature = (int)((298.15 * betta) / (betta * 1.0 + 298.15 * log(resistance / 10000.0)) - 273.15);
				if(temperature < 0)
					temperature = 0;
				adc_value[4] = (int)(temperature);

				adc_value[5] = (int)(adc_value[5] * 0.825);
				resistance = 10000 * adc_value[5] / (4096 - adc_value[5]);
				temperature = (int)((298.15 * betta) / (betta * 1.0 + 298.15 * log(resistance / 10000.0)) - 273.15);
				if(temperature < 0)
					temperature = 0;
				adc_value[5] = (int)(temperature);

				for(int i = 0; i < 4; ++i)
					adc_value[i] = (int)(adc_value[i] / 7.33);

				for(int i = 0; i < 6; ++i)
					transmit_value(&adc_value[i], 2, 3);

				if((GPIOD->IDR & 0x4000) == 0 && (flags & 0x8) == 0 && (flags & 0x10) == 0)
					digital_errors |= 0x8;
				if((GPIOD->IDR & 0x8000) == 0 && (flags & 0x8) == 0 && (flags & 0x10) == 0)
					digital_errors |= 0x10;
				if((GPIOC->IDR & 0x40) != 0)
					digital_errors |= 0x2;
				if((GPIOC->IDR & 0x80) != 0)
					digital_errors |= 0x1;
				if((GPIOC->IDR & 0x100) == 0)
					digital_errors |= 0x40;
				if((GPIOC->IDR & 0x200) == 0)
					digital_errors |= 0x100;
				if((GPIOA->IDR & 0x400) == 0)
					digital_errors |= 0x20;
				if((GPIOA->IDR & 0x800) == 0)
					digital_errors |= 0x80;
				if((GPIOA->IDR & 0x1000) == 0)
					digital_errors |= 0x4;
				if((GPIOD->IDR & 0x4000) == 0)
					digital_errors |= 0x8;
				if((GPIOD->IDR & 0x8000) == 0)
					digital_errors |= 0x10;

				if((control_flags & 0x1) != 0)
					digital_errors |= 0x200;
				if((control_flags & 0x2) != 0)
					digital_errors |= 0x400;
				if((control_flags & 0x4) != 0)
					digital_errors |= 0x800;
				if((control_flags & 0x8) != 0)
					digital_errors |= 0x1000;
				if((control_flags & 0x10) != 0)
					digital_errors |= 0x2000;

				transmit_value(&digital_errors, 2, 3);
				digital_errors = 0;

				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0;
				
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = crc_buffer;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xee)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 0x01)
					flags |= 0x100;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xee;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x01;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf4;
			}
			clear_command_buffer();
			crc_buffer = 0;
		}
	}
	flags &= ~0x100;
	clear_command_buffer();
}

void service_menu_generation()
{
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x05;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0xfa;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x02;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x01;
	flags |= 0x8000;
	while((flags & 0x100) == 0)
		ready_state();
	flags &= ~0x100;
	flags &= ~0x8000;
}

void send_start_rfid_message()
{
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x30;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x31;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x30;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x39;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x30;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x30;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x30;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x33;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x30;
	while((USART1->SR & USART_SR_TXE) == 0);
	USART1->DR = 0x34;
}

void service_menu()
{
	flags &= ~0x2000;
	GPIOB->BSRR |= GPIO_BSRR_BR_13;
	control_flags &= ~0x2;
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
	control_flags &= ~0x4;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x05;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0xfa;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0x00;
	while((USART3->SR & USART_SR_TXE) == 0);
	USART3->DR = 0xff;
	while((flags & 0x100) == 0)
	{
		if((USART3->SR & USART_SR_RXNE) != 0)
		{
			clear_command_buffer();
			crc_buffer = 0;
			crc_summ = 0;
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[0] = USART3->DR;
			crc_summ += command_buffer[0];
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[1] = USART3->DR;
			crc_summ += command_buffer[1];
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf5)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				send_start_rfid_message();
				while((USART1->SR & USART_SR_TXE) == 0);
				USART1->DR = 0x41;
				while((USART1->SR & USART_SR_TXE) == 0);
				USART1->DR = 0x30;
				while((USART1->SR & USART_SR_TXE) == 0);
				USART1->DR = 0x30;
				while((USART1->SR & USART_SR_TXE) == 0);
				USART1->DR = 0x31;
				for(int i = 0; i < 4; ++i)
				{
					while((USART1->SR & USART_SR_TXE) == 0);
					USART1->DR = 0x30;
				}
				for(int i = 0; i < 16; ++i)
				{
					while((USART1->SR & USART_SR_RXNE) == 0);
					rfid_buffer[i] = USART1->DR;
				}
				if(rfid_buffer[14] == 0x0d && rfid_buffer[15] == 0x0a)
					goto end_rfid;
				
				for(int i = 16; i < 33; ++i)
				{
					while((USART1->SR & USART_SR_RXNE) == 0);
					rfid_buffer[i] = USART1->DR;
				}
				clear_command_buffer();
				read_command(2);
				if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf5)
				{
					while ((USART3->SR & USART_SR_TXE) == 0);
					USART3->DR = 0x05;
					while ((USART3->SR & USART_SR_TXE) == 0);
					USART3->DR = 0xf5;
					for(int i = 15; i < 25; ++i)
					{
						while((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = rfid_buffer[i];
						crc_buffer += rfid_buffer[i];
					}
					crc_buffer += 0xfa;
					while ((USART3->SR & USART_SR_TXE) == 0);
					USART3->DR = crc_buffer;
					crc_buffer = 0;
				}
				for(int i = 0; i < 33; ++i)
					rfid_buffer[i] = 0;
				
				end_rfid:
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf3)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[3] = USART3->DR;
				crc_summ += command_buffer[3];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				betta = 256 * command_buffer[2] + command_buffer[3];
				clear_command_buffer();
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf3;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf8;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xfa)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				GPIOB->BSRR |= GPIO_BSRR_BR_13;
				if(command_buffer[2] == 0x01)
					service_menu_control();
				if(command_buffer[2] == 0x02)
					service_menu_generation();
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf4)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				set_duty_cycle(command_buffer[2]);
				clear_command_buffer();
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf4;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf9;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xfc)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				flags &= ~0x2000;
				GPIOB->BSRR |= GPIO_BSRR_BR_13;
				control_flags &= ~0x2;
				GPIOB->BSRR |= GPIO_BSRR_BR_14;
				control_flags &= ~0x4;
				if(command_buffer[2] == 0x1)
				{
					GPIOB->BSRR |= GPIO_BSRR_BR_13;
					control_flags &= ~0x2;
					GPIOB->BSRR |= GPIO_BSRR_BR_14;
					control_flags &= ~0x4;
				}
				if(command_buffer[2] == 0x4)
				{
					GPIOB->BSRR |= GPIO_BSRR_BS_13;
					control_flags |= 0x2;
					GPIOB->BSRR |= GPIO_BSRR_BR_14;
					control_flags &= ~0x4;
				}
				if(command_buffer[2] == 0x3)
				{
					GPIOB->BSRR |= GPIO_BSRR_BS_13;
					control_flags |= 0x2;
					GPIOB->BSRR |= GPIO_BSRR_BS_14;
					control_flags |= 0x4;
				}
				if(command_buffer[2] == 0x2)
				{
					GPIOB->BSRR |= GPIO_BSRR_BR_13;
					control_flags &= ~0x2;
					GPIOB->BSRR |= GPIO_BSRR_BS_14;
					control_flags |= 0x4;
				}
				clear_command_buffer();
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xfc;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x01;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xfd)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 0)
				{
					GPIOD->BSRR |= GPIO_BSRR_BR_13;
					control_flags &= ~0x8;
				}
				if(command_buffer[2] == 0x1)
				{
					GPIOD->BSRR |= GPIO_BSRR_BS_13;
					control_flags |= 0x8;
				}
				clear_command_buffer();
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xfd;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x02;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xfe)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[3] = USART3->DR;
				crc_summ += command_buffer[3];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				set_flag(command_buffer[2], command_buffer[3]);
				for(int i = 0; i <= 20000; ++i);
				status_word[10] = flash_read(0, 10);
				for(int i = 0; i <= 20000; ++i);
				status_word[11] = flash_read(0, 11);
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xfe;
				transmit_value(&status_word[10], 1, 3);
				transmit_value(&status_word[11], 1, 3);
				crc_buffer += 0x03;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = crc_buffer;
				crc_buffer = 0;
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xff)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				for(int i = 0; i <= 20000; ++i);
				status_word[10] = flash_read(0, 10);
				for(int i = 0; i <= 20000; ++i);
				status_word[11] = flash_read(0, 11);
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xff;		
				crc_buffer = 0x04;
				transmit_value(&status_word[10], 1, 3);
				transmit_value(&status_word[11], 1, 3);
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = crc_buffer;
				crc_buffer = 0;
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe0)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				for(int i = 0; i <= 20000; ++i);
				status_word[10] = flash_read(0, 10);
				for(int i = 0; i <= 20000; ++i);
				status_word[11] = flash_read(0, 11);
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe0;
				for(int i = 0; i < 78; ++i)
					transmit_value(&status_word[i], 1, 3);
				crc_buffer += 0xe5;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = crc_buffer;
				crc_buffer = 0;
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe1)
			{
				for(int i = 0; i < 78; ++i)
					read_value(&status_word[i], 1);
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;			
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe1;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe6;
				for(int i = 0; i <= 20000; ++i);
				status_word[10] = flash_read(0, 10);
				for(int i = 0; i <= 20000; ++i);
				status_word[11] = flash_read(0, 11);
				for(int i = 0; i < 78; ++i)
				{
					for(int k = 0; k <= 20000; ++k);
					flash_write(0, i, status_word[i]);
				}
				clear_command_buffer();

			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe2)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe2;
				transmit_value(&black_box, 5, 3);
				transmit_value(&black_box_time[0], 4, 3);
				transmit_value(&black_box_time[1], 4, 3);
				crc_buffer += 0xe7;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = crc_buffer;
				crc_buffer = 0;
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe3)
			{
				read_value(&black_box, 5);
				read_value(&black_box_time[0], 4);
				read_value(&black_box_time[1], 4);
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				for(int i = 78; i < 83; ++i)
				{
					for(int k = 0; k <= 20000; ++k);
					flash_write(0, i, *((char*)&black_box[i - 74]));
				}
				for(int i = 0; i < 2; ++i)
				{
					for(int k = 0; k <= 20000; ++k);
					flash_write(0, i * 4 + 83, *((char*)&black_box_time[i] + 0));
					for(int k = 0; k <= 20000; ++k);
					flash_write(0, i * 4 + 84, *((char*)&black_box_time[i] + 1));
					for(int k = 0; k <= 20000; ++k);
					flash_write(0, i * 4 + 85, *((char*)&black_box_time[i] + 2));
					for(int k = 0; k <= 20000; ++k);
					flash_write(0, i * 4 + 86, *((char*)&black_box_time[i] + 3));
				}
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe3;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe8;
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe4)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				flags |= 0x100000;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe4;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = bt_buffer[0];														// nalich bt
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = bt_buffer[5];														//bt power
				crc_buffer = crc_buffer + bt_buffer[0] + bt_buffer[5] + 0xe9;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = crc_buffer;
				crc_buffer = 0;
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe5)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				clear_command_buffer();
				flags |= 0x80000;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe5;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xea;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe6)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 0)
					GPIOB->BSRR |= GPIO_BSRR_BR_12;
				if(command_buffer[2] == 1)
					GPIOB->BSRR |= GPIO_BSRR_BS_12;
				clear_command_buffer();
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe6;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xeb;	
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xee)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[1] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 0x00)
					flags |= 0x100;
				clear_command_buffer();
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xee;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x00;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf3;
			}
		}
	}
	flags &= ~0x100;
	if(status_word[9] == 0x1 && (flags & 0x400) != 0)
	{
		flags &= ~0x400;
		bt_buffer[0] = 1;
		bt_buffer[4] = 0x1;
		bt_buffer[5] = 0x2c;
		for(int i = 0; i < 10; ++i)
		{
			if((GPIOB->IDR & 0x10) != 0)
			{
				bt_buffer[0] = 0;
				bt_buffer[4] = 0;
				bt_buffer[5] = 0;
			}
			for(int j = 0; j < 100000; ++j);
			for(int j = 0; j < 100000; ++j);
		}
		begin_bt:
		while((flags & 0x400) == 0)
		{
			if((USART3->SR & USART_SR_RXNE) != 0)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[0] = USART3->DR;
				crc_summ +=command_buffer[0];
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[1] = USART3->DR;
				crc_summ += command_buffer[1];
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				
				if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe4)
				{
					if(command_buffer[2] != 0x1)
					{
						if(bt_buffer[0] != 0)
						{
							bt_buffer[0] = 1;
							*((char*)&bt_power + 0) = bt_buffer[5];
							*((char*)&bt_power + 1) = bt_buffer[4];
							bt_power /= 3;
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = 0x05;
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = 0xe4;
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = bt_buffer[0];													// nalich bt		
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = bt_power;															//bt power
							crc_buffer = bt_buffer[0] + bt_power + 0xe9;
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = crc_buffer;	
							if(bt_buffer[0] == 0)
								goto begin_bt;
							else
								flags |= 0x400;
						}
					}
					else
					{
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0x05;
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0xe4;
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0;													
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0;
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0xe9;
						flags |= 0x400;
					}
					clear_command_buffer();
					crc_buffer = 0;
				}
			}
		}
	}
}

void input_generation_parameters_state()
{
	while(1)
	{
		flags |= 0x2;
		if((USART3->SR & USART_SR_RXNE) != 0)
		{
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[0] = USART3->DR;
			crc_summ +=command_buffer[0];
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[1] = USART3->DR;
			crc_summ += command_buffer[1];
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf8)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf1)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				error_send();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe7)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 1)
				{
					GPIOB->BSRR |= GPIO_BSRR_BS_14 | GPIO_BSRR_BR_13;
				}
				if(command_buffer[2] == 2)
				{
					flags |= 0x2000;
				}
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe7;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xec;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf0)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				connection_check();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf4)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				set_duty_cycle(command_buffer[2]);
				clear_command_buffer();
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf4;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xf9;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf5)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				send_start_rfid_message();
				while((USART1->SR & USART_SR_TXE) == 0);
				USART1->DR = 0x41;
				while((USART1->SR & USART_SR_TXE) == 0);
				USART1->DR = 0x30;
				while((USART1->SR & USART_SR_TXE) == 0);
				USART1->DR = 0x30;
				while((USART1->SR & USART_SR_TXE) == 0);
				USART1->DR = 0x31;
				for(int i = 0; i < 4; ++i)
				{
					while((USART1->SR & USART_SR_TXE) == 0);
					USART1->DR = 0x30;
				}
				for(int i = 0; i < 16; ++i)
				{
					while((USART1->SR & USART_SR_RXNE) == 0);
					rfid_buffer[i] = USART1->DR;
				}
				if(rfid_buffer[14] == 0x0d && rfid_buffer[15] == 0x0a)
					goto end_rfid;
				
				for(int i = 16; i < 33; ++i)
				{
					while((USART1->SR & USART_SR_RXNE) == 0);
					rfid_buffer[i] = USART1->DR;
				}
				clear_command_buffer();
				read_command(2);
				if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf5)
				{
					while ((USART3->SR & USART_SR_TXE) == 0);
					USART3->DR = 0x05;
					while ((USART3->SR & USART_SR_TXE) == 0);
					USART3->DR = 0xf5;
					for(int i = 15; i < 25; ++i)
					{
						while((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = rfid_buffer[i];
						crc_buffer += rfid_buffer[i];
					}
					crc_buffer += 0xfa;
					while ((USART3->SR & USART_SR_TXE) == 0);
					USART3->DR = crc_buffer;
					crc_buffer = 0;
				}
				for(int i = 0; i < 33; ++i)
					rfid_buffer[i] = 0;
				
				end_rfid:
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf6)
			{
				for(int i = 0; i < 7; ++i)
				{
					read_value(&generation_parametrs[i], 2);
				}
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				clear_command_buffer();
				flags &= ~0x2;
				ready_state();
				clear_command_buffer();
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf7)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xfa)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				flags &= ~0x20;
				if(command_buffer[2] == 0x00)
					service_menu();
				clear_command_buffer();
			}
			clear_command_buffer();
		}
	}
}
int main(void)
{
	for(int i = 0; i <= 1000000; ++i);
	init_all();
	GPIOD->BSRR |= GPIO_BSRR_BS_7;
	for(int i = 0; i <= 1000000; ++i);

	//flash
	for(int i = 0; i <= 100000; ++i);
	for(int i = 0; i < 74; ++i)
	{
		for(int k = 0; k <= 20000; ++k);
		status_word[i] = flash_read(0, i);
	}
	
/*
	black_box_erase = flash_read(0, 91);
	if(black_box_erase == 0xff)
	{
		for(int i = 78; i < 83; ++i)
		{
			for(int k = 0; k <= 20000; ++k);
			flash_write(0, i, *((char*)&black_box[i - 74]));
		}
		for(int i = 0; i < 2; ++i)
		{
			for(int k = 0; k <= 20000; ++k);
			flash_write(0, i * 4 + 83, *((char*)&black_box_time[i] + 0));
			for(int k = 0; k <= 20000; ++k);
			flash_write(0, i * 4 + 84, *((char*)&black_box_time[i] + 1));
			for(int k = 0; k <= 20000; ++k);
			flash_write(0, i * 4 + 85, *((char*)&black_box_time[i] + 2));
			for(int k = 0; k <= 20000; ++k);
			flash_write(0, i * 4 + 86, *((char*)&black_box_time[i] + 3));
		}
	}
	else
	{
		black_box_erase = 0;
		flash_write(0, 91, *((char*)&black_box_erase));
	}
*/

	for(int i = 78; i < 83; ++i)
	{
		for(int k = 0; k <= 20000; ++k);
		*((char*)&black_box[i - 78]) = flash_read(0, i);
	}
	for(int i = 0; i < 2; ++i)
	{
			for(int k = 0; k <= 20000; ++k);
			*((char*)&black_box_time[i] + 0) = flash_read(0, i * 4 + 83);
			for(int k = 0; k <= 20000; ++k);
			*((char*)&black_box_time[i] + 1) = flash_read(0, i * 4 + 84);
			for(int k = 0; k <= 20000; ++k);
			*((char*)&black_box_time[i] + 2) = flash_read(0, i * 4 + 85);
			for(int k = 0; k <= 20000; ++k);
			*((char*)&black_box_time[i] + 3) = flash_read(0, i * 4 + 86);
	}

	if((GPIOC->IDR & 0x40) != 0 && (status_word[10] & 0x20) == 0)
		digital_errors |= 0x2;
	if((GPIOC->IDR & 0x80) != 0 && (status_word[10] & 0x10) == 0)
		digital_errors |= 0x1;
	if((GPIOC->IDR & 0x100) == 0)
		digital_errors |= 0x40;
	if((GPIOC->IDR & 0x200) == 0)
		digital_errors |= 0x100;
	if((GPIOA->IDR & 0x400) == 0)
		digital_errors |= 0x20;
	if((GPIOA->IDR & 0x800) == 0)
		digital_errors |= 0x80;
	if((GPIOA->IDR & 0x1000) == 0)
		digital_errors |= 0x4;
	if((GPIOD->IDR & 0x4000) == 0)
		digital_errors |= 0x8;
	if((GPIOD->IDR & 0x8000) == 0)
		digital_errors |= 0x10;
	
	if((status_word[11] & 0x4) == 0)		
	{
		GPIOB->BSRR |= GPIO_BSRR_BS_13;
		for(int i = 0; i < 100000; ++i);
		GPIOB->BSRR |= GPIO_BSRR_BR_13;
		// 1 rfid message
		for(int i = 0; i <= 100000; ++i);
		USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
		send_start_rfid_message();
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x41;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x46;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x46;
		for(int i = 0; i < 4; ++i)
		{
			while((USART1->SR & USART_SR_TXE) == 0);
			USART1->DR = 0x30;
		}
		for(int i = 0; i <= 10000; ++i);

		// 2 rfid message
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x31;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x43;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x33;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x34;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x31;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x32;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x31;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x31;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x39;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x30;
		for(int i = 0; i <= 10000; ++i);

		// 3 rfid message
		send_start_rfid_message();
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x46;
		for(int i = 0; i < 7; ++i)
		{
			while((USART1->SR & USART_SR_TXE) == 0);
			USART1->DR = 0x30;
		}
		for(int i = 0; i <= 10000; ++i);

		// 4 rfid message
		send_start_rfid_message();
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x46;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x31;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x46;
		while((USART1->SR & USART_SR_TXE) == 0);
		USART1->DR = 0x46;
		for(int i = 0; i < 4; ++i)
		{
			while((USART1->SR & USART_SR_TXE) == 0);
			USART1->DR = 0x30;
		}
		for(int i = 0; i < 1000000; ++i);
		rfid_buffer[0] = USART1->DR;
		rfid_buffer[0] = 0;
	}

	GPIOB->BSRR |= GPIO_BSRR_BS_13;
	for(int i = 0; i < 100000; ++i);
	GPIOB->BSRR |= GPIO_BSRR_BR_13;

	flags = 0;
	read_command(2);
	if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf0)
	{
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x05;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0xf0;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x0a;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0xff;
	}
	clear_command_buffer();
	
 	read_command(2);
	if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe0)
	{
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x05;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0xe0;
		for(int i = 0; i < 78; ++i)
		{
			transmit_value(&status_word[i], 1, 3);
		}
		crc_buffer += 0xe5;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = crc_buffer;
		crc_buffer = 0;
		clear_command_buffer();
	}
	clear_command_buffer();
		
	if((status_word[10] & 0x4) != 0)
		analog_errors |= 0x10;
	if((status_word[10] & 0x8) != 0)
		analog_errors |= 0x20;	
	
	read_command(2);
	if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf0)
	{
		errors_check();
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0x05;
		while ((USART3->SR & USART_SR_TXE) == 0);
		USART3->DR = 0xf0;
		if(analog_errors == 0 && digital_errors == 0)
		{
			while ((USART3->SR & USART_SR_TXE)==0);
			USART3->DR = 0x0A;
			while ((USART3->SR & USART_SR_TXE) == 0);
			USART3->DR = 0xFF;
		}
		if(analog_errors != 0 || digital_errors != 0)
		{
			while ((USART3->SR & USART_SR_TXE) == 0);
			USART3->DR = 0x0c;
			while ((USART3->SR & USART_SR_TXE) == 0);
			USART3->DR = 0x01;
			clear_command_buffer();
			read_command(2);
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xf1)
			{
				error_send();
				clear_command_buffer();
			}
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[0] = USART3->DR;
			crc_summ +=command_buffer[0];
			while((USART3->SR & USART_SR_RXNE) == 0);
			command_buffer[1] = USART3->DR;
			crc_summ += command_buffer[1];
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe7)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				if(command_buffer[2] == 1)
				{
					GPIOB->BSRR |= GPIO_BSRR_BS_14 | GPIO_BSRR_BR_13;
				}
				if(command_buffer[2] == 2)
				{
					flags |= 0x2400;
				}
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0x05;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xe7;
				while ((USART3->SR & USART_SR_TXE) == 0);
				USART3->DR = 0xec;
			}
			if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe4)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				goto begin_bt_2;
			}
		}
		clear_command_buffer();
	}

	if(status_word[9] == 0x1 && (flags & 0x400) == 0)
	{
		bt_buffer[0] = 1;
		bt_buffer[4] = 0x1;
		bt_buffer[5] = 0x2c;
		for(int i = 0; i < 10; ++i)
		{
			if((GPIOB->IDR & 0x10) != 0)
			{
				bt_buffer[0] = 0;
				bt_buffer[4] = 0;
				bt_buffer[5] = 0;
			}
			for(int j = 0; j < 100000; ++j);
			for(int j = 0; j < 100000; ++j);
		}
		begin_bt:
		while((flags & 0x400) == 0)
		{
			if((USART3->SR & USART_SR_RXNE) != 0)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[0] = USART3->DR;
				crc_summ +=command_buffer[0];
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[1] = USART3->DR;
				crc_summ += command_buffer[1];
				while((USART3->SR & USART_SR_RXNE) == 0);
				command_buffer[2] = USART3->DR;
				crc_summ += command_buffer[2];
				while((USART3->SR & USART_SR_RXNE) == 0);
				crc = USART3->DR;
				if(crc_summ != crc)
					flags |= 0x1;
				crc_summ = 0;
				begin_bt_2:
				if(command_buffer[0] == 0x04 && command_buffer[1] == 0xe4)
				{
					if(command_buffer[2] != 0x1)
					{
						if(bt_buffer[0] != 0)
						{
							bt_buffer[0] = 1;
							*((char*)&bt_power + 0) = bt_buffer[5];
							*((char*)&bt_power + 1) = bt_buffer[4];
							bt_power /= 3;
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = 0x05;
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = 0xe4;
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = bt_buffer[0];													// nalich bt		
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = bt_power;															//bt power
							crc_buffer = bt_buffer[0] + bt_power + 0xe9;
							while ((USART3->SR & USART_SR_TXE) == 0);
							USART3->DR = crc_buffer;	
							if(bt_buffer[0] == 0)
								goto begin_bt;
							else
								flags |= 0x400;
						}
					}
					else
					{
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0x05;
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0xe4;
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0;													
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0;
						while ((USART3->SR & USART_SR_TXE) == 0);
						USART3->DR = 0xe9;
						flags |= 0x400;
					}
					clear_command_buffer();
					crc_buffer = 0;
				}
			}
		}
		flags &= ~0x400;
	}

	input_generation_parameters_state();
}
