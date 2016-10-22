#include "stm32f0xx.h"

__INLINE void UART_Init(void);
__INLINE void GPIO_Init(void);

int main(void){
	
	GPIO_Init();
	UART_Init();
	
	while(1){
		__WFI();
	}
}

__INLINE void GPIO_Init(void){
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;		//GPIOA clock enable
	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10))
								 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;	//PA9, PA10 as an alternate function
	
	return;
}

__INLINE void UART_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	//UART clock enable

	return;
}
