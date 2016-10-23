#include "stm32f0xx.h"

const uint8_t stringtosend[] = "Hardware Flow Control\n";
uint8_t send = 0;

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
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH1)) | (1<<((9-8)*4)); /* (5) */
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH2)) | (1<<((10-8)*4)); /* (6) */
	return;
}

__INLINE void UART_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	//UART clock enable
	USART1->BRR = 480000 / 96;
	USART1->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;
	while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)/* polling idle frame Transmission */
  { 
    /* add time out here for a robust application */
  }
	  /* Configure IT */
  /* (4) Set priority for USART1_IRQn */
  /* (5) Enable USART1_IRQn */
  NVIC_SetPriority(USART1_IRQn, 0); /* (3) */
  NVIC_EnableIRQ(USART1_IRQn); /* (4) */
	return;
}

void USART1_IRQHandler(void)
{
  uint8_t chartoreceive = 0;
  
  if((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
  {
    if(send == sizeof(stringtosend))
    {
      send=0;
      USART1->ICR |= USART_ICR_TCCF;/* Clear transfer complete flag */
    }
    else
    {
      /* clear transfer complete flag and fill TDR with a new char to send */
      USART1->TDR = stringtosend[send++];
    }
  }
  else if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
  {
    chartoreceive = (uint8_t)(USART1->RDR);/* Receive data, clear flag */
          
    switch(chartoreceive)
    {
    case 'g':
    case 'G': GPIOC->ODR ^= GPIO_ODR_9; /* Toggle Green LED */
              break;
    default: break;
    }
  }
  else
  {
    NVIC_DisableIRQ(USART1_IRQn);/* Disable USART1_IRQn */
  }
}
