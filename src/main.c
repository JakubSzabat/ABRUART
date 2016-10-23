#include "stm32f0xx.h"

uint8_t baudrate_tab[8];
uint8_t send = 0;

__INLINE void UART_Init(void);
__INLINE void GPIO_Init(void);
__INLINE void DEC_Place(void);

int main(void){
	
	GPIO_Init();
	UART_Init();
	DEC_Place();
	
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
	
	USART1->BRR = 8000000UL / 9600;
	USART1->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE | USART_CR1_TCIE;
	
	while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)/* polling idle frame Transmission */
  { 
    /* add time out here for a robust application */
  }
 	USART1->ICR |= USART_ICR_TCCF;/* Clear TC flag */

	USART1->CR2 &= (uint32_t)~((uint32_t)USART_CR2_ABRMODE); //ABR config
  //USART1->CR2 |= USART_CR2_ABRMODE_0;
	USART1->CR2 |= USART_CR2_ABREN;
	
	while((USART1->ISR & USART_ISR_REACK) == (uint16_t)RESET){}
	while((USART1->ISR & USART_ISR_TEACK) == (uint16_t)RESET){}
	while((USART1->ISR & USART_ISR_ABRF)  == (uint16_t)RESET){}
	if((USART1->ISR & USART_ISR_ABRE)  != (uint16_t)RESET){
		return;
	}
	
  NVIC_SetPriority(USART1_IRQn, 0); /* (3) */
  NVIC_EnableIRQ(USART1_IRQn); /* (4) */
	return;
}

void USART1_IRQHandler(void)
{
  uint8_t chartoreceive = 0;
	
	
  if((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
  {
		if(send == sizeof(baudrate_tab))
    {
      send=0;
      USART1->ICR |= USART_ICR_TCCF;/* Clear transfer complete flag */
    }
    else
    {
      /* clear transfer complete flag and fill TDR with a new char to send */
      USART1->TDR = baudrate_tab[send++];
    }
  }
  else if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
  {
    chartoreceive = (uint8_t)(USART1->RDR);/* Receive data, clear flag */
    USART1->TDR = baudrate_tab[send++];/* Will inititiate TXEI if TXE */
  }
  else
  {
    NVIC_DisableIRQ(USART1_IRQn);/* Disable USART1_IRQn */
  }
}

__INLINE void DEC_Place(void){											 //wyluskanie wartosci wspolczynnikow formatu dziesietnego
	uint32_t baudrate = 8000000UL / (USART1->BRR);
	baudrate_tab[0] =  0x0D;					//send return
	baudrate_tab[1] =  baudrate/100000;
	baudrate_tab[2] = (baudrate-baudrate_tab[1]*100000)/10000;
	baudrate_tab[3] = (baudrate-baudrate_tab[1]*100000-baudrate_tab[2]*10000)/1000;
	baudrate_tab[4] = (baudrate-baudrate_tab[1]*100000-baudrate_tab[2]*10000-baudrate_tab[3]*1000)/100;
	baudrate_tab[5] = (baudrate-baudrate_tab[1]*100000-baudrate_tab[2]*10000-baudrate_tab[3]*1000-baudrate_tab[4]*100)/10;
	baudrate_tab[6] = (baudrate-baudrate_tab[1]*100000-baudrate_tab[2]*10000-baudrate_tab[3]*1000-baudrate_tab[4]*100-baudrate_tab[5]*10);
	baudrate_tab[7] =  0x0A;					//send new-line
	
	baudrate_tab[1] += 0x30;
	baudrate_tab[2] += 0x30;
	baudrate_tab[3] += 0x30;
	baudrate_tab[4] += 0x30;
	baudrate_tab[5] += 0x30;
	baudrate_tab[6] += 0x30;
}
