#include "stm32f10x.h"
#include "main.h"


void makeItGoFast(void);
void GPIO_Init(void);

int main(void)
{
   makeItGoFast();
   GPIO_Init();
	 
	RCC->APB2ENR |= (1<<4U);

	 GPIOC->CRH |=  (1<<20U);
   GPIOC->CRH &= ~(1<<21U);
	 GPIOC->CRH &= ~(1<<22U);
	 GPIOC->CRH &= ~(1<<23U);

	while(1)
	{	
//		GPIOC->ODR ^= (1<<13);
//		for(int i=0 ; i<2000000;i++);
	
	}

	
}

void makeItGoFast(void)
{
	   // Conf clock : 72MHz using HSE 8MHz crystal w/ PLL X 9 (8MHz x 9 = 72MHz)
    FLASH->ACR      |= FLASH_ACR_LATENCY_2; // Two wait states, per datasheet
    RCC->CFGR       |= RCC_CFGR_PPRE1_2;    // prescale AHB1 = HCLK/2
    RCC->CR         |= RCC_CR_HSEON;        // enable HSE clock
    while( !(RCC->CR & RCC_CR_HSERDY) );    // wait for the HSEREADY flag
    
    RCC->CFGR       |= RCC_CFGR_PLLSRC;     // set PLL source to HSE
    RCC->CFGR       |= RCC_CFGR_PLLMULL9;   // multiply by 9
    RCC->CR         |= RCC_CR_PLLON;        // enable the PLL
    while( !(RCC->CR & RCC_CR_PLLRDY) );    // wait for the PLLRDY flag
    
    RCC->CFGR       |= RCC_CFGR_SW_PLL;     // set clock source to pll

    while( !(RCC->CFGR & RCC_CFGR_SWS_PLL) );    // wait for PLL to be CLK
    
    SystemCoreClockUpdate();                // calculate the SYSCLOCK value
}

void GPIO_Init(void)
{
	//turn gpio clk on
	RCC->APB2ENR |= (1<<3); //GPIOB CLOCK ON
	RCC->APB2ENR |= (1<<0); //AFIO CLOCK ON	
	
	//set pin as input

	GPIOB->CRL &= ~(1<<1);
	GPIOB->CRL &= ~(1<<0);   // PB0 SET AS INPUT
	
	GPIOB->CRL &= ~(1<<5);
	GPIOB->CRL &= ~(1<<4);   // PB1 SET AS INPUT
	
	GPIOB->CRL &= ~(1<<9);
	GPIOB->CRL &= ~(1<<8);   // PB2 SET AS INPUT
	
	GPIOB->CRL &= ~(1<<13);
	GPIOB->CRL &= ~(1<<12);   // PB3 SET AS INPUT
	
	GPIOB->CRL |= (1<<3);
	GPIOB->CRL &= ~(1<<2);   // PB0 INPUT with pull-up/pull-down

	GPIOB->CRL |= (1<<7);
	GPIOB->CRL &= ~(1<<6);   // PB1 INPUT with pull-up/pull-down
	
	GPIOB->CRL |= (1<<11);
	GPIOB->CRL &= ~(1<<10);   // PB2 INPUT with pull-up/pull-down

	GPIOB->CRL |= (1<<15);
	GPIOB->CRL &= ~(1<<14);   // PB3 INPUT with pull-up/pull-down	
	
	
	
	AFIO->EXTICR[0] |= (1<<0);  //PB0 EXTI0
	AFIO->EXTICR[0] |= (1<<4);  //PB1 EXTI1		
	AFIO->EXTICR[0] |= (1<<8);  //PB2 EXTI2
	AFIO->EXTICR[0] |= (1<<12);  //PB3 EXTI3
	 //do  inttrpt config
	
	 //1= configure EXTI->IMR
	 EXTI->IMR |= (1<<0); //for PB0
	 EXTI->IMR |= (1<<1); //for PB1	
	 EXTI->IMR |= (1<<2); //for PB2
	 EXTI->IMR |= (1<<3); //for PB3	
	 //2= configure EXTI->RTSR AND FTSER
	 EXTI->RTSR |= (1<<0);//for PB0
	 EXTI->RTSR |= (1<<1);//for PB1	 
	 EXTI->RTSR |= (1<<2);//for PB2
	 EXTI->RTSR |= (1<<3);//for PB3
	 //3= enable inttrupt for nvic
	 NVIC_EnableIRQ(EXTI0_IRQn);	//for PB0
	 NVIC_EnableIRQ(EXTI1_IRQn);	//for PB1
	 NVIC_EnableIRQ(EXTI2_IRQn);	//for PB2
	 NVIC_EnableIRQ(EXTI3_IRQn);	//for PB3
}

void EXTI0_IRQHandler (void)
{

GPIOC->ODR ^= (1<<13);
	
}

void EXTI1_IRQHandler (void)
{

GPIOC->ODR ^= (1<<13);
	
}

void EXTI2_IRQHandler (void)
{

GPIOC->ODR ^= (1<<13);
	
}

void EXTI3_IRQHandler (void)
{

GPIOC->ODR ^= (1<<13);
	
}

