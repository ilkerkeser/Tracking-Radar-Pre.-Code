#include "Req.h"

//Define APB1 freq (42MHz)
#define APB1_CLOCK_FREQ		42000000UL

//Period
#define TICK_PERIOD_MS	1

//Define Timer clock freq (42*2 = 84MHz)
#define TIMER_CLOCK_FREQ (APB1_CLOCK_FREQ * 2)

//Define prescaler value
#define PRESCALER_VALUE (TIMER_CLOCK_FREQ / 1000000 - 1)

//Define auto-reload value (1ms period)
#define AUTORELOAD_VALUE (1000 - 1)

//Global tick timer
volatile uint32_t uwTick = 0;

void TIM4_Init(void){
	//1. Enable Timer4 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	//2. Set prescaler
	TIM4->PSC = PRESCALER_VALUE;
	
	//3. Set Auto-reload 
	TIM4->ARR = AUTORELOAD_VALUE;
	
	//4. Set Update Interrupt
	TIM4->DIER |= TIM_DIER_UIE;
	
	//5. Enable Timer
	TIM4->CR1 |= TIM_CR1_CEN;
	
	//6. Enable TIM4 interrupt in NVIC
	NVIC_SetPriority(TIM4_IRQn,0); 
	NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void){
	if(TIM4->SR & TIM_SR_UIF){
		uwTick++;		//Global tick timer 
		TIM4->SR &= ~TIM_SR_UIF; //Clear interrupt flag
	}
}

uint32_t GetTick(void){
	return uwTick;
}
