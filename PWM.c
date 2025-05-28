#include "Req.h"

// System clock freq. (168 MHz)
#define SYSCLK_FREQ 168000000U

// APB1 ve APB2 clock frequency
#define APB1_CLK (SYSCLK_FREQ / 4) // 42MHz, APB1 Prescaler = 4
#define APB2_CLK (SYSCLK_FREQ / 2) // 84MHz, APB2 Prescaler = 2

#define PWM_FREQUENCY 1000		// PWM freq. 

#define PWM_RESOLUTION 1000		//@TODO Set according to the servo motor application

void tim1_pwm_init(void){
	//1. Enable timer 1 clock 
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	
	//2. Enable GPIOE clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	
	//3. Set alternate functions mode of PE13 (TIM1_CH3) and PE14 (TIM1_CH4)
	GPIOE->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1;
	
	//4. Set alternate functions of AF1 (TIM1)
	GPIOE->AFR[1] |= 	(1 << (14-2)*2) | (1 << (13-3)*2); //
	
	//5. Timer 1 settings
	//Prescaler calculation
	uint16_t tim1_prescaler = (uint16_t)((APB2_CLK/PWM_FREQUENCY/PWM_RESOLUTION)-1);
	
	uint16_t tim1_arr = PWM_RESOLUTION -1;
	
	TIM1->PSC = tim1_prescaler;
	
	TIM1->ARR = tim1_arr;
	
	//6. Channel 3 settings (PWM Mode 1)
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3CE;  // PWM Mode 1, Output Compare 3 Preload Enable
	TIM1->CCER |= TIM_CCER_CC3E;		// Capture/Compare 3 Output Enable
	
	//7. Channel 4 settings (PWM Mode 1)
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4PE; // PWM Mode 1, Output Compare 4 Preload Enable
	TIM1->CCER |= TIM_CCER_CC4E; //Capture/Compare 4 Output Enable
	
	//8. Enable main output enable
	TIM1->BDTR |= TIM_BDTR_MOE;
}



void tim2_pwm_init(void){
	//1. Enable TIM2 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//2. Enable GPIOB clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
	//3. Configure PB10 (TIM2_CH3) and PB11 (TIM2_CH4) are as Alternate Function mode
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1; 
	
	//4. Set alternate function AF1 (TIM2)
	GPIOB->AFR[1] |= (1 << 8) | (1 << 12);
	
	//5. Timer 2 settings
	//Prescaler calculation
	uint16_t tim2_prescaler = (uint16_t)((APB1_CLK/PWM_FREQUENCY/PWM_RESOLUTION) - 1);
	
	//Auto-reload value
	uint16_t tim2_arr = PWM_RESOLUTION -1; //Start 0
	
	TIM2->PSC = tim2_prescaler;
	
	TIM2->ARR = tim2_arr;
	
	// 6. Kanal 3 settings (PWM Mode 1)
  TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3PE; // PWM Mode 1, Output Compare 3 Preload Enable
  TIM2->CCER |= TIM_CCER_CC3E; // Capture/Compare 3 Output Enable

	// 7. Kanal 4 settings (PWM Mode 1)
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4PE; // PWM Mode 1, Output Compare 4 Preload Enable
	TIM2->CCER |= TIM_CCER_CC4E; // Capture/Compare 4 Output Enable
	
}


void tim3_pwm_init(void){
// 1. Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // 2. Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // 3. Configure PA6 (TIM3_CH1) ve PA7 (TIM3_CH2) olarak Alternate Function
    GPIOA->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;

    // 4. Seçenek fonksiyon AF2 (TIM3) seçimi
    GPIOA->AFR[0] |= (2 << 24) | (2 << 28); // AFRL6 -> PA6, AFRL7 -> PA7

    // 5. Timer 3 yapilandirmasi
    // Prescaler hesaplamasi
    uint16_t tim3_prescaler = (uint16_t)((APB1_CLK / PWM_FREQUENCY / PWM_RESOLUTION) - 1);

    // Auto-reload degeri
    uint16_t tim3_arr = PWM_RESOLUTION - 1; // 0'dan basladigi için

    TIM3->PSC = tim3_prescaler;
    TIM3->ARR = tim3_arr;

    // 6. Kanal 1 yapilandirmasi (PWM Mode 1)
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1PE; // PWM Mode 1, Output Compare 1 Preload Enable
    TIM3->CCER |= TIM_CCER_CC1E; // Capture/Compare 1 Output Enable

    // 7. Kanal 2 yapilandirmasi (PWM Mode 1)
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2PE; // PWM Mode 1, Output Compare 2 Preload Enable
    TIM3->CCER |= TIM_CCER_CC2E; // Capture/Compare 2 Output Enable	

}



void start_tim1(void){
	TIM1->CR1 |= TIM_CR1_CEN;
}


void start_tim2(void){
	TIM2->CR1 |= TIM_CR1_CEN;
}



void start_tim3(void){
	TIM3->CR1 |= TIM_CR1_CEN;
}



//Timer1 set duty-cycle functions
void set_tim1_ch3_duty_cycle(uint32_t duty){
    if (duty <= PWM_RESOLUTION) {
        TIM1->CCR3 = duty;
    }
}



void set_tim1_ch4_duty_cycle(uint32_t duty){
    if (duty <= PWM_RESOLUTION) {
        TIM1->CCR4 = duty;
    }
}


//Timer2 set duty-cycle functions
void set_tim2_ch3_duty_cycle(uint32_t duty){
    if (duty <= PWM_RESOLUTION) {
        TIM2->CCR3 = duty;
    }
}



void set_tim2_ch4_duty_cycle(uint32_t duty){
		if(duty <= PWM_RESOLUTION){
				TIM2->CCR4 = duty;
		}
}



//Timer3 set duty-cycle functions
void set_tim3_ch1_duty_cycle(uint32_t duty){
		if(duty <= PWM_RESOLUTION){
				TIM3->CCR1 = duty;
		}
}


void set_tim3_ch2_duty_cycle(uint32_t duty){
		if(duty <= PWM_RESOLUTION){
				TIM3->CCR2 = duty;
		}
}
