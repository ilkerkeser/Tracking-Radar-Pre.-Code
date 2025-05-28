#include "Req.h"


RCC_TypeDef *pRCC;
GPIO_TypeDef *pGPIOA;
GPIO_TypeDef *pGPIOB;
GPIO_TypeDef *pGPIOC;
GPIO_TypeDef *pGPIOD;
USART_TypeDef *pUSART1;
USART_TypeDef *pUSART2;
USART_TypeDef *pUSART3;
SPI_TypeDef *pSPI3;
I2C_TypeDef *pI2C1;

void gpio_usart1_init(void){
	
	pRCC = RCC;
	
	pGPIOA = GPIOA;
	
	pUSART1 = USART1;
	
	//1. enable the peripheral clock for the usart2 peripheral
	pRCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	//2. Configure the gpio pins for usart_tx and usart_rx functionality 
	//PA9 as TX , PA10 as RX
	
	//PA9 as USART1_TX
	//2.1 Enable the clock for the GPIOA peripheral	
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	//2.2 Change the mode of the PA9 to alternate function mode
	pGPIOA->MODER &= ~GPIO_MODER_MODER9;
	pGPIOA->MODER |= GPIO_MODER_MODE9_1;
	pGPIOA->AFR[1] &= ~GPIO_AFRH_AFRH1; //~(0xF << 4)
	pGPIOA->AFR[1] |= (0x7 << 4);
	
	//2.3 Enable or disable Pull-up resistor of the gpio pin if required. 
	pGPIOA->PUPDR &= ~(0x3 << 18);
	pGPIOA->PUPDR |= (0x1 << 18);
	
	//PA10 as USART1_RX
	//2.4 Change the mode of the PA10 to alternate function mode
	pGPIOA->MODER &= ~GPIO_MODER_MODER10;
	pGPIOA->MODER |= GPIO_MODER_MODE10_1;
	pGPIOA->AFR[1] &= GPIO_AFRH_AFRH2;
	pGPIOA->AFR[1] |= (0x7 << 8);
	
	//2.5 enable the pull-up resistor
	pGPIOA->PUPDR &= ~(0x3 << 20);
	pGPIOA->PUPDR |= (0x1 << 18);
	
	//3. Configure the baudrate
	pUSART1->BRR = 	0x222E;			//USARTDIV = USART_CLK/8x(2 - OVER8)x Baudrate--> Baudrate 9600, USART2_CLK = 84MHz, OVER8 =0
															//84MHz/16x9600 = 546,875 --> Mantissa = 546, Fraction = 0,875 * 16 = 14
															//Mantissa = 0x222, Fraction = 0xE---> USARTDIV = 0x222E
	//4. Enable the TX, RX of the usart peripheral and usart peripheral														
	pUSART1->CR1 |= (0x1 << 3) | (0x1 << 2) | (0x1 << 13);
}



void gpio_usart2_init(void){
	
	pRCC = RCC;
	
	pGPIOA = GPIOA;
	
	pUSART2 = USART2;
	
	//1. enable the peripheral clock for the usart2 peripheral
	pRCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	
	//2. Configure the gpio pins for usart_tx and usart_rx functionality 
	//PA2 as TX , PA3 as RX
	
	//PA2 as USART2_TX
	//2.1 Enable the clock for the GPIOA peripheral	
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	//2.2 Change the mode of the PA2 to alternate function mode
	pGPIOA->MODER &= ~GPIO_MODER_MODER2; //First set reset state ~(0x3 << 4)
	pGPIOA->MODER |= GPIO_MODER_MODE2_1;	//Set alternate function mode (0x2 << 4)
	pGPIOA->AFR[0] &= ~GPIO_AFRL_AFRL2; //~(0xF << 8)
	pGPIOA->AFR[0] |= (0x7 << 8); 	//Set alternate function 7 - AF7
	
	//2.3 Enable or disable Pull-up resistor of the gpio pin if required. 
	pGPIOA->PUPDR &= ~(0x3 << 4); 
	pGPIOA->PUPDR |= (0x1 << 4);
	
	//PA3 as USART2_RX
	//2.4 Change the mode of the PA3 to alternate function mode
	pGPIOA->MODER &= ~GPIO_MODER_MODER3; 	//First set reset state ~(0x3 << 6)
	pGPIOA->MODER |= GPIO_MODER_MODE3_1;	//Set alternate function mode (0x2 << 6)
	pGPIOA->AFR[0] &= ~GPIO_AFRL_AFRL3; //~(0xF << 12)
	pGPIOA->AFR[0] |= (0x7 << 12);	//Set the alternate function7
	
	//2.5 enable the pull-up resistor
	pGPIOA->PUPDR &= ~(0x3 << 6);
	pGPIOA->PUPDR |= (0x1 << 6);
	
	//3. Configure the baudrate
	pUSART2->BRR = 0x16C;  //USARTDIV = USART_CLK/8x(2 - OVER8)x Baudrate--> Baudrate 115200, USART2_CLK = 42MHz, OVER8 =0
												//42MHz/16x115200 = 22,78 --> Mantissa = 22, Fraction = 0,78 * 16 = 12
												//Mantissa = 0x16, Fraction = 0xC---> USARTDIV = 0x16C
											
	
	//4 . configure the data width, no of stop bits , etc
	// <no configuration reqd here , we will use default values
	
	//5. Enable the TX, RX of the usart peripheral and usart peripheral
	pUSART2->CR1 |= (0x1 << 3) | (0x1 << 2) | (0x1 << 13);
	
	//@TODO: Do Interrupt part
}

void gpio_usart3_init(void){
	
	pRCC = RCC;
	
	pGPIOD = GPIOD;
	
	pUSART3 = USART3;
	
	//1. Configure the gpio pins for usart_tx and usart_rx functionality 
	//PD8 as TX , PD9 as RX
	
	//PD8 as USART2_TX
	//1.1 Enable the clock for the GPIOD peripheral	
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //(1 << 3)
	
	//1.2 Change the mode of the PD8 to alternate function mode
	pGPIOD->MODER &= ~GPIO_MODER_MODER8;	//First set reset state ~(0x3 << 16)
	pGPIOD->MODER |= GPIO_MODER_MODE8_1;	//Set alternate function mode (0x2 << 16)
	pGPIOD->AFR[1] &= ~GPIO_AFRH_AFRH0; 	//~(0xF << 0)
	pGPIOD->AFR[1] |= (0x7 << 0);	//Set alternate function 7 -AF7
	
	//1.3 Enable or disable Pull-up resistor of the gpio pin if required.
	pGPIOD->PUPDR &= ~(0x3 << 16);
	pGPIOD->PUPDR |= (0x1 << 16);
	
	//PD9 as USART3_RX
	//1.4 Change the mode of the PD9 to alternate function mode
	pGPIOD->MODER &= ~GPIO_MODER_MODER9;	//First set reset state ~(0x3 << 18)
	pGPIOD->MODER |= GPIO_MODER_MODER9_1;	//Set alternate function mode (0x2 << 18)
	pGPIOD->AFR[1] &= ~GPIO_AFRH_AFRH1; //~(0xf << 4)
	pGPIOD->AFR[1] |= (0x7 << 4); //Set alternate function 7 - AF7
	
	//1.5 Enable or disable Pull-up resistor of the gpio pin if required.
	pGPIOD->PUPDR &= ~(0x3 << 18);
	pGPIOD->PUPDR |= (0x1 << 18);
	
	//3. enable the peripheral clock for the usart3 peripheral
	pRCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	//3.1 Configure the baudrate of USART3
	pUSART3->BRR = 0x1117; //USARTDIV = USART_CLK/8x(2 - OVER8)x Baudrate--> Baudrate 9600, USART2_CLK = 42MHz, OVER8 =0
												//42MHz/16x9600 = 273,4375 --> Mantissa = 273, Fraction = 0x4375 * 16 = 7
												//Mantissa = 0x111, Fraction = 0x7---> USARTDIV = 0x1117
	
	//4 . configure the data width, no of stop bits , etc
	// <no configuration reqd here , we will use default values
	
	//5. Enable the TX, RX of the usart peripheral and usart peripheral
	pUSART3->CR1 |= (0x1 << 3) | (0x1 << 2) | (0x1 << 13);
	pUSART3->CR3 |= USART_CR3_DMAR; //RX DMA enable
	
	//6. DMA parts
	//6.1 Enable the DMA peripheral clock
	pRCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		
	//6.2 DMA1 Stream1 Channel4 settings
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;				//DMA disable
	while(DMA1_Stream1->CR & DMA_SxCR_EN); 	//wait until disable
	
	DMA1->LIFCR |= DMA_LIFCR_CTCIF1;			//Clear transfer complete flag

	DMA1_Stream1->PAR = (uint32_t)&USART3->DR;		//Peripheral address
	DMA1_Stream1->M0AR = (uint32_t)rx_dma_buffer; //Memorry address
	DMA1_Stream1->NDTR = RX_BUFFER_SIZE;					//Memory size
	DMA1_Stream1->CR = 0;		//first clear configuration register (CR)
	DMA1_Stream1->CR |= (0x4 << 25); 		//Channel 4 selected
	DMA1_Stream1->CR |= (0x1 << 10);		//Meemory increment
	DMA1_Stream1->CR |= (0x1 << 4);			// Transfer complete interrupt enable
	DMA1_Stream1->CR |= ~(0x3 << 6);		//Peripheral to memory
	DMA1_Stream1->CR |= DMA_SxCR_EN;	//Stream enable, start DMA
	
	//6.3 Start the NVIC interrupt
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

void DMA1_Stream1_IRQHandler(void){
	if(DMA1->LISR & DMA_LISR_TCIF1){
			DMA1->LIFCR |= DMA_LIFCR_CTCIF1; //Clear the flag
			
		rx_data_ready_flag = 1;
		rx_data_length = RX_BUFFER_SIZE;
		
		//Restart DMA 
		DMA1_Stream1->CR &= ~DMA_SxCR_EN; 			//Disable DMA
		while(DMA1_Stream1->CR & DMA_SxCR_EN);	//wait until DMA is disable
		DMA1_Stream1->NDTR = RX_BUFFER_SIZE;		//Set number of data register
		DMA1_Stream1->CR |= DMA_SxCR_EN;				//Start DMA
	}
}

//I2C1 GPIO Settings
void I2C1_gpio_init(void){
	
	pRCC = RCC;
	
	pGPIOB = GPIOB;
	
	pI2C1 = I2C1;
	
	//1. Enable GPIOB's clocks
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
	//pRCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	//2. Set alternate function of PB8 (SCL) and PB9 (SDA)
	pGPIOB->MODER &= ~((3 << (8*2)) | (3 << (9*2))); //Clear moder 
	pGPIOB->MODER |= ((2 << (8*2)) | (2 << (9*2)));		//Set alternate function mode
	
	//3. Set AF4
	pGPIOB->AFR[1] &= ~((0xF << (8-8)*0) | (0xF << (9-8)*4));
	pGPIOB->AFR[1] |= ((4 << (8-8)*4) | (4 << (9-8)*4));
	
	//4. Enable Open drain and pull-up 
	pGPIOB->OTYPER |= ((1 << 8) | (1 << 9)); //Set open-drain
	pGPIOB->PUPDR &= ~((3 << (8*2)) | (3 << (9*2)));
	pGPIOB->PUPDR |= ((1 << (8*2)) | (1 << (9*2)));	//Set pull-up resistor
	
	//5. Set very high speed
	pGPIOB->OSPEEDR |= ((3 << (8*2)) | (3 << (9*2)));
	
	//6. Set I2C1 clock enable
	pRCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	//7.Reset I2C1
	pI2C1->CR1 |= I2C_CR1_SWRST;
	pI2C1->CR1 &= ~I2C_CR1_SWRST;
	
	//8. Set clock control settings
	pI2C1->CR2 = 42; //APB1 clock
	pI2C1->CCR = 35; //PCLK/(3*I2C_SPEED) -- fast mode
	pI2C1->TRISE = 13;
	
	//9. Select fast mode and set Duty:0
	pI2C1->CCR |= I2C_CCR_FS;
	pI2C1->CCR &= ~I2C_CCR_DUTY; //DUTY:0, Duty Cycle Tlow/Thigh =2
	
	//10. Enable I2C peripheral
	pI2C1->CR1 |= I2C_CR1_PE;
}

//SPI3 GPIO Settings
void SPI3_GPIO_Init(void){
	pRCC = RCC;
		
	pGPIOC = GPIOC;
	
	pGPIOD = GPIOD;

	//1. Enable the clocks for GPIOC and GPIOD
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
	
	//2. Set alternate functions for SPI3
	pGPIOC->MODER &= ~((3 << (10*2)) | (3 << (11*2)) | (3 << (12*2)));
	pGPIOC->MODER |= ((2 << (10*2)) | (2 << (11*2)) | (2 << (12*2)));
	pGPIOC->AFR[1] &= ~((0xF << ((10-8)*4)) | (0xF << ((11-8)*4)) | (0xF << ((12-8)*4)));
	pGPIOC->AFR[1] |= ((6 << ((10-8)*4)) | (6 << ((11-8)*4)) | (6 << ((12-8)*4)));
	
	//3. No need Output push-pull and pull-up/down
	pGPIOC->OSPEEDR |= (3 << (10*2) | (3 << (11*2)) | (3 << (12*2))); //Set very high speed
	pGPIOC->OTYPER &= ~((1 << 10) | (1 << 11) | (1 << 12)); //push-pull
	pGPIOC->PUPDR &= ~((3 << (10*2)) | (3 << (11*2)) | (3 << (12*2))); //no pull-up/down
	
	//4. Set CS (PD0) as output
	pGPIOD->MODER &= ~(3 << 0);
	pGPIOD->MODER |= (1 << 0); //output mode
	pGPIOD->ODR |= (1 << 0);	//CS high
	
	
	//5. Enable SPI3 clock
	pRCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	
	//6. SPI3 settings
	pSPI3->CR1 = 0;
	pSPI3->CR1 |= SPI_CR1_MSTR;	//Set master mode
	pSPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; //Enable software slave management and internal slave select
	pSPI3->CR1 |= SPI_CR1_BR_0; 		//Baud rate = fPLCK/4 = 10.5 Mbits/s
	pSPI3->CR1 &= ~SPI_CR1_CPOL; 		//Clock idle low
	pSPI3->CR1 &= ~SPI_CR1_CPHA;		//Clock phase is 1 edge
	
	pSPI3->CR1 |= SPI_CR1_SPE;		//SPI enable
}
//Other GPIO pin functions
void gpio_led_init(void){
	
	//PA5
	//1. Set the GPIOA clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	//2. Set the output mode of the GPIOA
	GPIOA->MODER &= ~(1 << 10);
	GPIOA->MODER |= (1 << 10);
	
	//3. Configure the OUTPUT Mode
	GPIOA->OTYPER &= ~(0x1 << 5);	//Output push-pull (reset state)
	GPIOA->OSPEEDR &= ~(0x3 << 10); //Speed is low 
	GPIOA->PUPDR	&= ~(0x3 << 10);
}

//RA_02 GPIO settings
void RA_02_GPIO_Init(void){
	pRCC = RCC;
	
	pGPIOD = GPIOD;
	
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	
	//PD2->RA02_CE(RST), PD1->RA02_IRQ, PD0->RA02_CS(NSS)
	
	//1. Set output mode
	pGPIOD->MODER &= ~((3 << 0) | (3 << (1*2)) | (3 << (2*2)));
	pGPIOD->MODER |= ((1 << 0) | (1 << (1*2)) | (1 << (2*2)));
	
	//2. Configure Output mode
	pGPIOD->OTYPER &= ~((1 << 0) | (1 << 1) | (1 << 2)); //output push-pull
	pGPIOD->OSPEEDR |= ((3 << 0) | (3 << (1*2)) | (3 << (2*2))); //Set very high speed 
	pGPIOD->PUPDR &= ~((3 << 0) | (3 << (1*2)) | (3 << (2*2)));	//no pull-up/down 
	
}

void RA_02_Reset(void){
	GPIOD->BSRR |= (1 << (18)); //Reset PD2
}

void RA_02_Set(void){
	GPIOD->BSRR |= (1 << 0); //Set HIGH PD2;
}

//RA_08H_915 GPIO settings
void RA08_GPIO_Init(void){
	
	pRCC = RCC;
	
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
	
	//RA08_RST (PA8) and RA08_IO2 (PC9) GPIO Settings
	pGPIOA = GPIOA;
	
	pGPIOC = GPIOC;
	
	//PA8
	pGPIOA->MODER &= ~(3 << (8*2));
	pGPIOA->MODER |= (1 << (8*2));
	
	//Output configuration
	pGPIOA->OTYPER &= ~(1 << 8); //Output push-pull
	pGPIOA->OSPEEDR |= (2 << (8*2)); //High speed output
	pGPIOA->PUPDR &= ~(3 << (8*2)); //No pull-up/down
	
	//PC9
	pGPIOC->MODER &= ~(3 << (9*2));
	pGPIOC->MODER |= (1 << (9*2));
	
	//Output configuration
	pGPIOC->OTYPER &= ~(1 << 9); //Output push-pull
	pGPIOC->OSPEEDR |= (2 << (9*2)); //High speed output
	pGPIOC->PUPDR &= ~(3 << (9*2)); //No pull-up/down
}


void RA08_Reset(void){
	GPIOA->BSRR |= (1 << (8*2)); //PA8 low
}

void RA08_Boot(void){
	GPIOC->BSRR |= (1 << 9); //Boot mode of RA08
}

//M10S GPS/GNSS GPIO Settings
void M10S_GPIO_Init(void){
	
	pRCC = RCC;
	
	pGPIOD = GPIOD;
	
	pRCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	
	//PD12(GPS_ONOFF), PD11(LNA_EN--Input), PD10(M10S_RST) 
	pGPIOD->MODER &= ~((3 << (10*2)) | (3 << (11*2)) | (3 << (12*2)));
	pGPIOD->MODER |= ((1 << (10*2)) | (0 <<  (11*2)) | (1 << (12*2)));
	
	
	//Output configuration
	pGPIOD->OTYPER &= ~((1 << 10) | (1 << 12));					//Output push-pull
	pGPIOD->OSPEEDR |= ((2 << (10*2)) | (2 << (12*2))); //High speed output
	pGPIOD->PUPDR &= ~((3 << (10*2)) | (3 << (11*2)) | (3 << (12*2))); //No pull-up/down
}

uint8_t M10S_LNA_EN(void){
	uint8_t lna_en = (GPIOD->IDR >> 11) & 1; //Status of PD11
	return lna_en;
}



void M10S_RST(void){
	GPIOD->BSRR |= (1 << 26); //Active low
}



void M10S_ONOFF(uint8_t state){
		if(state == GPS_ON){
			GPIOD->BSRR |= (1 << 12); //GPS ON 
		}else if(state == GPS_OFF){
			GPIOD->BSRR |= (1 << 28); //GPS OFF
		}
}

