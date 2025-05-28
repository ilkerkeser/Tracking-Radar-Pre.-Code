#include "Req.h"

void SPI3_Transmit(uint8_t *transmit_data, uint32_t data_length){
	SPI3_CS_Enable();
	
	for(uint32_t i =0;i< data_length; i++){
		while(!(SPI3->SR & SPI_SR_TXE));		//wait until TX buffer is empty
		SPI3->DR = transmit_data[i];
	}
	
	while(SPI3->SR & SPI_SR_BSY); 		//Wait until last message
	
	SPI3_CS_Disable();
}


void SPI3_Receive(uint8_t *receive_buffer, uint32_t receive_buffer_length){
	SPI3_CS_Enable();
	
	for(uint32_t i =0; i< receive_buffer_length; i++){
		while(!(SPI3->SR & SPI_SR_TXE));		//wait until TX buffer empty  
		SPI3->DR = 0xFF;		//Send Dummy data
		while(!(SPI3->SR & SPI_SR_RXNE));		//wait until RX buffer not empty
		receive_buffer[i] = (uint8_t)SPI3->DR;
	}
	
	SPI3_CS_Disable();
}


void SPI3_CS_Enable(void){
	GPIOD->BSRR |= (1 << 0); 
}


void SPI3_CS_Disable(void){
	GPIOD->BSRR |= (1 << 16);	//Depend on LoRa RA02 reset type (active low?)
}