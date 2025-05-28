
#include "Req.h"

char usart1_read_char(void){
	//check status of RX is ready ()
	while(!(USART1->SR & USART_SR_RXNE));
	return USART1->DR;
}

void usart1_read_string(char *buffer, uint16_t max_len){
	uint16_t i =0;
	char c;
	
	while(i < (max_len - 1)){ //Except null
		c = usart1_read_char();
		
		if(c == '\n' || c == '\r'){
				break;
		}
		
		buffer[i++] = c;
	
	}
	
	buffer[i] = '\0'; //End of the string
}

void usart1_send_char(char c){
	//check status of TX is ready (TDR is empty?)
	while(!(USART1->SR & USART_SR_TXE)); //(0x1 << 7)
	USART1->DR = c;
}

void usart1_send_string(const char *str){
	while(*str){
		usart1_send_char(*str++);
	}
}

char usart2_read_char(void){
	//check status of RX is ready ()
	while(!(USART2->SR & USART_SR_RXNE));
	return USART2->DR;
}

void usart2_read_string(char *buffer, uint16_t max_len){
	uint16_t i =0;
	char c;
	
	while(i < (max_len - 1)){ //Except null
		c = usart2_read_char();
		
		if(c == '\n' || c == '\r'){
				break;
		}
		
		buffer[i++] = c;
	
	}
	
	buffer[i] = '\0'; //End of the string

}

void usart2_send_char(char c){
	//check status of TX is ready (TDR is empty?)
	while(!(USART2->SR & USART_SR_TXE)); //(0x1 << 7)
	USART2->DR = c;
}


void usart2_send_string(const char *str){
	while(*str){
		usart2_send_char(*str++);
	}
}


/*char usart3_read_char(void){
	while(!(USART3->SR & USART_SR_RXNE));
	return USART3->DR;
}*/

/*void usart3_read_string(char *buffer, uint16_t max_len){
	uint16_t i =0;
	char c;

	while(i < (max_len -1)){
		c = usart3_read_char();
		
		if(c == '\n' || c == '\r'){
			break;
		}
		buffer[i++] = c;
	}
	
	buffer[i] = '\0'; //End of the string
	
}*/

void usart3_read_string_dma(char *buffer, uint16_t max_len){
	if(rx_data_ready_flag){
		rx_data_ready_flag =0;
		
		uint16_t copy_len = (rx_data_length < (max_len - 1)) ? rx_data_length : (max_len - 1);
		
		for (uint16_t i = 0; i < copy_len; i++)
    {
       buffer[i] = rx_dma_buffer[i];
    }
		
		buffer[copy_len] = '\0';  // Null-terminate
	}else{
		buffer[0] = '\0';  //There is no data
	}
	
}

void usart3_send_char(char c){
	
	//check status of TX is ready (TDR is empty?)
	while(!(USART3->SR & USART_SR_TXE)); //(0x1 << 7)
	USART3->DR = c;

}

void usart3_send_string(const char *str){
	while(*str){
		usart3_send_char(*str++);
	}
}