#include "Req.h"


void sendATCommand(char *command){
	usart1_send_string(command);
	Delay_ms(100);
	
	//End-of-line characters are added at the end of the command.
	char new_line[2] = "\r\n";
	usart1_send_string(new_line);
	Delay_ms(100);
}


char *receiveResponse(void){

	memset(responseBuffer,0,RESPONSE_BUFFER_SIZE); //clear buffer
	usart1_read_string((char *)responseBuffer,RESPONSE_BUFFER_SIZE-1);
	Delay_ms(100);
	return (char *)responseBuffer;
}

uint8_t *receiveMessage(char *data){
	
	sendATCommand("AT+DRX"); //Listen mode active
	
	while(1){
		data = receiveResponse();
		
		if(strlen(data)>0){
				//It can be checked whether the incoming data is an AT command response or a direct message.
				// If the incoming data is not an AT command response (for example, it does not start with "+"),
				// it can be considered a data message. This control can vary depending on the behavior of the module.
				if(data[0] != '+'){
						//@TODO process the message
						return (uint8_t *)data;
				}else{
						//Error message
				}
		}
		Delay_ms(100);
	}
}


void sendDataMessage(char *data){
	char sendCommand[64];

	sprintf(sendCommand,"AT+DTRX=%d,%s",strlen(data),data);
	sendATCommand(sendCommand);
	
	char *response = receiveResponse();
	
	if(strstr(response,"OK") != NULL){
			//Message is sent succesfully
	
	}else if(strstr(response,"ERROR") != NULL){
		//Error of sending message
		
	}else{
		//Unknown error
		
	}
	
}