#include "Req.h"

//@TODO Add Error Control Functions

void I2C1_start(void){
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)); //Wait until start bit is set
}


void I2C1_SendAddress(uint8_t address, uint8_t direction){
	I2C1->DR = (address << 1) | direction; //direction---> 0:write,1:read
	while(!(I2C1->SR1 & I2C_SR1_ADDR));	//Wait until address sent bit is set
	volatile uint32_t temp = I2C1->SR2 | I2C1->SR1; //Clear SR1 and SR2
	(void)temp;
}


void I2C1_SendData(uint8_t data){
	I2C1->DR = data;
	while(!(I2C1->SR1 & I2C_SR1_TXE)); //wait until TXE bit is high
}


uint8_t I2C1_ReceiveData(void){
	while(!(I2C1->SR1 & I2C_SR1_RXNE)); //wait until RXNE is high
	return (uint8_t)I2C1->DR;
}


void I2C1_EnableAcknowledge(void){
	I2C1->CR1 |= I2C_CR1_ACK;
}


void I2C1_DisableAcknowledge(void){
	I2C1->CR1 &= ~I2C_CR1_ACK;
}


void I2C1_Stop(void){
	I2C1->CR1 |= I2C_CR1_STOP;
}

//Write one byte
void I2C1_WriteByte(uint8_t slaveAddress, uint8_t data){
	I2C1_start();
	I2C1_SendAddress(slaveAddress,0); //0:write direction
	I2C1_SendData(data);
	I2C1_Stop();
}


uint8_t I2C1_ReadByte(uint8_t slaveAddress){
	uint8_t data;
	I2C1_start();
	I2C1_SendAddress(slaveAddress,1); //1:write
	I2C1_DisableAcknowledge(); //Send NACK before last byte
	data=I2C1_ReceiveData();
	I2C1_Stop();
	return data;
}

//Reading a single byte from a specific address
uint8_t I2C1_ReadRegister(uint8_t slaveAddress, uint8_t registerAddress){
	uint8_t data;
	I2C1_start();
	I2C1_SendAddress(slaveAddress,0); //0:write
	I2C1_SendData(registerAddress); //Send register address
	I2C1_start(); //Repeated start
	I2C1_SendAddress(slaveAddress,1); //1:read
	I2C1_DisableAcknowledge();
	data = I2C1_ReceiveData();
	I2C1_Stop();
	return data;
}

// Write a single byte to a specific address
void I2C1_WriteRegister(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data){
	I2C1_start();
	I2C1_SendAddress(slaveAddress,0); //0:write 
	I2C1_SendData(registerAddress); //Send the register address to be written 
	I2C1_SendData(data);	//Send the data address to be writen
	I2C1_Stop();
}