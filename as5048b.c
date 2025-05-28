#include "as5048b.h"

uint8_t AS5048B_Init(AS5048B_HandleTypeDef* has5048b, uint8_t ADDRESS)
{
  
  has5048b->ADDRESS = ADDRESS<<1;
	I2C1_gpio_init();
  return 1;
}

uint16_t AS5048B_Read_RawAngle(AS5048B_HandleTypeDef* has5048b)
{
		has5048b->TxBuffer[0] = READ_ANGLE_CMD;
	
		while(I2C1->SR2 & I2C_SR2_BUSY);
		
		
		I2C1_WriteByte((uint16_t)has5048b->ADDRESS,has5048b->TxBuffer[0]);
		Delay_ms(20);
		
		for(uint32_t i=0;i<2;i++){
			has5048b->RxBuffer[i]	= I2C1_ReadByte(has5048b->ADDRESS);
			Delay_ms(10);
		}
		
  return (has5048b->RxBuffer[1]<<6) | (has5048b->RxBuffer[0]&0x3F);
}

double AS5048B_Read_DegAngle(AS5048B_HandleTypeDef* has5048b)
{
  return AS5048B_Read_RawAngle(has5048b)*360.0/16383;
}