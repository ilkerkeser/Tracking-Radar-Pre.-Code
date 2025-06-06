#ifndef __as5048b_H
#define __as5048b_H


#include "Req.h"

#define READ_ANGLE_CMD 0xFE

typedef struct __AS5048B_HandleTypeDef {
  uint8_t ADDRESS;
  uint8_t TxBuffer[1];
  uint8_t RxBuffer[2];
} AS5048B_HandleTypeDef;

uint8_t AS5048B_Init(AS5048B_HandleTypeDef* has5048b, uint8_t ADDRESS);
uint16_t AS5048B_Read_RawAngle(AS5048B_HandleTypeDef* has5048b);
double AS5048B_Read_DegAngle(AS5048B_HandleTypeDef* has5048b);

#endif