#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"

#define RX_BUFFER_SIZE 64
#define MAX_DELAY 0xFFFFFFFFU
#define RA02_MESSAGE_SIZE		128
#define RESPONSE_BUFFER_SIZE 256
#define RECEIVE_BUFFER_SIZE	 256
#define RESPONSE_TIMEOUT 1000

#define GPS_ON		1
#define GPS_OFF 	0

extern uint8_t responseBuffer[RESPONSE_BUFFER_SIZE];
extern uint8_t receiveBuffer[RESPONSE_BUFFER_SIZE];

extern uint8_t u_ra02_receive_message[RA02_MESSAGE_SIZE];
extern uint8_t u_ra02_transmit_message[RA02_MESSAGE_SIZE];

extern volatile uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
extern volatile uint8_t rx_data_ready_flag;
extern volatile uint16_t rx_data_length;
void SysClockConfig(void);

//USART GPIO functions
void gpio_usart1_init(void);
void gpio_usart2_init(void);
void gpio_usart3_init(void);

//Timer for delay
void TIM5_Config(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
//USART1-3 Transmit and receive message functions
void usart1_send_char(char c);
void usart2_send_char(char c);
void usart3_send_char(char c);

char usart1_read_char(void);
char usart2_read_char(void);
//char usart3_read_char(void);

void usart1_read_string(char *buffer, uint16_t max_len);
void usart2_read_string(char *buffer, uint16_t max_len);
//void usart3_read_string(char *buffer, uint16_t max_len);

void usart1_send_string(const char *str);
void usart2_send_string(const char *str);
void usart3_send_string(const char *str);

//USART3 DMA functions
void DMA1_Stream1_IRQHandler(void);
void usart3_read_string_dma(char *buffer, uint16_t max_len);

//I2C1 functions
void I2C1_gpio_init(void);
void I2C1_start(void);
void I2C1_SendAddress(uint8_t address, uint8_t direction);
void I2C1_SendData(uint8_t data);
uint8_t I2C1_ReceiveData(void);
void I2C1_EnableAcknowledge(void);
void I2C1_DisableAcknowledge(void);
void I2C1_Stop(void);
void I2C1_WriteByte(uint8_t slaveAddress, uint8_t data);
uint8_t I2C1_ReadByte(uint8_t slaveAddress);
uint8_t I2C1_ReadRegister(uint8_t slaveAddress, uint8_t registerAddress);
void I2C1_WriteRegister(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data);

//SPI3 functions
void SPI3_GPIO_Init(void);
void SPI3_Transmit(uint8_t *transmit_data, uint32_t data_length);
void SPI3_Receive(uint8_t *receive_buffer, uint32_t receive_buffer_length);
void SPI3_CS_Enable(void);
void SPI3_CS_Disable(void);

//RA-08H-915 AT Commands with USART1
void sendATCommand(char *command);
char *receiveResponse(void);
void sendDataMessage(char *data);
uint8_t *receiveMessage(char *data);

//MAX-M10S GPIO settings
void M10S_GPIO_Init(void);
uint8_t M10S_LNA_EN(void);
void M10S_RST(void);
void M10S_ONOFF(uint8_t state);

//PWM Generations Functions
void tim1_pwm_init(void);
void tim2_pwm_init(void);
void tim3_pwm_init(void);
void start_tim1(void);
void start_tim2(void);
void start_tim3(void);
//Timer1 set duty-cycle functions
void set_tim1_ch3_duty_cycle(uint32_t duty);
void set_tim1_ch4_duty_cycle(uint32_t duty);
//Timer2 set duty-cycle functions
void set_tim2_ch3_duty_cycle(uint32_t duty);
void set_tim2_ch4_duty_cycle(uint32_t duty);
//Timer3 set duty-cycle functions
void set_tim3_ch1_duty_cycle(uint32_t duty);
void set_tim3_ch2_duty_cycle(uint32_t duty);


//Other GPIO Functions
void gpio_led_init(void);
void RA_02_GPIO_Init(void);
void RA_02_Reset(void);
void RA_02_Set(void);

void RA08_GPIO_Init(void);
void RA08_Reset(void);
void RA08_Boot(void);

//Timer4 functions
void TIM4_Init(void);
void TIM4_IRQHandler(void);
uint32_t GetTick(void);
