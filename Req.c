#include "Req.h"


volatile uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_data_ready_flag;
volatile uint16_t rx_data_length;
uint8_t responseBuffer[RESPONSE_BUFFER_SIZE];
uint8_t receiveBuffer[RESPONSE_BUFFER_SIZE];
uint8_t u_ra02_receive_message[RA02_MESSAGE_SIZE];
uint8_t u_ra02_transmit_message[RA02_MESSAGE_SIZE];

