/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * In this program, an attempt was made to write a tracking radar  
  * system that would track aircraft remotely.
  * 
  *In this program, an attempt was made to write a tracking 
	*radar system that would track aircraft remotely.

	*This demo program will be updated in the future if the system is implemented. 
	*The missing parts can be summarized as follows:
	*-->Calculating the angle from the angular encoder with the arrival of the RSSI value
	*-->Calculating the engine control algorithm with the angle calculation

	*This program was created as a preparation code until the system was created and concretized.
  * 
  * 
  *                        
  *
  ******************************************************************************
  */



#include "Req.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "as5048b.h"
#include "LoRa.h"
#include "M-GPS.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);
//ICs declaretion
AS5048B_HandleTypeDef as5048b; //Angular encoder
//LoRa RA02 433MHz variable
LoRa RA02;
int RA02_RSSI;
//GPS Latitude, Longitude variable
double gps_lat;
double gps_lon;

//Task Handle
TaskHandle_t GPSTaskHandle;
TaskHandle_t EncoderTaskHandle;
TaskHandle_t RA02TaskHandle;
TaskHandle_t RA08TaskHandle;
TaskHandle_t MotorTaskHandle;
//Create counting semaphore
SemaphoreHandle_t xCountingSemap;

//LoRa RA-02 GPIO Init functions
void RA_02_Init(void);

//Task Functions
static void StartRA02Task(void *parameters);
static void StartRA08Task(void *parameters);
static void StartGPSTask(void *parameters);
static void StartEncoderTask(void *parameters);
static void StartMotorsTask(void *parameters);


int main(void){
	
	SysClockConfig();
	//first, Peripheral gpio inits
	I2C1_gpio_init();
	gpio_usart3_init();
	gpio_usart2_init();
	gpio_usart1_init();
	SPI3_GPIO_Init();
	TIM4_Init();
	tim1_pwm_init();
	tim2_pwm_init();
	tim3_pwm_init();
	
	//Second, ICs init
	AS5048B_Init(&as5048b,0x43); //I2C slave address is 0x43 (if fails try 0x44)
	//RA-02 Init
	RA_02_Init();
	
	//GPS Init
	M_GPS_init();
	M_GPS_bufInit();
	
	
	BaseType_t status;
	
	//FreeRTOS task declare
	status = xTaskCreate(StartRA02Task,"RA02",256,NULL,2,&RA02TaskHandle); //Task priority might change
	
	configASSERT(status == pdPASS);
	
	status = xTaskCreate(StartRA08Task,"RA08H",256,NULL,2,&RA08TaskHandle);
	
	configASSERT(status == pdPASS);
	
	status = xTaskCreate(StartEncoderTask,"ENC",256,NULL,4,&EncoderTaskHandle);
	
	configASSERT(status == pdPASS);
	
	status = xTaskCreate(StartGPSTask,"GPS",256,NULL,1,&GPSTaskHandle); //GPS task priority is less than others
	
	configASSERT(status == pdPASS);
	
	status = xTaskCreate(StartMotorsTask,"MTR",256,NULL,5,&MotorTaskHandle);
	
	configASSERT(status == pdPASS);
	
	xCountingSemap = xSemaphoreCreateCounting(7,0); //Maximum count might change
	
	if(xCountingSemap != NULL){
		//The semaphore was created successfully.
	}
	
	
	
	while(1){
	
	
	}

	return 0;

}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName){

	usart2_send_string("Stack overflow detected in task: ");
	usart2_send_string(pcTaskName);
	usart2_send_string("\n");
	
	while(1){
	
	
	}

}

void vApplicationMallocFailedHook(void)
{
    __BKPT(0); // Debugger burada duracak
    while (1);
}

void RA_02_Init(void){
	
	// MODULE SETTINGS ----------------------------------------------
	RA02 = newLoRa();
	
	RA02.CS_port = GPIOD;
	RA02.CS_pin = 0;
	RA02.reset_port = GPIOD;
	RA02.reset_pin = 2;
	RA02.DIO0_port = GPIOD;
	RA02.DIO0_pin = 1;
	
	RA02.frequency = 433;
	RA02.spredingFactor = SF_7;
	RA02.bandWidth = BW_125KHz;
	RA02.crcRate = CR_4_5;
	RA02.power = POWER_20db;
	RA02.overCurrentProtection = 120;
	RA02.preamble = 10;
	
	LoRa_reset();
	LoRa_init(&RA02);
	
	// START CONTINUOUS RECEIVING -----------------------------------
	LoRa_startReceiving(&RA02);
	
}

//FreeRTOS Tasks
static void StartRA02Task(void *parameters){
	
	u_ra02_transmit_message[0] = 0x3B;	//ADDRESS
	
	
	for(;;){
	
		//@TODO LoRa RA02 433MHz transmit and receive message
		
	}

}


static void StartRA08Task(void *parameters){

	for(;;){
		
		//@TODO LoRa RA-08H 915 MHz transmit and receive message
	
	}

}


static void StartGPSTask(void *parameters){


	for(;;){
		//Receive latitude and longitude values
		gps_lat = M_GPS_getLatitude();
		gps_lon = M_GPS_getLongitude();
	}

}



static void StartEncoderTask(void *parameters){

	for(;;){
		//@TODO Indicate calculation angle from the RSSI value of LoRa (not decided which LoRa)
		//@TODO Notify task motor with angle 
	}

}


static void StartMotorsTask(void *parameters){


	for(;;){
		//@TODO Set pwm according to calculation of angle from angular encoder
	}

}

