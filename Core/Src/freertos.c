/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usart.h"
#include "gpio.h"
#include "gps.h"

#include "uartRingBuffer.h"
#include "NMEA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define uart &huart1

#define TIMEOUT_DEF 500  // 500ms timeout for the functions
uint16_t timeout;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t rx_data = 0;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;

GPSSTRUCT gpsData;

int flagGGA = 0, flagRMC = 0;
char nodeMCUBuffer [30];

int gpsEnabled = 0;

int VCCTimeout = 5000; // GGA or RMC will not be received if the VCC is not sufficient
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

char* int_to_string(int num) {

    char* str = malloc(sizeof(char)*4);

    sprintf(str, "%d\n\r", num);

    return str;

}

char* float_to_string(float num) {

    char* str = malloc(sizeof(char)*4);

    sprintf(str, "%f\n\r", num);

    return str;

}

void sendString(char * str, int uart_number){
	if(uart_number == 1)
		HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str) ,HAL_MAX_DELAY );
	else if(uart_number == 2)
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str) ,HAL_MAX_DELAY );
	else if(uart_number == 3)
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str) ,HAL_MAX_DELAY );
}

void sendChar(unsigned char c, int uart_number){
	unsigned char pChar = c;
	if(uart_number == 1)
		HAL_UART_Transmit(&huart1, &pChar,1 ,HAL_MAX_DELAY );
	else if(uart_number == 2)
		HAL_UART_Transmit(&huart2, &pChar, 1,HAL_MAX_DELAY );
	else if(uart_number == 3)
		HAL_UART_Transmit(&huart3, &pChar, 1,HAL_MAX_DELAY );
}

char readChar(int uart_number){
	unsigned char caracter;

	if(uart_number == 1){
		while(HAL_UART_Receive(&huart1, &caracter, 1, HAL_MAX_DELAY) != HAL_OK);
	}
	else if(uart_number == 2){
		while(HAL_UART_Receive(&huart2, &caracter, 1, HAL_MAX_DELAY) != HAL_OK);
	}
	else if(uart_number == 3){
		while(HAL_UART_Receive(&huart3, &caracter, 1, HAL_MAX_DELAY) != HAL_OK);
	}
	return caracter;
}

void test_task( void *vParam){

	char *places[4] = {"-20.7000000,-45.3666667,\n", "-5.9500000,-43.0000000,\n",
			"-17.4333333,-44.8500000,\n", "3.7500000,-61.3500000,\n"};

	while(1){

		for (int i = 0; i < 4; i++) {

			if (gpsEnabled == 1) {

				sendString(places[i], 3);

			}

			vTaskDelay(3000);

		}

	}

}

void recebe_da_node_mcu(void *vParam){
	unsigned char caracter;
	vTaskDelay(100);
	while(1){
		//Aguarda receber um caracter da UART3 que esta conectada ao Node MCU
		while(HAL_UART_Receive(&huart3, &caracter, 1, HAL_MAX_DELAY) != HAL_OK);

		//Envia o caracter recebido para a UART 2 que está conectada ao terminal serial
		sendChar(caracter, 2);
		vTaskDelay(1);
	}
}

void envia_para_node_mcu( void *vParam){
	unsigned char caracter;

	while(1){
		//Aguarda receber um carcater do terminal serial
		caracter = readChar(2);
		//Envia o caracter para a UART 3 que está conectada a placa NodeMCU
		sendChar(caracter, 3);
		vTaskDelay(1);
	}
}


int GPS_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;

    i=0;
    calculated_check=0;

    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;

    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }

    if(i >= 75){
        return 0;// the string was too long so return an error
    }

    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found there for invalid

    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
}

float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}

void GPS_parse(char *GPSstrParse){
    if(!strncmp(GPSstrParse, "$GPGGA", 6)){
//    	sendString("$GPGGA",2);
		VCCTimeout = 5000;  // Reset the VCC Timeout indicating the GGA is being received

//		Copy_upto("*", GGA);
		if (decodeGGA(GPSstrParse, &gpsData.ggastruct) == 0) flagGGA = 2;  // 2 indicates the data is valid
		else flagGGA = 1;  // 1 indicates the data is invalid
    }
    else if (!strncmp(GPSstrParse, "$GPRMC", 6)){
//    	sendString("$GPRMC",2);
		VCCTimeout = 5000;  // Reset the VCC Timeout indicating the RMC is being received

//		Copy_upto("*", RMC);
		if (decodeRMC(GPSstrParse, &gpsData.rmcstruct) == 0) flagRMC = 2;  // 2 indicates the data is valid
		else flagRMC = 1;  // 1 indicates the data is invalid
    }
}

void cli(void * vParam)
{
	while(1)
	{
//		0xFFFFFFFFU
//		sendChar('f', 2);
		while(HAL_UART_Receive(&huart1, &rx_data, 1, HAL_MAX_DELAY) != HAL_OK);
//		sendChar(rx_data, 2);
		if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_data;
		} else {

			#if (GPS_DEBUG == 1)
			GPS_print((char*)rx_buffer);
			#endif

			if(GPS_validate((char*) rx_buffer)) {
				GPS_parse((char*) rx_buffer);
			}

			rx_index = 0;
			memset(rx_buffer, 0, sizeof(rx_buffer));
		}

		if ((flagGGA == 2) | (flagRMC == 2))
		{
			if (gpsEnabled == 1) {
				sprintf(nodeMCUBuffer,"%f,%f,\n", gpsData.ggastruct.lcation.latitude, gpsData.ggastruct.lcation.longitude);
				sendString(nodeMCUBuffer, 3);
			}

		}

		else if ((flagGGA == 1) | (flagRMC == 1))
		{
		  // Instead of clearing the display, it's better if we print spaces.
		  // This will avoid the "refreshing" part
		  sendString("   NO FIX YET   \n\r", 2);
		  sendString("   Please wait  \n\r", 2);
		}

		if (VCCTimeout <= 0)
		{
		  VCCTimeout = 5000;  // Reset the timeout

		  //reset flags
		  flagGGA =flagRMC =0;

		  // You are here means the VCC is less, or maybe there is some connection issue
		  // Check the VCC, also you can try connecting to the external 5V
		  sendString("    VCC Issue   \n\r",2);
		  sendString("Check Connection\n\r",2);
		}

//		vTaskDelay(500);
	}
}

void gpsEnabled_task(void * vParam)
{
	unsigned char flag;
	vTaskDelay(100);
	while(1){
		//Aguarda receber um caracter da UART3 que esta conectada ao Node MCU
		while(HAL_UART_Receive(&huart3, &flag, 1, HAL_MAX_DELAY) != HAL_OK);

		if (flag == '1') {
			gpsEnabled = 1;
			sendChar('v', 2);
		} else {
			gpsEnabled = 0;
			sendChar('f', 2);
		}

		vTaskDelay(1);
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//  xTaskCreate(cli,
//		  "serialRcvr",
//		  configMINIMAL_STACK_SIZE,
//		  NULL,
//		  1,
//		  NULL);

//  xTaskCreate(recebe_da_node_mcu, "recebe_da_node_mcu",
//		  configMINIMAL_STACK_SIZE,
//		  NULL, 1,
//		  NULL);
//
//  xTaskCreate(envia_para_node_mcu, "envia_para_node_mcu",
//		  configMINIMAL_STACK_SIZE,
//		  NULL, 1,
//		  NULL);

  xTaskCreate(test_task, "test_task",
		  configMINIMAL_STACK_SIZE,
		  NULL, 1,
		  NULL);

  xTaskCreate(gpsEnabled_task, "gpsEnabled_task",
		  configMINIMAL_STACK_SIZE,
		  NULL, 1,
		  NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
