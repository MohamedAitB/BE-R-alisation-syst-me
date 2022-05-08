




#include <capteur.h>
#include "usart.h"
#include "main.h"
//#include "string.h"
#include "i2c.h"


void Initialisation(){
	MX_I2C1_Init();
}


void EnvoyezCommande(I2C_HandleTypeDef* I2Cx, uint16_t cmd){


	I2C_HandleTypeDef* Handle = I2Cx;

	uint8_t data[2];


	data[0] = cmd >>8;
	data[1] = cmd -data[0];


	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(Handle,SENSOR_ADDRESS,5,100);
	HAL_UART_Transmit(&huart2,(uint8_t *)data,2,10);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) newline, 2, 10);


	HAL_StatusTypeDef status2 = HAL_I2C_Master_Transmit(Handle, SENSOR_ADDRESS, data,2,5000);
	HAL_UART_Transmit(&huart2,(uint8_t *)status2,1,10);
	HAL_Delay(50);


	HAL_StatusTypeDef status3 = HAL_I2C_Master_Receive(Handle, SENSOR_ADDRESS, data,2,5000);
	HAL_UART_Transmit(&huart2,(uint8_t *)status3,1,10);
	HAL_Delay(50);
	printf(" ok \r\n");

	HAL_StatusTypeDef status4 = HAL_I2C_Master_Receive(Handle, SENSOR_ADDRESS, data,2,5000);
	HAL_UART_Transmit(&huart2,(uint8_t *)status4,1,10);
	HAL_Delay(50);



}

