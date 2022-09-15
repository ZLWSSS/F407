#include "I2C_TASK.h"
#include "cmsis_os.h"
#include "bsp_i2c.h"
#include "Motor_M2006_Task.h"
#include "servo_task.h"
#include "string.h"
#include "UPBOARD_IIC.h"
#include "calibrate_task.h"

uint8_t cali = 0;

I2C_RX_DATA_t I2C_RX_DATA;
I2C_TX_DATA_t I2C_TX_DATA;
I2C_RX_COMMAND_t I2C_RX_COMMAND;
I2C_TX_COMMAND_t I2C_TX_COMMAND;

uint8_t RxBuffer[COMMAND_LENGTH_RX];
uint8_t TxBuffer[COMMAND_LENGTH_TX];
int16_t I2C_TASK_RX_FLAG;
int32_t I2C_TASK_TX_FLAG;


void i2c_task(void const *pvParameters)
{
	
	I2C_INIT();
	//I2C2_tx_DMA_init();
	
	osDelay(1000);
	

	while(1)
	{
	    //HAL_I2C_Slave_Receive(&hi2c2, (uint8_t*)RxBuffer, COMMAND_LENGTH_RX, 10);
		  HAL_I2C_Slave_Receive_DMA(&hi2c2, (uint8_t*)RxBuffer, COMMAND_LENGTH_RX);
		  while(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_BUSY_RX  )
			{
				I2C_TASK_RX_FLAG++;
				if(cali == 0)
				{
					calibrate_task();
					cali = 1;
				}
				
			}
			I2C_TASK_RX_FLAG = 0;
		  //HAL_I2C_Slave_Transmit(&hi2c2, (uint8_t*)TxBuffer, COMMAND_LENGTH_TX, 10);
			GET_I2C_TX_DATA(&I2C_TX_DATA);
	    I2C_DATA_2_COMMAND(&I2C_TX_COMMAND, &I2C_TX_DATA);
	    uint32touint8((uint32_t*) &I2C_TX_COMMAND, (uint8_t*)TxBuffer, COMMAND_LENGTH_TX / 4);
		  HAL_I2C_Slave_Transmit_DMA(&hi2c2, (uint8_t*)TxBuffer, COMMAND_LENGTH_TX);
			while(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_BUSY_TX  )
			{
				I2C_TASK_TX_FLAG++;
		  }
			I2C_TASK_TX_FLAG = 0;
			osDelay(1);
			///uint8touint32((uint32_t*) &I2C_RX_COMMAND,(uint8_t*)RxBuffer, COMMAND_LENGTH_RX / 4 );
	    //I2C_COMMAND_2_DATA(&I2C_RX_COMMAND, &I2C_RX_DATA);
	}
}

void I2C_INIT()
{
	memset(&I2C_TX_COMMAND, 0, sizeof(I2C_TX_COMMAND_t));
	memset(&I2C_TX_DATA, 0, sizeof(I2C_TX_DATA_t));
	memset(&I2C_RX_COMMAND, 0, sizeof(I2C_RX_COMMAND_t));
	memset(&I2C_RX_DATA, 0, sizeof(I2C_RX_DATA_t));
	memset(RxBuffer, 0, COMMAND_LENGTH_RX);
	memset(TxBuffer, 0, COMMAND_LENGTH_TX);
	
	I2C_SET_INIT();
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* I2CHandle)
{
	uint8touint32((uint32_t*) &I2C_RX_COMMAND,(uint8_t*)RxBuffer, COMMAND_LENGTH_RX / 4 );
	I2C_COMMAND_2_DATA(&I2C_RX_COMMAND, &I2C_RX_DATA);
}

//TO DO: reset i2c when error occurs


