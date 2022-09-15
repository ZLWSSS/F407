#include "UPBOARD_IIC.h"
#include "INS_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "Motor_M2006_Task.h"
#include "INS_TASK.h"
#include "stdio.h"
#include "stdlib.h"
#include "servo_task.h"

extern M2006_Control_t M2006_Control[4];
const fp32* local_INS_gyro;
const fp32* local_INS_accel;
const fp32* local_INS_quat;
extern FeedBack_SMBL40_Servo Servo_SM40BL[4];

void I2C_SET_INIT()
{
	local_INS_gyro = get_gyro_data_point();
	local_INS_accel = get_accel_data_point();
	local_INS_quat = get_INS_quat_point();
}

void uint32touint8(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32)
{
	for(int i = 0; i < lengthof32; i++)
	{
		vector_8[4*i] = vector_32[i];
		vector_8[4*i + 1] = vector_32[i] >> 8;
		vector_8[4*i + 2] = vector_32[i] >> 16;
		vector_8[4*i + 3] = vector_32[i] >> 24;
	}
}

void uint8touint32(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32)
{
	for(int i = 0; i < lengthof32; i++)
	vector_32[i] = (vector_8[4*i + 3] << 24) + (vector_8[4*i + 2] << 16) + 
	               (vector_8[4*i + 1] << 8) + vector_8[4*i];
}

uint32_t data_checksum(uint32_t* data_to_check, uint16_t length)
{
	uint32_t t = 0;
	for(int i = 0; i < length; i++)
	{
		t = t^data_to_check[i];
	}
	return t;
}

void I2C_COMMAND_2_DATA(I2C_RX_COMMAND_t* i2c_rx_command, I2C_RX_DATA_t* i2c_rx_data)
{
	
	I2C_RX_DATA_t* i2c_rx_data_temp;
	i2c_rx_data_temp = (I2C_RX_DATA_t*) malloc (sizeof(I2C_RX_DATA_t));
	
	for(int i = 0; i < 4; i++)
	{
		i2c_rx_data_temp->Servo_Position[i] = i2c_rx_data->Servo_Position[i];
		i2c_rx_data_temp->Speed_Rpm[i] = i2c_rx_data->Speed_Rpm[i];
	}
	
	for(int i = 0; i < 4; i++)
	{
		i2c_rx_data->Speed_Rpm[i] = i2c_rx_command->Speed_Rpm[i];
		i2c_rx_data->Servo_Position[i] = i2c_rx_command->Servo_Position[i];
	}
	uint32_t checksum_temp = data_checksum((uint32_t*) i2c_rx_command, CHECKSUM_LENGTH_RX);
	
	if(checksum_temp != i2c_rx_command->Checksum)
	{
		for(int i = 0; i < 4; i++)
	  {
		  i2c_rx_data->Servo_Position[i] = i2c_rx_data_temp->Servo_Position[i];
		  i2c_rx_data->Speed_Rpm[i] = i2c_rx_data_temp->Speed_Rpm[i];
	  }
	}
	
	free(i2c_rx_data_temp);
		//to do something
}


void I2C_DATA_2_COMMAND(I2C_TX_COMMAND_t* i2c_tx_command, I2C_TX_DATA_t* i2c_tx_data)
{
	for(int i = 0; i < 3; i ++)
	{
		i2c_tx_command->accel[i] = i2c_tx_data->accel[i];
		i2c_tx_command->q[i] = i2c_tx_data->q[i];
		i2c_tx_command->gyro[i] = i2c_tx_data->gyro[i];
		i2c_tx_command->Speed_Rpm[i] = i2c_tx_data->Speed_Rpm[i];
		i2c_tx_command->Servo_Position[i] = i2c_tx_data->Servo_Position[i];
	}
	i2c_tx_command->q[3] = i2c_tx_data->q[3];
	i2c_tx_command->Speed_Rpm[3] = i2c_tx_data->Speed_Rpm[3];
	i2c_tx_command->Servo_Position[3] = i2c_tx_data->Servo_Position[3];
	
	i2c_tx_command->checksum = data_checksum((uint32_t*)i2c_tx_data, CHECKSUM_LENGTH_TX);
}

void GET_I2C_TX_DATA(I2C_TX_DATA_t* i2c_tx_data)
{
	for(int i = 0;i < 3;i++)
	{
		i2c_tx_data->gyro[i] = local_INS_gyro[i];
		i2c_tx_data->accel[i] = local_INS_accel[i];
		
	}
	for(int i = 0;i < 4;i++)
	{
		if(Servo_SM40BL[i].is_top_lock)
		{
		  i2c_tx_data->Servo_Position[i] = 1;
		}
		else
			i2c_tx_data->Servo_Position[i] = 0;
		i2c_tx_data->Speed_Rpm[i] =  M2006_Control[i].speed;
		i2c_tx_data->q[i] = local_INS_quat[i];
	}
	
}

