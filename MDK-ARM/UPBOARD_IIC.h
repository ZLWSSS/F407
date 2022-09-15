#ifndef UPBOARD_IIC_H_
#define UPBOARD_IIC_H_

#include "struct_typedef.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;

#define CHECKSUM_LENGTH_RX 8
#define CHECKSUM_LENGTH_TX 18
#define COMMAND_LENGTH_TX 76
#define COMMAND_LENGTH_RX 36

typedef struct{
  fp32 Speed_Rpm[4];
	int32_t Servo_Position[4];
	uint32_t Checksum;
} I2C_RX_COMMAND_t;

typedef struct{
  fp32 q[4];
	fp32 gyro[3];
	fp32 accel[3];
	fp32 Speed_Rpm[4];
	int32_t Servo_Position[4];
	uint32_t checksum;
} I2C_TX_COMMAND_t;

typedef struct{
  fp32 Speed_Rpm[4];
	int32_t Servo_Position[4];
} I2C_RX_DATA_t;

typedef struct{
  fp32 q[4];
	fp32 gyro[3];
	fp32 accel[3];
	fp32 Speed_Rpm[4];
	int32_t Servo_Position[4];
} I2C_TX_DATA_t;

void uint32touint8(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32);
void uint8touint32(uint32_t* vector_32, uint8_t* vector_8, uint16_t lengthof32);
uint32_t data_checksum(uint32_t* data_to_check, uint16_t length);
void I2C_COMMAND_2_DATA(I2C_RX_COMMAND_t* i2c_rx_command, I2C_RX_DATA_t* i2c_rx_data);
void I2C_DATA_2_COMMAND(I2C_TX_COMMAND_t* i2c_tx_command, I2C_TX_DATA_t* i2c_tx_data);

//gyro accel angle
void GET_I2C_TX_DATA(I2C_TX_DATA_t* i2c_tx_data);
extern void I2C_SET_INIT(void);


#endif
