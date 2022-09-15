#include "CAN_receive.h"
#include "Motor_M2006_Task.h"
#include "cmsis_os.h"

#include "main.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef  module_tx_message;
static uint8_t              module_can_send_data[8];

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

static motor_measure_t motor_M2006[4];



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
			case CAN_2006_M1_ID:
      case CAN_2006_M2_ID:
			case CAN_2006_M3_ID:
			case CAN_2006_M4_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_2006_M1_ID;
            get_motor_measure(&motor_M2006[i], rx_data);
            break;
        }
    }
		
}


void CAN_cmd_module(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    module_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    module_tx_message.IDE = CAN_ID_STD;
    module_tx_message.RTR = CAN_RTR_DATA;
    module_tx_message.DLC = 0x08;
    module_can_send_data[0] = motor1 >> 8;
    module_can_send_data[1] = motor1;
    module_can_send_data[2] = motor2 >> 8;
    module_can_send_data[3] = motor2;
    module_can_send_data[4] = motor3 >> 8;
    module_can_send_data[5] = motor3;
    module_can_send_data[6] = motor4 >> 8;
    module_can_send_data[7] = motor4;

	  HAL_CAN_AddTxMessage(&hcan2, &module_tx_message, module_can_send_data, &send_mail_box);
    //HAL_CAN_AddTxMessage(&hcan1, &module_tx_message, module_can_send_data, &send_mail_box);
	  
}


const motor_measure_t *get_module_motor_measure_point(uint8_t i)
{
    return &motor_M2006[(i & 0x03)];
}
