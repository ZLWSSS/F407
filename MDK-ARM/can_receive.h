#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "Motor_M2006_Task.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */

extern void CAN_cmd_module(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const motor_measure_t *get_module_motor_measure_point(uint8_t i);



#endif
