#ifndef MOTOR_M2006_H_
#define MOTOR_M2006_H_

#include "remote_control.h"
#include "pid.h"

#define M2006_SPEED_MAX_OUTPUT 16384
#define M2006_SPEED_PID_KP 5.0f
#define M2006_SPEED_PID_KI 0.05f
#define M2006_SPEED_PID_KD 0.0f

#define M2006_READY_PID_MAX_OUT   10000.0f
#define M2006_READY_PID_MAX_IOUT  9000.0f

#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.0000213052887206339059683067720762f
#define PI                          3.141592653589793238462643383279502f


//电机码盘中值与最大值
#define HALF_ECD_RANGE  7000
#define ECD_RANGE       8191
#define FULL_COUNT      18
#define RPM_FACTOR      416.0f / 60.0f
#define M2006_Angle_Speed 100.f

//做实验：在speed_Set为20 * 20300 / 660的情况下，设定值为12.5rpm，实际转了17.5rpm
#define RPM_TEST_FACTOR 17.5f / 12.5f 

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
	
	//控制电流[-10000,10000]
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	
} motor_measure_t;


typedef struct{
	const RC_ctrl_t* m2006_rc;
	const motor_measure_t* m2006_motor_measure;
	//Speed PID
	PID_TypeDef_M2006 m2006_speed_pid;
	fp32 speed;
	fp32 speed_set;
	fp32 angle;
	fp32 angle_set;
	uint16_t ecd_offset;
	int16_t given_current;
	int32_t ecd_count;
	uint8_t first_ecd;
} M2006_Control_t;

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_2006_M1_ID = 0x201,
    CAN_2006_M2_ID = 0x202,
    CAN_2006_M3_ID = 0x203,
    CAN_2006_M4_ID = 0x204,
} can_msg_id_e;

extern void CAN_cmd_module(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const motor_measure_t *get_module_motor_measure_point(uint8_t i);

void M2006_Control_init(void);
void M2006_feedback_update(void);
void M2006_Compute_Current(int8_t permit2spin, int8_t ifspeed);

extern void motor_m2006_task(void const *pvParameters);

#endif
