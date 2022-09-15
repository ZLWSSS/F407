#include "Motor_M2006_Task.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include "I2C_TASK.h"
#include "stdio.h"
#include "stdlib.h"
#include "user_lib.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern I2C_RX_COMMAND_t I2C_RX_DATA;
extern I2C_TX_COMMAND_t I2C_TX_DATA;

int16_t MOTOR_TASK_FLAG;
M2006_Control_t M2006_Control[4];
//mode number
static uint8_t mode_transition = 0;
static uint8_t tran_to_position = 0;

void M2006_Control_init()
{
	
	for(int i = 0;i<4;i++)
	{
	  M2006_Control[i].m2006_rc = get_remote_control_point();
		M2006_Control[i].m2006_motor_measure = get_module_motor_measure_point(i);
		pid_init(&(M2006_Control[i].m2006_speed_pid));
		//id,maxOutput,integralLimit,deadband,controlperiod,max_err,target,p,i,d
		M2006_Control[i].m2006_speed_pid.f_param_init(&M2006_Control[i].m2006_speed_pid, PID_Speed, M2006_SPEED_MAX_OUTPUT,5000,10,0,8000,0,M2006_SPEED_PID_KP, M2006_SPEED_PID_KI, M2006_SPEED_PID_KD);
		M2006_Control[i].ecd_offset = M2006_Control[i].m2006_motor_measure->ecd;
		M2006_Control[i].angle = M2006_Control[i].m2006_motor_measure->ecd* MOTOR_ECD_TO_ANGLE;
		M2006_Control[i].given_current = 0;
		M2006_Control[i].speed = 0.0f;
    M2006_Control[i].speed_set = 0.0f;
		M2006_Control[i].ecd_count = 0;
		M2006_Control[i].first_ecd = 1;
	}
}

void M2006_feedback_update(void)
{
	static fp32 speed_fliter_1[4];
  static fp32 speed_fliter_2[4];
  static fp32 speed_fliter_3[4];
	
	//memset(speed_fliter_1, 0, sizeof(speed_fliter_1));
	//memset(speed_fliter_2, 0, sizeof(speed_fliter_2));
	//memset(speed_fliter_3, 0, sizeof(speed_fliter_3));
	
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
	
	for(uint8_t i = 0; i < 4; i++)
	{
	  speed_fliter_1[i] = speed_fliter_2[i];
	  speed_fliter_2[i] = speed_fliter_3[i];
	  speed_fliter_3[i] = speed_fliter_2[i] * fliter_num[0] + speed_fliter_1[i] * fliter_num[1] + (M2006_Control[i].m2006_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
		//M2006_Control[i].speed = speed_fliter_3[i];
		
		if (M2006_Control[i].m2006_motor_measure->ecd - M2006_Control[i].m2006_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        M2006_Control[i].ecd_count--;
    }
    else if (M2006_Control[i].m2006_motor_measure->ecd - M2006_Control[i].m2006_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        M2006_Control[i].ecd_count++;
    }
		// 这里的速度系数可能需要修改
		M2006_Control[i].speed = speed_fliter_3[i] * RPM_FACTOR * RPM_TEST_FACTOR;
	}
}

void M2006_Compute_Current(int8_t permit2spin, int8_t ifspeed)
{
	if(ifspeed)
	{
			for(uint8_t i = 0; i < 4; i++)
			{
				if(permit2spin)
					M2006_Control[i].speed_set = (M2006_Control[i].m2006_rc->rc.ch[3]) * M2006_SPEED_MAX_OUTPUT / 660;
				//M2006_Control[i].speed_set = I2C_RX_DATA.Speed_Rpm[i];
				else
					M2006_Control[i].speed_set = 0;
				M2006_Control[i].m2006_speed_pid.target = M2006_Control[i].speed_set;
				M2006_Control[i].m2006_speed_pid.f_cal_pid(&(M2006_Control[i].m2006_speed_pid), M2006_Control[i].m2006_motor_measure->speed_rpm);
				M2006_Control[i].given_current = M2006_Control[i].m2006_speed_pid.output;
			}
	}
	else{
		for(uint8_t i = 0; i < 4; i++)
			{
				//计算输出轴角度
				if(mode_transition == 1)
				{
					M2006_Control[i].ecd_count = 0;
				}
				M2006_Control[i].angle = (M2006_Control[i].ecd_count * ECD_RANGE + M2006_Control[i].m2006_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
				M2006_Control[i].angle_set = M2006_Control[i].m2006_rc->rc.ch[4] * PI / 660.f;
				fp32 delta_angle = M2006_Control[i].angle_set - M2006_Control[i].angle;
				if(ABS(delta_angle) > 0.001f)
					M2006_Control[i].speed_set = delta_angle * 1200;
				else
					M2006_Control[i].speed_set= 0;
				
        M2006_Control[i].m2006_speed_pid.target = M2006_Control[i].speed_set;
				M2006_Control[i].m2006_speed_pid.f_cal_pid(&(M2006_Control[i].m2006_speed_pid), M2006_Control[i].m2006_motor_measure->speed_rpm);
				M2006_Control[i].given_current = M2006_Control[i].m2006_speed_pid.output;
			}
			mode_transition = 0;
		}
}

void motor_m2006_task(void const *pvParameters)
{
	osDelay(10);
	
	while(1)
	{
		M2006_feedback_update();
		//Speed Mode
		if(switch_is_down(M2006_Control[0].m2006_rc->rc.s[0]))
		{
			M2006_Compute_Current(0, 1);
			tran_to_position = 1;
		}
	  else if(M2006_Control[0].m2006_rc->rc.s[0] == 1)
		{
			M2006_Compute_Current(1, 1);
			tran_to_position = 1;
		}
		else if(M2006_Control[0].m2006_rc->rc.s[0] == 3)
		{
			if(tran_to_position == 1)
			{
				mode_transition = 1;
				tran_to_position = 0;
			}
			M2006_Compute_Current(1, 0);
		}
		
		CAN_cmd_module( M2006_Control[0].given_current, M2006_Control[1].given_current, M2006_Control[2]. given_current,M2006_Control[3].given_current );
	  osDelay(1);
	}
	
}

