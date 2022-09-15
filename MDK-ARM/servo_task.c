#include "servo_task.h"
#include "main.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "I2C_Task.h"
#include "string.h"
#include "UPBOARD_IIC.h"

#define CONTROL_SERVO_BY_RC 1
#define SERVO_SMBL40_PLACEMENT 12035

const RC_ctrl_t* servo_rc;
FeedBack_SMBL40_Servo Servo_SM40BL[4];
FeedBack_STS3032_Servo Servo_STS3032[4];
extern I2C_RX_COMMAND_t I2C_RX_COMMAND;
Move_Type COMMAND_MOVE;

static void parameter_init_SMB40L(FeedBack_SMBL40_Servo* servo,
	                      int ID,
                        Servo_Mode mode,
	                      int position,
												uint8_t is_top_lock
											  )
{
	servo->ID = ID;
	servo->Wheel_Mode = mode;
	servo->Position_start = position;
	servo->Position_goal = position;
	servo->Position_now = servo->Position_start;
	servo->Position_last = position;
	servo->is_top_lock = is_top_lock;
}

static void parameter_init_STS3032(FeedBack_STS3032_Servo* servo,
	                      int ID,
                        Servo_Mode mode,
												uint8_t is_top_lock
											  )
{
	servo->ID = ID;
	servo->Wheel_Mode = mode;
	servo->is_top_lock = is_top_lock;
}

static void parameter_read(FeedBack_SMBL40_Servo* servo)
{
	servo->Position_now = ReadPos(servo->ID);
}

void Servo_Read_init(FeedBack_SMBL40_Servo* Servo_SMBL40_read, FeedBack_STS3032_Servo* Servo_STS3032_read)
{
	Servo_SMBL40_read->f_param_init = parameter_init_SMB40L;
	Servo_SMBL40_read->f_para_read = parameter_read;
	
	Servo_STS3032_read->f_param_init = parameter_init_STS3032;
}

void Servos_init()
{
	for(int i = 0; i < 4; i++)
	{
	  Servo_Read_init(&(Servo_SM40BL[i]), &(Servo_STS3032[i]));
		
		// to change the ID representation
	  Servo_SM40BL[i].f_param_init(&(Servo_SM40BL[i]),i+1,Wheel_Position,0,1);
	  Servo_STS3032[i].f_param_init(&(Servo_STS3032[i]),i+5 ,Wheel_Position,0);
	}
}

void servo_task(void const *pvParameters)
{
	servo_rc = get_remote_control_point();
	
	for(int i = 0; i < 4; i++)
	{
		WritePosEx(Servo_STS3032[i].ID,0,5000,100);
		
		Servo_SM40BL[i].f_para_read(&(Servo_SM40BL[i]));		
		Servo_SM40BL[i].Position_last = Servo_SM40BL[i].Position_now;
		Servo_SM40BL[i].Position_start = Servo_SM40BL[i].Position_now;
		Servo_SM40BL[i].Position_goal = Servo_SM40BL[i].Position_now + SERVO_SMBL40_PLACEMENT;
	}


	while(1)
	{
		if(CONTROL_SERVO_BY_RC)
		{
			// 这里没有办法看有没有到目的位置，只有对比前后位置是否相同来判断是否到达目标位置。
			// to do: change RC command to Upboard Command 0/1
			switch(servo_rc->rc.s[0])
			{
				case 1: //lock
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_SM40BL[i].ID, Servo_SM40BL[i].Position_start + SERVO_SMBL40_PLACEMENT, 50, 0);
					}
					osDelay(3000);
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_STS3032[i].ID,2048,5000,100);
						Servo_STS3032[i].is_top_lock = 1;
					}
					break;
				case 2: // restore
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_STS3032[i].ID, 0,5000,100);
						Servo_STS3032[i].is_top_lock = 0;
					}
					osDelay(2000);
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_SM40BL[i].ID, Servo_SM40BL[i].Position_start, 50, 0);
					}
					break;
				default:
					break;

			}
				osDelay(10);
		}
	else
		{
			COMMAND_MOVE = Figure_move();
			switch(COMMAND_MOVE)
			{
				case LOCK:
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_SM40BL[i].ID, Servo_SM40BL[i].Position_start + SERVO_SMBL40_PLACEMENT, 50, 0);
					}
					osDelay(3000);
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_STS3032[i].ID,2048,5000,100);
						Servo_STS3032[i].is_top_lock = 1;
					}
					break;
				case RESTORE:
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_STS3032[i].ID, 0,5000,100);
						Servo_STS3032[i].is_top_lock = 0;
					}
					osDelay(2000);
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_SM40BL[i].ID, Servo_SM40BL[i].Position_start, 50, 0);
					}
					break;
				case MOVE:
					if((Servo_STS3032[0].is_top_lock + Servo_STS3032[1].is_top_lock + Servo_STS3032[2].is_top_lock + Servo_STS3032[3].is_top_lock) != 0)
					{
						for(int i = 0; i < 4; i++)
						{
							WritePosEx(Servo_STS3032[i].ID,0,5000,100);
							Servo_STS3032[i].is_top_lock = 0;
						}
						osDelay(2000);
					}
					for(int i = 0; i < 4; i++)
					{
						WritePosEx(Servo_SM40BL[i].ID, Servo_SM40BL[i].Position_start + I2C_RX_COMMAND.Servo_Position[i], 50, 0);
					}
					break;
				default:
					break;
			}
		}
		
		
	}
}

Move_Type Figure_move(void)
{
	int32_t Command_sum;
	for(int i = 0; i < 4; i++)
	{
		Command_sum += I2C_RX_COMMAND.Servo_Position[i];
	}
	if(Command_sum == 4 * 12035)
		return LOCK;
	else if (Command_sum == 0)
		return RESTORE;
	else
		return MOVE;
}
