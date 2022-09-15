#ifndef SERVO_TASK_H_
#define SERVO_TASK_H_

#include "SCS.h"
#include "INST.h"
#include "SMSBCL.h"
#include "struct_typedef.h"

extern void servo_task(void const *pvParameters);

typedef enum{

	Wheel_Position,
	Wheel_Speed
} Servo_Mode;

typedef enum{
	LOCK,
	MOVE,
	RESTORE
} Move_Type;

typedef struct _FeedBack_Servo{
  
	Servo_Mode Wheel_Mode;
	int Position_now;
	int Position_goal;
	int Position_start;
	int Position_last;
	int ID;
	uint8_t is_top_lock;
	
	void (*f_param_init)(struct _FeedBack_Servo* servo,
		                    int ID,
                        Servo_Mode mode,
	                      int position,
												uint8_t is_top_lock);
  void (*f_para_read)(struct _FeedBack_Servo* servo);
	                   
	
} FeedBack_SMBL40_Servo;

typedef struct _FeedBack_STS3032_Servo{
  
	Servo_Mode Wheel_Mode;
	int ID;
	uint8_t is_top_lock;
	
	void (*f_param_init)(struct _FeedBack_STS3032_Servo* servo,
		                    int ID,
                        Servo_Mode mode,
												uint8_t is_top_lock);
  void (*f_para_read)(struct _FeedBack_STS3032_Servo* servo);
	                   
} FeedBack_STS3032_Servo;

void Servo_Read_init(FeedBack_SMBL40_Servo* Servo_SMBL40_read, FeedBack_STS3032_Servo* Servo_STS3032_read );
void Servos_init(void);
Move_Type Figure_move(void);


#endif
