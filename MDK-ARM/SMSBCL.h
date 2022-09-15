#ifndef _SMSBCL_H
#define _SMSBCL_H

#include <stdint.h>

#define	SMSBCL_1M 0
#define	SMSBCL_0_5M 1
#define	SMSBCL_250K 2
#define	SMSBCL_128K 3
#define	SMSBCL_115200 4
#define	SMSBCL_76800	5
#define	SMSBCL_57600	6
#define	SMSBCL_38400	7

//?????
//-------EPROM(??)--------
#define SMSBCL_MODEL_L 3
#define SMSBCL_MODEL_H 4

//-------EPROM(??)--------
#define SMSBCL_ID 5
#define SMSBCL_BAUD_RATE 6
#define SMSBCL_MIN_ANGLE_LIMIT_L 9
#define SMSBCL_MIN_ANGLE_LIMIT_H 10
#define SMSBCL_MAX_ANGLE_LIMIT_L 11
#define SMSBCL_MAX_ANGLE_LIMIT_H 12
#define SMSBCL_CW_DEAD 26
#define SMSBCL_CCW_DEAD 27
#define SMSBCL_OFS_L 31
#define SMSBCL_OFS_H 32
#define SMSBCL_MODE 33

//-------SRAM(??)--------
#define SMSBCL_TORQUE_ENABLE 40
#define SMSBCL_ACC 41
#define SMSBCL_GOAL_POSITION_L 42
#define SMSBCL_GOAL_POSITION_H 43
#define SMSBCL_GOAL_TIME_L 44
#define SMSBCL_GOAL_TIME_H 45
#define SMSBCL_GOAL_SPEED_L 46
#define SMSBCL_GOAL_SPEED_H 47
#define SMSBCL_LOCK 55

//-------SRAM(??)--------
#define SMSBCL_PRESENT_POSITION_L 56
#define SMSBCL_PRESENT_POSITION_H 57
#define SMSBCL_PRESENT_SPEED_L 58
#define SMSBCL_PRESENT_SPEED_H 59
#define SMSBCL_PRESENT_LOAD_L 60
#define SMSBCL_PRESENT_LOAD_H 61
#define SMSBCL_PRESENT_VOLTAGE 62
#define SMSBCL_PRESENT_TEMPERATURE 63
#define SMSBCL_MOVING 66
#define SMSBCL_PRESENT_CURRENT_L 69
#define SMSBCL_PRESENT_CURRENT_H 70


extern int WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);//普通写位置
extern int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC);//异步写位置
extern void RegWriteAction(void);//异步写位置执行
extern void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]);//同步写位置执行
extern int WheelMode(uint8_t ID);
extern int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC);
extern int EnableTorque(uint8_t ID, uint8_t Enable);//
extern int unLockEprom(uint8_t ID);//eprom
extern int LockEprom(uint8_t ID);//eprom
extern int CalibrationOfs(uint8_t ID);//
extern int FeedBack(int ID);//
extern int ReadPos(int ID);//
extern int ReadSpeed(int ID);//
extern int ReadLoad(int ID);//
extern int ReadVoltage(int ID);//
extern int ReadTemper(int ID);//
extern int ReadMove(int ID);//
extern int ReadCurrent(int ID);//
extern int getErr(void);//

#endif
