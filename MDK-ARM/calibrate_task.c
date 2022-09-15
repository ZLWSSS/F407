#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"

#include "bsp_adc.h"
#include "bsp_flash.h"
#include "bsp_buzzer.h"

#include "can_receive.h"
#include "remote_control.h"
#include "INS_task.h"
#include "main.h"
#include "bsp_delay.h"

#define FLASH_WRITE_BUF_LENGTH (sizeof(head_cali_t)+sizeof(imu_cali_t) * 3 + CALI_LIST_LENGTH * 4) // name and is_cali_Done
#define imu_start_buzzer()     buzzer_on(31, 19999)
#define imu_cali_done()        buzzer_calidone(15, 19999)
#define cali_done_buzzer()     buzzer_off()

extern TIM_HandleTypeDef htim4;

void RC_cmd_to_calibrate(void);
void cali_data_read(void);
void cali_data_write(void);

//矫正函数
bool_t cali_gyro_hook(uint32_t* cali, bool_t cmd);
bool_t cali_head_hook(uint32_t* cali, bool_t cmd);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

const RC_ctrl_t *calibrate_RC; 

head_cali_t head_cali;
static imu_cali_t  accel_cali;      //accel cali data
static imu_cali_t  gyro_cali;       //gyro cali data
static imu_cali_t  mag_cali;        //mag cali data

uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGTH];

cali_sensor_t cali_sensor[CALI_LIST_LENGTH];

const uint8_t cali_name[CALI_LIST_LENGTH][3] = {"HD","GYR", "ACC", "MAG"};

uint32_t *cali_sensor_buf[CALI_LIST_LENGTH] = {(uint32_t*)&head_cali, (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali, (uint32_t *)&mag_cali};

uint8_t cali_sensor_size[CALI_LIST_LENGTH] = {sizeof(head_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

void *cali_hook_fun[CALI_LIST_LENGTH] = {cali_head_hook, cali_gyro_hook, NULL, NULL};

uint32_t calibrate_systemTick;

uint16_t rc_cmd_time;
const uint8_t BEGIN_FLAG   = 1;
const uint8_t GYRO_FLAG    = 2;

uint32_t rc_cmd_systemTick = 0;
uint8_t  rc_action_flag    = 0;
uint8_t Already_start = 0;
uint32_t cali_flag = 0;
osThreadId local_Thread_Id;

void calibrate_task(void const *pvParameters)
{
  static uint8_t i = 0;
	local_Thread_Id = osThreadGetId();
	osDelay(1000);
  while (1)
  {
    RC_cmd_to_calibrate();
		cali_flag++;
    for (i = 0; i < CALI_LIST_LENGTH; i++)
    {
			//如果需要校准
      if (cali_sensor[i].cali_cmd)
      {
			// 如果存在函数指针 则执行
				if (cali_sensor[i].cali_hook != NULL)
        {

          if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
          {
               //done
						cali_sensor[i].name[0] = cali_name[i][0];
						cali_sensor[i].name[1] = cali_name[i][1];
						cali_sensor[i].name[2] = cali_name[i][2];
						//set 0x55
						cali_sensor[i].cali_done = CALIED_FLAG;
						cali_sensor[i].cali_cmd = 0;
                        //write
						cali_data_write();
						
						//结束任务
						osThreadTerminate(local_Thread_Id);
			      osThreadYield ();
					}
				}
			}
		}
		
    osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

static void RC_cmd_to_calibrate(void)
{
	  static uint8_t  i;
    //if something is calibrating, return
    //如果已经在校准，就返回
    for (i = 0; i < CALI_LIST_LENGTH; i++)
    {
        if (cali_sensor[i].cali_cmd)
        {
            rc_cmd_time = 0;
            rc_action_flag = 0;
            return;
        }
    }
	  rc_cmd_time++;
		//启动校准
		//听到高频之后转换
    if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
      rc_cmd_systemTick = HAL_GetTick();
			Already_start = 1;
      rc_action_flag = BEGIN_FLAG;
      rc_cmd_time = 0;
    }
    else if (rc_action_flag == GYRO_FLAG && rc_cmd_time > RC_CMD_LONG_TIME + 500)
    {
        //gyro cali
      rc_action_flag = 0;
      //rc_cmd_time = 0;
			Already_start--;
      cali_sensor[CALI_GYRO].cali_cmd = 1;
      //update control temperature
      head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
      if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
      {
        head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
      }
				
		  cali_done_buzzer();
    }

		if ((rc_cmd_systemTick != 0) && (rc_action_flag != 0) && (Already_start))
    {
      imu_start_buzzer();
			rc_action_flag = GYRO_FLAG;
    }
    
}


int8_t get_control_temperature(void)
{
	return head_cali.temperature;
}

void get_flash_latitude(fp32* latitude)
{
	if(latitude == NULL)
	{
		return;
	}
	
	if(cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
	{
		*latitude = head_cali.latitude;
	}
	else
	{
		*latitude = 22.0f;
	}
}

//校准完之后每次从flash读取校准数据
void cali_param_init(void)
{
	uint8_t i = 0;
	for(i = 0; i < CALI_LIST_LENGTH; i++)
	{
		cali_sensor[i].flash_len = cali_sensor_size[i];
		cali_sensor[i].flash_buf = cali_sensor_buf[i];
		cali_sensor[i].cali_hook = (bool_t(*)(uint32_t*, bool_t))cali_hook_fun[i];
	}
	
	cali_data_read();
	
	for (i = 0; i < CALI_LIST_LENGTH; i++)
  {
    if (cali_sensor[i].cali_done == CALIED_FLAG)
    {
      if (cali_sensor[i].cali_hook != NULL)
      {
        //if has been calibrated, set to init 
        cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
      }
    }
	}
	
  calibrate_RC = get_remote_ctrl_point_cali();
	HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	
}

static void cali_data_read(void)
{
  uint8_t flash_read_buf[CALI_SENSOR_HEAD_LENGTH * 4];
  uint8_t i = 0;
  uint16_t offset = 0;
  for (i = 0; i < CALI_LIST_LENGTH; i++)
  {

        //read the data in flash, 
    cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        
    offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
    cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LENGTH);
        
    cali_sensor[i].name[0] = flash_read_buf[0];
    cali_sensor[i].name[1] = flash_read_buf[1];
    cali_sensor[i].name[2] = flash_read_buf[2];
    cali_sensor[i].cali_done = flash_read_buf[3];
        
    offset += CALI_SENSOR_HEAD_LENGTH * 4;

    if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
    {
      cali_sensor[i].cali_cmd = 1;
    }
  }
}

static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;


    for (i = 0; i < CALI_LIST_LENGTH; i++)
    {
        //copy the data of device calibration data
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;

        //copy the name and "CALI_FLAG" of device
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LENGTH * 4);
        offset += CALI_SENSOR_HEAD_LENGTH * 4;
    }

    //erase the page
    cali_flash_erase(FLASH_USER_ADDR,1);
    //write data
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGTH + 3) / 4);
}

static bool_t cali_head_hook(uint32_t* cali, bool_t cmd)
{
	head_cali_t* local_cali = (head_cali_t*)cali;
	if(cmd == CALI_FUNC_CMD_INIT)
	{
		return 1;//校准过了
	}
	
	local_cali->self_id = SELF_ID;
	//初始温度大概在22°
  //采集MCU的温度，然后+10度作为imu温度
	local_cali->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
	
	if(local_cali->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
	{
		local_cali->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
	}
	
	local_cali->firmware_version = FIRWARE_VERSION;
	local_cali->latitude = 22.0f;
	
	return 1;
}

static bool_t cali_gyro_hook(uint32_t* cali, bool_t cmd)
{
	imu_cali_t* local_cali = (imu_cali_t*)cali;
	if(cmd==CALI_FUNC_CMD_INIT)
	{
		gyro_set_cali(local_cali->scale, local_cali->offset);
		return 0;
	}
	else if(cmd == CALI_FUNC_CMD_ON)
	{
		static uint16_t count_time = 0;
		
		//计算零位漂移
    gyro_cali_fun(local_cali->scale, local_cali->offset, &count_time);
    if (count_time > GYRO_CALIBRATE_TIME)
    {
			imu_cali_done();
      gyro_cali_enable_control();
      return 1;
    }
    else
    {
      gyro_cali_disable_control(); //disable the remote control to make robot no move     
      return 0;
    }
	}
	return 0;
}
