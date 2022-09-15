#ifndef CALIBRATE_TASK_H_
#define CALIBRATE_TASK_H_

#include "struct_typedef.h"
#include "bsp_adc.h"

#define cali_get_mcu_temperature() get_temperature() //����¶�

//flash��д֮ǰ��Ҫ����
#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash ��ȡ����
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash д�뺯��
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num)) 

#define get_remote_ctrl_point_cali()        get_remote_control_point()  //get the remote control point����ȡң����ָ��
#define gyro_cali_disable_control()         RC_unable()                 //when imu is calibrating, disable the remote control.��imu��У׼ʱ��,ʧ��ң����
#define gyro_cali_enable_control()          RC_restart(SBUS_RX_BUF_NUM)

#define get_remote_ctrl_point_cali() get_remote_control_point()

#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))

#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_9 //write flash page 9,�����flashҳ��ַ

#define GYRO_CONST_MAX_TEMP     45.0f               //max control temperature of gyro,��������ǿ����¶�
#define CALI_SENSOR_HEAD_LENGTH  1

#define CALI_FUNC_CMD_ON        1                   //need calibrate,����У׼
#define CALI_FUNC_CMD_INIT      0                   //has been calibrated, set value to init.�Ѿ�У׼��������У׼ֵ

#define SELF_ID 0
#define FIRWARE_VERSION 12345
#define CALIBRATE_CONTROL_TIME  1             //osDelay time,  means 1ms.1ms ϵͳ��ʱ
#define CALIED_FLAG             0x55                // means it has been calibrated
//you have 20 seconds to calibrate by remote control. ��20s������ң��������У׼
#define GYRO_CALIBRATE_TIME         20000
#define RC_CMD_LONG_TIME        200
#define RC_CALI_VALUE_HOLE      600 
#define CALIBRATE_END_TIME      10000

#define RC_CALI_BUZZER_CYCLE_TIME 500
#define RC_CALI_BUZZER_PAUSE_TIME 250


typedef enum
{
	CALI_HEAD = 0,
  CALI_GYRO = 1,
  CALI_ACC = 2,
  CALI_MAG = 3,
  CALI_LIST_LENGTH,
} cali_id;

typedef __packed struct
{
    uint8_t self_id;            // the "SELF_ID"
    uint16_t firmware_version;  // set to the "FIRMWARE_VERSION"
    //'temperature' and 'latitude' should not be in the head_cali, because don't want to create a new sensor
    //'temperature' and 'latitude'��Ӧ����head_cali,��Ϊ���봴��һ���µ��豸�ͷ�����
    int8_t temperature;         // imu control temperature
    fp32 latitude;              // latitude
} head_cali_t;

typedef __packed struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);   //cali function
} cali_sensor_t;

typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;

extern void cali_param_init(void);
extern int8_t get_control_temperature(void);
extern void get_flash_latitude(float *latitude);
void calibrate_task(void const *pvParameters);
#endif













