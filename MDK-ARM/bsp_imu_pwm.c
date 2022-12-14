#include "bsp_imu_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim10;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

void imu_pwm_init(void)
{
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}
