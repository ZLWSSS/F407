#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_calidone(uint16_t psc, uint16_t pwm)
{
	__HAL_TIM_PRESCALER(&htim4, psc);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
	HAL_Delay(100);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
