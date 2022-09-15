#include "bsp_buzzer.h"
#include "main.h"
#include "cmsis_os.h"
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

void Cali_Complete_Buzzer(void)
{
	buzzer_on(3, 20000);
	osDelay(250);
	
	buzzer_on(2, 20000);
	osDelay(250);
	
	buzzer_on(1, 20000);
	osDelay(250);
	buzzer_off();
}

