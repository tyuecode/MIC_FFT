#include "key.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_1)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		HAL_TIM_Base_Start_IT(&htim4);		
	}
}
