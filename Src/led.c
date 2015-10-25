/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "led.h"

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, 
                                 LED2_GPIO_PORT};
                                 
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, 
                                 LED2_PIN};

void LED_Init(Led_TypeDef Led)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the GPIO_LED clock */
	LEDx_GPIO_CLK_ENABLE(Led);

	/* Configure the GPIO_LED Pin */
	GPIO_InitStruct.Pin 	= GPIO_PIN[Led];
	GPIO_InitStruct.Mode	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull	= GPIO_NOPULL;
	GPIO_InitStruct.Speed	= GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

	LED_Off(Led);
}

void LED_Toggle(Led_TypeDef Led)
{
	HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

void LED_On(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

void LED_Off(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led],GPIO_PIN_RESET);
}
