/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "hmc5883l.h"

static void hmc_io_init(void)
{
	GPIO_InitTypeDef gpio_init;
	
	gpio_init.Pin	= GPIO_PIN_8;
	gpio_init.Mode	= GPIO_MODE_IT_RISING;
	gpio_init.Pull	= GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio_init);

	/* Enable and set exti8(PA8) Interrupt to highest priority */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 2);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void hmc_init(void)
 {
	uint8_t config[3] = {0x74, 0x20, 0x00};

	hmc_io_init();

	I2C1_WriteBuffer(HMC_ADDR, 0x00, 3, config);
}

