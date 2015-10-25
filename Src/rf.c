/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "rf.h"

const uint8_t rf_addr[nRF_TX_ADR_WIDTH] = {0x59, 0x12, 0x67, 0x67, 0x25};

void rf_io_init(void)
{
	GPIO_InitTypeDef gpio_init;

	__GPIOA_CLK_ENABLE();

	/* Configure RF_CE_Pin(PA3) */
	gpio_init.Pin		= GPIO_PIN_3;
	gpio_init.Mode		= GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull		= GPIO_PULLUP;
	gpio_init.Speed		= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio_init);

	/* Configure RF_IRQ_Pin(PA2) */
	gpio_init.Pin		= GPIO_PIN_2;
	gpio_init.Mode		= GPIO_MODE_IT_FALLING;
	gpio_init.Pull		= GPIO_NOPULL;
	gpio_init.Speed		= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio_init);

	/* Enable and set EXTI2(PA2) Interrupt to the highest priority */
	HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 2);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	RF_CE_LOW();
}

uint32_t rf_check_connect(void)
{
	uint8_t buf[5];
	uint8_t index;

	spi1_write_buf(nRF_WRITE_REG+nRF_TX_ADDR, rf_addr, 5);
	spi1_read_buf(nRF_TX_ADDR, buf, 5);

	for (index = 0; index < 5; ++index)
	{
		if (buf[index] != rf_addr[index)
		{
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

void rf_mode_rx(void)
{
	RF_CE_LOW();

	spi1_write_buf(nRF_WRITE_REG+nRF_TX_ADDR,(const uint8_t *)rf_addr, nRF_TX_ADR_WIDTH);
	spi1_write_buf(nRF_WRITE_REG+nRF_RX_ADDR_P0,(const uint8_t *)rf_addr, nRF_TX_ADR_WIDTH);

	spi1_write_reg(nRF_WRITE_REG+nRF_EN_AA, 0x01);
	spi1_write_reg(nRF_WRITE_REG+nRF_EN_RXADDR, 0x01);
	spi1_write_reg(nRF_WRITE_REG+nRF_RF_CH, 67);
	spi1_write_reg(nRF_WRITE_REG+nRF_RX_PW_P0, nRF_RX_PLOAD_WIDTH);
	spi1_write_reg(nRF_WRITE_REG+nRF_RF_SETUP, 0x0F);
	spi1_write_reg(nRF_WRITE_REG+nRF_CONFIG, 0x0F);
	
	spi1_write_reg(nRF_WRITE_REG+nRF_STATUS, 0xFF);

	RF_CE_HIGH();
}

void rf_mode_tx(void)
{
	RF_CE_LOW();

	spi1_write_buf(nRF_WRITE_REG+nRF_TX_ADDR,(const uint8_t *)rf_addr, nRF_TX_ADR_WIDTH);
	spi1_write_buf(nRF_WRITE_REG+nRF_RX_ADDR_P0,(const uint8_t *)rf_addr, nRF_TX_ADR_WIDTH);

	spi1_write_reg(nRF_WRITE_REG+nRF_EN_AA, 0x01);
	spi1_write_reg(nRF_WRITE_REG+nRF_EN_RXADDR, 0x01);
	spi1_write_reg(nRF_WRITE_REG+nRF_SETUP_RETR, 0x03);
	spi1_write_reg(nRF_WRITE_REG+nRF_RF_CH, 67);
	spi1_write_reg(nRF_WRITE_REG+nRF_RX_PW_P0, nRF_RX_PLOAD_WIDTH);
	spi1_write_reg(nRF_WRITE_REG+nRF_RF_SETUP, 0x0F);
	spi1_write_reg(nRF_WRITE_REG+nRF_CONFIG, 0x0E);
	
	spi1_write_reg(nRF_WRITE_REG+nRF_STATUS, 0xFF);

	RF_CE_HIGH();
}

uint8_t rf_start_tx(uint8_t *buf, uint8_t len)
{
	EventBits_t ev_bits;
	const TickType_t tick_to_wait = 5;
	uint8_t ret_value = 0;

	if (buf == NULL)
	{
		return ret_value;
	}

	rf_mode_tx();

	RF_CE_LOW();

	spi1_write_reg(nRF_FLUSH_TX, 0xFF);
	spi1_write_buf(nRF_WR_TX_PLOAD, buf, len);

	RF_CE_HIGH();

	ev_bits = xEventGroupWaitBits(xRFEventGruop, nRF_State_TX_OK|nRF_State_TX_MAX, pdTRUE, pdFALSE, tick_to_wait);
	if (ev_bits & nRF_State_TX_OK)
	{
		ret_value = nRF_TX_OK;
	}
	else if (ev_bits & nRF_State_TX_MAX)
	{
		ret_value = nRF_MAX_TX;
	}
	else
	{
		ret_value = nRF_TIMEOUT;
	}

	RF_CE_LOW();
	spi1_write_reg(nRF_FLUSH_TX, 0xFF);
	RF_CE_HIGH();

	rf_mode_rx();

	return ret_value;
}

uint8_t rf_start_rx(uint8_t *buf, uint8_t len)
{
	EventBits_t ev_bits;
	const TickType_t tick_to_wait = 1000;
	uint8_t ret_value = 0;

	ev_bits = xEventGroupWaitBits(xRFEventGruop, nRF_State_RX_OK, pdTRUE, pdFALSE,tick_to_wait);
	if (ev_bits & nRF_State_RX_OK)
	{
		spi1_read_buf(nRF_RD_RX_PLOAD, buf, len);
		ret_value = nRF_RX_OK;
	}
	else
	{
		ret_value = nRF_TIMEOUT;
	}

	RF_CE_LOW();
	spi1_write_reg(nRF_FLUSH_RX, 0xFF);
	RF_CE_HIGH();

	return ret_value;
}

void rf_isr(void)
{
	BaseType_t result, higher_priority_task;
	uint8_t reg_value;

	reg_value = spi1_read_reg(nRF_STATUS);
	spi1_write_reg(nRF_WRITE_REG+nRF_STATUS, 0xFF);

	higher_priority_task = pdFALSE;

	if (reg_value & nRF_TX_OK)
	{
		result = xEventGroupSetBitsFromISR(xRFEventGruop, nRF_State_TX_OK, &higher_priority_task);
	}
	else if (reg_value & nRF_MAX_TX)
	{
		result = xEventGroupSetBitsFromISR(xRFEventGruop, nRF_State_TX_MAX, &higher_priority_task);
	}
	if (reg_value & nRF_RX_OK)
	{
		result = xEventGroupSetBitsFromISR(xRFEventGruop, nRF_State_RX_OK, &higher_priority_task);
	}

	if (result != pdFAIL)
	{
		portYIELD_FROM_ISR(higher_priority_task);
	}
}

