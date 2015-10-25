/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi[2];

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi[SPI_1].Instance = SPI1;
  hspi[SPI_1].Init.Mode = SPI_MODE_MASTER;
  hspi[SPI_1].Init.Direction = SPI_DIRECTION_2LINES;
  hspi[SPI_1].Init.DataSize = SPI_DATASIZE_8BIT;
  hspi[SPI_1].Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi[SPI_1].Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi[SPI_1].Init.NSS = SPI_NSS_SOFT;
  hspi[SPI_1].Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi[SPI_1].Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi[SPI_1].Init.TIMode = SPI_TIMODE_DISABLED;
  hspi[SPI_1].Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi[SPI_1].Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi[SPI_1]);

}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi[SPI_2].Instance = SPI2;
  hspi[SPI_2].Init.Mode = SPI_MODE_MASTER;
  hspi[SPI_2].Init.Direction = SPI_DIRECTION_2LINES;
  hspi[SPI_2].Init.DataSize = SPI_DATASIZE_8BIT;
  hspi[SPI_2].Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi[SPI_2].Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi[SPI_2].Init.NSS = SPI_NSS_SOFT;
  hspi[SPI_2].Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi[SPI_2].Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi[SPI_2].Init.TIMode = SPI_TIMODE_DISABLED;
  hspi[SPI_2].Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi[SPI_2].Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi[SPI_2]);

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __SPI1_CLK_ENABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */
  	/* SPI CS Pin:PA4 */
	GPIO_InitStruct.Pin	= GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	SPI1_CS_HIGH();
  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */
  
  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

static void spix_error(SPI_HandleTypeDef *phspi)
{
	HAL_SPI_DeInit(phspi);

	if (phspi->Instance == SPI1)
	{
		MX_SPI1_Init();
	}
	else if (phspi->Instance == SPI2)
	{
		MX_SPI2_Init();
	}
}

static uint8_t spix_rw_byte(SPI_Type_Def spix, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_value = 0;

	status = HAL_SPI_TransmitReceive(&hspi[spix], &value, &read_value, 1, SPIX_MAX_TIMEOUT);

	if (status != HAL_OK)
	{
		spix_error(&hspi[spix]);
	}

	return read_value;
}

/* USER CODE BEGIN 1 */
uint8_t spi1_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	SPI1_CS_LOW();

	status = spix_rw_byte(SPI_1, reg);
	spix_rw_byte(SPI_1, value);

	SPI1_CS_HIGH();

	return status;
}

uint8_t spi1_read_reg(uint8_t reg)
{
	uint8_t value = 0;

	SPI1_CS_LOW();

	spix_rw_byte(SPI_1, reg);
	value = spix_rw_byte(SPI_1, 0xFF);

	SPI1_CS_HIGH();

	return value;
}

uint8_t spi1_read_buf(uint8_t reg, uint8_t *pbuf, uint16_t len)
{
	uint8_t status = 0;

	SPI1_CS_LOW();

	status = spix_rw_byte(SPI_1, reg);
	while(len--)
	{
		*pbuf = spix_rw_byte(SPI_1, 0xFF);
		pbuf++;
	}

	SPI1_CS_HIGH();

	return status;	
}

uint8_t spi1_write_buf(uint8_t reg, const uint8_t *pbuf, uint16_t len)
{
	uint8_t status = 0;

	SPI1_CS_LOW();

	status = spix_rw_byte(SPI_1, reg);
	while(len--)
	{
		spix_rw_byte(SPI_1, *pbuf);
		pbuf++;
	}

	SPI1_CS_HIGH();

	return status;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
