#ifndef __HMC_H_
#define __HMC_H_

#include "stm32f1xx_hal.h"

#define HMC_ADDR	0x3C

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}direct_t;

typedef struct
{
	float x;
	float y;
	float z;
}angle_t;

typedef struct
{
	direct_t direct;
	angle_t angle;
}hmc_data_t;

void hmc_init(void);
uint32_t hmc_is_ready(uint32_t trials);
uint32_t hmc_get_status(void);
uint32_t hmc_get_dst(uint8_t dst_addr);
uint32_t hmc_get_angle(hmc_data_t *hmc_data);

#endif
