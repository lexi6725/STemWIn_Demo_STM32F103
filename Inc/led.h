#ifndef __LED_H_
#define __LED_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

typedef enum 
{
  LED1 = 0,
  LED2 = 1,
}Led_TypeDef;

#define LEDn                             2

#define LED1_PIN                         GPIO_PIN_13
#define LED1_GPIO_PORT                   GPIOD
#define LED1_GPIO_CLK_ENABLE()           __GPIOD_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __GPIOD_CLK_DISABLE()
  
#define LED2_PIN                         GPIO_PIN_14
#define LED2_GPIO_PORT                   GPIOG
#define LED2_GPIO_CLK_ENABLE()           __GPIOG_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __GPIOG_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) LED1_GPIO_CLK_ENABLE(); else \
                                            if((__INDEX__) == 1) LED2_GPIO_CLK_ENABLE(); \
                                            }while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED1_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 1) LED2_GPIO_CLK_DISABLE(); \
                                            }while(0)
                                            
void LED_Init(Led_TypeDef Led);
void LED_On(Led_TypeDef Led);
void LED_Off(Led_TypeDef Led);
void LED_Toggle(Led_TypeDef Led);

#ifdef __cplusplus
	}
#endif

#endif
