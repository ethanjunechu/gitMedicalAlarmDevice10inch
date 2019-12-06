#ifndef __BSP_LED_H__
#define __BSP_LED_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* LED 2 | 5 */
#define LED2C(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET))
#define LED2B(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET))
#define LED2A(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET))
/* LED 1 | 4 */
#define LED1C(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET))
#define LED1B(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET))
#define LED1A(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET))
/* LED 0 | 3 */
#define LED0C(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET))
#define LED0B(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET))
#define LED0A(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET))
/* LED片选 */
#define LED345(x)   (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET))
#define LED012(x)   (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET))
#endif
