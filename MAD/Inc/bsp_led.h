#ifndef __BSP_LED_H__
#define __BSP_LED_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* LED 1-7 */
#define LED1(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET))
#define LED2(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET))
#define LED3(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET))
#define LED4(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET))
#define LED5(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET))
#define LED6(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET))
#define LED7(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET))

/* LED片选 */
#define E3(x)   (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET))
#define A0(x)   (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET))
#define A1(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET))
#define A2(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET))

/* 双闪 */
#define DOUBLELIGHT(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET))
#endif
