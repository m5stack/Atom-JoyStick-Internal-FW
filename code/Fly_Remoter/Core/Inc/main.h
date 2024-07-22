/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t i2c_address[1];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEFT_BTN_Pin GPIO_PIN_0
#define LEFT_BTN_GPIO_Port GPIOF
#define RIGHT_BTN_Pin GPIO_PIN_1
#define RIGHT_BTN_GPIO_Port GPIOF
#define ADC_BAT_Pin GPIO_PIN_0
#define ADC_BAT_GPIO_Port GPIOA
#define ADC_BAT_2_Pin GPIO_PIN_1
#define ADC_BAT_2_GPIO_Port GPIOA
#define LEFT_SW_X_Pin GPIO_PIN_2
#define LEFT_SW_X_GPIO_Port GPIOA
#define LEFT_SW_Y_Pin GPIO_PIN_3
#define LEFT_SW_Y_GPIO_Port GPIOA
#define LEFT_SW_B_Pin GPIO_PIN_4
#define LEFT_SW_B_GPIO_Port GPIOA
#define RIGHT_SW_Y_Pin GPIO_PIN_5
#define RIGHT_SW_Y_GPIO_Port GPIOA
#define RIGHT_SW_X_Pin GPIO_PIN_6
#define RIGHT_SW_X_GPIO_Port GPIOA
#define RIGHT_SW_B_Pin GPIO_PIN_7
#define RIGHT_SW_B_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
