/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define bypass_Pin GPIO_PIN_6
#define bypass_GPIO_Port GPIOA
#define adc1_channel_7_isense_Pin GPIO_PIN_7
#define adc1_channel_7_isense_GPIO_Port GPIOA
#define adc1_channel_4_voutsense_Pin GPIO_PIN_4
#define adc1_channel_4_voutsense_GPIO_Port GPIOC
#define adc1_channel_8_vinsense_Pin GPIO_PIN_5
#define adc1_channel_8_vinsense_GPIO_Port GPIOC
#define reset_Pin GPIO_PIN_15
#define reset_GPIO_Port GPIOA
#define select_Pin GPIO_PIN_0
#define select_GPIO_Port GPIOD
#define MODBUS_nWR2_Pin GPIO_PIN_4
#define MODBUS_nWR2_GPIO_Port GPIOD
#define MODBUS_TX2_Pin GPIO_PIN_5
#define MODBUS_TX2_GPIO_Port GPIOD
#define MODBUS_RX2_Pin GPIO_PIN_6
#define MODBUS_RX2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
