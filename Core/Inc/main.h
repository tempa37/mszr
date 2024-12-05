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
#include "stm32f4xx_hal.h"

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
#define Fixing_the_leak_Pin GPIO_PIN_6
#define Fixing_the_leak_GPIO_Port GPIOA
#define WDI_Pin GPIO_PIN_2
#define WDI_GPIO_Port GPIOB
#define INT_LAN8710_Pin GPIO_PIN_7
#define INT_LAN8710_GPIO_Port GPIOE
#define LED_DISPLAY_Pin GPIO_PIN_10
#define LED_DISPLAY_GPIO_Port GPIOE
#define SPI_NCC_Pin GPIO_PIN_11
#define SPI_NCC_GPIO_Port GPIOE
#define OLED_RST_Pin GPIO_PIN_15
#define OLED_RST_GPIO_Port GPIOE
#define MCU_BLK_2_2_Pin GPIO_PIN_11
#define MCU_BLK_2_2_GPIO_Port GPIOD
#define MCU_BLK_2_1_Pin GPIO_PIN_12
#define MCU_BLK_2_1_GPIO_Port GPIOD
#define MCU_BLK_1_1_Pin GPIO_PIN_15
#define MCU_BLK_1_1_GPIO_Port GPIOD
#define MCU_BLK_1_2_Pin GPIO_PIN_6
#define MCU_BLK_1_2_GPIO_Port GPIOC
#define RST_PHYLAN_Pin GPIO_PIN_8
#define RST_PHYLAN_GPIO_Port GPIOC
#define RELAY_CONTROL_Pin GPIO_PIN_9
#define RELAY_CONTROL_GPIO_Port GPIOA
#define Checking_for_leaks_Pin GPIO_PIN_10
#define Checking_for_leaks_GPIO_Port GPIOA
#define PA12_Pin GPIO_PIN_12
#define PA12_GPIO_Port GPIOA
#define RS485_1_ON_Pin GPIO_PIN_15
#define RS485_1_ON_GPIO_Port GPIOA
#define UART1_RE_DE_Pin GPIO_PIN_12
#define UART1_RE_DE_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
