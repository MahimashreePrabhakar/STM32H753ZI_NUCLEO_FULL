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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define FEED1_EN_Pin GPIO_PIN_3
#define FEED1_EN_GPIO_Port GPIOF
#define TEMP2_OUT_Pin GPIO_PIN_5
#define TEMP2_OUT_GPIO_Port GPIOF
#define F1_OUT_Pin GPIO_PIN_0
#define F1_OUT_GPIO_Port GPIOC
#define TEMP1_OUT_Pin GPIO_PIN_3
#define TEMP1_OUT_GPIO_Port GPIOC
#define L3_PWM_Pin GPIO_PIN_0
#define L3_PWM_GPIO_Port GPIOA
#define F2_OUT_Pin GPIO_PIN_3
#define F2_OUT_GPIO_Port GPIOA
#define THCLK_Pin GPIO_PIN_5
#define THCLK_GPIO_Port GPIOA
#define TH_DO_MISO_Pin GPIO_PIN_6
#define TH_DO_MISO_GPIO_Port GPIOA
#define EXT1_DIR_Pin GPIO_PIN_7
#define EXT1_DIR_GPIO_Port GPIOA
#define L4_PWM_Pin GPIO_PIN_0
#define L4_PWM_GPIO_Port GPIOB
#define LB1_OUT_Pin GPIO_PIN_14
#define LB1_OUT_GPIO_Port GPIOF
#define FEED_EN_Pin GPIO_PIN_9
#define FEED_EN_GPIO_Port GPIOE
#define FEED4_DIR_Pin GPIO_PIN_11
#define FEED4_DIR_GPIO_Port GPIOE
#define LB2_OUT_Pin GPIO_PIN_13
#define LB2_OUT_GPIO_Port GPIOE
#define L1_PWM_Pin GPIO_PIN_10
#define L1_PWM_GPIO_Port GPIOB
#define L2_PWM_Pin GPIO_PIN_11
#define L2_PWM_GPIO_Port GPIOB
#define THCS1_Pin GPIO_PIN_13
#define THCS1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define FEED2_PWM_Pin GPIO_PIN_15
#define FEED2_PWM_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define EXT2_EN_Pin GPIO_PIN_14
#define EXT2_EN_GPIO_Port GPIOD
#define EXT2_DIR_Pin GPIO_PIN_15
#define EXT2_DIR_GPIO_Port GPIOD
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define EXT1_PWM_Pin GPIO_PIN_6
#define EXT1_PWM_GPIO_Port GPIOC
#define TX_Pin GPIO_PIN_5
#define TX_GPIO_Port GPIOD
#define RX_Pin GPIO_PIN_6
#define RX_GPIO_Port GPIOD
#define FEED1_DIR_Pin GPIO_PIN_12
#define FEED1_DIR_GPIO_Port GPIOG
#define LB3_OUT_Pin GPIO_PIN_14
#define LB3_OUT_GPIO_Port GPIOG
#define EXT1_EN_Pin GPIO_PIN_5
#define EXT1_EN_GPIO_Port GPIOB
#define LB4_OUT_Pin GPIO_PIN_6
#define LB4_OUT_GPIO_Port GPIOB
#define EXT2_PWM_Pin GPIO_PIN_8
#define EXT2_PWM_GPIO_Port GPIOB
#define FEED1_PWM_Pin GPIO_PIN_9
#define FEED1_PWM_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
