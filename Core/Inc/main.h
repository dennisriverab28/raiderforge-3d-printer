/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
extern TIM_HandleTypeDef htim1;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder_Button_Pin GPIO_PIN_11
#define Encoder_Button_GPIO_Port GPIOC
#define Screen_Data_Pin GPIO_PIN_12
#define Screen_Data_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define ENDSTOP_X_MAX_PIN_Pin GPIO_PIN_0
#define ENDSTOP_X_MAX_PIN_GPIO_Port GPIOC
#define ENDSTOP_Y_MAX_PIN_Pin GPIO_PIN_1
#define ENDSTOP_Y_MAX_PIN_GPIO_Port GPIOC
#define ENDSTOP_Z_MAX_PIN_Pin GPIO_PIN_2
#define ENDSTOP_Z_MAX_PIN_GPIO_Port GPIOC
#define ENDSTOP_X_PIN_Pin GPIO_PIN_3
#define ENDSTOP_X_PIN_GPIO_Port GPIOC
#define UART_TX_Pin GPIO_PIN_2
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_3
#define UART_RX_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define HEATER_PWM_Pin GPIO_PIN_6
#define HEATER_PWM_GPIO_Port GPIOA
#define ENDSTOP_Y_PIN_Pin GPIO_PIN_4
#define ENDSTOP_Y_PIN_GPIO_Port GPIOC
#define ENDSTOP_Z_PIN_Pin GPIO_PIN_5
#define ENDSTOP_Z_PIN_GPIO_Port GPIOC
#define DIR_PIN_Pin GPIO_PIN_0
#define DIR_PIN_GPIO_Port GPIOB
#define THERM_HOTEND_Pin GPIO_PIN_1
#define THERM_HOTEND_GPIO_Port GPIOB
#define THERM_BED_Pin GPIO_PIN_10
#define THERM_BED_GPIO_Port GPIOB
#define STEP_Z_PIN_Pin GPIO_PIN_14
#define STEP_Z_PIN_GPIO_Port GPIOB
#define DIR_B_PIN_Pin GPIO_PIN_8
#define DIR_B_PIN_GPIO_Port GPIOA
#define EN_B_PIN_Pin GPIO_PIN_9
#define EN_B_PIN_GPIO_Port GPIOA
#define Z_DIR_PIN_Pin GPIO_PIN_6
#define Z_DIR_PIN_GPIO_Port GPIOC
#define Z_EN_PIN_Pin GPIO_PIN_7
#define Z_EN_PIN_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define E_DIR_PIN_Pin GPIO_PIN_8
#define E_DIR_PIN_GPIO_Port GPIOC
#define E_EN_PIN_Pin GPIO_PIN_9
#define E_EN_PIN_GPIO_Port GPIOC
#define STEP_E_PIN_Pin GPIO_PIN_1
#define STEP_E_PIN_GPIO_Port GPIOD
#define Screen_CS_Pin GPIO_PIN_2
#define Screen_CS_GPIO_Port GPIOD
#define STEP_PIN_Pin GPIO_PIN_4
#define STEP_PIN_GPIO_Port GPIOB
#define STEP_B_PIN_Pin GPIO_PIN_5
#define STEP_B_PIN_GPIO_Port GPIOB
#define EN_PIN_Pin GPIO_PIN_6
#define EN_PIN_GPIO_Port GPIOB
#define BED_HEATER_PWM_Pin GPIO_PIN_7
#define BED_HEATER_PWM_GPIO_Port GPIOB
#define HOTEND_FAN_Pin GPIO_PIN_8
#define HOTEND_FAN_GPIO_Port GPIOB
#define PART_COOLING_FAN_Pin GPIO_PIN_9
#define PART_COOLING_FAN_GPIO_Port GPIOB
#define Screen_CLK_Pin GPIO_PIN_10
#define Screen_CLK_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
