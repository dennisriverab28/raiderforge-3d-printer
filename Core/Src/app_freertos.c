/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "integration_test.h"
//#include "ddr_globals.h"
#include "uart_debug.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void StartIntegrationTestTask(void);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 * IntegrationTestTask
 *
 * One-shot FreeRTOS task that runs the full system integration test once,
 * prints all results over UART (huart2, 115200 baud), then deletes itself.
 *
 * The 500ms delay at the start lets HeaterTask, MotionTask, and DisplayTask
 * get scheduled at least a few times first, so DDRlo is in a stable state
 * when we start probing it.
 */
static void IntegrationTestTask(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    vTaskDelete(NULL);
}

/*
 * StartIntegrationTestTask
 *
 * Call this from main() BEFORE osKernelStart() / vTaskStartScheduler().
 * Stack: 1024 words = 4KB on Cortex-M0+ (enough for printf + float math).
 * Priority: 1 (lowest user priority — integration test never blocks real tasks).
 */
void StartIntegrationTestTask(void)
{
    xTaskCreate(IntegrationTestTask,
                "IntTest",
                1024,
                NULL,
                1,
                NULL);
}

/* USER CODE END Application */

