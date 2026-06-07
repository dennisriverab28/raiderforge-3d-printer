/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stepper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim17;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    /* TIM1 is the HAL timebase (stm32g0xx_hal_timebase_tim.c).
     * Handle the update flag manually so we can service both the HAL tick
     * AND stepper step-counting without HAL_TIM_IRQHandler clearing the flag
     * before Stepper_TIM1_UpdateISR gets to check it. */
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) &&
        __HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE))
    {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        HAL_IncTick();               /* replaces HAL_TIM_IRQHandler for timebase */
        Stepper_TIM1_UpdateISR();    /* no-op when not in a counted move */
    }
}

void TIM3_TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) &&
        __HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE))
    {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        Stepper_TIM3_UpdateISR();
    }
}

void TIM15_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim15, TIM_FLAG_UPDATE) &&
        __HAL_TIM_GET_IT_SOURCE(&htim15, TIM_IT_UPDATE))
    {
        __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
        Stepper_TIM15_UpdateISR();
    }
}

void TIM17_FDCAN_IT1_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim17, TIM_FLAG_UPDATE) &&
        __HAL_TIM_GET_IT_SOURCE(&htim17, TIM_IT_UPDATE))
    {
        __HAL_TIM_CLEAR_FLAG(&htim17, TIM_FLAG_UPDATE);
        Stepper_TIM17_UpdateISR();
        return;
    }

    /* TIM17 shares this vector with FDCAN on STM32G0. Clear any stray TIM17
     * advanced-timer flags so the CPU does not get trapped servicing an
     * interrupt source unrelated to counted step updates. */
    __HAL_TIM_CLEAR_FLAG(&htim17, TIM_FLAG_CC1 | TIM_FLAG_TRIGGER |
                                  TIM_FLAG_BREAK | TIM_FLAG_UPDATE);
}

/* USER CODE END 1 */
