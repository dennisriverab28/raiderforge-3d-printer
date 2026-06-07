/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : RaiderForge CoreXY — STM32G0B1 | FreeRTOS Integration
  *
  *  CHANGES FROM ORIGINAL CUBEMX OUTPUT:
  *   [USER CODE Includes]   — added ddr_globals, printer_config, bed_heater
  *   [USER CODE PV]         — added HeaterTask + MotionTask handles/attrs
  *   [USER CODE PFP]        — added StartHeaterTask + StartMotionTask protos
  *   [USER CODE BEGIN 2]    — prescaler fixes, DDRlo init, module inits, test menu
  *   [USER CODE RTOS_THREADS] — registers HeaterTask + MotionTask
  *   [USER CODE BEGIN 4]    — StartHeaterTask + StartMotionTask implementations
  *
  *  DO NOT TOUCH: StartDisplay body — Matthew's code goes there
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "test_runner.h"
#include "uart_debug.h"
#include "ddr_globals.h"
#include "printer_config.h"
#include "bed_heater.h"
#include "thermistor.h"
#include "bed_thermistor.h"
#include "heater.h"
#include "fan.h"
#include "stepper.h"
//#include "corexy.h"
#include "endstop.h"
#include "homing.h"
//#include "logger.h"
//#include "z_e_motion.h"
//#include "motion_tests.h"
//#include "system_bench.h"
//#include "qbc_bridge.h"
#include "gcodefuncs.h"
#include "parser.h"
#include "sdcard.h"
#include "motion.h"
#include "display.h"        /* init_display, boot_display, telemetry_display, menu_display, menu_update_from_encoder */
//#include "gcode_dispatch.h"   /* GCode_Init, GCode_Execute */
//#include "sd_print.h"          /* SDPrint_Mount, SDPrint_Task, SDPrint_StartFile */
//#include "gcode_dispatch_test.h"
//#include "sd_print_test.h"
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* Definitions for LED1 */
osThreadId_t LED1Handle;
const osThreadAttr_t LED1_attributes = {
  .name = "LED1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .priority = (osPriority_t) osPriorityBelowNormal1,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* Definitions for HeaterTask */
osThreadId_t HeaterTaskHandle;
const osThreadAttr_t HeaterTask_attributes = {
  .name       = "HeaterTask",
  .priority   = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};

osThreadId_t BedHeaterTaskHandle;
const osThreadAttr_t BedHeaterTask_attributes = {
  .name       = "BedHeater",
  .priority   = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};

osThreadId_t PartFanTaskHandle;
const osThreadAttr_t PartFanTask_attributes = {
  .name       = "PartFan",
  .priority   = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
const osThreadAttr_t MotionTask_attributes = {
  .name       = "MotionTask",
  .priority   = (osPriority_t) osPriorityRealtime,
  .stack_size = 768 * 4
};

osThreadId_t SDCardTaskHandle;
const osThreadAttr_t SDCardTask_attributes = {
		.name = "SDCardTask",
		.priority = (osPriority_t) osPriorityAboveNormal,
		.stack_size = 512 * 4
};

osThreadId_t ParserTaskHandle;
const osThreadAttr_t ParserTask_attributes = {
			.name = "ParserTask",
			.priority = (osPriority_t) osPriorityHigh,
			.stack_size = 512 * 4
		};

/* UART mutex — guards all UARTDBG_Print calls from tasks.
   Without this: HeaterTask (AboveNormal) preempts LED1 (Normal) mid-transmission
   → HAL_UART_Transmit returns HAL_BUSY → HeaterTask spins waiting for UART free
   → LED1 never runs to finish → priority-inversion deadlock, system freezes. */
osMutexId_t uart_mutex_id;

#define TASK_PRINT(...) do { \
    UARTDBG_Print(__VA_ARGS__); \
} while(0)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM15_Init(void);
static void MX_SPI1_Init(void);
void StartLED1(void *argument);
void StartDisplay(void *argument);

/* USER CODE BEGIN PFP */
void StartHeaterTask(void *argument);
void StartMotionTask(void *argument);
void StartBedHeaterTask(void *argument);
void StartPartFanTask(void *argument);
void StartSDCardTask(void *argument);
void StartParserTask(void *argument);
void RaiderForge_StartNormalRTOSTasks(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM15_Init();
  MX_SPI1_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN 2 */

  // ═══════════════════════════════════════════════════════════════════════
  // PRESCALER FIXES
  // CubeMX generated PSC=79 (assumes 80 MHz from old L476 project).
  // G0B1 runs at 16 MHz HSI, no PLL.
  // Correct: PSC=15 → 16 MHz / (15+1) = 1 MHz timer clock.
  // With ARR=999: 1 MHz / 1000 = 1 kHz PWM on all channels.
  // ═══════════════════════════════════════════════════════════════════════

  __HAL_TIM_SET_PRESCALER(&htim3,  15);  htim3.Init.Prescaler  = 15;   // stepper A
  __HAL_TIM_SET_PRESCALER(&htim4,  15);  htim4.Init.Prescaler  = 15;   // fans
  __HAL_TIM_SET_AUTORELOAD(&htim4, 999); htim4.Init.Period     = 999;  // fans ARR fix
  __HAL_TIM_SET_PRESCALER(&htim16, 15);  htim16.Init.Prescaler = 15;   // hotend heater
  __HAL_TIM_SET_AUTORELOAD(&htim16, (1000000U / HOTEND_HEATER_PWM_FREQ_HZ) - 1U);
  htim16.Init.Period = (1000000U / HOTEND_HEATER_PWM_FREQ_HZ) - 1U;
  __HAL_TIM_SET_PRESCALER(&htim15, 15);  htim15.Init.Prescaler = 15;   // Z stepper
  __HAL_TIM_SET_PRESCALER(&htim17, 15);  htim17.Init.Prescaler = 15;   // E stepper

  // ═══════════════════════════════════════════════════════════════════════
  // TIM4 CH4 MODE FIX
  // CubeMX set CH4 to TIMING mode. Part cooling fan needs PWM1.
  // ═══════════════════════════════════════════════════════════════════════
  {
    TIM_OC_InitTypeDef fixOC = {0};
    fixOC.OCMode     = TIM_OCMODE_PWM1;
    fixOC.Pulse      = 0;
    fixOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    fixOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &fixOC, TIM_CHANNEL_4);
  }

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  {
    GPIO_InitTypeDef hotendFanPin = {0};
    hotendFanPin.Pin = HOTEND_FAN_Pin;
    hotendFanPin.Mode = GPIO_MODE_OUTPUT_PP;
    hotendFanPin.Pull = GPIO_NOPULL;
    hotendFanPin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HOTEND_FAN_GPIO_Port, &hotendFanPin);
    HAL_GPIO_WritePin(HOTEND_FAN_GPIO_Port, HOTEND_FAN_Pin, GPIO_PIN_RESET);
  }

  // ═══════════════════════════════════════════════════════════════════════
  // ADC CALIBRATION — must run before any thermistor reads
  // ═══════════════════════════════════════════════════════════════════════
  HAL_ADCEx_Calibration_Start(&hadc1);  // G0 HAL — single argument only (no ADC_SINGLE_ENDED)

  // ═══════════════════════════════════════════════════════════════════════
  // ADC SAMPLING TIME FIX
  // CubeMX generated ADC_SAMPLETIME_1CYCLE_5 — too fast for 4.7K source.
  // 160.5 cycles @ 8 MHz ADC clock = ~20 µs — safe for thermistor reads.
  // ═══════════════════════════════════════════════════════════════════════
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  HAL_ADC_Init(&hadc1);

  // ═══════════════════════════════════════════════════════════════════════
  // UART DEBUG
  // ═══════════════════════════════════════════════════════════════════════
  UARTDBG_Init(&huart2);
  UARTDBG_Print("[boot] uart debug online baud=115200\r\n");

  // ═══════════════════════════════════════════════════════════════════════
  // DDR GLOBALS — zero all state, set integrity markers
  // Must come before any module init that writes into DDRlo.
  // ═══════════════════════════════════════════════════════════════════════
//  DDR_InitGlobals();
  QBC_InitializeGlobals();   /* qbc_globals.c: allocates Quinton Glo, implements myprintf */

  // ═══════════════════════════════════════════════════════════════════════
  // THERMISTORS
  // Both share hadc1. Each call to Thermistor_ReadTemp / BedTherm_ReadTemp
  // reconfigures the channel (IN9 or IN10) before every read — safe to
  // call in any order from any task.
  // ═══════════════════════════════════════════════════════════════════════
//  Thermistor_Init(NULL);
//  BedTherm_Init(NULL);

  // ═══════════════════════════════════════════════════════════════════════
  // HOTEND HEATER — TIM16 CH1 (PA6), reads ADC1 IN9 (PB1)
  // kp=8, ki=0.3, kd=50 — tuned for 40W ceramic hotend
  // ═══════════════════════════════════════════════════════════════════════
//  {
//    Heater_Config_t hotend_cfg = {
//      .kp       = 8.0f,
//      .ki       = 0.3f,
//      .kd       = 50.0f,
//      .max_temp = 280.0f,
//      .min_temp = -10.0f
//    };
//    Heater_Init(&DDRlo.hotend, &htim16, TIM_CHANNEL_1, &hadc1, &hotend_cfg);
//  }

  // ═══════════════════════════════════════════════════════════════════════
  // BED HEATER — TIM17 CH1 (PA7), reads ADC1 IN10 (PB10)
  // Uses internal static Heater_t — access via BedHeater_* API only.
  // kp=3, ki=0.05, kd=80 — tuned for large aluminium bed (slow thermal mass)
  // ═══════════════════════════════════════════════════════════════════════
 // BedHeater_InitDefault();

  // ═══════════════════════════════════════════════════════════════════════
  // FANS
  // TIM4 CH3 (PB8) = hotend fan   — safety-critical, auto-on at 50°C
  // TIM4 CH4 (PB9) = part cooling — G-code controlled (M106/M107)
  // Fan_Off after init so PWM output starts low.
  // ═══════════════════════════════════════════════════════════════════════
  Fan_Init(&DDRlo.hotend_fan, NULL, 0, HOTEND_FAN_GPIO_Port, HOTEND_FAN_Pin);
  Fan_Off(&DDRlo.hotend_fan);

  Fan_Init(&DDRlo.part_fan, &htim4, TIM_CHANNEL_4, NULL, 0);
  Fan_Off(&DDRlo.part_fan);

  // ═══════════════════════════════════════════════════════════════════════
  // STEPPERS
  // Motor A: TIM3 CH1 (PB4) STEP | GPIOB DIR_PIN_Pin | GPIOB EN_PIN_Pin
  // Motor B: TIM3 CH2 (PB5) STEP | GPIOA DIR_B_PIN_Pin | GPIOA EN_B_PIN_Pin
  // Pin names confirmed from MX_GPIO_Init in this file.
  // 200 steps/rev, 1/16 microstepping → 3200 steps/rev
  // ═══════════════════════════════════════════════════════════════════════
//  Stepper_Init(&DDRlo.motor_a,
//               &htim3, TIM_CHANNEL_1,
//               GPIOB, DIR_PIN_Pin,
//               GPIOB, EN_PIN_Pin,
//               1, 200);
//
//  Stepper_Init(&DDRlo.motor_b,
//               &htim3, TIM_CHANNEL_2,
//               GPIOA, DIR_B_PIN_Pin,
//               GPIOA, EN_B_PIN_Pin,
//               1, 200);

  // ═══════════════════════════════════════════════════════════════════════
  // COREXY KINEMATICS
  // Sits on top of steppers. Move commands go through CoreXY_Move().
  // steps_per_mm = 80 (typical for GT2 + 20T pulley + 1/16 microstepping)
  // ═══════════════════════════════════════════════════════════════════════
//  {
//    CoreXY_Config_t cxy_cfg = {
//      .steps_per_mm   = 94.65f,  /* 20445 steps / 216mm measured */
//      .max_speed_mm_s = 200.0f,
//      .x_max          = 216.0f,  /* measured endstop to endstop */
//      .y_max          = 216.0f,  /* measured endstop to endstop */
//    };
//    CoreXY_Init(&DDRlo.corexy, &DDRlo.motor_a, &DDRlo.motor_b, &cxy_cfg);
//  }

  // ═══════════════════════════════════════════════════════════════════════
  // ENDSTOPS
  // PC3 = X endstop  (ENDSTOP_X_PIN_Pin, active_high=0 → normally open,
  //                   closes to GND. Set 1 if your switch closes to 3.3V.)
  // PC4 = Y endstop  (same wiring assumption)
  // ═══════════════════════════════════════════════════════════════════════
//  Endstop_Init(&DDRlo.endstop_x, GPIOC, ENDSTOP_X_PIN_Pin, 1);
//  Endstop_Init(&DDRlo.endstop_y, GPIOC, ENDSTOP_Y_PIN_Pin, 1);

  // ═══════════════════════════════════════════════════════════════════════
  // HOMING
  // Non-blocking state machine. Driven by Homing_Update() in MotionTask.
  // fast_speed: approach at 50 mm/s, slow_speed: re-probe at 5 mm/s
  // backoff_distance: 3 mm retract between fast and slow probe
  // ═══════════════════════════════════════════════════════════════════════
//  {
//    Homing_Config_t home_cfg = {
//      .fast_speed       = 30.0f,   /* reduced from 50 — more reliable endstop detection */
//      .slow_speed       = 5.0f,
//      .backoff_distance = 3.0f,
//      .timeout_ms       = 30000,
//      .settle_ms        = 100,
//      .x_dir            = -1,      /* -1 = toward X MIN endstop */
//      .y_dir            = -1       /* -1 = toward Y MIN endstop */
//    };
//    Homing_Init(&DDRlo.homing, &DDRlo.corexy,
//                &DDRlo.endstop_x, &DDRlo.endstop_y, &home_cfg);
//  }

  // ═══════════════════════════════════════════════════════════════════════
  // Z STEPPER — TIM15 CH1 (PB14) STEP
  // DIR and EN pins: set in printer_config.h (Z_DIR_PORT/PIN, Z_EN_PORT/PIN)
  // Default: PC6=DIR, PC7=EN — change if your wiring is different.
  // STEPS_PER_MM_Z = 1600 default (200 steps × 16 microsteps / 2mm pitch).
  // Measure your leadscrew pitch and update STEPS_PER_MM_Z in printer_config.h.
  // ═══════════════════════════════════════════════════════════════════════
//  Stepper_Init(&DDRlo.motor_z,
//               &htim15, TIM_CHANNEL_1,
//               Z_DIR_PORT, Z_DIR_PIN,
//               Z_EN_PORT,  Z_EN_PIN,
//               MOTOR_EN_ACTIVE_LOW,
//               MOTOR_STEPS_PER_REV);

  // ═══════════════════════════════════════════════════════════════════════
  // E STEPPER (extruder) — TIM15 CH2 (PB15) STEP
  // DIR and EN pins: set in printer_config.h (E_DIR_PORT/PIN, E_EN_PORT/PIN)
  // Default: PC8=DIR, PC9=EN — change if your wiring is different.
  // STEPS_PER_MM_E = 415 default (BMG clone typical).
  // ALWAYS calibrate E-steps: mark 100mm on filament, extrude, measure actual.
  // ═══════════════════════════════════════════════════════════════════════
//  Stepper_Init(&DDRlo.motor_e,
//               &htim15, TIM_CHANNEL_2,
//               E_DIR_PORT, E_DIR_PIN,
//               E_EN_PORT,  E_EN_PIN,
//               MOTOR_EN_ACTIVE_LOW,
//               MOTOR_STEPS_PER_REV);

  // ═══════════════════════════════════════════════════════════════════════
  // Z ENDSTOP — PC5 (ENDSTOP_Z_PIN_Pin, already in main.h + GPIO init)
  // NC switch, pull-up on. Same wiring as X/Y endstops.
  // ═══════════════════════════════════════════════════════════════════════
//  Endstop_Init(&DDRlo.endstop_z, GPIOC, ENDSTOP_Z_PIN_Pin, 1);

  // ═══════════════════════════════════════════════════════════════════════
  // MAX ENDSTOPS — safety travel limits (PC0=X_MAX, PC1=Y_MAX, PC2=Z_MAX)
  // Wiring: one wire to pin, one to GND. is_nc=1 because switches are COM+NC wired
  // These stop motion if carriage overshoots the build area.
  // CubeMX: add PC0/PC1/PC2 as GPIO_Input, pull-up, labels below.
  // ═══════════════════════════════════════════════════════════════════════
//  Endstop_Init(&DDRlo.endstop_x_max, GPIOC, ENDSTOP_X_MAX_PIN_Pin, 1);
//  Endstop_Init(&DDRlo.endstop_y_max, GPIOC, ENDSTOP_Y_MAX_PIN_Pin, 1);
//  Endstop_Init(&DDRlo.endstop_z_max, GPIOC, ENDSTOP_Z_MAX_PIN_Pin, 1);
//  UARTDBG_Print("[BOOT] Max endstops initialized (X_MAX PC0, Y_MAX PC1, Z_MAX PC2)\r\n");

  // ═══════════════════════════════════════════════════════════════════════
  // ZE MOTION MODULE — must come after Stepper_Init for motor_z and motor_e
  // Enables both motors and zeros status structs.
  // ═══════════════════════════════════════════════════════════════════════
//  ZE_Init();

  // =====================================================================
  // STEP-COUNTER ISR REGISTRATION
  // Must come after all four Stepper_Init calls, before osKernelStart.
//  // =====================================================================
//  Stepper_RegisterTimer(&DDRlo.motor_a, &htim3,  TIM_CHANNEL_1);  /* Motor A */
//  Stepper_RegisterTimer(&DDRlo.motor_b, &htim3,  TIM_CHANNEL_2);  /* Motor B */
//  Stepper_RegisterTimer(&DDRlo.motor_z, &htim15, TIM_CHANNEL_1);  /* Motor Z */
//  Stepper_RegisterTimer(&DDRlo.motor_e, &htim15, TIM_CHANNEL_2);  /* Motor E */


  // =====================================================================
  // G-CODE DISPATCHER + SD PRINT LOOP
  // =====================================================================
//  GCode_Init();
//  SDPrint_Mount();   /* mount SD card — gracefully handles missing card */
  // ═══════════════════════════════════════════════════════════════════════
    // ENSURE INTERRUPTS ARE ENABLED
    // SDPrint_Mount (FATFS SPI diskio) leaves PRIMASK=1 on SD card absent/fail.
    // Force-clear before test menu so HAL_Delay works in all test functions.
    // ═══════════════════════════════════════════════════════════════════════
    __enable_irq();



  // ═══════════════════════════════════════════════════════════════════════
  // LOGGER — UART output on huart2 (PA2/PA3, 115200 baud)
  // ═══════════════════════════════════════════════════════════════════════
//  Logger_Init(&DDRlo.logger, NULL, &huart2, NULL, NULL, 0);
//  Logger_SetLevel(&DDRlo.logger, LOG_LEVEL_INFO);
//
//  // ═══════════════════════════════════════════════════════════════════════
//  // TEST MENU — runs BLOCKING before RTOS starts
//  // ═══════════════════════════════════════════════════════════════════════
//
//  /* Helper macro — receive one key blocking */
//  #define WAIT_KEY(ch) do { while(HAL_UART_Receive(&huart2,&(ch),1,200)!=HAL_OK); } while(0)
//
//  uint8_t choice = 0;
//  uint8_t sub    = 0;
//
//  /* ── MAIN MENU ── */
//main_menu:
//  UARTDBG_Print("\r\n\r\n");
//  UARTDBG_Print("╔════════════════════════════════════════════╗\r\n");
//  UARTDBG_Print("║   RAIDERFORGE TEST MENU  (STM32G0B1)       ║\r\n");
//  UARTDBG_Print("╚════════════════════════════════════════════╝\r\n");
//  UARTDBG_Print("  [1] Thermal tests\r\n");
//  UARTDBG_Print("  [2] Motor tests\r\n");
//  UARTDBG_Print("  [3] Endstop tests\r\n");
//  UARTDBG_Print("  [4] Homing tests\r\n");
//  UARTDBG_Print("  [5] Display / Encoder tests\r\n");
//  UARTDBG_Print("  [6] Parser / SD card tests\r\n");
//  UARTDBG_Print("  [7] Step counter\r\n");
//  UARTDBG_Print("  [N] Complete system bench\r\n");
//  UARTDBG_Print("  [S] Skip — go straight to RTOS\r\n");
//  UARTDBG_Print("\r\nEnter choice: ");
//
//  choice = 0;
//  WAIT_KEY(choice);
//  UARTDBG_Print("%c\r\n", choice);
//
//  switch (choice) {
//
//    /* ── THERMAL SUB-MENU ── */
//    case '1': {
//      UARTDBG_Print("\r\n  ── THERMAL ─────────────────────────────\r\n");
//      UARTDBG_Print("  [1] Thermistor only\r\n");
//      UARTDBG_Print("  [2] Fan only\r\n");
//      UARTDBG_Print("  [3] Logger only\r\n");
//      UARTDBG_Print("  [4] Heater PID\r\n");
//      UARTDBG_Print("  [5] Thermistor + Heater\r\n");
//      UARTDBG_Print("  [6] Heater + Fan\r\n");
//      UARTDBG_Print("  [7] Heater + Logger\r\n");
//      UARTDBG_Print("  [8] Fan + Logger\r\n");
//      UARTDBG_Print("  [9] All thermal integrated\r\n");
//      UARTDBG_Print("  [A] Bed heater\r\n");
//      UARTDBG_Print("  [B] Bed thermistor\r\n");
//      UARTDBG_Print("  [C] Fan cooldown\r\n");
//      UARTDBG_Print("  [D] Hotend manual diagnostic\r\n");
//      UARTDBG_Print("  [E] Bed manual diagnostic\r\n");
//      UARTDBG_Print("\r\nChoice: ");
//      sub = 0; WAIT_KEY(sub); UARTDBG_Print("%c\r\n", sub);
//      switch (sub) {
//        case '1': Test_Thermistor_Only();        break;
//        case '2': Test_Fan_Only();               break;
//        case '3': Test_Logger_Only();            break;
//        case '4': Test_Heater_Only();            break;
//        case '5': Test_Thermistor_And_Heater();  break;
//        case '6': Test_Heater_And_Fan();         break;
//        case '7': Test_Heater_And_Logger();      break;
//        case '8': Test_Fan_And_Logger();         break;
//        case '9': Test_All_Modules_Integrated(); break;
//        case 'A': case 'a': Test_Bed_Heater_Only();       break;
//        case 'B': case 'b': Test_Bed_Thermistor_Only();   break;
//        case 'C': case 'c': Test_Fan_Cooldown();          break;
//        case 'D': case 'd': Test_Manual_Diagnostic();     break;
//        case 'E': case 'e': Test_Bed_Manual_Diagnostic(); break;
//        default: UARTDBG_Print("  Invalid.\r\n"); break;
//      }
//      break;
//    }
//
//    /* ── MOTOR SUB-MENU ── */
//    case '2': {
//      UARTDBG_Print("\r\n  ── MOTORS ──────────────────────────────\r\n");
//      UARTDBG_Print("  [A] Motor A (interactive)\r\n");
//      UARTDBG_Print("  [B] Motor B (interactive)\r\n");
//      UARTDBG_Print("  [C] Motor Z (interactive)\r\n");
//      UARTDBG_Print("  [D] Motor E (interactive)\r\n");
//      UARTDBG_Print("  [E] All 4 motors synced\r\n");
//      UARTDBG_Print("  [F] CoreXY X/Y jog\r\n");
//      UARTDBG_Print("\r\nChoice: ");
//      sub = 0; WAIT_KEY(sub); UARTDBG_Print("%c\r\n", sub);
//      switch (sub) {
//        case 'A': case 'a': Test_Motor_A_Menu();    break;
//        case 'B': case 'b': Test_Motor_B_Menu();    break;
//        case 'C': case 'c': Test_Motor_Z_Menu();    break;
//        case 'D': case 'd': Test_Motor_E_Menu();    break;
//        case 'E': case 'e': Test_All_Motors_Menu(); break;
//        case 'F': case 'f': Test_CoreXY_Jog();      break;
//        default: UARTDBG_Print("  Invalid.\r\n"); break;
//      }
//      break;
//    }
//
//    /* ── ENDSTOP SUB-MENU ── */
//    case '3': {
//      UARTDBG_Print("\r\n  ── ENDSTOPS ────────────────────────────\r\n");
//      UARTDBG_Print("  [A] X MIN (PC3) — homing\r\n");
//      UARTDBG_Print("  [B] Y MIN (PC4) — homing\r\n");
//      UARTDBG_Print("  [C] Z MIN (PC5) — homing\r\n");
//      UARTDBG_Print("  [D] X MAX (PC0) — kill switch\r\n");
//      UARTDBG_Print("  [E] Y MAX (PC1) — kill switch\r\n");
//      UARTDBG_Print("  [F] Z MAX (PC2) — kill switch\r\n");
//      UARTDBG_Print("\r\nChoice: ");
//      sub = 0; WAIT_KEY(sub); UARTDBG_Print("%c\r\n", sub);
//      switch (sub) {
//        case 'A': case 'a': Test_Endstop_X_Min(); break;
//        case 'B': case 'b': Test_Endstop_Y_Min(); break;
//        case 'C': case 'c': Test_Endstop_Z_Min(); break;
//        case 'D': case 'd': Test_Endstop_X_Max(); break;
//        case 'E': case 'e': Test_Endstop_Y_Max(); break;
//        case 'F': case 'f': Test_Endstop_Z_Max(); break;
//        default: UARTDBG_Print("  Invalid.\r\n"); break;
//      }
//      break;
//    }
//
//    /* ── HOMING SUB-MENU ── */
//    case '4': {
//      UARTDBG_Print("\r\n  ── HOMING ──────────────────────────────\r\n");
//      UARTDBG_Print("  [A] Home XY\r\n");
//      UARTDBG_Print("  [B] Home Z\r\n");
//      UARTDBG_Print("  [C] Full homing (XY + Z)\r\n");
//      UARTDBG_Print("  [D] Full motion sequence\r\n");
//      UARTDBG_Print("\r\nChoice: ");
//      sub = 0; WAIT_KEY(sub); UARTDBG_Print("%c\r\n", sub);
//      switch (sub) {
//        case 'A': case 'a': Test_Homing_XY();    break;
//        case 'B': case 'b': Test_Homing_Z();     break;
//        case 'C': case 'c': Test_Homing_Full();  break;
//        case 'D': case 'd': Test_Motion_Full();  break;
//        default: UARTDBG_Print("  Invalid.\r\n"); break;
//      }
//      break;
//    }
//
//    /* ── DISPLAY / ENCODER SUB-MENU ── */
//    case '5': {
//      UARTDBG_Print("\r\n  ── DISPLAY / ENCODER ───────────────────\r\n");
//      UARTDBG_Print("  [A] Boot screen\r\n");
//      UARTDBG_Print("  [B] Telemetry (live values)\r\n");
//      UARTDBG_Print("  [C] Menu screen\r\n");
//      UARTDBG_Print("  [D] Encoder full test\r\n");
//      UARTDBG_Print("\r\nChoice: ");
//      sub = 0; WAIT_KEY(sub); UARTDBG_Print("%c\r\n", sub);
//      switch (sub) {
//        case 'A': case 'a': Test_Display_Boot();      break;
//        case 'B': case 'b': Test_Display_Telemetry(); break;
//        case 'C': case 'c': Test_Display_Menu();      break;
//        case 'D': case 'd': Test_Encoder_Full();      break;
//        default: UARTDBG_Print("  Invalid.\r\n"); break;
//      }
//      break;
//    }
//
//    /* ── PARSER / SD SUB-MENU ── */
//    case '6': {
//      UARTDBG_Print("\r\n  ── PARSER / SD ─────────────────────────\r\n");
//      UARTDBG_Print("  [A] Parser G-code test\r\n");
//      UARTDBG_Print("  [B] SD card mount + list\r\n");
//      UARTDBG_Print("  [C] G-code dispatcher test bench\r\n");
//      UARTDBG_Print("  [D] SD print loop test bench\r\n");
//      UARTDBG_Print("\r\nChoice: ");
//      sub = 0; WAIT_KEY(sub); UARTDBG_Print("%c\r\n", sub);
//      switch (sub) {
//        case 'A': case 'a': Test_Parser_Individual();  break;
//        case 'B': case 'b': Test_SDCard_Individual();  break;
//        case 'C': case 'c': Test_GCodeDispatcher();    break;
//        case 'D': case 'd': Test_SDPrintLoop();         break;
//        default: UARTDBG_Print("  Invalid.\r\n"); break;
//      }
//      break;
//    }
//
//    /* ── STEP COUNTER ── */
//    case '7': Test_StepCounter(); break;
//
//    /* ── SYSTEM BENCH ── */
//    case 'N': case 'n': SystemBench_RunAll(); break;
//
//    /* ── SKIP ── */
//    case 'S': case 's': UARTDBG_Print("Skipping tests.\r\n"); break;
//
//    default: UARTDBG_Print("Invalid — skipping.\r\n"); break;
//  }
//
//  UARTDBG_Print("\r\nTest phase complete. Starting RTOS...\r\n");
//  UARTDBG_Print("\r\n");
//  UARTDBG_Print("╔════════════════════════════════════════════════════════╗\r\n");
//  UARTDBG_Print("║         RAIDERFORGE RTOS — ALL TASKS RUNNING           ║\r\n");
//  UARTDBG_Print("║                                                        ║\r\n");
//  UARTDBG_Print("║  HeaterTask   10 Hz  PID + fan auto-control            ║\r\n");
//  UARTDBG_Print("║  MotionTask  200 Hz  endstops + homing + CoreXY        ║\r\n");
//  UARTDBG_Print("║  DisplayTask  20 Hz  screen                            ║\r\n");
//  UARTDBG_Print("║  LED1          1 Hz  heartbeat blink                   ║\r\n");
//  UARTDBG_Print("║                                                        ║\r\n");
//  UARTDBG_Print("║  [HH:MM:SS] HE:temp/target BED:temp/target FAN STATE   ║\r\n");
//  UARTDBG_Print("╚════════════════════════════════════════════════════════╝\r\n");
//  UARTDBG_Print("\r\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  uart_mutex_id = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  if (xBedReadySem == NULL) {
    xBedReadySem = xSemaphoreCreateBinary();
  }
  if (xHotendReadySem == NULL) {
    xHotendReadySem = xSemaphoreCreateBinary();
  }
  if (xFanReadySem == NULL) {
    xFanReadySem = xSemaphoreCreateBinary();
  }
  if (xMotionDoneSem == NULL) {
    xMotionDoneSem = xSemaphoreCreateBinary();
  }
  if (xHomingDoneSem == NULL) {
    xHomingDoneSem = xSemaphoreCreateBinary();
  }

  configASSERT(xBedReadySem    != NULL);
  configASSERT(xHotendReadySem != NULL);
  configASSERT(xFanReadySem    != NULL);
  configASSERT(xMotionDoneSem  != NULL);
  configASSERT(xHomingDoneSem  != NULL);

  if (xSDStateMutex == NULL) {
    xSDStateMutex = xSemaphoreCreateMutex();
  }
  if (xGArgsMutex == NULL) {
    xGArgsMutex = xSemaphoreCreateMutex();
  }
  configASSERT(xSDStateMutex != NULL);
  configASSERT(xGArgsMutex != NULL);
  if (xMotionStateMutex == NULL) {
    xMotionStateMutex = xSemaphoreCreateMutex();
  }
  if (xMotionUpdateMutex == NULL) {
    xMotionUpdateMutex = xSemaphoreCreateBinary();
  }
  if (xHeaterStateMutex == NULL) {
    xHeaterStateMutex = xSemaphoreCreateMutex();
  }
  if (xFanStateMutex == NULL) {
    xFanStateMutex = xSemaphoreCreateMutex();
  }
  if (xHeaterUpdateMutex == NULL) {
    xHeaterUpdateMutex = xSemaphoreCreateMutex();
  }
  configASSERT(xMotionStateMutex != NULL);
  configASSERT(xMotionUpdateMutex != NULL);
  configASSERT(xHeaterStateMutex != NULL);
  configASSERT(xFanStateMutex != NULL);
  configASSERT(xHeaterUpdateMutex != NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  if (xMotionQueue == NULL) {
    xMotionQueue = xQueueCreate(1U,  sizeof(MotionCommand_t));
  }
  if (xSDCardQueue == NULL) {
    xSDCardQueue = xQueueCreate(20U,  128U);
  }
  if (xGCodeQueue == NULL) {
    xGCodeQueue  = xQueueCreate(20U, 128U);
  }
  if (xHotendQueue == NULL) {
    xHotendQueue = xQueueCreate(1U, sizeof(Hotend_Command_t));
  }
  if (xBedQueue == NULL) {
    xBedQueue = xQueueCreate(1U, sizeof(Bed_Command_t));
  }
  if (xFanQueue == NULL) {
    xFanQueue = xQueueCreate(4U, sizeof(Fan_Command_t));
  }

  configASSERT(xMotionQueue != NULL);
  configASSERT(xSDCardQueue != NULL);
  configASSERT(xGCodeQueue  != NULL);
  configASSERT(xHotendQueue != NULL);
  configASSERT(xBedQueue != NULL);
  configASSERT(xFanQueue != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LED1 */
  LED1Handle = osThreadNew(StartLED1, NULL, &LED1_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(StartDisplay, NULL, &Display_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  RaiderForge_StartNormalRTOSTasks();

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */
  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */
  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 15;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */
  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */
  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 79;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */
  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */
  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 79;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */
  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|DIR_B_PIN_Pin|EN_B_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_PIN_Pin|EN_PIN_Pin|HOTEND_FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Z_DIR_PIN_Pin|Z_EN_PIN_Pin|E_DIR_PIN_Pin|E_EN_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Screen_CS_GPIO_Port, Screen_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Encoder_Button_Pin ENDSTOP_X_MAX_PIN_Pin ENDSTOP_Y_MAX_PIN_Pin ENDSTOP_Z_MAX_PIN_Pin
                           ENDSTOP_X_PIN_Pin ENDSTOP_Y_PIN_Pin ENDSTOP_Z_PIN_Pin */
  GPIO_InitStruct.Pin = Encoder_Button_Pin|ENDSTOP_X_MAX_PIN_Pin|ENDSTOP_Y_MAX_PIN_Pin|ENDSTOP_Z_MAX_PIN_Pin
                          |ENDSTOP_X_PIN_Pin|ENDSTOP_Y_PIN_Pin|ENDSTOP_Z_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_PIN_Pin EN_PIN_Pin HOTEND_FAN_Pin */
  GPIO_InitStruct.Pin = DIR_PIN_Pin|EN_PIN_Pin|HOTEND_FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_B_PIN_Pin EN_B_PIN_Pin */
  GPIO_InitStruct.Pin = DIR_B_PIN_Pin|EN_B_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Z_DIR_PIN_Pin Z_EN_PIN_Pin E_DIR_PIN_Pin E_EN_PIN_Pin */
  GPIO_InitStruct.Pin = Z_DIR_PIN_Pin|Z_EN_PIN_Pin|E_DIR_PIN_Pin|E_EN_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Screen_CS_Pin */
  GPIO_InitStruct.Pin = Screen_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Screen_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief HeaterTask — 10 Hz
  *
  * Runs nozzle PID (Heater_Update) and bed PID (BedHeater_Update) every 100 ms.
  * Also manages hotend fan auto-on based on nozzle temperature.
  *
  * Priority AboveNormal — thermal safety must stay responsive even if motion
  * is busy computing a long move.
  *
  * Fan hysteresis: ON at >= 50°C, OFF at < 45°C. Prevents relay-chatter if
  * the sensor is noisy right at the threshold.
  */
//void StartHeaterTask(void *argument)
//{
//  static uint32_t loop_count = 0;
//
//  for (;;)
//  {
//    Heater_Update(&DDRlo.hotend);
//    BedHeater_Update();
//
//    float hotend_temp   = Heater_GetTemp(&DDRlo.hotend);
//    float hotend_target = Heater_GetTarget(&DDRlo.hotend);
//    float bed_temp      = BedHeater_GetTemp();
//    uint8_t hotend_pwm  = Heater_GetPWM(&DDRlo.hotend);
//    uint8_t bed_pwm     = BedHeater_GetPWM();
//
//    // Fan auto-control with hysteresis
//    if      (hotend_temp >= 50.0f && !Fan_IsRunning(&DDRlo.hotend_fan)) {
//      Fan_On(&DDRlo.hotend_fan);
//      TASK_PRINT("[HEATER] Hotend fan ON (temp >= 50 C)\r\n");
//    }
//    else if (hotend_temp <  45.0f &&  Fan_IsRunning(&DDRlo.hotend_fan)) {
//      Fan_Off(&DDRlo.hotend_fan);
//      TASK_PRINT("[HEATER] Hotend fan OFF (temp < 45 C)\r\n");
//    }
//
//    loop_count++;
//
//    // Print status every 2 seconds (200 loops at 10 Hz)
//    if (loop_count >= 20) {
//      loop_count = 0;
//      uint32_t uptime_s = HAL_GetTick() / 1000;
//
//      // Integer math for floats (no -u _printf_float needed)
//      int he_w = (int)hotend_temp;
//      int he_f = (int)((hotend_temp - he_w) * 10);
//      int ht_w = (int)hotend_target;
//      int be_w = (int)bed_temp;
//      int be_f = (int)((bed_temp - be_w) * 10);
//
//      TASK_PRINT("[%3lu:%02lu:%02lu] "
//                    "HE:%d.%dC/%dC PWM:%u%%  "
//                    "BED:%d.%dC/%dC PWM:%u%%  "
//                    "FAN:%s  "
//                    "STATE:%d\r\n",
//                    uptime_s / 3600,
//                    (uptime_s % 3600) / 60,
//                    uptime_s % 60,
//                    he_w, he_f < 0 ? -he_f : he_f, ht_w, hotend_pwm,
//                    be_w, be_f < 0 ? -be_f : be_f, (int)BedHeater_GetTarget(), bed_pwm,
//                    Fan_IsRunning(&DDRlo.hotend_fan) ? "ON " : "OFF",
//                    (int)DDRlo.state);
//    }
//
//    osDelay(100);  // 10 Hz
//  }
//}

/**
  * @brief MotionTask — 200 Hz
  *
  * Polls both endstops, advances homing state machine, runs CoreXY step loop.
  * Runs at 200 Hz (5 ms period) so the non-blocking homing settle timers and
  * step pulse generation stay accurate.
  *
  * Priority Normal — yields to HeaterTask if both are ready simultaneously,
  * which is the safe default for a printer (thermal before motion).
  */
//void StartMotionTask(void *argument)
//{
//  static uint32_t motion_loops = 0;
//  static uint8_t  last_ex = 2, last_ey = 2;   // 2 = unknown, forces first print
//  static uint32_t last_homing_state = 99;
//
//  for (;;)
//  {
//    Endstop_Update(&DDRlo.endstop_x);
//    Endstop_Update(&DDRlo.endstop_y);
//    Endstop_Update(&DDRlo.endstop_z);
//    Endstop_Update(&DDRlo.endstop_x_max);
//    Endstop_Update(&DDRlo.endstop_y_max);
//    Endstop_Update(&DDRlo.endstop_z_max);
//    Homing_Update(&DDRlo.homing, &GArgs);
//    CoreXY_Task(&DDRlo.corexy);
//    ZE_Task();
//
//    // ── MAX ENDSTOP SAFETY: kill switch behaviour ──────────────────────
//    // MIN endstops = limit/homing switches (define position 0)
//    // MAX endstops = kill switches (emergency stop if hit)
//    // If homing and MAX triggered → carriage going wrong way → abort
//    if (DDRlo.homing.in_progress) {
//      if (Endstop_IsTriggered(&DDRlo.endstop_x_max) ||
//          Endstop_IsTriggered(&DDRlo.endstop_y_max)) {
//        Homing_Abort(&DDRlo.homing);
//        TASK_PRINT("[MOTION] MAX endstop during homing — ABORTED\r\n");
//      }
//    }
//    // ── MAX ENDSTOP SAFETY: emergency stop if travel limit hit ──────────
//    if (DDRlo.corexy.is_moving || DDRlo.corexy.is_jogging) {
//      if (Endstop_IsTriggered(&DDRlo.endstop_x_max)) {
//        CoreXY_EmergencyStop(&DDRlo.corexy);
//        TASK_PRINT("[MOTION] X_MAX TRIGGERED — emergency stop!\r\n");
//      }
//      if (Endstop_IsTriggered(&DDRlo.endstop_y_max)) {
//        CoreXY_EmergencyStop(&DDRlo.corexy);
//        TASK_PRINT("[MOTION] Y_MAX TRIGGERED — emergency stop!\r\n");
//      }
//    }
//    if (Z_IsMoving() && Endstop_IsTriggeredImmediate(&DDRlo.endstop_z)) {
//      Z_Stop();
//      TASK_PRINT("[MOTION] Z bottom endstop triggered — Z emergency stop!\r\n");
//    }
//
//    uint8_t ex = Endstop_IsTriggered(&DDRlo.endstop_x);
//    uint8_t ey = Endstop_IsTriggered(&DDRlo.endstop_y);
//
//    // Print on endstop state change
//    if (ex != last_ex || ey != last_ey) {
//      last_ex = ex;
//      last_ey = ey;
//      TASK_PRINT("[MOTION] Endstop X:%s  Y:%s\r\n",
//                    ex ? "TRIGGERED" : "open",
//                    ey ? "TRIGGERED" : "open");
//    }
//
//    // Print on homing state change
//    if ((uint32_t)DDRlo.homing.state != last_homing_state) {
//      last_homing_state = (uint32_t)DDRlo.homing.state;
//      const char *hstate[] = {
//        "IDLE", "FAST_X", "BACK_X", "SLOW_X",
//        "FAST_Y", "BACK_Y", "SLOW_Y", "SETTLE", "COMPLETE", "TIMEOUT"
//      };
//      uint8_t si = (uint8_t)DDRlo.homing.state;
//      TASK_PRINT("[MOTION] Homing state -> %s\r\n",
//                    si < 10 ? hstate[si] : "UNKNOWN");
//    }
//
//    motion_loops++;
//
//    // Print position every 4 seconds (800 loops at 200 Hz)
//    if (motion_loops >= 800) {
//      motion_loops = 0;
//      int px = (int)DDRlo.corexy.pos_x;
//      int py = (int)DDRlo.corexy.pos_y;
//      TASK_PRINT("[MOTION] Pos X:%dmm Y:%dmm  Homed:%s\r\n",
//                    px, py,
//                    DDRlo.homing.state == HOMING_COMPLETE ? "YES" : "NO");
//    }
//
//    SDPrint_Task();   /* advance SD print state machine one line per tick */
//    osDelay(5);   // 200 Hz
//  }
//}

void RaiderForge_StartNormalRTOSTasks(void)
{
  static uint8_t normal_tasks_started = 0U;

  if (normal_tasks_started) {
    UARTDBG_Print("[main.c]: Normal RTOS print tasks already running\r\n");
    return;
  }

  HeaterTaskHandle = osThreadNew(StartHeaterTask, NULL, &HeaterTask_attributes);
  if (HeaterTaskHandle == NULL) {
    UARTDBG_Print("[main.c]: ERROR: Heater task create failed\r\n");
    configASSERT(HeaterTaskHandle != NULL);
  }
  BedHeaterTaskHandle = osThreadNew(StartBedHeaterTask, NULL, &BedHeaterTask_attributes);
  if (BedHeaterTaskHandle == NULL) {
    UARTDBG_Print("[main.c]: ERROR: BedHeater task create failed\r\n");
    configASSERT(BedHeaterTaskHandle != NULL);
  }
  PartFanTaskHandle = osThreadNew(StartPartFanTask, NULL, &PartFanTask_attributes);
  if (PartFanTaskHandle == NULL) {
    UARTDBG_Print("[main.c]: ERROR: PartFan task create failed\r\n");
    configASSERT(PartFanTaskHandle != NULL);
  }
  MotionTaskHandle = osThreadNew(StartMotionTask, NULL, &MotionTask_attributes);
  if (MotionTaskHandle == NULL) {
    UARTDBG_Print("[main.c]: ERROR: Motion task create failed\r\n");
    configASSERT(MotionTaskHandle != NULL);
  }
  SDCardTaskHandle = osThreadNew(StartSDCardTask, NULL, &SDCardTask_attributes);
  if (SDCardTaskHandle == NULL) {
    UARTDBG_Print("[main.c]: ERROR: SDCard task create failed\r\n");
    configASSERT(SDCardTaskHandle != NULL);
  }
  ParserTaskHandle = osThreadNew(StartParserTask, NULL, &ParserTask_attributes);
  if (ParserTaskHandle == NULL) {
    UARTDBG_Print("[main.c]: ERROR: Parser task create failed\r\n");
    configASSERT(ParserTaskHandle != NULL);
  }

  normal_tasks_started = 1U;
  UARTDBG_Print("[main.c]: Normal RTOS print tasks started\r\n");
}

void StartSDCardTask(void *argument)
{
  SDCard_Task(argument);
}

void StartHeaterTask(void *argument)
{
  Hotend_Task(argument);
}

void StartBedHeaterTask(void *argument)
{
  BedHeater_InitDefault();
  BedHeater_ModuleInit();
  BedHeater_Task(argument);
}

void StartPartFanTask(void *argument)
{
  PartFan_Task(argument);
}

void StartMotionTask(void *argument)
{
  Motion_Task(argument);
}

void StartParserTask(void *argument)
{
  Parser_task(argument);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLED1 */
/**
  * @brief LED1 task — placeholder, blink if needed.
  */
/* USER CODE END Header_StartLED1 */
void StartLED1(void *argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;

  // 5 rapid blinks = RTOS scheduler confirmed running
  // (Banner already printed pre-RTOS to avoid UART contention)
  for (int i = 0; i < 5; i++) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    osDelay(80);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    osDelay(80);
  }
  osDelay(400);

  for (;;)
  {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplay */
/**
  * @brief Display task — Matthew's code goes inside USER CODE BEGIN StartDisplay.
  *        DO NOT touch the function signature or remove the for(;;)/osDelay.
  */
/* USER CODE END Header_StartDisplay */
void StartDisplay(void *argument)
{
  /* USER CODE BEGIN StartDisplay */
  (void)argument;
//  static display_state_t disp_state = DISP_BOOT;
//  static uint32_t boot_time = 0;
//
//  /* Show boot screen for 2 seconds then switch to telemetry */
//  init_display();
//  boot_display();
//  boot_time = HAL_GetTick();
//
//  for (;;)
//  {
//    /* After 2 seconds switch from boot screen to telemetry */
//    if (disp_state == DISP_BOOT && (HAL_GetTick() - boot_time) > 2000) {
//      disp_state = DISP_TELE;
//    }
//
//    /* Check encoder button to toggle between telemetry and menu */
//    if (HAL_GPIO_ReadPin(Encoder_Button_GPIO_Port, Encoder_Button_Pin) == GPIO_PIN_RESET) {
//      osDelay(200);  /* debounce */
//      if (disp_state == DISP_TELE) disp_state = DISP_MENU;
//      else if (disp_state == DISP_MENU) disp_state = DISP_TELE;
//    }
//
//    /* Update encoder in menu mode */
//    if (disp_state == DISP_MENU) {
//      menu_update_from_encoder(&htim2);
//    }
//
//    /* Render current screen */
//    switch (disp_state) {
//      case DISP_BOOT:
//        boot_display();
//        break;
//      case DISP_TELE:
//        telemetry_display();
//        break;
//      case DISP_MENU:
//        menu_display();
//        break;
//      default:
//        telemetry_display();
//        break;
//    }
//
//    osDelay(50);  /* 20 Hz */
//  }

  for (;;)
  {
    osDelay(50);
  }
  /* USER CODE END StartDisplay */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
