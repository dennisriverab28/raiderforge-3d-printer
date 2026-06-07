/*
 * printer_config.h
 *
 * Central configuration for RaiderForge CoreXY.
 * Updated: added Z axis and extruder (E) constants.
 *
 * HOW TO SET STEPS/MM:
 *   Z (leadscrew):
 *     steps_per_mm = (motor_steps x microstepping) / leadscrew_pitch_mm
 *     Example: 200 steps x 16 microsteps / 2mm pitch = 1600 steps/mm
 *
 *   E (extruder):
 *     Calibrate with Mark-and-Measure method (see comment below).
 */

#ifndef PRINTER_CONFIG_H
#define PRINTER_CONFIG_H

#include "stm32g0xx_hal.h"
#include "heater.h"
#include "qbc_globals.h"
//#include "ddr_globals.h"

/* XY MOTION */
#define STEPS_PER_MM_XY         94.65f   /* 20445 steps / 216mm measured */
#define BUILD_SIZE_X            216.0f   /* measured endstop to endstop */
#define BUILD_SIZE_Y            216.0f   /* measured endstop to endstop */
#define MAX_SPEED_XY            150.0f
#define JOG_SPEED_XY            20.0f
#define MAX_ACCEL_XY            1000.0f
#define HOMING_SPEED_XY         30.0f
#define HOME_DIR_X              (-1)
#define HOME_DIR_Y              (-1)
#define MOTOR_STEPS_PER_REV     200
#define MOTOR_EN_ACTIVE_LOW     1
#define MOVE_COMPLETE_BUFFER_MS 50

/* Z AXIS */
#define STEPS_PER_MM_Z          1161.25f
#define BUILD_SIZE_Z            200.0f
#define Z_MAX_MM                BUILD_SIZE_Z
#define Z_FEEDRATE_MM_MIN       240.0f
#define Z_HOMING_FAST_MM_MIN    450.0f
#define Z_HOMING_SLOW_MM_MIN    45.0f
#define Z_HOME_BACKOFF_MM       5.0f
#define Z_HOME_TIMEOUT_MS       60000
#define Z_DIR_UP                1
#define Z_DIR_DOWN              0
#define Z_DIR_PORT              GPIOC
#define Z_DIR_PIN               GPIO_PIN_6
#define Z_EN_PORT               GPIOC
#define Z_EN_PIN                GPIO_PIN_7

/* EXTRUDER (E) */
#define STEPS_PER_MM_E          139.5f
#define E_FEEDRATE_MIN_MM_MIN   60.0f
#define E_FEEDRATE_MAX_MM_MIN   600.0f
#define E_RETRACT_MAX_MM_MIN    3600.0f
#define E_MIN_STEP_HZ_EXTRUDE   1U      /* Tiny slicer segments must not be forced to 1 kHz. */
#define E_MIN_STEP_HZ_RETRACT   1U
#define E_RETRACT_LENGTH_PLA    0.8f
#define E_RETRACT_LENGTH_PETG   1.5f
#define E_RETRACT_SPEED_MM_MIN  1800.0f
#define E_RECOVER_SPEED_MM_MIN  600.0f
#define E_DIR_EXTRUDE           0
#define E_DIR_RETRACT           1
#define E_DIR_PORT              GPIOC
#define E_DIR_PIN               GPIO_PIN_8
#define E_EN_PORT               GPIOC
#define E_EN_PIN                GPIO_PIN_9

/* HOTEND */
#define HOTEND_MAX_TEMP         300.0f
#define HOTEND_MIN_TEMP        -10.0f
#define HOTEND_FAN_ON_TEMP      50.0f
#define HOTEND_HEATER_PWM_FREQ_HZ 50U
#define HOTEND_KP               8.0f
#define HOTEND_KI               0.3f
#define HOTEND_KD               50.0f

/* BED */
#define BED_MAX_TEMP            120.0f
#define BED_MIN_TEMP           -10.0f
#define BED_HEATER_PWM_FREQ_HZ  20U
#define BED_KP                  6.0f
#define BED_KI                  0.22f
#define BED_KD                  4.0f

/* CONVENIENCE GETTERS — all read from DDRlo */
//static inline float Printer_GetNozzleTemp(void)     { return Heater_GetTemp(&DDRlo.hotend); }
//static inline float Printer_GetNozzleTarget(void)   { return Heater_GetTarget(&DDRlo.hotend); }
//static inline uint8_t Printer_GetNozzlePWM(void)    { return Heater_GetPWM(&DDRlo.hotend); }
//static inline uint8_t Printer_NozzleAtTarget(float t){ return Heater_AtTarget(&DDRlo.hotend, t); }
//static inline uint8_t Printer_NozzleHasFault(void)  { return Heater_HasFault(&DDRlo.hotend); }
//
//static inline void Printer_SplitTemp(float temp, int *whole, int *frac)
//{
//    *whole = (int)temp;
//    float r = temp - (float)(*whole);
//    if (r < 0.0f) r = -r;
//    *frac = (int)(r * 10.0f);
//}
//
#endif /* PRINTER_CONFIG_H */
