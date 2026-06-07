/*
 * bed_heater.h
 * PA7 → TIM17 CH1 | PB10 → ADC1_IN10
 * Created: Feb 2026 | Author: Cherryman125
 */

#ifndef BED_HEATER_H
#define BED_HEATER_H

#include "stm32g0xx_hal.h"
#include "heater.h"
#include "bed_thermistor.h"
#include "printer_config.h"
#include <stdint.h>

#define BED_PWM_TIMER    htim4
#define BED_PWM_CHANNEL  TIM_CHANNEL_2
#define BED_ADC_HANDLE   hadc1

extern TIM_HandleTypeDef  htim4;   /* PB7 TIM4_CH2 — PA7 freed for SPI1_MOSI */
extern ADC_HandleTypeDef  hadc1;

// Init
void BedHeater_ModuleInit(void);
void BedHeater_Init(TIM_HandleTypeDef *pwm_timer,
                    uint32_t           pwm_channel,
                    ADC_HandleTypeDef *adc);
void BedHeater_InitDefault(void);   // uses BED_PWM_TIMER / BED_ADC_HANDLE

// Temperature
float   BedHeater_ReadTemp(void);   // fresh ADC read
float   BedHeater_GetTemp(void);    // cached value

// PID — call at 10 Hz
void    BedHeater_Task(void *pvParameters);
void    BedHeater_Update(void);
void    BedHeater_TaskStep(void);

// Control
void    BedHeater_SetTarget(float temp_c);
void    BedHeater_Enable(uint8_t enable);
float   BedHeater_GetTarget(void);







// Faults
uint8_t        BedHeater_HasFault(void);
Heater_Fault_t BedHeater_GetFaultLevel(void);
void           BedHeater_ClearFault(void);
uint32_t       BedHeater_GetDebugFlags(void);
uint8_t        BedHeater_IsSteadyState(void);

// Manual / diagnostic
void      BedHeater_SetPWM(uint8_t percent);
Heater_t *BedHeater_GetHandle(void);
void      BedHeater_GetDiagnostics(Heater_Diagnostics_t *diag);

#endif /* BED_HEATER_H */
