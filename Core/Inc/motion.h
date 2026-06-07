/*
 * motion.h
 *
 *  Created on: Apr 13, 2026
 *      Author: Quinton B. Cook
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include "stm32g0xx_hal.h"
#include "stepper.h"
#include "qbc_globals.h"
#include "motion_types.h"
#include "parser.h"
#include <stdint.h>

typedef struct {

    uint8_t is_moving;
    uint8_t is_jogging;
    uint32_t target_steps;
    uint32_t move_start_time;

} Motor_Status_t;

typedef struct {

	float target_mm;
	uint8_t is_homed;

} Axis_Status_t;

typedef struct {

    float a_steps_per_mm;
    float b_steps_per_mm;
    float z_steps_per_mm;
    float e_steps_per_mm;

    float max_speed_mm_s;

    float x_max_dist;
    float y_max_dist;
    float z_max_dist;

} Config_t;

typedef struct {
    uint8_t has_e_word;
    uint8_t e_enabled;
    uint8_t synchronized_with_xyz;
    uint8_t frequency_limited;
    uint8_t direction;

    float pos_e;
    float target_e;
    float delta_e_mm;
    float scaled_delta_e_mm;
    float flowrate;
    float residual_before;
    float residual_after;

    float xy_distance_mm;
    float z_distance_mm;
    float move_time_s;
    float feedrate_mm_s;
    float requested_step_hz;
    float step_delay_us;

    int32_t signed_steps;
    uint32_t abs_steps;
    uint32_t step_hz;
} Motion_EPlan_t;

typedef struct {

	uint32_t Motion_Head;

    Stepper_t *motor_a;
    Stepper_t *motor_b;
    Stepper_t *motor_z;
    Stepper_t *motor_e;
    Motor_Status_t a_status;
    Motor_Status_t b_status;
    Motor_Status_t z_status;
    Motor_Status_t e_status;
    Axis_Status_t x_axis_status;
    Axis_Status_t y_axis_status;
    Axis_Status_t z_axis_status;
    Motion_States_t state;
    Config_t *config;
    float pos_x;
    float pos_y;
    float pos_z;
    float pos_e;
    float e_step_residual;
    float target_x;
    float target_y;
    float target_e;
    float current_speed;
    float cxy_move_distance;
    float z_move_distance;
    float e_move_distance;
    uint8_t motion_finished;
    uint8_t homing_finished;
    uint32_t move_start_time;

    uint32_t Motion_Tail;

} Motion_t;

// Motion Initializers
void Motion_Config_Init(Config_t *config);
void Status_Init(Motor_Status_t *status);
void Motors_Init(Motion_t *motion, Config_t *config);

// Motion Mutators
void Motion_SetState(Motion_t *motion, Motion_States_t newState);
Motion_States_t Motion_GetState(Motion_t *motion);

// Regular Motion Functions
float clamp_f(float val, float min_val, float max_val);
uint32_t steps_per_sec(float mm_per_min, float steps_per_mm);
uint32_t steps_from_distance(float distance_mm, float steps_per_mm);
void Motion_EvaluateExtruderPlan(const Motion_t *motion,
                                  const MotionCommand_t *cmd,
                                  float target_x,
                                  float target_y,
                                  float target_z,
                                  float target_e,
                                  float speed_mm_s,
                                  Motion_EPlan_t *plan);
void Motion_Move(Motion_t *motion, MotionCommand_t *cmd);
void Motion_Stop(Motion_t *motion);
void Motion_Emergency_Stop(Motion_t *motion);

// Jogging Functions
void Motion_JogX(Motion_t *motion, int8_t direction, float speed_mm_s);
void Motion_JogY(Motion_t *motion, int8_t direction, float speed_mm_s);
void Motion_JogA(Motion_t *motion, int8_t direction, float speed_mm_s);
void Motion_JogB(Motion_t *motion, int8_t direction, float speed_mm_s);
void Motion_JogZ(Motion_t *motion, int8_t direction, float speed_mm_s);
void Motion_JogStop(Motion_t *motion);

// Motion Task Setup
void Motion_Task(void *pvParameters);






#endif /* INC_MOTION_H_ */
