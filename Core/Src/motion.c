/*
 * motion.c
 *
 *  Created on: Apr 13, 2026
 *      Author: Quinton B. Cook
 */
#define MOTION

#include <qbc_globals.h>
#include "printer_config.h"
#include "homing.h"
#include "uart_debug.h"
#include <math.h>
#include <main.h>
#include <stdlib.h>
#include <string.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim17;

void Motion_Config_Init(Config_t *config){

	config->a_steps_per_mm = STEPS_PER_MM_XY;
	config->b_steps_per_mm = STEPS_PER_MM_XY;
	config->z_steps_per_mm = STEPS_PER_MM_Z;
	config->e_steps_per_mm = STEPS_PER_MM_E;

	config->max_speed_mm_s = MAX_SPEED_XY;


    config->x_max_dist = 200;
    config->y_max_dist = 200;
    config->z_max_dist = Z_MAX_MM;
}

void Status_Init(Motor_Status_t *status)
{
	status->is_moving = 0;
	status->is_jogging = 0;
	status->target_steps = 0;
	status->move_start_time = 0;
}

// Stepper_Init Has to be called before Motors_Init
void Motors_Init(Motion_t *motion, Config_t *config)
{
	motion->Motion_Head = 0xDEADBEEF;
	motion->Motion_Tail = 0xDEADBEEF;

	motion->config = config;
	motion->current_speed = 0;
	motion->cxy_move_distance = 0;
	motion->z_move_distance = 0;
	motion->e_move_distance = 0;
	motion->e_step_residual = 0.0f;
	motion->move_start_time = 0;
	motion->state = MOTION_INIT;

	Status_Init(&motion->a_status);
	Status_Init(&motion->b_status);
	Status_Init(&motion->z_status);
	Status_Init(&motion->e_status);

	Stepper_Enable(motion->motor_a, 1);
	Stepper_Enable(motion->motor_b, 1);
	Stepper_Enable(motion->motor_z, 1);
	Stepper_Enable(motion->motor_e, 1);

	Stepper_RegisterTimer(motion->motor_a, &htim3,  TIM_CHANNEL_1);  /* Motor A on PB4 / TIM3_CH1 */
	Stepper_RegisterTimer(motion->motor_b, &htim3,  TIM_CHANNEL_2);  /* Motor B on PB5 / TIM3_CH2 */
	Stepper_RegisterTimer(motion->motor_z, &htim15, TIM_CHANNEL_1);  /* Motor Z */
	Stepper_RegisterTimer(motion->motor_e, &htim17, TIM_CHANNEL_1);  /* Motor E */

}

float clamp_f(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

uint32_t steps_per_sec(float mm_per_min, float steps_per_mm)
{
    if (mm_per_min <= 0.0f) return 0;

    float sps = (mm_per_min / 60.0f) * steps_per_mm;
    if (sps < 10.0f) sps = 10.0f;
    if (sps > 50000.0f) sps = 50000.0f;
    return (uint32_t)sps;
}

uint32_t steps_from_distance(float distance_mm, float steps_per_mm)
{
    if (distance_mm <= 0.0f || steps_per_mm <= 0.0f) return 0;
    return (uint32_t)(distance_mm * steps_per_mm + 0.5f);
}

static uint32_t rounded_hz(float hz)
{
    if (hz <= 0.0f) {
        return 0U;
    }

    if (hz > 50000.0f) {
        return 50000U;
    }

    uint32_t rounded = (uint32_t)(hz + 0.5f);
    return (rounded == 0U) ? 1U : rounded;
}

static uint32_t clamp_e_freq(uint32_t freq_hz,
                             int32_t steps_e,
                             float steps_per_mm,
                             uint8_t *limited)
{
    float max_mm_min = (steps_e < 0) ? E_RETRACT_MAX_MM_MIN : E_FEEDRATE_MAX_MM_MIN;
    uint32_t max_e_hz = rounded_hz((max_mm_min / 60.0f) * steps_per_mm);

    if (limited) {
        *limited = 0U;
    }

    uint32_t min_e_hz = (steps_e < 0) ? E_MIN_STEP_HZ_RETRACT : E_MIN_STEP_HZ_EXTRUDE;
    if (freq_hz > 0U && freq_hz < min_e_hz) {
        freq_hz = min_e_hz;
    }

    if (freq_hz > max_e_hz) {
        freq_hz = max_e_hz;
        if (limited) {
            *limited = 1U;
        }
    }

    return freq_hz;
}

void Motion_EvaluateExtruderPlan(const Motion_t *motion,
                                  const MotionCommand_t *cmd,
                                  float target_x,
                                  float target_y,
                                  float target_z,
                                  float target_e,
                                  float speed_mm_s,
                                  Motion_EPlan_t *plan)
{
    if (plan == NULL) return;
    memset(plan, 0, sizeof(*plan));

    if (motion == NULL || motion->config == NULL || cmd == NULL) {
        return;
    }

    float dx = target_x - motion->pos_x;
    float dy = target_y - motion->pos_y;
    float dz = target_z - motion->pos_z;
    float de = target_e - motion->pos_e;
    float flowrate = cmd->E_Flowrate;
    if (flowrate < 0.0f) {
        flowrate = 0.0f;
    }

    float xy_distance = sqrtf((dx * dx) + (dy * dy));
    float z_distance = fabsf(dz);
    float xy_time_s = (xy_distance > 0.001f && speed_mm_s > 0.0f)
                      ? xy_distance / speed_mm_s : 0.0f;
    float z_speed_mm_s = Z_FEEDRATE_MM_MIN / 60.0f;
    float z_time_s = (z_distance > 0.001f && z_speed_mm_s > 0.0f)
                     ? z_distance / z_speed_mm_s : 0.0f;
    float e_time_s = (fabsf(de) > 0.0001f && speed_mm_s > 0.0f)
                     ? fabsf(de) / speed_mm_s : 0.0f;
    float move_time_s = (xy_time_s > z_time_s) ? xy_time_s : z_time_s;
    uint8_t synchronized_with_xyz = (move_time_s > 0.0f) ? 1U : 0U;
    if (move_time_s <= 0.0f) {
        move_time_s = e_time_s;
    }

    float scaled_de = de * flowrate;
    int32_t steps_e = 0;
    float residual_after = motion->e_step_residual;
    if (cmd->echk && cmd->E_en) {
        float e_steps_f = (scaled_de * motion->config->e_steps_per_mm) +
                          motion->e_step_residual;
        if (e_steps_f >= 0.0f) {
            steps_e = (int32_t)floorf(e_steps_f);
        } else {
            steps_e = (int32_t)ceilf(e_steps_f);
        }
        residual_after = e_steps_f - (float)steps_e;
    }

    uint32_t abs_e = (uint32_t)labs(steps_e);
    float requested_hz = 0.0f;
    uint32_t step_hz = 0U;
    uint8_t frequency_limited = 0U;
    if (abs_e > 0U) {
        if (move_time_s > 0.0f) {
            requested_hz = (float)abs_e / move_time_s;
        } else {
            requested_hz = (E_FEEDRATE_MIN_MM_MIN / 60.0f) *
                           motion->config->e_steps_per_mm;
        }

        step_hz = rounded_hz(requested_hz);
        step_hz = clamp_e_freq(step_hz, steps_e,
                               motion->config->e_steps_per_mm,
                               &frequency_limited);
    }

    plan->has_e_word = cmd->echk;
    plan->e_enabled = cmd->E_en;
    plan->synchronized_with_xyz = synchronized_with_xyz;
    plan->frequency_limited = frequency_limited;
    plan->direction = (steps_e > 0 || (steps_e == 0 && scaled_de >= 0.0f))
                      ? E_DIR_EXTRUDE : E_DIR_RETRACT;
    plan->pos_e = motion->pos_e;
    plan->target_e = target_e;
    plan->delta_e_mm = de;
    plan->scaled_delta_e_mm = scaled_de;
    plan->flowrate = flowrate;
    plan->residual_before = motion->e_step_residual;
    plan->residual_after = residual_after;
    plan->xy_distance_mm = xy_distance;
    plan->z_distance_mm = z_distance;
    plan->move_time_s = move_time_s;
    plan->feedrate_mm_s = speed_mm_s;
    plan->requested_step_hz = requested_hz;
    plan->step_delay_us = (step_hz > 0U) ? (1000000.0f / (float)step_hz) : 0.0f;
    plan->signed_steps = steps_e;
    plan->abs_steps = abs_e;
    plan->step_hz = step_hz;
}

static void motion_debug_e_anomaly(const Motion_t *motion,
                                   const MotionCommand_t *cmd,
                                   float target_e,
                                   float de,
                                   uint32_t abs_e,
                                   int32_t steps_e,
                                   uint32_t freq_e,
                                   float move_time_s,
                                   float cxy_dist)
{
    static uint8_t  last_dir_valid = 0U;
    static uint8_t  last_dir = 0U;
    static uint32_t last_log_ms = 0U;
    static uint32_t last_flip_ms = 0U;
    uint32_t now = HAL_GetTick();
    uint8_t dir = (steps_e > 0) ? E_DIR_EXTRUDE : E_DIR_RETRACT;
    uint8_t should_log = 0U;

    if (abs_e == 0U) {
        if (cmd->echk || cmd->E_en) {
            should_log = 1U;
        }
    } else {
        if (last_dir_valid && dir != last_dir) {
            if ((now - last_flip_ms) <= 500U) {
                should_log = 1U;
            }
            last_flip_ms = now;
        }
        if (freq_e == 0U) {
            should_log = 1U;
        }
        if (abs_e > 0U && cxy_dist < 0.05f && move_time_s > 0.0f) {
            should_log = 1U;
        }
    }

    if (!should_log) {
        if (abs_e > 0U) {
            last_dir = dir;
            last_dir_valid = 1U;
        }
        return;
    }

    if ((now - last_log_ms) < 120U) {
        return;
    }
    last_log_ms = now;

    UARTDBG_Print(
        "[E dbg] id=%lu kind=%c en=%u e_abs=%u pos_e=%.4f cmd_e=%.4f target_e=%.4f de=%.4f steps_e=%ld abs_e=%lu dir=%u e_hz=%lu xy_mm=%.4f move_t=%.5f e_res=%.4f\r\n",
        cmd->trace_id,
        cmd->trace_kind ? cmd->trace_kind : '?',
        cmd->E_en,
        cmd->E_Absolute,
        motion->pos_e,
        cmd->e,
        target_e,
        de,
        (long)steps_e,
        abs_e,
        dir,
        freq_e,
        cxy_dist,
        move_time_s,
        motion->e_step_residual);

    if (abs_e > 0U) {
        last_dir = dir;
        last_dir_valid = 1U;
    }
}

void Motion_SetState(Motion_t *motion, Motion_States_t newState)
{
	if(xSemaphoreTake(xMotionStateMutex, pdMS_TO_TICKS(50)) == pdTRUE)
	{
		motion->state = newState;
		xSemaphoreGive(xMotionStateMutex);
	}
}

Motion_States_t Motion_GetState(Motion_t *motion)
{
	//Motion_States_t state;

	return motion->state;

}

void Motion_Move(Motion_t *motion, MotionCommand_t *cmd)
{
    if (!motion) return;

    // Resolve absolute target positions — do NOT update pos here; pos is
    // committed by the caller (MOTION_MOVE state) only after the move finishes.
    float target_x, target_y, target_z, target_e;
    if (cmd->Absolute) {
        target_x = cmd->xchk ? clamp_f(cmd->x, 0, motion->config->x_max_dist) : motion->pos_x;
        target_y = cmd->ychk ? clamp_f(cmd->y, 0, motion->config->y_max_dist) : motion->pos_y;
        target_z = cmd->zchk ? clamp_f(cmd->z, 0, motion->config->z_max_dist) : motion->pos_z;
    } else {
        target_x = motion->pos_x + (cmd->xchk ? cmd->x : 0.0f);
        target_y = motion->pos_y + (cmd->ychk ? cmd->y : 0.0f);
        target_z = motion->pos_z + (cmd->zchk ? cmd->z : 0.0f);
    }
    if (cmd->E_Absolute) {
        target_e = cmd->echk ? cmd->e : motion->pos_e;
    } else {
        target_e = motion->pos_e + (cmd->echk ? cmd->e : 0.0f);
    }

    if (cmd->speed_mm_s > motion->config->max_speed_mm_s)
        cmd->speed_mm_s = motion->config->max_speed_mm_s;
    if (cmd->speed_mm_s < 0.5f) cmd->speed_mm_s = 0.5f;

    // Deltas from current position to resolved target
    float dx     = target_x - motion->pos_x;
    float dy     = target_y - motion->pos_y;
    float z_dist = target_z - motion->pos_z;

    float cxy_dist = sqrtf(dx * dx + dy * dy);

    int32_t steps_a = (int32_t)((dx + dy) * motion->config->a_steps_per_mm);
    int32_t steps_b = (int32_t)((dx - dy) * motion->config->b_steps_per_mm);
    int32_t steps_z = (int32_t)(z_dist * (motion->config->z_steps_per_mm));
    Motion_EPlan_t e_plan;
    Motion_EvaluateExtruderPlan(motion, cmd, target_x, target_y, target_z,
                                 target_e, cmd->speed_mm_s, &e_plan);
    int32_t steps_e = e_plan.signed_steps;
    motion->e_step_residual = e_plan.residual_after;

    uint32_t abs_a = (uint32_t)labs(steps_a);
    uint32_t abs_b = (uint32_t)labs(steps_b);
    uint32_t abs_z = (uint32_t)labs(steps_z);
    uint32_t abs_e = e_plan.abs_steps;

    if (abs_e > 0U) {
        Stepper_Enable(motion->motor_e, 1U);
    }
    Stepper_SetDir(motion->motor_a, steps_a > 0 ? 1 : 0);
    Stepper_SetDir(motion->motor_b, steps_b > 0 ? 1 : 0);
    Stepper_SetDir(motion->motor_z, (steps_z > 0) ? Z_DIR_UP : Z_DIR_DOWN);
    if (abs_e > 0U) {
        Stepper_SetDir(motion->motor_e, e_plan.direction);
    }

    uint32_t freq_xy = 0U;
    uint32_t max_xy_steps = (abs_a > abs_b) ? abs_a : abs_b;
    float move_time_s = (cxy_dist > 0.001f && cmd->speed_mm_s > 0.0f)
                        ? cxy_dist / cmd->speed_mm_s : 0.0f;
    if (max_xy_steps > 0U && move_time_s > 0.0f) {
        freq_xy = (uint32_t)((float)max_xy_steps / move_time_s);
    }
    if (freq_xy > 50000U) freq_xy = 50000U;

    /* Stop idle channels before arming active ones.
     * TIM3 (A/B) shares a single update interrupt per timer, so stopping one
     * channel after starting its sibling can disable counting for the active move. */
    if (abs_b == 0) { Stepper_Stop(motion->motor_b); motion->motor_b->move_complete = 1; }
    if (abs_a == 0) { Stepper_Stop(motion->motor_a); motion->motor_a->move_complete = 1; }
    if (abs_z == 0) { Stepper_Stop(motion->motor_z); motion->motor_z->move_complete = 1; }
    if (abs_e == 0) { Stepper_Stop(motion->motor_e); motion->motor_e->move_complete = 1; }

    if ((abs_a > 0U) || (abs_b > 0U)) {
        Stepper_MoveStepsSharedTimer(motion->motor_a, abs_a,
                                     motion->motor_b, abs_b,
                                     freq_xy);
    }
    if (abs_z > 0U) Stepper_MoveSteps(motion->motor_z, abs_z, steps_per_sec(Z_FEEDRATE_MM_MIN, STEPS_PER_MM_Z));

    uint32_t freq_e = 0U;
    if (abs_e > 0U) {
        freq_e = e_plan.step_hz;
        motion_debug_e_anomaly(motion, cmd, target_e, e_plan.delta_e_mm, abs_e, steps_e,
                               freq_e, e_plan.move_time_s, cxy_dist);
        Stepper_MoveSteps(motion->motor_e, abs_e, freq_e);
    } else if (cmd->echk || cmd->E_en) {
        motion_debug_e_anomaly(motion, cmd, target_e, e_plan.delta_e_mm, abs_e, steps_e,
                               freq_e, e_plan.move_time_s, cxy_dist);
    }

//    uint32_t freq_e = 0U;
//
//    if (abs_e > 0U) {
//    	if (move_time_s > 0.0f) {
//    		freq_e = (uint32_t)((float)abs_e / move_time_s);
//    	} else {
//    		freq_e = steps_per_sec(E_FEEDRATE_MIN_MM_MIN, STEPS_PER_MM_E);
//    	}
//    	if (freq_e < 10U) freq_e = 10U;
//    	if (freq_e > 50000U) freq_e = 50000U;
//
//    	Stepper_MoveSteps(motion->motor_e, abs_e, freq_e);
//    }

    // Store resolved targets so the MOTION_MOVE state can commit pos on completion
    motion->target_x             = target_x;
    motion->target_y             = target_y;
    motion->target_e             = target_e;
    motion->x_axis_status.target_mm = target_x;
    motion->y_axis_status.target_mm = target_y;
    motion->z_axis_status.target_mm = target_z;
//    UARTDBG_Print("[motion.c]: Distance to move :: %d mm\r\n Current Position :: %d mm", (int)z_dist, (int)motion->pos_z);
    motion->current_speed     = cmd->speed_mm_s;
    motion->cxy_move_distance = cxy_dist;
    motion->z_move_distance   = z_dist;
    motion->e_move_distance   = e_plan.delta_e_mm;
    motion->move_start_time   = HAL_GetTick();

    motion->a_status.is_moving  = 1;
    motion->b_status.is_moving  = 1;
    motion->z_status.is_moving  = 1;
    motion->e_status.is_moving  = 1;
    motion->a_status.is_jogging = 0;
    motion->b_status.is_jogging = 0;
    motion->z_status.is_jogging = 0;
    motion->e_status.is_jogging = 0;


}

void Motion_Stop(Motion_t *motion)
{
    if (!motion) return;

    Stepper_Stop(motion->motor_a);
    Stepper_Stop(motion->motor_b);
    Stepper_Stop(motion->motor_z);
    Stepper_Stop(motion->motor_e);

    motion->target_x    = motion->pos_x;
    motion->target_y    = motion->pos_y;
    motion->z_axis_status.target_mm    = motion->pos_z;

    motion->a_status.is_jogging = 0;
    motion->b_status.is_jogging = 0;
    motion->z_status.is_jogging = 0;
    motion->e_status.is_jogging = 0;

    motion->a_status.is_moving  = 0;
    motion->b_status.is_moving  = 0;
    motion->z_status.is_moving  = 0;
    motion->e_status.is_moving  = 0;
}

void Motion_Emergency_Stop(Motion_t *motion)
{
    if (!motion) return;
    Stepper_Stop(motion->motor_a);
    Stepper_Stop(motion->motor_b);
    Stepper_Stop(motion->motor_z);
    Stepper_Stop(motion->motor_e);

    Stepper_Enable(motion->motor_a, 0);
    Stepper_Enable(motion->motor_b, 0);
    Stepper_Enable(motion->motor_z, 0);
    Stepper_Enable(motion->motor_e, 0);

    motion->a_status.is_jogging = 0;
    motion->b_status.is_jogging = 0;
    motion->z_status.is_jogging = 0;
    motion->e_status.is_jogging = 0;

    motion->a_status.is_moving  = 0;
    motion->b_status.is_moving  = 0;
    motion->z_status.is_moving  = 0;
    motion->e_status.is_moving  = 0;

    motion->x_axis_status.is_homed = 0;
    motion->y_axis_status.is_homed = 0;
    motion->z_axis_status.is_homed = 0;

}

void Motion_JogX(Motion_t *motion, int8_t direction, float speed_mm_s)
{
    if (!motion) return;
    if (speed_mm_s > motion->config->max_speed_mm_s) speed_mm_s = motion->config->max_speed_mm_s;
    uint32_t speed_steps = (uint32_t)(speed_mm_s * motion->config->a_steps_per_mm);
    uint8_t dir = (direction < 0) ? 0 : 1;

    Stepper_Enable(motion->motor_a, 1);
    Stepper_Enable(motion->motor_b, 1);
    Stepper_SetDir(motion->motor_a, dir);
    Stepper_SetDir(motion->motor_b, dir);
    Stepper_Jog(motion->motor_a, speed_steps);
    Stepper_Jog(motion->motor_b, speed_steps);

    motion->a_status.is_jogging      = 1;
    motion->b_status.is_jogging      = 1;

    motion->a_status.is_moving = 0;
    motion->b_status.is_moving = 0;
    motion->move_start_time = HAL_GetTick();
    motion->current_speed   = speed_mm_s * (direction > 0 ? 1.0f : -1.0f);
}

void Motion_JogY(Motion_t *motion, int8_t direction, float speed_mm_s)
{
    if (!motion) return;
    if (speed_mm_s > motion->config->max_speed_mm_s) speed_mm_s = motion->config->max_speed_mm_s;
    uint32_t speed_steps = (uint32_t)(speed_mm_s * motion->config->a_steps_per_mm);

    Stepper_Enable(motion->motor_a, 1);
    Stepper_Enable(motion->motor_b, 1);
    Stepper_SetDir(motion->motor_a, (direction > 0) ? 0 : 1);
    Stepper_SetDir(motion->motor_b, (direction > 0) ? 1 : 0);
    Stepper_Jog(motion->motor_a, speed_steps);
    Stepper_Jog(motion->motor_b, speed_steps);

    motion->a_status.is_jogging      = 1;
    motion->b_status.is_jogging      = 1;

    motion->a_status.is_moving       = 0;
    motion->b_status.is_moving       = 0;

    motion->move_start_time = HAL_GetTick();
    motion->current_speed   = speed_mm_s * (direction > 0 ? 1.0f : -1.0f);
}

void Motion_JogA(Motion_t *motion, int8_t direction, float speed_mm_s)
{
    if (!motion) return;
    if (speed_mm_s > motion->config->max_speed_mm_s) speed_mm_s = motion->config->max_speed_mm_s;
    uint32_t speed_steps = (uint32_t)(speed_mm_s * motion->config->a_steps_per_mm);
    Stepper_Enable(motion->motor_a, 1);
    Stepper_SetDir(motion->motor_a, (direction > 0) ? 0 : 1);
    Stepper_Jog(motion->motor_a, speed_steps);
    Stepper_Stop(motion->motor_b);

    motion->a_status.is_jogging = 1;

    motion->b_status.is_moving  = 0;
}

void Motion_JogB(Motion_t *motion, int8_t direction, float speed_mm_s)
{
    if (!motion) return;
    if (speed_mm_s > motion->config->max_speed_mm_s) speed_mm_s = motion->config->max_speed_mm_s;
    uint32_t speed_steps = (uint32_t)(speed_mm_s * motion->config->b_steps_per_mm);
    Stepper_Enable(motion->motor_b, 1);
    Stepper_Stop(motion->motor_a);
    Stepper_SetDir(motion->motor_b, (direction > 0) ? 0 : 1);
    Stepper_Jog(motion->motor_b, speed_steps);

    motion->b_status.is_jogging = 1;

    motion->b_status.is_moving  = 0;
}

void Motion_JogZ(Motion_t *motion, int8_t direction, float speed_mm_s)
{
	if (!motion) return;
	if (speed_mm_s > motion->config->max_speed_mm_s) speed_mm_s = motion->config->max_speed_mm_s;

    uint32_t speed_steps = (uint32_t)(speed_mm_s * motion->config->z_steps_per_mm);

    Stepper_Enable(motion->motor_z, 1);
    Stepper_SetDir(motion->motor_z, (direction > 0) ? 0 : 1);
    Stepper_Jog(motion->motor_z, speed_steps);

    motion->z_status.is_jogging = 1;

    motion->z_status.is_moving = 0;

}

void Motion_JogStop(Motion_t *motion)
{
    if (!motion) return;
    Stepper_Stop(motion->motor_a);
    Stepper_Stop(motion->motor_b);
    Stepper_Stop(motion->motor_z);

    motion->a_status.is_jogging = 0;
    motion->b_status.is_jogging = 0;
    motion->z_status.is_jogging = 0;


    motion->x_axis_status.is_homed = 0;  /* position unknown after unsupervised jog */
    motion->y_axis_status.is_homed = 0;
    motion->z_axis_status.is_homed = 0;

}



void Motion_Task(void *pvParameters)
{
	(void)pvParameters;

	static Motion_t motion;
	static Config_t config;
	static Homing_t homing;
	static MotionCommand_t command;
    static Stepper_t motor_a;
    static Stepper_t motor_b;
    static Stepper_t motor_z;
    static Stepper_t motor_e;
    static Endstop_t endstop_x;
    static Endstop_t endstop_y;
    static Endstop_t endstop_z;
    static uint8_t initialized = 0U;

    if (!initialized) {
        memset(&motion, 0, sizeof(motion));
        memset(&config, 0, sizeof(config));
        memset(&homing, 0, sizeof(homing));
        memset(&command, 0, sizeof(command));
        memset(&motor_a, 0, sizeof(motor_a));
        memset(&motor_b, 0, sizeof(motor_b));
        memset(&motor_z, 0, sizeof(motor_z));
        memset(&motor_e, 0, sizeof(motor_e));
        memset(&endstop_x, 0, sizeof(endstop_x));
        memset(&endstop_y, 0, sizeof(endstop_y));
        memset(&endstop_z, 0, sizeof(endstop_z));

        motion.motor_a = &motor_a;
        motion.motor_b = &motor_b;
        motion.motor_z = &motor_z;
        motion.motor_e = &motor_e;

        homing.endstop_x = &endstop_x;
        homing.endstop_y = &endstop_y;
        homing.endstop_z = &endstop_z;

        initialized = 1U;
    }

	Motion_SetState(&motion, MOTION_INIT);

	for(;;)
	{
		switch(Motion_GetState(&motion)){

			case MOTION_INIT:
                motion.motion_finished = 0U;
                motion.homing_finished = 0U;

				Stepper_Init(motion.motor_a,
				               &htim3, TIM_CHANNEL_1,
				               GPIOB, DIR_PIN_Pin,
				               GPIOB, EN_PIN_Pin,
				               1, 200);
				Stepper_Init(motion.motor_b,
				               &htim3, TIM_CHANNEL_2,
				               GPIOA, DIR_B_PIN_Pin,
				               GPIOA, EN_B_PIN_Pin,
				               1, 200);

				Stepper_Init(motion.motor_z,
				               &htim15, TIM_CHANNEL_1,
				               Z_DIR_PORT, Z_DIR_PIN,
				               Z_EN_PORT,  Z_EN_PIN,
				               MOTOR_EN_ACTIVE_LOW,
				               MOTOR_STEPS_PER_REV);

					Stepper_Init(motion.motor_e,
								   &htim17, TIM_CHANNEL_1,
								   E_DIR_PORT, E_DIR_PIN,
								   E_EN_PORT,  E_EN_PIN,
								   MOTOR_EN_ACTIVE_LOW,
								   MOTOR_STEPS_PER_REV);
					Endstop_Init(homing.endstop_x, GPIOC, ENDSTOP_X_PIN_Pin, 1);
				Endstop_Init(homing.endstop_y, GPIOC, ENDSTOP_Y_PIN_Pin, 1);
				Endstop_Init(homing.endstop_z, GPIOC, ENDSTOP_Z_PIN_Pin, 1);

				Motion_Config_Init(&config);

				Motors_Init(&motion, &config);

				Homing_Init(&homing, &motion);
                homing.config.fast_speed = 30.0f;
                homing.config.slow_speed = 5.0f;
                homing.config.z_fast_speed = 5.0f;
                homing.config.z_slow_speed = 2.5f;
                homing.config.backoff_distance = 5.0f;
                homing.config.timeout_ms = 60000U;
                homing.config.settle_ms = 200U;
                homing.config.x_dir = -1;
                homing.config.y_dir = 1;
                homing.config.z_dir = 1;

				Motion_SetState(&motion,MOTION_IDLE);
				break;

			case MOTION_IDLE:
				if(xQueueReceive(xMotionQueue, &command, portMAX_DELAY) == pdTRUE)
				{
					Motion_SetState(&motion, command.state);
				}
				break;

			case MOTION_HOME:

			    motion.homing_finished = 0;
			    Stepper_Enable(motion.motor_a, 1);
			    Stepper_Enable(motion.motor_b, 1);
			    Stepper_Enable(motion.motor_z, 1);

			    homing_change_state(&homing, HOMING_START);

				while(!motion.homing_finished)
				{
					Homing_Update(&motion, &homing, &command);
					vTaskDelay(pdMS_TO_TICKS(10));
				}

				command.state = MOTION_IDLE;
				xSemaphoreGive(xMotionUpdateMutex);
				xSemaphoreGive(xHomingDoneSem);

				Motion_SetState(&motion, MOTION_IDLE);

				break;

			case MOTION_MOVE:

				motion.motion_finished = 0;
				Motion_Move(&motion, &command);
					while(!motion.motion_finished)
					{
						Stepper_Task(motion.motor_e);

						if (Stepper_IsDone(motion.motor_a) &&
						    Stepper_IsDone(motion.motor_b) &&
						    Stepper_IsDone(motion.motor_z) &&
							Stepper_IsDone(motion.motor_e)) {
						motion.motion_finished = 1;
						break;
					}

					if ((HAL_GetTick() - motion.move_start_time) > 30000U) {
						Motion_Emergency_Stop(&motion);
						Motion_SetState(&motion, MOTION_FAULT);
						break;
					}

					vTaskDelay(pdMS_TO_TICKS(1));
				}

				if (Motion_GetState(&motion) == MOTION_FAULT) {
					xSemaphoreGive(xMotionUpdateMutex);
					break;
				}

				motion.pos_x = motion.target_x;
				motion.pos_y = motion.target_y;
				motion.pos_z = motion.z_axis_status.target_mm;
				motion.pos_e = motion.target_e;
				motion.motion_finished = 1;
//				Motion_SetState(&motion, MOTION_IDLE);
				motion.a_status.is_moving = 0;
				motion.b_status.is_moving = 0;
				motion.z_status.is_moving = 0;
				motion.e_status.is_moving = 0;
				xSemaphoreGive(xMotionDoneSem);
				xSemaphoreGive(xMotionUpdateMutex);
				if(xQueueReceive(xMotionQueue, &command, portMAX_DELAY) == pdTRUE)
					{
						Motion_SetState(&motion, command.state);
					}
				break;
			case MOTION_SET_ACCEL:
					Stepper_SetAcceleration(motion.motor_a, command.accel);
					Stepper_SetAcceleration(motion.motor_b, command.accel);
					Stepper_SetAcceleration(motion.motor_z, command.accel);
					Stepper_SetAcceleration(motion.motor_e, command.accel);

					if(xQueueReceive(xMotionQueue, &command, portMAX_DELAY) == pdTRUE)
						{
							Motion_SetState(&motion, command.state);
						}

				break;
			case MOTION_FAULT:

				Motion_Emergency_Stop(&motion);

				break;
			case MOTION_SET_POS:
				if (command.xchk) {
					motion.pos_x = command.x;
					motion.target_x = command.x;
					motion.x_axis_status.target_mm = command.x;
				}
				if (command.ychk) {
					motion.pos_y = command.y;
					motion.target_y = command.y;
					motion.y_axis_status.target_mm = command.y;
				}
				if (command.zchk) {
					motion.pos_z = command.z;
					motion.z_axis_status.target_mm = command.z;
				}
				if (command.echk) {
					motion.pos_e = command.e;
					motion.e_step_residual = 0.0f;
				}
				Motion_SetState(&motion, MOTION_IDLE);
				break;
		}
	}
}
