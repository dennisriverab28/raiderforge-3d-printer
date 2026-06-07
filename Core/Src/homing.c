#include "homing.h"
#include "qbc_globals.h"
#include "uart_debug.h"

// ═══════════════════════════════════════════════════════════════════════════
// INIT
// ═══════════════════════════════════════════════════════════════════════════

void Homing_Init(Homing_t *home, Motion_t *motion)
{
    if (home == NULL) return;

    home->motion = motion;
    Homing_Config_t config = {
        		.fast_speed = 0,
        		.slow_speed = 0,
        		.z_fast_speed = 0,
        		.z_slow_speed = 0,
        		.backoff_distance = 0,
        		.x_dir = 0,
        		.y_dir = 0,
        		.z_dir = 0,
        		.timeout_ms = 0,
        		.settle_ms = 0
        };
    home->config = config;

    // Default settle time if not set
    if (home->config.settle_ms == 0) {
        home->config.settle_ms = 100;
    }

    home->state = HOMING_IDLE;
    home->state_start_time = 0;
    home->in_progress = 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// HELPER: Change state and reset timer
// ═══════════════════════════════════════════════════════════════════════════

void homing_change_state(Homing_t *home, Homing_State_t new_state)
{
    if (home == NULL) return;

    if (new_state == HOMING_START) {
        home->in_progress = 1;
    }

    home->state = new_state;
    home->state_start_time = HAL_GetTick();
}

// ═══════════════════════════════════════════════════════════════════════════
// START HOMING
// ═══════════════════════════════════════════════════════════════════════════

/*void Homing_Start(Homing_t *home)
{
    if (home == NULL || home->in_progress) return;


}
*/
// ═══════════════════════════════════════════════════════════════════════════
// UPDATE STATE MACHINE
// FIX #16: Replaced all HAL_Delay(100) with non-blocking SETTLE states
// Each SETTLE state waits for settle_ms before transitioning
// ═══════════════════════════════════════════════════════════════════════════

void Homing_Update(Motion_t *motion, Homing_t *home, MotionCommand_t *flags)
{
    if (home == NULL || !home->in_progress) return;

    // Update endstops FIRST
    Endstop_Update(home->endstop_x);
    Endstop_Update(home->endstop_y);
    Endstop_Update(home->endstop_z);

    // Calculate elapsed time in current state
    uint32_t elapsed = HAL_GetTick() - home->state_start_time; //?? why calculate time

    // Check overall timeout (but not for COMPLETE/FAILED/IDLE/SETTLE states)
    if (home->state != HOMING_COMPLETE &&
        home->state != HOMING_FAILED &&
        home->state != HOMING_IDLE) {
        if (elapsed > home->config.timeout_ms) {
            UARTDBG_Print("[HOMING] TIMEOUT! Aborting.\r\n");
            Motion_JogStop(home->motion);
            home->state = HOMING_FAILED;
            home->in_progress = 0;
            return;
        }
    }

    // Calculate backoff time once
    uint32_t backoff_time_xy_ms = (uint32_t)((home->config.backoff_distance / home->config.slow_speed) * 1000.0f);
    uint32_t backoff_time_z_ms  = (uint32_t)((home->config.backoff_distance / home->config.z_slow_speed) * 1000.0f);

    // State machine
    switch (home->state) {

    	case HOMING_START:

    	    home->in_progress = 1;

    	    if(!flags->xchk) homing_change_state(home, HOMING_FAST_X);
    	    else if(!flags->ychk) homing_change_state(home, HOMING_FAST_Y);
    	    else if(!flags->zchk) homing_change_state(home, HOMING_FAST_Z);
    	    else {
    	    	homing_change_state(home, HOMING_COMPLETE);
    	    }
    	    break;
        // ─── FAST X APPROACH ───
        case HOMING_FAST_X:
			Motion_JogX(home->motion, home->config.x_dir, home->config.fast_speed);
            if (Endstop_IsTriggered(home->endstop_x)) {
                Motion_JogStop(home->motion);
                homing_change_state(home, HOMING_SETTLE_FAST_X);
            }
            break;

        // ─── SETTLE after fast X (replaces HAL_Delay(100)) ───
        case HOMING_SETTLE_FAST_X:
            if (elapsed >= home->config.settle_ms) {
            	homing_change_state(home, HOMING_BACKOFF_X);

            }
            break;

        // ─── BACKOFF X ───
        case HOMING_BACKOFF_X:
        	Motion_JogX(home->motion, -home->config.x_dir, home->config.slow_speed);
            if (elapsed >= backoff_time_xy_ms) {
                Motion_JogStop(home->motion);
                homing_change_state(home, HOMING_SETTLE_BACKOFF_X);
            }
            break;

        // ─── SETTLE after backoff X ───
        case HOMING_SETTLE_BACKOFF_X:
            if (elapsed >= home->config.settle_ms) {
            	homing_change_state(home, HOMING_SLOW_X);
            }
            break;

        // ─── SLOW X APPROACH ───
        case HOMING_SLOW_X:
            Motion_JogX(home->motion, home->config.x_dir, home->config.slow_speed);
            if (Endstop_IsTriggered(home->endstop_x)) {
                Motion_JogStop(home->motion);
                homing_change_state(home, HOMING_SETTLE_SLOW_X);
            }
            break;

        // ─── SETTLE after slow X ───
        case HOMING_SETTLE_SLOW_X:
            if (elapsed >= home->config.settle_ms) {

                if(!flags->ychk) homing_change_state(home, HOMING_FAST_Y);
                else if(!flags->zchk) homing_change_state(home, HOMING_FAST_Z);
                else homing_change_state(home, HOMING_COMPLETE);


            }
            break;

        // ─── FAST Y APPROACH ───
        case HOMING_FAST_Y:
            Motion_JogY(home->motion, home->config.y_dir, home->config.fast_speed);
            if (Endstop_IsTriggered(home->endstop_y)) {
                Motion_JogStop(home->motion);
                homing_change_state(home, HOMING_SETTLE_FAST_Y);
            }
            break;

        // ─── SETTLE after fast Y ───
        case HOMING_SETTLE_FAST_Y:
            if (elapsed >= home->config.settle_ms) {
            	homing_change_state(home, HOMING_BACKOFF_Y);
            }
            break;

        // ─── BACKOFF Y ───
        case HOMING_BACKOFF_Y:
            Motion_JogY(home->motion, -home->config.y_dir, home->config.slow_speed);
            if (elapsed >= backoff_time_xy_ms) {
                Motion_JogStop(home->motion);
                homing_change_state(home, HOMING_SETTLE_BACKOFF_Y);
            }
            break;

        // ─── SETTLE after backoff Y ───
        case HOMING_SETTLE_BACKOFF_Y:
            if (elapsed >= home->config.settle_ms) {
            	homing_change_state(home, HOMING_SLOW_Y);
            }
            break;

        // ─── SLOW Y APPROACH ───
        case HOMING_SLOW_Y:
            Motion_JogY(home->motion, home->config.y_dir, home->config.slow_speed);
            if (Endstop_IsTriggered(home->endstop_y)) {
                Motion_JogStop(home->motion);
                homing_change_state(home, HOMING_SETTLE_SLOW_Y);
            }
            break;

        // ─── SETTLE after slow Y ───
        case HOMING_SETTLE_SLOW_Y:
            if (elapsed >= home->config.settle_ms) {
               if(!flags->zchk) homing_change_state(home, HOMING_FAST_Z);
               else homing_change_state(home, HOMING_COMPLETE);

            }
            break;

        // Home the Z axis
            // ─── FAST Y APPROACH ───
		case HOMING_FAST_Z:
			Motion_JogZ(home->motion, home->config.z_dir, home->config.z_fast_speed);
			if (Endstop_IsTriggered(home->endstop_z)) {
				Motion_JogStop(home->motion);
				homing_change_state(home, HOMING_SETTLE_FAST_Z);
			}
			break;

		// ─── SETTLE after fast Y ───
		case HOMING_SETTLE_FAST_Z:
			if (elapsed >= home->config.settle_ms) {
				homing_change_state(home, HOMING_BACKOFF_Z);
			}
			break;

		// ─── BACKOFF Y ───
		case HOMING_BACKOFF_Z:
			Motion_JogZ(home->motion, -home->config.z_dir, home->config.z_slow_speed);
			if (elapsed >= backoff_time_z_ms) {
				Motion_JogStop(home->motion);
				homing_change_state(home, HOMING_SETTLE_BACKOFF_Z);
			}
			break;

		// ─── SETTLE after backoff Y ───
		case HOMING_SETTLE_BACKOFF_Z:
			if (elapsed >= home->config.settle_ms) {
				homing_change_state(home, HOMING_SLOW_Z);
			}
			break;

		// ─── SLOW Y APPROACH ───
		case HOMING_SLOW_Z:
			Motion_JogZ(home->motion, home->config.z_dir, home->config.z_slow_speed);
			if (Endstop_IsTriggered(home->endstop_z)) {
				Motion_JogStop(home->motion);
				homing_change_state(home, HOMING_SETTLE_SLOW_Z);
			}
			break;

		// ─── SETTLE after slow Y ───
		case HOMING_SETTLE_SLOW_Z:
			if (elapsed >= home->config.settle_ms) {
				homing_change_state(home, HOMING_COMPLETE);

			}
			break;
        case HOMING_COMPLETE:
             home->in_progress = 0;
             home->motion->homing_finished = 1;
             home->motion->pos_x = 0.0f;
             home->motion->pos_y = 0.0f;
             home->motion->pos_z = 0.0f;
             home->motion->x_axis_status.is_homed = 1;
             home->motion->y_axis_status.is_homed = 1;
             home->motion->z_axis_status.is_homed = 1;
             home->state = HOMING_IDLE;
             return;
        case HOMING_FAILED:
        case HOMING_IDLE:
            // Nothing to do - motors should already be stopped
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// STATUS QUERIES
// ═══════════════════════════════════════════════════════════════════════════

uint8_t Homing_InProgress(Homing_t *home)
{
    if (home == NULL) return 0;
    return home->in_progress;
}

uint8_t Homing_IsComplete(Homing_t *home)
{
    if (home == NULL) return 0;
    return (home->state == HOMING_COMPLETE);
}

uint8_t Homing_HasFailed(Homing_t *home)
{
    if (home == NULL) return 0;
    return (home->state == HOMING_FAILED);
}

Homing_State_t Homing_GetState(Homing_t *home)
{
    if (home == NULL) return HOMING_IDLE;
    return home->state;
}

// ═══════════════════════════════════════════════════════════════════════════
// ABORT
// ═══════════════════════════════════════════════════════════════════════════

void Homing_Abort(Homing_t *home)
{
    if (home == NULL) return;
    if (home->in_progress) {
        UARTDBG_Print("[HOMING] Aborted!\r\n");
        Motion_JogStop(home->motion);
        home->state = HOMING_FAILED;
        home->in_progress = 0;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BLOCKING HOMING
// ═══════════════════════════════════════════════════════════════════════════

//uint8_t Homing_StartAndWait(Homing_t *home)
//{
//    if (home == NULL) return 0;
//
//    change_state(home, HOMING_START);
//    Homing_Update(home, &GArgs);
//
//    while (Homing_InProgress(home)) {
//        Homing_Update(home, &GArgs);
//        HAL_Delay(5);
//    }
//
//    // Make absolutely sure motors are stopped
//    CoreXY_JogStop(home->corexy);
//
//    return Homing_IsComplete(home);
//}
