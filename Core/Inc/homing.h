#ifndef HOMING_H
#define HOMING_H

#include "stm32g0xx_hal.h"
#include "motion.h"
#include "motion_types.h"
#include "endstop.h"
#include "qbc_globals.h"
#include <stdint.h>

// ═══════════════════════════════════════════════════════════════════════════
// HOMING CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

typedef struct {
    float fast_speed;        // XY speed for initial approach (mm/s)
    float slow_speed;        // XY speed for slow/precise approach (mm/s)
    float z_fast_speed;      // Z speed for initial approach (mm/s)
    float z_slow_speed;      // Z speed for slow/precise approach (mm/s)
    float backoff_distance;  // Distance to back off after trigger (mm)
    int8_t x_dir;            // Direction to home X: -1 = toward min, +1 = toward max
    int8_t y_dir;            // Direction to home Y: -1 = toward min, +1 = toward max
    int8_t z_dir;			 // Direction to home Z: -1 = toware min, +1 = toward max
    uint32_t timeout_ms;     // Timeout for homing (ms)
    uint32_t settle_ms;      // Motor settle time after stop (default 100ms)
} Homing_Config_t;

// ═══════════════════════════════════════════════════════════════════════════
// HOMING STATE
// FIX #16: Added SETTLE states to replace blocking HAL_Delay() calls
// ═══════════════════════════════════════════════════════════════════════════

typedef enum {
    HOMING_IDLE = 0,
	HOMING_START,
    HOMING_FAST_X,
    HOMING_SETTLE_FAST_X,    // NEW: non-blocking wait after fast X hit
    HOMING_BACKOFF_X,
    HOMING_SETTLE_BACKOFF_X, // NEW: non-blocking wait after backoff X
    HOMING_SLOW_X,
    HOMING_SETTLE_SLOW_X,    // NEW: non-blocking wait after slow X hit
    HOMING_FAST_Y,
    HOMING_SETTLE_FAST_Y,    // NEW: non-blocking wait after fast Y hit
    HOMING_BACKOFF_Y,
    HOMING_SETTLE_BACKOFF_Y, // NEW: non-blocking wait after backoff Y
    HOMING_SLOW_Y,
    HOMING_SETTLE_SLOW_Y,    // NEW: non-blocking wait after slow Y hit
	HOMING_FAST_Z,
    HOMING_SETTLE_FAST_Z,    // NEW: non-blocking wait after fast Y hit
    HOMING_BACKOFF_Z,
    HOMING_SETTLE_BACKOFF_Z, // NEW: non-blocking wait after backoff Y
    HOMING_SLOW_Z,
    HOMING_SETTLE_SLOW_Z,
    HOMING_COMPLETE,
    HOMING_FAILED
} Homing_State_t;

typedef struct {
    Homing_Config_t config;
    Homing_State_t state;
    Motion_t *motion;
    Endstop_t *endstop_x;
    Endstop_t *endstop_y;
    Endstop_t *endstop_z;
    uint32_t state_start_time;
    uint8_t in_progress;
} Homing_t;

// ═══════════════════════════════════════════════════════════════════════════
// FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Initialize homing module
void Homing_Init(Homing_t *home, Motion_t *motion);

// Start homing sequence (non-blocking)
//void Homing_Start(Homing_t *home); // Made Obsolete by START State

// Update homing state machine (call in main loop)
void Homing_Update(Motion_t *motion, Homing_t *home, MotionCommand_t *flags);

// Check if homing is in progress
uint8_t Homing_InProgress(Homing_t *home);

// Check if homing completed successfully
uint8_t Homing_IsComplete(Homing_t *home);

// Check if homing failed
uint8_t Homing_HasFailed(Homing_t *home);

// Get current homing state (for debugging)
Homing_State_t Homing_GetState(Homing_t *home);
void homing_change_state(Homing_t *home, Homing_State_t new_state);

// Abort homing
void Homing_Abort(Homing_t *home);

// Blocking homing (waits until complete or failed)
uint8_t Homing_StartAndWait(Homing_t *home);

#endif
