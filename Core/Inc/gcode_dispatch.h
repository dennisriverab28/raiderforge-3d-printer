/*
 * gcode_dispatch.h
 *
 * G-code dispatcher for RaiderForge CoreXY FDM printer.
 *
 * Call sequence for each line:
 *   Parse(line);          // parser.c — lowercases, handles g28 internally
 *   ParseArgs(line);      // extracts X Y Z E F S P into GArgs
 *   QBC_SyncArgs();       // bridges GArgs → DDRGArgs
 *   GCode_Execute(line);  // this module — reads DDRGArgs, calls hardware
 *
 * Supported commands (minimum set for first print):
 *   G0 / G1 [X][Y][Z][E][F]  — linear move + optional extrusion
 *   G4  P                     — dwell (ms)
 *   G28                       — XY home (blocking, non-starving)
 *   G90 / G91                 — absolute / relative positioning
 *   G92 [X][Y][Z][E]          — set current position (usually G92 E0)
 *   M82 / M83                 — E absolute / E relative
 *   M104 S                    — set hotend temperature
 *   M106 S                    — set part fan speed (0–255 Marlin scale)
 *   M107                      — part fan off
 *   M109 S                    — set + wait for hotend temperature
 *   M140 S                    — set bed temperature
 *   M190 S                    — set + wait for bed temperature
 *   M0 / M1                   — stop all motion
 *   M84 / M18                 — disable steppers (ignored — motors stay on)
 *
 * RTOS notes:
 *   — G28 polls with osDelay(5) so HeaterTask stays alive during homing.
 *   — M109/M190 poll with osDelay(2000) so HeaterTask keeps running.
 *   — Never call HAL_Delay inside any handler here.
 *
 * Created: Apr 2026 | Author: Cherryman125 (Dennis)
 */

#ifndef GCODE_DISPATCH_H
#define GCODE_DISPATCH_H

#include "stm32g0xx_hal.h"
#include "qbc_globals.h"
#include <stdint.h>

/* ─── Result codes ──────────────────────────────────────────────────────── */

typedef enum {
    GCODE_OK              = 0,   /* Executed successfully                    */
    GCODE_UNKNOWN,               /* Command not in our supported set (skip)  */
    GCODE_ERROR,                 /* Execution error (e.g. homing timeout)    */
    GCODE_NOT_HOMED,             /* Motion requested before G28              */
    GCODE_TEMP_OUT_OF_RANGE,     /* S value outside safe limits              */
} GCode_Result_t;

/* ─── Dispatcher state (exposed for display and testing) ────────────────── */

typedef struct {
    float    pos_x;             /* Current X position (mm, absolute)        */
    float    pos_y;             /* Current Y position (mm, absolute)        */
    float    pos_z;             /* Current Z position (mm, absolute)        */
    float    pos_e;             /* Current E position (mm)                  */
    float    feedrate_mm_min;   /* Active feedrate (mm/min)                 */
    uint8_t  relative_mode;     /* 1 = G91 relative XYZ, 0 = G90 absolute  */
    uint8_t  e_relative;        /* 1 = M83 E relative, 0 = M82 absolute    */
    uint8_t  initialized;       /* 1 after first successful G28             */
    uint32_t lines_executed;    /* Running count of dispatched lines        */
} GCode_State_t;

/* ─── Public API ────────────────────────────────────────────────────────── */

/*
 * GCode_Init — call once from main() after all DDRlo inits, before RTOS start.
 */
void GCode_Init(void);

/*
 * GCode_Execute — dispatch one G-code line to hardware.
 *
 * Preconditions:
 *   Parse(line) and ParseArgs(line) and QBC_SyncArgs() have been called.
 *   line is already lowercased (Parse() does this in-place).
 *
 * Returns GCODE_UNKNOWN for commands not in the supported set — this is
 * normal; slicers emit many optional commands we safely skip.
 */
GCode_Result_t GCode_Execute(const char *line);

/*
 * GCode_GetState — read-only view of current dispatcher state.
 */
const GCode_State_t *GCode_GetState(void);

/*
 * GCode_ResetPosition — mark as homed and zero all position tracking.
 * Called automatically after a successful G28.
 */
void GCode_ResetPosition(void);

#endif /* GCODE_DISPATCH_H */
