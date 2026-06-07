/*
 * ddr_globals.h
 *
 * Updated Mar 2026:
 *   - Added motor_z, motor_e (Z axis and extruder)
 *   - Added endstop_x_max, endstop_y_max, endstop_z_max (double endstops)
 *
 * ENDSTOP WIRING:
 *   MIN endstops (homing): PC3=X  PC4=Y  PC5=Z   (is_nc=0, NO wiring)
 *   MAX endstops (safety): PC0=X  PC1=Y  PC2=Z   (is_nc=0, NO wiring)
 */

#ifndef INC_DDR_GLOBALS_H_
#define INC_DDR_GLOBALS_H_

#include "stm32g0xx_hal.h"
#include "heater.h"
#include "fan.h"
#include "stepper.h"
//#include "corexy.h"
#include "endstop.h"
#include "homing.h"
//#include "logger.h"
#include <stdint.h>

typedef enum {
    DDR_IDLE = 0,
    DDR_HOMING,
    DDR_HEATING,
    DDR_PRINTING,
    DDR_PAUSED,
    DDR_ERROR
} DDRState_t;

typedef struct _ddr_globals {
    uint32_t    integrityHead;

    DDRState_t  state;

    /* ── Thermal ── */
    Heater_t    hotend;
    Heater_t    bed;
    Fan_t       hotend_fan;
    Fan_t       part_fan;

    /* ── XY motion ── */
    Stepper_t   motor_a;
    Stepper_t   motor_b;
//    CoreXY_t    corexy;

    /* ── Z axis ── */
    Stepper_t   motor_z;

    /* ── Extruder ── */
    Stepper_t   motor_e;

    /* ── Endstops: MIN (homing) ── */
    Endstop_t   endstop_x;        /* X min — PC3, ENDSTOP_X_PIN_Pin    */
    Endstop_t   endstop_y;        /* Y min — PC4, ENDSTOP_Y_PIN_Pin    */
    Endstop_t   endstop_z;        /* Z bottom kill — PC5, ENDSTOP_Z_PIN_Pin    */

    /* ── Endstops: MAX (safety limits) ── */
    Endstop_t   endstop_x_max;    /* X max — PC0, ENDSTOP_X_MAX_Pin   */
    Endstop_t   endstop_y_max;    /* Y max — PC1, ENDSTOP_Y_MAX_Pin   */
    Endstop_t   endstop_z_max;    /* Z top home — PC2, ENDSTOP_Z_MAX_Pin   */

    /* ── Homing ── */
    Homing_t    homing;

    /* ── Logging ── */
//    Logger_t    logger;

    /* ── G-code flag ── */
    uint8_t     cmd_ready;

    uint32_t    integrityTail;
} DDRGlobals;

typedef struct _ddr_args {
    float   x;
    float   y;
    float   z;
    float   e;
    float   f;
    float   s;
    float   p;
    uint8_t xchk;
    uint8_t ychk;
    uint8_t zchk;
    uint8_t echk;
    uint8_t schk;
    uint8_t pchk;
} DDRArgs;

#ifndef DDR_MAIN
extern
#endif
DDRGlobals DDRlo;

#ifndef DDR_OWNER
extern
#endif
DDRArgs DDRGArgs;

void DDR_InitGlobals(void);

#endif /* INC_DDR_GLOBALS_H_ */
