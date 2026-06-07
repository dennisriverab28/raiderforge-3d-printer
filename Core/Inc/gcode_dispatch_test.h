/*
 * gcode_dispatch_test.h
 *
 * Test bench for gcode_dispatch.c.
 *
 * Tests the dispatcher in isolation — verifies that each G/M command:
 *   a) Returns the correct GCode_Result_t
 *   b) Produces the correct side-effect on DDRlo hardware state
 *   c) Correctly updates GCode_State_t (position, mode flags)
 *
 * No motors are driven during these tests (no G28/G1 hardware moves).
 * Temperature commands are verified by reading DDRlo target values.
 * Fan commands are verified by reading Fan_t state.
 *
 * Usage:
 *   Call GCodeTest_RunAll() from the test menu (test_runner.c).
 *   All output goes to UART at 115200 baud.
 *
 * Created: Apr 2026 | Author: Cherryman125 (Dennis)
 */

#ifndef GCODE_DISPATCH_TEST_H
#define GCODE_DISPATCH_TEST_H

#include "integration_test.h"   /* IntTest_Stats_t, INT_CHECK, INT_WARN */

/*
 * GCodeTest_RunAll — run all dispatcher tests.
 * Prints section header, runs each test group, prints summary.
 */
void GCodeTest_RunAll(void);

/* Individual test groups */
void GCodeTest_Init         (IntTest_Stats_t *s);   /* Init state is clean      */
void GCodeTest_SafetyGating (IntTest_Stats_t *s);   /* G1 blocked before G28    */
void GCodeTest_ModeFlags    (IntTest_Stats_t *s);   /* G90/G91, M82/M83         */
void GCodeTest_PositionReset(IntTest_Stats_t *s);   /* G92 position tracking    */
void GCodeTest_HotendTemp   (IntTest_Stats_t *s);   /* M104 sets target         */
void GCodeTest_BedTemp      (IntTest_Stats_t *s);   /* M140 sets bed target     */
void GCodeTest_FanControl   (IntTest_Stats_t *s);   /* M106 speed, M107 off     */
void GCodeTest_UnknownCmd   (IntTest_Stats_t *s);   /* Unknown → GCODE_UNKNOWN  */
void GCodeTest_TempRangeClamp(IntTest_Stats_t *s);  /* Out-of-range S rejected  */
void GCodeTest_Feedrate     (IntTest_Stats_t *s);   /* F value stored in state  */

#endif /* GCODE_DISPATCH_TEST_H */
