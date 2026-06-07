/*
 * extruder_test.c
 *
 * UART dry-run test harness for Motor E. This task accepts simple G-code-like
 * lines over USART2 and prints the E-axis plan without driving any motors.
 */

#include "extruder_test.h"

#include "FreeRTOS.h"
#include "main.h"
#include "motion.h"
#include "printer_config.h"
#include "task.h"
#include "uart_debug.h"

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

typedef struct {
    float x;
    float y;
    float z;
    float e;
    float f;
    float s;
    uint8_t xchk;
    uint8_t ychk;
    uint8_t zchk;
    uint8_t echk;
    uint8_t fchk;
    uint8_t schk;
} ExtruderTestArgs_t;

typedef struct {
    Motion_t motion;
    Config_t config;
    MotionCommand_t modal;
    uint8_t initialized;
} ExtruderTestContext_t;

static ExtruderTestContext_t g_etest;

static void etest_reset(void)
{
    memset(&g_etest, 0, sizeof(g_etest));
    Motion_Config_Init(&g_etest.config);
    g_etest.motion.config = &g_etest.config;

    g_etest.modal.Absolute = 1U;
    g_etest.modal.E_Absolute = 1U;
    g_etest.modal.E_Flowrate = 1.0f;
    g_etest.modal.speed_mm_s = 0.0f;
    g_etest.modal.state = MOTION_MOVE;
    g_etest.initialized = 1U;
}

static void trim_and_lower(char *line)
{
    char *comment = strchr(line, ';');
    if (comment != NULL) {
        *comment = '\0';
    }

    size_t len = strlen(line);
    while (len > 0U && isspace((unsigned char)line[len - 1U])) {
        line[len - 1U] = '\0';
        len--;
    }

    char *start = line;
    while (*start != '\0' && isspace((unsigned char)*start)) {
        start++;
    }
    if (start != line) {
        memmove(line, start, strlen(start) + 1U);
    }

    for (char *p = line; *p != '\0'; p++) {
        *p = (char)tolower((unsigned char)*p);
    }
}

static uint8_t code_is(const char *line, const char *code)
{
    size_t len = strlen(code);
    return (strncmp(line, code, len) == 0 &&
            (line[len] == '\0' || isspace((unsigned char)line[len]))) ? 1U : 0U;
}

static void parse_args(const char *line, ExtruderTestArgs_t *args)
{
    char buf[128];
    memset(args, 0, sizeof(*args));
    strncpy(buf, line, sizeof(buf) - 1U);
    buf[sizeof(buf) - 1U] = '\0';

    char *tok = strtok(buf, " \t");
    while (tok != NULL) {
        char name = (char)tolower((unsigned char)tok[0]);
        float val = 0.0f;

        if (tok[1] != '\0') {
            val = (float)atof(&tok[1]);
        }

        switch (name) {
            case 'x':
                args->x = val;
                args->xchk = 1U;
                break;
            case 'y':
                args->y = val;
                args->ychk = 1U;
                break;
            case 'z':
                args->z = val;
                args->zchk = 1U;
                break;
            case 'e':
                args->e = val;
                args->echk = 1U;
                break;
            case 'f':
                args->f = val;
                args->fchk = 1U;
                break;
            case 's':
                args->s = val;
                args->schk = 1U;
                break;
            default:
                break;
        }

        tok = strtok(NULL, " \t");
    }
}

static const char *dir_name(uint8_t direction)
{
    return (direction == E_DIR_EXTRUDE) ? "extrude" : "retract";
}

static void print_help(void)
{
    UARTDBG_Print("\r\n[E test] Motor E dry-run commands:\r\n");
    UARTDBG_Print("  G1 X.. Y.. Z.. E.. F..  - plan a fake move\r\n");
    UARTDBG_Print("  G0 X.. Y.. Z.. F..      - plan a fake travel move\r\n");
    UARTDBG_Print("  G90/G91                 - XYZ absolute/relative\r\n");
    UARTDBG_Print("  M82/M83                 - E absolute/relative\r\n");
    UARTDBG_Print("  G92 X.. Y.. Z.. E..     - set fake position\r\n");
    UARTDBG_Print("  M221 S..                - set flow percent\r\n");
    UARTDBG_Print("  RESET                   - reset fake planner state\r\n\r\n");
}

static void print_plan(const char *line,
                       const MotionCommand_t *cmd,
                       const Motion_EPlan_t *plan)
{
    UARTDBG_Print("\r\n[E test] %s\r\n", line);
    UARTDBG_Print("[E test] mode xyz=%s e=%s flow=%.1f%% feed=%.3fmm/s\r\n",
                  cmd->Absolute ? "abs" : "rel",
                  cmd->E_Absolute ? "abs" : "rel",
                  plan->flowrate * 100.0f,
                  plan->feedrate_mm_s);

    UARTDBG_Print("[E test] e pos=%.4f target=%.4f de=%.4f scaled=%.4f residual %.4f -> %.4f\r\n",
                  plan->pos_e,
                  plan->target_e,
                  plan->delta_e_mm,
                  plan->scaled_delta_e_mm,
                  plan->residual_before,
                  plan->residual_after);

    UARTDBG_Print("[E test] steps signed=%ld abs=%lu dir=%s\r\n",
                  (long)plan->signed_steps,
                  plan->abs_steps,
                  dir_name(plan->direction));

    UARTDBG_Print("[E test] timing xyz_sync=%u xy=%.4fmm z=%.4fmm duration=%.5fs requested=%.2fHz actual=%luHz delay=%.1fus limited=%u\r\n",
                  plan->synchronized_with_xyz,
                  plan->xy_distance_mm,
                  plan->z_distance_mm,
                  plan->move_time_s,
                  plan->requested_step_hz,
                  plan->step_hz,
                  plan->step_delay_us,
                  plan->frequency_limited);

    if (!plan->has_e_word) {
        UARTDBG_Print("[E test] verdict: no E word; Motor E should stay enabled but idle.\r\n");
    } else if (plan->abs_steps == 0U) {
        UARTDBG_Print("[E test] verdict: E word is below one full step after residual; no pulse yet.\r\n");
    } else if (plan->frequency_limited) {
        UARTDBG_Print("[E test] verdict: requested E speed was capped by Motor E max feedrate.\r\n");
    } else {
        UARTDBG_Print("[E test] verdict: Motor E plan is plausible for this command.\r\n");
    }
}

static void process_move(const char *line)
{
    ExtruderTestArgs_t args;
    parse_args(line, &args);

    MotionCommand_t cmd = g_etest.modal;
    cmd.x = args.x;
    cmd.y = args.y;
    cmd.z = args.z;
    cmd.e = args.e;
    cmd.xchk = args.xchk;
    cmd.ychk = args.ychk;
    cmd.zchk = args.zchk;
    cmd.echk = args.echk;
    cmd.E_en = args.echk ? 1U : 0U;
    cmd.state = MOTION_MOVE;

    if (args.fchk) {
        cmd.speed_mm_s = args.f / 60.0f;
        g_etest.modal.speed_mm_s = cmd.speed_mm_s;
    }

    if (cmd.speed_mm_s > g_etest.config.max_speed_mm_s) {
        cmd.speed_mm_s = g_etest.config.max_speed_mm_s;
    }
    if (cmd.speed_mm_s < 0.5f) {
        cmd.speed_mm_s = 0.5f;
    }

    float target_x;
    float target_y;
    float target_z;
    float target_e;

    if (cmd.Absolute) {
        target_x = cmd.xchk ? clamp_f(cmd.x, 0.0f, g_etest.config.x_max_dist) : g_etest.motion.pos_x;
        target_y = cmd.ychk ? clamp_f(cmd.y, 0.0f, g_etest.config.y_max_dist) : g_etest.motion.pos_y;
        target_z = cmd.zchk ? clamp_f(cmd.z, 0.0f, g_etest.config.z_max_dist) : g_etest.motion.pos_z;
    } else {
        target_x = g_etest.motion.pos_x + (cmd.xchk ? cmd.x : 0.0f);
        target_y = g_etest.motion.pos_y + (cmd.ychk ? cmd.y : 0.0f);
        target_z = g_etest.motion.pos_z + (cmd.zchk ? cmd.z : 0.0f);
    }

    if (cmd.E_Absolute) {
        target_e = cmd.echk ? cmd.e : g_etest.motion.pos_e;
    } else {
        target_e = g_etest.motion.pos_e + (cmd.echk ? cmd.e : 0.0f);
    }

    Motion_EPlan_t plan;
    Motion_EvaluateExtruderPlan(&g_etest.motion, &cmd,
                                 target_x, target_y, target_z, target_e,
                                 cmd.speed_mm_s, &plan);
    print_plan(line, &cmd, &plan);

    g_etest.motion.pos_x = target_x;
    g_etest.motion.pos_y = target_y;
    g_etest.motion.pos_z = target_z;
    g_etest.motion.pos_e = target_e;
    g_etest.motion.e_step_residual = plan.residual_after;
}

static void process_set_position(const char *line)
{
    ExtruderTestArgs_t args;
    parse_args(line, &args);

    if (args.xchk) g_etest.motion.pos_x = args.x;
    if (args.ychk) g_etest.motion.pos_y = args.y;
    if (args.zchk) g_etest.motion.pos_z = args.z;
    if (args.echk) {
        g_etest.motion.pos_e = args.e;
        g_etest.motion.e_step_residual = 0.0f;
    }

    UARTDBG_Print("[E test] set pos X=%.3f Y=%.3f Z=%.3f E=%.4f residual=%.4f\r\n",
                  g_etest.motion.pos_x,
                  g_etest.motion.pos_y,
                  g_etest.motion.pos_z,
                  g_etest.motion.pos_e,
                  g_etest.motion.e_step_residual);
}

void ExtruderTest_ProcessLine(const char *raw_line)
{
    char line[128];

    if (!g_etest.initialized) {
        etest_reset();
    }

    if (raw_line == NULL) {
        return;
    }

    strncpy(line, raw_line, sizeof(line) - 1U);
    line[sizeof(line) - 1U] = '\0';
    trim_and_lower(line);

    if (line[0] == '\0') {
        return;
    }

    if (strncmp(line, "etest ", 6U) == 0) {
        memmove(line, line + 6U, strlen(line + 6U) + 1U);
        trim_and_lower(line);
    } else if (strncmp(line, "et ", 3U) == 0) {
        memmove(line, line + 3U, strlen(line + 3U) + 1U);
        trim_and_lower(line);
    }

    if (strcmp(line, "?") == 0 || strcmp(line, "help") == 0) {
        print_help();
    } else if (strcmp(line, "reset") == 0) {
        etest_reset();
        UARTDBG_Print("[E test] reset complete\r\n");
    } else if (code_is(line, "g90")) {
        g_etest.modal.Absolute = 1U;
        UARTDBG_Print("[E test] XYZ absolute mode\r\n");
    } else if (code_is(line, "g91")) {
        g_etest.modal.Absolute = 0U;
        UARTDBG_Print("[E test] XYZ relative mode\r\n");
    } else if (code_is(line, "m82")) {
        g_etest.modal.E_Absolute = 1U;
        UARTDBG_Print("[E test] E absolute mode\r\n");
    } else if (code_is(line, "m83")) {
        g_etest.modal.E_Absolute = 0U;
        UARTDBG_Print("[E test] E relative mode\r\n");
    } else if (code_is(line, "m221")) {
        ExtruderTestArgs_t args;
        parse_args(line, &args);
        if (args.schk) {
            if (args.s < 0.0f) args.s = 0.0f;
            g_etest.modal.E_Flowrate = args.s / 100.0f;
        }
        UARTDBG_Print("[E test] flow=%.1f%%\r\n", g_etest.modal.E_Flowrate * 100.0f);
    } else if (code_is(line, "g92")) {
        process_set_position(line);
    } else if (code_is(line, "g0") || code_is(line, "g1")) {
        process_move(line);
    } else {
        UARTDBG_Print("[E test] ignored: %s\r\n", line);
    }
}

void ExtruderTest_Task(void *argument)
{
    (void)argument;
    uint8_t ch;
    char line[128];
    size_t idx = 0U;

    etest_reset();
    UARTDBG_Print("[E test] UART dry-run ready. Type HELP or paste a G1 line.\r\n");

    for (;;) {
        if (HAL_UART_Receive(&huart2, &ch, 1U, 2U) == HAL_OK) {
            if (ch == '\r' || ch == '\n') {
                if (idx > 0U) {
                    line[idx] = '\0';
                    ExtruderTest_ProcessLine(line);
                    idx = 0U;
                }
            } else if (ch == 0x08U || ch == 0x7FU) {
                if (idx > 0U) {
                    idx--;
                }
            } else if (idx < (sizeof(line) - 1U)) {
                line[idx++] = (char)ch;
            } else {
                idx = 0U;
                UARTDBG_Print("[E test] line too long; dropped\r\n");
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
    }
}
