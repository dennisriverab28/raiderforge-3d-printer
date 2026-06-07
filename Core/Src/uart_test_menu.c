#include "uart_test_menu.h"

#include "main.h"
#include "motion.h"
#include "parser.h"
#include "printer_config.h"
#include "qbc_globals.h"
#include "uart_debug.h"
#include "heater.h"
#include "bed_heater.h"
#include "thermistor.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

#define MENU_LINE_BUF                  128U
#define MENU_THERMAL_MONITOR_MS        500U
#define MENU_ENDSTOP_MONITOR_MS        250U
#define MENU_DEFAULT_HOTEND_C          200.0f
#define MENU_DEFAULT_BED_C              60.0f
#define MENU_DEFAULT_ACCEL_MM_S2       600.0f
#define MENU_DEFAULT_SQUARE_SIDE_MM     40.0f
#define MENU_DEFAULT_PRINT_Z_MM          0.30f
#define MENU_DEFAULT_PRINT_FEED_MM_MIN 1200.0f
#define MENU_DEFAULT_TRAVEL_FEED_MM_MIN 2400.0f
#define MENU_DEFAULT_PRIME_E_MM          1.0f
#define MENU_DEFAULT_EDGE_E_MM           2.0f

typedef struct {
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
} MenuEndstop_t;

static const MenuEndstop_t k_menu_endstops[] = {
    { "X MIN", GPIOC, ENDSTOP_X_PIN_Pin },
    { "Y MIN", GPIOC, ENDSTOP_Y_PIN_Pin },
    { "Z MIN", GPIOC, ENDSTOP_Z_PIN_Pin },
    { "X MAX", GPIOC, ENDSTOP_X_MAX_PIN_Pin },
    { "Y MAX", GPIOC, ENDSTOP_Y_MAX_PIN_Pin },
    { "Z MAX", GPIOC, ENDSTOP_Z_MAX_PIN_Pin }
};

static uint8_t menu_read_char(uint8_t *ch, uint32_t timeout_ms)
{
    return (HAL_UART_Receive(&huart2, ch, 1U, timeout_ms) == HAL_OK) ? 1U : 0U;
}

static uint8_t menu_wait_char(void)
{
    uint8_t ch = 0U;

    while (!menu_read_char(&ch, 100U)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ch;
}

static char menu_upper_ascii(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        return (char)(ch - ('a' - 'A'));
    }

    return ch;
}

static void menu_read_line(char *buf, size_t len)
{
    size_t idx = 0U;
    uint8_t ch = 0U;

    if (buf == NULL || len == 0U) {
        return;
    }

    memset(buf, 0, len);

    for (;;) {
        if (!menu_read_char(&ch, 100U)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            UARTDBG_Print("\r\n");
            buf[idx] = '\0';
            return;
        }

        if ((ch == 8U || ch == 127U) && idx > 0U) {
            idx--;
            UARTDBG_Print("\b \b");
            continue;
        }

        if (idx < (len - 1U)) {
            buf[idx++] = (char)ch;
            HAL_UART_Transmit(&huart2, &ch, 1U, 100U);
        }
    }
}

static void menu_wait_motion_ready(void)
{
    Motion_t *motion;

    while (!Motion_IsInitialized()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    do {
        motion = Motion_GetContext();
        vTaskDelay(pdMS_TO_TICKS(50));
    } while (motion == NULL || Motion_GetState(motion) == MOTION_INIT);
}

static void menu_pause(void)
{
    UARTDBG_Print("Press any key to continue...\r\n");
    (void)menu_wait_char();
}

static void menu_copy_line(char *dst, size_t dst_len, const char *src)
{
    size_t n;

    if (dst == NULL || dst_len == 0U) {
        return;
    }

    memset(dst, 0, dst_len);
    if (src == NULL) {
        return;
    }

    n = strlen(src);
    if (n >= dst_len) {
        n = dst_len - 1U;
    }
    memcpy(dst, src, n);
}

static void menu_run_gcode_direct(const char *line)
{
    char cmd[MENU_LINE_BUF];

    menu_copy_line(cmd, sizeof(cmd), line);
    UARTDBG_Print("[UART TEST] direct G-code: %s\r\n", cmd);
    Parse(cmd);
}

static void menu_queue_gcode(const char *line)
{
    char cmd[MENU_LINE_BUF];

    menu_copy_line(cmd, sizeof(cmd), line);
    if (xGCodeQueue == NULL) {
        UARTDBG_Print("[UART TEST] FAIL xGCodeQueue is NULL\r\n");
        return;
    }

    if (xQueueSend(xGCodeQueue, cmd, pdMS_TO_TICKS(200U)) == pdTRUE) {
        UARTDBG_Print("[UART TEST] queued: %s\r\n", cmd);
    } else {
        UARTDBG_Print("[UART TEST] FAIL queue send timed out\r\n");
    }
}

static void menu_print_header(const char *title)
{
    UARTDBG_Print("\r\n========================================\r\n");
    UARTDBG_Print("%s\r\n", title);
    UARTDBG_Print("========================================\r\n");
}

static void menu_print_main(void)
{
    menu_print_header("RAIDERFORGE UART TEST MENU");
    UARTDBG_Print("[1] Status and sensors\r\n");
    UARTDBG_Print("[2] Motion and homing\r\n");
    UARTDBG_Print("[3] Thermal and heaters\r\n");
    UARTDBG_Print("[4] Bench and print tests\r\n");
    UARTDBG_Print("[5] Raw G-code tools\r\n");
    UARTDBG_Print("[M] Reprint menu\r\n");
    UARTDBG_Print("Choice: ");
}

static void menu_print_status_menu(void)
{
    menu_print_header("STATUS AND SENSORS");
    UARTDBG_Print("[1] Full status snapshot\r\n");
    UARTDBG_Print("[2] Endstop tests\r\n");
    UARTDBG_Print("[3] Thermal snapshot and monitor\r\n");
    UARTDBG_Print("[R] Return\r\n");
    UARTDBG_Print("Choice: ");
}

static void menu_print_motion_menu(void)
{
    menu_print_header("MOTION AND HOMING");
    UARTDBG_Print("[1] Motor tests\r\n");
    UARTDBG_Print("[2] Homing tests\r\n");
    UARTDBG_Print("[3] Step counter\r\n");
    UARTDBG_Print("[4] Acceleration tests\r\n");
    UARTDBG_Print("[R] Return\r\n");
    UARTDBG_Print("Choice: ");
}

static void menu_print_thermal_menu(void)
{
    menu_print_header("THERMAL AND HEATERS");
    UARTDBG_Print("[1] Thermal tests\r\n");
    UARTDBG_Print("[R] Return\r\n");
    UARTDBG_Print("Choice: ");
}

static void menu_print_bench_menu(void)
{
    menu_print_header("BENCH AND PRINT TESTS");
    UARTDBG_Print("[1] System bench\r\n");
    UARTDBG_Print("[2] Pseudo print test\r\n");
    UARTDBG_Print("[R] Return\r\n");
    UARTDBG_Print("Choice: ");
}

static void menu_print_gcode_menu(void)
{
    menu_print_header("RAW G-CODE TOOLS");
    UARTDBG_Print("[1] Raw G-code entry\r\n");
    UARTDBG_Print("[R] Return\r\n");
    UARTDBG_Print("Choice: ");
}

static const char *menu_heater_state_name(Heater_State_t state)
{
    switch (state) {
        case HEATER_STATE_INIT: return "INIT";
        case HEATER_STATE_IDLE: return "IDLE";
        case HEATER_STATE_OFF: return "OFF";
        case HEATER_STATE_HEATING: return "HEATING";
        case HEATER_STATE_HOLD: return "HOLD";
        case HEATER_STATE_COOLING: return "COOLING";
        case HEATER_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

static const char *menu_heater_fault_name(Heater_Fault_t fault)
{
    switch (fault) {
        case FAULT_NONE: return "NONE";
        case FAULT_WARNING_TEMP_DRIFT: return "WARN_TEMP_DRIFT";
        case FAULT_WARNING_ADC_NOISY: return "WARN_ADC_NOISY";
        case FAULT_ERROR_SENSOR_SHORT: return "ERR_SENSOR_SHORT";
        case FAULT_ERROR_SENSOR_OPEN: return "ERR_SENSOR_OPEN";
        case FAULT_ERROR_TEMP_RUNAWAY: return "ERR_TEMP_RUNAWAY";
        case FAULT_CRITICAL_SHUTDOWN: return "CRITICAL_SHUTDOWN";
        default: return "UNKNOWN";
    }
}

static uint8_t menu_endstop_read(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static float menu_prompt_float_default(const char *prompt, float default_value)
{
    char line[32];

    UARTDBG_Print("%s [default %.1f]: ", prompt, (double)default_value);
    menu_read_line(line, sizeof(line));
    if (line[0] == '\0') {
        return default_value;
    }

    return (float)atof(line);
}

static uint32_t menu_prompt_uint_default(const char *prompt, uint32_t default_value)
{
    char line[32];

    UARTDBG_Print("%s [default %lu]: ", prompt, (unsigned long)default_value);
    menu_read_line(line, sizeof(line));
    if (line[0] == '\0') {
        return default_value;
    }

    return (uint32_t)strtoul(line, NULL, 10);
}

static uint8_t menu_motion_idle(Motion_t *motion)
{
    if (motion == NULL) {
        UARTDBG_Print("Motion context not ready.\r\n");
        return 0U;
    }
    if (Motion_GetState(motion) != MOTION_IDLE) {
        UARTDBG_Print("Motion task is busy (state=%d). Wait for idle first.\r\n",
                      (int)Motion_GetState(motion));
        return 0U;
    }
    return 1U;
}

static uint8_t menu_axes_homed(Motion_t *motion, uint8_t want_x, uint8_t want_y, uint8_t want_z)
{
    if (motion == NULL) {
        return 0U;
    }

    if (want_x && motion->x_axis_status.is_homed == 0U) {
        return 0U;
    }
    if (want_y && motion->y_axis_status.is_homed == 0U) {
        return 0U;
    }
    if (want_z && motion->z_axis_status.is_homed == 0U) {
        return 0U;
    }

    return 1U;
}

static void menu_print_thermal_snapshot(void)
{
    Heater_t *hotend = Hotend_GetHandle();
    Heater_t *bed = BedHeater_GetHandle();
    Heater_Diagnostics_t hot_diag;
    Heater_Diagnostics_t bed_diag;

    memset(&hot_diag, 0, sizeof(hot_diag));
    memset(&bed_diag, 0, sizeof(bed_diag));

    if (hotend != NULL) {
        Heater_GetDiagnostics(hotend, &hot_diag);
        UARTDBG_Print("Hotend: temp=%.2fC raw=%.2fC target=%.2fC pwm=%u%% state=%s therm=%s fault=%s flags=0x%08lx\r\n",
                      (double)hot_diag.current_temp,
                      (double)hot_diag.raw_temp,
                      (double)hot_diag.target_temp,
                      (unsigned)hot_diag.output_percent,
                      menu_heater_state_name(hotend->state),
                      Thermistor_GetStatusText(hot_diag.thermistor_status),
                      menu_heater_fault_name(hot_diag.fault_level),
                      (unsigned long)hot_diag.debug_flags);
    } else {
        UARTDBG_Print("Hotend: handle unavailable\r\n");
    }

    if (bed != NULL) {
        Heater_GetDiagnostics(bed, &bed_diag);
        UARTDBG_Print("Bed:    temp=%.2fC raw=%.2fC target=%.2fC pwm=%u%% state=%s therm=%s fault=%s flags=0x%08lx\r\n",
                      (double)bed_diag.current_temp,
                      (double)bed_diag.raw_temp,
                      (double)bed_diag.target_temp,
                      (unsigned)bed_diag.output_percent,
                      menu_heater_state_name(bed->state),
                      Thermistor_GetStatusText(bed_diag.thermistor_status),
                      menu_heater_fault_name(bed_diag.fault_level),
                      (unsigned long)bed_diag.debug_flags);
    } else {
        UARTDBG_Print("Bed: handle unavailable\r\n");
    }
}

static void menu_build_center_square(Motion_t *motion,
                                     float side_mm,
                                     float *x0,
                                     float *y0,
                                     float *x1,
                                     float *y1)
{
    float build_x = BUILD_SIZE_X;
    float build_y = BUILD_SIZE_Y;
    float start_x;
    float start_y;
    float end_x;
    float end_y;

    if (motion != NULL && motion->config != NULL) {
        build_x = motion->config->x_max_dist;
        build_y = motion->config->y_max_dist;
    }

    if (side_mm < 10.0f) {
        side_mm = 10.0f;
    }
    if (side_mm > (build_x - 20.0f)) {
        side_mm = build_x - 20.0f;
    }
    if (side_mm > (build_y - 20.0f)) {
        side_mm = build_y - 20.0f;
    }

    start_x = (build_x * 0.5f) - (side_mm * 0.5f);
    start_y = (build_y * 0.5f) - (side_mm * 0.5f);
    if (start_x < 5.0f) {
        start_x = 5.0f;
    }
    if (start_y < 5.0f) {
        start_y = 5.0f;
    }

    end_x = start_x + side_mm;
    end_y = start_y + side_mm;

    if (x0 != NULL) *x0 = start_x;
    if (y0 != NULL) *y0 = start_y;
    if (x1 != NULL) *x1 = end_x;
    if (y1 != NULL) *y1 = end_y;
}

static void menu_run_center_square(Motion_t *motion, float side_mm, float feed_mm_min)
{
    char cmd[96];
    float x0 = 0.0f;
    float y0 = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;

    menu_build_center_square(motion, side_mm, &x0, &y0, &x1, &y1);

    menu_run_gcode_direct("g90");
    snprintf(cmd, sizeof(cmd), "g0 x%.2f y%.2f f%.0f", (double)x0, (double)y0, (double)MENU_DEFAULT_TRAVEL_FEED_MM_MIN);
    menu_run_gcode_direct(cmd);
    snprintf(cmd, sizeof(cmd), "g0 x%.2f y%.2f f%.0f", (double)x1, (double)y0, (double)feed_mm_min);
    menu_run_gcode_direct(cmd);
    snprintf(cmd, sizeof(cmd), "g0 x%.2f y%.2f f%.0f", (double)x1, (double)y1, (double)feed_mm_min);
    menu_run_gcode_direct(cmd);
    snprintf(cmd, sizeof(cmd), "g0 x%.2f y%.2f f%.0f", (double)x0, (double)y1, (double)feed_mm_min);
    menu_run_gcode_direct(cmd);
    snprintf(cmd, sizeof(cmd), "g0 x%.2f y%.2f f%.0f", (double)x0, (double)y0, (double)feed_mm_min);
    menu_run_gcode_direct(cmd);
}

static void menu_run_print_square(Motion_t *motion,
                                  float side_mm,
                                  float z_mm,
                                  float feed_mm_min,
                                  float prime_e_mm,
                                  float edge_e_mm)
{
    char cmd[96];
    float x0 = 0.0f;
    float y0 = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;
    float e_total = 0.0f;

    menu_build_center_square(motion, side_mm, &x0, &y0, &x1, &y1);

    menu_run_gcode_direct("g90");
    menu_run_gcode_direct("m82 s0");
    menu_run_gcode_direct("g92 e0");

    snprintf(cmd, sizeof(cmd), "g0 z%.2f f300", (double)z_mm);
    menu_run_gcode_direct(cmd);
    snprintf(cmd, sizeof(cmd), "g0 x%.2f y%.2f f%.0f", (double)x0, (double)y0, (double)MENU_DEFAULT_TRAVEL_FEED_MM_MIN);
    menu_run_gcode_direct(cmd);

    e_total += prime_e_mm;
    snprintf(cmd, sizeof(cmd), "g1 e%.3f f180", (double)e_total);
    menu_run_gcode_direct(cmd);

    e_total += edge_e_mm;
    snprintf(cmd, sizeof(cmd), "g1 x%.2f y%.2f e%.3f f%.0f", (double)x1, (double)y0, (double)e_total, (double)feed_mm_min);
    menu_run_gcode_direct(cmd);
    e_total += edge_e_mm;
    snprintf(cmd, sizeof(cmd), "g1 x%.2f y%.2f e%.3f f%.0f", (double)x1, (double)y1, (double)e_total, (double)feed_mm_min);
    menu_run_gcode_direct(cmd);
    e_total += edge_e_mm;
    snprintf(cmd, sizeof(cmd), "g1 x%.2f y%.2f e%.3f f%.0f", (double)x0, (double)y1, (double)e_total, (double)feed_mm_min);
    menu_run_gcode_direct(cmd);
    e_total += edge_e_mm;
    snprintf(cmd, sizeof(cmd), "g1 x%.2f y%.2f e%.3f f%.0f", (double)x0, (double)y0, (double)e_total, (double)feed_mm_min);
    menu_run_gcode_direct(cmd);
}

static void menu_print_status(void)
{
    Motion_t *motion = Motion_GetContext();
    float xy_accel = 0.0f;
    float z_accel = 0.0f;
    float e_accel = 0.0f;

    menu_print_header("STATUS");
    Motion_GetAccelerationConfig(&xy_accel, &z_accel, &e_accel);

    UARTDBG_Print("Motion init: %s\r\n", motion ? "YES" : "NO");
    UARTDBG_Print("Queues: GCode=%s Motion=%s SD=%s Hotend=%s Bed=%s\r\n",
                  xGCodeQueue ? "OK" : "NULL",
                  xMotionQueue ? "OK" : "NULL",
                  xSDCardQueue ? "OK" : "NULL",
                  xHotendQueue ? "OK" : "NULL",
                  xBedQueue ? "OK" : "NULL");
    UARTDBG_Print("Ready sems: Hotend=%s Bed=%s\r\n",
                  xHotendReadySem ? "OK" : "NULL",
                  xBedReadySem ? "OK" : "NULL");
    UARTDBG_Print("Accel (mm/s^2): XY=%.1f Z=%.1f E=%.1f\r\n",
                  (double)xy_accel, (double)z_accel, (double)e_accel);
    UARTDBG_Print("Endstops: Xmin=%u Ymin=%u Zmin=%u Xmax=%u Ymax=%u Zmax=%u\r\n",
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_X_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_Y_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_Z_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_X_MAX_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_Y_MAX_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_Z_MAX_PIN_Pin));

    if (motion != NULL) {
        UARTDBG_Print("Motion state=%d pos=(%.3f, %.3f, %.3f) e=%.3f\r\n",
                      (int)Motion_GetState(motion),
                      (double)motion->pos_x,
                      (double)motion->pos_y,
                      (double)motion->pos_z,
                      (double)motion->pos_e);
        UARTDBG_Print("Homed: X=%u Y=%u Z=%u\r\n",
                      (unsigned)motion->x_axis_status.is_homed,
                      (unsigned)motion->y_axis_status.is_homed,
                      (unsigned)motion->z_axis_status.is_homed);
        UARTDBG_Print("Stepper speed Hz: A=%lu B=%lu Z=%lu E=%lu\r\n",
                      (unsigned long)Stepper_GetCurrentSpeed(motion->motor_a),
                      (unsigned long)Stepper_GetCurrentSpeed(motion->motor_b),
                      (unsigned long)Stepper_GetCurrentSpeed(motion->motor_z),
                      (unsigned long)Stepper_GetCurrentSpeed(motion->motor_e));
    }

    menu_print_thermal_snapshot();
}

static void menu_raw_gcode(void)
{
    char line[MENU_LINE_BUF];
    uint8_t choice;

    menu_print_header("RAW G-CODE");
    UARTDBG_Print("[D] Direct parse now\r\n");
    UARTDBG_Print("[Q] Queue through parser task\r\n");
    UARTDBG_Print("Mode: ");
    choice = menu_wait_char();
    UARTDBG_Print("%c\r\n", choice);

    UARTDBG_Print("Enter one G-code line: ");
    menu_read_line(line, sizeof(line));

    if (line[0] == '\0') {
        UARTDBG_Print("No command entered.\r\n");
        return;
    }

    if (menu_upper_ascii((char)choice) == 'Q') {
        menu_queue_gcode(line);
    } else {
        menu_run_gcode_direct(line);
    }
}

static void menu_motor_tests(void)
{
    Motion_t *motion = Motion_GetContext();
    uint8_t axis;
    uint8_t dir_char;
    int8_t direction = 1;
    uint32_t speed_hz;
    float speed_mm_s = 0.0f;
    uint8_t stop_key = 0U;
    uint32_t steps_a = 0U;
    uint32_t steps_b = 0U;
    uint32_t steps_main = 0U;
    float mm = 0.0f;

    if (!menu_motion_idle(motion)) {
        return;
    }

    menu_print_header("MOTOR TESTS");
    UARTDBG_Print("Axis [A/B/X/Y/Z/E]: ");
    axis = menu_wait_char();
    UARTDBG_Print("%c\r\n", axis);

    UARTDBG_Print("Direction [+/-]: ");
    dir_char = menu_wait_char();
    UARTDBG_Print("%c\r\n", dir_char);
    if (dir_char == '-') {
        direction = -1;
    }

    speed_hz = menu_prompt_uint_default("Stepper frequency Hz", 1000U);

    Stepper_ResetStepCount(motion->motor_a);
    Stepper_ResetStepCount(motion->motor_b);
    Stepper_ResetStepCount(motion->motor_z);
    Stepper_ResetStepCount(motion->motor_e);

    switch (menu_upper_ascii((char)axis)) {
        case 'A':
            speed_mm_s = (float)speed_hz / motion->config->a_steps_per_mm;
            Motion_JogA(motion, direction, speed_mm_s);
            break;
        case 'B':
            speed_mm_s = (float)speed_hz / motion->config->b_steps_per_mm;
            Motion_JogB(motion, direction, speed_mm_s);
            break;
        case 'X':
            speed_mm_s = (float)speed_hz / motion->config->a_steps_per_mm;
            Motion_JogX(motion, direction, speed_mm_s);
            break;
        case 'Y':
            speed_mm_s = (float)speed_hz / motion->config->a_steps_per_mm;
            Motion_JogY(motion, direction, speed_mm_s);
            break;
        case 'Z':
            speed_mm_s = (float)speed_hz / motion->config->z_steps_per_mm;
            Motion_JogZ(motion, direction, speed_mm_s);
            break;
        case 'E':
            speed_mm_s = (float)speed_hz / motion->config->e_steps_per_mm;
            Motion_JogE(motion, direction, speed_mm_s);
            break;
        default:
            UARTDBG_Print("Unknown axis.\r\n");
            return;
    }

    UARTDBG_Print("Running axis %c at %lu Hz (%.3f mm/s). Press any key to stop.\r\n",
                  axis,
                  (unsigned long)speed_hz,
                  (double)speed_mm_s);
    while (!menu_read_char(&stop_key, 50U)) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    Motion_JogStop(motion);

    steps_a = Stepper_GetStepCount(motion->motor_a);
    steps_b = Stepper_GetStepCount(motion->motor_b);
    switch (menu_upper_ascii((char)axis)) {
        case 'A':
            steps_main = steps_a;
            mm = (float)steps_main / motion->config->a_steps_per_mm;
            UARTDBG_Print("Motor A counted steps=%lu distance=%.3f mm\r\n",
                          (unsigned long)steps_main, (double)mm);
            break;
        case 'B':
            steps_main = steps_b;
            mm = (float)steps_main / motion->config->b_steps_per_mm;
            UARTDBG_Print("Motor B counted steps=%lu distance=%.3f mm\r\n",
                          (unsigned long)steps_main, (double)mm);
            break;
        case 'X':
        case 'Y':
            UARTDBG_Print("CoreXY counts: A=%lu (%.3f mm) B=%lu (%.3f mm)\r\n",
                          (unsigned long)steps_a,
                          (double)((float)steps_a / motion->config->a_steps_per_mm),
                          (unsigned long)steps_b,
                          (double)((float)steps_b / motion->config->b_steps_per_mm));
            break;
        case 'Z':
            steps_main = Stepper_GetStepCount(motion->motor_z);
            mm = (float)steps_main / motion->config->z_steps_per_mm;
            UARTDBG_Print("Motor Z counted steps=%lu distance=%.3f mm\r\n",
                          (unsigned long)steps_main, (double)mm);
            break;
        case 'E':
            steps_main = Stepper_GetStepCount(motion->motor_e);
            mm = (float)steps_main / motion->config->e_steps_per_mm;
            UARTDBG_Print("Motor E counted steps=%lu distance=%.3f mm\r\n",
                          (unsigned long)steps_main, (double)mm);
            break;
        default:
            break;
    }
}

static void menu_endstop_tests(void)
{
    uint8_t choice;
    uint8_t stop_key = 0U;
    size_t i;

    menu_print_header("ENDSTOP TESTS");
    UARTDBG_Print("[1] Snapshot all endstops\r\n");
    UARTDBG_Print("[2] Live monitor all endstops\r\n");
    UARTDBG_Print("[3] Watch one endstop\r\n");
    UARTDBG_Print("Choice: ");
    choice = menu_wait_char();
    UARTDBG_Print("%c\r\n", choice);

    if (choice == '1') {
        for (i = 0U; i < (sizeof(k_menu_endstops) / sizeof(k_menu_endstops[0])); ++i) {
            UARTDBG_Print("%s = %s\r\n",
                          k_menu_endstops[i].name,
                          menu_endstop_read(k_menu_endstops[i].port, k_menu_endstops[i].pin) ? "TRIGGERED" : "OPEN");
        }
        return;
    }

    if (choice == '2') {
        UARTDBG_Print("Live monitor running. Press any key to stop.\r\n");
        while (!menu_read_char(&stop_key, MENU_ENDSTOP_MONITOR_MS)) {
            for (i = 0U; i < (sizeof(k_menu_endstops) / sizeof(k_menu_endstops[0])); ++i) {
                UARTDBG_Print("%s=%s ",
                              k_menu_endstops[i].name,
                              menu_endstop_read(k_menu_endstops[i].port, k_menu_endstops[i].pin) ? "TRIG" : "OPEN");
            }
            UARTDBG_Print("\r\n");
        }
        return;
    }

    if (choice == '3') {
        UARTDBG_Print("[1] X MIN [2] Y MIN [3] Z MIN [4] X MAX [5] Y MAX [6] Z MAX\r\n");
        UARTDBG_Print("Endstop: ");
        choice = menu_wait_char();
        UARTDBG_Print("%c\r\n", choice);
        if (choice < '1' || choice > '6') {
            UARTDBG_Print("Invalid endstop selection.\r\n");
            return;
        }

        i = (size_t)(choice - '1');
        UARTDBG_Print("Watching %s. Press any key to stop.\r\n", k_menu_endstops[i].name);
        while (!menu_read_char(&stop_key, MENU_ENDSTOP_MONITOR_MS)) {
            UARTDBG_Print("%s = %s\r\n",
                          k_menu_endstops[i].name,
                          menu_endstop_read(k_menu_endstops[i].port, k_menu_endstops[i].pin) ? "TRIGGERED" : "OPEN");
        }
        return;
    }

    UARTDBG_Print("Unknown endstop test selection.\r\n");
}

static void menu_homing_tests(void)
{
    Motion_t *motion = Motion_GetContext();
    uint8_t choice;

    menu_print_header("HOMING TESTS");
    UARTDBG_Print("[1] Home X\r\n");
    UARTDBG_Print("[2] Home Y\r\n");
    UARTDBG_Print("[3] Home Z\r\n");
    UARTDBG_Print("[4] Full home\r\n");
    UARTDBG_Print("[5] Report homed flags only\r\n");
    UARTDBG_Print("Choice: ");
    choice = menu_wait_char();
    UARTDBG_Print("%c\r\n", choice);

    if (motion == NULL) {
        UARTDBG_Print("Motion context unavailable.\r\n");
        return;
    }

    switch (choice) {
        case '1':
            menu_run_gcode_direct("g28 x");
            break;
        case '2':
            menu_run_gcode_direct("g28 y");
            break;
        case '3':
            menu_run_gcode_direct("g28 z");
            break;
        case '4':
            menu_run_gcode_direct("g28");
            break;
        case '5':
            break;
        default:
            UARTDBG_Print("Unknown homing selection.\r\n");
            return;
    }

    UARTDBG_Print("Homed flags: X=%u Y=%u Z=%u pos=(%.3f, %.3f, %.3f)\r\n",
                  (unsigned)motion->x_axis_status.is_homed,
                  (unsigned)motion->y_axis_status.is_homed,
                  (unsigned)motion->z_axis_status.is_homed,
                  (double)motion->pos_x,
                  (double)motion->pos_y,
                  (double)motion->pos_z);
}

static void menu_thermal_monitor(void)
{
    uint8_t stop_key = 0U;

    UARTDBG_Print("Live thermistor monitor running. Press any key to stop.\r\n");
    while (!menu_read_char(&stop_key, MENU_THERMAL_MONITOR_MS)) {
        menu_print_thermal_snapshot();
    }
}

static void menu_thermal_tests(void)
{
    uint8_t choice;
    float hotend_target;
    float bed_target;
    char cmd[64];

    menu_print_header("THERMAL TESTS");
    menu_print_thermal_snapshot();
    UARTDBG_Print("[1] Snapshot now\r\n");
    UARTDBG_Print("[2] Live thermistor monitor\r\n");
    UARTDBG_Print("[3] Hotend heat test via G-code\r\n");
    UARTDBG_Print("[4] Bed heat test via G-code\r\n");
    UARTDBG_Print("[5] Heat both and wait/hold\r\n");
    UARTDBG_Print("[6] Heaters off\r\n");
    UARTDBG_Print("Choice: ");
    choice = menu_wait_char();
    UARTDBG_Print("%c\r\n", choice);

    switch (choice) {
        case '1':
            menu_print_thermal_snapshot();
            break;
        case '2':
            menu_thermal_monitor();
            break;
        case '3':
            hotend_target = menu_prompt_float_default("Hotend target C", MENU_DEFAULT_HOTEND_C);
            snprintf(cmd, sizeof(cmd), "m109 s%.0f", (double)hotend_target);
            menu_run_gcode_direct(cmd);
            menu_print_thermal_snapshot();
            break;
        case '4':
            bed_target = menu_prompt_float_default("Bed target C", MENU_DEFAULT_BED_C);
            snprintf(cmd, sizeof(cmd), "m190 s%.0f", (double)bed_target);
            menu_run_gcode_direct(cmd);
            menu_print_thermal_snapshot();
            break;
        case '5':
            bed_target = menu_prompt_float_default("Bed target C", MENU_DEFAULT_BED_C);
            hotend_target = menu_prompt_float_default("Hotend target C", MENU_DEFAULT_HOTEND_C);
            snprintf(cmd, sizeof(cmd), "m140 s%.0f", (double)bed_target);
            menu_run_gcode_direct(cmd);
            snprintf(cmd, sizeof(cmd), "m104 s%.0f", (double)hotend_target);
            menu_run_gcode_direct(cmd);
            snprintf(cmd, sizeof(cmd), "m190 s%.0f", (double)bed_target);
            menu_run_gcode_direct(cmd);
            snprintf(cmd, sizeof(cmd), "m109 s%.0f", (double)hotend_target);
            menu_run_gcode_direct(cmd);
            menu_print_thermal_snapshot();
            break;
        case '6':
            menu_run_gcode_direct("m104 s0");
            menu_run_gcode_direct("m140 s0");
            menu_print_thermal_snapshot();
            break;
        default:
            UARTDBG_Print("Unknown thermal selection.\r\n");
            break;
    }
}

static void menu_step_counter(void)
{
    Motion_t *motion = Motion_GetContext();
    char line[32];
    uint8_t axis;
    uint32_t speed_hz;
    uint32_t steps = 0U;
    float mm = 0.0f;
    uint8_t stop_key = 0U;

    if (!menu_motion_idle(motion)) {
        return;
    }

    menu_print_header("STEP COUNTER");
    UARTDBG_Print("Axis [A/B/X/Y/Z/E]: ");
    axis = menu_wait_char();
    UARTDBG_Print("%c\r\n", axis);

    UARTDBG_Print("Speed in Hz (example 1000): ");
    menu_read_line(line, sizeof(line));
    speed_hz = (uint32_t)strtoul(line, NULL, 10);
    if (speed_hz == 0U) {
        speed_hz = 1000U;
    }

    Stepper_ResetStepCount(motion->motor_a);
    Stepper_ResetStepCount(motion->motor_b);
    Stepper_ResetStepCount(motion->motor_z);
    Stepper_ResetStepCount(motion->motor_e);

    switch (menu_upper_ascii((char)axis)) {
        case 'A':
            Motion_JogA(motion, 1, (float)speed_hz / motion->config->a_steps_per_mm);
            break;
        case 'B':
            Motion_JogB(motion, 1, (float)speed_hz / motion->config->b_steps_per_mm);
            break;
        case 'X':
            Motion_JogX(motion, 1, (float)speed_hz / motion->config->a_steps_per_mm);
            break;
        case 'Y':
            Motion_JogY(motion, 1, (float)speed_hz / motion->config->a_steps_per_mm);
            break;
        case 'Z':
            Motion_JogZ(motion, 1, (float)speed_hz / motion->config->z_steps_per_mm);
            break;
        case 'E':
            Motion_JogE(motion, 1, (float)speed_hz / motion->config->e_steps_per_mm);
            break;
        default:
            UARTDBG_Print("Unknown axis.\r\n");
            return;
    }

    UARTDBG_Print("Running. Press any key to stop.\r\n");
    while (!menu_read_char(&stop_key, 50U)) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    Motion_JogStop(motion);

    switch (menu_upper_ascii((char)axis)) {
        case 'A':
            steps = Stepper_GetStepCount(motion->motor_a);
            mm = (float)steps / motion->config->a_steps_per_mm;
            break;
        case 'B':
            steps = Stepper_GetStepCount(motion->motor_b);
            mm = (float)steps / motion->config->b_steps_per_mm;
            break;
        case 'X':
        case 'Y':
            steps = Stepper_GetStepCount(motion->motor_a);
            mm = (float)steps / motion->config->a_steps_per_mm;
            break;
        case 'Z':
            steps = Stepper_GetStepCount(motion->motor_z);
            mm = (float)steps / motion->config->z_steps_per_mm;
            break;
        case 'E':
            steps = Stepper_GetStepCount(motion->motor_e);
            mm = (float)steps / motion->config->e_steps_per_mm;
            break;
        default:
            break;
    }

    UARTDBG_Print("Counted steps=%lu estimated distance=%.3f mm\r\n",
                  (unsigned long)steps, (double)mm);
}

static void menu_accel_tests(void)
{
    float xy_accel = 0.0f;
    float z_accel = 0.0f;
    float e_accel = 0.0f;
    float requested_accel;
    float side_mm;
    uint8_t choice;
    char cmd[64];

    menu_print_header("ACCELERATION TESTS");
    Motion_GetAccelerationConfig(&xy_accel, &z_accel, &e_accel);
    UARTDBG_Print("Current accel (mm/s^2): XY=%.1f Z=%.1f E=%.1f\r\n",
                  (double)xy_accel, (double)z_accel, (double)e_accel);
    UARTDBG_Print("[1] Set acceleration with M204 S...\r\n");
    UARTDBG_Print("[2] Run accelerated center square\r\n");
    UARTDBG_Print("Choice: ");
    choice = menu_wait_char();
    UARTDBG_Print("%c\r\n", choice);

    if (choice == '1') {
        requested_accel = menu_prompt_float_default("Acceleration mm/s^2", xy_accel > 0.0f ? xy_accel : MENU_DEFAULT_ACCEL_MM_S2);
        snprintf(cmd, sizeof(cmd), "m204 s%.0f", (double)requested_accel);
        menu_run_gcode_direct(cmd);
    } else if (choice == '2') {
        requested_accel = menu_prompt_float_default("Acceleration mm/s^2", xy_accel > 0.0f ? xy_accel : MENU_DEFAULT_ACCEL_MM_S2);
        side_mm = menu_prompt_float_default("Center square side mm", MENU_DEFAULT_SQUARE_SIDE_MM);
        snprintf(cmd, sizeof(cmd), "m204 s%.0f", (double)requested_accel);
        menu_run_gcode_direct(cmd);
        menu_run_center_square(Motion_GetContext(), side_mm, 3600.0f);
    } else {
        UARTDBG_Print("Unknown acceleration selection.\r\n");
    }
}

static void menu_pseudo_print_test(void)
{
    Motion_t *motion = Motion_GetContext();
    float hotend_target;
    float bed_target;
    float accel;
    float side_mm;
    float z_mm;
    float feed_mm_min;
    float prime_e_mm;
    float edge_e_mm;
    char cmd[64];

    if (!menu_motion_idle(motion)) {
        return;
    }

    menu_print_header("PSEUDO PRINT TEST");
    bed_target = menu_prompt_float_default("Bed target C", MENU_DEFAULT_BED_C);
    hotend_target = menu_prompt_float_default("Hotend target C", MENU_DEFAULT_HOTEND_C);
    accel = menu_prompt_float_default("Acceleration mm/s^2", MENU_DEFAULT_ACCEL_MM_S2);
    side_mm = menu_prompt_float_default("Square side mm", MENU_DEFAULT_SQUARE_SIDE_MM);
    z_mm = menu_prompt_float_default("Print Z height mm", MENU_DEFAULT_PRINT_Z_MM);
    feed_mm_min = menu_prompt_float_default("Print feedrate mm/min", MENU_DEFAULT_PRINT_FEED_MM_MIN);
    prime_e_mm = menu_prompt_float_default("Prime extrusion mm", MENU_DEFAULT_PRIME_E_MM);
    edge_e_mm = menu_prompt_float_default("Extrusion per edge mm", MENU_DEFAULT_EDGE_E_MM);

    snprintf(cmd, sizeof(cmd), "m140 s%.0f", (double)bed_target);
    menu_run_gcode_direct(cmd);
    snprintf(cmd, sizeof(cmd), "m104 s%.0f", (double)hotend_target);
    menu_run_gcode_direct(cmd);
    snprintf(cmd, sizeof(cmd), "m190 s%.0f", (double)bed_target);
    menu_run_gcode_direct(cmd);
    snprintf(cmd, sizeof(cmd), "m109 s%.0f", (double)hotend_target);
    menu_run_gcode_direct(cmd);

    menu_run_gcode_direct("g28");
    snprintf(cmd, sizeof(cmd), "m204 s%.0f", (double)accel);
    menu_run_gcode_direct(cmd);
    menu_run_print_square(motion, side_mm, z_mm, feed_mm_min, prime_e_mm, edge_e_mm);
    vTaskDelay(pdMS_TO_TICKS(1500));
    menu_print_thermal_snapshot();

    menu_run_gcode_direct("m104 s0");
    menu_run_gcode_direct("m140 s0");
    UARTDBG_Print("Pseudo print finished and heaters were turned off.\r\n");
}

static void menu_system_bench(void)
{
    Motion_t *motion = Motion_GetContext();
    uint32_t passed = 0U;
    uint32_t warned = 0U;
    float xy_accel = 0.0f;
    float z_accel = 0.0f;
    float e_accel = 0.0f;
    float requested_accel;
    float side_mm;
    float expected_x = 0.0f;
    float expected_y = 0.0f;
    float end_x = 0.0f;
    float end_y = 0.0f;
    char extra_cmd[MENU_LINE_BUF];
    char cmd[64];

    menu_print_header("SYSTEM BENCH");

    if (motion != NULL &&
        motion->Motion_Head == 0xDEADBEEF &&
        motion->Motion_Tail == 0xDEADBEEF) {
        UARTDBG_Print("[PASS] motion context integrity markers intact\r\n");
        passed++;
    } else {
        UARTDBG_Print("[FAIL] motion context unavailable or corrupted\r\n");
    }

    if (xMotionQueue != NULL && xGCodeQueue != NULL && xHotendQueue != NULL && xBedQueue != NULL) {
        UARTDBG_Print("[PASS] motion, parser, and thermal queues are ready\r\n");
        passed++;
    } else {
        UARTDBG_Print("[FAIL] one or more queues are NULL\r\n");
    }

    if (xHotendReadySem != NULL && xBedReadySem != NULL) {
        UARTDBG_Print("[PASS] heater ready semaphores are ready\r\n");
        passed++;
    } else {
        UARTDBG_Print("[WARN] heater ready semaphores missing\r\n");
        warned++;
    }

    UARTDBG_Print("[PASS] endstop snapshot Xmin=%u Ymin=%u Zmin=%u Xmax=%u Ymax=%u Zmax=%u\r\n",
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_X_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_Y_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_Z_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_X_MAX_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_Y_MAX_PIN_Pin),
                  (unsigned)menu_endstop_read(GPIOC, ENDSTOP_Z_MAX_PIN_Pin));
    passed++;

    if (!menu_motion_idle(motion)) {
        UARTDBG_Print("[FAIL] motion is not idle, bench aborted\r\n");
        UARTDBG_Print("Bench summary: PASS=%lu WARN=%lu\r\n",
                      (unsigned long)passed,
                      (unsigned long)warned);
        return;
    }

    Motion_GetAccelerationConfig(&xy_accel, &z_accel, &e_accel);
    requested_accel = menu_prompt_float_default("Bench acceleration mm/s^2", xy_accel > 0.0f ? xy_accel : MENU_DEFAULT_ACCEL_MM_S2);
    side_mm = menu_prompt_float_default("Bench center square side mm", MENU_DEFAULT_SQUARE_SIDE_MM);
    UARTDBG_Print("Optional extra G-code after homing/bench move (blank to skip): ");
    menu_read_line(extra_cmd, sizeof(extra_cmd));

    menu_run_gcode_direct("g28");
    if (menu_axes_homed(motion, 1U, 1U, 1U)) {
        UARTDBG_Print("[PASS] full homing completed\r\n");
        passed++;
    } else {
        UARTDBG_Print("[WARN] one or more axes did not report homed after G28\r\n");
        warned++;
    }

    snprintf(cmd, sizeof(cmd), "m204 s%.0f", (double)requested_accel);
    menu_run_gcode_direct(cmd);
    Motion_GetAccelerationConfig(&xy_accel, &z_accel, &e_accel);
    if (fabsf(xy_accel - requested_accel) < 1.0f) {
        UARTDBG_Print("[PASS] acceleration updated to %.1f mm/s^2\r\n", (double)xy_accel);
        passed++;
    } else {
        UARTDBG_Print("[WARN] requested accel %.1f but current XY accel is %.1f\r\n",
                      (double)requested_accel,
                      (double)xy_accel);
        warned++;
    }

    menu_build_center_square(motion, side_mm, &expected_x, &expected_y, NULL, NULL);
    menu_run_center_square(motion, side_mm, 2400.0f);
    end_x = motion->pos_x;
    end_y = motion->pos_y;
    if (fabsf(end_x - expected_x) < 0.5f && fabsf(end_y - expected_y) < 0.5f) {
        UARTDBG_Print("[PASS] accelerated center square finished at expected position (%.3f, %.3f)\r\n",
                      (double)end_x,
                      (double)end_y);
        passed++;
    } else {
        UARTDBG_Print("[WARN] center square ended at (%.3f, %.3f), expected near (%.3f, %.3f)\r\n",
                      (double)end_x,
                      (double)end_y,
                      (double)expected_x,
                      (double)expected_y);
        warned++;
    }

    if (extra_cmd[0] != '\0') {
        menu_run_gcode_direct(extra_cmd);
        UARTDBG_Print("[PASS] custom bench G-code executed\r\n");
        passed++;
    }

    menu_queue_gcode("g90");
    UARTDBG_Print("[PASS] parser task queue path exercised\r\n");
    passed++;

    menu_print_thermal_snapshot();
    UARTDBG_Print("Bench summary: PASS=%lu WARN=%lu\r\n",
                  (unsigned long)passed,
                  (unsigned long)warned);
}

static void menu_status_group(void)
{
    uint8_t choice;

    for (;;) {
        menu_print_status_menu();
        choice = menu_wait_char();
        UARTDBG_Print("%c\r\n", choice);

        switch (menu_upper_ascii((char)choice)) {
            case '1':
                menu_print_status();
                menu_pause();
                break;
            case '2':
                menu_endstop_tests();
                menu_pause();
                break;
            case '3':
                menu_thermal_tests();
                menu_pause();
                break;
            case 'R':
                return;
            default:
                UARTDBG_Print("Unknown selection.\r\n");
                menu_pause();
                break;
        }
    }
}

static void menu_motion_group(void)
{
    uint8_t choice;

    for (;;) {
        menu_print_motion_menu();
        choice = menu_wait_char();
        UARTDBG_Print("%c\r\n", choice);

        switch (menu_upper_ascii((char)choice)) {
            case '1':
                menu_motor_tests();
                menu_pause();
                break;
            case '2':
                menu_homing_tests();
                menu_pause();
                break;
            case '3':
                menu_step_counter();
                menu_pause();
                break;
            case '4':
                menu_accel_tests();
                menu_pause();
                break;
            case 'R':
                return;
            default:
                UARTDBG_Print("Unknown selection.\r\n");
                menu_pause();
                break;
        }
    }
}

static void menu_thermal_group(void)
{
    uint8_t choice;

    for (;;) {
        menu_print_thermal_menu();
        choice = menu_wait_char();
        UARTDBG_Print("%c\r\n", choice);

        switch (menu_upper_ascii((char)choice)) {
            case '1':
                menu_thermal_tests();
                menu_pause();
                break;
            case 'R':
                return;
            default:
                UARTDBG_Print("Unknown selection.\r\n");
                menu_pause();
                break;
        }
    }
}

static void menu_bench_group(void)
{
    uint8_t choice;

    for (;;) {
        menu_print_bench_menu();
        choice = menu_wait_char();
        UARTDBG_Print("%c\r\n", choice);

        switch (menu_upper_ascii((char)choice)) {
            case '1':
                menu_system_bench();
                menu_pause();
                break;
            case '2':
                menu_pseudo_print_test();
                menu_pause();
                break;
            case 'R':
                return;
            default:
                UARTDBG_Print("Unknown selection.\r\n");
                menu_pause();
                break;
        }
    }
}

static void menu_gcode_group(void)
{
    uint8_t choice;

    for (;;) {
        menu_print_gcode_menu();
        choice = menu_wait_char();
        UARTDBG_Print("%c\r\n", choice);

        switch (menu_upper_ascii((char)choice)) {
            case '1':
                menu_raw_gcode();
                menu_pause();
                break;
            case 'R':
                return;
            default:
                UARTDBG_Print("Unknown selection.\r\n");
                menu_pause();
                break;
        }
    }
}

void UARTTestMenu_Task(void *argument)
{
    uint8_t choice;

    (void)argument;

    menu_wait_motion_ready();
    vTaskDelay(pdMS_TO_TICKS(500));

    UARTDBG_Print("\r\n[UART TEST] runtime diagnostics task ready\r\n");
    menu_print_main();

    for (;;) {
        choice = menu_wait_char();
        UARTDBG_Print("%c\r\n", choice);

        switch (menu_upper_ascii((char)choice)) {
            case '1':
                menu_status_group();
                break;
            case '2':
                menu_motion_group();
                break;
            case '3':
                menu_thermal_group();
                break;
            case '4':
                menu_bench_group();
                break;
            case '5':
                menu_gcode_group();
                break;
            case 'M':
                break;
            default:
                UARTDBG_Print("Unknown selection.\r\n");
                menu_pause();
                break;
        }

        menu_print_main();
    }
}
