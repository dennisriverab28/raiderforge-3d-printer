#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "u8g2.h"
#include <stdint.h>
#include "globals_m.h"

//This Extern is needed because the u8g2 is defined the in the display.c
extern u8g2_t u8g2;

#define MENU_COUNT 7
#define BOX_X_OFFSET  -3
#define BOX_Y          22

// declare — not define
extern int8_t menu_sel;
extern const uint8_t menu_x[MENU_COUNT];

/* USER CODE BEGIN PTD */
typedef enum {
    DISP_INIT,
    DISP_BOOT,
    DISP_MENU,
    DISP_TELE
} display_state_t;
/* USER CODE END PTD */



void init_display(void);
void boot_display(void);
void telemetry_display(void);
void menu_display(void);
int8_t menu_update_from_encoder(TIM_HandleTypeDef *htim);



#endif /* INC_DISPLAY_H_ */
