/*
 * heater_types.h
 *
 *  Created on: Apr 15, 2026
 *      Author: Quinton B. Cook
 */

#ifndef INC_HEATER_TYPES_H_
#define INC_HEATER_TYPES_H_

typedef enum {
    HEATER_STATE_INIT = 0,
    HEATER_STATE_IDLE,
	HEATER_STATE_OFF,
    HEATER_STATE_HEATING,
    HEATER_STATE_HOLD,
    HEATER_STATE_COOLING,
    HEATER_STATE_FAULT
} Heater_State_t;

typedef enum {
    FAN_STATE_INIT = 0,
    FAN_STATE_IDLE,
    FAN_STATE_SET,
	FAN_STATE_HOLD,
    FAN_STATE_FAULT
} Fan_State_t;

typedef struct {
	uint8_t hotend_en;
	float hotend_target;
	Heater_State_t state;

} Hotend_Command_t;

typedef struct {
	uint8_t bed_en;
	float bed_target;
	Heater_State_t state;

} Bed_Command_t;

typedef struct {
	uint8_t fan_en;
	float fan_target;
	Fan_State_t state;

} Fan_Command_t;

#endif /* INC_HEATER_TYPES_H_ */
