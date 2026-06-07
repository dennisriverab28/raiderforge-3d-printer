/*
 * motion_types.h
 *
 *  Created on: Apr 14, 2026
 *      Author: Quinton B. Cook
 */

#ifndef INC_MOTION_TYPES_H_
#define INC_MOTION_TYPES_H_

#include <stdint.h>

typedef enum {

	MOTION_INIT = 0,
	MOTION_IDLE,
	MOTION_HOME,
	MOTION_MOVE,
	MOTION_SET_POS,
	MOTION_SET_ACCEL,
	MOTION_FAULT

} Motion_States_t;


typedef struct {
	float x;
	float y;
	float z;
	float e;
	float speed_mm_s;
	float accel;
	uint32_t trace_id;
	uint8_t xchk;
	uint8_t ychk;
	uint8_t zchk;
	uint8_t echk;
	uint8_t E_en;
	uint8_t Absolute;
	uint8_t E_Absolute;
	float E_Flowrate;
	uint8_t trace_kind;
	Motion_States_t state;
} MotionCommand_t;


#endif /* INC_MOTION_TYPES_H_ */
