/*
 * _globals.h
 *
 *  Created on: Feb 28, 2026
 *      Author: Quinton B. Cook
 */

#ifndef INC_QBC_GLOBALS_H_
#define INC_QBC_GLOBALS_H_

#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <motion.h>



// Queues
extern QueueHandle_t xMotionQueue;
extern QueueHandle_t xSDCardQueue;
extern QueueHandle_t xGCodeQueue;

// Semaphores
extern SemaphoreHandle_t xBedReadySem;
extern SemaphoreHandle_t xHotendReadySem;

// Task handles (useful for debugging and vTaskDelete if needed)
extern TaskHandle_t xParserTaskHandle;
extern TaskHandle_t xMotionTaskHandle;
extern TaskHandle_t xSDCardTaskHandle;

extern QueueHandle_t xGCodeQueue;
extern QueueHandle_t xMotionQueue;
extern QueueHandle_t xSDCardQueue;
extern QueueHandle_t xHotendQueue;
extern QueueHandle_t xBedQueue;
extern QueueHandle_t xFanQueue;


extern SemaphoreHandle_t xGArgsMutex;
extern SemaphoreHandle_t xMotionDoneSem;
extern SemaphoreHandle_t xHomingDoneSem;
extern SemaphoreHandle_t xSDStateMutex;
extern SemaphoreHandle_t xMotionStateMutex;
extern SemaphoreHandle_t xMotionUpdateMutex;
extern SemaphoreHandle_t xHeaterStateMutex;
extern SemaphoreHandle_t xHeaterUpdateMutex;
extern SemaphoreHandle_t xFanReadySem;
extern SemaphoreHandle_t xFanStateMutex;




//Structure Definitions
typedef struct _globals_{
	uint32_t integrityHead;
	char *cmd;
	uint8_t rdbuff;
	uint8_t cmdbuff;
	uint32_t integrityTail;

}QBC_Globals;
#ifndef MAIN
extern
#endif
QBC_Globals QBCGlo;


typedef struct _args{
	float x;
	float y;
	float z;
	float e;
	float f;
	float p;
	float s;
	uint8_t t;
	uint8_t xchk;
	uint8_t ychk;
	uint8_t zchk;
	uint8_t echk;
	uint8_t fchk;
}Args;
#ifndef PARSER
extern
#endif
Args GArgs;

typedef struct _commands{
	uint32_t CMDHead;
	uint8_t CmdArrSize;
	uint8_t CmdMAXSIZE;
	uint8_t CmdArrIter;
	uint32_t CMDTail;

}Commands;
#ifndef SDCARD
extern
#endif
Commands CommandLines;

//typedef enum {
//	MOTION_IDLE = 0,
//	MOTION_MOVE,
//	MOTION_HOME,
//	MOTION_COMPLETE,
//	MOTION_FAULT
//}MotionType_t;


#ifndef PARSER
extern
#endif

//typedef struct {
//	float hotend_target;
//	float bed_target;
//	float part_fan_speed;
//	uint8_t hotend_en;
//	uint8_t bed_en;
//	uint8_t part_fan_en;
//	Heater_RunState_t state;
//} ThermalCommand_t;

//temp heater enum

void QBC_InitializeGlobals(void);
int MatchSubString(char *chk, char *cmd); // matching for commands
void ParseArgs(char *cmd); // parse the commands to floats

char** SepArgs(char *cmd);



// GCode Commands
void AutoHome(char *cmd);
void ExtrusionLinearMv(char *cmd);
void NonExtrusionLinearMv(char *cmd);
void SetTool(char *cmd);
void GTest(char *cmd);



#endif /* INC_QBC_GLOBALS_H_ */
