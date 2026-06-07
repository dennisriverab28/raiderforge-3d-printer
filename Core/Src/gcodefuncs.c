/*
 * gcodefuncs.c
 *
 *  Created on: Apr 10, 2026
 *      Author: Quinton B. Cook
 */


#define GCODEFUNCS

#include "parser.h"
#include "qbc_globals.h"
#include "sdcard.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <gcodefuncs.h>
#include <homing.h>
#include <heater_types.h>
#include <math.h>
#include <ddr_globals.h>
#include <stepper.h>
//#include <z_e_motion.h>
#include <semphr.h>
#include "uart_debug.h"

MotionCommand_t motion_command;
Hotend_Command_t hotend_command;
Bed_Command_t bed_command;
Fan_Command_t fan_command;
static uint32_t g_motion_trace_id = 1U;

static void QueueLinearMove(char *cmd, char trace_kind);

static uint32_t MotionTrace_NextId(void)
{
	return g_motion_trace_id++;
}

static float FeedrateMmMinToMmS(float feedrate_mm_min)
{
	if (feedrate_mm_min <= 0.0f) {
		return 0.0f;
	}

	return feedrate_mm_min / 60.0f;
}

void GCode_Functions_Init(void){
	motion_command.Absolute = 1;
	motion_command.E_Absolute = 1;
	motion_command.E_Flowrate = 1.0f;
	motion_command.E_en = 0;
	motion_command.e = 0;
	motion_command.speed_mm_s = 0;
	motion_command.trace_id = 0;
	motion_command.state = MOTION_INIT;
	motion_command.trace_kind = '?';
	motion_command.x = 0;
	motion_command.xchk = 0;
	motion_command.y = 0;
	motion_command.ychk = 0;
	motion_command.z = 0;
	motion_command.zchk = 0;
	motion_command.echk = 0;

	hotend_command.hotend_en = 0;
	hotend_command.hotend_target = 0;
	hotend_command.state = HEATER_STATE_INIT;

	bed_command.bed_en = 0;
	bed_command.bed_target = 0;
	bed_command.state = HEATER_STATE_INIT;

	fan_command.fan_en = 0;
	fan_command.fan_target = 0;
	fan_command.state = FAN_STATE_INIT;
}

static void QueueLinearMove(char *cmd, char trace_kind)
{
	ParseArgs(cmd);

	motion_command.x = GArgs.x;
	motion_command.y = GArgs.y;
	motion_command.z = GArgs.z;
	motion_command.e = GArgs.e;
	motion_command.xchk = GArgs.xchk;
	motion_command.ychk = GArgs.ychk;
	motion_command.zchk = GArgs.zchk;
	motion_command.echk = GArgs.echk;
	if (GArgs.fchk) {
		motion_command.speed_mm_s = FeedrateMmMinToMmS(GArgs.f);
	}
	motion_command.E_en = GArgs.echk ? 1U : 0U;
	motion_command.state = MOTION_MOVE;
	motion_command.trace_id = MotionTrace_NextId();
	motion_command.trace_kind = (uint8_t)trace_kind;

	(void)xSemaphoreTake(xMotionDoneSem, 0U);
	xQueueSend(xMotionQueue, &motion_command, portMAX_DELAY);
}

void ExtrusionLinearMv(char *cmd){
	QueueLinearMove(cmd, '1');
}

void NonExtrusionLinearMv(char *cmd){
	QueueLinearMove(cmd, '0');
}

void SetPos(char *cmd){
	ParseArgs(cmd);

	motion_command.x = GArgs.x;
	motion_command.y = GArgs.y;
	motion_command.z = GArgs.z;
	motion_command.e = GArgs.e;
	motion_command.xchk = GArgs.xchk;
	motion_command.ychk = GArgs.ychk;
	motion_command.zchk = GArgs.zchk;
	motion_command.echk = GArgs.echk;
	motion_command.state = MOTION_SET_POS;
	motion_command.trace_id = MotionTrace_NextId();
	motion_command.trace_kind = 'P';

	xQueueSend(xMotionQueue, &motion_command, portMAX_DELAY);

}

void SetFanSpeed(char *cmd){
	ParseArgs(cmd);
	if (xFanQueue == NULL || xFanReadySem == NULL) {
		return;
	}
	fan_command.fan_en = (GArgs.s > 0.0f) ? 1U : 0U;
	fan_command.fan_target = (GArgs.s <= 0.0f) ? 0.0f : ((GArgs.s * 100.0f) / 255.0f);
	if (fan_command.fan_target > 100.0f) {
		fan_command.fan_target = 100.0f;
	}
	fan_command.state = FAN_STATE_SET;

	(void)xSemaphoreTake(xFanReadySem, 0U);
	xQueueSend(xFanQueue, &fan_command, portMAX_DELAY);
	(void)xSemaphoreTake(xFanReadySem, pdMS_TO_TICKS(1000U));
	GArgs.s = 0;

}

void SetFanOff(void){
	if (xFanQueue == NULL || xFanReadySem == NULL) {
		return;
	}
	fan_command.fan_en = 0;
	fan_command.fan_target = 0.0f;
	fan_command.state = FAN_STATE_SET;

	(void)xSemaphoreTake(xFanReadySem, 0U);
	xQueueSend(xFanQueue, &fan_command, portMAX_DELAY);
	(void)xSemaphoreTake(xFanReadySem, pdMS_TO_TICKS(1000U));
}

void SetHotendTemperature(char *cmd){
	ParseArgs(cmd);
	hotend_command.hotend_en = 1;
	hotend_command.hotend_target = GArgs.s;
	hotend_command.state = HEATER_STATE_HEATING;

	xQueueSend(xHotendQueue, &hotend_command, portMAX_DELAY);
	//Heater_SetTarget(&DDRlo.hotend, GArgs.s);
	GArgs.s = 0;

}

void SetFlowPercent(char *cmd){
	ParseArgs(cmd);
	motion_command.E_Flowrate = GArgs.s / 100.0f;
	GArgs.s = 0;



}

void SetBedTemp(char *cmd){
	ParseArgs(cmd);

	bed_command.bed_en = 1;
	bed_command.bed_target = GArgs.s;
	bed_command.state = HEATER_STATE_HEATING;
	//BedHeater_SetTarget(GArgs.s);
	xQueueSend(xBedQueue, &bed_command, portMAX_DELAY);

	GArgs.s = 0;


}

void SetUnitsMM(void){
	return;
}

void AutoHome(char *cmd){
	ParseArgs(cmd);

	motion_command.xchk = GArgs.xchk;
	motion_command.ychk = GArgs.ychk;
	motion_command.zchk = GArgs.zchk;
	motion_command.state = MOTION_HOME;
	motion_command.trace_id = MotionTrace_NextId();
	motion_command.trace_kind = 'H';

	(void)xSemaphoreTake(xHomingDoneSem, 0U);
	xQueueSend(xMotionQueue, &motion_command, portMAX_DELAY);

	if(xSemaphoreTake(xHomingDoneSem, portMAX_DELAY) != pdTRUE){
		return;
	}
}

void AbsPos(void){
	motion_command.Absolute = 1;
}

void RelPos(void){
	motion_command.Absolute = 0;
}

void EAbsolute(void){
	motion_command.E_Absolute = 1;
}

void ERelative(void){
	motion_command.E_Absolute = 0;
}

void DisableSteppers(void){

//    Stepper_Enable(&DDRlo.motor_a, 0);
//    Stepper_Enable(&DDRlo.motor_b, 0);
//    Stepper_Enable(&DDRlo.motor_z, 0);
//    Stepper_Enable(&DDRlo.motor_e, 0);
}

void WaitForHotendTemperature(char *cmd){
	SetHotendTemperature(cmd);

	if(xSemaphoreTake(xHotendReadySem, pdMS_TO_TICKS(600000U)) != pdTRUE)
	{
		UARTDBG_Print("[gcodefuncs.c]: hotend wait timed out\r\n");
		return;
	}
	GArgs.s = 0;


}


void SetLCDMessage(char *cmd){
	ParseArgs(cmd);
}

void WaitForBedTemp(char *cmd){
	SetBedTemp(cmd);

	if(xSemaphoreTake(xBedReadySem, pdMS_TO_TICKS(600000U)) != pdTRUE)
	{
		UARTDBG_Print("[gcodefuncs.c]: bed wait timed out\r\n");
		return;
	}

	GArgs.s = 0;
}

void SetStartAccel(char *cmd){
	ParseArgs(cmd);
//	Stepper_SetAcceleration(&DDRlo.motor_a, GArgs.s);
//	Stepper_SetAcceleration(&DDRlo.motor_b, GArgs.s);
//	Stepper_SetAcceleration(&DDRlo.motor_z, GArgs.s);
//	Stepper_SetAcceleration(&DDRlo.motor_e, GArgs.s);
}

void SetTool(char *cmd){
	ParseArgs(cmd);
}

void CmdNotFound(char *cmd){
	// print something saying the command isn't found
}

