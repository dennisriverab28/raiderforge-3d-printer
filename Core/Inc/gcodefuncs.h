/*
 * gcodefuncs.h
 *
 *  Created on: Apr 11, 2026
 *      Author: Quinton B. Cook
 */

#ifndef INC_GCODEFUNCS_H_
#define INC_GCODEFUNCS_H_

#include "qbc_globals.h"
#include "parser.h"
#include "motion_types.h"
#include "heater.h"

extern QueueHandle_t xMotionQueue;
extern SemaphoreHandle_t xMotionCommands;

void GCode_Functions_Init(void);
void ExtrusionLinearMv(char *cmd);
void NonExtrusionLinearMv(char *cmd);
void SetPos(char *cmd);
void SetFanSpeed(char *cmd);
void SetFanOff(void);
void SetHotendTemperature(char *cmd);
void SetFlowPercent(char *cmd);
void SetBedTemp(char *cmd);
void SetUnitsMM(void);
void AutoHome(char *cmd);
void AbsPos(void);
void RelPos(void);
void EAbsolute(void);
void ERelative(void);
void DisableSteppers(void);
void WaitForHotendTemperature(char *cmd);
void SetLCDMessage(char *cmd);
void WaitForBedTemp(char *cmd);
void SetStartAccel(char *cmd);
void SetTool(char *cmd);
void CmdNotFound(char *cmd);

#endif /* INC_GCODEFUNCS_H_ */
