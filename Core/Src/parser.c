/*
 * parser.c
 *
 *  Created on: Feb 5, 2026
 *      Author: Quinton B. Cook
 */
#define PARSER

#include "parser.h"
#include "qbc_globals.h"
#include "sdcard.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <gcodefuncs.h>
#include "uart_debug.h"

Parser_t parser;

int MatchSubString(char *chk, char *cmd){
	uint8_t i;
	size_t chk_len = strlen(chk);
	size_t cmd_len = strlen(cmd);
	if(cmd_len < chk_len){
		return 0;
	}
/*	uint8_t n;
	for(n = 0; n<strlen(cmd); n++){
		myprintf("Loop iter(%d) cmd[i] = %c\r\n",n,cmd[n]);
		if(cmd[n] == ' ' && n >= strlen(chk)){
			myprintf("Checked to n(%d) with chk length(%d)\r\n",n,strlen(chk));
			myprintf("Check returned false: Loop 1 \r\n");
			return 0;
		}
	}*/
	for(i = 0; i<strlen(chk); i++){
		if(chk[i] != cmd[i]){
			return 0;
		}
	}
	return 1;
}

static void TrimLine(char *cmd)
{
	size_t len;

	if(cmd == NULL){
		return;
	}

	len = strlen(cmd);
	while(len > 0U && (cmd[len - 1U] == '\r' || cmd[len - 1U] == '\n' ||
			cmd[len - 1U] == ' ' || cmd[len - 1U] == '\t')){
		cmd[len - 1U] = '\0';
		len--;
	}
}


void Parse(char *cmd){
		int lcd_msg = 0;
		if (cmd == NULL || cmd[0] == '\0')
			return;

		TrimLine(cmd);
		if(cmd[0] == '\0')
			return;

		int32_t i;


		// set all letters to lowercase
		lcd_msg = MatchSubString("M117 ", cmd);
		if(!lcd_msg){
			for(i = 0; i<strlen(cmd); i++){
				if(cmd[i] >= 'A' && cmd[i] <= 'Z'){
					cmd[i] = cmd[i] + 32;
				}
			}
		}
		else SetLCDMessage(cmd);
		if(lcd_msg)
			return;



		// Begin Parsing
		if(MatchSubString("g1 ", cmd))
			ExtrusionLinearMv(cmd);
		else if(MatchSubString("g0 ", cmd))
			NonExtrusionLinearMv(cmd);
		else if(MatchSubString("g92 ", cmd))
			SetPos(cmd);
		else if(MatchSubString("m106 ", cmd))
			SetFanSpeed(cmd);
		else if(MatchSubString("m107", cmd))
			SetFanOff();
		else if(MatchSubString("m104 ", cmd))
			SetHotendTemperature(cmd);
		else if(MatchSubString("m221 ", cmd))
			SetFlowPercent(cmd);
		else if(MatchSubString("m140 ", cmd))
			SetBedTemp(cmd);
		else if(MatchSubString("g21 ", cmd))
			SetUnitsMM();
		else if(MatchSubString("g28", cmd)){
			AutoHome(cmd);
		}
		else if(MatchSubString("g90", cmd))
			AbsPos();
		else if(MatchSubString("g91", cmd))
			RelPos();
		else if(MatchSubString("m82", cmd))
			EAbsolute();
		else if(MatchSubString("m83", cmd))
			ERelative();
		else if(MatchSubString("m84 ", cmd))
			DisableSteppers();
		else if(MatchSubString("m109 ", cmd))
			WaitForHotendTemperature(cmd);
		else if(MatchSubString("m190 ", cmd))
			WaitForBedTemp(cmd);
		else if(MatchSubString("m204 ", cmd))
			SetStartAccel(cmd);
		else if(MatchSubString("t0 ", cmd))
			SetTool(cmd);
		else
		{
			CmdNotFound(cmd);
		}

	/*	else if(MatchSubString("g2 ", cmd))
			ClkCntrArcMv(cmd);
		else if(MatchSubString("g3 ", cmd))
			CntrClkCntrArcMv(cmd);
		else if(MatchSubString("g4 ", cmd))
			Dwell(cmd);
		else if(MatchSubString("g6 ", cmd))
			DirectStepperMove(cmd);
		else if(MatchSubString("g10 ", cmd))
			Retract(cmd);
		else if(MatchSubString("g11 ", cmd))
			Recover(cmd);
		else if(MatchSubString("g12 ", cmd))
			CleanNozzle(cmd);
		else if(MatchSubString("g20 ", cmd))
			SetUnitsInch(cmd);
	*/

	/*	else if(MatchSubString("g26 ", cmd))
			MeshValidationPattern(cmd);
		else if(MatchSubString("g27 ", cmd))
			ParkToolhead(cmd);


		else if(MatchSubString("g280 ", cmd)){
			myprintf("Test Function");
			GTest(cmd);
		}


		else if(MatchSubString("g29 ", cmd))
			BedLevelling(cmd);
		else if(MatchSubString("g30 ", cmd))
			SingleZProbe(cmd);
		else if(MatchSubString("g35 ", cmd))
			TrammingAssist(cmd);
		else if(MatchSubString("g60 ", cmd))
			StorePos(cmd);
		else if(MatchSubString("g61 ", cmd))
			RetStorePos(cmd);
		else if(MatchSubString("g76 ", cmd))
			PrbTempCali(cmd);
		else if(MatchSubString("g80 ", cmd))
			CancelCurrentMotionMode(cmd);
	*/
	/*	else if(MatchSubString("g91 ", cmd))
			RelPos(cmd);
		else if(MatchSubString("m0 ", cmd))
			UnconditionalStop(cmd);
		else if(MatchSubString("m1 ", cmd))
			UnconditionalStop(cmd);
		else if(MatchSubString("m17 ", cmd))
			EnableSteppers(cmd);
		else if(MatchSubString("m18 ", cmd))
			DisbaleSteppers(cmd);
		else if(MatchSubString("m20 ", cmd))
			ListSDCard(cmd);
		else if(MatchSubString("m21 ", cmd))
			InitSDCard(cmd);
		else if(MatchSubString("m22 ", cmd))
			EjectSDCard(cmd);
		else if(MatchSubString("m23 ", cmd))
			SelectSDFile(cmd);
		else if(MatchSubString("m24 ", cmd))
			StartResumePrint(cmd);
		else if(MatchSubString("m25 ", cmd))
			PausePrint(cmd);
		else if(MatchSubString("m26 ", cmd))
			SetSDPosition(cmd);
		else if(MatchSubString("m27 ", cmd))
			ReportSDPrintStatus(cmd);
		else if(MatchSubString("m30 ", cmd))
			DeleteSDfile(cmd);
		else if(MatchSubString("m31 ", cmd))
			ReportPrintTime(cmd);
		else if(MatchSubString("m32 ", cmd))
			SelectStart(cmd);
		else if(MatchSubString("m33 ", cmd))
			GetLongPath(cmd);
		else if(MatchSubString("m34 ", cmd))
			SDCardSorting(cmd);
		else if(MatchSubString("m42 ", cmd))
			SetPinState(cmd);
		else if(MatchSubString("m43 ", cmd))
			TogglePins(cmd);
		else if(MatchSubString("m48 ", cmd))
			ProbeRepeatTest(cmd);
		else if(MatchSubString("m73 ", cmd))
			SetPrintProgress();
		else if(MatchSubString("m75 ", cmd))
			StartPrintJobTimer();
		else if(MatchSubString("m76 ", cmd))
			PausePrintJobTimer();
		else if(MatchSubString("m77 ", cmd))
			StopPrintJobTimer();
		else if(MatchSubString("m78 ", cmd))
			PrintJobStats();
		else if(MatchSubString("m80 ", cmd))
			PowerOn();
		else if(MatchSubString("m81 ", cmd))
			PowerOff();
	*/

	/*	else if(MatchSubString("m83 ", cmd))
			ERelative();
		else if(MatchSubString("m85 ", cmd))
			InactivieyShutdown();
		else if(MatchSubString("m86 ", cmd))
			HotendIdleTimeout();
		else if(MatchSubString("m87 ", cmd))
			DisableHotendIdleTimeout();
		else if(MatchSubString("m92 ", cmd))
			SetAxisStepsPerUnit();
		else if(MatchSubString("m100 ", cmd))
			FreeMemory();
		else if(MatchSubString("m102 ", cmd))
			ConfigureBedDistanceSensor();

		else if(MatchSubString("m105 ", cmd))
			ReportTemperatures();

		else if(MatchSubString("m107 ", cmd))
			FanOff();
		else if(MatchSubString("m108 ", cmd))
			BreakandConinue();

		else if(MatchSubString("m110 ", cmd))
			SetGetLineNumber();
		else if(MatchSubString("m111 ", cmd))
			DebugLevel();
		else if(MatchSubString("m112 ", cmd))
			FullShutdown();
		else if(MatchSubString("m113 ", cmd))
			HostKeepalive();
		else if(MatchSubString("m114 ", cmd))
			GetCurrentPos();
		else if(MatchSubString("m115 ", cmd))
			FirmwareInfo();

		else if(MatchSubString("m118 ", cmd))
			SerialPrint();
		else if(MatchSubString("m119 ", cmd))
			EndstopStates();
		else if(MatchSubString("m120 ", cmd))
			EnableEndstops();
		else if(MatchSubString("m121 ", cmd))
			DisableEndstops();
		else if(MatchSubString("m122 ", cmd))
			TMCDebug();
		else if(MatchSubString("m123 ", cmd))
			FanTach();
		else if(MatchSubString("m125 ", cmd))
			ParkHead();

		else if(MatchSubString("m141 ", cmd))
			SetChamberTemp();
		else if(MatchSubString("m145 ", cmd))
			SetMatPreset();
		else if(MatchSubString("m149 ", cmd))
			SetTempUnits();
		else if(MatchSubString("m150 ", cmd))
			SetRGBColor();
		else if(MatchSubString("m154 ", cmd))
			PosAutoRep();
		else if(MatchSubString("m155 ", cmd))
			TempAutoRep();

		else if(MatchSubString("m191 ", cmd))
			WaitForChamberTemp();
		else if(MatchSubString("m192 ", cmd))
			WaitForProbeTemp();
		else if(MatchSubString("m200 ", cmd))
			VolExtrusionDiameter();
		else if(MatchSubString("m201 ", cmd))
			PrintTravelMvLmts();
		else if(MatchSubString("m203 ", cmd))
			SetMaxFeedrate();

		else if(MatchSubString("m205 ", cmd))
			SetAdvancedSettings();
		else if(MatchSubString("m206 ", cmd))
			SetHomeOffsets();
		else if(MatchSubString("m207 ", cmd))
			FirmRetrSettings();
		else if(MatchSubString("m208 ", cmd))
			FirmwareRecoverSettings();
		else if(MatchSubString("m209 ", cmd))
			SetAutoRetract();
		else if(MatchSubString("m210 ", cmd))
			HomingFeedrate();
		else if(MatchSubString("m211 ", cmd))
			SoftwareEndstops(cmd);
		else if(MatchSubString("m217 ", cmd))
			FilamentSwapParameters();
		else if(MatchSubString("m218 ", cmd))
			SetHotendOffset();
		else if(MatchSubString("m220 ", cmd))
			SetFeedratePercent();

		else if(MatchSubString("m290 ", cmd))
			BabyStep();
		else if(MatchSubString("m300 ", cmd))
			PlayTone();
		else if(MatchSubString("m301 ", cmd))
			SetHotendPID();
		else if(MatchSubString("m302 ", cmd))
			ColdExtrude();
		else if(MatchSubString("m303 ", cmd))
			PIDAutoTune();
		else if(MatchSubString("m304 ", cmd))
			SetBedPID();
		else if(MatchSubString("m305 ", cmd))
			UserThrmstrParams();
		else if(MatchSubString("m306 ", cmd))
			ModelPredictTempCntrl();
		else if(MatchSubString("m309 ", cmd))
			SetChambPID();
		else if(MatchSubString("m350 ", cmd))
			SetMicroStep();
		else if(MatchSubString("m351 ", cmd))
			SetMicroStepPins();
		else if(MatchSubString("m355 ", cmd))
			CaseLightCntrl();
		else if(MatchSubString("m400 ", cmd))
			FinishMoves();
		else if(MatchSubString("m402 ", cmd))
			StowProbe();
		else if(MatchSubString("m410 ", cmd))
			QuickStop();
		else if(MatchSubString("m412 ", cmd))
			FilamentRunout();
		else if(MatchSubString("m413 ", cmd))
			PwrLossRecover();
		else if(MatchSubString("m414 ", cmd))
			LCDLang();
		else if(MatchSubString("m420 ", cmd))
			BedLevelState();
		else if(MatchSubString("m421 ", cmd))
			SetMeshVal();
		else if(MatchSubString("m422 ", cmd))
			SetZMotXY();
		else if(MatchSubString("m425 ", cmd))
			BacklashCompensation();
		else if(MatchSubString("m428 ", cmd))
			HomeOffsetsHere();
		else if(MatchSubString("m430 ", cmd))
			PwrMonitor();
		else if(MatchSubString("m486 ", cmd))
			CnclObjs(cmd);
		else if(MatchSubString("m493 ", cmd))
			FixedTimeMotion(cmd);
		else if(MatchSubString("m494 ", cmd))
			FTMotionTrajSmooth(cmd);
		else if(MatchSubString("m500 ", cmd))
			SaveSettings(cmd);
		else if(MatchSubString("m501 ", cmd))
			RestoreSettings(cmd);
		else if(MatchSubString("m502 ", cmd))
			FactoryReset(cmd);
		else if(MatchSubString("m503 ", cmd))
			ReportSettings(cmd);
		else if(MatchSubString("m504 ", cmd))
			ValidateEEPROM(cmd);
		else if(MatchSubString("m524 ", cmd))
			AbortPrint(cmd);
		else if(MatchSubString("m540 ", cmd))
			EndstopAbortSD(cmd);
		else if(MatchSubString("m550 ", cmd))
			MachineName(cmd);
		else if(MatchSubString("m569 ", cmd))
			SetTMCStepMode(cmd);
		else if(MatchSubString("m575 ", cmd))
			SerialBaudRate(cmd);
		else if(MatchSubString("m592 ", cmd))
			NonlinearExtrusionControl();
		else if(MatchSubString("m593 ", cmd))
			ZVInputShaping(cmd);
		else if(MatchSubString("m600 ", cmd))
			FilamentChng(cmd);
		else if(MatchSubString("m603 ", cmd))
			ConfigFilChng(cmd);
		else if(MatchSubString("m666 ", cmd))
			DualEndstopOffsets(cmd);
		else if(MatchSubString("m701 ", cmd))
			LoadFil(cmd);
		else if(MatchSubString("m702 ", cmd))
			UnloadFil(cmd);
		else if(MatchSubString("m710 ", cmd))
			ControllerFanSettings(cmd);
		else if(MatchSubString("m851 ", cmd))
			XYZProbeOffset(cmd);
		else if(MatchSubString("m852 ", cmd))
			BedSkewComp(cmd);
		else if(MatchSubString("m871 ", cmd))
			ProbeTempConfig(cmd);
		else if(MatchSubString("m906 ", cmd))
			StepMotCurrent(cmd);
		else if(MatchSubString("m911 ", cmd))
			TMCOTPerWarn(cmd);
		else if(MatchSubString("m912 ", cmd))
			ClearTMCOTPreWarn(cmd);
		else if(MatchSubString("m999 ", cmd))
			STOPRestart(cmd);

		else if(MatchSubString("t1 ", cmd))
			SetTool(cmd);
*/


}

char** SepArgs(char *cmd){
	uint8_t i = 0;
	const char *deli = " ";
	static char *args[10];
	char *token = strtok(cmd, deli);
	while(token != NULL && i < 10){
		args[i] = token;
		i++;
		token = strtok(NULL, " ");
	}

	args[i] = NULL;
	return args;
}

void ParseArgs(char *cmd){
	char **args = SepArgs(cmd);
	uint8_t i;
	char name;
	char num[32];
	size_t copy_len;
	float flt;
	GArgs.x = 0.0f;
	GArgs.y = 0.0f;
	GArgs.z = 0.0f;
	GArgs.e = 0.0f;
	GArgs.f = 0.0f;
	GArgs.s = 0.0f;
	GArgs.p = 0.0f;
	GArgs.t = 0U;
	GArgs.xchk = 0;
	GArgs.ychk = 0;
	GArgs.zchk = 0;
	GArgs.echk = 0;
	GArgs.fchk = 0;
		for(i = 0; args[i] != NULL; i++){
			name = args[i][0];
			if(strlen(args[i]) > 1){
				memset(num, 0, sizeof(num));
				copy_len = strlen(args[i] + 1);
				if(copy_len >= sizeof(num)){
					copy_len = sizeof(num) - 1U;
				}
				memcpy(num, args[i] + 1, copy_len);
				num[copy_len] = '\0';
				flt = atof(num);
				if(name == 'm' || name == 'g'){
					continue;
				}
				else if(name == 'x'){
					GArgs.x = flt;
					GArgs.xchk = 1;
				}
				else if(name == 'y'){
					GArgs.y = flt;
					GArgs.ychk = 1;
				}
				else if(name == 'z'){
					GArgs.z = flt;
					GArgs.zchk = 1;
				}
				else if(name == 'e'){
					GArgs.e = flt;
					GArgs.echk = 1;
				}
				else if(name == 'f'){
					GArgs.f = flt;
					GArgs.fchk = 1;
				}
				else if(name == 's'){
					GArgs.s = flt;
				}
				else if(name == 'p'){
					GArgs.p = flt;
				}
				else if(name == 't'){
					GArgs.t = atoi(&args[i][1]);
				}
				else{
				}
			}
		}
}

void parser_set_state(Parser_State_t newState)
{
	parser.state = newState;
}

static Parser_State_t parser_get_state()
{
	return parser.state;
}
void Parser_task(void *pvParameters)
{
	(void)pvParameters;
	char cmd[128];
		for(;;)
		{
		switch(parser_get_state())
		{
			case PARSER_INIT:
				GCode_Functions_Init();
					parser_set_state(PARSER_SDCARD);
					break;
				case PARSER_SDCARD:
					if(xQueueReceive(xGCodeQueue, cmd, portMAX_DELAY) == pdTRUE){
						Parse(cmd);
					}
				break;
			case PARSER_SCREEN:
//				if(xQueueReceive(xScreenQueue, cmd, portMAX_DELAY) == pdTRUE){
//					Parse(*cmd);
//				}
				break;
			case PARSER_FAULT:

				break;
		}

	}
	vTaskDelay(pdMS_TO_TICKS(10));
}
/* might be usefull for clearing memory
 * for (i = 0; i<strlen(cmd); i++){
 * 	cmd[i] = cmd[i+1];
 * }
 */
