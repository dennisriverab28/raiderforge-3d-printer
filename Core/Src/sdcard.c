/*
 * sdcard.c
 *
 *  Created on: Feb 28, 2026
 *      Author: Quinton B. Cook
 */

#define SDCARD

#include "app_fatfs.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "sdcard.h"
#include "user_diskio.h"
#include <qbc_globals.h>
#include <uart_debug.h>

extern UART_HandleTypeDef huart2;

/*void change_state(sd_card *state, SDCard_States_t new_state){
	state = new_state;
}*/

void Initialize_SDCard(void){
	sdc.state = Init;
}


char* IgnoreComments(char *line){
	char comm = ';';
	char *uncomm = strchr(line, comm);
	if(uncomm != NULL)
		*uncomm = '\0';
	if(line[0] != '\0'){
		QBCGlo.cmd[QBCGlo.cmdbuff-1] = '\0';
	}
	return line;
}

void SDCard_SetState(SDCard_States_t newState){
	sdc.state = newState;

}

SDCard_States_t SDCard_GetState(void){

    return sdc.state;

}

void SDCard_Task(void *pvParameters){
    (void)pvParameters;
    static FATFS FatFs;
    static FIL fil;
    static DIR dir;
    FRESULT fres;
    FILINFO finfo;
    static BYTE readbuff[128];
    static TCHAR *rres;
    static char selected_file[64];
    static char queue_line[128];
    char comm = ';';
    char *uncomm;
    size_t line_len;
	SDCard_SetState(Init);

    for(;;)
    {
        switch(SDCard_GetState())
        {
	            case Init:
		            	Initialize_SDCard();
		                f_mount(NULL, "", 1);
		                fres = f_mount(&FatFs, "", 1);
		                if(fres != FR_OK){
	                    SDCard_SetState(SDERROR);
	                } else {
	                    SDCard_SetState(List);
		                }
	                break;

            case IDLE:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case List:
                selected_file[0] = '\0';
                fres = f_opendir(&dir, "/");
                if(fres == FR_OK){
                    while(1){
                        fres = f_readdir(&dir, &finfo);
                        if(fres != FR_OK || finfo.fname[0] == '\0') break;
                        if(!(finfo.fattrib & AM_DIR)){
                            if (selected_file[0] == '\0') {
                                strncpy(selected_file, finfo.fname, sizeof(selected_file) - 1U);
                                selected_file[sizeof(selected_file) - 1U] = '\0';
                            }
                        }
                    }
	                    if (selected_file[0] != '\0') {
	                        SDCard_SetState(Open);
		                    } else {
	                            fres = FR_NO_FILE;
	                        SDCard_SetState(SDERROR);
	                    }
	                } else {
	                    SDCard_SetState(SDERROR);
	                }
                break;

	            case Open:
		                fres = f_open(&fil, selected_file, FA_READ);
		                if(fres != FR_OK){
	                    SDCard_SetState(SDERROR);
	                } else {
		                    SDCard_SetState(ReadLn);
	                }
	                break;

            case ReadLn:
                memset(readbuff, 0, sizeof(readbuff));
                rres = f_gets((TCHAR*)readbuff, sizeof(readbuff), &fil);
                readbuff[sizeof(readbuff) - 1] = '\0';
                if(rres != NULL){
                    uncomm = strchr((const char *)readbuff, comm);
                    if(uncomm != NULL) *uncomm = '\0';
                    line_len = strlen((const char *)readbuff);
                    while(line_len > 0U &&
                          (readbuff[line_len - 1U] == '\r' ||
                           readbuff[line_len - 1U] == '\n' ||
                           readbuff[line_len - 1U] == ' '  ||
                           readbuff[line_len - 1U] == '\t')){
                        readbuff[line_len - 1U] = '\0';
                        line_len--;
                    }

	                    if(readbuff[0] != '\0'){
	                        memset(queue_line, 0, sizeof(queue_line));
	                        strncpy(queue_line, (const char *)readbuff, sizeof(queue_line) - 1U);
                            xQueueSend(xGCodeQueue, queue_line, portMAX_DELAY);
	                    }
		                } else if (f_eof(&fil)) {
		                    SDCard_SetState(CLOSE);
		                } else {
	                        fres = FR_DISK_ERR;
		                    SDCard_SetState(SDERROR);
		                }
	                if(SDCard_GetState() == ReadLn && f_eof(&fil)){
		                    SDCard_SetState(CLOSE);
	                }
	                vTaskDelay(pdMS_TO_TICKS(10));
	                break;

	            case CLOSE:
	                fres = f_close(&fil);
		                SDCard_SetState(IDLE);
	                break;

            case SDERROR:
	                f_mount(NULL, "", 1);
	                USER_disk_reset();
                    SDCard_SetState(IDLE);
	                break;
        }
    }
}
