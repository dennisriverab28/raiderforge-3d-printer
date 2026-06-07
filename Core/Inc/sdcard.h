/*
 * sdcard.h
 *
 *  Created on: Feb 28, 2026
 *      Author: Quinton B. Cook
 */

#ifndef INC_SDCARD_H_
#define INC_SDCARD_H_



typedef enum{
	Init = 0,
	IDLE,
	List,
	Open,
	ReadLn,
	CLOSE,
	SDERROR
} SDCard_States_t;
#ifndef SDCARD
extern
#endif
SDCard_States_t SD_States;

typedef struct{
	SDCard_States_t state;
}sd_card;
#ifndef SDCARD
extern
#endif
sd_card sdc;

void Initialize_SDCard(void);
void SDCard_Task(void *pvParameters);
char* IgnoreComments(char *line);
void SDCard_SetState(SDCard_States_t newState);
SDCard_States_t SDCard_GetState(void);
void SDCard(void);
void LoadMemory(char *line);
void myprintf(const char *fmt, ...);
void InitializeFATFS(void);
void OpenFile(void);
void ListFiles(void);

#endif /* INC_SDCARD_H_ */
