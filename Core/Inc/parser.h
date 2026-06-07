/*
 * parser.h
 *
 *  Created on: Feb 8, 2026
 *      Author: Quinton B. Cook
 */

#ifndef SRC_PARSER_H_
#define SRC_PARSER_H_

typedef enum {
	PARSER_INIT = 0,
	PARSER_SDCARD,
	PARSER_SCREEN,
	PARSER_FAULT
}Parser_State_t;

typedef struct {
	Parser_State_t state;
}Parser_t;

void Parser_task(void *pvParameters);
void GetArgs(char *cmd);
void Parse(char *cmd);

#endif /* SRC_PARSER_H_ */
