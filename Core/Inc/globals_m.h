/*
 * globals_m.h
 *
 *  Created on: Feb 4, 2026
 *      Author: Matthew Garcia
 */

#ifndef INC_GLOBALS_M_H_
#define INC_GLOBALS_M_H_

#include <stdint.h>
#include <main.h>




typedef struct _globals {

} Globals;


//This allows the other libraries to see hspi
extern SPI_HandleTypeDef hspi3;



#ifndef main
extern
#endif
Globals Glo; //This allows us to glo.variable

#endif /* INC_GLOBALS_M_H_ */
