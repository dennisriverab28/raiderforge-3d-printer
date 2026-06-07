/*
 * ddr_globals.c
 *
 *  Created on: Mar 2026
 *      Author: Cherryman125
 *
 *  Defines DDRlo and DDRGArgs — one definition, everyone else externs via ddr_globals.h.
 *  DDR_MAIN and DDR_OWNER must be defined HERE ONLY before the include.
 */

#define DDR_MAIN
#define DDR_OWNER
#include "ddr_globals.h"

void DDR_InitGlobals(void)
{
    DDRlo.integrityHead = 0xDEADBEEF;
    DDRlo.state         = DDR_IDLE;
    DDRlo.cmd_ready     = 0;
    DDRlo.integrityTail = 0xDEADBEEF;

    DDRGArgs.x    = 0.0f; DDRGArgs.y    = 0.0f;
    DDRGArgs.z    = 0.0f; DDRGArgs.e    = 0.0f;
    DDRGArgs.f    = 0.0f; DDRGArgs.s    = 0.0f;
    DDRGArgs.p    = 0.0f;
    DDRGArgs.xchk = 0;    DDRGArgs.ychk = 0;
    DDRGArgs.zchk = 0;    DDRGArgs.echk = 0;
    DDRGArgs.schk = 0;    DDRGArgs.pchk = 0;
}
