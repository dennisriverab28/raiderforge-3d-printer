/*
 * qbc_globals.c — Dennis's integration bridge, Mar 2026
 * Allocates Quinton's Glo, implements InitializeGlobals + myprintf,
 * bridges GArgs to DDRGArgs.
 * DO NOT include both qbc_globals.h and globals_m.h in the same .c file.
 */

#define MAIN

#include "qbc_globals.h"
#include "uart_debug.h"
//#include "ddr_globals.h"
#include "uart_debug.h"
#include "heater_types.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//Queues
QueueHandle_t xGCodeQueue = NULL;
QueueHandle_t xMotionQueue = NULL;
QueueHandle_t xSDCardQueue = NULL;
QueueHandle_t xHotendQueue = NULL;
QueueHandle_t xBedQueue = NULL;
QueueHandle_t xFanQueue = NULL;

//Semaphores
SemaphoreHandle_t xGArgsMutex = NULL;
SemaphoreHandle_t xSDStateMutex = NULL;
SemaphoreHandle_t xMotionDoneSem = NULL;
SemaphoreHandle_t xHotendReadySem = NULL;
SemaphoreHandle_t xBedReadySem = NULL;
SemaphoreHandle_t xHomingDoneSem = NULL;
SemaphoreHandle_t xMotionStateMutex = NULL;
SemaphoreHandle_t xMotionUpdateMutex = NULL;
SemaphoreHandle_t xHeaterStateMutex = NULL;
SemaphoreHandle_t xHeaterUpdateMutex = NULL;
SemaphoreHandle_t xFanReadySem = NULL;
SemaphoreHandle_t xFanStateMutex = NULL;

//Mutex




/* myprintf — declared in sdcard.h, never implemented elsewhere → linker error.
 * Routes Quinton's debug output through the same UART as the rest of the firmware. */
void myprintf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UARTDBG_Print("%s", buf);
}

/* InitializeGlobals — sets up Glo so sdcard.c Glo.rdbuff/cmdbuff don't fault. */
void QBC_InitializeGlobals(void)
{
    QBCGlo.integrityHead = 0xBEEFCAFE;
    QBCGlo.rdbuff        = 255;
    QBCGlo.cmdbuff       = 128;
    QBCGlo.cmd           = NULL;
    QBCGlo.integrityTail = 0xBEEFCAFE;

    CommandLines.CMDHead    = 0xCAFEBABE;
    CommandLines.CmdArrSize = 10;
    CommandLines.CmdMAXSIZE = 128;
    CommandLines.CMDTail    = 0xCAFEBABE;

}

/* QBC_SyncArgs — call after Parse(). Copies GArgs into DDRGArgs. */
//void QBC_SyncArgs(void)
//{
//    DDRGArgs.x    = GArgs.x;
//    DDRGArgs.y    = GArgs.y;
//    DDRGArgs.z    = GArgs.z;
//    DDRGArgs.e    = GArgs.e;
//    DDRGArgs.f    = GArgs.f;
//    DDRGArgs.s    = GArgs.s;
//    DDRGArgs.p    = GArgs.p;
//    DDRGArgs.xchk = GArgs.xchk;
//    DDRGArgs.ychk = GArgs.ychk;
//    DDRGArgs.zchk = GArgs.zchk;
//    DDRGArgs.echk = GArgs.echk;
//}

/* ── Accessor functions ──────────────────────────────────────────────────────
 * system_bench.c includes display.h → globals_m.h which defines an EMPTY
 * 'Globals' struct. So Glo.rdbuff in system_bench.c would hit Matthew's empty
 * struct, not Quinton's. These wrappers expose Quinton's Glo fields safely
 * through qbc_bridge.h without any struct typedef in the caller's scope.     */

uint8_t QBC_GetRdbuff(void)     { return QBCGlo.rdbuff; }
uint8_t QBC_GetCmdbuff(void)    { return QBCGlo.cmdbuff; }
uint8_t QBC_GetGloIntact(void)  { return (QBCGlo.integrityHead == 0xBEEFCAFE &&
		QBCGlo.integrityTail == 0xBEEFCAFE) ? 1 : 0; }
uint8_t QBC_GetCmdArrSize(void) { return CommandLines.CmdArrSize; }
