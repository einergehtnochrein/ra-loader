
#ifndef __TASK_LOADER_H
#define __TASK_LOADER_H

#include "lpclib.h"
#include "pt.h"


typedef enum {
    LOADER_MODE_FIRMWARE = 0,
    LOADER_MODE_CONFIG,
} LOADER_Mode;


typedef struct LOADER_Context *LOADER_Handle;

LPCLIB_Result LOADER_open (LOADER_Handle *pHandle);
LPCLIB_Result LOADER_setMode (LOADER_Handle handle, LOADER_Mode mode);
LPCLIB_Result LOADER_processCommand (LOADER_Handle handle, const char *commandLine);

/** Loader task. */
PT_THREAD(LOADER_thread (LOADER_Handle handle));

#endif
