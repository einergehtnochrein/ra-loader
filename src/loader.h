
#ifndef __TASK_LOADER_H
#define __TASK_LOADER_H

#include "lpclib.h"
#include "pt.h"


typedef struct LOADER_Context *LOADER_Handle;

LPCLIB_Result LOADER_open (LOADER_Handle *pHandle);
LPCLIB_Result LOADER_processCommand (LOADER_Handle handle, const char *commandLine);

/** System management task. */
PT_THREAD(LOADER_thread (LOADER_Handle handle));

#endif
