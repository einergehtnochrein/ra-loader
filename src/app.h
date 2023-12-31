#ifndef __APP_H
#define __APP_H

#include "bsp.h"
#include "loader.h"

#define LOADER_VERSION                      6

#define HOST_CHANNEL_PING                   0
#define HOST_CHANNEL_GUI                    3
#define HOST_CHANNEL_FIRMWAREUPDATE         9
#define HOST_CHANNEL_CONFIGUPDATE           10

extern LOADER_Handle loaderTask;

LPCLIB_Result SYS_send2Host (int channel, const char *message);
LPCLIB_Result SYS_sendBreak (int durationMilliseconds);

#endif
