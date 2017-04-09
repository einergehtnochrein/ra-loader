#ifndef __APP_H
#define __APP_H

#include "loader.h"

#define FIRMWARE_START_ADDRESS              0x00008000
#define FIRMWARE_END_ADDRESS                0x0006FFF0

#define HOST_CHANNEL_PING                   0
#define HOST_CHANNEL_GUI                    3
#define HOST_CHANNEL_FIRMWAREUPDATE         9

extern LOADER_Handle loaderTask;

LPCLIB_Result SYS_send2Host (int channel, const char *message);

#endif
