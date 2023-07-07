
#ifndef __BL652_H
#define __BL652_H

#include "lpclib.h"

typedef struct BL652_Context *BL652_Handle;

enum {
    BL652_MODE_COMMAND,
    BL652_MODE_VSP_COMMAND,
    BL652_MODE_VSP_BRIDGE,
};

#define BL652_MAKE_FIRMWARE_VERSION(a,b,c,d) \
    (((a) << 24) | ((b) << 16) | ((c) << 8) | ((d) << 0))

LPCLIB_Result BL652_open (
    UART_Handle uart,
    GPIO_Pin gpioNAUTORUN,
    GPIO_Pin gpioSIO02,
    GPIO_Pin gpioNRESET,
    BL652_Handle *pHandle);

LPCLIB_Result BL652_findBaudrate (BL652_Handle handle);
LPCLIB_Result BL652_readParameters (BL652_Handle handle);
LPCLIB_Result BL652_updateParameters (BL652_Handle handle);
LPCLIB_Result BL652_setMode (BL652_Handle handle, int mode);
LPCLIB_Result BL652_getFirmwareVersion (BL652_Handle handle, uint32_t *pFirmwareVersion);

#endif
