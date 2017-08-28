
#ifndef __CONFIG_H
#define __CONFIG_H

#include "lpclib.h"

__PACKED(struct _Config_t {
    float referenceFrequency;
    char nameBluetooth[44];
    uint16_t usbVID;
    uint16_t usbPID;
    uint16_t usbVERSION;
    uint16_t __reserved36__;
    wchar_t usbVendorString[48];
    wchar_t usbProductString[32];
    float rssiCorrectionHighGain;
    float rssiCorrectionLowGain;
    uint8_t __reservedD8__[32];
});

const struct _Config_t * CONFIG_getConfig (void);

#endif
