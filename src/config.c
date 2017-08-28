
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "config.h"

__SECTION(".config")
static const struct _Config_t _factorySettings = {
    .referenceFrequency = 12.8e6f,
    .nameBluetooth = "LAIRD BL652",
    .usbVID = 0x169C,
    .usbPID = 0x05DC,
    .usbVERSION = 0x0100,
    .usbVendorString = L"radiosonde.leckasemmel.de",
    .usbProductString = L"Sondengott Ra",
    .rssiCorrectionHighGain = -16.0f,
    .rssiCorrectionLowGain = 13.0f,
};

const struct _Config_t * CONFIG_getConfig (void)
{
    return &_factorySettings;
}

