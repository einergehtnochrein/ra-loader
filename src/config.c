
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "config.h"

__SECTION(".config")
static const struct _Config_t _factorySettingsDefault = {
    .version = 3,
    .baudrate = 115200.0f,

    .usbVID = 0x169C,
    .usbPID = 0x05DC,
    .usbVERSION = 0x0100,

    .referenceFrequency = 12.8e6,
    .rssiCorrectionLnaOn = -13.0,
    .rssiCorrectionLnaOff = 16.0,

    .nameBluetooth = "",
    .usbVendorString = L"leckasemmel.de/ra",
    .usbProductString = L"Sondengott Ra",

    .att_mtu = 23,
    .att_data_length = 20,
    .max_packet_length = 27,
};

__SECTION(".config")
static volatile const Config_t _factorySettingsUser;


/* Pointer to default configuration set at link time */
const Config_t *config_g = &_factorySettingsDefault;


