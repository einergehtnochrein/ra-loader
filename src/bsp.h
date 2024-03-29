
#ifndef __BSP_H
#define __BSP_H

#include "app.h"
#include "lpclib.h"


#define GPIO_ADF7021_CE                     GPIO_1_4
#define GPIO_ADF7021_SLE                    GPIO_1_3
#define GPIO_BLE_RESET                      GPIO_1_12
#define GPIO_BLE_AUTORUN                    GPIO_0_8
#define GPIO_BLE_MODESEL                    GPIO_0_2
#define GPIO_CHARGER_LED1                   GPIO_0_7
#define GPIO_ENABLE_VDDA                    GPIO_0_6
#define GPIO_FORCE_LOADER                   GPIO_0_22
#define GPIO_FORCE_LOADER2                  GPIO_0_4
#define GPIO_LNA_GAIN                       GPIO_1_9
#define GPIO_VBAT_ADC_ENABLE                GPIO_0_0

#define FIRMWARE_START_ADDRESS              0x00008000
#define FIRMWARE_END_ADDRESS                0x00037FF0

/** To be called as the first step in SystemInit(). */
void BSP_systemInit (void);

#endif

