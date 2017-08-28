
#ifndef __BSP_H
#define __BSP_H

#include "app.h"
#include "lpclib.h"


#if (BOARD_RA == 1)
#define GPIO_BLE_RESET                      GPIO_1_11
#define GPIO_BLE_AUTORUN                    GPIO_0_7
#define GPIO_POWER_SWITCH                   GPIO_1_10
#define GPIO_BUTTON                         GPIO_1_9
#define GPIO_SSEL_DISP                      GPIO_1_4
#define GPIO_ENABLE_DISP                    GPIO_0_6
#define GPIO_ADF7021_CE                     GPIO_1_3
#define GPIO_ADF7021_SLE                    GPIO_1_5
#define GPIO_LNA_GAIN                       GPIO_0_19
#define GPIO_DEBUG1                         GPIO_0_22
#define GPIO_ENABLE_VDDA                    GPIO_0_30
#define GPIO_FORCE_LOADER                   GPIO_1_15

#define FIRMWARE_START_ADDRESS              0x00008000
#define FIRMWARE_END_ADDRESS                0x0006FFF0
#endif

#if (BOARD_RA == 2)
#define GPIO_ADF7021_CE                     GPIO_1_4
#define GPIO_ADF7021_SLE                    GPIO_1_3
#define GPIO_BLE_RESET                      GPIO_1_12
#define GPIO_BLE_AUTORUN                    GPIO_0_8
#define GPIO_BLE_MODESEL                    GPIO_0_2
#define GPIO_CHARGER_LED1                   GPIO_0_7
#define GPIO_ENABLE_VDDA                    GPIO_0_6
#define GPIO_FORCE_LOADER                   GPIO_0_22
#define GPIO_LNA_GAIN                       GPIO_1_9
#define GPIO_VBAT_ADC_ENABLE                GPIO_0_0

#define FIRMWARE_START_ADDRESS              0x00008000
#define FIRMWARE_END_ADDRESS                0x00037FF0
#endif

/** To be called as the first step in SystemInit(). */
void BSP_systemInit (void);

#endif

