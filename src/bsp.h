
#ifndef __BSP_H
#define __BSP_H

#include "app.h"
#include "lpclib.h"

/* GPIO functions */
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


/** To be called as the first step in SystemInit(). */
void BSP_systemInit (void);

#endif

