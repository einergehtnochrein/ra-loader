
#include <stdio.h>

#include "lpclib.h"
#include "bsp.h"



/* To be called as the first step in SystemInit(). */
void BSP_systemInit (void)
{
    uint32_t *pFrom;
    uint32_t *pTo;
    int length;


    /* Allow user mode code to pend interrupts via NVIC->STIR */
    SCB->CCR |= SCB_CCR_USERSETMPEND_Msk;

    /* Individual exception handlers for Usage Fault, Bus Fault, MemManage Fault */
    SCB->SHCSR |= 0
               | SCB_SHCSR_USGFAULTENA_Msk
               | SCB_SHCSR_BUSFAULTENA_Msk
               | SCB_SHCSR_MEMFAULTENA_Msk
               ;

    /* Enable the FPU (allow full access to coprocessors CP10 and CP11) */
    SCB->CPACR |= (3 << 2*10) | (3 << 2*11);

    /* Pin initialization opens up complete address range for external flash
     * This is very likely a reuirement for the following scatter loading stage.
     */
    IOCON_open();

#if (BOARD_RA == 1)
    IOCON_configurePinDefault(PIN_P0_0,  PIN_FUNCTION_1, PIN_PULL_UP);          /* U0_RXD       BLE_RXD */
    IOCON_configurePinDefault(PIN_P0_1,  PIN_FUNCTION_1, PIN_PULL_NONE);        /* U0_TXD       BLE_TXD */
    IOCON_configurePinDefault(PIN_P0_2,  PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* U0_CTS       BLE_CTS */
    IOCON_configurePinDefault(PIN_P0_3,  PIN_FUNCTION_1, PIN_PULL_NONE);        /* U0_RTS       BLE_RTS */
    IOCON_configurePinDefault(PIN_P0_4,  PIN_FUNCTION_2, PIN_PULL_UP);          /* SPI0_SSELN2  internal use by EZH */
    IOCON_configurePinDefault(PIN_P0_6,  PIN_FUNCTION_0, PIN_PULL_UP);          /* GPIO_0_6     ENABLE_DISP */
    IOCON_configurePinDefault(PIN_P0_7,  PIN_FUNCTION_0, PIN_PULL_UP);          /* GPIO_0_7     BLE_AUTORUN */
//    IOCON_configurePinDefault(PIN_P0_9,  PIN_FUNCTION_2, PIN_PULL_UP);          /* SCT0_OUT2    MOSI_DISP */
//    IOCON_configurePinDefault(PIN_P0_10, PIN_FUNCTION_2, PIN_PULL_NONE);        /* SCT0_OUT3    SCK_DISP */
    IOCON_configurePinDefault(PIN_P0_11, PIN_FUNCTION_1, PIN_PULL_NONE);        /* SPI0_SCK     SCLK_D_PDM_CLK */
    IOCON_configurePinDefault(PIN_P0_12, PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* SPI0_MOSI    MOSI_D */
    IOCON_configurePinDefault(PIN_P0_13, PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* SPI0_MISO    MISO_D */
    IOCON_configurePinDefault(PIN_P0_19, PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_19    LNA_GAIN */
    IOCON_configurePinDefault(PIN_P0_21, PIN_FUNCTION_3, PIN_PULL_REPEATER);    /* CT32B3_MAT3  DISP_CLK */
    IOCON_configurePinDefault(PIN_P0_22, PIN_FUNCTION_4, PIN_PULL_REPEATER);    /* EZH22        DEBUG1 */
    IOCON_configurePinDefault(PIN_P0_23, PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_23    MUXOUT */
    IOCON_configurePinDefault(PIN_P0_24, PIN_FUNCTION_4, PIN_PULL_REPEATER);    /* EZH24        DEBUG2 */
//    IOCON_configurePinDefault(PIN_P0_25, PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_0_25    SCL */
//    IOCON_configurePinDefault(PIN_P0_26, PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_0_26    SDA */
    IOCON_configurePinDefault(PIN_P0_27, PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_27    SYNC_PDM_DATA */
    IOCON_configurePinDefault(PIN_P1_3,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_1_1     ADF7021_CE */
    IOCON_configurePinDefault(PIN_P1_4,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_1_4     SSEL_DISP */
    IOCON_configurePinDefault(PIN_P1_5,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_1_5     SLE_C */
    IOCON_configurePinDefault(PIN_P1_6,  PIN_FUNCTION_2, PIN_PULL_NONE);        /* SPI1_SCK     SCLK_C */
    IOCON_configurePinDefault(PIN_P1_7,  PIN_FUNCTION_2, PIN_PULL_REPEATER);    /* SPI1_MOSI    MOSI_C */
    IOCON_configurePinDefault(PIN_P1_8,  PIN_FUNCTION_2, PIN_PULL_REPEATER);    /* SPI1_MISO    MISO_C */
    IOCON_configurePinDefault(PIN_P1_9,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_1_9     BUTTON */
    IOCON_configurePinDefault(PIN_P1_10, PIN_FUNCTION_0, PIN_PULL_DOWN);        /* GPIO_1_10    POWER_SWITCH */
    IOCON_configurePinDefault(PIN_P1_11, PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_1_11    BLE_RESET */
#if PATCHLEVEL >= 1
    IOCON_configurePinDefault(PIN_P0_30, PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_30    ENABLE_VDDA */
#else
    IOCON_configurePinDefault(PIN_P0_14, PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_14    ENABLE_VDDA */
#endif
#endif

#if (BOARD_RA == 2)
    IOCON_configurePinDefault(PIN_P0_0,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_0_0     VBAT_ADC_ENABLE */
    IOCON_configurePinDefault(PIN_P0_2,  PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_2     BLE_MODESEL */
    IOCON_configurePinDefault(PIN_P0_4,  PIN_FUNCTION_0, PIN_PULL_UP);          /* GPIO_0_4     FORCE_LOADER2 */
    IOCON_configurePinDefault(PIN_P0_6,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_0_6     ENABLE_VDDA */
    IOCON_configurePinDefault(PIN_P0_7,  PIN_FUNCTION_0, PIN_PULL_UP);          /* GPIO_0_7     CHARGER_LED1 */
    IOCON_configurePinDefault(PIN_P0_8,  PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_8     BLE_AUTORUN */
    IOCON_configurePinDefault(PIN_P0_9,  PIN_FUNCTION_5, PIN_PULL_REPEATER);    /* FC3_CTS      BLE_CTS */
    IOCON_configurePinDefault(PIN_P0_10, PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_0_10    BLE_EXTRA1 */
    IOCON_configurePinDefault(PIN_P0_11, PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_0_11    BLE_EXTRA2 */
    IOCON_configurePinDefault(PIN_P0_12, PIN_FUNCTION_1, PIN_PULL_UP);          /* FC3_RXD      BLE_RXD */
    IOCON_configurePinDefault(PIN_P0_13, PIN_FUNCTION_1, PIN_PULL_NONE);        /* FC3_TXD      BLE_TXD */
    IOCON_configurePinDefault(PIN_P0_14, PIN_FUNCTION_5, PIN_PULL_NONE);        /* FC1_SCK      SCLK_D_PDM_CLK */
    IOCON_configurePinDefault(PIN_P0_15, PIN_FUNCTION_1, PIN_PULL_NONE);        /* FC3_RTS      BLE_RTS */
    IOCON_configurePinDefault(PIN_P0_22, PIN_FUNCTION_0, PIN_PULL_UP);          /* GPIO_0_22    FORCE_LOADER */
    IOCON_configurePinDefault(PIN_P0_29, PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* FC1_MOSI     MOSI_D */
    IOCON_configurePinDefault(PIN_P0_30, PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* FC1_MISO     MISO_D */
    IOCON_configurePinDefault(PIN_P1_0,  PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* PDM0_DATA    SYNC_PDM_DATA */
//    IOCON_configurePinDefault(PIN_P1_1,  PIN_FUNCTION_2, PIN_PULL_REPEATER);    /* SWO          SWO */
    IOCON_configurePinDefault(PIN_P1_2,  PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_1_2     MUXOUT */
    IOCON_configurePinDefault(PIN_P1_3,  PIN_FUNCTION_2, PIN_PULL_DOWN);        /* FC7_SSEL2    SLE_C */
    IOCON_configurePinDefault(PIN_P1_4,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_1_4     ADF7021_CE */
    IOCON_configurePin(PIN_P1_5, IOCON_makeConfigA(PIN_FUNCTION_0,              /* ADC0_8       VBAT_ADC */
                                                   PIN_PULL_NONE,
                                                   PIN_INPUT_NOT_INVERTED,
                                                   PIN_ADMODE_ANALOG,
                                                   PIN_FILTER_OFF,
                                                   PIN_OPENDRAIN_OFF));
    IOCON_configurePinDefault(PIN_P1_6,  PIN_FUNCTION_2, PIN_PULL_NONE);        /* FC7_SCK      SCLK_C */
    IOCON_configurePinDefault(PIN_P1_7,  PIN_FUNCTION_2, PIN_PULL_REPEATER);    /* FC7_MOSI     MOSI_C */
    IOCON_configurePinDefault(PIN_P1_8,  PIN_FUNCTION_2, PIN_PULL_REPEATER);    /* FC7_MISO     MISO_C */
    IOCON_configurePinDefault(PIN_P1_9,  PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_1_9     LNA_GAIN */
    IOCON_configurePinDefault(PIN_P1_11, PIN_FUNCTION_7, PIN_PULL_NONE);        /* USB0_VBUS    VBUS */
    IOCON_configurePinDefault(PIN_P1_12, PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_1_12    BLE_RESET */
    IOCON_configurePinDefault(PIN_P1_15, PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* PDM0_CLK     SCLK_D_PDM_CLK */
#endif

    if (IOCON_checkErrors() != LPCLIB_SUCCESS) {
        while (1);  //DEBUGGING ONLY
    }

    /* Do a manual scatter loading of vital RAM functions
     * Pins must be initialized already, because we almost certainly need the higher address
     * lines (beyond 16 KiB) to access the load region.
     */
    extern uint32_t Load$$LPCLIB_RAMCODE$$Base;
    extern uint32_t Image$$LPCLIB_RAMCODE$$Base;
    extern uint32_t Image$$LPCLIB_RAMCODE$$Length;

    pFrom = &Load$$LPCLIB_RAMCODE$$Base;
    pTo = &Image$$LPCLIB_RAMCODE$$Base;
    length = (int)&Image$$LPCLIB_RAMCODE$$Length;
    while (length > 0) {    /* The loop copies in multiples of 4! */
        *pTo = *pFrom;
        ++pFrom;
        ++pTo;
        length -= 4;
    }

    SystemCoreClock = 0;
}


/********** Standard Library I/O **********/

#include <sys/stat.h>

extern uint32_t __heap_start;

int _fstat (int fd, struct stat *st);
int _fstat (int fd, struct stat *st)
{
    (void) fd;
    (void) st;

    return -1;
}


int _read (int fd, char *ptr, int len);
int _read (int fd, char *ptr, int len)
{
    (void) fd;

    int bytesUnRead;

    (void) ptr;
    bytesUnRead = 0;

    return len - bytesUnRead;
}


int _write (int fd, const char *ptr, int len);
int _write (int fd, const char *ptr, int len)
{
    (void) fd;

    while (len) {
        ITM_SendChar(*ptr);
        ++ptr;
        --len;
    }

    return 0;
}

int _close (int fd);
int _close (int fd)
{
    (void) fd;

    return 0;
}


int _lseek (int fd, int ptr, int dir);
int _lseek (int fd, int ptr, int dir)
{
    (void)fd;
    (void)ptr;
    (void)dir;

    return 0;
}


int _isatty (int fd);
int _isatty (int fd)
{
    return (fd <= 2) ? 1 : 0;   /* one of stdin, stdout, stderr */
}


caddr_t _sbrk (int incr);
caddr_t _sbrk (int incr)
{
    static caddr_t heap = NULL;
    caddr_t prev_heap;

    if (heap == NULL) {
        heap = (caddr_t)&__heap_start;
    }

    prev_heap = heap;
    heap += incr;

    return prev_heap;
}


