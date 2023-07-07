
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


