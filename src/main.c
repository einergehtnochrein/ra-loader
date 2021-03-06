
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"
#include "app.h"
#include "loader.h"
#include "validimage.h"
#include "config.h"
#if (BOARD_RA == 2)
#  include "usbuser_config.h"
#endif

LOADER_Handle loaderTask;

//char s[100];

UART_Handle blePort;

#if (BOARD_RA == 1)
#define BLE_UART                            UART0
#endif
#if (BOARD_RA == 2)
#define BLE_UART                            UART3
#endif
#define BLE_PORT_TXBUF_SIZE                 2048
#define BLE_PORT_RXBUF_SIZE                 2048
static uint8_t blePortTxBuf[BLE_PORT_TXBUF_SIZE];
static uint8_t blePortRxBuf[BLE_PORT_RXBUF_SIZE];
static volatile int blePortBreakTime;
static volatile bool blePortSetBreak;



static const UART_Config blePortConfig[] = {
    {.opcode = UART_OPCODE_SET_TX_BUFFER,
        {.buffer = {
            .pBuffer = blePortTxBuf,
            .size = BLE_PORT_TXBUF_SIZE, }}},

    {.opcode = UART_OPCODE_SET_RX_BUFFER,
        {.buffer = {
            .pBuffer = blePortRxBuf,
            .size = BLE_PORT_RXBUF_SIZE, }}},

    {.opcode = UART_OPCODE_SET_ASYNC_FORMAT,
        {.asyncFormat = {
            .databits = UART_DATABITS_8,
            .stopbits = UART_STOPBITS_1,
            .parity = UART_PARITY_NONE,}}},

    {.opcode = UART_OPCODE_SET_BAUDRATE,
        {.baudrate = 115200,}},

    {.opcode = UART_OPCODE_SET_HARDWARE_HANDSHAKE,
        {.hardwareHandshake = ENABLE,}},

    UART_CONFIG_END
};

#if (BOARD_RA == 2)
static const UART_Config blePortConfigBreakEnable[] = {
    {.opcode = UART_OPCODE_SET_BREAK,
        {.enableBreak = LPCLIB_YES, }},

    UART_CONFIG_END
};

static const UART_Config blePortConfigBreakDisable[] = {
    {.opcode = UART_OPCODE_SET_BREAK,
        {.enableBreak = LPCLIB_NO, }},

    UART_CONFIG_END
};
#endif


#define COMMAND_LINE_SIZE   1024
char commandLine[COMMAND_LINE_SIZE];

static void handleBleCommunication (void) {
    if (UART_readLine(blePort, commandLine, sizeof(commandLine)) > 0) {
        /* Valid command line starts with hash ('#') and a channel number.
         * Optional arguments are separated by commas.
         * A command line ends with a comma and a checksum
         */
        int channel;
        if (sscanf(commandLine, "#%d", &channel) == 1) {
            /* Find the last token (checksum), and verify it */
            char *work = strdup(commandLine);
            if (work) {
                char *checkstr = work;
                int checksum = 0;
                char *token;
                do {
                    token = strsep(&checkstr, ",");
                    if (checkstr && token) {
                        checksum += ',';
                        while (*token) {
                            checksum += *token++;
                        }
                    }
                } while (checkstr);
                /* "token" now points to the checksum */
                bool checksumOk = false;
                if (token) {
                    int receivedChecksum = -1; 
                    if (sscanf(token, "%d", &receivedChecksum) == 1) {
                        if ((checksum % 100) == receivedChecksum) {
                            checksumOk = true;
                        }
                    }
                }
                free(work);

                if (checksumOk) {
                    switch (channel) {
                    case HOST_CHANNEL_PING:
                        {
                            /* Send status: loader mode */
                            char s[20];
                            sprintf(s, "0,%d,%d",
                                    LOADER_VERSION,
#if (LOADER_VERSION >= 2) && (BOARD_RA == 1)
                                    1
#endif
#if (LOADER_VERSION >= 2) && (BOARD_RA == 2)
                                    2
#endif
                                   );
                            SYS_send2Host(HOST_CHANNEL_PING, s);
                        }
                        break;

                    case HOST_CHANNEL_FIRMWAREUPDATE:
                        {
                            LOADER_processCommand(loaderTask, &commandLine[1]);
                        }
                        break;
                    }
                }
            }
        }


        commandLine[0] = 0;
    }
}


/* Send a message to the host.
 * Formatter adds channel ID and line feed to the message string.
 */
LPCLIB_Result SYS_send2Host (int channel, const char *message)
{
    channel %= 1000;

    /* Get enough space for TX string
     * 1: '#'
     * 4: channel number (3 digits max) + comma
     * 3: comma and checksum (2 digits)
     * 1: \r
     * 1: termination
     */
    char *s = (char *)malloc(1 + 4 + strlen(message) + 3 + 1 + 1);
    if (s == NULL) {
        return LPCLIB_ERROR;
    }

    sprintf(s, "#%d,%s,", channel, message);
    int checksum = 0;
    for (int i = 0; i < (int)strlen(s); i++) {
        checksum += s[i];
    }
    checksum %= 100;
    sprintf(s, "%s%d\r", s, checksum);

    UART_write(blePort, s, strlen(s));

    free(s);

    return LPCLIB_SUCCESS;
}


LPCLIB_Result SYS_sendBreak (int durationMilliseconds)
{
    blePortBreakTime = durationMilliseconds;
    blePortSetBreak = true;

    return LPCLIB_SUCCESS;
}



uint8_t ble2usb[2048];
uint8_t usb2ble[2048];


int main (void)
{
    GPIO_open();

    GPIO_setDirBit(GPIO_FORCE_LOADER, DISABLE);

    /* Shall we start the firmware image, or do we have to enter loader mode?
     * Loader mode is started if no valid firmware image is poresent or if
     * a hardware override exists.
     */
    bool executeFirmware = true;

    /* Check if image is valid */
    executeFirmware = executeFirmware && check4ValidImage(FIRMWARE_START_ADDRESS, FIRMWARE_END_ADDRESS);

    /* GPIO PIO1_15 (TP8) on Ra1 can be used as override (force loader entry when pulled low).
     * On Ra2 this is PIO0_22 (TP3).
     * Ra2 has an additional override (PIO0_4, TP2) to allow for up to three override scenarios:
     * TP3=0, TP2=1: Force loader (firmware update via BLE).
     * TP3=1, TP2=0: Bridge between BL652 and USB
     * TP3=0, TP2=0: reserved
     */
    int override = 0;
    if (GPIO_readBit(GPIO_FORCE_LOADER) == 0) {
        override += 1;
    }
#if (BOARD_RA == 2)
    if (GPIO_readBit(GPIO_FORCE_LOADER2) == 0) {
        override += 2;
    }
#endif

    executeFirmware = executeFirmware && (override == 0);

    if (executeFirmware) {
        /* Start firmware */
        uint32_t image_SP = ((volatile uint32_t *)FIRMWARE_START_ADDRESS)[0];
        uint32_t image_PC = ((volatile uint32_t *)FIRMWARE_START_ADDRESS)[1];

        __set_MSP(image_SP);
        ((void(*)(void))image_PC)();
    }

    /* If we are here because of missing firmware, assume override=1 */
    if (override == 0) {
        override = 1;
    }

    /********** Loader mode **********/

    /* Increase internal voltages so we can run up to 96 MHz */
//    pRom->pPwrd->set_voltage(0, 96000000);
//    CLKPWR_setCpuClock(48000000);

#if (BOARD_RA == 1)
    CLKPWR_setCpuClock(12000000, CLKPWR_CLOCK_IRC);
#endif
#if (BOARD_RA == 2)
    CLKPWR_setCpuClock(12000000, CLKPWR_CLOCK_FRO12);
#endif

#if (BOARD_RA == 1)
    /* Prepare system FIFO */
    LPC_FIFO->FIFOCFGUSART0 = 0x00000404;
    LPC_FIFO->FIFOUPDATEUSART = 0x000F000F;
    LPC_FIFO->FIFOCFGSPI0 = 0x00000404;
    LPC_FIFO->FIFOCFGSPI1 = 0x00000404;
    LPC_FIFO->FIFOUPDATESPI = 0x00030003;
//    LPC_SYSCON->FIFOCTRL = 0x00003030;  // SPI0/SPI1 RX/TX

    LPC_ASYNCSYSCON->FRGCTRL = (0 << 8) | (255 << 0);   /* No FRAC, i.e. UART clock = 12 MHz */
#endif
#if (BOARD_RA == 2)
    LPC_SYSCON->FRGCTRL = (0 << 8) | (255 << 0);        /* No FRAC, i.e. UART clock = 12 MHz */

    LPC_SYSCON->FCLKSEL[3] = 0;                         /* FLEXCOMM3 clock = FRO12M */
#endif

    SystemCoreClock = CLKPWR_getBusClock(CLKPWR_CLOCK_CPU);

    osKernelInitialize();
    osKernelStart();

    GPIO_setDirBit(GPIO_BLE_RESET, ENABLE);
    GPIO_setDirBit(GPIO_BLE_AUTORUN, ENABLE);
#if (BOARD_RA == 2)
    GPIO_setDirBit(GPIO_BLE_MODESEL, ENABLE);
#endif
    GPIO_setDirBit(GPIO_ADF7021_CE, ENABLE);
    GPIO_setDirBit(GPIO_ADF7021_SLE, ENABLE);
    GPIO_setDirBit(GPIO_ENABLE_VDDA, ENABLE);
    GPIO_setDirBit(GPIO_LNA_GAIN, ENABLE);
#if (BOARD_RA == 1)
    GPIO_setDirBit(GPIO_POWER_SWITCH, DISABLE);
    GPIO_setDirBit(GPIO_BUTTON, DISABLE);
    GPIO_setDirBit(GPIO_SSEL_DISP, ENABLE);
    GPIO_setDirBit(GPIO_ENABLE_DISP, ENABLE);
#endif

    GPIO_writeBit(GPIO_ADF7021_CE, 0);
    GPIO_writeBit(GPIO_LNA_GAIN, 0);
    GPIO_writeBit(GPIO_ENABLE_VDDA, 0);     /* RF part off */
#if (BOARD_RA == 1)
    GPIO_writeBit(GPIO_SSEL_DISP, 0);
#endif

    /* Set BL652 operating mode depending on selected override. */
    switch (override) {
        case 1:
#if (BOARD_RA == 2)
            GPIO_writeBit(GPIO_BLE_MODESEL, 1); /* Select VSP mode */
#endif
            GPIO_writeBit(GPIO_BLE_AUTORUN, 1); /* VSP bridge mode (not command mode) */
            GPIO_writeBit(GPIO_BLE_RESET, 1);   /* Release BL652 reset */
            break;

        case 2:
#if (BOARD_RA == 2)
            GPIO_writeBit(GPIO_BLE_MODESEL, 0); /* Unselect VSP mode */
#endif
            GPIO_writeBit(GPIO_BLE_AUTORUN, 0); /* Command mode (not VSP bridge mode) */
            GPIO_writeBit(GPIO_BLE_RESET, 1);   /* Release BL652 reset */
            break;

        case 3:
            /* Reserved. Restart CPU */
            NVIC_SystemReset();
            while(1);
    }

    UART_open(BLE_UART, &blePort);
    UART_ioctl(blePort, blePortConfig);

#if (BOARD_RA == 1)
    NVIC_EnableIRQ(UART0_IRQn);
#endif
#if (BOARD_RA == 2)
    NVIC_EnableIRQ(UART3_IRQn);
#endif

    if (override == 1) {
        LOADER_open(&loaderTask);
        while (1) {
            handleBleCommunication();
            LOADER_thread(loaderTask);
            __WFI();
        }
    }

#if (BOARD_RA == 2)
    if (override == 2) {
        CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_USB);
        CLKPWR_unitPowerUp(CLKPWR_UNIT_USBPAD);

        LPC_SYSCON->USBCLKSEL = 0;                          /* USB clock = FROHF */
        LPC_SYSCON->USBCLKDIV = 0;                          /* USB clock divider = 1 (FROHF = 48 MHz) */
        LPC_SYSCON->FROCTRL |= (1u << 24) | (1u << 30);     /* USBCLKADJ=1, enable FROHF */

        NVIC_EnableIRQ(USB_IRQn);
        USBUSER_open();

        int nReadUart = 0;
        int nReadUsb = 0;
        int nWrittenUart = 0;

        while (1) {
            USBSERIAL_worker();

            nReadUart = UART_read(blePort, ble2usb, sizeof(ble2usb));
            if (nReadUart > 0) {
                USBSerial_write(ble2usb, nReadUart);
            }

            if (nReadUsb == 0) {
                nReadUsb = USBSerial_read(usb2ble, sizeof(usb2ble));
                nWrittenUart = 0;
            }
            if (nReadUsb > 0) {
                nWrittenUart += UART_write(blePort, &usb2ble[nWrittenUart], nReadUsb);
                nReadUsb -= nWrittenUart;
                if (nReadUsb < 0) {
                    nReadUsb = 0;
                }
            }

            /* Change TX break? */
            if (blePortSetBreak) {
                blePortSetBreak = false;

                if (blePortBreakTime != 0) {
                    UART_ioctl(blePort, blePortConfigBreakEnable);
                }
                else {
                    UART_ioctl(blePort, blePortConfigBreakDisable);
                }
            }
        }
    }
#endif
}


void SystemInit (void)
{
    BSP_systemInit();

#if (BOARD_RA == 1)
    /* Enable Asynchronous APB bus */
    LPC_SYSCON->ASYNCAPBCTRL = 1;           /* on */
    LPC_ASYNCSYSCON->ASYNCCLKDIV = 1;       /* Don't divide */
    LPC_ASYNCSYSCON->ASYNCAPBCLKSELA = 0;   /* IRC */
    LPC_ASYNCSYSCON->ASYNCAPBCLKSELB = 3;   /* Use CLKSELA */

    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_FIFO);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_FRG0);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM1);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM2);
#endif

#if (BOARD_RA == 2)
    /* Enable Asynchronous APB bus */
    LPC_SYSCON->ASYNCAPBCTRL = 1;           /* on */
    LPC_ASYNCSYSCON->ASYNCAPBCLKSELA = 0;   /* Main clock */

    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM1);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM2);
#endif
}

