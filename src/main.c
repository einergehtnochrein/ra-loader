
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"
#include "app.h"
#include "bl652.h"
#include "loader.h"
#include "validimage.h"
#include "config.h"
#include "usbuser_config.h"

LOADER_Handle loaderTask;

//char s[100];

UART_Handle blePort;

#define BLE_UART                            UART3
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
#if (LOADER_VERSION >= 2)
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
    char s[20];
    int checksum = 0;

    snprintf(s, sizeof(s), "#%d,", channel);
    for (int i = 0; i < (int)strlen(s); i++) {
        checksum += s[i];
    }
    UART_write(blePort, s, strlen(s));

    for (int i = 0; i < (int)strlen(message); i++) {
        checksum += message[i];
    }
    UART_write(blePort, message, strlen(message));

    checksum += ',';
    snprintf(s, sizeof(s), ",%d\r", checksum % 100);
    UART_write(blePort, s, strlen(s));

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
BL652_Handle ble;


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
    if (GPIO_readBit(GPIO_FORCE_LOADER2) == 0) {
        override += 2;
    }

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

    CLKPWR_setCpuClock(12000000, CLKPWR_CLOCK_FRO12);

    LPC_SYSCON->FRGCTRL = (0 << 8) | (255 << 0);        /* No FRAC, i.e. UART clock = 12 MHz */

    LPC_SYSCON->FCLKSEL[3] = 0;                         /* FLEXCOMM3 clock = FRO12M */

    SystemCoreClock = CLKPWR_getBusClock(CLKPWR_CLOCK_CPU);

    osKernelInitialize();
    osKernelStart();

    GPIO_setDirBit(GPIO_BLE_RESET, ENABLE);
    GPIO_setDirBit(GPIO_BLE_AUTORUN, ENABLE);
    GPIO_setDirBit(GPIO_BLE_MODESEL, ENABLE);
    GPIO_setDirBit(GPIO_ADF7021_CE, ENABLE);
    GPIO_setDirBit(GPIO_ADF7021_SLE, ENABLE);
    GPIO_setDirBit(GPIO_ENABLE_VDDA, ENABLE);
    GPIO_setDirBit(GPIO_LNA_GAIN, ENABLE);

    GPIO_writeBit(GPIO_ADF7021_CE, 0);
    GPIO_writeBit(GPIO_LNA_GAIN, 0);
    GPIO_writeBit(GPIO_ENABLE_VDDA, 0);     /* RF part off */

    UART_open(BLE_UART, &blePort);
    UART_ioctl(blePort, blePortConfig);

    NVIC_EnableIRQ(UART3_IRQn);

    if (BL652_open(blePort, GPIO_BLE_AUTORUN, GPIO_BLE_MODESEL, GPIO_BLE_RESET, &ble) == LPCLIB_SUCCESS) {
        if (BL652_findBaudrate(ble) == LPCLIB_SUCCESS) {
            if (BL652_readParameters(ble) == LPCLIB_SUCCESS) {
                if (BL652_updateParameters(ble)) {
                }
            }
        }

        BL652_setMode(ble, BL652_MODE_VSP_BRIDGE);
    }

    if (override == 1) {
        BL652_setMode(ble, BL652_MODE_VSP_BRIDGE);

        LOADER_open(&loaderTask);
        while (1) {
            handleBleCommunication();
            LOADER_thread(loaderTask);
            __WFI();
        }
    }

    if (override == 2) {
        BL652_setMode(ble, BL652_MODE_VSP_COMMAND);

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

    if (override == 3) {
        /* Reserved. Restart CPU */
        NVIC_SystemReset();
        while(1);
    }
}


void SystemInit (void)
{
    BSP_systemInit();

    /* Enable Asynchronous APB bus */
    LPC_SYSCON->ASYNCAPBCTRL = 1;           /* on */
    LPC_ASYNCSYSCON->ASYNCAPBCLKSELA = 0;   /* Main clock */

    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM1);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM2);
}

