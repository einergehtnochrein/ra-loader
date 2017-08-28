
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"
#include "app.h"
#include "loader.h"
#include "validimage.h"
#include "config.h"

LOADER_Handle loaderTask;

//char s[100];

UART_Handle blePort;

#if (BOARD_RA == 1)
#define BLE_UART                            UART0
#endif
#if (BOARD_RA == 2)
#define BLE_UART                            UART3
#endif
#define BLE_PORT_TXBUF_SIZE                 512
#define BLE_PORT_RXBUF_SIZE                 2048
static uint8_t blePortTxBuf[BLE_PORT_TXBUF_SIZE];
static uint8_t blePortRxBuf[BLE_PORT_RXBUF_SIZE];



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

    /* A GPIO (PIO1_15, TP8) can be used as override (force loader entry) */
    executeFirmware = executeFirmware && (GPIO_readBit(GPIO_FORCE_LOADER) == 1);

    if (executeFirmware) {
        /* Start firmware */
        uint32_t image_SP = ((volatile uint32_t *)FIRMWARE_START_ADDRESS)[0];
        uint32_t image_PC = ((volatile uint32_t *)FIRMWARE_START_ADDRESS)[1];

        __set_MSP(image_SP);
        ((void(*)(void))image_PC)();
    }

    /********** Loader mode **********/

    /* Increase internal voltages so we can run up to 96 MHz */
//    pRom->pPwrd->set_voltage(0, 96000000);
//    CLKPWR_setCpuClock(48000000);


    CLKPWR_setCpuClock(12000000);

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

#if (BOARD_RA == 2)
    GPIO_writeBit(GPIO_BLE_MODESEL, 1);
#endif
    GPIO_writeBit(GPIO_BLE_AUTORUN, 1);     /* Select BL652 "VSP Bridge to UART" mode */
    GPIO_writeBit(GPIO_ADF7021_CE, 0);
    GPIO_writeBit(GPIO_LNA_GAIN, 0);
    GPIO_writeBit(GPIO_ENABLE_VDDA, 0);     /* RF part off */
#if (BOARD_RA == 1)
    GPIO_writeBit(GPIO_SSEL_DISP, 0);
#endif
    GPIO_writeBit(GPIO_BLE_RESET, 1);       /* Release BL652 reset */

    UART_open(BLE_UART, &blePort);
    UART_ioctl(blePort, blePortConfig);

#if (BOARD_RA == 1)
    NVIC_EnableIRQ(UART0_IRQn);
#endif
#if (BOARD_RA == 2)
    NVIC_EnableIRQ(UART3_IRQn);
#endif

    LOADER_open(&loaderTask);

    while (1) {
        handleBleCommunication();
        LOADER_thread(loaderTask);
        __WFI();
    }
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

