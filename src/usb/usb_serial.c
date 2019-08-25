/* Copyright (c) 2018-2019, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 * Neither the name of the author nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <string.h>

#include "lpclib.h"

#include "usbuser_config.h"
#include "app.h"


#define TX_BUFFER_SIZE1                     1024
#define RX_BUFFER_SIZE1                     1024

struct _USBSerial_Context {
    USBD_HANDLE_T hUsb;
    USBD_HANDLE_T hCdc;

    uint8_t cif_if;
    uint8_t int_ep;
    uint8_t in_ep;
    uint8_t out_ep;

    uint8_t *pTxBuffer;
    int txBufferSize;
    int txWriteIndex;
    volatile int txReadIndex;
    int txPendingIndex;
    bool txBusy;

    uint8_t *pRxBuffer;
    int rxBufferSize;
    int rxWriteIndex;
    volatile int rxReadIndex;
    volatile bool haveRxData;

    uint16_t controlLineState;
    bool sendNotification;
    struct {
        CDC_NOTIFICATION_HEADER header;
        uint16_t payload;
    } notification;
} usbSerialContext;


/* Set line coding call back routine */
static ErrorCode_t USBSerial_SetControlLineState (USBD_HANDLE_T hCDC, uint16_t state)
{
    struct _USBSerial_Context *handle = &usbSerialContext;
//    int count;


    if (hCDC != handle->hCdc) {
        return ERR_USBD_UNHANDLED;
    }

    //TODO  control AUTORUN pin based on DTR setting
    GPIO_writeBit(GPIO_BLE_AUTORUN, (state & 0x01) ? 0 : 1);

    /* Any change? */
    if (state ^ handle->controlLineState) {
        handle->controlLineState = state;

        handle->notification.payload = 0
                    | ((handle->controlLineState & 2) ? CDC_SERIAL_STATE_TX_CARRIER : 0)
                    | CDC_SERIAL_STATE_RX_CARRIER
                    ;
        handle->notification.header.bmRequestType.BM.Recipient = REQUEST_TO_INTERFACE;
        handle->notification.header.bmRequestType.BM.Type = REQUEST_CLASS;
        handle->notification.header.bmRequestType.BM.Dir = REQUEST_DEVICE_TO_HOST;
        handle->notification.header.bRequest = CDC_NOTIFICATION_SERIAL_STATE;
        handle->notification.header.wIndex.W = handle->cif_if;
        handle->notification.header.wLength = sizeof(handle->notification.payload);
        /*count =*/ pRom->pUsbd->hw->WriteEP(handle->hUsb, handle->int_ep, (uint8_t *)&handle->notification, sizeof(handle->notification));
    }

    return LPC_OK;
}



static ErrorCode_t USBSerial_SendBreak (USBD_HANDLE_T hCDC, uint16_t mstime)
{
    (void)hCDC;

    int durationMilliseconds = -1;
    if (mstime != 0xFFFF) {
        durationMilliseconds = mstime;
    }

    SYS_sendBreak(durationMilliseconds);

    return LPC_OK;
}



static UART_Config _uartConfig[] = {
    {.opcode = UART_OPCODE_SET_ASYNC_FORMAT,
        {.asyncFormat = {
            .databits = UART_DATABITS_8,
            .stopbits = UART_STOPBITS_1,
            .parity = UART_PARITY_NONE,}}},

    {.opcode = UART_OPCODE_SET_BAUDRATE,
        {.baudrate = 115200,}},

    UART_CONFIG_END
};



/* Set line coding call back routine */
static ErrorCode_t USBSerial_SetLineCode(USBD_HANDLE_T hCDC, CDC_LINE_CODING *line_coding)
{
    (void)hCDC;

    switch (line_coding->bDataBits) {
    case 7:
        _uartConfig[0].asyncFormat.databits = UART_DATABITS_7;
        break;
    case 8:
    default:
        _uartConfig[0].asyncFormat.databits = UART_DATABITS_8;
        break;
    }

    switch (line_coding->bCharFormat) {
    case 1: /* 1.5 Stop Bits not supported */
        return ERR_USBD_INVALID_REQ;

    case 2: /* 2 Stop Bits */
        _uartConfig[0].asyncFormat.stopbits = UART_STOPBITS_2;
        break;

    default:
    case 0: /* 1 Stop Bit */
        _uartConfig[0].asyncFormat.stopbits = UART_STOPBITS_1;
        break;
    }

    switch (line_coding->bParityType) {
    case 1:
        _uartConfig[0].asyncFormat.parity = UART_PARITY_ODD;
        break;

    case 2:
        _uartConfig[0].asyncFormat.parity = UART_PARITY_EVEN;
        break;

    case 3:
        break;

    case 4:
        break;

    default:
    case 0:
        _uartConfig[0].asyncFormat.parity = UART_PARITY_NONE;
        break;
    }

    if (line_coding->dwDTERate < 3125000) {
        _uartConfig[1].baudrate = line_coding->dwDTERate;
    }

//TODO don't do this in interrupt...
extern UART_Handle blePort;
UART_ioctl(blePort, _uartConfig);

    return LPC_OK;
}



/* UCOM bulk EP_IN and EP_OUT endpoints handler */
static ErrorCode_t USBSerial_endpointHandler (USBD_HANDLE_T hUsb, void *data, uint32_t event)
{
    struct _USBSerial_Context *handle = (struct _USBSerial_Context *) data;
    int count = 0;
    int wi, ri;

    switch (event) {
    case USB_EVT_IN:
        handle->txReadIndex = handle->txPendingIndex;

        /* More data to send? */
        wi = handle->txWriteIndex;
        ri = handle->txReadIndex;
        count = (wi >= ri) ? wi - ri : (int)handle->txBufferSize - ri;
        if (count > 64) {
            count = 64;
        }
        handle->txBusy = (count > 0);

        if (count) {
            count = pRom->pUsbd->hw->WriteEP(hUsb, handle->in_ep, &handle->pTxBuffer[handle->txReadIndex], count);
            handle->txPendingIndex = (handle->txReadIndex + count) % handle->txBufferSize;
        }
        break;

    case USB_EVT_OUT:
        handle->haveRxData = true;
        break;

    default:
        break;
    }

    return LPC_OK;
}



ErrorCode_t USBSerial_init(USBD_HANDLE_T hUsb,
                           const USB_INTERFACE_DESCRIPTOR *pCifIntfDesc,
                           const USB_INTERFACE_DESCRIPTOR *pDifIntfDesc,
                           uint32_t *mem_base,
                           uint32_t *mem_size)
{
    struct _USBSerial_Context *handle = &usbSerialContext;
    USBD_CDC_INIT_PARAM_T cdcParam;
    ErrorCode_t ret = LPC_OK;

    /* Do a quick check of if the interface descriptor passed is the right one. */
    if ((pCifIntfDesc == 0) || (pCifIntfDesc->bInterfaceClass != USB_DEVICE_CLASS_COMMUNICATIONS)) {
        return ERR_FAILED;
    }

    handle->hUsb = hUsb;

    /* Init CDC params */
    memset((void *) &cdcParam, 0, sizeof(cdcParam));
    cdcParam.mem_base = *mem_base;
    cdcParam.mem_size = *mem_size;
    cdcParam.cif_intf_desc = (uint8_t *)pCifIntfDesc;
    cdcParam.dif_intf_desc = (uint8_t *)pDifIntfDesc;
    cdcParam.SetCtrlLineState = USBSerial_SetControlLineState;
    cdcParam.SendBreak = USBSerial_SendBreak;
    cdcParam.SetLineCode = USBSerial_SetLineCode;
    ret = pRom->pUsbd->cdc->init(handle->hUsb, &cdcParam, &handle->hCdc);

    if (ret == LPC_OK) {
        /* Allocate buffers from USB memory */
        //TODO: check...
        handle->pTxBuffer = (uint8_t *)cdcParam.mem_base;
        handle->txBufferSize = TX_BUFFER_SIZE1;
        cdcParam.mem_base += TX_BUFFER_SIZE1;
        cdcParam.mem_size -= TX_BUFFER_SIZE1;
        handle->pRxBuffer = (uint8_t *)cdcParam.mem_base;
        handle->rxBufferSize = RX_BUFFER_SIZE1;
        cdcParam.mem_base += RX_BUFFER_SIZE1;
        cdcParam.mem_size -= RX_BUFFER_SIZE1;

        handle->cif_if = pCifIntfDesc->bInterfaceNumber;
        handle->int_ep = USBCONFIG_CIF_EP_INT;
        handle->in_ep = USBCONFIG_DIF_EP_IN;
        handle->out_ep = USBCONFIG_DIF_EP_OUT;

        /* Register IN endpoint handler */
        ret = pRom->pUsbd->core->RegisterEpHandler(handle->hUsb, 5, USBSerial_endpointHandler, handle); //TODO

        if (ret == LPC_OK) {
            /* Register OUT endpoint handler */
            ret = pRom->pUsbd->core->RegisterEpHandler(handle->hUsb, 4, USBSerial_endpointHandler, handle); //TODO
        }
    }

    *mem_base = cdcParam.mem_base;
    *mem_size = cdcParam.mem_size;

    return ret;
}



int USBSerial_read (void *message, int maxLen)
{
    struct _USBSerial_Context *handle = &usbSerialContext;
    int wi;
    int ri;
    int i;
    int bytesAvailable;


    if (!USBUSER_isConfigured()) {
        return 0;
    }

    /* Find out how many bytes are available in the RX buffer */
    wi = handle->rxWriteIndex;
    ri = handle->rxReadIndex;
    bytesAvailable = (wi >= ri) ? wi - ri : (int)handle->rxBufferSize - (ri - wi);

    /* Limit to max amount possible */
    if (bytesAvailable > maxLen) {
        bytesAvailable = maxLen;
    }

    for (i = 0; i < bytesAvailable; i++) {
        ((uint8_t *)message)[i] = handle->pRxBuffer[handle->rxReadIndex];
        ++handle->rxReadIndex;
        if (handle->rxReadIndex >= handle->rxBufferSize) {
            handle->rxReadIndex = 0;
        }
    }

    return bytesAvailable;
}



void USBSerial_write (const void *message, int len)
{
    struct _USBSerial_Context *handle = &usbSerialContext;
    int i;
    int wi;
    int ri;
    int count;
    int bytesFree;

    if (!USBUSER_isConfigured()) {
        return;
    }

    /* Find out how many bytes are free in the TX buffer */
    wi = handle->txWriteIndex;
    ri = handle->txReadIndex;
    bytesFree = (ri > wi) ? ri - wi - 1 : (int)handle->txBufferSize - (wi - ri);

    /* Drop data if not enough space left */
    if (bytesFree < len) {
        return;
    }

    /* Copy into TX buffer */
    for (i = 0; i < len; i++) {
        handle->pTxBuffer[wi] = ((const uint8_t *)message)[i];
        wi = (wi + 1) % handle->txBufferSize;
    }
    handle->txWriteIndex = wi;

    /* If TX machine is no longer running, kick-start it now */
    if (!handle->txBusy) {
        /* Calculate amount of data to send (max: 64 bytes) */
        wi = handle->txWriteIndex;
        ri = handle->txReadIndex;
        count = (wi >= ri) ? wi - ri : (int)handle->txBufferSize - ri;
        if (count > 64) {
            count = 64;
        }
        handle->txBusy = (count > 0);

        if (count) {
            count = pRom->pUsbd->hw->WriteEP(handle->hUsb, handle->in_ep, &handle->pTxBuffer[handle->txReadIndex], count);
            handle->txPendingIndex = (handle->txReadIndex + count) % handle->txBufferSize;
        }
    }
}


void USBSERIAL_worker (void)
{
    struct _USBSerial_Context *handle = &usbSerialContext;

    if (handle->haveRxData) {
        /* Enough room in RX buffer for next fragment? */
        int roomLeft = 0;
        if (handle->rxWriteIndex < handle->rxReadIndex) {
            roomLeft = handle->rxReadIndex - handle->rxWriteIndex - 1;
        }
        else {
            roomLeft = handle->rxBufferSize - 1 - (handle->rxWriteIndex - handle->rxReadIndex);
        }

        if (roomLeft >= 64) {
            handle->haveRxData = false;
            uint8_t fragment[64];
            int i;
            int nRead = pRom->pUsbd->hw->ReadEP(handle->hUsb, handle->out_ep, fragment);
            if (nRead > 0) {
                for (i = 0; i < nRead; i++) {
                    handle->pRxBuffer[handle->rxWriteIndex] = fragment[i];
                    ++handle->rxWriteIndex;
                    if (handle->rxWriteIndex >= handle->rxBufferSize) {
                        handle->rxWriteIndex = 0;
                    }
                }
            }
        }
    }
}

