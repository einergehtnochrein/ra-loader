/* Copyright (c) 2017, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#include "lpclib.h"

#include "usbuser_config.h"
#include "usbuser_descriptors.c"


struct _USBContext {
    USBD_HANDLE_T hUsb;
    const USBD_API_T *pUsbApi;
    USBD_HANDLE_T hDfu;
    USBD_API_INIT_PARAM_T param;
    USB_CORE_DESCS_T desc;

    bool configured;
} usbContext;

__SECTION(".usbworkspace")
static uint8_t _usbWorkspace[0x2000] __ALIGN(1024*1024*4);


static ErrorCode_t _USBUSER_handleConfigureEvent (USBD_HANDLE_T hUsb)
{
    USB_CORE_CTRL_T *core = (USB_CORE_CTRL_T *)hUsb;

    usbContext.configured = core->config_value != 0;

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
static void USBUSER_serialSetLineCode(
    int dataBits,
    int stopBits,
    int parity,
    int baudrate
)
{
    _uartConfig[0].asyncFormat.databits = (dataBits == 8) ? UART_DATABITS_8 : UART_DATABITS_7;
    _uartConfig[0].asyncFormat.stopbits = (stopBits == 2) ? UART_STOPBITS_2 : UART_STOPBITS_1;
    _uartConfig[0].asyncFormat.parity = UART_PARITY_NONE;
    if (parity == 1) {
        _uartConfig[0].asyncFormat.parity = UART_PARITY_ODD;
    }
    if (parity == 2) {
        _uartConfig[0].asyncFormat.parity = UART_PARITY_EVEN;
    }
    _uartConfig[1].baudrate = baudrate;

extern UART_Handle blePort;
UART_ioctl(blePort, _uartConfig);
}


void USBUSER_open (_Bool uartBridge)
{
    struct _USBContext *handle = &usbContext;
    ErrorCode_t ret = LPC_OK;


    /* Init pointer to ROM API entry */
    handle->pUsbApi = (const USBD_API_T *) pRom->pUsbd;

    handle->param.usb_reg_base = LPC_USB_BASE;
    handle->param.mem_base = (uint32_t)&_usbWorkspace;
    handle->param.mem_size = sizeof(_usbWorkspace);
    handle->param.max_num_ep = USBCONFIG_MAX_NUM_EP;
    handle->param.USB_Configure_Event = _USBUSER_handleConfigureEvent;

    /* Set the USB descriptors */
    handle->desc.device_desc = (uint8_t *) &appDeviceDescriptor;
    handle->desc.string_desc = uartBridge ?
            (uint8_t *) &bl652Bridge_StringDescriptor
          : (uint8_t *) &noBridge_StringDescriptor;

    /* Note, to pass USBCV test full-speed only devices should have both
     * descriptor arrays point to same location and device_qualifier set
     * to 0.
     */
    handle->desc.high_speed_desc = (uint8_t *)&appConfiguration1;
    handle->desc.full_speed_desc = (uint8_t *)&appConfiguration1;
    handle->desc.device_qualifier = 0;

    /* Start ROM stack */
    ret = handle->pUsbApi->hw->Init(&handle->hUsb, &handle->desc, &handle->param);
    if (ret == LPC_OK) {
        ret = USBSerial_init(
                    handle->hUsb,
                    &appConfiguration1.CIF.interface,
                    &appConfiguration1.DIF.interface,
                    &handle->param.mem_base,
                    &handle->param.mem_size);

        if (ret == LPC_OK) {
            if (uartBridge) {
                USBSERIAL_installSetLineCodeHandler(USBUSER_serialSetLineCode);
            }

            /* now connect */
            handle->pUsbApi->hw->Connect(handle->hUsb, 1);
        }
    }
}


bool USBUSER_isConfigured (void)
{
    return usbContext.configured;
}


void USBUSER_worker (void)
{
}


void USB_IRQHandler (void)
{
    /* Call the ROM driver, which may call application callbacks as required. */
    usbContext.pUsbApi->hw->ISR(usbContext.hUsb);
}
