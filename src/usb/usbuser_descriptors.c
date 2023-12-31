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


#include <stdlib.h>

#include "usbuser_config.h"
#include "usbd_strings.h"


    /* --- String Descriptors --- */

/** Enumerate all string descriptors.
 *
 *  Use these names in the string descriptor table and the device/configuration descriptors
 *  to achieve an automatic numbering of all string descriptors.
 *
 *  The first entry must be assigned the index 1.
 *  The last entry must be USB_NUMBER_OF_STRING_DESCRIPTORS.
 */
enum {
    USBSTR_IDVENDOR = 1,
    USBSTR_IDPRODUCT,

    USBSTR_IAD,
    USBSTR_CIF,
    USBSTR_DIF,
};


/* !!! Be careful not to add a comma after the last entry !!! */
static const DECLARE_USBD_STRINGS (
    bl652Bridge_StringDescriptor,
    0x0409,
    L"github.com/einergehtnochrein/ra-hardware",
    L"Ra2 BL652 Bridge",

    L"IAD",
    L"CIF",
    L"DIF"
);
static const DECLARE_USBD_STRINGS (
    noBridge_StringDescriptor,
    0x0409,
    L"github.com/einergehtnochrein/ra-hardware",
    L"Ra2 Configurator",

    L"IAD",
    L"CIF",
    L"DIF"
);


/** Device descriptor. */
ALIGNED(4) const USB_DEVICE_DESCRIPTOR appDeviceDescriptor = {
    .bLength                    = USB_DEVICE_DESC_SIZE,
    .bDescriptorType            = USB_DEVICE_DESCRIPTOR_TYPE,
    .bcdUSB                     = 0x0200,
    .bDeviceClass               = 0,
    .bDeviceSubClass            = 0,
    .bDeviceProtocol            = 0,
    .bMaxPacketSize0            = 64,
    .idVendor                   = 0x16C0,
    .idProduct                  = 0x05DC,
    .bcdDevice                  = 0x0103,
    .iManufacturer              = USBSTR_IDVENDOR,
    .iProduct                   = USBSTR_IDPRODUCT,
    .iSerialNumber              = 0,
    .bNumConfigurations         = USBCONFIG_NUM_CONFIGURATIONS,
};


/** Configuration 1 */
ALIGNED(4) const __PACKED(struct {
    USB_CONFIGURATION_DESCRIPTOR                    config;

    USB_INTERFACE_ASSOCIATION_DESCRIPTOR            IAD;
    __PACKED(struct {
        USB_INTERFACE_DESCRIPTOR                    interface;
        __PACKED(struct _kissControlInterfaceSpec {
            CDC_HEADER_DESCRIPTOR                       functionalHeader;
            CDC_ABSTRACT_CONTROL_MANAGEMENT_DESCRIPTOR  acm;
            CDC_UNION_DESCRIPTOR                        unionInterface;
            CDC_CALL_MANAGEMENT_DESCRIPTOR              callManagement;
            USB_ENDPOINT_DESCRIPTOR                     endpoint_int;
        }) specification;
    }) CIF;
    __PACKED(struct {
        USB_INTERFACE_DESCRIPTOR                    interface;
        USB_ENDPOINT_DESCRIPTOR                         endpoint_out;
        USB_ENDPOINT_DESCRIPTOR                         endpoint_in;
    }) DIF;

    uint8_t                                         terminator;

}) appConfiguration1 = {

    .config = {
        .bLength                = USB_CONFIGURATION_DESC_SIZE,
        .bDescriptorType        = USB_CONFIGURATION_DESCRIPTOR_TYPE,
        .wTotalLength           = sizeof(appConfiguration1) - 1,
        .bNumInterfaces         = USBCONFIG_NUM_INTERFACES,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0x80,     /* bus-powered */
        .bMaxPower              = 80/2,
    },

    .IAD = {
        .bLength                = USB_INTERFACE_ASSOCIATION_DESC_SIZE,
        .bDescriptorType        = USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,
        .bFirstInterface        = USBCONFIG_INTERFACE_CIF,
        .bInterfaceCount        = 2,
        .bFunctionClass         = CDC_COMMUNICATION_INTERFACE_CLASS,
        .bFunctionSubClass      = CDC_ABSTRACT_CONTROL_MODEL,
        .bFunctionProtocol      = CDC_PROTOCOL_COMMON_AT_COMMANDS,
        .iFunction              = 0,
    },

    .CIF = {
        .interface = {
            .bLength                = USB_INTERFACE_DESC_SIZE,
            .bDescriptorType        = USB_INTERFACE_DESCRIPTOR_TYPE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_CIF,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = CDC_COMMUNICATION_INTERFACE_CLASS,
            .bInterfaceSubClass     = CDC_ABSTRACT_CONTROL_MODEL,
            .bInterfaceProtocol     = CDC_PROTOCOL_COMMON_AT_COMMANDS,
            .iInterface             = USBSTR_CIF,
        },

        .specification = {
            .functionalHeader = {
                .bLength                = CDC_HEADER_DESC_SIZE,
                .bDescriptorType        = CDC_CS_INTERFACE,
                .bDescriptorSubtype     = CDC_HEADER,
                .bcdCDC                 = 0x0110,   /* 1.10 */
            },

            .acm = {
                .bLength                = CDC_ABSTRACT_CONTROL_MANAGEMENT_DESC_SIZE,
                .bDescriptorType        = CDC_CS_INTERFACE,
                .bDescriptorSubtype     = CDC_ABSTRACT_CONTROL_MANAGEMENT,
                .bmCapabilities         = 0
                                        | (1u << 1)     /* Set_Line_Coding, Get_Line_Coding
                                                         * Set_Control_Line_State
                                                         * Serial_State notification
                                                         */
                                        | (1u << 2)     /* Send_Break */
                                        ,
            },

            .unionInterface = {
                .bLength                = CDC_UNION_DESC_SIZE,
                .bDescriptorType        = CDC_CS_INTERFACE,
                .bDescriptorSubtype     = CDC_UNION,
                .bMasterInterface       = USBCONFIG_INTERFACE_CIF,
                .bSlaveInterface0       = USBCONFIG_INTERFACE_DIF,
            },

            .callManagement = {
                .bLength                = CDC_CALL_MANAGEMENT_DESC_SIZE,
                .bDescriptorType        = CDC_CS_INTERFACE,
                .bDescriptorSubtype     = CDC_CALL_MANAGEMENT,
                .bmCapabilities         = 0x01,
                .bDataInterface         = 0,
            },

            .endpoint_int = {
                .bLength                = USB_ENDPOINT_DESC_SIZE,
                .bDescriptorType        = USB_ENDPOINT_DESCRIPTOR_TYPE,
                .bEndpointAddress       = USBCONFIG_CIF_EP_INT,
                .bmAttributes           = USB_ENDPOINT_TYPE_INTERRUPT,
                .wMaxPacketSize         = USBCONFIG_CIF_EP_INT_SIZE,
                .bInterval              = 10,
            },
        },
    },

    .DIF = {
        /* DIF */
        .interface = {
            .bLength                = USB_INTERFACE_DESC_SIZE,
            .bDescriptorType        = USB_INTERFACE_DESCRIPTOR_TYPE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_DIF,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 2,
            .bInterfaceClass        = CDC_DATA_INTERFACE_CLASS,
            .bInterfaceSubClass     = 0,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_DIF,
        },

        /* DIF OUT endpoint */
        .endpoint_out = {
            .bLength                = USB_ENDPOINT_DESC_SIZE,
            .bDescriptorType        = USB_ENDPOINT_DESCRIPTOR_TYPE,
            .bEndpointAddress       = USBCONFIG_DIF_EP_OUT,
            .bmAttributes           = USB_ENDPOINT_TYPE_BULK,
            .wMaxPacketSize         = USBCONFIG_DIF_EP_OUT_SIZE,
            .bInterval              = 0,
        },

        /* DIF IN endpoint */
        .endpoint_in = {
            .bLength                = USB_ENDPOINT_DESC_SIZE,
            .bDescriptorType        = USB_ENDPOINT_DESCRIPTOR_TYPE,
            .bEndpointAddress       = USBCONFIG_DIF_EP_IN,
            .bmAttributes           = USB_ENDPOINT_TYPE_BULK,
            .wMaxPacketSize         = USBCONFIG_DIF_EP_IN_SIZE,
            .bInterval              = 0,
        },
    },

    .terminator = 0,
};

