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


#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"

#include "config.h"
#include "validimage.h"
#include "loader.h"



#define LOADER_QUEUE_LENGTH             10
#define LOADER_MAX_SIZE                 (LOADER_IMAGE_END - LOADER_IMAGE_START)

#define LOADER_TIMERMAGIC_RESTART       1

extern uint32_t __CONFIG_ADDRESS__;     /* Defined by linker script. Start of config section. */

typedef struct {
    uint8_t opcode;
} EPHEMUPDATE_Message;


/** Message opcodes for SYS task. */
enum {
    EPHEMUPDATE_OPCODE_xxx,
};



/** Identifiers for OS timers. */

enum {
    EPHEMUPDATE_TIMERMAGIC_ACTION,
};

typedef enum {
    LOADER_STATE_IDLE = 0,
    LOADER_STATE_TRANSFER,

    __LOADER_STATE_FLASH_ACTION__,

    LOADER_STATE_TRANSFER_WAIT_ERASE,
    LOADER_STATE_TRANSFER_WAIT_PROGRAM,
    LOADER_STATE_FINALIZE_WAIT_ERASE,
    LOADER_STATE_FINALIZE_WAIT_PROGRAM,
} LOADER_State;


#define LOADER_RESPONSE_OK          "0"
#define LOADER_RESPONSE_ERROR       "1"


struct LOADER_Context {
    struct pt pt;

    osTimerId actionTick;

    CRC_Handle crc;
    uint32_t segment;
    LOADER_State state;
    LOADER_Mode mode;

    uint16_t currentPage;
    bool pageDirty;
    uint8_t page[256] __ALIGN(4);

    uint32_t configOffset;
    Config_t config;

    bool terminate;
} loaderContext;




LPCLIB_Result LOADER_open (LOADER_Handle *pHandle)
{
    *pHandle = &loaderContext;

    loaderContext.state = LOADER_STATE_IDLE;
    loaderContext.mode = LOADER_MODE_FIRMWARE;

    return LPCLIB_SUCCESS;
}


LPCLIB_Result LOADER_setMode (LOADER_Handle handle, LOADER_Mode mode)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    switch (mode) {
        case LOADER_MODE_FIRMWARE:
        case LOADER_MODE_CONFIG:
            /* Reset loader state when mode changes */
            if (mode != handle->mode) {
                handle->state = LOADER_STATE_IDLE;
            }
            handle->mode = mode;
            break;

        default:
            return LPCLIB_ILLEGAL_PARAMETER;
    }

    return LPCLIB_SUCCESS;
}


static void _LOADER_sendResponse (LOADER_Handle handle, int action, const char *status)
{
    static char s[512];
    snprintf(s, sizeof(s), "%d,%s", action, status);
    SYS_send2Host(
        handle->mode == LOADER_MODE_FIRMWARE ? HOST_CHANNEL_FIRMWAREUPDATE : HOST_CHANNEL_CONFIGUPDATE,
        s);
}


/* Erase flash area occipied by firmware image */
static LPCLIB_Result _LOADER_eraseFirmware (LOADER_Handle handle)
{
    (void) handle;

    LPCLIB_Result iapResult;
    uint32_t sectorNumberFirst;
    uint32_t sectorNumberLast;


    __disable_irq();

    IAP_address2SectorNumber(FIRMWARE_START_ADDRESS, &sectorNumberFirst, NULL);
    IAP_address2SectorNumber(FIRMWARE_END_ADDRESS - 1, &sectorNumberLast, NULL);

    iapResult = IAP_prepareSectorsForWriteOperation(
            sectorNumberFirst,
            sectorNumberLast,
            NULL);
    if (iapResult == LPCLIB_SUCCESS) {
        iapResult = IAP_eraseSectors(
                sectorNumberFirst,
                sectorNumberLast,
                NULL);
    }

    __enable_irq();

    return iapResult;
}



/* Flush a dirty page to flash memory.
 * An attempt to write beyond the allowed flash range is treated as success
 * (silently ignore any data not part of the firmware image).
 */
static LPCLIB_Result _LOADER_flushPage (LOADER_Handle handle)
{
    LPCLIB_Result iapResult;
    uint32_t sectorNumber;
    uint32_t pageNumber;
    uint32_t address;


    if (!handle->pageDirty) {
        return LPCLIB_SUCCESS;
    }
    handle->pageDirty = false;

    if (handle->mode == LOADER_MODE_FIRMWARE) {
        /* In firmware update mode data is written to flash */
        address = 256 * handle->currentPage;  //TODO
        if (address < FIRMWARE_START_ADDRESS) {
            return LPCLIB_ERROR;    // Trying to write to loader sector is an error
        }
        if (address >= FIRMWARE_END_ADDRESS) {
            return LPCLIB_SUCCESS;  // Trying to write beyond image address range is accepted
        }

        __disable_irq();

        IAP_address2SectorNumber(address, &sectorNumber, NULL);
        IAP_address2PageNumber(address, &pageNumber, NULL);

        iapResult = IAP_prepareSectorsForWriteOperation(
                sectorNumber,
                sectorNumber,
                NULL);
        if (iapResult == LPCLIB_SUCCESS) {
            iapResult = IAP_copyRamToFlash(
                    address,
                    (uint32_t)&handle->page,
                    256,
                    NULL);
        }

        __enable_irq();

        return iapResult;
    }

    if (handle->mode == LOADER_MODE_CONFIG) {
        /* In config mode data is saved to RAM copy */
        address = 256 * handle->currentPage;
        if (address >= sizeof(Config_t)) {
            return LPCLIB_ERROR;
        }

        uint8_t *p = (uint8_t *)&handle->config;
        memcpy(&p[address], handle->page, 256);

        return LPCLIB_SUCCESS;
    }

    return LPCLIB_ERROR;
}



static LPCLIB_Result _LOADER_processHexRecord (LOADER_Handle handle, const char *record)
{
    LPCLIB_Result result = LPCLIB_ERROR;

    /* The record must have the format:
     * :LLaaaa00xxcc
     * LL = number of payload bytes (xx)
     * aaaa = address offset of first byte in record
     * xx = payload byte (repeat LL times)
     * cc = checksum
     */

    /* Read count and address */
    unsigned int count, address, type;
    if (sscanf(record, ":%2X%4X%2X", &count, &address, &type) == 3) {
        if (type == 2) {
            _LOADER_flushPage(handle);

            unsigned int val;
            if (sscanf(record, ":%*2X%*4X%*2X%4X", &val) == 1) {
                handle->segment = val;
                result = LPCLIB_SUCCESS;
            }

            /* Only firmware update uses segments */
            if (handle->mode == LOADER_MODE_CONFIG) {
                handle->segment = 0;
            }
        }

        if (type == 0) {
           result = LPCLIB_SUCCESS;

           /* The address in a hex record is the offset into the current 64-KiB block.
            * (block base address is in multiples of 16 bytes)
            */
            address += handle->segment * 16;

            /* Do the values make any sense? */
            if (count > 0) {
                unsigned int i;
                const char *p = &record[9];
                unsigned char checksum = count + (address % 256) + (address / 256);
                for (i = 0; i < count; i++) {
                    /* Preload page if new */
                    if ((address + i) / 256 != handle->currentPage) {
                        if (_LOADER_flushPage(handle) != LPCLIB_SUCCESS) {
                            result = LPCLIB_ERROR;
                        }

                        handle->pageDirty = false;
                        handle->currentPage = (address + i) / 256;
                        memset(handle->page, 0xFF, sizeof(handle->page));
                    }

                    /* Read payload and calculate checksum */
                    unsigned int byte;
                    if (sscanf(p, "%2X", &byte) == 1) {
                        handle->page[(address + i) % 256] = byte;
                        checksum += byte;
                        p += 2;

                        CRC_write(handle->crc, &byte, 1, NULL, NULL);

                        handle->pageDirty = true;
                    }

                    // TODO Verify checksum
                }
            }
        }
    }

    return result;
}



LPCLIB_Result LOADER_processCommand (LOADER_Handle handle, const char *commandLine)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;
    const char *response = LOADER_RESPONSE_ERROR;
    static char s[300];

    /* Read action (second field) */
    int action;
    if (sscanf(commandLine, "%*d,%d", &action) == 1) {
        switch (action) {
        case 0:     /* Ping. Response "1" indicates loader mode. */
            response = "1";
            break;

        case 1:     /* Start new transfer */
            /* If there is still flash activity going on, just note that we need to terminate */
            if (handle->state > __LOADER_STATE_FLASH_ACTION__) {
                handle->terminate = true;
            }
            else {
                if (handle->mode == LOADER_MODE_FIRMWARE) {
                    /* Erase firmware image */
                    _LOADER_eraseFirmware(handle);
                }

                CRC_seed(handle->crc, 0xFFFFFFFF);

                handle->state = LOADER_STATE_TRANSFER;
                handle->segment = 0;
                handle->currentPage = (uint16_t)-1;
                handle->pageDirty = false;
                handle->configOffset = 0;
                response = LOADER_RESPONSE_OK;
            }
            break;

        case 2:     /* HEX record */
            /* New records can only be accepted in transfer state */
            if (handle->state == LOADER_STATE_TRANSFER) {
                /* A HEX record must start with ':'. Process from there. */
                char *record = strchr(commandLine, ':');
                if (record) {
                    if (_LOADER_processHexRecord(handle, record) == LPCLIB_SUCCESS) {
                        /* Successfully processed */
                        response = LOADER_RESPONSE_OK;
                    }
                }
            }
            break;

        case 3:     /* Finalize transmission */
            /* We can only finalize if we are in transfer state */
            if (handle->state == LOADER_STATE_TRANSFER) {
                /* Flush partially filled buffer */
                if (_LOADER_flushPage(handle) == LPCLIB_SUCCESS) {
                    /* Verify received CRC-32 against the locally created checksum */
                    uint32_t hostCRC32;
                    if (sscanf(commandLine, "%*d,%*d,%"PRIu32, &hostCRC32) == 1) {
                        if (hostCRC32 == CRC_read(handle->crc)) {
                            if (handle->mode == LOADER_MODE_FIRMWARE) {
                                /* Check firmware stack address to distinguish Niobe1/2 */
                                uint32_t stackAddress = ((volatile uint32_t *)FIRMWARE_START_ADDRESS)[0];
                                bool firmwareAcceptable = false;
#if (BOARD_RA == 1)
                                firmwareAcceptable = (stackAddress < 0x04000000);
#endif
#if (BOARD_RA == 2)
                                firmwareAcceptable = (stackAddress >= 0x04000000);
#endif

                                if (firmwareAcceptable) {
                                    CRC_close(&handle->crc);

                                    /* Calculate signature of loaded firmware */
                                    signImage(FIRMWARE_START_ADDRESS, FIRMWARE_END_ADDRESS);

                                    response = LOADER_RESPONSE_OK;
                                    handle->state = LOADER_STATE_IDLE;

                                    // Restart system after some time
                                    osTimerStart(handle->actionTick, 500);
                                }
                            }

                            if (handle->mode == LOADER_MODE_CONFIG) {
                                uint32_t sectorNumber;
                                uint32_t pageNumber1;
                                uint32_t pageNumber2;

                                /* Save config block in flash. */
                                __disable_irq();

                                IAP_address2SectorNumber((uint32_t)&__CONFIG_ADDRESS__, &sectorNumber, NULL);
                                IAP_address2PageNumber((uint32_t)&__CONFIG_ADDRESS__, &pageNumber1, NULL);
                                IAP_address2PageNumber((uint32_t)&__CONFIG_ADDRESS__ + sizeof(Config_t) - 1, &pageNumber2, NULL);

                                result = IAP_prepareSectorsForWriteOperation(sectorNumber, sectorNumber, NULL);
                                if (result == LPCLIB_SUCCESS) {
                                    result = IAP_erasePages(pageNumber1, pageNumber2, NULL);
                                    if (result == LPCLIB_SUCCESS) {
                                        result = IAP_prepareSectorsForWriteOperation(sectorNumber, sectorNumber, NULL);
                                        if (result == LPCLIB_SUCCESS) {
                                            result = IAP_copyRamToFlash(
                                                    (uint32_t)&__CONFIG_ADDRESS__,
                                                    (uint32_t)&handle->config,
                                                    sizeof(Config_t),   /* NOTE: This MUST be a multiple of 256! */
                                                    NULL);
                                            if (result == LPCLIB_SUCCESS) {
                                                response = LOADER_RESPONSE_OK;
                                            }
                                        }
                                    }
                                }

                                __enable_irq();
                            }
                        }
                    }
                }
            }
            break;

        case 4:     /* Read */
            {
                if (handle->configOffset >= sizeof(Config_t)) {
                    /* When end of config structure reached, just send indicator */
                    response = "1";
                } else {
                    /* Send next chunk of config structure */
                    uint8_t *p = (uint8_t *)&__CONFIG_ADDRESS__;
                    int n = snprintf(s, sizeof(s), "0,:00%04"PRIX32"80", handle->configOffset);
                    uint8_t checksum = 0;
                    for (int i = 0; i < 128; i++) {
                        uint8_t b = p[handle->configOffset + i];
                        n += snprintf(&s[n], sizeof(s) - n, "%02X", b);
                        checksum -= b;
                    }
                    snprintf(&s[n], sizeof(s) - n, "%02X", checksum);
                    response = s;

                    handle->configOffset = (handle->configOffset >= sizeof(Config_t)) ?
                            handle->configOffset
                        : handle->configOffset + 128;
                }
            }
            break;
        }

        _LOADER_sendResponse(handle, action, response);
    }

    return result;
}


static bool _LOADER_checkEvent (LOADER_Handle handle)
{
    (void) handle;

    bool haveEvent = false;

    return haveEvent;
}


static void _LOADER_rtosCallback (void const *argument)
{
    (void) argument;

    //TODO
    NVIC_SystemReset();
}


osTimerDef(actionTickDef, _LOADER_rtosCallback);

PT_THREAD(LOADER_thread (LOADER_Handle handle))
{
    PT_BEGIN(&handle->pt);

//    handle->queue = osMailCreate(osMailQ(loaderQueueDef), NULL);

    handle->actionTick = osTimerCreate(osTimer(actionTickDef), osTimerOnce, (void *)LOADER_TIMERMAGIC_RESTART);

    CRC_open(
        CRC_makeMode(CRC_POLY_CRC32,
                        CRC_DATAORDER_REVERSE,
                        CRC_SUMORDER_REVERSE,
                        CRC_DATAPOLARITY_NORMAL,
                        CRC_SUMPOLARITY_INVERSE),
        &handle->crc);

    while (1) {
        /* Wait for an event */
        PT_WAIT_UNTIL(&handle->pt, _LOADER_checkEvent(handle));
#if 0
        /* Is there a new message? */
        if (handle->rtosEvent.status == osEventMail) {
            pMessage = (SYS_Message *)handle->rtosEvent.value.p;
            switch (pMessage->opcode) {
            }

            osMailFree(handle->queue, pMessage);
        }
#endif
    }

    PT_END(&handle->pt);
}


