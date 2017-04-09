/* Copyright (c) 2016, DF9DQ
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

#include "lpc54xxx_libconfig.h"

/** \file
 *  \brief SPI driver implementation.
 *
 *  This file contains the driver code for the SPI peripheral.
 */


/** \addtogroup SPI
 *  @{
 */

#include <string.h>

#include "lpc54xxx_clkpwr.h"
#include "lpc54xxx_spi.h"


LPCLIB_DefineRegBit(SPI_CFG_ENABLE,                 0,  1);
LPCLIB_DefineRegBit(SPI_CFG_MASTER,                 2,  1);
LPCLIB_DefineRegBit(SPI_CFG_LSBF,                   3,  1);
LPCLIB_DefineRegBit(SPI_CFG_CPHA,                   4,  1);
LPCLIB_DefineRegBit(SPI_CFG_CPOL,                   5,  1);
LPCLIB_DefineRegBit(SPI_CFG_LOOP,                   7,  1);
LPCLIB_DefineRegBit(SPI_CFG_SPOL0,                  8,  1);
LPCLIB_DefineRegBit(SPI_CFG_SPOL1,                  9,  1);
LPCLIB_DefineRegBit(SPI_CFG_SPOL2,                  10, 1);
LPCLIB_DefineRegBit(SPI_CFG_SPOL3,                  11, 1);

LPCLIB_DefineRegBit(SPI_DLY_PRE_DELAY,              0,  4);
LPCLIB_DefineRegBit(SPI_DLY_POST_DELAY,             4,  4);
LPCLIB_DefineRegBit(SPI_DLY_FRAME_DELAY,            8,  4);
LPCLIB_DefineRegBit(SPI_DLY_TRANSFER_DELAY,         12, 4);

LPCLIB_DefineRegBit(SPI_STAT_RXRDY,                 0,  1);
LPCLIB_DefineRegBit(SPI_STAT_TXRDY,                 1,  1);
LPCLIB_DefineRegBit(SPI_STAT_RXOV,                  2,  1);
LPCLIB_DefineRegBit(SPI_STAT_TXUR,                  3,  1);
LPCLIB_DefineRegBit(SPI_STAT_SSA,                   4,  1);
LPCLIB_DefineRegBit(SPI_STAT_SSD,                   5,  1);
LPCLIB_DefineRegBit(SPI_STAT_STALLED,               6,  1);
LPCLIB_DefineRegBit(SPI_STAT_END_TRANSFER,          7,  1);
LPCLIB_DefineRegBit(SPI_STAT_MSTIDLE,               8,  1);

LPCLIB_DefineRegBit(SPI_INTENSET_RXRDY,             0,  1);
LPCLIB_DefineRegBit(SPI_INTENSET_TXRDY,             1,  1);
LPCLIB_DefineRegBit(SPI_INTENSET_RXOV,              2,  1);
LPCLIB_DefineRegBit(SPI_INTENSET_TXUR,              3,  1);
LPCLIB_DefineRegBit(SPI_INTENSET_SSA,               4,  1);
LPCLIB_DefineRegBit(SPI_INTENSET_SSD,               5,  1);
LPCLIB_DefineRegBit(SPI_INTENSET_MSTIDLE,           8,  1);

LPCLIB_DefineRegBit(SPI_INTENCLR_RXRDY,             0,  1);
LPCLIB_DefineRegBit(SPI_INTENCLR_TXRDY,             1,  1);
LPCLIB_DefineRegBit(SPI_INTENCLR_RXOV,              2,  1);
LPCLIB_DefineRegBit(SPI_INTENCLR_TXUR,              3,  1);
LPCLIB_DefineRegBit(SPI_INTENCLR_SSA,               4,  1);
LPCLIB_DefineRegBit(SPI_INTENCLR_SSD,               5,  1);
LPCLIB_DefineRegBit(SPI_INTENCLR_MSTIDLE,           8,  1);

LPCLIB_DefineRegBit(SPI_RXDAT_RXDAT,                0,  16);
LPCLIB_DefineRegBit(SPI_RXDAT_RXSSEL0_N,            16, 1);
LPCLIB_DefineRegBit(SPI_RXDAT_RXSSEL1_N,            17, 1);
LPCLIB_DefineRegBit(SPI_RXDAT_RXSSEL2_N,            18, 1);
LPCLIB_DefineRegBit(SPI_RXDAT_RXSSEL3_N,            19, 1);
LPCLIB_DefineRegBit(SPI_RXDAT_SOT,                  20, 1);

LPCLIB_DefineRegBit(SPI_TXDATCTL_TXDAT,             0,  16);
LPCLIB_DefineRegBit(SPI_TXDATCTL_TXSSEL0_N,         16, 1);
LPCLIB_DefineRegBit(SPI_TXDATCTL_TXSSEL1_N,         17, 1);
LPCLIB_DefineRegBit(SPI_TXDATCTL_TXSSEL2_N,         18, 1);
LPCLIB_DefineRegBit(SPI_TXDATCTL_TXSSEL3_N,         19, 1);
LPCLIB_DefineRegBit(SPI_TXDATCTL_EOT,               20, 1);
LPCLIB_DefineRegBit(SPI_TXDATCTL_EOF,               21, 1);
LPCLIB_DefineRegBit(SPI_TXDATCTL_RXIGNORE,          22, 1);
LPCLIB_DefineRegBit(SPI_TXDATCTL_LEN,               24, 4);

LPCLIB_DefineRegBit(SPI_TXDAT_DATA,                 0,  16);

LPCLIB_DefineRegBit(SPI_TXCTL_TXSSEL0_N,            16, 1);
LPCLIB_DefineRegBit(SPI_TXCTL_TXSSEL1_N,            17, 1);
LPCLIB_DefineRegBit(SPI_TXCTL_TXSSEL2_N,            18, 1);
LPCLIB_DefineRegBit(SPI_TXCTL_TXSSEL3_N,            19, 1);
LPCLIB_DefineRegBit(SPI_TXCTL_EOT,                  20, 1);
LPCLIB_DefineRegBit(SPI_TXCTL_EOF,                  21, 1);
LPCLIB_DefineRegBit(SPI_TXCTL_RXIGNORE,             22, 1);
LPCLIB_DefineRegBit(SPI_TXCTL_LEN,                  24, 4);

LPCLIB_DefineRegBit(SPI_DIV_DIVVAL,                 0,  16);

LPCLIB_DefineRegBit(SPI_INTSTAT_RXRDY,              0,  1);
LPCLIB_DefineRegBit(SPI_INTSTAT_TXRDY,              1,  1);
LPCLIB_DefineRegBit(SPI_INTSTAT_RXOV,               2,  1);
LPCLIB_DefineRegBit(SPI_INTSTAT_TXUR,               3,  1);
LPCLIB_DefineRegBit(SPI_INTSTAT_SSA,                4,  1);
LPCLIB_DefineRegBit(SPI_INTSTAT_SSD,                5,  1);
LPCLIB_DefineRegBit(SPI_INTSTAT_MSTIDLE,            8,  1);


static struct SPI_Context {
    SPI_Name bus;                           /**< Bus identifier */
    LPCLIB_Switch inUse;                    /**< Set if interface open */
    uint32_t bitrate;                       /**< Bit clock frequency (Hz) */
    SPI_Bits bitsPerFrame;

    LPCLIB_Result errorStatus;              /**< Error status returned with end of transaction */
    uint16_t nRxDummies;
    const SPI_Job *job;
    SPI_JobPhase *txPhase;
    SPI_JobPhase *rxPhase;
    osMutexId accessMutex;
    osSemaphoreId syncSema;
    uint16_t nsent;                         /**< Local context: TX frame counter */
    uint16_t nreceived;                     /**< Local context: RX frame counter */
} spiContext[SPI_NUM_BUSSES];


//TODO make sure we have enough (and not more) mutexes/syncs!
//TODO must have *globally* unique name...
osMutexDef(spiAccessMutexDef0);
osMutexDef(spiAccessMutexDef1);
osSemaphoreDef(spiSyncSemaDef0);
osSemaphoreDef(spiSyncSemaDef1);


static const osMutexDef_t * const spiMutexes[SPI_NUM_BUSSES] = {
    osMutex(spiAccessMutexDef0), osMutex(spiAccessMutexDef1),
    };
static const osSemaphoreDef_t * const spiSemas[SPI_NUM_BUSSES] = {
    osSemaphore(spiSyncSemaDef0), osSemaphore(spiSyncSemaDef1),
    };
static LPC_SPI_Type * const spiPtr[SPI_NUM_BUSSES] = {LPC_SPI0, LPC_SPI1,};
static const CLKPWR_Clockswitch spiClockswitch[SPI_NUM_BUSSES] = {CLKPWR_CLOCKSWITCH_SPI0, CLKPWR_CLOCKSWITCH_SPI1,};
static const CLKPWR_Clock spiClock[SPI_NUM_BUSSES] = {CLKPWR_CLOCK_SPI0, CLKPWR_CLOCK_SPI1,};


/* Open an SPI bus. */
LPCLIB_Result SPI_open (SPI_Name bus, SPI_Handle *pHandle)
{
    SPI_Handle handle = &spiContext[bus];

    CLKPWR_enableClock(spiClockswitch[bus]);            /* Enable SPIx peripheral clock */

    if (!handle->inUse) {                               /* Do nothing if already enabled */
        handle->accessMutex = osMutexCreate(spiMutexes[bus]);
        handle->syncSema = osSemaphoreCreate(spiSemas[bus], 1);

        handle->bus = bus;
        handle->inUse = LPCLIB_YES;
        *pHandle = handle;                              /* Return bus handle to caller */

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;
}



/* Close an SSP bus. */
void SPI_close (SPI_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    CLKPWR_disableClock(spiClockswitch[(*pHandle)->bus]);   /* Disable SPIx peripheral clock */

//TODO delete RTOS objects

    (*pHandle)->inUse = LPCLIB_NO;
    *pHandle = LPCLIB_INVALID_HANDLE;
}



/* Set options of the SPI block. */
void SPI_ioctl (SPI_Handle handle, const SPI_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    uint32_t divider;
    LPC_SPI_Type * const spi = spiPtr[handle->bus];

    if (pConfig == NULL) {
        return;
    }

    while (pConfig->opcode != SPI_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case SPI_OPCODE_SET_FORMAT:                     /* Set phase/polarity */
            handle->bitsPerFrame = pConfig->format.bits;
            spi->TXCTL = (spi->TXCTL & ~SPI_TXCTL_LEN_Msk) | (handle->bitsPerFrame << SPI_TXCTL_LEN_Pos);
            spi->CFG = (spi->CFG & ~(SPI_CFG_CPHA_Msk | SPI_CFG_CPOL_Msk)) |
                        ((uint32_t)pConfig->format.clockFormat << SPI_CFG_CPHA_Pos);
            break;

        case SPI_OPCODE_SET_MODE:                       /* Select master/slave mode */
            spi->CFG = (spi->CFG & ~SPI_CFG_MASTER_Msk) | (pConfig->mode << SPI_CFG_MASTER_Pos);
            break;

        case SPI_OPCODE_SET_BITRATE:                    /* Set clock speed */
            /* Find divider that guarantees pConfig->bitrate is not exceeded */
            divider = (CLKPWR_getBusClock(spiClock[handle->bus]) + pConfig->bitrate - 1) / pConfig->bitrate;
            if (divider > 0) {
                divider -= 1;
            }

            if (!(spi->CFG & SPI_CFG_MASTER_Msk)) {
                divider = 0;                            /* Slave mode: Set divider to zero (divider i not used anyway) */
            }

            spi->DIV = (divider << SPI_DIV_DIVVAL_Pos) & SPI_DIV_DIVVAL_Msk;
            break;

        case SPI_OPCODE_INVALID:
            /* not handled */
            break;
        }

        ++pConfig;
    }

    spi->CFG |= SPI_CFG_ENABLE_Msk;
}



/* Submit a job to the SPI driver. */
LPCLIB_Result SPI_submitJob (SPI_Handle handle, const SPI_Job *pJob)
{
    LPCLIB_Event event;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Get exclusive access */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        handle->job = pJob;
        SPI_ioctl(handle, pJob->pConfig);               /* Configure the device */

        /* Make sure to reset the sync flag */
        osSemaphoreWait(handle->syncSema, 0);

    //TODO Check master/slave
    //Here: master
        /* Assert chip select before sending data */
        /* Prepare event */
        if (pJob->pDeviceSelect) {
            if (pJob->pDeviceSelect->callback) {
                event.id = LPCLIB_EVENTID_SPI;
                event.block = handle->bus;
                event.opcode = SPI_EVENT_ASSERT_CHIPSELECT;
                event.channel = pJob->pDeviceSelect->channel;
                event.parameter = pJob->extraParameter;
                pJob->pDeviceSelect->callback(event);
            }
        }

        handle->nsent = 0;
        handle->nreceived = 0;
        handle->txPhase = pJob->firstPhase;
        handle->rxPhase = pJob->firstPhase;

        /* Enable RX interrupts now. Enabling TX interrupts (= starting transmission)
         * must be done separately by calling SSP_run().
         */
        spiPtr[handle->bus]->INTENSET = SPI_INTENSET_RXRDY_Msk;

        /* Return now for asynchronous call */
        if (pJob->callback) {
            osMutexRelease(handle->accessMutex);

            return LPCLIB_PENDING;
        }

        if (pJob->noWaitComplete) {
            osMutexRelease(handle->accessMutex);
            return LPCLIB_SUCCESS;
        }

        /* In sync mode force TX interrupt */
        spiPtr[handle->bus]->INTENSET = SPI_INTENSET_TXRDY_Msk;

        /* In sync mode we wait for the end of transaction. */
        if (osSemaphoreWait(handle->syncSema, 1000 /*TODO osWaitForever*/) > 0) {
            osMutexRelease(handle->accessMutex);

            return handle->errorStatus;
        }

        osMutexRelease(handle->accessMutex);

        //TODO: serious error! Restart SSP bus, re-init semaphores
        //TODO: when will semaacquire bring us here??
        return LPCLIB_ERROR;
    }

    return LPCLIB_BUSY;
}



/* Enable the SPI. */
LPCLIB_Result SPI_run (SPI_Handle handle)
{
    LPC_SPI_Type * const spi = spiPtr[handle->bus];

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if ((spi->CFG & (SPI_CFG_ENABLE_Msk | SPI_CFG_MASTER_Msk)) != (SPI_CFG_ENABLE_Msk | SPI_CFG_MASTER_Msk)) {
        return LPCLIB_NOT_PREPARED;                     /* Not active in master mode */
    }
    if (spi->INTENSET & SPI_INTENSET_TXRDY_Msk) {       //TODO is this ok?
        return LPCLIB_NOT_PREPARED;                     /* Already active */
    }

    spi->INTENSET = SPI_INTENSET_TXRDY_Msk;

    return LPCLIB_SUCCESS;
}



/* Cancel an active SPI job */
LPCLIB_Result SPI_cancel (SPI_Handle handle)
{
    LPC_SPI_Type * const spi = spiPtr[handle->bus];
    LPCLIB_Event event;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Stop all SPI activity */
    spi->INTENCLR = 0                       /* Disable all interrupts */
        | SPI_INTENCLR_RXRDY_Msk
        | SPI_INTENCLR_TXRDY_Msk
        | SPI_INTENCLR_RXOV_Msk
        | SPI_INTENCLR_TXUR_Msk
        | SPI_INTENCLR_SSA_Msk
        | SPI_INTENCLR_SSD_Msk
        | SPI_INTENCLR_MSTIDLE_Msk
        ;
    spi->CFG &= ~SPI_CFG_ENABLE_Msk;        /* Disable SPI block */

    if (handle->job) {
        event.id = LPCLIB_EVENTID_SPI;
        event.block = handle->bus;

        if (handle->job->pDeviceSelect) {
            if (handle->job->pDeviceSelect->callback) {
                event.opcode = SPI_EVENT_DEASSERT_CHIPSELECT;
                event.channel = handle->job->pDeviceSelect->channel;
                event.parameter = handle->job->extraParameter;
                handle->job->pDeviceSelect->callback(event);
            }
        }

        event.opcode = SPI_EVENT_CANCELLED;
        if (handle->job->callback) {
            handle->job->callback(event);
        }

        handle->job = NULL;
    }

    return LPCLIB_SUCCESS;
}



/* Fill data into an SSP_JobPhase descriptor. */
SPI_JobPhase *SPI_makePhase (SPI_JobPhase *pPhase, const void *pTx, uint8_t *pRx, uint16_t length)
{
    pPhase->next = NULL;
    pPhase->txstart8 = (uint8_t *)pTx;
    pPhase->rxstart8 = pRx;
    pPhase->length = length;
    pPhase->idlePattern = 0;
    pPhase->interrupt = DISABLE;
    pPhase->txBarrier = DISABLE;

    return pPhase;
}


/** Common IRQ handler for SPI blocks.
 *
 *  \param[in] sspNum Indicator for SPI block.
 */
static void SPI_commonIRQHandler (SPI_Name bus)
{
    LPC_SPI_Type * const spi = spiPtr[bus];
    SPI_Handle handle = &spiContext[bus];
    uint32_t status;
    uint32_t rxFrame;
    int nFrames;
    LPCLIB_Event event;
    int i;
    _Bool stopTx;
    _Bool done = false;


    /* Prepare event */
    event.id = LPCLIB_EVENTID_SPI;
    event.block = bus;

    status = spi->INTSTAT;

    if (status & SPI_INTSTAT_RXRDY_Msk) {
        nFrames = 1;

        for (i = 0; i < nFrames; i++) {
            rxFrame = (spi->RXDAT & SPI_RXDAT_RXDAT_Msk) >> SPI_RXDAT_RXDAT_Pos;

            if (handle->job) {
                /* Store the frame? */
                if (handle->bitsPerFrame <= SPI_BITS_8) {
                    if (handle->rxPhase->rxstart8) {
                        handle->rxPhase->rxstart8[handle->nreceived] = rxFrame;
                    }
                }
                else {
                    if (handle->rxPhase->rxstart16) {
                        handle->rxPhase->rxstart16[handle->nreceived] = rxFrame;
                    }
                }
                ++(handle->nreceived);

                /* End of transfer? */
                if (handle->nreceived >= handle->rxPhase->length) {
                    handle->nreceived = 0;

                    if (handle->rxPhase->interrupt) {           /* Send "interrupt" event to application? */
                        event.opcode = SPI_EVENT_PHASE_COMPLETE;
                        event.parameter = handle->rxPhase;
                        if (handle->job->callback) {
                            handle->job->callback(event);
                        }

                        spi->INTENSET = SPI_INTENSET_TXRDY_Msk; /* TX interrupts allowed */
                    }

                    handle->rxPhase = handle->rxPhase->next;    /* Next RX phase */

                    if (!handle->rxPhase) {                     /* Will there be another phase? */
                        spi->INTENCLR =                         /* No. Disable further interrupts */
                            SPI_INTENCLR_RXRDY_Msk | SPI_INTENCLR_TXRDY_Msk;

                        if (handle->job->pDeviceSelect) {
                            if (handle->job->pDeviceSelect->callback) {
                                event.opcode = SPI_EVENT_DEASSERT_CHIPSELECT;
                                event.channel = handle->job->pDeviceSelect->channel;
                                handle->job->pDeviceSelect->callback(event);
                            }
                        }

                        done = true;
                        break;
                    }
                }
            }
        }
    }

    /* Handle TX interrupts */
    if (status & SPI_INTSTAT_TXRDY_Msk) {
        if (handle->job) {
            nFrames = handle->txPhase->length               /* How much is left to be sent? */
                        - handle->nsent;
            if (nFrames > 1) {
                nFrames = 1;                                /* Limit to half the FIFO size */
            }
if(nFrames<0)nFrames=0;
            /* Send that block */
            if (handle->txPhase->txstart8) {
                if (handle->bitsPerFrame <= SPI_BITS_8) {
                    for (i = 0; i < nFrames; i++) {
                        spi->TXDAT = handle->txPhase->txstart8[handle->nsent + i];
                    }
                }
                else {
                    for (i = 0; i < nFrames; i++) {
                        spi->TXDAT = handle->txPhase->txstart16[handle->nsent + i];
                    }
                }
            }
            else {
                for (i = 0; i < nFrames; i++) {                 /* Send that block */
                    spi->TXDAT = handle->txPhase->idlePattern;
                }
            }
            handle->nsent += nFrames;

            if (nFrames <= 0) {                             /* Check for end of transmission (in this phase) */
                handle->nsent = 0;                          /* Reset counter for next phase */
                stopTx =                                    /* Shall we stop further TX activity for now? */
                    (handle->txPhase->txBarrier != DISABLE) /*   Yes if current phase has TX barrier */
                    || (handle->txPhase->next == NULL);        /*   Yes if we are in the last phase already */
                handle->txPhase = handle->txPhase->next;    /* Next TX phase */
                if (stopTx) {
                    spi->INTENCLR = SPI_INTENCLR_TXRDY_Msk; /* Disable further TX interrupts */
                }
            }
        }
        else {
            spi->INTENCLR = SPI_INTENCLR_TXRDY_Msk;         /* Disable further TX interrupts */
        }
    }

    /* Let the caller know when the transaction is over. */
    if (done) {
        if (handle->job->callback) {
            event.opcode = SPI_EVENT_JOB_COMPLETE;
            event.channel = handle->job->pDeviceSelect->channel;
            handle->job->callback(event);
        }
        osSemaphoreRelease(handle->syncSema);
    }
}


/** Hardware entry for SPI0 interrupt. */
void SPI0_IRQHandler (void)
{
    SPI_commonIRQHandler(SPI0);
}


/** Hardware entry for SPI1 interrupt. */
void SPI1_IRQHandler (void)
{
    SPI_commonIRQHandler(SPI1);
}


/** @} SSP */
