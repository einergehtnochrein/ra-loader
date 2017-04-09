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
 *  \brief I2C driver implementation.
 *
 *  This file contains the driver code for the I2C peripheral.
 */


/** \addtogroup I2C
 *  @{
 */


#include "lpc54xxx_i2c.h"
#if LPCLIB_I2CEMU
#include "i2c-bitbang.h"
#endif
#include "lpc54xxx_clkpwr.h"


/** Field definition for hardware register I2CxCONSET. */
LPCLIB_DefineRegBit(I2C_CFG_MSTEN,              0,  1);
LPCLIB_DefineRegBit(I2C_CFG_SLVEN,              1,  1);
LPCLIB_DefineRegBit(I2C_CFG_MONEN,              2,  1);
LPCLIB_DefineRegBit(I2C_CFG_TIMEOUTEN,          3,  1);
LPCLIB_DefineRegBit(I2C_CFG_MONCLKSTR,          4,  1);
LPCLIB_DefineRegBit(I2C_CFG_HSCAPABLE,          5,  1);

LPCLIB_DefineRegBit(I2C_INTENSET_MSTPENDINGEN,  0,  1);
LPCLIB_DefineRegBit(I2C_INTENSET_MSTARBLOSSEN,  4,  1);
LPCLIB_DefineRegBit(I2C_INTENSET_MSTSTSTPERREN, 6,  1);
LPCLIB_DefineRegBit(I2C_INTENSET_SLVPENDINGEN,  8,  1);
LPCLIB_DefineRegBit(I2C_INTENSET_SLVNOTSTREN,   11, 1);
LPCLIB_DefineRegBit(I2C_INTENSET_SLVDESELEN,    15, 1);
LPCLIB_DefineRegBit(I2C_INTENSET_MONRDYEN,      16, 1);
LPCLIB_DefineRegBit(I2C_INTENSET_MONOVEN,       17, 1);
LPCLIB_DefineRegBit(I2C_INTENSET_MONIDLEEN,     19, 1);
LPCLIB_DefineRegBit(I2C_INTENSET_EVENTTIMEOUTEN,24, 1);
LPCLIB_DefineRegBit(I2C_INTENSET_SCLTIMEOUTEN,  25, 1);

LPCLIB_DefineRegBit(I2C_MSTCTL_MSTCONTINUE,     0,  1);
LPCLIB_DefineRegBit(I2C_MSTCTL_MSTSTART,        1,  1);
LPCLIB_DefineRegBit(I2C_MSTCTL_MSTSTOP,         2,  1);
LPCLIB_DefineRegBit(I2C_MSTCTL_MSTDMA,          3,  1);

LPCLIB_DefineRegBit(I2C_MSTTIME_MSTSCLLOW,      0,  3);
LPCLIB_DefineRegBit(I2C_MSTTIME_MSTSCLHIGH,     4,  3);

LPCLIB_DefineRegBit(I2C_STAT_MSTPENDING,        0,  1);
LPCLIB_DefineRegBit(I2C_STAT_MSTSTATE,          1,  3);
LPCLIB_DefineRegBit(I2C_STAT_MSTARBLOSS,        4,  1);
LPCLIB_DefineRegBit(I2C_STAT_MSTSTSTPERR,       6,  1);
LPCLIB_DefineRegBit(I2C_STAT_SLVPENDING,        8,  1);
LPCLIB_DefineRegBit(I2C_STAT_SLVSTATE,          9,  2);
LPCLIB_DefineRegBit(I2C_STAT_SLVNOTSTR,         11, 1);
LPCLIB_DefineRegBit(I2C_STAT_SLVIDX,            12, 2);
LPCLIB_DefineRegBit(I2C_STAT_SLVSEL,            14, 1);
LPCLIB_DefineRegBit(I2C_STAT_SLVDESEL,          15, 1);
LPCLIB_DefineRegBit(I2C_STAT_MONRDY,            16, 1);
LPCLIB_DefineRegBit(I2C_STAT_MONOV,             17, 1);
LPCLIB_DefineRegBit(I2C_STAT_MONACTIVE,         18, 1);
LPCLIB_DefineRegBit(I2C_STAT_MONIDLE,           19, 1);
LPCLIB_DefineRegBit(I2C_STAT_EVENTTIMEOUT,      24, 1);
LPCLIB_DefineRegBit(I2C_STAT_SCLTIMEOUT,        25, 1);

/** List of possible status codes. */
typedef enum {
    I2C_MASTERSTATE_IDLE = 0,
    I2C_MASTERSTATE_RX_READY = 1,
    I2C_MASTERSTATE_TX_READY = 2,
    I2C_MASTERSTATE_NACK_ADDRESS = 3,
    I2C_MASTERSTATE_NACK_DATA = 4,
} I2C_MasterState;



/* I2C state machine */
typedef enum {
    I2C_STATE_UNDEFINED = 0,
    I2C_STATE_IDLE,
    I2C_STATE_MASTER_SENT_SLAVEADDRESS,
} I2C_State;


/** Local context of I2C busses. */
static struct I2C_Context {
    /* NOTE:
     * The first element MUST be "I2C_Name bus".
     * This structure definition can be done in multiple C source files
     * (I2C GPIO bit-banging), and "bus" must exist in all of these
     * possible implementations at offset 0.
     */
    I2C_Name bus;                           /**< Bus identifier */
    LPCLIB_Switch inUse;                    /**< Set if interface open */
    LPCLIB_Result errorStatus;              /**< Error status returned with end of transaction */
    volatile uint32_t transferred;          /**< Transfer counter. */
    volatile I2C_Job *job;
    volatile I2C_Job *jobOnHold;
    volatile const I2C_JobPhase *phase;
    osMutexId accessMutex;
    osSemaphoreId syncSema;
    I2C_State state;

#if LPCLIB_I2C_SLAVE
    uint8_t addressedAs;                    /**< Actual address by which the device
                                             *   was addressed in slave mode.
                                             */
    LPCLIB_Callback eventHandler;           /**< Receives slave/monitor events */
  #if (LPCLIB_I2C_NUM_SLAVE_ADDRESSES > 1)
    uint8_t addressUsed[4];                 /* Flag to indicate that address comparator is used */
  #endif
#endif

} i2cContext[I2C_NUM_BUSSES];


//TODO make sure we have enough (and not more) mutexes/syncs!
//TODO must have *globally* unique name...
osMutexDef(i2cAccessMutexDef0);
osMutexDef(i2cAccessMutexDef1);
osMutexDef(i2cAccessMutexDef2);
osSemaphoreDef(i2cSyncSemaDef0);
osSemaphoreDef(i2cSyncSemaDef1);
osSemaphoreDef(i2cSyncSemaDef2);


static const osMutexDef_t * const i2cMutexes[I2C_NUM_BUSSES] = {
    osMutex(i2cAccessMutexDef0), osMutex(i2cAccessMutexDef1), osMutex(i2cAccessMutexDef2),
    };
static const osSemaphoreDef_t * const i2cSemas[I2C_NUM_BUSSES] = {
    osSemaphore(i2cSyncSemaDef0), osSemaphore(i2cSyncSemaDef1), osSemaphore(i2cSyncSemaDef2),
    };

/** Peripheral bus address of I2C block(s). */
static LPC_I2C_Type * const i2c_ptr[I2C_NUM_BUSSES] = {LPC_I2C0, LPC_I2C1, LPC_I2C2};
static const CLKPWR_Clockswitch i2c_clockSwitch[I2C_NUM_BUSSES] = {
        CLKPWR_CLOCKSWITCH_I2C0,
        CLKPWR_CLOCKSWITCH_I2C1,
        CLKPWR_CLOCKSWITCH_I2C2,
        };
static const CLKPWR_Clock i2c_clock[I2C_NUM_BUSSES] = {
        CLKPWR_CLOCK_I2C0,
        CLKPWR_CLOCK_I2C1,
        CLKPWR_CLOCK_I2C2,
        };


/* Open an I2C bus for use. */
LPCLIB_Result I2C_open (I2C_Name bus, I2C_Handle *pHandle)
{
    LPC_I2C_Type *i2c;
    I2C_Handle handle;

#if LPCLIB_I2CEMU
    /* Check for bus emulation */
    if (bus > I2C_NUM_BUSSES) {
        return I2CEMU_open(bus, pHandle);
    }
#endif

    i2c = i2c_ptr[bus];
    handle = &i2cContext[bus];

    CLKPWR_enableClock(i2c_clockSwitch[bus]);           /* Enable I2C peripheral clock */

    /* Cannot open an I2C twice */
    if (!handle->inUse) {
        handle->accessMutex = osMutexCreate(i2cMutexes[bus]);
        handle->syncSema = osSemaphoreCreate(i2cSemas[bus], 1);

        handle->bus = bus;
        handle->inUse = LPCLIB_YES;
        handle->state = I2C_STATE_IDLE;
        *pHandle = handle;                              /* Return handle */

    #if LPCLIB_I2C_SLAVE
        i2c->CFG = 0
                | (1u << I2C_CFG_MSTEN_Pos)
                | (1u << I2C_CFG_SLVEN_Msk)
                ;
        i2c->SLVADR0 = (LPCLIB_I2C_SLAVE_DEFAULT_ADDRESS) << 1;
    #else
        i2c->CFG = 0
                | (1u << I2C_CFG_MSTEN_Pos)
                | (0u << I2C_CFG_SLVEN_Msk)
                ;
    #endif

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;
}



/* Close an I2C bus. */
void I2C_close (I2C_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

#if LPCLIB_I2CEMU
    if ((*pHandle)->bus > I2C_NUM_BUSSES) {
        I2CEMU_close(pHandle);
        return;
    }
#endif

    CLKPWR_disableClock(i2c_clockSwitch[(*pHandle)->bus]);  /* Disable I2C peripheral clock */

    (*pHandle)->inUse = LPCLIB_NO;
    *pHandle = LPCLIB_INVALID_HANDLE;
}



/* Configure the I2C block. */
void I2C_ioctl (I2C_Handle handle, const I2C_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    uint32_t temp;
    LPC_I2C_Type *i2c;

#if LPCLIB_I2CEMU
    if (handle->bus > I2C_NUM_BUSSES) {
        I2CEMU_ioctl(handle, pConfig);
        return;
    }
#endif

    i2c = i2c_ptr[handle->bus];

    while (pConfig->opcode != I2C_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case I2C_OPCODE_SET_BITRATE:
            temp = CLKPWR_getBusClock(i2c_clock[handle->bus]);
            temp /= pConfig->bitrate;
//TODO TODO
            i2c->CLKDIV = temp / 4;
            i2c->MSTTIME = 0
                | (0 << I2C_MSTTIME_MSTSCLLOW_Pos)
                | (0 << I2C_MSTTIME_MSTSCLHIGH_Pos)
                ;
            break;

#if LPCLIB_I2C_SLAVE
        case I2C_OPCODE_SET_SLAVE_ADDRESS:
            if (LPCLIB_I2C_NUM_SLAVE_ADDRESSES > 1) {
                if (pConfig->slave.index < 4) {
                    i2c->SLVADR[pConfig->slave.index] = pConfig->slave.address;
                }
            }
            else {
                i2c->SLVADR0 = pConfig->slave.address;
            }
            break;
#endif

#if LPCLIB_I2CEMU
        case I2C_OPCODE_EMU_DEFINE_PINS:
            /* Nothing to handle here (only used in bit-banging emulator) */
            break;
#endif

        case I2C_OPCODE_INVALID:
            /* Dummy to suppress compiler warning */
            break;
        }

        ++pConfig;
    }
}



/* Single buffer write operation to a device. */
LPCLIB_Result I2C_write (I2C_Handle handle,
                         I2C_Address address,
                         int numBytes,
                         const void *txBuffer,
                         LPCLIB_Callback callback)
{
    I2C_Job job;
    I2C_JobPhase phase;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    phase.next = NULL;
    phase.option = I2C_PHASE_SEND;
    phase.txstart = txBuffer;
    phase.length = (uint16_t)numBytes;

    job.firstPhase = &phase;
    job.slaveAddress = address;
    job.callback = callback;

    return I2C_submitJob(handle, &job);
}



/* Single buffer read operation from a device. */
LPCLIB_Result I2C_read (I2C_Handle handle,
                        I2C_Address address,
                        int numBytes,
                        void *rxBuffer,
                        LPCLIB_Callback callback)
{
    I2C_Job job;
    I2C_JobPhase phase;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    phase.next = NULL;
    phase.option = I2C_PHASE_RECEIVE;
    phase.rxstart = rxBuffer;
    phase.length = (uint16_t)numBytes;

    job.firstPhase = &phase;
    job.slaveAddress = address;
    job.callback = callback;

    return I2C_submitJob(handle, &job);
}


/* Single buffer write+read operation. */
LPCLIB_Result I2C_writeAndRead (I2C_Handle handle,
                                I2C_Address address,
                                int numTxBytes,
                                const void *txBuffer,
                                int numRxBytes,
                                void *rxBuffer,
                                LPCLIB_Callback callback)
{
    I2C_Job job;
    LPCLIB_Result result;
    I2C_JobPhase pPhase[2];


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    pPhase[0].next = &pPhase[1];
    pPhase[0].option = I2C_PHASE_SEND;
    pPhase[0].txstart = txBuffer;
    pPhase[0].length = (uint16_t)numTxBytes;
    pPhase[1].next = NULL;
    pPhase[1].option = I2C_PHASE_RECEIVE;
    pPhase[1].txstart = rxBuffer;
    pPhase[1].length = (uint16_t)numRxBytes;

    job.firstPhase = &pPhase[0];
    job.slaveAddress = address;
    job.callback = callback;

    result = I2C_submitJob(handle, &job);

    return result;
}



/* Submit a new I2C job to the driver. */
LPCLIB_Result I2C_submitJob (I2C_Handle handle, I2C_Job *pJob)
{
    LPC_I2C_Type * i2c;
    uint8_t address;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

#if LPCLIB_I2CEMU
    /* Check for bus emulation */
    if (handle->bus > I2C_NUM_BUSSES) {
        return I2CEMU_submitJob(handle, pJob);
    }
#endif

    i2c = i2c_ptr[handle->bus];

    /* Get exclusive access */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        /* Make sure to reset the sync flag */
        osSemaphoreWait(handle->syncSema, 0);

        handle->job = pJob;
        handle->errorStatus = LPCLIB_SUCCESS;
        handle->state = I2C_STATE_MASTER_SENT_SLAVEADDRESS;
        handle->phase = handle->job->firstPhase;
        address = handle->job->slaveAddress << 1;       /* SLA+W */
        if (handle->phase->option == I2C_PHASE_RECEIVE) {
            address |= 1;                               /* SLA+R */
        }
        i2c->MSTDAT = address;                          /* Send SLA+R/SLA+W */
        i2c->MSTCTL = I2C_MSTCTL_MSTSTART_Msk;          /* Start transmission in master mode */
        i2c->INTENSET = 0
                | I2C_INTENSET_MSTPENDINGEN_Msk
                ;

        /* In sync mode we wait for the end of transaction. */
        if (osSemaphoreWait(handle->syncSema, osWaitForever) > 0) {
            osMutexRelease(handle->accessMutex);

            return handle->errorStatus;
        }

        osMutexRelease(handle->accessMutex);

        //TODO: serious error! Restart I2C bus, re-init semaphores
        //TODO: when will semaacquire bring us here??
        return LPCLIB_ERROR;
    }

    return LPCLIB_BUSY;                                 /* Internal error. Couldn't get semaphore */
}



static const I2C_JobPhase dummyProbePhase = {
    .next = NULL,
    .option = I2C_PHASE_SEND,
};


/* Check for the existence of a slave with given address. */
LPCLIB_Result I2C_probe (I2C_Handle handle, uint8_t address)
{
    I2C_Job job = {
        .callback = NULL,                   //TODO: add support for non-rtos!
        .firstPhase = &dummyProbePhase,     /* No data phase, just SLA+W */
        .slaveAddress = address,
    };

    return I2C_submitJob(handle, &job);
}



/* Helper function to fill in phase data */
#include <string.h>
#if 0
I2C_JobPhase * I2C_makeTxPhase (I2C_JobPhase *pPhase, I2C_Direction direction, const void *txData, int length)
{
    (void) direction; //TODO
    (void) txData;
    (void) length;

    return pPhase;
}
#endif


/** Common (for all I2C blocks) interrupt handler.
 *
 *  \param[in] i2c_num Indicator that selects an I2C interface block
 */
static void I2C_commonIRQHandler (I2C_Name bus)
{
    LPC_I2C_Type * const i2c = i2c_ptr[bus];
    I2C_Handle handle = &i2cContext[bus];
    uint16_t current;

    uint32_t status = i2c->STAT;
    I2C_MasterState mstate = (I2C_MasterState)((status >> I2C_STAT_MSTSTATE_Pos) & I2C_STAT_MSTSTATE_Msk);

    switch (handle->state) {
        case I2C_STATE_MASTER_SENT_SLAVEADDRESS:
            if (mstate != I2C_MASTERSTATE_TX_READY) {           /* Slave did not respond */
                i2c->MSTCTL = I2C_MSTCTL_MSTSTOP_Msk;           /* Send STOP */
                handle->errorStatus = LPCLIB_NO_RESPONSE;
                osSemaphoreRelease(handle->syncSema);
                handle->state = I2C_STATE_IDLE;
                break;
            }

            /* Is there more data to send in this phase? */
            current = handle->transferred;                      /* Intermediate step to suppress IAR warning */
            if (current < handle->phase->length) {
                /* Yes. Send next data byte, and update counter. */
                i2c->MSTDAT = handle->phase->txstart[current];
                i2c->MSTCTL = I2C_MSTCTL_MSTCONTINUE_Msk;
                ++handle->transferred;
                break;
            }

            /**** NO BREAK. INTENTIONALLY FALL THROUGH TO NEXT PHASE HANDLING! ****/
//TEST TEST
i2c->MSTDAT = (0x4A<<1)+1;
i2c->MSTCTL = I2C_MSTCTL_MSTSTART_Msk;
while(1);

            break;

        default:
            handle->state = I2C_STATE_IDLE;
            break;
    }
}



/** I2C block interrupt entry (interrupt number \ref I2C0_IRQn).
 */
void I2C0_IRQHandler (void)
{
    I2C_commonIRQHandler(I2C0);
}

/** I2C block interrupt entry (interrupt number \ref I2C1_IRQn).
 */
void I2C1_IRQHandler (void)
{
    I2C_commonIRQHandler(I2C1);
}

/** I2C block interrupt entry (interrupt number \ref I2C2_IRQn).
 */
void I2C2_IRQHandler (void)
{
    I2C_commonIRQHandler(I2C2);
}


/** @} */

/** @} addtogroup I2C */

