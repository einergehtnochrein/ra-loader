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

/** \defgroup SPI
 *  \ingroup API
 *  @{
 */

#ifndef __LPC54XXX_SPI_H__
#define __LPC54XXX_SPI_H__

#include "lpc54xxx_libconfig.h"
#include "lpclib_types.h"



/** \defgroup SPI_Public_Types SPI Types, enums, macros
 *  @{
 */

typedef enum SPI_Name {
    SPI0 = 0,
    SPI1,
    SPI_NUM_BUSSES      /* In order for this element to reflect the number of SPI busses,
                         * you mustn't assign an explicit value to any of the other elements
                         * of this enum! (except for the first element which may get assigned to 0).
                         */
} SPI_Name;

/** Handle for an open SPI block */
typedef struct SPI_Context *SPI_Handle;

typedef enum SPI_Opcode {
    SPI_OPCODE_INVALID = 0,                 /**< (List terminator) */
    SPI_OPCODE_SET_MODE,                    /**< Config action: Set mode (M/S) */
    SPI_OPCODE_SET_FORMAT,                  /**< Config action: Set format */
    SPI_OPCODE_SET_BITRATE,                 /**< Config action: Set bit rate */
} SPI_Opcode;

typedef enum SPI_Mode {
    SPI_MODE_MASTER = 1,                    /**< Master mode */
    SPI_MODE_SLAVE =  0,                    /**< Slave mode */
} SPI_Mode;

typedef enum SPI_ClockFormat {
    SPI_CLOCK_CPHA0_CPOL0 = (0 << 1) | (0 << 0),    /**< CPHA = 0, CPOL = 0 */
    SPI_CLOCK_CPHA0_CPOL1 = (1 << 1) | (0 << 0),    /**< CPHA = 0, CPOL = 1 */
    SPI_CLOCK_CPHA1_CPOL0 = (0 << 1) | (1 << 0),    /**< CPHA = 1, CPOL = 0 */
    SPI_CLOCK_CPHA1_CPOL1 = (1 << 1) | (1 << 0),    /**< CPHA = 1, CPOL = 1 */
    SPI_CLOCK_MODE0 = SPI_CLOCK_CPHA0_CPOL0,    /**< alias */
    SPI_CLOCK_MODE1 = SPI_CLOCK_CPHA1_CPOL0,    /**< alias */
    SPI_CLOCK_MODE2 = SPI_CLOCK_CPHA0_CPOL1,    /**< alias */
    SPI_CLOCK_MODE3 = SPI_CLOCK_CPHA1_CPOL1,    /**< alias */
} SPI_ClockFormat;

typedef enum SPI_FrameFormat {
    SPI_FRAMEFORMAT_SPI = (0 << 4),         /**< Frame format: SPI */
} SPI_FrameFormat;

typedef enum SPI_Bits {
    SPI_BITS_4  = 3,                        /**< 4 bits/frame */
    SPI_BITS_5  = 4,                        /**< 5 bits/frame */
    SPI_BITS_6  = 5,                        /**< 6 bits/frame */
    SPI_BITS_7  = 6,                        /**< 7 bits/frame */
    SPI_BITS_8  = 7,                        /**< 8 bits/frame */
    SPI_BITS_9  = 8,                        /**< 9 bits/frame */
    SPI_BITS_10 = 9,                        /**< 10 bits/frame */
    SPI_BITS_11 = 10,                       /**< 11 bits/frame */
    SPI_BITS_12 = 11,                       /**< 12 bits/frame */
    SPI_BITS_13 = 12,                       /**< 13 bits/frame */
    SPI_BITS_14 = 13,                       /**< 14 bits/frame */
    SPI_BITS_15 = 14,                       /**< 15 bits/frame */
    SPI_BITS_16 = 15,                       /**< 16 bits/frame */
} SPI_Bits;

struct SPI_ConfigFormat {
    SPI_Bits bits;                          /**< Format config: bits/frame */
    SPI_ClockFormat clockFormat;            /**< Format config: clock phase/polarity */
    SPI_FrameFormat frameFormat;            /**< Format config: SPI/TI/Microwire */
};

typedef struct SPI_Config {
    SPI_Opcode opcode;                      /**< Config action opcode */

    union {
        SPI_Mode mode;                      /**< Config mode */
        struct SPI_ConfigFormat format;     /**< Config format */
        uint32_t bitrate;                   /**< Config clock speed */
    };
} SPI_Config;

/** Config list terminator. */
#define SPI_CONFIG_END \
    {.opcode = SPI_OPCODE_INVALID}


typedef enum SPI_CallbackEvent {
    SPI_EVENT_PHASE_COMPLETE = 0,           /**< Phase completed successfully */
    SPI_EVENT_JOB_COMPLETE,                 /**< Job completed successfully */
    SPI_EVENT_ASSERT_CHIPSELECT,            /**< Request to assert chip select line */
    SPI_EVENT_DEASSERT_CHIPSELECT,          /**< Request to deassert chip select line */
    SPI_EVENT_INTERRUPT,                    /**< "Interrupt" RX event */
    SPI_EVENT_CANCELLED,                    /**< Job cancelled */
} SPI_CallbackEvent;


/** Method to select a device (SSEL demuxing). */
typedef struct SPI_DeviceSelect {
    LPCLIB_Callback callback;               /**< Callback for device select */
    uint8_t channel;                        /**< Channel number (arbitrary ID to identify device) */
} SPI_DeviceSelect;


typedef struct SPI_JobPhase {
    struct SPI_JobPhase *next;              /**< Pointer to following phase (or NULL) */
    union {
        const uint8_t *txstart8;            /**< Pointer to TX block start (8 bits/frame) */
        const uint16_t *txstart16;          /**< Pointer to TX block start (16 bits/frame) */
    };
    union {
        uint8_t *rxstart8;                  /**< Pointer to RX block start (8 bits/frame) */
        uint16_t *rxstart16;                /**< Pointer to RX block start (16 bits/frame) */
    };
    uint16_t length;                        /**< Number of frames in this phase */
    uint16_t idlePattern;                   /**< Idle pattern to be sent if no TX data available */
    LPCLIB_Switch interrupt;                /**< Trigger interrupt (= callback) at end of phase */
    LPCLIB_Switch txBarrier;                /**< Do not preload TX FIFO beyond this point */
} SPI_JobPhase;


typedef struct SPI_Job {
    SPI_JobPhase *firstPhase;               /**< Descriptor of first job phase */
    const SPI_DeviceSelect *pDeviceSelect;  /**< Device callback */
    const SPI_Config *pConfig;              /**< Interface parameters */
    void *extraParameter;                   /**< Extra parameter to be passed on to device select */
    LPCLIB_Callback callback;               /**< Callback for data transfer */
    _Bool noWaitComplete;                   /**< Do not wait for completion */
} SPI_Job;


/** @} SPI Types, enums, macros */



/** \defgroup SPI_Public_Functions SPI API Functions
 *  @{
 */

/** Open an SPI bus.
 *
 *  Prepare the selected bus. Obtain a handle that must be used in calls to all other
 *  SPI module functions.
 *
 *  \param[in] sspNum Indicator that selects a bus.
 *  \param[out] pHandle Receives the handle
 *  \retval LPCLIB_SUCCESS Ok. Valid handle returned.
 *  \retval LPCLIB_BUSY Error. No valid handle returned. (Bus already open)
 */
LPCLIB_Result SPI_open (SPI_Name sspNum, SPI_Handle *pHandle);


/** Close an SPI bus.
 *
 *  Disable interrupts and clocks (power saving). Make sure there are no more
 *  active transactions.
 *
 *  \param pBus Pointer to handle of the SPI bus to be closed.
 */
void SPI_close (SPI_Handle *pBus);


/** Set options of the SPI block.
 *
 *  \param[in] bus Handle of the SPI bus
 *  \param[in] config Configuration descriptor
 */
void SPI_ioctl (SPI_Handle bus, const SPI_Config *pConfig);


/** Submit a job to the SPI driver.
 *
 *  \param[in] bus Handle for SPI bus
 *  \param[in] pJob Job descriptor
 *  \return ....
 */
LPCLIB_Result SPI_submitJob (SPI_Handle bus, const SPI_Job * pJob);


/** Enable the SPI.
 *
 *  The SPI can be initialized from task level with a job, but put on hold until
 *  this function is called to actually start it.
 *  Only valid in master mode.
 *  Can also be called from an interrupt context.
 *
 *  \param[in] bus Handle for SPI bus
 *  \retval LPCLIB_SUCCESS
 *  \retval LPCLIB_NOT_PREPARED Not on hold in master mode
 *  \retval LPCLIB_ILLEGAL_PARAMETER Not a valid handle
 */
LPCLIB_Result SPI_run (SPI_Handle handle);


/** Cancel an active SPI job.
 *
 *  \param[in] bus Handle for SPI bus
 *  \retval LPCLIB_SUCCESS
 *  \retval LPCLIB_ILLEGAL_PARAMETER Not a valid handle
 */
LPCLIB_Result SPI_cancel (SPI_Handle handle);


/** Fill data into an SPI_JobPhase descriptor.
 *
 *  \param[in] phase Pointer to existing phase descriptor.
 *  \param[in] pTx Pointer to TX data (or NULL to send idle frames).
 *  \param[in] pRx Pointer to RX buffer (or NULL for no reception).
 *  \param[in] length Number of frames in this phase.
 *  \return Pointer to the phase descriptor (same as \a phase).
 */
SPI_JobPhase *SPI_makePhase (SPI_JobPhase *pPhase, const void *pTx, uint8_t *pRx, uint16_t length);

/** @} SPI API Functions */

#endif /* #ifndef __LPC54XXX_SPI_H__ */

/** @} */



