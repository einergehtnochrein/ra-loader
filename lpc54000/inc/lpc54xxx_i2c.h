/* Copyright (c) 2014, DF9DQ
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

/** \file
 *  \brief I2C driver interface.
 *  This file defines all interface objects needed to use the I2C driver.
 */


#ifndef __LPC54XXX_I2C_H__
#define __LPC54XXX_I2C_H__

/** \defgroup I2C
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"

#include "lpclib_types.h"
#if LPCLIB_I2CEMU
#include "lpc54xxx_gpio.h"
#endif



/** \defgroup I2C_Public_Types I2C Types, enums, macros
 *  @{
 */


/** Enumerator for the I2C block.
 */
typedef enum I2C_Name {
    I2C0 = 0,           /**< First I2C interface block */
    I2C1,               /**< Second I2C interface block */
    I2C2,               /**< Third I2C interface block */
    I2C_NUM_BUSSES,     /* In order for this element to reflect the number of I2C busses,
                         * you mustn't assign an explicit value to any of the other elements
                         * of this enum! (except for the first element which may get assigned to 0).
                         */
#if LPCLIB_I2CEMU
    I2CEMU0,            /* I2C emulation (GPIO bit-banging) */
#endif
} I2C_Name;


/** Handle for an open I2C block, as obtained by \ref I2C_open. */
typedef struct I2C_Context *I2C_Handle;


/** Opcodes to specify the configuration command in a call to \ref I2C_ioctl. */
typedef enum I2C_Opcode {
    I2C_OPCODE_INVALID = 0,                 /**< (List terminator) */
    I2C_OPCODE_SET_BITRATE,                 /**< Config action: Set bus clock */
#if LPCLIB_I2C_SLAVE
    I2C_OPCODE_SET_SLAVE_ADDRESS,           /**< Config action: Set a slave address */
    I2C_OPCODE_SET_CALLBACK,                /**< Control action: Install callback handler */
#endif
#if LPCLIB_I2CEMU
    I2C_OPCODE_EMU_DEFINE_PINS,             /**< Define SCL/SDA GPIO pins */
#endif
} I2C_Opcode;


/** I2C device address */
typedef uint16_t I2C_Address;


struct I2C_ConfigAddress {
    uint8_t index;                          /**< Slave address storage location (0...3) */
    uint8_t mask;                           /**< Mask for slave address */
    I2C_Address address;                    /**< Slave address */
};


struct I2C_ConfigCallback {
    LPCLIB_Callback callback;               /**< New callback handler */
    LPCLIB_Callback *pOldCallback;          /**< Takes previously installed callback handler */
};


/** Config data: Specify GPIO pins for I2C emulation. */
#if LPCLIB_I2CEMU
struct I2C_EmuGpios {
    GPIO_Pin pinSCL;
    GPIO_Pin pinSDA;
};
#endif


/** Descriptor to specify the configuration in a call to \ref I2C_ioctl. */
typedef struct I2C_Config {
    I2C_Opcode opcode;                      /**< Config action opcode */

    union {
        uint32_t bitrate;                   /**< Config bus clock speed */
        struct I2C_ConfigAddress slave;     /**< Config slave address */
        struct I2C_ConfigCallback callback; /**< Callback in interrupt context */
#if LPCLIB_I2CEMU
        struct I2C_EmuGpios pins;           /**< Port pins for bit-banging */
#endif
    };
} I2C_Config;

/** Config list terminator. */
#define I2C_CONFIG_END \
    {.opcode = I2C_OPCODE_INVALID}


typedef enum I2C_Direction {
    I2C_DIRECTION_NONE = 0,                 /**< Direction: undefined */
    I2C_DIRECTION_TRANSMIT = 1,             /**< Direction: Transmit */
    I2C_DIRECTION_RECEIVE = 2,              /**< Direction: Receive */
} I2C_Direction;




typedef enum I2C_CallbackEvent {
    I2C_EVENT_SUCCESS = 0,                  /**< Transaction completed successfully */
    I2C_EVENT_NEED_SLAVE_JOB_RX,            /**< Application must provide a job descriptor (receive) */
    I2C_EVENT_NEED_SLAVE_JOB_TX,            /**< Application must provide a job descriptor (transmit) */
    I2C_EVENT_NO_SUCH_DEVICE,               /**< Addressed slave doesn't respond */
    I2C_EVENT_NOT_ACCEPTED,                 /**< Slave rejected the transmission */
} I2C_CallbackEvent;


typedef enum I2C_JobOption {
    I2C_PHASE_SEND,
    I2C_PHASE_RECEIVE,
} I2C_JobOption;

typedef struct I2C_JobPhase {
    struct I2C_JobPhase *next;              /**< Pointer to next transaction phase (or NULL) */
    I2C_JobOption option;                   /**< ... */
    union {
        uint8_t *rxstart;                   /**< Start of RX buffer */
        const uint8_t *txstart;             /**< Start of TX buffer */
    };
    uint16_t length;                        /**< Transfer length (in bytes) */
    LPCLIB_Switch askBefore;                /**< Ask (callback) before using this phase */
} I2C_JobPhase;


/** Descriptor for an I2C transaction. */
typedef struct I2C_Job {
    LPCLIB_Callback callback;               /**< Event handler */
    const I2C_JobPhase *firstPhase;         /**< Points to the first in a list of
                                             *   phase descriptors.
                                             */
    I2C_Address slaveAddress;               /**< Slave address */
} I2C_Job;



/** @} I2C Types, enums, macros */



/** \defgroup I2C_Public_Functions I2C API Functions
 *  @{
 */


/** Open an I2C bus for use.
 *
 *  Enable the peripheral clock to the indicated I2C block, then enable
 *  the block. Set the default bit clock and slave address (the latter only
 *  if configured for slave mode), and enable the interrupts.
 *  The block is ready for use after that.
 *
 *  \param[in] bus Indicator that selects an I2C interface block
 *  \param[out] pHandle Handle to be used in future API calls to the I2C module.
 *  \retval LPCLIB_SUCCESS Success. \ref handle contains a valid handle.
 *  \retval LPCLIB_BUSY Failure (interface already open). \ref pHandle does not
 *  contain a valid handle in this case.
 */
LPCLIB_Result I2C_open (I2C_Name bus, I2C_Handle *pHandle);


/** Close an I2C bus.
 *
 *  Disables interrupts for the I2C block, and cuts the clock to the block.
 *  Make sure to call this only if there is no ongoing transaction, since no
 *  check is made to prevent this.
 *
 *  \param[in] pHandle I2C bus handle as obtained by \ref I2C_open.
 */
void I2C_close (I2C_Handle *pHandle);


/** Configure the I2C block.
 *
 *  Pass a configuration command to the I2C block. Configuration options
 *  include setting the bus clock frequency or specifying a slave address
 *  and its associated address mask.
 *
 *  \param[in] handle I2C bus handle as obtained by \ref I2C_open.
 *  \param[in] pConfig Pointer to a configuration descriptor of type
 *             \ref I2C_Config.
 */
void I2C_ioctl (I2C_Handle handle, const I2C_Config *pConfig);


/** Single buffer write operation to a device.
 *
 *  \param[in] handle I2C bus handle.
 *  \param[in] address Device slave address
 *  \param[in] numBytes Number of bytes to be written.
 *  \param[in] txBuffer Pointer to TX data
 *  \param[in] callback Callback handler (NULL = blocking)
 *
 *  \retval LPCLIB_SUCCESS Success.
 */
LPCLIB_Result I2C_write (I2C_Handle handle,
                         I2C_Address address,
                         int numBytes,
                         const void *txBuffer,
                         LPCLIB_Callback callback);


/** Single buffer read operation from a device.
 *
 *  \param[in] handle I2C bus handle.
 *  \param[in] address Device slave address
 *  \param[in] numBytes Number of bytes to be received.
 *  \param[in] rxBuffer Pointer to RX data
 *  \param[in] callback Callback handler (NULL = blocking)
 *
 *  \retval LPCLIB_SUCCESS Success.
 */
LPCLIB_Result I2C_read (I2C_Handle handle,
                        I2C_Address address,
                        int numBytes,
                        void *rxBuffer,
                        LPCLIB_Callback callback);


/** Single buffer write+read operation.
 *
 *  \param[in] handle I2C bus handle.
 *  \param[in] address Device slave address
 *  \param[in] numTxBytes Number of bytes to be sent.
 *  \param[in] txBuffer Pointer to TX data
 *  \param[in] numRxBytes Number of bytes to be received.
 *  \param[in] rxBuffer Pointer to RX data
 *  \param[in] callback Callback handler (NULL = blocking)
 *
 *  \retval LPCLIB_SUCCESS Success.
 */
LPCLIB_Result I2C_writeAndRead (I2C_Handle handle,
                                I2C_Address address,
                                int numTxBytes,
                                const void *txBuffer,
                                int numRxBytes,
                                void *rxBuffer,
                                LPCLIB_Callback callback);


/** Submit a new I2C job to the driver.
 *
 *  \param[in] handle I2C bus handle as obtained by \ref I2C_open.
 *  \param[in] pJob Pointer to a job descriptor that describes the details
 *      of the transaction.
 *
 *  \retval LPCLIB_SUCCESS Success. Job will be executed, and the callback function
 *      called once the transaction ends.
 *  \retval LPCLIB_BUSY Failure. Job NOT submitted.
 */
LPCLIB_Result I2C_submitJob (I2C_Handle bus, I2C_Job *pJob);


/** Check for the existence of a slave with given address.
 *
 *  \param[in] handle Bus handle
 *  \param[in] address Slave address to be probed.
 *  \retval LPCLIB_SUCCESS Device exists
 *  \retval LPCLIB_NO_RESPONSE Device doesn't exist. */
LPCLIB_Result I2C_probe (I2C_Handle bus, uint8_t address);


/** @} I2C API Functions */

/** @} I2C */

#endif /* #ifndef __LPC54XXX_I2C_H__ */

