/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _FSL_I2C_H_
#define _FSL_I2C_H_

#include <stddef.h>
#include "fsl_device_registers.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define I2C_CFG_MASK 0x1f

/*!
 * @addtogroup i2c_driver
 * @{
 */

/*! @file */

/*! @name Driver version */
/*@{*/
/*! @brief I2C driver version 2.0.2. */
#define FSL_I2C_DRIVER_VERSION (MAKE_VERSION(2, 0, 2))
/*@}*/

/*! @brief Timeout times for waiting flag. */
#ifndef I2C_WAIT_TIMEOUT
#define I2C_WAIT_TIMEOUT 0U /* Define to zero means keep waiting until the flag is assert/deassert. */
#endif

/* definitions for MSTCODE bits in I2C Status register STAT */
#define I2C_STAT_MSTCODE_IDLE (0)    /*!< Master Idle State Code */
#define I2C_STAT_MSTCODE_RXREADY (1) /*!< Master Receive Ready State Code */
#define I2C_STAT_MSTCODE_TXREADY (2) /*!< Master Transmit Ready State Code */
#define I2C_STAT_MSTCODE_NACKADR (3) /*!< Master NACK by slave on address State Code */
#define I2C_STAT_MSTCODE_NACKDAT (4) /*!< Master NACK by slave on data State Code */

/* definitions for SLVSTATE bits in I2C Status register STAT */
#define I2C_STAT_SLVST_ADDR (0)
#define I2C_STAT_SLVST_RX (1)
#define I2C_STAT_SLVST_TX (2)

/*! @brief I2C status return codes. */
enum _i2c_status
{
    kStatus_I2C_Busy = MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 0), /*!< The master is already performing a transfer. */
    kStatus_I2C_Idle = MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 1), /*!< The slave driver is idle. */
    kStatus_I2C_Nak =
        MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 2), /*!< The slave device sent a NAK in response to a byte. */
    kStatus_I2C_InvalidParameter =
        MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 3), /*!< Unable to proceed due to invalid parameter. */
    kStatus_I2C_BitError = MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 4), /*!< Transferred bit was not seen on the bus. */
    kStatus_I2C_ArbitrationLost = MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 5), /*!< Arbitration lost error. */
    kStatus_I2C_NoTransferInProgress =
        MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 6), /*!< Attempt to abort a transfer when one is not in progress. */
    kStatus_I2C_DmaRequestFail = MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 7), /*!< DMA request failed. */
    kStatus_I2C_StartStopError = MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 8),
    kStatus_I2C_UnexpectedState = MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 9),
    kStatus_I2C_Timeout = MAKE_STATUS(kStatusGroup_FLEXCOMM_I2C, 10), /*!< Timeout poling status flags. */
};

/*! @} */

/*!
 * @addtogroup i2c_master_driver
 * @{
 */

/*!
 * @brief I2C master peripheral flags.
 *
 * @note These enums are meant to be OR'd together to form a bit mask.
 */
enum _i2c_master_flags
{
    kI2C_MasterPendingFlag = I2C_STAT_MSTPENDING_MASK, /*!< The I2C module is waiting for software interaction. */
    kI2C_MasterArbitrationLostFlag =
        I2C_STAT_MSTARBLOSS_MASK, /*!< The arbitration of the bus was lost. There was collision on the bus */
    kI2C_MasterStartStopErrorFlag =
        I2C_STAT_MSTSTSTPERR_MASK /*!< There was an error during start or stop phase of the transaction. */
};

/*! @brief Direction of master and slave transfers. */
typedef enum _i2c_direction
{
    kI2C_Write = 0U, /*!< Master transmit. */
    kI2C_Read = 1U   /*!< Master receive. */
} i2c_direction_t;

/*!
 * @brief Structure with settings to initialize the I2C master module.
 *
 * This structure holds configuration settings for the I2C peripheral. To initialize this
 * structure to reasonable defaults, call the I2C_MasterGetDefaultConfig() function and
 * pass a pointer to your configuration structure instance.
 *
 * The configuration structure can be made constant so it resides in flash.
 */
typedef struct _i2c_master_config
{
    bool enableMaster;     /*!< Whether to enable master mode. */
    uint32_t baudRate_Bps; /*!< Desired baud rate in bits per second. */
    bool enableTimeout;    /*!< Enable internal timeout function. */
} i2c_master_config_t;

/* Forward declaration of the transfer descriptor and handle typedefs. */
/*! @brief I2C master transfer typedef */
typedef struct _i2c_master_transfer i2c_master_transfer_t;

/*! @brief I2C master handle typedef */
typedef struct _i2c_master_handle i2c_master_handle_t;

/*!
 * @brief Master completion callback function pointer type.
 *
 * This callback is used only for the non-blocking master transfer API. Specify the callback you wish to use
 * in the call to I2C_MasterTransferCreateHandle().
 *
 * @param base The I2C peripheral base address.
 * @param completionStatus Either kStatus_Success or an error code describing how the transfer completed.
 * @param userData Arbitrary pointer-sized value passed from the application.
 */
typedef void (*i2c_master_transfer_callback_t)(I2C_Type *base,
                                               i2c_master_handle_t *handle,
                                               status_t completionStatus,
                                               void *userData);

/*!
 * @brief Transfer option flags.
 *
 * @note These enumerations are intended to be OR'd together to form a bit mask of options for
 * the #_i2c_master_transfer::flags field.
 */
enum _i2c_master_transfer_flags
{
    kI2C_TransferDefaultFlag = 0x00U,       /*!< Transfer starts with a start signal, stops with a stop signal. */
    kI2C_TransferNoStartFlag = 0x01U,       /*!< Don't send a start condition, address, and sub address */
    kI2C_TransferRepeatedStartFlag = 0x02U, /*!< Send a repeated start condition */
    kI2C_TransferNoStopFlag = 0x04U,        /*!< Don't send a stop condition. */
};

/*! @brief States for the state machine used by transactional APIs. */
enum _i2c_transfer_states
{
    kIdleState = 0,
    kTransmitSubaddrState,
    kTransmitDataState,
    kReceiveDataState,
    kReceiveLastDataState,
    kStartState,
    kStopState,
    kWaitForCompletionState
};

/*!
 * @brief Non-blocking transfer descriptor structure.
 *
 * This structure is used to pass transaction parameters to the I2C_MasterTransferNonBlocking() API.
 */
struct _i2c_master_transfer
{
    uint32_t flags; /*!< Bit mask of options for the transfer. See enumeration #_i2c_master_transfer_flags for available
                       options. Set to 0 or #kI2C_TransferDefaultFlag for normal transfers. */
    uint16_t slaveAddress;     /*!< The 7-bit slave address. */
    i2c_direction_t direction; /*!< Either #kI2C_Read or #kI2C_Write. */
    uint32_t subaddress;       /*!< Sub address. Transferred MSB first. */
    size_t subaddressSize;     /*!< Length of sub address to send in bytes. Maximum size is 4 bytes. */
    void *data;                /*!< Pointer to data to transfer. */
    size_t dataSize;           /*!< Number of bytes to transfer. */
};

/*!
 * @brief Driver handle for master non-blocking APIs.
 * @note The contents of this structure are private and subject to change.
 */
struct _i2c_master_handle
{
    uint8_t state;           /*!< Transfer state machine current state. */
    uint32_t transferCount;  /*!< Indicates progress of the transfer */
    uint32_t remainingBytes; /*!< Remaining byte count in current state. */
    uint8_t *buf;            /*!< Buffer pointer for current state. */
    uint32_t remainingSubaddr;
    uint8_t subaddrBuf[4];
    i2c_master_transfer_t transfer;                    /*!< Copy of the current transfer info. */
    i2c_master_transfer_callback_t completionCallback; /*!< Callback function pointer. */
    void *userData;                                    /*!< Application data passed to callback. */
};

/*! @} */

/*!
 * @addtogroup i2c_slave_driver
 * @{
 */

/*!
* @brief I2C slave peripheral flags.
*
* @note These enums are meant to be OR'd together to form a bit mask.
*/
enum _i2c_slave_flags
{
    kI2C_SlavePendingFlag = I2C_STAT_SLVPENDING_MASK, /*!< The I2C module is waiting for software interaction. */
    kI2C_SlaveNotStretching =
        I2C_STAT_SLVNOTSTR_MASK, /*!< Indicates whether the slave is currently stretching clock (0 = yes, 1 = no). */
    kI2C_SlaveSelected = I2C_STAT_SLVSEL_MASK, /*!< Indicates whether the slave is selected by an address match. */
    kI2C_SaveDeselected =
        I2C_STAT_SLVDESEL_MASK /*!< Indicates that slave was previously deselected (deselect event took place, w1c). */
};

/*! @brief I2C slave address register. */
typedef enum _i2c_slave_address_register
{
    kI2C_SlaveAddressRegister0 = 0U, /*!< Slave Address 0 register. */
    kI2C_SlaveAddressRegister1 = 1U, /*!< Slave Address 1 register. */
    kI2C_SlaveAddressRegister2 = 2U, /*!< Slave Address 2 register. */
    kI2C_SlaveAddressRegister3 = 3U, /*!< Slave Address 3 register. */
} i2c_slave_address_register_t;

/*! @brief Data structure with 7-bit Slave address and Slave address disable. */
typedef struct _i2c_slave_address
{
    uint8_t address;     /*!< 7-bit Slave address SLVADR. */
    bool addressDisable; /*!< Slave address disable SADISABLE. */
} i2c_slave_address_t;

/*! @brief I2C slave address match options. */
typedef enum _i2c_slave_address_qual_mode
{
    kI2C_QualModeMask = 0U, /*!< The SLVQUAL0 field (qualAddress) is used as a logical mask for matching address0. */
    kI2C_QualModeExtend =
        1U, /*!< The SLVQUAL0 (qualAddress) field is used to extend address 0 matching in a range of addresses. */
} i2c_slave_address_qual_mode_t;

/*! @brief I2C slave bus speed options. */
typedef enum _i2c_slave_bus_speed
{
    kI2C_SlaveStandardMode = 0U,
    kI2C_SlaveFastMode = 1U,
    kI2C_SlaveFastModePlus = 2U,
    kI2C_SlaveHsMode = 3U,
} i2c_slave_bus_speed_t;

/*!
 * @brief Structure with settings to initialize the I2C slave module.
 *
 * This structure holds configuration settings for the I2C slave peripheral. To initialize this
 * structure to reasonable defaults, call the I2C_SlaveGetDefaultConfig() function and
 * pass a pointer to your configuration structure instance.
 *
 * The configuration structure can be made constant so it resides in flash.
 */
typedef struct _i2c_slave_config
{
    i2c_slave_address_t address0;           /*!< Slave's 7-bit address and disable. */
    i2c_slave_address_t address1;           /*!< Alternate slave 7-bit address and disable. */
    i2c_slave_address_t address2;           /*!< Alternate slave 7-bit address and disable. */
    i2c_slave_address_t address3;           /*!< Alternate slave 7-bit address and disable. */
    i2c_slave_address_qual_mode_t qualMode; /*!< Qualify mode for slave address 0. */
    uint8_t qualAddress;                    /*!< Slave address qualifier for address 0. */
    i2c_slave_bus_speed_t
        busSpeed; /*!< Slave bus speed mode. If the slave function stretches SCL to allow for software response, it must
                       provide sufficient data setup time to the master before releasing the stretched clock.
                       This is accomplished by inserting one clock time of CLKDIV at that point.
                       The #busSpeed value is used to configure CLKDIV
                       such that one clock time is greater than the tSU;DAT value noted
                       in the I2C bus specification for the I2C mode that is being used.
                       If the #busSpeed mode is unknown at compile time, use the longest data setup time
                       kI2C_SlaveStandardMode (250 ns) */
    bool enableSlave; /*!< Enable slave mode. */
} i2c_slave_config_t;

/*!
 * @brief Set of events sent to the callback for non blocking slave transfers.
 *
 * These event enumerations are used for two related purposes. First, a bit mask created by OR'ing together
 * events is passed to I2C_SlaveTransferNonBlocking() in order to specify which events to enable.
 * Then, when the slave callback is invoked, it is passed the current event through its @a transfer
 * parameter.
 *
 * @note These enumerations are meant to be OR'd together to form a bit mask of events.
 */
typedef enum _i2c_slave_transfer_event
{
    kI2C_SlaveAddressMatchEvent = 0x01U, /*!< Received the slave address after a start or repeated start. */
    kI2C_SlaveTransmitEvent = 0x02U,     /*!< Callback is requested to provide data to transmit
                                                (slave-transmitter role). */
    kI2C_SlaveReceiveEvent = 0x04U,      /*!< Callback is requested to provide a buffer in which to place received
                                                 data (slave-receiver role). */
    kI2C_SlaveCompletionEvent = 0x20U,   /*!< All data in the active transfer have been consumed. */
    kI2C_SlaveDeselectedEvent =
        0x40U, /*!< The slave function has become deselected (SLVSEL flag changing from 1 to 0. */

    /*! Bit mask of all available events. */
    kI2C_SlaveAllEvents = kI2C_SlaveAddressMatchEvent | kI2C_SlaveTransmitEvent | kI2C_SlaveReceiveEvent |
                          kI2C_SlaveCompletionEvent | kI2C_SlaveDeselectedEvent,
} i2c_slave_transfer_event_t;

/*! @brief I2C slave handle typedef. */
typedef struct _i2c_slave_handle i2c_slave_handle_t;

/*! @brief I2C slave transfer structure */
typedef struct _i2c_slave_transfer
{
    i2c_slave_handle_t *handle;       /*!< Pointer to handle that contains this transfer. */
    i2c_slave_transfer_event_t event; /*!< Reason the callback is being invoked. */
    uint8_t receivedAddress;          /*!< Matching address send by master. 7-bits plus R/nW bit0 */
    uint32_t eventMask;               /*!< Mask of enabled events. */
    uint8_t *rxData;                  /*!< Transfer buffer for receive data */
    const uint8_t *txData;            /*!< Transfer buffer for transmit data */
    size_t txSize;                    /*!< Transfer size */
    size_t rxSize;                    /*!< Transfer size */
    size_t transferredCount;          /*!< Number of bytes transferred during this transfer. */
    status_t completionStatus;        /*!< Success or error code describing how the transfer completed. Only applies for
                                         #kI2C_SlaveCompletionEvent. */
} i2c_slave_transfer_t;

/*!
 * @brief Slave event callback function pointer type.
 *
 * This callback is used only for the slave non-blocking transfer API. To install a callback,
 * use the I2C_SlaveSetCallback() function after you have created a handle.
 *
 * @param base Base address for the I2C instance on which the event occurred.
 * @param transfer Pointer to transfer descriptor containing values passed to and/or from the callback.
 * @param userData Arbitrary pointer-sized value passed from the application.
 */
typedef void (*i2c_slave_transfer_callback_t)(I2C_Type *base, volatile i2c_slave_transfer_t *transfer, void *userData);

/*!
 * @brief I2C slave software finite state machine states.
 */
typedef enum _i2c_slave_fsm
{
    kI2C_SlaveFsmAddressMatch = 0u,
    kI2C_SlaveFsmReceive = 2u,
    kI2C_SlaveFsmTransmit = 3u,
} i2c_slave_fsm_t;

/*!
 * @brief I2C slave handle structure.
 * @note The contents of this structure are private and subject to change.
 */
struct _i2c_slave_handle
{
    volatile i2c_slave_transfer_t transfer; /*!< I2C slave transfer. */
    volatile bool isBusy;                   /*!< Whether transfer is busy. */
    volatile i2c_slave_fsm_t slaveFsm;      /*!< slave transfer state machine. */
    i2c_slave_transfer_callback_t callback; /*!< Callback function called at transfer event. */
    void *userData;                         /*!< Callback parameter passed to callback. */
};

/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @addtogroup i2c_master_driver
 * @{
 */

/*! @name Initialization and deinitialization */
/*@{*/

/*!
 * @brief Provides a default configuration for the I2C master peripheral.
 *
 * This function provides the following default configuration for the I2C master peripheral:
 * @code
 *  masterConfig->enableMaster            = true;
 *  masterConfig->baudRate_Bps            = 100000U;
 *  masterConfig->enableTimeout           = false;
 * @endcode
 *
 * After calling this function, you can override any settings in order to customize the configuration,
 * prior to initializing the master driver with I2C_MasterInit().
 *
 * @param[out] masterConfig User provided configuration structure for default values. Refer to #i2c_master_config_t.
 */
void I2C_MasterGetDefaultConfig(i2c_master_config_t *masterConfig);

/*!
 * @brief Initializes the I2C master peripheral.
 *
 * This function enables the peripheral clock and initializes the I2C master peripheral as described by the user
 * provided configuration. A software reset is performed prior to configuration.
 *
 * @param base The I2C peripheral base address.
 * @param masterConfig User provided peripheral configuration. Use I2C_MasterGetDefaultConfig() to get a set of
 * defaults
 *      that you can override.
 * @param srcClock_Hz Frequency in Hertz of the I2C functional clock. Used to calculate the baud rate divisors,
 *      filter widths, and timeout periods.
 */
void I2C_MasterInit(I2C_Type *base, const i2c_master_config_t *masterConfig, uint32_t srcClock_Hz);

/*!
* @brief Deinitializes the I2C master peripheral.
*
 * This function disables the I2C master peripheral and gates the clock. It also performs a software
 * reset to restore the peripheral to reset conditions.
 *
 * @param base The I2C peripheral base address.
 */
void I2C_MasterDeinit(I2C_Type *base);

/*!
 * @brief Performs a software reset.
 *
 * Restores the I2C master peripheral to reset conditions.
 *
 * @param base The I2C peripheral base address.
 */
static inline void I2C_MasterReset(I2C_Type *base)
{
}

/*!
 * @brief Enables or disables the I2C module as master.
 *
 * @param base The I2C peripheral base address.
 * @param enable Pass true to enable or false to disable the specified I2C as master.
 */
static inline void I2C_MasterEnable(I2C_Type *base, bool enable)
{
    if (enable)
    {
        base->CFG = (base->CFG & I2C_CFG_MASK) | I2C_CFG_MSTEN_MASK;
    }
    else
    {
        base->CFG = (base->CFG & I2C_CFG_MASK) & ~I2C_CFG_MSTEN_MASK;
    }
}

/*@}*/

/*! @name Status */
/*@{*/

/*!
 * @brief Gets the I2C status flags.
 *
 * A bit mask with the state of all I2C status flags is returned. For each flag, the corresponding bit
 * in the return value is set if the flag is asserted.
 *
 * @param base The I2C peripheral base address.
 * @return State of the status flags:
 *         - 1: related status flag is set.
 *         - 0: related status flag is not set.
 * @see _i2c_master_flags
 */
static inline uint32_t I2C_GetStatusFlags(I2C_Type *base)
{
    return base->STAT;
}

/*!
 * @brief Clears the I2C master status flag state.
 *
 * The following status register flags can be cleared:
 * - #kI2C_MasterArbitrationLostFlag
 * - #kI2C_MasterStartStopErrorFlag
 *
 * Attempts to clear other flags has no effect.
 *
 * @param base The I2C peripheral base address.
 * @param statusMask A bitmask of status flags that are to be cleared. The mask is composed of
 *  #_i2c_master_flags enumerators OR'd together. You may pass the result of a previous call to
 *  I2C_GetStatusFlags().
 * @see _i2c_master_flags.
 */
static inline void I2C_MasterClearStatusFlags(I2C_Type *base, uint32_t statusMask)
{
    /* Allow clearing just master status flags */
    base->STAT = statusMask & (I2C_STAT_MSTARBLOSS_MASK | I2C_STAT_MSTSTSTPERR_MASK);
}

/*@}*/

/*! @name Interrupts */
/*@{*/

/*!
 * @brief Enables the I2C master interrupt requests.
 *
 * @param base The I2C peripheral base address.
 * @param interruptMask Bit mask of interrupts to enable. See #_i2c_master_flags for the set
 *      of constants that should be OR'd together to form the bit mask.
 */
static inline void I2C_EnableInterrupts(I2C_Type *base, uint32_t interruptMask)
{
    base->INTENSET = interruptMask;
}

/*!
 * @brief Disables the I2C master interrupt requests.
 *
 * @param base The I2C peripheral base address.
 * @param interruptMask Bit mask of interrupts to disable. See #_i2c_master_flags for the set
 *      of constants that should be OR'd together to form the bit mask.
 */
static inline void I2C_DisableInterrupts(I2C_Type *base, uint32_t interruptMask)
{
    base->INTENCLR = interruptMask;
}

/*!
 * @brief Returns the set of currently enabled I2C master interrupt requests.
 *
 * @param base The I2C peripheral base address.
 * @return A bitmask composed of #_i2c_master_flags enumerators OR'd together to indicate the
 *      set of enabled interrupts.
 */
static inline uint32_t I2C_GetEnabledInterrupts(I2C_Type *base)
{
    return base->INTSTAT;
}

/*@}*/

/*! @name Bus operations */
/*@{*/

/*!
 * @brief Sets the I2C bus frequency for master transactions.
 *
 * The I2C master is automatically disabled and re-enabled as necessary to configure the baud
 * rate. Do not call this function during a transfer, or the transfer is aborted.
 *
 * @param base The I2C peripheral base address.
 * @param srcClock_Hz I2C functional clock frequency in Hertz.
 * @param baudRate_Bps Requested bus frequency in bits per second.
 */
void I2C_MasterSetBaudRate(I2C_Type *base, uint32_t baudRate_Bps, uint32_t srcClock_Hz);

/*!
 * @brief Returns whether the bus is idle.
 *
 * Requires the master mode to be enabled.
 *
 * @param base The I2C peripheral base address.
 * @retval true Bus is busy.
 * @retval false Bus is idle.
 */
static inline bool I2C_MasterGetBusIdleState(I2C_Type *base)
{
    /* True if MSTPENDING flag is set and MSTSTATE is zero == idle */
    return ((base->STAT & (I2C_STAT_MSTPENDING_MASK | I2C_STAT_MSTSTATE_MASK)) == I2C_STAT_MSTPENDING_MASK);
}

/*!
 * @brief Sends a START on the I2C bus.
 *
 * This function is used to initiate a new master mode transfer by sending the START signal.
 * The slave address is sent following the I2C START signal.
 *
 * @param base I2C peripheral base pointer
 * @param address 7-bit slave device address.
 * @param direction Master transfer directions(transmit/receive).
 * @retval kStatus_Success Successfully send the start signal.
 * @retval kStatus_I2C_Busy Current bus is busy.
 */
status_t I2C_MasterStart(I2C_Type *base, uint8_t address, i2c_direction_t direction);

/*!
 * @brief Sends a STOP signal on the I2C bus.
 *
 * @retval kStatus_Success Successfully send the stop signal.
 * @retval kStatus_I2C_Timeout Send stop signal failed, timeout.
 */
status_t I2C_MasterStop(I2C_Type *base);

/*!
 * @brief Sends a REPEATED START on the I2C bus.
 *
 * @param base I2C peripheral base pointer
 * @param address 7-bit slave device address.
 * @param direction Master transfer directions(transmit/receive).
 * @retval kStatus_Success Successfully send the start signal.
 * @retval kStatus_I2C_Busy Current bus is busy but not occupied by current I2C master.
 */
static inline status_t I2C_MasterRepeatedStart(I2C_Type *base, uint8_t address, i2c_direction_t direction)
{
    return I2C_MasterStart(base, address, direction);
}

/*!
 * @brief Performs a polling send transfer on the I2C bus.
 *
 * Sends up to @a txSize number of bytes to the previously addressed slave device. The slave may
 * reply with a NAK to any byte in order to terminate the transfer early. If this happens, this
 * function returns #kStatus_I2C_Nak.
 *
 * @param base  The I2C peripheral base address.
 * @param txBuff The pointer to the data to be transferred.
 * @param txSize The length in bytes of the data to be transferred.
 * @param flags Transfer control flag to control special behavior like suppressing start or stop, for normal transfers
 * use kI2C_TransferDefaultFlag
 * @retval kStatus_Success Data was sent successfully.
 * @retval #kStatus_I2C_Busy Another master is currently utilizing the bus.
 * @retval #kStatus_I2C_Nak The slave device sent a NAK in response to a byte.
 * @retval #kStatus_I2C_ArbitrationLost Arbitration lost error.
 */
status_t I2C_MasterWriteBlocking(I2C_Type *base, const void *txBuff, size_t txSize, uint32_t flags);

/*!
 * @brief Performs a polling receive transfer on the I2C bus.
 *
 * @param base  The I2C peripheral base address.
 * @param rxBuff The pointer to the data to be transferred.
 * @param rxSize The length in bytes of the data to be transferred.
 * @param flags Transfer control flag to control special behavior like suppressing start or stop, for normal transfers
 * use kI2C_TransferDefaultFlag
 * @retval kStatus_Success Data was received successfully.
 * @retval #kStatus_I2C_Busy Another master is currently utilizing the bus.
 * @retval #kStatus_I2C_Nak The slave device sent a NAK in response to a byte.
 * @retval #kStatus_I2C_ArbitrationLost Arbitration lost error.
 */
status_t I2C_MasterReadBlocking(I2C_Type *base, void *rxBuff, size_t rxSize, uint32_t flags);

/*!
 * @brief Performs a master polling transfer on the I2C bus.
 *
 * @note The API does not return until the transfer succeeds or fails due
 * to arbitration lost or receiving a NAK.
 *
 * @param base I2C peripheral base address.
 * @param xfer Pointer to the transfer structure.
 * @retval kStatus_Success Successfully complete the data transmission.
 * @retval kStatus_I2C_Busy Previous transmission still not finished.
 * @retval kStatus_I2C_Timeout Transfer error, wait signal timeout.
 * @retval kStatus_I2C_ArbitrationLost Transfer error, arbitration lost.
 * @retval kStataus_I2C_Nak Transfer error, receive NAK during transfer.
 */
status_t I2C_MasterTransferBlocking(I2C_Type *base, i2c_master_transfer_t *xfer);

/*@}*/

/*! @name Non-blocking */
/*@{*/

/*!
 * @brief Creates a new handle for the I2C master non-blocking APIs.
 *
 * The creation of a handle is for use with the non-blocking APIs. Once a handle
 * is created, there is not a corresponding destroy handle. If the user wants to
 * terminate a transfer, the I2C_MasterTransferAbort() API shall be called.
 *
 * @param base The I2C peripheral base address.
 * @param[out] handle Pointer to the I2C master driver handle.
 * @param callback User provided pointer to the asynchronous callback function.
 * @param userData User provided pointer to the application callback data.
 */
void I2C_MasterTransferCreateHandle(I2C_Type *base,
                                    i2c_master_handle_t *handle,
                                    i2c_master_transfer_callback_t callback,
                                    void *userData);

/*!
 * @brief Performs a non-blocking transaction on the I2C bus.
 *
 * @param base The I2C peripheral base address.
 * @param handle Pointer to the I2C master driver handle.
 * @param xfer The pointer to the transfer descriptor.
 * @retval kStatus_Success The transaction was started successfully.
 * @retval #kStatus_I2C_Busy Either another master is currently utilizing the bus, or a non-blocking
 *      transaction is already in progress.
 */
status_t I2C_MasterTransferNonBlocking(I2C_Type *base, i2c_master_handle_t *handle, i2c_master_transfer_t *xfer);

/*!
 * @brief Returns number of bytes transferred so far.
 * @param base The I2C peripheral base address.
 * @param handle Pointer to the I2C master driver handle.
 * @param[out] count Number of bytes transferred so far by the non-blocking transaction.
 * @retval kStatus_Success
 * @retval #kStatus_I2C_Busy
 */
status_t I2C_MasterTransferGetCount(I2C_Type *base, i2c_master_handle_t *handle, size_t *count);

/*!
 * @brief Terminates a non-blocking I2C master transmission early.
 *
 * @note It is not safe to call this function from an IRQ handler that has a higher priority than the
 *      I2C peripheral's IRQ priority.
 *
 * @param base The I2C peripheral base address.
 * @param handle Pointer to the I2C master driver handle.
 * @retval kStatus_Success A transaction was successfully aborted.
 * @retval #kStatus_I2C_Timeout Timeout during polling for flags.
 */
status_t I2C_MasterTransferAbort(I2C_Type *base, i2c_master_handle_t *handle);

/*@}*/

/*! @name IRQ handler */
/*@{*/

/*!
 * @brief Reusable routine to handle master interrupts.
 * @note This function does not need to be called unless you are reimplementing the
 *  nonblocking API's interrupt handler routines to add special functionality.
 * @param base The I2C peripheral base address.
 * @param handle Pointer to the I2C master driver handle.
 */
void I2C_MasterTransferHandleIRQ(I2C_Type *base, i2c_master_handle_t *handle);

/*@}*/

/*! @} */ /* end of i2c_master_driver */

/*!
 * @addtogroup i2c_slave_driver
 * @{
 */

/*! @name Slave initialization and deinitialization */
/*@{*/

/*!
 * @brief Provides a default configuration for the I2C slave peripheral.
 *
 * This function provides the following default configuration for the I2C slave peripheral:
 * @code
 *  slaveConfig->enableSlave = true;
 *  slaveConfig->address0.disable = false;
 *  slaveConfig->address0.address = 0u;
 *  slaveConfig->address1.disable = true;
 *  slaveConfig->address2.disable = true;
 *  slaveConfig->address3.disable = true;
 *  slaveConfig->busSpeed = kI2C_SlaveStandardMode;
 * @endcode
 *
 * After calling this function, override any settings  to customize the configuration,
 * prior to initializing the master driver with I2C_SlaveInit(). Be sure to override at least the @a
 * address0.address member of the configuration structure with the desired slave address.
 *
 * @param[out] slaveConfig User provided configuration structure that is set to default values. Refer to
 *      #i2c_slave_config_t.
 */
void I2C_SlaveGetDefaultConfig(i2c_slave_config_t *slaveConfig);

/*!
 * @brief Initializes the I2C slave peripheral.
 *
 * This function enables the peripheral clock and initializes the I2C slave peripheral as described by the user
 * provided configuration.
 *
 * @param base The I2C peripheral base address.
 * @param slaveConfig User provided peripheral configuration. Use I2C_SlaveGetDefaultConfig() to get a set of defaults
 *      that you can override.
 * @param srcClock_Hz Frequency in Hertz of the I2C functional clock. Used to calculate CLKDIV value to provide
 * enough
 *                       data setup time for master when slave stretches the clock.
 */
status_t I2C_SlaveInit(I2C_Type *base, const i2c_slave_config_t *slaveConfig, uint32_t srcClock_Hz);

/*!
 * @brief Configures Slave Address n register.
 *
 * This function writes new value to Slave Address register.
 *
 * @param base The I2C peripheral base address.
 * @param addressRegister The module supports multiple address registers. The parameter determines which one shall be
 * changed.
 * @param address The slave address to be stored to the address register for matching.
 * @param addressDisable Disable matching of the specified address register.
  */
void I2C_SlaveSetAddress(I2C_Type *base,
                         i2c_slave_address_register_t addressRegister,
                         uint8_t address,
                         bool addressDisable);

/*!
* @brief Deinitializes the I2C slave peripheral.
*
 * This function disables the I2C slave peripheral and gates the clock. It also performs a software
 * reset to restore the peripheral to reset conditions.
 *
 * @param base The I2C peripheral base address.
 */
void I2C_SlaveDeinit(I2C_Type *base);

/*!
 * @brief Enables or disables the I2C module as slave.
 *
 * @param base The I2C peripheral base address.
 * @param enable True to enable or flase to disable.
 */
static inline void I2C_SlaveEnable(I2C_Type *base, bool enable)
{
    /* Set or clear the SLVEN bit in the CFG register. */
    base->CFG = I2C_CFG_SLVEN(enable);
}

/*@}*/ /* end of Slave initialization and deinitialization */

/*! @name Slave status */
/*@{*/

/*!
 * @brief Clears the I2C status flag state.
 *
 * The following status register flags can be cleared:
 * - slave deselected flag
 *
 * Attempts to clear other flags has no effect.
 *
 * @param base The I2C peripheral base address.
 * @param statusMask A bitmask of status flags that are to be cleared. The mask is composed of
 *  #_i2c_slave_flags enumerators OR'd together. You may pass the result of a previous call to
 *  I2C_SlaveGetStatusFlags().
 * @see _i2c_slave_flags.
 */
static inline void I2C_SlaveClearStatusFlags(I2C_Type *base, uint32_t statusMask)
{
    /* Allow clearing just slave status flags */
    base->STAT = statusMask & I2C_STAT_SLVDESEL_MASK;
}

/*@}*/ /* end of Slave status */

/*! @name Slave bus operations */
/*@{*/

/*!
 * @brief Performs a polling send transfer on the I2C bus.
 *
 * The function executes blocking address phase and blocking data phase.
 *
 * @param base  The I2C peripheral base address.
 * @param txBuff The pointer to the data to be transferred.
 * @param txSize The length in bytes of the data to be transferred.
 * @return kStatus_Success Data has been sent.
 * @return kStatus_Fail Unexpected slave state (master data write while master read from slave is expected).
 */
status_t I2C_SlaveWriteBlocking(I2C_Type *base, const uint8_t *txBuff, size_t txSize);

/*!
 * @brief Performs a polling receive transfer on the I2C bus.
 *
 * The function executes blocking address phase and blocking data phase.
 *
 * @param base  The I2C peripheral base address.
 * @param rxBuff The pointer to the data to be transferred.
 * @param rxSize The length in bytes of the data to be transferred.
 * @return kStatus_Success Data has been received.
 * @return kStatus_Fail Unexpected slave state (master data read while master write to slave is expected).
 */
status_t I2C_SlaveReadBlocking(I2C_Type *base, uint8_t *rxBuff, size_t rxSize);

/*@}*/ /* end of Slave bus operations */

/*! @name Slave non-blocking */
/*@{*/

/*!
 * @brief Creates a new handle for the I2C slave non-blocking APIs.
 *
 * The creation of a handle is for use with the non-blocking APIs. Once a handle
 * is created, there is not a corresponding destroy handle. If the user wants to
 * terminate a transfer, the I2C_SlaveTransferAbort() API shall be called.
 *
 * @param base The I2C peripheral base address.
 * @param[out] handle Pointer to the I2C slave driver handle.
 * @param callback User provided pointer to the asynchronous callback function.
 * @param userData User provided pointer to the application callback data.
 */
void I2C_SlaveTransferCreateHandle(I2C_Type *base,
                                   i2c_slave_handle_t *handle,
                                   i2c_slave_transfer_callback_t callback,
                                   void *userData);

/*!
 * @brief Starts accepting slave transfers.
 *
 * Call this API after calling I2C_SlaveInit() and I2C_SlaveTransferCreateHandle() to start processing
 * transactions driven by an I2C master. The slave monitors the I2C bus and pass events to the
 * callback that was passed into the call to I2C_SlaveTransferCreateHandle(). The callback is always invoked
 * from the interrupt context.
 *
 * If no slave Tx transfer is busy, a master read from slave request invokes #kI2C_SlaveTransmitEvent callback.
 * If no slave Rx transfer is busy, a master write to slave request invokes #kI2C_SlaveReceiveEvent callback.
 *
 * The set of events received by the callback is customizable. To do so, set the @a eventMask parameter to
 * the OR'd combination of #i2c_slave_transfer_event_t enumerators for the events you wish to receive.
 * The #kI2C_SlaveTransmitEvent and #kI2C_SlaveReceiveEvent events are always enabled and do not need
 * to be included in the mask. Alternatively, you can pass 0 to get a default set of only the transmit and
 * receive events that are always enabled. In addition, the #kI2C_SlaveAllEvents constant is provided as
 * a convenient way to enable all events.
 *
 * @param base The I2C peripheral base address.
 * @param handle Pointer to i2c_slave_handle_t structure which stores the transfer state.
 * @param eventMask Bit mask formed by OR'ing together #i2c_slave_transfer_event_t enumerators to specify
 *      which events to send to the callback. Other accepted values are 0 to get a default set of
 *      only the transmit and receive events, and #kI2C_SlaveAllEvents to enable all events.
 *
 * @retval kStatus_Success Slave transfers were successfully started.
 * @retval #kStatus_I2C_Busy Slave transfers have already been started on this handle.
 */
status_t I2C_SlaveTransferNonBlocking(I2C_Type *base, i2c_slave_handle_t *handle, uint32_t eventMask);

/*!
 * @brief Starts accepting master read from slave requests.
 *
 * The function can be called in response to #kI2C_SlaveTransmitEvent callback to start a new slave Tx transfer
 * from within the transfer callback.
 *
 * The set of events received by the callback is customizable. To do so, set the @a eventMask parameter to
 * the OR'd combination of #i2c_slave_transfer_event_t enumerators for the events you wish to receive.
 * The #kI2C_SlaveTransmitEvent and #kI2C_SlaveReceiveEvent events are always enabled and do not need
 * to be included in the mask. Alternatively, you can pass 0 to get a default set of only the transmit and
 * receive events that are always enabled. In addition, the #kI2C_SlaveAllEvents constant is provided as
 * a convenient way to enable all events.
 *
 * @param base The I2C peripheral base address.
 * @param transfer Pointer to #i2c_slave_transfer_t structure.
 * @param txData Pointer to data to send to master.
 * @param txSize Size of txData in bytes.
 * @param eventMask Bit mask formed by OR'ing together #i2c_slave_transfer_event_t enumerators to specify
 *      which events to send to the callback. Other accepted values are 0 to get a default set of
 *      only the transmit and receive events, and #kI2C_SlaveAllEvents to enable all events.
 *
 * @retval kStatus_Success Slave transfers were successfully started.
 * @retval #kStatus_I2C_Busy Slave transfers have already been started on this handle.
 */
status_t I2C_SlaveSetSendBuffer(
    I2C_Type *base, volatile i2c_slave_transfer_t *transfer, const void *txData, size_t txSize, uint32_t eventMask);

/*!
 * @brief Starts accepting master write to slave requests.
  *
 * The function can be called in response to #kI2C_SlaveReceiveEvent callback to start a new slave Rx transfer
 * from within the transfer callback.
 *
 * The set of events received by the callback is customizable. To do so, set the @a eventMask parameter to
 * the OR'd combination of #i2c_slave_transfer_event_t enumerators for the events you wish to receive.
 * The #kI2C_SlaveTransmitEvent and #kI2C_SlaveReceiveEvent events are always enabled and do not need
 * to be included in the mask. Alternatively, you can pass 0 to get a default set of only the transmit and
 * receive events that are always enabled. In addition, the #kI2C_SlaveAllEvents constant is provided as
 * a convenient way to enable all events.
 *
 * @param base The I2C peripheral base address.
 * @param transfer Pointer to #i2c_slave_transfer_t structure.
 * @param rxData Pointer to data to store data from master.
 * @param rxSize Size of rxData in bytes.
 * @param eventMask Bit mask formed by OR'ing together #i2c_slave_transfer_event_t enumerators to specify
 *      which events to send to the callback. Other accepted values are 0 to get a default set of
 *      only the transmit and receive events, and #kI2C_SlaveAllEvents to enable all events.
 *
 * @retval kStatus_Success Slave transfers were successfully started.
 * @retval #kStatus_I2C_Busy Slave transfers have already been started on this handle.
 */
status_t I2C_SlaveSetReceiveBuffer(
    I2C_Type *base, volatile i2c_slave_transfer_t *transfer, void *rxData, size_t rxSize, uint32_t eventMask);

/*!
 * @brief Returns the slave address sent by the I2C master.
 *
 * This function should only be called from the address match event callback #kI2C_SlaveAddressMatchEvent.
 *
 * @param base The I2C peripheral base address.
 * @param transfer The I2C slave transfer.
 * @return The 8-bit address matched by the I2C slave. Bit 0 contains the R/w direction bit, and
 *      the 7-bit slave address is in the upper 7 bits.
 */
static inline uint32_t I2C_SlaveGetReceivedAddress(I2C_Type *base, volatile i2c_slave_transfer_t *transfer)
{
    return transfer->receivedAddress;
}

/*!
 * @brief Aborts the slave non-blocking transfers.
 * @note This API could be called at any time to stop slave for handling the bus events.
 * @param base The I2C peripheral base address.
 * @param handle Pointer to i2c_slave_handle_t structure which stores the transfer state.
 * @retval kStatus_Success
 * @retval #kStatus_I2C_Idle
 */
void I2C_SlaveTransferAbort(I2C_Type *base, i2c_slave_handle_t *handle);

/*!
 * @brief Gets the slave transfer remaining bytes during a interrupt non-blocking transfer.
 *
 * @param base I2C base pointer.
 * @param handle pointer to i2c_slave_handle_t structure.
 * @param count Number of bytes transferred so far by the non-blocking transaction.
 * @retval kStatus_InvalidArgument count is Invalid.
 * @retval kStatus_Success Successfully return the count.
 */
status_t I2C_SlaveTransferGetCount(I2C_Type *base, i2c_slave_handle_t *handle, size_t *count);

/*@}*/ /* end of Slave non-blocking */

/*! @name Slave IRQ handler */
/*@{*/

/*!
 * @brief Reusable routine to handle slave interrupts.
 * @note This function does not need to be called unless you are reimplementing the
 *  non blocking API's interrupt handler routines to add special functionality.
 * @param base The I2C peripheral base address.
 * @param handle Pointer to i2c_slave_handle_t structure which stores the transfer state.
 */
void I2C_SlaveTransferHandleIRQ(I2C_Type *base, i2c_slave_handle_t *handle);

/*@}*/ /* end of Slave IRQ handler */

/*! @} */ /* end of i2c_slave_driver */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_I2C_H_ */
