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

#ifndef _FSL_DMA_H_
#define _FSL_DMA_H_

#include "fsl_common.h"

/*!
 * @addtogroup dma
 * @{
 */

/*! @file */
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief DMA driver version */
#define FSL_DMA_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */
/*@}*/

#define DMA_MAX_TRANSFER_COUNT 0x400

#if defined FSL_FEATURE_DMA_NUMBER_OF_CHANNELS
#define FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(x) FSL_FEATURE_DMA_NUMBER_OF_CHANNELS
#define FSL_FEATURE_DMA_MAX_CHANNELS FSL_FEATURE_DMA_NUMBER_OF_CHANNELS
#define FSL_FEATURE_DMA_ALL_CHANNELS (FSL_FEATURE_DMA_NUMBER_OF_CHANNELS * FSL_FEATURE_SOC_DMA_COUNT)
#define FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE (512)
#endif

/* Channel group consists of 32 channels. channel_group = (channel / 32) */
#define DMA_CHANNEL_GROUP(channel) (((uint8_t)channel) >> 5U)
/* Channel index in channel group. channel_index = (channel % 32) */
#define DMA_CHANNEL_INDEX(channel) (((uint8_t)channel) & 0x1F)

/*! @brief DMA descriptor structure */
typedef struct _dma_descriptor
{
    uint32_t xfercfg;     /*!< Transfer configuration */
    void *srcEndAddr;     /*!< Last source address of DMA transfer */
    void *dstEndAddr;     /*!< Last destination address of DMA transfer */
    void *linkToNextDesc; /*!< Address of next DMA descriptor in chain */
} dma_descriptor_t;

/*! @brief DMA transfer configuration */
typedef struct _dma_xfercfg
{
    bool valid;             /*!< Descriptor is ready to transfer */
    bool reload;            /*!< Reload channel configuration register after
                                 current descriptor is exhausted */
    bool swtrig;            /*!< Perform software trigger. Transfer if fired
                                 when 'valid' is set */
    bool clrtrig;           /*!< Clear trigger */
    bool intA;              /*!< Raises IRQ when transfer is done and set IRQA status register flag */
    bool intB;              /*!< Raises IRQ when transfer is done and set IRQB status register flag */
    uint8_t byteWidth;      /*!< Byte width of data to transfer */
    uint8_t srcInc;         /*!< Increment source address by 'srcInc' x 'byteWidth' */
    uint8_t dstInc;         /*!< Increment destination address by 'dstInc' x 'byteWidth' */
    uint16_t transferCount; /*!< Number of transfers */
} dma_xfercfg_t;

/*! @brief DMA channel priority */
typedef enum _dma_priority
{
    kDMA_ChannelPriority0 = 0, /*!< Highest channel priority - priority 0 */
    kDMA_ChannelPriority1,     /*!< Channel priority 1 */
    kDMA_ChannelPriority2,     /*!< Channel priority 2 */
    kDMA_ChannelPriority3,     /*!< Channel priority 3 */
    kDMA_ChannelPriority4,     /*!< Channel priority 4 */
    kDMA_ChannelPriority5,     /*!< Channel priority 5 */
    kDMA_ChannelPriority6,     /*!< Channel priority 6 */
    kDMA_ChannelPriority7,     /*!< Lowest channel priority - priority 7 */
} dma_priority_t;

/*! @brief DMA interrupt flags */
typedef enum _dma_int
{
    kDMA_IntA,     /*!< DMA interrupt flag A */
    kDMA_IntB,     /*!< DMA interrupt flag B */
    kDMA_IntError, /*!< DMA interrupt flag error */
} dma_irq_t;

/*! @brief DMA trigger type*/
typedef enum _dma_trigger_type
{
    kDMA_NoTrigger = 0,                                                               /*!< Trigger is disabled */
    kDMA_LowLevelTrigger = DMA_CHANNEL_CFG_HWTRIGEN(1) | DMA_CHANNEL_CFG_TRIGTYPE(1), /*!< Low level active trigger */
    kDMA_HighLevelTrigger = DMA_CHANNEL_CFG_HWTRIGEN(1) | DMA_CHANNEL_CFG_TRIGTYPE(1) |
                            DMA_CHANNEL_CFG_TRIGPOL(1),    /*!< High level active trigger */
    kDMA_FallingEdgeTrigger = DMA_CHANNEL_CFG_HWTRIGEN(1), /*!< Falling edge active trigger */
    kDMA_RisingEdgeTrigger =
        DMA_CHANNEL_CFG_HWTRIGEN(1) | DMA_CHANNEL_CFG_TRIGPOL(1), /*!< Rising edge active trigger */
} dma_trigger_type_t;

/*! @brief DMA trigger burst */
typedef enum _dma_trigger_burst
{
    kDMA_SingleTransfer = 0,                                /*!< Single transfer */
    kDMA_LevelBurstTransfer = DMA_CHANNEL_CFG_TRIGBURST(1), /*!< Burst transfer driven by level trigger */
    kDMA_EdgeBurstTransfer1 = DMA_CHANNEL_CFG_TRIGBURST(1), /*!< Perform 1 transfer by edge trigger */
    kDMA_EdgeBurstTransfer2 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(1), /*!< Perform 2 transfers by edge trigger */
    kDMA_EdgeBurstTransfer4 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(2), /*!< Perform 4 transfers by edge trigger */
    kDMA_EdgeBurstTransfer8 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(3), /*!< Perform 8 transfers by edge trigger */
    kDMA_EdgeBurstTransfer16 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(4), /*!< Perform 16 transfers by edge trigger */
    kDMA_EdgeBurstTransfer32 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(5), /*!< Perform 32 transfers by edge trigger */
    kDMA_EdgeBurstTransfer64 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(6), /*!< Perform 64 transfers by edge trigger */
    kDMA_EdgeBurstTransfer128 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(7), /*!< Perform 128 transfers by edge trigger */
    kDMA_EdgeBurstTransfer256 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(8), /*!< Perform 256 transfers by edge trigger */
    kDMA_EdgeBurstTransfer512 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(9), /*!< Perform 512 transfers by edge trigger */
    kDMA_EdgeBurstTransfer1024 =
        DMA_CHANNEL_CFG_TRIGBURST(1) | DMA_CHANNEL_CFG_BURSTPOWER(10), /*!< Perform 1024 transfers by edge trigger */
} dma_trigger_burst_t;

/*! @brief DMA burst wrapping */
typedef enum _dma_burst_wrap
{
    kDMA_NoWrap = 0,                                /*!< Wrapping is disabled */
    kDMA_SrcWrap = DMA_CHANNEL_CFG_SRCBURSTWRAP(1), /*!< Wrapping is enabled for source */
    kDMA_DstWrap = DMA_CHANNEL_CFG_DSTBURSTWRAP(1), /*!< Wrapping is enabled for destination */
    kDMA_SrcAndDstWrap = DMA_CHANNEL_CFG_SRCBURSTWRAP(1) |
                         DMA_CHANNEL_CFG_DSTBURSTWRAP(1), /*!< Wrapping is enabled for source and destination */
} dma_burst_wrap_t;

/*! @brief DMA transfer type */
typedef enum _dma_transfer_type
{
    kDMA_MemoryToMemory = 0x0U, /*!< Transfer from memory to memory (increment source and destination) */
    kDMA_PeripheralToMemory,    /*!< Transfer from peripheral to memory (increment only destination) */
    kDMA_MemoryToPeripheral,    /*!< Transfer from memory to peripheral (increment only source)*/
    kDMA_StaticToStatic,        /*!< Peripheral to static memory (do not increment source or destination) */
} dma_transfer_type_t;

/*! @brief DMA channel trigger */
typedef struct _dma_channel_trigger
{
    dma_trigger_type_t type;   /*!< Select hardware trigger as edge triggered or level triggered. */
    dma_trigger_burst_t burst; /*!< Select whether hardware triggers cause a single or burst transfer. */
    dma_burst_wrap_t wrap;     /*!< Select wrap type, source wrap or dest wrap, or both. */
} dma_channel_trigger_t;

/*! @brief DMA transfer status */
enum _dma_transfer_status
{
    kStatus_DMA_Busy = MAKE_STATUS(kStatusGroup_DMA, 0), /*!< Channel is busy and can't handle the
                                                                transfer request. */
};

/*! @brief DMA transfer configuration */
typedef struct _dma_transfer_config
{
    uint8_t *srcAddr;      /*!< Source data address */
    uint8_t *dstAddr;      /*!< Destination data address */
    uint8_t *nextDesc;     /*!< Chain custom descriptor */
    dma_xfercfg_t xfercfg; /*!< Transfer options */
    bool isPeriph;         /*!< DMA transfer is driven by peripheral */
} dma_transfer_config_t;

/*! @brief Callback for DMA */
struct _dma_handle;

/*! @brief Define Callback function for DMA. */
typedef void (*dma_callback)(struct _dma_handle *handle, void *userData, bool transferDone, uint32_t intmode);

/*! @brief DMA transfer handle structure */
typedef struct _dma_handle
{
    dma_callback callback; /*!< Callback function. Invoked when transfer
                               of descriptor with interrupt flag finishes */
    void *userData;        /*!< Callback function parameter */
    DMA_Type *base;        /*!< DMA peripheral base address */
    uint8_t channel;       /*!< DMA channel number */
} dma_handle_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @name DMA initialization and De-initialization
 * @{
 */

/*!
 * @brief Initializes DMA peripheral.
 *
 * This function enable the DMA clock, set descriptor table and
 * enable DMA peripheral.
 *
 * @param base DMA peripheral base address.
 */
void DMA_Init(DMA_Type *base);

/*!
 * @brief Deinitializes DMA peripheral.
 *
 * This function gates the DMA clock.
 *
 * @param base DMA peripheral base address.
 */
void DMA_Deinit(DMA_Type *base);

/* @} */
/*!
 * @name DMA Channel Operation
 * @{
 */

/*!
* @brief Return whether DMA channel is processing transfer
*
* @param base DMA peripheral base address.
* @param channel DMA channel number.
* @return True for active state, false otherwise.
*/
static inline bool DMA_ChannelIsActive(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    return (base->COMMON[DMA_CHANNEL_GROUP(channel)].ACTIVE & (1U << DMA_CHANNEL_INDEX(channel))) ? true : false;
}

/*!
 * @brief Enables the interrupt source for the DMA transfer.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 */
static inline void DMA_EnableChannelInterrupts(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    base->COMMON[DMA_CHANNEL_GROUP(channel)].INTENSET |= 1U << DMA_CHANNEL_INDEX(channel);
}

/*!
 * @brief Disables the interrupt source for the DMA transfer.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 */
static inline void DMA_DisableChannelInterrupts(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    base->COMMON[DMA_CHANNEL_GROUP(channel)].INTENCLR |= 1U << DMA_CHANNEL_INDEX(channel);
}

/*!
 * @brief Enable DMA channel.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 */
static inline void DMA_EnableChannel(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    base->COMMON[DMA_CHANNEL_GROUP(channel)].ENABLESET |= 1U << DMA_CHANNEL_INDEX(channel);
}

/*!
 * @brief Disable DMA channel.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 */
static inline void DMA_DisableChannel(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    base->COMMON[DMA_CHANNEL_GROUP(channel)].ENABLECLR |= 1U << DMA_CHANNEL_INDEX(channel);
}

/*!
 * @brief Set PERIPHREQEN of channel configuration register.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 */
static inline void DMA_EnableChannelPeriphRq(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    base->CHANNEL[channel].CFG |= DMA_CHANNEL_CFG_PERIPHREQEN_MASK;
}

/*!
 * @brief Get PERIPHREQEN value of channel configuration register.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 * @return True for enabled PeriphRq, false for disabled.
 */
static inline void DMA_DisableChannelPeriphRq(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    base->CHANNEL[channel].CFG &= ~DMA_CHANNEL_CFG_PERIPHREQEN_MASK;
}

/*!
 * @brief Set trigger settings of DMA channel.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 * @param trigger trigger configuration.
 */
void DMA_ConfigureChannelTrigger(DMA_Type *base, uint32_t channel, dma_channel_trigger_t *trigger);

/*!
 * @brief Gets the remaining bytes of the current DMA descriptor transfer.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 * @return The number of bytes which have not been transferred yet.
 */
uint32_t DMA_GetRemainingBytes(DMA_Type *base, uint32_t channel);

/*!
 * @brief Set priority of channel configuration register.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 * @param priority Channel priority value.
 */
static inline void DMA_SetChannelPriority(DMA_Type *base, uint32_t channel, dma_priority_t priority)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    base->CHANNEL[channel].CFG =
        (base->CHANNEL[channel].CFG & (~(DMA_CHANNEL_CFG_CHPRIORITY_MASK))) | DMA_CHANNEL_CFG_CHPRIORITY(priority);
}

/*!
 * @brief Get priority of channel configuration register.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 * @return Channel priority value.
 */
static inline dma_priority_t DMA_GetChannelPriority(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));
    return (dma_priority_t)((base->CHANNEL[channel].CFG & DMA_CHANNEL_CFG_CHPRIORITY_MASK) >>
                            DMA_CHANNEL_CFG_CHPRIORITY_SHIFT);
}

/*!
 * @brief Create application specific DMA descriptor
 *        to be used in a chain in transfer
 *
 * @param desc DMA descriptor address.
 * @param xfercfg Transfer configuration for DMA descriptor.
 * @param srcAddr Address of last item to transmit
 * @param dstAddr Address of last item to receive.
 * @param nextDesc Address of next descriptor in chain.
 */
void DMA_CreateDescriptor(dma_descriptor_t *desc, dma_xfercfg_t *xfercfg, void *srcAddr, void *dstAddr, void *nextDesc);

/* @} */

/*!
 * @name DMA Transactional Operation
 * @{
 */

/*!
 * @brief Abort running transfer by handle.
 *
 * This function aborts DMA transfer specified by handle.
 *
 * @param handle DMA handle pointer.
 */
void DMA_AbortTransfer(dma_handle_t *handle);

/*!
 * @brief Creates the DMA handle.
 *
 * This function is called if using transaction API for DMA. This function
 * initializes the internal state of DMA handle.
 *
 * @param handle DMA handle pointer. The DMA handle stores callback function and
 *               parameters.
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 */
void DMA_CreateHandle(dma_handle_t *handle, DMA_Type *base, uint32_t channel);

/*!
 * @brief Installs a callback function for the DMA transfer.
 *
 * This callback is called in DMA IRQ handler. Use the callback to do something after
 * the current major loop transfer completes.
 *
 * @param handle DMA handle pointer.
 * @param callback DMA callback function pointer.
 * @param userData Parameter for callback function.
 */
void DMA_SetCallback(dma_handle_t *handle, dma_callback callback, void *userData);

/*!
 * @brief Prepares the DMA transfer structure.
 *
 * This function prepares the transfer configuration structure according to the user input.
 *
 * @param config The user configuration structure of type dma_transfer_t.
 * @param srcAddr DMA transfer source address.
 * @param dstAddr DMA transfer destination address.
 * @param byteWidth DMA transfer destination address width(bytes).
 * @param transferBytes DMA transfer bytes to be transferred.
 * @param type DMA transfer type.
 * @param nextDesc Chain custom descriptor to transfer.
 * @note The data address and the data width must be consistent. For example, if the SRC
 *       is 4 bytes, so the source address must be 4 bytes aligned, or it shall result in
 *       source address error(SAE).
 */
void DMA_PrepareTransfer(dma_transfer_config_t *config,
                         void *srcAddr,
                         void *dstAddr,
                         uint32_t byteWidth,
                         uint32_t transferBytes,
                         dma_transfer_type_t type,
                         void *nextDesc);

/*!
 * @brief Submits the DMA transfer request.
 *
 * This function submits the DMA transfer request according to the transfer configuration structure.
 * If the user submits the transfer request repeatedly, this function packs an unprocessed request as
 * a TCD and enables scatter/gather feature to process it in the next time.
 *
 * @param handle DMA handle pointer.
 * @param config Pointer to DMA transfer configuration structure.
 * @retval kStatus_DMA_Success It means submit transfer request succeed.
 * @retval kStatus_DMA_QueueFull It means TCD queue is full. Submit transfer request is not allowed.
 * @retval kStatus_DMA_Busy It means the given channel is busy, need to submit request later.
 */
status_t DMA_SubmitTransfer(dma_handle_t *handle, dma_transfer_config_t *config);

/*!
 * @brief DMA start transfer.
 *
 * This function enables the channel request. User can call this function after submitting the transfer request
 * or before submitting the transfer request.
 *
 * @param handle DMA handle pointer.
 */
void DMA_StartTransfer(dma_handle_t *handle);

/*!
 * @brief DMA IRQ handler for descriptor transfer complete.
 *
 * This function clears the channel major interrupt flag and call
 * the callback function if it is not NULL.
 */
void DMA_HandleIRQ(void);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/* @} */

#endif /*_FSL_DMA_H_*/
