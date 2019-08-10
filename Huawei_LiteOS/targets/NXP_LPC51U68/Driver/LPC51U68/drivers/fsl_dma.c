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

#include "fsl_dma.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get instance number for DMA.
 *
 * @param base DMA peripheral base address.
 */
static int32_t DMA_GetInstance(DMA_Type *base);

/*!
 * @brief Get virtual channel number.
 *
 * @param base DMA peripheral base address.
 */
static uint32_t DMA_GetVirtualStartChannel(DMA_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Array to map DMA instance number to base pointer. */
static DMA_Type *const s_dmaBases[] = DMA_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Array to map DMA instance number to clock name. */
static const clock_ip_name_t s_dmaClockName[] = DMA_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*! @brief Array to map DMA instance number to IRQ number. */
static const IRQn_Type s_dmaIRQNumber[] = DMA_IRQS;

/*! @brief Pointers to transfer handle for each DMA channel. */
static dma_handle_t *s_DMAHandle[FSL_FEATURE_DMA_ALL_CHANNELS];

/*! @brief Static table of descriptors */
#if defined(__ICCARM__)
#pragma data_alignment = FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE
dma_descriptor_t s_dma_descriptor_table[FSL_FEATURE_SOC_DMA_COUNT][FSL_FEATURE_DMA_MAX_CHANNELS] = {0};
#elif defined(__CC_ARM)
__attribute__((aligned(FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE)))
dma_descriptor_t s_dma_descriptor_table[FSL_FEATURE_SOC_DMA_COUNT][FSL_FEATURE_DMA_MAX_CHANNELS] = {0};
#elif defined(__GNUC__)
__attribute__((aligned(FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE)))
dma_descriptor_t s_dma_descriptor_table[FSL_FEATURE_SOC_DMA_COUNT][FSL_FEATURE_DMA_MAX_CHANNELS] = {0};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

static int32_t DMA_GetInstance(DMA_Type *base)
{
    int32_t instance;
    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_dmaBases); instance++)
    {
        if (s_dmaBases[instance] == base)
        {
            break;
        }
    }
    assert(instance < ARRAY_SIZE(s_dmaBases));
    return instance < ARRAY_SIZE(s_dmaBases) ? instance : -1;
}

static uint32_t DMA_GetVirtualStartChannel(DMA_Type *base)
{
    uint32_t startChannel = 0, instance = 0;
    uint32_t i = 0;

    instance = DMA_GetInstance(base);

    /* Compute start channel */
    for (i = 0; i < instance; i++)
    {
        startChannel += FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(s_dmaBases[i]);
    }

    return startChannel;
}

void DMA_Init(DMA_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* enable dma clock gate */
    CLOCK_EnableClock(s_dmaClockName[DMA_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
    /* set descriptor table */
    base->SRAMBASE = (uint32_t)s_dma_descriptor_table;
    /* enable dma peripheral */
    base->CTRL |= DMA_CTRL_ENABLE_MASK;
}

void DMA_Deinit(DMA_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_DisableClock(s_dmaClockName[DMA_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
    /* Disable DMA peripheral */
    base->CTRL &= ~(DMA_CTRL_ENABLE_MASK);
}

void DMA_ConfigureChannelTrigger(DMA_Type *base, uint32_t channel, dma_channel_trigger_t *trigger)
{
    assert((channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base)) && (NULL != trigger));

    uint32_t tmp = (DMA_CHANNEL_CFG_HWTRIGEN_MASK | DMA_CHANNEL_CFG_TRIGPOL_MASK | DMA_CHANNEL_CFG_TRIGTYPE_MASK |
                    DMA_CHANNEL_CFG_TRIGBURST_MASK | DMA_CHANNEL_CFG_BURSTPOWER_MASK |
                    DMA_CHANNEL_CFG_SRCBURSTWRAP_MASK | DMA_CHANNEL_CFG_DSTBURSTWRAP_MASK);
    tmp = base->CHANNEL[channel].CFG & (~tmp);
    tmp |= (uint32_t)(trigger->type) | (uint32_t)(trigger->burst) | (uint32_t)(trigger->wrap);
    base->CHANNEL[channel].CFG = tmp;
}

/*!
 * @brief Gets the remaining bytes of the current DMA descriptor transfer.
 *
 * @param base DMA peripheral base address.
 * @param channel DMA channel number.
 * @return The number of bytes which have not been transferred yet.
 */
uint32_t DMA_GetRemainingBytes(DMA_Type *base, uint32_t channel)
{
    assert(channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base));

    /* NOTE: when descriptors are chained, ACTIVE bit is set for whole chain. It makes
     * impossible to distinguish between:
     * - transfer finishes (represented by value '0x3FF')
     * - and remaining 1024 bytes to transfer (value 0x3FF)
     * for all descriptor in chain, except the last one.
     * If you decide to use this function, please use 1023 transfers as maximal value */

    /* Channel not active (transfer finished) and value is 0x3FF - nothing to transfer */
    if ((!(base->COMMON[DMA_CHANNEL_GROUP(channel)].ACTIVE & (1U << (DMA_CHANNEL_INDEX(channel))))) &&
        (0x3FF == ((base->CHANNEL[channel].XFERCFG & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) >>
                   DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT)))
    {
        return 0;
    }

    return ((base->CHANNEL[channel].XFERCFG & DMA_CHANNEL_XFERCFG_XFERCOUNT_MASK) >>
            DMA_CHANNEL_XFERCFG_XFERCOUNT_SHIFT) +
           1;
}

static void DMA_SetupDescriptor(
    dma_descriptor_t *desc, uint32_t xfercfg, void *srcEndAddr, void *dstEndAddr, void *nextDesc)
{
    desc->xfercfg = xfercfg;
    desc->srcEndAddr = srcEndAddr;
    desc->dstEndAddr = dstEndAddr;
    desc->linkToNextDesc = nextDesc;
}

/* Verify and convert dma_xfercfg_t to XFERCFG register */
static void DMA_SetupXferCFG(dma_xfercfg_t *xfercfg, uint32_t *xfercfg_addr)
{
    assert(xfercfg != NULL);
    /* check source increment */
    assert((xfercfg->srcInc == 0) || (xfercfg->srcInc == 1) || (xfercfg->srcInc == 2) || (xfercfg->srcInc == 4));
    /* check destination increment */
    assert((xfercfg->dstInc == 0) || (xfercfg->dstInc == 1) || (xfercfg->dstInc == 2) || (xfercfg->dstInc == 4));
    /* check data width */
    assert((xfercfg->byteWidth == 1) || (xfercfg->byteWidth == 2) || (xfercfg->byteWidth == 4));
    /* check transfer count */
    assert(xfercfg->transferCount <= DMA_MAX_TRANSFER_COUNT);

    uint32_t xfer = 0, tmp;
    /* set valid flag - descriptor is ready now */
    xfer |= DMA_CHANNEL_XFERCFG_CFGVALID(xfercfg->valid ? 1 : 0);
    /* set reload - allow link to next descriptor */
    xfer |= DMA_CHANNEL_XFERCFG_RELOAD(xfercfg->reload ? 1 : 0);
    /* set swtrig flag - start transfer */
    xfer |= DMA_CHANNEL_XFERCFG_SWTRIG(xfercfg->swtrig ? 1 : 0);
    /* set transfer count */
    xfer |= DMA_CHANNEL_XFERCFG_CLRTRIG(xfercfg->clrtrig ? 1 : 0);
    /* set INTA */
    xfer |= DMA_CHANNEL_XFERCFG_SETINTA(xfercfg->intA ? 1 : 0);
    /* set INTB */
    xfer |= DMA_CHANNEL_XFERCFG_SETINTB(xfercfg->intB ? 1 : 0);
    /* set data width */
    tmp = xfercfg->byteWidth == 4 ? 2 : xfercfg->byteWidth - 1;
    xfer |= DMA_CHANNEL_XFERCFG_WIDTH(tmp);
    /* set source increment value */
    tmp = xfercfg->srcInc == 4 ? 3 : xfercfg->srcInc;
    xfer |= DMA_CHANNEL_XFERCFG_SRCINC(tmp);
    /* set destination increment value */
    tmp = xfercfg->dstInc == 4 ? 3 : xfercfg->dstInc;
    xfer |= DMA_CHANNEL_XFERCFG_DSTINC(tmp);
    /* set transfer count */
    xfer |= DMA_CHANNEL_XFERCFG_XFERCOUNT(xfercfg->transferCount - 1);

    /* store xferCFG */
    *xfercfg_addr = xfer;
}

void DMA_CreateDescriptor(dma_descriptor_t *desc, dma_xfercfg_t *xfercfg, void *srcAddr, void *dstAddr, void *nextDesc)
{
    uint32_t xfercfg_reg = 0;

    assert((NULL != desc) && (0 == (uint32_t)desc % 16) && (NULL != xfercfg));
    assert((NULL != srcAddr) && (0 == (uint32_t)srcAddr % xfercfg->byteWidth));
    assert((NULL != dstAddr) && (0 == (uint32_t)dstAddr % xfercfg->byteWidth));
    assert((NULL == nextDesc) || (0 == (uint32_t)nextDesc % 16));

    /* Setup channel configuration */
    DMA_SetupXferCFG(xfercfg, &xfercfg_reg);

    /* Set descriptor structure */
    DMA_SetupDescriptor(
        desc, xfercfg_reg, (uint8_t *)srcAddr + (xfercfg->srcInc * xfercfg->byteWidth * (xfercfg->transferCount - 1)),
        (uint8_t *)dstAddr + (xfercfg->dstInc * xfercfg->byteWidth * (xfercfg->transferCount - 1)), nextDesc);
}

void DMA_AbortTransfer(dma_handle_t *handle)
{
    assert(NULL != handle);

    DMA_DisableChannel(handle->base, handle->channel);
    while (handle->base->COMMON[DMA_CHANNEL_GROUP(handle->channel)].BUSY & (1U << DMA_CHANNEL_INDEX(handle->channel)))
    {
    }
    handle->base->COMMON[DMA_CHANNEL_GROUP(handle->channel)].ABORT |= 1U << DMA_CHANNEL_INDEX(handle->channel);
    DMA_EnableChannel(handle->base, handle->channel);
}

void DMA_CreateHandle(dma_handle_t *handle, DMA_Type *base, uint32_t channel)
{
    assert((NULL != handle) && (channel < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base)));

    int32_t dmaInstance;
    uint32_t startChannel = 0;
    /* base address is invalid DMA instance */
    dmaInstance = DMA_GetInstance(base);
    startChannel = DMA_GetVirtualStartChannel(base);

    memset(handle, 0, sizeof(*handle));
    handle->base = base;
    handle->channel = channel;
    s_DMAHandle[startChannel + channel] = handle;
    /* Enable NVIC interrupt */
    EnableIRQ(s_dmaIRQNumber[dmaInstance]);
}

void DMA_SetCallback(dma_handle_t *handle, dma_callback callback, void *userData)
{
    assert(handle != NULL);

    handle->callback = callback;
    handle->userData = userData;
}

void DMA_PrepareTransfer(dma_transfer_config_t *config,
                         void *srcAddr,
                         void *dstAddr,
                         uint32_t byteWidth,
                         uint32_t transferBytes,
                         dma_transfer_type_t type,
                         void *nextDesc)
{
    uint32_t xfer_count;
    assert((NULL != config) && (NULL != srcAddr) && (NULL != dstAddr));
    assert((byteWidth == 1) || (byteWidth == 2) || (byteWidth == 4));

    /* check max */
    xfer_count = transferBytes / byteWidth;
    assert((xfer_count <= DMA_MAX_TRANSFER_COUNT) && (0 == transferBytes % byteWidth));

    memset(config, 0, sizeof(*config));
    switch (type)
    {
        case kDMA_MemoryToMemory:
            config->xfercfg.srcInc = 1;
            config->xfercfg.dstInc = 1;
            config->isPeriph = false;
            break;
        case kDMA_PeripheralToMemory:
            /* Peripheral register - source doesn't increment */
            config->xfercfg.srcInc = 0;
            config->xfercfg.dstInc = 1;
            config->isPeriph = true;
            break;
        case kDMA_MemoryToPeripheral:
            /* Peripheral register - destination doesn't increment */
            config->xfercfg.srcInc = 1;
            config->xfercfg.dstInc = 0;
            config->isPeriph = true;
            break;
        case kDMA_StaticToStatic:
            config->xfercfg.srcInc = 0;
            config->xfercfg.dstInc = 0;
            config->isPeriph = true;
            break;
        default:
            return;
    }

    config->dstAddr = (uint8_t *)dstAddr;
    config->srcAddr = (uint8_t *)srcAddr;
    config->nextDesc = (uint8_t *)nextDesc;
    config->xfercfg.transferCount = xfer_count;
    config->xfercfg.byteWidth = byteWidth;
    config->xfercfg.intA = true;
    config->xfercfg.reload = nextDesc != NULL;
    config->xfercfg.valid = true;
}

status_t DMA_SubmitTransfer(dma_handle_t *handle, dma_transfer_config_t *config)
{
    assert((NULL != handle) && (NULL != config));
    uint32_t instance = DMA_GetInstance(handle->base);

    /* Previous transfer has not finished */
    if (DMA_ChannelIsActive(handle->base, handle->channel))
    {
        return kStatus_DMA_Busy;
    }

    /* enable/disable peripheral request */
    if (config->isPeriph)
    {
        DMA_EnableChannelPeriphRq(handle->base, handle->channel);
    }
    else
    {
        DMA_DisableChannelPeriphRq(handle->base, handle->channel);
    }

    DMA_CreateDescriptor(&(s_dma_descriptor_table[instance][handle->channel]), &config->xfercfg, config->srcAddr,
                         config->dstAddr, config->nextDesc);

    return kStatus_Success;
}

void DMA_StartTransfer(dma_handle_t *handle)
{
    assert(NULL != handle);

    uint32_t instance = DMA_GetInstance(handle->base);

    /* Enable channel interrupt */
    handle->base->COMMON[DMA_CHANNEL_GROUP(handle->channel)].INTENSET |= 1U << DMA_CHANNEL_INDEX(handle->channel);

    /* If HW trigger is enabled - disable SW trigger */
    if (handle->base->CHANNEL[handle->channel].CFG & DMA_CHANNEL_CFG_HWTRIGEN_MASK)
    {
        s_dma_descriptor_table[instance][handle->channel].xfercfg &= ~(DMA_CHANNEL_XFERCFG_SWTRIG_MASK);
    }
    /* Otherwise enable SW trigger */
    else
    {
        s_dma_descriptor_table[instance][handle->channel].xfercfg |= DMA_CHANNEL_XFERCFG_SWTRIG_MASK;
    }

    /* Set channel XFERCFG register according first channel descriptor. */
    handle->base->CHANNEL[handle->channel].XFERCFG = s_dma_descriptor_table[instance][handle->channel].xfercfg;
    /* At this moment, the channel ACTIVE bit is set and application cannot modify
     * or start another transfer using this channel. Channel ACTIVE bit is cleared by
    * 'AbortTransfer' function or when the transfer finishes */
}

void DMA_IRQHandle(DMA_Type *base)
{
    dma_handle_t *handle;
    int32_t channel_group;
    int32_t channel_index;
    uint32_t startChannel = DMA_GetVirtualStartChannel(base);
    uint32_t i = 0;

    /* Find channels that have completed transfer */
    for (i = 0; i < FSL_FEATURE_DMA_NUMBER_OF_CHANNELSn(base); i++)
    {
        handle = s_DMAHandle[i + startChannel];
        /* Handle is not present */
        if (NULL == handle)
        {
            continue;
        }
        channel_group = DMA_CHANNEL_GROUP(handle->channel);
        channel_index = DMA_CHANNEL_INDEX(handle->channel);
        /* Channel uses INTA flag */
        if (handle->base->COMMON[channel_group].INTA & (1U << channel_index))
        {
            /* Clear INTA flag */
            handle->base->COMMON[channel_group].INTA = 1U << channel_index;
            if (handle->callback)
            {
                (handle->callback)(handle, handle->userData, true, kDMA_IntA);
            }
        }
        /* Channel uses INTB flag */
        if (handle->base->COMMON[channel_group].INTB & (1U << channel_index))
        {
            /* Clear INTB flag */
            handle->base->COMMON[channel_group].INTB = 1U << channel_index;
            if (handle->callback)
            {
                (handle->callback)(handle, handle->userData, true, kDMA_IntB);
            }
        }
        /* Error flag */
        if (handle->base->COMMON[channel_group].ERRINT & (1U << channel_index))
        {
            /* Clear error flag */
            handle->base->COMMON[channel_group].ERRINT = 1U << channel_index;
            if (handle->callback)
            {
                (handle->callback)(handle, handle->userData, false, kDMA_IntError);
            }
        }
    }
}

void DMA0_DriverIRQHandler(void)
{
    DMA_IRQHandle(DMA0);
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

#if defined(DMA1)
void DMA1_DriverIRQHandler(void)
{
    DMA_IRQHandle(DMA1);
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
