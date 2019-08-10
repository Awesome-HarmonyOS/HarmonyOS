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

#include "fsl_pint.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Irq number array */
static const IRQn_Type s_pintIRQ[FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS] = PINT_IRQS;

/*! @brief Callback function array for PINT(s). */
static pint_cb_t s_pintCallback[FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS];

/*******************************************************************************
 * Code
 ******************************************************************************/

void PINT_Init(PINT_Type *base)
{
    uint32_t i;
    uint32_t pmcfg;

    assert(base);

    pmcfg = 0;
    for (i = 0; i < FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS; i++)
    {
        s_pintCallback[i] = NULL;
    }

    /* Disable all bit slices */
    for (i = 0; i < PINT_PIN_INT_COUNT; i++)
    {
        pmcfg = pmcfg | (kPINT_PatternMatchNever << (PININT_BITSLICE_CFG_START + (i * 3U)));
    }

    /* Enable the peripheral clock */
    CLOCK_EnableClock(kCLOCK_Pint);

    /* Reset the peripheral */
    RESET_PeripheralReset(kPINT_RST_SHIFT_RSTn);

    /* Disable all pattern match bit slices */
    base->PMCFG = pmcfg;
}

void PINT_PinInterruptConfig(PINT_Type *base, pint_pin_int_t intr, pint_pin_enable_t enable, pint_cb_t callback)
{
    assert(base);

    /* Clear Rise and Fall flags first */
    PINT_PinInterruptClrRiseFlag(base, intr);
    PINT_PinInterruptClrFallFlag(base, intr);

    /* select level or edge sensitive */
    base->ISEL = (base->ISEL & ~(1U << intr)) | ((enable & PINT_PIN_INT_LEVEL) ? (1U << intr) : 0U);

    /* enable rising or level interrupt */
    if (enable & (PINT_PIN_INT_LEVEL | PINT_PIN_INT_RISE))
    {
        base->SIENR = 1U << intr;
    }
    else
    {
        base->CIENR = 1U << intr;
    }

    /* Enable falling or select high level */
    if (enable & PINT_PIN_INT_FALL_OR_HIGH_LEVEL)
    {
        base->SIENF = 1U << intr;
    }
    else
    {
        base->CIENF = 1U << intr;
    }

    s_pintCallback[intr] = callback;
}

void PINT_PinInterruptGetConfig(PINT_Type *base, pint_pin_int_t pintr, pint_pin_enable_t *enable, pint_cb_t *callback)
{
    uint32_t mask;
    bool level;

    assert(base);

    *enable = kPINT_PinIntEnableNone;
    level = false;

    mask = 1U << pintr;
    if (base->ISEL & mask)
    {
        /* Pin interrupt is level sensitive */
        level = true;
    }

    if (base->IENR & mask)
    {
        if (level)
        {
            /* Level interrupt is enabled */
            *enable = kPINT_PinIntEnableLowLevel;
        }
        else
        {
            /* Rising edge interrupt */
            *enable = kPINT_PinIntEnableRiseEdge;
        }
    }

    if (base->IENF & mask)
    {
        if (level)
        {
            /* Level interrupt is active high */
            *enable = kPINT_PinIntEnableHighLevel;
        }
        else
        {
            /* Either falling or both edge */
            if (*enable == kPINT_PinIntEnableRiseEdge)
            {
                /* Rising and faling edge */
                *enable = kPINT_PinIntEnableBothEdges;
            }
            else
            {
                /* Falling edge */
                *enable = kPINT_PinIntEnableFallEdge;
            }
        }
    }

    *callback = s_pintCallback[pintr];
}

void PINT_PatternMatchConfig(PINT_Type *base, pint_pmatch_bslice_t bslice, pint_pmatch_cfg_t *cfg)
{
    uint32_t src_shift;
    uint32_t cfg_shift;
    uint32_t pmcfg;

    assert(base);

    src_shift = PININT_BITSLICE_SRC_START + (bslice * 3U);
    cfg_shift = PININT_BITSLICE_CFG_START + (bslice * 3U);

    /* Input source selection for selected bit slice */
    base->PMSRC = (base->PMSRC & ~(PININT_BITSLICE_SRC_MASK << src_shift)) | (cfg->bs_src << src_shift);

    /* Bit slice configuration */
    pmcfg = base->PMCFG;
    pmcfg = (pmcfg & ~(PININT_BITSLICE_CFG_MASK << cfg_shift)) | (cfg->bs_cfg << cfg_shift);

    /* If end point is true, enable the bits */
    if (bslice != 7U)
    {
        if (cfg->end_point)
        {
            pmcfg |= (0x1U << bslice);
        }
        else
        {
            pmcfg &= ~(0x1U << bslice);
        }
    }

    base->PMCFG = pmcfg;

    /* Save callback pointer */
    s_pintCallback[bslice] = cfg->callback;
}

void PINT_PatternMatchGetConfig(PINT_Type *base, pint_pmatch_bslice_t bslice, pint_pmatch_cfg_t *cfg)
{
    uint32_t src_shift;
    uint32_t cfg_shift;

    assert(base);

    src_shift = PININT_BITSLICE_SRC_START + (bslice * 3U);
    cfg_shift = PININT_BITSLICE_CFG_START + (bslice * 3U);

    cfg->bs_src = (pint_pmatch_input_src_t)((base->PMSRC & (PININT_BITSLICE_SRC_MASK << src_shift)) >> src_shift);
    cfg->bs_cfg = (pint_pmatch_bslice_cfg_t)((base->PMCFG & (PININT_BITSLICE_CFG_MASK << cfg_shift)) >> cfg_shift);

    if (bslice == 7U)
    {
        cfg->end_point = true;
    }
    else
    {
        cfg->end_point = (base->PMCFG & (0x1U << bslice)) >> bslice;
    }
    cfg->callback = s_pintCallback[bslice];
}

uint32_t PINT_PatternMatchResetDetectLogic(PINT_Type *base)
{
    uint32_t pmctrl;
    uint32_t pmstatus;
    uint32_t pmsrc;

    pmctrl = PINT->PMCTRL;
    pmstatus = pmctrl >> PINT_PMCTRL_PMAT_SHIFT;
    if (pmstatus)
    {
        /* Reset Pattern match engine detection logic */
        pmsrc = base->PMSRC;
        base->PMSRC = pmsrc;
    }
    return (pmstatus);
}

void PINT_EnableCallback(PINT_Type *base)
{
    uint32_t i;

    assert(base);

    PINT_PinInterruptClrStatusAll(base);
    for (i = 0; i < FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS; i++)
    {
        NVIC_ClearPendingIRQ(s_pintIRQ[i]);
        PINT_PinInterruptClrStatus(base, (pint_pin_int_t)i);
        EnableIRQ(s_pintIRQ[i]);
    }
}

void PINT_DisableCallback(PINT_Type *base)
{
    uint32_t i;

    assert(base);

    for (i = 0; i < FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS; i++)
    {
        DisableIRQ(s_pintIRQ[i]);
        PINT_PinInterruptClrStatus(base, (pint_pin_int_t)i);
        NVIC_ClearPendingIRQ(s_pintIRQ[i]);
    }
}

void PINT_Deinit(PINT_Type *base)
{
    uint32_t i;

    assert(base);

    /* Cleanup */
    PINT_DisableCallback(base);
    for (i = 0; i < FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS; i++)
    {
        s_pintCallback[i] = NULL;
    }

    /* Reset the peripheral */
    RESET_PeripheralReset(kPINT_RST_SHIFT_RSTn);

    /* Disable the peripheral clock */
    CLOCK_DisableClock(kCLOCK_Pint);
}

/* IRQ handler functions overloading weak symbols in the startup */
void PIN_INT0_DriverIRQHandler(void)
{
    uint32_t pmstatus;

    /* Reset pattern match detection */
    pmstatus = PINT_PatternMatchResetDetectLogic(PINT);
    /* Call user function */
    if (s_pintCallback[kPINT_PinInt0] != NULL)
    {
        s_pintCallback[kPINT_PinInt0](kPINT_PinInt0, pmstatus);
    }
    if((PINT->ISEL & 0x1U) == 0x0U)
    {
        /* Edge sensitive: clear Pin interrupt after callback */
        PINT_PinInterruptClrStatus(PINT, kPINT_PinInt0);
    } 
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 1U)
void PIN_INT1_DriverIRQHandler(void)
{
    uint32_t pmstatus;

    /* Reset pattern match detection */
    pmstatus = PINT_PatternMatchResetDetectLogic(PINT);
    /* Call user function */
    if (s_pintCallback[kPINT_PinInt1] != NULL)
    {
        s_pintCallback[kPINT_PinInt1](kPINT_PinInt1, pmstatus);
    }
    if((PINT->ISEL & 0x2U) == 0x0U)
    {
        /* Edge sensitive: clear Pin interrupt after callback */
        PINT_PinInterruptClrStatus(PINT, kPINT_PinInt1);
    } 
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 2U)
void PIN_INT2_DriverIRQHandler(void)
{
    uint32_t pmstatus;

    /* Reset pattern match detection */
    pmstatus = PINT_PatternMatchResetDetectLogic(PINT);
    /* Call user function */
    if (s_pintCallback[kPINT_PinInt2] != NULL)
    {
        s_pintCallback[kPINT_PinInt2](kPINT_PinInt2, pmstatus);
    }
    if((PINT->ISEL & 0x4U) == 0x0U)
    {
        /* Edge sensitive: clear Pin interrupt after callback */
        PINT_PinInterruptClrStatus(PINT, kPINT_PinInt2);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 3U)
void PIN_INT3_DriverIRQHandler(void)
{
    uint32_t pmstatus;

    /* Reset pattern match detection */
    pmstatus = PINT_PatternMatchResetDetectLogic(PINT);
    /* Call user function */
    if (s_pintCallback[kPINT_PinInt3] != NULL)
    {
        s_pintCallback[kPINT_PinInt3](kPINT_PinInt3, pmstatus);
    }
    if((PINT->ISEL & 0x8U) == 0x0U)
    {
        /* Edge sensitive: clear Pin interrupt after callback */
        PINT_PinInterruptClrStatus(PINT, kPINT_PinInt3);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 4U)
void PIN_INT4_DriverIRQHandler(void)
{
    uint32_t pmstatus;

    /* Reset pattern match detection */
    pmstatus = PINT_PatternMatchResetDetectLogic(PINT);
    /* Call user function */
    if (s_pintCallback[kPINT_PinInt4] != NULL)
    {
        s_pintCallback[kPINT_PinInt4](kPINT_PinInt4, pmstatus);
    }
    if((PINT->ISEL & 0x10U) == 0x0U)
    {
        /* Edge sensitive: clear Pin interrupt after callback */
        PINT_PinInterruptClrStatus(PINT, kPINT_PinInt4);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 5U)
void PIN_INT5_DriverIRQHandler(void)
{
    uint32_t pmstatus;

    /* Reset pattern match detection */
    pmstatus = PINT_PatternMatchResetDetectLogic(PINT);
    /* Call user function */
    if (s_pintCallback[kPINT_PinInt5] != NULL)
    {
        s_pintCallback[kPINT_PinInt5](kPINT_PinInt5, pmstatus);
    }
    if((PINT->ISEL & 0x20U) == 0x0U)
    {
        /* Edge sensitive: clear Pin interrupt after callback */
        PINT_PinInterruptClrStatus(PINT, kPINT_PinInt5);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 6U)
void PIN_INT6_DriverIRQHandler(void)
{
    uint32_t pmstatus;

    /* Reset pattern match detection */
    pmstatus = PINT_PatternMatchResetDetectLogic(PINT);
    /* Call user function */
    if (s_pintCallback[kPINT_PinInt6] != NULL)
    {
        s_pintCallback[kPINT_PinInt6](kPINT_PinInt6, pmstatus);
    }
    if((PINT->ISEL & 0x40U) == 0x0U)
    {
        /* Edge sensitive: clear Pin interrupt after callback */
        PINT_PinInterruptClrStatus(PINT, kPINT_PinInt6);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if (FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS > 7U)
void PIN_INT7_DriverIRQHandler(void)
{
    uint32_t pmstatus;

    /* Reset pattern match detection */
    pmstatus = PINT_PatternMatchResetDetectLogic(PINT);
    /* Call user function */
    if (s_pintCallback[kPINT_PinInt7] != NULL)
    {
        s_pintCallback[kPINT_PinInt7](kPINT_PinInt7, pmstatus);
    }
    if((PINT->ISEL & 0x80U) == 0x0U)
    {
        /* Edge sensitive: clear Pin interrupt after callback */
        PINT_PinInterruptClrStatus(PINT, kPINT_PinInt7);
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif
