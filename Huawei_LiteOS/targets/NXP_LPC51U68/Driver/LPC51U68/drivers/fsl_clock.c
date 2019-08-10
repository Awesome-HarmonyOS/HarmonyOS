/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016 - 2017 , NXP
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of copyright holder nor the names of its
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

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define NVALMAX (0x100U)
#define PVALMAX (0x20U)
#define MVALMAX (0x8000U)

#define PLL_MAX_N_DIV 0x100U

#define INDEX_SECTOR_TRIM48 ((uint32_t *)0x01000444U)
#define INDEX_SECTOR_TRIM96 ((uint32_t *)0x01000430U)
/*--------------------------------------------------------------------------
!!! If required these #defines can be moved to chip library file
----------------------------------------------------------------------------*/

#define PLL_SSCG0_MDEC_VAL_P (0U)                                /* MDEC is in bits  16 downto 0 */
#define PLL_SSCG0_MDEC_VAL_M (0x1FFFFUL << PLL_SSCG0_MDEC_VAL_P) /* NDEC is in bits  9 downto 0 */
#define PLL_NDEC_VAL_P (0U)                                      /* NDEC is in bits  9:0 */
#define PLL_NDEC_VAL_M (0x3FFUL << PLL_NDEC_VAL_P)
#define PLL_PDEC_VAL_P (0U) /* PDEC is in bits 6:0 */
#define PLL_PDEC_VAL_M (0x7FUL << PLL_PDEC_VAL_P)

#define PLL_MIN_CCO_FREQ_MHZ (75000000U)
#define PLL_MAX_CCO_FREQ_MHZ (150000000U)
#define PLL_LOWER_IN_LIMIT (4000U) /*!< Minimum PLL input rate */
#define PLL_MIN_IN_SSMODE (2000000U)
#define PLL_MAX_IN_SSMODE (4000000U)

/* Middle of the range values for spread-spectrum */
#define PLL_SSCG_MF_FREQ_VALUE 4U
#define PLL_SSCG_MC_COMP_VALUE 2U
#define PLL_SSCG_MR_DEPTH_VALUE 4U
#define PLL_SSCG_DITHER_VALUE 0U

/* PLL NDEC reg */
#define PLL_NDEC_VAL_SET(value) (((unsigned long)(value) << PLL_NDEC_VAL_P) & PLL_NDEC_VAL_M)
/* PLL PDEC reg */
#define PLL_PDEC_VAL_SET(value) (((unsigned long)(value) << PLL_PDEC_VAL_P) & PLL_PDEC_VAL_M)
/* SSCG control0 */
#define PLL_SSCG0_MDEC_VAL_SET(value) (((unsigned long)(value) << PLL_SSCG0_MDEC_VAL_P) & PLL_SSCG0_MDEC_VAL_M)

/* SSCG control1 */
#define PLL_SSCG1_MD_FRACT_P 0U
#define PLL_SSCG1_MD_INT_P 11U
#define PLL_SSCG1_MD_FRACT_M (0x7FFUL << PLL_SSCG1_MD_FRACT_P)
#define PLL_SSCG1_MD_INT_M (0xFFUL << PLL_SSCG1_MD_INT_P)

#define PLL_SSCG1_MD_FRACT_SET(value) (((unsigned long)(value) << PLL_SSCG1_MD_FRACT_P) & PLL_SSCG1_MD_FRACT_M)
#define PLL_SSCG1_MD_INT_SET(value) (((unsigned long)(value) << PLL_SSCG1_MD_INT_P) & PLL_SSCG1_MD_INT_M)

/* Saved value of PLL output rate, computed whenever needed to save run-time
   computation on each call to retrive the PLL rate. */
static uint32_t s_Pll_Freq;

uint32_t g_I2S_Mclk_Freq = 0U;

/** External clock rate on the CLKIN pin in Hz. If not used,
    set this to 0. Otherwise, set it to the exact rate in Hz this pin is
    being driven at. */
const uint32_t g_Ext_Clk_Freq = 0U;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Find encoded NDEC value for raw N value, max N = NVALMAX */
static uint32_t pllEncodeN(uint32_t N);
/* Find decoded N value for raw NDEC value */
static uint32_t pllDecodeN(uint32_t NDEC);
/* Find encoded PDEC value for raw P value, max P = PVALMAX */
static uint32_t pllEncodeP(uint32_t P);
/* Find decoded P value for raw PDEC value */
static uint32_t pllDecodeP(uint32_t PDEC);
/* Find encoded MDEC value for raw M value, max M = MVALMAX */
static uint32_t pllEncodeM(uint32_t M);
/* Find decoded M value for raw MDEC value */
static uint32_t pllDecodeM(uint32_t MDEC);
/* Find SELP, SELI, and SELR values for raw M value, max M = MVALMAX */
static void pllFindSel(uint32_t M, bool bypassFBDIV2, uint32_t *pSelP, uint32_t *pSelI, uint32_t *pSelR);
/* Get predivider (N) from PLL NDEC setting */
static uint32_t findPllPreDiv(uint32_t ctrlReg, uint32_t nDecReg);
/* Get postdivider (P) from PLL PDEC setting */
static uint32_t findPllPostDiv(uint32_t ctrlReg, uint32_t pDecReg);
/* Get multiplier (M) from PLL MDEC and BYPASS_FBDIV2 settings */
static uint32_t findPllMMult(uint32_t ctrlReg, uint32_t mDecReg);
/* Get the greatest common divisor */
static uint32_t FindGreatestCommonDivisor(uint32_t m, uint32_t n);
/* Set PLL output based on desired output rate */
static pll_error_t CLOCK_GetPllConfig(
    uint32_t finHz, uint32_t foutHz, pll_setup_t *pSetup, bool useFeedbackDiv2, bool useSS);
/* Update local PLL rate variable */
static void CLOCK_GetSystemPLLOutFromSetupUpdate(pll_setup_t *pSetup);

static const uint8_t wdtFreqLookup[32] = {0,  8,  12, 15, 18, 20, 24, 26, 28, 30, 32, 34, 36, 38, 40, 41,
                                          42, 44, 45, 46, 48, 49, 50, 52, 53, 54, 56, 57, 58, 59, 60, 61};
/*******************************************************************************
 * Code
 ******************************************************************************/

void CLOCK_AttachClk(clock_attach_id_t connection)
{
    bool final_descriptor = false;
    uint8_t mux;
    uint8_t pos;
    uint32_t i;
    volatile uint32_t *pClkSel;

    pClkSel = &(SYSCON->MAINCLKSELA);

    for (i = 0U; (i <= 2U) && (!final_descriptor); i++)
    {
        connection = (clock_attach_id_t)(connection >> (i * 12U)); /* pick up next descriptor */
        mux = (uint8_t)connection;
        if (connection)
        {
            pos = ((connection & 0xf00U) >> 8U) - 1U;
            if (mux == CM_ASYNCAPB)
            {
                ASYNC_SYSCON->ASYNCAPBCLKSELA = pos;
            }
            else
            {
                pClkSel[mux] = pos;
            }
        }
        else
        {
            final_descriptor = true;
        }
    }
}

void CLOCK_SetClkDiv(clock_div_name_t div_name, uint32_t divided_by_value, bool reset)
{
    volatile uint32_t *pClkDiv;

    pClkDiv = &(SYSCON->SYSTICKCLKDIV);
    if (reset)
    {
        pClkDiv[div_name] = 1U << 29U;
    }
    if (divided_by_value == 0U) /* halt */
    {
        pClkDiv[div_name] = 1U << 30U;
    }
    else
    {
        pClkDiv[div_name] = (divided_by_value - 1U);
    }
}

/* Set FRO Clocking */
status_t CLOCK_SetupFROClocking(uint32_t iFreq)
{
    uint32_t usb_adj;
    if ((iFreq != 12000000U) && (iFreq != 48000000U) && (iFreq != 96000000U))
    {
        return kStatus_Fail;
    }
    /* Power up the FRO and set this as the base clock */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);
    /* back up the value of whether USB adj is selected, in which case we will have a value of 1 else 0 */
    usb_adj = ((SYSCON->FROCTRL) & SYSCON_FROCTRL_USBCLKADJ_MASK) >> SYSCON_FROCTRL_USBCLKADJ_SHIFT;
    if (iFreq > 12000000U)
    {
        if (iFreq == 96000000U)
        {
            SYSCON->FROCTRL = ((SYSCON_FROCTRL_TRIM_MASK | SYSCON_FROCTRL_FREQTRIM_MASK) & *INDEX_SECTOR_TRIM96) |
                              SYSCON_FROCTRL_SEL(1) | SYSCON_FROCTRL_WRTRIM(1) | SYSCON_FROCTRL_USBCLKADJ(usb_adj) |
                              SYSCON_FROCTRL_HSPDCLK(1);
        }
        else
        {
            SYSCON->FROCTRL = ((SYSCON_FROCTRL_TRIM_MASK | SYSCON_FROCTRL_FREQTRIM_MASK) & *INDEX_SECTOR_TRIM48) |
                              SYSCON_FROCTRL_SEL(0) | SYSCON_FROCTRL_WRTRIM(1) | SYSCON_FROCTRL_USBCLKADJ(usb_adj) |
                              SYSCON_FROCTRL_HSPDCLK(1);
        }
    }
    else
    {
        SYSCON->FROCTRL &= ~SYSCON_FROCTRL_HSPDCLK(1);
    }

    return 0U;
}

uint32_t CLOCK_GetFro12MFreq(void)
{
    return (SYSCON->PDRUNCFG[0] & SYSCON_PDRUNCFG_PDEN_FRO_MASK) ? 0U : 12000000U;
}

uint32_t CLOCK_GetExtClkFreq(void)
{
    return (g_Ext_Clk_Freq);
}
uint32_t CLOCK_GetWdtOscFreq(void)
{
    uint8_t freq_sel, div_sel;
    if (SYSCON->PDRUNCFG[0] & (1UL << (kPDRUNCFG_PD_WDT_OSC & 0xffU)))
    {
        return 0U;
    }
    else
    {
        div_sel = ((SYSCON->WDTOSCCTRL & 0x1f) + 1) << 1;
        freq_sel =
            wdtFreqLookup[((SYSCON->WDTOSCCTRL & SYSCON_WDTOSCCTRL_FREQSEL_MASK) >> SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)];
        return ((uint32_t)freq_sel * 50000U) / ((uint32_t)div_sel);
    }
}

uint32_t CLOCK_GetFroHfFreq(void)
{
    return (SYSCON->PDRUNCFG[0] & SYSCON_PDRUNCFG_PDEN_FRO_MASK) ?
               0 :
               !(SYSCON->FROCTRL & SYSCON_FROCTRL_HSPDCLK_MASK) ? 0 : (SYSCON->FROCTRL & SYSCON_FROCTRL_SEL_MASK) ?
                                                                  96000000U :
                                                                  48000000U;
}

uint32_t CLOCK_GetPllOutFreq(void)
{
    return s_Pll_Freq;
}

uint32_t CLOCK_GetOsc32KFreq(void)
{
    return CLK_RTC_32K_CLK; /* Needs to be corrected to check that RTC Clock is enabled */
}
uint32_t CLOCK_GetCoreSysClkFreq(void)
{
    return ((SYSCON->MAINCLKSELB == 0U) && (SYSCON->MAINCLKSELA == 0U)) ?
               CLOCK_GetFro12MFreq() :
               ((SYSCON->MAINCLKSELB == 0U) && (SYSCON->MAINCLKSELA == 1U)) ?
               CLOCK_GetExtClkFreq() :
               ((SYSCON->MAINCLKSELB == 0U) && (SYSCON->MAINCLKSELA == 2U)) ?
               CLOCK_GetWdtOscFreq() :
               ((SYSCON->MAINCLKSELB == 0U) && (SYSCON->MAINCLKSELA == 3U)) ?
               CLOCK_GetFroHfFreq() :
               (SYSCON->MAINCLKSELB == 2U) ? CLOCK_GetPllOutFreq() :
                                             (SYSCON->MAINCLKSELB == 3U) ? CLOCK_GetOsc32KFreq() : 0U;
}
uint32_t CLOCK_GetI2SMClkFreq(void)
{
    return g_I2S_Mclk_Freq;
}

uint32_t CLOCK_GetAsyncApbClkFreq(void)
{
    async_clock_src_t clkSrc;
    uint32_t clkRate;

    clkSrc = CLOCK_GetAsyncApbClkSrc();

    switch (clkSrc)
    {
        case kCLOCK_AsyncMainClk:
            clkRate = CLOCK_GetCoreSysClkFreq();
            break;
        case kCLOCK_AsyncFro12Mhz:
            clkRate = CLK_FRO_12MHZ;
            break;
        default:
            clkRate = 0U;
            break;
    }

    return clkRate;
}

uint32_t CLOCK_GetFlexCommClkFreq(uint32_t id)
{
    return (SYSCON->FXCOMCLKSEL[id] == 0U) ? CLOCK_GetFro12MFreq() : (SYSCON->FXCOMCLKSEL[id] == 1U) ?
                                             CLOCK_GetFroHfFreq() :
                                             (SYSCON->FXCOMCLKSEL[id] == 2U) ?
                                             CLOCK_GetPllOutFreq() :
                                             (SYSCON->FXCOMCLKSEL[id] == 3U) ?
                                             CLOCK_GetI2SMClkFreq() :
                                             (SYSCON->FXCOMCLKSEL[id] == 4U) ? CLOCK_GetFreq(kCLOCK_Frg) : 0U;
}

uint32_t CLOCK_GetFRGInputClock(void)
{
    return (SYSCON->FRGCLKSEL == 0U) ? CLOCK_GetCoreSysClkFreq() : (SYSCON->FRGCLKSEL == 1U) ?
                                       CLOCK_GetPllOutFreq() :
                                       (SYSCON->FRGCLKSEL == 2U) ? CLOCK_GetFro12MFreq() : (SYSCON->FRGCLKSEL == 3U) ?
                                                                   CLOCK_GetFroHfFreq() :
                                                                   0U;
}

uint32_t CLOCK_SetFRGClock(uint32_t freq)
{
    uint32_t input = CLOCK_GetFRGInputClock();
    uint32_t mul;

    if ((freq > 48000000) || (freq > input) || (input / freq >= 2))
    {
        /* FRG output frequency should be less than equal to 48MHz */
        return 0;
    }
    else
    {
        mul = ((uint64_t)(input - freq) * 256) / ((uint64_t)freq);
        SYSCON->FRGCTRL = (mul << SYSCON_FRGCTRL_MULT_SHIFT) | SYSCON_FRGCTRL_DIV_MASK;
        return 1;
    }
}

uint32_t CLOCK_GetFreq(clock_name_t clockName)
{
    uint32_t freq;
    switch (clockName)
    {
        case kCLOCK_CoreSysClk:
            freq = CLOCK_GetCoreSysClkFreq();
            break;
        case kCLOCK_BusClk:
            freq = CLOCK_GetCoreSysClkFreq() / ((SYSCON->AHBCLKDIV & 0xffU) + 1U);
            break;
        case kCLOCK_FroHf:
            freq = CLOCK_GetFroHfFreq();
            break;
        case kCLOCK_Fro12M:
            freq = CLOCK_GetFro12MFreq();
            break;
        case kCLOCK_PllOut:
            freq = CLOCK_GetPllOutFreq();
            break;
        case kCLOCK_UsbClk:
            freq = (SYSCON->USBCLKSEL == 0U) ? CLOCK_GetFroHfFreq() : (SYSCON->USBCLKSEL == 1) ? CLOCK_GetPllOutFreq() :
                                                                                                 0U;
            freq = freq / ((SYSCON->USBCLKDIV & 0xffU) + 1U);
            break;
        case kClock_WdtOsc:
            freq = CLOCK_GetWdtOscFreq();
            break;
        case kCLOCK_Frg:
            freq = ((SYSCON->FRGCTRL & SYSCON_FRGCTRL_DIV_MASK) == SYSCON_FRGCTRL_DIV_MASK) ?
                       ((uint64_t)CLOCK_GetFRGInputClock() * (SYSCON_FRGCTRL_DIV_MASK + 1)) /
                           ((SYSCON_FRGCTRL_DIV_MASK + 1) +
                            ((SYSCON->FRGCTRL & SYSCON_FRGCTRL_MULT_MASK) >> SYSCON_FRGCTRL_MULT_SHIFT)) :
                       0;
            break;

        case kCLOCK_AsyncApbClk:
            freq = CLOCK_GetAsyncApbClkFreq();
            break;

        case kCLOCK_FlexI2S:
            freq = CLOCK_GetI2SMClkFreq();
            break;

        case kCLOCK_Flexcomm0:
            freq = CLOCK_GetFlexCommClkFreq(0U);
            break;
        case kCLOCK_Flexcomm1:
            freq = CLOCK_GetFlexCommClkFreq(1U);
            break;
        case kCLOCK_Flexcomm2:
            freq = CLOCK_GetFlexCommClkFreq(2U);
            break;
        case kCLOCK_Flexcomm3:
            freq = CLOCK_GetFlexCommClkFreq(3U);
            break;
        case kCLOCK_Flexcomm4:
            freq = CLOCK_GetFlexCommClkFreq(4U);
            break;
        case kCLOCK_Flexcomm5:
            freq = CLOCK_GetFlexCommClkFreq(5U);
            break;
        case kCLOCK_Flexcomm6:
            freq = CLOCK_GetFlexCommClkFreq(6U);
            break;
        case kCLOCK_Flexcomm7:
            freq = CLOCK_GetFlexCommClkFreq(7U);
            break;
        default:
            freq = 0U;
            break;
    }

    return freq;
}

/* Set the FLASH wait states for the passed frequency */
void CLOCK_SetFLASHAccessCyclesForFreq(uint32_t iFreq)
{
    if (iFreq <= 12000000U)
    {
        CLOCK_SetFLASHAccessCycles(kCLOCK_Flash1Cycle);
    }
    else if (iFreq <= 30000000U)
    {
        CLOCK_SetFLASHAccessCycles(kCLOCK_Flash2Cycle);
    }
    else if (iFreq <= 60000000U)
    {
        CLOCK_SetFLASHAccessCycles(kCLOCK_Flash3Cycle);
    }
    else if (iFreq <= 85000000U)
    {
        CLOCK_SetFLASHAccessCycles(kCLOCK_Flash4Cycle);
    }
    else
    {
        CLOCK_SetFLASHAccessCycles(kCLOCK_Flash5Cycle);
    }
}

/* Find encoded NDEC value for raw N value, max N = NVALMAX */
static uint32_t pllEncodeN(uint32_t N)
{
    uint32_t x, i;

    /* Find NDec */
    switch (N)
    {
        case 0U:
            x = 0x3FFU;
            break;

        case 1U:
            x = 0x302U;
            break;

        case 2U:
            x = 0x202U;
            break;

        default:
            x = 0x080U;
            for (i = N; i <= NVALMAX; i++)
            {
                x = (((x ^ (x >> 2U) ^ (x >> 3U) ^ (x >> 4U)) & 1U) << 7U) | ((x >> 1U) & 0x7FU);
            }
            break;
    }

    return x & (PLL_NDEC_VAL_M >> PLL_NDEC_VAL_P);
}

/* Find decoded N value for raw NDEC value */
static uint32_t pllDecodeN(uint32_t NDEC)
{
    uint32_t n, x, i;

    /* Find NDec */
    switch (NDEC)
    {
        case 0x3FFU:
            n = 0U;
            break;

        case 0x302U:
            n = 1U;
            break;

        case 0x202U:
            n = 2U;
            break;

        default:
            x = 0x080U;
            n = 0xFFFFFFFFU;
            for (i = NVALMAX; ((i >= 3U) && (n == 0xFFFFFFFFU)); i--)
            {
                x = (((x ^ (x >> 2U) ^ (x >> 3U) ^ (x >> 4U)) & 1U) << 7U) | ((x >> 1U) & 0x7FU);
                if ((x & (PLL_NDEC_VAL_M >> PLL_NDEC_VAL_P)) == NDEC)
                {
                    /* Decoded value of NDEC */
                    n = i;
                }
            }
            break;
    }

    return n;
}

/* Find encoded PDEC value for raw P value, max P = PVALMAX */
static uint32_t pllEncodeP(uint32_t P)
{
    uint32_t x, i;

    /* Find PDec */
    switch (P)
    {
        case 0U:
            x = 0x7FU;
            break;

        case 1U:
            x = 0x62U;
            break;

        case 2U:
            x = 0x42U;
            break;

        default:
            x = 0x10U;
            for (i = P; i <= PVALMAX; i++)
            {
                x = (((x ^ (x >> 2U)) & 1U) << 4U) | ((x >> 1U) & 0xFU);
            }
            break;
    }

    return x & (PLL_PDEC_VAL_M >> PLL_PDEC_VAL_P);
}

/* Find decoded P value for raw PDEC value */
static uint32_t pllDecodeP(uint32_t PDEC)
{
    uint32_t p, x, i;

    /* Find PDec */
    switch (PDEC)
    {
        case 0x7FU:
            p = 0U;
            break;

        case 0x62U:
            p = 1U;
            break;

        case 0x42U:
            p = 2U;
            break;

        default:
            x = 0x10U;
            p = 0xFFFFFFFFU;
            for (i = PVALMAX; ((i >= 3U) && (p == 0xFFFFFFFFU)); i--)
            {
                x = (((x ^ (x >> 2U)) & 1U) << 4U) | ((x >> 1U) & 0xFU);
                if ((x & (PLL_PDEC_VAL_M >> PLL_PDEC_VAL_P)) == PDEC)
                {
                    /* Decoded value of PDEC */
                    p = i;
                }
            }
            break;
    }

    return p;
}

/* Find encoded MDEC value for raw M value, max M = MVALMAX */
static uint32_t pllEncodeM(uint32_t M)
{
    uint32_t i, x;

    /* Find MDec */
    switch (M)
    {
        case 0U:
            x = 0x1FFFFU;
            break;

        case 1U:
            x = 0x18003U;
            break;

        case 2U:
            x = 0x10003U;
            break;

        default:
            x = 0x04000U;
            for (i = M; i <= MVALMAX; i++)
            {
                x = (((x ^ (x >> 1U)) & 1U) << 14U) | ((x >> 1U) & 0x3FFFU);
            }
            break;
    }

    return x & (PLL_SSCG0_MDEC_VAL_M >> PLL_SSCG0_MDEC_VAL_P);
}

/* Find decoded M value for raw MDEC value */
static uint32_t pllDecodeM(uint32_t MDEC)
{
    uint32_t m, i, x;

    /* Find MDec */
    switch (MDEC)
    {
        case 0x1FFFFU:
            m = 0U;
            break;

        case 0x18003U:
            m = 1U;
            break;

        case 0x10003U:
            m = 2U;
            break;

        default:
            x = 0x04000U;
            m = 0xFFFFFFFFU;
            for (i = MVALMAX; ((i >= 3U) && (m == 0xFFFFFFFFU)); i--)
            {
                x = (((x ^ (x >> 1U)) & 1) << 14U) | ((x >> 1U) & 0x3FFFU);
                if ((x & (PLL_SSCG0_MDEC_VAL_M >> PLL_SSCG0_MDEC_VAL_P)) == MDEC)
                {
                    /* Decoded value of MDEC */
                    m = i;
                }
            }
            break;
    }

    return m;
}

/* Find SELP, SELI, and SELR values for raw M value, max M = MVALMAX */
static void pllFindSel(uint32_t M, bool bypassFBDIV2, uint32_t *pSelP, uint32_t *pSelI, uint32_t *pSelR)
{
    /* bandwidth: compute selP from Multiplier */
    if (M < 60U)
    {
        *pSelP = (M >> 1U) + 1U;
    }
    else
    {
        *pSelP = PVALMAX - 1U;
    }

    /* bandwidth: compute selI from Multiplier */
    if (M > 16384U)
    {
        *pSelI = 1U;
    }
    else if (M > 8192U)
    {
        *pSelI = 2U;
    }
    else if (M > 2048U)
    {
        *pSelI = 4U;
    }
    else if (M >= 501U)
    {
        *pSelI = 8U;
    }
    else if (M >= 60U)
    {
        *pSelI = 4U * (1024U / (M + 9U));
    }
    else
    {
        *pSelI = (M & 0x3CU) + 4U;
    }

    if (*pSelI > ((0x3FUL << SYSCON_SYSPLLCTRL_SELI_SHIFT) >> SYSCON_SYSPLLCTRL_SELI_SHIFT))
    {
        *pSelI = ((0x3FUL << SYSCON_SYSPLLCTRL_SELI_SHIFT) >> SYSCON_SYSPLLCTRL_SELI_SHIFT);
    }

    *pSelR = 0U;
}

/* Get predivider (N) from PLL NDEC setting */
static uint32_t findPllPreDiv(uint32_t ctrlReg, uint32_t nDecReg)
{
    uint32_t preDiv = 1;

    /* Direct input is not used? */
    if ((ctrlReg & (1UL << SYSCON_SYSPLLCTRL_DIRECTI_SHIFT)) == 0U)
    {
        /* Decode NDEC value to get (N) pre divider */
        preDiv = pllDecodeN(nDecReg & 0x3FFU);
        if (preDiv == 0U)
        {
            preDiv = 1U;
        }
    }

    /* Adjusted by 1, directi is used to bypass */
    return preDiv;
}

/* Get postdivider (P) from PLL PDEC setting */
static uint32_t findPllPostDiv(uint32_t ctrlReg, uint32_t pDecReg)
{
    uint32_t postDiv = 1U;

    /* Direct input is not used? */
    if ((ctrlReg & SYSCON_SYSPLLCTRL_DIRECTO_MASK) == 0U)
    {
        /* Decode PDEC value to get (P) post divider */
        postDiv = 2U * pllDecodeP(pDecReg & 0x7FU);
        if (postDiv == 0U)
        {
            postDiv = 2U;
        }
    }

    /* Adjusted by 1, directo is used to bypass */
    return postDiv;
}

/* Get multiplier (M) from PLL MDEC and BYPASS_FBDIV2 settings */
static uint32_t findPllMMult(uint32_t ctrlReg, uint32_t mDecReg)
{
    uint32_t mMult = 1U;

    /* Decode MDEC value to get (M) multiplier */
    mMult = pllDecodeM(mDecReg & 0x1FFFFU);

    /* Extra multiply by 2 needed? */
    if ((ctrlReg & (SYSCON_SYSPLLCTRL_BYPASSCCODIV2_MASK)) == 0U)
    {
        mMult = mMult << 1U;
    }

    if (mMult == 0U)
    {
        mMult = 1U;
    }

    return mMult;
}

static uint32_t FindGreatestCommonDivisor(uint32_t m, uint32_t n)
{
    uint32_t tmp;

    while (n != 0U)
    {
        tmp = n;
        n = m % n;
        m = tmp;
    }

    return m;
}

/*
 * Set PLL output based on desired output rate.
 * In this function, the it calculates the PLL setting for output frequency from input clock
 * frequency. The calculation would cost a few time. So it is not recommaned to use it frequently.
 * the "pllctrl", "pllndec", "pllpdec", "pllmdec" would updated in this function.
 */
static pll_error_t CLOCK_GetPllConfigInternal(
    uint32_t finHz, uint32_t foutHz, pll_setup_t *pSetup, bool useFeedbackDiv2, bool useSS)
{
    uint32_t nDivOutHz, fccoHz, multFccoDiv;
    uint32_t pllPreDivider, pllMultiplier, pllBypassFBDIV2, pllPostDivider;
    uint32_t pllDirectInput, pllDirectOutput;
    uint32_t pllSelP, pllSelI, pllSelR, bandsel, uplimoff;

    /* Baseline parameters (no input or output dividers) */
    pllPreDivider = 1U;  /* 1 implies pre-divider will be disabled */
    pllPostDivider = 0U; /* 0 implies post-divider will be disabled */
    pllDirectOutput = 1U;
    if (useFeedbackDiv2)
    {
        /* Using feedback divider for M, so disable bypass */
        pllBypassFBDIV2 = 0U;
    }
    else
    {
        pllBypassFBDIV2 = 1U;
    }
    multFccoDiv = (2U - pllBypassFBDIV2);

    /* Verify output rate parameter */
    if (foutHz > PLL_MAX_CCO_FREQ_MHZ)
    {
        /* Maximum PLL output with post divider=1 cannot go above this frequency */
        return kStatus_PLL_OutputTooHigh;
    }
    if (foutHz < (PLL_MIN_CCO_FREQ_MHZ / (PVALMAX << 1U)))
    {
        /* Minmum PLL output with maximum post divider cannot go below this frequency */
        return kStatus_PLL_OutputTooLow;
    }

    /* If using SS mode, input clock needs to be between 2MHz and 4MHz */
    if (useSS)
    {
        /* Verify input rate parameter */
        if (finHz < PLL_MIN_IN_SSMODE)
        {
            /* Input clock into the PLL cannot be lower than this */
            return kStatus_PLL_InputTooLow;
        }
        /* PLL input in SS mode must be under 4MHz */
        pllPreDivider = finHz / ((PLL_MIN_IN_SSMODE + PLL_MAX_IN_SSMODE) / 2);
        if (pllPreDivider > NVALMAX)
        {
            return kStatus_PLL_InputTooHigh;
        }
    }
    else
    {
        /* Verify input rate parameter */
        if (finHz < PLL_LOWER_IN_LIMIT)
        {
            /* Input clock into the PLL cannot be lower than this */
            return kStatus_PLL_InputTooLow;
        }
    }

    /* Find the optimal CCO frequency for the output and input that
       will keep it inside the PLL CCO range. This may require
       tweaking the post-divider for the PLL. */
    fccoHz = foutHz;
    while (fccoHz < PLL_MIN_CCO_FREQ_MHZ)
    {
        /* CCO output is less than minimum CCO range, so the CCO output
           needs to be bumped up and the post-divider is used to bring
           the PLL output back down. */
        pllPostDivider++;
        if (pllPostDivider > PVALMAX)
        {
            return kStatus_PLL_OutsideIntLimit;
        }

        /* Target CCO goes up, PLL output goes down */
        fccoHz = foutHz * (pllPostDivider * 2U);
        pllDirectOutput = 0U;
    }

    /* Determine if a pre-divider is needed to get the best frequency */
    if ((finHz > PLL_LOWER_IN_LIMIT) && (fccoHz >= finHz) && (useSS == false))
    {
        uint32_t a = FindGreatestCommonDivisor(fccoHz, (multFccoDiv * finHz));

        if (a > 20000U)
        {
            a = (multFccoDiv * finHz) / a;
            if ((a != 0U) && (a < PLL_MAX_N_DIV))
            {
                pllPreDivider = a;
            }
        }
    }

    /* Bypass pre-divider hardware if pre-divider is 1 */
    if (pllPreDivider > 1U)
    {
        pllDirectInput = 0U;
    }
    else
    {
        pllDirectInput = 1U;
    }

    /* Determine PLL multipler */
    nDivOutHz = (finHz / pllPreDivider);
    pllMultiplier = (fccoHz / nDivOutHz) / multFccoDiv;

    /* Find optimal values for filter */
    if (useSS == false)
    {
        /* Will bumping up M by 1 get us closer to the desired CCO frequency? */
        if ((nDivOutHz * ((multFccoDiv * pllMultiplier * 2U) + 1U)) < (fccoHz * 2U))
        {
            pllMultiplier++;
        }

        /* Setup filtering */
        pllFindSel(pllMultiplier, pllBypassFBDIV2, &pllSelP, &pllSelI, &pllSelR);
        bandsel = 1U;
        uplimoff = 0U;

        /* Get encoded value for M (mult) and use manual filter, disable SS mode */
        pSetup->syspllssctrl[0] =
            (PLL_SSCG0_MDEC_VAL_SET(pllEncodeM(pllMultiplier)) | (1U << SYSCON_SYSPLLSSCTRL0_SEL_EXT_SHIFT));

        /* Power down SSC, not used */
        pSetup->syspllssctrl[1] = (1U << SYSCON_SYSPLLSSCTRL1_PD_SHIFT);
    }
    else
    {
        uint64_t fc;

        /* Filtering will be handled by SSC */
        pllSelR = pllSelI = pllSelP = 0U;
        bandsel = 0U;
        uplimoff = 1U;

        /* The PLL multiplier will get very close and slightly under the
           desired target frequency. A small fractional component can be
           added to fine tune the frequency upwards to the target. */
        fc = ((uint64_t)(fccoHz % (multFccoDiv * nDivOutHz)) << 11U) / (multFccoDiv * nDivOutHz);

        /* MDEC set by SSC */
        pSetup->syspllssctrl[0U] = 0U;

        /* Set multiplier */
        pSetup->syspllssctrl[1] = PLL_SSCG1_MD_INT_SET(pllMultiplier) | PLL_SSCG1_MD_FRACT_SET((uint32_t)fc);
    }

    /* Get encoded values for N (prediv) and P (postdiv) */
    pSetup->syspllndec = PLL_NDEC_VAL_SET(pllEncodeN(pllPreDivider));
    pSetup->syspllpdec = PLL_PDEC_VAL_SET(pllEncodeP(pllPostDivider));

    /* PLL control */
    pSetup->syspllctrl = (pllSelR << SYSCON_SYSPLLCTRL_SELR_SHIFT) |                  /* Filter coefficient */
                         (pllSelI << SYSCON_SYSPLLCTRL_SELI_SHIFT) |                  /* Filter coefficient */
                         (pllSelP << SYSCON_SYSPLLCTRL_SELP_SHIFT) |                  /* Filter coefficient */
                         (0 << SYSCON_SYSPLLCTRL_BYPASS_SHIFT) |                      /* PLL bypass mode disabled */
                         (pllBypassFBDIV2 << SYSCON_SYSPLLCTRL_BYPASSCCODIV2_SHIFT) | /* Extra M / 2 divider? */
                         (uplimoff << SYSCON_SYSPLLCTRL_UPLIMOFF_SHIFT) |             /* SS/fractional mode disabled */
                         (bandsel << SYSCON_SYSPLLCTRL_BANDSEL_SHIFT) |        /* Manual bandwidth selection enabled */
                         (pllDirectInput << SYSCON_SYSPLLCTRL_DIRECTI_SHIFT) | /* Bypass pre-divider? */
                         (pllDirectOutput << SYSCON_SYSPLLCTRL_DIRECTO_SHIFT); /* Bypass post-divider? */

    return kStatus_PLL_Success;
}

#if (defined(CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT) && CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT)
/* Alloct the static buffer for cache. */
pll_setup_t gPllSetupCacheStruct[CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT];
uint32_t gFinHzCache[CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT] = {0};
uint32_t gFoutHzCache[CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT] = {0};
bool gUseFeedbackDiv2Cache[CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT] = {false};
bool gUseSSCache[CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT] = {false};
uint32_t gPllSetupCacheIdx = 0U;
#endif /* CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT */

/*
 * Calculate the PLL setting values from input clock freq to output freq.
 */
static pll_error_t CLOCK_GetPllConfig(
    uint32_t finHz, uint32_t foutHz, pll_setup_t *pSetup, bool useFeedbackDiv2, bool useSS)
{
    pll_error_t retErr;
#if (defined(CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT) && CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT)
    uint32_t i;

    for (i = 0U; i < CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT; i++)
    {
        if (   (finHz == gFinHzCache[i])
            && (foutHz == gFoutHzCache[i]) 
            && (useFeedbackDiv2 == gUseFeedbackDiv2Cache[i])
            && (useSS == gUseSSCache[i]) )
        {
            /* Hit the target in cache buffer. */
            pSetup->syspllctrl = gPllSetupCacheStruct[i].syspllctrl;
            pSetup->syspllndec = gPllSetupCacheStruct[i].syspllndec;
            pSetup->syspllpdec = gPllSetupCacheStruct[i].syspllpdec;
            pSetup->syspllssctrl[0] = gPllSetupCacheStruct[i].syspllssctrl[0];
            pSetup->syspllssctrl[1] = gPllSetupCacheStruct[i].syspllssctrl[1];
            retErr = kStatus_PLL_Success;
        }
    }

    if (i < CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT)
    {
        return retErr;
    }
#endif /* CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT */

    retErr = CLOCK_GetPllConfigInternal( finHz, foutHz, pSetup, useFeedbackDiv2, useSS);

#if (defined(CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT) && CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT)
    /* Cache the most recent calulation result into buffer. */
    gFinHzCache[gPllSetupCacheIdx] = finHz;
    gFoutHzCache[gPllSetupCacheIdx] = foutHz;
    gUseFeedbackDiv2Cache[gPllSetupCacheIdx] = useFeedbackDiv2;
    gUseSSCache[gPllSetupCacheIdx] = useSS;

    gPllSetupCacheStruct[gPllSetupCacheIdx].syspllctrl = pSetup->syspllctrl;
    gPllSetupCacheStruct[gPllSetupCacheIdx].syspllndec = pSetup->syspllndec;
    gPllSetupCacheStruct[gPllSetupCacheIdx].syspllpdec = pSetup->syspllpdec;
    gPllSetupCacheStruct[gPllSetupCacheIdx].syspllssctrl[0] = pSetup->syspllssctrl[0];
    gPllSetupCacheStruct[gPllSetupCacheIdx].syspllssctrl[1] = pSetup->syspllssctrl[1];
    /* Update the index for next available buffer. */
    gPllSetupCacheIdx = (gPllSetupCacheIdx + 1U) % CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT;
#endif /* CLOCK_USR_CFG_PLL_CONFIG_CACHE_COUNT */

    return retErr;
}

/* Update local PLL rate variable */
static void CLOCK_GetSystemPLLOutFromSetupUpdate(pll_setup_t *pSetup)
{
    s_Pll_Freq = CLOCK_GetSystemPLLOutFromSetup(pSetup);
}

/* Return System PLL input clock rate */
uint32_t CLOCK_GetSystemPLLInClockRate(void)
{
    uint32_t clkRate = 0U;

    switch ((SYSCON->SYSPLLCLKSEL & SYSCON_SYSPLLCLKSEL_SEL_MASK))
    {
        case 0x00U:
            clkRate = CLK_FRO_12MHZ;
            break;

        case 0x01U:
            clkRate = CLOCK_GetExtClkFreq();
            break;

        case 0x02U:
            clkRate = CLOCK_GetWdtOscFreq();
            break;

        case 0x03U:
            clkRate = CLOCK_GetOsc32KFreq();
            break;

        default:
            clkRate = 0U;
            break;
    }

    return clkRate;
}

/* Return System PLL output clock rate from setup structure */
uint32_t CLOCK_GetSystemPLLOutFromSetup(pll_setup_t *pSetup)
{
    uint32_t prediv, postdiv, mMult, inPllRate;
    uint64_t workRate;

    /* Get the input clock frequency of PLL. */
    inPllRate = CLOCK_GetSystemPLLInClockRate();

    /*
    * If the PLL is bypassed, PLL would not be used and the output of PLL module would just be the input clock.
    */
    if ((pSetup->syspllctrl & (SYSCON_SYSPLLCTRL_BYPASS_MASK)) == 0U)
    {
        /* PLL is not in bypass mode, get pre-divider, and M divider, post-divider. */
        /*
         * 1. Pre-divider
         * Pre-divider is only available when the DIRECTI is disabled.
         */
        if (0U == (pSetup->syspllctrl & SYSCON_SYSPLLCTRL_DIRECTI_MASK))
        {
            prediv = findPllPreDiv(pSetup->syspllctrl, pSetup->syspllndec);
        }
        else
        {
            prediv = 1U; /* The pre-divider is bypassed. */
        }
        /* Adjust input clock */
        inPllRate = inPllRate / prediv;

        /*
         * 2. M divider
         * If using the SS, use the multiplier.
         */
        if (pSetup->syspllssctrl[1] & (SYSCON_SYSPLLSSCTRL1_PD_MASK))
        {
            /* MDEC used for rate */
            mMult = findPllMMult(pSetup->syspllctrl, pSetup->syspllssctrl[0]);
            workRate = (uint64_t)inPllRate * (uint64_t)mMult;
        }
        else
        {
            uint64_t fract;

            /* SS multipler used for rate */
            mMult = (pSetup->syspllssctrl[1] & PLL_SSCG1_MD_INT_M) >> PLL_SSCG1_MD_INT_P;
            workRate = (uint64_t)inPllRate * (uint64_t)mMult;

            /* Adjust by fractional */
            fract = (uint64_t)(pSetup->syspllssctrl[1] & PLL_SSCG1_MD_FRACT_M) >> PLL_SSCG1_MD_FRACT_P;
            workRate = workRate + ((inPllRate * fract) / 0x800U);
        }

        /*
         * 3. Post-divider
         * Post-divider is only available when the DIRECTO is disabled.
         */
        if (0U == (pSetup->syspllctrl & SYSCON_SYSPLLCTRL_DIRECTO_MASK))
        {
            postdiv = findPllPostDiv(pSetup->syspllctrl, pSetup->syspllpdec);
        }
        else
        {
            postdiv = 1U; /* The post-divider is bypassed. */
        }
        workRate = workRate / ((uint64_t)postdiv);
    }
    else
    {
        /* In bypass mode */
        workRate = (uint64_t)inPllRate;
    }

    return (uint32_t)workRate;
}

/* Set the current PLL Rate */
void CLOCK_SetStoredPLLClockRate(uint32_t rate)
{
    s_Pll_Freq = rate;
}

/* Return System PLL output clock rate */
uint32_t CLOCK_GetSystemPLLOutClockRate(bool recompute)
{
    pll_setup_t Setup;
    uint32_t rate;

    if ((recompute) || (s_Pll_Freq == 0U))
    {
        Setup.syspllctrl = SYSCON->SYSPLLCTRL;
        Setup.syspllndec = SYSCON->SYSPLLNDEC;
        Setup.syspllpdec = SYSCON->SYSPLLPDEC;
        Setup.syspllssctrl[0] = SYSCON->SYSPLLSSCTRL0;
        Setup.syspllssctrl[1] = SYSCON->SYSPLLSSCTRL1;

        CLOCK_GetSystemPLLOutFromSetupUpdate(&Setup);
    }

    rate = s_Pll_Freq;

    return rate;
}

/* Set PLL output based on the passed PLL setup data */
pll_error_t CLOCK_SetupPLLData(pll_config_t *pControl, pll_setup_t *pSetup)
{
    uint32_t inRate;
    bool useSS = (bool)((pControl->flags & PLL_CONFIGFLAG_FORCENOFRACT) == 0U);
    bool useFbDiv2;

    pll_error_t pllError;

    /* Determine input rate for the PLL */
    if ((pControl->flags & PLL_CONFIGFLAG_USEINRATE) != 0U)
    {
        inRate = pControl->inputRate;
    }
    else
    {
        inRate = CLOCK_GetSystemPLLInClockRate();
    }

    if ((pSetup->flags & PLL_SETUPFLAG_USEFEEDBACKDIV2) != 0U)
    {
        useFbDiv2 = true;
    }
    else
    {
        useFbDiv2 = false;
    }

    /* PLL flag options */
    pllError = CLOCK_GetPllConfig(inRate, pControl->desiredRate, pSetup, useFbDiv2, useSS);
    if ((useSS) && (pllError == kStatus_PLL_Success))
    {
        /* If using SS mode, then some tweaks are made to the generated setup */
        pSetup->syspllssctrl[1] |= (uint32_t)pControl->ss_mf | (uint32_t)pControl->ss_mr | (uint32_t)pControl->ss_mc;
        if (pControl->mfDither)
        {
            pSetup->syspllssctrl[1] |= (1U << SYSCON_SYSPLLSSCTRL1_DITHER_SHIFT);
        }
    }

    return pllError;
}

/* Set PLL output from PLL setup structure */
pll_error_t CLOCK_SetupSystemPLLPrec(pll_setup_t *pSetup, uint32_t flagcfg)
{
    /* Power off PLL during setup changes */
    POWER_EnablePD(kPDRUNCFG_PD_SYS_PLL0);

    pSetup->flags = flagcfg;

    /* Write PLL setup data */
    SYSCON->SYSPLLCTRL = pSetup->syspllctrl;
    SYSCON->SYSPLLNDEC = pSetup->syspllndec;
    SYSCON->SYSPLLNDEC = pSetup->syspllndec | (1U << SYSCON_SYSPLLNDEC_NREQ_SHIFT); /* latch */
    SYSCON->SYSPLLPDEC = pSetup->syspllpdec;
    SYSCON->SYSPLLPDEC = pSetup->syspllpdec | (1U << SYSCON_SYSPLLPDEC_PREQ_SHIFT); /* latch */
    SYSCON->SYSPLLSSCTRL0 = pSetup->syspllssctrl[0];
    SYSCON->SYSPLLSSCTRL0 = pSetup->syspllssctrl[0] | (1U << SYSCON_SYSPLLSSCTRL0_MREQ_SHIFT); /* latch */
    SYSCON->SYSPLLSSCTRL1 = pSetup->syspllssctrl[1];
    SYSCON->SYSPLLSSCTRL1 = pSetup->syspllssctrl[1] | (1U << SYSCON_SYSPLLSSCTRL1_MDREQ_SHIFT); /* latch */

    /* Flags for lock or power on */
    if ((pSetup->flags & (PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_WAITLOCK)) != 0U)
    {
        /* If turning the PLL back on, perform the following sequence to accelerate PLL lock */
        volatile uint32_t delayX;
        uint32_t maxCCO = (1U << 18U) | 0x5dd2U; /* CCO = 1.6Ghz + MDEC enabled*/
        uint32_t curSSCTRL = SYSCON->SYSPLLSSCTRL0 & ~(1U << 17U);

        /* Initialize  and power up PLL */
        SYSCON->SYSPLLSSCTRL0 = maxCCO;
        POWER_DisablePD(kPDRUNCFG_PD_SYS_PLL0);

        /* Set mreq to activate */
        SYSCON->SYSPLLSSCTRL0 = maxCCO | (1U << 17U);

        /* Delay for 72 uSec @ 12Mhz */
        for (delayX = 0U; delayX < 172U; ++delayX)
        {
        }

        /* clear mreq to prepare for restoring mreq */
        SYSCON->SYSPLLSSCTRL0 = curSSCTRL;

        /* set original value back and activate */
        SYSCON->SYSPLLSSCTRL0 = curSSCTRL | (1U << 17U);

        /* Enable peripheral states by setting low */
        POWER_DisablePD(kPDRUNCFG_PD_SYS_PLL0);
    }
    if ((pSetup->flags & PLL_SETUPFLAG_WAITLOCK) != 0U)
    {
        while (CLOCK_IsSystemPLLLocked() == false)
        {
        }
    }

    /* Update current programmed PLL rate var */
    CLOCK_GetSystemPLLOutFromSetupUpdate(pSetup);

    /* System voltage adjustment, occurs prior to setting main system clock */
    if ((pSetup->flags & PLL_SETUPFLAG_ADGVOLT) != 0U)
    {
        POWER_SetVoltageForFreq(s_Pll_Freq);
    }

    return kStatus_PLL_Success;
}

/* Setup PLL Frequency from pre-calculated value */
pll_error_t CLOCK_SetPLLFreq(const pll_setup_t *pSetup)
{
    /* Power off PLL during setup changes */
    POWER_EnablePD(kPDRUNCFG_PD_SYS_PLL0);

    /* Write PLL setup data */
    SYSCON->SYSPLLCTRL = pSetup->syspllctrl;
    SYSCON->SYSPLLNDEC = pSetup->syspllndec;
    SYSCON->SYSPLLNDEC = pSetup->syspllndec | (1U << SYSCON_SYSPLLNDEC_NREQ_SHIFT); /* latch */
    SYSCON->SYSPLLPDEC = pSetup->syspllpdec;
    SYSCON->SYSPLLPDEC = pSetup->syspllpdec | (1U << SYSCON_SYSPLLPDEC_PREQ_SHIFT); /* latch */
    SYSCON->SYSPLLSSCTRL0 = pSetup->syspllssctrl[0];
    SYSCON->SYSPLLSSCTRL0 = pSetup->syspllssctrl[0] | (1U << SYSCON_SYSPLLSSCTRL0_MREQ_SHIFT); /* latch */
    SYSCON->SYSPLLSSCTRL1 = pSetup->syspllssctrl[1];
    SYSCON->SYSPLLSSCTRL1 = pSetup->syspllssctrl[1] | (1U << SYSCON_SYSPLLSSCTRL1_MDREQ_SHIFT); /* latch */

    /* Flags for lock or power on */
    if ((pSetup->flags & (PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_WAITLOCK)) != 0)
    {
        /* If turning the PLL back on, perform the following sequence to accelerate PLL lock */
        volatile uint32_t delayX;
        uint32_t maxCCO = (1U << 18U) | 0x5dd2U; /* CCO = 1.6Ghz + MDEC enabled*/
        uint32_t curSSCTRL = SYSCON->SYSPLLSSCTRL0 & ~(1U << 17U);

        /* Initialize  and power up PLL */
        SYSCON->SYSPLLSSCTRL0 = maxCCO;
        POWER_DisablePD(kPDRUNCFG_PD_SYS_PLL0);

        /* Set mreq to activate */
        SYSCON->SYSPLLSSCTRL0 = maxCCO | (1U << 17U);

        /* Delay for 72 uSec @ 12Mhz */
        for (delayX = 0U; delayX < 172U; ++delayX)
        {
        }

        /* clear mreq to prepare for restoring mreq */
        SYSCON->SYSPLLSSCTRL0 = curSSCTRL;

        /* set original value back and activate */
        SYSCON->SYSPLLSSCTRL0 = curSSCTRL | (1U << 17U);

        /* Enable peripheral states by setting low */
        POWER_DisablePD(kPDRUNCFG_PD_SYS_PLL0);
    }
    if ((pSetup->flags & PLL_SETUPFLAG_WAITLOCK) != 0U)
    {
        while (CLOCK_IsSystemPLLLocked() == false)
        {
        }
    }

    /* Update current programmed PLL rate var */
    s_Pll_Freq = pSetup->pllRate;

    return kStatus_PLL_Success;
}

/* Set System PLL clock based on the input frequency and multiplier */
void CLOCK_SetupSystemPLLMult(uint32_t multiply_by, uint32_t input_freq)
{
    uint32_t cco_freq = input_freq * multiply_by;
    uint32_t pdec = 1U;
    uint32_t selr;
    uint32_t seli;
    uint32_t selp;
    uint32_t mdec, ndec;

    uint32_t directo = SYSCON_SYSPLLCTRL_DIRECTO(1);

    while (cco_freq < 75000000U)
    {
        multiply_by <<= 1U; /* double value in each iteration */
        pdec <<= 1U;        /* correspondingly double pdec to cancel effect of double msel */
        cco_freq = input_freq * multiply_by;
    }
    selr = 0U;
    if (multiply_by < 60U)
    {
        seli = (multiply_by & 0x3cU) + 4U;
        selp = (multiply_by >> 1U) + 1U;
    }
    else
    {
        selp = 31U;
        if (multiply_by > 16384U)
        {
            seli = 1U;
        }
        else if (multiply_by > 8192U)
        {
            seli = 2U;
        }
        else if (multiply_by > 2048U)
        {
            seli = 4U;
        }
        else if (multiply_by >= 501U)
        {
            seli = 8U;
        }
        else
        {
            seli = 4U * (1024U / (multiply_by + 9U));
        }
    }

    if (pdec > 1U)
    {
        directo = 0U;     /* use post divider */
        pdec = pdec / 2U; /* Account for minus 1 encoding */
                          /* Translate P value */
        switch (pdec)
        {
            case 1U:
                pdec = 0x62U; /* 1  * 2 */
                break;
            case 2U:
                pdec = 0x42U; /* 2  * 2 */
                break;
            case 4U:
                pdec = 0x02U; /* 4  * 2 */
                break;
            case 8U:
                pdec = 0x0bU; /* 8  * 2 */
                break;
            case 16U:
                pdec = 0x11U; /* 16 * 2 */
                break;
            case 32U:
                pdec = 0x08U; /* 32 * 2 */
                break;
            default:
                pdec = 0x08U;
                break;
        }
    }

    mdec = PLL_SSCG0_MDEC_VAL_SET(pllEncodeM(multiply_by));
    ndec = 0x302U; /* pre divide by 1 (hardcoded) */

    SYSCON->SYSPLLCTRL = SYSCON_SYSPLLCTRL_BANDSEL(1) | directo | SYSCON_SYSPLLCTRL_BYPASSCCODIV2(1) |
                         (selr << SYSCON_SYSPLLCTRL_SELR_SHIFT) | (seli << SYSCON_SYSPLLCTRL_SELI_SHIFT) |
                         (selp << SYSCON_SYSPLLCTRL_SELP_SHIFT);
    SYSCON->SYSPLLPDEC = pdec | (1U << 7U);  /* set Pdec value and assert preq */
    SYSCON->SYSPLLNDEC = ndec | (1U << 10U); /* set Pdec value and assert preq */
    SYSCON->SYSPLLSSCTRL0 =
        (1U << 18U) | (1U << 17U) | mdec; /* select non sscg MDEC value, assert mreq and select mdec value */
}
bool CLOCK_EnableUsbfs0Clock(clock_usb_src_t src, uint32_t freq)
{
    bool ret = true;

    CLOCK_DisableClock(kCLOCK_Usbd0);

    if (kCLOCK_UsbSrcFro == src)
    {
        switch (freq)
        {
            case 96000000U:
                CLOCK_SetClkDiv(kCLOCK_DivUsbClk, 2, false); /*!< Div by 2 to get 48MHz, no divider reset */
                break;
            case 48000000U:
                CLOCK_SetClkDiv(kCLOCK_DivUsbClk, 1, false); /*!< Div by 1 to get 48MHz, no divider reset */
                break;
            default:
                ret = false;
                break;
        }
        /* Turn ON FRO HF and let it adjust TRIM value based on USB SOF */
        SYSCON->FROCTRL = (SYSCON->FROCTRL & ~((0x01U << 15U) | (0xFU << 26U))) | SYSCON_FROCTRL_HSPDCLK_MASK |
                          SYSCON_FROCTRL_USBCLKADJ_MASK;
        /* select FRO 96 or 48 MHz */
        CLOCK_AttachClk(kFRO_HF_to_USB_CLK);
    }
    else
    {
        /*TODO , we only implement FRO as usb clock source*/
        ret = false;
    }

    CLOCK_EnableClock(kCLOCK_Usbd0);

    return ret;
}
