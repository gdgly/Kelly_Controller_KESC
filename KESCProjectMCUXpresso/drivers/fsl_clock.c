/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2019 , NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_clock.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.clock"
#endif

#define ICS_C2_BDIV_VAL ((ICS->C2 & ICS_C2_BDIV_MASK) >> ICS_C2_BDIV_SHIFT)
#define ICS_S_CLKST_VAL ((ICS->S & ICS_S_CLKST_MASK) >> ICS_S_CLKST_SHIFT)
#define ICS_S_IREFST_VAL ((ICS->S & ICS_S_IREFST_MASK) >> ICS_S_IREFST_SHIFT)
#define ICS_C1_RDIV_VAL ((ICS->C1 & ICS_C1_RDIV_MASK) >> ICS_C1_RDIV_SHIFT)
#define OSC_CR_RANGE_VAL ((OSC0->CR & OSC_CR_RANGE_MASK) >> OSC_CR_RANGE_SHIFT)
#define OSC_MODE_MASK \
    (OSC_CR_OSCOS_MASK | OSC_CR_HGO_MASK | OSC_CR_RANGE_MASK | OSC_CR_OSCEN_MASK | OSC_CR_OSCSTEN_MASK)
#define ICS_C2_LP_VAL ((ICS->C2 & ICS_C2_LP_MASK) >> ICS_C2_LP_SHIFT)
#define SIM_BUSDIV_VAL ((SIM->BUSDIV & SIM_BUSDIV_BUSDIV_MASK) >> SIM_BUSDIV_BUSDIV_SHIFT)

/* ICS_S_CLKST definition. */
enum _ics_clkout_stat
{
    kICS_ClkOutStatFll, /* FLL.            */
    kICS_ClkOutStatInt, /* Internal clock. */
    kICS_ClkOutStatExt, /* External clock. */
};

/* ICS fll clock factor. */
#define ICS_FLL_CLOCK_FACTOR (1024U)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Slow internal reference clock frequency. */
static uint32_t s_slowIrcFreq = 32000U;

/* External XTAL0 (OSC0) clock frequency. */
volatile uint32_t g_xtal0Freq;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get the ICS external reference clock frequency.
 *
 * Get the current ICS external reference clock frequency in Hz.This is an internal function.
 *
 * @return ICS external reference clock frequency in Hz.
 */
static uint32_t CLOCK_GetICSExtClkFreq(void);

/*!
 * @brief Get the ICS FLL external reference clock frequency.
 *
 * Get the current ICS FLL external reference clock frequency in Hz. It is
 * the frequency after by ICS_C1[RDIV]. This is an internal function.
 *
 * @return ICS FLL external reference clock frequency in Hz.
 */
static uint32_t CLOCK_GetFllExtRefClkFreq(void);

/*!
 * @brief Get the ICS FLL reference clock frequency.
 *
 * Get the current ICS FLL reference clock frequency in Hz. It is
 * the frequency select by ICS_C1[IREFS]. This is an internal function.
 *
 * @return ICS FLL reference clock frequency in Hz.
 */
static uint32_t CLOCK_GetFllRefClkFreq(void);

/*!
 * @brief Calculate the RANGE value base on crystal frequency.
 *
 * To setup external crystal oscillator, must set the register bits RANGE
 * base on the crystal frequency. This function returns the RANGE base on the
 * input frequency. This is an internal function.
 *
 * @param freq Crystal frequency in Hz.
 * @return The RANGE value.
 */
static uint8_t CLOCK_GetOscRangeFromFreq(uint32_t freq);

/*******************************************************************************
 * Code
 ******************************************************************************/

static uint32_t CLOCK_GetICSExtClkFreq(void)
{
    /* Please call CLOCK_SetXtal0Freq base on board setting before using OSC0 clock. */
    assert(g_xtal0Freq);
    return g_xtal0Freq;
}

static uint32_t CLOCK_GetFllExtRefClkFreq(void)
{
    /* FllExtRef = ICSExtRef / FllExtRefDiv */
    uint8_t rDiv;
    uint8_t range;

    uint32_t freq = CLOCK_GetICSExtClkFreq();

    if (!freq)
    {
        return freq;
    }
    /* get reference clock divider */
    rDiv = ICS_C1_RDIV_VAL;

    freq >>= rDiv;
    /* OSC clock range */
    range = OSC_CR_RANGE_VAL;

    /*
       When should use divider 32, 64, 128, 256, 512, 1024.
    */
    if (((0U != range)))
    {
        switch (rDiv)
        {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
                freq >>= 5u;
                break;
            case 6:
            case 7:
                break;
            default:
                freq = 0u;
                break;
        }
    }

    return freq;
}

static uint32_t CLOCK_GetFllRefClkFreq(void)
{
    /* If use external reference clock. */
    if (kICS_FllSrcExternal == ICS_S_IREFST_VAL)
    {
        return CLOCK_GetFllExtRefClkFreq();
    }
    /* If use internal reference clock. */
    else
    {
        return s_slowIrcFreq;
    }
}

static uint8_t CLOCK_GetOscRangeFromFreq(uint32_t freq)
{
    assert((freq <= 32768U) || (freq >= 4000000U));

    uint8_t range = 0U;

    if (freq <= 32768U)
    {
        range = 0U;
    }
    /* high freq range 4M-20M */
    else
    {
        range = 1U;
    }

    return range;
}

/*!
 * brief Get the OSC0 external reference clock frequency (OSC0ERCLK).
 *
 * return Clock frequency in Hz.
 */
uint32_t CLOCK_GetOsc0ErClkFreq(void)
{
    if (OSC0->CR & OSC_CR_OSCEN_MASK)
    {
        /* Please call CLOCK_SetXtal0Freq base on board setting before using OSC0 clock. */
        assert(g_xtal0Freq);
        return g_xtal0Freq;
    }
    else
    {
        return 0U;
    }
}

/*!
 * brief Get the flash clock frequency.
 *
 * return Clock frequency in Hz.
 */
uint32_t CLOCK_GetFlashClkFreq(void)
{
    uint32_t freq;

    freq = CLOCK_GetICSOutClkFreq() / (SIM_BUSDIV_VAL + 1);

    return freq;
}

/*!
 * brief Get the bus clock frequency.
 *
 * return Clock frequency in Hz.
 */
uint32_t CLOCK_GetBusClkFreq(void)
{
    return CLOCK_GetFlashClkFreq();
}

/*!
 * brief Get the core clock or system clock frequency.
 *
 * return Clock frequency in Hz.
 */
uint32_t CLOCK_GetCoreSysClkFreq(void)
{
    return CLOCK_GetICSOutClkFreq();
}

/*!
 * brief Gets the clock frequency for a specific clock name.
 *
 * This function checks the current clock configurations and then calculates
 * the clock frequency for a specific clock name defined in clock_name_t.
 * The ICS must be properly configured before using this function.
 *
 * param clockName Clock names defined in clock_name_t
 * return Clock frequency value in Hertz
 */
uint32_t CLOCK_GetFreq(clock_name_t clockName)
{
    uint32_t freq;

    switch (clockName)
    {
        case kCLOCK_CoreSysClk:
        case kCLOCK_PlatClk:
            freq = CLOCK_GetCoreSysClkFreq();
            break;

        case kCLOCK_BusClk:
        case kCLOCK_FlashClk:
            freq = CLOCK_GetFlashClkFreq();
            break;

        case kCLOCK_Osc0ErClk:
            freq = CLOCK_GetOsc0ErClkFreq();
            break;

        case kCLOCK_ICSInternalRefClk:
            freq = CLOCK_GetInternalRefClkFreq();
            break;
        case kCLOCK_ICSFixedFreqClk:
            freq = CLOCK_GetICSFixedFreqClkFreq();
            break;
        case kCLOCK_ICSFllClk:
            freq = CLOCK_GetFllFreq();
            break;
        case kCLOCK_ICSOutClk:
            freq = CLOCK_GetICSOutClkFreq();
            break;

        case kCLOCK_LpoClk:
            freq = LPO_CLK_FREQ;
            break;
        default:
            freq = 0U;
            break;
    }

    return freq;
}

/*!
 * brief Set the clock configure in SIM module.
 *
 * This function sets system layer clock settings in SIM module.
 *
 * param config Pointer to the configure structure.
 */
void CLOCK_SetSimConfig(sim_clock_config_t const *config)
{
    /* config divider */
    SIM->BUSDIV = config->busDiv;
    /* config bus clock prescaler optional */
    SIM->SOPT |= SIM_SOPT_BUSREF(config->busClkPrescaler);
}

/*!
 * brief Gets the ICS output clock (ICSOUTCLK) frequency.
 *
 * This function gets the ICS output clock frequency in Hz based on the current ICS
 * register value.
 *
 * return The frequency of ICSOUTCLK.
 */
uint32_t CLOCK_GetICSOutClkFreq(void)
{
    uint32_t icsoutclk;
    uint32_t clkst = ICS_S_CLKST_VAL;

    switch (clkst)
    {
        case kICS_ClkOutStatFll:
            icsoutclk = CLOCK_GetFllFreq();
            break;
        case kICS_ClkOutStatInt:
            icsoutclk = s_slowIrcFreq;
            break;
        case kICS_ClkOutStatExt:
            icsoutclk = CLOCK_GetICSExtClkFreq();
            break;
        default:
            icsoutclk = 0U;
            break;
    }

    return (icsoutclk / (1 << ICS_C2_BDIV_VAL));
}

/*!
 * brief Gets the ICS FLL clock (ICSFLLCLK) frequency.
 *
 * This function gets the ICS FLL clock frequency in Hz based on the current ICS
 * register value. The FLL is enabled in FEI/FBI/FEE/FBE mode and
 * disabled in low power state in other modes.
 *
 * return The frequency of ICSFLLCLK.
 */
uint32_t CLOCK_GetFllFreq(void)
{
    /* If FLL is not enabled currently, then return 0U. */
    if ((ICS->C2 & ICS_C2_LP_MASK))
    {
        return 0U;
    }

    /* Get FLL reference clock frequency. */
    return CLOCK_GetFllRefClkFreq() * ICS_FLL_CLOCK_FACTOR;
}

/*!
 * brief Gets the ICS internal reference clock (ICSIRCLK) frequency.
 *
 * This function gets the ICS internal reference clock frequency in Hz based
 * on the current ICS register value.
 *
 * return The frequency of ICSIRCLK.
 */
uint32_t CLOCK_GetInternalRefClkFreq(void)
{
    /* If ICSIRCLK is gated. */
    if (!(ICS->C1 & ICS_C1_IRCLKEN_MASK))
    {
        return 0U;
    }

    return s_slowIrcFreq;
}

/*!
 * brief Gets the ICS fixed frequency clock (ICSFFCLK) frequency.
 *
 * This function gets the ICS fixed frequency clock frequency in Hz based
 * on the current ICS register value.
 *
 * return The frequency of ICSFFCLK.
 */
uint32_t CLOCK_GetICSFixedFreqClkFreq(void)
{
    uint32_t freq = CLOCK_GetFllRefClkFreq();

    /* ICSFFCLK must be no more than ICSOUTCLK/4. */
    if ((freq) && (freq <= (CLOCK_GetICSOutClkFreq() / 4U)))
    {
        return freq;
    }
    else
    {
        return 0U;
    }
}

/*!
 * brief Initializes the OSC0.
 *
 * This function initializes the OSC0 according to the board configuration.
 *
 * param  config Pointer to the OSC0 configuration structure.
 */
void CLOCK_InitOsc0(osc_config_t const *config)
{
    uint8_t range = CLOCK_GetOscRangeFromFreq(config->freq);

    OSC0->CR = ((OSC0->CR & ~OSC_MODE_MASK) | OSC_CR_RANGE(range) | ((uint8_t)config->workMode) |
                ((uint8_t)config->enableMode));

    if ((kOSC_ModeExt != config->workMode) && (OSC0->CR & OSC_CR_OSCEN_MASK))
    {
        /* Wait for stable. */
        while (!(OSC0->CR & OSC_CR_OSCINIT_MASK))
        {
        }
    }
}

/*!
 * brief Deinitializes the OSC0.
 *
 * This function deinitializes the OSC0.
 */
void CLOCK_DeinitOsc0(void)
{
    OSC0->CR = 0U;
}

/*!
 * brief Gets the current ICS mode.
 *
 * This function checks the ICS registers and determines the current ICS mode.
 *
 * return Current ICS mode or error code; See ref ics_mode_t.
 */
ics_mode_t CLOCK_GetMode(void)
{
    ics_mode_t mode = kICS_ModeError;
    uint32_t clkst  = ICS_S_CLKST_VAL;
    uint32_t irefst = ICS_S_IREFST_VAL;
    uint32_t lp     = ICS_C2_LP_VAL;

    /*------------------------------------------------------------------
                           Mode and Registers
    ____________________________________________________________________

      Mode   |   CLKST    |   IREFST   |      LP
    ____________________________________________________________________

      FEI    |  00(FLL)   |   1(INT)   |      X
    ____________________________________________________________________

      FEE    |  00(FLL)   |   0(EXT)   |      X
    ____________________________________________________________________

      FBE    |  10(EXT)   |   0(EXT)   |   0(NORMAL)
    ____________________________________________________________________

      FBI    |  01(INT)   |   1(INT)   |   0(NORMAL)
    ____________________________________________________________________

      FBILP   |  01(INT)   |   1(INT)   |   1(LOW POWER)
    ____________________________________________________________________

      FBELP   |  10(EXT)   |   0(EXT)   |   1(LOW POWER)
    ____________________________________________________________________

    ----------------------------------------------------------------------*/

    switch (clkst)
    {
        case kICS_ClkOutStatFll:
            if (kICS_FllSrcExternal == irefst)
            {
                mode = kICS_ModeFEE;
            }
            else
            {
                mode = kICS_ModeFEI;
            }
            break;
        case kICS_ClkOutStatInt:
            if (lp)
            {
                mode = kICS_ModeBILP;
            }
            else
            {
                mode = kICS_ModeFBI;
            }
            break;
        case kICS_ClkOutStatExt:
            if (lp)
            {
                mode = kICS_ModeBELP;
            }
            else
            {
                mode = kICS_ModeFBE;
            }
            break;
        default:
            break;
    }

    return mode;
}

/*!
 * brief Sets the ICS to FEI mode.
 *
 * This function sets the ICS to FEI mode. If setting to FEI mode fails
 * from the current mode, this function returns an error.
 *
 * param       bDiv bus clock divider
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_SetFeiMode(uint8_t bDiv)
{
#if (defined(ICS_CONFIG_CHECK_PARAM) && ICS_CONFIG_CHECK_PARAM)
    ics_mode_t mode = CLOCK_GetMode();
    if (!((kICS_ModeFEI == mode) || (kICS_ModeFBI == mode) || (kICS_ModeFBE == mode) || (kICS_ModeFEE == mode)))
    {
        return kStatus_ICS_ModeUnreachable;
    }
#endif

    /* Set IREFS. */
    ICS->C1 = (ICS->C1 & ~(ICS_C1_IREFS_MASK)) | ICS_C1_IREFS(kICS_FllSrcInternal); /* IREFS = 1 */

    /* Set CLKS */
    ICS->C1 = (ICS->C1 & (~ICS_C1_CLKS_MASK)) | ICS_C1_CLKS(kICS_ClkOutSrcFll); /* CLKS = 0 */
    /* set bus clock divider */
    ICS->C2 = (ICS->C2 & (~ICS_C2_BDIV_MASK)) | ICS_C2_BDIV(bDiv);

    /* Wait and check status. */
    while (kICS_FllSrcInternal != ICS_S_IREFST_VAL)
    {
    }
    /* Check ICS_S[CLKST] */
    while (kICS_ClkOutStatFll != ICS_S_CLKST_VAL)
    {
    }

    /* wait for FLL to lock */
    while (!(ICS->S & ICS_S_LOCK_MASK))
    {
    }

    /* clear Loss of lock sticky bit */
    ICS->S |= ICS_S_LOLS_MASK;

    return kStatus_Success;
}

/*!
 * brief Sets the ICS to FEE mode.
 *
 * This function sets the ICS to FEE mode. If setting to FEE mode fails
 * from the current mode, this function returns an error.
 *
 * param   bDiv bus clock divider
 * param   rdiv  FLL reference clock divider setting, RDIV.
 *
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_SetFeeMode(uint8_t bDiv, uint8_t rDiv)
{
#if (defined(ICS_CONFIG_CHECK_PARAM) && ICS_CONFIG_CHECK_PARAM)
    ics_mode_t mode = CLOCK_GetMode();
    if (!((kICS_ModeFEE == mode) || (kICS_ModeFBI == mode) || (kICS_ModeFBE == mode) || (kICS_ModeFEI == mode)))
    {
        return kStatus_ICS_ModeUnreachable;
    }
#endif

    /* Set CLKS, rDiv and IREFS. */
    ICS->C1 = ((ICS->C1 & ~(ICS_C1_CLKS_MASK | ICS_C1_RDIV_MASK | ICS_C1_IREFS_MASK)) |
               (ICS_C1_CLKS(kICS_ClkOutSrcFll)         /* CLKS = 0 */
                | ICS_C1_RDIV(rDiv)                    /* FRDIV */
                | ICS_C1_IREFS(kICS_FllSrcExternal))); /* IREFS = 0 */
    /* set bus clock divider */
    ICS->C2 = (ICS->C2 & (~ICS_C2_BDIV_MASK)) | ICS_C2_BDIV(bDiv);

    /* If use external crystal as clock source, wait for it stable. */
    {
        if (OSC0->CR & OSC_CR_OSCOS_MASK)
        {
            while (!(OSC0->CR & OSC_CR_OSCINIT_MASK))
            {
            }
        }
    }

    /* Wait and check status. */
    while (kICS_FllSrcExternal != ICS_S_IREFST_VAL)
    {
    }

    /* Check ICS_S[CLKST] */
    while (kICS_ClkOutStatFll != ICS_S_CLKST_VAL)
    {
    }

    /* wait for FLL to lock */
    while (!(ICS->S & ICS_S_LOCK_MASK))
    {
    }

    /* clear Loss of lock sticky bit */
    ICS->S |= ICS_S_LOLS_MASK;

    return kStatus_Success;
}

/*!
 * brief Sets the ICS to FBI mode.
 *
 * This function sets the ICS to FBI mode. If setting to FBI mode fails
 * from the current mode, this function returns an error.
 *
 * param bDiv bus clock divider
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.s
 */
status_t CLOCK_SetFbiMode(uint8_t bDiv)
{
#if (defined(ICS_CONFIG_CHECK_PARAM) && ICS_CONFIG_CHECK_PARAM)
    ics_mode_t mode = CLOCK_GetMode();

    if (!((kICS_ModeFEE == mode) || (kICS_ModeFBI == mode) || (kICS_ModeFBE == mode) || (kICS_ModeFEI == mode) ||
          (kICS_ModeBILP == mode)))

    {
        return kStatus_ICS_ModeUnreachable;
    }
#endif

    /* set bus clock divider and disable low power */
    ICS->C2 = (ICS->C2 & (~(ICS_C2_BDIV_MASK | ICS_C2_LP_MASK))) | ICS_C2_BDIV(bDiv);
    /* Set CLKS and IREFS. */
    ICS->C1 =
        ((ICS->C1 & ~(ICS_C1_CLKS_MASK | ICS_C1_IREFS_MASK)) | (ICS_C1_CLKS(kICS_ClkOutSrcInternal)    /* CLKS = 1 */
                                                                | ICS_C1_IREFS(kICS_FllSrcInternal))); /* IREFS = 1 */

    /* Wait and check status. */
    while (kICS_FllSrcInternal != ICS_S_IREFST_VAL)
    {
    }

    while (kICS_ClkOutStatInt != ICS_S_CLKST_VAL)
    {
    }

    /* clear Loss of lock sticky bit */
    ICS->S |= ICS_S_LOLS_MASK;

    return kStatus_Success;
}

/*!
 * brief Sets the ICS to FBE mode.
 *
 * This function sets the ICS to FBE mode. If setting to FBE mode fails
 * from the current mode, this function returns an error.
 *
 * param   bDiv bus clock divider
 * param   rdiv  FLL reference clock divider setting, RDIV.
 *
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_SetFbeMode(uint8_t bDiv, uint8_t rDiv)
{
#if (defined(ICS_CONFIG_CHECK_PARAM) && ICS_CONFIG_CHECK_PARAM)
    ics_mode_t mode = CLOCK_GetMode();
    if (!((kICS_ModeFEE == mode) || (kICS_ModeFBI == mode) || (kICS_ModeFBE == mode) || (kICS_ModeFEI == mode) ||
          (kICS_ModeBELP == mode)))
    {
        return kStatus_ICS_ModeUnreachable;
    }
#endif

    /* set bus clock divider and disable low power */
    ICS->C2 = (ICS->C2 & (~(ICS_C2_BDIV_MASK | ICS_C2_LP_MASK))) | ICS_C2_BDIV(bDiv);

    /* Set CLKS and IREFS. */
    ICS->C1 = ((ICS->C1 & ~(ICS_C1_CLKS_MASK | ICS_C1_RDIV_MASK | ICS_C1_IREFS_MASK)) |
               (ICS_C1_CLKS(kICS_ClkOutSrcExternal)    /* CLKS = 2 */
                | ICS_C1_RDIV(rDiv)                    /* FRDIV = frDiv */
                | ICS_C1_IREFS(kICS_FllSrcExternal))); /* IREFS = 0 */

    /* If use external crystal as clock source, wait for it stable. */
    {
        if (OSC0->CR & OSC_CR_OSCOS_MASK)
        {
            while (!(OSC0->CR & OSC_CR_OSCINIT_MASK))
            {
            }
        }
    }

    /* Wait for Reference clock Status bit to clear */
    while (kICS_FllSrcExternal != ICS_S_IREFST_VAL)
    {
    }

    /* Wait for clock status bits to show clock source is ext ref clk */
    while (kICS_ClkOutStatExt != ICS_S_CLKST_VAL)
    {
    }

    /* clear Loss of lock sticky bit */
    ICS->S |= ICS_S_LOLS_MASK;

    return kStatus_Success;
}

/*!
 * brief Sets the ICS to BILP mode.
 *
 * This function sets the ICS to BILP mode. If setting to BILP mode fails
 * from the current mode, this function returns an error.
 *
 * param   bDiv bus clock divider
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_SetBilpMode(uint8_t bDiv)
{
#if (defined(ICS_CONFIG_CHECK_PARAM) && ICS_CONFIG_CHECK_PARAM)
    if (ICS_S_CLKST_VAL != kICS_ClkOutStatInt)
    {
        return kStatus_ICS_ModeUnreachable;
    }
#endif /* ICS_CONFIG_CHECK_PARAM */

    /* set bus clock divider and enable low power */
    ICS->C2 = (ICS->C2 & (~ICS_C2_BDIV_MASK)) | ICS_C2_BDIV(bDiv) | ICS_C2_LP_MASK;

    return kStatus_Success;
}

/*!
 * brief Sets the ICS to BELP mode.
 *
 * This function sets the ICS to BELP mode. If setting to BELP mode fails
 * from the current mode, this function returns an error.
 *
 * param   bDiv bus clock divider
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_SetBelpMode(uint8_t bDiv)
{
#if (defined(ICS_CONFIG_CHECK_PARAM) && ICS_CONFIG_CHECK_PARAM)
    if (ICS_S_CLKST_VAL != kICS_ClkOutStatExt)
    {
        return kStatus_ICS_ModeUnreachable;
    }
#endif

    /* set bus clock divider and enable low power */
    ICS->C2 = (ICS->C2 & (~ICS_C2_BDIV_MASK)) | ICS_C2_BDIV(bDiv) | ICS_C2_LP_MASK;

    return kStatus_Success;
}

/*!
 * brief Sets the ICS to FEI mode during system boot up.
 *
 * This function sets the ICS to FEI mode from the reset mode. It can also be used to
 * set up ICS during system boot up.
 *
 * param  bDiv bus clock divider.
 *
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_BootToFeiMode(uint8_t bDiv)
{
    return CLOCK_SetFeiMode(bDiv);
}

/*!
 * brief Sets the ICS to FEE mode during system bootup.
 *
 * This function sets ICS to FEE mode from the reset mode. It can also be used to
 * set up the ICS during system boot up.
 *
 * param   bDiv bus clock divider.
 * param   rdiv  FLL reference clock divider setting, RDIV.
 *
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_BootToFeeMode(uint8_t bDiv, uint8_t rDiv)
{
    return CLOCK_SetFeeMode(bDiv, rDiv);
}

/*!
 * brief Sets the ICS to BILP mode during system boot up.
 *
 * This function sets the ICS to BILP mode from the reset mode. It can also be used to
 * set up the ICS during system boot up.
 *
 * param   bDiv bus clock divider.
 * retval kStatus_ICS_SourceUsed Could not change ICSIRCLK setting.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_BootToBilpMode(uint8_t bDiv)
{
    /* If reset mode is not BILP, first enter FBI mode. */
    ICS->C1 = (ICS->C1 & ~ICS_C1_CLKS_MASK) | ICS_C1_CLKS(kICS_ClkOutSrcInternal);
    while (ICS_S_CLKST_VAL != kICS_ClkOutStatInt)
    {
    }

    /* set bus clock divider and enable low power */
    ICS->C2 = (ICS->C2 & (~ICS_C2_BDIV_MASK)) | ICS_C2_BDIV(bDiv) | ICS_C2_LP_MASK;

    return kStatus_Success;
}

/*!
 * brief Sets the ICS to BELP mode during system boot up.
 *
 * This function sets the ICS to BELP mode from the reset mode. It can also be used to
 * set up the ICS during system boot up.
 *
 * param   bDiv bus clock divider.
 * retval kStatus_ICS_ModeUnreachable Could not switch to the target mode.
 * retval kStatus_Success Switched to the target mode successfully.
 */
status_t CLOCK_BootToBelpMode(uint8_t bDiv)
{
    /* Set to FBE mode. */
    ICS->C1 =
        ((ICS->C1 & ~(ICS_C1_CLKS_MASK | ICS_C1_IREFS_MASK)) | (ICS_C1_CLKS(kICS_ClkOutSrcExternal)    /* CLKS = 2 */
                                                                | ICS_C1_IREFS(kICS_FllSrcExternal))); /* IREFS = 0 */

    /* If use external crystal as clock source, wait for it stable. */
    {
        if (OSC0->CR & OSC_CR_OSCOS_MASK)
        {
            while (!(OSC0->CR & OSC_CR_OSCINIT_MASK))
            {
            }
        }
    }

    /* Wait for ICS_S[CLKST] and ICS_S[IREFST]. */
    while ((ICS->S & (ICS_S_IREFST_MASK | ICS_S_CLKST_MASK)) !=
           (ICS_S_IREFST(kICS_FllSrcExternal) | ICS_S_CLKST(kICS_ClkOutStatExt)))
    {
    }

    /* set bus clock divider and enable low power */
    ICS->C2 = (ICS->C2 & (~ICS_C2_BDIV_MASK)) | ICS_C2_BDIV(bDiv) | ICS_C2_LP_MASK;

    return kStatus_Success;
}

/*
   The transaction matrix. It defines the path for mode switch, the row is for
   current mode and the column is target mode.
   For example, switch from FEI to BELP:
   1. Current mode FEI, next mode is ICSModeMatrix[FEI][BELP] = FBE, so swith to FBE.
   2. Current mode FBE, next mode is ICSModeMatrix[FBE][BELP] = BELP, so swith to BELP.
   Thus the ICS mode has changed from FEI to BELP.
 */
static const ics_mode_t ICSModeMatrix[6][6] = {
    {kICS_ModeFEI, kICS_ModeFBI, kICS_ModeFBI, kICS_ModeFEE, kICS_ModeFBE, kICS_ModeFBE},  /* FEI */
    {kICS_ModeFEI, kICS_ModeFBI, kICS_ModeBILP, kICS_ModeFEE, kICS_ModeFBE, kICS_ModeFBE}, /* FBI */
    {kICS_ModeFBI, kICS_ModeFBI, kICS_ModeBILP, kICS_ModeFBI, kICS_ModeFBI, kICS_ModeFBI}, /* BILP */
    {kICS_ModeFEI, kICS_ModeFBI, kICS_ModeFBI, kICS_ModeFEE, kICS_ModeFBE, kICS_ModeFBE},  /* FEE */
    {kICS_ModeFEI, kICS_ModeFBI, kICS_ModeFBI, kICS_ModeFEE, kICS_ModeFBE, kICS_ModeBELP}, /* FBE */
    {kICS_ModeFBE, kICS_ModeFBE, kICS_ModeFBE, kICS_ModeFBE, kICS_ModeFBE, kICS_ModeBELP}, /* BELP */
    /*      FEI           FBI           BILP          FEE           FBE           BELP      */
};

/*!
 * brief Sets the ICS to a target mode.
 *
 * This function sets ICS to a target mode defined by the configuration
 * structure. If switching to the target mode fails, this function
 * chooses the correct path.
 *
 * param  config Pointer to the target ICS mode configuration structure.
 * return Return kStatus_Success if switched successfully; Otherwise, it returns an error code #_ICS_status.
 *
 * note If the external clock is used in the target mode, ensure that it is
 * enabled. For example, if the OSC0 is used, set up OSC0 correctly before calling this
 * function.
 */
status_t CLOCK_SetIcsConfig(const ics_config_t *config)
{
    ics_mode_t next_mode;
    status_t status = kStatus_Success;

    /* Configure ICSIRCLK. */
    CLOCK_SetInternalRefClkConfig(config->irClkEnableMode);

    next_mode = CLOCK_GetMode();

    do
    {
        next_mode = ICSModeMatrix[next_mode][config->icsMode];

        switch (next_mode)
        {
            case kICS_ModeFEI:
                status = CLOCK_SetFeiMode(config->bDiv);
                break;
            case kICS_ModeFEE:
                status = CLOCK_SetFeeMode(config->bDiv, config->rDiv);
                break;
            case kICS_ModeFBI:
                status = CLOCK_SetFbiMode(config->bDiv);
                break;
            case kICS_ModeFBE:
                status = CLOCK_SetFbeMode(config->bDiv, config->rDiv);
                break;
            case kICS_ModeBILP:
                status = CLOCK_SetBilpMode(config->bDiv);
                break;
            case kICS_ModeBELP:
                status = CLOCK_SetBelpMode(config->bDiv);
                break;
            default:
                break;
        }
        if (kStatus_Success != status)
        {
            return status;
        }
    } while (next_mode != config->icsMode);

    return kStatus_Success;
}

/*!
 * brief Delay at least for several microseconds.
 * Please note that, this API will calculate the microsecond period with the maximum devices
 * supported CPU frequency, so this API will only delay for at least the given microseconds, if precise
 * delay count was needed, please implement a new timer count to achieve this function.
 *
 * param delay_us  Delay time in unit of microsecond.
 */
__attribute__((weak)) void SDK_DelayAtLeastUs(uint32_t delay_us)
{
    assert(0U != delay_us);

    uint32_t count = (uint32_t)USEC_TO_COUNT(delay_us, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

    /*
     * Calculate the real delay count depend on the excute instructions cycles,
     * users can change the divider value to adapt to the real IDE optimise level.
     */
    count = (count / 4U);

    for (; count > 0UL; count--)
    {
        __NOP();
    }
}
