/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
 
#ifndef __FLEXSPI_HARDWARE_INIT_RT1020_H__
#define __FLEXSPI_HARDWARE_INIT_RT1020_H__ 
 
#include <assert.h>
#include <stdbool.h>

#include "bl_flexspi.h"
#include "bl_common.h"
#include "flexspi_hardware_init_rt1020.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config_rt1020(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue = FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;

    if (flexspi_is_padsetting_override_enable(config))
    {
        csPadCtlValue = config->csPadSettingOverride;
        dqsPadCtlValue = config->dqsPadSettingOverride;
        sclkPadCtlValue = config->sclkPadSettingOverride;
        dataPadCtlValue = config->dataPadSettingOverride;
    }

    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondPinMux))
    {
        // The secondary FlexSPI Pinmux, supports only 1 Flash
        if (config->sflashA1Size > 0)
        {
            // FLEXSPIA_SS0_B
            IOMUXC->SW_MUX_CTL_PAD[63] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[63] = csPadCtlValue;
            // FLEXSPIA_SCLK
            IOMUXC->SW_MUX_CTL_PAD[59] =
                FLEXSPIA_SEC_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[59] = sclkPadCtlValue;
            IOMUXC->SELECT_INPUT[31] = 0x01;

            // FLEXSPIA_DATA0
            IOMUXC->SW_MUX_CTL_PAD[60] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[60] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[27] = 0x01;

            // FLEXSPIA_DATA1
            IOMUXC->SW_MUX_CTL_PAD[62] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[62] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[28] = 0x01;

            // FLEXSPIA_DATA2
            IOMUXC->SW_MUX_CTL_PAD[61] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[61] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[29] = 0x01;

            // FLEXSPIA_DATA3
            IOMUXC->SW_MUX_CTL_PAD[58] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[58] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[30] = 0x01;
        }
    }
    else // The primary FlexSPI pinmux, support octal Flash and up to 4 QuadSPI NOR Flash
    {
        // Pinmux configuration for FLEXSPI PortA
        if (config->sflashA1Size || config->sflashA2Size)
        {
            if (config->sflashA2Size)
            {
                // FLEXSPIA_SS1_B
                IOMUXC->SW_MUX_CTL_PAD[74] = FLEXSPIA_SS1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[74] = csPadCtlValue;
            }

            // Basic pinmux configuration for FLEXSPI
            if (config->sflashA1Size)
            {
                // FLEXSPIA_SS0_B
                IOMUXC->SW_MUX_CTL_PAD[92] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[92] = csPadCtlValue;
            }

            // FLEXSPIA_SCLK
            IOMUXC->SW_MUX_CTL_PAD[88] = FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[88] = sclkPadCtlValue;

            // FLEXSPIA_DATA0
            IOMUXC->SW_MUX_CTL_PAD[89] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[89] = dataPadCtlValue;

            // FLEXSPIA_DATA1
            IOMUXC->SW_MUX_CTL_PAD[91] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[91] = dataPadCtlValue;

            // FLEXSPIA_DATA2
            IOMUXC->SW_MUX_CTL_PAD[90] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[90] = dataPadCtlValue;

            // FLEXSPIA_DATA3
            IOMUXC->SW_MUX_CTL_PAD[87] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[87] = dataPadCtlValue;

            if (config->sflashPadType == kSerialFlash_8Pads)
            {
                // FLEXSPIA_DATA4 / FLEXSPIB_DATA0
                IOMUXC->SW_MUX_CTL_PAD[83] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[83] = dataPadCtlValue;

                // FLEXSPIA_DATA5 / FLEXSPIB_DATA1
                IOMUXC->SW_MUX_CTL_PAD[85] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[85] = dataPadCtlValue;

                // FLEXSPIA_DATA6 / FLEXSPIB_DATA2
                IOMUXC->SW_MUX_CTL_PAD[84] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[84] = dataPadCtlValue;

                // FLEXSPIA_DATA7 / FLEXSPIB_DATA3
                IOMUXC->SW_MUX_CTL_PAD[81] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[81] = dataPadCtlValue;
            }

            // Configure DQS pad
            if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
            {
                // FLEXSPIA_DQS
                IOMUXC->SW_MUX_CTL_PAD[86] =
                    FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[86] = dqsPadCtlValue;
            }

            // Configure Differential Clock pin
            if (flexspi_is_differential_clock_enable(config))
            {
                IOMUXC->SW_MUX_CTL_PAD[82] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[82] = sclkPadCtlValue;
            }
        }

        // Pinmux configuration for FLEXSPI PortB
        if (config->sflashB1Size || config->sflashB2Size)
        {
            if (config->sflashB2Size)
            {
                // FLEXSPIB_SS1_B
                IOMUXC->SW_MUX_CTL_PAD[75] = FLEXSPIB_SS1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[75] = csPadCtlValue;
            }

            // Basic pinmux configuration for FLEXSPI
            if (config->sflashB1Size)
            {
                // FLEXSPIB_SS0_B
                IOMUXC->SW_MUX_CTL_PAD[78] = FLEXSPIB_SS0_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[78] = csPadCtlValue;
            }

            // FLEXSPIB_SCLK
            IOMUXC->SW_MUX_CTL_PAD[82] = FLEXSPIB_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[82] = sclkPadCtlValue;

            // FLEXSPIB_DATA0
            IOMUXC->SW_MUX_CTL_PAD[83] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[83] = dataPadCtlValue;

            // FLEXSPIB_DATA1
            IOMUXC->SW_MUX_CTL_PAD[85] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[85] = dataPadCtlValue;

            // FLEXSPIB_DATA2
            IOMUXC->SW_MUX_CTL_PAD[84] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[84] = dataPadCtlValue;

            // FLEXSPIB_DATA3
            IOMUXC->SW_MUX_CTL_PAD[81] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[81] = dataPadCtlValue;

            // Configure DQS pad
            if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
            {
                // FLEXSPIB_DQS
                IOMUXC->SW_MUX_CTL_PAD[79] =
                    FLEXSPIB_DQS_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[79] = dqsPadCtlValue;
            }
        }
    }
}

void flexspi_update_padsetting_rt1020(flexspi_mem_config_t *config, uint32_t driveStrength)
{
    if (driveStrength)
    {
        config->controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_PadSettingOverrideEnable);
        config->dqsPadSettingOverride =
            (FLEXSPI_DQS_SW_PAD_CTL_VAL & ~(0x07 << 3)) | (((driveStrength) << 3) & (0x07 << 3));
        config->sclkPadSettingOverride =
            (FLEXSPI_SW_PAD_CTL_VAL & ~(0x07 << 3)) | (((driveStrength) << 3) & (0x07 << 3));
        config->dataPadSettingOverride =
            (FLEXSPI_SW_PAD_CTL_VAL & ~(0x07 << 3)) | (((driveStrength) << 3) & (0x07 << 3));

        config->csPadSettingOverride =
            (FLEXSPI_DQS_SW_PAD_CTL_VAL & ~(0x07 << 3)) | (((driveStrength) << 3) & (0x07 << 3));
    }
}

void flexspi_clock_config_rt1020(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
{
    uint32_t pfd480 = 0;
    uint32_t cscmr1 = 0;
    uint32_t frac = 0;
    uint32_t podf = 0;

    typedef struct _flexspi_clock_param
    {
        uint8_t frac;
        uint8_t podf;
    } flexspi_clock_param_t;

    const flexspi_clock_param_t k_sdr_clock_config[kFlexSpiSerialClk_200MHz + 1] = {
        // Reserved, 30MHz   50MHz      60MHz      75MHz       80MHz      100MHz     133MHz     166MHz     200MHz
        { 0, 0 }, { 34, 8 }, { 22, 8 }, { 24, 6 }, { 30, 4 }, { 18, 6 }, { 14, 6 }, { 13, 5 }, { 13, 4 }, { 22, 2 }
    };
    const flexspi_clock_param_t k_ddr_clock_config[kFlexSpiSerialClk_200MHz + 1] = {
        // Reserved, 30MHz,     50MHz,   60MHz,      75MHz,     80Mhz,    100MHz,    133MHz,     166MHz,  200MHz
        { 0, 0 }, { 24, 6 }, { 22, 4 }, { 12, 6 }, { 30, 2 }, { 18, 3 }, { 22, 2 }, { 33, 1 }, { 13, 2 }, { 22, 1 }
    };

    do
    {
        if ((sampleClkMode != kFlexSpiClk_SDR) && (sampleClkMode != kFlexSpiClk_DDR))
        {
            break;
        }

        pfd480 = CCM_ANALOG->PFD_480 & (~CCM_ANALOG_PFD_480_PFD0_FRAC_MASK);
        cscmr1 = CCM->CSCMR1 & (~CCM_CSCMR1_FLEXSPI_PODF_MASK);

        // Note: Per ANALOG IP Owner's recommendation, FRAC should be even number,
        //       PODF should be even nubmer as well if the divider is greater than 1

        const flexspi_clock_param_t *flexspi_config_array = NULL;
        if (sampleClkMode == kFlexSpiClk_SDR)
        {
            flexspi_config_array = &k_sdr_clock_config[0];
        }
        else
        {
            flexspi_config_array = &k_ddr_clock_config[0];
        }

        if (freq >= kFlexSpiSerialClk_30MHz)
        {
            if (freq > kFlexSpiSerialClk_200MHz)
            {
                freq = kFlexSpiSerialClk_30MHz;
            }

            frac = flexspi_config_array[freq].frac;
            podf = flexspi_config_array[freq].podf;

            pfd480 |= CCM_ANALOG_PFD_480_PFD0_FRAC(frac);
            cscmr1 |= CCM_CSCMR1_FLEXSPI_PODF(podf - 1);

            FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
            flexspi_clock_gate_disable(instance);

            if (pfd480 != CCM_ANALOG->PFD_480)
            {
                CCM_ANALOG->PFD_480 = pfd480;
            }
            if (cscmr1 != CCM->CSCMR1)
            {
                CCM->CSCMR1 = cscmr1;
            }
            flexspi_clock_gate_enable(instance);
            FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
            __ISB();
        }
        else
        {
            // Do nothing
        }
    } while (0);
}

// Set failsafe settings
status_t flexspi_set_failsafe_setting_rt1020(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
// This is an example that shows how to override the default pad setting in ROM, for now, the pad setting in ROM is
// idential to below values
// So, below codes are not required.
#if 0
        // See IOMUXC pad setting definitions for more details.
        config->controllerMiscOption |= (1<<kFlexSpiMiscOffset_PadSettingOverrideEnable);
        config->dqsPadSettingOverride = 0x130f1;
        config->sclkPadSettingOverride = 0x10f1;
        config->csPadSettingOverride = 0x10f1;
        config->dataPadSettingOverride = 0x10f1;
#endif
        if (config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad)
        {
            if (config->controllerMiscOption & (1 << kFlexSpiMiscOffset_DdrModeEnable))
            {
                config->dataValidTime[0].time_100ps = 19; // 1.9 ns // 1/4 * cycle of 133MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 38; // 3.8 ns // 1/2 * cycle of 133MHz DDR
                }
            }
        }
        status = kStatus_Success;

    } while (0);

    return status;
}



// Get max supported Frequency in this SoC
status_t flexspi_get_max_supported_freq_rt1020(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if ((instance != 0) || (freq == NULL))
        {
            break;
        }

        *freq = (133UL * 1000 * 1000);
        status = kStatus_Success;

    } while (0);

    return status;
}

static inline bool CLOCK_IsPllEnabled(CCM_ANALOG_Type *base, clock_pll_t pll)
{
    return (bool)(CCM_ANALOG_TUPLE_REG(base, pll) & (1UL << CCM_ANALOG_TUPLE_SHIFT(pll)));
}

static inline uint32_t CLOCK_GetOscFreq(void)
{
	  /* External XTAL (OSC) clock frequency. */
    volatile uint32_t g_xtalFreq = 24000000UL;
    return ((XTALOSC24M->LOWPWR_CTRL & (uint32_t)XTALOSC24M_LOWPWR_CTRL_OSC_SEL_MASK) != 0UL) ? 24000000UL : g_xtalFreq;
}

static inline uint32_t CLOCK_GetPllBypassRefClk(CCM_ANALOG_Type *base, clock_pll_t pll)
{
    return ((((uint32_t)(CCM_ANALOG_TUPLE_REG(base, pll) & CCM_ANALOG_PLL_BYPASS_CLK_SRC_MASK)) >>
             CCM_ANALOG_PLL_BYPASS_CLK_SRC_SHIFT) == (uint32_t)kCLOCK_PllClkSrc24M) ?
               CLOCK_GetOscFreq() :
               CLKPN_FREQ;
}

static inline bool CLOCK_IsPllBypassed(CCM_ANALOG_Type *base, clock_pll_t pll)
{
    return (bool)(CCM_ANALOG_TUPLE_REG(base, pll) & (1UL << CCM_ANALOG_PLL_BYPASS_SHIFT));
}

static uint32_t CLOCK_GetPllFreq(clock_pll_t pll)
{
    uint32_t freq;
    uint32_t divSelect;
    clock_64b_t freqTmp;

    const uint32_t enetRefClkFreq[] = {
        25000000U,  /* 25M */
        50000000U,  /* 50M */
        100000000U, /* 100M */
        125000000U  /* 125M */
    };

    /* check if PLL is enabled */
    if (!CLOCK_IsPllEnabled(CCM_ANALOG, pll))
    {
        return 0U;
    }

    /* get pll reference clock */
    freq = CLOCK_GetPllBypassRefClk(CCM_ANALOG, pll);

    /* check if pll is bypassed */
    if (CLOCK_IsPllBypassed(CCM_ANALOG, pll))
    {
        return freq;
    }

    switch (pll)
    {
        case kCLOCK_PllSys:
            /* PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM). */
            freqTmp = ((clock_64b_t)freq * ((clock_64b_t)(CCM_ANALOG->PLL_SYS_NUM)));
            freqTmp /= ((clock_64b_t)(CCM_ANALOG->PLL_SYS_DENOM));

            if ((CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_DIV_SELECT_MASK) != 0UL)
            {
                freq *= 22U;
            }
            else
            {
                freq *= 20U;
            }

            freq += (uint32_t)freqTmp;
            break;

        case kCLOCK_PllUsb1:
            freq = (freq * (((CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK) != 0UL) ? 22U : 20U));
            break;

        case kCLOCK_PllAudio:
            /* PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM). */
            divSelect =
                (CCM_ANALOG->PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK) >> CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT;

            freqTmp = ((clock_64b_t)freq * ((clock_64b_t)(CCM_ANALOG->PLL_AUDIO_NUM)));
            freqTmp /= ((clock_64b_t)(CCM_ANALOG->PLL_AUDIO_DENOM));

            freq = freq * divSelect + (uint32_t)freqTmp;

            /* AUDIO PLL output = PLL output frequency / POSTDIV. */

            /*
             * Post divider:
             *
             * PLL_AUDIO[POST_DIV_SELECT]:
             * 0x00: 4
             * 0x01: 2
             * 0x02: 1
             *
             * MISC2[AUDO_DIV]:
             * 0x00: 1
             * 0x01: 2
             * 0x02: 1
             * 0x03: 4
             */
            switch (CCM_ANALOG->PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_MASK)
            {
                case CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(0U):
                    freq = freq >> 2U;
                    break;

                case CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(1U):
                    freq = freq >> 1U;
                    break;

                case CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2U):
                    freq = freq >> 0U;
                    break;

                default:
                    assert(false);
                    break;
            }

            switch (CCM_ANALOG->MISC2 & (CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK | CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK))
            {
                case CCM_ANALOG_MISC2_AUDIO_DIV_MSB(1) | CCM_ANALOG_MISC2_AUDIO_DIV_LSB(1):
                    freq >>= 2U;
                    break;

                case CCM_ANALOG_MISC2_AUDIO_DIV_MSB(0) | CCM_ANALOG_MISC2_AUDIO_DIV_LSB(1):
                    freq >>= 1U;
                    break;

                case CCM_ANALOG_MISC2_AUDIO_DIV_MSB(0) | CCM_ANALOG_MISC2_AUDIO_DIV_LSB(0):
                case CCM_ANALOG_MISC2_AUDIO_DIV_MSB(1) | CCM_ANALOG_MISC2_AUDIO_DIV_LSB(0):
                    freq >>= 0U;
                    break;

                default:
                    assert(false);
                    break;
            }
            break;

        case kCLOCK_PllEnet:
            divSelect =
                (CCM_ANALOG->PLL_ENET & CCM_ANALOG_PLL_ENET_DIV_SELECT_MASK) >> CCM_ANALOG_PLL_ENET_DIV_SELECT_SHIFT;
            freq = enetRefClkFreq[divSelect];
            break;

        case kCLOCK_PllEnet25M:
            /* ref_enetpll1 if fixed at 25MHz. */
            freq = 25000000UL;
            break;

        case kCLOCK_PllEnet500M:
            /* PLL6 is fixed at 25MHz. */
            freq = 500000000UL;
            break;

        default:
            freq = 0U;
            break;
    }

    return freq;
}


static uint32_t CLOCK_GetUsb1PfdFreq(clock_pfd_t pfd)
{
    uint32_t freq = CLOCK_GetPllFreq(kCLOCK_PllUsb1);

    switch (pfd)
    {
        case kCLOCK_Pfd0:
            freq /= ((CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd1:
            freq /= ((CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD1_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD1_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd2:
            freq /= ((CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd3:
            freq /= ((CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD3_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT);
            break;

        default:
            freq = 0U;
            break;
    }
    freq *= 18U;

    return freq;
}

static uint32_t CLOCK_GetSysPfdFreq(clock_pfd_t pfd)
{
    uint32_t freq = CLOCK_GetPllFreq(kCLOCK_PllSys);

    switch (pfd)
    {
        case kCLOCK_Pfd0:
            freq /= ((CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd1:
            freq /= ((CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD1_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD1_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd2:
            freq /= ((CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd3:
            freq /= ((CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD3_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD3_FRAC_SHIFT);
            break;

        default:
            freq = 0U;
            break;
    }
    freq *= 18U;

    return freq;
}

uint32_t CLOCK_GetCPUFreq_RT1020(void)
{
    uint32_t freq;

    /* Periph_clk2_clk ---> Periph_clk */
    if ((CCM->CBCDR & CCM_CBCDR_PERIPH_CLK_SEL_MASK) != 0UL)
    {
        switch (CCM->CBCMR & CCM_CBCMR_PERIPH_CLK2_SEL_MASK)
        {
            /* Pll3_sw_clk ---> Periph_clk2_clk ---> Periph_clk */
            case CCM_CBCMR_PERIPH_CLK2_SEL(0U):
                freq = CLOCK_GetPllFreq(kCLOCK_PllUsb1);
                break;

            /* Osc_clk ---> Periph_clk2_clk ---> Periph_clk */
            case CCM_CBCMR_PERIPH_CLK2_SEL(1U):
                freq = CLOCK_GetOscFreq();
                break;

            case CCM_CBCMR_PERIPH_CLK2_SEL(2U):
                freq = CLOCK_GetPllFreq(kCLOCK_PllSys);
                break;

            case CCM_CBCMR_PERIPH_CLK2_SEL(3U):
            default:
                freq = 0U;
                break;
        }

        freq /= (((CCM->CBCDR & CCM_CBCDR_PERIPH_CLK2_PODF_MASK) >> CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT) + 1U);
    }
    /* Pre_Periph_clk ---> Periph_clk */
    else
    {
        switch (CCM->CBCMR & CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK)
        {
            /* PLL2 */
            case CCM_CBCMR_PRE_PERIPH_CLK_SEL(0U):
                freq = CLOCK_GetPllFreq(kCLOCK_PllSys);
                break;

            /* PLL3 PFD3 */
            case CCM_CBCMR_PRE_PERIPH_CLK_SEL(1U):
                freq = CLOCK_GetUsb1PfdFreq(kCLOCK_Pfd3);
                break;

            /* PLL2 PFD3 */
            case CCM_CBCMR_PRE_PERIPH_CLK_SEL(2U):
                freq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd3);
                break;

            /* PLL6 divided(/1) */
            case CCM_CBCMR_PRE_PERIPH_CLK_SEL(3U):
                freq = 500000000U;
                break;

            default:
                freq = 0U;
                break;
        }
    }
	
		freq = freq / (((CCM->CBCDR & CCM_CBCDR_AHB_PODF_MASK) >> CCM_CBCDR_AHB_PODF_SHIFT) + 1U);;
		
    return freq;
}

static uint32_t get_arm_pll(void)
{
    uint32_t arm_pll = 0;
    uint32_t frac = 0;
    uint32_t arm_podf = 0;
    uint32_t clock_source = 0;
    uint32_t periph_clk2_podf = 0;

    if (CCM->CBCDR & CCM_CBCDR_PERIPH_CLK_SEL_MASK) // Derive clock from periph_clk2_sel
    {
        clock_source = (CCM->CBCMR & CCM_CBCMR_PERIPH_CLK2_SEL_MASK) >> CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT;
        if (clock_source == 0)
        {
            arm_pll = FREQ_480MHz;
        }
        else
        {
            arm_pll = FREQ_24MHz;
        }

        periph_clk2_podf = 1 + ((CCM->CBCDR & CCM_CBCDR_PERIPH_CLK2_PODF_MASK) >> CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT);

        arm_pll /= periph_clk2_podf;
    }
    else // Derive clock from pre_periph_clk_sel
    {
        clock_source = (CCM->CBCMR & CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK) >> CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT;

        switch(clock_source)
        {
        case 2: // PFD_528_PFD3
            frac = (CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD3_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD3_FRAC_SHIFT;
            arm_pll = FREQ_528MHz / frac * 18;
            break;
        case 0: // SYS_PLL
            arm_pll = FREQ_528MHz;
            break;
        case 1: // PFD_528_PFD2
            frac = (CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT;
            arm_pll = FREQ_528MHz / frac * 18;
            break;
        default: // ENET PLL / ARM_PODF
            if (CCM_ANALOG->PLL_ENET & CCM_ANALOG_PLL_ENET_BYPASS_MASK)
            {
                arm_pll = FREQ_24MHz;
            }
            else
            {
                arm_pll = FREQ_500MHz;
            }
            arm_podf = 1 + ((CCM->CACRR & CCM_CACRR_ARM_PODF_MASK) >> CCM_CACRR_ARM_PODF_SHIFT);
            arm_pll /= arm_podf;
            break;
        }
    }

    return arm_pll;
}

//!@brief Get Clock for FlexSPI peripheral
status_t flexspi_get_clock_rt1020(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    uint32_t ahbBusDivider;
    uint32_t seralRootClkDivider;
	
    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = CLOCK_GetCPUFreq_RT1020();
            break;
    case kFlexSpiClock_AhbClock:
        {
            uint32_t arm_pll = get_arm_pll();
            uint32_t ahb_podf = 1 + ((CCM->CBCDR & CCM_CBCDR_AHB_PODF_MASK) >> CCM_CBCDR_AHB_PODF_SHIFT);
            // Note: In I.MXRT_1020, actual AHB clock is IPG_CLOCK_ROOT
            ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK)>>CCM_CBCDR_IPG_PODF_SHIFT) + 1;
            clockFrequency = arm_pll / ahb_podf / ahbBusDivider;
        }
        break;
        case kFlexSpiClock_SerialRootClock:
        {
            uint32_t pfdFrac;
            uint32_t pfdClk;

            // FLEXPI CLK SEL
            uint32_t flexspi_clk_src =
                (CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK) >> CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT;

            // PLL_480_PFD0
            pfdFrac = (CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT;
            pfdClk = FREQ_480MHz / pfdFrac * 18;

            seralRootClkDivider = ((CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_PODF_MASK) >> CCM_CSCMR1_FLEXSPI_PODF_SHIFT) + 1;

            clockFrequency = pfdClk / seralRootClkDivider;
        }
        break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }
    *freq = clockFrequency;

    return status;
}


//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable_rt1020(uint32_t instance)
{
    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;
}

//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable_rt1020(uint32_t instance)
{
    CCM->CCGR6 &= (uint32_t)~CCM_CCGR6_CG5_MASK;
}

//!@brief Write FlexSPI persistent content
status_t flexspi_nor_write_persistent_rt1020(const uint32_t data)
{
    SRC->GPR[2] = data;

    return kStatus_Success;
}
//!@brief Read FlexSPI persistent content
status_t flexspi_nor_read_persistent_rt1020(uint32_t *data)
{
    *data = SRC->GPR[2];

    return kStatus_Success;
}

#endif
