/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
 
#ifndef __FLEXSPI_HARDWARE_INIT_RT1010_H__
#define __FLEXSPI_HARDWARE_INIT_RT1010_H__ 
 
#include <assert.h>
#include <stdbool.h>

#include "bl_flexspi.h"
#include "bl_common.h"
#include "flexspi_hardware_init_rt1010.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config_rt1010(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue = config->csPadSettingOverride ? config->csPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue = config->dqsPadSettingOverride ? config->dqsPadSettingOverride : FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = config->sclkPadSettingOverride ? config->sclkPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = config->dataPadSettingOverride ? config->dataPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;

    if (instance == 0)
    {
        // The primary FlexSPI pinmux, support octal Flash and up to 4 QuadSPI NOR Flash
        {
            // Pinmux configuration for FLEXSPI1 PortA
            if (config->sflashA1Size || config->sflashA2Size)
            {
                if (config->sflashA2Size)
                {
                    // FLEXSPI1A_SS1_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SS1_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SS1_B_IDX] = csPadCtlValue;
                }

                // Basic pinmux configuration for FLEXSPI1
                if (config->sflashA1Size)
                {
                    // FLEXSPI1A_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SS0_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPI1A_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX] = \
                    FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPI1A_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA3_IDX] = dataPadCtlValue;

                if (config->sflashPadType == kSerialFlash_8Pads)
                {
                    // FLEXSPI1A_DATA4 / FLEXSPI1B_DATA0
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA5 / FLEXSPI1B_DATA1
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA6 / FLEXSPI1B_DATA2
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA7 / FLEXSPI1B_DATA3
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX] = dataPadCtlValue;
                }

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondDqsPinMux))
                    {
                        // FLEXSPI1A_SEC_DQS
                        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX] = \
                            FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX] = dqsPadCtlValue;
                        IOMUXC->SELECT_INPUT[SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX] = kFLEXSPI1A_DQS_SRC_GPIO_SD_14;
                    }
                    else
                    {
                        // FLEXSPI1A_DQS
                        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DQS_IDX] = \
                            FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DQS_IDX] = dqsPadCtlValue;
                        IOMUXC->SELECT_INPUT[SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX] = kFLEXSPI1A_DQS_SRC_GPIO_SD_12;
                    }
                }

                // Configure Differential Clock pin
                if (flexspi_is_differential_clock_enable(config))
                {
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX] = sclkPadCtlValue;
                }
            }

            // Pinmux configuration for FLEXSPI1 PortB
            if (config->sflashB1Size || config->sflashB2Size)
            {
                // Basic pinmux configuration for FLEXSPI1
                if (config->sflashB1Size)
                {
                    // FLEXSPI1B_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPI1B_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX] = \
                    FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPI1B_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX] = dataPadCtlValue;

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    // FLEXSPI1B_DQS
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DQS_IDX] =
                        FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DQS_IDX] = dqsPadCtlValue;
                }
            }
        }
    }
}

void flexspi_update_padsetting_rt1010(flexspi_mem_config_t *config, uint32_t driveStrength)
{

}


//!@brief Configure clock for FlexSPI peripheral
void flexspi_clock_config_rt1010(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
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
        // Reserved, 30MHz     50MHz     60MHz        75MHz    80MHz       100MHz   133MHz       166MHz   200MHz
        { 0, 0 }, { 34, 8 }, { 22, 8 }, { 24, 6 }, { 30, 4 }, { 18, 6 }, { 14, 6 }, { 17, 4 }, { 26, 2 }, { 22, 2 }
    };
    const flexspi_clock_param_t k_ddr_clock_config[kFlexSpiSerialClk_200MHz + 1] = {
        // Reserved, 30MHz,  50MHz,       60MHz,      75MHz,   80Mhz,   100MHz,      133MHz,   166MHz,     200MHz
        { 0, 0 }, { 24, 6 }, { 22, 4 }, { 12, 6 }, { 30, 2 }, { 18, 3 }, { 22, 2 }, { 33, 1 }, { 26, 1 }, { 22, 1 }
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
        }
        else
        {
            // Do nothing
        }
    } while (0);
}

// Set failsafe settings
status_t flexspi_set_failsafe_setting_rt1010(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
				
        if (config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad)
        {
            if (config->controllerMiscOption & (1 << kFlexSpiMiscOffset_DdrModeEnable))
            {
                config->dataValidTime[0].time_100ps = 15; // 1.5 ns // 1/4 * cycle of 166MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 30; // 3 ns // 1/2 * cycle of 166MHz DDR
                }
            }
        }
        status = kStatus_Success;

    } while (0);

    return status;
}

//!@brief Get maximum frequency supported by FlexSPI
status_t flexspi_get_max_supported_freq_rt1010(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (freq == NULL)
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
    return ((XTALOSC24M->LOWPWR_CTRL & (uint32_t)XTALOSC24M_LOWPWR_CTRL_OSC_SEL_MASK) != 0UL) ? 24000000UL : 24000000UL;
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

                default:
                    /* Add comment to avoid MISRA C-2012 rule 16.4 */
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

                default:
                    /* Add comment to avoid MISRA C-2012 rule 16.4 */
                    break;
            }
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

uint32_t CLOCK_GetCPUFreq_RT1010(void)
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
	
		freq =  freq / (((CCM->CBCDR & CCM_CBCDR_AHB_PODF_MASK) >> CCM_CBCDR_AHB_PODF_SHIFT) + 1U);
		return freq;
}

//!@brief Get Clock for FlexSPI peripheral
status_t flexspi_get_clock_rt1010(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    if ((instance > 0) || (freq == NULL) || (type > kFlexSpiClock_IpgClock))
    {
        return kStatus_InvalidArgument;
    }

    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = CLOCK_GetCPUFreq_RT1010();
            break;
        case kFlexSpiClock_SerialRootClock:
        {
            switch ((CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK)>>CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)
            {
                case 0: // PLL_SYS
                    clockFrequency = FREQ_528MHz;    
                    if (CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_BYPASS_MASK)
                    {
                        clockFrequency = FREQ_24MHz; 
                    }
                    break;
                case 1: // PLL_USB1
                    clockFrequency = FREQ_480MHz;
                    if (CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_BYPASS_MASK)
                    {
                        clockFrequency = FREQ_24MHz;
                    }
                    break;
                case 2: // PFD_528_PFD2
                    if (CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_BYPASS_MASK)
                    {
                        clockFrequency = FREQ_24MHz;
                    }
                    else
                    {
                        uint32_t pfd = (CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT;
                        clockFrequency = FREQ_528MHz / pfd * 18;
                    }
                    break;
                case 3: // PFD_480_PFD0
                    if (CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_BYPASS_MASK)
                    {
                        clockFrequency = FREQ_24MHz;
                    }
                    else
                    {
                        uint32_t pfd = (CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT;
                        clockFrequency = FREQ_528MHz / pfd * 18;
                    }
                    break;
            }
                       
            uint32_t flexspi_podf_divider = 1 + ((CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_PODF_MASK) >> CCM_CSCMR1_FLEXSPI_PODF_SHIFT);
           
            clockFrequency /= flexspi_podf_divider;
        }
        break;
        default:
            status = kStatus_InvalidArgument;
        break;
    }
    if (status == kStatus_Success)
    {
        *freq = clockFrequency;
    }

    return status;
}

//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable_rt1010(uint32_t instance)
{
    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;
}

//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable_rt1010(uint32_t instance)
{
    CCM->CCGR6 &= (uint32_t)~CCM_CCGR6_CG5_MASK;
}

//!@brief Write FlexSPI persistent content
status_t flexspi_nor_write_persistent_rt1010(const uint32_t data)
{
    SRC->GPR[2] = data;

    return kStatus_Success;
}
//!@brief Read FlexSPI persistent content
status_t flexspi_nor_read_persistent_rt1010(uint32_t *data)
{
    *data = SRC->GPR[2];

    return kStatus_Success;
}

#endif
