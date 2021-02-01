/*
 * Copyright 2019-2020 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

//#include "bl_context.h"
#include "bl_flexspi.h"
//#include "bl_ocotp.h"
#include "bootloader_common.h"
#include "fsl_assert.h"
#include "fsl_device_registers.h"
#include "fsl_clock.h"
#include "fusemap.h"
#include "microseconds.h"
//#include "property.h"
#include "target_config.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#ifndef FREQ_396MHz
#define FREQ_396MHz (396UL * 1000 * 1000)
#endif
#ifndef FREQ_528MHz
#define FREQ_528MHz (528UL * 1000 * 1000)
#endif
#ifndef FREQ_24MHz
#define FREQ_24MHz (24UL * 1000 * 1000)
#endif
#ifndef FREQ_480MHz
#define FREQ_480MHz (480UL * 1000 * 1000)
#endif
#ifndef FREQ_432MHz
#define FREQ_432MHz (432UL * 1000 * 1000)
#endif
#ifndef FREQ_508MHz
#define FREQ_508MHz (508UL * 1000 * 1000)
#endif
#ifndef FREQ_1MHz
#define FREQ_1MHz (1UL * 1000 * 1000)
#endif
#ifndef FREQ_500MHz
#define FREQ_500MHz (500UL * 1000 * 1000)
#endif

enum
{
    kMaxIpgClock = 144000000UL,
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
//uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;
////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

bool usb_clock_init(void)
{
    // Enable clock gate
    CCM->CCGR6 |= CCM_CCGR6_CG0_MASK;

    // Enable USB Clocks
    CCM_ANALOG->PLL_USB1_SET = CCM_ANALOG_PLL_USB1_EN_USB_CLKS_MASK;

    // Clear SFTRST
    USBPHY->CTRL_CLR = USBPHY_CTRL_SFTRST_MASK;

    // Clear Clock gate
    USBPHY->CTRL_CLR = USBPHY_CTRL_CLKGATE_MASK;

    // Clear power down register
    USBPHY->PWD = 0;

    // Disable Charger Detect
    USB_ANALOG->INSTANCE[0].CHRG_DETECT |= (USB_ANALOG_CHRG_DETECT_EN_B_MASK | USB_ANALOG_CHRG_DETECT_CHK_CHRG_B_MASK);

    USB->USBCMD &= (uint32_t)~USBHS_USBCMD_RS_MASK;

    return true;
}

// Get OCOTP clock
uint32_t get_ocotp_clock(void)
{
    uint32_t ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
    return SystemCoreClock / ahbBusDivider;
}

#if BL_FEATURE_FLEXSPI_NOR_MODULE
//!@brief Configure clock for FlexSPI peripheral
void flexspi_clock_config(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
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

//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable(uint32_t instance)
{
    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;
}

//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable(uint32_t instance)
{
    CCM->CCGR6 &= (uint32_t)~CCM_CCGR6_CG5_MASK;
}

#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE

//!@brief Get maximum frequency supported by FlexSPI
status_t flexspi_get_max_supported_freq(uint32_t instance, uint32_t *freq, uint32_t clkMode)
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

//! @brief Gets the clock value used for microseconds driver
uint32_t microseconds_get_clock(void)
{
    // Get PIT clock source
    uint32_t ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
    uint32_t periphDivider = ((CCM->CSCMR1 & CCM_CSCMR1_PERCLK_PODF_MASK) >> CCM_CSCMR1_PERCLK_PODF_SHIFT) + 1;
    return SystemCoreClock / ahbBusDivider / periphDivider;
}

//! @brief Get BUS clock value
uint32_t get_bus_clock(void)
{
    uint32_t ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
    return SystemCoreClock / ahbBusDivider;
}

static uint32_t get_periph_clk(void)
{
    uint32_t periph_clk = 0;
    uint32_t frac = 0;
    uint32_t clock_source = 0;

    if (CCM->CBCDR & CCM_CBCDR_PERIPH_CLK_SEL_MASK) // Derive clock from periph_clk2_sel
    {
        clock_source = (CCM->CBCMR & CCM_CBCMR_PERIPH_CLK2_SEL_MASK) >> CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT;
        if (clock_source == 0)
        {
            periph_clk = FREQ_480MHz;
        }
        else
        {
            periph_clk = FREQ_24MHz;
        }
    }
    else // Derive clock from pre_periph_clk_sel
    {
        clock_source = (CCM->CBCMR & CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK) >> CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT;

        switch(clock_source)
        {
        case 0: // SYS_PLL
            periph_clk = FREQ_528MHz;
            break;
        case 1: // PFD_480_PFD3
            frac = (CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD3_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT;
            periph_clk = FREQ_480MHz / frac * 18; // Div-before-Mul: in case of overflow
            break;
        case 2: // PFD_528_PFD3
            frac = (CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD3_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD3_FRAC_SHIFT;
            periph_clk = FREQ_528MHz / frac * 18; // Div-before-Mul: in case of overflow
            break;
        default: // ENET PLL / ARM_PODF
            if (CCM_ANALOG->PLL_ENET & CCM_ANALOG_PLL_ENET_BYPASS_MASK)
            {
                periph_clk = FREQ_24MHz;
            }
            else
            {
                periph_clk = FREQ_500MHz;
            }
            break;
        }
    }

    return periph_clk;
}

uint32_t get_core_clock(void)
{
    uint32_t periph_clk = get_periph_clk();
    uint32_t ahb_podf = 1 + ((CCM->CBCDR & CCM_CBCDR_AHB_PODF_MASK) >> CCM_CBCDR_AHB_PODF_SHIFT);
    uint32_t core_clock = periph_clk / ahb_podf;

    return core_clock;
}

//!@brief Get Clock for FlexSPI peripheral
status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    if ((instance > 0) || (freq == NULL) || (type > kFlexSpiClock_IpgClock))
    {
        return kStatus_InvalidArgument;
    }

#ifndef BL_TARGET_FPGA
    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = get_core_clock();
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
//            TRACE("BootROM: FlexSPI Serial Root Clock = %dMHz\n", clockFrequency / FREQ_1MHz);
        }
        break;
        default:
            status = kStatus_InvalidArgument;
        break;
    }
#else
    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = 24000000;
            break;
        case kFlexSpiClock_SerialRootClock:
            clockFrequency = 12000000;
            break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }
#endif
    if (status == kStatus_Success)
    {
        *freq = clockFrequency;
    }

    return status;
}


void flexspi_sw_delay_us(uint64_t us)
{
    uint32_t ticks_per_us = get_core_clock() / 1000000;
    while (us--)
    {
        register uint32_t ticks = 1 + ticks_per_us / 4;
        while (--ticks)
        {
            __NOP();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
