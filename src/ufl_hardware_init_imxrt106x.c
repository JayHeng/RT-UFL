/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_hardware_init.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/



/*******************************************************************************
 * Prototypes
 ******************************************************************************/



/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

void ufl_init_hardware_imxrt106x(void)
{
    // Disable Watchdog Power Down Counter
    //WDOG1->WMCR &= ~WDOG_WMCR_PDE_MASK;
    MEM_WriteU16(0x400B8008u, MEM_ReadU16(0x400B8008u) & (~0x1U));
    //WDOG2->WMCR &= ~WDOG_WMCR_PDE_MASK;
    MEM_WriteU16(0x400D0008u, MEM_ReadU16(0x400D0008u) & (~0x1U));

    // Watchdog disable.
    //if (WDOG1->WCR & WDOG_WCR_WDE_MASK)
    if (MEM_ReadU16(0x400B8000u) & 0x4U)
    {
        //WDOG1->WCR &= ~WDOG_WCR_WDE_MASK;
        MEM_WriteU16(0x400B8000u, MEM_ReadU16(0x400B8000u) & (~0x4U));
    }
    //if (WDOG2->WCR & WDOG_WCR_WDE_MASK)
    if (MEM_ReadU16(0x400D0000u) & 0x4U)
    {
        //WDOG2->WCR &= ~WDOG_WCR_WDE_MASK;
        MEM_WriteU16(0x400D0000u, MEM_ReadU16(0x400D0000u) & (~0x4U));
    }
    //RTWDOG->CNT   = 0xD928C520U; // 0xD928C520U is the update key
    MEM_WriteU32(0x400BC004u, 0xD928C520U);
    //RTWDOG->TOVAL = 0xFFFF;
    MEM_WriteU32(0x400BC008u, 0xFFFF);
    //RTWDOG->CS    = (uint32_t)((RTWDOG->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;
    MEM_WriteU32(0x400BC000u, (uint32_t)(MEM_ReadU32(0x400BC000u) & (~0x80U)) | 0x20U);

    //if (CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_BYPASS_MASK)
    if (MEM_ReadU32(0x400D8000u) & 0x10000U)
    {
        // Configure ARM_PLL
        //CCM_ANALOG->PLL_ARM = CCM_ANALOG_PLL_ARM_BYPASS(1) | CCM_ANALOG_PLL_ARM_ENABLE(1) | CCM_ANALOG_PLL_ARM_DIV_SELECT(24);
        MEM_WriteU32(0x400D8000u, 0x12018U);
        // Wait Until clock is locked
        //while ((CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_LOCK_MASK) == 0)
        while ((MEM_ReadU32(0x400D8000u) & 0x80000000U) == 0)
        {
        }

        // Configure PLL_SYS
        //CCM_ANALOG->PLL_SYS &= ~CCM_ANALOG_PLL_SYS_POWERDOWN_MASK;
        MEM_WriteU32(0x400D8030u, MEM_ReadU32(0x400D8030u) & (~0x1000U));
        // Wait Until clock is locked
        //while ((CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_LOCK_MASK) == 0)
        while ((MEM_ReadU32(0x400D8030u) & 0x80000000U) == 0)
        {
        }

        // Configure PFD_528
        //CCM_ANALOG->PFD_528 = CCM_ANALOG_PFD_528_PFD0_FRAC(24) | CCM_ANALOG_PFD_528_PFD1_FRAC(24) |
        //                      CCM_ANALOG_PFD_528_PFD2_FRAC(19) | CCM_ANALOG_PFD_528_PFD3_FRAC(24);
        MEM_WriteU32(0x400D8100u, 0x18131818U);

        // Configure USB1_PLL
        //CCM_ANALOG->PLL_USB1 = CCM_ANALOG_PLL_USB1_DIV_SELECT(0) | CCM_ANALOG_PLL_USB1_POWER(1) | CCM_ANALOG_PLL_USB1_ENABLE(1);
        MEM_WriteU32(0x400D8010u, 0x3000U);
        //while ((CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_LOCK_MASK) == 0)
        while ((MEM_ReadU32(0x400D8010u) & 0x80000000U) == 0)
        {
        }
        //CCM_ANALOG->PLL_USB1 &= ~CCM_ANALOG_PLL_USB1_BYPASS_MASK;
        MEM_WriteU32(0x400D8010u, MEM_ReadU32(0x400D8010u) & (~0x10000U));

        // Configure PFD_480
        //CCM_ANALOG->PFD_480 = CCM_ANALOG_PFD_480_PFD0_FRAC(35) | CCM_ANALOG_PFD_480_PFD1_FRAC(35) |
        //                      CCM_ANALOG_PFD_480_PFD2_FRAC(26) | CCM_ANALOG_PFD_480_PFD3_FRAC(15);
        MEM_WriteU32(0x400D80F0u, 0xF1A2323U);

        // Configure Clock PODF
        //CCM->CACRR = CCM_CACRR_ARM_PODF(1);
        MEM_WriteU32(0x400FC010u, 0x1U);

        //CCM->CBCDR = (CCM->CBCDR & (~(CCM_CBCDR_SEMC_PODF_MASK | CCM_CBCDR_AHB_PODF_MASK | CCM_CBCDR_IPG_PODF_MASK))) |
        //             CCM_CBCDR_SEMC_PODF(2) | CCM_CBCDR_AHB_PODF(2) | CCM_CBCDR_IPG_PODF(2);
        MEM_WriteU32(0x400FC014u, (uint32_t)(MEM_ReadU32(0x400FC014u) & (~0x71F00U)) | 0x20A00U);

        // Configure FLEXSPI2 CLOCKS
        //CCM->CBCMR = (CCM->CBCMR & (~(CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK | CCM_CBCMR_FLEXSPI2_CLK_SEL_MASK | CCM_CBCMR_FLEXSPI2_PODF_MASK))) |
        //             CCM_CBCMR_PRE_PERIPH_CLK_SEL(3) | CCM_CBCMR_FLEXSPI2_CLK_SEL(1) | CCM_CBCMR_FLEXSPI2_PODF(7);
        MEM_WriteU32(0x400FC018u, (uint32_t)(MEM_ReadU32(0x400FC018u) & (~0xE00C0300U)) | 0xE00C0100U);

        // Confgiure FLEXSPI CLOCKS
        //CCM->CSCMR1 = ((CCM->CSCMR1 & ~(CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK | CCM_CSCMR1_FLEXSPI_PODF_MASK | CCM_CSCMR1_PERCLK_PODF_MASK | CCM_CSCMR1_PERCLK_CLK_SEL_MASK)) |
        //               CCM_CSCMR1_FLEXSPI_CLK_SEL(3) | CCM_CSCMR1_FLEXSPI_PODF(7) | CCM_CSCMR1_PERCLK_PODF(1));
        MEM_WriteU32(0x400FC01Cu, (uint32_t)(MEM_ReadU32(0x400FC01Cu) & (~0x6380007FU)) | 0x63800001U);

        // Finally, Enable PLL_ARM, PLL_SYS and PLL_USB1
        //CCM_ANALOG->PLL_ARM &= ~CCM_ANALOG_PLL_ARM_BYPASS_MASK;
        MEM_WriteU32(0x400D8000u, MEM_ReadU32(0x400D8000u) & (~0x10000U));
        //CCM_ANALOG->PLL_SYS &= ~CCM_ANALOG_PLL_SYS_BYPASS_MASK;
        MEM_WriteU32(0x400D8030u, MEM_ReadU32(0x400D8030u) & (~0x10000U));
        //CCM_ANALOG->PLL_USB1 &= ~CCM_ANALOG_PLL_USB1_BYPASS_MASK;
        MEM_WriteU32(0x400D8010u, MEM_ReadU32(0x400D8010u) & (~0x10000U));
    }
}

