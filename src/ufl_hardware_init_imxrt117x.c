/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_hardware_init.h"
#if defined(CPU_MIMXRT1176DVMAA_cm7)
#include "fsl_clock.h"
#endif
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

#if defined(CPU_MIMXRT1176DVMAA_cm7)
static void restore_clock(void)
{
    uint32_t i = 0;
    for (i = 0; i < 79; i++)
    {
        CCM->CLOCK_ROOT[i].CONTROL = 0;
    }
}
#endif

void ufl_init_hardware_imxrt117x(void)
{
#if defined(CPU_MIMXRT1176DVMAA_cm7)
    if ((*(uint32_t *)0x40C84800) == 0x1170A0)
    {
        if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR))
        {
            SCB_DisableDCache();
        }

        restore_clock();
        CCM->CLOCK_ROOT[kCLOCK_Root_M7].CONTROL = 0x200;
        CCM->CLOCK_ROOT[kCLOCK_Root_M4].CONTROL = 0x201;
        // PLL LDO shall be enabled first before enable PLLs 
        CLOCK_EnableOsc24M();
        // SYS PLL2 528MHz.
        const clock_sys_pll_config_t sysPllConfig = {
            .loopDivider = 1,
            // Using 24Mhz OSC 
            .mfn = 0,
            .mfi = 22,
        };
        CLOCK_InitSysPll2(&sysPllConfig);
        const clock_sys_pll3_config_t sysPll3Config = {
            .divSelect = 3,
        };
        CLOCK_InitSysPll3(&sysPll3Config);

        CCM->CLOCK_ROOT[kCLOCK_Root_Flexspi1].CONTROL_SET = 0x503;
    }

    SRC->GPR[9]            = 0;
    IOMUXC_LPSR_GPR->GPR26 = 0x4200;
#endif
}

