/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_hardware_init.h"
#if defined(CPU_MIMXRT1176DVMAA_cm7)
#include "core_cm7.h"
#else
#include "core_scb.h"
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

void ufl_init_hardware_imxrt101x(void)
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

    //if (RTWDOG->CS & RTWDOG_CS_EN_MASK)
    if (MEM_ReadU32(0x400BC000u) & 0x80U)
    {
        //RTWDOG->CNT   = 0xD928C520U; // 0xD928C520U is the update key
        MEM_WriteU32(0x400BC004u, 0xD928C520U);
        //RTWDOG->TOVAL = 0xFFFF;
        MEM_WriteU32(0x400BC008u, 0xFFFF);
        //RTWDOG->CS    = (uint32_t)((RTWDOG->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;
        MEM_WriteU32(0x400BC000u, (uint32_t)(MEM_ReadU32(0x400BC000u) & (~0x80U)) | 0x20U);
    }

    /* Disable Systick which might be enabled by bootrom */
    //if (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)
    if (MEM_ReadU32(0xE000E010UL) & 1UL)
    {
        //SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        MEM_WriteU32(0xE000E010UL, (uint32_t)(MEM_ReadU32(0xE000E010UL) & (~1UL)));
    }

    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR))
    {
        SCB_DisableDCache();
    }
}

