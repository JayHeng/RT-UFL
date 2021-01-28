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

void ufl_init_hardware_imxrt105x(void)
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
}

