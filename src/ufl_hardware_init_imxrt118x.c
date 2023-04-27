/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_hardware_init.h"
#include "core_scb.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/** RTWDOG - Register Layout Typedef */
typedef struct {
  volatile uint32_t CS;
  volatile uint32_t CNT;
  volatile uint32_t TOVAL;
  volatile uint32_t WIN;
} RT118X_RTWDOG_Type;

#define RT118X_RTWDOG_BASE_PTRS  { ((RT118X_RTWDOG_Type *)0x542D0000u), \
                                   ((RT118X_RTWDOG_Type *)0x542E0000u), \
                                   ((RT118X_RTWDOG_Type *)0x52490000u), \
                                   ((RT118X_RTWDOG_Type *)0x524A0000u), \
                                   ((RT118X_RTWDOG_Type *)0x524B0000u)}

/** XCACHE - Register Layout Typedef */
typedef struct {
  __IO uint32_t CCR;
  __IO uint32_t CLCR;
  __IO uint32_t CSAR;
  __IO uint32_t CCVR;
} RT118X_XCACHE_Type;

#define RT118X_XCACHE_PC_BASE       (0x44400000u)
#define RT118X_XCACHE_PC            ((RT118X_XCACHE_Type *)RT118X_XCACHE_PC_BASE)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/



/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

void ufl_init_hardware_imxrt118x(void)
{
#if 1
    /*
     * Prevent unexpected interrupt, only for FPGA?
     */
    //GPIO1_0_IRQn                 = 10,
  
    //if (__NVIC_GetEnableIRQ(10))
    uint32_t GPIO1_0_IRQn = 10;
    uint32_t *nvicIser = (uint32_t *)(0xE000E100UL + (GPIO1_0_IRQn >> 5UL) * 4);
    uint32_t *nvicIcer = (uint32_t *)(0xE000E180UL + (GPIO1_0_IRQn >> 5UL) * 4);
    if ((uint32_t)((((*nvicIser) & (1UL << ((GPIO1_0_IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL))
    {
        //__NVIC_DisableIRQ(10);
        *nvicIcer = (uint32_t)(1UL << (GPIO1_0_IRQn & 0x1FUL));
        __DSB();
        __ISB();
    }
  
    RT118X_RTWDOG_Type *rtwdogBase[] = RT118X_RTWDOG_BASE_PTRS;
    for (uint32_t i = 0; i < sizeof(rtwdogBase) / 4; i++)
    {
        if ((rtwdogBase[i]->CS & (0x2000U)) != 0U)
        {
            rtwdogBase[i]->CNT = 0xD928C520U;
        }
        else
        {
            rtwdogBase[i]->CNT = 0xC520U;
            rtwdogBase[i]->CNT = 0xD928U;
        }
        rtwdogBase[i]->TOVAL = 0xFFFF;
        rtwdogBase[i]->CS    = (uint32_t)((rtwdogBase[i]->CS) & ~(0x80U)) | (0x20U);
    }

    /* Disable Systick which might be enabled by bootrom */
    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0U)
    {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    }

    if ((RT118X_XCACHE_PC->CCR & 0x1U) == 0U) /* set XCACHE if not configured */
    {
        /* set command to invalidate all ways and write GO bit to initiate command */
        RT118X_XCACHE_PC->CCR = 0x4000000U | 0x1000000U;
        RT118X_XCACHE_PC->CCR |= 0x80000000U;
        /* Wait until the command completes */
        while ((RT118X_XCACHE_PC->CCR & 0x80000000U) != 0U)
        {
        }
        /* Enable cache */
        RT118X_XCACHE_PC->CCR = 0x1U;

        __ISB();
        __DSB();
    }

    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    __DSB();
    __ISB();

#endif
}

