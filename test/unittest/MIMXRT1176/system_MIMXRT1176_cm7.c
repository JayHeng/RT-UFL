/*
** ###################################################################
**     Processors:          MIMXRT1176AVM8A_cm7
**                          MIMXRT1176CVM8A_cm7
**                          MIMXRT1176DVMAA_cm7
**
**     Compilers:           Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          Keil ARM C/C++ Compiler
**                          MCUXpresso Compiler
**
**     Reference manual:    IMXRT1170RM, Rev E, 12/2019
**     Version:             rev. 0.1, 2018-03-05
**     Build:               b200219
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2020 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 0.1 (2018-03-05)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file MIMXRT1176_cm7
 * @version 0.1
 * @date 2018-03-05
 * @brief Device specific configuration file for MIMXRT1176_cm7 (implementation
 *        file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"
#include "system_MIMXRT1176_cm7.h" 



/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access in Secure mode */
  #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
  SCB_NS->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access in Non-secure mode */
  #endif /* (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

#if defined(__MCUXPRESSO)
    extern uint32_t g_pfnVectors[];  // Vector table defined in startup code
    SCB->VTOR = (uint32_t)g_pfnVectors;
#endif

/* Watchdog disable */

#if (DISABLE_WDOG)
    if ((WDOG1->WCR & WDOG_WCR_WDE_MASK) != 0U)
    {
        WDOG1->WCR &= ~WDOG_WCR_WDE_MASK;
    }
    if ((WDOG2->WCR & WDOG_WCR_WDE_MASK) != 0U)
    {
        WDOG2->WCR &= ~WDOG_WCR_WDE_MASK;
    }
    if ((RTWDOG3->CS & RTWDOG_CS_CMD32EN_MASK) != 0U)
    {
        RTWDOG3->CNT = 0xD928C520U; /* 0xD928C520U is the update key */
    }
    else
    {
        RTWDOG3->CNT = 0xC520U;
        RTWDOG3->CNT = 0xD928U;
    }
    RTWDOG3->TOVAL = 0xFFFF;
    RTWDOG3->CS = (uint32_t) ((RTWDOG3->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;
    if ((RTWDOG4->CS & RTWDOG_CS_CMD32EN_MASK) != 0U)
    {
        RTWDOG4->CNT = 0xD928C520U; /* 0xD928C520U is the update key */
    }
    else
    {
        RTWDOG4->CNT = 0xC520U;
        RTWDOG4->CNT = 0xD928U;
    }
    RTWDOG4->TOVAL = 0xFFFF;
    RTWDOG4->CS = (uint32_t) ((RTWDOG4->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;
#endif /* (DISABLE_WDOG) */

    /* Disable Systick which might be enabled by bootrom */
    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0U)
    {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    }

/* Enable instruction and data caches */
#if defined(__ICACHE_PRESENT) && __ICACHE_PRESENT
    if (SCB_CCR_IC_Msk != (SCB_CCR_IC_Msk & SCB->CCR)) {
        SCB_EnableICache();
    }
#endif
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
    if (SCB_CCR_DC_Msk != (SCB_CCR_DC_Msk & SCB->CCR)) {
        SCB_EnableDCache();
    }
#endif

    /* Clear bit 13 to its reset value since it might be set by ROM. */
    IOMUXC_GPR->GPR28 &= ~IOMUXC_GPR_GPR28_CACHE_USB_MASK;

    /* When ECC is enabled, SRC->SRSR need to be cleared since only correct SRSR value can trigger ROM ECC preload procedure.
       Save SRSR to SRC->GPR[10] so that application can still check SRSR value from SRC->GPR[10]. */
    SRC->GPR[10] = SRC->SRSR;
    /* clear SRSR */
    SRC->SRSR = 0xFFFFFFFF;

    /* Enable entry to thread mode when divide by zero */
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    __DSB();
    __ISB();

  SystemInitHook();
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

/* TBD */

}

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void) {
  /* Void implementation of the weak function. */
}
