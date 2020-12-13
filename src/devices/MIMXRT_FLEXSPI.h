/*
** ###################################################################
**     Processors:          MIMXRT685SFAWBR_cm33
**                          MIMXRT685SFFOB_cm33
**                          MIMXRT685SFVKB_cm33
**
**     Compilers:           GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          Keil ARM C/C++ Compiler
**                          MCUXpresso Compiler
**
**     Reference manual:    MIMXRT685 User manual Rev. 0.95 11 November 2019
**     Version:             rev. 2.0, 2019-11-12
**     Build:               b200413
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MIMXRT685S_cm33
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2020 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2018-06-19)
**         Initial version.
**     - rev. 2.0 (2019-11-12)
**         Base on rev 0.95 RM (B0 Header)
**
** ###################################################################
*/

/*!
 * @file MIMXRT685S_cm33.h
 * @version 2.0
 * @date 2019-11-12
 * @brief CMSIS Peripheral Access Layer for MIMXRT685S_cm33
 *
 * CMSIS Peripheral Access Layer for MIMXRT685S_cm33
 */

#ifndef _MIMXRT_FLEXSPI_H_
#define _MIMXRT_FLEXSPI_H_                      /**< Symbol preventing repeated inclusion */

#include "core_cm7.h"                 /* Core Peripheral Access Layer */

/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */

/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
  #else
    #pragma push
    #pragma anon_unions
  #endif
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- FLEXSPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FLEXSPI_Peripheral_Access_Layer FLEXSPI Peripheral Access Layer
 * @{
 */

/** FLEXSPI - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR0;                              /**< Module Control Register 0, offset: 0x0 */
  __IO uint32_t MCR1;                              /**< Module Control Register 1, offset: 0x4 */
  __IO uint32_t MCR2;                              /**< Module Control Register 2, offset: 0x8 */
  __IO uint32_t AHBCR;                             /**< AHB Bus Control Register, offset: 0xC */
  __IO uint32_t INTEN;                             /**< Interrupt Enable Register, offset: 0x10 */
  __IO uint32_t INTR;                              /**< Interrupt Register, offset: 0x14 */
  __IO uint32_t LUTKEY;                            /**< LUT Key Register, offset: 0x18 */
  __IO uint32_t LUTCR;                             /**< LUT Control Register, offset: 0x1C */
  __IO uint32_t AHBRXBUFCR0[8];                    /**< AHB RX Buffer 0 Control Register 0..AHB RX Buffer 7 Control Register 0, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_0[32];
  __IO uint32_t FLSHCR0[4];                        /**< Flash Control Register 0, array offset: 0x60, array step: 0x4 */
  __IO uint32_t FLSHCR1[4];                        /**< Flash Control Register 1, array offset: 0x70, array step: 0x4 */
  __IO uint32_t FLSHCR2[4];                        /**< Flash Control Register 2, array offset: 0x80, array step: 0x4 */
       uint8_t RESERVED_1[4];
  __IO uint32_t FLSHCR4;                           /**< Flash Control Register 4, offset: 0x94 */
       uint8_t RESERVED_2[8];
  __IO uint32_t IPCR0;                             /**< IP Control Register 0, offset: 0xA0 */
  __IO uint32_t IPCR1;                             /**< IP Control Register 1, offset: 0xA4 */
       uint8_t RESERVED_3[8];
  __IO uint32_t IPCMD;                             /**< IP Command Register, offset: 0xB0 */
  __IO uint32_t DLPR;                              /**< Data Learn Pattern Register, offset: 0xB4 */
  __IO uint32_t IPRXFCR;                           /**< IP RX FIFO Control Register, offset: 0xB8 */
  __IO uint32_t IPTXFCR;                           /**< IP TX FIFO Control Register, offset: 0xBC */
  __IO uint32_t DLLCR[2];                          /**< DLL Control Register 0, array offset: 0xC0, array step: 0x4 */
       uint8_t RESERVED_4[24];
  __I  uint32_t STS0;                              /**< Status Register 0, offset: 0xE0 */
  __I  uint32_t STS1;                              /**< Status Register 1, offset: 0xE4 */
  __I  uint32_t STS2;                              /**< Status Register 2, offset: 0xE8 */
  __I  uint32_t AHBSPNDSTS;                        /**< AHB Suspend Status Register, offset: 0xEC */
  __I  uint32_t IPRXFSTS;                          /**< IP RX FIFO Status Register, offset: 0xF0 */
  __I  uint32_t IPTXFSTS;                          /**< IP TX FIFO Status Register, offset: 0xF4 */
       uint8_t RESERVED_5[8];
  __I  uint32_t RFDR[32];                          /**< IP RX FIFO Data Register 0..IP RX FIFO Data Register 31, array offset: 0x100, array step: 0x4 */
  __O  uint32_t TFDR[32];                          /**< IP TX FIFO Data Register 0..IP TX FIFO Data Register 31, array offset: 0x180, array step: 0x4 */
  __IO uint32_t LUT[128];                          /**< LUT 0..LUT 127, array offset: 0x200, array step: 0x4 */
} FLEXSPI_Type;

/*!
 * @}
 */ /* end of group FLEXSPI_Peripheral_Access_Layer */
 
 /*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
  #else
    #pragma pop
  #endif
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


/* ----------------------------------------------------------------------------
   -- SDK Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDK_Compatibility_Symbols SDK Compatibility
 * @{
 */

/* No SDK compatibility issues. */

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */


#endif  /* _MIMXRT_FLEXSPI_H_ */


