/*
** ###################################################################
**     Processors:          MIMXRT1062CVJ5A
**                          MIMXRT1062CVL5A
**                          MIMXRT1062DVJ6A
**                          MIMXRT1062DVL6A
**
**     Compilers:           Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          Keil ARM C/C++ Compiler
**                          MCUXpresso Compiler
**
**     Reference manual:    IMXRT1060RM Rev.1, 12/2018 | IMXRT1060SRM Rev.3
**     Version:             rev. 1.2, 2019-04-29
**     Build:               b191115
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MIMXRT1062
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2019 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 0.1 (2017-01-10)
**         Initial version.
**     - rev. 1.0 (2018-11-16)
**         Update header files to align with IMXRT1060RM Rev.0.
**     - rev. 1.1 (2018-11-27)
**         Update header files to align with IMXRT1060RM Rev.1.
**     - rev. 1.2 (2019-04-29)
**         Add SET/CLR/TOG register group to register CTRL, STAT, CHANNELCTRL, CH0STAT, CH0OPTS, CH1STAT, CH1OPTS, CH2STAT, CH2OPTS, CH3STAT, CH3OPTS of DCP module.
**
** ###################################################################
*/

/*!
 * @file MIMXRT1062.h
 * @version 1.2
 * @date 2019-04-29
 * @brief CMSIS Peripheral Access Layer for MIMXRT1062
 *
 * CMSIS Peripheral Access Layer for MIMXRT1062
 */

#ifndef _MIMXRT1062_H_
#define _MIMXRT1062_H_                           /**< Symbol preventing repeated inclusion */

#include "core_scb.h"                  /* Core Peripheral Access Layer */


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
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif


/* ----------------------------------------------------------------------------
   -- CCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CCM_Peripheral_Access_Layer CCM Peripheral Access Layer
 * @{
 */

/** CCM - Register Layout Typedef */
typedef struct {
  __IO uint32_t CCR;                               /**< CCM Control Register, offset: 0x0 */
       uint8_t RESERVED_0[4];
  __I  uint32_t CSR;                               /**< CCM Status Register, offset: 0x8 */
  __IO uint32_t CCSR;                              /**< CCM Clock Switcher Register, offset: 0xC */
  __IO uint32_t CACRR;                             /**< CCM Arm Clock Root Register, offset: 0x10 */
  __IO uint32_t CBCDR;                             /**< CCM Bus Clock Divider Register, offset: 0x14 */
  __IO uint32_t CBCMR;                             /**< CCM Bus Clock Multiplexer Register, offset: 0x18 */
  __IO uint32_t CSCMR1;                            /**< CCM Serial Clock Multiplexer Register 1, offset: 0x1C */
  __IO uint32_t CSCMR2;                            /**< CCM Serial Clock Multiplexer Register 2, offset: 0x20 */
  __IO uint32_t CSCDR1;                            /**< CCM Serial Clock Divider Register 1, offset: 0x24 */
  __IO uint32_t CS1CDR;                            /**< CCM Clock Divider Register, offset: 0x28 */
  __IO uint32_t CS2CDR;                            /**< CCM Clock Divider Register, offset: 0x2C */
  __IO uint32_t CDCDR;                             /**< CCM D1 Clock Divider Register, offset: 0x30 */
       uint8_t RESERVED_1[4];
  __IO uint32_t CSCDR2;                            /**< CCM Serial Clock Divider Register 2, offset: 0x38 */
  __IO uint32_t CSCDR3;                            /**< CCM Serial Clock Divider Register 3, offset: 0x3C */
       uint8_t RESERVED_2[8];
  __I  uint32_t CDHIPR;                            /**< CCM Divider Handshake In-Process Register, offset: 0x48 */
       uint8_t RESERVED_3[8];
  __IO uint32_t CLPCR;                             /**< CCM Low Power Control Register, offset: 0x54 */
  __IO uint32_t CISR;                              /**< CCM Interrupt Status Register, offset: 0x58 */
  __IO uint32_t CIMR;                              /**< CCM Interrupt Mask Register, offset: 0x5C */
  __IO uint32_t CCOSR;                             /**< CCM Clock Output Source Register, offset: 0x60 */
  __IO uint32_t CGPR;                              /**< CCM General Purpose Register, offset: 0x64 */
  __IO uint32_t CCGR0;                             /**< CCM Clock Gating Register 0, offset: 0x68 */
  __IO uint32_t CCGR1;                             /**< CCM Clock Gating Register 1, offset: 0x6C */
  __IO uint32_t CCGR2;                             /**< CCM Clock Gating Register 2, offset: 0x70 */
  __IO uint32_t CCGR3;                             /**< CCM Clock Gating Register 3, offset: 0x74 */
  __IO uint32_t CCGR4;                             /**< CCM Clock Gating Register 4, offset: 0x78 */
  __IO uint32_t CCGR5;                             /**< CCM Clock Gating Register 5, offset: 0x7C */
  __IO uint32_t CCGR6;                             /**< CCM Clock Gating Register 6, offset: 0x80 */
  __IO uint32_t CCGR7;                             /**< CCM Clock Gating Register 7, offset: 0x84 */
  __IO uint32_t CMEOR;                             /**< CCM Module Enable Overide Register, offset: 0x88 */
} CCM_Type;

/* ----------------------------------------------------------------------------
   -- CCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CCM_Register_Masks CCM Register Masks
 * @{
 */

/*! @name CCR - CCM Control Register */
/*! @{ */
#define CCM_CCR_OSCNT_MASK                       (0xFFU)
#define CCM_CCR_OSCNT_SHIFT                      (0U)
/*! OSCNT - Oscillator ready counter value. These bits define value of 32KHz counter, that serve as
 *    counter for oscillator lock time (count to n+1 ckil's). This is used for oscillator lock time.
 *    Current estimation is ~5ms. This counter will be used in ignition sequence and in wake from
 *    stop sequence if sbyos bit was defined, to notify that on chip oscillator output is ready for
 *    the dpll_ip to use and only then the gate in dpll_ip can be opened.
 */
#define CCM_CCR_OSCNT(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCR_OSCNT_SHIFT)) & CCM_CCR_OSCNT_MASK)
#define CCM_CCR_COSC_EN_MASK                     (0x1000U)
#define CCM_CCR_COSC_EN_SHIFT                    (12U)
/*! COSC_EN
 *  0b0..disable on chip oscillator
 *  0b1..enable on chip oscillator
 */
#define CCM_CCR_COSC_EN(x)                       (((uint32_t)(((uint32_t)(x)) << CCM_CCR_COSC_EN_SHIFT)) & CCM_CCR_COSC_EN_MASK)
#define CCM_CCR_REG_BYPASS_COUNT_MASK            (0x7E00000U)
#define CCM_CCR_REG_BYPASS_COUNT_SHIFT           (21U)
/*! REG_BYPASS_COUNT
 *  0b000000..no delay
 *  0b000001..1 CKIL clock period delay
 *  0b111111..63 CKIL clock periods delay
 */
#define CCM_CCR_REG_BYPASS_COUNT(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CCR_REG_BYPASS_COUNT_SHIFT)) & CCM_CCR_REG_BYPASS_COUNT_MASK)
#define CCM_CCR_RBC_EN_MASK                      (0x8000000U)
#define CCM_CCR_RBC_EN_SHIFT                     (27U)
/*! RBC_EN
 *  0b1..REG_BYPASS_COUNTER enabled.
 *  0b0..REG_BYPASS_COUNTER disabled
 */
#define CCM_CCR_RBC_EN(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCR_RBC_EN_SHIFT)) & CCM_CCR_RBC_EN_MASK)
/*! @} */

/*! @name CSR - CCM Status Register */
/*! @{ */
#define CCM_CSR_REF_EN_B_MASK                    (0x1U)
#define CCM_CSR_REF_EN_B_SHIFT                   (0U)
/*! REF_EN_B
 *  0b0..value of CCM_REF_EN_B is '0'
 *  0b1..value of CCM_REF_EN_B is '1'
 */
#define CCM_CSR_REF_EN_B(x)                      (((uint32_t)(((uint32_t)(x)) << CCM_CSR_REF_EN_B_SHIFT)) & CCM_CSR_REF_EN_B_MASK)
#define CCM_CSR_CAMP2_READY_MASK                 (0x8U)
#define CCM_CSR_CAMP2_READY_SHIFT                (3U)
/*! CAMP2_READY
 *  0b0..CAMP2 is not ready.
 *  0b1..CAMP2 is ready.
 */
#define CCM_CSR_CAMP2_READY(x)                   (((uint32_t)(((uint32_t)(x)) << CCM_CSR_CAMP2_READY_SHIFT)) & CCM_CSR_CAMP2_READY_MASK)
#define CCM_CSR_COSC_READY_MASK                  (0x20U)
#define CCM_CSR_COSC_READY_SHIFT                 (5U)
/*! COSC_READY
 *  0b0..on board oscillator is not ready.
 *  0b1..on board oscillator is ready.
 */
#define CCM_CSR_COSC_READY(x)                    (((uint32_t)(((uint32_t)(x)) << CCM_CSR_COSC_READY_SHIFT)) & CCM_CSR_COSC_READY_MASK)
/*! @} */

/*! @name CCSR - CCM Clock Switcher Register */
/*! @{ */
#define CCM_CCSR_PLL3_SW_CLK_SEL_MASK            (0x1U)
#define CCM_CCSR_PLL3_SW_CLK_SEL_SHIFT           (0U)
/*! PLL3_SW_CLK_SEL
 *  0b0..pll3_main_clk
 *  0b1..pll3 bypass clock
 */
#define CCM_CCSR_PLL3_SW_CLK_SEL(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CCSR_PLL3_SW_CLK_SEL_SHIFT)) & CCM_CCSR_PLL3_SW_CLK_SEL_MASK)
/*! @} */

/*! @name CACRR - CCM Arm Clock Root Register */
/*! @{ */
#define CCM_CACRR_ARM_PODF_MASK                  (0x7U)
#define CCM_CACRR_ARM_PODF_SHIFT                 (0U)
/*! ARM_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CACRR_ARM_PODF(x)                    (((uint32_t)(((uint32_t)(x)) << CCM_CACRR_ARM_PODF_SHIFT)) & CCM_CACRR_ARM_PODF_MASK)
/*! @} */

/*! @name CBCDR - CCM Bus Clock Divider Register */
/*! @{ */
#define CCM_CBCDR_SEMC_CLK_SEL_MASK              (0x40U)
#define CCM_CBCDR_SEMC_CLK_SEL_SHIFT             (6U)
/*! SEMC_CLK_SEL
 *  0b0..Periph_clk output will be used as SEMC clock root
 *  0b1..SEMC alternative clock will be used as SEMC clock root
 */
#define CCM_CBCDR_SEMC_CLK_SEL(x)                (((uint32_t)(((uint32_t)(x)) << CCM_CBCDR_SEMC_CLK_SEL_SHIFT)) & CCM_CBCDR_SEMC_CLK_SEL_MASK)
#define CCM_CBCDR_SEMC_ALT_CLK_SEL_MASK          (0x80U)
#define CCM_CBCDR_SEMC_ALT_CLK_SEL_SHIFT         (7U)
/*! SEMC_ALT_CLK_SEL
 *  0b0..PLL2 PFD2 will be selected as alternative clock for SEMC root clock
 *  0b1..PLL3 PFD1 will be selected as alternative clock for SEMC root clock
 */
#define CCM_CBCDR_SEMC_ALT_CLK_SEL(x)            (((uint32_t)(((uint32_t)(x)) << CCM_CBCDR_SEMC_ALT_CLK_SEL_SHIFT)) & CCM_CBCDR_SEMC_ALT_CLK_SEL_MASK)
#define CCM_CBCDR_IPG_PODF_MASK                  (0x300U)
#define CCM_CBCDR_IPG_PODF_SHIFT                 (8U)
/*! IPG_PODF
 *  0b00..divide by 1
 *  0b01..divide by 2
 *  0b10..divide by 3
 *  0b11..divide by 4
 */
#define CCM_CBCDR_IPG_PODF(x)                    (((uint32_t)(((uint32_t)(x)) << CCM_CBCDR_IPG_PODF_SHIFT)) & CCM_CBCDR_IPG_PODF_MASK)
#define CCM_CBCDR_AHB_PODF_MASK                  (0x1C00U)
#define CCM_CBCDR_AHB_PODF_SHIFT                 (10U)
/*! AHB_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CBCDR_AHB_PODF(x)                    (((uint32_t)(((uint32_t)(x)) << CCM_CBCDR_AHB_PODF_SHIFT)) & CCM_CBCDR_AHB_PODF_MASK)
#define CCM_CBCDR_SEMC_PODF_MASK                 (0x70000U)
#define CCM_CBCDR_SEMC_PODF_SHIFT                (16U)
/*! SEMC_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CBCDR_SEMC_PODF(x)                   (((uint32_t)(((uint32_t)(x)) << CCM_CBCDR_SEMC_PODF_SHIFT)) & CCM_CBCDR_SEMC_PODF_MASK)
#define CCM_CBCDR_PERIPH_CLK_SEL_MASK            (0x2000000U)
#define CCM_CBCDR_PERIPH_CLK_SEL_SHIFT           (25U)
/*! PERIPH_CLK_SEL
 *  0b0..derive clock from pre_periph_clk_sel
 *  0b1..derive clock from periph_clk2_clk_divided
 */
#define CCM_CBCDR_PERIPH_CLK_SEL(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CBCDR_PERIPH_CLK_SEL_SHIFT)) & CCM_CBCDR_PERIPH_CLK_SEL_MASK)
#define CCM_CBCDR_PERIPH_CLK2_PODF_MASK          (0x38000000U)
#define CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT         (27U)
/*! PERIPH_CLK2_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CBCDR_PERIPH_CLK2_PODF(x)            (((uint32_t)(((uint32_t)(x)) << CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT)) & CCM_CBCDR_PERIPH_CLK2_PODF_MASK)
/*! @} */

/*! @name CBCMR - CCM Bus Clock Multiplexer Register */
/*! @{ */
#define CCM_CBCMR_LPSPI_CLK_SEL_MASK             (0x30U)
#define CCM_CBCMR_LPSPI_CLK_SEL_SHIFT            (4U)
/*! LPSPI_CLK_SEL
 *  0b00..derive clock from PLL3 PFD1 clk
 *  0b01..derive clock from PLL3 PFD0
 *  0b10..derive clock from PLL2
 *  0b11..derive clock from PLL2 PFD2
 */
#define CCM_CBCMR_LPSPI_CLK_SEL(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CBCMR_LPSPI_CLK_SEL_SHIFT)) & CCM_CBCMR_LPSPI_CLK_SEL_MASK)
#define CCM_CBCMR_FLEXSPI2_CLK_SEL_MASK          (0x300U)
#define CCM_CBCMR_FLEXSPI2_CLK_SEL_SHIFT         (8U)
/*! FLEXSPI2_CLK_SEL
 *  0b00..derive clock from PLL2 PFD2
 *  0b01..derive clock from PLL3 PFD0
 *  0b10..derive clock from PLL3 PFD1
 *  0b11..derive clock from PLL2 (pll2_main_clk)
 */
#define CCM_CBCMR_FLEXSPI2_CLK_SEL(x)            (((uint32_t)(((uint32_t)(x)) << CCM_CBCMR_FLEXSPI2_CLK_SEL_SHIFT)) & CCM_CBCMR_FLEXSPI2_CLK_SEL_MASK)
#define CCM_CBCMR_PERIPH_CLK2_SEL_MASK           (0x3000U)
#define CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT          (12U)
/*! PERIPH_CLK2_SEL
 *  0b00..derive clock from pll3_sw_clk
 *  0b01..derive clock from osc_clk (pll1_ref_clk)
 *  0b10..derive clock from pll2_bypass_clk
 *  0b11..reserved
 */
#define CCM_CBCMR_PERIPH_CLK2_SEL(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CBCMR_PERIPH_CLK2_SEL_SHIFT)) & CCM_CBCMR_PERIPH_CLK2_SEL_MASK)
#define CCM_CBCMR_TRACE_CLK_SEL_MASK             (0xC000U)
#define CCM_CBCMR_TRACE_CLK_SEL_SHIFT            (14U)
/*! TRACE_CLK_SEL
 *  0b00..derive clock from PLL2
 *  0b01..derive clock from PLL2 PFD2
 *  0b10..derive clock from PLL2 PFD0
 *  0b11..derive clock from PLL2 PFD1
 */
#define CCM_CBCMR_TRACE_CLK_SEL(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CBCMR_TRACE_CLK_SEL_SHIFT)) & CCM_CBCMR_TRACE_CLK_SEL_MASK)
#define CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK        (0xC0000U)
#define CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT       (18U)
/*! PRE_PERIPH_CLK_SEL
 *  0b00..derive clock from PLL2
 *  0b01..derive clock from PLL2 PFD2
 *  0b10..derive clock from PLL2 PFD0
 *  0b11..derive clock from divided PLL1
 */
#define CCM_CBCMR_PRE_PERIPH_CLK_SEL(x)          (((uint32_t)(((uint32_t)(x)) << CCM_CBCMR_PRE_PERIPH_CLK_SEL_SHIFT)) & CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK)
#define CCM_CBCMR_LCDIF_PODF_MASK                (0x3800000U)
#define CCM_CBCMR_LCDIF_PODF_SHIFT               (23U)
/*! LCDIF_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CBCMR_LCDIF_PODF(x)                  (((uint32_t)(((uint32_t)(x)) << CCM_CBCMR_LCDIF_PODF_SHIFT)) & CCM_CBCMR_LCDIF_PODF_MASK)
#define CCM_CBCMR_LPSPI_PODF_MASK                (0x1C000000U)
#define CCM_CBCMR_LPSPI_PODF_SHIFT               (26U)
/*! LPSPI_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CBCMR_LPSPI_PODF(x)                  (((uint32_t)(((uint32_t)(x)) << CCM_CBCMR_LPSPI_PODF_SHIFT)) & CCM_CBCMR_LPSPI_PODF_MASK)
#define CCM_CBCMR_FLEXSPI2_PODF_MASK             (0xE0000000U)
#define CCM_CBCMR_FLEXSPI2_PODF_SHIFT            (29U)
/*! FLEXSPI2_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CBCMR_FLEXSPI2_PODF(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CBCMR_FLEXSPI2_PODF_SHIFT)) & CCM_CBCMR_FLEXSPI2_PODF_MASK)
/*! @} */

/*! @name CSCMR1 - CCM Serial Clock Multiplexer Register 1 */
/*! @{ */
#define CCM_CSCMR1_PERCLK_PODF_MASK              (0x3FU)
#define CCM_CSCMR1_PERCLK_PODF_SHIFT             (0U)
/*! PERCLK_PODF - Divider for perclk podf.
 *  0b000000..Divide by 1
 *  0b000001..Divide by 2
 *  0b000010..Divide by 3
 *  0b000011..Divide by 4
 *  0b000100..Divide by 5
 *  0b000101..Divide by 6
 *  0b000110..Divide by 7
 *  0b000111..Divide by 8
 *  0b001000..Divide by 9
 *  0b001001..Divide by 10
 *  0b001010..Divide by 11
 *  0b001011..Divide by 12
 *  0b001100..Divide by 13
 *  0b001101..Divide by 14
 *  0b001110..Divide by 15
 *  0b001111..Divide by 16
 *  0b010000..Divide by 17
 *  0b010001..Divide by 18
 *  0b010010..Divide by 19
 *  0b010011..Divide by 20
 *  0b010100..Divide by 21
 *  0b010101..Divide by 22
 *  0b010110..Divide by 23
 *  0b010111..Divide by 24
 *  0b011000..Divide by 25
 *  0b011001..Divide by 26
 *  0b011010..Divide by 27
 *  0b011011..Divide by 28
 *  0b011100..Divide by 29
 *  0b011101..Divide by 30
 *  0b011110..Divide by 31
 *  0b011111..Divide by 32
 *  0b100000..Divide by 33
 *  0b100001..Divide by 34
 *  0b100010..Divide by 35
 *  0b100011..Divide by 36
 *  0b100100..Divide by 37
 *  0b100101..Divide by 38
 *  0b100110..Divide by 39
 *  0b100111..Divide by 40
 *  0b101000..Divide by 41
 *  0b101001..Divide by 42
 *  0b101010..Divide by 43
 *  0b101011..Divide by 44
 *  0b101100..Divide by 45
 *  0b101101..Divide by 46
 *  0b101110..Divide by 47
 *  0b101111..Divide by 48
 *  0b110000..Divide by 49
 *  0b110001..Divide by 50
 *  0b110010..Divide by 51
 *  0b110011..Divide by 52
 *  0b110100..Divide by 53
 *  0b110101..Divide by 54
 *  0b110110..Divide by 55
 *  0b110111..Divide by 56
 *  0b111000..Divide by 57
 *  0b111001..Divide by 58
 *  0b111010..Divide by 59
 *  0b111011..Divide by 60
 *  0b111100..Divide by 61
 *  0b111101..Divide by 62
 *  0b111110..Divide by 63
 *  0b111111..Divide by 64
 */
#define CCM_CSCMR1_PERCLK_PODF(x)                (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_PERCLK_PODF_SHIFT)) & CCM_CSCMR1_PERCLK_PODF_MASK)
#define CCM_CSCMR1_PERCLK_CLK_SEL_MASK           (0x40U)
#define CCM_CSCMR1_PERCLK_CLK_SEL_SHIFT          (6U)
/*! PERCLK_CLK_SEL
 *  0b0..derive clock from ipg clk root
 *  0b1..derive clock from osc_clk
 */
#define CCM_CSCMR1_PERCLK_CLK_SEL(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_PERCLK_CLK_SEL_SHIFT)) & CCM_CSCMR1_PERCLK_CLK_SEL_MASK)
#define CCM_CSCMR1_SAI1_CLK_SEL_MASK             (0xC00U)
#define CCM_CSCMR1_SAI1_CLK_SEL_SHIFT            (10U)
/*! SAI1_CLK_SEL
 *  0b00..derive clock from PLL3 PFD2
 *  0b01..derive clock from PLL5
 *  0b10..derive clock from PLL4
 *  0b11..Reserved
 */
#define CCM_CSCMR1_SAI1_CLK_SEL(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_SAI1_CLK_SEL_SHIFT)) & CCM_CSCMR1_SAI1_CLK_SEL_MASK)
#define CCM_CSCMR1_SAI2_CLK_SEL_MASK             (0x3000U)
#define CCM_CSCMR1_SAI2_CLK_SEL_SHIFT            (12U)
/*! SAI2_CLK_SEL
 *  0b00..derive clock from PLL3 PFD2
 *  0b01..derive clock from PLL5
 *  0b10..derive clock from PLL4
 *  0b11..Reserved
 */
#define CCM_CSCMR1_SAI2_CLK_SEL(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_SAI2_CLK_SEL_SHIFT)) & CCM_CSCMR1_SAI2_CLK_SEL_MASK)
#define CCM_CSCMR1_SAI3_CLK_SEL_MASK             (0xC000U)
#define CCM_CSCMR1_SAI3_CLK_SEL_SHIFT            (14U)
/*! SAI3_CLK_SEL
 *  0b00..derive clock from PLL3 PFD2
 *  0b01..derive clock from PLL5
 *  0b10..derive clock from PLL4
 *  0b11..Reserved
 */
#define CCM_CSCMR1_SAI3_CLK_SEL(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_SAI3_CLK_SEL_SHIFT)) & CCM_CSCMR1_SAI3_CLK_SEL_MASK)
#define CCM_CSCMR1_USDHC1_CLK_SEL_MASK           (0x10000U)
#define CCM_CSCMR1_USDHC1_CLK_SEL_SHIFT          (16U)
/*! USDHC1_CLK_SEL
 *  0b0..derive clock from PLL2 PFD2
 *  0b1..derive clock from PLL2 PFD0
 */
#define CCM_CSCMR1_USDHC1_CLK_SEL(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_USDHC1_CLK_SEL_SHIFT)) & CCM_CSCMR1_USDHC1_CLK_SEL_MASK)
#define CCM_CSCMR1_USDHC2_CLK_SEL_MASK           (0x20000U)
#define CCM_CSCMR1_USDHC2_CLK_SEL_SHIFT          (17U)
/*! USDHC2_CLK_SEL
 *  0b0..derive clock from PLL2 PFD2
 *  0b1..derive clock from PLL2 PFD0
 */
#define CCM_CSCMR1_USDHC2_CLK_SEL(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_USDHC2_CLK_SEL_SHIFT)) & CCM_CSCMR1_USDHC2_CLK_SEL_MASK)
#define CCM_CSCMR1_FLEXSPI_PODF_MASK             (0x3800000U)
#define CCM_CSCMR1_FLEXSPI_PODF_SHIFT            (23U)
/*! FLEXSPI_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CSCMR1_FLEXSPI_PODF(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_FLEXSPI_PODF_SHIFT)) & CCM_CSCMR1_FLEXSPI_PODF_MASK)
#define CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK          (0x60000000U)
#define CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT         (29U)
/*! FLEXSPI_CLK_SEL
 *  0b00..derive clock from semc_clk_root_pre
 *  0b01..derive clock from pll3_sw_clk
 *  0b10..derive clock from PLL2 PFD2
 *  0b11..derive clock from PLL3 PFD0
 */
#define CCM_CSCMR1_FLEXSPI_CLK_SEL(x)            (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)) & CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK)
/*! @} */

/*! @name CSCMR2 - CCM Serial Clock Multiplexer Register 2 */
/*! @{ */
#define CCM_CSCMR2_CAN_CLK_PODF_MASK             (0xFCU)
#define CCM_CSCMR2_CAN_CLK_PODF_SHIFT            (2U)
/*! CAN_CLK_PODF - Divider for CAN/CANFD clock podf.
 *  0b000000..Divide by 1
 *  0b000001..Divide by 2
 *  0b000010..Divide by 3
 *  0b000011..Divide by 4
 *  0b000100..Divide by 5
 *  0b000101..Divide by 6
 *  0b000110..Divide by 7
 *  0b000111..Divide by 8
 *  0b001000..Divide by 9
 *  0b001001..Divide by 10
 *  0b001010..Divide by 11
 *  0b001011..Divide by 12
 *  0b001100..Divide by 13
 *  0b001101..Divide by 14
 *  0b001110..Divide by 15
 *  0b001111..Divide by 16
 *  0b010000..Divide by 17
 *  0b010001..Divide by 18
 *  0b010010..Divide by 19
 *  0b010011..Divide by 20
 *  0b010100..Divide by 21
 *  0b010101..Divide by 22
 *  0b010110..Divide by 23
 *  0b010111..Divide by 24
 *  0b011000..Divide by 25
 *  0b011001..Divide by 26
 *  0b011010..Divide by 27
 *  0b011011..Divide by 28
 *  0b011100..Divide by 29
 *  0b011101..Divide by 30
 *  0b011110..Divide by 31
 *  0b011111..Divide by 32
 *  0b100000..Divide by 33
 *  0b100001..Divide by 34
 *  0b100010..Divide by 35
 *  0b100011..Divide by 36
 *  0b100100..Divide by 37
 *  0b100101..Divide by 38
 *  0b100110..Divide by 39
 *  0b100111..Divide by 40
 *  0b101000..Divide by 41
 *  0b101001..Divide by 42
 *  0b101010..Divide by 43
 *  0b101011..Divide by 44
 *  0b101100..Divide by 45
 *  0b101101..Divide by 46
 *  0b101110..Divide by 47
 *  0b101111..Divide by 48
 *  0b110000..Divide by 49
 *  0b110001..Divide by 50
 *  0b110010..Divide by 51
 *  0b110011..Divide by 52
 *  0b110100..Divide by 53
 *  0b110101..Divide by 54
 *  0b110110..Divide by 55
 *  0b110111..Divide by 56
 *  0b111000..Divide by 57
 *  0b111001..Divide by 58
 *  0b111010..Divide by 59
 *  0b111011..Divide by 60
 *  0b111100..Divide by 61
 *  0b111101..Divide by 62
 *  0b111110..Divide by 63
 *  0b111111..Divide by 64
 */
#define CCM_CSCMR2_CAN_CLK_PODF(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR2_CAN_CLK_PODF_SHIFT)) & CCM_CSCMR2_CAN_CLK_PODF_MASK)
#define CCM_CSCMR2_CAN_CLK_SEL_MASK              (0x300U)
#define CCM_CSCMR2_CAN_CLK_SEL_SHIFT             (8U)
/*! CAN_CLK_SEL
 *  0b00..derive clock from pll3_sw_clk divided clock (60M)
 *  0b01..derive clock from osc_clk (24M)
 *  0b10..derive clock from pll3_sw_clk divided clock (80M)
 *  0b11..Disable FlexCAN clock
 */
#define CCM_CSCMR2_CAN_CLK_SEL(x)                (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR2_CAN_CLK_SEL_SHIFT)) & CCM_CSCMR2_CAN_CLK_SEL_MASK)
#define CCM_CSCMR2_FLEXIO2_CLK_SEL_MASK          (0x180000U)
#define CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT         (19U)
/*! FLEXIO2_CLK_SEL
 *  0b00..derive clock from PLL4 divided clock
 *  0b01..derive clock from PLL3 PFD2 clock
 *  0b10..derive clock from PLL5 clock
 *  0b11..derive clock from pll3_sw_clk
 */
#define CCM_CSCMR2_FLEXIO2_CLK_SEL(x)            (((uint32_t)(((uint32_t)(x)) << CCM_CSCMR2_FLEXIO2_CLK_SEL_SHIFT)) & CCM_CSCMR2_FLEXIO2_CLK_SEL_MASK)
/*! @} */

/*! @name CSCDR1 - CCM Serial Clock Divider Register 1 */
/*! @{ */
#define CCM_CSCDR1_UART_CLK_PODF_MASK            (0x3FU)
#define CCM_CSCDR1_UART_CLK_PODF_SHIFT           (0U)
/*! UART_CLK_PODF - Divider for uart clock podf.
 *  0b000000..Divide by 1
 *  0b000001..Divide by 2
 *  0b000010..Divide by 3
 *  0b000011..Divide by 4
 *  0b000100..Divide by 5
 *  0b000101..Divide by 6
 *  0b000110..Divide by 7
 *  0b000111..Divide by 8
 *  0b001000..Divide by 9
 *  0b001001..Divide by 10
 *  0b001010..Divide by 11
 *  0b001011..Divide by 12
 *  0b001100..Divide by 13
 *  0b001101..Divide by 14
 *  0b001110..Divide by 15
 *  0b001111..Divide by 16
 *  0b010000..Divide by 17
 *  0b010001..Divide by 18
 *  0b010010..Divide by 19
 *  0b010011..Divide by 20
 *  0b010100..Divide by 21
 *  0b010101..Divide by 22
 *  0b010110..Divide by 23
 *  0b010111..Divide by 24
 *  0b011000..Divide by 25
 *  0b011001..Divide by 26
 *  0b011010..Divide by 27
 *  0b011011..Divide by 28
 *  0b011100..Divide by 29
 *  0b011101..Divide by 30
 *  0b011110..Divide by 31
 *  0b011111..Divide by 32
 *  0b100000..Divide by 33
 *  0b100001..Divide by 34
 *  0b100010..Divide by 35
 *  0b100011..Divide by 36
 *  0b100100..Divide by 37
 *  0b100101..Divide by 38
 *  0b100110..Divide by 39
 *  0b100111..Divide by 40
 *  0b101000..Divide by 41
 *  0b101001..Divide by 42
 *  0b101010..Divide by 43
 *  0b101011..Divide by 44
 *  0b101100..Divide by 45
 *  0b101101..Divide by 46
 *  0b101110..Divide by 47
 *  0b101111..Divide by 48
 *  0b110000..Divide by 49
 *  0b110001..Divide by 50
 *  0b110010..Divide by 51
 *  0b110011..Divide by 52
 *  0b110100..Divide by 53
 *  0b110101..Divide by 54
 *  0b110110..Divide by 55
 *  0b110111..Divide by 56
 *  0b111000..Divide by 57
 *  0b111001..Divide by 58
 *  0b111010..Divide by 59
 *  0b111011..Divide by 60
 *  0b111100..Divide by 61
 *  0b111101..Divide by 62
 *  0b111110..Divide by 63
 *  0b111111..Divide by 64
 */
#define CCM_CSCDR1_UART_CLK_PODF(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR1_UART_CLK_PODF_SHIFT)) & CCM_CSCDR1_UART_CLK_PODF_MASK)
#define CCM_CSCDR1_UART_CLK_SEL_MASK             (0x40U)
#define CCM_CSCDR1_UART_CLK_SEL_SHIFT            (6U)
/*! UART_CLK_SEL
 *  0b0..derive clock from pll3_80m
 *  0b1..derive clock from osc_clk
 */
#define CCM_CSCDR1_UART_CLK_SEL(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR1_UART_CLK_SEL_SHIFT)) & CCM_CSCDR1_UART_CLK_SEL_MASK)
#define CCM_CSCDR1_USDHC1_PODF_MASK              (0x3800U)
#define CCM_CSCDR1_USDHC1_PODF_SHIFT             (11U)
/*! USDHC1_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CSCDR1_USDHC1_PODF(x)                (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR1_USDHC1_PODF_SHIFT)) & CCM_CSCDR1_USDHC1_PODF_MASK)
#define CCM_CSCDR1_USDHC2_PODF_MASK              (0x70000U)
#define CCM_CSCDR1_USDHC2_PODF_SHIFT             (16U)
/*! USDHC2_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CSCDR1_USDHC2_PODF(x)                (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR1_USDHC2_PODF_SHIFT)) & CCM_CSCDR1_USDHC2_PODF_MASK)
#define CCM_CSCDR1_TRACE_PODF_MASK               (0x6000000U)
#define CCM_CSCDR1_TRACE_PODF_SHIFT              (25U)
/*! TRACE_PODF
 *  0b00..divide by 1
 *  0b01..divide by 2
 *  0b10..divide by 3
 *  0b11..divide by 4
 */
#define CCM_CSCDR1_TRACE_PODF(x)                 (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR1_TRACE_PODF_SHIFT)) & CCM_CSCDR1_TRACE_PODF_MASK)
/*! @} */

/*! @name CS1CDR - CCM Clock Divider Register */
/*! @{ */
#define CCM_CS1CDR_SAI1_CLK_PODF_MASK            (0x3FU)
#define CCM_CS1CDR_SAI1_CLK_PODF_SHIFT           (0U)
/*! SAI1_CLK_PODF - Divider for sai1 clock podf. The input clock to this divider should be lower
 *    than 300Mhz, the predivider can be used to achieve this.
 *  0b000000..Divide by 1
 *  0b000001..Divide by 2
 *  0b000010..Divide by 3
 *  0b000011..Divide by 4
 *  0b000100..Divide by 5
 *  0b000101..Divide by 6
 *  0b000110..Divide by 7
 *  0b000111..Divide by 8
 *  0b001000..Divide by 9
 *  0b001001..Divide by 10
 *  0b001010..Divide by 11
 *  0b001011..Divide by 12
 *  0b001100..Divide by 13
 *  0b001101..Divide by 14
 *  0b001110..Divide by 15
 *  0b001111..Divide by 16
 *  0b010000..Divide by 17
 *  0b010001..Divide by 18
 *  0b010010..Divide by 19
 *  0b010011..Divide by 20
 *  0b010100..Divide by 21
 *  0b010101..Divide by 22
 *  0b010110..Divide by 23
 *  0b010111..Divide by 24
 *  0b011000..Divide by 25
 *  0b011001..Divide by 26
 *  0b011010..Divide by 27
 *  0b011011..Divide by 28
 *  0b011100..Divide by 29
 *  0b011101..Divide by 30
 *  0b011110..Divide by 31
 *  0b011111..Divide by 32
 *  0b100000..Divide by 33
 *  0b100001..Divide by 34
 *  0b100010..Divide by 35
 *  0b100011..Divide by 36
 *  0b100100..Divide by 37
 *  0b100101..Divide by 38
 *  0b100110..Divide by 39
 *  0b100111..Divide by 40
 *  0b101000..Divide by 41
 *  0b101001..Divide by 42
 *  0b101010..Divide by 43
 *  0b101011..Divide by 44
 *  0b101100..Divide by 45
 *  0b101101..Divide by 46
 *  0b101110..Divide by 47
 *  0b101111..Divide by 48
 *  0b110000..Divide by 49
 *  0b110001..Divide by 50
 *  0b110010..Divide by 51
 *  0b110011..Divide by 52
 *  0b110100..Divide by 53
 *  0b110101..Divide by 54
 *  0b110110..Divide by 55
 *  0b110111..Divide by 56
 *  0b111000..Divide by 57
 *  0b111001..Divide by 58
 *  0b111010..Divide by 59
 *  0b111011..Divide by 60
 *  0b111100..Divide by 61
 *  0b111101..Divide by 62
 *  0b111110..Divide by 63
 *  0b111111..Divide by 64
 */
#define CCM_CS1CDR_SAI1_CLK_PODF(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CS1CDR_SAI1_CLK_PODF_SHIFT)) & CCM_CS1CDR_SAI1_CLK_PODF_MASK)
#define CCM_CS1CDR_SAI1_CLK_PRED_MASK            (0x1C0U)
#define CCM_CS1CDR_SAI1_CLK_PRED_SHIFT           (6U)
/*! SAI1_CLK_PRED
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CS1CDR_SAI1_CLK_PRED(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CS1CDR_SAI1_CLK_PRED_SHIFT)) & CCM_CS1CDR_SAI1_CLK_PRED_MASK)
#define CCM_CS1CDR_FLEXIO2_CLK_PRED_MASK         (0xE00U)
#define CCM_CS1CDR_FLEXIO2_CLK_PRED_SHIFT        (9U)
/*! FLEXIO2_CLK_PRED
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CS1CDR_FLEXIO2_CLK_PRED(x)           (((uint32_t)(((uint32_t)(x)) << CCM_CS1CDR_FLEXIO2_CLK_PRED_SHIFT)) & CCM_CS1CDR_FLEXIO2_CLK_PRED_MASK)
#define CCM_CS1CDR_SAI3_CLK_PODF_MASK            (0x3F0000U)
#define CCM_CS1CDR_SAI3_CLK_PODF_SHIFT           (16U)
/*! SAI3_CLK_PODF - Divider for sai3 clock podf. The input clock to this divider should be lower
 *    than 300Mhz, the predivider can be used to achieve this.
 *  0b000000..Divide by 1
 *  0b000001..Divide by 2
 *  0b000010..Divide by 3
 *  0b000011..Divide by 4
 *  0b000100..Divide by 5
 *  0b000101..Divide by 6
 *  0b000110..Divide by 7
 *  0b000111..Divide by 8
 *  0b001000..Divide by 9
 *  0b001001..Divide by 10
 *  0b001010..Divide by 11
 *  0b001011..Divide by 12
 *  0b001100..Divide by 13
 *  0b001101..Divide by 14
 *  0b001110..Divide by 15
 *  0b001111..Divide by 16
 *  0b010000..Divide by 17
 *  0b010001..Divide by 18
 *  0b010010..Divide by 19
 *  0b010011..Divide by 20
 *  0b010100..Divide by 21
 *  0b010101..Divide by 22
 *  0b010110..Divide by 23
 *  0b010111..Divide by 24
 *  0b011000..Divide by 25
 *  0b011001..Divide by 26
 *  0b011010..Divide by 27
 *  0b011011..Divide by 28
 *  0b011100..Divide by 29
 *  0b011101..Divide by 30
 *  0b011110..Divide by 31
 *  0b011111..Divide by 32
 *  0b100000..Divide by 33
 *  0b100001..Divide by 34
 *  0b100010..Divide by 35
 *  0b100011..Divide by 36
 *  0b100100..Divide by 37
 *  0b100101..Divide by 38
 *  0b100110..Divide by 39
 *  0b100111..Divide by 40
 *  0b101000..Divide by 41
 *  0b101001..Divide by 42
 *  0b101010..Divide by 43
 *  0b101011..Divide by 44
 *  0b101100..Divide by 45
 *  0b101101..Divide by 46
 *  0b101110..Divide by 47
 *  0b101111..Divide by 48
 *  0b110000..Divide by 49
 *  0b110001..Divide by 50
 *  0b110010..Divide by 51
 *  0b110011..Divide by 52
 *  0b110100..Divide by 53
 *  0b110101..Divide by 54
 *  0b110110..Divide by 55
 *  0b110111..Divide by 56
 *  0b111000..Divide by 57
 *  0b111001..Divide by 58
 *  0b111010..Divide by 59
 *  0b111011..Divide by 60
 *  0b111100..Divide by 61
 *  0b111101..Divide by 62
 *  0b111110..Divide by 63
 *  0b111111..Divide by 64
 */
#define CCM_CS1CDR_SAI3_CLK_PODF(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CS1CDR_SAI3_CLK_PODF_SHIFT)) & CCM_CS1CDR_SAI3_CLK_PODF_MASK)
#define CCM_CS1CDR_SAI3_CLK_PRED_MASK            (0x1C00000U)
#define CCM_CS1CDR_SAI3_CLK_PRED_SHIFT           (22U)
/*! SAI3_CLK_PRED
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CS1CDR_SAI3_CLK_PRED(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CS1CDR_SAI3_CLK_PRED_SHIFT)) & CCM_CS1CDR_SAI3_CLK_PRED_MASK)
#define CCM_CS1CDR_FLEXIO2_CLK_PODF_MASK         (0xE000000U)
#define CCM_CS1CDR_FLEXIO2_CLK_PODF_SHIFT        (25U)
/*! FLEXIO2_CLK_PODF - Divider for flexio2/flexio3 clock. Divider should be updated when output clock is gated.
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 3
 *  0b011..Divide by 4
 *  0b100..Divide by 5
 *  0b101..Divide by 6
 *  0b110..Divide by 7
 *  0b111..Divide by 8
 */
#define CCM_CS1CDR_FLEXIO2_CLK_PODF(x)           (((uint32_t)(((uint32_t)(x)) << CCM_CS1CDR_FLEXIO2_CLK_PODF_SHIFT)) & CCM_CS1CDR_FLEXIO2_CLK_PODF_MASK)
/*! @} */

/*! @name CS2CDR - CCM Clock Divider Register */
/*! @{ */
#define CCM_CS2CDR_SAI2_CLK_PODF_MASK            (0x3FU)
#define CCM_CS2CDR_SAI2_CLK_PODF_SHIFT           (0U)
/*! SAI2_CLK_PODF - Divider for sai2 clock podf. The input clock to this divider should be lower
 *    than 300Mhz, the predivider can be used to achieve this.
 *  0b000000..Divide by 1
 *  0b000001..Divide by 2
 *  0b000010..Divide by 3
 *  0b000011..Divide by 4
 *  0b000100..Divide by 5
 *  0b000101..Divide by 6
 *  0b000110..Divide by 7
 *  0b000111..Divide by 8
 *  0b001000..Divide by 9
 *  0b001001..Divide by 10
 *  0b001010..Divide by 11
 *  0b001011..Divide by 12
 *  0b001100..Divide by 13
 *  0b001101..Divide by 14
 *  0b001110..Divide by 15
 *  0b001111..Divide by 16
 *  0b010000..Divide by 17
 *  0b010001..Divide by 18
 *  0b010010..Divide by 19
 *  0b010011..Divide by 20
 *  0b010100..Divide by 21
 *  0b010101..Divide by 22
 *  0b010110..Divide by 23
 *  0b010111..Divide by 24
 *  0b011000..Divide by 25
 *  0b011001..Divide by 26
 *  0b011010..Divide by 27
 *  0b011011..Divide by 28
 *  0b011100..Divide by 29
 *  0b011101..Divide by 30
 *  0b011110..Divide by 31
 *  0b011111..Divide by 32
 *  0b100000..Divide by 33
 *  0b100001..Divide by 34
 *  0b100010..Divide by 35
 *  0b100011..Divide by 36
 *  0b100100..Divide by 37
 *  0b100101..Divide by 38
 *  0b100110..Divide by 39
 *  0b100111..Divide by 40
 *  0b101000..Divide by 41
 *  0b101001..Divide by 42
 *  0b101010..Divide by 43
 *  0b101011..Divide by 44
 *  0b101100..Divide by 45
 *  0b101101..Divide by 46
 *  0b101110..Divide by 47
 *  0b101111..Divide by 48
 *  0b110000..Divide by 49
 *  0b110001..Divide by 50
 *  0b110010..Divide by 51
 *  0b110011..Divide by 52
 *  0b110100..Divide by 53
 *  0b110101..Divide by 54
 *  0b110110..Divide by 55
 *  0b110111..Divide by 56
 *  0b111000..Divide by 57
 *  0b111001..Divide by 58
 *  0b111010..Divide by 59
 *  0b111011..Divide by 60
 *  0b111100..Divide by 61
 *  0b111101..Divide by 62
 *  0b111110..Divide by 63
 *  0b111111..Divide by 64
 */
#define CCM_CS2CDR_SAI2_CLK_PODF(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CS2CDR_SAI2_CLK_PODF_SHIFT)) & CCM_CS2CDR_SAI2_CLK_PODF_MASK)
#define CCM_CS2CDR_SAI2_CLK_PRED_MASK            (0x1C0U)
#define CCM_CS2CDR_SAI2_CLK_PRED_SHIFT           (6U)
/*! SAI2_CLK_PRED
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CS2CDR_SAI2_CLK_PRED(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CS2CDR_SAI2_CLK_PRED_SHIFT)) & CCM_CS2CDR_SAI2_CLK_PRED_MASK)
/*! @} */

/*! @name CDCDR - CCM D1 Clock Divider Register */
/*! @{ */
#define CCM_CDCDR_FLEXIO1_CLK_SEL_MASK           (0x180U)
#define CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT          (7U)
/*! FLEXIO1_CLK_SEL
 *  0b00..derive clock from PLL4
 *  0b01..derive clock from PLL3 PFD2
 *  0b10..derive clock from PLL5
 *  0b11..derive clock from pll3_sw_clk
 */
#define CCM_CDCDR_FLEXIO1_CLK_SEL(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CDCDR_FLEXIO1_CLK_SEL_SHIFT)) & CCM_CDCDR_FLEXIO1_CLK_SEL_MASK)
#define CCM_CDCDR_FLEXIO1_CLK_PODF_MASK          (0xE00U)
#define CCM_CDCDR_FLEXIO1_CLK_PODF_SHIFT         (9U)
/*! FLEXIO1_CLK_PODF - Divider for flexio1 clock podf. Divider should be updated when output clock is gated.
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 3
 *  0b011..Divide by 4
 *  0b100..Divide by 5
 *  0b101..Divide by 6
 *  0b110..Divide by 7
 *  0b111..Divide by 8
 */
#define CCM_CDCDR_FLEXIO1_CLK_PODF(x)            (((uint32_t)(((uint32_t)(x)) << CCM_CDCDR_FLEXIO1_CLK_PODF_SHIFT)) & CCM_CDCDR_FLEXIO1_CLK_PODF_MASK)
#define CCM_CDCDR_FLEXIO1_CLK_PRED_MASK          (0x7000U)
#define CCM_CDCDR_FLEXIO1_CLK_PRED_SHIFT         (12U)
/*! FLEXIO1_CLK_PRED - Divider for flexio1 clock pred. Divider should be updated when output clock is gated.
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 3
 *  0b011..Divide by 4
 *  0b100..Divide by 5
 *  0b101..Divide by 6
 *  0b110..Divide by 7
 *  0b111..Divide by 8
 */
#define CCM_CDCDR_FLEXIO1_CLK_PRED(x)            (((uint32_t)(((uint32_t)(x)) << CCM_CDCDR_FLEXIO1_CLK_PRED_SHIFT)) & CCM_CDCDR_FLEXIO1_CLK_PRED_MASK)
#define CCM_CDCDR_SPDIF0_CLK_SEL_MASK            (0x300000U)
#define CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT           (20U)
/*! SPDIF0_CLK_SEL
 *  0b00..derive clock from PLL4
 *  0b01..derive clock from PLL3 PFD2
 *  0b10..derive clock from PLL5
 *  0b11..derive clock from pll3_sw_clk
 */
#define CCM_CDCDR_SPDIF0_CLK_SEL(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CDCDR_SPDIF0_CLK_SEL_SHIFT)) & CCM_CDCDR_SPDIF0_CLK_SEL_MASK)
#define CCM_CDCDR_SPDIF0_CLK_PODF_MASK           (0x1C00000U)
#define CCM_CDCDR_SPDIF0_CLK_PODF_SHIFT          (22U)
/*! SPDIF0_CLK_PODF - Divider for spdif0 clock podf. Divider should be updated when output clock is gated.
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 3
 *  0b011..Divide by 4
 *  0b100..Divide by 5
 *  0b101..Divide by 6
 *  0b110..Divide by 7
 *  0b111..Divide by 8
 */
#define CCM_CDCDR_SPDIF0_CLK_PODF(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CDCDR_SPDIF0_CLK_PODF_SHIFT)) & CCM_CDCDR_SPDIF0_CLK_PODF_MASK)
#define CCM_CDCDR_SPDIF0_CLK_PRED_MASK           (0xE000000U)
#define CCM_CDCDR_SPDIF0_CLK_PRED_SHIFT          (25U)
/*! SPDIF0_CLK_PRED - Divider for spdif0 clock pred. Divider should be updated when output clock is gated.
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 3
 *  0b011..Divide by 4
 *  0b100..Divide by 5
 *  0b101..Divide by 6
 *  0b110..Divide by 7
 *  0b111..Divide by 8
 */
#define CCM_CDCDR_SPDIF0_CLK_PRED(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CDCDR_SPDIF0_CLK_PRED_SHIFT)) & CCM_CDCDR_SPDIF0_CLK_PRED_MASK)
/*! @} */

/*! @name CSCDR2 - CCM Serial Clock Divider Register 2 */
/*! @{ */
#define CCM_CSCDR2_LCDIF_PRED_MASK               (0x7000U)
#define CCM_CSCDR2_LCDIF_PRED_SHIFT              (12U)
/*! LCDIF_PRED
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CSCDR2_LCDIF_PRED(x)                 (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR2_LCDIF_PRED_SHIFT)) & CCM_CSCDR2_LCDIF_PRED_MASK)
#define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_MASK        (0x38000U)
#define CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT       (15U)
/*! LCDIF_PRE_CLK_SEL
 *  0b000..derive clock from PLL2
 *  0b001..derive clock from PLL3 PFD3
 *  0b010..derive clock from PLL5
 *  0b011..derive clock from PLL2 PFD0
 *  0b100..derive clock from PLL2 PFD1
 *  0b101..derive clock from PLL3 PFD1
 */
#define CCM_CSCDR2_LCDIF_PRE_CLK_SEL(x)          (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR2_LCDIF_PRE_CLK_SEL_SHIFT)) & CCM_CSCDR2_LCDIF_PRE_CLK_SEL_MASK)
#define CCM_CSCDR2_LPI2C_CLK_SEL_MASK            (0x40000U)
#define CCM_CSCDR2_LPI2C_CLK_SEL_SHIFT           (18U)
/*! LPI2C_CLK_SEL
 *  0b0..derive clock from pll3_60m
 *  0b1..derive clock from osc_clk
 */
#define CCM_CSCDR2_LPI2C_CLK_SEL(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR2_LPI2C_CLK_SEL_SHIFT)) & CCM_CSCDR2_LPI2C_CLK_SEL_MASK)
#define CCM_CSCDR2_LPI2C_CLK_PODF_MASK           (0x1F80000U)
#define CCM_CSCDR2_LPI2C_CLK_PODF_SHIFT          (19U)
/*! LPI2C_CLK_PODF - Divider for lpi2c clock podf. Divider should be updated when output clock is
 *    gated. The input clock to this divider should be lower than 300Mhz, the predivider can be used
 *    to achieve this.
 *  0b000000..Divide by 1
 *  0b000001..Divide by 2
 *  0b000010..Divide by 3
 *  0b000011..Divide by 4
 *  0b000100..Divide by 5
 *  0b000101..Divide by 6
 *  0b000110..Divide by 7
 *  0b000111..Divide by 8
 *  0b001000..Divide by 9
 *  0b001001..Divide by 10
 *  0b001010..Divide by 11
 *  0b001011..Divide by 12
 *  0b001100..Divide by 13
 *  0b001101..Divide by 14
 *  0b001110..Divide by 15
 *  0b001111..Divide by 16
 *  0b010000..Divide by 17
 *  0b010001..Divide by 18
 *  0b010010..Divide by 19
 *  0b010011..Divide by 20
 *  0b010100..Divide by 21
 *  0b010101..Divide by 22
 *  0b010110..Divide by 23
 *  0b010111..Divide by 24
 *  0b011000..Divide by 25
 *  0b011001..Divide by 26
 *  0b011010..Divide by 27
 *  0b011011..Divide by 28
 *  0b011100..Divide by 29
 *  0b011101..Divide by 30
 *  0b011110..Divide by 31
 *  0b011111..Divide by 32
 *  0b100000..Divide by 33
 *  0b100001..Divide by 34
 *  0b100010..Divide by 35
 *  0b100011..Divide by 36
 *  0b100100..Divide by 37
 *  0b100101..Divide by 38
 *  0b100110..Divide by 39
 *  0b100111..Divide by 40
 *  0b101000..Divide by 41
 *  0b101001..Divide by 42
 *  0b101010..Divide by 43
 *  0b101011..Divide by 44
 *  0b101100..Divide by 45
 *  0b101101..Divide by 46
 *  0b101110..Divide by 47
 *  0b101111..Divide by 48
 *  0b110000..Divide by 49
 *  0b110001..Divide by 50
 *  0b110010..Divide by 51
 *  0b110011..Divide by 52
 *  0b110100..Divide by 53
 *  0b110101..Divide by 54
 *  0b110110..Divide by 55
 *  0b110111..Divide by 56
 *  0b111000..Divide by 57
 *  0b111001..Divide by 58
 *  0b111010..Divide by 59
 *  0b111011..Divide by 60
 *  0b111100..Divide by 61
 *  0b111101..Divide by 62
 *  0b111110..Divide by 63
 *  0b111111..Divide by 64
 */
#define CCM_CSCDR2_LPI2C_CLK_PODF(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR2_LPI2C_CLK_PODF_SHIFT)) & CCM_CSCDR2_LPI2C_CLK_PODF_MASK)
/*! @} */

/*! @name CSCDR3 - CCM Serial Clock Divider Register 3 */
/*! @{ */
#define CCM_CSCDR3_CSI_CLK_SEL_MASK              (0x600U)
#define CCM_CSCDR3_CSI_CLK_SEL_SHIFT             (9U)
/*! CSI_CLK_SEL
 *  0b00..derive clock from osc_clk (24M)
 *  0b01..derive clock from PLL2 PFD2
 *  0b10..derive clock from pll3_120M
 *  0b11..derive clock from PLL3 PFD1
 */
#define CCM_CSCDR3_CSI_CLK_SEL(x)                (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR3_CSI_CLK_SEL_SHIFT)) & CCM_CSCDR3_CSI_CLK_SEL_MASK)
#define CCM_CSCDR3_CSI_PODF_MASK                 (0x3800U)
#define CCM_CSCDR3_CSI_PODF_SHIFT                (11U)
/*! CSI_PODF
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CSCDR3_CSI_PODF(x)                   (((uint32_t)(((uint32_t)(x)) << CCM_CSCDR3_CSI_PODF_SHIFT)) & CCM_CSCDR3_CSI_PODF_MASK)
/*! @} */

/*! @name CDHIPR - CCM Divider Handshake In-Process Register */
/*! @{ */
#define CCM_CDHIPR_SEMC_PODF_BUSY_MASK           (0x1U)
#define CCM_CDHIPR_SEMC_PODF_BUSY_SHIFT          (0U)
/*! SEMC_PODF_BUSY
 *  0b0..divider is not busy and its value represents the actual division.
 *  0b1..divider is busy with handshake process with module. The value read in the divider represents the previous
 *       value of the division factor, and after the handshake the written value of the semc_podf will be applied.
 */
#define CCM_CDHIPR_SEMC_PODF_BUSY(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CDHIPR_SEMC_PODF_BUSY_SHIFT)) & CCM_CDHIPR_SEMC_PODF_BUSY_MASK)
#define CCM_CDHIPR_AHB_PODF_BUSY_MASK            (0x2U)
#define CCM_CDHIPR_AHB_PODF_BUSY_SHIFT           (1U)
/*! AHB_PODF_BUSY
 *  0b0..divider is not busy and its value represents the actual division.
 *  0b1..divider is busy with handshake process with module. The value read in the divider represents the previous
 *       value of the division factor, and after the handshake the written value of the ahb_podf will be applied.
 */
#define CCM_CDHIPR_AHB_PODF_BUSY(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CDHIPR_AHB_PODF_BUSY_SHIFT)) & CCM_CDHIPR_AHB_PODF_BUSY_MASK)
#define CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY_MASK     (0x8U)
#define CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY_SHIFT    (3U)
/*! PERIPH2_CLK_SEL_BUSY
 *  0b0..mux is not busy and its value represents the actual division.
 *  0b1..mux is busy with handshake process with module. The value read in the periph2_clk_sel represents the
 *       previous value of select, and after the handshake periph2_clk_sel value will be applied.
 */
#define CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY(x)       (((uint32_t)(((uint32_t)(x)) << CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY_SHIFT)) & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY_MASK)
#define CCM_CDHIPR_PERIPH_CLK_SEL_BUSY_MASK      (0x20U)
#define CCM_CDHIPR_PERIPH_CLK_SEL_BUSY_SHIFT     (5U)
/*! PERIPH_CLK_SEL_BUSY
 *  0b0..mux is not busy and its value represents the actual division.
 *  0b1..mux is busy with handshake process with module. The value read in the periph_clk_sel represents the
 *       previous value of select, and after the handshake periph_clk_sel value will be applied.
 */
#define CCM_CDHIPR_PERIPH_CLK_SEL_BUSY(x)        (((uint32_t)(((uint32_t)(x)) << CCM_CDHIPR_PERIPH_CLK_SEL_BUSY_SHIFT)) & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY_MASK)
#define CCM_CDHIPR_ARM_PODF_BUSY_MASK            (0x10000U)
#define CCM_CDHIPR_ARM_PODF_BUSY_SHIFT           (16U)
/*! ARM_PODF_BUSY
 *  0b0..divider is not busy and its value represents the actual division.
 *  0b1..divider is busy with handshake process with module. The value read in the divider represents the previous
 *       value of the division factor, and after the handshake the written value of the arm_podf will be applied.
 */
#define CCM_CDHIPR_ARM_PODF_BUSY(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CDHIPR_ARM_PODF_BUSY_SHIFT)) & CCM_CDHIPR_ARM_PODF_BUSY_MASK)
/*! @} */

/*! @name CLPCR - CCM Low Power Control Register */
/*! @{ */
#define CCM_CLPCR_LPM_MASK                       (0x3U)
#define CCM_CLPCR_LPM_SHIFT                      (0U)
/*! LPM
 *  0b00..Remain in run mode
 *  0b01..Transfer to wait mode
 *  0b10..Transfer to stop mode
 *  0b11..Reserved
 */
#define CCM_CLPCR_LPM(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_LPM_SHIFT)) & CCM_CLPCR_LPM_MASK)
#define CCM_CLPCR_ARM_CLK_DIS_ON_LPM_MASK        (0x20U)
#define CCM_CLPCR_ARM_CLK_DIS_ON_LPM_SHIFT       (5U)
/*! ARM_CLK_DIS_ON_LPM
 *  0b0..ARM clock enabled on wait mode.
 *  0b1..ARM clock disabled on wait mode. .
 */
#define CCM_CLPCR_ARM_CLK_DIS_ON_LPM(x)          (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_ARM_CLK_DIS_ON_LPM_SHIFT)) & CCM_CLPCR_ARM_CLK_DIS_ON_LPM_MASK)
#define CCM_CLPCR_SBYOS_MASK                     (0x40U)
#define CCM_CLPCR_SBYOS_SHIFT                    (6U)
/*! SBYOS
 *  0b0..On-chip oscillator will not be powered down, after next entrance to STOP mode. (CCM_REF_EN_B will remain
 *       asserted - '0' and cosc_pwrdown will remain de asserted - '0')
 *  0b1..On-chip oscillator will be powered down, after next entrance to STOP mode. (CCM_REF_EN_B will be
 *       deasserted - '1' and cosc_pwrdown will be asserted - '1'). When returning from STOP mode, external oscillator will
 *       be enabled again, on-chip oscillator will return to oscillator mode, and after oscnt count, CCM will
 *       continue with the exit from the STOP mode process.
 */
#define CCM_CLPCR_SBYOS(x)                       (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_SBYOS_SHIFT)) & CCM_CLPCR_SBYOS_MASK)
#define CCM_CLPCR_DIS_REF_OSC_MASK               (0x80U)
#define CCM_CLPCR_DIS_REF_OSC_SHIFT              (7U)
/*! DIS_REF_OSC
 *  0b0..external high frequency oscillator will be enabled, i.e. CCM_REF_EN_B = '0'.
 *  0b1..external high frequency oscillator will be disabled, i.e. CCM_REF_EN_B = '1'
 */
#define CCM_CLPCR_DIS_REF_OSC(x)                 (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_DIS_REF_OSC_SHIFT)) & CCM_CLPCR_DIS_REF_OSC_MASK)
#define CCM_CLPCR_VSTBY_MASK                     (0x100U)
#define CCM_CLPCR_VSTBY_SHIFT                    (8U)
/*! VSTBY
 *  0b0..Voltage will not be changed to standby voltage after next entrance to STOP mode. ( PMIC_STBY_REQ will remain negated - '0')
 *  0b1..Voltage will be requested to change to standby voltage after next entrance to stop mode. ( PMIC_STBY_REQ will be asserted - '1').
 */
#define CCM_CLPCR_VSTBY(x)                       (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_VSTBY_SHIFT)) & CCM_CLPCR_VSTBY_MASK)
#define CCM_CLPCR_STBY_COUNT_MASK                (0x600U)
#define CCM_CLPCR_STBY_COUNT_SHIFT               (9U)
/*! STBY_COUNT
 *  0b00..CCM will wait (1*pmic_delay_scaler)+1 ckil clock cycles
 *  0b01..CCM will wait (3*pmic_delay_scaler)+1 ckil clock cycles
 *  0b10..CCM will wait (7*pmic_delay_scaler)+1 ckil clock cycles
 *  0b11..CCM will wait (15*pmic_delay_scaler)+1 ckil clock cycles
 */
#define CCM_CLPCR_STBY_COUNT(x)                  (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_STBY_COUNT_SHIFT)) & CCM_CLPCR_STBY_COUNT_MASK)
#define CCM_CLPCR_COSC_PWRDOWN_MASK              (0x800U)
#define CCM_CLPCR_COSC_PWRDOWN_SHIFT             (11U)
/*! COSC_PWRDOWN
 *  0b0..On chip oscillator will not be powered down, i.e. cosc_pwrdown = '0'.
 *  0b1..On chip oscillator will be powered down, i.e. cosc_pwrdown = '1'.
 */
#define CCM_CLPCR_COSC_PWRDOWN(x)                (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_COSC_PWRDOWN_SHIFT)) & CCM_CLPCR_COSC_PWRDOWN_MASK)
#define CCM_CLPCR_BYPASS_LPM_HS1_MASK            (0x80000U)
#define CCM_CLPCR_BYPASS_LPM_HS1_SHIFT           (19U)
#define CCM_CLPCR_BYPASS_LPM_HS1(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_BYPASS_LPM_HS1_SHIFT)) & CCM_CLPCR_BYPASS_LPM_HS1_MASK)
#define CCM_CLPCR_BYPASS_LPM_HS0_MASK            (0x200000U)
#define CCM_CLPCR_BYPASS_LPM_HS0_SHIFT           (21U)
#define CCM_CLPCR_BYPASS_LPM_HS0(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_BYPASS_LPM_HS0_SHIFT)) & CCM_CLPCR_BYPASS_LPM_HS0_MASK)
#define CCM_CLPCR_MASK_CORE0_WFI_MASK            (0x400000U)
#define CCM_CLPCR_MASK_CORE0_WFI_SHIFT           (22U)
/*! MASK_CORE0_WFI
 *  0b0..WFI of core0 is not masked
 *  0b1..WFI of core0 is masked
 */
#define CCM_CLPCR_MASK_CORE0_WFI(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_MASK_CORE0_WFI_SHIFT)) & CCM_CLPCR_MASK_CORE0_WFI_MASK)
#define CCM_CLPCR_MASK_SCU_IDLE_MASK             (0x4000000U)
#define CCM_CLPCR_MASK_SCU_IDLE_SHIFT            (26U)
/*! MASK_SCU_IDLE
 *  0b1..SCU IDLE is masked
 *  0b0..SCU IDLE is not masked
 */
#define CCM_CLPCR_MASK_SCU_IDLE(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_MASK_SCU_IDLE_SHIFT)) & CCM_CLPCR_MASK_SCU_IDLE_MASK)
#define CCM_CLPCR_MASK_L2CC_IDLE_MASK            (0x8000000U)
#define CCM_CLPCR_MASK_L2CC_IDLE_SHIFT           (27U)
/*! MASK_L2CC_IDLE
 *  0b1..L2CC IDLE is masked
 *  0b0..L2CC IDLE is not masked
 */
#define CCM_CLPCR_MASK_L2CC_IDLE(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CLPCR_MASK_L2CC_IDLE_SHIFT)) & CCM_CLPCR_MASK_L2CC_IDLE_MASK)
/*! @} */

/*! @name CISR - CCM Interrupt Status Register */
/*! @{ */
#define CCM_CISR_LRF_PLL_MASK                    (0x1U)
#define CCM_CISR_LRF_PLL_SHIFT                   (0U)
/*! LRF_PLL
 *  0b0..interrupt is not generated due to lock ready of all enabled and not bypaseed PLLs
 *  0b1..interrupt generated due to lock ready of all enabled and not bypaseed PLLs
 */
#define CCM_CISR_LRF_PLL(x)                      (((uint32_t)(((uint32_t)(x)) << CCM_CISR_LRF_PLL_SHIFT)) & CCM_CISR_LRF_PLL_MASK)
#define CCM_CISR_COSC_READY_MASK                 (0x40U)
#define CCM_CISR_COSC_READY_SHIFT                (6U)
/*! COSC_READY
 *  0b0..interrupt is not generated due to on board oscillator ready
 *  0b1..interrupt generated due to on board oscillator ready
 */
#define CCM_CISR_COSC_READY(x)                   (((uint32_t)(((uint32_t)(x)) << CCM_CISR_COSC_READY_SHIFT)) & CCM_CISR_COSC_READY_MASK)
#define CCM_CISR_SEMC_PODF_LOADED_MASK           (0x20000U)
#define CCM_CISR_SEMC_PODF_LOADED_SHIFT          (17U)
/*! SEMC_PODF_LOADED
 *  0b0..interrupt is not generated due to frequency change of semc_podf
 *  0b1..interrupt generated due to frequency change of semc_podf
 */
#define CCM_CISR_SEMC_PODF_LOADED(x)             (((uint32_t)(((uint32_t)(x)) << CCM_CISR_SEMC_PODF_LOADED_SHIFT)) & CCM_CISR_SEMC_PODF_LOADED_MASK)
#define CCM_CISR_PERIPH2_CLK_SEL_LOADED_MASK     (0x80000U)
#define CCM_CISR_PERIPH2_CLK_SEL_LOADED_SHIFT    (19U)
/*! PERIPH2_CLK_SEL_LOADED
 *  0b0..interrupt is not generated due to frequency change of periph2_clk_sel
 *  0b1..interrupt generated due to frequency change of periph2_clk_sel
 */
#define CCM_CISR_PERIPH2_CLK_SEL_LOADED(x)       (((uint32_t)(((uint32_t)(x)) << CCM_CISR_PERIPH2_CLK_SEL_LOADED_SHIFT)) & CCM_CISR_PERIPH2_CLK_SEL_LOADED_MASK)
#define CCM_CISR_AHB_PODF_LOADED_MASK            (0x100000U)
#define CCM_CISR_AHB_PODF_LOADED_SHIFT           (20U)
/*! AHB_PODF_LOADED
 *  0b0..interrupt is not generated due to frequency change of ahb_podf
 *  0b1..interrupt generated due to frequency change of ahb_podf
 */
#define CCM_CISR_AHB_PODF_LOADED(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CISR_AHB_PODF_LOADED_SHIFT)) & CCM_CISR_AHB_PODF_LOADED_MASK)
#define CCM_CISR_PERIPH_CLK_SEL_LOADED_MASK      (0x400000U)
#define CCM_CISR_PERIPH_CLK_SEL_LOADED_SHIFT     (22U)
/*! PERIPH_CLK_SEL_LOADED
 *  0b0..interrupt is not generated due to update of periph_clk_sel.
 *  0b1..interrupt generated due to update of periph_clk_sel.
 */
#define CCM_CISR_PERIPH_CLK_SEL_LOADED(x)        (((uint32_t)(((uint32_t)(x)) << CCM_CISR_PERIPH_CLK_SEL_LOADED_SHIFT)) & CCM_CISR_PERIPH_CLK_SEL_LOADED_MASK)
#define CCM_CISR_ARM_PODF_LOADED_MASK            (0x4000000U)
#define CCM_CISR_ARM_PODF_LOADED_SHIFT           (26U)
/*! ARM_PODF_LOADED
 *  0b0..interrupt is not generated due to frequency change of arm_podf
 *  0b1..interrupt generated due to frequency change of arm_podf
 */
#define CCM_CISR_ARM_PODF_LOADED(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CISR_ARM_PODF_LOADED_SHIFT)) & CCM_CISR_ARM_PODF_LOADED_MASK)
/*! @} */

/*! @name CIMR - CCM Interrupt Mask Register */
/*! @{ */
#define CCM_CIMR_MASK_LRF_PLL_MASK               (0x1U)
#define CCM_CIMR_MASK_LRF_PLL_SHIFT              (0U)
/*! MASK_LRF_PLL
 *  0b0..don't mask interrupt due to lrf of PLLs - interrupt will be created
 *  0b1..mask interrupt due to lrf of PLLs
 */
#define CCM_CIMR_MASK_LRF_PLL(x)                 (((uint32_t)(((uint32_t)(x)) << CCM_CIMR_MASK_LRF_PLL_SHIFT)) & CCM_CIMR_MASK_LRF_PLL_MASK)
#define CCM_CIMR_MASK_COSC_READY_MASK            (0x40U)
#define CCM_CIMR_MASK_COSC_READY_SHIFT           (6U)
/*! MASK_COSC_READY
 *  0b0..don't mask interrupt due to on board oscillator ready - interrupt will be created
 *  0b1..mask interrupt due to on board oscillator ready
 */
#define CCM_CIMR_MASK_COSC_READY(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CIMR_MASK_COSC_READY_SHIFT)) & CCM_CIMR_MASK_COSC_READY_MASK)
#define CCM_CIMR_MASK_SEMC_PODF_LOADED_MASK      (0x20000U)
#define CCM_CIMR_MASK_SEMC_PODF_LOADED_SHIFT     (17U)
/*! MASK_SEMC_PODF_LOADED
 *  0b0..don't mask interrupt due to frequency change of semc_podf - interrupt will be created
 *  0b1..mask interrupt due to frequency change of semc_podf
 */
#define CCM_CIMR_MASK_SEMC_PODF_LOADED(x)        (((uint32_t)(((uint32_t)(x)) << CCM_CIMR_MASK_SEMC_PODF_LOADED_SHIFT)) & CCM_CIMR_MASK_SEMC_PODF_LOADED_MASK)
#define CCM_CIMR_MASK_PERIPH2_CLK_SEL_LOADED_MASK (0x80000U)
#define CCM_CIMR_MASK_PERIPH2_CLK_SEL_LOADED_SHIFT (19U)
/*! MASK_PERIPH2_CLK_SEL_LOADED
 *  0b0..don't mask interrupt due to update of periph2_clk_sel - interrupt will be created
 *  0b1..mask interrupt due to update of periph2_clk_sel
 */
#define CCM_CIMR_MASK_PERIPH2_CLK_SEL_LOADED(x)  (((uint32_t)(((uint32_t)(x)) << CCM_CIMR_MASK_PERIPH2_CLK_SEL_LOADED_SHIFT)) & CCM_CIMR_MASK_PERIPH2_CLK_SEL_LOADED_MASK)
#define CCM_CIMR_MASK_AHB_PODF_LOADED_MASK       (0x100000U)
#define CCM_CIMR_MASK_AHB_PODF_LOADED_SHIFT      (20U)
/*! MASK_AHB_PODF_LOADED
 *  0b0..don't mask interrupt due to frequency change of ahb_podf - interrupt will be created
 *  0b1..mask interrupt due to frequency change of ahb_podf
 */
#define CCM_CIMR_MASK_AHB_PODF_LOADED(x)         (((uint32_t)(((uint32_t)(x)) << CCM_CIMR_MASK_AHB_PODF_LOADED_SHIFT)) & CCM_CIMR_MASK_AHB_PODF_LOADED_MASK)
#define CCM_CIMR_MASK_PERIPH_CLK_SEL_LOADED_MASK (0x400000U)
#define CCM_CIMR_MASK_PERIPH_CLK_SEL_LOADED_SHIFT (22U)
/*! MASK_PERIPH_CLK_SEL_LOADED
 *  0b0..don't mask interrupt due to update of periph_clk_sel - interrupt will be created
 *  0b1..mask interrupt due to update of periph_clk_sel
 */
#define CCM_CIMR_MASK_PERIPH_CLK_SEL_LOADED(x)   (((uint32_t)(((uint32_t)(x)) << CCM_CIMR_MASK_PERIPH_CLK_SEL_LOADED_SHIFT)) & CCM_CIMR_MASK_PERIPH_CLK_SEL_LOADED_MASK)
#define CCM_CIMR_ARM_PODF_LOADED_MASK            (0x4000000U)
#define CCM_CIMR_ARM_PODF_LOADED_SHIFT           (26U)
/*! ARM_PODF_LOADED
 *  0b0..don't mask interrupt due to frequency change of arm_podf - interrupt will be created
 *  0b1..mask interrupt due to frequency change of arm_podf
 */
#define CCM_CIMR_ARM_PODF_LOADED(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CIMR_ARM_PODF_LOADED_SHIFT)) & CCM_CIMR_ARM_PODF_LOADED_MASK)
/*! @} */

/*! @name CCOSR - CCM Clock Output Source Register */
/*! @{ */
#define CCM_CCOSR_CLKO1_SEL_MASK                 (0xFU)
#define CCM_CCOSR_CLKO1_SEL_SHIFT                (0U)
/*! CLKO1_SEL
 *  0b0000..USB1 PLL clock (divided by 2)
 *  0b0001..SYS PLL clock (divided by 2)
 *  0b0011..VIDEO PLL clock (divided by 2)
 *  0b0101..semc_clk_root
 *  0b0110..Reserved
 *  0b1010..lcdif_pix_clk_root
 *  0b1011..ahb_clk_root
 *  0b1100..ipg_clk_root
 *  0b1101..perclk_root
 *  0b1110..ckil_sync_clk_root
 *  0b1111..pll4_main_clk
 */
#define CCM_CCOSR_CLKO1_SEL(x)                   (((uint32_t)(((uint32_t)(x)) << CCM_CCOSR_CLKO1_SEL_SHIFT)) & CCM_CCOSR_CLKO1_SEL_MASK)
#define CCM_CCOSR_CLKO1_DIV_MASK                 (0x70U)
#define CCM_CCOSR_CLKO1_DIV_SHIFT                (4U)
/*! CLKO1_DIV
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CCOSR_CLKO1_DIV(x)                   (((uint32_t)(((uint32_t)(x)) << CCM_CCOSR_CLKO1_DIV_SHIFT)) & CCM_CCOSR_CLKO1_DIV_MASK)
#define CCM_CCOSR_CLKO1_EN_MASK                  (0x80U)
#define CCM_CCOSR_CLKO1_EN_SHIFT                 (7U)
/*! CLKO1_EN
 *  0b0..CCM_CLKO1 disabled.
 *  0b1..CCM_CLKO1 enabled.
 */
#define CCM_CCOSR_CLKO1_EN(x)                    (((uint32_t)(((uint32_t)(x)) << CCM_CCOSR_CLKO1_EN_SHIFT)) & CCM_CCOSR_CLKO1_EN_MASK)
#define CCM_CCOSR_CLK_OUT_SEL_MASK               (0x100U)
#define CCM_CCOSR_CLK_OUT_SEL_SHIFT              (8U)
/*! CLK_OUT_SEL
 *  0b0..CCM_CLKO1 output drives CCM_CLKO1 clock
 *  0b1..CCM_CLKO1 output drives CCM_CLKO2 clock
 */
#define CCM_CCOSR_CLK_OUT_SEL(x)                 (((uint32_t)(((uint32_t)(x)) << CCM_CCOSR_CLK_OUT_SEL_SHIFT)) & CCM_CCOSR_CLK_OUT_SEL_MASK)
#define CCM_CCOSR_CLKO2_SEL_MASK                 (0x1F0000U)
#define CCM_CCOSR_CLKO2_SEL_SHIFT                (16U)
/*! CLKO2_SEL
 *  0b00011..usdhc1_clk_root
 *  0b00110..lpi2c_clk_root
 *  0b01011..csi_clk_root
 *  0b01110..osc_clk
 *  0b10001..usdhc2_clk_root
 *  0b10010..sai1_clk_root
 *  0b10011..sai2_clk_root
 *  0b10100..sai3_clk_root (shared with ADC1 and ADC2 alt_clk root)
 *  0b10111..can_clk_root (FlexCAN, shared with CANFD)
 *  0b11011..flexspi_clk_root
 *  0b11100..uart_clk_root
 *  0b11101..spdif0_clk_root
 *  0b11111..Reserved
 */
#define CCM_CCOSR_CLKO2_SEL(x)                   (((uint32_t)(((uint32_t)(x)) << CCM_CCOSR_CLKO2_SEL_SHIFT)) & CCM_CCOSR_CLKO2_SEL_MASK)
#define CCM_CCOSR_CLKO2_DIV_MASK                 (0xE00000U)
#define CCM_CCOSR_CLKO2_DIV_SHIFT                (21U)
/*! CLKO2_DIV
 *  0b000..divide by 1
 *  0b001..divide by 2
 *  0b010..divide by 3
 *  0b011..divide by 4
 *  0b100..divide by 5
 *  0b101..divide by 6
 *  0b110..divide by 7
 *  0b111..divide by 8
 */
#define CCM_CCOSR_CLKO2_DIV(x)                   (((uint32_t)(((uint32_t)(x)) << CCM_CCOSR_CLKO2_DIV_SHIFT)) & CCM_CCOSR_CLKO2_DIV_MASK)
#define CCM_CCOSR_CLKO2_EN_MASK                  (0x1000000U)
#define CCM_CCOSR_CLKO2_EN_SHIFT                 (24U)
/*! CLKO2_EN
 *  0b0..CCM_CLKO2 disabled.
 *  0b1..CCM_CLKO2 enabled.
 */
#define CCM_CCOSR_CLKO2_EN(x)                    (((uint32_t)(((uint32_t)(x)) << CCM_CCOSR_CLKO2_EN_SHIFT)) & CCM_CCOSR_CLKO2_EN_MASK)
/*! @} */

/*! @name CGPR - CCM General Purpose Register */
/*! @{ */
#define CCM_CGPR_PMIC_DELAY_SCALER_MASK          (0x1U)
#define CCM_CGPR_PMIC_DELAY_SCALER_SHIFT         (0U)
/*! PMIC_DELAY_SCALER
 *  0b0..clock is not divided
 *  0b1..clock is divided /8
 */
#define CCM_CGPR_PMIC_DELAY_SCALER(x)            (((uint32_t)(((uint32_t)(x)) << CCM_CGPR_PMIC_DELAY_SCALER_SHIFT)) & CCM_CGPR_PMIC_DELAY_SCALER_MASK)
#define CCM_CGPR_EFUSE_PROG_SUPPLY_GATE_MASK     (0x10U)
#define CCM_CGPR_EFUSE_PROG_SUPPLY_GATE_SHIFT    (4U)
/*! EFUSE_PROG_SUPPLY_GATE
 *  0b0..fuse programing supply voltage is gated off to the efuse module
 *  0b1..allow fuse programing.
 */
#define CCM_CGPR_EFUSE_PROG_SUPPLY_GATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_CGPR_EFUSE_PROG_SUPPLY_GATE_SHIFT)) & CCM_CGPR_EFUSE_PROG_SUPPLY_GATE_MASK)
#define CCM_CGPR_SYS_MEM_DS_CTRL_MASK            (0xC000U)
#define CCM_CGPR_SYS_MEM_DS_CTRL_SHIFT           (14U)
/*! SYS_MEM_DS_CTRL
 *  0b00..Disable memory DS mode always
 *  0b01..Enable memory (outside ARM platform) DS mode when system STOP and PLL are disabled
 *  0b1x..enable memory (outside ARM platform) DS mode when system is in STOP mode
 */
#define CCM_CGPR_SYS_MEM_DS_CTRL(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CGPR_SYS_MEM_DS_CTRL_SHIFT)) & CCM_CGPR_SYS_MEM_DS_CTRL_MASK)
#define CCM_CGPR_FPL_MASK                        (0x10000U)
#define CCM_CGPR_FPL_SHIFT                       (16U)
/*! FPL - Fast PLL enable.
 *  0b0..Engage PLL enable default way.
 *  0b1..Engage PLL enable 3 CKIL clocks earlier at exiting low power mode (STOP). Should be used only if 24MHz OSC was active in low power mode.
 */
#define CCM_CGPR_FPL(x)                          (((uint32_t)(((uint32_t)(x)) << CCM_CGPR_FPL_SHIFT)) & CCM_CGPR_FPL_MASK)
#define CCM_CGPR_INT_MEM_CLK_LPM_MASK            (0x20000U)
#define CCM_CGPR_INT_MEM_CLK_LPM_SHIFT           (17U)
/*! INT_MEM_CLK_LPM
 *  0b0..Disable the clock to the ARM platform memories when entering Low Power Mode
 *  0b1..Keep the clocks to the ARM platform memories enabled only if an interrupt is pending when entering Low
 *       Power Modes (WAIT and STOP without power gating)
 */
#define CCM_CGPR_INT_MEM_CLK_LPM(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CGPR_INT_MEM_CLK_LPM_SHIFT)) & CCM_CGPR_INT_MEM_CLK_LPM_MASK)
/*! @} */

/*! @name CCGR0 - CCM Clock Gating Register 0 */
/*! @{ */
#define CCM_CCGR0_CG0_MASK                       (0x3U)
#define CCM_CCGR0_CG0_SHIFT                      (0U)
#define CCM_CCGR0_CG0(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG0_SHIFT)) & CCM_CCGR0_CG0_MASK)
#define CCM_CCGR0_CG1_MASK                       (0xCU)
#define CCM_CCGR0_CG1_SHIFT                      (2U)
#define CCM_CCGR0_CG1(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG1_SHIFT)) & CCM_CCGR0_CG1_MASK)
#define CCM_CCGR0_CG2_MASK                       (0x30U)
#define CCM_CCGR0_CG2_SHIFT                      (4U)
#define CCM_CCGR0_CG2(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG2_SHIFT)) & CCM_CCGR0_CG2_MASK)
#define CCM_CCGR0_CG3_MASK                       (0xC0U)
#define CCM_CCGR0_CG3_SHIFT                      (6U)
#define CCM_CCGR0_CG3(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG3_SHIFT)) & CCM_CCGR0_CG3_MASK)
#define CCM_CCGR0_CG4_MASK                       (0x300U)
#define CCM_CCGR0_CG4_SHIFT                      (8U)
#define CCM_CCGR0_CG4(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG4_SHIFT)) & CCM_CCGR0_CG4_MASK)
#define CCM_CCGR0_CG5_MASK                       (0xC00U)
#define CCM_CCGR0_CG5_SHIFT                      (10U)
#define CCM_CCGR0_CG5(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG5_SHIFT)) & CCM_CCGR0_CG5_MASK)
#define CCM_CCGR0_CG6_MASK                       (0x3000U)
#define CCM_CCGR0_CG6_SHIFT                      (12U)
#define CCM_CCGR0_CG6(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG6_SHIFT)) & CCM_CCGR0_CG6_MASK)
#define CCM_CCGR0_CG7_MASK                       (0xC000U)
#define CCM_CCGR0_CG7_SHIFT                      (14U)
#define CCM_CCGR0_CG7(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG7_SHIFT)) & CCM_CCGR0_CG7_MASK)
#define CCM_CCGR0_CG8_MASK                       (0x30000U)
#define CCM_CCGR0_CG8_SHIFT                      (16U)
#define CCM_CCGR0_CG8(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG8_SHIFT)) & CCM_CCGR0_CG8_MASK)
#define CCM_CCGR0_CG9_MASK                       (0xC0000U)
#define CCM_CCGR0_CG9_SHIFT                      (18U)
#define CCM_CCGR0_CG9(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG9_SHIFT)) & CCM_CCGR0_CG9_MASK)
#define CCM_CCGR0_CG10_MASK                      (0x300000U)
#define CCM_CCGR0_CG10_SHIFT                     (20U)
#define CCM_CCGR0_CG10(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG10_SHIFT)) & CCM_CCGR0_CG10_MASK)
#define CCM_CCGR0_CG11_MASK                      (0xC00000U)
#define CCM_CCGR0_CG11_SHIFT                     (22U)
#define CCM_CCGR0_CG11(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG11_SHIFT)) & CCM_CCGR0_CG11_MASK)
#define CCM_CCGR0_CG12_MASK                      (0x3000000U)
#define CCM_CCGR0_CG12_SHIFT                     (24U)
#define CCM_CCGR0_CG12(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG12_SHIFT)) & CCM_CCGR0_CG12_MASK)
#define CCM_CCGR0_CG13_MASK                      (0xC000000U)
#define CCM_CCGR0_CG13_SHIFT                     (26U)
#define CCM_CCGR0_CG13(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG13_SHIFT)) & CCM_CCGR0_CG13_MASK)
#define CCM_CCGR0_CG14_MASK                      (0x30000000U)
#define CCM_CCGR0_CG14_SHIFT                     (28U)
#define CCM_CCGR0_CG14(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG14_SHIFT)) & CCM_CCGR0_CG14_MASK)
#define CCM_CCGR0_CG15_MASK                      (0xC0000000U)
#define CCM_CCGR0_CG15_SHIFT                     (30U)
#define CCM_CCGR0_CG15(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR0_CG15_SHIFT)) & CCM_CCGR0_CG15_MASK)
/*! @} */

/*! @name CCGR1 - CCM Clock Gating Register 1 */
/*! @{ */
#define CCM_CCGR1_CG0_MASK                       (0x3U)
#define CCM_CCGR1_CG0_SHIFT                      (0U)
#define CCM_CCGR1_CG0(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG0_SHIFT)) & CCM_CCGR1_CG0_MASK)
#define CCM_CCGR1_CG1_MASK                       (0xCU)
#define CCM_CCGR1_CG1_SHIFT                      (2U)
#define CCM_CCGR1_CG1(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG1_SHIFT)) & CCM_CCGR1_CG1_MASK)
#define CCM_CCGR1_CG2_MASK                       (0x30U)
#define CCM_CCGR1_CG2_SHIFT                      (4U)
#define CCM_CCGR1_CG2(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG2_SHIFT)) & CCM_CCGR1_CG2_MASK)
#define CCM_CCGR1_CG3_MASK                       (0xC0U)
#define CCM_CCGR1_CG3_SHIFT                      (6U)
#define CCM_CCGR1_CG3(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG3_SHIFT)) & CCM_CCGR1_CG3_MASK)
#define CCM_CCGR1_CG4_MASK                       (0x300U)
#define CCM_CCGR1_CG4_SHIFT                      (8U)
#define CCM_CCGR1_CG4(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG4_SHIFT)) & CCM_CCGR1_CG4_MASK)
#define CCM_CCGR1_CG5_MASK                       (0xC00U)
#define CCM_CCGR1_CG5_SHIFT                      (10U)
#define CCM_CCGR1_CG5(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG5_SHIFT)) & CCM_CCGR1_CG5_MASK)
#define CCM_CCGR1_CG6_MASK                       (0x3000U)
#define CCM_CCGR1_CG6_SHIFT                      (12U)
#define CCM_CCGR1_CG6(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG6_SHIFT)) & CCM_CCGR1_CG6_MASK)
#define CCM_CCGR1_CG7_MASK                       (0xC000U)
#define CCM_CCGR1_CG7_SHIFT                      (14U)
#define CCM_CCGR1_CG7(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG7_SHIFT)) & CCM_CCGR1_CG7_MASK)
#define CCM_CCGR1_CG8_MASK                       (0x30000U)
#define CCM_CCGR1_CG8_SHIFT                      (16U)
#define CCM_CCGR1_CG8(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG8_SHIFT)) & CCM_CCGR1_CG8_MASK)
#define CCM_CCGR1_CG9_MASK                       (0xC0000U)
#define CCM_CCGR1_CG9_SHIFT                      (18U)
#define CCM_CCGR1_CG9(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG9_SHIFT)) & CCM_CCGR1_CG9_MASK)
#define CCM_CCGR1_CG10_MASK                      (0x300000U)
#define CCM_CCGR1_CG10_SHIFT                     (20U)
#define CCM_CCGR1_CG10(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG10_SHIFT)) & CCM_CCGR1_CG10_MASK)
#define CCM_CCGR1_CG11_MASK                      (0xC00000U)
#define CCM_CCGR1_CG11_SHIFT                     (22U)
#define CCM_CCGR1_CG11(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG11_SHIFT)) & CCM_CCGR1_CG11_MASK)
#define CCM_CCGR1_CG12_MASK                      (0x3000000U)
#define CCM_CCGR1_CG12_SHIFT                     (24U)
#define CCM_CCGR1_CG12(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG12_SHIFT)) & CCM_CCGR1_CG12_MASK)
#define CCM_CCGR1_CG13_MASK                      (0xC000000U)
#define CCM_CCGR1_CG13_SHIFT                     (26U)
#define CCM_CCGR1_CG13(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG13_SHIFT)) & CCM_CCGR1_CG13_MASK)
#define CCM_CCGR1_CG14_MASK                      (0x30000000U)
#define CCM_CCGR1_CG14_SHIFT                     (28U)
#define CCM_CCGR1_CG14(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG14_SHIFT)) & CCM_CCGR1_CG14_MASK)
#define CCM_CCGR1_CG15_MASK                      (0xC0000000U)
#define CCM_CCGR1_CG15_SHIFT                     (30U)
#define CCM_CCGR1_CG15(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR1_CG15_SHIFT)) & CCM_CCGR1_CG15_MASK)
/*! @} */

/*! @name CCGR2 - CCM Clock Gating Register 2 */
/*! @{ */
#define CCM_CCGR2_CG0_MASK                       (0x3U)
#define CCM_CCGR2_CG0_SHIFT                      (0U)
#define CCM_CCGR2_CG0(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG0_SHIFT)) & CCM_CCGR2_CG0_MASK)
#define CCM_CCGR2_CG1_MASK                       (0xCU)
#define CCM_CCGR2_CG1_SHIFT                      (2U)
#define CCM_CCGR2_CG1(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG1_SHIFT)) & CCM_CCGR2_CG1_MASK)
#define CCM_CCGR2_CG2_MASK                       (0x30U)
#define CCM_CCGR2_CG2_SHIFT                      (4U)
#define CCM_CCGR2_CG2(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG2_SHIFT)) & CCM_CCGR2_CG2_MASK)
#define CCM_CCGR2_CG3_MASK                       (0xC0U)
#define CCM_CCGR2_CG3_SHIFT                      (6U)
#define CCM_CCGR2_CG3(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG3_SHIFT)) & CCM_CCGR2_CG3_MASK)
#define CCM_CCGR2_CG4_MASK                       (0x300U)
#define CCM_CCGR2_CG4_SHIFT                      (8U)
#define CCM_CCGR2_CG4(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG4_SHIFT)) & CCM_CCGR2_CG4_MASK)
#define CCM_CCGR2_CG5_MASK                       (0xC00U)
#define CCM_CCGR2_CG5_SHIFT                      (10U)
#define CCM_CCGR2_CG5(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG5_SHIFT)) & CCM_CCGR2_CG5_MASK)
#define CCM_CCGR2_CG6_MASK                       (0x3000U)
#define CCM_CCGR2_CG6_SHIFT                      (12U)
#define CCM_CCGR2_CG6(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG6_SHIFT)) & CCM_CCGR2_CG6_MASK)
#define CCM_CCGR2_CG7_MASK                       (0xC000U)
#define CCM_CCGR2_CG7_SHIFT                      (14U)
#define CCM_CCGR2_CG7(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG7_SHIFT)) & CCM_CCGR2_CG7_MASK)
#define CCM_CCGR2_CG8_MASK                       (0x30000U)
#define CCM_CCGR2_CG8_SHIFT                      (16U)
#define CCM_CCGR2_CG8(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG8_SHIFT)) & CCM_CCGR2_CG8_MASK)
#define CCM_CCGR2_CG9_MASK                       (0xC0000U)
#define CCM_CCGR2_CG9_SHIFT                      (18U)
#define CCM_CCGR2_CG9(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG9_SHIFT)) & CCM_CCGR2_CG9_MASK)
#define CCM_CCGR2_CG10_MASK                      (0x300000U)
#define CCM_CCGR2_CG10_SHIFT                     (20U)
#define CCM_CCGR2_CG10(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG10_SHIFT)) & CCM_CCGR2_CG10_MASK)
#define CCM_CCGR2_CG11_MASK                      (0xC00000U)
#define CCM_CCGR2_CG11_SHIFT                     (22U)
#define CCM_CCGR2_CG11(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG11_SHIFT)) & CCM_CCGR2_CG11_MASK)
#define CCM_CCGR2_CG12_MASK                      (0x3000000U)
#define CCM_CCGR2_CG12_SHIFT                     (24U)
#define CCM_CCGR2_CG12(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG12_SHIFT)) & CCM_CCGR2_CG12_MASK)
#define CCM_CCGR2_CG13_MASK                      (0xC000000U)
#define CCM_CCGR2_CG13_SHIFT                     (26U)
#define CCM_CCGR2_CG13(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG13_SHIFT)) & CCM_CCGR2_CG13_MASK)
#define CCM_CCGR2_CG14_MASK                      (0x30000000U)
#define CCM_CCGR2_CG14_SHIFT                     (28U)
#define CCM_CCGR2_CG14(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG14_SHIFT)) & CCM_CCGR2_CG14_MASK)
#define CCM_CCGR2_CG15_MASK                      (0xC0000000U)
#define CCM_CCGR2_CG15_SHIFT                     (30U)
#define CCM_CCGR2_CG15(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR2_CG15_SHIFT)) & CCM_CCGR2_CG15_MASK)
/*! @} */

/*! @name CCGR3 - CCM Clock Gating Register 3 */
/*! @{ */
#define CCM_CCGR3_CG0_MASK                       (0x3U)
#define CCM_CCGR3_CG0_SHIFT                      (0U)
#define CCM_CCGR3_CG0(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG0_SHIFT)) & CCM_CCGR3_CG0_MASK)
#define CCM_CCGR3_CG1_MASK                       (0xCU)
#define CCM_CCGR3_CG1_SHIFT                      (2U)
#define CCM_CCGR3_CG1(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG1_SHIFT)) & CCM_CCGR3_CG1_MASK)
#define CCM_CCGR3_CG2_MASK                       (0x30U)
#define CCM_CCGR3_CG2_SHIFT                      (4U)
#define CCM_CCGR3_CG2(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG2_SHIFT)) & CCM_CCGR3_CG2_MASK)
#define CCM_CCGR3_CG3_MASK                       (0xC0U)
#define CCM_CCGR3_CG3_SHIFT                      (6U)
#define CCM_CCGR3_CG3(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG3_SHIFT)) & CCM_CCGR3_CG3_MASK)
#define CCM_CCGR3_CG4_MASK                       (0x300U)
#define CCM_CCGR3_CG4_SHIFT                      (8U)
#define CCM_CCGR3_CG4(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG4_SHIFT)) & CCM_CCGR3_CG4_MASK)
#define CCM_CCGR3_CG5_MASK                       (0xC00U)
#define CCM_CCGR3_CG5_SHIFT                      (10U)
#define CCM_CCGR3_CG5(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG5_SHIFT)) & CCM_CCGR3_CG5_MASK)
#define CCM_CCGR3_CG6_MASK                       (0x3000U)
#define CCM_CCGR3_CG6_SHIFT                      (12U)
#define CCM_CCGR3_CG6(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG6_SHIFT)) & CCM_CCGR3_CG6_MASK)
#define CCM_CCGR3_CG7_MASK                       (0xC000U)
#define CCM_CCGR3_CG7_SHIFT                      (14U)
#define CCM_CCGR3_CG7(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG7_SHIFT)) & CCM_CCGR3_CG7_MASK)
#define CCM_CCGR3_CG8_MASK                       (0x30000U)
#define CCM_CCGR3_CG8_SHIFT                      (16U)
#define CCM_CCGR3_CG8(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG8_SHIFT)) & CCM_CCGR3_CG8_MASK)
#define CCM_CCGR3_CG9_MASK                       (0xC0000U)
#define CCM_CCGR3_CG9_SHIFT                      (18U)
#define CCM_CCGR3_CG9(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG9_SHIFT)) & CCM_CCGR3_CG9_MASK)
#define CCM_CCGR3_CG10_MASK                      (0x300000U)
#define CCM_CCGR3_CG10_SHIFT                     (20U)
#define CCM_CCGR3_CG10(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG10_SHIFT)) & CCM_CCGR3_CG10_MASK)
#define CCM_CCGR3_CG11_MASK                      (0xC00000U)
#define CCM_CCGR3_CG11_SHIFT                     (22U)
#define CCM_CCGR3_CG11(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG11_SHIFT)) & CCM_CCGR3_CG11_MASK)
#define CCM_CCGR3_CG12_MASK                      (0x3000000U)
#define CCM_CCGR3_CG12_SHIFT                     (24U)
#define CCM_CCGR3_CG12(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG12_SHIFT)) & CCM_CCGR3_CG12_MASK)
#define CCM_CCGR3_CG13_MASK                      (0xC000000U)
#define CCM_CCGR3_CG13_SHIFT                     (26U)
#define CCM_CCGR3_CG13(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG13_SHIFT)) & CCM_CCGR3_CG13_MASK)
#define CCM_CCGR3_CG14_MASK                      (0x30000000U)
#define CCM_CCGR3_CG14_SHIFT                     (28U)
/*! CG14 - The OCRAM clock cannot be turned off when the CM cache is running on this device.
 */
#define CCM_CCGR3_CG14(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG14_SHIFT)) & CCM_CCGR3_CG14_MASK)
#define CCM_CCGR3_CG15_MASK                      (0xC0000000U)
#define CCM_CCGR3_CG15_SHIFT                     (30U)
#define CCM_CCGR3_CG15(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR3_CG15_SHIFT)) & CCM_CCGR3_CG15_MASK)
/*! @} */

/*! @name CCGR4 - CCM Clock Gating Register 4 */
/*! @{ */
#define CCM_CCGR4_CG0_MASK                       (0x3U)
#define CCM_CCGR4_CG0_SHIFT                      (0U)
#define CCM_CCGR4_CG0(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG0_SHIFT)) & CCM_CCGR4_CG0_MASK)
#define CCM_CCGR4_CG1_MASK                       (0xCU)
#define CCM_CCGR4_CG1_SHIFT                      (2U)
#define CCM_CCGR4_CG1(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG1_SHIFT)) & CCM_CCGR4_CG1_MASK)
#define CCM_CCGR4_CG2_MASK                       (0x30U)
#define CCM_CCGR4_CG2_SHIFT                      (4U)
#define CCM_CCGR4_CG2(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG2_SHIFT)) & CCM_CCGR4_CG2_MASK)
#define CCM_CCGR4_CG3_MASK                       (0xC0U)
#define CCM_CCGR4_CG3_SHIFT                      (6U)
#define CCM_CCGR4_CG3(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG3_SHIFT)) & CCM_CCGR4_CG3_MASK)
#define CCM_CCGR4_CG4_MASK                       (0x300U)
#define CCM_CCGR4_CG4_SHIFT                      (8U)
#define CCM_CCGR4_CG4(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG4_SHIFT)) & CCM_CCGR4_CG4_MASK)
#define CCM_CCGR4_CG5_MASK                       (0xC00U)
#define CCM_CCGR4_CG5_SHIFT                      (10U)
#define CCM_CCGR4_CG5(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG5_SHIFT)) & CCM_CCGR4_CG5_MASK)
#define CCM_CCGR4_CG6_MASK                       (0x3000U)
#define CCM_CCGR4_CG6_SHIFT                      (12U)
#define CCM_CCGR4_CG6(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG6_SHIFT)) & CCM_CCGR4_CG6_MASK)
#define CCM_CCGR4_CG7_MASK                       (0xC000U)
#define CCM_CCGR4_CG7_SHIFT                      (14U)
#define CCM_CCGR4_CG7(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG7_SHIFT)) & CCM_CCGR4_CG7_MASK)
#define CCM_CCGR4_CG8_MASK                       (0x30000U)
#define CCM_CCGR4_CG8_SHIFT                      (16U)
#define CCM_CCGR4_CG8(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG8_SHIFT)) & CCM_CCGR4_CG8_MASK)
#define CCM_CCGR4_CG9_MASK                       (0xC0000U)
#define CCM_CCGR4_CG9_SHIFT                      (18U)
#define CCM_CCGR4_CG9(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG9_SHIFT)) & CCM_CCGR4_CG9_MASK)
#define CCM_CCGR4_CG10_MASK                      (0x300000U)
#define CCM_CCGR4_CG10_SHIFT                     (20U)
#define CCM_CCGR4_CG10(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG10_SHIFT)) & CCM_CCGR4_CG10_MASK)
#define CCM_CCGR4_CG11_MASK                      (0xC00000U)
#define CCM_CCGR4_CG11_SHIFT                     (22U)
#define CCM_CCGR4_CG11(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG11_SHIFT)) & CCM_CCGR4_CG11_MASK)
#define CCM_CCGR4_CG12_MASK                      (0x3000000U)
#define CCM_CCGR4_CG12_SHIFT                     (24U)
#define CCM_CCGR4_CG12(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG12_SHIFT)) & CCM_CCGR4_CG12_MASK)
#define CCM_CCGR4_CG13_MASK                      (0xC000000U)
#define CCM_CCGR4_CG13_SHIFT                     (26U)
#define CCM_CCGR4_CG13(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG13_SHIFT)) & CCM_CCGR4_CG13_MASK)
#define CCM_CCGR4_CG14_MASK                      (0x30000000U)
#define CCM_CCGR4_CG14_SHIFT                     (28U)
#define CCM_CCGR4_CG14(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG14_SHIFT)) & CCM_CCGR4_CG14_MASK)
#define CCM_CCGR4_CG15_MASK                      (0xC0000000U)
#define CCM_CCGR4_CG15_SHIFT                     (30U)
#define CCM_CCGR4_CG15(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR4_CG15_SHIFT)) & CCM_CCGR4_CG15_MASK)
/*! @} */

/*! @name CCGR5 - CCM Clock Gating Register 5 */
/*! @{ */
#define CCM_CCGR5_CG0_MASK                       (0x3U)
#define CCM_CCGR5_CG0_SHIFT                      (0U)
#define CCM_CCGR5_CG0(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG0_SHIFT)) & CCM_CCGR5_CG0_MASK)
#define CCM_CCGR5_CG1_MASK                       (0xCU)
#define CCM_CCGR5_CG1_SHIFT                      (2U)
#define CCM_CCGR5_CG1(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG1_SHIFT)) & CCM_CCGR5_CG1_MASK)
#define CCM_CCGR5_CG2_MASK                       (0x30U)
#define CCM_CCGR5_CG2_SHIFT                      (4U)
#define CCM_CCGR5_CG2(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG2_SHIFT)) & CCM_CCGR5_CG2_MASK)
#define CCM_CCGR5_CG3_MASK                       (0xC0U)
#define CCM_CCGR5_CG3_SHIFT                      (6U)
#define CCM_CCGR5_CG3(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG3_SHIFT)) & CCM_CCGR5_CG3_MASK)
#define CCM_CCGR5_CG4_MASK                       (0x300U)
#define CCM_CCGR5_CG4_SHIFT                      (8U)
#define CCM_CCGR5_CG4(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG4_SHIFT)) & CCM_CCGR5_CG4_MASK)
#define CCM_CCGR5_CG5_MASK                       (0xC00U)
#define CCM_CCGR5_CG5_SHIFT                      (10U)
#define CCM_CCGR5_CG5(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG5_SHIFT)) & CCM_CCGR5_CG5_MASK)
#define CCM_CCGR5_CG6_MASK                       (0x3000U)
#define CCM_CCGR5_CG6_SHIFT                      (12U)
#define CCM_CCGR5_CG6(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG6_SHIFT)) & CCM_CCGR5_CG6_MASK)
#define CCM_CCGR5_CG7_MASK                       (0xC000U)
#define CCM_CCGR5_CG7_SHIFT                      (14U)
#define CCM_CCGR5_CG7(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG7_SHIFT)) & CCM_CCGR5_CG7_MASK)
#define CCM_CCGR5_CG8_MASK                       (0x30000U)
#define CCM_CCGR5_CG8_SHIFT                      (16U)
#define CCM_CCGR5_CG8(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG8_SHIFT)) & CCM_CCGR5_CG8_MASK)
#define CCM_CCGR5_CG9_MASK                       (0xC0000U)
#define CCM_CCGR5_CG9_SHIFT                      (18U)
#define CCM_CCGR5_CG9(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG9_SHIFT)) & CCM_CCGR5_CG9_MASK)
#define CCM_CCGR5_CG10_MASK                      (0x300000U)
#define CCM_CCGR5_CG10_SHIFT                     (20U)
#define CCM_CCGR5_CG10(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG10_SHIFT)) & CCM_CCGR5_CG10_MASK)
#define CCM_CCGR5_CG11_MASK                      (0xC00000U)
#define CCM_CCGR5_CG11_SHIFT                     (22U)
#define CCM_CCGR5_CG11(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG11_SHIFT)) & CCM_CCGR5_CG11_MASK)
#define CCM_CCGR5_CG12_MASK                      (0x3000000U)
#define CCM_CCGR5_CG12_SHIFT                     (24U)
#define CCM_CCGR5_CG12(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG12_SHIFT)) & CCM_CCGR5_CG12_MASK)
#define CCM_CCGR5_CG13_MASK                      (0xC000000U)
#define CCM_CCGR5_CG13_SHIFT                     (26U)
#define CCM_CCGR5_CG13(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG13_SHIFT)) & CCM_CCGR5_CG13_MASK)
#define CCM_CCGR5_CG14_MASK                      (0x30000000U)
#define CCM_CCGR5_CG14_SHIFT                     (28U)
#define CCM_CCGR5_CG14(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG14_SHIFT)) & CCM_CCGR5_CG14_MASK)
#define CCM_CCGR5_CG15_MASK                      (0xC0000000U)
#define CCM_CCGR5_CG15_SHIFT                     (30U)
#define CCM_CCGR5_CG15(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR5_CG15_SHIFT)) & CCM_CCGR5_CG15_MASK)
/*! @} */

/*! @name CCGR6 - CCM Clock Gating Register 6 */
/*! @{ */
#define CCM_CCGR6_CG0_MASK                       (0x3U)
#define CCM_CCGR6_CG0_SHIFT                      (0U)
#define CCM_CCGR6_CG0(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG0_SHIFT)) & CCM_CCGR6_CG0_MASK)
#define CCM_CCGR6_CG1_MASK                       (0xCU)
#define CCM_CCGR6_CG1_SHIFT                      (2U)
#define CCM_CCGR6_CG1(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG1_SHIFT)) & CCM_CCGR6_CG1_MASK)
#define CCM_CCGR6_CG2_MASK                       (0x30U)
#define CCM_CCGR6_CG2_SHIFT                      (4U)
#define CCM_CCGR6_CG2(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG2_SHIFT)) & CCM_CCGR6_CG2_MASK)
#define CCM_CCGR6_CG3_MASK                       (0xC0U)
#define CCM_CCGR6_CG3_SHIFT                      (6U)
#define CCM_CCGR6_CG3(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG3_SHIFT)) & CCM_CCGR6_CG3_MASK)
#define CCM_CCGR6_CG4_MASK                       (0x300U)
#define CCM_CCGR6_CG4_SHIFT                      (8U)
#define CCM_CCGR6_CG4(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG4_SHIFT)) & CCM_CCGR6_CG4_MASK)
#define CCM_CCGR6_CG5_MASK                       (0xC00U)
#define CCM_CCGR6_CG5_SHIFT                      (10U)
#define CCM_CCGR6_CG5(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG5_SHIFT)) & CCM_CCGR6_CG5_MASK)
#define CCM_CCGR6_CG6_MASK                       (0x3000U)
#define CCM_CCGR6_CG6_SHIFT                      (12U)
#define CCM_CCGR6_CG6(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG6_SHIFT)) & CCM_CCGR6_CG6_MASK)
#define CCM_CCGR6_CG7_MASK                       (0xC000U)
#define CCM_CCGR6_CG7_SHIFT                      (14U)
#define CCM_CCGR6_CG7(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG7_SHIFT)) & CCM_CCGR6_CG7_MASK)
#define CCM_CCGR6_CG8_MASK                       (0x30000U)
#define CCM_CCGR6_CG8_SHIFT                      (16U)
#define CCM_CCGR6_CG8(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG8_SHIFT)) & CCM_CCGR6_CG8_MASK)
#define CCM_CCGR6_CG9_MASK                       (0xC0000U)
#define CCM_CCGR6_CG9_SHIFT                      (18U)
#define CCM_CCGR6_CG9(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG9_SHIFT)) & CCM_CCGR6_CG9_MASK)
#define CCM_CCGR6_CG10_MASK                      (0x300000U)
#define CCM_CCGR6_CG10_SHIFT                     (20U)
#define CCM_CCGR6_CG10(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG10_SHIFT)) & CCM_CCGR6_CG10_MASK)
#define CCM_CCGR6_CG11_MASK                      (0xC00000U)
#define CCM_CCGR6_CG11_SHIFT                     (22U)
#define CCM_CCGR6_CG11(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG11_SHIFT)) & CCM_CCGR6_CG11_MASK)
#define CCM_CCGR6_CG12_MASK                      (0x3000000U)
#define CCM_CCGR6_CG12_SHIFT                     (24U)
#define CCM_CCGR6_CG12(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG12_SHIFT)) & CCM_CCGR6_CG12_MASK)
#define CCM_CCGR6_CG13_MASK                      (0xC000000U)
#define CCM_CCGR6_CG13_SHIFT                     (26U)
#define CCM_CCGR6_CG13(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG13_SHIFT)) & CCM_CCGR6_CG13_MASK)
#define CCM_CCGR6_CG14_MASK                      (0x30000000U)
#define CCM_CCGR6_CG14_SHIFT                     (28U)
#define CCM_CCGR6_CG14(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG14_SHIFT)) & CCM_CCGR6_CG14_MASK)
#define CCM_CCGR6_CG15_MASK                      (0xC0000000U)
#define CCM_CCGR6_CG15_SHIFT                     (30U)
#define CCM_CCGR6_CG15(x)                        (((uint32_t)(((uint32_t)(x)) << CCM_CCGR6_CG15_SHIFT)) & CCM_CCGR6_CG15_MASK)
/*! @} */

/*! @name CCGR7 - CCM Clock Gating Register 7 */
/*! @{ */
#define CCM_CCGR7_CG0_MASK                       (0x3U)
#define CCM_CCGR7_CG0_SHIFT                      (0U)
#define CCM_CCGR7_CG0(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR7_CG0_SHIFT)) & CCM_CCGR7_CG0_MASK)
#define CCM_CCGR7_CG1_MASK                       (0xCU)
#define CCM_CCGR7_CG1_SHIFT                      (2U)
#define CCM_CCGR7_CG1(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR7_CG1_SHIFT)) & CCM_CCGR7_CG1_MASK)
#define CCM_CCGR7_CG2_MASK                       (0x30U)
#define CCM_CCGR7_CG2_SHIFT                      (4U)
#define CCM_CCGR7_CG2(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR7_CG2_SHIFT)) & CCM_CCGR7_CG2_MASK)
#define CCM_CCGR7_CG3_MASK                       (0xC0U)
#define CCM_CCGR7_CG3_SHIFT                      (6U)
#define CCM_CCGR7_CG3(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR7_CG3_SHIFT)) & CCM_CCGR7_CG3_MASK)
#define CCM_CCGR7_CG4_MASK                       (0x300U)
#define CCM_CCGR7_CG4_SHIFT                      (8U)
#define CCM_CCGR7_CG4(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR7_CG4_SHIFT)) & CCM_CCGR7_CG4_MASK)
#define CCM_CCGR7_CG5_MASK                       (0xC00U)
#define CCM_CCGR7_CG5_SHIFT                      (10U)
#define CCM_CCGR7_CG5(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR7_CG5_SHIFT)) & CCM_CCGR7_CG5_MASK)
#define CCM_CCGR7_CG6_MASK                       (0x3000U)
#define CCM_CCGR7_CG6_SHIFT                      (12U)
#define CCM_CCGR7_CG6(x)                         (((uint32_t)(((uint32_t)(x)) << CCM_CCGR7_CG6_SHIFT)) & CCM_CCGR7_CG6_MASK)
/*! @} */

/*! @name CMEOR - CCM Module Enable Overide Register */
/*! @{ */
#define CCM_CMEOR_MOD_EN_OV_GPT_MASK             (0x20U)
#define CCM_CMEOR_MOD_EN_OV_GPT_SHIFT            (5U)
/*! MOD_EN_OV_GPT
 *  0b0..don't override module enable signal
 *  0b1..override module enable signal
 */
#define CCM_CMEOR_MOD_EN_OV_GPT(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CMEOR_MOD_EN_OV_GPT_SHIFT)) & CCM_CMEOR_MOD_EN_OV_GPT_MASK)
#define CCM_CMEOR_MOD_EN_OV_PIT_MASK             (0x40U)
#define CCM_CMEOR_MOD_EN_OV_PIT_SHIFT            (6U)
/*! MOD_EN_OV_PIT
 *  0b0..don't override module enable signal
 *  0b1..override module enable signal
 */
#define CCM_CMEOR_MOD_EN_OV_PIT(x)               (((uint32_t)(((uint32_t)(x)) << CCM_CMEOR_MOD_EN_OV_PIT_SHIFT)) & CCM_CMEOR_MOD_EN_OV_PIT_MASK)
#define CCM_CMEOR_MOD_EN_USDHC_MASK              (0x80U)
#define CCM_CMEOR_MOD_EN_USDHC_SHIFT             (7U)
/*! MOD_EN_USDHC
 *  0b0..don't override module enable signal
 *  0b1..override module enable signal
 */
#define CCM_CMEOR_MOD_EN_USDHC(x)                (((uint32_t)(((uint32_t)(x)) << CCM_CMEOR_MOD_EN_USDHC_SHIFT)) & CCM_CMEOR_MOD_EN_USDHC_MASK)
#define CCM_CMEOR_MOD_EN_OV_TRNG_MASK            (0x200U)
#define CCM_CMEOR_MOD_EN_OV_TRNG_SHIFT           (9U)
/*! MOD_EN_OV_TRNG
 *  0b0..don't override module enable signal
 *  0b1..override module enable signal
 */
#define CCM_CMEOR_MOD_EN_OV_TRNG(x)              (((uint32_t)(((uint32_t)(x)) << CCM_CMEOR_MOD_EN_OV_TRNG_SHIFT)) & CCM_CMEOR_MOD_EN_OV_TRNG_MASK)
#define CCM_CMEOR_MOD_EN_OV_CANFD_CPI_MASK       (0x400U)
#define CCM_CMEOR_MOD_EN_OV_CANFD_CPI_SHIFT      (10U)
/*! MOD_EN_OV_CANFD_CPI
 *  0b0..don't override module enable signal
 *  0b1..override module enable signal
 */
#define CCM_CMEOR_MOD_EN_OV_CANFD_CPI(x)         (((uint32_t)(((uint32_t)(x)) << CCM_CMEOR_MOD_EN_OV_CANFD_CPI_SHIFT)) & CCM_CMEOR_MOD_EN_OV_CANFD_CPI_MASK)
#define CCM_CMEOR_MOD_EN_OV_CAN2_CPI_MASK        (0x10000000U)
#define CCM_CMEOR_MOD_EN_OV_CAN2_CPI_SHIFT       (28U)
/*! MOD_EN_OV_CAN2_CPI
 *  0b0..don't override module enable signal
 *  0b1..override module enable signal
 */
#define CCM_CMEOR_MOD_EN_OV_CAN2_CPI(x)          (((uint32_t)(((uint32_t)(x)) << CCM_CMEOR_MOD_EN_OV_CAN2_CPI_SHIFT)) & CCM_CMEOR_MOD_EN_OV_CAN2_CPI_MASK)
#define CCM_CMEOR_MOD_EN_OV_CAN1_CPI_MASK        (0x40000000U)
#define CCM_CMEOR_MOD_EN_OV_CAN1_CPI_SHIFT       (30U)
/*! MOD_EN_OV_CAN1_CPI
 *  0b0..don't overide module enable signal
 *  0b1..overide module enable signal
 */
#define CCM_CMEOR_MOD_EN_OV_CAN1_CPI(x)          (((uint32_t)(((uint32_t)(x)) << CCM_CMEOR_MOD_EN_OV_CAN1_CPI_SHIFT)) & CCM_CMEOR_MOD_EN_OV_CAN1_CPI_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group CCM_Register_Masks */


/* CCM - Peripheral instance base addresses */
/** Peripheral CCM base address */
#define CCM_BASE                                 (0x400FC000u)
/** Peripheral CCM base pointer */
#define CCM                                      ((CCM_Type *)CCM_BASE)
/** Array initializer of CCM peripheral base addresses */
#define CCM_BASE_ADDRS                           { CCM_BASE }
/** Array initializer of CCM peripheral base pointers */
#define CCM_BASE_PTRS                            { CCM }
/** Interrupt vectors for the CCM peripheral type */
#define CCM_IRQS                                 { CCM_1_IRQn, CCM_2_IRQn }

/*!
 * @}
 */ /* end of group CCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CCM_ANALOG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CCM_ANALOG_Peripheral_Access_Layer CCM_ANALOG Peripheral Access Layer
 * @{
 */

/** CCM_ANALOG - Register Layout Typedef */
typedef struct {
  __IO uint32_t PLL_ARM;                           /**< Analog ARM PLL control Register, offset: 0x0 */
  __IO uint32_t PLL_ARM_SET;                       /**< Analog ARM PLL control Register, offset: 0x4 */
  __IO uint32_t PLL_ARM_CLR;                       /**< Analog ARM PLL control Register, offset: 0x8 */
  __IO uint32_t PLL_ARM_TOG;                       /**< Analog ARM PLL control Register, offset: 0xC */
  __IO uint32_t PLL_USB1;                          /**< Analog USB1 480MHz PLL Control Register, offset: 0x10 */
  __IO uint32_t PLL_USB1_SET;                      /**< Analog USB1 480MHz PLL Control Register, offset: 0x14 */
  __IO uint32_t PLL_USB1_CLR;                      /**< Analog USB1 480MHz PLL Control Register, offset: 0x18 */
  __IO uint32_t PLL_USB1_TOG;                      /**< Analog USB1 480MHz PLL Control Register, offset: 0x1C */
  __IO uint32_t PLL_USB2;                          /**< Analog USB2 480MHz PLL Control Register, offset: 0x20 */
  __IO uint32_t PLL_USB2_SET;                      /**< Analog USB2 480MHz PLL Control Register, offset: 0x24 */
  __IO uint32_t PLL_USB2_CLR;                      /**< Analog USB2 480MHz PLL Control Register, offset: 0x28 */
  __IO uint32_t PLL_USB2_TOG;                      /**< Analog USB2 480MHz PLL Control Register, offset: 0x2C */
  __IO uint32_t PLL_SYS;                           /**< Analog System PLL Control Register, offset: 0x30 */
  __IO uint32_t PLL_SYS_SET;                       /**< Analog System PLL Control Register, offset: 0x34 */
  __IO uint32_t PLL_SYS_CLR;                       /**< Analog System PLL Control Register, offset: 0x38 */
  __IO uint32_t PLL_SYS_TOG;                       /**< Analog System PLL Control Register, offset: 0x3C */
  __IO uint32_t PLL_SYS_SS;                        /**< 528MHz System PLL Spread Spectrum Register, offset: 0x40 */
       uint8_t RESERVED_0[12];
  __IO uint32_t PLL_SYS_NUM;                       /**< Numerator of 528MHz System PLL Fractional Loop Divider Register, offset: 0x50 */
       uint8_t RESERVED_1[12];
  __IO uint32_t PLL_SYS_DENOM;                     /**< Denominator of 528MHz System PLL Fractional Loop Divider Register, offset: 0x60 */
       uint8_t RESERVED_2[12];
  __IO uint32_t PLL_AUDIO;                         /**< Analog Audio PLL control Register, offset: 0x70 */
  __IO uint32_t PLL_AUDIO_SET;                     /**< Analog Audio PLL control Register, offset: 0x74 */
  __IO uint32_t PLL_AUDIO_CLR;                     /**< Analog Audio PLL control Register, offset: 0x78 */
  __IO uint32_t PLL_AUDIO_TOG;                     /**< Analog Audio PLL control Register, offset: 0x7C */
  __IO uint32_t PLL_AUDIO_NUM;                     /**< Numerator of Audio PLL Fractional Loop Divider Register, offset: 0x80 */
       uint8_t RESERVED_3[12];
  __IO uint32_t PLL_AUDIO_DENOM;                   /**< Denominator of Audio PLL Fractional Loop Divider Register, offset: 0x90 */
       uint8_t RESERVED_4[12];
  __IO uint32_t PLL_VIDEO;                         /**< Analog Video PLL control Register, offset: 0xA0 */
  __IO uint32_t PLL_VIDEO_SET;                     /**< Analog Video PLL control Register, offset: 0xA4 */
  __IO uint32_t PLL_VIDEO_CLR;                     /**< Analog Video PLL control Register, offset: 0xA8 */
  __IO uint32_t PLL_VIDEO_TOG;                     /**< Analog Video PLL control Register, offset: 0xAC */
  __IO uint32_t PLL_VIDEO_NUM;                     /**< Numerator of Video PLL Fractional Loop Divider Register, offset: 0xB0 */
       uint8_t RESERVED_5[12];
  __IO uint32_t PLL_VIDEO_DENOM;                   /**< Denominator of Video PLL Fractional Loop Divider Register, offset: 0xC0 */
       uint8_t RESERVED_6[28];
  __IO uint32_t PLL_ENET;                          /**< Analog ENET PLL Control Register, offset: 0xE0 */
  __IO uint32_t PLL_ENET_SET;                      /**< Analog ENET PLL Control Register, offset: 0xE4 */
  __IO uint32_t PLL_ENET_CLR;                      /**< Analog ENET PLL Control Register, offset: 0xE8 */
  __IO uint32_t PLL_ENET_TOG;                      /**< Analog ENET PLL Control Register, offset: 0xEC */
  __IO uint32_t PFD_480;                           /**< 480MHz Clock (PLL3) Phase Fractional Divider Control Register, offset: 0xF0 */
  __IO uint32_t PFD_480_SET;                       /**< 480MHz Clock (PLL3) Phase Fractional Divider Control Register, offset: 0xF4 */
  __IO uint32_t PFD_480_CLR;                       /**< 480MHz Clock (PLL3) Phase Fractional Divider Control Register, offset: 0xF8 */
  __IO uint32_t PFD_480_TOG;                       /**< 480MHz Clock (PLL3) Phase Fractional Divider Control Register, offset: 0xFC */
  __IO uint32_t PFD_528;                           /**< 528MHz Clock (PLL2) Phase Fractional Divider Control Register, offset: 0x100 */
  __IO uint32_t PFD_528_SET;                       /**< 528MHz Clock (PLL2) Phase Fractional Divider Control Register, offset: 0x104 */
  __IO uint32_t PFD_528_CLR;                       /**< 528MHz Clock (PLL2) Phase Fractional Divider Control Register, offset: 0x108 */
  __IO uint32_t PFD_528_TOG;                       /**< 528MHz Clock (PLL2) Phase Fractional Divider Control Register, offset: 0x10C */
       uint8_t RESERVED_7[64];
  __IO uint32_t MISC0;                             /**< Miscellaneous Register 0, offset: 0x150 */
  __IO uint32_t MISC0_SET;                         /**< Miscellaneous Register 0, offset: 0x154 */
  __IO uint32_t MISC0_CLR;                         /**< Miscellaneous Register 0, offset: 0x158 */
  __IO uint32_t MISC0_TOG;                         /**< Miscellaneous Register 0, offset: 0x15C */
  __IO uint32_t MISC1;                             /**< Miscellaneous Register 1, offset: 0x160 */
  __IO uint32_t MISC1_SET;                         /**< Miscellaneous Register 1, offset: 0x164 */
  __IO uint32_t MISC1_CLR;                         /**< Miscellaneous Register 1, offset: 0x168 */
  __IO uint32_t MISC1_TOG;                         /**< Miscellaneous Register 1, offset: 0x16C */
  __IO uint32_t MISC2;                             /**< Miscellaneous Register 2, offset: 0x170 */
  __IO uint32_t MISC2_SET;                         /**< Miscellaneous Register 2, offset: 0x174 */
  __IO uint32_t MISC2_CLR;                         /**< Miscellaneous Register 2, offset: 0x178 */
  __IO uint32_t MISC2_TOG;                         /**< Miscellaneous Register 2, offset: 0x17C */
} CCM_ANALOG_Type;

/* ----------------------------------------------------------------------------
   -- CCM_ANALOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CCM_ANALOG_Register_Masks CCM_ANALOG Register Masks
 * @{
 */

/*! @name PLL_ARM - Analog ARM PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_ARM_DIV_SELECT_MASK       (0x7FU)
#define CCM_ANALOG_PLL_ARM_DIV_SELECT_SHIFT      (0U)
#define CCM_ANALOG_PLL_ARM_DIV_SELECT(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ARM_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ARM_POWERDOWN_MASK        (0x1000U)
#define CCM_ANALOG_PLL_ARM_POWERDOWN_SHIFT       (12U)
#define CCM_ANALOG_PLL_ARM_POWERDOWN(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_ARM_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_ARM_ENABLE_MASK           (0x2000U)
#define CCM_ANALOG_PLL_ARM_ENABLE_SHIFT          (13U)
#define CCM_ANALOG_PLL_ARM_ENABLE(x)             (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_ENABLE_SHIFT)) & CCM_ANALOG_PLL_ARM_ENABLE_MASK)
#define CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_MASK   (0xC000U)
#define CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_SHIFT  (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_ARM_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_ARM_BYPASS_MASK           (0x10000U)
#define CCM_ANALOG_PLL_ARM_BYPASS_SHIFT          (16U)
#define CCM_ANALOG_PLL_ARM_BYPASS(x)             (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_BYPASS_SHIFT)) & CCM_ANALOG_PLL_ARM_BYPASS_MASK)
#define CCM_ANALOG_PLL_ARM_PLL_SEL_MASK          (0x80000U)
#define CCM_ANALOG_PLL_ARM_PLL_SEL_SHIFT         (19U)
#define CCM_ANALOG_PLL_ARM_PLL_SEL(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_PLL_SEL_SHIFT)) & CCM_ANALOG_PLL_ARM_PLL_SEL_MASK)
#define CCM_ANALOG_PLL_ARM_LOCK_MASK             (0x80000000U)
#define CCM_ANALOG_PLL_ARM_LOCK_SHIFT            (31U)
#define CCM_ANALOG_PLL_ARM_LOCK(x)               (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_LOCK_SHIFT)) & CCM_ANALOG_PLL_ARM_LOCK_MASK)
/*! @} */

/*! @name PLL_ARM_SET - Analog ARM PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_ARM_SET_DIV_SELECT_MASK   (0x7FU)
#define CCM_ANALOG_PLL_ARM_SET_DIV_SELECT_SHIFT  (0U)
#define CCM_ANALOG_PLL_ARM_SET_DIV_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_SET_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ARM_SET_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ARM_SET_POWERDOWN_MASK    (0x1000U)
#define CCM_ANALOG_PLL_ARM_SET_POWERDOWN_SHIFT   (12U)
#define CCM_ANALOG_PLL_ARM_SET_POWERDOWN(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_SET_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_ARM_SET_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_ARM_SET_ENABLE_MASK       (0x2000U)
#define CCM_ANALOG_PLL_ARM_SET_ENABLE_SHIFT      (13U)
#define CCM_ANALOG_PLL_ARM_SET_ENABLE(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_SET_ENABLE_SHIFT)) & CCM_ANALOG_PLL_ARM_SET_ENABLE_MASK)
#define CCM_ANALOG_PLL_ARM_SET_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_ARM_SET_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_ARM_SET_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_SET_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_ARM_SET_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_ARM_SET_BYPASS_MASK       (0x10000U)
#define CCM_ANALOG_PLL_ARM_SET_BYPASS_SHIFT      (16U)
#define CCM_ANALOG_PLL_ARM_SET_BYPASS(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_SET_BYPASS_SHIFT)) & CCM_ANALOG_PLL_ARM_SET_BYPASS_MASK)
#define CCM_ANALOG_PLL_ARM_SET_PLL_SEL_MASK      (0x80000U)
#define CCM_ANALOG_PLL_ARM_SET_PLL_SEL_SHIFT     (19U)
#define CCM_ANALOG_PLL_ARM_SET_PLL_SEL(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_SET_PLL_SEL_SHIFT)) & CCM_ANALOG_PLL_ARM_SET_PLL_SEL_MASK)
#define CCM_ANALOG_PLL_ARM_SET_LOCK_MASK         (0x80000000U)
#define CCM_ANALOG_PLL_ARM_SET_LOCK_SHIFT        (31U)
#define CCM_ANALOG_PLL_ARM_SET_LOCK(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_SET_LOCK_SHIFT)) & CCM_ANALOG_PLL_ARM_SET_LOCK_MASK)
/*! @} */

/*! @name PLL_ARM_CLR - Analog ARM PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_ARM_CLR_DIV_SELECT_MASK   (0x7FU)
#define CCM_ANALOG_PLL_ARM_CLR_DIV_SELECT_SHIFT  (0U)
#define CCM_ANALOG_PLL_ARM_CLR_DIV_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_CLR_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ARM_CLR_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ARM_CLR_POWERDOWN_MASK    (0x1000U)
#define CCM_ANALOG_PLL_ARM_CLR_POWERDOWN_SHIFT   (12U)
#define CCM_ANALOG_PLL_ARM_CLR_POWERDOWN(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_CLR_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_ARM_CLR_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_ARM_CLR_ENABLE_MASK       (0x2000U)
#define CCM_ANALOG_PLL_ARM_CLR_ENABLE_SHIFT      (13U)
#define CCM_ANALOG_PLL_ARM_CLR_ENABLE(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_CLR_ENABLE_SHIFT)) & CCM_ANALOG_PLL_ARM_CLR_ENABLE_MASK)
#define CCM_ANALOG_PLL_ARM_CLR_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_ARM_CLR_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_ARM_CLR_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_CLR_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_ARM_CLR_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_ARM_CLR_BYPASS_MASK       (0x10000U)
#define CCM_ANALOG_PLL_ARM_CLR_BYPASS_SHIFT      (16U)
#define CCM_ANALOG_PLL_ARM_CLR_BYPASS(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_CLR_BYPASS_SHIFT)) & CCM_ANALOG_PLL_ARM_CLR_BYPASS_MASK)
#define CCM_ANALOG_PLL_ARM_CLR_PLL_SEL_MASK      (0x80000U)
#define CCM_ANALOG_PLL_ARM_CLR_PLL_SEL_SHIFT     (19U)
#define CCM_ANALOG_PLL_ARM_CLR_PLL_SEL(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_CLR_PLL_SEL_SHIFT)) & CCM_ANALOG_PLL_ARM_CLR_PLL_SEL_MASK)
#define CCM_ANALOG_PLL_ARM_CLR_LOCK_MASK         (0x80000000U)
#define CCM_ANALOG_PLL_ARM_CLR_LOCK_SHIFT        (31U)
#define CCM_ANALOG_PLL_ARM_CLR_LOCK(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_CLR_LOCK_SHIFT)) & CCM_ANALOG_PLL_ARM_CLR_LOCK_MASK)
/*! @} */

/*! @name PLL_ARM_TOG - Analog ARM PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_ARM_TOG_DIV_SELECT_MASK   (0x7FU)
#define CCM_ANALOG_PLL_ARM_TOG_DIV_SELECT_SHIFT  (0U)
#define CCM_ANALOG_PLL_ARM_TOG_DIV_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_TOG_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ARM_TOG_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ARM_TOG_POWERDOWN_MASK    (0x1000U)
#define CCM_ANALOG_PLL_ARM_TOG_POWERDOWN_SHIFT   (12U)
#define CCM_ANALOG_PLL_ARM_TOG_POWERDOWN(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_TOG_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_ARM_TOG_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_ARM_TOG_ENABLE_MASK       (0x2000U)
#define CCM_ANALOG_PLL_ARM_TOG_ENABLE_SHIFT      (13U)
#define CCM_ANALOG_PLL_ARM_TOG_ENABLE(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_TOG_ENABLE_SHIFT)) & CCM_ANALOG_PLL_ARM_TOG_ENABLE_MASK)
#define CCM_ANALOG_PLL_ARM_TOG_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_ARM_TOG_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_ARM_TOG_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_TOG_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_ARM_TOG_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_ARM_TOG_BYPASS_MASK       (0x10000U)
#define CCM_ANALOG_PLL_ARM_TOG_BYPASS_SHIFT      (16U)
#define CCM_ANALOG_PLL_ARM_TOG_BYPASS(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_TOG_BYPASS_SHIFT)) & CCM_ANALOG_PLL_ARM_TOG_BYPASS_MASK)
#define CCM_ANALOG_PLL_ARM_TOG_PLL_SEL_MASK      (0x80000U)
#define CCM_ANALOG_PLL_ARM_TOG_PLL_SEL_SHIFT     (19U)
#define CCM_ANALOG_PLL_ARM_TOG_PLL_SEL(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_TOG_PLL_SEL_SHIFT)) & CCM_ANALOG_PLL_ARM_TOG_PLL_SEL_MASK)
#define CCM_ANALOG_PLL_ARM_TOG_LOCK_MASK         (0x80000000U)
#define CCM_ANALOG_PLL_ARM_TOG_LOCK_SHIFT        (31U)
#define CCM_ANALOG_PLL_ARM_TOG_LOCK(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ARM_TOG_LOCK_SHIFT)) & CCM_ANALOG_PLL_ARM_TOG_LOCK_MASK)
/*! @} */

/*! @name PLL_USB1 - Analog USB1 480MHz PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK      (0x2U)
#define CCM_ANALOG_PLL_USB1_DIV_SELECT_SHIFT     (1U)
#define CCM_ANALOG_PLL_USB1_DIV_SELECT(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_USB1_EN_USB_CLKS_MASK     (0x40U)
#define CCM_ANALOG_PLL_USB1_EN_USB_CLKS_SHIFT    (6U)
/*! EN_USB_CLKS
 *  0b0..PLL outputs for USBPHYn off.
 *  0b1..PLL outputs for USBPHYn on.
 */
#define CCM_ANALOG_PLL_USB1_EN_USB_CLKS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_EN_USB_CLKS_SHIFT)) & CCM_ANALOG_PLL_USB1_EN_USB_CLKS_MASK)
#define CCM_ANALOG_PLL_USB1_POWER_MASK           (0x1000U)
#define CCM_ANALOG_PLL_USB1_POWER_SHIFT          (12U)
#define CCM_ANALOG_PLL_USB1_POWER(x)             (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_POWER_SHIFT)) & CCM_ANALOG_PLL_USB1_POWER_MASK)
#define CCM_ANALOG_PLL_USB1_ENABLE_MASK          (0x2000U)
#define CCM_ANALOG_PLL_USB1_ENABLE_SHIFT         (13U)
#define CCM_ANALOG_PLL_USB1_ENABLE(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_ENABLE_SHIFT)) & CCM_ANALOG_PLL_USB1_ENABLE_MASK)
#define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_MASK  (0xC000U)
#define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 */
#define CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_USB1_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_USB1_BYPASS_MASK          (0x10000U)
#define CCM_ANALOG_PLL_USB1_BYPASS_SHIFT         (16U)
#define CCM_ANALOG_PLL_USB1_BYPASS(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_BYPASS_SHIFT)) & CCM_ANALOG_PLL_USB1_BYPASS_MASK)
#define CCM_ANALOG_PLL_USB1_LOCK_MASK            (0x80000000U)
#define CCM_ANALOG_PLL_USB1_LOCK_SHIFT           (31U)
#define CCM_ANALOG_PLL_USB1_LOCK(x)              (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_LOCK_SHIFT)) & CCM_ANALOG_PLL_USB1_LOCK_MASK)
/*! @} */

/*! @name PLL_USB1_SET - Analog USB1 480MHz PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_USB1_SET_DIV_SELECT_MASK  (0x2U)
#define CCM_ANALOG_PLL_USB1_SET_DIV_SELECT_SHIFT (1U)
#define CCM_ANALOG_PLL_USB1_SET_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_SET_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_USB1_SET_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_USB1_SET_EN_USB_CLKS_MASK (0x40U)
#define CCM_ANALOG_PLL_USB1_SET_EN_USB_CLKS_SHIFT (6U)
/*! EN_USB_CLKS
 *  0b0..PLL outputs for USBPHYn off.
 *  0b1..PLL outputs for USBPHYn on.
 */
#define CCM_ANALOG_PLL_USB1_SET_EN_USB_CLKS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_SET_EN_USB_CLKS_SHIFT)) & CCM_ANALOG_PLL_USB1_SET_EN_USB_CLKS_MASK)
#define CCM_ANALOG_PLL_USB1_SET_POWER_MASK       (0x1000U)
#define CCM_ANALOG_PLL_USB1_SET_POWER_SHIFT      (12U)
#define CCM_ANALOG_PLL_USB1_SET_POWER(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_SET_POWER_SHIFT)) & CCM_ANALOG_PLL_USB1_SET_POWER_MASK)
#define CCM_ANALOG_PLL_USB1_SET_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_USB1_SET_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_USB1_SET_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_SET_ENABLE_SHIFT)) & CCM_ANALOG_PLL_USB1_SET_ENABLE_MASK)
#define CCM_ANALOG_PLL_USB1_SET_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_USB1_SET_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 */
#define CCM_ANALOG_PLL_USB1_SET_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_SET_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_USB1_SET_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_USB1_SET_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_USB1_SET_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_USB1_SET_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_SET_BYPASS_SHIFT)) & CCM_ANALOG_PLL_USB1_SET_BYPASS_MASK)
#define CCM_ANALOG_PLL_USB1_SET_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_USB1_SET_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_USB1_SET_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_SET_LOCK_SHIFT)) & CCM_ANALOG_PLL_USB1_SET_LOCK_MASK)
/*! @} */

/*! @name PLL_USB1_CLR - Analog USB1 480MHz PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_USB1_CLR_DIV_SELECT_MASK  (0x2U)
#define CCM_ANALOG_PLL_USB1_CLR_DIV_SELECT_SHIFT (1U)
#define CCM_ANALOG_PLL_USB1_CLR_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_CLR_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_USB1_CLR_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_USB1_CLR_EN_USB_CLKS_MASK (0x40U)
#define CCM_ANALOG_PLL_USB1_CLR_EN_USB_CLKS_SHIFT (6U)
/*! EN_USB_CLKS
 *  0b0..PLL outputs for USBPHYn off.
 *  0b1..PLL outputs for USBPHYn on.
 */
#define CCM_ANALOG_PLL_USB1_CLR_EN_USB_CLKS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_CLR_EN_USB_CLKS_SHIFT)) & CCM_ANALOG_PLL_USB1_CLR_EN_USB_CLKS_MASK)
#define CCM_ANALOG_PLL_USB1_CLR_POWER_MASK       (0x1000U)
#define CCM_ANALOG_PLL_USB1_CLR_POWER_SHIFT      (12U)
#define CCM_ANALOG_PLL_USB1_CLR_POWER(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_CLR_POWER_SHIFT)) & CCM_ANALOG_PLL_USB1_CLR_POWER_MASK)
#define CCM_ANALOG_PLL_USB1_CLR_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_USB1_CLR_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_USB1_CLR_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_CLR_ENABLE_SHIFT)) & CCM_ANALOG_PLL_USB1_CLR_ENABLE_MASK)
#define CCM_ANALOG_PLL_USB1_CLR_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_USB1_CLR_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 */
#define CCM_ANALOG_PLL_USB1_CLR_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_CLR_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_USB1_CLR_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_USB1_CLR_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_USB1_CLR_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_USB1_CLR_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_CLR_BYPASS_SHIFT)) & CCM_ANALOG_PLL_USB1_CLR_BYPASS_MASK)
#define CCM_ANALOG_PLL_USB1_CLR_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_USB1_CLR_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_USB1_CLR_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_CLR_LOCK_SHIFT)) & CCM_ANALOG_PLL_USB1_CLR_LOCK_MASK)
/*! @} */

/*! @name PLL_USB1_TOG - Analog USB1 480MHz PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_USB1_TOG_DIV_SELECT_MASK  (0x2U)
#define CCM_ANALOG_PLL_USB1_TOG_DIV_SELECT_SHIFT (1U)
#define CCM_ANALOG_PLL_USB1_TOG_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_TOG_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_USB1_TOG_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_USB1_TOG_EN_USB_CLKS_MASK (0x40U)
#define CCM_ANALOG_PLL_USB1_TOG_EN_USB_CLKS_SHIFT (6U)
/*! EN_USB_CLKS
 *  0b0..PLL outputs for USBPHYn off.
 *  0b1..PLL outputs for USBPHYn on.
 */
#define CCM_ANALOG_PLL_USB1_TOG_EN_USB_CLKS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_TOG_EN_USB_CLKS_SHIFT)) & CCM_ANALOG_PLL_USB1_TOG_EN_USB_CLKS_MASK)
#define CCM_ANALOG_PLL_USB1_TOG_POWER_MASK       (0x1000U)
#define CCM_ANALOG_PLL_USB1_TOG_POWER_SHIFT      (12U)
#define CCM_ANALOG_PLL_USB1_TOG_POWER(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_TOG_POWER_SHIFT)) & CCM_ANALOG_PLL_USB1_TOG_POWER_MASK)
#define CCM_ANALOG_PLL_USB1_TOG_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_USB1_TOG_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_USB1_TOG_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_TOG_ENABLE_SHIFT)) & CCM_ANALOG_PLL_USB1_TOG_ENABLE_MASK)
#define CCM_ANALOG_PLL_USB1_TOG_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_USB1_TOG_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 */
#define CCM_ANALOG_PLL_USB1_TOG_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_TOG_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_USB1_TOG_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_USB1_TOG_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_USB1_TOG_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_USB1_TOG_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_TOG_BYPASS_SHIFT)) & CCM_ANALOG_PLL_USB1_TOG_BYPASS_MASK)
#define CCM_ANALOG_PLL_USB1_TOG_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_USB1_TOG_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_USB1_TOG_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB1_TOG_LOCK_SHIFT)) & CCM_ANALOG_PLL_USB1_TOG_LOCK_MASK)
/*! @} */

/*! @name PLL_USB2 - Analog USB2 480MHz PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_USB2_DIV_SELECT_MASK      (0x2U)
#define CCM_ANALOG_PLL_USB2_DIV_SELECT_SHIFT     (1U)
#define CCM_ANALOG_PLL_USB2_DIV_SELECT(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_USB2_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_USB2_EN_USB_CLKS_MASK     (0x40U)
#define CCM_ANALOG_PLL_USB2_EN_USB_CLKS_SHIFT    (6U)
#define CCM_ANALOG_PLL_USB2_EN_USB_CLKS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_EN_USB_CLKS_SHIFT)) & CCM_ANALOG_PLL_USB2_EN_USB_CLKS_MASK)
#define CCM_ANALOG_PLL_USB2_POWER_MASK           (0x1000U)
#define CCM_ANALOG_PLL_USB2_POWER_SHIFT          (12U)
#define CCM_ANALOG_PLL_USB2_POWER(x)             (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_POWER_SHIFT)) & CCM_ANALOG_PLL_USB2_POWER_MASK)
#define CCM_ANALOG_PLL_USB2_ENABLE_MASK          (0x2000U)
#define CCM_ANALOG_PLL_USB2_ENABLE_SHIFT         (13U)
#define CCM_ANALOG_PLL_USB2_ENABLE(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_ENABLE_SHIFT)) & CCM_ANALOG_PLL_USB2_ENABLE_MASK)
#define CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_MASK  (0xC000U)
#define CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_USB2_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_USB2_BYPASS_MASK          (0x10000U)
#define CCM_ANALOG_PLL_USB2_BYPASS_SHIFT         (16U)
#define CCM_ANALOG_PLL_USB2_BYPASS(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_BYPASS_SHIFT)) & CCM_ANALOG_PLL_USB2_BYPASS_MASK)
#define CCM_ANALOG_PLL_USB2_LOCK_MASK            (0x80000000U)
#define CCM_ANALOG_PLL_USB2_LOCK_SHIFT           (31U)
#define CCM_ANALOG_PLL_USB2_LOCK(x)              (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_LOCK_SHIFT)) & CCM_ANALOG_PLL_USB2_LOCK_MASK)
/*! @} */

/*! @name PLL_USB2_SET - Analog USB2 480MHz PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_USB2_SET_DIV_SELECT_MASK  (0x2U)
#define CCM_ANALOG_PLL_USB2_SET_DIV_SELECT_SHIFT (1U)
#define CCM_ANALOG_PLL_USB2_SET_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_SET_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_USB2_SET_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_USB2_SET_EN_USB_CLKS_MASK (0x40U)
#define CCM_ANALOG_PLL_USB2_SET_EN_USB_CLKS_SHIFT (6U)
#define CCM_ANALOG_PLL_USB2_SET_EN_USB_CLKS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_SET_EN_USB_CLKS_SHIFT)) & CCM_ANALOG_PLL_USB2_SET_EN_USB_CLKS_MASK)
#define CCM_ANALOG_PLL_USB2_SET_POWER_MASK       (0x1000U)
#define CCM_ANALOG_PLL_USB2_SET_POWER_SHIFT      (12U)
#define CCM_ANALOG_PLL_USB2_SET_POWER(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_SET_POWER_SHIFT)) & CCM_ANALOG_PLL_USB2_SET_POWER_MASK)
#define CCM_ANALOG_PLL_USB2_SET_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_USB2_SET_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_USB2_SET_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_SET_ENABLE_SHIFT)) & CCM_ANALOG_PLL_USB2_SET_ENABLE_MASK)
#define CCM_ANALOG_PLL_USB2_SET_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_USB2_SET_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_USB2_SET_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_SET_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_USB2_SET_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_USB2_SET_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_USB2_SET_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_USB2_SET_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_SET_BYPASS_SHIFT)) & CCM_ANALOG_PLL_USB2_SET_BYPASS_MASK)
#define CCM_ANALOG_PLL_USB2_SET_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_USB2_SET_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_USB2_SET_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_SET_LOCK_SHIFT)) & CCM_ANALOG_PLL_USB2_SET_LOCK_MASK)
/*! @} */

/*! @name PLL_USB2_CLR - Analog USB2 480MHz PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_USB2_CLR_DIV_SELECT_MASK  (0x2U)
#define CCM_ANALOG_PLL_USB2_CLR_DIV_SELECT_SHIFT (1U)
#define CCM_ANALOG_PLL_USB2_CLR_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_CLR_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_USB2_CLR_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_USB2_CLR_EN_USB_CLKS_MASK (0x40U)
#define CCM_ANALOG_PLL_USB2_CLR_EN_USB_CLKS_SHIFT (6U)
#define CCM_ANALOG_PLL_USB2_CLR_EN_USB_CLKS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_CLR_EN_USB_CLKS_SHIFT)) & CCM_ANALOG_PLL_USB2_CLR_EN_USB_CLKS_MASK)
#define CCM_ANALOG_PLL_USB2_CLR_POWER_MASK       (0x1000U)
#define CCM_ANALOG_PLL_USB2_CLR_POWER_SHIFT      (12U)
#define CCM_ANALOG_PLL_USB2_CLR_POWER(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_CLR_POWER_SHIFT)) & CCM_ANALOG_PLL_USB2_CLR_POWER_MASK)
#define CCM_ANALOG_PLL_USB2_CLR_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_USB2_CLR_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_USB2_CLR_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_CLR_ENABLE_SHIFT)) & CCM_ANALOG_PLL_USB2_CLR_ENABLE_MASK)
#define CCM_ANALOG_PLL_USB2_CLR_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_USB2_CLR_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_USB2_CLR_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_CLR_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_USB2_CLR_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_USB2_CLR_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_USB2_CLR_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_USB2_CLR_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_CLR_BYPASS_SHIFT)) & CCM_ANALOG_PLL_USB2_CLR_BYPASS_MASK)
#define CCM_ANALOG_PLL_USB2_CLR_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_USB2_CLR_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_USB2_CLR_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_CLR_LOCK_SHIFT)) & CCM_ANALOG_PLL_USB2_CLR_LOCK_MASK)
/*! @} */

/*! @name PLL_USB2_TOG - Analog USB2 480MHz PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_USB2_TOG_DIV_SELECT_MASK  (0x2U)
#define CCM_ANALOG_PLL_USB2_TOG_DIV_SELECT_SHIFT (1U)
#define CCM_ANALOG_PLL_USB2_TOG_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_TOG_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_USB2_TOG_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_USB2_TOG_EN_USB_CLKS_MASK (0x40U)
#define CCM_ANALOG_PLL_USB2_TOG_EN_USB_CLKS_SHIFT (6U)
#define CCM_ANALOG_PLL_USB2_TOG_EN_USB_CLKS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_TOG_EN_USB_CLKS_SHIFT)) & CCM_ANALOG_PLL_USB2_TOG_EN_USB_CLKS_MASK)
#define CCM_ANALOG_PLL_USB2_TOG_POWER_MASK       (0x1000U)
#define CCM_ANALOG_PLL_USB2_TOG_POWER_SHIFT      (12U)
#define CCM_ANALOG_PLL_USB2_TOG_POWER(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_TOG_POWER_SHIFT)) & CCM_ANALOG_PLL_USB2_TOG_POWER_MASK)
#define CCM_ANALOG_PLL_USB2_TOG_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_USB2_TOG_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_USB2_TOG_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_TOG_ENABLE_SHIFT)) & CCM_ANALOG_PLL_USB2_TOG_ENABLE_MASK)
#define CCM_ANALOG_PLL_USB2_TOG_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_USB2_TOG_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_USB2_TOG_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_TOG_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_USB2_TOG_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_USB2_TOG_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_USB2_TOG_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_USB2_TOG_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_TOG_BYPASS_SHIFT)) & CCM_ANALOG_PLL_USB2_TOG_BYPASS_MASK)
#define CCM_ANALOG_PLL_USB2_TOG_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_USB2_TOG_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_USB2_TOG_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_USB2_TOG_LOCK_SHIFT)) & CCM_ANALOG_PLL_USB2_TOG_LOCK_MASK)
/*! @} */

/*! @name PLL_SYS - Analog System PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_SYS_DIV_SELECT_MASK       (0x1U)
#define CCM_ANALOG_PLL_SYS_DIV_SELECT_SHIFT      (0U)
#define CCM_ANALOG_PLL_SYS_DIV_SELECT(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_SYS_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_SYS_POWERDOWN_MASK        (0x1000U)
#define CCM_ANALOG_PLL_SYS_POWERDOWN_SHIFT       (12U)
#define CCM_ANALOG_PLL_SYS_POWERDOWN(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_SYS_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_SYS_ENABLE_MASK           (0x2000U)
#define CCM_ANALOG_PLL_SYS_ENABLE_SHIFT          (13U)
#define CCM_ANALOG_PLL_SYS_ENABLE(x)             (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_ENABLE_SHIFT)) & CCM_ANALOG_PLL_SYS_ENABLE_MASK)
#define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_MASK   (0xC000U)
#define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_SHIFT  (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 */
#define CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_SYS_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_SYS_BYPASS_MASK           (0x10000U)
#define CCM_ANALOG_PLL_SYS_BYPASS_SHIFT          (16U)
#define CCM_ANALOG_PLL_SYS_BYPASS(x)             (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_BYPASS_SHIFT)) & CCM_ANALOG_PLL_SYS_BYPASS_MASK)
#define CCM_ANALOG_PLL_SYS_LOCK_MASK             (0x80000000U)
#define CCM_ANALOG_PLL_SYS_LOCK_SHIFT            (31U)
#define CCM_ANALOG_PLL_SYS_LOCK(x)               (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_LOCK_SHIFT)) & CCM_ANALOG_PLL_SYS_LOCK_MASK)
/*! @} */

/*! @name PLL_SYS_SET - Analog System PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_SYS_SET_DIV_SELECT_MASK   (0x1U)
#define CCM_ANALOG_PLL_SYS_SET_DIV_SELECT_SHIFT  (0U)
#define CCM_ANALOG_PLL_SYS_SET_DIV_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SET_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_SYS_SET_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_SYS_SET_POWERDOWN_MASK    (0x1000U)
#define CCM_ANALOG_PLL_SYS_SET_POWERDOWN_SHIFT   (12U)
#define CCM_ANALOG_PLL_SYS_SET_POWERDOWN(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SET_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_SYS_SET_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_SYS_SET_ENABLE_MASK       (0x2000U)
#define CCM_ANALOG_PLL_SYS_SET_ENABLE_SHIFT      (13U)
#define CCM_ANALOG_PLL_SYS_SET_ENABLE(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SET_ENABLE_SHIFT)) & CCM_ANALOG_PLL_SYS_SET_ENABLE_MASK)
#define CCM_ANALOG_PLL_SYS_SET_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_SYS_SET_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 */
#define CCM_ANALOG_PLL_SYS_SET_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SET_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_SYS_SET_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_SYS_SET_BYPASS_MASK       (0x10000U)
#define CCM_ANALOG_PLL_SYS_SET_BYPASS_SHIFT      (16U)
#define CCM_ANALOG_PLL_SYS_SET_BYPASS(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SET_BYPASS_SHIFT)) & CCM_ANALOG_PLL_SYS_SET_BYPASS_MASK)
#define CCM_ANALOG_PLL_SYS_SET_LOCK_MASK         (0x80000000U)
#define CCM_ANALOG_PLL_SYS_SET_LOCK_SHIFT        (31U)
#define CCM_ANALOG_PLL_SYS_SET_LOCK(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SET_LOCK_SHIFT)) & CCM_ANALOG_PLL_SYS_SET_LOCK_MASK)
/*! @} */

/*! @name PLL_SYS_CLR - Analog System PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_SYS_CLR_DIV_SELECT_MASK   (0x1U)
#define CCM_ANALOG_PLL_SYS_CLR_DIV_SELECT_SHIFT  (0U)
#define CCM_ANALOG_PLL_SYS_CLR_DIV_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_CLR_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_SYS_CLR_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_SYS_CLR_POWERDOWN_MASK    (0x1000U)
#define CCM_ANALOG_PLL_SYS_CLR_POWERDOWN_SHIFT   (12U)
#define CCM_ANALOG_PLL_SYS_CLR_POWERDOWN(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_CLR_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_SYS_CLR_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_SYS_CLR_ENABLE_MASK       (0x2000U)
#define CCM_ANALOG_PLL_SYS_CLR_ENABLE_SHIFT      (13U)
#define CCM_ANALOG_PLL_SYS_CLR_ENABLE(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_CLR_ENABLE_SHIFT)) & CCM_ANALOG_PLL_SYS_CLR_ENABLE_MASK)
#define CCM_ANALOG_PLL_SYS_CLR_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_SYS_CLR_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 */
#define CCM_ANALOG_PLL_SYS_CLR_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_CLR_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_SYS_CLR_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_SYS_CLR_BYPASS_MASK       (0x10000U)
#define CCM_ANALOG_PLL_SYS_CLR_BYPASS_SHIFT      (16U)
#define CCM_ANALOG_PLL_SYS_CLR_BYPASS(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_CLR_BYPASS_SHIFT)) & CCM_ANALOG_PLL_SYS_CLR_BYPASS_MASK)
#define CCM_ANALOG_PLL_SYS_CLR_LOCK_MASK         (0x80000000U)
#define CCM_ANALOG_PLL_SYS_CLR_LOCK_SHIFT        (31U)
#define CCM_ANALOG_PLL_SYS_CLR_LOCK(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_CLR_LOCK_SHIFT)) & CCM_ANALOG_PLL_SYS_CLR_LOCK_MASK)
/*! @} */

/*! @name PLL_SYS_TOG - Analog System PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_SYS_TOG_DIV_SELECT_MASK   (0x1U)
#define CCM_ANALOG_PLL_SYS_TOG_DIV_SELECT_SHIFT  (0U)
#define CCM_ANALOG_PLL_SYS_TOG_DIV_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_TOG_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_SYS_TOG_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_SYS_TOG_POWERDOWN_MASK    (0x1000U)
#define CCM_ANALOG_PLL_SYS_TOG_POWERDOWN_SHIFT   (12U)
#define CCM_ANALOG_PLL_SYS_TOG_POWERDOWN(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_TOG_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_SYS_TOG_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_SYS_TOG_ENABLE_MASK       (0x2000U)
#define CCM_ANALOG_PLL_SYS_TOG_ENABLE_SHIFT      (13U)
#define CCM_ANALOG_PLL_SYS_TOG_ENABLE(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_TOG_ENABLE_SHIFT)) & CCM_ANALOG_PLL_SYS_TOG_ENABLE_MASK)
#define CCM_ANALOG_PLL_SYS_TOG_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_SYS_TOG_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 */
#define CCM_ANALOG_PLL_SYS_TOG_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_TOG_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_SYS_TOG_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_SYS_TOG_BYPASS_MASK       (0x10000U)
#define CCM_ANALOG_PLL_SYS_TOG_BYPASS_SHIFT      (16U)
#define CCM_ANALOG_PLL_SYS_TOG_BYPASS(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_TOG_BYPASS_SHIFT)) & CCM_ANALOG_PLL_SYS_TOG_BYPASS_MASK)
#define CCM_ANALOG_PLL_SYS_TOG_LOCK_MASK         (0x80000000U)
#define CCM_ANALOG_PLL_SYS_TOG_LOCK_SHIFT        (31U)
#define CCM_ANALOG_PLL_SYS_TOG_LOCK(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_TOG_LOCK_SHIFT)) & CCM_ANALOG_PLL_SYS_TOG_LOCK_MASK)
/*! @} */

/*! @name PLL_SYS_SS - 528MHz System PLL Spread Spectrum Register */
/*! @{ */
#define CCM_ANALOG_PLL_SYS_SS_STEP_MASK          (0x7FFFU)
#define CCM_ANALOG_PLL_SYS_SS_STEP_SHIFT         (0U)
#define CCM_ANALOG_PLL_SYS_SS_STEP(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SS_STEP_SHIFT)) & CCM_ANALOG_PLL_SYS_SS_STEP_MASK)
#define CCM_ANALOG_PLL_SYS_SS_ENABLE_MASK        (0x8000U)
#define CCM_ANALOG_PLL_SYS_SS_ENABLE_SHIFT       (15U)
/*! ENABLE - Enable bit
 *  0b0..Spread spectrum modulation disabled
 *  0b1..Soread spectrum modulation enabled
 */
#define CCM_ANALOG_PLL_SYS_SS_ENABLE(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SS_ENABLE_SHIFT)) & CCM_ANALOG_PLL_SYS_SS_ENABLE_MASK)
#define CCM_ANALOG_PLL_SYS_SS_STOP_MASK          (0xFFFF0000U)
#define CCM_ANALOG_PLL_SYS_SS_STOP_SHIFT         (16U)
#define CCM_ANALOG_PLL_SYS_SS_STOP(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SS_STOP_SHIFT)) & CCM_ANALOG_PLL_SYS_SS_STOP_MASK)
/*! @} */

/*! @name PLL_SYS_NUM - Numerator of 528MHz System PLL Fractional Loop Divider Register */
/*! @{ */
#define CCM_ANALOG_PLL_SYS_NUM_A_MASK            (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_SYS_NUM_A_SHIFT           (0U)
#define CCM_ANALOG_PLL_SYS_NUM_A(x)              (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_NUM_A_SHIFT)) & CCM_ANALOG_PLL_SYS_NUM_A_MASK)
/*! @} */

/*! @name PLL_SYS_DENOM - Denominator of 528MHz System PLL Fractional Loop Divider Register */
/*! @{ */
#define CCM_ANALOG_PLL_SYS_DENOM_B_MASK          (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_SYS_DENOM_B_SHIFT         (0U)
#define CCM_ANALOG_PLL_SYS_DENOM_B(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_DENOM_B_SHIFT)) & CCM_ANALOG_PLL_SYS_DENOM_B_MASK)
/*! @} */

/*! @name PLL_AUDIO - Analog Audio PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK     (0x7FU)
#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT    (0U)
#define CCM_ANALOG_PLL_AUDIO_DIV_SELECT(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_AUDIO_POWERDOWN_MASK      (0x1000U)
#define CCM_ANALOG_PLL_AUDIO_POWERDOWN_SHIFT     (12U)
#define CCM_ANALOG_PLL_AUDIO_POWERDOWN(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_AUDIO_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_AUDIO_ENABLE_MASK         (0x2000U)
#define CCM_ANALOG_PLL_AUDIO_ENABLE_SHIFT        (13U)
#define CCM_ANALOG_PLL_AUDIO_ENABLE(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_ENABLE_SHIFT)) & CCM_ANALOG_PLL_AUDIO_ENABLE_MASK)
#define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_AUDIO_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_AUDIO_BYPASS_MASK         (0x10000U)
#define CCM_ANALOG_PLL_AUDIO_BYPASS_SHIFT        (16U)
#define CCM_ANALOG_PLL_AUDIO_BYPASS(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_BYPASS_SHIFT)) & CCM_ANALOG_PLL_AUDIO_BYPASS_MASK)
#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_AUDIO_LOCK_MASK           (0x80000000U)
#define CCM_ANALOG_PLL_AUDIO_LOCK_SHIFT          (31U)
#define CCM_ANALOG_PLL_AUDIO_LOCK(x)             (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_LOCK_SHIFT)) & CCM_ANALOG_PLL_AUDIO_LOCK_MASK)
/*! @} */

/*! @name PLL_AUDIO_SET - Analog Audio PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_AUDIO_SET_DIV_SELECT_MASK (0x7FU)
#define CCM_ANALOG_PLL_AUDIO_SET_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_AUDIO_SET_DIV_SELECT(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_SET_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_SET_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_AUDIO_SET_POWERDOWN_MASK  (0x1000U)
#define CCM_ANALOG_PLL_AUDIO_SET_POWERDOWN_SHIFT (12U)
#define CCM_ANALOG_PLL_AUDIO_SET_POWERDOWN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_SET_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_AUDIO_SET_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_AUDIO_SET_ENABLE_MASK     (0x2000U)
#define CCM_ANALOG_PLL_AUDIO_SET_ENABLE_SHIFT    (13U)
#define CCM_ANALOG_PLL_AUDIO_SET_ENABLE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_SET_ENABLE_SHIFT)) & CCM_ANALOG_PLL_AUDIO_SET_ENABLE_MASK)
#define CCM_ANALOG_PLL_AUDIO_SET_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_AUDIO_SET_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_AUDIO_SET_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_SET_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_AUDIO_SET_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_AUDIO_SET_BYPASS_MASK     (0x10000U)
#define CCM_ANALOG_PLL_AUDIO_SET_BYPASS_SHIFT    (16U)
#define CCM_ANALOG_PLL_AUDIO_SET_BYPASS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_SET_BYPASS_SHIFT)) & CCM_ANALOG_PLL_AUDIO_SET_BYPASS_MASK)
#define CCM_ANALOG_PLL_AUDIO_SET_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_AUDIO_SET_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
#define CCM_ANALOG_PLL_AUDIO_SET_POST_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_SET_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_SET_POST_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_AUDIO_SET_LOCK_MASK       (0x80000000U)
#define CCM_ANALOG_PLL_AUDIO_SET_LOCK_SHIFT      (31U)
#define CCM_ANALOG_PLL_AUDIO_SET_LOCK(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_SET_LOCK_SHIFT)) & CCM_ANALOG_PLL_AUDIO_SET_LOCK_MASK)
/*! @} */

/*! @name PLL_AUDIO_CLR - Analog Audio PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_AUDIO_CLR_DIV_SELECT_MASK (0x7FU)
#define CCM_ANALOG_PLL_AUDIO_CLR_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_AUDIO_CLR_DIV_SELECT(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_CLR_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_CLR_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_AUDIO_CLR_POWERDOWN_MASK  (0x1000U)
#define CCM_ANALOG_PLL_AUDIO_CLR_POWERDOWN_SHIFT (12U)
#define CCM_ANALOG_PLL_AUDIO_CLR_POWERDOWN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_CLR_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_AUDIO_CLR_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_AUDIO_CLR_ENABLE_MASK     (0x2000U)
#define CCM_ANALOG_PLL_AUDIO_CLR_ENABLE_SHIFT    (13U)
#define CCM_ANALOG_PLL_AUDIO_CLR_ENABLE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_CLR_ENABLE_SHIFT)) & CCM_ANALOG_PLL_AUDIO_CLR_ENABLE_MASK)
#define CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_MASK     (0x10000U)
#define CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_SHIFT    (16U)
#define CCM_ANALOG_PLL_AUDIO_CLR_BYPASS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_SHIFT)) & CCM_ANALOG_PLL_AUDIO_CLR_BYPASS_MASK)
#define CCM_ANALOG_PLL_AUDIO_CLR_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_AUDIO_CLR_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
#define CCM_ANALOG_PLL_AUDIO_CLR_POST_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_CLR_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_CLR_POST_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_AUDIO_CLR_LOCK_MASK       (0x80000000U)
#define CCM_ANALOG_PLL_AUDIO_CLR_LOCK_SHIFT      (31U)
#define CCM_ANALOG_PLL_AUDIO_CLR_LOCK(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_CLR_LOCK_SHIFT)) & CCM_ANALOG_PLL_AUDIO_CLR_LOCK_MASK)
/*! @} */

/*! @name PLL_AUDIO_TOG - Analog Audio PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_AUDIO_TOG_DIV_SELECT_MASK (0x7FU)
#define CCM_ANALOG_PLL_AUDIO_TOG_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_AUDIO_TOG_DIV_SELECT(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_TOG_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_TOG_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_AUDIO_TOG_POWERDOWN_MASK  (0x1000U)
#define CCM_ANALOG_PLL_AUDIO_TOG_POWERDOWN_SHIFT (12U)
#define CCM_ANALOG_PLL_AUDIO_TOG_POWERDOWN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_TOG_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_AUDIO_TOG_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_AUDIO_TOG_ENABLE_MASK     (0x2000U)
#define CCM_ANALOG_PLL_AUDIO_TOG_ENABLE_SHIFT    (13U)
#define CCM_ANALOG_PLL_AUDIO_TOG_ENABLE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_TOG_ENABLE_SHIFT)) & CCM_ANALOG_PLL_AUDIO_TOG_ENABLE_MASK)
#define CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_MASK     (0x10000U)
#define CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_SHIFT    (16U)
#define CCM_ANALOG_PLL_AUDIO_TOG_BYPASS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_SHIFT)) & CCM_ANALOG_PLL_AUDIO_TOG_BYPASS_MASK)
#define CCM_ANALOG_PLL_AUDIO_TOG_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_AUDIO_TOG_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
#define CCM_ANALOG_PLL_AUDIO_TOG_POST_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_TOG_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_AUDIO_TOG_POST_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_AUDIO_TOG_LOCK_MASK       (0x80000000U)
#define CCM_ANALOG_PLL_AUDIO_TOG_LOCK_SHIFT      (31U)
#define CCM_ANALOG_PLL_AUDIO_TOG_LOCK(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_TOG_LOCK_SHIFT)) & CCM_ANALOG_PLL_AUDIO_TOG_LOCK_MASK)
/*! @} */

/*! @name PLL_AUDIO_NUM - Numerator of Audio PLL Fractional Loop Divider Register */
/*! @{ */
#define CCM_ANALOG_PLL_AUDIO_NUM_A_MASK          (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_AUDIO_NUM_A_SHIFT         (0U)
#define CCM_ANALOG_PLL_AUDIO_NUM_A(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_NUM_A_SHIFT)) & CCM_ANALOG_PLL_AUDIO_NUM_A_MASK)
/*! @} */

/*! @name PLL_AUDIO_DENOM - Denominator of Audio PLL Fractional Loop Divider Register */
/*! @{ */
#define CCM_ANALOG_PLL_AUDIO_DENOM_B_MASK        (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_AUDIO_DENOM_B_SHIFT       (0U)
#define CCM_ANALOG_PLL_AUDIO_DENOM_B(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_DENOM_B_SHIFT)) & CCM_ANALOG_PLL_AUDIO_DENOM_B_MASK)
/*! @} */

/*! @name PLL_VIDEO - Analog Video PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_VIDEO_DIV_SELECT_MASK     (0x7FU)
#define CCM_ANALOG_PLL_VIDEO_DIV_SELECT_SHIFT    (0U)
#define CCM_ANALOG_PLL_VIDEO_DIV_SELECT(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_VIDEO_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_VIDEO_POWERDOWN_MASK      (0x1000U)
#define CCM_ANALOG_PLL_VIDEO_POWERDOWN_SHIFT     (12U)
#define CCM_ANALOG_PLL_VIDEO_POWERDOWN(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_VIDEO_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_VIDEO_ENABLE_MASK         (0x2000U)
#define CCM_ANALOG_PLL_VIDEO_ENABLE_SHIFT        (13U)
#define CCM_ANALOG_PLL_VIDEO_ENABLE(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_ENABLE_SHIFT)) & CCM_ANALOG_PLL_VIDEO_ENABLE_MASK)
#define CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_VIDEO_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_VIDEO_BYPASS_MASK         (0x10000U)
#define CCM_ANALOG_PLL_VIDEO_BYPASS_SHIFT        (16U)
#define CCM_ANALOG_PLL_VIDEO_BYPASS(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_BYPASS_SHIFT)) & CCM_ANALOG_PLL_VIDEO_BYPASS_MASK)
#define CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
#define CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_VIDEO_POST_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_VIDEO_LOCK_MASK           (0x80000000U)
#define CCM_ANALOG_PLL_VIDEO_LOCK_SHIFT          (31U)
#define CCM_ANALOG_PLL_VIDEO_LOCK(x)             (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_LOCK_SHIFT)) & CCM_ANALOG_PLL_VIDEO_LOCK_MASK)
/*! @} */

/*! @name PLL_VIDEO_SET - Analog Video PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_VIDEO_SET_DIV_SELECT_MASK (0x7FU)
#define CCM_ANALOG_PLL_VIDEO_SET_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_VIDEO_SET_DIV_SELECT(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_SET_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_VIDEO_SET_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_VIDEO_SET_POWERDOWN_MASK  (0x1000U)
#define CCM_ANALOG_PLL_VIDEO_SET_POWERDOWN_SHIFT (12U)
#define CCM_ANALOG_PLL_VIDEO_SET_POWERDOWN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_SET_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_VIDEO_SET_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_VIDEO_SET_ENABLE_MASK     (0x2000U)
#define CCM_ANALOG_PLL_VIDEO_SET_ENABLE_SHIFT    (13U)
#define CCM_ANALOG_PLL_VIDEO_SET_ENABLE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_SET_ENABLE_SHIFT)) & CCM_ANALOG_PLL_VIDEO_SET_ENABLE_MASK)
#define CCM_ANALOG_PLL_VIDEO_SET_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_VIDEO_SET_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_VIDEO_SET_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_SET_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_VIDEO_SET_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_VIDEO_SET_BYPASS_MASK     (0x10000U)
#define CCM_ANALOG_PLL_VIDEO_SET_BYPASS_SHIFT    (16U)
#define CCM_ANALOG_PLL_VIDEO_SET_BYPASS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_SET_BYPASS_SHIFT)) & CCM_ANALOG_PLL_VIDEO_SET_BYPASS_MASK)
#define CCM_ANALOG_PLL_VIDEO_SET_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_VIDEO_SET_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
#define CCM_ANALOG_PLL_VIDEO_SET_POST_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_SET_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_VIDEO_SET_POST_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_VIDEO_SET_LOCK_MASK       (0x80000000U)
#define CCM_ANALOG_PLL_VIDEO_SET_LOCK_SHIFT      (31U)
#define CCM_ANALOG_PLL_VIDEO_SET_LOCK(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_SET_LOCK_SHIFT)) & CCM_ANALOG_PLL_VIDEO_SET_LOCK_MASK)
/*! @} */

/*! @name PLL_VIDEO_CLR - Analog Video PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_VIDEO_CLR_DIV_SELECT_MASK (0x7FU)
#define CCM_ANALOG_PLL_VIDEO_CLR_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_VIDEO_CLR_DIV_SELECT(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_CLR_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_VIDEO_CLR_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_VIDEO_CLR_POWERDOWN_MASK  (0x1000U)
#define CCM_ANALOG_PLL_VIDEO_CLR_POWERDOWN_SHIFT (12U)
#define CCM_ANALOG_PLL_VIDEO_CLR_POWERDOWN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_CLR_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_VIDEO_CLR_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_VIDEO_CLR_ENABLE_MASK     (0x2000U)
#define CCM_ANALOG_PLL_VIDEO_CLR_ENABLE_SHIFT    (13U)
#define CCM_ANALOG_PLL_VIDEO_CLR_ENABLE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_CLR_ENABLE_SHIFT)) & CCM_ANALOG_PLL_VIDEO_CLR_ENABLE_MASK)
#define CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_MASK     (0x10000U)
#define CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_SHIFT    (16U)
#define CCM_ANALOG_PLL_VIDEO_CLR_BYPASS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_SHIFT)) & CCM_ANALOG_PLL_VIDEO_CLR_BYPASS_MASK)
#define CCM_ANALOG_PLL_VIDEO_CLR_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_VIDEO_CLR_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
#define CCM_ANALOG_PLL_VIDEO_CLR_POST_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_CLR_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_VIDEO_CLR_POST_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_VIDEO_CLR_LOCK_MASK       (0x80000000U)
#define CCM_ANALOG_PLL_VIDEO_CLR_LOCK_SHIFT      (31U)
#define CCM_ANALOG_PLL_VIDEO_CLR_LOCK(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_CLR_LOCK_SHIFT)) & CCM_ANALOG_PLL_VIDEO_CLR_LOCK_MASK)
/*! @} */

/*! @name PLL_VIDEO_TOG - Analog Video PLL control Register */
/*! @{ */
#define CCM_ANALOG_PLL_VIDEO_TOG_DIV_SELECT_MASK (0x7FU)
#define CCM_ANALOG_PLL_VIDEO_TOG_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_VIDEO_TOG_DIV_SELECT(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_TOG_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_VIDEO_TOG_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_VIDEO_TOG_POWERDOWN_MASK  (0x1000U)
#define CCM_ANALOG_PLL_VIDEO_TOG_POWERDOWN_SHIFT (12U)
#define CCM_ANALOG_PLL_VIDEO_TOG_POWERDOWN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_TOG_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_VIDEO_TOG_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_VIDEO_TOG_ENABLE_MASK     (0x2000U)
#define CCM_ANALOG_PLL_VIDEO_TOG_ENABLE_SHIFT    (13U)
#define CCM_ANALOG_PLL_VIDEO_TOG_ENABLE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_TOG_ENABLE_SHIFT)) & CCM_ANALOG_PLL_VIDEO_TOG_ENABLE_MASK)
#define CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_MASK     (0x10000U)
#define CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_SHIFT    (16U)
#define CCM_ANALOG_PLL_VIDEO_TOG_BYPASS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_SHIFT)) & CCM_ANALOG_PLL_VIDEO_TOG_BYPASS_MASK)
#define CCM_ANALOG_PLL_VIDEO_TOG_POST_DIV_SELECT_MASK (0x180000U)
#define CCM_ANALOG_PLL_VIDEO_TOG_POST_DIV_SELECT_SHIFT (19U)
/*! POST_DIV_SELECT
 *  0b00..Divide by 4.
 *  0b01..Divide by 2.
 *  0b10..Divide by 1.
 *  0b11..Reserved
 */
#define CCM_ANALOG_PLL_VIDEO_TOG_POST_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_TOG_POST_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_VIDEO_TOG_POST_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_VIDEO_TOG_LOCK_MASK       (0x80000000U)
#define CCM_ANALOG_PLL_VIDEO_TOG_LOCK_SHIFT      (31U)
#define CCM_ANALOG_PLL_VIDEO_TOG_LOCK(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_TOG_LOCK_SHIFT)) & CCM_ANALOG_PLL_VIDEO_TOG_LOCK_MASK)
/*! @} */

/*! @name PLL_VIDEO_NUM - Numerator of Video PLL Fractional Loop Divider Register */
/*! @{ */
#define CCM_ANALOG_PLL_VIDEO_NUM_A_MASK          (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_VIDEO_NUM_A_SHIFT         (0U)
#define CCM_ANALOG_PLL_VIDEO_NUM_A(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_NUM_A_SHIFT)) & CCM_ANALOG_PLL_VIDEO_NUM_A_MASK)
/*! @} */

/*! @name PLL_VIDEO_DENOM - Denominator of Video PLL Fractional Loop Divider Register */
/*! @{ */
#define CCM_ANALOG_PLL_VIDEO_DENOM_B_MASK        (0x3FFFFFFFU)
#define CCM_ANALOG_PLL_VIDEO_DENOM_B_SHIFT       (0U)
#define CCM_ANALOG_PLL_VIDEO_DENOM_B(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_DENOM_B_SHIFT)) & CCM_ANALOG_PLL_VIDEO_DENOM_B_MASK)
/*! @} */

/*! @name PLL_ENET - Analog ENET PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_ENET_DIV_SELECT_MASK      (0x3U)
#define CCM_ANALOG_PLL_ENET_DIV_SELECT_SHIFT     (0U)
#define CCM_ANALOG_PLL_ENET_DIV_SELECT(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ENET_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT_MASK (0xCU)
#define CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT_SHIFT (2U)
/*! ENET2_DIV_SELECT
 *  0b00..25MHz
 *  0b01..50MHz
 *  0b10..100MHz (not 50% duty cycle)
 *  0b11..125MHz
 */
#define CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ENET_ENET2_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ENET_POWERDOWN_MASK       (0x1000U)
#define CCM_ANALOG_PLL_ENET_POWERDOWN_SHIFT      (12U)
#define CCM_ANALOG_PLL_ENET_POWERDOWN(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_ENET_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_ENET_ENABLE_MASK          (0x2000U)
#define CCM_ANALOG_PLL_ENET_ENABLE_SHIFT         (13U)
#define CCM_ANALOG_PLL_ENET_ENABLE(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_ENABLE_SHIFT)) & CCM_ANALOG_PLL_ENET_ENABLE_MASK)
#define CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_MASK  (0xC000U)
#define CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_ENET_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_ENET_BYPASS_MASK          (0x10000U)
#define CCM_ANALOG_PLL_ENET_BYPASS_SHIFT         (16U)
#define CCM_ANALOG_PLL_ENET_BYPASS(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_BYPASS_SHIFT)) & CCM_ANALOG_PLL_ENET_BYPASS_MASK)
#define CCM_ANALOG_PLL_ENET_ENET2_REF_EN_MASK    (0x100000U)
#define CCM_ANALOG_PLL_ENET_ENET2_REF_EN_SHIFT   (20U)
#define CCM_ANALOG_PLL_ENET_ENET2_REF_EN(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_ENET2_REF_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_ENET2_REF_EN_MASK)
#define CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN_MASK (0x200000U)
#define CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN_SHIFT (21U)
#define CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN_MASK)
#define CCM_ANALOG_PLL_ENET_LOCK_MASK            (0x80000000U)
#define CCM_ANALOG_PLL_ENET_LOCK_SHIFT           (31U)
#define CCM_ANALOG_PLL_ENET_LOCK(x)              (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_LOCK_SHIFT)) & CCM_ANALOG_PLL_ENET_LOCK_MASK)
/*! @} */

/*! @name PLL_ENET_SET - Analog ENET PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_ENET_SET_DIV_SELECT_MASK  (0x3U)
#define CCM_ANALOG_PLL_ENET_SET_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_ENET_SET_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ENET_SET_ENET2_DIV_SELECT_MASK (0xCU)
#define CCM_ANALOG_PLL_ENET_SET_ENET2_DIV_SELECT_SHIFT (2U)
/*! ENET2_DIV_SELECT
 *  0b00..25MHz
 *  0b01..50MHz
 *  0b10..100MHz (not 50% duty cycle)
 *  0b11..125MHz
 */
#define CCM_ANALOG_PLL_ENET_SET_ENET2_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_ENET2_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_ENET2_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ENET_SET_POWERDOWN_MASK   (0x1000U)
#define CCM_ANALOG_PLL_ENET_SET_POWERDOWN_SHIFT  (12U)
#define CCM_ANALOG_PLL_ENET_SET_POWERDOWN(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_ENET_SET_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_ENET_SET_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_ENET_SET_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_ENABLE_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_ENABLE_MASK)
#define CCM_ANALOG_PLL_ENET_SET_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_ENET_SET_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_ENET_SET_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_ENET_SET_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_ENET_SET_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_ENET_SET_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_BYPASS_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_BYPASS_MASK)
#define CCM_ANALOG_PLL_ENET_SET_ENET2_REF_EN_MASK (0x100000U)
#define CCM_ANALOG_PLL_ENET_SET_ENET2_REF_EN_SHIFT (20U)
#define CCM_ANALOG_PLL_ENET_SET_ENET2_REF_EN(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_ENET2_REF_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_ENET2_REF_EN_MASK)
#define CCM_ANALOG_PLL_ENET_SET_ENET_25M_REF_EN_MASK (0x200000U)
#define CCM_ANALOG_PLL_ENET_SET_ENET_25M_REF_EN_SHIFT (21U)
#define CCM_ANALOG_PLL_ENET_SET_ENET_25M_REF_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_ENET_25M_REF_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_ENET_25M_REF_EN_MASK)
#define CCM_ANALOG_PLL_ENET_SET_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_ENET_SET_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_ENET_SET_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_LOCK_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_LOCK_MASK)
/*! @} */

/*! @name PLL_ENET_CLR - Analog ENET PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_ENET_CLR_DIV_SELECT_MASK  (0x3U)
#define CCM_ANALOG_PLL_ENET_CLR_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_ENET_CLR_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ENET_CLR_ENET2_DIV_SELECT_MASK (0xCU)
#define CCM_ANALOG_PLL_ENET_CLR_ENET2_DIV_SELECT_SHIFT (2U)
/*! ENET2_DIV_SELECT
 *  0b00..25MHz
 *  0b01..50MHz
 *  0b10..100MHz (not 50% duty cycle)
 *  0b11..125MHz
 */
#define CCM_ANALOG_PLL_ENET_CLR_ENET2_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_ENET2_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_ENET2_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ENET_CLR_POWERDOWN_MASK   (0x1000U)
#define CCM_ANALOG_PLL_ENET_CLR_POWERDOWN_SHIFT  (12U)
#define CCM_ANALOG_PLL_ENET_CLR_POWERDOWN(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_ENET_CLR_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_ENET_CLR_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_ENET_CLR_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_ENABLE_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_ENABLE_MASK)
#define CCM_ANALOG_PLL_ENET_CLR_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_ENET_CLR_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_ENET_CLR_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_ENET_CLR_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_ENET_CLR_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_ENET_CLR_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_BYPASS_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_BYPASS_MASK)
#define CCM_ANALOG_PLL_ENET_CLR_ENET2_REF_EN_MASK (0x100000U)
#define CCM_ANALOG_PLL_ENET_CLR_ENET2_REF_EN_SHIFT (20U)
#define CCM_ANALOG_PLL_ENET_CLR_ENET2_REF_EN(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_ENET2_REF_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_ENET2_REF_EN_MASK)
#define CCM_ANALOG_PLL_ENET_CLR_ENET_25M_REF_EN_MASK (0x200000U)
#define CCM_ANALOG_PLL_ENET_CLR_ENET_25M_REF_EN_SHIFT (21U)
#define CCM_ANALOG_PLL_ENET_CLR_ENET_25M_REF_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_ENET_25M_REF_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_ENET_25M_REF_EN_MASK)
#define CCM_ANALOG_PLL_ENET_CLR_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_ENET_CLR_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_ENET_CLR_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_LOCK_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_LOCK_MASK)
/*! @} */

/*! @name PLL_ENET_TOG - Analog ENET PLL Control Register */
/*! @{ */
#define CCM_ANALOG_PLL_ENET_TOG_DIV_SELECT_MASK  (0x3U)
#define CCM_ANALOG_PLL_ENET_TOG_DIV_SELECT_SHIFT (0U)
#define CCM_ANALOG_PLL_ENET_TOG_DIV_SELECT(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ENET_TOG_ENET2_DIV_SELECT_MASK (0xCU)
#define CCM_ANALOG_PLL_ENET_TOG_ENET2_DIV_SELECT_SHIFT (2U)
/*! ENET2_DIV_SELECT
 *  0b00..25MHz
 *  0b01..50MHz
 *  0b10..100MHz (not 50% duty cycle)
 *  0b11..125MHz
 */
#define CCM_ANALOG_PLL_ENET_TOG_ENET2_DIV_SELECT(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_ENET2_DIV_SELECT_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_ENET2_DIV_SELECT_MASK)
#define CCM_ANALOG_PLL_ENET_TOG_POWERDOWN_MASK   (0x1000U)
#define CCM_ANALOG_PLL_ENET_TOG_POWERDOWN_SHIFT  (12U)
#define CCM_ANALOG_PLL_ENET_TOG_POWERDOWN(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_POWERDOWN_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_POWERDOWN_MASK)
#define CCM_ANALOG_PLL_ENET_TOG_ENABLE_MASK      (0x2000U)
#define CCM_ANALOG_PLL_ENET_TOG_ENABLE_SHIFT     (13U)
#define CCM_ANALOG_PLL_ENET_TOG_ENABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_ENABLE_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_ENABLE_MASK)
#define CCM_ANALOG_PLL_ENET_TOG_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_ENET_TOG_BYPASS_CLK_SRC_SHIFT (14U)
/*! BYPASS_CLK_SRC
 *  0b00..Select the 24MHz oscillator as source.
 *  0b01..Select the CLK1_N / CLK1_P as source.
 *  0b10..Reserved1
 *  0b11..Reserved2
 */
#define CCM_ANALOG_PLL_ENET_TOG_BYPASS_CLK_SRC(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_BYPASS_CLK_SRC_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_BYPASS_CLK_SRC_MASK)
#define CCM_ANALOG_PLL_ENET_TOG_BYPASS_MASK      (0x10000U)
#define CCM_ANALOG_PLL_ENET_TOG_BYPASS_SHIFT     (16U)
#define CCM_ANALOG_PLL_ENET_TOG_BYPASS(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_BYPASS_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_BYPASS_MASK)
#define CCM_ANALOG_PLL_ENET_TOG_ENET2_REF_EN_MASK (0x100000U)
#define CCM_ANALOG_PLL_ENET_TOG_ENET2_REF_EN_SHIFT (20U)
#define CCM_ANALOG_PLL_ENET_TOG_ENET2_REF_EN(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_ENET2_REF_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_ENET2_REF_EN_MASK)
#define CCM_ANALOG_PLL_ENET_TOG_ENET_25M_REF_EN_MASK (0x200000U)
#define CCM_ANALOG_PLL_ENET_TOG_ENET_25M_REF_EN_SHIFT (21U)
#define CCM_ANALOG_PLL_ENET_TOG_ENET_25M_REF_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_ENET_25M_REF_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_ENET_25M_REF_EN_MASK)
#define CCM_ANALOG_PLL_ENET_TOG_LOCK_MASK        (0x80000000U)
#define CCM_ANALOG_PLL_ENET_TOG_LOCK_SHIFT       (31U)
#define CCM_ANALOG_PLL_ENET_TOG_LOCK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_LOCK_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_LOCK_MASK)
/*! @} */

/*! @name PFD_480 - 480MHz Clock (PLL3) Phase Fractional Divider Control Register */
/*! @{ */
#define CCM_ANALOG_PFD_480_PFD0_FRAC_MASK        (0x3FU)
#define CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT       (0U)
#define CCM_ANALOG_PFD_480_PFD0_FRAC(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK)
#define CCM_ANALOG_PFD_480_PFD0_STABLE_MASK      (0x40U)
#define CCM_ANALOG_PFD_480_PFD0_STABLE_SHIFT     (6U)
#define CCM_ANALOG_PFD_480_PFD0_STABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD0_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_PFD0_STABLE_MASK)
#define CCM_ANALOG_PFD_480_PFD0_CLKGATE_MASK     (0x80U)
#define CCM_ANALOG_PFD_480_PFD0_CLKGATE_SHIFT    (7U)
#define CCM_ANALOG_PFD_480_PFD0_CLKGATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD0_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_PFD0_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_PFD1_FRAC_MASK        (0x3F00U)
#define CCM_ANALOG_PFD_480_PFD1_FRAC_SHIFT       (8U)
#define CCM_ANALOG_PFD_480_PFD1_FRAC(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD1_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_PFD1_FRAC_MASK)
#define CCM_ANALOG_PFD_480_PFD1_STABLE_MASK      (0x4000U)
#define CCM_ANALOG_PFD_480_PFD1_STABLE_SHIFT     (14U)
#define CCM_ANALOG_PFD_480_PFD1_STABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD1_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_PFD1_STABLE_MASK)
#define CCM_ANALOG_PFD_480_PFD1_CLKGATE_MASK     (0x8000U)
#define CCM_ANALOG_PFD_480_PFD1_CLKGATE_SHIFT    (15U)
#define CCM_ANALOG_PFD_480_PFD1_CLKGATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD1_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_PFD1_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_PFD2_FRAC_MASK        (0x3F0000U)
#define CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT       (16U)
#define CCM_ANALOG_PFD_480_PFD2_FRAC(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_PFD2_FRAC_MASK)
#define CCM_ANALOG_PFD_480_PFD2_STABLE_MASK      (0x400000U)
#define CCM_ANALOG_PFD_480_PFD2_STABLE_SHIFT     (22U)
#define CCM_ANALOG_PFD_480_PFD2_STABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD2_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_PFD2_STABLE_MASK)
#define CCM_ANALOG_PFD_480_PFD2_CLKGATE_MASK     (0x800000U)
#define CCM_ANALOG_PFD_480_PFD2_CLKGATE_SHIFT    (23U)
#define CCM_ANALOG_PFD_480_PFD2_CLKGATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD2_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_PFD2_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_PFD3_FRAC_MASK        (0x3F000000U)
#define CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT       (24U)
#define CCM_ANALOG_PFD_480_PFD3_FRAC(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_PFD3_FRAC_MASK)
#define CCM_ANALOG_PFD_480_PFD3_STABLE_MASK      (0x40000000U)
#define CCM_ANALOG_PFD_480_PFD3_STABLE_SHIFT     (30U)
#define CCM_ANALOG_PFD_480_PFD3_STABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD3_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_PFD3_STABLE_MASK)
#define CCM_ANALOG_PFD_480_PFD3_CLKGATE_MASK     (0x80000000U)
#define CCM_ANALOG_PFD_480_PFD3_CLKGATE_SHIFT    (31U)
#define CCM_ANALOG_PFD_480_PFD3_CLKGATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_PFD3_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_PFD3_CLKGATE_MASK)
/*! @} */

/*! @name PFD_480_SET - 480MHz Clock (PLL3) Phase Fractional Divider Control Register */
/*! @{ */
#define CCM_ANALOG_PFD_480_SET_PFD0_FRAC_MASK    (0x3FU)
#define CCM_ANALOG_PFD_480_SET_PFD0_FRAC_SHIFT   (0U)
#define CCM_ANALOG_PFD_480_SET_PFD0_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD0_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD0_FRAC_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD0_STABLE_MASK  (0x40U)
#define CCM_ANALOG_PFD_480_SET_PFD0_STABLE_SHIFT (6U)
#define CCM_ANALOG_PFD_480_SET_PFD0_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD0_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD0_STABLE_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD0_CLKGATE_MASK (0x80U)
#define CCM_ANALOG_PFD_480_SET_PFD0_CLKGATE_SHIFT (7U)
#define CCM_ANALOG_PFD_480_SET_PFD0_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD0_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD0_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD1_FRAC_MASK    (0x3F00U)
#define CCM_ANALOG_PFD_480_SET_PFD1_FRAC_SHIFT   (8U)
#define CCM_ANALOG_PFD_480_SET_PFD1_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD1_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD1_FRAC_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD1_STABLE_MASK  (0x4000U)
#define CCM_ANALOG_PFD_480_SET_PFD1_STABLE_SHIFT (14U)
#define CCM_ANALOG_PFD_480_SET_PFD1_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD1_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD1_STABLE_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD1_CLKGATE_MASK (0x8000U)
#define CCM_ANALOG_PFD_480_SET_PFD1_CLKGATE_SHIFT (15U)
#define CCM_ANALOG_PFD_480_SET_PFD1_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD1_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD1_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD2_FRAC_MASK    (0x3F0000U)
#define CCM_ANALOG_PFD_480_SET_PFD2_FRAC_SHIFT   (16U)
#define CCM_ANALOG_PFD_480_SET_PFD2_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD2_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD2_FRAC_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD2_STABLE_MASK  (0x400000U)
#define CCM_ANALOG_PFD_480_SET_PFD2_STABLE_SHIFT (22U)
#define CCM_ANALOG_PFD_480_SET_PFD2_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD2_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD2_STABLE_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD2_CLKGATE_MASK (0x800000U)
#define CCM_ANALOG_PFD_480_SET_PFD2_CLKGATE_SHIFT (23U)
#define CCM_ANALOG_PFD_480_SET_PFD2_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD2_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD2_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD3_FRAC_MASK    (0x3F000000U)
#define CCM_ANALOG_PFD_480_SET_PFD3_FRAC_SHIFT   (24U)
#define CCM_ANALOG_PFD_480_SET_PFD3_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD3_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD3_FRAC_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD3_STABLE_MASK  (0x40000000U)
#define CCM_ANALOG_PFD_480_SET_PFD3_STABLE_SHIFT (30U)
#define CCM_ANALOG_PFD_480_SET_PFD3_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD3_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD3_STABLE_MASK)
#define CCM_ANALOG_PFD_480_SET_PFD3_CLKGATE_MASK (0x80000000U)
#define CCM_ANALOG_PFD_480_SET_PFD3_CLKGATE_SHIFT (31U)
#define CCM_ANALOG_PFD_480_SET_PFD3_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_SET_PFD3_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_SET_PFD3_CLKGATE_MASK)
/*! @} */

/*! @name PFD_480_CLR - 480MHz Clock (PLL3) Phase Fractional Divider Control Register */
/*! @{ */
#define CCM_ANALOG_PFD_480_CLR_PFD0_FRAC_MASK    (0x3FU)
#define CCM_ANALOG_PFD_480_CLR_PFD0_FRAC_SHIFT   (0U)
#define CCM_ANALOG_PFD_480_CLR_PFD0_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD0_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD0_FRAC_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD0_STABLE_MASK  (0x40U)
#define CCM_ANALOG_PFD_480_CLR_PFD0_STABLE_SHIFT (6U)
#define CCM_ANALOG_PFD_480_CLR_PFD0_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD0_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD0_STABLE_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD0_CLKGATE_MASK (0x80U)
#define CCM_ANALOG_PFD_480_CLR_PFD0_CLKGATE_SHIFT (7U)
#define CCM_ANALOG_PFD_480_CLR_PFD0_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD0_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD0_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD1_FRAC_MASK    (0x3F00U)
#define CCM_ANALOG_PFD_480_CLR_PFD1_FRAC_SHIFT   (8U)
#define CCM_ANALOG_PFD_480_CLR_PFD1_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD1_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD1_FRAC_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD1_STABLE_MASK  (0x4000U)
#define CCM_ANALOG_PFD_480_CLR_PFD1_STABLE_SHIFT (14U)
#define CCM_ANALOG_PFD_480_CLR_PFD1_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD1_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD1_STABLE_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD1_CLKGATE_MASK (0x8000U)
#define CCM_ANALOG_PFD_480_CLR_PFD1_CLKGATE_SHIFT (15U)
#define CCM_ANALOG_PFD_480_CLR_PFD1_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD1_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD1_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD2_FRAC_MASK    (0x3F0000U)
#define CCM_ANALOG_PFD_480_CLR_PFD2_FRAC_SHIFT   (16U)
#define CCM_ANALOG_PFD_480_CLR_PFD2_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD2_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD2_FRAC_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD2_STABLE_MASK  (0x400000U)
#define CCM_ANALOG_PFD_480_CLR_PFD2_STABLE_SHIFT (22U)
#define CCM_ANALOG_PFD_480_CLR_PFD2_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD2_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD2_STABLE_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD2_CLKGATE_MASK (0x800000U)
#define CCM_ANALOG_PFD_480_CLR_PFD2_CLKGATE_SHIFT (23U)
#define CCM_ANALOG_PFD_480_CLR_PFD2_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD2_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD2_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD3_FRAC_MASK    (0x3F000000U)
#define CCM_ANALOG_PFD_480_CLR_PFD3_FRAC_SHIFT   (24U)
#define CCM_ANALOG_PFD_480_CLR_PFD3_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD3_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD3_FRAC_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD3_STABLE_MASK  (0x40000000U)
#define CCM_ANALOG_PFD_480_CLR_PFD3_STABLE_SHIFT (30U)
#define CCM_ANALOG_PFD_480_CLR_PFD3_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD3_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD3_STABLE_MASK)
#define CCM_ANALOG_PFD_480_CLR_PFD3_CLKGATE_MASK (0x80000000U)
#define CCM_ANALOG_PFD_480_CLR_PFD3_CLKGATE_SHIFT (31U)
#define CCM_ANALOG_PFD_480_CLR_PFD3_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_CLR_PFD3_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_CLR_PFD3_CLKGATE_MASK)
/*! @} */

/*! @name PFD_480_TOG - 480MHz Clock (PLL3) Phase Fractional Divider Control Register */
/*! @{ */
#define CCM_ANALOG_PFD_480_TOG_PFD0_FRAC_MASK    (0x3FU)
#define CCM_ANALOG_PFD_480_TOG_PFD0_FRAC_SHIFT   (0U)
#define CCM_ANALOG_PFD_480_TOG_PFD0_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD0_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD0_FRAC_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD0_STABLE_MASK  (0x40U)
#define CCM_ANALOG_PFD_480_TOG_PFD0_STABLE_SHIFT (6U)
#define CCM_ANALOG_PFD_480_TOG_PFD0_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD0_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD0_STABLE_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD0_CLKGATE_MASK (0x80U)
#define CCM_ANALOG_PFD_480_TOG_PFD0_CLKGATE_SHIFT (7U)
#define CCM_ANALOG_PFD_480_TOG_PFD0_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD0_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD0_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD1_FRAC_MASK    (0x3F00U)
#define CCM_ANALOG_PFD_480_TOG_PFD1_FRAC_SHIFT   (8U)
#define CCM_ANALOG_PFD_480_TOG_PFD1_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD1_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD1_FRAC_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD1_STABLE_MASK  (0x4000U)
#define CCM_ANALOG_PFD_480_TOG_PFD1_STABLE_SHIFT (14U)
#define CCM_ANALOG_PFD_480_TOG_PFD1_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD1_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD1_STABLE_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD1_CLKGATE_MASK (0x8000U)
#define CCM_ANALOG_PFD_480_TOG_PFD1_CLKGATE_SHIFT (15U)
#define CCM_ANALOG_PFD_480_TOG_PFD1_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD1_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD1_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD2_FRAC_MASK    (0x3F0000U)
#define CCM_ANALOG_PFD_480_TOG_PFD2_FRAC_SHIFT   (16U)
#define CCM_ANALOG_PFD_480_TOG_PFD2_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD2_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD2_FRAC_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD2_STABLE_MASK  (0x400000U)
#define CCM_ANALOG_PFD_480_TOG_PFD2_STABLE_SHIFT (22U)
#define CCM_ANALOG_PFD_480_TOG_PFD2_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD2_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD2_STABLE_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD2_CLKGATE_MASK (0x800000U)
#define CCM_ANALOG_PFD_480_TOG_PFD2_CLKGATE_SHIFT (23U)
#define CCM_ANALOG_PFD_480_TOG_PFD2_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD2_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD2_CLKGATE_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD3_FRAC_MASK    (0x3F000000U)
#define CCM_ANALOG_PFD_480_TOG_PFD3_FRAC_SHIFT   (24U)
#define CCM_ANALOG_PFD_480_TOG_PFD3_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD3_FRAC_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD3_FRAC_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD3_STABLE_MASK  (0x40000000U)
#define CCM_ANALOG_PFD_480_TOG_PFD3_STABLE_SHIFT (30U)
#define CCM_ANALOG_PFD_480_TOG_PFD3_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD3_STABLE_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD3_STABLE_MASK)
#define CCM_ANALOG_PFD_480_TOG_PFD3_CLKGATE_MASK (0x80000000U)
#define CCM_ANALOG_PFD_480_TOG_PFD3_CLKGATE_SHIFT (31U)
#define CCM_ANALOG_PFD_480_TOG_PFD3_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_480_TOG_PFD3_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_480_TOG_PFD3_CLKGATE_MASK)
/*! @} */

/*! @name PFD_528 - 528MHz Clock (PLL2) Phase Fractional Divider Control Register */
/*! @{ */
#define CCM_ANALOG_PFD_528_PFD0_FRAC_MASK        (0x3FU)
#define CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT       (0U)
#define CCM_ANALOG_PFD_528_PFD0_FRAC(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_PFD0_FRAC_MASK)
#define CCM_ANALOG_PFD_528_PFD0_STABLE_MASK      (0x40U)
#define CCM_ANALOG_PFD_528_PFD0_STABLE_SHIFT     (6U)
#define CCM_ANALOG_PFD_528_PFD0_STABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD0_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_PFD0_STABLE_MASK)
#define CCM_ANALOG_PFD_528_PFD0_CLKGATE_MASK     (0x80U)
#define CCM_ANALOG_PFD_528_PFD0_CLKGATE_SHIFT    (7U)
#define CCM_ANALOG_PFD_528_PFD0_CLKGATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD0_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_PFD0_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_PFD1_FRAC_MASK        (0x3F00U)
#define CCM_ANALOG_PFD_528_PFD1_FRAC_SHIFT       (8U)
#define CCM_ANALOG_PFD_528_PFD1_FRAC(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD1_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_PFD1_FRAC_MASK)
#define CCM_ANALOG_PFD_528_PFD1_STABLE_MASK      (0x4000U)
#define CCM_ANALOG_PFD_528_PFD1_STABLE_SHIFT     (14U)
#define CCM_ANALOG_PFD_528_PFD1_STABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD1_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_PFD1_STABLE_MASK)
#define CCM_ANALOG_PFD_528_PFD1_CLKGATE_MASK     (0x8000U)
#define CCM_ANALOG_PFD_528_PFD1_CLKGATE_SHIFT    (15U)
#define CCM_ANALOG_PFD_528_PFD1_CLKGATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD1_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_PFD1_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_PFD2_FRAC_MASK        (0x3F0000U)
#define CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT       (16U)
#define CCM_ANALOG_PFD_528_PFD2_FRAC(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_PFD2_FRAC_MASK)
#define CCM_ANALOG_PFD_528_PFD2_STABLE_MASK      (0x400000U)
#define CCM_ANALOG_PFD_528_PFD2_STABLE_SHIFT     (22U)
#define CCM_ANALOG_PFD_528_PFD2_STABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD2_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_PFD2_STABLE_MASK)
#define CCM_ANALOG_PFD_528_PFD2_CLKGATE_MASK     (0x800000U)
#define CCM_ANALOG_PFD_528_PFD2_CLKGATE_SHIFT    (23U)
#define CCM_ANALOG_PFD_528_PFD2_CLKGATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD2_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_PFD2_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_PFD3_FRAC_MASK        (0x3F000000U)
#define CCM_ANALOG_PFD_528_PFD3_FRAC_SHIFT       (24U)
#define CCM_ANALOG_PFD_528_PFD3_FRAC(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD3_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_PFD3_FRAC_MASK)
#define CCM_ANALOG_PFD_528_PFD3_STABLE_MASK      (0x40000000U)
#define CCM_ANALOG_PFD_528_PFD3_STABLE_SHIFT     (30U)
#define CCM_ANALOG_PFD_528_PFD3_STABLE(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD3_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_PFD3_STABLE_MASK)
#define CCM_ANALOG_PFD_528_PFD3_CLKGATE_MASK     (0x80000000U)
#define CCM_ANALOG_PFD_528_PFD3_CLKGATE_SHIFT    (31U)
#define CCM_ANALOG_PFD_528_PFD3_CLKGATE(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_PFD3_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_PFD3_CLKGATE_MASK)
/*! @} */

/*! @name PFD_528_SET - 528MHz Clock (PLL2) Phase Fractional Divider Control Register */
/*! @{ */
#define CCM_ANALOG_PFD_528_SET_PFD0_FRAC_MASK    (0x3FU)
#define CCM_ANALOG_PFD_528_SET_PFD0_FRAC_SHIFT   (0U)
#define CCM_ANALOG_PFD_528_SET_PFD0_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD0_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD0_FRAC_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD0_STABLE_MASK  (0x40U)
#define CCM_ANALOG_PFD_528_SET_PFD0_STABLE_SHIFT (6U)
#define CCM_ANALOG_PFD_528_SET_PFD0_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD0_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD0_STABLE_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD0_CLKGATE_MASK (0x80U)
#define CCM_ANALOG_PFD_528_SET_PFD0_CLKGATE_SHIFT (7U)
#define CCM_ANALOG_PFD_528_SET_PFD0_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD0_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD0_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD1_FRAC_MASK    (0x3F00U)
#define CCM_ANALOG_PFD_528_SET_PFD1_FRAC_SHIFT   (8U)
#define CCM_ANALOG_PFD_528_SET_PFD1_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD1_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD1_FRAC_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD1_STABLE_MASK  (0x4000U)
#define CCM_ANALOG_PFD_528_SET_PFD1_STABLE_SHIFT (14U)
#define CCM_ANALOG_PFD_528_SET_PFD1_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD1_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD1_STABLE_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD1_CLKGATE_MASK (0x8000U)
#define CCM_ANALOG_PFD_528_SET_PFD1_CLKGATE_SHIFT (15U)
#define CCM_ANALOG_PFD_528_SET_PFD1_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD1_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD1_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD2_FRAC_MASK    (0x3F0000U)
#define CCM_ANALOG_PFD_528_SET_PFD2_FRAC_SHIFT   (16U)
#define CCM_ANALOG_PFD_528_SET_PFD2_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD2_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD2_FRAC_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD2_STABLE_MASK  (0x400000U)
#define CCM_ANALOG_PFD_528_SET_PFD2_STABLE_SHIFT (22U)
#define CCM_ANALOG_PFD_528_SET_PFD2_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD2_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD2_STABLE_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD2_CLKGATE_MASK (0x800000U)
#define CCM_ANALOG_PFD_528_SET_PFD2_CLKGATE_SHIFT (23U)
#define CCM_ANALOG_PFD_528_SET_PFD2_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD2_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD2_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD3_FRAC_MASK    (0x3F000000U)
#define CCM_ANALOG_PFD_528_SET_PFD3_FRAC_SHIFT   (24U)
#define CCM_ANALOG_PFD_528_SET_PFD3_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD3_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD3_FRAC_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD3_STABLE_MASK  (0x40000000U)
#define CCM_ANALOG_PFD_528_SET_PFD3_STABLE_SHIFT (30U)
#define CCM_ANALOG_PFD_528_SET_PFD3_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD3_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD3_STABLE_MASK)
#define CCM_ANALOG_PFD_528_SET_PFD3_CLKGATE_MASK (0x80000000U)
#define CCM_ANALOG_PFD_528_SET_PFD3_CLKGATE_SHIFT (31U)
#define CCM_ANALOG_PFD_528_SET_PFD3_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_SET_PFD3_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_SET_PFD3_CLKGATE_MASK)
/*! @} */

/*! @name PFD_528_CLR - 528MHz Clock (PLL2) Phase Fractional Divider Control Register */
/*! @{ */
#define CCM_ANALOG_PFD_528_CLR_PFD0_FRAC_MASK    (0x3FU)
#define CCM_ANALOG_PFD_528_CLR_PFD0_FRAC_SHIFT   (0U)
#define CCM_ANALOG_PFD_528_CLR_PFD0_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD0_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD0_FRAC_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD0_STABLE_MASK  (0x40U)
#define CCM_ANALOG_PFD_528_CLR_PFD0_STABLE_SHIFT (6U)
#define CCM_ANALOG_PFD_528_CLR_PFD0_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD0_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD0_STABLE_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD0_CLKGATE_MASK (0x80U)
#define CCM_ANALOG_PFD_528_CLR_PFD0_CLKGATE_SHIFT (7U)
#define CCM_ANALOG_PFD_528_CLR_PFD0_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD0_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD0_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD1_FRAC_MASK    (0x3F00U)
#define CCM_ANALOG_PFD_528_CLR_PFD1_FRAC_SHIFT   (8U)
#define CCM_ANALOG_PFD_528_CLR_PFD1_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD1_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD1_FRAC_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD1_STABLE_MASK  (0x4000U)
#define CCM_ANALOG_PFD_528_CLR_PFD1_STABLE_SHIFT (14U)
#define CCM_ANALOG_PFD_528_CLR_PFD1_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD1_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD1_STABLE_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD1_CLKGATE_MASK (0x8000U)
#define CCM_ANALOG_PFD_528_CLR_PFD1_CLKGATE_SHIFT (15U)
#define CCM_ANALOG_PFD_528_CLR_PFD1_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD1_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD1_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD2_FRAC_MASK    (0x3F0000U)
#define CCM_ANALOG_PFD_528_CLR_PFD2_FRAC_SHIFT   (16U)
#define CCM_ANALOG_PFD_528_CLR_PFD2_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD2_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD2_FRAC_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD2_STABLE_MASK  (0x400000U)
#define CCM_ANALOG_PFD_528_CLR_PFD2_STABLE_SHIFT (22U)
#define CCM_ANALOG_PFD_528_CLR_PFD2_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD2_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD2_STABLE_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD2_CLKGATE_MASK (0x800000U)
#define CCM_ANALOG_PFD_528_CLR_PFD2_CLKGATE_SHIFT (23U)
#define CCM_ANALOG_PFD_528_CLR_PFD2_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD2_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD2_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD3_FRAC_MASK    (0x3F000000U)
#define CCM_ANALOG_PFD_528_CLR_PFD3_FRAC_SHIFT   (24U)
#define CCM_ANALOG_PFD_528_CLR_PFD3_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD3_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD3_FRAC_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD3_STABLE_MASK  (0x40000000U)
#define CCM_ANALOG_PFD_528_CLR_PFD3_STABLE_SHIFT (30U)
#define CCM_ANALOG_PFD_528_CLR_PFD3_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD3_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD3_STABLE_MASK)
#define CCM_ANALOG_PFD_528_CLR_PFD3_CLKGATE_MASK (0x80000000U)
#define CCM_ANALOG_PFD_528_CLR_PFD3_CLKGATE_SHIFT (31U)
#define CCM_ANALOG_PFD_528_CLR_PFD3_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_CLR_PFD3_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_CLR_PFD3_CLKGATE_MASK)
/*! @} */

/*! @name PFD_528_TOG - 528MHz Clock (PLL2) Phase Fractional Divider Control Register */
/*! @{ */
#define CCM_ANALOG_PFD_528_TOG_PFD0_FRAC_MASK    (0x3FU)
#define CCM_ANALOG_PFD_528_TOG_PFD0_FRAC_SHIFT   (0U)
#define CCM_ANALOG_PFD_528_TOG_PFD0_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD0_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD0_FRAC_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD0_STABLE_MASK  (0x40U)
#define CCM_ANALOG_PFD_528_TOG_PFD0_STABLE_SHIFT (6U)
#define CCM_ANALOG_PFD_528_TOG_PFD0_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD0_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD0_STABLE_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD0_CLKGATE_MASK (0x80U)
#define CCM_ANALOG_PFD_528_TOG_PFD0_CLKGATE_SHIFT (7U)
#define CCM_ANALOG_PFD_528_TOG_PFD0_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD0_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD0_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD1_FRAC_MASK    (0x3F00U)
#define CCM_ANALOG_PFD_528_TOG_PFD1_FRAC_SHIFT   (8U)
#define CCM_ANALOG_PFD_528_TOG_PFD1_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD1_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD1_FRAC_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD1_STABLE_MASK  (0x4000U)
#define CCM_ANALOG_PFD_528_TOG_PFD1_STABLE_SHIFT (14U)
#define CCM_ANALOG_PFD_528_TOG_PFD1_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD1_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD1_STABLE_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD1_CLKGATE_MASK (0x8000U)
#define CCM_ANALOG_PFD_528_TOG_PFD1_CLKGATE_SHIFT (15U)
#define CCM_ANALOG_PFD_528_TOG_PFD1_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD1_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD1_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD2_FRAC_MASK    (0x3F0000U)
#define CCM_ANALOG_PFD_528_TOG_PFD2_FRAC_SHIFT   (16U)
#define CCM_ANALOG_PFD_528_TOG_PFD2_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD2_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD2_FRAC_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD2_STABLE_MASK  (0x400000U)
#define CCM_ANALOG_PFD_528_TOG_PFD2_STABLE_SHIFT (22U)
#define CCM_ANALOG_PFD_528_TOG_PFD2_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD2_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD2_STABLE_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD2_CLKGATE_MASK (0x800000U)
#define CCM_ANALOG_PFD_528_TOG_PFD2_CLKGATE_SHIFT (23U)
#define CCM_ANALOG_PFD_528_TOG_PFD2_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD2_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD2_CLKGATE_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD3_FRAC_MASK    (0x3F000000U)
#define CCM_ANALOG_PFD_528_TOG_PFD3_FRAC_SHIFT   (24U)
#define CCM_ANALOG_PFD_528_TOG_PFD3_FRAC(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD3_FRAC_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD3_FRAC_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD3_STABLE_MASK  (0x40000000U)
#define CCM_ANALOG_PFD_528_TOG_PFD3_STABLE_SHIFT (30U)
#define CCM_ANALOG_PFD_528_TOG_PFD3_STABLE(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD3_STABLE_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD3_STABLE_MASK)
#define CCM_ANALOG_PFD_528_TOG_PFD3_CLKGATE_MASK (0x80000000U)
#define CCM_ANALOG_PFD_528_TOG_PFD3_CLKGATE_SHIFT (31U)
#define CCM_ANALOG_PFD_528_TOG_PFD3_CLKGATE(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PFD_528_TOG_PFD3_CLKGATE_SHIFT)) & CCM_ANALOG_PFD_528_TOG_PFD3_CLKGATE_MASK)
/*! @} */

/*! @name MISC0 - Miscellaneous Register 0 */
/*! @{ */
#define CCM_ANALOG_MISC0_REFTOP_PWD_MASK         (0x1U)
#define CCM_ANALOG_MISC0_REFTOP_PWD_SHIFT        (0U)
#define CCM_ANALOG_MISC0_REFTOP_PWD(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_REFTOP_PWD_SHIFT)) & CCM_ANALOG_MISC0_REFTOP_PWD_MASK)
#define CCM_ANALOG_MISC0_REFTOP_SELFBIASOFF_MASK (0x8U)
#define CCM_ANALOG_MISC0_REFTOP_SELFBIASOFF_SHIFT (3U)
/*! REFTOP_SELFBIASOFF
 *  0b0..Uses coarse bias currents for startup
 *  0b1..Uses bandgap-based bias currents for best performance.
 */
#define CCM_ANALOG_MISC0_REFTOP_SELFBIASOFF(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_REFTOP_SELFBIASOFF_SHIFT)) & CCM_ANALOG_MISC0_REFTOP_SELFBIASOFF_MASK)
#define CCM_ANALOG_MISC0_REFTOP_VBGADJ_MASK      (0x70U)
#define CCM_ANALOG_MISC0_REFTOP_VBGADJ_SHIFT     (4U)
/*! REFTOP_VBGADJ
 *  0b000..Nominal VBG
 *  0b001..VBG+0.78%
 *  0b010..VBG+1.56%
 *  0b011..VBG+2.34%
 *  0b100..VBG-0.78%
 *  0b101..VBG-1.56%
 *  0b110..VBG-2.34%
 *  0b111..VBG-3.12%
 */
#define CCM_ANALOG_MISC0_REFTOP_VBGADJ(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_REFTOP_VBGADJ_SHIFT)) & CCM_ANALOG_MISC0_REFTOP_VBGADJ_MASK)
#define CCM_ANALOG_MISC0_REFTOP_VBGUP_MASK       (0x80U)
#define CCM_ANALOG_MISC0_REFTOP_VBGUP_SHIFT      (7U)
#define CCM_ANALOG_MISC0_REFTOP_VBGUP(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_REFTOP_VBGUP_SHIFT)) & CCM_ANALOG_MISC0_REFTOP_VBGUP_MASK)
#define CCM_ANALOG_MISC0_STOP_MODE_CONFIG_MASK   (0xC00U)
#define CCM_ANALOG_MISC0_STOP_MODE_CONFIG_SHIFT  (10U)
/*! STOP_MODE_CONFIG
 *  0b00..All analog except RTC powered down on stop mode assertion.
 *  0b01..Beside RTC, analog bandgap, 1p1 and 2p5 regulators are also on.
 *  0b10..Beside RTC, 1p1 and 2p5 regulators are also on, low-power bandgap is selected so that the normal analog
 *        bandgap together with the rest analog is powered down.
 *  0b11..Beside RTC, low-power bandgap is selected and the rest analog is powered down.
 */
#define CCM_ANALOG_MISC0_STOP_MODE_CONFIG(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_STOP_MODE_CONFIG_SHIFT)) & CCM_ANALOG_MISC0_STOP_MODE_CONFIG_MASK)
#define CCM_ANALOG_MISC0_DISCON_HIGH_SNVS_MASK   (0x1000U)
#define CCM_ANALOG_MISC0_DISCON_HIGH_SNVS_SHIFT  (12U)
/*! DISCON_HIGH_SNVS
 *  0b0..Turn on the switch
 *  0b1..Turn off the switch
 */
#define CCM_ANALOG_MISC0_DISCON_HIGH_SNVS(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_DISCON_HIGH_SNVS_SHIFT)) & CCM_ANALOG_MISC0_DISCON_HIGH_SNVS_MASK)
#define CCM_ANALOG_MISC0_OSC_I_MASK              (0x6000U)
#define CCM_ANALOG_MISC0_OSC_I_SHIFT             (13U)
/*! OSC_I
 *  0b00..Nominal
 *  0b01..Decrease current by 12.5%
 *  0b10..Decrease current by 25.0%
 *  0b11..Decrease current by 37.5%
 */
#define CCM_ANALOG_MISC0_OSC_I(x)                (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_OSC_I_SHIFT)) & CCM_ANALOG_MISC0_OSC_I_MASK)
#define CCM_ANALOG_MISC0_OSC_XTALOK_MASK         (0x8000U)
#define CCM_ANALOG_MISC0_OSC_XTALOK_SHIFT        (15U)
#define CCM_ANALOG_MISC0_OSC_XTALOK(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_OSC_XTALOK_SHIFT)) & CCM_ANALOG_MISC0_OSC_XTALOK_MASK)
#define CCM_ANALOG_MISC0_OSC_XTALOK_EN_MASK      (0x10000U)
#define CCM_ANALOG_MISC0_OSC_XTALOK_EN_SHIFT     (16U)
#define CCM_ANALOG_MISC0_OSC_XTALOK_EN(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_OSC_XTALOK_EN_SHIFT)) & CCM_ANALOG_MISC0_OSC_XTALOK_EN_MASK)
#define CCM_ANALOG_MISC0_CLKGATE_CTRL_MASK       (0x2000000U)
#define CCM_ANALOG_MISC0_CLKGATE_CTRL_SHIFT      (25U)
/*! CLKGATE_CTRL
 *  0b0..Allow the logic to automatically gate the clock when the XTAL is powered down.
 *  0b1..Prevent the logic from ever gating off the clock.
 */
#define CCM_ANALOG_MISC0_CLKGATE_CTRL(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLKGATE_CTRL_SHIFT)) & CCM_ANALOG_MISC0_CLKGATE_CTRL_MASK)
#define CCM_ANALOG_MISC0_CLKGATE_DELAY_MASK      (0x1C000000U)
#define CCM_ANALOG_MISC0_CLKGATE_DELAY_SHIFT     (26U)
/*! CLKGATE_DELAY
 *  0b000..0.5ms
 *  0b001..1.0ms
 *  0b010..2.0ms
 *  0b011..3.0ms
 *  0b100..4.0ms
 *  0b101..5.0ms
 *  0b110..6.0ms
 *  0b111..7.0ms
 */
#define CCM_ANALOG_MISC0_CLKGATE_DELAY(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLKGATE_DELAY_SHIFT)) & CCM_ANALOG_MISC0_CLKGATE_DELAY_MASK)
#define CCM_ANALOG_MISC0_RTC_XTAL_SOURCE_MASK    (0x20000000U)
#define CCM_ANALOG_MISC0_RTC_XTAL_SOURCE_SHIFT   (29U)
/*! RTC_XTAL_SOURCE
 *  0b0..Internal ring oscillator
 *  0b1..RTC_XTAL
 */
#define CCM_ANALOG_MISC0_RTC_XTAL_SOURCE(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_RTC_XTAL_SOURCE_SHIFT)) & CCM_ANALOG_MISC0_RTC_XTAL_SOURCE_MASK)
#define CCM_ANALOG_MISC0_XTAL_24M_PWD_MASK       (0x40000000U)
#define CCM_ANALOG_MISC0_XTAL_24M_PWD_SHIFT      (30U)
#define CCM_ANALOG_MISC0_XTAL_24M_PWD(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_XTAL_24M_PWD_SHIFT)) & CCM_ANALOG_MISC0_XTAL_24M_PWD_MASK)
/*! @} */

/*! @name MISC0_SET - Miscellaneous Register 0 */
/*! @{ */
#define CCM_ANALOG_MISC0_SET_REFTOP_PWD_MASK     (0x1U)
#define CCM_ANALOG_MISC0_SET_REFTOP_PWD_SHIFT    (0U)
#define CCM_ANALOG_MISC0_SET_REFTOP_PWD(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_REFTOP_PWD_SHIFT)) & CCM_ANALOG_MISC0_SET_REFTOP_PWD_MASK)
#define CCM_ANALOG_MISC0_SET_REFTOP_SELFBIASOFF_MASK (0x8U)
#define CCM_ANALOG_MISC0_SET_REFTOP_SELFBIASOFF_SHIFT (3U)
/*! REFTOP_SELFBIASOFF
 *  0b0..Uses coarse bias currents for startup
 *  0b1..Uses bandgap-based bias currents for best performance.
 */
#define CCM_ANALOG_MISC0_SET_REFTOP_SELFBIASOFF(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_REFTOP_SELFBIASOFF_SHIFT)) & CCM_ANALOG_MISC0_SET_REFTOP_SELFBIASOFF_MASK)
#define CCM_ANALOG_MISC0_SET_REFTOP_VBGADJ_MASK  (0x70U)
#define CCM_ANALOG_MISC0_SET_REFTOP_VBGADJ_SHIFT (4U)
/*! REFTOP_VBGADJ
 *  0b000..Nominal VBG
 *  0b001..VBG+0.78%
 *  0b010..VBG+1.56%
 *  0b011..VBG+2.34%
 *  0b100..VBG-0.78%
 *  0b101..VBG-1.56%
 *  0b110..VBG-2.34%
 *  0b111..VBG-3.12%
 */
#define CCM_ANALOG_MISC0_SET_REFTOP_VBGADJ(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_REFTOP_VBGADJ_SHIFT)) & CCM_ANALOG_MISC0_SET_REFTOP_VBGADJ_MASK)
#define CCM_ANALOG_MISC0_SET_REFTOP_VBGUP_MASK   (0x80U)
#define CCM_ANALOG_MISC0_SET_REFTOP_VBGUP_SHIFT  (7U)
#define CCM_ANALOG_MISC0_SET_REFTOP_VBGUP(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_REFTOP_VBGUP_SHIFT)) & CCM_ANALOG_MISC0_SET_REFTOP_VBGUP_MASK)
#define CCM_ANALOG_MISC0_SET_STOP_MODE_CONFIG_MASK (0xC00U)
#define CCM_ANALOG_MISC0_SET_STOP_MODE_CONFIG_SHIFT (10U)
/*! STOP_MODE_CONFIG
 *  0b00..All analog except RTC powered down on stop mode assertion.
 *  0b01..Beside RTC, analog bandgap, 1p1 and 2p5 regulators are also on.
 *  0b10..Beside RTC, 1p1 and 2p5 regulators are also on, low-power bandgap is selected so that the normal analog
 *        bandgap together with the rest analog is powered down.
 *  0b11..Beside RTC, low-power bandgap is selected and the rest analog is powered down.
 */
#define CCM_ANALOG_MISC0_SET_STOP_MODE_CONFIG(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_STOP_MODE_CONFIG_SHIFT)) & CCM_ANALOG_MISC0_SET_STOP_MODE_CONFIG_MASK)
#define CCM_ANALOG_MISC0_SET_DISCON_HIGH_SNVS_MASK (0x1000U)
#define CCM_ANALOG_MISC0_SET_DISCON_HIGH_SNVS_SHIFT (12U)
/*! DISCON_HIGH_SNVS
 *  0b0..Turn on the switch
 *  0b1..Turn off the switch
 */
#define CCM_ANALOG_MISC0_SET_DISCON_HIGH_SNVS(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_DISCON_HIGH_SNVS_SHIFT)) & CCM_ANALOG_MISC0_SET_DISCON_HIGH_SNVS_MASK)
#define CCM_ANALOG_MISC0_SET_OSC_I_MASK          (0x6000U)
#define CCM_ANALOG_MISC0_SET_OSC_I_SHIFT         (13U)
/*! OSC_I
 *  0b00..Nominal
 *  0b01..Decrease current by 12.5%
 *  0b10..Decrease current by 25.0%
 *  0b11..Decrease current by 37.5%
 */
#define CCM_ANALOG_MISC0_SET_OSC_I(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_OSC_I_SHIFT)) & CCM_ANALOG_MISC0_SET_OSC_I_MASK)
#define CCM_ANALOG_MISC0_SET_OSC_XTALOK_MASK     (0x8000U)
#define CCM_ANALOG_MISC0_SET_OSC_XTALOK_SHIFT    (15U)
#define CCM_ANALOG_MISC0_SET_OSC_XTALOK(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_OSC_XTALOK_SHIFT)) & CCM_ANALOG_MISC0_SET_OSC_XTALOK_MASK)
#define CCM_ANALOG_MISC0_SET_OSC_XTALOK_EN_MASK  (0x10000U)
#define CCM_ANALOG_MISC0_SET_OSC_XTALOK_EN_SHIFT (16U)
#define CCM_ANALOG_MISC0_SET_OSC_XTALOK_EN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_OSC_XTALOK_EN_SHIFT)) & CCM_ANALOG_MISC0_SET_OSC_XTALOK_EN_MASK)
#define CCM_ANALOG_MISC0_SET_CLKGATE_CTRL_MASK   (0x2000000U)
#define CCM_ANALOG_MISC0_SET_CLKGATE_CTRL_SHIFT  (25U)
/*! CLKGATE_CTRL
 *  0b0..Allow the logic to automatically gate the clock when the XTAL is powered down.
 *  0b1..Prevent the logic from ever gating off the clock.
 */
#define CCM_ANALOG_MISC0_SET_CLKGATE_CTRL(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_CLKGATE_CTRL_SHIFT)) & CCM_ANALOG_MISC0_SET_CLKGATE_CTRL_MASK)
#define CCM_ANALOG_MISC0_SET_CLKGATE_DELAY_MASK  (0x1C000000U)
#define CCM_ANALOG_MISC0_SET_CLKGATE_DELAY_SHIFT (26U)
/*! CLKGATE_DELAY
 *  0b000..0.5ms
 *  0b001..1.0ms
 *  0b010..2.0ms
 *  0b011..3.0ms
 *  0b100..4.0ms
 *  0b101..5.0ms
 *  0b110..6.0ms
 *  0b111..7.0ms
 */
#define CCM_ANALOG_MISC0_SET_CLKGATE_DELAY(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_CLKGATE_DELAY_SHIFT)) & CCM_ANALOG_MISC0_SET_CLKGATE_DELAY_MASK)
#define CCM_ANALOG_MISC0_SET_RTC_XTAL_SOURCE_MASK (0x20000000U)
#define CCM_ANALOG_MISC0_SET_RTC_XTAL_SOURCE_SHIFT (29U)
/*! RTC_XTAL_SOURCE
 *  0b0..Internal ring oscillator
 *  0b1..RTC_XTAL
 */
#define CCM_ANALOG_MISC0_SET_RTC_XTAL_SOURCE(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_RTC_XTAL_SOURCE_SHIFT)) & CCM_ANALOG_MISC0_SET_RTC_XTAL_SOURCE_MASK)
#define CCM_ANALOG_MISC0_SET_XTAL_24M_PWD_MASK   (0x40000000U)
#define CCM_ANALOG_MISC0_SET_XTAL_24M_PWD_SHIFT  (30U)
#define CCM_ANALOG_MISC0_SET_XTAL_24M_PWD(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_SET_XTAL_24M_PWD_SHIFT)) & CCM_ANALOG_MISC0_SET_XTAL_24M_PWD_MASK)
/*! @} */

/*! @name MISC0_CLR - Miscellaneous Register 0 */
/*! @{ */
#define CCM_ANALOG_MISC0_CLR_REFTOP_PWD_MASK     (0x1U)
#define CCM_ANALOG_MISC0_CLR_REFTOP_PWD_SHIFT    (0U)
#define CCM_ANALOG_MISC0_CLR_REFTOP_PWD(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_REFTOP_PWD_SHIFT)) & CCM_ANALOG_MISC0_CLR_REFTOP_PWD_MASK)
#define CCM_ANALOG_MISC0_CLR_REFTOP_SELFBIASOFF_MASK (0x8U)
#define CCM_ANALOG_MISC0_CLR_REFTOP_SELFBIASOFF_SHIFT (3U)
/*! REFTOP_SELFBIASOFF
 *  0b0..Uses coarse bias currents for startup
 *  0b1..Uses bandgap-based bias currents for best performance.
 */
#define CCM_ANALOG_MISC0_CLR_REFTOP_SELFBIASOFF(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_REFTOP_SELFBIASOFF_SHIFT)) & CCM_ANALOG_MISC0_CLR_REFTOP_SELFBIASOFF_MASK)
#define CCM_ANALOG_MISC0_CLR_REFTOP_VBGADJ_MASK  (0x70U)
#define CCM_ANALOG_MISC0_CLR_REFTOP_VBGADJ_SHIFT (4U)
/*! REFTOP_VBGADJ
 *  0b000..Nominal VBG
 *  0b001..VBG+0.78%
 *  0b010..VBG+1.56%
 *  0b011..VBG+2.34%
 *  0b100..VBG-0.78%
 *  0b101..VBG-1.56%
 *  0b110..VBG-2.34%
 *  0b111..VBG-3.12%
 */
#define CCM_ANALOG_MISC0_CLR_REFTOP_VBGADJ(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_REFTOP_VBGADJ_SHIFT)) & CCM_ANALOG_MISC0_CLR_REFTOP_VBGADJ_MASK)
#define CCM_ANALOG_MISC0_CLR_REFTOP_VBGUP_MASK   (0x80U)
#define CCM_ANALOG_MISC0_CLR_REFTOP_VBGUP_SHIFT  (7U)
#define CCM_ANALOG_MISC0_CLR_REFTOP_VBGUP(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_REFTOP_VBGUP_SHIFT)) & CCM_ANALOG_MISC0_CLR_REFTOP_VBGUP_MASK)
#define CCM_ANALOG_MISC0_CLR_STOP_MODE_CONFIG_MASK (0xC00U)
#define CCM_ANALOG_MISC0_CLR_STOP_MODE_CONFIG_SHIFT (10U)
/*! STOP_MODE_CONFIG
 *  0b00..All analog except RTC powered down on stop mode assertion.
 *  0b01..Beside RTC, analog bandgap, 1p1 and 2p5 regulators are also on.
 *  0b10..Beside RTC, 1p1 and 2p5 regulators are also on, low-power bandgap is selected so that the normal analog
 *        bandgap together with the rest analog is powered down.
 *  0b11..Beside RTC, low-power bandgap is selected and the rest analog is powered down.
 */
#define CCM_ANALOG_MISC0_CLR_STOP_MODE_CONFIG(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_STOP_MODE_CONFIG_SHIFT)) & CCM_ANALOG_MISC0_CLR_STOP_MODE_CONFIG_MASK)
#define CCM_ANALOG_MISC0_CLR_DISCON_HIGH_SNVS_MASK (0x1000U)
#define CCM_ANALOG_MISC0_CLR_DISCON_HIGH_SNVS_SHIFT (12U)
/*! DISCON_HIGH_SNVS
 *  0b0..Turn on the switch
 *  0b1..Turn off the switch
 */
#define CCM_ANALOG_MISC0_CLR_DISCON_HIGH_SNVS(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_DISCON_HIGH_SNVS_SHIFT)) & CCM_ANALOG_MISC0_CLR_DISCON_HIGH_SNVS_MASK)
#define CCM_ANALOG_MISC0_CLR_OSC_I_MASK          (0x6000U)
#define CCM_ANALOG_MISC0_CLR_OSC_I_SHIFT         (13U)
/*! OSC_I
 *  0b00..Nominal
 *  0b01..Decrease current by 12.5%
 *  0b10..Decrease current by 25.0%
 *  0b11..Decrease current by 37.5%
 */
#define CCM_ANALOG_MISC0_CLR_OSC_I(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_OSC_I_SHIFT)) & CCM_ANALOG_MISC0_CLR_OSC_I_MASK)
#define CCM_ANALOG_MISC0_CLR_OSC_XTALOK_MASK     (0x8000U)
#define CCM_ANALOG_MISC0_CLR_OSC_XTALOK_SHIFT    (15U)
#define CCM_ANALOG_MISC0_CLR_OSC_XTALOK(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_OSC_XTALOK_SHIFT)) & CCM_ANALOG_MISC0_CLR_OSC_XTALOK_MASK)
#define CCM_ANALOG_MISC0_CLR_OSC_XTALOK_EN_MASK  (0x10000U)
#define CCM_ANALOG_MISC0_CLR_OSC_XTALOK_EN_SHIFT (16U)
#define CCM_ANALOG_MISC0_CLR_OSC_XTALOK_EN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_OSC_XTALOK_EN_SHIFT)) & CCM_ANALOG_MISC0_CLR_OSC_XTALOK_EN_MASK)
#define CCM_ANALOG_MISC0_CLR_CLKGATE_CTRL_MASK   (0x2000000U)
#define CCM_ANALOG_MISC0_CLR_CLKGATE_CTRL_SHIFT  (25U)
/*! CLKGATE_CTRL
 *  0b0..Allow the logic to automatically gate the clock when the XTAL is powered down.
 *  0b1..Prevent the logic from ever gating off the clock.
 */
#define CCM_ANALOG_MISC0_CLR_CLKGATE_CTRL(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_CLKGATE_CTRL_SHIFT)) & CCM_ANALOG_MISC0_CLR_CLKGATE_CTRL_MASK)
#define CCM_ANALOG_MISC0_CLR_CLKGATE_DELAY_MASK  (0x1C000000U)
#define CCM_ANALOG_MISC0_CLR_CLKGATE_DELAY_SHIFT (26U)
/*! CLKGATE_DELAY
 *  0b000..0.5ms
 *  0b001..1.0ms
 *  0b010..2.0ms
 *  0b011..3.0ms
 *  0b100..4.0ms
 *  0b101..5.0ms
 *  0b110..6.0ms
 *  0b111..7.0ms
 */
#define CCM_ANALOG_MISC0_CLR_CLKGATE_DELAY(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_CLKGATE_DELAY_SHIFT)) & CCM_ANALOG_MISC0_CLR_CLKGATE_DELAY_MASK)
#define CCM_ANALOG_MISC0_CLR_RTC_XTAL_SOURCE_MASK (0x20000000U)
#define CCM_ANALOG_MISC0_CLR_RTC_XTAL_SOURCE_SHIFT (29U)
/*! RTC_XTAL_SOURCE
 *  0b0..Internal ring oscillator
 *  0b1..RTC_XTAL
 */
#define CCM_ANALOG_MISC0_CLR_RTC_XTAL_SOURCE(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_RTC_XTAL_SOURCE_SHIFT)) & CCM_ANALOG_MISC0_CLR_RTC_XTAL_SOURCE_MASK)
#define CCM_ANALOG_MISC0_CLR_XTAL_24M_PWD_MASK   (0x40000000U)
#define CCM_ANALOG_MISC0_CLR_XTAL_24M_PWD_SHIFT  (30U)
#define CCM_ANALOG_MISC0_CLR_XTAL_24M_PWD(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_CLR_XTAL_24M_PWD_SHIFT)) & CCM_ANALOG_MISC0_CLR_XTAL_24M_PWD_MASK)
/*! @} */

/*! @name MISC0_TOG - Miscellaneous Register 0 */
/*! @{ */
#define CCM_ANALOG_MISC0_TOG_REFTOP_PWD_MASK     (0x1U)
#define CCM_ANALOG_MISC0_TOG_REFTOP_PWD_SHIFT    (0U)
#define CCM_ANALOG_MISC0_TOG_REFTOP_PWD(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_REFTOP_PWD_SHIFT)) & CCM_ANALOG_MISC0_TOG_REFTOP_PWD_MASK)
#define CCM_ANALOG_MISC0_TOG_REFTOP_SELFBIASOFF_MASK (0x8U)
#define CCM_ANALOG_MISC0_TOG_REFTOP_SELFBIASOFF_SHIFT (3U)
/*! REFTOP_SELFBIASOFF
 *  0b0..Uses coarse bias currents for startup
 *  0b1..Uses bandgap-based bias currents for best performance.
 */
#define CCM_ANALOG_MISC0_TOG_REFTOP_SELFBIASOFF(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_REFTOP_SELFBIASOFF_SHIFT)) & CCM_ANALOG_MISC0_TOG_REFTOP_SELFBIASOFF_MASK)
#define CCM_ANALOG_MISC0_TOG_REFTOP_VBGADJ_MASK  (0x70U)
#define CCM_ANALOG_MISC0_TOG_REFTOP_VBGADJ_SHIFT (4U)
/*! REFTOP_VBGADJ
 *  0b000..Nominal VBG
 *  0b001..VBG+0.78%
 *  0b010..VBG+1.56%
 *  0b011..VBG+2.34%
 *  0b100..VBG-0.78%
 *  0b101..VBG-1.56%
 *  0b110..VBG-2.34%
 *  0b111..VBG-3.12%
 */
#define CCM_ANALOG_MISC0_TOG_REFTOP_VBGADJ(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_REFTOP_VBGADJ_SHIFT)) & CCM_ANALOG_MISC0_TOG_REFTOP_VBGADJ_MASK)
#define CCM_ANALOG_MISC0_TOG_REFTOP_VBGUP_MASK   (0x80U)
#define CCM_ANALOG_MISC0_TOG_REFTOP_VBGUP_SHIFT  (7U)
#define CCM_ANALOG_MISC0_TOG_REFTOP_VBGUP(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_REFTOP_VBGUP_SHIFT)) & CCM_ANALOG_MISC0_TOG_REFTOP_VBGUP_MASK)
#define CCM_ANALOG_MISC0_TOG_STOP_MODE_CONFIG_MASK (0xC00U)
#define CCM_ANALOG_MISC0_TOG_STOP_MODE_CONFIG_SHIFT (10U)
/*! STOP_MODE_CONFIG
 *  0b00..All analog except RTC powered down on stop mode assertion.
 *  0b01..Beside RTC, analog bandgap, 1p1 and 2p5 regulators are also on.
 *  0b10..Beside RTC, 1p1 and 2p5 regulators are also on, low-power bandgap is selected so that the normal analog
 *        bandgap together with the rest analog is powered down.
 *  0b11..Beside RTC, low-power bandgap is selected and the rest analog is powered down.
 */
#define CCM_ANALOG_MISC0_TOG_STOP_MODE_CONFIG(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_STOP_MODE_CONFIG_SHIFT)) & CCM_ANALOG_MISC0_TOG_STOP_MODE_CONFIG_MASK)
#define CCM_ANALOG_MISC0_TOG_DISCON_HIGH_SNVS_MASK (0x1000U)
#define CCM_ANALOG_MISC0_TOG_DISCON_HIGH_SNVS_SHIFT (12U)
/*! DISCON_HIGH_SNVS
 *  0b0..Turn on the switch
 *  0b1..Turn off the switch
 */
#define CCM_ANALOG_MISC0_TOG_DISCON_HIGH_SNVS(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_DISCON_HIGH_SNVS_SHIFT)) & CCM_ANALOG_MISC0_TOG_DISCON_HIGH_SNVS_MASK)
#define CCM_ANALOG_MISC0_TOG_OSC_I_MASK          (0x6000U)
#define CCM_ANALOG_MISC0_TOG_OSC_I_SHIFT         (13U)
/*! OSC_I
 *  0b00..Nominal
 *  0b01..Decrease current by 12.5%
 *  0b10..Decrease current by 25.0%
 *  0b11..Decrease current by 37.5%
 */
#define CCM_ANALOG_MISC0_TOG_OSC_I(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_OSC_I_SHIFT)) & CCM_ANALOG_MISC0_TOG_OSC_I_MASK)
#define CCM_ANALOG_MISC0_TOG_OSC_XTALOK_MASK     (0x8000U)
#define CCM_ANALOG_MISC0_TOG_OSC_XTALOK_SHIFT    (15U)
#define CCM_ANALOG_MISC0_TOG_OSC_XTALOK(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_OSC_XTALOK_SHIFT)) & CCM_ANALOG_MISC0_TOG_OSC_XTALOK_MASK)
#define CCM_ANALOG_MISC0_TOG_OSC_XTALOK_EN_MASK  (0x10000U)
#define CCM_ANALOG_MISC0_TOG_OSC_XTALOK_EN_SHIFT (16U)
#define CCM_ANALOG_MISC0_TOG_OSC_XTALOK_EN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_OSC_XTALOK_EN_SHIFT)) & CCM_ANALOG_MISC0_TOG_OSC_XTALOK_EN_MASK)
#define CCM_ANALOG_MISC0_TOG_CLKGATE_CTRL_MASK   (0x2000000U)
#define CCM_ANALOG_MISC0_TOG_CLKGATE_CTRL_SHIFT  (25U)
/*! CLKGATE_CTRL
 *  0b0..Allow the logic to automatically gate the clock when the XTAL is powered down.
 *  0b1..Prevent the logic from ever gating off the clock.
 */
#define CCM_ANALOG_MISC0_TOG_CLKGATE_CTRL(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_CLKGATE_CTRL_SHIFT)) & CCM_ANALOG_MISC0_TOG_CLKGATE_CTRL_MASK)
#define CCM_ANALOG_MISC0_TOG_CLKGATE_DELAY_MASK  (0x1C000000U)
#define CCM_ANALOG_MISC0_TOG_CLKGATE_DELAY_SHIFT (26U)
/*! CLKGATE_DELAY
 *  0b000..0.5ms
 *  0b001..1.0ms
 *  0b010..2.0ms
 *  0b011..3.0ms
 *  0b100..4.0ms
 *  0b101..5.0ms
 *  0b110..6.0ms
 *  0b111..7.0ms
 */
#define CCM_ANALOG_MISC0_TOG_CLKGATE_DELAY(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_CLKGATE_DELAY_SHIFT)) & CCM_ANALOG_MISC0_TOG_CLKGATE_DELAY_MASK)
#define CCM_ANALOG_MISC0_TOG_RTC_XTAL_SOURCE_MASK (0x20000000U)
#define CCM_ANALOG_MISC0_TOG_RTC_XTAL_SOURCE_SHIFT (29U)
/*! RTC_XTAL_SOURCE
 *  0b0..Internal ring oscillator
 *  0b1..RTC_XTAL
 */
#define CCM_ANALOG_MISC0_TOG_RTC_XTAL_SOURCE(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_RTC_XTAL_SOURCE_SHIFT)) & CCM_ANALOG_MISC0_TOG_RTC_XTAL_SOURCE_MASK)
#define CCM_ANALOG_MISC0_TOG_XTAL_24M_PWD_MASK   (0x40000000U)
#define CCM_ANALOG_MISC0_TOG_XTAL_24M_PWD_SHIFT  (30U)
#define CCM_ANALOG_MISC0_TOG_XTAL_24M_PWD(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC0_TOG_XTAL_24M_PWD_SHIFT)) & CCM_ANALOG_MISC0_TOG_XTAL_24M_PWD_MASK)
/*! @} */

/*! @name MISC1 - Miscellaneous Register 1 */
/*! @{ */
#define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_MASK      (0x1FU)
#define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT     (0U)
/*! LVDS1_CLK_SEL
 *  0b00000..Arm PLL
 *  0b00001..System PLL
 *  0b00010..ref_pfd4_clk == pll2_pfd0_clk
 *  0b00011..ref_pfd5_clk == pll2_pfd1_clk
 *  0b00100..ref_pfd6_clk == pll2_pfd2_clk
 *  0b00101..ref_pfd7_clk == pll2_pfd3_clk
 *  0b00110..Audio PLL
 *  0b00111..Video PLL
 *  0b01001..ethernet ref clock (ENET_PLL)
 *  0b01100..USB1 PLL clock
 *  0b01101..USB2 PLL clock
 *  0b01110..ref_pfd0_clk == pll3_pfd0_clk
 *  0b01111..ref_pfd1_clk == pll3_pfd1_clk
 *  0b10000..ref_pfd2_clk == pll3_pfd2_clk
 *  0b10001..ref_pfd3_clk == pll3_pfd3_clk
 *  0b10010..xtal (24M)
 */
#define CCM_ANALOG_MISC1_LVDS1_CLK_SEL(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_LVDS1_CLK_SEL_SHIFT)) & CCM_ANALOG_MISC1_LVDS1_CLK_SEL_MASK)
#define CCM_ANALOG_MISC1_LVDSCLK1_OBEN_MASK      (0x400U)
#define CCM_ANALOG_MISC1_LVDSCLK1_OBEN_SHIFT     (10U)
#define CCM_ANALOG_MISC1_LVDSCLK1_OBEN(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_LVDSCLK1_OBEN_SHIFT)) & CCM_ANALOG_MISC1_LVDSCLK1_OBEN_MASK)
#define CCM_ANALOG_MISC1_LVDSCLK1_IBEN_MASK      (0x1000U)
#define CCM_ANALOG_MISC1_LVDSCLK1_IBEN_SHIFT     (12U)
#define CCM_ANALOG_MISC1_LVDSCLK1_IBEN(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_LVDSCLK1_IBEN_SHIFT)) & CCM_ANALOG_MISC1_LVDSCLK1_IBEN_MASK)
#define CCM_ANALOG_MISC1_PFD_480_AUTOGATE_EN_MASK (0x10000U)
#define CCM_ANALOG_MISC1_PFD_480_AUTOGATE_EN_SHIFT (16U)
#define CCM_ANALOG_MISC1_PFD_480_AUTOGATE_EN(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_PFD_480_AUTOGATE_EN_SHIFT)) & CCM_ANALOG_MISC1_PFD_480_AUTOGATE_EN_MASK)
#define CCM_ANALOG_MISC1_PFD_528_AUTOGATE_EN_MASK (0x20000U)
#define CCM_ANALOG_MISC1_PFD_528_AUTOGATE_EN_SHIFT (17U)
#define CCM_ANALOG_MISC1_PFD_528_AUTOGATE_EN(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_PFD_528_AUTOGATE_EN_SHIFT)) & CCM_ANALOG_MISC1_PFD_528_AUTOGATE_EN_MASK)
#define CCM_ANALOG_MISC1_IRQ_TEMPPANIC_MASK      (0x8000000U)
#define CCM_ANALOG_MISC1_IRQ_TEMPPANIC_SHIFT     (27U)
#define CCM_ANALOG_MISC1_IRQ_TEMPPANIC(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_IRQ_TEMPPANIC_SHIFT)) & CCM_ANALOG_MISC1_IRQ_TEMPPANIC_MASK)
#define CCM_ANALOG_MISC1_IRQ_TEMPLOW_MASK        (0x10000000U)
#define CCM_ANALOG_MISC1_IRQ_TEMPLOW_SHIFT       (28U)
#define CCM_ANALOG_MISC1_IRQ_TEMPLOW(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_IRQ_TEMPLOW_SHIFT)) & CCM_ANALOG_MISC1_IRQ_TEMPLOW_MASK)
#define CCM_ANALOG_MISC1_IRQ_TEMPHIGH_MASK       (0x20000000U)
#define CCM_ANALOG_MISC1_IRQ_TEMPHIGH_SHIFT      (29U)
#define CCM_ANALOG_MISC1_IRQ_TEMPHIGH(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_IRQ_TEMPHIGH_SHIFT)) & CCM_ANALOG_MISC1_IRQ_TEMPHIGH_MASK)
#define CCM_ANALOG_MISC1_IRQ_ANA_BO_MASK         (0x40000000U)
#define CCM_ANALOG_MISC1_IRQ_ANA_BO_SHIFT        (30U)
#define CCM_ANALOG_MISC1_IRQ_ANA_BO(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_IRQ_ANA_BO_SHIFT)) & CCM_ANALOG_MISC1_IRQ_ANA_BO_MASK)
#define CCM_ANALOG_MISC1_IRQ_DIG_BO_MASK         (0x80000000U)
#define CCM_ANALOG_MISC1_IRQ_DIG_BO_SHIFT        (31U)
#define CCM_ANALOG_MISC1_IRQ_DIG_BO(x)           (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_IRQ_DIG_BO_SHIFT)) & CCM_ANALOG_MISC1_IRQ_DIG_BO_MASK)
/*! @} */

/*! @name MISC1_SET - Miscellaneous Register 1 */
/*! @{ */
#define CCM_ANALOG_MISC1_SET_LVDS1_CLK_SEL_MASK  (0x1FU)
#define CCM_ANALOG_MISC1_SET_LVDS1_CLK_SEL_SHIFT (0U)
/*! LVDS1_CLK_SEL
 *  0b00000..Arm PLL
 *  0b00001..System PLL
 *  0b00010..ref_pfd4_clk == pll2_pfd0_clk
 *  0b00011..ref_pfd5_clk == pll2_pfd1_clk
 *  0b00100..ref_pfd6_clk == pll2_pfd2_clk
 *  0b00101..ref_pfd7_clk == pll2_pfd3_clk
 *  0b00110..Audio PLL
 *  0b00111..Video PLL
 *  0b01001..ethernet ref clock (ENET_PLL)
 *  0b01100..USB1 PLL clock
 *  0b01101..USB2 PLL clock
 *  0b01110..ref_pfd0_clk == pll3_pfd0_clk
 *  0b01111..ref_pfd1_clk == pll3_pfd1_clk
 *  0b10000..ref_pfd2_clk == pll3_pfd2_clk
 *  0b10001..ref_pfd3_clk == pll3_pfd3_clk
 *  0b10010..xtal (24M)
 */
#define CCM_ANALOG_MISC1_SET_LVDS1_CLK_SEL(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_LVDS1_CLK_SEL_SHIFT)) & CCM_ANALOG_MISC1_SET_LVDS1_CLK_SEL_MASK)
#define CCM_ANALOG_MISC1_SET_LVDSCLK1_OBEN_MASK  (0x400U)
#define CCM_ANALOG_MISC1_SET_LVDSCLK1_OBEN_SHIFT (10U)
#define CCM_ANALOG_MISC1_SET_LVDSCLK1_OBEN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_LVDSCLK1_OBEN_SHIFT)) & CCM_ANALOG_MISC1_SET_LVDSCLK1_OBEN_MASK)
#define CCM_ANALOG_MISC1_SET_LVDSCLK1_IBEN_MASK  (0x1000U)
#define CCM_ANALOG_MISC1_SET_LVDSCLK1_IBEN_SHIFT (12U)
#define CCM_ANALOG_MISC1_SET_LVDSCLK1_IBEN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_LVDSCLK1_IBEN_SHIFT)) & CCM_ANALOG_MISC1_SET_LVDSCLK1_IBEN_MASK)
#define CCM_ANALOG_MISC1_SET_PFD_480_AUTOGATE_EN_MASK (0x10000U)
#define CCM_ANALOG_MISC1_SET_PFD_480_AUTOGATE_EN_SHIFT (16U)
#define CCM_ANALOG_MISC1_SET_PFD_480_AUTOGATE_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_PFD_480_AUTOGATE_EN_SHIFT)) & CCM_ANALOG_MISC1_SET_PFD_480_AUTOGATE_EN_MASK)
#define CCM_ANALOG_MISC1_SET_PFD_528_AUTOGATE_EN_MASK (0x20000U)
#define CCM_ANALOG_MISC1_SET_PFD_528_AUTOGATE_EN_SHIFT (17U)
#define CCM_ANALOG_MISC1_SET_PFD_528_AUTOGATE_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_PFD_528_AUTOGATE_EN_SHIFT)) & CCM_ANALOG_MISC1_SET_PFD_528_AUTOGATE_EN_MASK)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPPANIC_MASK  (0x8000000U)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPPANIC_SHIFT (27U)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPPANIC(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_IRQ_TEMPPANIC_SHIFT)) & CCM_ANALOG_MISC1_SET_IRQ_TEMPPANIC_MASK)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPLOW_MASK    (0x10000000U)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPLOW_SHIFT   (28U)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPLOW(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_IRQ_TEMPLOW_SHIFT)) & CCM_ANALOG_MISC1_SET_IRQ_TEMPLOW_MASK)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPHIGH_MASK   (0x20000000U)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPHIGH_SHIFT  (29U)
#define CCM_ANALOG_MISC1_SET_IRQ_TEMPHIGH(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_IRQ_TEMPHIGH_SHIFT)) & CCM_ANALOG_MISC1_SET_IRQ_TEMPHIGH_MASK)
#define CCM_ANALOG_MISC1_SET_IRQ_ANA_BO_MASK     (0x40000000U)
#define CCM_ANALOG_MISC1_SET_IRQ_ANA_BO_SHIFT    (30U)
#define CCM_ANALOG_MISC1_SET_IRQ_ANA_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_IRQ_ANA_BO_SHIFT)) & CCM_ANALOG_MISC1_SET_IRQ_ANA_BO_MASK)
#define CCM_ANALOG_MISC1_SET_IRQ_DIG_BO_MASK     (0x80000000U)
#define CCM_ANALOG_MISC1_SET_IRQ_DIG_BO_SHIFT    (31U)
#define CCM_ANALOG_MISC1_SET_IRQ_DIG_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_SET_IRQ_DIG_BO_SHIFT)) & CCM_ANALOG_MISC1_SET_IRQ_DIG_BO_MASK)
/*! @} */

/*! @name MISC1_CLR - Miscellaneous Register 1 */
/*! @{ */
#define CCM_ANALOG_MISC1_CLR_LVDS1_CLK_SEL_MASK  (0x1FU)
#define CCM_ANALOG_MISC1_CLR_LVDS1_CLK_SEL_SHIFT (0U)
/*! LVDS1_CLK_SEL
 *  0b00000..Arm PLL
 *  0b00001..System PLL
 *  0b00010..ref_pfd4_clk == pll2_pfd0_clk
 *  0b00011..ref_pfd5_clk == pll2_pfd1_clk
 *  0b00100..ref_pfd6_clk == pll2_pfd2_clk
 *  0b00101..ref_pfd7_clk == pll2_pfd3_clk
 *  0b00110..Audio PLL
 *  0b00111..Video PLL
 *  0b01001..ethernet ref clock (ENET_PLL)
 *  0b01100..USB1 PLL clock
 *  0b01101..USB2 PLL clock
 *  0b01110..ref_pfd0_clk == pll3_pfd0_clk
 *  0b01111..ref_pfd1_clk == pll3_pfd1_clk
 *  0b10000..ref_pfd2_clk == pll3_pfd2_clk
 *  0b10001..ref_pfd3_clk == pll3_pfd3_clk
 *  0b10010..xtal (24M)
 */
#define CCM_ANALOG_MISC1_CLR_LVDS1_CLK_SEL(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_LVDS1_CLK_SEL_SHIFT)) & CCM_ANALOG_MISC1_CLR_LVDS1_CLK_SEL_MASK)
#define CCM_ANALOG_MISC1_CLR_LVDSCLK1_OBEN_MASK  (0x400U)
#define CCM_ANALOG_MISC1_CLR_LVDSCLK1_OBEN_SHIFT (10U)
#define CCM_ANALOG_MISC1_CLR_LVDSCLK1_OBEN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_LVDSCLK1_OBEN_SHIFT)) & CCM_ANALOG_MISC1_CLR_LVDSCLK1_OBEN_MASK)
#define CCM_ANALOG_MISC1_CLR_LVDSCLK1_IBEN_MASK  (0x1000U)
#define CCM_ANALOG_MISC1_CLR_LVDSCLK1_IBEN_SHIFT (12U)
#define CCM_ANALOG_MISC1_CLR_LVDSCLK1_IBEN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_LVDSCLK1_IBEN_SHIFT)) & CCM_ANALOG_MISC1_CLR_LVDSCLK1_IBEN_MASK)
#define CCM_ANALOG_MISC1_CLR_PFD_480_AUTOGATE_EN_MASK (0x10000U)
#define CCM_ANALOG_MISC1_CLR_PFD_480_AUTOGATE_EN_SHIFT (16U)
#define CCM_ANALOG_MISC1_CLR_PFD_480_AUTOGATE_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_PFD_480_AUTOGATE_EN_SHIFT)) & CCM_ANALOG_MISC1_CLR_PFD_480_AUTOGATE_EN_MASK)
#define CCM_ANALOG_MISC1_CLR_PFD_528_AUTOGATE_EN_MASK (0x20000U)
#define CCM_ANALOG_MISC1_CLR_PFD_528_AUTOGATE_EN_SHIFT (17U)
#define CCM_ANALOG_MISC1_CLR_PFD_528_AUTOGATE_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_PFD_528_AUTOGATE_EN_SHIFT)) & CCM_ANALOG_MISC1_CLR_PFD_528_AUTOGATE_EN_MASK)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPPANIC_MASK  (0x8000000U)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPPANIC_SHIFT (27U)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPPANIC(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_IRQ_TEMPPANIC_SHIFT)) & CCM_ANALOG_MISC1_CLR_IRQ_TEMPPANIC_MASK)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPLOW_MASK    (0x10000000U)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPLOW_SHIFT   (28U)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPLOW(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_IRQ_TEMPLOW_SHIFT)) & CCM_ANALOG_MISC1_CLR_IRQ_TEMPLOW_MASK)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPHIGH_MASK   (0x20000000U)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPHIGH_SHIFT  (29U)
#define CCM_ANALOG_MISC1_CLR_IRQ_TEMPHIGH(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_IRQ_TEMPHIGH_SHIFT)) & CCM_ANALOG_MISC1_CLR_IRQ_TEMPHIGH_MASK)
#define CCM_ANALOG_MISC1_CLR_IRQ_ANA_BO_MASK     (0x40000000U)
#define CCM_ANALOG_MISC1_CLR_IRQ_ANA_BO_SHIFT    (30U)
#define CCM_ANALOG_MISC1_CLR_IRQ_ANA_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_IRQ_ANA_BO_SHIFT)) & CCM_ANALOG_MISC1_CLR_IRQ_ANA_BO_MASK)
#define CCM_ANALOG_MISC1_CLR_IRQ_DIG_BO_MASK     (0x80000000U)
#define CCM_ANALOG_MISC1_CLR_IRQ_DIG_BO_SHIFT    (31U)
#define CCM_ANALOG_MISC1_CLR_IRQ_DIG_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_CLR_IRQ_DIG_BO_SHIFT)) & CCM_ANALOG_MISC1_CLR_IRQ_DIG_BO_MASK)
/*! @} */

/*! @name MISC1_TOG - Miscellaneous Register 1 */
/*! @{ */
#define CCM_ANALOG_MISC1_TOG_LVDS1_CLK_SEL_MASK  (0x1FU)
#define CCM_ANALOG_MISC1_TOG_LVDS1_CLK_SEL_SHIFT (0U)
/*! LVDS1_CLK_SEL
 *  0b00000..Arm PLL
 *  0b00001..System PLL
 *  0b00010..ref_pfd4_clk == pll2_pfd0_clk
 *  0b00011..ref_pfd5_clk == pll2_pfd1_clk
 *  0b00100..ref_pfd6_clk == pll2_pfd2_clk
 *  0b00101..ref_pfd7_clk == pll2_pfd3_clk
 *  0b00110..Audio PLL
 *  0b00111..Video PLL
 *  0b01001..ethernet ref clock (ENET_PLL)
 *  0b01100..USB1 PLL clock
 *  0b01101..USB2 PLL clock
 *  0b01110..ref_pfd0_clk == pll3_pfd0_clk
 *  0b01111..ref_pfd1_clk == pll3_pfd1_clk
 *  0b10000..ref_pfd2_clk == pll3_pfd2_clk
 *  0b10001..ref_pfd3_clk == pll3_pfd3_clk
 *  0b10010..xtal (24M)
 */
#define CCM_ANALOG_MISC1_TOG_LVDS1_CLK_SEL(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_LVDS1_CLK_SEL_SHIFT)) & CCM_ANALOG_MISC1_TOG_LVDS1_CLK_SEL_MASK)
#define CCM_ANALOG_MISC1_TOG_LVDSCLK1_OBEN_MASK  (0x400U)
#define CCM_ANALOG_MISC1_TOG_LVDSCLK1_OBEN_SHIFT (10U)
#define CCM_ANALOG_MISC1_TOG_LVDSCLK1_OBEN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_LVDSCLK1_OBEN_SHIFT)) & CCM_ANALOG_MISC1_TOG_LVDSCLK1_OBEN_MASK)
#define CCM_ANALOG_MISC1_TOG_LVDSCLK1_IBEN_MASK  (0x1000U)
#define CCM_ANALOG_MISC1_TOG_LVDSCLK1_IBEN_SHIFT (12U)
#define CCM_ANALOG_MISC1_TOG_LVDSCLK1_IBEN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_LVDSCLK1_IBEN_SHIFT)) & CCM_ANALOG_MISC1_TOG_LVDSCLK1_IBEN_MASK)
#define CCM_ANALOG_MISC1_TOG_PFD_480_AUTOGATE_EN_MASK (0x10000U)
#define CCM_ANALOG_MISC1_TOG_PFD_480_AUTOGATE_EN_SHIFT (16U)
#define CCM_ANALOG_MISC1_TOG_PFD_480_AUTOGATE_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_PFD_480_AUTOGATE_EN_SHIFT)) & CCM_ANALOG_MISC1_TOG_PFD_480_AUTOGATE_EN_MASK)
#define CCM_ANALOG_MISC1_TOG_PFD_528_AUTOGATE_EN_MASK (0x20000U)
#define CCM_ANALOG_MISC1_TOG_PFD_528_AUTOGATE_EN_SHIFT (17U)
#define CCM_ANALOG_MISC1_TOG_PFD_528_AUTOGATE_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_PFD_528_AUTOGATE_EN_SHIFT)) & CCM_ANALOG_MISC1_TOG_PFD_528_AUTOGATE_EN_MASK)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPPANIC_MASK  (0x8000000U)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPPANIC_SHIFT (27U)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPPANIC(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_IRQ_TEMPPANIC_SHIFT)) & CCM_ANALOG_MISC1_TOG_IRQ_TEMPPANIC_MASK)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPLOW_MASK    (0x10000000U)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPLOW_SHIFT   (28U)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPLOW(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_IRQ_TEMPLOW_SHIFT)) & CCM_ANALOG_MISC1_TOG_IRQ_TEMPLOW_MASK)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPHIGH_MASK   (0x20000000U)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPHIGH_SHIFT  (29U)
#define CCM_ANALOG_MISC1_TOG_IRQ_TEMPHIGH(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_IRQ_TEMPHIGH_SHIFT)) & CCM_ANALOG_MISC1_TOG_IRQ_TEMPHIGH_MASK)
#define CCM_ANALOG_MISC1_TOG_IRQ_ANA_BO_MASK     (0x40000000U)
#define CCM_ANALOG_MISC1_TOG_IRQ_ANA_BO_SHIFT    (30U)
#define CCM_ANALOG_MISC1_TOG_IRQ_ANA_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_IRQ_ANA_BO_SHIFT)) & CCM_ANALOG_MISC1_TOG_IRQ_ANA_BO_MASK)
#define CCM_ANALOG_MISC1_TOG_IRQ_DIG_BO_MASK     (0x80000000U)
#define CCM_ANALOG_MISC1_TOG_IRQ_DIG_BO_SHIFT    (31U)
#define CCM_ANALOG_MISC1_TOG_IRQ_DIG_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC1_TOG_IRQ_DIG_BO_SHIFT)) & CCM_ANALOG_MISC1_TOG_IRQ_DIG_BO_MASK)
/*! @} */

/*! @name MISC2 - Miscellaneous Register 2 */
/*! @{ */
#define CCM_ANALOG_MISC2_REG0_BO_OFFSET_MASK     (0x7U)
#define CCM_ANALOG_MISC2_REG0_BO_OFFSET_SHIFT    (0U)
/*! REG0_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_REG0_BO_OFFSET(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG0_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_REG0_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_REG0_BO_STATUS_MASK     (0x8U)
#define CCM_ANALOG_MISC2_REG0_BO_STATUS_SHIFT    (3U)
/*! REG0_BO_STATUS
 *  0b1..Brownout, supply is below target minus brownout offset.
 */
#define CCM_ANALOG_MISC2_REG0_BO_STATUS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG0_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_REG0_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_REG0_ENABLE_BO_MASK     (0x20U)
#define CCM_ANALOG_MISC2_REG0_ENABLE_BO_SHIFT    (5U)
#define CCM_ANALOG_MISC2_REG0_ENABLE_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG0_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_REG0_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_REG0_OK_MASK            (0x40U)
#define CCM_ANALOG_MISC2_REG0_OK_SHIFT           (6U)
#define CCM_ANALOG_MISC2_REG0_OK(x)              (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG0_OK_SHIFT)) & CCM_ANALOG_MISC2_REG0_OK_MASK)
#define CCM_ANALOG_MISC2_PLL3_DISABLE_MASK       (0x80U)
#define CCM_ANALOG_MISC2_PLL3_DISABLE_SHIFT      (7U)
/*! PLL3_DISABLE
 *  0b0..PLL3 is being used by peripherals and is enabled when SoC is not in any low power mode
 *  0b1..PLL3 can be disabled when the SoC is not in any low power mode
 */
#define CCM_ANALOG_MISC2_PLL3_DISABLE(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_PLL3_DISABLE_SHIFT)) & CCM_ANALOG_MISC2_PLL3_DISABLE_MASK)
#define CCM_ANALOG_MISC2_REG1_BO_OFFSET_MASK     (0x700U)
#define CCM_ANALOG_MISC2_REG1_BO_OFFSET_SHIFT    (8U)
/*! REG1_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_REG1_BO_OFFSET(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG1_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_REG1_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_REG1_BO_STATUS_MASK     (0x800U)
#define CCM_ANALOG_MISC2_REG1_BO_STATUS_SHIFT    (11U)
/*! REG1_BO_STATUS
 *  0b1..Brownout, supply is below target minus brownout offset.
 */
#define CCM_ANALOG_MISC2_REG1_BO_STATUS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG1_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_REG1_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_REG1_ENABLE_BO_MASK     (0x2000U)
#define CCM_ANALOG_MISC2_REG1_ENABLE_BO_SHIFT    (13U)
#define CCM_ANALOG_MISC2_REG1_ENABLE_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG1_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_REG1_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_REG1_OK_MASK            (0x4000U)
#define CCM_ANALOG_MISC2_REG1_OK_SHIFT           (14U)
#define CCM_ANALOG_MISC2_REG1_OK(x)              (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG1_OK_SHIFT)) & CCM_ANALOG_MISC2_REG1_OK_MASK)
#define CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK      (0x8000U)
#define CCM_ANALOG_MISC2_AUDIO_DIV_LSB_SHIFT     (15U)
/*! AUDIO_DIV_LSB
 *  0b0..divide by 1 (Default)
 *  0b1..divide by 2
 */
#define CCM_ANALOG_MISC2_AUDIO_DIV_LSB(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_AUDIO_DIV_LSB_SHIFT)) & CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK)
#define CCM_ANALOG_MISC2_REG2_BO_OFFSET_MASK     (0x70000U)
#define CCM_ANALOG_MISC2_REG2_BO_OFFSET_SHIFT    (16U)
/*! REG2_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_REG2_BO_OFFSET(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG2_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_REG2_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_REG2_BO_STATUS_MASK     (0x80000U)
#define CCM_ANALOG_MISC2_REG2_BO_STATUS_SHIFT    (19U)
#define CCM_ANALOG_MISC2_REG2_BO_STATUS(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG2_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_REG2_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_REG2_ENABLE_BO_MASK     (0x200000U)
#define CCM_ANALOG_MISC2_REG2_ENABLE_BO_SHIFT    (21U)
#define CCM_ANALOG_MISC2_REG2_ENABLE_BO(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG2_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_REG2_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_REG2_OK_MASK            (0x400000U)
#define CCM_ANALOG_MISC2_REG2_OK_SHIFT           (22U)
#define CCM_ANALOG_MISC2_REG2_OK(x)              (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG2_OK_SHIFT)) & CCM_ANALOG_MISC2_REG2_OK_MASK)
#define CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK      (0x800000U)
#define CCM_ANALOG_MISC2_AUDIO_DIV_MSB_SHIFT     (23U)
/*! AUDIO_DIV_MSB
 *  0b0..divide by 1 (Default)
 *  0b1..divide by 2
 */
#define CCM_ANALOG_MISC2_AUDIO_DIV_MSB(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_AUDIO_DIV_MSB_SHIFT)) & CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK)
#define CCM_ANALOG_MISC2_REG0_STEP_TIME_MASK     (0x3000000U)
#define CCM_ANALOG_MISC2_REG0_STEP_TIME_SHIFT    (24U)
/*! REG0_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_REG0_STEP_TIME(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG0_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_REG0_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_REG1_STEP_TIME_MASK     (0xC000000U)
#define CCM_ANALOG_MISC2_REG1_STEP_TIME_SHIFT    (26U)
/*! REG1_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_REG1_STEP_TIME(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG1_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_REG1_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_REG2_STEP_TIME_MASK     (0x30000000U)
#define CCM_ANALOG_MISC2_REG2_STEP_TIME_SHIFT    (28U)
/*! REG2_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_REG2_STEP_TIME(x)       (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_REG2_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_REG2_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_VIDEO_DIV_MASK          (0xC0000000U)
#define CCM_ANALOG_MISC2_VIDEO_DIV_SHIFT         (30U)
/*! VIDEO_DIV
 *  0b00..divide by 1 (Default)
 *  0b01..divide by 2
 *  0b10..divide by 1
 *  0b11..divide by 4
 */
#define CCM_ANALOG_MISC2_VIDEO_DIV(x)            (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_VIDEO_DIV_SHIFT)) & CCM_ANALOG_MISC2_VIDEO_DIV_MASK)
/*! @} */

/*! @name MISC2_SET - Miscellaneous Register 2 */
/*! @{ */
#define CCM_ANALOG_MISC2_SET_REG0_BO_OFFSET_MASK (0x7U)
#define CCM_ANALOG_MISC2_SET_REG0_BO_OFFSET_SHIFT (0U)
/*! REG0_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_SET_REG0_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG0_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_SET_REG0_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_SET_REG0_BO_STATUS_MASK (0x8U)
#define CCM_ANALOG_MISC2_SET_REG0_BO_STATUS_SHIFT (3U)
/*! REG0_BO_STATUS
 *  0b1..Brownout, supply is below target minus brownout offset.
 */
#define CCM_ANALOG_MISC2_SET_REG0_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG0_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_SET_REG0_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_SET_REG0_ENABLE_BO_MASK (0x20U)
#define CCM_ANALOG_MISC2_SET_REG0_ENABLE_BO_SHIFT (5U)
#define CCM_ANALOG_MISC2_SET_REG0_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG0_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_SET_REG0_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_SET_REG0_OK_MASK        (0x40U)
#define CCM_ANALOG_MISC2_SET_REG0_OK_SHIFT       (6U)
#define CCM_ANALOG_MISC2_SET_REG0_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG0_OK_SHIFT)) & CCM_ANALOG_MISC2_SET_REG0_OK_MASK)
#define CCM_ANALOG_MISC2_SET_PLL3_DISABLE_MASK   (0x80U)
#define CCM_ANALOG_MISC2_SET_PLL3_DISABLE_SHIFT  (7U)
/*! PLL3_DISABLE
 *  0b0..PLL3 is being used by peripherals and is enabled when SoC is not in any low power mode
 *  0b1..PLL3 can be disabled when the SoC is not in any low power mode
 */
#define CCM_ANALOG_MISC2_SET_PLL3_DISABLE(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_PLL3_DISABLE_SHIFT)) & CCM_ANALOG_MISC2_SET_PLL3_DISABLE_MASK)
#define CCM_ANALOG_MISC2_SET_REG1_BO_OFFSET_MASK (0x700U)
#define CCM_ANALOG_MISC2_SET_REG1_BO_OFFSET_SHIFT (8U)
/*! REG1_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_SET_REG1_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG1_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_SET_REG1_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_SET_REG1_BO_STATUS_MASK (0x800U)
#define CCM_ANALOG_MISC2_SET_REG1_BO_STATUS_SHIFT (11U)
/*! REG1_BO_STATUS
 *  0b1..Brownout, supply is below target minus brownout offset.
 */
#define CCM_ANALOG_MISC2_SET_REG1_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG1_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_SET_REG1_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_SET_REG1_ENABLE_BO_MASK (0x2000U)
#define CCM_ANALOG_MISC2_SET_REG1_ENABLE_BO_SHIFT (13U)
#define CCM_ANALOG_MISC2_SET_REG1_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG1_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_SET_REG1_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_SET_REG1_OK_MASK        (0x4000U)
#define CCM_ANALOG_MISC2_SET_REG1_OK_SHIFT       (14U)
#define CCM_ANALOG_MISC2_SET_REG1_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG1_OK_SHIFT)) & CCM_ANALOG_MISC2_SET_REG1_OK_MASK)
#define CCM_ANALOG_MISC2_SET_AUDIO_DIV_LSB_MASK  (0x8000U)
#define CCM_ANALOG_MISC2_SET_AUDIO_DIV_LSB_SHIFT (15U)
/*! AUDIO_DIV_LSB
 *  0b0..divide by 1 (Default)
 *  0b1..divide by 2
 */
#define CCM_ANALOG_MISC2_SET_AUDIO_DIV_LSB(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_AUDIO_DIV_LSB_SHIFT)) & CCM_ANALOG_MISC2_SET_AUDIO_DIV_LSB_MASK)
#define CCM_ANALOG_MISC2_SET_REG2_BO_OFFSET_MASK (0x70000U)
#define CCM_ANALOG_MISC2_SET_REG2_BO_OFFSET_SHIFT (16U)
/*! REG2_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_SET_REG2_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG2_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_SET_REG2_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_SET_REG2_BO_STATUS_MASK (0x80000U)
#define CCM_ANALOG_MISC2_SET_REG2_BO_STATUS_SHIFT (19U)
#define CCM_ANALOG_MISC2_SET_REG2_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG2_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_SET_REG2_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_SET_REG2_ENABLE_BO_MASK (0x200000U)
#define CCM_ANALOG_MISC2_SET_REG2_ENABLE_BO_SHIFT (21U)
#define CCM_ANALOG_MISC2_SET_REG2_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG2_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_SET_REG2_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_SET_REG2_OK_MASK        (0x400000U)
#define CCM_ANALOG_MISC2_SET_REG2_OK_SHIFT       (22U)
#define CCM_ANALOG_MISC2_SET_REG2_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG2_OK_SHIFT)) & CCM_ANALOG_MISC2_SET_REG2_OK_MASK)
#define CCM_ANALOG_MISC2_SET_AUDIO_DIV_MSB_MASK  (0x800000U)
#define CCM_ANALOG_MISC2_SET_AUDIO_DIV_MSB_SHIFT (23U)
/*! AUDIO_DIV_MSB
 *  0b0..divide by 1 (Default)
 *  0b1..divide by 2
 */
#define CCM_ANALOG_MISC2_SET_AUDIO_DIV_MSB(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_AUDIO_DIV_MSB_SHIFT)) & CCM_ANALOG_MISC2_SET_AUDIO_DIV_MSB_MASK)
#define CCM_ANALOG_MISC2_SET_REG0_STEP_TIME_MASK (0x3000000U)
#define CCM_ANALOG_MISC2_SET_REG0_STEP_TIME_SHIFT (24U)
/*! REG0_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_SET_REG0_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG0_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_SET_REG0_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_SET_REG1_STEP_TIME_MASK (0xC000000U)
#define CCM_ANALOG_MISC2_SET_REG1_STEP_TIME_SHIFT (26U)
/*! REG1_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_SET_REG1_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG1_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_SET_REG1_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_SET_REG2_STEP_TIME_MASK (0x30000000U)
#define CCM_ANALOG_MISC2_SET_REG2_STEP_TIME_SHIFT (28U)
/*! REG2_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_SET_REG2_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_REG2_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_SET_REG2_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_SET_VIDEO_DIV_MASK      (0xC0000000U)
#define CCM_ANALOG_MISC2_SET_VIDEO_DIV_SHIFT     (30U)
/*! VIDEO_DIV
 *  0b00..divide by 1 (Default)
 *  0b01..divide by 2
 *  0b10..divide by 1
 *  0b11..divide by 4
 */
#define CCM_ANALOG_MISC2_SET_VIDEO_DIV(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_VIDEO_DIV_SHIFT)) & CCM_ANALOG_MISC2_SET_VIDEO_DIV_MASK)
/*! @} */

/*! @name MISC2_CLR - Miscellaneous Register 2 */
/*! @{ */
#define CCM_ANALOG_MISC2_CLR_REG0_BO_OFFSET_MASK (0x7U)
#define CCM_ANALOG_MISC2_CLR_REG0_BO_OFFSET_SHIFT (0U)
/*! REG0_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_CLR_REG0_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG0_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG0_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_CLR_REG0_BO_STATUS_MASK (0x8U)
#define CCM_ANALOG_MISC2_CLR_REG0_BO_STATUS_SHIFT (3U)
/*! REG0_BO_STATUS
 *  0b1..Brownout, supply is below target minus brownout offset.
 */
#define CCM_ANALOG_MISC2_CLR_REG0_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG0_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG0_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_CLR_REG0_ENABLE_BO_MASK (0x20U)
#define CCM_ANALOG_MISC2_CLR_REG0_ENABLE_BO_SHIFT (5U)
#define CCM_ANALOG_MISC2_CLR_REG0_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG0_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG0_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_CLR_REG0_OK_MASK        (0x40U)
#define CCM_ANALOG_MISC2_CLR_REG0_OK_SHIFT       (6U)
#define CCM_ANALOG_MISC2_CLR_REG0_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG0_OK_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG0_OK_MASK)
#define CCM_ANALOG_MISC2_CLR_PLL3_DISABLE_MASK   (0x80U)
#define CCM_ANALOG_MISC2_CLR_PLL3_DISABLE_SHIFT  (7U)
/*! PLL3_DISABLE
 *  0b0..PLL3 is being used by peripherals and is enabled when SoC is not in any low power mode
 *  0b1..PLL3 can be disabled when the SoC is not in any low power mode
 */
#define CCM_ANALOG_MISC2_CLR_PLL3_DISABLE(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_PLL3_DISABLE_SHIFT)) & CCM_ANALOG_MISC2_CLR_PLL3_DISABLE_MASK)
#define CCM_ANALOG_MISC2_CLR_REG1_BO_OFFSET_MASK (0x700U)
#define CCM_ANALOG_MISC2_CLR_REG1_BO_OFFSET_SHIFT (8U)
/*! REG1_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_CLR_REG1_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG1_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG1_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_CLR_REG1_BO_STATUS_MASK (0x800U)
#define CCM_ANALOG_MISC2_CLR_REG1_BO_STATUS_SHIFT (11U)
/*! REG1_BO_STATUS
 *  0b1..Brownout, supply is below target minus brownout offset.
 */
#define CCM_ANALOG_MISC2_CLR_REG1_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG1_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG1_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_CLR_REG1_ENABLE_BO_MASK (0x2000U)
#define CCM_ANALOG_MISC2_CLR_REG1_ENABLE_BO_SHIFT (13U)
#define CCM_ANALOG_MISC2_CLR_REG1_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG1_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG1_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_CLR_REG1_OK_MASK        (0x4000U)
#define CCM_ANALOG_MISC2_CLR_REG1_OK_SHIFT       (14U)
#define CCM_ANALOG_MISC2_CLR_REG1_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG1_OK_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG1_OK_MASK)
#define CCM_ANALOG_MISC2_CLR_AUDIO_DIV_LSB_MASK  (0x8000U)
#define CCM_ANALOG_MISC2_CLR_AUDIO_DIV_LSB_SHIFT (15U)
/*! AUDIO_DIV_LSB
 *  0b0..divide by 1 (Default)
 *  0b1..divide by 2
 */
#define CCM_ANALOG_MISC2_CLR_AUDIO_DIV_LSB(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_AUDIO_DIV_LSB_SHIFT)) & CCM_ANALOG_MISC2_CLR_AUDIO_DIV_LSB_MASK)
#define CCM_ANALOG_MISC2_CLR_REG2_BO_OFFSET_MASK (0x70000U)
#define CCM_ANALOG_MISC2_CLR_REG2_BO_OFFSET_SHIFT (16U)
/*! REG2_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_CLR_REG2_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG2_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG2_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_CLR_REG2_BO_STATUS_MASK (0x80000U)
#define CCM_ANALOG_MISC2_CLR_REG2_BO_STATUS_SHIFT (19U)
#define CCM_ANALOG_MISC2_CLR_REG2_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG2_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG2_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_CLR_REG2_ENABLE_BO_MASK (0x200000U)
#define CCM_ANALOG_MISC2_CLR_REG2_ENABLE_BO_SHIFT (21U)
#define CCM_ANALOG_MISC2_CLR_REG2_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG2_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG2_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_CLR_REG2_OK_MASK        (0x400000U)
#define CCM_ANALOG_MISC2_CLR_REG2_OK_SHIFT       (22U)
#define CCM_ANALOG_MISC2_CLR_REG2_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG2_OK_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG2_OK_MASK)
#define CCM_ANALOG_MISC2_CLR_AUDIO_DIV_MSB_MASK  (0x800000U)
#define CCM_ANALOG_MISC2_CLR_AUDIO_DIV_MSB_SHIFT (23U)
/*! AUDIO_DIV_MSB
 *  0b0..divide by 1 (Default)
 *  0b1..divide by 2
 */
#define CCM_ANALOG_MISC2_CLR_AUDIO_DIV_MSB(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_AUDIO_DIV_MSB_SHIFT)) & CCM_ANALOG_MISC2_CLR_AUDIO_DIV_MSB_MASK)
#define CCM_ANALOG_MISC2_CLR_REG0_STEP_TIME_MASK (0x3000000U)
#define CCM_ANALOG_MISC2_CLR_REG0_STEP_TIME_SHIFT (24U)
/*! REG0_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_CLR_REG0_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG0_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG0_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_CLR_REG1_STEP_TIME_MASK (0xC000000U)
#define CCM_ANALOG_MISC2_CLR_REG1_STEP_TIME_SHIFT (26U)
/*! REG1_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_CLR_REG1_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG1_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG1_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_CLR_REG2_STEP_TIME_MASK (0x30000000U)
#define CCM_ANALOG_MISC2_CLR_REG2_STEP_TIME_SHIFT (28U)
/*! REG2_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_CLR_REG2_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_REG2_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_CLR_REG2_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_CLR_VIDEO_DIV_MASK      (0xC0000000U)
#define CCM_ANALOG_MISC2_CLR_VIDEO_DIV_SHIFT     (30U)
/*! VIDEO_DIV
 *  0b00..divide by 1 (Default)
 *  0b01..divide by 2
 *  0b10..divide by 1
 *  0b11..divide by 4
 */
#define CCM_ANALOG_MISC2_CLR_VIDEO_DIV(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_VIDEO_DIV_SHIFT)) & CCM_ANALOG_MISC2_CLR_VIDEO_DIV_MASK)
/*! @} */

/*! @name MISC2_TOG - Miscellaneous Register 2 */
/*! @{ */
#define CCM_ANALOG_MISC2_TOG_REG0_BO_OFFSET_MASK (0x7U)
#define CCM_ANALOG_MISC2_TOG_REG0_BO_OFFSET_SHIFT (0U)
/*! REG0_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_TOG_REG0_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG0_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG0_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_TOG_REG0_BO_STATUS_MASK (0x8U)
#define CCM_ANALOG_MISC2_TOG_REG0_BO_STATUS_SHIFT (3U)
/*! REG0_BO_STATUS
 *  0b1..Brownout, supply is below target minus brownout offset.
 */
#define CCM_ANALOG_MISC2_TOG_REG0_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG0_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG0_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_TOG_REG0_ENABLE_BO_MASK (0x20U)
#define CCM_ANALOG_MISC2_TOG_REG0_ENABLE_BO_SHIFT (5U)
#define CCM_ANALOG_MISC2_TOG_REG0_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG0_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG0_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_TOG_REG0_OK_MASK        (0x40U)
#define CCM_ANALOG_MISC2_TOG_REG0_OK_SHIFT       (6U)
#define CCM_ANALOG_MISC2_TOG_REG0_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG0_OK_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG0_OK_MASK)
#define CCM_ANALOG_MISC2_TOG_PLL3_DISABLE_MASK   (0x80U)
#define CCM_ANALOG_MISC2_TOG_PLL3_DISABLE_SHIFT  (7U)
/*! PLL3_DISABLE
 *  0b0..PLL3 is being used by peripherals and is enabled when SoC is not in any low power mode
 *  0b1..PLL3 can be disabled when the SoC is not in any low power mode
 */
#define CCM_ANALOG_MISC2_TOG_PLL3_DISABLE(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_PLL3_DISABLE_SHIFT)) & CCM_ANALOG_MISC2_TOG_PLL3_DISABLE_MASK)
#define CCM_ANALOG_MISC2_TOG_REG1_BO_OFFSET_MASK (0x700U)
#define CCM_ANALOG_MISC2_TOG_REG1_BO_OFFSET_SHIFT (8U)
/*! REG1_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_TOG_REG1_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG1_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG1_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_TOG_REG1_BO_STATUS_MASK (0x800U)
#define CCM_ANALOG_MISC2_TOG_REG1_BO_STATUS_SHIFT (11U)
/*! REG1_BO_STATUS
 *  0b1..Brownout, supply is below target minus brownout offset.
 */
#define CCM_ANALOG_MISC2_TOG_REG1_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG1_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG1_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_TOG_REG1_ENABLE_BO_MASK (0x2000U)
#define CCM_ANALOG_MISC2_TOG_REG1_ENABLE_BO_SHIFT (13U)
#define CCM_ANALOG_MISC2_TOG_REG1_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG1_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG1_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_TOG_REG1_OK_MASK        (0x4000U)
#define CCM_ANALOG_MISC2_TOG_REG1_OK_SHIFT       (14U)
#define CCM_ANALOG_MISC2_TOG_REG1_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG1_OK_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG1_OK_MASK)
#define CCM_ANALOG_MISC2_TOG_AUDIO_DIV_LSB_MASK  (0x8000U)
#define CCM_ANALOG_MISC2_TOG_AUDIO_DIV_LSB_SHIFT (15U)
/*! AUDIO_DIV_LSB
 *  0b0..divide by 1 (Default)
 *  0b1..divide by 2
 */
#define CCM_ANALOG_MISC2_TOG_AUDIO_DIV_LSB(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_AUDIO_DIV_LSB_SHIFT)) & CCM_ANALOG_MISC2_TOG_AUDIO_DIV_LSB_MASK)
#define CCM_ANALOG_MISC2_TOG_REG2_BO_OFFSET_MASK (0x70000U)
#define CCM_ANALOG_MISC2_TOG_REG2_BO_OFFSET_SHIFT (16U)
/*! REG2_BO_OFFSET
 *  0b100..Brownout offset = 0.100V
 *  0b111..Brownout offset = 0.175V
 */
#define CCM_ANALOG_MISC2_TOG_REG2_BO_OFFSET(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG2_BO_OFFSET_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG2_BO_OFFSET_MASK)
#define CCM_ANALOG_MISC2_TOG_REG2_BO_STATUS_MASK (0x80000U)
#define CCM_ANALOG_MISC2_TOG_REG2_BO_STATUS_SHIFT (19U)
#define CCM_ANALOG_MISC2_TOG_REG2_BO_STATUS(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG2_BO_STATUS_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG2_BO_STATUS_MASK)
#define CCM_ANALOG_MISC2_TOG_REG2_ENABLE_BO_MASK (0x200000U)
#define CCM_ANALOG_MISC2_TOG_REG2_ENABLE_BO_SHIFT (21U)
#define CCM_ANALOG_MISC2_TOG_REG2_ENABLE_BO(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG2_ENABLE_BO_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG2_ENABLE_BO_MASK)
#define CCM_ANALOG_MISC2_TOG_REG2_OK_MASK        (0x400000U)
#define CCM_ANALOG_MISC2_TOG_REG2_OK_SHIFT       (22U)
#define CCM_ANALOG_MISC2_TOG_REG2_OK(x)          (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG2_OK_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG2_OK_MASK)
#define CCM_ANALOG_MISC2_TOG_AUDIO_DIV_MSB_MASK  (0x800000U)
#define CCM_ANALOG_MISC2_TOG_AUDIO_DIV_MSB_SHIFT (23U)
/*! AUDIO_DIV_MSB
 *  0b0..divide by 1 (Default)
 *  0b1..divide by 2
 */
#define CCM_ANALOG_MISC2_TOG_AUDIO_DIV_MSB(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_AUDIO_DIV_MSB_SHIFT)) & CCM_ANALOG_MISC2_TOG_AUDIO_DIV_MSB_MASK)
#define CCM_ANALOG_MISC2_TOG_REG0_STEP_TIME_MASK (0x3000000U)
#define CCM_ANALOG_MISC2_TOG_REG0_STEP_TIME_SHIFT (24U)
/*! REG0_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_TOG_REG0_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG0_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG0_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_TOG_REG1_STEP_TIME_MASK (0xC000000U)
#define CCM_ANALOG_MISC2_TOG_REG1_STEP_TIME_SHIFT (26U)
/*! REG1_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_TOG_REG1_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG1_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG1_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_TOG_REG2_STEP_TIME_MASK (0x30000000U)
#define CCM_ANALOG_MISC2_TOG_REG2_STEP_TIME_SHIFT (28U)
/*! REG2_STEP_TIME
 *  0b00..64
 *  0b01..128
 *  0b10..256
 *  0b11..512
 */
#define CCM_ANALOG_MISC2_TOG_REG2_STEP_TIME(x)   (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_REG2_STEP_TIME_SHIFT)) & CCM_ANALOG_MISC2_TOG_REG2_STEP_TIME_MASK)
#define CCM_ANALOG_MISC2_TOG_VIDEO_DIV_MASK      (0xC0000000U)
#define CCM_ANALOG_MISC2_TOG_VIDEO_DIV_SHIFT     (30U)
/*! VIDEO_DIV
 *  0b00..divide by 1 (Default)
 *  0b01..divide by 2
 *  0b10..divide by 1
 *  0b11..divide by 4
 */
#define CCM_ANALOG_MISC2_TOG_VIDEO_DIV(x)        (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_VIDEO_DIV_SHIFT)) & CCM_ANALOG_MISC2_TOG_VIDEO_DIV_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group CCM_ANALOG_Register_Masks */


/* CCM_ANALOG - Peripheral instance base addresses */
/** Peripheral CCM_ANALOG base address */
#define CCM_ANALOG_BASE                          (0x400D8000u)
/** Peripheral CCM_ANALOG base pointer */
#define CCM_ANALOG                               ((CCM_ANALOG_Type *)CCM_ANALOG_BASE)
/** Array initializer of CCM_ANALOG peripheral base addresses */
#define CCM_ANALOG_BASE_ADDRS                    { CCM_ANALOG_BASE }
/** Array initializer of CCM_ANALOG peripheral base pointers */
#define CCM_ANALOG_BASE_PTRS                     { CCM_ANALOG }

/*!
 * @}
 */ /* end of group CCM_ANALOG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RTWDOG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTWDOG_Peripheral_Access_Layer RTWDOG Peripheral Access Layer
 * @{
 */

/** RTWDOG - Register Layout Typedef */
typedef struct {
  __IO uint32_t CS;                                /**< Watchdog Control and Status Register, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Watchdog Counter Register, offset: 0x4 */
  __IO uint32_t TOVAL;                             /**< Watchdog Timeout Value Register, offset: 0x8 */
  __IO uint32_t WIN;                               /**< Watchdog Window Register, offset: 0xC */
} RTWDOG_Type;

/* ----------------------------------------------------------------------------
   -- RTWDOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTWDOG_Register_Masks RTWDOG Register Masks
 * @{
 */

/*! @name CS - Watchdog Control and Status Register */
/*! @{ */
#define RTWDOG_CS_STOP_MASK                      (0x1U)
#define RTWDOG_CS_STOP_SHIFT                     (0U)
/*! STOP - Stop Enable
 *  0b0..Watchdog disabled in chip stop mode.
 *  0b1..Watchdog enabled in chip stop mode.
 */
#define RTWDOG_CS_STOP(x)                        (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_STOP_SHIFT)) & RTWDOG_CS_STOP_MASK)
#define RTWDOG_CS_WAIT_MASK                      (0x2U)
#define RTWDOG_CS_WAIT_SHIFT                     (1U)
/*! WAIT - Wait Enable
 *  0b0..Watchdog disabled in chip wait mode.
 *  0b1..Watchdog enabled in chip wait mode.
 */
#define RTWDOG_CS_WAIT(x)                        (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_WAIT_SHIFT)) & RTWDOG_CS_WAIT_MASK)
#define RTWDOG_CS_DBG_MASK                       (0x4U)
#define RTWDOG_CS_DBG_SHIFT                      (2U)
/*! DBG - Debug Enable
 *  0b0..Watchdog disabled in chip debug mode.
 *  0b1..Watchdog enabled in chip debug mode.
 */
#define RTWDOG_CS_DBG(x)                         (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_DBG_SHIFT)) & RTWDOG_CS_DBG_MASK)
#define RTWDOG_CS_TST_MASK                       (0x18U)
#define RTWDOG_CS_TST_SHIFT                      (3U)
/*! TST - Watchdog Test
 *  0b00..Watchdog test mode disabled.
 *  0b01..Watchdog user mode enabled. (Watchdog test mode disabled.) After testing the watchdog, software should
 *        use this setting to indicate that the watchdog is functioning normally in user mode.
 *  0b10..Watchdog test mode enabled, only the low byte is used. CNT[CNTLOW] is compared with TOVAL[TOVALLOW].
 *  0b11..Watchdog test mode enabled, only the high byte is used. CNT[CNTHIGH] is compared with TOVAL[TOVALHIGH].
 */
#define RTWDOG_CS_TST(x)                         (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_TST_SHIFT)) & RTWDOG_CS_TST_MASK)
#define RTWDOG_CS_UPDATE_MASK                    (0x20U)
#define RTWDOG_CS_UPDATE_SHIFT                   (5U)
/*! UPDATE - Allow updates
 *  0b0..Updates not allowed. After the initial configuration, the watchdog cannot be later modified without forcing a reset.
 *  0b1..Updates allowed. Software can modify the watchdog configuration registers within 128 bus clocks after performing the unlock write sequence.
 */
#define RTWDOG_CS_UPDATE(x)                      (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_UPDATE_SHIFT)) & RTWDOG_CS_UPDATE_MASK)
#define RTWDOG_CS_INT_MASK                       (0x40U)
#define RTWDOG_CS_INT_SHIFT                      (6U)
/*! INT - Watchdog Interrupt
 *  0b0..Watchdog interrupts are disabled. Watchdog resets are not delayed.
 *  0b1..Watchdog interrupts are enabled. Watchdog resets are delayed by 128 bus clocks from the interrupt vector fetch.
 */
#define RTWDOG_CS_INT(x)                         (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_INT_SHIFT)) & RTWDOG_CS_INT_MASK)
#define RTWDOG_CS_EN_MASK                        (0x80U)
#define RTWDOG_CS_EN_SHIFT                       (7U)
/*! EN - Watchdog Enable
 *  0b0..Watchdog disabled.
 *  0b1..Watchdog enabled.
 */
#define RTWDOG_CS_EN(x)                          (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_EN_SHIFT)) & RTWDOG_CS_EN_MASK)
#define RTWDOG_CS_CLK_MASK                       (0x300U)
#define RTWDOG_CS_CLK_SHIFT                      (8U)
/*! CLK - Watchdog Clock
 *  0b00..Bus clock
 *  0b01..LPO clock
 *  0b10..INTCLK (internal clock)
 *  0b11..ERCLK (external reference clock)
 */
#define RTWDOG_CS_CLK(x)                         (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_CLK_SHIFT)) & RTWDOG_CS_CLK_MASK)
#define RTWDOG_CS_RCS_MASK                       (0x400U)
#define RTWDOG_CS_RCS_SHIFT                      (10U)
/*! RCS - Reconfiguration Success
 *  0b0..Reconfiguring WDOG.
 *  0b1..Reconfiguration is successful.
 */
#define RTWDOG_CS_RCS(x)                         (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_RCS_SHIFT)) & RTWDOG_CS_RCS_MASK)
#define RTWDOG_CS_ULK_MASK                       (0x800U)
#define RTWDOG_CS_ULK_SHIFT                      (11U)
/*! ULK - Unlock status
 *  0b0..WDOG is locked.
 *  0b1..WDOG is unlocked.
 */
#define RTWDOG_CS_ULK(x)                         (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_ULK_SHIFT)) & RTWDOG_CS_ULK_MASK)
#define RTWDOG_CS_PRES_MASK                      (0x1000U)
#define RTWDOG_CS_PRES_SHIFT                     (12U)
/*! PRES - Watchdog prescaler
 *  0b0..256 prescaler disabled.
 *  0b1..256 prescaler enabled.
 */
#define RTWDOG_CS_PRES(x)                        (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_PRES_SHIFT)) & RTWDOG_CS_PRES_MASK)
#define RTWDOG_CS_CMD32EN_MASK                   (0x2000U)
#define RTWDOG_CS_CMD32EN_SHIFT                  (13U)
/*! CMD32EN - Enables or disables WDOG support for 32-bit (otherwise 16-bit or 8-bit) refresh/unlock command write words
 *  0b0..Disables support for 32-bit refresh/unlock command write words. Only 16-bit or 8-bit is supported.
 *  0b1..Enables support for 32-bit refresh/unlock command write words. 16-bit or 8-bit is NOT supported.
 */
#define RTWDOG_CS_CMD32EN(x)                     (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_CMD32EN_SHIFT)) & RTWDOG_CS_CMD32EN_MASK)
#define RTWDOG_CS_FLG_MASK                       (0x4000U)
#define RTWDOG_CS_FLG_SHIFT                      (14U)
/*! FLG - Watchdog Interrupt Flag
 *  0b0..No interrupt occurred.
 *  0b1..An interrupt occurred.
 */
#define RTWDOG_CS_FLG(x)                         (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_FLG_SHIFT)) & RTWDOG_CS_FLG_MASK)
#define RTWDOG_CS_WIN_MASK                       (0x8000U)
#define RTWDOG_CS_WIN_SHIFT                      (15U)
/*! WIN - Watchdog Window
 *  0b0..Window mode disabled.
 *  0b1..Window mode enabled.
 */
#define RTWDOG_CS_WIN(x)                         (((uint32_t)(((uint32_t)(x)) << RTWDOG_CS_WIN_SHIFT)) & RTWDOG_CS_WIN_MASK)
/*! @} */

/*! @name CNT - Watchdog Counter Register */
/*! @{ */
#define RTWDOG_CNT_CNTLOW_MASK                   (0xFFU)
#define RTWDOG_CNT_CNTLOW_SHIFT                  (0U)
/*! CNTLOW - Low byte of the Watchdog Counter
 */
#define RTWDOG_CNT_CNTLOW(x)                     (((uint32_t)(((uint32_t)(x)) << RTWDOG_CNT_CNTLOW_SHIFT)) & RTWDOG_CNT_CNTLOW_MASK)
#define RTWDOG_CNT_CNTHIGH_MASK                  (0xFF00U)
#define RTWDOG_CNT_CNTHIGH_SHIFT                 (8U)
/*! CNTHIGH - High byte of the Watchdog Counter
 */
#define RTWDOG_CNT_CNTHIGH(x)                    (((uint32_t)(((uint32_t)(x)) << RTWDOG_CNT_CNTHIGH_SHIFT)) & RTWDOG_CNT_CNTHIGH_MASK)
/*! @} */

/*! @name TOVAL - Watchdog Timeout Value Register */
/*! @{ */
#define RTWDOG_TOVAL_TOVALLOW_MASK               (0xFFU)
#define RTWDOG_TOVAL_TOVALLOW_SHIFT              (0U)
/*! TOVALLOW - Low byte of the timeout value
 */
#define RTWDOG_TOVAL_TOVALLOW(x)                 (((uint32_t)(((uint32_t)(x)) << RTWDOG_TOVAL_TOVALLOW_SHIFT)) & RTWDOG_TOVAL_TOVALLOW_MASK)
#define RTWDOG_TOVAL_TOVALHIGH_MASK              (0xFF00U)
#define RTWDOG_TOVAL_TOVALHIGH_SHIFT             (8U)
/*! TOVALHIGH - High byte of the timeout value
 */
#define RTWDOG_TOVAL_TOVALHIGH(x)                (((uint32_t)(((uint32_t)(x)) << RTWDOG_TOVAL_TOVALHIGH_SHIFT)) & RTWDOG_TOVAL_TOVALHIGH_MASK)
/*! @} */

/*! @name WIN - Watchdog Window Register */
/*! @{ */
#define RTWDOG_WIN_WINLOW_MASK                   (0xFFU)
#define RTWDOG_WIN_WINLOW_SHIFT                  (0U)
/*! WINLOW - Low byte of Watchdog Window
 */
#define RTWDOG_WIN_WINLOW(x)                     (((uint32_t)(((uint32_t)(x)) << RTWDOG_WIN_WINLOW_SHIFT)) & RTWDOG_WIN_WINLOW_MASK)
#define RTWDOG_WIN_WINHIGH_MASK                  (0xFF00U)
#define RTWDOG_WIN_WINHIGH_SHIFT                 (8U)
/*! WINHIGH - High byte of Watchdog Window
 */
#define RTWDOG_WIN_WINHIGH(x)                    (((uint32_t)(((uint32_t)(x)) << RTWDOG_WIN_WINHIGH_SHIFT)) & RTWDOG_WIN_WINHIGH_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group RTWDOG_Register_Masks */


/* RTWDOG - Peripheral instance base addresses */
/** Peripheral RTWDOG base address */
#define RTWDOG_BASE                              (0x400BC000u)
/** Peripheral RTWDOG base pointer */
#define RTWDOG                                   ((RTWDOG_Type *)RTWDOG_BASE)
/** Array initializer of RTWDOG peripheral base addresses */
#define RTWDOG_BASE_ADDRS                        { RTWDOG_BASE }
/** Array initializer of RTWDOG peripheral base pointers */
#define RTWDOG_BASE_PTRS                         { RTWDOG }
/** Interrupt vectors for the RTWDOG peripheral type */
#define RTWDOG_IRQS                              { RTWDOG_IRQn }
/* Extra definition */
#define RTWDOG_UPDATE_KEY                        (0xD928C520U)
#define RTWDOG_REFRESH_KEY                       (0xB480A602U)


/*!
 * @}
 */ /* end of group RTWDOG_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- WDOG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Peripheral_Access_Layer WDOG Peripheral Access Layer
 * @{
 */

/** WDOG - Register Layout Typedef */
typedef struct {
  __IO uint16_t WCR;                               /**< Watchdog Control Register, offset: 0x0 */
  __IO uint16_t WSR;                               /**< Watchdog Service Register, offset: 0x2 */
  __I  uint16_t WRSR;                              /**< Watchdog Reset Status Register, offset: 0x4 */
  __IO uint16_t WICR;                              /**< Watchdog Interrupt Control Register, offset: 0x6 */
  __IO uint16_t WMCR;                              /**< Watchdog Miscellaneous Control Register, offset: 0x8 */
} WDOG_Type;

/* ----------------------------------------------------------------------------
   -- WDOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Masks WDOG Register Masks
 * @{
 */

/*! @name WCR - Watchdog Control Register */
/*! @{ */
#define WDOG_WCR_WDZST_MASK                      (0x1U)
#define WDOG_WCR_WDZST_SHIFT                     (0U)
/*! WDZST
 *  0b0..Continue timer operation (Default).
 *  0b1..Suspend the watchdog timer.
 */
#define WDOG_WCR_WDZST(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDZST_SHIFT)) & WDOG_WCR_WDZST_MASK)
#define WDOG_WCR_WDBG_MASK                       (0x2U)
#define WDOG_WCR_WDBG_SHIFT                      (1U)
/*! WDBG
 *  0b0..Continue WDOG timer operation (Default).
 *  0b1..Suspend the watchdog timer.
 */
#define WDOG_WCR_WDBG(x)                         (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDBG_SHIFT)) & WDOG_WCR_WDBG_MASK)
#define WDOG_WCR_WDE_MASK                        (0x4U)
#define WDOG_WCR_WDE_SHIFT                       (2U)
/*! WDE
 *  0b0..Disable the Watchdog (Default).
 *  0b1..Enable the Watchdog.
 */
#define WDOG_WCR_WDE(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDE_SHIFT)) & WDOG_WCR_WDE_MASK)
#define WDOG_WCR_WDT_MASK                        (0x8U)
#define WDOG_WCR_WDT_SHIFT                       (3U)
/*! WDT
 *  0b0..No effect on WDOG_B (Default).
 *  0b1..Assert WDOG_B upon a Watchdog Time-out event.
 */
#define WDOG_WCR_WDT(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDT_SHIFT)) & WDOG_WCR_WDT_MASK)
#define WDOG_WCR_SRS_MASK                        (0x10U)
#define WDOG_WCR_SRS_SHIFT                       (4U)
/*! SRS
 *  0b0..Assert system reset signal.
 *  0b1..No effect on the system (Default).
 */
#define WDOG_WCR_SRS(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_SRS_SHIFT)) & WDOG_WCR_SRS_MASK)
#define WDOG_WCR_WDA_MASK                        (0x20U)
#define WDOG_WCR_WDA_SHIFT                       (5U)
/*! WDA
 *  0b0..Assert WDOG_B output.
 *  0b1..No effect on system (Default).
 */
#define WDOG_WCR_WDA(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDA_SHIFT)) & WDOG_WCR_WDA_MASK)
#define WDOG_WCR_SRE_MASK                        (0x40U)
#define WDOG_WCR_SRE_SHIFT                       (6U)
/*! SRE - software reset extension, an option way to generate software reset.
 *  0b0..using original way to generate software reset (default)
 *  0b1..using new way to generate software reset.
 */
#define WDOG_WCR_SRE(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_SRE_SHIFT)) & WDOG_WCR_SRE_MASK)
#define WDOG_WCR_WDW_MASK                        (0x80U)
#define WDOG_WCR_WDW_SHIFT                       (7U)
/*! WDW
 *  0b0..Continue WDOG timer operation (Default).
 *  0b1..Suspend WDOG timer operation.
 */
#define WDOG_WCR_WDW(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDW_SHIFT)) & WDOG_WCR_WDW_MASK)
#define WDOG_WCR_WT_MASK                         (0xFF00U)
#define WDOG_WCR_WT_SHIFT                        (8U)
/*! WT
 *  0b00000000..- 0.5 Seconds (Default).
 *  0b00000001..- 1.0 Seconds.
 *  0b00000010..- 1.5 Seconds.
 *  0b00000011..- 2.0 Seconds.
 *  0b11111111..- 128 Seconds.
 */
#define WDOG_WCR_WT(x)                           (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WT_SHIFT)) & WDOG_WCR_WT_MASK)
/*! @} */

/*! @name WSR - Watchdog Service Register */
/*! @{ */
#define WDOG_WSR_WSR_MASK                        (0xFFFFU)
#define WDOG_WSR_WSR_SHIFT                       (0U)
/*! WSR
 *  0b0101010101010101..Write to the Watchdog Service Register (WDOG_WSR).
 *  0b1010101010101010..Write to the Watchdog Service Register (WDOG_WSR).
 */
#define WDOG_WSR_WSR(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WSR_WSR_SHIFT)) & WDOG_WSR_WSR_MASK)
/*! @} */

/*! @name WRSR - Watchdog Reset Status Register */
/*! @{ */
#define WDOG_WRSR_SFTW_MASK                      (0x1U)
#define WDOG_WRSR_SFTW_SHIFT                     (0U)
/*! SFTW
 *  0b0..Reset is not the result of a software reset.
 *  0b1..Reset is the result of a software reset.
 */
#define WDOG_WRSR_SFTW(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WRSR_SFTW_SHIFT)) & WDOG_WRSR_SFTW_MASK)
#define WDOG_WRSR_TOUT_MASK                      (0x2U)
#define WDOG_WRSR_TOUT_SHIFT                     (1U)
/*! TOUT
 *  0b0..Reset is not the result of a WDOG timeout.
 *  0b1..Reset is the result of a WDOG timeout.
 */
#define WDOG_WRSR_TOUT(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WRSR_TOUT_SHIFT)) & WDOG_WRSR_TOUT_MASK)
#define WDOG_WRSR_POR_MASK                       (0x10U)
#define WDOG_WRSR_POR_SHIFT                      (4U)
/*! POR
 *  0b0..Reset is not the result of a power on reset.
 *  0b1..Reset is the result of a power on reset.
 */
#define WDOG_WRSR_POR(x)                         (((uint16_t)(((uint16_t)(x)) << WDOG_WRSR_POR_SHIFT)) & WDOG_WRSR_POR_MASK)
/*! @} */

/*! @name WICR - Watchdog Interrupt Control Register */
/*! @{ */
#define WDOG_WICR_WICT_MASK                      (0xFFU)
#define WDOG_WICR_WICT_SHIFT                     (0U)
/*! WICT
 *  0b00000000..WICT[7:0] = Time duration between interrupt and time-out is 0 seconds.
 *  0b00000001..WICT[7:0] = Time duration between interrupt and time-out is 0.5 seconds.
 *  0b00000100..WICT[7:0] = Time duration between interrupt and time-out is 2 seconds (Default).
 *  0b11111111..WICT[7:0] = Time duration between interrupt and time-out is 127.5 seconds.
 */
#define WDOG_WICR_WICT(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WICR_WICT_SHIFT)) & WDOG_WICR_WICT_MASK)
#define WDOG_WICR_WTIS_MASK                      (0x4000U)
#define WDOG_WICR_WTIS_SHIFT                     (14U)
/*! WTIS
 *  0b0..No interrupt has occurred (Default).
 *  0b1..Interrupt has occurred
 */
#define WDOG_WICR_WTIS(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WICR_WTIS_SHIFT)) & WDOG_WICR_WTIS_MASK)
#define WDOG_WICR_WIE_MASK                       (0x8000U)
#define WDOG_WICR_WIE_SHIFT                      (15U)
/*! WIE
 *  0b0..Disable Interrupt (Default).
 *  0b1..Enable Interrupt.
 */
#define WDOG_WICR_WIE(x)                         (((uint16_t)(((uint16_t)(x)) << WDOG_WICR_WIE_SHIFT)) & WDOG_WICR_WIE_MASK)
/*! @} */

/*! @name WMCR - Watchdog Miscellaneous Control Register */
/*! @{ */
#define WDOG_WMCR_PDE_MASK                       (0x1U)
#define WDOG_WMCR_PDE_SHIFT                      (0U)
/*! PDE
 *  0b0..Power Down Counter of WDOG is disabled.
 *  0b1..Power Down Counter of WDOG is enabled (Default).
 */
#define WDOG_WMCR_PDE(x)                         (((uint16_t)(((uint16_t)(x)) << WDOG_WMCR_PDE_SHIFT)) & WDOG_WMCR_PDE_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group WDOG_Register_Masks */


/* WDOG - Peripheral instance base addresses */
/** Peripheral WDOG1 base address */
#define WDOG1_BASE                               (0x400B8000u)
/** Peripheral WDOG1 base pointer */
#define WDOG1                                    ((WDOG_Type *)WDOG1_BASE)
/** Peripheral WDOG2 base address */
#define WDOG2_BASE                               (0x400D0000u)
/** Peripheral WDOG2 base pointer */
#define WDOG2                                    ((WDOG_Type *)WDOG2_BASE)
/** Array initializer of WDOG peripheral base addresses */
#define WDOG_BASE_ADDRS                          { 0u, WDOG1_BASE, WDOG2_BASE }
/** Array initializer of WDOG peripheral base pointers */
#define WDOG_BASE_PTRS                           { (WDOG_Type *)0u, WDOG1, WDOG2 }
/** Interrupt vectors for the WDOG peripheral type */
#define WDOG_IRQS                                { NotAvail_IRQn, WDOG1_IRQn, WDOG2_IRQn }

/*!
 * @}
 */ /* end of group WDOG_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
  #else
    #pragma pop
  #endif
#elif defined(__CWCC__)
  #pragma pop
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
   -- Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Bit_Field_Generic_Macros Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
 * @{
 */

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang system_header
  #endif
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma system_include
#endif

/**
 * @brief Mask and left-shift a bit field value for use in a register bit range.
 * @param field Name of the register bit field.
 * @param value Value of the bit field.
 * @return Masked and shifted value.
 */
#define NXP_VAL2FLD(field, value)    (((value) << (field ## _SHIFT)) & (field ## _MASK))
/**
 * @brief Mask and right-shift a register value to extract a bit field value.
 * @param field Name of the register bit field.
 * @param value Value of the register.
 * @return Masked and shifted bit field value.
 */
#define NXP_FLD2VAL(field, value)    (((value) & (field ## _MASK)) >> (field ## _SHIFT))

/*!
 * @}
 */ /* end of group Bit_Field_Generic_Macros */


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


#endif  /* _MIMXRT1062_H_ */

