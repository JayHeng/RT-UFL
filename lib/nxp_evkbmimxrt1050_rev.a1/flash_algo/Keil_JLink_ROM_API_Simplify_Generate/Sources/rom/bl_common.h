/*
 * Copyright (c) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BL_COMMON_H__
#define __BL_COMMON_H__

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 168                /**< Number of interrupts in the Vector table */

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M7 SV Hard Fault Interrupt */
  MemoryManagement_IRQn        = -12,              /**< Cortex-M7 Memory Management Interrupt */
  BusFault_IRQn                = -11,              /**< Cortex-M7 Bus Fault Interrupt */
  UsageFault_IRQn              = -10,              /**< Cortex-M7 Usage Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M7 SV Call Interrupt */
  DebugMonitor_IRQn            = -4,               /**< Cortex-M7 Debug Monitor Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M7 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M7 System Tick Interrupt */

  /* Device specific interrupts */
  DMA0_DMA16_IRQn              = 0,                /**< DMA channel 0/16 transfer complete */
  DMA1_DMA17_IRQn              = 1,                /**< DMA channel 1/17 transfer complete */
  DMA2_DMA18_IRQn              = 2,                /**< DMA channel 2/18 transfer complete */
  DMA3_DMA19_IRQn              = 3,                /**< DMA channel 3/19 transfer complete */
  DMA4_DMA20_IRQn              = 4,                /**< DMA channel 4/20 transfer complete */
  DMA5_DMA21_IRQn              = 5,                /**< DMA channel 5/21 transfer complete */
  DMA6_DMA22_IRQn              = 6,                /**< DMA channel 6/22 transfer complete */
  DMA7_DMA23_IRQn              = 7,                /**< DMA channel 7/23 transfer complete */
  DMA8_DMA24_IRQn              = 8,                /**< DMA channel 8/24 transfer complete */
  DMA9_DMA25_IRQn              = 9,                /**< DMA channel 9/25 transfer complete */
  DMA10_DMA26_IRQn             = 10,               /**< DMA channel 10/26 transfer complete */
  DMA11_DMA27_IRQn             = 11,               /**< DMA channel 11/27 transfer complete */
  DMA12_DMA28_IRQn             = 12,               /**< DMA channel 12/28 transfer complete */
  DMA13_DMA29_IRQn             = 13,               /**< DMA channel 13/29 transfer complete */
  DMA14_DMA30_IRQn             = 14,               /**< DMA channel 14/30 transfer complete */
  DMA15_DMA31_IRQn             = 15,               /**< DMA channel 15/31 transfer complete */
  DMA_ERROR_IRQn               = 16,               /**< DMA error interrupt channels 0-15 / 16-31 */
  CTI0_ERROR_IRQn              = 17,               /**< CTI0_Error */
  CTI1_ERROR_IRQn              = 18,               /**< CTI1_Error */
  CORE_IRQn                    = 19,               /**< CorePlatform exception IRQ */
  LPUART1_IRQn                 = 20,               /**< LPUART1 TX interrupt and RX interrupt */
  LPUART2_IRQn                 = 21,               /**< LPUART2 TX interrupt and RX interrupt */
  LPUART3_IRQn                 = 22,               /**< LPUART3 TX interrupt and RX interrupt */
  LPUART4_IRQn                 = 23,               /**< LPUART4 TX interrupt and RX interrupt */
  LPUART5_IRQn                 = 24,               /**< LPUART5 TX interrupt and RX interrupt */
  LPUART6_IRQn                 = 25,               /**< LPUART6 TX interrupt and RX interrupt */
  LPUART7_IRQn                 = 26,               /**< LPUART7 TX interrupt and RX interrupt */
  LPUART8_IRQn                 = 27,               /**< LPUART8 TX interrupt and RX interrupt */
  LPI2C1_IRQn                  = 28,               /**< LPI2C1 interrupt */
  LPI2C2_IRQn                  = 29,               /**< LPI2C2 interrupt */
  LPI2C3_IRQn                  = 30,               /**< LPI2C3 interrupt */
  LPI2C4_IRQn                  = 31,               /**< LPI2C4 interrupt */
  LPSPI1_IRQn                  = 32,               /**< LPSPI1 single interrupt vector for all sources */
  LPSPI2_IRQn                  = 33,               /**< LPSPI2 single interrupt vector for all sources */
  LPSPI3_IRQn                  = 34,               /**< LPSPI3 single interrupt vector for all sources */
  LPSPI4_IRQn                  = 35,               /**< LPSPI4  single interrupt vector for all sources */
  CAN1_IRQn                    = 36,               /**< CAN1 interrupt */
  CAN2_IRQn                    = 37,               /**< CAN2 interrupt */
  FLEXRAM_IRQn                 = 38,               /**< FlexRAM address out of range Or access hit IRQ */
  KPP_IRQn                     = 39,               /**< Keypad nterrupt */
  TSC_DIG_IRQn                 = 40,               /**< TSC interrupt */
  GPR_IRQ_IRQn                 = 41,               /**< GPR interrupt */
  LCDIF_IRQn                   = 42,               /**< LCDIF interrupt */
  CSI_IRQn                     = 43,               /**< CSI interrupt */
  PXP_IRQn                     = 44,               /**< PXP interrupt */
  WDOG2_IRQn                   = 45,               /**< WDOG2 interrupt */
  SNVS_HP_WRAPPER_IRQn         = 46,               /**< SRTC Consolidated Interrupt. Non TZ */
  SNVS_HP_WRAPPER_TZ_IRQn      = 47,               /**< SRTC Security Interrupt. TZ */
  SNVS_LP_WRAPPER_IRQn         = 48,               /**< ON-OFF button press shorter than 5 secs (pulse event) */
  CSU_IRQn                     = 49,               /**< CSU interrupt */
  DCP_IRQn                     = 50,               /**< DCP_IRQ interrupt */
  DCP_VMI_IRQn                 = 51,               /**< DCP_VMI_IRQ interrupt */
  Reserved68_IRQn              = 52,               /**< Reserved interrupt */
  TRNG_IRQn                    = 53,               /**< TRNG interrupt */
  SJC_IRQn                     = 54,               /**< SJC interrupt */
  BEE_IRQn                     = 55,               /**< BEE interrupt */
  SAI1_IRQn                    = 56,               /**< SAI1 interrupt */
  SAI2_IRQn                    = 57,               /**< SAI1 interrupt */
  SAI3_RX_IRQn                 = 58,               /**< SAI3 interrupt */
  SAI3_TX_IRQn                 = 59,               /**< SAI3 interrupt */
  SPDIF_IRQn                   = 60,               /**< SPDIF interrupt */
  PMU_EVENT_IRQn               = 61,               /**< Brown-out event interrupt */
  Reserved78_IRQn              = 62,               /**< Reserved interrupt */
  TEMP_LOW_HIGH_IRQn           = 63,               /**< TempSensor low/high interrupt */
  TEMP_PANIC_IRQn              = 64,               /**< TempSensor panic interrupt */
  USB_PHY1_IRQn                = 65,               /**< USBPHY (UTMI0), Interrupt */
  USB_PHY2_IRQn                = 66,               /**< USBPHY (UTMI0), Interrupt */
  ADC1_IRQn                    = 67,               /**< ADC1 interrupt */
  ADC2_IRQn                    = 68,               /**< ADC2 interrupt */
  DCDC_IRQn                    = 69,               /**< DCDC interrupt */
  Reserved86_IRQn              = 70,               /**< Reserved interrupt */
  Reserved87_IRQn              = 71,               /**< Reserved interrupt */
  GPIO1_INT0_IRQn              = 72,               /**< Active HIGH Interrupt from INT0 from GPIO */
  GPIO1_INT1_IRQn              = 73,               /**< Active HIGH Interrupt from INT1 from GPIO */
  GPIO1_INT2_IRQn              = 74,               /**< Active HIGH Interrupt from INT2 from GPIO */
  GPIO1_INT3_IRQn              = 75,               /**< Active HIGH Interrupt from INT3 from GPIO */
  GPIO1_INT4_IRQn              = 76,               /**< Active HIGH Interrupt from INT4 from GPIO */
  GPIO1_INT5_IRQn              = 77,               /**< Active HIGH Interrupt from INT5 from GPIO */
  GPIO1_INT6_IRQn              = 78,               /**< Active HIGH Interrupt from INT6 from GPIO */
  GPIO1_INT7_IRQn              = 79,               /**< Active HIGH Interrupt from INT7 from GPIO */
  GPIO1_Combined_0_15_IRQn     = 80,               /**< Combined interrupt indication for GPIO1 signal 0 throughout 15 */
  GPIO1_Combined_16_31_IRQn    = 81,               /**< Combined interrupt indication for GPIO1 signal 16 throughout 31 */
  GPIO2_Combined_0_15_IRQn     = 82,               /**< Combined interrupt indication for GPIO2 signal 0 throughout 15 */
  GPIO2_Combined_16_31_IRQn    = 83,               /**< Combined interrupt indication for GPIO2 signal 16 throughout 31 */
  GPIO3_Combined_0_15_IRQn     = 84,               /**< Combined interrupt indication for GPIO3 signal 0 throughout 15 */
  GPIO3_Combined_16_31_IRQn    = 85,               /**< Combined interrupt indication for GPIO3 signal 16 throughout 31 */
  GPIO4_Combined_0_15_IRQn     = 86,               /**< Combined interrupt indication for GPIO4 signal 0 throughout 15 */
  GPIO4_Combined_16_31_IRQn    = 87,               /**< Combined interrupt indication for GPIO4 signal 16 throughout 31 */
  GPIO5_Combined_0_15_IRQn     = 88,               /**< Combined interrupt indication for GPIO5 signal 0 throughout 15 */
  GPIO5_Combined_16_31_IRQn    = 89,               /**< Combined interrupt indication for GPIO5 signal 16 throughout 31 */
  FLEXIO1_IRQn                 = 90,               /**< FLEXIO1 interrupt */
  FLEXIO2_IRQn                 = 91,               /**< FLEXIO2 interrupt */
  WDOG1_IRQn                   = 92,               /**< WDOG1 interrupt */
  RTWDOG_IRQn                  = 93,               /**< RTWDOG interrupt */
  EWM_IRQn                     = 94,               /**< EWM interrupt */
  CCM_1_IRQn                   = 95,               /**< CCM IRQ1 interrupt */
  CCM_2_IRQn                   = 96,               /**< CCM IRQ2 interrupt */
  GPC_IRQn                     = 97,               /**< GPC interrupt */
  SRC_IRQn                     = 98,               /**< SRC interrupt */
  Reserved115_IRQn             = 99,               /**< Reserved interrupt */
  GPT1_IRQn                    = 100,              /**< GPT1 interrupt */
  GPT2_IRQn                    = 101,              /**< GPT2 interrupt */
  PWM1_0_IRQn                  = 102,              /**< PWM1 capture 0, compare 0, or reload 0 interrupt */
  PWM1_1_IRQn                  = 103,              /**< PWM1 capture 1, compare 1, or reload 0 interrupt */
  PWM1_2_IRQn                  = 104,              /**< PWM1 capture 2, compare 2, or reload 0 interrupt */
  PWM1_3_IRQn                  = 105,              /**< PWM1 capture 3, compare 3, or reload 0 interrupt */
  PWM1_FAULT_IRQn              = 106,              /**< PWM1 fault or reload error interrupt */
  Reserved123_IRQn             = 107,              /**< Reserved interrupt */
  FLEXSPI_IRQn                 = 108,              /**< FlexSPI0 interrupt */
  SEMC_IRQn                    = 109,              /**< Reserved interrupt */
  USDHC1_IRQn                  = 110,              /**< USDHC1 interrupt */
  USDHC2_IRQn                  = 111,              /**< USDHC2 interrupt */
  USB_OTG2_IRQn                = 112,              /**< USBO2 USB OTG2 */
  USB_OTG1_IRQn                = 113,              /**< USBO2 USB OTG1 */
  ENET_IRQn                    = 114,              /**< ENET interrupt */
  ENET_1588_Timer_IRQn         = 115,              /**< ENET_1588_Timer interrupt */
  XBAR1_IRQ_0_1_IRQn           = 116,              /**< XBAR1 interrupt */
  XBAR1_IRQ_2_3_IRQn           = 117,              /**< XBAR1 interrupt */
  ADC_ETC_IRQ0_IRQn            = 118,              /**< ADCETC IRQ0 interrupt */
  ADC_ETC_IRQ1_IRQn            = 119,              /**< ADCETC IRQ1 interrupt */
  ADC_ETC_IRQ2_IRQn            = 120,              /**< ADCETC IRQ2 interrupt */
  ADC_ETC_ERROR_IRQ_IRQn       = 121,              /**< ADCETC Error IRQ interrupt */
  PIT_IRQn                     = 122,              /**< PIT interrupt */
  ACMP1_IRQn                   = 123,              /**< ACMP interrupt */
  ACMP2_IRQn                   = 124,              /**< ACMP interrupt */
  ACMP3_IRQn                   = 125,              /**< ACMP interrupt */
  ACMP4_IRQn                   = 126,              /**< ACMP interrupt */
  Reserved143_IRQn             = 127,              /**< Reserved interrupt */
  Reserved144_IRQn             = 128,              /**< Reserved interrupt */
  ENC1_IRQn                    = 129,              /**< ENC1 interrupt */
  ENC2_IRQn                    = 130,              /**< ENC2 interrupt */
  ENC3_IRQn                    = 131,              /**< ENC3 interrupt */
  ENC4_IRQn                    = 132,              /**< ENC4 interrupt */
  TMR1_IRQn                    = 133,              /**< TMR1 interrupt */
  TMR2_IRQn                    = 134,              /**< TMR2 interrupt */
  TMR3_IRQn                    = 135,              /**< TMR3 interrupt */
  TMR4_IRQn                    = 136,              /**< TMR4 interrupt */
  PWM2_0_IRQn                  = 137,              /**< PWM2 capture 0, compare 0, or reload 0 interrupt */
  PWM2_1_IRQn                  = 138,              /**< PWM2 capture 1, compare 1, or reload 0 interrupt */
  PWM2_2_IRQn                  = 139,              /**< PWM2 capture 2, compare 2, or reload 0 interrupt */
  PWM2_3_IRQn                  = 140,              /**< PWM2 capture 3, compare 3, or reload 0 interrupt */
  PWM2_FAULT_IRQn              = 141,              /**< PWM2 fault or reload error interrupt */
  PWM3_0_IRQn                  = 142,              /**< PWM3 capture 0, compare 0, or reload 0 interrupt */
  PWM3_1_IRQn                  = 143,              /**< PWM3 capture 1, compare 1, or reload 0 interrupt */
  PWM3_2_IRQn                  = 144,              /**< PWM3 capture 2, compare 2, or reload 0 interrupt */
  PWM3_3_IRQn                  = 145,              /**< PWM3 capture 3, compare 3, or reload 0 interrupt */
  PWM3_FAULT_IRQn              = 146,              /**< PWM3 fault or reload error interrupt */
  PWM4_0_IRQn                  = 147,              /**< PWM4 capture 0, compare 0, or reload 0 interrupt */
  PWM4_1_IRQn                  = 148,              /**< PWM4 capture 1, compare 1, or reload 0 interrupt */
  PWM4_2_IRQn                  = 149,              /**< PWM4 capture 2, compare 2, or reload 0 interrupt */
  PWM4_3_IRQn                  = 150,              /**< PWM4 capture 3, compare 3, or reload 0 interrupt */
  PWM4_FAULT_IRQn              = 151               /**< PWM4 fault or reload error interrupt */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */


/* ----------------------------------------------------------------------------
   -- Cortex M7 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M7 Core Configuration
 * @{
 */

#define __MPU_PRESENT                  1         /**< Defines if an MPU is present or not */
#define __ICACHE_PRESENT               1         /**< Defines if an ICACHE is present or not */
#define __DCACHE_PRESENT               1         /**< Defines if an DCACHE is present or not */
#define __DTCM_PRESENT                 1         /**< Defines if an DTCM is present or not */
#define __NVIC_PRIO_BITS               4         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig         0         /**< Vendor specific implementation of SysTickConfig is defined */
#define __FPU_PRESENT                  1         /**< Defines if an FPU is present or not */

#include "core_cm7.h"                  /* Core Peripheral Access Layer */

/*!
 * @}
 */ /* end of group Cortex_Core_Configuration */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Construct a status code value from a group and code number. */
#define MAKE_STATUS(group, code) ((((group)*100) + (code)))

/*! @brief Construct the version number for drivers. */
#define MAKE_VERSION(major, minor, bugfix) (((major) << 16) | ((minor) << 8) | (bugfix))

/*! @name Driver version */
/*@{*/
/*! @brief common driver version 2.2.4. */
#define FSL_COMMON_DRIVER_VERSION (MAKE_VERSION(2, 2, 4))
/*@}*/

/* Debug console type definition. */
#define DEBUG_CONSOLE_DEVICE_TYPE_NONE          0U      /*!< No debug console.             */
#define DEBUG_CONSOLE_DEVICE_TYPE_UART          1U      /*!< Debug console based on UART.   */
#define DEBUG_CONSOLE_DEVICE_TYPE_LPUART        2U      /*!< Debug console based on LPUART. */
#define DEBUG_CONSOLE_DEVICE_TYPE_LPSCI         3U      /*!< Debug console based on LPSCI.  */
#define DEBUG_CONSOLE_DEVICE_TYPE_USBCDC        4U      /*!< Debug console based on USBCDC. */
#define DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM      5U      /*!< Debug console based on FLEXCOMM. */
#define DEBUG_CONSOLE_DEVICE_TYPE_IUART         6U      /*!< Debug console based on i.MX UART. */
#define DEBUG_CONSOLE_DEVICE_TYPE_VUSART        7U      /*!< Debug console based on LPC_VUSART. */
#define DEBUG_CONSOLE_DEVICE_TYPE_MINI_USART    8U      /*!< Debug console based on LPC_USART. */
#define DEBUG_CONSOLE_DEVICE_TYPE_SWO           9U      /*!< Debug console based on SWO. */

/*! @brief Status group numbers. */
enum _status_groups
{
    kStatusGroup_Generic = 0,                 /*!< Group number for generic status codes. */
    kStatusGroup_FLASH = 1,                   /*!< Group number for FLASH status codes. */
    kStatusGroup_LPSPI = 4,                   /*!< Group number for LPSPI status codes. */
    kStatusGroup_FLEXIO_SPI = 5,              /*!< Group number for FLEXIO SPI status codes. */
    kStatusGroup_DSPI = 6,                    /*!< Group number for DSPI status codes. */
    kStatusGroup_FLEXIO_UART = 7,             /*!< Group number for FLEXIO UART status codes. */
    kStatusGroup_FLEXIO_I2C = 8,              /*!< Group number for FLEXIO I2C status codes. */
    kStatusGroup_LPI2C = 9,                   /*!< Group number for LPI2C status codes. */
    kStatusGroup_UART = 10,                   /*!< Group number for UART status codes. */
    kStatusGroup_I2C = 11,                    /*!< Group number for UART status codes. */
    kStatusGroup_LPSCI = 12,                  /*!< Group number for LPSCI status codes. */
    kStatusGroup_LPUART = 13,                 /*!< Group number for LPUART status codes. */
    kStatusGroup_SPI = 14,                    /*!< Group number for SPI status code.*/
    kStatusGroup_XRDC = 15,                   /*!< Group number for XRDC status code.*/
    kStatusGroup_SEMA42 = 16,                 /*!< Group number for SEMA42 status code.*/
    kStatusGroup_SDHC = 17,                   /*!< Group number for SDHC status code */
    kStatusGroup_SDMMC = 18,                  /*!< Group number for SDMMC status code */
    kStatusGroup_SAI = 19,                    /*!< Group number for SAI status code */
    kStatusGroup_MCG = 20,                    /*!< Group number for MCG status codes. */
    kStatusGroup_SCG = 21,                    /*!< Group number for SCG status codes. */
    kStatusGroup_SDSPI = 22,                  /*!< Group number for SDSPI status codes. */
    kStatusGroup_FLEXIO_I2S = 23,             /*!< Group number for FLEXIO I2S status codes */
    kStatusGroup_FLEXIO_MCULCD = 24,          /*!< Group number for FLEXIO LCD status codes */
    kStatusGroup_FLASHIAP = 25,               /*!< Group number for FLASHIAP status codes */
    kStatusGroup_FLEXCOMM_I2C = 26,           /*!< Group number for FLEXCOMM I2C status codes */
    kStatusGroup_I2S = 27,                    /*!< Group number for I2S status codes */
    kStatusGroup_IUART = 28,                  /*!< Group number for IUART status codes */
    kStatusGroup_CSI = 29,                    /*!< Group number for CSI status codes */
    kStatusGroup_MIPI_DSI = 30,               /*!< Group number for MIPI DSI status codes */
    kStatusGroup_SDRAMC = 35,                 /*!< Group number for SDRAMC status codes. */
    kStatusGroup_POWER = 39,                  /*!< Group number for POWER status codes. */
    kStatusGroup_ENET = 40,                   /*!< Group number for ENET status codes. */
    kStatusGroup_PHY = 41,                    /*!< Group number for PHY status codes. */
    kStatusGroup_TRGMUX = 42,                 /*!< Group number for TRGMUX status codes. */
    kStatusGroup_SMARTCARD = 43,              /*!< Group number for SMARTCARD status codes. */
    kStatusGroup_LMEM = 44,                   /*!< Group number for LMEM status codes. */
    kStatusGroup_QSPI = 45,                   /*!< Group number for QSPI status codes. */
    kStatusGroup_DMA = 50,                    /*!< Group number for DMA status codes. */
    kStatusGroup_EDMA = 51,                   /*!< Group number for EDMA status codes. */
    kStatusGroup_DMAMGR = 52,                 /*!< Group number for DMAMGR status codes. */
    kStatusGroup_FLEXCAN = 53,                /*!< Group number for FlexCAN status codes. */
    kStatusGroup_LTC = 54,                    /*!< Group number for LTC status codes. */
    kStatusGroup_FLEXIO_CAMERA = 55,          /*!< Group number for FLEXIO CAMERA status codes. */
    kStatusGroup_LPC_SPI = 56,                /*!< Group number for LPC_SPI status codes. */
    kStatusGroup_LPC_USART = 57,              /*!< Group number for LPC_USART status codes. */
    kStatusGroup_DMIC = 58,                   /*!< Group number for DMIC status codes. */
    kStatusGroup_SDIF = 59,                   /*!< Group number for SDIF status codes.*/
    kStatusGroup_SPIFI = 60,                  /*!< Group number for SPIFI status codes. */
    kStatusGroup_OTP = 61,                    /*!< Group number for OTP status codes. */
    kStatusGroup_MCAN = 62,                   /*!< Group number for MCAN status codes. */
    kStatusGroup_CAAM = 63,                   /*!< Group number for CAAM status codes. */
    kStatusGroup_ECSPI = 64,                  /*!< Group number for ECSPI status codes. */
    kStatusGroup_USDHC = 65,                  /*!< Group number for USDHC status codes.*/
    kStatusGroup_LPC_I2C = 66,                /*!< Group number for LPC_I2C status codes.*/
    kStatusGroup_DCP = 67,                    /*!< Group number for DCP status codes.*/
    kStatusGroup_MSCAN = 68,                  /*!< Group number for MSCAN status codes.*/
    kStatusGroup_ESAI = 69,                   /*!< Group number for ESAI status codes. */
    kStatusGroup_FLEXSPI = 70,                /*!< Group number for FLEXSPI status codes. */
    kStatusGroup_MMDC = 71,                   /*!< Group number for MMDC status codes. */
    kStatusGroup_PDM = 72,                    /*!< Group number for MIC status codes. */
    kStatusGroup_SDMA = 73,                   /*!< Group number for SDMA status codes. */
    kStatusGroup_ICS = 74,                    /*!< Group number for ICS status codes. */
    kStatusGroup_SPDIF = 75,                  /*!< Group number for SPDIF status codes. */
    kStatusGroup_LPC_MINISPI = 76,            /*!< Group number for LPC_MINISPI status codes. */
    kStatusGroup_HASHCRYPT = 77,              /*!< Group number for Hashcrypt status codes */
    kStatusGroup_LPC_SPI_SSP = 78,            /*!< Group number for LPC_SPI_SSP status codes. */
    kStatusGroup_I3C = 79,                    /*!< Group number for I3C status codes */
    kStatusGroup_LPC_I2C_1 = 97,              /*!< Group number for LPC_I2C_1 status codes. */
    kStatusGroup_NOTIFIER = 98,               /*!< Group number for NOTIFIER status codes. */
    kStatusGroup_DebugConsole = 99,           /*!< Group number for debug console status codes. */
    kStatusGroup_SEMC = 100,                  /*!< Group number for SEMC status codes. */
    kStatusGroup_ApplicationRangeStart = 101, /*!< Starting number for application groups. */
    kStatusGroup_IAP = 102,                   /*!< Group number for IAP status codes */
    kStatusGroup_SFA = 103,                   /*!< Group number for SFA status codes*/
    kStatusGroup_SPC = 104,                   /*!< Group number for SPC status codes. */
    kStatusGroup_PUF = 105,                   /*!< Group number for PUF status codes. */

    kStatusGroup_HAL_GPIO = 121,              /*!< Group number for HAL GPIO status codes. */
    kStatusGroup_HAL_UART = 122,              /*!< Group number for HAL UART status codes. */
    kStatusGroup_HAL_TIMER = 123,             /*!< Group number for HAL TIMER status codes. */
    kStatusGroup_HAL_SPI = 124,               /*!< Group number for HAL SPI status codes. */
    kStatusGroup_HAL_I2C = 125,               /*!< Group number for HAL I2C status codes. */
    kStatusGroup_HAL_FLASH = 126,             /*!< Group number for HAL FLASH status codes. */
    kStatusGroup_HAL_PWM = 127,               /*!< Group number for HAL PWM status codes. */
    kStatusGroup_HAL_RNG = 128,               /*!< Group number for HAL RNG status codes. */
    kStatusGroup_TIMERMANAGER = 135,          /*!< Group number for TiMER MANAGER status codes. */
    kStatusGroup_SERIALMANAGER = 136,         /*!< Group number for SERIAL MANAGER status codes. */
    kStatusGroup_LED = 137,                   /*!< Group number for LED status codes. */
    kStatusGroup_BUTTON = 138,                /*!< Group number for BUTTON status codes. */
    kStatusGroup_EXTERN_EEPROM = 139,         /*!< Group number for EXTERN EEPROM status codes. */
    kStatusGroup_SHELL = 140,                 /*!< Group number for SHELL status codes. */
    kStatusGroup_MEM_MANAGER = 141,           /*!< Group number for MEM MANAGER status codes. */
    kStatusGroup_LIST = 142,                  /*!< Group number for List status codes. */
    kStatusGroup_OSA = 143,                   /*!< Group number for OSA status codes. */
    kStatusGroup_COMMON_TASK = 144,           /*!< Group number for Common task status codes. */
    kStatusGroup_MSG = 145,                   /*!< Group number for messaging status codes. */
    kStatusGroup_SDK_OCOTP = 146,             /*!< Group number for OCOTP status codes. */
    kStatusGroup_SDK_FLEXSPINOR = 147,        /*!< Group number for FLEXSPINOR status codes.*/
    kStatusGroup_CODEC = 148,                 /*!< Group number for codec status codes. */
    kStatusGroup_ASRC = 149,                 /*!< Group number for codec status ASRC. */
    kStatusGroup_OTFAD = 150,                 /*!< Group number for codec status codes. */
    kStatusGroup_SDIOSLV = 151,                 /*!< Group number for SDIOSLV status codes. */

};

/*! @brief Generic status return codes. */
enum
{
    kStatus_Success = MAKE_STATUS(kStatusGroup_Generic, 0),
    kStatus_Fail = MAKE_STATUS(kStatusGroup_Generic, 1),
    kStatus_ReadOnly = MAKE_STATUS(kStatusGroup_Generic, 2),
    kStatus_OutOfRange = MAKE_STATUS(kStatusGroup_Generic, 3),
    kStatus_InvalidArgument = MAKE_STATUS(kStatusGroup_Generic, 4),
    kStatus_Timeout = MAKE_STATUS(kStatusGroup_Generic, 5),
    kStatus_NoTransferInProgress = MAKE_STATUS(kStatusGroup_Generic, 6),
};

/*! @brief Type used for all status and error return values. */
typedef int32_t status_t;


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
//! @name Alignment macros
//@{
#ifndef ALIGN_DOWN
#define ALIGN_DOWN(x, a) ((x) & -(a))
#endif
#ifndef ALIGN_UP
#define ALIGN_UP(x, a) (-(-(x) & -(a)))
#endif
//@}

//! @brief Bootloader status group numbers.
//!
//! @ingroup bl_core
enum _bl_status_groups
{
    kStatusGroup_Bootloader = 100,      //!< Bootloader status group number (100).
    kStatusGroup_SBLoader = 101,        //!< SB loader status group number (101).
    kStatusGroup_MemoryInterface = 102, //!< Memory interface status group number (102).
    kStatusGroup_PropertyStore = 103,   //!< Property store status group number (103).
    kStatusGroup_AppCrcCheck = 104,     //!< Application crc check status group number (104).
    kStatusGroup_Packetizer = 105,      //!< Packetizer status group number (105).
    kStatusGroup_ReliableUpdate = 106,  //!< Reliable Update status groupt number (106).

    kStatusGroup_SerialNorEeprom = 107, //!< Serial NOR/EEPROM status group number
    kStatusGroup_FlexSPINAND = 200,     //!< FlexSPINAND status group number.
    kStatusGroup_FLEXSPINOR = 201,      //!< FlexSPINOR status group number.
    kStatusGroup_OCOTP = 202,           //!< OCOTP status group number.
    kStatusGroup_SemcNOR = 211,         //!< SEMC NOR status group number.
    kStatusGroup_SemcNAND = 212,        //!< SEMC NAND status group number.
};

enum
{
    kFlexSpiSerialClk_30MHz = 1,
    kFlexSpiSerialClk_50MHz = 2,
    kFlexSpiSerialClk_60MHz = 3,
    kFlexSpiSerialClk_75MHz = 4,
    kFlexSpiSerialClk_80MHz = 5,
    kFlexSpiSerialClk_100MHz = 6,
    kFlexSpiSerialClk_133MHz = 7,
    kFlexSpiSerialClk_166MHz = 8,
    kFlexSpiSerialClk_200MHz = 9,
};




////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#ifndef FREQ_1MHz
#define FREQ_1MHz (1UL * 1000 * 1000)
#endif
#ifndef FREQ_24MHz
#define FREQ_24MHz (24UL * 1000 * 1000)
#endif
#ifndef FREQ_396MHz
#define FREQ_396MHz (396UL * 1000 * 1000)
#endif
#ifndef FREQ_432MHz
#define FREQ_432MHz (432UL * 1000 * 1000)
#endif
#ifndef FREQ_480MHz
#define FREQ_480MHz (480UL * 1000 * 1000)
#endif
#ifndef FREQ_500MHz
#define FREQ_500MHz (500UL * 1000 * 1000)
#endif
#ifndef FREQ_508MHz
#define FREQ_508MHz (508UL * 1000 * 1000)
#endif
#ifndef FREQ_528MHz
#define FREQ_528MHz (528UL * 1000 * 1000)
#endif

#define SystemCoreClock 528000000UL

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
  __IO uint32_t AHBRXBUFCR0[4];                    /**< AHB RX Buffer 0 Control Register 0..AHB RX Buffer 3 Control Register 0, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_0[48];
  __IO uint32_t FLSHCR0[4];                        /**< Flash A1 Control Register 0..Flash B2 Control Register 0, array offset: 0x60, array step: 0x4 */
  __IO uint32_t FLSHCR1[4];                        /**< Flash A1 Control Register 1..Flash B2 Control Register 1, array offset: 0x70, array step: 0x4 */
  __IO uint32_t FLSHCR2[4];                        /**< Flash A1 Control Register 2..Flash B2 Control Register 2, array offset: 0x80, array step: 0x4 */
       uint8_t RESERVED_1[4];
  __IO uint32_t FLSHCR4;                           /**< Flash Control Register 4, offset: 0x94 */
       uint8_t RESERVED_2[8];
  __IO uint32_t IPCR0;                             /**< IP Control Register 0, offset: 0xA0 */
  __IO uint32_t IPCR1;                             /**< IP Control Register 1, offset: 0xA4 */
       uint8_t RESERVED_3[8];
  __IO uint32_t IPCMD;                             /**< IP Command Register, offset: 0xB0 */
       uint8_t RESERVED_4[4];
  __IO uint32_t IPRXFCR;                           /**< IP RX FIFO Control Register, offset: 0xB8 */
  __IO uint32_t IPTXFCR;                           /**< IP TX FIFO Control Register, offset: 0xBC */
  __IO uint32_t DLLCR[2];                          /**< DLL Control Register 0, array offset: 0xC0, array step: 0x4 */
       uint8_t RESERVED_5[24];
  __I  uint32_t STS0;                              /**< Status Register 0, offset: 0xE0 */
  __I  uint32_t STS1;                              /**< Status Register 1, offset: 0xE4 */
  __I  uint32_t STS2;                              /**< Status Register 2, offset: 0xE8 */
  __I  uint32_t AHBSPNDSTS;                        /**< AHB Suspend Status Register, offset: 0xEC */
  __I  uint32_t IPRXFSTS;                          /**< IP RX FIFO Status Register, offset: 0xF0 */
  __I  uint32_t IPTXFSTS;                          /**< IP TX FIFO Status Register, offset: 0xF4 */
       uint8_t RESERVED_6[8];
  __I  uint32_t RFDR[32];                          /**< IP RX FIFO Data Register 0..IP RX FIFO Data Register 31, array offset: 0x100, array step: 0x4 */
  __O  uint32_t TFDR[32];                          /**< IP TX FIFO Data Register 0..IP TX FIFO Data Register 31, array offset: 0x180, array step: 0x4 */
  __IO uint32_t LUT[64];                           /**< LUT 0..LUT 63, array offset: 0x200, array step: 0x4 */
} FLEXSPI_Type;

/* ----------------------------------------------------------------------------
   -- FLEXSPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FLEXSPI_Register_Masks FLEXSPI Register Masks
 * @{
 */

/*! @name MCR0 - Module Control Register 0 */
/*! @{ */
#define FLEXSPI_MCR0_SWRESET_MASK                (0x1U)
#define FLEXSPI_MCR0_SWRESET_SHIFT               (0U)
/*! SWRESET - Software Reset
 */
#define FLEXSPI_MCR0_SWRESET(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_SWRESET_SHIFT)) & FLEXSPI_MCR0_SWRESET_MASK)
#define FLEXSPI_MCR0_MDIS_MASK                   (0x2U)
#define FLEXSPI_MCR0_MDIS_SHIFT                  (1U)
/*! MDIS - Module Disable
 */
#define FLEXSPI_MCR0_MDIS(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_MDIS_SHIFT)) & FLEXSPI_MCR0_MDIS_MASK)
#define FLEXSPI_MCR0_RXCLKSRC_MASK               (0x30U)
#define FLEXSPI_MCR0_RXCLKSRC_SHIFT              (4U)
/*! RXCLKSRC - Sample Clock source selection for Flash Reading
 *  0b00..Dummy Read strobe generated by FlexSPI Controller and loopback internally.
 *  0b01..Dummy Read strobe generated by FlexSPI Controller and loopback from DQS pad.
 *  0b10..Reserved
 *  0b11..Flash provided Read strobe and input from DQS pad
 */
#define FLEXSPI_MCR0_RXCLKSRC(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_RXCLKSRC_SHIFT)) & FLEXSPI_MCR0_RXCLKSRC_MASK)
#define FLEXSPI_MCR0_ARDFEN_MASK                 (0x40U)
#define FLEXSPI_MCR0_ARDFEN_SHIFT                (6U)
/*! ARDFEN - Enable AHB bus Read Access to IP RX FIFO.
 *  0b0..IP RX FIFO should be read by IP Bus. AHB Bus read access to IP RX FIFO memory space will get bus error response.
 *  0b1..IP RX FIFO should be read by AHB Bus. IP Bus read access to IP RX FIFO memory space will always return data zero but no bus error response.
 */
#define FLEXSPI_MCR0_ARDFEN(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_ARDFEN_SHIFT)) & FLEXSPI_MCR0_ARDFEN_MASK)
#define FLEXSPI_MCR0_ATDFEN_MASK                 (0x80U)
#define FLEXSPI_MCR0_ATDFEN_SHIFT                (7U)
/*! ATDFEN - Enable AHB bus Write Access to IP TX FIFO.
 *  0b0..IP TX FIFO should be written by IP Bus. AHB Bus write access to IP TX FIFO memory space will get bus error response.
 *  0b1..IP TX FIFO should be written by AHB Bus. IP Bus write access to IP TX FIFO memory space will be ignored but no bus error response.
 */
#define FLEXSPI_MCR0_ATDFEN(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_ATDFEN_SHIFT)) & FLEXSPI_MCR0_ATDFEN_MASK)
#define FLEXSPI_MCR0_HSEN_MASK                   (0x800U)
#define FLEXSPI_MCR0_HSEN_SHIFT                  (11U)
/*! HSEN - Half Speed Serial Flash access Enable.
 *  0b0..Disable divide by 2 of serial flash clock for half speed commands.
 *  0b1..Enable divide by 2 of serial flash clock for half speed commands.
 */
#define FLEXSPI_MCR0_HSEN(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_HSEN_SHIFT)) & FLEXSPI_MCR0_HSEN_MASK)
#define FLEXSPI_MCR0_DOZEEN_MASK                 (0x1000U)
#define FLEXSPI_MCR0_DOZEEN_SHIFT                (12U)
/*! DOZEEN - Doze mode enable bit
 *  0b0..Doze mode support disabled. AHB clock and serial clock will not be gated off when there is doze mode request from system.
 *  0b1..Doze mode support enabled. AHB clock and serial clock will be gated off when there is doze mode request from system.
 */
#define FLEXSPI_MCR0_DOZEEN(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_DOZEEN_SHIFT)) & FLEXSPI_MCR0_DOZEEN_MASK)
#define FLEXSPI_MCR0_COMBINATIONEN_MASK          (0x2000U)
#define FLEXSPI_MCR0_COMBINATIONEN_SHIFT         (13U)
/*! COMBINATIONEN - This bit is to support Flash Octal mode access by combining Port A and B Data pins (SIOA[3:0] and SIOB[3:0]).
 *  0b0..Disable.
 *  0b1..Enable.
 */
#define FLEXSPI_MCR0_COMBINATIONEN(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_COMBINATIONEN_SHIFT)) & FLEXSPI_MCR0_COMBINATIONEN_MASK)
#define FLEXSPI_MCR0_SCKFREERUNEN_MASK           (0x4000U)
#define FLEXSPI_MCR0_SCKFREERUNEN_SHIFT          (14U)
/*! SCKFREERUNEN - This bit is used to force SCK output free-running. For FPGA applications,
 *    external device may use SCK clock as reference clock to its internal PLL. If SCK free-running is
 *    enabled, data sampling with loopback clock from SCK pad is not supported (MCR0[RXCLKSRC]=2).
 *  0b0..Disable.
 *  0b1..Enable.
 */
#define FLEXSPI_MCR0_SCKFREERUNEN(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_SCKFREERUNEN_SHIFT)) & FLEXSPI_MCR0_SCKFREERUNEN_MASK)
#define FLEXSPI_MCR0_IPGRANTWAIT_MASK            (0xFF0000U)
#define FLEXSPI_MCR0_IPGRANTWAIT_SHIFT           (16U)
/*! IPGRANTWAIT - Time out wait cycle for IP command grant.
 */
#define FLEXSPI_MCR0_IPGRANTWAIT(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_IPGRANTWAIT_SHIFT)) & FLEXSPI_MCR0_IPGRANTWAIT_MASK)
#define FLEXSPI_MCR0_AHBGRANTWAIT_MASK           (0xFF000000U)
#define FLEXSPI_MCR0_AHBGRANTWAIT_SHIFT          (24U)
/*! AHBGRANTWAIT - Timeout wait cycle for AHB command grant.
 */
#define FLEXSPI_MCR0_AHBGRANTWAIT(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR0_AHBGRANTWAIT_SHIFT)) & FLEXSPI_MCR0_AHBGRANTWAIT_MASK)
/*! @} */

/*! @name MCR1 - Module Control Register 1 */
/*! @{ */
#define FLEXSPI_MCR1_AHBBUSWAIT_MASK             (0xFFFFU)
#define FLEXSPI_MCR1_AHBBUSWAIT_SHIFT            (0U)
#define FLEXSPI_MCR1_AHBBUSWAIT(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR1_AHBBUSWAIT_SHIFT)) & FLEXSPI_MCR1_AHBBUSWAIT_MASK)
#define FLEXSPI_MCR1_SEQWAIT_MASK                (0xFFFF0000U)
#define FLEXSPI_MCR1_SEQWAIT_SHIFT               (16U)
#define FLEXSPI_MCR1_SEQWAIT(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR1_SEQWAIT_SHIFT)) & FLEXSPI_MCR1_SEQWAIT_MASK)
/*! @} */

/*! @name MCR2 - Module Control Register 2 */
/*! @{ */
#define FLEXSPI_MCR2_CLRAHBBUFOPT_MASK           (0x800U)
#define FLEXSPI_MCR2_CLRAHBBUFOPT_SHIFT          (11U)
/*! CLRAHBBUFOPT - This bit determines whether AHB RX Buffer and AHB TX Buffer will be cleaned
 *    automaticaly when FlexSPI returns STOP mode ACK. Software should set this bit if AHB RX Buffer or
 *    AHB TX Buffer will be powered off in STOP mode. Otherwise AHB read access after exiting STOP
 *    mode may hit AHB RX Buffer or AHB TX Buffer but their data entries are invalid.
 *  0b0..AHB RX/TX Buffer will not be cleaned automatically when FlexSPI return Stop mode ACK.
 *  0b1..AHB RX/TX Buffer will be cleaned automatically when FlexSPI return Stop mode ACK.
 */
#define FLEXSPI_MCR2_CLRAHBBUFOPT(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_CLRAHBBUFOPT_SHIFT)) & FLEXSPI_MCR2_CLRAHBBUFOPT_MASK)
#define FLEXSPI_MCR2_CLRLEARNPHASE_MASK          (0x4000U)
#define FLEXSPI_MCR2_CLRLEARNPHASE_SHIFT         (14U)
/*! CLRLEARNPHASE - The sampling clock phase selection will be reset to phase 0 when this bit is
 *    written with 0x1. This bit will be auto-cleared immediately.
 */
#define FLEXSPI_MCR2_CLRLEARNPHASE(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_CLRLEARNPHASE_SHIFT)) & FLEXSPI_MCR2_CLRLEARNPHASE_MASK)
#define FLEXSPI_MCR2_SAMEDEVICEEN_MASK           (0x8000U)
#define FLEXSPI_MCR2_SAMEDEVICEEN_SHIFT          (15U)
/*! SAMEDEVICEEN - All external devices are same devices (both in types and size) for A1/A2/B1/B2.
 *  0b0..In Individual mode, FLSHA1CRx/FLSHA2CRx/FLSHB1CRx/FLSHB2CRx register setting will be applied to Flash
 *       A1/A2/B1/B2 seperately. In Parallel mode, FLSHA1CRx register setting will be applied to Flash A1 and B1,
 *       FLSHA2CRx register setting will be applied to Flash A2 and B2. FLSHB1CRx/FLSHB2CRx register settings will be
 *       ignored.
 *  0b1..FLSHA1CR0/FLSHA1CR1/FLSHA1CR2 register settings will be applied to Flash A1/A2/B1/B2. FLSHA2CRx/FLSHB1CRx/FLSHB2CRx will be ignored.
 */
#define FLEXSPI_MCR2_SAMEDEVICEEN(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_SAMEDEVICEEN_SHIFT)) & FLEXSPI_MCR2_SAMEDEVICEEN_MASK)
#define FLEXSPI_MCR2_SCKBDIFFOPT_MASK            (0x80000U)
#define FLEXSPI_MCR2_SCKBDIFFOPT_SHIFT           (19U)
/*! SCKBDIFFOPT - SCKB pad can be used as SCKA differential clock output (inverted clock to SCKA).
 *    In this case, port B flash access is not available. After change the value of this feild,
 *    MCR0[SWRESET] should be set.
 *  0b1..SCKB pad is used as port A SCK inverted clock output (Differential clock to SCKA). Port B flash access is not available.
 *  0b0..SCKB pad is used as port B SCK clock output. Port B flash access is available.
 */
#define FLEXSPI_MCR2_SCKBDIFFOPT(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_SCKBDIFFOPT_SHIFT)) & FLEXSPI_MCR2_SCKBDIFFOPT_MASK)
#define FLEXSPI_MCR2_RESUMEWAIT_MASK             (0xFF000000U)
#define FLEXSPI_MCR2_RESUMEWAIT_SHIFT            (24U)
/*! RESUMEWAIT - Wait cycle (in AHB clock cycle) for idle state before suspended command sequence resumed.
 */
#define FLEXSPI_MCR2_RESUMEWAIT(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_MCR2_RESUMEWAIT_SHIFT)) & FLEXSPI_MCR2_RESUMEWAIT_MASK)
/*! @} */

/*! @name AHBCR - AHB Bus Control Register */
/*! @{ */
#define FLEXSPI_AHBCR_APAREN_MASK                (0x1U)
#define FLEXSPI_AHBCR_APAREN_SHIFT               (0U)
/*! APAREN - Parallel mode enabled for AHB triggered Command (both read and write) .
 *  0b0..Flash will be accessed in Individual mode.
 *  0b1..Flash will be accessed in Parallel mode.
 */
#define FLEXSPI_AHBCR_APAREN(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_APAREN_SHIFT)) & FLEXSPI_AHBCR_APAREN_MASK)
#define FLEXSPI_AHBCR_CACHABLEEN_MASK            (0x8U)
#define FLEXSPI_AHBCR_CACHABLEEN_SHIFT           (3U)
/*! CACHABLEEN - Enable AHB bus cachable read access support.
 *  0b0..Disabled. When there is AHB bus cachable read access, FlexSPI will not check whether it hit AHB TX Buffer.
 *  0b1..Enabled. When there is AHB bus cachable read access, FlexSPI will check whether it hit AHB TX Buffer first.
 */
#define FLEXSPI_AHBCR_CACHABLEEN(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_CACHABLEEN_SHIFT)) & FLEXSPI_AHBCR_CACHABLEEN_MASK)
#define FLEXSPI_AHBCR_BUFFERABLEEN_MASK          (0x10U)
#define FLEXSPI_AHBCR_BUFFERABLEEN_SHIFT         (4U)
/*! BUFFERABLEEN - Enable AHB bus bufferable write access support. This field affects the last beat
 *    of AHB write access, refer for more details about AHB bufferable write.
 *  0b0..Disabled. For all AHB write access (no matter bufferable or non-bufferable ), FlexSPI will return AHB Bus
 *       ready after all data is transmitted to External device and AHB command finished.
 *  0b1..Enabled. For AHB bufferable write access, FlexSPI will return AHB Bus ready when the AHB command is
 *       granted by arbitrator and will not wait for AHB command finished.
 */
#define FLEXSPI_AHBCR_BUFFERABLEEN(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_BUFFERABLEEN_SHIFT)) & FLEXSPI_AHBCR_BUFFERABLEEN_MASK)
#define FLEXSPI_AHBCR_PREFETCHEN_MASK            (0x20U)
#define FLEXSPI_AHBCR_PREFETCHEN_SHIFT           (5U)
/*! PREFETCHEN - AHB Read Prefetch Enable.
 */
#define FLEXSPI_AHBCR_PREFETCHEN(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_PREFETCHEN_SHIFT)) & FLEXSPI_AHBCR_PREFETCHEN_MASK)
#define FLEXSPI_AHBCR_READADDROPT_MASK           (0x40U)
#define FLEXSPI_AHBCR_READADDROPT_SHIFT          (6U)
/*! READADDROPT - AHB Read Address option bit. This option bit is intend to remove AHB burst start address alignment limitation.
 *  0b0..There is AHB read burst start address alignment limitation when flash is accessed in parallel mode or flash is wordaddressable.
 *  0b1..There is no AHB read burst start address alignment limitation. FlexSPI will fetch more datas than AHB
 *       burst required to meet the alignment requirement.
 */
#define FLEXSPI_AHBCR_READADDROPT(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBCR_READADDROPT_SHIFT)) & FLEXSPI_AHBCR_READADDROPT_MASK)
/*! @} */

/*! @name INTEN - Interrupt Enable Register */
/*! @{ */
#define FLEXSPI_INTEN_IPCMDDONEEN_MASK           (0x1U)
#define FLEXSPI_INTEN_IPCMDDONEEN_SHIFT          (0U)
/*! IPCMDDONEEN - IP triggered Command Sequences Execution finished interrupt enable.
 */
#define FLEXSPI_INTEN_IPCMDDONEEN(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPCMDDONEEN_SHIFT)) & FLEXSPI_INTEN_IPCMDDONEEN_MASK)
#define FLEXSPI_INTEN_IPCMDGEEN_MASK             (0x2U)
#define FLEXSPI_INTEN_IPCMDGEEN_SHIFT            (1U)
/*! IPCMDGEEN - IP triggered Command Sequences Grant Timeout interrupt enable.
 */
#define FLEXSPI_INTEN_IPCMDGEEN(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPCMDGEEN_SHIFT)) & FLEXSPI_INTEN_IPCMDGEEN_MASK)
#define FLEXSPI_INTEN_AHBCMDGEEN_MASK            (0x4U)
#define FLEXSPI_INTEN_AHBCMDGEEN_SHIFT           (2U)
/*! AHBCMDGEEN - AHB triggered Command Sequences Grant Timeout interrupt enable.
 */
#define FLEXSPI_INTEN_AHBCMDGEEN(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_AHBCMDGEEN_SHIFT)) & FLEXSPI_INTEN_AHBCMDGEEN_MASK)
#define FLEXSPI_INTEN_IPCMDERREN_MASK            (0x8U)
#define FLEXSPI_INTEN_IPCMDERREN_SHIFT           (3U)
/*! IPCMDERREN - IP triggered Command Sequences Error Detected interrupt enable.
 */
#define FLEXSPI_INTEN_IPCMDERREN(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPCMDERREN_SHIFT)) & FLEXSPI_INTEN_IPCMDERREN_MASK)
#define FLEXSPI_INTEN_AHBCMDERREN_MASK           (0x10U)
#define FLEXSPI_INTEN_AHBCMDERREN_SHIFT          (4U)
/*! AHBCMDERREN - AHB triggered Command Sequences Error Detected interrupt enable.
 */
#define FLEXSPI_INTEN_AHBCMDERREN(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_AHBCMDERREN_SHIFT)) & FLEXSPI_INTEN_AHBCMDERREN_MASK)
#define FLEXSPI_INTEN_IPRXWAEN_MASK              (0x20U)
#define FLEXSPI_INTEN_IPRXWAEN_SHIFT             (5U)
/*! IPRXWAEN - IP RX FIFO WaterMark available interrupt enable.
 */
#define FLEXSPI_INTEN_IPRXWAEN(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPRXWAEN_SHIFT)) & FLEXSPI_INTEN_IPRXWAEN_MASK)
#define FLEXSPI_INTEN_IPTXWEEN_MASK              (0x40U)
#define FLEXSPI_INTEN_IPTXWEEN_SHIFT             (6U)
/*! IPTXWEEN - IP TX FIFO WaterMark empty interrupt enable.
 */
#define FLEXSPI_INTEN_IPTXWEEN(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_IPTXWEEN_SHIFT)) & FLEXSPI_INTEN_IPTXWEEN_MASK)
#define FLEXSPI_INTEN_SCKSTOPBYRDEN_MASK         (0x100U)
#define FLEXSPI_INTEN_SCKSTOPBYRDEN_SHIFT        (8U)
/*! SCKSTOPBYRDEN - SCK is stopped during command sequence because Async RX FIFO full interrupt enable.
 */
#define FLEXSPI_INTEN_SCKSTOPBYRDEN(x)           (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_SCKSTOPBYRDEN_SHIFT)) & FLEXSPI_INTEN_SCKSTOPBYRDEN_MASK)
#define FLEXSPI_INTEN_SCKSTOPBYWREN_MASK         (0x200U)
#define FLEXSPI_INTEN_SCKSTOPBYWREN_SHIFT        (9U)
/*! SCKSTOPBYWREN - SCK is stopped during command sequence because Async TX FIFO empty interrupt enable.
 */
#define FLEXSPI_INTEN_SCKSTOPBYWREN(x)           (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_SCKSTOPBYWREN_SHIFT)) & FLEXSPI_INTEN_SCKSTOPBYWREN_MASK)
#define FLEXSPI_INTEN_AHBBUSTIMEOUTEN_MASK       (0x400U)
#define FLEXSPI_INTEN_AHBBUSTIMEOUTEN_SHIFT      (10U)
/*! AHBBUSTIMEOUTEN - AHB Bus timeout interrupt.Refer Interrupts chapter for more details.
 */
#define FLEXSPI_INTEN_AHBBUSTIMEOUTEN(x)         (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_AHBBUSTIMEOUTEN_SHIFT)) & FLEXSPI_INTEN_AHBBUSTIMEOUTEN_MASK)
#define FLEXSPI_INTEN_SEQTIMEOUTEN_MASK          (0x800U)
#define FLEXSPI_INTEN_SEQTIMEOUTEN_SHIFT         (11U)
/*! SEQTIMEOUTEN - Sequence execution timeout interrupt enable.Refer Interrupts chapter for more details.
 */
#define FLEXSPI_INTEN_SEQTIMEOUTEN(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTEN_SEQTIMEOUTEN_SHIFT)) & FLEXSPI_INTEN_SEQTIMEOUTEN_MASK)
/*! @} */

/*! @name INTR - Interrupt Register */
/*! @{ */
#define FLEXSPI_INTR_IPCMDDONE_MASK              (0x1U)
#define FLEXSPI_INTR_IPCMDDONE_SHIFT             (0U)
/*! IPCMDDONE - IP triggered Command Sequences Execution finished interrupt. This interrupt is also
 *    generated when there is IPCMDGE or IPCMDERR interrupt generated.
 */
#define FLEXSPI_INTR_IPCMDDONE(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPCMDDONE_SHIFT)) & FLEXSPI_INTR_IPCMDDONE_MASK)
#define FLEXSPI_INTR_IPCMDGE_MASK                (0x2U)
#define FLEXSPI_INTR_IPCMDGE_SHIFT               (1U)
/*! IPCMDGE - IP triggered Command Sequences Grant Timeout interrupt.
 */
#define FLEXSPI_INTR_IPCMDGE(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPCMDGE_SHIFT)) & FLEXSPI_INTR_IPCMDGE_MASK)
#define FLEXSPI_INTR_AHBCMDGE_MASK               (0x4U)
#define FLEXSPI_INTR_AHBCMDGE_SHIFT              (2U)
/*! AHBCMDGE - AHB triggered Command Sequences Grant Timeout interrupt.
 */
#define FLEXSPI_INTR_AHBCMDGE(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_AHBCMDGE_SHIFT)) & FLEXSPI_INTR_AHBCMDGE_MASK)
#define FLEXSPI_INTR_IPCMDERR_MASK               (0x8U)
#define FLEXSPI_INTR_IPCMDERR_SHIFT              (3U)
/*! IPCMDERR - IP triggered Command Sequences Error Detected interrupt. When an error detected for
 *    IP command, this command will be ignored and not executed at all.
 */
#define FLEXSPI_INTR_IPCMDERR(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPCMDERR_SHIFT)) & FLEXSPI_INTR_IPCMDERR_MASK)
#define FLEXSPI_INTR_AHBCMDERR_MASK              (0x10U)
#define FLEXSPI_INTR_AHBCMDERR_SHIFT             (4U)
/*! AHBCMDERR - AHB triggered Command Sequences Error Detected interrupt. When an error detected for
 *    AHB command, this command will be ignored and not executed at all.
 */
#define FLEXSPI_INTR_AHBCMDERR(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_AHBCMDERR_SHIFT)) & FLEXSPI_INTR_AHBCMDERR_MASK)
#define FLEXSPI_INTR_IPRXWA_MASK                 (0x20U)
#define FLEXSPI_INTR_IPRXWA_SHIFT                (5U)
/*! IPRXWA - IP RX FIFO watermark available interrupt.
 */
#define FLEXSPI_INTR_IPRXWA(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPRXWA_SHIFT)) & FLEXSPI_INTR_IPRXWA_MASK)
#define FLEXSPI_INTR_IPTXWE_MASK                 (0x40U)
#define FLEXSPI_INTR_IPTXWE_SHIFT                (6U)
/*! IPTXWE - IP TX FIFO watermark empty interrupt.
 */
#define FLEXSPI_INTR_IPTXWE(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_IPTXWE_SHIFT)) & FLEXSPI_INTR_IPTXWE_MASK)
#define FLEXSPI_INTR_SCKSTOPBYRD_MASK            (0x100U)
#define FLEXSPI_INTR_SCKSTOPBYRD_SHIFT           (8U)
/*! SCKSTOPBYRD - SCK is stopped during command sequence because Async RX FIFO full interrupt.
 */
#define FLEXSPI_INTR_SCKSTOPBYRD(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_SCKSTOPBYRD_SHIFT)) & FLEXSPI_INTR_SCKSTOPBYRD_MASK)
#define FLEXSPI_INTR_SCKSTOPBYWR_MASK            (0x200U)
#define FLEXSPI_INTR_SCKSTOPBYWR_SHIFT           (9U)
/*! SCKSTOPBYWR - SCK is stopped during command sequence because Async TX FIFO empty interrupt.
 */
#define FLEXSPI_INTR_SCKSTOPBYWR(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_SCKSTOPBYWR_SHIFT)) & FLEXSPI_INTR_SCKSTOPBYWR_MASK)
#define FLEXSPI_INTR_AHBBUSTIMEOUT_MASK          (0x400U)
#define FLEXSPI_INTR_AHBBUSTIMEOUT_SHIFT         (10U)
/*! AHBBUSTIMEOUT - AHB Bus timeout interrupt.Refer Interrupts chapter for more details.
 */
#define FLEXSPI_INTR_AHBBUSTIMEOUT(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_AHBBUSTIMEOUT_SHIFT)) & FLEXSPI_INTR_AHBBUSTIMEOUT_MASK)
#define FLEXSPI_INTR_SEQTIMEOUT_MASK             (0x800U)
#define FLEXSPI_INTR_SEQTIMEOUT_SHIFT            (11U)
/*! SEQTIMEOUT - Sequence execution timeout interrupt.
 */
#define FLEXSPI_INTR_SEQTIMEOUT(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_INTR_SEQTIMEOUT_SHIFT)) & FLEXSPI_INTR_SEQTIMEOUT_MASK)
/*! @} */

/*! @name LUTKEY - LUT Key Register */
/*! @{ */
#define FLEXSPI_LUTKEY_KEY_MASK                  (0xFFFFFFFFU)
#define FLEXSPI_LUTKEY_KEY_SHIFT                 (0U)
/*! KEY - The Key to lock or unlock LUT.
 */
#define FLEXSPI_LUTKEY_KEY(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUTKEY_KEY_SHIFT)) & FLEXSPI_LUTKEY_KEY_MASK)
/*! @} */

/*! @name LUTCR - LUT Control Register */
/*! @{ */
#define FLEXSPI_LUTCR_LOCK_MASK                  (0x1U)
#define FLEXSPI_LUTCR_LOCK_SHIFT                 (0U)
/*! LOCK - Lock LUT
 */
#define FLEXSPI_LUTCR_LOCK(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUTCR_LOCK_SHIFT)) & FLEXSPI_LUTCR_LOCK_MASK)
#define FLEXSPI_LUTCR_UNLOCK_MASK                (0x2U)
#define FLEXSPI_LUTCR_UNLOCK_SHIFT               (1U)
/*! UNLOCK - Unlock LUT
 */
#define FLEXSPI_LUTCR_UNLOCK(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUTCR_UNLOCK_SHIFT)) & FLEXSPI_LUTCR_UNLOCK_MASK)
/*! @} */

/*! @name AHBRXBUFCR0 - AHB RX Buffer 0 Control Register 0..AHB RX Buffer 3 Control Register 0 */
/*! @{ */
#define FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK           (0xFFU)
#define FLEXSPI_AHBRXBUFCR0_BUFSZ_SHIFT          (0U)
/*! BUFSZ - AHB RX Buffer Size in 64 bits.Refer AHB RX Buffer Management for more details.
 */
#define FLEXSPI_AHBRXBUFCR0_BUFSZ(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBRXBUFCR0_BUFSZ_SHIFT)) & FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK)
#define FLEXSPI_AHBRXBUFCR0_MSTRID_MASK          (0xF0000U)
#define FLEXSPI_AHBRXBUFCR0_MSTRID_SHIFT         (16U)
/*! MSTRID - This AHB RX Buffer is assigned according to AHB Master with ID (MSTR_ID). Please refer to for AHB RX Buffer allocation.
 */
#define FLEXSPI_AHBRXBUFCR0_MSTRID(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBRXBUFCR0_MSTRID_SHIFT)) & FLEXSPI_AHBRXBUFCR0_MSTRID_MASK)
#define FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK        (0x3000000U)
#define FLEXSPI_AHBRXBUFCR0_PRIORITY_SHIFT       (24U)
/*! PRIORITY - This priority for AHB Master Read which this AHB RX Buffer is assigned. Refer for more details.
 */
#define FLEXSPI_AHBRXBUFCR0_PRIORITY(x)          (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBRXBUFCR0_PRIORITY_SHIFT)) & FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK)
#define FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK      (0x80000000U)
#define FLEXSPI_AHBRXBUFCR0_PREFETCHEN_SHIFT     (31U)
/*! PREFETCHEN - AHB Read Prefetch Enable for current AHB RX Buffer corresponding Master.
 */
#define FLEXSPI_AHBRXBUFCR0_PREFETCHEN(x)        (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBRXBUFCR0_PREFETCHEN_SHIFT)) & FLEXSPI_AHBRXBUFCR0_PREFETCHEN_MASK)
/*! @} */

/* The count of FLEXSPI_AHBRXBUFCR0 */
#define FLEXSPI_AHBRXBUFCR0_COUNT                (4U)

/*! @name FLSHCR0 - Flash A1 Control Register 0..Flash B2 Control Register 0 */
/*! @{ */
#define FLEXSPI_FLSHCR0_FLSHSZ_MASK              (0x7FFFFFU)
#define FLEXSPI_FLSHCR0_FLSHSZ_SHIFT             (0U)
/*! FLSHSZ - Flash Size in KByte.
 */
#define FLEXSPI_FLSHCR0_FLSHSZ(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR0_FLSHSZ_SHIFT)) & FLEXSPI_FLSHCR0_FLSHSZ_MASK)
/*! @} */

/* The count of FLEXSPI_FLSHCR0 */
#define FLEXSPI_FLSHCR0_COUNT                    (4U)

/*! @name FLSHCR1 - Flash A1 Control Register 1..Flash B2 Control Register 1 */
/*! @{ */
#define FLEXSPI_FLSHCR1_TCSS_MASK                (0x1FU)
#define FLEXSPI_FLSHCR1_TCSS_SHIFT               (0U)
/*! TCSS - Serial Flash CS setup time.
 */
#define FLEXSPI_FLSHCR1_TCSS(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_TCSS_SHIFT)) & FLEXSPI_FLSHCR1_TCSS_MASK)
#define FLEXSPI_FLSHCR1_TCSH_MASK                (0x3E0U)
#define FLEXSPI_FLSHCR1_TCSH_SHIFT               (5U)
/*! TCSH - Serial Flash CS Hold time.
 */
#define FLEXSPI_FLSHCR1_TCSH(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_TCSH_SHIFT)) & FLEXSPI_FLSHCR1_TCSH_MASK)
#define FLEXSPI_FLSHCR1_WA_MASK                  (0x400U)
#define FLEXSPI_FLSHCR1_WA_SHIFT                 (10U)
/*! WA - Word Addressable.
 */
#define FLEXSPI_FLSHCR1_WA(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_WA_SHIFT)) & FLEXSPI_FLSHCR1_WA_MASK)
#define FLEXSPI_FLSHCR1_CAS_MASK                 (0x7800U)
#define FLEXSPI_FLSHCR1_CAS_SHIFT                (11U)
/*! CAS - Column Address Size.
 */
#define FLEXSPI_FLSHCR1_CAS(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_CAS_SHIFT)) & FLEXSPI_FLSHCR1_CAS_MASK)
#define FLEXSPI_FLSHCR1_CSINTERVALUNIT_MASK      (0x8000U)
#define FLEXSPI_FLSHCR1_CSINTERVALUNIT_SHIFT     (15U)
/*! CSINTERVALUNIT - CS interval unit
 *  0b0..The CS interval unit is 1 serial clock cycle
 *  0b1..The CS interval unit is 256 serial clock cycle
 */
#define FLEXSPI_FLSHCR1_CSINTERVALUNIT(x)        (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_CSINTERVALUNIT_SHIFT)) & FLEXSPI_FLSHCR1_CSINTERVALUNIT_MASK)
#define FLEXSPI_FLSHCR1_CSINTERVAL_MASK          (0xFFFF0000U)
#define FLEXSPI_FLSHCR1_CSINTERVAL_SHIFT         (16U)
/*! CSINTERVAL - This field is used to set the minimum interval between flash device Chip selection
 *    deassertion and flash device Chip selection assertion. If external flash has a limitation on
 *    the interval between command sequences, this field should be set accordingly. If there is no
 *    limitation, set this field with value 0x0.
 */
#define FLEXSPI_FLSHCR1_CSINTERVAL(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR1_CSINTERVAL_SHIFT)) & FLEXSPI_FLSHCR1_CSINTERVAL_MASK)
/*! @} */

/* The count of FLEXSPI_FLSHCR1 */
#define FLEXSPI_FLSHCR1_COUNT                    (4U)

/*! @name FLSHCR2 - Flash A1 Control Register 2..Flash B2 Control Register 2 */
/*! @{ */
#define FLEXSPI_FLSHCR2_ARDSEQID_MASK            (0xFU)
#define FLEXSPI_FLSHCR2_ARDSEQID_SHIFT           (0U)
/*! ARDSEQID - Sequence Index for AHB Read triggered Command in LUT.
 */
#define FLEXSPI_FLSHCR2_ARDSEQID(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_ARDSEQID_SHIFT)) & FLEXSPI_FLSHCR2_ARDSEQID_MASK)
#define FLEXSPI_FLSHCR2_ARDSEQNUM_MASK           (0xE0U)
#define FLEXSPI_FLSHCR2_ARDSEQNUM_SHIFT          (5U)
/*! ARDSEQNUM - Sequence Number for AHB Read triggered Command in LUT.
 */
#define FLEXSPI_FLSHCR2_ARDSEQNUM(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_ARDSEQNUM_SHIFT)) & FLEXSPI_FLSHCR2_ARDSEQNUM_MASK)
#define FLEXSPI_FLSHCR2_AWRSEQID_MASK            (0xF00U)
#define FLEXSPI_FLSHCR2_AWRSEQID_SHIFT           (8U)
/*! AWRSEQID - Sequence Index for AHB Write triggered Command.
 */
#define FLEXSPI_FLSHCR2_AWRSEQID(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_AWRSEQID_SHIFT)) & FLEXSPI_FLSHCR2_AWRSEQID_MASK)
#define FLEXSPI_FLSHCR2_AWRSEQNUM_MASK           (0xE000U)
#define FLEXSPI_FLSHCR2_AWRSEQNUM_SHIFT          (13U)
/*! AWRSEQNUM - Sequence Number for AHB Write triggered Command.
 */
#define FLEXSPI_FLSHCR2_AWRSEQNUM(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_AWRSEQNUM_SHIFT)) & FLEXSPI_FLSHCR2_AWRSEQNUM_MASK)
#define FLEXSPI_FLSHCR2_AWRWAIT_MASK             (0xFFF0000U)
#define FLEXSPI_FLSHCR2_AWRWAIT_SHIFT            (16U)
#define FLEXSPI_FLSHCR2_AWRWAIT(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_AWRWAIT_SHIFT)) & FLEXSPI_FLSHCR2_AWRWAIT_MASK)
#define FLEXSPI_FLSHCR2_AWRWAITUNIT_MASK         (0x70000000U)
#define FLEXSPI_FLSHCR2_AWRWAITUNIT_SHIFT        (28U)
/*! AWRWAITUNIT - AWRWAIT unit
 *  0b000..The AWRWAIT unit is 2 ahb clock cycle
 *  0b001..The AWRWAIT unit is 8 ahb clock cycle
 *  0b010..The AWRWAIT unit is 32 ahb clock cycle
 *  0b011..The AWRWAIT unit is 128 ahb clock cycle
 *  0b100..The AWRWAIT unit is 512 ahb clock cycle
 *  0b101..The AWRWAIT unit is 2048 ahb clock cycle
 *  0b110..The AWRWAIT unit is 8192 ahb clock cycle
 *  0b111..The AWRWAIT unit is 32768 ahb clock cycle
 */
#define FLEXSPI_FLSHCR2_AWRWAITUNIT(x)           (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_AWRWAITUNIT_SHIFT)) & FLEXSPI_FLSHCR2_AWRWAITUNIT_MASK)
#define FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK         (0x80000000U)
#define FLEXSPI_FLSHCR2_CLRINSTRPTR_SHIFT        (31U)
/*! CLRINSTRPTR - Clear the instruction pointer which is internally saved pointer by JMP_ON_CS.
 *    Refer Programmable Sequence Engine for details.
 */
#define FLEXSPI_FLSHCR2_CLRINSTRPTR(x)           (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR2_CLRINSTRPTR_SHIFT)) & FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK)
/*! @} */

/* The count of FLEXSPI_FLSHCR2 */
#define FLEXSPI_FLSHCR2_COUNT                    (4U)

/*! @name FLSHCR4 - Flash Control Register 4 */
/*! @{ */
#define FLEXSPI_FLSHCR4_WMOPT1_MASK              (0x1U)
#define FLEXSPI_FLSHCR4_WMOPT1_SHIFT             (0U)
/*! WMOPT1 - Write mask option bit 1. This option bit could be used to remove AHB write burst start address alignment limitation.
 *  0b0..DQS pin will be used as Write Mask when writing to external device. There is no limitation on AHB write
 *       burst start address alignment when flash is accessed in individual mode.
 *  0b1..DQS pin will not be used as Write Mask when writing to external device. There is limitation on AHB write
 *       burst start address alignment when flash is accessed in individual mode.
 */
#define FLEXSPI_FLSHCR4_WMOPT1(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR4_WMOPT1_SHIFT)) & FLEXSPI_FLSHCR4_WMOPT1_MASK)
#define FLEXSPI_FLSHCR4_WMENA_MASK               (0x4U)
#define FLEXSPI_FLSHCR4_WMENA_SHIFT              (2U)
/*! WMENA - Write mask enable bit for flash device on port A. When write mask function is needed for
 *    memory device on port A, this bit must be set.
 *  0b0..Write mask is disabled, DQS(RWDS) pin will be un-driven when writing to external device.
 *  0b1..Write mask is enabled, DQS(RWDS) pin will be driven by FlexSPI as write mask output when writing to external device.
 */
#define FLEXSPI_FLSHCR4_WMENA(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR4_WMENA_SHIFT)) & FLEXSPI_FLSHCR4_WMENA_MASK)
#define FLEXSPI_FLSHCR4_WMENB_MASK               (0x8U)
#define FLEXSPI_FLSHCR4_WMENB_SHIFT              (3U)
/*! WMENB - Write mask enable bit for flash device on port B. When write mask function is needed for
 *    memory device on port B, this bit must be set.
 *  0b0..Write mask is disabled, DQS(RWDS) pin will be un-driven when writing to external device.
 *  0b1..Write mask is enabled, DQS(RWDS) pin will be driven by FlexSPI as write mask output when writing to external device.
 */
#define FLEXSPI_FLSHCR4_WMENB(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_FLSHCR4_WMENB_SHIFT)) & FLEXSPI_FLSHCR4_WMENB_MASK)
/*! @} */

/*! @name IPCR0 - IP Control Register 0 */
/*! @{ */
#define FLEXSPI_IPCR0_SFAR_MASK                  (0xFFFFFFFFU)
#define FLEXSPI_IPCR0_SFAR_SHIFT                 (0U)
/*! SFAR - Serial Flash Address for IP command.
 */
#define FLEXSPI_IPCR0_SFAR(x)                    (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR0_SFAR_SHIFT)) & FLEXSPI_IPCR0_SFAR_MASK)
/*! @} */

/*! @name IPCR1 - IP Control Register 1 */
/*! @{ */
#define FLEXSPI_IPCR1_IDATSZ_MASK                (0xFFFFU)
#define FLEXSPI_IPCR1_IDATSZ_SHIFT               (0U)
/*! IDATSZ - Flash Read/Program Data Size (in Bytes) for IP command.
 */
#define FLEXSPI_IPCR1_IDATSZ(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR1_IDATSZ_SHIFT)) & FLEXSPI_IPCR1_IDATSZ_MASK)
#define FLEXSPI_IPCR1_ISEQID_MASK                (0xF0000U)
#define FLEXSPI_IPCR1_ISEQID_SHIFT               (16U)
/*! ISEQID - Sequence Index in LUT for IP command.
 */
#define FLEXSPI_IPCR1_ISEQID(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR1_ISEQID_SHIFT)) & FLEXSPI_IPCR1_ISEQID_MASK)
#define FLEXSPI_IPCR1_ISEQNUM_MASK               (0x7000000U)
#define FLEXSPI_IPCR1_ISEQNUM_SHIFT              (24U)
/*! ISEQNUM - Sequence Number for IP command: ISEQNUM+1.
 */
#define FLEXSPI_IPCR1_ISEQNUM(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR1_ISEQNUM_SHIFT)) & FLEXSPI_IPCR1_ISEQNUM_MASK)
#define FLEXSPI_IPCR1_IPAREN_MASK                (0x80000000U)
#define FLEXSPI_IPCR1_IPAREN_SHIFT               (31U)
/*! IPAREN - Parallel mode Enabled for IP command.
 *  0b0..Flash will be accessed in Individual mode.
 *  0b1..Flash will be accessed in Parallel mode.
 */
#define FLEXSPI_IPCR1_IPAREN(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCR1_IPAREN_SHIFT)) & FLEXSPI_IPCR1_IPAREN_MASK)
/*! @} */

/*! @name IPCMD - IP Command Register */
/*! @{ */
#define FLEXSPI_IPCMD_TRG_MASK                   (0x1U)
#define FLEXSPI_IPCMD_TRG_SHIFT                  (0U)
/*! TRG - Setting this bit will trigger an IP Command.
 */
#define FLEXSPI_IPCMD_TRG(x)                     (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPCMD_TRG_SHIFT)) & FLEXSPI_IPCMD_TRG_MASK)
/*! @} */

/*! @name IPRXFCR - IP RX FIFO Control Register */
/*! @{ */
#define FLEXSPI_IPRXFCR_CLRIPRXF_MASK            (0x1U)
#define FLEXSPI_IPRXFCR_CLRIPRXF_SHIFT           (0U)
/*! CLRIPRXF - Clear all valid data entries in IP RX FIFO.
 */
#define FLEXSPI_IPRXFCR_CLRIPRXF(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFCR_CLRIPRXF_SHIFT)) & FLEXSPI_IPRXFCR_CLRIPRXF_MASK)
#define FLEXSPI_IPRXFCR_RXDMAEN_MASK             (0x2U)
#define FLEXSPI_IPRXFCR_RXDMAEN_SHIFT            (1U)
/*! RXDMAEN - IP RX FIFO reading by DMA enabled.
 *  0b0..IP RX FIFO would be read by processor.
 *  0b1..IP RX FIFO would be read by DMA.
 */
#define FLEXSPI_IPRXFCR_RXDMAEN(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFCR_RXDMAEN_SHIFT)) & FLEXSPI_IPRXFCR_RXDMAEN_MASK)
#define FLEXSPI_IPRXFCR_RXWMRK_MASK              (0x3CU)
#define FLEXSPI_IPRXFCR_RXWMRK_SHIFT             (2U)
/*! RXWMRK - Watermark level is (RXWMRK+1)*64 Bits.
 */
#define FLEXSPI_IPRXFCR_RXWMRK(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFCR_RXWMRK_SHIFT)) & FLEXSPI_IPRXFCR_RXWMRK_MASK)
/*! @} */

/*! @name IPTXFCR - IP TX FIFO Control Register */
/*! @{ */
#define FLEXSPI_IPTXFCR_CLRIPTXF_MASK            (0x1U)
#define FLEXSPI_IPTXFCR_CLRIPTXF_SHIFT           (0U)
/*! CLRIPTXF - Clear all valid data entries in IP TX FIFO.
 */
#define FLEXSPI_IPTXFCR_CLRIPTXF(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFCR_CLRIPTXF_SHIFT)) & FLEXSPI_IPTXFCR_CLRIPTXF_MASK)
#define FLEXSPI_IPTXFCR_TXDMAEN_MASK             (0x2U)
#define FLEXSPI_IPTXFCR_TXDMAEN_SHIFT            (1U)
/*! TXDMAEN - IP TX FIFO filling by DMA enabled.
 *  0b0..IP TX FIFO would be filled by processor.
 *  0b1..IP TX FIFO would be filled by DMA.
 */
#define FLEXSPI_IPTXFCR_TXDMAEN(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFCR_TXDMAEN_SHIFT)) & FLEXSPI_IPTXFCR_TXDMAEN_MASK)
#define FLEXSPI_IPTXFCR_TXWMRK_MASK              (0x3CU)
#define FLEXSPI_IPTXFCR_TXWMRK_SHIFT             (2U)
/*! TXWMRK - Watermark level is (TXWMRK+1)*64 Bits.
 */
#define FLEXSPI_IPTXFCR_TXWMRK(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFCR_TXWMRK_SHIFT)) & FLEXSPI_IPTXFCR_TXWMRK_MASK)
/*! @} */

/*! @name DLLCR - DLL Control Register 0 */
/*! @{ */
#define FLEXSPI_DLLCR_DLLEN_MASK                 (0x1U)
#define FLEXSPI_DLLCR_DLLEN_SHIFT                (0U)
/*! DLLEN - DLL calibration enable.
 */
#define FLEXSPI_DLLCR_DLLEN(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_DLLEN_SHIFT)) & FLEXSPI_DLLCR_DLLEN_MASK)
#define FLEXSPI_DLLCR_DLLRESET_MASK              (0x2U)
#define FLEXSPI_DLLCR_DLLRESET_SHIFT             (1U)
/*! DLLRESET - Software could force a reset on DLL by setting this field to 0x1. This will cause the
 *    DLL to lose lock and re-calibrate to detect an ref_clock half period phase shift. The reset
 *    action is edge triggered, so software need to clear this bit after set this bit (no delay
 *    limitation).
 */
#define FLEXSPI_DLLCR_DLLRESET(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_DLLRESET_SHIFT)) & FLEXSPI_DLLCR_DLLRESET_MASK)
#define FLEXSPI_DLLCR_SLVDLYTARGET_MASK          (0x78U)
#define FLEXSPI_DLLCR_SLVDLYTARGET_SHIFT         (3U)
/*! SLVDLYTARGET - The delay target for slave delay line is: ((SLVDLYTARGET+1) * 1/32 * clock cycle of reference clock (serial clock).
 */
#define FLEXSPI_DLLCR_SLVDLYTARGET(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_SLVDLYTARGET_SHIFT)) & FLEXSPI_DLLCR_SLVDLYTARGET_MASK)
#define FLEXSPI_DLLCR_OVRDEN_MASK                (0x100U)
#define FLEXSPI_DLLCR_OVRDEN_SHIFT               (8U)
/*! OVRDEN - Slave clock delay line delay cell number selection override enable.
 */
#define FLEXSPI_DLLCR_OVRDEN(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_OVRDEN_SHIFT)) & FLEXSPI_DLLCR_OVRDEN_MASK)
#define FLEXSPI_DLLCR_OVRDVAL_MASK               (0x7E00U)
#define FLEXSPI_DLLCR_OVRDVAL_SHIFT              (9U)
/*! OVRDVAL - Slave clock delay line delay cell number selection override value.
 */
#define FLEXSPI_DLLCR_OVRDVAL(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_DLLCR_OVRDVAL_SHIFT)) & FLEXSPI_DLLCR_OVRDVAL_MASK)
/*! @} */

/* The count of FLEXSPI_DLLCR */
#define FLEXSPI_DLLCR_COUNT                      (2U)

/*! @name STS0 - Status Register 0 */
/*! @{ */
#define FLEXSPI_STS0_SEQIDLE_MASK                (0x1U)
#define FLEXSPI_STS0_SEQIDLE_SHIFT               (0U)
/*! SEQIDLE - This status bit indicates the state machine in SEQ_CTL is idle and there is command
 *    sequence executing on FlexSPI interface.
 */
#define FLEXSPI_STS0_SEQIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS0_SEQIDLE_SHIFT)) & FLEXSPI_STS0_SEQIDLE_MASK)
#define FLEXSPI_STS0_ARBIDLE_MASK                (0x2U)
#define FLEXSPI_STS0_ARBIDLE_SHIFT               (1U)
/*! ARBIDLE - This status bit indicates the state machine in ARB_CTL is busy and there is command
 *    sequence granted by arbitrator and not finished yet on FlexSPI interface. When ARB_CTL state
 *    (ARBIDLE=0x1) is idle, there will be no transaction on FlexSPI interface also (SEQIDLE=0x1). So
 *    this bit should be polled to wait for FlexSPI controller become idle instead of SEQIDLE.
 */
#define FLEXSPI_STS0_ARBIDLE(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS0_ARBIDLE_SHIFT)) & FLEXSPI_STS0_ARBIDLE_MASK)
#define FLEXSPI_STS0_ARBCMDSRC_MASK              (0xCU)
#define FLEXSPI_STS0_ARBCMDSRC_SHIFT             (2U)
/*! ARBCMDSRC - This status field indicates the trigger source of current command sequence granted
 *    by arbitrator. This field value is meaningless when ARB_CTL is not busy (STS0[ARBIDLE]=0x1).
 *  0b00..Triggered by AHB read command (triggered by AHB read).
 *  0b01..Triggered by AHB write command (triggered by AHB Write).
 *  0b10..Triggered by IP command (triggered by setting register bit IPCMD.TRG).
 *  0b11..Triggered by suspended command (resumed).
 */
#define FLEXSPI_STS0_ARBCMDSRC(x)                (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS0_ARBCMDSRC_SHIFT)) & FLEXSPI_STS0_ARBCMDSRC_MASK)
/*! @} */

/*! @name STS1 - Status Register 1 */
/*! @{ */
#define FLEXSPI_STS1_AHBCMDERRID_MASK            (0xFU)
#define FLEXSPI_STS1_AHBCMDERRID_SHIFT           (0U)
/*! AHBCMDERRID - Indicates the sequence index when an AHB command error is detected. This field
 *    will be cleared when INTR[AHBCMDERR] is write-1-clear(w1c).
 */
#define FLEXSPI_STS1_AHBCMDERRID(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS1_AHBCMDERRID_SHIFT)) & FLEXSPI_STS1_AHBCMDERRID_MASK)
#define FLEXSPI_STS1_AHBCMDERRCODE_MASK          (0xF00U)
#define FLEXSPI_STS1_AHBCMDERRCODE_SHIFT         (8U)
/*! AHBCMDERRCODE - Indicates the Error Code when AHB command Error detected. This field will be
 *    cleared when INTR[AHBCMDERR] is write-1-clear(w1c).
 *  0b0000..No error.
 *  0b0010..AHB Write command with JMP_ON_CS instruction used in the sequence.
 *  0b0011..There is unknown instruction opcode in the sequence.
 *  0b0100..Instruction DUMMY_SDR/DUMMY_RWDS_SDR used in DDR sequence.
 *  0b0101..Instruction DUMMY_DDR/DUMMY_RWDS_DDR used in SDR sequence.
 *  0b1110..Sequence execution timeout.
 */
#define FLEXSPI_STS1_AHBCMDERRCODE(x)            (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS1_AHBCMDERRCODE_SHIFT)) & FLEXSPI_STS1_AHBCMDERRCODE_MASK)
#define FLEXSPI_STS1_IPCMDERRID_MASK             (0xF0000U)
#define FLEXSPI_STS1_IPCMDERRID_SHIFT            (16U)
/*! IPCMDERRID - Indicates the sequence Index when IP command error detected. This field will be
 *    cleared when INTR[IPCMDERR] is write-1-clear(w1c).
 */
#define FLEXSPI_STS1_IPCMDERRID(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS1_IPCMDERRID_SHIFT)) & FLEXSPI_STS1_IPCMDERRID_MASK)
#define FLEXSPI_STS1_IPCMDERRCODE_MASK           (0xF000000U)
#define FLEXSPI_STS1_IPCMDERRCODE_SHIFT          (24U)
/*! IPCMDERRCODE - Indicates the Error Code when IP command Error detected. This field will be
 *    cleared when INTR[IPCMDERR] is write-1-clear(w1c).
 *  0b0000..No error.
 *  0b0010..IP command with JMP_ON_CS instruction used in the sequence.
 *  0b0011..There is unknown instruction opcode in the sequence.
 *  0b0100..Instruction DUMMY_SDR/DUMMY_RWDS_SDR used in DDR sequence.
 *  0b0101..Instruction DUMMY_DDR/DUMMY_RWDS_DDR used in SDR sequence.
 *  0b0110..Flash access start address exceed the whole flash address range (A1/A2/B1/B2).
 *  0b1110..Sequence execution timeout.
 *  0b1111..Flash boundary crossed.
 */
#define FLEXSPI_STS1_IPCMDERRCODE(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS1_IPCMDERRCODE_SHIFT)) & FLEXSPI_STS1_IPCMDERRCODE_MASK)
/*! @} */

/*! @name STS2 - Status Register 2 */
/*! @{ */
#define FLEXSPI_STS2_ASLVLOCK_MASK               (0x1U)
#define FLEXSPI_STS2_ASLVLOCK_SHIFT              (0U)
/*! ASLVLOCK - Flash A sample clock slave delay line locked.
 */
#define FLEXSPI_STS2_ASLVLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_ASLVLOCK_SHIFT)) & FLEXSPI_STS2_ASLVLOCK_MASK)
#define FLEXSPI_STS2_AREFLOCK_MASK               (0x2U)
#define FLEXSPI_STS2_AREFLOCK_SHIFT              (1U)
/*! AREFLOCK - Flash A sample clock reference delay line locked.
 */
#define FLEXSPI_STS2_AREFLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_AREFLOCK_SHIFT)) & FLEXSPI_STS2_AREFLOCK_MASK)
#define FLEXSPI_STS2_ASLVSEL_MASK                (0xFCU)
#define FLEXSPI_STS2_ASLVSEL_SHIFT               (2U)
/*! ASLVSEL - Flash A sample clock slave delay line delay cell number selection .
 */
#define FLEXSPI_STS2_ASLVSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_ASLVSEL_SHIFT)) & FLEXSPI_STS2_ASLVSEL_MASK)
#define FLEXSPI_STS2_AREFSEL_MASK                (0x3F00U)
#define FLEXSPI_STS2_AREFSEL_SHIFT               (8U)
/*! AREFSEL - Flash A sample clock reference delay line delay cell number selection.
 */
#define FLEXSPI_STS2_AREFSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_AREFSEL_SHIFT)) & FLEXSPI_STS2_AREFSEL_MASK)
#define FLEXSPI_STS2_BSLVLOCK_MASK               (0x10000U)
#define FLEXSPI_STS2_BSLVLOCK_SHIFT              (16U)
/*! BSLVLOCK - Flash B sample clock slave delay line locked.
 */
#define FLEXSPI_STS2_BSLVLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_BSLVLOCK_SHIFT)) & FLEXSPI_STS2_BSLVLOCK_MASK)
#define FLEXSPI_STS2_BREFLOCK_MASK               (0x20000U)
#define FLEXSPI_STS2_BREFLOCK_SHIFT              (17U)
/*! BREFLOCK - Flash B sample clock reference delay line locked.
 */
#define FLEXSPI_STS2_BREFLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_BREFLOCK_SHIFT)) & FLEXSPI_STS2_BREFLOCK_MASK)
#define FLEXSPI_STS2_BSLVSEL_MASK                (0xFC0000U)
#define FLEXSPI_STS2_BSLVSEL_SHIFT               (18U)
/*! BSLVSEL - Flash B sample clock slave delay line delay cell number selection.
 */
#define FLEXSPI_STS2_BSLVSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_BSLVSEL_SHIFT)) & FLEXSPI_STS2_BSLVSEL_MASK)
#define FLEXSPI_STS2_BREFSEL_MASK                (0x3F000000U)
#define FLEXSPI_STS2_BREFSEL_SHIFT               (24U)
/*! BREFSEL - Flash B sample clock reference delay line delay cell number selection.
 */
#define FLEXSPI_STS2_BREFSEL(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_STS2_BREFSEL_SHIFT)) & FLEXSPI_STS2_BREFSEL_MASK)
/*! @} */

/*! @name AHBSPNDSTS - AHB Suspend Status Register */
/*! @{ */
#define FLEXSPI_AHBSPNDSTS_ACTIVE_MASK           (0x1U)
#define FLEXSPI_AHBSPNDSTS_ACTIVE_SHIFT          (0U)
/*! ACTIVE - Indicates if an AHB read prefetch command sequence has been suspended.
 */
#define FLEXSPI_AHBSPNDSTS_ACTIVE(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBSPNDSTS_ACTIVE_SHIFT)) & FLEXSPI_AHBSPNDSTS_ACTIVE_MASK)
#define FLEXSPI_AHBSPNDSTS_BUFID_MASK            (0xEU)
#define FLEXSPI_AHBSPNDSTS_BUFID_SHIFT           (1U)
/*! BUFID - AHB RX BUF ID for suspended command sequence.
 */
#define FLEXSPI_AHBSPNDSTS_BUFID(x)              (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBSPNDSTS_BUFID_SHIFT)) & FLEXSPI_AHBSPNDSTS_BUFID_MASK)
#define FLEXSPI_AHBSPNDSTS_DATLFT_MASK           (0xFFFF0000U)
#define FLEXSPI_AHBSPNDSTS_DATLFT_SHIFT          (16U)
/*! DATLFT - Left Data size for suspended command sequence (in byte).
 */
#define FLEXSPI_AHBSPNDSTS_DATLFT(x)             (((uint32_t)(((uint32_t)(x)) << FLEXSPI_AHBSPNDSTS_DATLFT_SHIFT)) & FLEXSPI_AHBSPNDSTS_DATLFT_MASK)
/*! @} */

/*! @name IPRXFSTS - IP RX FIFO Status Register */
/*! @{ */
#define FLEXSPI_IPRXFSTS_FILL_MASK               (0xFFU)
#define FLEXSPI_IPRXFSTS_FILL_SHIFT              (0U)
/*! FILL - Fill level of IP RX FIFO.
 */
#define FLEXSPI_IPRXFSTS_FILL(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFSTS_FILL_SHIFT)) & FLEXSPI_IPRXFSTS_FILL_MASK)
#define FLEXSPI_IPRXFSTS_RDCNTR_MASK             (0xFFFF0000U)
#define FLEXSPI_IPRXFSTS_RDCNTR_SHIFT            (16U)
/*! RDCNTR - Total Read Data Counter: RDCNTR * 64 Bits.
 */
#define FLEXSPI_IPRXFSTS_RDCNTR(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPRXFSTS_RDCNTR_SHIFT)) & FLEXSPI_IPRXFSTS_RDCNTR_MASK)
/*! @} */

/*! @name IPTXFSTS - IP TX FIFO Status Register */
/*! @{ */
#define FLEXSPI_IPTXFSTS_FILL_MASK               (0xFFU)
#define FLEXSPI_IPTXFSTS_FILL_SHIFT              (0U)
/*! FILL - Fill level of IP TX FIFO.
 */
#define FLEXSPI_IPTXFSTS_FILL(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFSTS_FILL_SHIFT)) & FLEXSPI_IPTXFSTS_FILL_MASK)
#define FLEXSPI_IPTXFSTS_WRCNTR_MASK             (0xFFFF0000U)
#define FLEXSPI_IPTXFSTS_WRCNTR_SHIFT            (16U)
/*! WRCNTR - Total Write Data Counter: WRCNTR * 64 Bits.
 */
#define FLEXSPI_IPTXFSTS_WRCNTR(x)               (((uint32_t)(((uint32_t)(x)) << FLEXSPI_IPTXFSTS_WRCNTR_SHIFT)) & FLEXSPI_IPTXFSTS_WRCNTR_MASK)
/*! @} */

/*! @name RFDR - IP RX FIFO Data Register 0..IP RX FIFO Data Register 31 */
/*! @{ */
#define FLEXSPI_RFDR_RXDATA_MASK                 (0xFFFFFFFFU)
#define FLEXSPI_RFDR_RXDATA_SHIFT                (0U)
/*! RXDATA - RX Data
 */
#define FLEXSPI_RFDR_RXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_RFDR_RXDATA_SHIFT)) & FLEXSPI_RFDR_RXDATA_MASK)
/*! @} */

/* The count of FLEXSPI_RFDR */
#define FLEXSPI_RFDR_COUNT                       (32U)

/*! @name TFDR - IP TX FIFO Data Register 0..IP TX FIFO Data Register 31 */
/*! @{ */
#define FLEXSPI_TFDR_TXDATA_MASK                 (0xFFFFFFFFU)
#define FLEXSPI_TFDR_TXDATA_SHIFT                (0U)
/*! TXDATA - TX Data
 */
#define FLEXSPI_TFDR_TXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_TFDR_TXDATA_SHIFT)) & FLEXSPI_TFDR_TXDATA_MASK)
/*! @} */

/* The count of FLEXSPI_TFDR */
#define FLEXSPI_TFDR_COUNT                       (32U)

/*! @name LUT - LUT 0..LUT 63 */
/*! @{ */
#define FLEXSPI_LUT_OPERAND0_MASK                (0xFFU)
#define FLEXSPI_LUT_OPERAND0_SHIFT               (0U)
/*! OPERAND0 - OPERAND0
 */
#define FLEXSPI_LUT_OPERAND0(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPERAND0_SHIFT)) & FLEXSPI_LUT_OPERAND0_MASK)
#define FLEXSPI_LUT_NUM_PADS0_MASK               (0x300U)
#define FLEXSPI_LUT_NUM_PADS0_SHIFT              (8U)
/*! NUM_PADS0 - NUM_PADS0
 */
#define FLEXSPI_LUT_NUM_PADS0(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_NUM_PADS0_SHIFT)) & FLEXSPI_LUT_NUM_PADS0_MASK)
#define FLEXSPI_LUT_OPCODE0_MASK                 (0xFC00U)
#define FLEXSPI_LUT_OPCODE0_SHIFT                (10U)
/*! OPCODE0 - OPCODE
 */
#define FLEXSPI_LUT_OPCODE0(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPCODE0_SHIFT)) & FLEXSPI_LUT_OPCODE0_MASK)
#define FLEXSPI_LUT_OPERAND1_MASK                (0xFF0000U)
#define FLEXSPI_LUT_OPERAND1_SHIFT               (16U)
/*! OPERAND1 - OPERAND1
 */
#define FLEXSPI_LUT_OPERAND1(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPERAND1_SHIFT)) & FLEXSPI_LUT_OPERAND1_MASK)
#define FLEXSPI_LUT_NUM_PADS1_MASK               (0x3000000U)
#define FLEXSPI_LUT_NUM_PADS1_SHIFT              (24U)
/*! NUM_PADS1 - NUM_PADS1
 */
#define FLEXSPI_LUT_NUM_PADS1(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_NUM_PADS1_SHIFT)) & FLEXSPI_LUT_NUM_PADS1_MASK)
#define FLEXSPI_LUT_OPCODE1_MASK                 (0xFC000000U)
#define FLEXSPI_LUT_OPCODE1_SHIFT                (26U)
/*! OPCODE1 - OPCODE1
 */
#define FLEXSPI_LUT_OPCODE1(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPCODE1_SHIFT)) & FLEXSPI_LUT_OPCODE1_MASK)
/*! @} */

/* The count of FLEXSPI_LUT */
#define FLEXSPI_LUT_COUNT                        (64U)


/*!
 * @}
 */ /* end of group FLEXSPI_Register_Masks */


/* FLEXSPI - Peripheral instance base addresses */
/** Peripheral FLEXSPI base address */
#define FLEXSPI_BASE                             (0x402A8000u)
/** Peripheral FLEXSPI base pointer */
#define FLEXSPI                                  ((FLEXSPI_Type *)FLEXSPI_BASE)
/** Array initializer of FLEXSPI peripheral base addresses */
#define FLEXSPI_BASE_ADDRS                       { FLEXSPI_BASE }
/** Array initializer of FLEXSPI peripheral base pointers */
#define FLEXSPI_BASE_PTRS                        { FLEXSPI }
/** Interrupt vectors for the FLEXSPI peripheral type */
#define FLEXSPI_IRQS                             { FLEXSPI_IRQn }
/* FlexSPI AMBA address. */
#define FlexSPI_AMBA_BASE                       (0x60000000U)
/* FlexSPI ASFM address. */
#define FlexSPI_ASFM_BASE                        (0x00000000U)
/* Base Address of AHB address space mapped to IP RX FIFO. */
#define FlexSPI_ARDF_BASE                        (0x7FC00000U)
/* Base Address of AHB address space mapped to IP TX FIFO. */
#define FlexSPI_ATDF_BASE                        (0x7F800000U)


/*!
 * @}
 */ /* end of group FLEXSPI_Peripheral_Access_Layer */



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
       uint8_t RESERVED_4[4];
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
/*! CAN_CLK_PODF - Divider for CAN clock podf.
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
/*! FLEXIO2_CLK_PODF - Divider for flexio2 clock. Divider should be updated when output clock is gated.
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
 *  0b00101..wrck_clk_root
 *  0b00110..lpi2c_clk_root
 *  0b01011..csi_clk_root
 *  0b01110..osc_clk
 *  0b10001..usdhc2_clk_root
 *  0b10010..sai1_clk_root
 *  0b10011..sai2_clk_root
 *  0b10100..sai3_clk_root
 *  0b10111..can_clk_root
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
#define CCM_ANALOG_PLL_SYS_PFD_OFFSET_EN_MASK    (0x40000U)
#define CCM_ANALOG_PLL_SYS_PFD_OFFSET_EN_SHIFT   (18U)
#define CCM_ANALOG_PLL_SYS_PFD_OFFSET_EN(x)      (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_SYS_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_SYS_SET_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_SYS_SET_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_SYS_SET_PFD_OFFSET_EN(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_SET_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_SYS_SET_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_SYS_CLR_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_SYS_CLR_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_SYS_CLR_PFD_OFFSET_EN(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_CLR_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_SYS_CLR_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_SYS_TOG_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_SYS_TOG_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_SYS_TOG_PFD_OFFSET_EN(x)  (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_SYS_TOG_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_SYS_TOG_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_AUDIO_PFD_OFFSET_EN_MASK  (0x40000U)
#define CCM_ANALOG_PLL_AUDIO_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_AUDIO_PFD_OFFSET_EN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_AUDIO_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_AUDIO_SET_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_AUDIO_SET_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_AUDIO_SET_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_SET_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_AUDIO_SET_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_AUDIO_CLR_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_AUDIO_CLR_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_AUDIO_CLR_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_CLR_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_AUDIO_CLR_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_AUDIO_TOG_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_AUDIO_TOG_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_AUDIO_TOG_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_AUDIO_TOG_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_AUDIO_TOG_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_VIDEO_PFD_OFFSET_EN_MASK  (0x40000U)
#define CCM_ANALOG_PLL_VIDEO_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_VIDEO_PFD_OFFSET_EN(x)    (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_VIDEO_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_VIDEO_SET_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_VIDEO_SET_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_VIDEO_SET_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_SET_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_VIDEO_SET_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_VIDEO_CLR_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_VIDEO_CLR_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_VIDEO_CLR_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_CLR_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_VIDEO_CLR_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_VIDEO_TOG_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_VIDEO_TOG_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_VIDEO_TOG_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_VIDEO_TOG_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_VIDEO_TOG_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_ENET_PFD_OFFSET_EN_MASK   (0x40000U)
#define CCM_ANALOG_PLL_ENET_PFD_OFFSET_EN_SHIFT  (18U)
#define CCM_ANALOG_PLL_ENET_PFD_OFFSET_EN(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_ENET_SET_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_ENET_SET_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_ENET_SET_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_SET_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_SET_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_ENET_CLR_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_ENET_CLR_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_ENET_CLR_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_CLR_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_CLR_PFD_OFFSET_EN_MASK)
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
#define CCM_ANALOG_PLL_ENET_TOG_PFD_OFFSET_EN_MASK (0x40000U)
#define CCM_ANALOG_PLL_ENET_TOG_PFD_OFFSET_EN_SHIFT (18U)
#define CCM_ANALOG_PLL_ENET_TOG_PFD_OFFSET_EN(x) (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_PLL_ENET_TOG_PFD_OFFSET_EN_SHIFT)) & CCM_ANALOG_PLL_ENET_TOG_PFD_OFFSET_EN_MASK)
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
 *  0b00..All analog except rtc powered down on stop mode assertion. XtalOsc=on, RCOsc=off;
 *  0b01..Certain analog functions such as certain regulators left up. XtalOsc=on, RCOsc=off;
 *  0b10..XtalOsc=off, RCOsc=on, Old BG=on, New BG=off.
 *  0b11..XtalOsc=off, RCOsc=on, Old BG=off, New BG=on.
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
 *  0b00..All analog except rtc powered down on stop mode assertion. XtalOsc=on, RCOsc=off;
 *  0b01..Certain analog functions such as certain regulators left up. XtalOsc=on, RCOsc=off;
 *  0b10..XtalOsc=off, RCOsc=on, Old BG=on, New BG=off.
 *  0b11..XtalOsc=off, RCOsc=on, Old BG=off, New BG=on.
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
 *  0b00..All analog except rtc powered down on stop mode assertion. XtalOsc=on, RCOsc=off;
 *  0b01..Certain analog functions such as certain regulators left up. XtalOsc=on, RCOsc=off;
 *  0b10..XtalOsc=off, RCOsc=on, Old BG=on, New BG=off.
 *  0b11..XtalOsc=off, RCOsc=on, Old BG=off, New BG=on.
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
 *  0b00..All analog except rtc powered down on stop mode assertion. XtalOsc=on, RCOsc=off;
 *  0b01..Certain analog functions such as certain regulators left up. XtalOsc=on, RCOsc=off;
 *  0b10..XtalOsc=off, RCOsc=on, Old BG=on, New BG=off.
 *  0b11..XtalOsc=off, RCOsc=on, Old BG=off, New BG=on.
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
#define CCM_ANALOG_MISC2_PLL3_disable_MASK       (0x80U)
#define CCM_ANALOG_MISC2_PLL3_disable_SHIFT      (7U)
/*! PLL3_disable
 *  0b0..PLL3 is being used by peripherals and is enabled when SoC is not in any low power mode
 *  0b1..PLL3 can be disabled when the SoC is not in any low power mode
 */
#define CCM_ANALOG_MISC2_PLL3_disable(x)         (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_PLL3_disable_SHIFT)) & CCM_ANALOG_MISC2_PLL3_disable_MASK)
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
#define CCM_ANALOG_MISC2_SET_PLL3_disable_MASK   (0x80U)
#define CCM_ANALOG_MISC2_SET_PLL3_disable_SHIFT  (7U)
/*! PLL3_disable
 *  0b0..PLL3 is being used by peripherals and is enabled when SoC is not in any low power mode
 *  0b1..PLL3 can be disabled when the SoC is not in any low power mode
 */
#define CCM_ANALOG_MISC2_SET_PLL3_disable(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_SET_PLL3_disable_SHIFT)) & CCM_ANALOG_MISC2_SET_PLL3_disable_MASK)
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
#define CCM_ANALOG_MISC2_CLR_PLL3_disable_MASK   (0x80U)
#define CCM_ANALOG_MISC2_CLR_PLL3_disable_SHIFT  (7U)
/*! PLL3_disable
 *  0b0..PLL3 is being used by peripherals and is enabled when SoC is not in any low power mode
 *  0b1..PLL3 can be disabled when the SoC is not in any low power mode
 */
#define CCM_ANALOG_MISC2_CLR_PLL3_disable(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_CLR_PLL3_disable_SHIFT)) & CCM_ANALOG_MISC2_CLR_PLL3_disable_MASK)
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
#define CCM_ANALOG_MISC2_TOG_PLL3_disable_MASK   (0x80U)
#define CCM_ANALOG_MISC2_TOG_PLL3_disable_SHIFT  (7U)
/*! PLL3_disable
 *  0b0..PLL3 is being used by peripherals and is enabled when SoC is not in any low power mode
 *  0b1..PLL3 can be disabled when the SoC is not in any low power mode
 */
#define CCM_ANALOG_MISC2_TOG_PLL3_disable(x)     (((uint32_t)(((uint32_t)(x)) << CCM_ANALOG_MISC2_TOG_PLL3_disable_SHIFT)) & CCM_ANALOG_MISC2_TOG_PLL3_disable_MASK)
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
/*! WDZST - WDZST
 *  0b0..Continue timer operation (Default).
 *  0b1..Suspend the watchdog timer.
 */
#define WDOG_WCR_WDZST(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDZST_SHIFT)) & WDOG_WCR_WDZST_MASK)
#define WDOG_WCR_WDBG_MASK                       (0x2U)
#define WDOG_WCR_WDBG_SHIFT                      (1U)
/*! WDBG - WDBG
 *  0b0..Continue WDOG timer operation (Default).
 *  0b1..Suspend the watchdog timer.
 */
#define WDOG_WCR_WDBG(x)                         (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDBG_SHIFT)) & WDOG_WCR_WDBG_MASK)
#define WDOG_WCR_WDE_MASK                        (0x4U)
#define WDOG_WCR_WDE_SHIFT                       (2U)
/*! WDE - WDE
 *  0b0..Disable the Watchdog (Default).
 *  0b1..Enable the Watchdog.
 */
#define WDOG_WCR_WDE(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDE_SHIFT)) & WDOG_WCR_WDE_MASK)
#define WDOG_WCR_WDT_MASK                        (0x8U)
#define WDOG_WCR_WDT_SHIFT                       (3U)
/*! WDT - WDT
 *  0b0..No effect on WDOG_B (Default).
 *  0b1..Assert WDOG_B upon a Watchdog Time-out event.
 */
#define WDOG_WCR_WDT(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDT_SHIFT)) & WDOG_WCR_WDT_MASK)
#define WDOG_WCR_SRS_MASK                        (0x10U)
#define WDOG_WCR_SRS_SHIFT                       (4U)
/*! SRS - SRS
 *  0b0..Assert system reset signal.
 *  0b1..No effect on the system (Default).
 */
#define WDOG_WCR_SRS(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_SRS_SHIFT)) & WDOG_WCR_SRS_MASK)
#define WDOG_WCR_WDA_MASK                        (0x20U)
#define WDOG_WCR_WDA_SHIFT                       (5U)
/*! WDA - WDA
 *  0b0..Assert WDOG_B output.
 *  0b1..No effect on system (Default).
 */
#define WDOG_WCR_WDA(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDA_SHIFT)) & WDOG_WCR_WDA_MASK)
#define WDOG_WCR_SRE_MASK                        (0x40U)
#define WDOG_WCR_SRE_SHIFT                       (6U)
/*! SRE - software reset extension, an option way to generate software reset
 *  0b0..using original way to generate software reset (default)
 *  0b1..using new way to generate software reset.
 */
#define WDOG_WCR_SRE(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_SRE_SHIFT)) & WDOG_WCR_SRE_MASK)
#define WDOG_WCR_WDW_MASK                        (0x80U)
#define WDOG_WCR_WDW_SHIFT                       (7U)
/*! WDW - WDW
 *  0b0..Continue WDOG timer operation (Default).
 *  0b1..Suspend WDOG timer operation.
 */
#define WDOG_WCR_WDW(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WCR_WDW_SHIFT)) & WDOG_WCR_WDW_MASK)
#define WDOG_WCR_WT_MASK                         (0xFF00U)
#define WDOG_WCR_WT_SHIFT                        (8U)
/*! WT - WT
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
/*! WSR - WSR
 *  0b0101010101010101..Write to the Watchdog Service Register (WDOG_WSR).
 *  0b1010101010101010..Write to the Watchdog Service Register (WDOG_WSR).
 */
#define WDOG_WSR_WSR(x)                          (((uint16_t)(((uint16_t)(x)) << WDOG_WSR_WSR_SHIFT)) & WDOG_WSR_WSR_MASK)
/*! @} */

/*! @name WRSR - Watchdog Reset Status Register */
/*! @{ */
#define WDOG_WRSR_SFTW_MASK                      (0x1U)
#define WDOG_WRSR_SFTW_SHIFT                     (0U)
/*! SFTW - SFTW
 *  0b0..Reset is not the result of a software reset.
 *  0b1..Reset is the result of a software reset.
 */
#define WDOG_WRSR_SFTW(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WRSR_SFTW_SHIFT)) & WDOG_WRSR_SFTW_MASK)
#define WDOG_WRSR_TOUT_MASK                      (0x2U)
#define WDOG_WRSR_TOUT_SHIFT                     (1U)
/*! TOUT - TOUT
 *  0b0..Reset is not the result of a WDOG timeout.
 *  0b1..Reset is the result of a WDOG timeout.
 */
#define WDOG_WRSR_TOUT(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WRSR_TOUT_SHIFT)) & WDOG_WRSR_TOUT_MASK)
#define WDOG_WRSR_POR_MASK                       (0x10U)
#define WDOG_WRSR_POR_SHIFT                      (4U)
/*! POR - POR
 *  0b0..Reset is not the result of a power on reset.
 *  0b1..Reset is the result of a power on reset.
 */
#define WDOG_WRSR_POR(x)                         (((uint16_t)(((uint16_t)(x)) << WDOG_WRSR_POR_SHIFT)) & WDOG_WRSR_POR_MASK)
/*! @} */

/*! @name WICR - Watchdog Interrupt Control Register */
/*! @{ */
#define WDOG_WICR_WICT_MASK                      (0xFFU)
#define WDOG_WICR_WICT_SHIFT                     (0U)
/*! WICT - WICT
 *  0b00000000..WICT[7:0] = Time duration between interrupt and time-out is 0 seconds.
 *  0b00000001..WICT[7:0] = Time duration between interrupt and time-out is 0.5 seconds.
 *  0b00000100..WICT[7:0] = Time duration between interrupt and time-out is 2 seconds (Default).
 *  0b11111111..WICT[7:0] = Time duration between interrupt and time-out is 127.5 seconds.
 */
#define WDOG_WICR_WICT(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WICR_WICT_SHIFT)) & WDOG_WICR_WICT_MASK)
#define WDOG_WICR_WTIS_MASK                      (0x4000U)
#define WDOG_WICR_WTIS_SHIFT                     (14U)
/*! WTIS - WTIS
 *  0b0..No interrupt has occurred (Default).
 *  0b1..Interrupt has occurred
 */
#define WDOG_WICR_WTIS(x)                        (((uint16_t)(((uint16_t)(x)) << WDOG_WICR_WTIS_SHIFT)) & WDOG_WICR_WTIS_MASK)
#define WDOG_WICR_WIE_MASK                       (0x8000U)
#define WDOG_WICR_WIE_SHIFT                      (15U)
/*! WIE - WIE
 *  0b0..Disable Interrupt (Default).
 *  0b1..Enable Interrupt.
 */
#define WDOG_WICR_WIE(x)                         (((uint16_t)(((uint16_t)(x)) << WDOG_WICR_WIE_SHIFT)) & WDOG_WICR_WIE_MASK)
/*! @} */

/*! @name WMCR - Watchdog Miscellaneous Control Register */
/*! @{ */
#define WDOG_WMCR_PDE_MASK                       (0x1U)
#define WDOG_WMCR_PDE_SHIFT                      (0U)
/*! PDE - PDE
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
   -- XTALOSC24M Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XTALOSC24M_Peripheral_Access_Layer XTALOSC24M Peripheral Access Layer
 * @{
 */

/** XTALOSC24M - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[336];
  __IO uint32_t MISC0;                             /**< Miscellaneous Register 0, offset: 0x150 */
  __IO uint32_t MISC0_SET;                         /**< Miscellaneous Register 0, offset: 0x154 */
  __IO uint32_t MISC0_CLR;                         /**< Miscellaneous Register 0, offset: 0x158 */
  __IO uint32_t MISC0_TOG;                         /**< Miscellaneous Register 0, offset: 0x15C */
       uint8_t RESERVED_1[272];
  __IO uint32_t LOWPWR_CTRL;                       /**< XTAL OSC (LP) Control Register, offset: 0x270 */
  __IO uint32_t LOWPWR_CTRL_SET;                   /**< XTAL OSC (LP) Control Register, offset: 0x274 */
  __IO uint32_t LOWPWR_CTRL_CLR;                   /**< XTAL OSC (LP) Control Register, offset: 0x278 */
  __IO uint32_t LOWPWR_CTRL_TOG;                   /**< XTAL OSC (LP) Control Register, offset: 0x27C */
       uint8_t RESERVED_2[32];
  __IO uint32_t OSC_CONFIG0;                       /**< XTAL OSC Configuration 0 Register, offset: 0x2A0 */
  __IO uint32_t OSC_CONFIG0_SET;                   /**< XTAL OSC Configuration 0 Register, offset: 0x2A4 */
  __IO uint32_t OSC_CONFIG0_CLR;                   /**< XTAL OSC Configuration 0 Register, offset: 0x2A8 */
  __IO uint32_t OSC_CONFIG0_TOG;                   /**< XTAL OSC Configuration 0 Register, offset: 0x2AC */
  __IO uint32_t OSC_CONFIG1;                       /**< XTAL OSC Configuration 1 Register, offset: 0x2B0 */
  __IO uint32_t OSC_CONFIG1_SET;                   /**< XTAL OSC Configuration 1 Register, offset: 0x2B4 */
  __IO uint32_t OSC_CONFIG1_CLR;                   /**< XTAL OSC Configuration 1 Register, offset: 0x2B8 */
  __IO uint32_t OSC_CONFIG1_TOG;                   /**< XTAL OSC Configuration 1 Register, offset: 0x2BC */
  __IO uint32_t OSC_CONFIG2;                       /**< XTAL OSC Configuration 2 Register, offset: 0x2C0 */
  __IO uint32_t OSC_CONFIG2_SET;                   /**< XTAL OSC Configuration 2 Register, offset: 0x2C4 */
  __IO uint32_t OSC_CONFIG2_CLR;                   /**< XTAL OSC Configuration 2 Register, offset: 0x2C8 */
  __IO uint32_t OSC_CONFIG2_TOG;                   /**< XTAL OSC Configuration 2 Register, offset: 0x2CC */
} XTALOSC24M_Type;

/* ----------------------------------------------------------------------------
   -- XTALOSC24M Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XTALOSC24M_Register_Masks XTALOSC24M Register Masks
 * @{
 */

/*! @name MISC0 - Miscellaneous Register 0 */
/*! @{ */
#define XTALOSC24M_MISC0_REFTOP_PWD_MASK         (0x1U)
#define XTALOSC24M_MISC0_REFTOP_PWD_SHIFT        (0U)
#define XTALOSC24M_MISC0_REFTOP_PWD(x)           (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_REFTOP_PWD_SHIFT)) & XTALOSC24M_MISC0_REFTOP_PWD_MASK)
#define XTALOSC24M_MISC0_REFTOP_SELFBIASOFF_MASK (0x8U)
#define XTALOSC24M_MISC0_REFTOP_SELFBIASOFF_SHIFT (3U)
/*! REFTOP_SELFBIASOFF
 *  0b0..Uses coarse bias currents for startup
 *  0b1..Uses bandgap-based bias currents for best performance.
 */
#define XTALOSC24M_MISC0_REFTOP_SELFBIASOFF(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_REFTOP_SELFBIASOFF_SHIFT)) & XTALOSC24M_MISC0_REFTOP_SELFBIASOFF_MASK)
#define XTALOSC24M_MISC0_REFTOP_VBGADJ_MASK      (0x70U)
#define XTALOSC24M_MISC0_REFTOP_VBGADJ_SHIFT     (4U)
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
#define XTALOSC24M_MISC0_REFTOP_VBGADJ(x)        (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_REFTOP_VBGADJ_SHIFT)) & XTALOSC24M_MISC0_REFTOP_VBGADJ_MASK)
#define XTALOSC24M_MISC0_REFTOP_VBGUP_MASK       (0x80U)
#define XTALOSC24M_MISC0_REFTOP_VBGUP_SHIFT      (7U)
#define XTALOSC24M_MISC0_REFTOP_VBGUP(x)         (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_REFTOP_VBGUP_SHIFT)) & XTALOSC24M_MISC0_REFTOP_VBGUP_MASK)
#define XTALOSC24M_MISC0_STOP_MODE_CONFIG_MASK   (0xC00U)
#define XTALOSC24M_MISC0_STOP_MODE_CONFIG_SHIFT  (10U)
/*! STOP_MODE_CONFIG
 *  0b00..All analog except rtc powered down on stop mode assertion. XtalOsc=on, RCOsc=off;
 *  0b01..Certain analog functions such as certain regulators left up. XtalOsc=on, RCOsc=off;
 *  0b10..XtalOsc=off, RCOsc=on, Old BG=on, New BG=off.
 *  0b11..XtalOsc=off, RCOsc=on, Old BG=off, New BG=on.
 */
#define XTALOSC24M_MISC0_STOP_MODE_CONFIG(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_STOP_MODE_CONFIG_SHIFT)) & XTALOSC24M_MISC0_STOP_MODE_CONFIG_MASK)
#define XTALOSC24M_MISC0_DISCON_HIGH_SNVS_MASK   (0x1000U)
#define XTALOSC24M_MISC0_DISCON_HIGH_SNVS_SHIFT  (12U)
/*! DISCON_HIGH_SNVS
 *  0b0..Turn on the switch
 *  0b1..Turn off the switch
 */
#define XTALOSC24M_MISC0_DISCON_HIGH_SNVS(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_DISCON_HIGH_SNVS_SHIFT)) & XTALOSC24M_MISC0_DISCON_HIGH_SNVS_MASK)
#define XTALOSC24M_MISC0_OSC_I_MASK              (0x6000U)
#define XTALOSC24M_MISC0_OSC_I_SHIFT             (13U)
/*! OSC_I
 *  0b00..Nominal
 *  0b01..Decrease current by 12.5%
 *  0b10..Decrease current by 25.0%
 *  0b11..Decrease current by 37.5%
 */
#define XTALOSC24M_MISC0_OSC_I(x)                (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_OSC_I_SHIFT)) & XTALOSC24M_MISC0_OSC_I_MASK)
#define XTALOSC24M_MISC0_OSC_XTALOK_MASK         (0x8000U)
#define XTALOSC24M_MISC0_OSC_XTALOK_SHIFT        (15U)
#define XTALOSC24M_MISC0_OSC_XTALOK(x)           (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_OSC_XTALOK_SHIFT)) & XTALOSC24M_MISC0_OSC_XTALOK_MASK)
#define XTALOSC24M_MISC0_OSC_XTALOK_EN_MASK      (0x10000U)
#define XTALOSC24M_MISC0_OSC_XTALOK_EN_SHIFT     (16U)
#define XTALOSC24M_MISC0_OSC_XTALOK_EN(x)        (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_OSC_XTALOK_EN_SHIFT)) & XTALOSC24M_MISC0_OSC_XTALOK_EN_MASK)
#define XTALOSC24M_MISC0_CLKGATE_CTRL_MASK       (0x2000000U)
#define XTALOSC24M_MISC0_CLKGATE_CTRL_SHIFT      (25U)
/*! CLKGATE_CTRL
 *  0b0..Allow the logic to automatically gate the clock when the XTAL is powered down.
 *  0b1..Prevent the logic from ever gating off the clock.
 */
#define XTALOSC24M_MISC0_CLKGATE_CTRL(x)         (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLKGATE_CTRL_SHIFT)) & XTALOSC24M_MISC0_CLKGATE_CTRL_MASK)
#define XTALOSC24M_MISC0_CLKGATE_DELAY_MASK      (0x1C000000U)
#define XTALOSC24M_MISC0_CLKGATE_DELAY_SHIFT     (26U)
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
#define XTALOSC24M_MISC0_CLKGATE_DELAY(x)        (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLKGATE_DELAY_SHIFT)) & XTALOSC24M_MISC0_CLKGATE_DELAY_MASK)
#define XTALOSC24M_MISC0_RTC_XTAL_SOURCE_MASK    (0x20000000U)
#define XTALOSC24M_MISC0_RTC_XTAL_SOURCE_SHIFT   (29U)
/*! RTC_XTAL_SOURCE
 *  0b0..Internal ring oscillator
 *  0b1..RTC_XTAL
 */
#define XTALOSC24M_MISC0_RTC_XTAL_SOURCE(x)      (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_RTC_XTAL_SOURCE_SHIFT)) & XTALOSC24M_MISC0_RTC_XTAL_SOURCE_MASK)
#define XTALOSC24M_MISC0_XTAL_24M_PWD_MASK       (0x40000000U)
#define XTALOSC24M_MISC0_XTAL_24M_PWD_SHIFT      (30U)
#define XTALOSC24M_MISC0_XTAL_24M_PWD(x)         (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_XTAL_24M_PWD_SHIFT)) & XTALOSC24M_MISC0_XTAL_24M_PWD_MASK)
#define XTALOSC24M_MISC0_VID_PLL_PREDIV_MASK     (0x80000000U)
#define XTALOSC24M_MISC0_VID_PLL_PREDIV_SHIFT    (31U)
/*! VID_PLL_PREDIV
 *  0b0..Divide by 1
 *  0b1..Divide by 2
 */
#define XTALOSC24M_MISC0_VID_PLL_PREDIV(x)       (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_VID_PLL_PREDIV_SHIFT)) & XTALOSC24M_MISC0_VID_PLL_PREDIV_MASK)
/*! @} */

/*! @name MISC0_SET - Miscellaneous Register 0 */
/*! @{ */
#define XTALOSC24M_MISC0_SET_REFTOP_PWD_MASK     (0x1U)
#define XTALOSC24M_MISC0_SET_REFTOP_PWD_SHIFT    (0U)
#define XTALOSC24M_MISC0_SET_REFTOP_PWD(x)       (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_REFTOP_PWD_SHIFT)) & XTALOSC24M_MISC0_SET_REFTOP_PWD_MASK)
#define XTALOSC24M_MISC0_SET_REFTOP_SELFBIASOFF_MASK (0x8U)
#define XTALOSC24M_MISC0_SET_REFTOP_SELFBIASOFF_SHIFT (3U)
/*! REFTOP_SELFBIASOFF
 *  0b0..Uses coarse bias currents for startup
 *  0b1..Uses bandgap-based bias currents for best performance.
 */
#define XTALOSC24M_MISC0_SET_REFTOP_SELFBIASOFF(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_REFTOP_SELFBIASOFF_SHIFT)) & XTALOSC24M_MISC0_SET_REFTOP_SELFBIASOFF_MASK)
#define XTALOSC24M_MISC0_SET_REFTOP_VBGADJ_MASK  (0x70U)
#define XTALOSC24M_MISC0_SET_REFTOP_VBGADJ_SHIFT (4U)
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
#define XTALOSC24M_MISC0_SET_REFTOP_VBGADJ(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_REFTOP_VBGADJ_SHIFT)) & XTALOSC24M_MISC0_SET_REFTOP_VBGADJ_MASK)
#define XTALOSC24M_MISC0_SET_REFTOP_VBGUP_MASK   (0x80U)
#define XTALOSC24M_MISC0_SET_REFTOP_VBGUP_SHIFT  (7U)
#define XTALOSC24M_MISC0_SET_REFTOP_VBGUP(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_REFTOP_VBGUP_SHIFT)) & XTALOSC24M_MISC0_SET_REFTOP_VBGUP_MASK)
#define XTALOSC24M_MISC0_SET_STOP_MODE_CONFIG_MASK (0xC00U)
#define XTALOSC24M_MISC0_SET_STOP_MODE_CONFIG_SHIFT (10U)
/*! STOP_MODE_CONFIG
 *  0b00..All analog except rtc powered down on stop mode assertion. XtalOsc=on, RCOsc=off;
 *  0b01..Certain analog functions such as certain regulators left up. XtalOsc=on, RCOsc=off;
 *  0b10..XtalOsc=off, RCOsc=on, Old BG=on, New BG=off.
 *  0b11..XtalOsc=off, RCOsc=on, Old BG=off, New BG=on.
 */
#define XTALOSC24M_MISC0_SET_STOP_MODE_CONFIG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_STOP_MODE_CONFIG_SHIFT)) & XTALOSC24M_MISC0_SET_STOP_MODE_CONFIG_MASK)
#define XTALOSC24M_MISC0_SET_DISCON_HIGH_SNVS_MASK (0x1000U)
#define XTALOSC24M_MISC0_SET_DISCON_HIGH_SNVS_SHIFT (12U)
/*! DISCON_HIGH_SNVS
 *  0b0..Turn on the switch
 *  0b1..Turn off the switch
 */
#define XTALOSC24M_MISC0_SET_DISCON_HIGH_SNVS(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_DISCON_HIGH_SNVS_SHIFT)) & XTALOSC24M_MISC0_SET_DISCON_HIGH_SNVS_MASK)
#define XTALOSC24M_MISC0_SET_OSC_I_MASK          (0x6000U)
#define XTALOSC24M_MISC0_SET_OSC_I_SHIFT         (13U)
/*! OSC_I
 *  0b00..Nominal
 *  0b01..Decrease current by 12.5%
 *  0b10..Decrease current by 25.0%
 *  0b11..Decrease current by 37.5%
 */
#define XTALOSC24M_MISC0_SET_OSC_I(x)            (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_OSC_I_SHIFT)) & XTALOSC24M_MISC0_SET_OSC_I_MASK)
#define XTALOSC24M_MISC0_SET_OSC_XTALOK_MASK     (0x8000U)
#define XTALOSC24M_MISC0_SET_OSC_XTALOK_SHIFT    (15U)
#define XTALOSC24M_MISC0_SET_OSC_XTALOK(x)       (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_OSC_XTALOK_SHIFT)) & XTALOSC24M_MISC0_SET_OSC_XTALOK_MASK)
#define XTALOSC24M_MISC0_SET_OSC_XTALOK_EN_MASK  (0x10000U)
#define XTALOSC24M_MISC0_SET_OSC_XTALOK_EN_SHIFT (16U)
#define XTALOSC24M_MISC0_SET_OSC_XTALOK_EN(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_OSC_XTALOK_EN_SHIFT)) & XTALOSC24M_MISC0_SET_OSC_XTALOK_EN_MASK)
#define XTALOSC24M_MISC0_SET_CLKGATE_CTRL_MASK   (0x2000000U)
#define XTALOSC24M_MISC0_SET_CLKGATE_CTRL_SHIFT  (25U)
/*! CLKGATE_CTRL
 *  0b0..Allow the logic to automatically gate the clock when the XTAL is powered down.
 *  0b1..Prevent the logic from ever gating off the clock.
 */
#define XTALOSC24M_MISC0_SET_CLKGATE_CTRL(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_CLKGATE_CTRL_SHIFT)) & XTALOSC24M_MISC0_SET_CLKGATE_CTRL_MASK)
#define XTALOSC24M_MISC0_SET_CLKGATE_DELAY_MASK  (0x1C000000U)
#define XTALOSC24M_MISC0_SET_CLKGATE_DELAY_SHIFT (26U)
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
#define XTALOSC24M_MISC0_SET_CLKGATE_DELAY(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_CLKGATE_DELAY_SHIFT)) & XTALOSC24M_MISC0_SET_CLKGATE_DELAY_MASK)
#define XTALOSC24M_MISC0_SET_RTC_XTAL_SOURCE_MASK (0x20000000U)
#define XTALOSC24M_MISC0_SET_RTC_XTAL_SOURCE_SHIFT (29U)
/*! RTC_XTAL_SOURCE
 *  0b0..Internal ring oscillator
 *  0b1..RTC_XTAL
 */
#define XTALOSC24M_MISC0_SET_RTC_XTAL_SOURCE(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_RTC_XTAL_SOURCE_SHIFT)) & XTALOSC24M_MISC0_SET_RTC_XTAL_SOURCE_MASK)
#define XTALOSC24M_MISC0_SET_XTAL_24M_PWD_MASK   (0x40000000U)
#define XTALOSC24M_MISC0_SET_XTAL_24M_PWD_SHIFT  (30U)
#define XTALOSC24M_MISC0_SET_XTAL_24M_PWD(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_XTAL_24M_PWD_SHIFT)) & XTALOSC24M_MISC0_SET_XTAL_24M_PWD_MASK)
#define XTALOSC24M_MISC0_SET_VID_PLL_PREDIV_MASK (0x80000000U)
#define XTALOSC24M_MISC0_SET_VID_PLL_PREDIV_SHIFT (31U)
/*! VID_PLL_PREDIV
 *  0b0..Divide by 1
 *  0b1..Divide by 2
 */
#define XTALOSC24M_MISC0_SET_VID_PLL_PREDIV(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_SET_VID_PLL_PREDIV_SHIFT)) & XTALOSC24M_MISC0_SET_VID_PLL_PREDIV_MASK)
/*! @} */

/*! @name MISC0_CLR - Miscellaneous Register 0 */
/*! @{ */
#define XTALOSC24M_MISC0_CLR_REFTOP_PWD_MASK     (0x1U)
#define XTALOSC24M_MISC0_CLR_REFTOP_PWD_SHIFT    (0U)
#define XTALOSC24M_MISC0_CLR_REFTOP_PWD(x)       (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_REFTOP_PWD_SHIFT)) & XTALOSC24M_MISC0_CLR_REFTOP_PWD_MASK)
#define XTALOSC24M_MISC0_CLR_REFTOP_SELFBIASOFF_MASK (0x8U)
#define XTALOSC24M_MISC0_CLR_REFTOP_SELFBIASOFF_SHIFT (3U)
/*! REFTOP_SELFBIASOFF
 *  0b0..Uses coarse bias currents for startup
 *  0b1..Uses bandgap-based bias currents for best performance.
 */
#define XTALOSC24M_MISC0_CLR_REFTOP_SELFBIASOFF(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_REFTOP_SELFBIASOFF_SHIFT)) & XTALOSC24M_MISC0_CLR_REFTOP_SELFBIASOFF_MASK)
#define XTALOSC24M_MISC0_CLR_REFTOP_VBGADJ_MASK  (0x70U)
#define XTALOSC24M_MISC0_CLR_REFTOP_VBGADJ_SHIFT (4U)
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
#define XTALOSC24M_MISC0_CLR_REFTOP_VBGADJ(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_REFTOP_VBGADJ_SHIFT)) & XTALOSC24M_MISC0_CLR_REFTOP_VBGADJ_MASK)
#define XTALOSC24M_MISC0_CLR_REFTOP_VBGUP_MASK   (0x80U)
#define XTALOSC24M_MISC0_CLR_REFTOP_VBGUP_SHIFT  (7U)
#define XTALOSC24M_MISC0_CLR_REFTOP_VBGUP(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_REFTOP_VBGUP_SHIFT)) & XTALOSC24M_MISC0_CLR_REFTOP_VBGUP_MASK)
#define XTALOSC24M_MISC0_CLR_STOP_MODE_CONFIG_MASK (0xC00U)
#define XTALOSC24M_MISC0_CLR_STOP_MODE_CONFIG_SHIFT (10U)
/*! STOP_MODE_CONFIG
 *  0b00..All analog except rtc powered down on stop mode assertion. XtalOsc=on, RCOsc=off;
 *  0b01..Certain analog functions such as certain regulators left up. XtalOsc=on, RCOsc=off;
 *  0b10..XtalOsc=off, RCOsc=on, Old BG=on, New BG=off.
 *  0b11..XtalOsc=off, RCOsc=on, Old BG=off, New BG=on.
 */
#define XTALOSC24M_MISC0_CLR_STOP_MODE_CONFIG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_STOP_MODE_CONFIG_SHIFT)) & XTALOSC24M_MISC0_CLR_STOP_MODE_CONFIG_MASK)
#define XTALOSC24M_MISC0_CLR_DISCON_HIGH_SNVS_MASK (0x1000U)
#define XTALOSC24M_MISC0_CLR_DISCON_HIGH_SNVS_SHIFT (12U)
/*! DISCON_HIGH_SNVS
 *  0b0..Turn on the switch
 *  0b1..Turn off the switch
 */
#define XTALOSC24M_MISC0_CLR_DISCON_HIGH_SNVS(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_DISCON_HIGH_SNVS_SHIFT)) & XTALOSC24M_MISC0_CLR_DISCON_HIGH_SNVS_MASK)
#define XTALOSC24M_MISC0_CLR_OSC_I_MASK          (0x6000U)
#define XTALOSC24M_MISC0_CLR_OSC_I_SHIFT         (13U)
/*! OSC_I
 *  0b00..Nominal
 *  0b01..Decrease current by 12.5%
 *  0b10..Decrease current by 25.0%
 *  0b11..Decrease current by 37.5%
 */
#define XTALOSC24M_MISC0_CLR_OSC_I(x)            (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_OSC_I_SHIFT)) & XTALOSC24M_MISC0_CLR_OSC_I_MASK)
#define XTALOSC24M_MISC0_CLR_OSC_XTALOK_MASK     (0x8000U)
#define XTALOSC24M_MISC0_CLR_OSC_XTALOK_SHIFT    (15U)
#define XTALOSC24M_MISC0_CLR_OSC_XTALOK(x)       (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_OSC_XTALOK_SHIFT)) & XTALOSC24M_MISC0_CLR_OSC_XTALOK_MASK)
#define XTALOSC24M_MISC0_CLR_OSC_XTALOK_EN_MASK  (0x10000U)
#define XTALOSC24M_MISC0_CLR_OSC_XTALOK_EN_SHIFT (16U)
#define XTALOSC24M_MISC0_CLR_OSC_XTALOK_EN(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_OSC_XTALOK_EN_SHIFT)) & XTALOSC24M_MISC0_CLR_OSC_XTALOK_EN_MASK)
#define XTALOSC24M_MISC0_CLR_CLKGATE_CTRL_MASK   (0x2000000U)
#define XTALOSC24M_MISC0_CLR_CLKGATE_CTRL_SHIFT  (25U)
/*! CLKGATE_CTRL
 *  0b0..Allow the logic to automatically gate the clock when the XTAL is powered down.
 *  0b1..Prevent the logic from ever gating off the clock.
 */
#define XTALOSC24M_MISC0_CLR_CLKGATE_CTRL(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_CLKGATE_CTRL_SHIFT)) & XTALOSC24M_MISC0_CLR_CLKGATE_CTRL_MASK)
#define XTALOSC24M_MISC0_CLR_CLKGATE_DELAY_MASK  (0x1C000000U)
#define XTALOSC24M_MISC0_CLR_CLKGATE_DELAY_SHIFT (26U)
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
#define XTALOSC24M_MISC0_CLR_CLKGATE_DELAY(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_CLKGATE_DELAY_SHIFT)) & XTALOSC24M_MISC0_CLR_CLKGATE_DELAY_MASK)
#define XTALOSC24M_MISC0_CLR_RTC_XTAL_SOURCE_MASK (0x20000000U)
#define XTALOSC24M_MISC0_CLR_RTC_XTAL_SOURCE_SHIFT (29U)
/*! RTC_XTAL_SOURCE
 *  0b0..Internal ring oscillator
 *  0b1..RTC_XTAL
 */
#define XTALOSC24M_MISC0_CLR_RTC_XTAL_SOURCE(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_RTC_XTAL_SOURCE_SHIFT)) & XTALOSC24M_MISC0_CLR_RTC_XTAL_SOURCE_MASK)
#define XTALOSC24M_MISC0_CLR_XTAL_24M_PWD_MASK   (0x40000000U)
#define XTALOSC24M_MISC0_CLR_XTAL_24M_PWD_SHIFT  (30U)
#define XTALOSC24M_MISC0_CLR_XTAL_24M_PWD(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_XTAL_24M_PWD_SHIFT)) & XTALOSC24M_MISC0_CLR_XTAL_24M_PWD_MASK)
#define XTALOSC24M_MISC0_CLR_VID_PLL_PREDIV_MASK (0x80000000U)
#define XTALOSC24M_MISC0_CLR_VID_PLL_PREDIV_SHIFT (31U)
/*! VID_PLL_PREDIV
 *  0b0..Divide by 1
 *  0b1..Divide by 2
 */
#define XTALOSC24M_MISC0_CLR_VID_PLL_PREDIV(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_CLR_VID_PLL_PREDIV_SHIFT)) & XTALOSC24M_MISC0_CLR_VID_PLL_PREDIV_MASK)
/*! @} */

/*! @name MISC0_TOG - Miscellaneous Register 0 */
/*! @{ */
#define XTALOSC24M_MISC0_TOG_REFTOP_PWD_MASK     (0x1U)
#define XTALOSC24M_MISC0_TOG_REFTOP_PWD_SHIFT    (0U)
#define XTALOSC24M_MISC0_TOG_REFTOP_PWD(x)       (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_REFTOP_PWD_SHIFT)) & XTALOSC24M_MISC0_TOG_REFTOP_PWD_MASK)
#define XTALOSC24M_MISC0_TOG_REFTOP_SELFBIASOFF_MASK (0x8U)
#define XTALOSC24M_MISC0_TOG_REFTOP_SELFBIASOFF_SHIFT (3U)
/*! REFTOP_SELFBIASOFF
 *  0b0..Uses coarse bias currents for startup
 *  0b1..Uses bandgap-based bias currents for best performance.
 */
#define XTALOSC24M_MISC0_TOG_REFTOP_SELFBIASOFF(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_REFTOP_SELFBIASOFF_SHIFT)) & XTALOSC24M_MISC0_TOG_REFTOP_SELFBIASOFF_MASK)
#define XTALOSC24M_MISC0_TOG_REFTOP_VBGADJ_MASK  (0x70U)
#define XTALOSC24M_MISC0_TOG_REFTOP_VBGADJ_SHIFT (4U)
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
#define XTALOSC24M_MISC0_TOG_REFTOP_VBGADJ(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_REFTOP_VBGADJ_SHIFT)) & XTALOSC24M_MISC0_TOG_REFTOP_VBGADJ_MASK)
#define XTALOSC24M_MISC0_TOG_REFTOP_VBGUP_MASK   (0x80U)
#define XTALOSC24M_MISC0_TOG_REFTOP_VBGUP_SHIFT  (7U)
#define XTALOSC24M_MISC0_TOG_REFTOP_VBGUP(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_REFTOP_VBGUP_SHIFT)) & XTALOSC24M_MISC0_TOG_REFTOP_VBGUP_MASK)
#define XTALOSC24M_MISC0_TOG_STOP_MODE_CONFIG_MASK (0xC00U)
#define XTALOSC24M_MISC0_TOG_STOP_MODE_CONFIG_SHIFT (10U)
/*! STOP_MODE_CONFIG
 *  0b00..All analog except rtc powered down on stop mode assertion. XtalOsc=on, RCOsc=off;
 *  0b01..Certain analog functions such as certain regulators left up. XtalOsc=on, RCOsc=off;
 *  0b10..XtalOsc=off, RCOsc=on, Old BG=on, New BG=off.
 *  0b11..XtalOsc=off, RCOsc=on, Old BG=off, New BG=on.
 */
#define XTALOSC24M_MISC0_TOG_STOP_MODE_CONFIG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_STOP_MODE_CONFIG_SHIFT)) & XTALOSC24M_MISC0_TOG_STOP_MODE_CONFIG_MASK)
#define XTALOSC24M_MISC0_TOG_DISCON_HIGH_SNVS_MASK (0x1000U)
#define XTALOSC24M_MISC0_TOG_DISCON_HIGH_SNVS_SHIFT (12U)
/*! DISCON_HIGH_SNVS
 *  0b0..Turn on the switch
 *  0b1..Turn off the switch
 */
#define XTALOSC24M_MISC0_TOG_DISCON_HIGH_SNVS(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_DISCON_HIGH_SNVS_SHIFT)) & XTALOSC24M_MISC0_TOG_DISCON_HIGH_SNVS_MASK)
#define XTALOSC24M_MISC0_TOG_OSC_I_MASK          (0x6000U)
#define XTALOSC24M_MISC0_TOG_OSC_I_SHIFT         (13U)
/*! OSC_I
 *  0b00..Nominal
 *  0b01..Decrease current by 12.5%
 *  0b10..Decrease current by 25.0%
 *  0b11..Decrease current by 37.5%
 */
#define XTALOSC24M_MISC0_TOG_OSC_I(x)            (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_OSC_I_SHIFT)) & XTALOSC24M_MISC0_TOG_OSC_I_MASK)
#define XTALOSC24M_MISC0_TOG_OSC_XTALOK_MASK     (0x8000U)
#define XTALOSC24M_MISC0_TOG_OSC_XTALOK_SHIFT    (15U)
#define XTALOSC24M_MISC0_TOG_OSC_XTALOK(x)       (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_OSC_XTALOK_SHIFT)) & XTALOSC24M_MISC0_TOG_OSC_XTALOK_MASK)
#define XTALOSC24M_MISC0_TOG_OSC_XTALOK_EN_MASK  (0x10000U)
#define XTALOSC24M_MISC0_TOG_OSC_XTALOK_EN_SHIFT (16U)
#define XTALOSC24M_MISC0_TOG_OSC_XTALOK_EN(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_OSC_XTALOK_EN_SHIFT)) & XTALOSC24M_MISC0_TOG_OSC_XTALOK_EN_MASK)
#define XTALOSC24M_MISC0_TOG_CLKGATE_CTRL_MASK   (0x2000000U)
#define XTALOSC24M_MISC0_TOG_CLKGATE_CTRL_SHIFT  (25U)
/*! CLKGATE_CTRL
 *  0b0..Allow the logic to automatically gate the clock when the XTAL is powered down.
 *  0b1..Prevent the logic from ever gating off the clock.
 */
#define XTALOSC24M_MISC0_TOG_CLKGATE_CTRL(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_CLKGATE_CTRL_SHIFT)) & XTALOSC24M_MISC0_TOG_CLKGATE_CTRL_MASK)
#define XTALOSC24M_MISC0_TOG_CLKGATE_DELAY_MASK  (0x1C000000U)
#define XTALOSC24M_MISC0_TOG_CLKGATE_DELAY_SHIFT (26U)
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
#define XTALOSC24M_MISC0_TOG_CLKGATE_DELAY(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_CLKGATE_DELAY_SHIFT)) & XTALOSC24M_MISC0_TOG_CLKGATE_DELAY_MASK)
#define XTALOSC24M_MISC0_TOG_RTC_XTAL_SOURCE_MASK (0x20000000U)
#define XTALOSC24M_MISC0_TOG_RTC_XTAL_SOURCE_SHIFT (29U)
/*! RTC_XTAL_SOURCE
 *  0b0..Internal ring oscillator
 *  0b1..RTC_XTAL
 */
#define XTALOSC24M_MISC0_TOG_RTC_XTAL_SOURCE(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_RTC_XTAL_SOURCE_SHIFT)) & XTALOSC24M_MISC0_TOG_RTC_XTAL_SOURCE_MASK)
#define XTALOSC24M_MISC0_TOG_XTAL_24M_PWD_MASK   (0x40000000U)
#define XTALOSC24M_MISC0_TOG_XTAL_24M_PWD_SHIFT  (30U)
#define XTALOSC24M_MISC0_TOG_XTAL_24M_PWD(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_XTAL_24M_PWD_SHIFT)) & XTALOSC24M_MISC0_TOG_XTAL_24M_PWD_MASK)
#define XTALOSC24M_MISC0_TOG_VID_PLL_PREDIV_MASK (0x80000000U)
#define XTALOSC24M_MISC0_TOG_VID_PLL_PREDIV_SHIFT (31U)
/*! VID_PLL_PREDIV
 *  0b0..Divide by 1
 *  0b1..Divide by 2
 */
#define XTALOSC24M_MISC0_TOG_VID_PLL_PREDIV(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_MISC0_TOG_VID_PLL_PREDIV_SHIFT)) & XTALOSC24M_MISC0_TOG_VID_PLL_PREDIV_MASK)
/*! @} */

/*! @name LOWPWR_CTRL - XTAL OSC (LP) Control Register */
/*! @{ */
#define XTALOSC24M_LOWPWR_CTRL_RC_OSC_EN_MASK    (0x1U)
#define XTALOSC24M_LOWPWR_CTRL_RC_OSC_EN_SHIFT   (0U)
/*! RC_OSC_EN
 *  0b0..Use XTAL OSC to source the 24MHz clock
 *  0b1..Use RC OSC
 */
#define XTALOSC24M_LOWPWR_CTRL_RC_OSC_EN(x)      (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_RC_OSC_EN_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_RC_OSC_EN_MASK)
#define XTALOSC24M_LOWPWR_CTRL_OSC_SEL_MASK      (0x10U)
#define XTALOSC24M_LOWPWR_CTRL_OSC_SEL_SHIFT     (4U)
/*! OSC_SEL
 *  0b0..XTAL OSC
 *  0b1..RC OSC
 */
#define XTALOSC24M_LOWPWR_CTRL_OSC_SEL(x)        (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_OSC_SEL_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_OSC_SEL_MASK)
#define XTALOSC24M_LOWPWR_CTRL_LPBG_SEL_MASK     (0x20U)
#define XTALOSC24M_LOWPWR_CTRL_LPBG_SEL_SHIFT    (5U)
/*! LPBG_SEL
 *  0b0..Normal power bandgap
 *  0b1..Low power bandgap
 */
#define XTALOSC24M_LOWPWR_CTRL_LPBG_SEL(x)       (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_LPBG_SEL_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_LPBG_SEL_MASK)
#define XTALOSC24M_LOWPWR_CTRL_LPBG_TEST_MASK    (0x40U)
#define XTALOSC24M_LOWPWR_CTRL_LPBG_TEST_SHIFT   (6U)
#define XTALOSC24M_LOWPWR_CTRL_LPBG_TEST(x)      (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_LPBG_TEST_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_LPBG_TEST_MASK)
#define XTALOSC24M_LOWPWR_CTRL_REFTOP_IBIAS_OFF_MASK (0x80U)
#define XTALOSC24M_LOWPWR_CTRL_REFTOP_IBIAS_OFF_SHIFT (7U)
#define XTALOSC24M_LOWPWR_CTRL_REFTOP_IBIAS_OFF(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_REFTOP_IBIAS_OFF_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_REFTOP_IBIAS_OFF_MASK)
#define XTALOSC24M_LOWPWR_CTRL_L1_PWRGATE_MASK   (0x100U)
#define XTALOSC24M_LOWPWR_CTRL_L1_PWRGATE_SHIFT  (8U)
#define XTALOSC24M_LOWPWR_CTRL_L1_PWRGATE(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_L1_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_L1_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_L2_PWRGATE_MASK   (0x200U)
#define XTALOSC24M_LOWPWR_CTRL_L2_PWRGATE_SHIFT  (9U)
#define XTALOSC24M_LOWPWR_CTRL_L2_PWRGATE(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_L2_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_L2_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CPU_PWRGATE_MASK  (0x400U)
#define XTALOSC24M_LOWPWR_CTRL_CPU_PWRGATE_SHIFT (10U)
#define XTALOSC24M_LOWPWR_CTRL_CPU_PWRGATE(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CPU_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CPU_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_DISPLAY_PWRGATE_MASK (0x800U)
#define XTALOSC24M_LOWPWR_CTRL_DISPLAY_PWRGATE_SHIFT (11U)
#define XTALOSC24M_LOWPWR_CTRL_DISPLAY_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_DISPLAY_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_DISPLAY_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_RCOSC_CG_OVERRIDE_MASK (0x2000U)
#define XTALOSC24M_LOWPWR_CTRL_RCOSC_CG_OVERRIDE_SHIFT (13U)
#define XTALOSC24M_LOWPWR_CTRL_RCOSC_CG_OVERRIDE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_RCOSC_CG_OVERRIDE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_RCOSC_CG_OVERRIDE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_DELAY_MASK (0xC000U)
#define XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_DELAY_SHIFT (14U)
/*! XTALOSC_PWRUP_DELAY
 *  0b00..0.25ms
 *  0b01..0.5ms
 *  0b10..1ms
 *  0b11..2ms
 */
#define XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_DELAY(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_DELAY_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_DELAY_MASK)
#define XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_STAT_MASK (0x10000U)
#define XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_STAT_SHIFT (16U)
/*! XTALOSC_PWRUP_STAT
 *  0b0..Not stable
 *  0b1..Stable and ready to use
 */
#define XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_STAT(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_STAT_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_XTALOSC_PWRUP_STAT_MASK)
#define XTALOSC24M_LOWPWR_CTRL_MIX_PWRGATE_MASK  (0x20000U)
#define XTALOSC24M_LOWPWR_CTRL_MIX_PWRGATE_SHIFT (17U)
#define XTALOSC24M_LOWPWR_CTRL_MIX_PWRGATE(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_MIX_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_MIX_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_GPU_PWRGATE_MASK  (0x40000U)
#define XTALOSC24M_LOWPWR_CTRL_GPU_PWRGATE_SHIFT (18U)
#define XTALOSC24M_LOWPWR_CTRL_GPU_PWRGATE(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_GPU_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_GPU_PWRGATE_MASK)
/*! @} */

/*! @name LOWPWR_CTRL_SET - XTAL OSC (LP) Control Register */
/*! @{ */
#define XTALOSC24M_LOWPWR_CTRL_SET_RC_OSC_EN_MASK (0x1U)
#define XTALOSC24M_LOWPWR_CTRL_SET_RC_OSC_EN_SHIFT (0U)
/*! RC_OSC_EN
 *  0b0..Use XTAL OSC to source the 24MHz clock
 *  0b1..Use RC OSC
 */
#define XTALOSC24M_LOWPWR_CTRL_SET_RC_OSC_EN(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_RC_OSC_EN_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_RC_OSC_EN_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_OSC_SEL_MASK  (0x10U)
#define XTALOSC24M_LOWPWR_CTRL_SET_OSC_SEL_SHIFT (4U)
/*! OSC_SEL
 *  0b0..XTAL OSC
 *  0b1..RC OSC
 */
#define XTALOSC24M_LOWPWR_CTRL_SET_OSC_SEL(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_OSC_SEL_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_OSC_SEL_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_LPBG_SEL_MASK (0x20U)
#define XTALOSC24M_LOWPWR_CTRL_SET_LPBG_SEL_SHIFT (5U)
/*! LPBG_SEL
 *  0b0..Normal power bandgap
 *  0b1..Low power bandgap
 */
#define XTALOSC24M_LOWPWR_CTRL_SET_LPBG_SEL(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_LPBG_SEL_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_LPBG_SEL_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_LPBG_TEST_MASK (0x40U)
#define XTALOSC24M_LOWPWR_CTRL_SET_LPBG_TEST_SHIFT (6U)
#define XTALOSC24M_LOWPWR_CTRL_SET_LPBG_TEST(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_LPBG_TEST_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_LPBG_TEST_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_REFTOP_IBIAS_OFF_MASK (0x80U)
#define XTALOSC24M_LOWPWR_CTRL_SET_REFTOP_IBIAS_OFF_SHIFT (7U)
#define XTALOSC24M_LOWPWR_CTRL_SET_REFTOP_IBIAS_OFF(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_REFTOP_IBIAS_OFF_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_REFTOP_IBIAS_OFF_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_L1_PWRGATE_MASK (0x100U)
#define XTALOSC24M_LOWPWR_CTRL_SET_L1_PWRGATE_SHIFT (8U)
#define XTALOSC24M_LOWPWR_CTRL_SET_L1_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_L1_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_L1_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_L2_PWRGATE_MASK (0x200U)
#define XTALOSC24M_LOWPWR_CTRL_SET_L2_PWRGATE_SHIFT (9U)
#define XTALOSC24M_LOWPWR_CTRL_SET_L2_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_L2_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_L2_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_CPU_PWRGATE_MASK (0x400U)
#define XTALOSC24M_LOWPWR_CTRL_SET_CPU_PWRGATE_SHIFT (10U)
#define XTALOSC24M_LOWPWR_CTRL_SET_CPU_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_CPU_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_CPU_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_DISPLAY_PWRGATE_MASK (0x800U)
#define XTALOSC24M_LOWPWR_CTRL_SET_DISPLAY_PWRGATE_SHIFT (11U)
#define XTALOSC24M_LOWPWR_CTRL_SET_DISPLAY_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_DISPLAY_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_DISPLAY_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_RCOSC_CG_OVERRIDE_MASK (0x2000U)
#define XTALOSC24M_LOWPWR_CTRL_SET_RCOSC_CG_OVERRIDE_SHIFT (13U)
#define XTALOSC24M_LOWPWR_CTRL_SET_RCOSC_CG_OVERRIDE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_RCOSC_CG_OVERRIDE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_RCOSC_CG_OVERRIDE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_DELAY_MASK (0xC000U)
#define XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_DELAY_SHIFT (14U)
/*! XTALOSC_PWRUP_DELAY
 *  0b00..0.25ms
 *  0b01..0.5ms
 *  0b10..1ms
 *  0b11..2ms
 */
#define XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_DELAY(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_DELAY_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_DELAY_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_STAT_MASK (0x10000U)
#define XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_STAT_SHIFT (16U)
/*! XTALOSC_PWRUP_STAT
 *  0b0..Not stable
 *  0b1..Stable and ready to use
 */
#define XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_STAT(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_STAT_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_XTALOSC_PWRUP_STAT_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_MIX_PWRGATE_MASK (0x20000U)
#define XTALOSC24M_LOWPWR_CTRL_SET_MIX_PWRGATE_SHIFT (17U)
#define XTALOSC24M_LOWPWR_CTRL_SET_MIX_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_MIX_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_MIX_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_SET_GPU_PWRGATE_MASK (0x40000U)
#define XTALOSC24M_LOWPWR_CTRL_SET_GPU_PWRGATE_SHIFT (18U)
#define XTALOSC24M_LOWPWR_CTRL_SET_GPU_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_SET_GPU_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_SET_GPU_PWRGATE_MASK)
/*! @} */

/*! @name LOWPWR_CTRL_CLR - XTAL OSC (LP) Control Register */
/*! @{ */
#define XTALOSC24M_LOWPWR_CTRL_CLR_RC_OSC_EN_MASK (0x1U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_RC_OSC_EN_SHIFT (0U)
/*! RC_OSC_EN
 *  0b0..Use XTAL OSC to source the 24MHz clock
 *  0b1..Use RC OSC
 */
#define XTALOSC24M_LOWPWR_CTRL_CLR_RC_OSC_EN(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_RC_OSC_EN_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_RC_OSC_EN_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_OSC_SEL_MASK  (0x10U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_OSC_SEL_SHIFT (4U)
/*! OSC_SEL
 *  0b0..XTAL OSC
 *  0b1..RC OSC
 */
#define XTALOSC24M_LOWPWR_CTRL_CLR_OSC_SEL(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_OSC_SEL_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_OSC_SEL_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_SEL_MASK (0x20U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_SEL_SHIFT (5U)
/*! LPBG_SEL
 *  0b0..Normal power bandgap
 *  0b1..Low power bandgap
 */
#define XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_SEL(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_SEL_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_SEL_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_TEST_MASK (0x40U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_TEST_SHIFT (6U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_TEST(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_TEST_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_LPBG_TEST_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_REFTOP_IBIAS_OFF_MASK (0x80U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_REFTOP_IBIAS_OFF_SHIFT (7U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_REFTOP_IBIAS_OFF(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_REFTOP_IBIAS_OFF_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_REFTOP_IBIAS_OFF_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_L1_PWRGATE_MASK (0x100U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_L1_PWRGATE_SHIFT (8U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_L1_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_L1_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_L1_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_L2_PWRGATE_MASK (0x200U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_L2_PWRGATE_SHIFT (9U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_L2_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_L2_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_L2_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_CPU_PWRGATE_MASK (0x400U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_CPU_PWRGATE_SHIFT (10U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_CPU_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_CPU_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_CPU_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_DISPLAY_PWRGATE_MASK (0x800U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_DISPLAY_PWRGATE_SHIFT (11U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_DISPLAY_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_DISPLAY_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_DISPLAY_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_RCOSC_CG_OVERRIDE_MASK (0x2000U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_RCOSC_CG_OVERRIDE_SHIFT (13U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_RCOSC_CG_OVERRIDE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_RCOSC_CG_OVERRIDE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_RCOSC_CG_OVERRIDE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_DELAY_MASK (0xC000U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_DELAY_SHIFT (14U)
/*! XTALOSC_PWRUP_DELAY
 *  0b00..0.25ms
 *  0b01..0.5ms
 *  0b10..1ms
 *  0b11..2ms
 */
#define XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_DELAY(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_DELAY_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_DELAY_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_STAT_MASK (0x10000U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_STAT_SHIFT (16U)
/*! XTALOSC_PWRUP_STAT
 *  0b0..Not stable
 *  0b1..Stable and ready to use
 */
#define XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_STAT(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_STAT_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_XTALOSC_PWRUP_STAT_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_MIX_PWRGATE_MASK (0x20000U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_MIX_PWRGATE_SHIFT (17U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_MIX_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_MIX_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_MIX_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_CLR_GPU_PWRGATE_MASK (0x40000U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_GPU_PWRGATE_SHIFT (18U)
#define XTALOSC24M_LOWPWR_CTRL_CLR_GPU_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_CLR_GPU_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_CLR_GPU_PWRGATE_MASK)
/*! @} */

/*! @name LOWPWR_CTRL_TOG - XTAL OSC (LP) Control Register */
/*! @{ */
#define XTALOSC24M_LOWPWR_CTRL_TOG_RC_OSC_EN_MASK (0x1U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_RC_OSC_EN_SHIFT (0U)
/*! RC_OSC_EN
 *  0b0..Use XTAL OSC to source the 24MHz clock
 *  0b1..Use RC OSC
 */
#define XTALOSC24M_LOWPWR_CTRL_TOG_RC_OSC_EN(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_RC_OSC_EN_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_RC_OSC_EN_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_OSC_SEL_MASK  (0x10U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_OSC_SEL_SHIFT (4U)
/*! OSC_SEL
 *  0b0..XTAL OSC
 *  0b1..RC OSC
 */
#define XTALOSC24M_LOWPWR_CTRL_TOG_OSC_SEL(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_OSC_SEL_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_OSC_SEL_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_SEL_MASK (0x20U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_SEL_SHIFT (5U)
/*! LPBG_SEL
 *  0b0..Normal power bandgap
 *  0b1..Low power bandgap
 */
#define XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_SEL(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_SEL_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_SEL_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_TEST_MASK (0x40U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_TEST_SHIFT (6U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_TEST(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_TEST_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_LPBG_TEST_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_REFTOP_IBIAS_OFF_MASK (0x80U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_REFTOP_IBIAS_OFF_SHIFT (7U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_REFTOP_IBIAS_OFF(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_REFTOP_IBIAS_OFF_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_REFTOP_IBIAS_OFF_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_L1_PWRGATE_MASK (0x100U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_L1_PWRGATE_SHIFT (8U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_L1_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_L1_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_L1_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_L2_PWRGATE_MASK (0x200U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_L2_PWRGATE_SHIFT (9U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_L2_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_L2_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_L2_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_CPU_PWRGATE_MASK (0x400U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_CPU_PWRGATE_SHIFT (10U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_CPU_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_CPU_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_CPU_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_DISPLAY_PWRGATE_MASK (0x800U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_DISPLAY_PWRGATE_SHIFT (11U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_DISPLAY_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_DISPLAY_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_DISPLAY_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_RCOSC_CG_OVERRIDE_MASK (0x2000U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_RCOSC_CG_OVERRIDE_SHIFT (13U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_RCOSC_CG_OVERRIDE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_RCOSC_CG_OVERRIDE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_RCOSC_CG_OVERRIDE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_DELAY_MASK (0xC000U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_DELAY_SHIFT (14U)
/*! XTALOSC_PWRUP_DELAY
 *  0b00..0.25ms
 *  0b01..0.5ms
 *  0b10..1ms
 *  0b11..2ms
 */
#define XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_DELAY(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_DELAY_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_DELAY_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_STAT_MASK (0x10000U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_STAT_SHIFT (16U)
/*! XTALOSC_PWRUP_STAT
 *  0b0..Not stable
 *  0b1..Stable and ready to use
 */
#define XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_STAT(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_STAT_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_XTALOSC_PWRUP_STAT_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_MIX_PWRGATE_MASK (0x20000U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_MIX_PWRGATE_SHIFT (17U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_MIX_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_MIX_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_MIX_PWRGATE_MASK)
#define XTALOSC24M_LOWPWR_CTRL_TOG_GPU_PWRGATE_MASK (0x40000U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_GPU_PWRGATE_SHIFT (18U)
#define XTALOSC24M_LOWPWR_CTRL_TOG_GPU_PWRGATE(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_LOWPWR_CTRL_TOG_GPU_PWRGATE_SHIFT)) & XTALOSC24M_LOWPWR_CTRL_TOG_GPU_PWRGATE_MASK)
/*! @} */

/*! @name OSC_CONFIG0 - XTAL OSC Configuration 0 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG0_START_MASK        (0x1U)
#define XTALOSC24M_OSC_CONFIG0_START_SHIFT       (0U)
#define XTALOSC24M_OSC_CONFIG0_START(x)          (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_START_SHIFT)) & XTALOSC24M_OSC_CONFIG0_START_MASK)
#define XTALOSC24M_OSC_CONFIG0_ENABLE_MASK       (0x2U)
#define XTALOSC24M_OSC_CONFIG0_ENABLE_SHIFT      (1U)
#define XTALOSC24M_OSC_CONFIG0_ENABLE(x)         (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_ENABLE_SHIFT)) & XTALOSC24M_OSC_CONFIG0_ENABLE_MASK)
#define XTALOSC24M_OSC_CONFIG0_BYPASS_MASK       (0x4U)
#define XTALOSC24M_OSC_CONFIG0_BYPASS_SHIFT      (2U)
#define XTALOSC24M_OSC_CONFIG0_BYPASS(x)         (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_BYPASS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_BYPASS_MASK)
#define XTALOSC24M_OSC_CONFIG0_INVERT_MASK       (0x8U)
#define XTALOSC24M_OSC_CONFIG0_INVERT_SHIFT      (3U)
#define XTALOSC24M_OSC_CONFIG0_INVERT(x)         (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_INVERT_SHIFT)) & XTALOSC24M_OSC_CONFIG0_INVERT_MASK)
#define XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_MASK  (0xFF0U)
#define XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_SHIFT (4U)
#define XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG(x)    (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_SHIFT)) & XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_MASK)
#define XTALOSC24M_OSC_CONFIG0_HYST_PLUS_MASK    (0xF000U)
#define XTALOSC24M_OSC_CONFIG0_HYST_PLUS_SHIFT   (12U)
#define XTALOSC24M_OSC_CONFIG0_HYST_PLUS(x)      (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_HYST_PLUS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_HYST_PLUS_MASK)
#define XTALOSC24M_OSC_CONFIG0_HYST_MINUS_MASK   (0xF0000U)
#define XTALOSC24M_OSC_CONFIG0_HYST_MINUS_SHIFT  (16U)
#define XTALOSC24M_OSC_CONFIG0_HYST_MINUS(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_HYST_MINUS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_HYST_MINUS_MASK)
#define XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_CUR_MASK (0xFF000000U)
#define XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_CUR_SHIFT (24U)
#define XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_CUR(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_CUR_SHIFT)) & XTALOSC24M_OSC_CONFIG0_RC_OSC_PROG_CUR_MASK)
/*! @} */

/*! @name OSC_CONFIG0_SET - XTAL OSC Configuration 0 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG0_SET_START_MASK    (0x1U)
#define XTALOSC24M_OSC_CONFIG0_SET_START_SHIFT   (0U)
#define XTALOSC24M_OSC_CONFIG0_SET_START(x)      (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_SET_START_SHIFT)) & XTALOSC24M_OSC_CONFIG0_SET_START_MASK)
#define XTALOSC24M_OSC_CONFIG0_SET_ENABLE_MASK   (0x2U)
#define XTALOSC24M_OSC_CONFIG0_SET_ENABLE_SHIFT  (1U)
#define XTALOSC24M_OSC_CONFIG0_SET_ENABLE(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_SET_ENABLE_SHIFT)) & XTALOSC24M_OSC_CONFIG0_SET_ENABLE_MASK)
#define XTALOSC24M_OSC_CONFIG0_SET_BYPASS_MASK   (0x4U)
#define XTALOSC24M_OSC_CONFIG0_SET_BYPASS_SHIFT  (2U)
#define XTALOSC24M_OSC_CONFIG0_SET_BYPASS(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_SET_BYPASS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_SET_BYPASS_MASK)
#define XTALOSC24M_OSC_CONFIG0_SET_INVERT_MASK   (0x8U)
#define XTALOSC24M_OSC_CONFIG0_SET_INVERT_SHIFT  (3U)
#define XTALOSC24M_OSC_CONFIG0_SET_INVERT(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_SET_INVERT_SHIFT)) & XTALOSC24M_OSC_CONFIG0_SET_INVERT_MASK)
#define XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_MASK (0xFF0U)
#define XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_SHIFT (4U)
#define XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_SHIFT)) & XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_MASK)
#define XTALOSC24M_OSC_CONFIG0_SET_HYST_PLUS_MASK (0xF000U)
#define XTALOSC24M_OSC_CONFIG0_SET_HYST_PLUS_SHIFT (12U)
#define XTALOSC24M_OSC_CONFIG0_SET_HYST_PLUS(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_SET_HYST_PLUS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_SET_HYST_PLUS_MASK)
#define XTALOSC24M_OSC_CONFIG0_SET_HYST_MINUS_MASK (0xF0000U)
#define XTALOSC24M_OSC_CONFIG0_SET_HYST_MINUS_SHIFT (16U)
#define XTALOSC24M_OSC_CONFIG0_SET_HYST_MINUS(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_SET_HYST_MINUS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_SET_HYST_MINUS_MASK)
#define XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_CUR_MASK (0xFF000000U)
#define XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_CUR_SHIFT (24U)
#define XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_CUR(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_CUR_SHIFT)) & XTALOSC24M_OSC_CONFIG0_SET_RC_OSC_PROG_CUR_MASK)
/*! @} */

/*! @name OSC_CONFIG0_CLR - XTAL OSC Configuration 0 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG0_CLR_START_MASK    (0x1U)
#define XTALOSC24M_OSC_CONFIG0_CLR_START_SHIFT   (0U)
#define XTALOSC24M_OSC_CONFIG0_CLR_START(x)      (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_CLR_START_SHIFT)) & XTALOSC24M_OSC_CONFIG0_CLR_START_MASK)
#define XTALOSC24M_OSC_CONFIG0_CLR_ENABLE_MASK   (0x2U)
#define XTALOSC24M_OSC_CONFIG0_CLR_ENABLE_SHIFT  (1U)
#define XTALOSC24M_OSC_CONFIG0_CLR_ENABLE(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_CLR_ENABLE_SHIFT)) & XTALOSC24M_OSC_CONFIG0_CLR_ENABLE_MASK)
#define XTALOSC24M_OSC_CONFIG0_CLR_BYPASS_MASK   (0x4U)
#define XTALOSC24M_OSC_CONFIG0_CLR_BYPASS_SHIFT  (2U)
#define XTALOSC24M_OSC_CONFIG0_CLR_BYPASS(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_CLR_BYPASS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_CLR_BYPASS_MASK)
#define XTALOSC24M_OSC_CONFIG0_CLR_INVERT_MASK   (0x8U)
#define XTALOSC24M_OSC_CONFIG0_CLR_INVERT_SHIFT  (3U)
#define XTALOSC24M_OSC_CONFIG0_CLR_INVERT(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_CLR_INVERT_SHIFT)) & XTALOSC24M_OSC_CONFIG0_CLR_INVERT_MASK)
#define XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_MASK (0xFF0U)
#define XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_SHIFT (4U)
#define XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_SHIFT)) & XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_MASK)
#define XTALOSC24M_OSC_CONFIG0_CLR_HYST_PLUS_MASK (0xF000U)
#define XTALOSC24M_OSC_CONFIG0_CLR_HYST_PLUS_SHIFT (12U)
#define XTALOSC24M_OSC_CONFIG0_CLR_HYST_PLUS(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_CLR_HYST_PLUS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_CLR_HYST_PLUS_MASK)
#define XTALOSC24M_OSC_CONFIG0_CLR_HYST_MINUS_MASK (0xF0000U)
#define XTALOSC24M_OSC_CONFIG0_CLR_HYST_MINUS_SHIFT (16U)
#define XTALOSC24M_OSC_CONFIG0_CLR_HYST_MINUS(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_CLR_HYST_MINUS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_CLR_HYST_MINUS_MASK)
#define XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_CUR_MASK (0xFF000000U)
#define XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_CUR_SHIFT (24U)
#define XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_CUR(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_CUR_SHIFT)) & XTALOSC24M_OSC_CONFIG0_CLR_RC_OSC_PROG_CUR_MASK)
/*! @} */

/*! @name OSC_CONFIG0_TOG - XTAL OSC Configuration 0 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG0_TOG_START_MASK    (0x1U)
#define XTALOSC24M_OSC_CONFIG0_TOG_START_SHIFT   (0U)
#define XTALOSC24M_OSC_CONFIG0_TOG_START(x)      (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_TOG_START_SHIFT)) & XTALOSC24M_OSC_CONFIG0_TOG_START_MASK)
#define XTALOSC24M_OSC_CONFIG0_TOG_ENABLE_MASK   (0x2U)
#define XTALOSC24M_OSC_CONFIG0_TOG_ENABLE_SHIFT  (1U)
#define XTALOSC24M_OSC_CONFIG0_TOG_ENABLE(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_TOG_ENABLE_SHIFT)) & XTALOSC24M_OSC_CONFIG0_TOG_ENABLE_MASK)
#define XTALOSC24M_OSC_CONFIG0_TOG_BYPASS_MASK   (0x4U)
#define XTALOSC24M_OSC_CONFIG0_TOG_BYPASS_SHIFT  (2U)
#define XTALOSC24M_OSC_CONFIG0_TOG_BYPASS(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_TOG_BYPASS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_TOG_BYPASS_MASK)
#define XTALOSC24M_OSC_CONFIG0_TOG_INVERT_MASK   (0x8U)
#define XTALOSC24M_OSC_CONFIG0_TOG_INVERT_SHIFT  (3U)
#define XTALOSC24M_OSC_CONFIG0_TOG_INVERT(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_TOG_INVERT_SHIFT)) & XTALOSC24M_OSC_CONFIG0_TOG_INVERT_MASK)
#define XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_MASK (0xFF0U)
#define XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_SHIFT (4U)
#define XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_SHIFT)) & XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_MASK)
#define XTALOSC24M_OSC_CONFIG0_TOG_HYST_PLUS_MASK (0xF000U)
#define XTALOSC24M_OSC_CONFIG0_TOG_HYST_PLUS_SHIFT (12U)
#define XTALOSC24M_OSC_CONFIG0_TOG_HYST_PLUS(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_TOG_HYST_PLUS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_TOG_HYST_PLUS_MASK)
#define XTALOSC24M_OSC_CONFIG0_TOG_HYST_MINUS_MASK (0xF0000U)
#define XTALOSC24M_OSC_CONFIG0_TOG_HYST_MINUS_SHIFT (16U)
#define XTALOSC24M_OSC_CONFIG0_TOG_HYST_MINUS(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_TOG_HYST_MINUS_SHIFT)) & XTALOSC24M_OSC_CONFIG0_TOG_HYST_MINUS_MASK)
#define XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_CUR_MASK (0xFF000000U)
#define XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_CUR_SHIFT (24U)
#define XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_CUR(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_CUR_SHIFT)) & XTALOSC24M_OSC_CONFIG0_TOG_RC_OSC_PROG_CUR_MASK)
/*! @} */

/*! @name OSC_CONFIG1 - XTAL OSC Configuration 1 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG1_COUNT_RC_TRG_MASK (0xFFFU)
#define XTALOSC24M_OSC_CONFIG1_COUNT_RC_TRG_SHIFT (0U)
#define XTALOSC24M_OSC_CONFIG1_COUNT_RC_TRG(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG1_COUNT_RC_TRG_SHIFT)) & XTALOSC24M_OSC_CONFIG1_COUNT_RC_TRG_MASK)
#define XTALOSC24M_OSC_CONFIG1_COUNT_RC_CUR_MASK (0xFFF00000U)
#define XTALOSC24M_OSC_CONFIG1_COUNT_RC_CUR_SHIFT (20U)
#define XTALOSC24M_OSC_CONFIG1_COUNT_RC_CUR(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG1_COUNT_RC_CUR_SHIFT)) & XTALOSC24M_OSC_CONFIG1_COUNT_RC_CUR_MASK)
/*! @} */

/*! @name OSC_CONFIG1_SET - XTAL OSC Configuration 1 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_TRG_MASK (0xFFFU)
#define XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_TRG_SHIFT (0U)
#define XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_TRG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_TRG_SHIFT)) & XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_TRG_MASK)
#define XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_CUR_MASK (0xFFF00000U)
#define XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_CUR_SHIFT (20U)
#define XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_CUR(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_CUR_SHIFT)) & XTALOSC24M_OSC_CONFIG1_SET_COUNT_RC_CUR_MASK)
/*! @} */

/*! @name OSC_CONFIG1_CLR - XTAL OSC Configuration 1 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_TRG_MASK (0xFFFU)
#define XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_TRG_SHIFT (0U)
#define XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_TRG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_TRG_SHIFT)) & XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_TRG_MASK)
#define XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_CUR_MASK (0xFFF00000U)
#define XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_CUR_SHIFT (20U)
#define XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_CUR(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_CUR_SHIFT)) & XTALOSC24M_OSC_CONFIG1_CLR_COUNT_RC_CUR_MASK)
/*! @} */

/*! @name OSC_CONFIG1_TOG - XTAL OSC Configuration 1 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_TRG_MASK (0xFFFU)
#define XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_TRG_SHIFT (0U)
#define XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_TRG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_TRG_SHIFT)) & XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_TRG_MASK)
#define XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_CUR_MASK (0xFFF00000U)
#define XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_CUR_SHIFT (20U)
#define XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_CUR(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_CUR_SHIFT)) & XTALOSC24M_OSC_CONFIG1_TOG_COUNT_RC_CUR_MASK)
/*! @} */

/*! @name OSC_CONFIG2 - XTAL OSC Configuration 2 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_MASK (0xFFFU)
#define XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_SHIFT (0U)
#define XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG(x)   (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_SHIFT)) & XTALOSC24M_OSC_CONFIG2_COUNT_1M_TRG_MASK)
#define XTALOSC24M_OSC_CONFIG2_ENABLE_1M_MASK    (0x10000U)
#define XTALOSC24M_OSC_CONFIG2_ENABLE_1M_SHIFT   (16U)
#define XTALOSC24M_OSC_CONFIG2_ENABLE_1M(x)      (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_ENABLE_1M_SHIFT)) & XTALOSC24M_OSC_CONFIG2_ENABLE_1M_MASK)
#define XTALOSC24M_OSC_CONFIG2_MUX_1M_MASK       (0x20000U)
#define XTALOSC24M_OSC_CONFIG2_MUX_1M_SHIFT      (17U)
#define XTALOSC24M_OSC_CONFIG2_MUX_1M(x)         (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_MUX_1M_SHIFT)) & XTALOSC24M_OSC_CONFIG2_MUX_1M_MASK)
#define XTALOSC24M_OSC_CONFIG2_CLK_1M_ERR_FL_MASK (0x80000000U)
#define XTALOSC24M_OSC_CONFIG2_CLK_1M_ERR_FL_SHIFT (31U)
#define XTALOSC24M_OSC_CONFIG2_CLK_1M_ERR_FL(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_CLK_1M_ERR_FL_SHIFT)) & XTALOSC24M_OSC_CONFIG2_CLK_1M_ERR_FL_MASK)
/*! @} */

/*! @name OSC_CONFIG2_SET - XTAL OSC Configuration 2 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG2_SET_COUNT_1M_TRG_MASK (0xFFFU)
#define XTALOSC24M_OSC_CONFIG2_SET_COUNT_1M_TRG_SHIFT (0U)
#define XTALOSC24M_OSC_CONFIG2_SET_COUNT_1M_TRG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_SET_COUNT_1M_TRG_SHIFT)) & XTALOSC24M_OSC_CONFIG2_SET_COUNT_1M_TRG_MASK)
#define XTALOSC24M_OSC_CONFIG2_SET_ENABLE_1M_MASK (0x10000U)
#define XTALOSC24M_OSC_CONFIG2_SET_ENABLE_1M_SHIFT (16U)
#define XTALOSC24M_OSC_CONFIG2_SET_ENABLE_1M(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_SET_ENABLE_1M_SHIFT)) & XTALOSC24M_OSC_CONFIG2_SET_ENABLE_1M_MASK)
#define XTALOSC24M_OSC_CONFIG2_SET_MUX_1M_MASK   (0x20000U)
#define XTALOSC24M_OSC_CONFIG2_SET_MUX_1M_SHIFT  (17U)
#define XTALOSC24M_OSC_CONFIG2_SET_MUX_1M(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_SET_MUX_1M_SHIFT)) & XTALOSC24M_OSC_CONFIG2_SET_MUX_1M_MASK)
#define XTALOSC24M_OSC_CONFIG2_SET_CLK_1M_ERR_FL_MASK (0x80000000U)
#define XTALOSC24M_OSC_CONFIG2_SET_CLK_1M_ERR_FL_SHIFT (31U)
#define XTALOSC24M_OSC_CONFIG2_SET_CLK_1M_ERR_FL(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_SET_CLK_1M_ERR_FL_SHIFT)) & XTALOSC24M_OSC_CONFIG2_SET_CLK_1M_ERR_FL_MASK)
/*! @} */

/*! @name OSC_CONFIG2_CLR - XTAL OSC Configuration 2 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG2_CLR_COUNT_1M_TRG_MASK (0xFFFU)
#define XTALOSC24M_OSC_CONFIG2_CLR_COUNT_1M_TRG_SHIFT (0U)
#define XTALOSC24M_OSC_CONFIG2_CLR_COUNT_1M_TRG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_CLR_COUNT_1M_TRG_SHIFT)) & XTALOSC24M_OSC_CONFIG2_CLR_COUNT_1M_TRG_MASK)
#define XTALOSC24M_OSC_CONFIG2_CLR_ENABLE_1M_MASK (0x10000U)
#define XTALOSC24M_OSC_CONFIG2_CLR_ENABLE_1M_SHIFT (16U)
#define XTALOSC24M_OSC_CONFIG2_CLR_ENABLE_1M(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_CLR_ENABLE_1M_SHIFT)) & XTALOSC24M_OSC_CONFIG2_CLR_ENABLE_1M_MASK)
#define XTALOSC24M_OSC_CONFIG2_CLR_MUX_1M_MASK   (0x20000U)
#define XTALOSC24M_OSC_CONFIG2_CLR_MUX_1M_SHIFT  (17U)
#define XTALOSC24M_OSC_CONFIG2_CLR_MUX_1M(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_CLR_MUX_1M_SHIFT)) & XTALOSC24M_OSC_CONFIG2_CLR_MUX_1M_MASK)
#define XTALOSC24M_OSC_CONFIG2_CLR_CLK_1M_ERR_FL_MASK (0x80000000U)
#define XTALOSC24M_OSC_CONFIG2_CLR_CLK_1M_ERR_FL_SHIFT (31U)
#define XTALOSC24M_OSC_CONFIG2_CLR_CLK_1M_ERR_FL(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_CLR_CLK_1M_ERR_FL_SHIFT)) & XTALOSC24M_OSC_CONFIG2_CLR_CLK_1M_ERR_FL_MASK)
/*! @} */

/*! @name OSC_CONFIG2_TOG - XTAL OSC Configuration 2 Register */
/*! @{ */
#define XTALOSC24M_OSC_CONFIG2_TOG_COUNT_1M_TRG_MASK (0xFFFU)
#define XTALOSC24M_OSC_CONFIG2_TOG_COUNT_1M_TRG_SHIFT (0U)
#define XTALOSC24M_OSC_CONFIG2_TOG_COUNT_1M_TRG(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_TOG_COUNT_1M_TRG_SHIFT)) & XTALOSC24M_OSC_CONFIG2_TOG_COUNT_1M_TRG_MASK)
#define XTALOSC24M_OSC_CONFIG2_TOG_ENABLE_1M_MASK (0x10000U)
#define XTALOSC24M_OSC_CONFIG2_TOG_ENABLE_1M_SHIFT (16U)
#define XTALOSC24M_OSC_CONFIG2_TOG_ENABLE_1M(x)  (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_TOG_ENABLE_1M_SHIFT)) & XTALOSC24M_OSC_CONFIG2_TOG_ENABLE_1M_MASK)
#define XTALOSC24M_OSC_CONFIG2_TOG_MUX_1M_MASK   (0x20000U)
#define XTALOSC24M_OSC_CONFIG2_TOG_MUX_1M_SHIFT  (17U)
#define XTALOSC24M_OSC_CONFIG2_TOG_MUX_1M(x)     (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_TOG_MUX_1M_SHIFT)) & XTALOSC24M_OSC_CONFIG2_TOG_MUX_1M_MASK)
#define XTALOSC24M_OSC_CONFIG2_TOG_CLK_1M_ERR_FL_MASK (0x80000000U)
#define XTALOSC24M_OSC_CONFIG2_TOG_CLK_1M_ERR_FL_SHIFT (31U)
#define XTALOSC24M_OSC_CONFIG2_TOG_CLK_1M_ERR_FL(x) (((uint32_t)(((uint32_t)(x)) << XTALOSC24M_OSC_CONFIG2_TOG_CLK_1M_ERR_FL_SHIFT)) & XTALOSC24M_OSC_CONFIG2_TOG_CLK_1M_ERR_FL_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group XTALOSC24M_Register_Masks */


/* XTALOSC24M - Peripheral instance base addresses */
/** Peripheral XTALOSC24M base address */
#define XTALOSC24M_BASE                          (0x400D8000u)
/** Peripheral XTALOSC24M base pointer */
#define XTALOSC24M                               ((XTALOSC24M_Type *)XTALOSC24M_BASE)
/** Array initializer of XTALOSC24M peripheral base addresses */
#define XTALOSC24M_BASE_ADDRS                    { XTALOSC24M_BASE }
/** Array initializer of XTALOSC24M peripheral base pointers */
#define XTALOSC24M_BASE_PTRS                     { XTALOSC24M }

/*!
 * @}
 */ /* end of group XTALOSC24M_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SRC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SRC_Peripheral_Access_Layer SRC Peripheral Access Layer
 * @{
 */

/** SRC - Register Layout Typedef */
typedef struct {
  __IO uint32_t SCR;                               /**< SRC Control Register, offset: 0x0 */
  __I  uint32_t SBMR1;                             /**< SRC Boot Mode Register 1, offset: 0x4 */
  __IO uint32_t SRSR;                              /**< SRC Reset Status Register, offset: 0x8 */
       uint8_t RESERVED_0[16];
  __I  uint32_t SBMR2;                             /**< SRC Boot Mode Register 2, offset: 0x1C */
  __IO uint32_t GPR[10];                           /**< SRC General Purpose Register 1..SRC General Purpose Register 10, array offset: 0x20, array step: 0x4 */
} SRC_Type;

/* ----------------------------------------------------------------------------
   -- SRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SRC_Register_Masks SRC Register Masks
 * @{
 */

/*! @name SCR - SRC Control Register */
/*! @{ */
#define SRC_SCR_MASK_WDOG_RST_MASK               (0x780U)
#define SRC_SCR_MASK_WDOG_RST_SHIFT              (7U)
/*! mask_wdog_rst
 *  0b0101..wdog_rst_b is masked
 *  0b1010..wdog_rst_b is not masked (default)
 */
#define SRC_SCR_MASK_WDOG_RST(x)                 (((uint32_t)(((uint32_t)(x)) << SRC_SCR_MASK_WDOG_RST_SHIFT)) & SRC_SCR_MASK_WDOG_RST_MASK)
#define SRC_SCR_CORE0_RST_MASK                   (0x2000U)
#define SRC_SCR_CORE0_RST_SHIFT                  (13U)
/*! core0_rst
 *  0b0..do not assert core0 reset
 *  0b1..assert core0 reset
 */
#define SRC_SCR_CORE0_RST(x)                     (((uint32_t)(((uint32_t)(x)) << SRC_SCR_CORE0_RST_SHIFT)) & SRC_SCR_CORE0_RST_MASK)
#define SRC_SCR_CORE0_DBG_RST_MASK               (0x20000U)
#define SRC_SCR_CORE0_DBG_RST_SHIFT              (17U)
/*! core0_dbg_rst
 *  0b0..do not assert core0 debug reset
 *  0b1..assert core0 debug reset
 */
#define SRC_SCR_CORE0_DBG_RST(x)                 (((uint32_t)(((uint32_t)(x)) << SRC_SCR_CORE0_DBG_RST_SHIFT)) & SRC_SCR_CORE0_DBG_RST_MASK)
#define SRC_SCR_DBG_RST_MSK_PG_MASK              (0x2000000U)
#define SRC_SCR_DBG_RST_MSK_PG_SHIFT             (25U)
/*! dbg_rst_msk_pg
 *  0b0..do not mask core debug resets (debug resets will be asserted after power gating event)
 *  0b1..mask core debug resets (debug resets won't be asserted after power gating event)
 */
#define SRC_SCR_DBG_RST_MSK_PG(x)                (((uint32_t)(((uint32_t)(x)) << SRC_SCR_DBG_RST_MSK_PG_SHIFT)) & SRC_SCR_DBG_RST_MSK_PG_MASK)
#define SRC_SCR_MASK_WDOG3_RST_MASK              (0xF0000000U)
#define SRC_SCR_MASK_WDOG3_RST_SHIFT             (28U)
/*! mask_wdog3_rst
 *  0b0101..wdog3_rst_b is masked
 *  0b1010..wdog3_rst_b is not masked
 */
#define SRC_SCR_MASK_WDOG3_RST(x)                (((uint32_t)(((uint32_t)(x)) << SRC_SCR_MASK_WDOG3_RST_SHIFT)) & SRC_SCR_MASK_WDOG3_RST_MASK)
/*! @} */

/*! @name SBMR1 - SRC Boot Mode Register 1 */
/*! @{ */
#define SRC_SBMR1_BOOT_CFG1_MASK                 (0xFFU)
#define SRC_SBMR1_BOOT_CFG1_SHIFT                (0U)
#define SRC_SBMR1_BOOT_CFG1(x)                   (((uint32_t)(((uint32_t)(x)) << SRC_SBMR1_BOOT_CFG1_SHIFT)) & SRC_SBMR1_BOOT_CFG1_MASK)
#define SRC_SBMR1_BOOT_CFG2_MASK                 (0xFF00U)
#define SRC_SBMR1_BOOT_CFG2_SHIFT                (8U)
#define SRC_SBMR1_BOOT_CFG2(x)                   (((uint32_t)(((uint32_t)(x)) << SRC_SBMR1_BOOT_CFG2_SHIFT)) & SRC_SBMR1_BOOT_CFG2_MASK)
#define SRC_SBMR1_BOOT_CFG3_MASK                 (0xFF0000U)
#define SRC_SBMR1_BOOT_CFG3_SHIFT                (16U)
#define SRC_SBMR1_BOOT_CFG3(x)                   (((uint32_t)(((uint32_t)(x)) << SRC_SBMR1_BOOT_CFG3_SHIFT)) & SRC_SBMR1_BOOT_CFG3_MASK)
#define SRC_SBMR1_BOOT_CFG4_MASK                 (0xFF000000U)
#define SRC_SBMR1_BOOT_CFG4_SHIFT                (24U)
#define SRC_SBMR1_BOOT_CFG4(x)                   (((uint32_t)(((uint32_t)(x)) << SRC_SBMR1_BOOT_CFG4_SHIFT)) & SRC_SBMR1_BOOT_CFG4_MASK)
/*! @} */

/*! @name SRSR - SRC Reset Status Register */
/*! @{ */
#define SRC_SRSR_IPP_RESET_B_MASK                (0x1U)
#define SRC_SRSR_IPP_RESET_B_SHIFT               (0U)
/*! ipp_reset_b
 *  0b0..Reset is not a result of ipp_reset_b pin.
 *  0b1..Reset is a result of ipp_reset_b pin.
 */
#define SRC_SRSR_IPP_RESET_B(x)                  (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_IPP_RESET_B_SHIFT)) & SRC_SRSR_IPP_RESET_B_MASK)
#define SRC_SRSR_LOCKUP_SYSRESETREQ_MASK         (0x2U)
#define SRC_SRSR_LOCKUP_SYSRESETREQ_SHIFT        (1U)
/*! lockup_sysresetreq
 *  0b0..Reset is not a result of the mentioned case.
 *  0b1..Reset is a result of the mentioned case.
 */
#define SRC_SRSR_LOCKUP_SYSRESETREQ(x)           (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_LOCKUP_SYSRESETREQ_SHIFT)) & SRC_SRSR_LOCKUP_SYSRESETREQ_MASK)
#define SRC_SRSR_CSU_RESET_B_MASK                (0x4U)
#define SRC_SRSR_CSU_RESET_B_SHIFT               (2U)
/*! csu_reset_b
 *  0b0..Reset is not a result of the csu_reset_b event.
 *  0b1..Reset is a result of the csu_reset_b event.
 */
#define SRC_SRSR_CSU_RESET_B(x)                  (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_CSU_RESET_B_SHIFT)) & SRC_SRSR_CSU_RESET_B_MASK)
#define SRC_SRSR_IPP_USER_RESET_B_MASK           (0x8U)
#define SRC_SRSR_IPP_USER_RESET_B_SHIFT          (3U)
/*! ipp_user_reset_b
 *  0b0..Reset is not a result of the ipp_user_reset_b qualified as COLD reset event.
 *  0b1..Reset is a result of the ipp_user_reset_b qualified as COLD reset event.
 */
#define SRC_SRSR_IPP_USER_RESET_B(x)             (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_IPP_USER_RESET_B_SHIFT)) & SRC_SRSR_IPP_USER_RESET_B_MASK)
#define SRC_SRSR_WDOG_RST_B_MASK                 (0x10U)
#define SRC_SRSR_WDOG_RST_B_SHIFT                (4U)
/*! wdog_rst_b
 *  0b0..Reset is not a result of the watchdog time-out event.
 *  0b1..Reset is a result of the watchdog time-out event.
 */
#define SRC_SRSR_WDOG_RST_B(x)                   (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_WDOG_RST_B_SHIFT)) & SRC_SRSR_WDOG_RST_B_MASK)
#define SRC_SRSR_JTAG_RST_B_MASK                 (0x20U)
#define SRC_SRSR_JTAG_RST_B_SHIFT                (5U)
/*! jtag_rst_b
 *  0b0..Reset is not a result of HIGH-Z reset from JTAG.
 *  0b1..Reset is a result of HIGH-Z reset from JTAG.
 */
#define SRC_SRSR_JTAG_RST_B(x)                   (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_JTAG_RST_B_SHIFT)) & SRC_SRSR_JTAG_RST_B_MASK)
#define SRC_SRSR_JTAG_SW_RST_MASK                (0x40U)
#define SRC_SRSR_JTAG_SW_RST_SHIFT               (6U)
/*! jtag_sw_rst
 *  0b0..Reset is not a result of software reset from JTAG.
 *  0b1..Reset is a result of software reset from JTAG.
 */
#define SRC_SRSR_JTAG_SW_RST(x)                  (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_JTAG_SW_RST_SHIFT)) & SRC_SRSR_JTAG_SW_RST_MASK)
#define SRC_SRSR_WDOG3_RST_B_MASK                (0x80U)
#define SRC_SRSR_WDOG3_RST_B_SHIFT               (7U)
/*! wdog3_rst_b
 *  0b0..Reset is not a result of the watchdog3 time-out event.
 *  0b1..Reset is a result of the watchdog3 time-out event.
 */
#define SRC_SRSR_WDOG3_RST_B(x)                  (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_WDOG3_RST_B_SHIFT)) & SRC_SRSR_WDOG3_RST_B_MASK)
#define SRC_SRSR_TEMPSENSE_RST_B_MASK            (0x100U)
#define SRC_SRSR_TEMPSENSE_RST_B_SHIFT           (8U)
/*! tempsense_rst_b
 *  0b0..Reset is not a result of software reset from Temperature Sensor.
 *  0b1..Reset is a result of software reset from Temperature Sensor.
 */
#define SRC_SRSR_TEMPSENSE_RST_B(x)              (((uint32_t)(((uint32_t)(x)) << SRC_SRSR_TEMPSENSE_RST_B_SHIFT)) & SRC_SRSR_TEMPSENSE_RST_B_MASK)
/*! @} */

/*! @name SBMR2 - SRC Boot Mode Register 2 */
/*! @{ */
#define SRC_SBMR2_SEC_CONFIG_MASK                (0x3U)
#define SRC_SBMR2_SEC_CONFIG_SHIFT               (0U)
#define SRC_SBMR2_SEC_CONFIG(x)                  (((uint32_t)(((uint32_t)(x)) << SRC_SBMR2_SEC_CONFIG_SHIFT)) & SRC_SBMR2_SEC_CONFIG_MASK)
#define SRC_SBMR2_DIR_BT_DIS_MASK                (0x8U)
#define SRC_SBMR2_DIR_BT_DIS_SHIFT               (3U)
#define SRC_SBMR2_DIR_BT_DIS(x)                  (((uint32_t)(((uint32_t)(x)) << SRC_SBMR2_DIR_BT_DIS_SHIFT)) & SRC_SBMR2_DIR_BT_DIS_MASK)
#define SRC_SBMR2_BT_FUSE_SEL_MASK               (0x10U)
#define SRC_SBMR2_BT_FUSE_SEL_SHIFT              (4U)
#define SRC_SBMR2_BT_FUSE_SEL(x)                 (((uint32_t)(((uint32_t)(x)) << SRC_SBMR2_BT_FUSE_SEL_SHIFT)) & SRC_SBMR2_BT_FUSE_SEL_MASK)
#define SRC_SBMR2_BMOD_MASK                      (0x3000000U)
#define SRC_SBMR2_BMOD_SHIFT                     (24U)
#define SRC_SBMR2_BMOD(x)                        (((uint32_t)(((uint32_t)(x)) << SRC_SBMR2_BMOD_SHIFT)) & SRC_SBMR2_BMOD_MASK)
/*! @} */

/*! @name GPR - SRC General Purpose Register 1..SRC General Purpose Register 10 */
/*! @{ */
#define SRC_GPR_PERSISTENT_ARG0_MASK             (0xFFFFFFFFU)
#define SRC_GPR_PERSISTENT_ARG0_SHIFT            (0U)
#define SRC_GPR_PERSISTENT_ARG0(x)               (((uint32_t)(((uint32_t)(x)) << SRC_GPR_PERSISTENT_ARG0_SHIFT)) & SRC_GPR_PERSISTENT_ARG0_MASK)
#define SRC_GPR_PERSISTENT_ENTRY0_MASK           (0xFFFFFFFFU)
#define SRC_GPR_PERSISTENT_ENTRY0_SHIFT          (0U)
#define SRC_GPR_PERSISTENT_ENTRY0(x)             (((uint32_t)(((uint32_t)(x)) << SRC_GPR_PERSISTENT_ENTRY0_SHIFT)) & SRC_GPR_PERSISTENT_ENTRY0_MASK)
/*! @} */

/* The count of SRC_GPR */
#define SRC_GPR_COUNT                            (10U)


/*!
 * @}
 */ /* end of group SRC_Register_Masks */


/* SRC - Peripheral instance base addresses */
/** Peripheral SRC base address */
#define SRC_BASE                                 (0x400F8000u)
/** Peripheral SRC base pointer */
#define SRC                                      ((SRC_Type *)SRC_BASE)
/** Array initializer of SRC peripheral base addresses */
#define SRC_BASE_ADDRS                           { SRC_BASE }
/** Array initializer of SRC peripheral base pointers */
#define SRC_BASE_PTRS                            { SRC }
/** Interrupt vectors for the SRC peripheral type */
#define SRC_IRQS                                 { SRC_IRQn }
/* Backward compatibility */
#define SRC_SCR_MWDR_MASK                      SRC_SCR_MASK_WDOG_RST_MASK
#define SRC_SCR_MWDR_SHIFT                     SRC_SCR_MASK_WDOG_RST_SHIFT
#define SRC_SCR_MWDR(x)                        SRC_SCR_MASK_WDOG_RST(x)
#define SRC_SRSR_WDOG_MASK                     SRC_SRSR_WDOG_RST_B_MASK
#define SRC_SRSR_WDOG_SHIFT                    SRC_SRSR_WDOG_RST_B_SHIFT
#define SRC_SRSR_WDOG(x)                       SRC_SRSR_WDOG_RST_B(x)
#define SRC_SRSR_JTAG_MASK                     SRC_SRSR_JTAG_RST_B_MASK
#define SRC_SRSR_JTAG_SHIFT                    SRC_SRSR_JTAG_RST_B_SHIFT
#define SRC_SRSR_JTAG(x)                       SRC_SRSR_JTAG_RST_B(x)
#define SRC_SRSR_SJC_MASK                      SRC_SRSR_JTAG_SW_RST_MASK
#define SRC_SRSR_SJC_SHIFT                     SRC_SRSR_JTAG_SW_RST_SHIFT
#define SRC_SRSR_SJC(x)                        SRC_SRSR_JTAG_SW_RST(x)
#define SRC_SRSR_TSR_MASK                      SRC_SRSR_TEMPSENSE_RST_B_MASK
#define SRC_SRSR_TSR_SHIFT                     SRC_SRSR_TEMPSENSE_RST_B_SHIFT
#define SRC_SRSR_TSR(x)                        SRC_SRSR_TEMPSENSE_RST_B(x)
/* Extra definition */
#define SRC_SRSR_W1C_BITS_MASK  ( SRC_SRSR_WDOG3_RST_B_MASK \
                                | SRC_SRSR_JTAG_SW_RST_MASK \
                                | SRC_SRSR_JTAG_RST_B_MASK \
                                | SRC_SRSR_WDOG_RST_B_MASK \
                                | SRC_SRSR_IPP_USER_RESET_B_MASK \
                                | SRC_SRSR_CSU_RESET_B_MASK \
                                | SRC_SRSR_LOCKUP_SYSRESETREQ_MASK \
                                | SRC_SRSR_IPP_RESET_B_MASK)


/*!
 * @}
 */ /* end of group SRC_Peripheral_Access_Layer */

/* ----------------------------------------------------------------------------
   -- IOMUXC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup IOMUXC_Peripheral_Access_Layer IOMUXC Peripheral Access Layer
 * @{
 */

/** IOMUXC - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[20];
  __IO uint32_t SW_MUX_CTL_PAD[124];               /**< SW_MUX_CTL_PAD_GPIO_EMC_00 SW MUX Control Register..SW_MUX_CTL_PAD_GPIO_SD_B1_11 SW MUX Control Register, array offset: 0x14, array step: 0x4 */
  __IO uint32_t SW_PAD_CTL_PAD[124];               /**< SW_PAD_CTL_PAD_GPIO_EMC_00 SW PAD Control Register..SW_PAD_CTL_PAD_GPIO_SD_B1_11 SW PAD Control Register, array offset: 0x204, array step: 0x4 */
  __IO uint32_t SELECT_INPUT[154];                 /**< ANATOP_USB_OTG1_ID_SELECT_INPUT DAISY Register..XBAR1_IN23_SELECT_INPUT DAISY Register, array offset: 0x3F4, array step: 0x4 */
} IOMUXC_Type;

/* ----------------------------------------------------------------------------
   -- IOMUXC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup IOMUXC_Register_Masks IOMUXC Register Masks
 * @{
 */

/*! @name SW_MUX_CTL_PAD - SW_MUX_CTL_PAD_GPIO_EMC_00 SW MUX Control Register..SW_MUX_CTL_PAD_GPIO_SD_B1_11 SW MUX Control Register */
/*! @{ */
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_MASK      (0x7U)
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_SHIFT     (0U)
/*! MUX_MODE - MUX Mode Select Field.
 *  0b000..Select mux mode: ALT0 mux port: SEMC_DATA00 of instance: semc
 *  0b001..Select mux mode: ALT1 mux port: FLEXPWM4_PWMA00 of instance: flexpwm4
 *  0b010..Select mux mode: ALT2 mux port: LPSPI2_SCK of instance: lpspi2
 *  0b011..Select mux mode: ALT3 mux port: XBAR1_XBAR_IN02 of instance: xbar1
 *  0b100..Select mux mode: ALT4 mux port: FLEXIO1_FLEXIO00 of instance: flexio1
 *  0b101..Select mux mode: ALT5 mux port: GPIO4_IO00 of instance: gpio4
 */
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(x)        (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_SHIFT)) & IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_MASK)
#define IOMUXC_SW_MUX_CTL_PAD_SION_MASK          (0x10U)
#define IOMUXC_SW_MUX_CTL_PAD_SION_SHIFT         (4U)
/*! SION - Software Input On Field.
 *  0b1..Force input path of pad GPIO_EMC_00
 *  0b0..Input Path is determined by functionality
 */
#define IOMUXC_SW_MUX_CTL_PAD_SION(x)            (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_MUX_CTL_PAD_SION_SHIFT)) & IOMUXC_SW_MUX_CTL_PAD_SION_MASK)
/*! @} */

/* The count of IOMUXC_SW_MUX_CTL_PAD */
#define IOMUXC_SW_MUX_CTL_PAD_COUNT              (124U)

/*! @name SW_PAD_CTL_PAD - SW_PAD_CTL_PAD_GPIO_EMC_00 SW PAD Control Register..SW_PAD_CTL_PAD_GPIO_SD_B1_11 SW PAD Control Register */
/*! @{ */
#define IOMUXC_SW_PAD_CTL_PAD_SRE_MASK           (0x1U)
#define IOMUXC_SW_PAD_CTL_PAD_SRE_SHIFT          (0U)
/*! SRE - Slew Rate Field
 *  0b0..Slow Slew Rate
 *  0b1..Fast Slew Rate
 */
#define IOMUXC_SW_PAD_CTL_PAD_SRE(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_SRE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_SRE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_DSE_MASK           (0x38U)
#define IOMUXC_SW_PAD_CTL_PAD_DSE_SHIFT          (3U)
/*! DSE - Drive Strength Field
 *  0b000..output driver disabled;
 *  0b001..R0(150 Ohm @ 3.3V, 260 Ohm@1.8V)
 *  0b010..R0/2
 *  0b011..R0/3
 *  0b100..R0/4
 *  0b101..R0/5
 *  0b110..R0/6
 *  0b111..R0/7
 */
#define IOMUXC_SW_PAD_CTL_PAD_DSE(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_DSE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_DSE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_SPEED_MASK         (0xC0U)
#define IOMUXC_SW_PAD_CTL_PAD_SPEED_SHIFT        (6U)
/*! SPEED - Speed Field
 *  0b00..low(50MHz)
 *  0b01..medium(100MHz)
 *  0b10..medium(100MHz)
 *  0b11..max(200MHz)
 */
#define IOMUXC_SW_PAD_CTL_PAD_SPEED(x)           (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_SPEED_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_SPEED_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_ODE_MASK           (0x800U)
#define IOMUXC_SW_PAD_CTL_PAD_ODE_SHIFT          (11U)
/*! ODE - Open Drain Enable Field
 *  0b0..Open Drain Disabled
 *  0b1..Open Drain Enabled
 */
#define IOMUXC_SW_PAD_CTL_PAD_ODE(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_ODE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_ODE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_PKE_MASK           (0x1000U)
#define IOMUXC_SW_PAD_CTL_PAD_PKE_SHIFT          (12U)
/*! PKE - Pull / Keep Enable Field
 *  0b0..Pull/Keeper Disabled
 *  0b1..Pull/Keeper Enabled
 */
#define IOMUXC_SW_PAD_CTL_PAD_PKE(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_PKE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_PKE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_PUE_MASK           (0x2000U)
#define IOMUXC_SW_PAD_CTL_PAD_PUE_SHIFT          (13U)
/*! PUE - Pull / Keep Select Field
 *  0b0..Keeper
 *  0b1..Pull
 */
#define IOMUXC_SW_PAD_CTL_PAD_PUE(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_PUE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_PUE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_PUS_MASK           (0xC000U)
#define IOMUXC_SW_PAD_CTL_PAD_PUS_SHIFT          (14U)
/*! PUS - Pull Up / Down Config. Field
 *  0b00..100K Ohm Pull Down
 *  0b01..47K Ohm Pull Up
 *  0b10..100K Ohm Pull Up
 *  0b11..22K Ohm Pull Up
 */
#define IOMUXC_SW_PAD_CTL_PAD_PUS(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_PUS_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_PUS_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_HYS_MASK           (0x10000U)
#define IOMUXC_SW_PAD_CTL_PAD_HYS_SHIFT          (16U)
/*! HYS - Hyst. Enable Field
 *  0b0..Hysteresis Disabled
 *  0b1..Hysteresis Enabled
 */
#define IOMUXC_SW_PAD_CTL_PAD_HYS(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_HYS_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_HYS_MASK)
/*! @} */

/* The count of IOMUXC_SW_PAD_CTL_PAD */
#define IOMUXC_SW_PAD_CTL_PAD_COUNT              (124U)

/*! @name SELECT_INPUT - ANATOP_USB_OTG1_ID_SELECT_INPUT DAISY Register..XBAR1_IN23_SELECT_INPUT DAISY Register */
/*! @{ */
#define IOMUXC_SELECT_INPUT_DAISY_MASK           (0x7U)  /* Merged from fields with different position or width, of widths (1, 2, 3), largest definition used */
#define IOMUXC_SELECT_INPUT_DAISY_SHIFT          (0U)
/*! DAISY - Selecting Pads Involved in Daisy Chain.
 *  0b0..Selecting Pad: GPIO_AD_B0_01 for Mode: ALT3
 *  0b1..Selecting Pad: GPIO_AD_B1_02 for Mode: ALT0
 */
#define IOMUXC_SELECT_INPUT_DAISY(x)             (((uint32_t)(((uint32_t)(x)) << IOMUXC_SELECT_INPUT_DAISY_SHIFT)) & IOMUXC_SELECT_INPUT_DAISY_MASK)  /* Merged from fields with different position or width, of widths (1, 2, 3), largest definition used */
/*! @} */

/* The count of IOMUXC_SELECT_INPUT */
#define IOMUXC_SELECT_INPUT_COUNT                (154U)


/*!
 * @}
 */ /* end of group IOMUXC_Register_Masks */


/* IOMUXC - Peripheral instance base addresses */
/** Peripheral IOMUXC base address */
#define IOMUXC_BASE                              (0x401F8000u)
/** Peripheral IOMUXC base pointer */
#define IOMUXC                                   ((IOMUXC_Type *)IOMUXC_BASE)
/** Array initializer of IOMUXC peripheral base addresses */
#define IOMUXC_BASE_ADDRS                        { IOMUXC_BASE }
/** Array initializer of IOMUXC peripheral base pointers */
#define IOMUXC_BASE_PTRS                         { IOMUXC }

/*!
 * @}
 */ /* end of group IOMUXC_Peripheral_Access_Layer */


/*====================== FLEXSPI Secondary IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX          73
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX           72
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX          71
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX          70
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX          69
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX          68
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DQS_IDX            67

#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX          73
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX           72
#define SELECT_INPUT_FLEXSPIA_SEC_SCLK_IDX             53
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX          71
#define SELECT_INPUT_FLEXSPIA_SEC_DATA0_IDX            45
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX          70
#define SELECT_INPUT_FLEXSPIA_SEC_DATA1_IDX            46
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX          69
#define SELECT_INPUT_FLEXSPIA_SEC_DATA2_IDX            47
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX          68
#define SELECT_INPUT_FLEXSPIA_SEC_DATA3_IDX            48
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DQS_IDX            67
#define SELECT_INPUT_FLEXSPIA_SEC_DQS_IDX              44		 
	
#define FLEXSPIA_SEC_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0)

/*====================== FLEXSPI IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPIB_DQS_IDX            111
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX          112
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX          113
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX          114
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX          115
#define SW_MUX_CTL_PAD_FLEXSPIB_SS0_B_IDX          110
#define SW_MUX_CTL_PAD_FLEXSPIB_SS1_B_IDX          107
#define SW_MUX_CTL_PAD_FLEXSPIB_SCLK_IDX           116

#define SW_MUX_CTL_PAD_FLEXSPIA_DQS_IDX            117
#define SW_MUX_CTL_PAD_FLEXSPIA_SS0_B_IDX          118
#define SW_MUX_CTL_PAD_FLEXSPIA_SS1_B_IDX          106
#define SW_MUX_CTL_PAD_FLEXSPIA_SCLK_IDX           119
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA0_IDX          120
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA1_IDX          121
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA2_IDX          122
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA3_IDX          123
#define SW_MUX_CTL_PAD_FLEXSPIA_SCLK_B_IDX         116

#define SW_PAD_CTL_PAD_FLEXSPIB_DQS_IDX            111
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX          112
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX          113
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX          114
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX          115
#define SW_PAD_CTL_PAD_FLEXSPIB_SS0_B_IDX          110
#define SW_PAD_CTL_PAD_FLEXSPIB_SS1_B_IDX          107
#define SW_PAD_CTL_PAD_FLEXSPIB_SCLK_IDX           116

#define SW_PAD_CTL_PAD_FLEXSPIA_DQS_IDX            117
#define SW_PAD_CTL_PAD_FLEXSPIA_SS0_B_IDX          118
#define SW_PAD_CTL_PAD_FLEXSPIA_SS1_B_IDX          106
#define SW_PAD_CTL_PAD_FLEXSPIA_SCLK_IDX           119
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA0_IDX          120
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA1_IDX          121
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA2_IDX          122
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA3_IDX          123
#define SW_PAD_CTL_PAD_FLEXSPIA_SCLK_B_IDX         116

#define FLEXSPIA_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIB_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIA_SS1_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS1_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS0_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(4)
#define FLEXSPIB_DQS_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(4)

#define FLEXSPI_DQS_SW_PAD_CTL_VAL                                                                  \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |   \
     IOMUXC_SW_PAD_CTL_PAD_HYS(1))
#define FLEXSPI_SW_PAD_CTL_VAL                                                                      \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))

#define IOMUXC_PAD_SETTING_DSE_SHIFT (3)
#define IOMUXC_PAD_SETTING_DSE_MASK (0x07 << IOMUXC_PAD_SETTING_DSE_SHIFT)
#define IOMUXC_PAD_SETTING_DSE(x) (((x) << IOMUXC_PAD_SETTING_DSE_SHIFT) & IOMUXC_PAD_SETTING_DSE_MASK)

//ROM FUSEMAP
#define OCOTP_BASE                               (0x401F4000u)

#define FUSE_BANK0_OFFSET         0x400
#define HW_FUSE_REG_ADDR(n)       (OCOTP_BASE + FUSE_BANK0_OFFSET + ((n) * 0x10))
#define HW_OCOTP_REG_RD(n)        (*(volatile uint32_t *)HW_FUSE_REG_ADDR(n))
/* Flash Type */
#define ROM_OCOTP_FLASH_TYPE_MASK   0x00000700
#define ROM_OCOTP_FLASH_TYPE_SHIFT  ((uint8_t)8)
#define ROM_OCOTP_FLASH_TYPE_VALUE()    \
        ((SRC->SBMR1&ROM_OCOTP_FLASH_TYPE_MASK) >> ROM_OCOTP_FLASH_TYPE_SHIFT)
				
/* QSPI 2ND pinmux */
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT 20U
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_MASK (1U << ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT)
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_VALUE() \
            ((HW_OCOTP_REG_RD(4) & ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_MASK) >> ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT)				


/*====================== CLOCK Definitions ===========================*/
#define CCM_ANALOG_TUPLE(reg, shift) ((((reg)&0xFFFU) << 16U) | (shift))
#define CCM_ANALOG_TUPLE_SHIFT(tuple) (((uint32_t)tuple) & 0x1FU)
#define CCM_ANALOG_TUPLE_REG_OFF(base, tuple, off) \
    (*((volatile uint32_t *)((uint32_t)(base) + (((uint32_t)(tuple) >> 16U) & 0xFFFU) + (off))))
#define CCM_ANALOG_TUPLE_REG(base, tuple) CCM_ANALOG_TUPLE_REG_OFF(base, tuple, 0U)
/*!
 * @brief CCM Analog registers offset.
 */
#define PLL_ARM_OFFSET 0x00
#define PLL_SYS_OFFSET 0x30
#define PLL_USB1_OFFSET 0x10
#define PLL_AUDIO_OFFSET 0x70
#define PLL_VIDEO_OFFSET 0xA0
#define PLL_ENET_OFFSET 0xE0
#define PLL_USB2_OFFSET 0x20
		
#define CCM_ANALOG_PLL_BYPASS_SHIFT (16U)
#define CCM_ANALOG_PLL_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_BYPASS_CLK_SRC_SHIFT (14U)

#define CLKPN_FREQ 0U

/*! @brief PLL name */
typedef enum _clock_pll
{
    kCLOCK_PllArm   = CCM_ANALOG_TUPLE(PLL_ARM_OFFSET, CCM_ANALOG_PLL_ARM_ENABLE_SHIFT),     /*!< PLL ARM */
    kCLOCK_PllSys   = CCM_ANALOG_TUPLE(PLL_SYS_OFFSET, CCM_ANALOG_PLL_SYS_ENABLE_SHIFT),     /*!< PLL SYS */
    kCLOCK_PllUsb1  = CCM_ANALOG_TUPLE(PLL_USB1_OFFSET, CCM_ANALOG_PLL_USB1_ENABLE_SHIFT),   /*!< PLL USB1 */
    kCLOCK_PllAudio = CCM_ANALOG_TUPLE(PLL_AUDIO_OFFSET, CCM_ANALOG_PLL_AUDIO_ENABLE_SHIFT), /*!< PLL Audio */
    kCLOCK_PllVideo = CCM_ANALOG_TUPLE(PLL_VIDEO_OFFSET, CCM_ANALOG_PLL_VIDEO_ENABLE_SHIFT), /*!< PLL Video */

    kCLOCK_PllEnet = CCM_ANALOG_TUPLE(PLL_ENET_OFFSET, CCM_ANALOG_PLL_ENET_ENABLE_SHIFT), /*!< PLL Enet0 */

    kCLOCK_PllEnet25M = CCM_ANALOG_TUPLE(PLL_ENET_OFFSET, CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN_SHIFT), /*!< PLL Enet1 */

    kCLOCK_PllUsb2 = CCM_ANALOG_TUPLE(PLL_USB2_OFFSET, CCM_ANALOG_PLL_USB2_ENABLE_SHIFT), /*!< PLL USB2 */

} clock_pll_t;

/*! @brief PLL PFD name */
typedef enum _clock_pfd
{
    kCLOCK_Pfd0 = 0U, /*!< PLL PFD0 */
    kCLOCK_Pfd1 = 1U, /*!< PLL PFD1 */
    kCLOCK_Pfd2 = 2U, /*!< PLL PFD2 */
    kCLOCK_Pfd3 = 3U, /*!< PLL PFD3 */
} clock_pfd_t;

typedef uint64_t clock_64b_t;
		
/*!@brief PLL clock source, bypass cloco source also */
enum _clock_pll_clk_src
{
    kCLOCK_PllClkSrc24M = 0U, /*!< Pll clock source 24M */
    kCLOCK_PllSrcClkPN  = 1U, /*!< Pll clock source CLK1_P and CLK1_N */
};

#endif // __BL_COMMON_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
