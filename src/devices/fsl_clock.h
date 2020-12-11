/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_CLOCK_H_
#define _FSL_CLOCK_H_

#include "fsl_common.h"

/*! @addtogroup clock */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Configurations
 ******************************************************************************/

/*! @brief Configure whether driver controls clock
 *
 * When set to 0, peripheral drivers will enable clock in initialize function
 * and disable clock in de-initialize function. When set to 1, peripheral
 * driver will not control the clock, application could control the clock out of
 * the driver.
 *
 * @note All drivers share this feature switcher. If it is set to 1, application
 * should handle clock enable and disable for all drivers.
 */
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL))
#define FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL 0
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief CLOCK driver version 2.1.1. */
#define FSL_CLOCK_DRIVER_VERSION (MAKE_VERSION(2, 1, 1))

/* Definition for delay API in clock driver, users can redefine it to the real application. */
#ifndef SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY
#define SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY (1000000000UL)
#endif

/* analog pll definition */
#define CCM_ANALOG_PLL_BYPASS_SHIFT (16U)
#define CCM_ANALOG_PLL_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_BYPASS_CLK_SRC_SHIFT (14U)

/*@}*/

/*!
 * @brief CCM registers offset.
 */
#define CCSR_OFFSET 0x0C
#define CBCDR_OFFSET 0x14
#define CBCMR_OFFSET 0x18
#define CSCMR1_OFFSET 0x1C
#define CSCMR2_OFFSET 0x20
#define CSCDR1_OFFSET 0x24
#define CDCDR_OFFSET 0x30
#define CSCDR2_OFFSET 0x38
#define CSCDR3_OFFSET 0x3C
#define CACRR_OFFSET 0x10
#define CS1CDR_OFFSET 0x28
#define CS2CDR_OFFSET 0x2C

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

#define CCM_TUPLE(reg, shift, mask, busyShift) \
    (int)((reg & 0xFFU) | ((shift) << 8U) | ((((mask) >> (shift)) & 0x1FFFU) << 13U) | ((busyShift) << 26U))
#define CCM_TUPLE_REG(base, tuple) (*((volatile uint32_t *)(((uint32_t)(base)) + ((tuple)&0xFFU))))
#define CCM_TUPLE_SHIFT(tuple) (((tuple) >> 8U) & 0x1FU)
#define CCM_TUPLE_MASK(tuple) ((uint32_t)((((tuple) >> 13U) & 0x1FFFU) << ((((tuple) >> 8U) & 0x1FU))))
#define CCM_TUPLE_BUSY_SHIFT(tuple) (((tuple) >> 26U) & 0x3FU)

#define CCM_BUSY_WAIT (0x20U)

/*!
 * @brief CCM ANALOG tuple macros to map corresponding registers and bit fields.
 */
#define CCM_ANALOG_TUPLE(reg, shift) (((reg & 0xFFFU) << 16U) | (shift))
#define CCM_ANALOG_TUPLE_SHIFT(tuple) (((uint32_t)tuple) & 0x1FU)
#define CCM_ANALOG_TUPLE_REG_OFF(base, tuple, off) \
    (*((volatile uint32_t *)((uint32_t)base + (((uint32_t)tuple >> 16U) & 0xFFFU) + off)))
#define CCM_ANALOG_TUPLE_REG(base, tuple) CCM_ANALOG_TUPLE_REG_OFF(base, tuple, 0U)

/*!
 * @brief SYS_PLL_FREQ frequency in Hz.
 */
#define SYS_PLL1_FREQ (1000000000U)
#define SYS_PLL2_FREQ (528000000U)
#define SYS_PLL3_FREQ (480000000U)
#define XTAL_FREQ (24000000U)

/*! @brief Clock gate name array for ADC. */
#define LPADC_CLOCKS                                 \
    {                                              \
        kCLOCK_IpInvalid, kCLOCK_Lpadc1, kCLOCK_Lpadc2 \
    }

/*! @brief Clock gate name array for ADC. */
#define ADC_ETC_CLOCKS                                 \
    {                                              \
        kCLOCK_Adc_Etc \
    }

/*! @brief Clock gate name array for AOI. */
#define AOI_CLOCKS                                 \
    {                                              \
        kCLOCK_IpInvalid, kCLOCK_Aoi1, kCLOCK_Aoi2 \
    }

/*! @brief Clock gate name array for DCDC. */
#define DCDC_CLOCKS \
    {              \
        kCLOCK_Dcdc \
    }

/*! @brief Clock gate name array for SRC. */
#define SRC_CLOCKS \
    {              \
        kCLOCK_Src \
    }

/*! @brief Clock gate name array for GPC. */
#define GPC_CLOCKS \
    {              \
        kCLOCK_Gpc \
    }

/*! @brief Clock gate name array for SSARC. */
#define SSARC_CLOCKS \
    {              \
        kCLOCK_Ssarc \
    }

/*! @brief Clock gate name array for WDOG. */
#define WDOG_CLOCKS \
    {              \
        kCLOCK_IpInvalid, kCLOCK_Wdog1, kCLOCK_Wdog2, kCLOCK_Wdog3, kCLOCK_Wdog4\
    }

/*! @brief Clock gate name array for EWM. */
#define EWM_CLOCKS \
    {              \
        kCLOCK_Ewm0 \
    }

/*! @brief Clock gate name array for Sema. */
#define SEMA_CLOCKS \
    {              \
        kCLOCK_Sema \
    }

/*! @brief Clock gate name array for MU. */
#define MU_CLOCKS \
    {              \
        kCLOCK_Mu_A, kCLOCK_Mu_B \
    }

/*! @brief Clock gate name array for EDMA. */
#define EDMA_CLOCKS \
    {              \
        kCLOCK_Edma, kCLOCK_Edma_Lpsr \
    }

/*! @brief Clock gate name array for FLEXRAM. */
#define FLEXRAM_CLOCKS \
    {              \
        kCLOCK_Flexram \
    }

/*! @brief Clock gate name array for LMEM. */
#define LMEM_CLOCKS \
    {              \
        kCLOCK_Lmem \
    }

/*! @brief Clock gate name array for FLEXSPI. */
#define FLEXSPI_CLOCKS \
    {              \
        kCLOCK_IpInvalid, kCLOCK_Flexspi1, kCLOCK_Flexspi2 \
    }

/*! @brief Clock gate name array for RDC. */
#define RDC_CLOCKS \
    {              \
        kCLOCK_Rdc, kCLOCK_M7_Xrdc, kCLOCK_M4_Xrdc \
    }

/*! @brief Clock ip name array for DCDC. */
#define DCDC_CLOCKS \
    {               \
        kCLOCK_Dcdc \
    }

/*! @brief Clock ip name array for SEMC. */
#define SEMC_CLOCKS \
    {               \
        kCLOCK_Semc \
    }

/*! @brief Clock ip name array for XECC. */
#define XECC_CLOCKS \
    {               \
        kCLOCK_Xecc \
    }

/*! @brief Clock ip name array for IEE. */
#define IEE_CLOCKS \
    {               \
        kCLOCK_Iee \
    }

/*! @brief Clock ip name array for KEY_MANAGER. */
#define KEYMANAGER_CLOCKS \
    {               \
        kCLOCK_Key_Manager \
    }

/*! @brief Clock ip name array for OCOTP. */
#define OCOTP_CLOCKS \
    {               \
        kCLOCK_Ocotp \
    }

/*! @brief Clock ip name array for CAAM. */
#define CAAM_CLOCKS \
    {               \
        kCLOCK_Caam\
    }

/*! @brief Clock ip name array for XBAR. */
#define XBAR_CLOCKS \
    {               \
        kCLOCK_IpInvalid, kCLOCK_Xbar1, kCLOCK_Xbar2, kCLOCK_Xbar3 \
    }

/*! @brief Clock ip name array for IOMUXC. */
#define IOMUXC_CLOCKS \
    {               \
        kCLOCK_Iomuxc, kCLOCK_Iomuxc_Lpsr \
    }

/*! @brief Clock ip name array for GPIO. */
#define GPIO_CLOCKS \
    {               \
        kCLOCK_IpInvalid, kCLOCK_Gpio, kCLOCK_Gpio, kCLOCK_Gpio, kCLOCK_Gpio,  \
        kCLOCK_Gpio, kCLOCK_Gpio, kCLOCK_Gpio, kCLOCK_Gpio, \
        kCLOCK_Gpio, kCLOCK_Gpio, kCLOCK_Gpio, kCLOCK_Gpio, kCLOCK_Gpio, \
    }

/*! @brief Clock ip name array for KPP. */
#define KPP_CLOCKS \
    {               \
        kCLOCK_Kpp \
    }

/*! @brief Clock ip name array for FLEXIO. */
#define FLEXIO_CLOCKS                                                    \
    {                                                                    \
        kCLOCK_IpInvalid, kCLOCK_Flexio1, kCLOCK_Flexio2 \
    }

/*! @brief Clock ip name array for DAC. */
#define DAC_CLOCKS \
    {               \
        kCLOCK_Dac \
    }

/*! @brief Clock ip name array for CMP. */
#define CMP_CLOCKS                                                               \
    {                                                                            \
        kCLOCK_IpInvalid, kCLOCK_Acmp1, kCLOCK_Acmp2, kCLOCK_Acmp3, kCLOCK_Acmp4 \
    }

/*! @brief Clock ip name array for PIT. */
#define PIT_CLOCKS \
    {              \
        kCLOCK_IpInvalid, kCLOCK_Pit1, kCLOCK_Pit2 \
    }

/*! @brief Clock ip name array for GPT. */
#define GPT_CLOCKS                                 \
    {                                              \
        kCLOCK_IpInvalid, kCLOCK_Gpt1, kCLOCK_Gpt2, kCLOCK_Gpt3, kCLOCK_Gpt4, \
        kCLOCK_Gpt5, kCLOCK_Gpt6 \
    }

/*! @brief Clock ip name array for QTIMER. */
#define TMR_CLOCKS                                                                       \
    {                                                                                    \
        kCLOCK_IpInvalid, kCLOCK_Qtimer1, kCLOCK_Qtimer2, kCLOCK_Qtimer3, kCLOCK_Qtimer4 \
    }

/*! @brief Clock ip name array for ENC. */
#define ENC_CLOCKS                                                           \
    {                                                                        \
        kCLOCK_IpInvalid, kCLOCK_Enc1, kCLOCK_Enc2, kCLOCK_Enc3, kCLOCK_Enc4 \
    }

/*! @brief Clock ip name array for PWM. */
#define PWM_CLOCKS                                                                \
    {                                                                             \
        {kCLOCK_IpInvalid, kCLOCK_IpInvalid, kCLOCK_IpInvalid, kCLOCK_IpInvalid}, \
            {kCLOCK_Pwm1, kCLOCK_Pwm1, kCLOCK_Pwm1, kCLOCK_Pwm1},                 \
            {kCLOCK_Pwm2, kCLOCK_Pwm2, kCLOCK_Pwm2, kCLOCK_Pwm2},                 \
            {kCLOCK_Pwm3, kCLOCK_Pwm3, kCLOCK_Pwm3, kCLOCK_Pwm3},                 \
        {                                                                         \
            kCLOCK_Pwm4, kCLOCK_Pwm4, kCLOCK_Pwm4, kCLOCK_Pwm4                    \
        }                                                                         \
    }

/*! @brief Clock ip name array for FLEXCAN. */
#define FLEXCAN_CLOCKS                                          \
    {                                                           \
        kCLOCK_IpInvalid, kCLOCK_Can1, kCLOCK_Can2, kCLOCK_Can3 \
    }

/*! @brief Clock ip name array for LPUART. */
#define LPUART_CLOCKS                    \
    {                                    \
        kCLOCK_IpInvalid, kCLOCK_Lpuart1, kCLOCK_Lpuart2, kCLOCK_Lpuart3, \
        kCLOCK_Lpuart4, kCLOCK_Lpuart5, kCLOCK_Lpuart6, kCLOCK_Lpuart7, \
        kCLOCK_Lpuart8, kCLOCK_Lpuart9, kCLOCK_Lpuart10, kCLOCK_Lpuart11, \
        kCLOCK_Lpuart12 \
    }

/*! @brief Clock ip name array for LPI2C. */
#define LPI2C_CLOCKS                                                                 \
    {                                                                                \
        kCLOCK_IpInvalid, kCLOCK_Lpi2c1, kCLOCK_Lpi2c2, kCLOCK_Lpi2c3, kCLOCK_Lpi2c4, \
        kCLOCK_Lpi2c5, kCLOCK_Lpi2c6 \
    }

/*! @brief Clock ip name array for LPSPI. */
#define LPSPI_CLOCKS                                                                  \
    {                                                                                 \
        kCLOCK_IpInvalid, kCLOCK_Lpspi1, kCLOCK_Lpspi2, kCLOCK_Lpspi3, kCLOCK_Lpspi4, \
        kCLOCK_Lpspi5, kCLOCK_Lpspi6 \
    }

/*! @brief Clock ip name array for EMVSIM. */
#define EMVSIM_CLOCKS                  \
    {                                  \
        kCLOCK_IpInvalid, kCLOCK_Sim1, kCLOCK_Sim2    \
    } 

/*! @brief Clock ip name array for ENET. */
#define ENET_CLOCKS                                 \
    {                                               \
        kCLOCK_Enet, kCLOCK_Enet_1g \
    }

/*! @brief Clock ip name array for ENET_QOS. */
#define ENETQOS_CLOCKS                                 \
    {                                               \
        kCLOCK_Enet_Qos \
    }

/*! @brief Clock ip name array for USB. */
#define USB_CLOCKS                                 \
    {                                               \
        kCLOCK_Usb \
    }

/*! @brief Clock ip name array for SDIO. */
#define SDIO_CLOCKS                                 \
    {                                               \
        kCLOCK_Sdio \
    }

/*! @brief Clock ip name array for USDHC. */
#define USDHC_CLOCKS                                   \
    {                                                  \
        kCLOCK_IpInvalid, kCLOCK_Usdhc1, kCLOCK_Usdhc2 \
    }

/*! @brief Clock ip name array for ASRC. */
#define ASRC_CLOCKS                                 \
    {                                               \
        kCLOCK_Asrc \
    }

/*! @brief Clock ip name array for MQS. */
#define MQS_CLOCKS \
    {              \
        kCLOCK_Mqs \
    }

/*! @brief Clock ip name array for PDM. */
#define PDM_CLOCKS \
    {              \
        kCLOCK_Pdm \
    }

/*! @brief Clock ip name array for SPDIF. */
#define SPDIF_CLOCKS \
    {                \
        kCLOCK_Spdif \
    }

/*! @brief Clock ip name array for SAI. */
#define SAI_CLOCKS                                              \
    {                                                           \
        kCLOCK_IpInvalid, kCLOCK_Sai1, kCLOCK_Sai2, kCLOCK_Sai3, kCLOCK_Sai4 \
    }

/*! @brief Clock ip name array for PXP. */
#define PXP_CLOCKS \
    {              \
        kCLOCK_Pxp \
    }

/*! @brief Clock ip name array for GPU2d. */
#define GPU2D_CLOCKS \
    {              \
        kCLOCK_Gpu2d \
    }

/*! @brief Clock ip name array for LCDIF. */
#define LCDIF_CLOCKS \
    {                \
        kCLOCK_Lcdif   \
    }

/*! @brief Clock ip name array for LCDIFV2. */
#define LCDIFV2_CLOCKS \
    {                       \
        kCLOCK_Lcdifv2 \
    }

/*! @brief Clock ip name array for MIPI_DSI. */
#define MIPI_DSI_HOST_CLOCKS \
    {                       \
        kCLOCK_Mipi_Dsi \
    }

/*! @brief Clock ip name array for MIPI_CSI. */
#define MIPI_CSI2RX_CLOCKS \
    {                       \
        kCLOCK_Mipi_Csi \
    }

/*! @brief Clock ip name array for CSI. */
#define CSI_CLOCKS \
    {              \
        kCLOCK_Csi \
    }

/*! @brief Clock ip name array for DCIC_MIPI. */
#define DCIC_MIPI_CLOCKS \
    {              \
        kCLOCK_Dcic_Mipi \
    }

/*! @brief Clock ip name array for DCIC_LCD. */
#define DCIC_LCD_CLOCKS \
    {              \
        kCLOCK_Dcic_Lcd \
    }

/*! @brief Clock ip name array for DMAMUX_CLOCKS. */
#define DMAMUX_CLOCKS \
    {                 \
        kCLOCK_Edma, kCLOCK_Edma_Lpsr    \
    }

/*! @brief Clock ip name array for XBARA. */
#define XBARA_CLOCKS \
    {                \
        kCLOCK_Xbar1 \
    }

/*! @brief Clock ip name array for XBARB. */
#define XBARB_CLOCKS                                                   \
    {                                                                  \
        kCLOCK_IpInvalid, kCLOCK_IpInvalid, kCLOCK_Xbar2, kCLOCK_Xbar3 \
    }

/* Clock LPCG index */
typedef enum _clock_lpcg
{
    kCLOCK_M7 = 0,
    kCLOCK_M4 = 1,
    kCLOCK_Sim_M7 = 2,
    kCLOCK_Sim_M = 3,
    kCLOCK_Sim_Disp = 4,
    kCLOCK_Sim_Per = 5,
    kCLOCK_Sim_Lpsr = 6,
    kCLOCK_Anadig = 7,
    kCLOCK_Dcdc = 8,
    kCLOCK_Src = 9,
    kCLOCK_Ccm = 10,
    kCLOCK_Gpc = 11,
    kCLOCK_Ssarc = 12,
    kCLOCK_Sim_R = 13,
    kCLOCK_Wdog1 = 14,
    kCLOCK_Wdog2 = 15,
    kCLOCK_Wdog3 = 16,
    kCLOCK_Wdog4 = 17,
    kCLOCK_Ewm0 = 18,
    kCLOCK_Sema = 19,
    kCLOCK_Mu_A = 20,
    kCLOCK_Mu_B = 21,
    kCLOCK_Edma = 22,
    kCLOCK_Edma_Lpsr = 23,
    kCLOCK_Romcp = 24,
    kCLOCK_Ocram = 25,
    kCLOCK_Flexram = 26,
    kCLOCK_Lmem = 27,
    kCLOCK_Flexspi1 = 28,
    kCLOCK_Flexspi2 = 29,
    kCLOCK_Rdc = 30,
    kCLOCK_M7_Xrdc = 31,
    kCLOCK_M4_Xrdc = 32,
    kCLOCK_Semc = 33,
    kCLOCK_Xecc = 34,
    kCLOCK_Iee = 35,
    kCLOCK_Key_Manager = 36,
    kCLOCK_Ocotp = 37,
    kCLOCK_Snvs_Hp = 38,
    kCLOCK_Snvs = 39,
    kCLOCK_Caam = 40,
    kCLOCK_Jtag_Mux = 41,
    kCLOCK_Cstrace = 42,
    kCLOCK_Xbar1 = 43,
    kCLOCK_Xbar2 = 44,
    kCLOCK_Xbar3 = 45,
    kCLOCK_Aoi1 = 46,
    kCLOCK_Aoi2 = 47,
    kCLOCK_Adc_Etc = 48,
    kCLOCK_Iomuxc = 49,
    kCLOCK_Iomuxc_Lpsr = 50,
    kCLOCK_Gpio = 51,
    kCLOCK_Kpp = 52,
    kCLOCK_Flexio1 = 53,
    kCLOCK_Flexio2 = 54,
    kCLOCK_Lpadc1 = 55,
    kCLOCK_Lpadc2 = 56,
    kCLOCK_Dac = 57,
    kCLOCK_Acmp1 = 58,
    kCLOCK_Acmp2 = 59,
    kCLOCK_Acmp3 = 60,
    kCLOCK_Acmp4 = 61,
    kCLOCK_Pit1 = 62,
    kCLOCK_Pit2 = 63,
    kCLOCK_Gpt1 = 64,
    kCLOCK_Gpt2 = 65,
    kCLOCK_Gpt3 = 66,
    kCLOCK_Gpt4 = 67,
    kCLOCK_Gpt5 = 68,
    kCLOCK_Gpt6 = 69,
    kCLOCK_Qtimer1 = 70,
    kCLOCK_Qtimer2 = 71,
    kCLOCK_Qtimer3 = 72,
    kCLOCK_Qtimer4 = 73,
    kCLOCK_Enc1 = 74,
    kCLOCK_Enc2 = 75,
    kCLOCK_Enc3 = 76,
    kCLOCK_Enc4 = 77,
    kCLOCK_Hrtimer = 78,
    kCLOCK_Pwm1 = 79,
    kCLOCK_Pwm2 = 80,
    kCLOCK_Pwm3 = 81,
    kCLOCK_Pwm4 = 82,
    kCLOCK_Can1 = 83,
    kCLOCK_Can2 = 84,
    kCLOCK_Can3 = 85,
    kCLOCK_Lpuart1 = 86,
    kCLOCK_Lpuart2 = 87,
    kCLOCK_Lpuart3 = 88,
    kCLOCK_Lpuart4 = 89,
    kCLOCK_Lpuart5 = 90,
    kCLOCK_Lpuart6 = 91,
    kCLOCK_Lpuart7 = 92,
    kCLOCK_Lpuart8 = 93,
    kCLOCK_Lpuart9 = 94,
    kCLOCK_Lpuart10 = 95,
    kCLOCK_Lpuart11 = 96,
    kCLOCK_Lpuart12 = 97,
    kCLOCK_Lpi2c1 = 98,
    kCLOCK_Lpi2c2 = 99,
    kCLOCK_Lpi2c3 = 100,
    kCLOCK_Lpi2c4 = 101,
    kCLOCK_Lpi2c5 = 102,
    kCLOCK_Lpi2c6 = 103,
    kCLOCK_Lpspi1 = 104,
    kCLOCK_Lpspi2 = 105,
    kCLOCK_Lpspi3 = 106,
    kCLOCK_Lpspi4 = 107,
    kCLOCK_Lpspi5 = 108,
    kCLOCK_Lpspi6 = 109,
    kCLOCK_Sim1 = 110,
    kCLOCK_Sim2 = 111,
    kCLOCK_Enet = 112,
    kCLOCK_Enet_1g = 113,
    kCLOCK_Enet_Qos = 114,
    kCLOCK_Usb = 115,
    kCLOCK_Sdio = 116,
    kCLOCK_Usdhc1 = 117,
    kCLOCK_Usdhc2 = 118,
    kCLOCK_Asrc = 119,
    kCLOCK_Mqs = 120,
    kCLOCK_Pdm = 121,
    kCLOCK_Spdif = 122,
    kCLOCK_Sai1 = 123,
    kCLOCK_Sai2 = 124,
    kCLOCK_Sai3 = 125,
    kCLOCK_Sai4 = 126,
    kCLOCK_Pxp = 127,
    kCLOCK_Gpu2d = 128,
    kCLOCK_Lcdif = 129,
    kCLOCK_Lcdifv2 = 130,
    kCLOCK_Mipi_Dsi = 131,
    kCLOCK_Mipi_Csi = 132,
    kCLOCK_Csi = 133,
    kCLOCK_Dcic_Mipi = 134,
    kCLOCK_Dcic_Lcd = 135,
    kCLOCK_Video_Mux = 136,
    kCLOCK_Uniq_Edt_I = 137,

    kCLOCK_IpInvalid,
} clock_lpcg_t;

/* Clock name */
typedef enum _clock_name
{
    kCLOCK_OscRc16M = 0,
    kCLOCK_OscRc48M = 1,
    kCLOCK_OscRc48MDiv2 = 2,
    kCLOCK_OscRc400M = 3,
    kCLOCK_Osc24M = 4,
    kCLOCK_Osc24MOut = 5,
    kCLOCK_ArmPll = 6,
    kCLOCK_ArmPllOut = 7,
    kCLOCK_SysPll2 = 8,
    kCLOCK_SysPll2Out = 9,
    kCLOCK_SysPll2Pfd0 = 10,
    kCLOCK_SysPll2Pfd1 = 11,
    kCLOCK_SysPll2Pfd2 = 12,
    kCLOCK_SysPll2Pfd3 = 13,
    kCLOCK_SysPll3 = 14,
    kCLOCK_SysPll3Out = 15,
    kCLOCK_SysPll3Div2 = 16,
    kCLOCK_SysPll3Pfd0 = 17,
    kCLOCK_SysPll3Pfd1 = 18,
    kCLOCK_SysPll3Pfd2 = 19,
    kCLOCK_SysPll3Pfd3 = 20,
    kCLOCK_SysPll1 = 21,
    kCLOCK_SysPll1Out = 22,
    kCLOCK_SysPll1Div2 = 23,
    kCLOCK_SysPll1Div5 = 24,
    kCLOCK_AudioPll = 25,
    kCLOCK_AudioPllOut = 26,
    kCLOCK_VideoPll = 27,
    kCLOCK_VideoPllOut = 28,
    kCLOCK_CpuClk,
    kCLOCK_CoreSysClk,
} clock_name_t;

/* Clock OBSERVE SIGNALS */
#define CCM_OBS_CCM_SOC_POWERDOWN                                      1, 2
#define CCM_OBS_M7_1_CH_POWERDOWN                                      2, 2
#define CCM_OBS_M7_2_CH_POWERDOWN                                      3, 2
#define CCM_OBS_OSC_RC_16M_CH_SILENT_N                                32, 0
#define CCM_OBS_OSC_RC_48M_CH_SILENT_N                                33, 0
#define CCM_OBS_OSC_RC_48M_DIV2_CH_SILENT_N                           34, 0
#define CCM_OBS_OSC_RC_400M_CH_SILENT_N                               35, 0
#define CCM_OBS_OSC_24M_OUT_CH_SILENT_N                               37, 0
#define CCM_OBS_PLL_ARM_OUT_CH_SILENT_N                               39, 1
#define CCM_OBS_PLL_528_OUT_CH_SILENT_N                               41, 1
#define CCM_OBS_PLL_528_PFD0_CH_SILENT_N                              42, 1
#define CCM_OBS_PLL_528_PFD1_CH_SILENT_N                              43, 1
#define CCM_OBS_PLL_528_PFD2_CH_SILENT_N                              44, 1
#define CCM_OBS_PLL_528_PFD3_CH_SILENT_N                              45, 1
#define CCM_OBS_PLL_480_OUT_CH_SILENT_N                               47, 1
#define CCM_OBS_PLL_480_DIV2_CH_SILENT_N                              48, 1
#define CCM_OBS_PLL_480_PFD0_CH_SILENT_N                              49, 1
#define CCM_OBS_PLL_480_PFD1_CH_SILENT_N                              50, 1
#define CCM_OBS_PLL_480_PFD2_CH_SILENT_N                              51, 1
#define CCM_OBS_PLL_480_PFD3_CH_SILENT_N                              52, 1
#define CCM_OBS_PLL_1G_OUT_CH_SILENT_N                                54, 1
#define CCM_OBS_PLL_1G_DIV2_CH_SILENT_N                               55, 1
#define CCM_OBS_PLL_1G_DIV5_CH_SILENT_N                               56, 1
#define CCM_OBS_PLL_AUDIO_OUT_CH_SILENT_N                             58, 0
#define CCM_OBS_PLL_VIDEO_OUT_CH_SILENT_N                             60, 1
#define CCM_OBS_CCM_SOC_OSC_RC_16M_IN_USE                             64, 2
#define CCM_OBS_CCM_SOC_OSC_RC_48M_IN_USE                             65, 2
#define CCM_OBS_CCM_SOC_OSC_RC_48M_DIV2_IN_USE                        66, 2
#define CCM_OBS_CCM_SOC_OSC_RC_400M_IN_USE                            67, 2
#define CCM_OBS_CCM_SOC_OSC_24M_OUT_IN_USE                            69, 2
#define CCM_OBS_CCM_SOC_PLL_ARM_OUT_IN_USE                            71, 2
#define CCM_OBS_CCM_SOC_PLL_528_OUT_IN_USE                            73, 2
#define CCM_OBS_CCM_SOC_PLL_528_PFD0_IN_USE                           74, 2
#define CCM_OBS_CCM_SOC_PLL_528_PFD1_IN_USE                           75, 2
#define CCM_OBS_CCM_SOC_PLL_528_PFD2_IN_USE                           76, 2
#define CCM_OBS_CCM_SOC_PLL_528_PFD3_IN_USE                           77, 2
#define CCM_OBS_CCM_SOC_PLL_480_OUT_IN_USE                            79, 2
#define CCM_OBS_CCM_SOC_PLL_480_DIV2_IN_USE                           80, 2
#define CCM_OBS_CCM_SOC_PLL_480_PFD0_IN_USE                           81, 2
#define CCM_OBS_CCM_SOC_PLL_480_PFD1_IN_USE                           82, 2
#define CCM_OBS_CCM_SOC_PLL_480_PFD2_IN_USE                           83, 2
#define CCM_OBS_CCM_SOC_PLL_480_PFD3_IN_USE                           84, 2
#define CCM_OBS_CCM_SOC_PLL_1G_OUT_IN_USE                             86, 2
#define CCM_OBS_CCM_SOC_PLL_1G_DIV2_IN_USE                            87, 2
#define CCM_OBS_CCM_SOC_PLL_1G_DIV5_IN_USE                            88, 2
#define CCM_OBS_CCM_SOC_PLL_AUDIO_OUT_IN_USE                          90, 2
#define CCM_OBS_CCM_SOC_PLL_VIDEO_OUT_IN_USE                          92, 2
#define CCM_OBS_M7_CLK_ROOT                                          128, 4
#define CCM_OBS_M4_CLK_ROOT                                          129, 0
#define CCM_OBS_BUS_CLK_ROOT                                         130, 2
#define CCM_OBS_BUS_LPSR_CLK_ROOT                                    131, 0
#define CCM_OBS_SEMC_CLK_ROOT                                        132, 2
#define CCM_OBS_CSSYS_CLK_ROOT                                       133, 2
#define CCM_OBS_CSTRACE_CLK_ROOT                                     134, 2
#define CCM_OBS_M4_SYSTICK_CLK_ROOT                                  135, 0
#define CCM_OBS_M7_SYSTICK_CLK_ROOT                                  136, 2
#define CCM_OBS_ADC1_CLK_ROOT                                        137, 2
#define CCM_OBS_ADC2_CLK_ROOT                                        138, 2
#define CCM_OBS_ACMP_CLK_ROOT                                        139, 2
#define CCM_OBS_FLEXIO1_CLK_ROOT                                     140, 2
#define CCM_OBS_FLEXIO2_CLK_ROOT                                     141, 2
#define CCM_OBS_GPT1_CLK_ROOT                                        142, 2
#define CCM_OBS_GPT2_CLK_ROOT                                        143, 2
#define CCM_OBS_GPT3_CLK_ROOT                                        144, 2
#define CCM_OBS_GPT4_CLK_ROOT                                        145, 2
#define CCM_OBS_GPT5_CLK_ROOT                                        146, 2
#define CCM_OBS_GPT6_CLK_ROOT                                        147, 2
#define CCM_OBS_FLEXSPI1_CLK_ROOT                                    148, 2
#define CCM_OBS_FLEXSPI2_CLK_ROOT                                    149, 2
#define CCM_OBS_CAN1_CLK_ROOT                                        150, 2
#define CCM_OBS_CAN2_CLK_ROOT                                        151, 2
#define CCM_OBS_CAN3_CLK_ROOT                                        152, 0
#define CCM_OBS_LPUART1_CLK_ROOT                                     153, 2
#define CCM_OBS_LPUART2_CLK_ROOT                                     154, 2
#define CCM_OBS_LPUART3_CLK_ROOT                                     155, 2
#define CCM_OBS_LPUART4_CLK_ROOT                                     156, 2
#define CCM_OBS_LPUART5_CLK_ROOT                                     157, 2
#define CCM_OBS_LPUART6_CLK_ROOT                                     158, 2
#define CCM_OBS_LPUART7_CLK_ROOT                                     159, 2
#define CCM_OBS_LPUART8_CLK_ROOT                                     160, 2
#define CCM_OBS_LPUART9_CLK_ROOT                                     161, 2
#define CCM_OBS_LPUART10_CLK_ROOT                                    162, 2
#define CCM_OBS_LPUART11_CLK_ROOT                                    163, 0
#define CCM_OBS_LPUART12_CLK_ROOT                                    164, 0
#define CCM_OBS_LPI2C1_CLK_ROOT                                      165, 2
#define CCM_OBS_LPI2C2_CLK_ROOT                                      166, 2
#define CCM_OBS_LPI2C3_CLK_ROOT                                      167, 2
#define CCM_OBS_LPI2C4_CLK_ROOT                                      168, 2
#define CCM_OBS_LPI2C5_CLK_ROOT                                      169, 0
#define CCM_OBS_LPI2C6_CLK_ROOT                                      170, 0
#define CCM_OBS_LPSPI1_CLK_ROOT                                      171, 2
#define CCM_OBS_LPSPI2_CLK_ROOT                                      172, 2
#define CCM_OBS_LPSPI3_CLK_ROOT                                      173, 2
#define CCM_OBS_LPSPI4_CLK_ROOT                                      174, 2
#define CCM_OBS_LPSPI5_CLK_ROOT                                      175, 0
#define CCM_OBS_LPSPI6_CLK_ROOT                                      176, 0
#define CCM_OBS_EMV1_CLK_ROOT                                        177, 2
#define CCM_OBS_EMV2_CLK_ROOT                                        178, 2
#define CCM_OBS_ENET1_CLK_ROOT                                       179, 2
#define CCM_OBS_ENET2_CLK_ROOT                                       180, 2
#define CCM_OBS_ENET_QOS_CLK_ROOT                                    181, 2
#define CCM_OBS_ENET_25M_CLK_ROOT                                    182, 2
#define CCM_OBS_ENET_TIMER1_CLK_ROOT                                 183, 2
#define CCM_OBS_ENET_TIMER2_CLK_ROOT                                 184, 2
#define CCM_OBS_ENET_TIMER3_CLK_ROOT                                 185, 2
#define CCM_OBS_USDHC1_CLK_ROOT                                      186, 2
#define CCM_OBS_USDHC2_CLK_ROOT                                      187, 2
#define CCM_OBS_ASRC_CLK_ROOT                                        188, 2
#define CCM_OBS_MQS_CLK_ROOT                                         189, 2
#define CCM_OBS_MIC_CLK_ROOT                                         190, 0
#define CCM_OBS_SPDIF_CLK_ROOT                                       191, 2
#define CCM_OBS_SAI1_CLK_ROOT                                        192, 2
#define CCM_OBS_SAI2_CLK_ROOT                                        193, 2
#define CCM_OBS_SAI3_CLK_ROOT                                        194, 2
#define CCM_OBS_SAI4_CLK_ROOT                                        195, 0
#define CCM_OBS_GC355_CLK_ROOT                                       196, 2
#define CCM_OBS_LCDIF_CLK_ROOT                                       197, 2
#define CCM_OBS_LCDIFV2_CLK_ROOT                                     198, 2
#define CCM_OBS_MIPI_REF_CLK_ROOT                                    199, 2
#define CCM_OBS_MIPI_ESC_CLK_ROOT                                    200, 2
#define CCM_OBS_CSI2_CLK_ROOT                                        201, 2
#define CCM_OBS_CSI2_ESC_CLK_ROOT                                    202, 2
#define CCM_OBS_CSI2_UI_CLK_ROOT                                     203, 2
#define CCM_OBS_CSI_CLK_ROOT                                         204, 2
#define CCM_OBS_CCM_CKO1_CLK_ROOT                                    205, 0
#define CCM_OBS_CCM_CKO2_CLK_ROOT                                    206, 2
#define CCM_OBS_CM7_CORE_STCLKEN                                     207, 4
#define CCM_OBS_CCM_FLEXRAM_CLK_ROOT                                 208, 4
#define CCM_OBS_MIPI_DSI_TXESC                                       209, 2
#define CCM_OBS_MIPI_DSI_RXESC                                       210, 2
#define CCM_OBS_OSC_RC_16M                                           224, 0
#define CCM_OBS_OSC_RC_48M                                           225, 0
#define CCM_OBS_OSC_RC_48M_DIV2                                      226, 0
#define CCM_OBS_OSC_RC_400M                                          227, 0
#define CCM_OBS_OSC_24M_OUT                                          229, 0
#define CCM_OBS_PLL_ARM_OUT                                          231, 2
#define CCM_OBS_PLL_528_OUT                                          233, 2
#define CCM_OBS_PLL_528_PFD0                                         234, 2
#define CCM_OBS_PLL_528_PFD1                                         235, 2
#define CCM_OBS_PLL_528_PFD2                                         236, 2
#define CCM_OBS_PLL_528_PFD3                                         237, 2
#define CCM_OBS_PLL_480_OUT                                          239, 2
#define CCM_OBS_PLL_480_DIV2                                         240, 2
#define CCM_OBS_PLL_480_PFD0                                         241, 2
#define CCM_OBS_PLL_480_PFD1                                         242, 2
#define CCM_OBS_PLL_480_PFD2                                         243, 2
#define CCM_OBS_PLL_480_PFD3                                         244, 2
#define CCM_OBS_PLL_1G_OUT                                           246, 2
#define CCM_OBS_PLL_1G_DIV2                                          247, 2
#define CCM_OBS_PLL_1G_DIV5                                          248, 2
#define CCM_OBS_PLL_AUDIO_OUT                                        250, 2
#define CCM_OBS_PLL_VIDEO_OUT                                        252, 2
#define CCM_OBS_OSC_32K                                              253, 0
#define CCM_OBS_CPU_POWER_MODE_0_0                                   256, 0
#define CCM_OBS_CPU_POWER_MODE_0_1                                   257, 0
#define CCM_OBS_CPU_MODE_TRANS_LPCG_DONE_0                           258, 0
#define CCM_OBS_CPU_MODE_TRANS_LPCG_REQUEST_0                        259, 0
#define CCM_OBS_CPU_MODE_TRANS_OSCPLL_DONE_0                         260, 0
#define CCM_OBS_CPU_MODE_TRANS_OSCPLL_REQUEST_0                      261, 0
#define CCM_OBS_CPU_POWER_MODE_1_0                                   264, 0
#define CCM_OBS_CPU_POWER_MODE_1_1                                   265, 0
#define CCM_OBS_CPU_MODE_TRANS_LPCG_DONE_1                           266, 0
#define CCM_OBS_CPU_MODE_TRANS_LPCG_REQUEST_1                        267, 0
#define CCM_OBS_CPU_MODE_TRANS_OSCPLL_DONE_1                         268, 0
#define CCM_OBS_CPU_MODE_TRANS_OSCPLL_REQUEST_1                      269, 0
#define CCM_OBS_CPU_POWER_MODE_2_0                                   272, 0
#define CCM_OBS_CPU_POWER_MODE_2_1                                   273, 0
#define CCM_OBS_CPU_MODE_TRANS_LPCG_DONE_2                           274, 0
#define CCM_OBS_CPU_MODE_TRANS_LPCG_REQUEST_2                        275, 0
#define CCM_OBS_CPU_MODE_TRANS_OSCPLL_DONE_2                         276, 0
#define CCM_OBS_CPU_MODE_TRANS_OSCPLL_REQUEST_2                      277, 0
#define CCM_OBS_CPU_POWER_MODE_3_0                                   280, 0
#define CCM_OBS_CPU_POWER_MODE_3_1                                   281, 0
#define CCM_OBS_CPU_MODE_TRANS_LPCG_DONE_3                           282, 0
#define CCM_OBS_CPU_MODE_TRANS_LPCG_REQUEST_3                        283, 0
#define CCM_OBS_CPU_MODE_TRANS_OSCPLL_DONE_3                         284, 0
#define CCM_OBS_CPU_MODE_TRANS_OSCPLL_REQUEST_3                      285, 0
#define CCM_OBS_SETPOINT_0                                           288, 0
#define CCM_OBS_SETPOINT_1                                           289, 0
#define CCM_OBS_SETPOINT_2                                           290, 0
#define CCM_OBS_SETPOINT_3                                           291, 0
#define CCM_OBS_SETPOINT_TRANS_LPCG_OFF_DONE                         292, 0
#define CCM_OBS_SETPOINT_TRANS_LPCG_OFF_REQUEST                      293, 0
#define CCM_OBS_SETPOINT_TRANS_LPCG_ON_DONE                          294, 0
#define CCM_OBS_SETPOINT_TRANS_LPCG_ON_REQUEST                       295, 0
#define CCM_OBS_STANDBY_TRANS_LPCG_IN_DONE                           296, 0
#define CCM_OBS_STANDBY_TRANS_LPCG_IN_REQUEST                        297, 0
#define CCM_OBS_STANDBY_TRANS_LPCG_OUT_DONE                          298, 0
#define CCM_OBS_STANDBY_TRANS_LPCG_OUT_REQUEST                       299, 0
#define CCM_OBS_SETPOINT_TRANS_GROUP_DOWN_DONE                       300, 0
#define CCM_OBS_SETPOINT_TRANS_GROUP_DOWN_REQUES                     301, 0
#define CCM_OBS_SETPOINT_TRANS_GROUP_UP_DONE                         302, 0
#define CCM_OBS_SETPOINT_TRANS_GROUP_UP_REQUEST                      303, 0
#define CCM_OBS_SETPOINT_TRANS_ROOT_DOWN_DONE                        304, 0
#define CCM_OBS_SETPOINT_TRANS_ROOT_DOWN_REQUEST                     305, 0
#define CCM_OBS_SETPOINT_TRANS_ROOT_UP_DONE                          306, 0
#define CCM_OBS_SETPOINT_TRANS_ROOT_UP_REQUEST                       307, 0
#define CCM_OBS_SETPOINT_TRANS_OSCPLL_OFF_DONE                       308, 0
#define CCM_OBS_SETPOINT_TRANS_OSCPLL_OFF_REQUES                     309, 0
#define CCM_OBS_SETPOINT_TRANS_OSCPLL_ON_DONE                        310, 0
#define CCM_OBS_SETPOINT_TRANS_OSCPLL_ON_REQUEST                     311, 0
#define CCM_OBS_STANDBY_TRANS_OSCPLL_IN_DONE                         312, 0
#define CCM_OBS_STANDBY_TRANS_OSCPLL_IN_REQUEST                      313, 0
#define CCM_OBS_STANDBY_TRANS_OSCPLL_OUT_DONE                        314, 0
#define CCM_OBS_STANDBY_TRANS_OSCPLL_OUT_REQUEST                     315, 0

#define CCM_OBS_DIV     3

/* Clock Source Definitions */
#define ROOT_CLOCK_SOURCES \
    {   /*SRC0,               SRC1,             SRC2,             SRC3,            SRC4,             SRC5,              SRC6,              SRC7,                      name             index */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_ArmPllOut, kCLOCK_SysPll1Out, kCLOCK_SysPll3Out, kCLOCK_VideoPllOut},     /* M7               0    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll2Out, kCLOCK_SysPll1Div5},   /* M4               1    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Out, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},   /* BUS              2    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll2Out, kCLOCK_SysPll1Div5},   /* BUS_LPSR         3    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd1, kCLOCK_SysPll3Pfd0},  /* SEMC             4    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* CSSYS            5    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd1, kCLOCK_SysPll2Out},  /* CSTRACE          6    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd0, kCLOCK_SysPll1Div5},  /* M4_SYSTICK       7    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Out, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd0},  /* M7_SYSTICK       8    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* ADC1             9    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* ADC2            10    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Out, kCLOCK_SysPll1Div5, kCLOCK_AudioPllOut, kCLOCK_SysPll2Pfd3},  /* ACMP            11    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* FLEXIO1         12    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* FLEXIO2         13    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll3Pfd3}, /* GPT1            14    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_AudioPllOut, kCLOCK_VideoPllOut}, /* GPT2            15    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_AudioPllOut, kCLOCK_VideoPllOut}, /* GPT3            16    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll3Pfd3}, /* GPT4            17    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll3Pfd3}, /* GPT5            18    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll3Pfd3}, /* GPT6            19    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd0, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll3Out},   /* FLEXSPI1        20    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd0, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll3Out},   /* FLEXSPI2        21    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* CAN1            22    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* CAN2            23    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd3, kCLOCK_SysPll1Div5},  /* CAN3            24    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART1         25    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART2         26    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART3         27    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART4         28    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART5         29    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART6         30    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART7         31    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART8         32    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART9         33    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPUART10        34    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd3, kCLOCK_SysPll1Div5},  /* LPUART11        35    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd3, kCLOCK_SysPll1Div5},  /* LPUART12        36    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPI2C1          37    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPI2C2          38    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPI2C3          39    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPI2C4          40    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd3, kCLOCK_SysPll1Div5},  /* LPI2C5          41    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd3, kCLOCK_SysPll1Div5},  /* LPI2C6          42    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPSPI1          43    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPSPI2          44    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPSPI3          45    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* LPSPI4          46    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5},  /* LPSPI5          47    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5},  /* LPSPI6          48    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* EMV1            49    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Div2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd3},  /* EMV2            50    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd1}, /* ENET1           51    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd1}, /* ENET2           52    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd1}, /* ENET_QOS        53    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd1}, /* ENET_25M        54    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd1}, /* ENET_TIMER1     55    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd1}, /* ENET_TIMER2     56    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd1}, /* ENET_TIMER3     57    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll2Pfd0, kCLOCK_SysPll1Div5, kCLOCK_ArmPllOut},   /* USDHC1          58    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll2Pfd0, kCLOCK_SysPll1Div5, kCLOCK_ArmPllOut},   /* USDHC2          59    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div5, kCLOCK_SysPll3Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll2Pfd3}, /* ASRC            60    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll1Div5, kCLOCK_SysPll3Div2, kCLOCK_AudioPllOut, kCLOCK_SysPll2Pfd3}, /* MQS             61    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5},  /* MIC             62    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_AudioPllOut, kCLOCK_SysPll3Out, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll2Pfd3},  /* SPDIF           63    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_AudioPllOut, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd3}, /* SAI1            64    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_AudioPllOut, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd3}, /* SAI2            65    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_AudioPllOut, kCLOCK_SysPll3Pfd2, kCLOCK_SysPll1Div5, kCLOCK_SysPll2Pfd3}, /* SAI3            66    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll3Pfd3, kCLOCK_SysPll3Out, kCLOCK_AudioPllOut, kCLOCK_SysPll1Div5},  /* SAI4            67    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd1, kCLOCK_SysPll3Out, kCLOCK_VideoPllOut},   /* GC355           68    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll3Pfd0, kCLOCK_VideoPllOut},  /* LCDIF           69    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll3Pfd0, kCLOCK_VideoPllOut},  /* LCDIFV2         70    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd0, kCLOCK_SysPll3Pfd0, kCLOCK_VideoPllOut},  /* MIPI_REF        71    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Out, kCLOCK_SysPll2Pfd0, kCLOCK_SysPll3Pfd0, kCLOCK_VideoPllOut},  /* MIPI_ESC        72    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd0, kCLOCK_VideoPllOut},  /* CSI2            73    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd0, kCLOCK_VideoPllOut},  /* CSI2_ESC        74    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll3Out, kCLOCK_SysPll2Pfd0, kCLOCK_VideoPllOut},  /* CSI2_UI         75    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll3Out, kCLOCK_SysPll3Pfd1, kCLOCK_VideoPllOut},  /* CSI             76    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Pfd2, kCLOCK_SysPll2Out, kCLOCK_SysPll3Pfd1, kCLOCK_SysPll1Div5},  /* CKO1            77    */ \
        {kCLOCK_OscRc48MDiv2, kCLOCK_Osc24MOut, kCLOCK_OscRc400M, kCLOCK_OscRc16M, kCLOCK_SysPll2Pfd3, kCLOCK_OscRc48M, kCLOCK_SysPll3Pfd1, kCLOCK_AudioPllOut}     /* CKO2            78    */ \
    }



/* Root clock index */
typedef enum _clock_root
{
    kCLOCK_Root_M7 = 0,
    kCLOCK_Root_M4 = 1,
    kCLOCK_Root_Bus = 2,
    kCLOCK_Root_Bus_Lpsr = 3,
    kCLOCK_Root_Semc = 4,
    kCLOCK_Root_Cssys = 5,
    kCLOCK_Root_Cstrace = 6,
    kCLOCK_Root_M4_Systick = 7,
    kCLOCK_Root_M7_Systick = 8,
    kCLOCK_Root_Adc1 = 9,
    kCLOCK_Root_Adc2 = 10,
    kCLOCK_Root_Acmp = 11,
    kCLOCK_Root_Flexio1 = 12,
    kCLOCK_Root_Flexio2 = 13,
    kCLOCK_Root_Gpt1 = 14,
    kCLOCK_Root_Gpt2 = 15,
    kCLOCK_Root_Gpt3 = 16,
    kCLOCK_Root_Gpt4 = 17,
    kCLOCK_Root_Gpt5 = 18,
    kCLOCK_Root_Gpt6 = 19,
    kCLOCK_Root_Flexspi1 = 20,
    kCLOCK_Root_Flexspi2 = 21,
    kCLOCK_Root_Can1 = 22,
    kCLOCK_Root_Can2 = 23,
    kCLOCK_Root_Can3 = 24,
    kCLOCK_Root_Lpuart1 = 25,
    kCLOCK_Root_Lpuart2 = 26,
    kCLOCK_Root_Lpuart3 = 27,
    kCLOCK_Root_Lpuart4 = 28,
    kCLOCK_Root_Lpuart5 = 29,
    kCLOCK_Root_Lpuart6 = 30,
    kCLOCK_Root_Lpuart7 = 31,
    kCLOCK_Root_Lpuart8 = 32,
    kCLOCK_Root_Lpuart9 = 33,
    kCLOCK_Root_Lpuart10 = 34,
    kCLOCK_Root_Lpuart11 = 35,
    kCLOCK_Root_Lpuart12 = 36,
    kCLOCK_Root_Lpi2c1 = 37,
    kCLOCK_Root_Lpi2c2 = 38,
    kCLOCK_Root_Lpi2c3 = 39,
    kCLOCK_Root_Lpi2c4 = 40,
    kCLOCK_Root_Lpi2c5 = 41,
    kCLOCK_Root_Lpi2c6 = 42,
    kCLOCK_Root_Lpspi1 = 43,
    kCLOCK_Root_Lpspi2 = 44,
    kCLOCK_Root_Lpspi3 = 45,
    kCLOCK_Root_Lpspi4 = 46,
    kCLOCK_Root_Lpspi5 = 47,
    kCLOCK_Root_Lpspi6 = 48,
    kCLOCK_Root_Emv1 = 49,
    kCLOCK_Root_Emv2 = 50,
    kCLOCK_Root_Enet1 = 51,
    kCLOCK_Root_Enet2 = 52,
    kCLOCK_Root_Enet_Qos = 53,
    kCLOCK_Root_Enet_25m = 54,
    kCLOCK_Root_Enet_Timer1 = 55,
    kCLOCK_Root_Enet_Timer2 = 56,
    kCLOCK_Root_Enet_Timer3 = 57,
    kCLOCK_Root_Usdhc1 = 58,
    kCLOCK_Root_Usdhc2 = 59,
    kCLOCK_Root_Asrc = 60,
    kCLOCK_Root_Mqs = 61,
    kCLOCK_Root_Mic = 62,
    kCLOCK_Root_Spdif = 63,
    kCLOCK_Root_Sai1 = 64,
    kCLOCK_Root_Sai2 = 65,
    kCLOCK_Root_Sai3 = 66,
    kCLOCK_Root_Sai4 = 67,
    kCLOCK_Root_Gc355 = 68,
    kCLOCK_Root_Lcdif = 69,
    kCLOCK_Root_Lcdifv2 = 70,
    kCLOCK_Root_Mipi_Ref = 71,
    kCLOCK_Root_Mipi_Esc = 72,
    kCLOCK_Root_Csi2 = 73,
    kCLOCK_Root_Csi2_Esc = 74,
    kCLOCK_Root_Csi2_Ui = 75,
    kCLOCK_Root_Csi = 76,
    kCLOCK_Root_Cko1 = 77,
    kCLOCK_Root_Cko2 = 78,
} clock_root_t;

typedef enum _clock_group
{
    kCLOCK_Group_FlexRAM = 0,
    kCLOCK_Group_MipiDsi = 1,
    kCLOCK_Group_Last,
} clock_group_t;


typedef struct _clock_group_config
{
    bool clockOff;  /*!< Turn off the clock. */
    uint16_t resetDiv; /*!< resetDiv + 1 should be common multiple of all dividers, valid range 0 ~ 255. */
    uint8_t div0 ; /*!< Divide root clock by div0 + 1, valid range: 0 ~ 15. */
    uint8_t div1 ; /*!< Divide root clock by div1 + 1, valid range: 0 ~ 15. */
    uint8_t div2 ; /*!< Divide root clock by div2 + 1, valid range: 0 ~ 15. */
    uint8_t div3 ; /*!< Divide root clock by div3 + 1, valid range: 0 ~ 15. */
    uint8_t div4 ; /*!< Divide root clock by div4 + 1, valid range: 0 ~ 15. */
    uint8_t div5 ; /*!< Divide root clock by div5 + 1, valid range: 0 ~ 15. */
    uint8_t div6 ; /*!< Divide root clock by div6 + 1, valid range: 0 ~ 15. */
    uint8_t div7 ; /*!< Divide root clock by div7 + 1, valid range: 0 ~ 15. */
    uint8_t div8 ; /*!< Divide root clock by div8 + 1, valid range: 0 ~ 15. */
    uint8_t div9 ; /*!< Divide root clock by div9 + 1, valid range: 0 ~ 15. */
    uint8_t div10; /*!< Divide root clock by div10 + 1, valid range: 0 ~ 15. */
    uint8_t div11; /*!< Divide root clock by div11 + 1, valid range: 0 ~ 15. */
} clock_group_config_t;

#define clock_ip_name_t clock_lpcg_t

#if (__CORTEX_M == 7)
#define CLOCK_GetCpuClkFreq CLOCK_GetM7Freq
#else
#define CLOCK_GetCpuClkFreq CLOCK_GetM4Freq
#endif

#define CLOCK_GetCoreSysClkFreq CLOCK_GetCpuClkFreq /*!< For compatible with other platforms without CCM. */

/* uncomment the following line if want to use OBS to retrieve frequency */
/* #define GET_FREQ_FROM_OBS */

/*! @brief OSC 24M sorce select */
typedef enum _clock_osc
{
    kCLOCK_RcOsc = 0U,   /*!< On chip OSC. */
    kCLOCK_XtalOsc = 1U, /*!< 24M Xtal OSC */
} clock_osc_t;

/*! @brief Clock gate value */
typedef enum _clock_gate_value
{
    kCLOCK_Off = (int)~CCM_LPCG_DIRECT_ON_MASK,     /*!< Clock is off. */
    kCLOCK_On = CCM_LPCG_DIRECT_ON_MASK,     /*!< Clock is on*/
} clock_gate_value_t;

/*! @brief System clock mode */
typedef enum _clock_mode_t
{
    kCLOCK_ModeRun = 0U,  /*!< Remain in run mode. */
    kCLOCK_ModeWait = 1U, /*!< Transfer to wait mode. */
    kCLOCK_ModeStop = 2U, /*!< Transfer to stop mode. */
} clock_mode_t;

/*! @brief USB clock source definition. */
typedef enum _clock_usb_src
{
    kCLOCK_Usb480M = 0,                     /*!< Use 480M.      */
    kCLOCK_UsbSrcUnused = (int)0xFFFFFFFFU, /*!< Used when the function does not
                                            care the clock source. */
} clock_usb_src_t;

/*! @brief Source of the USB HS PHY. */
typedef enum _clock_usb_phy_src
{
    kCLOCK_Usbphy480M = 0, /*!< Use 480M.      */
} clock_usb_phy_src_t;

/*!@brief PLL clock source, bypass cloco source also */
enum _clock_pll_clk_src
{
    kCLOCK_PllClkSrc24M = 0U, /*!< Pll clock source 24M */
    kCLOCK_PllSrcClkPN = 1U,  /*!< Pll clock source CLK1_P and CLK1_N */
};

typedef enum _clock_pll_post_div
{
    kCLOCK_PllPostDiv2 = 0U, /*!< Divide by 2. */
    kCLOCK_PllPostDiv4 = 1U, /*!< Divide by 4. */
    kCLOCK_PllPostDiv8 = 2U, /*!< Divide by 8. */
    kCLOCK_PllPostDiv1 = 3U, /*!< Divide by 1. */
} clock_pll_post_div_t;

/*!
 * @brief PLL configuration for ARM.
 *
 * The output clock frequency is:
 *
 *  Fout=Fin*loopDivider /(2 * postDivider).
 *
 * Fin is always 24MHz.
 */
typedef struct _clock_arm_pll_config
{
    clock_pll_post_div_t postDivider; /*!< Post divider. */
    uint32_t loopDivider; /*!< PLL loop divider. Valid range: 104-208. */
} clock_arm_pll_config_t;

/*! @brief PLL configuration for USB */
typedef struct _clock_usb_pll_config
{
    uint8_t loopDivider; /*!< PLL loop divider.
                              0 - Fout=Fref*20;
                              1 - Fout=Fref*22 */
    uint8_t src;         /*!< Pll clock source, reference _clock_pll_clk_src */

} clock_usb_pll_config_t;

typedef struct _clock_sys_pll3_config
{
    uint32_t divSelect;
} clock_sys_pll3_config_t;

/*! @brief PLL configuration for System */
typedef struct _clock_sys_pll_config
{
    uint8_t loopDivider;  /*!< PLL loop divider. Intended to be 1 (528M).
                               0 - Fout=Fref*20;
                               1 - Fout=Fref*22 */
    uint32_t mfn;   /*!< 30 bit mfn of fractional loop divider.*/
    uint32_t mfi; /*!< 30 bit of fractional loop divider */
    uint16_t ss_stop;     /*!< Stop value to get frequency change. */
    uint8_t ss_enable;    /*!< Enable spread spectrum modulation */
    uint16_t ss_step;     /*!< Step value to get frequency change step. */
} clock_sys_pll_config_t;

/*! @brief PLL configure for Sys Pll1 */
typedef struct _clock_sys_pll1_config
{
    bool pllDiv2En;  /*!< Enable Sys Pll1 divide-by-2 clock or not. */
    bool pllDiv5En;  /*!< Enable Sys Pll1 divide-by-5 clock or not. */
} clock_sys_pll1_config_t;

/*! @brief PLL configuration for AUDIO and VIDEO */
typedef struct _clock_audio_pll_config
{
    uint8_t loopDivider;  /*!< PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    uint8_t postDivider;  /*!< Divider after the PLL, should only be 1, 2, 4, 8, 16, 32. */
    uint32_t numerator;   /*!< 30 bit numerator of fractional loop divider.*/
    uint32_t denominator; /*!< 30 bit denominator of fractional loop divider */
} clock_av_pll_config_t, clock_audio_pll_config_t, clock_video_pll_config_t;

/*! @brief PLL configuration for ENET */
typedef struct _clock_enet_pll_config
{
    bool enableClkOutput;    /*!< Power on and enable PLL clock output for ENET0 (ref_enetpll0). */
    bool enableClkOutput25M; /*!< Power on and enable PLL clock output for ENET2 (ref_enetpll2). */
    uint8_t loopDivider;     /*!< Controls the frequency of the ENET0 reference clock.
                                  b00 25MHz
                                  b01 50MHz
                                  b10 100MHz (not 50% duty cycle)
                                  b11 125MHz */
    uint8_t src;             /*!< Pll clock source, reference _clock_pll_clk_src */
    bool enableClkOutput1;   /*!< Power on and enable PLL clock output for ENET1 (ref_enetpll1). */
    uint8_t loopDivider1;    /*!< Controls the frequency of the ENET1 reference clock.
                                  b00 25MHz
                                  b01 50MHz
                                  b10 100MHz (not 50% duty cycle)
                                  b11 125MHz */
} clock_enet_pll_config_t;

typedef struct _clock_root_config_t
{
    bool clockOff;
    uint8_t mfn;
    uint8_t mfd;
    uint8_t mux;
    uint8_t div;
} clock_root_config_t;

/*! @brief PLL name */
typedef enum _clock_pll
{
    kCLOCK_Pll_ArmPll,
    kCLOCK_Pll_SysPll1,
    kCLOCK_Pll_SysPll2,
    kCLOCK_Pll_SysPll3,
    kCLOCK_Pll_AudioPll,
    kCLOCK_Pll_VideoPll,
    kCLOCK_PllInvalid = -1,
} clock_pll_t;

#define PLL_PFD_COUNT 4
/*! @brief PLL PFD name */
typedef enum _clock_pfd
{
    kCLOCK_Pfd0 = 0U, /*!< PLL PFD0 */
    kCLOCK_Pfd1 = 1U, /*!< PLL PFD1 */
    kCLOCK_Pfd2 = 2U, /*!< PLL PFD2 */
    kCLOCK_Pfd3 = 3U, /*!< PLL PFD3 */
} clock_pfd_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Set CCM Root Clock MUX node to certain value.
 *
 * @param root Which root clock node to set, see \ref clock_root_t.
 * @param src Clock mux value to set, different mux has different value range.
 */
static inline void CLOCK_SetRootClockMux(clock_root_t root, uint8_t src)
{
    assert(src < 8);
    CCM->CLOCK_ROOT[root].CONTROL = src << CCM_CLOCK_ROOT_CONTROL_MUX_SHIFT;
}

/*!
 * @brief Get CCM Root Clock MUX value.
 *
 * @param mux   Which mux node to get, see \ref clock_mux_t.
 * @return Clock mux value.
 */
static inline uint32_t CLOCK_GetRootClockMux(clock_root_t root)
{
    return (CCM->CLOCK_ROOT[root].CONTROL & CCM_CLOCK_ROOT_CONTROL_MUX_MASK) >> CCM_CLOCK_ROOT_CONTROL_MUX_SHIFT;
}

/*!
 * @brief Get CCM Root Clock Source.
 *
 * @param root Which root clock node to get, see \ref clock_root_t.
 * @param src Clock mux value to get
 * @return Clock source
 */
static inline clock_name_t CLOCK_GetRootClockSource(clock_root_t root, uint32_t src)
{
    const clock_name_t source[][8] = ROOT_CLOCK_SOURCES;
    return source[root][src];
}

/*!
 * @brief Set CCM Root Clock DIV certain value.
 *
 * @param root Which root clock to set, see \ref clock_root_t.
 * @param div Clock div value to set, different divider has different value range.
 */
static inline void CLOCK_SetRootClockDiv(clock_root_t root, uint8_t div)
{
    assert(div);
    CCM->CLOCK_ROOT[root].CONTROL_SET = (div - 1) << CCM_CLOCK_ROOT_CONTROL_DIV_SHIFT;
}

/*!
 * @brief Get CCM DIV node value.
 *
 * @param root Which root clock node to get, see \ref clock_root_t.
 */
static inline uint32_t CLOCK_GetRootClockDiv(clock_root_t root)
{
    return (CCM->CLOCK_ROOT[root].CONTROL & CCM_CLOCK_ROOT_CONTROL_DIV_MASK) >> CCM_CLOCK_ROOT_CONTROL_DIV_SHIFT;
}

/*!
 * @brief Power Off Root Clock
 *
 * @param root Which root clock node to set, see \ref clock_root_t.
 */
static inline void CLOCK_PowerOffRootClock(clock_root_t root)
{
    if (!(CCM->CLOCK_ROOT[root].CONTROL & CCM_CLOCK_ROOT_CONTROL_OFF_MASK))
    {
        CCM->CLOCK_ROOT[root].CONTROL_SET = CCM_CLOCK_ROOT_CONTROL_OFF_MASK;
    }
}

/*!
 * @brief Power On Root Clock
 *
 * @param root Which root clock node to set, see \ref clock_root_t.
 */
static inline void CLOCK_PowerOnRootClock(clock_root_t root)
{
    CCM->CLOCK_ROOT[root].CONTROL_CLR = CCM_CLOCK_ROOT_CONTROL_OFF_MASK;
}

/*!
 * @brief Set MFN and MFD Root Clock
 *
 * @param root Which root clock node to set, see \ref clock_root_t.
 * @param mfn MFN value
 * @param mfd MFD value
 */
static inline void CLOCK_SetRootClockMFx(clock_root_t root, uint32_t mfn, uint32_t mfd)
{
    CCM->CLOCK_ROOT[root].CONTROL_SET = CCM_CLOCK_ROOT_CONTROL_MFN(mfn) | CCM_CLOCK_ROOT_CONTROL_MFD(mfd);
}

/*!
 * @brief Configure Root Clock
 *
 * @param root Which root clock node to set, see \ref clock_root_t.
 * @param config root clock config, see \ref clock_root_config_t
 */
static inline void CLOCK_SetRootClock(clock_root_t root, const clock_root_config_t *config)
{
    assert(config);
    CCM->CLOCK_ROOT[root].CONTROL = CCM_CLOCK_ROOT_CONTROL_MFN(config->mfn)
                                    | CCM_CLOCK_ROOT_CONTROL_MFD(config->mfd)
                                    | CCM_CLOCK_ROOT_CONTROL_MUX(config->mux)
                                    | CCM_CLOCK_ROOT_CONTROL_DIV(config->div)
                                    | (config->clockOff ? CCM_CLOCK_ROOT_CONTROL_OFF(config->clockOff) : 0);
}

/*!
 * @brief Control the clock gate for specific IP.
 *
 * @param name  Which clock to enable, see \ref clock_ip_name_t.
 * @param value Clock gate value to set, see \ref clock_gate_value_t.
 */
static inline void CLOCK_ControlGate(clock_ip_name_t name, clock_gate_value_t value)
{
    if ((value & CCM_LPCG_DIRECT_ON_MASK) != (CCM->LPCG[name].DIRECT & CCM_LPCG_DIRECT_ON_MASK))
    {
        CCM->LPCG[name].DIRECT = (value & CCM_LPCG_DIRECT_ON_MASK);
    }
}

/*!
 * @brief Enable the clock for specific IP.
 *
 * @param name  Which clock to enable, see \ref clock_ip_name_t.
 */
static inline void CLOCK_EnableClock(clock_ip_name_t name)
{
    CLOCK_ControlGate(name, kCLOCK_On);
}

/*!
 * @brief Disable the clock for specific IP.
 *
 * @param name  Which clock to disable, see \ref clock_ip_name_t.
 */
static inline void CLOCK_DisableClock(clock_ip_name_t name)
{
    CLOCK_ControlGate(name, kCLOCK_Off);
}

/*!
 * @brief Set the clock group configuration.
 *
 * @param group  Which group to configure, see \ref clock_group_t.
 * @param config Configuration to set.
 */
void CLOCK_SetGroupConfig(clock_group_t group, const clock_group_config_t *config);

/*!
 * @brief Setting the low power mode that system will enter on next assertion of dsm_request signal.
 *
 * @param mode  Which mode to enter, see \ref clock_mode_t.
 */
static inline void CLOCK_SetMode(clock_mode_t mode)
{
}

/*!
 * @brief Gets the clock frequency for a specific clock name.
 *
 * This function checks the current clock configurations and then calculates
 * the clock frequency for a specific clock name defined in clock_name_t.
 *
 * @param clockName Clock names defined in clock_name_t
 * @return Clock frequency value in hertz
 */
uint32_t CLOCK_GetFreq(clock_name_t name);

/*!
 * @brief Gets the clock frequency for a specific root clock name.
 *
 * This function checks the current clock configurations and then calculates
 * the clock frequency for a specific clock name defined in clock_root_t.
 *
 * @param clockName Clock names defined in clock_root_t
 * @return Clock frequency value in hertz
 */
static inline uint32_t CLOCK_GetRootClockFreq(clock_root_t root)
{
    uint32_t freq, mux;
    const clock_name_t source[][8] = ROOT_CLOCK_SOURCES;
    mux = CLOCK_GetRootClockMux(root);
    freq = CLOCK_GetFreq(source[root][mux]) / (CLOCK_GetRootClockDiv(root) + 1);
    assert(freq);
    return freq;
}

/*!
 * @brief Get the CCM CPU/core/system frequency.
 *
 * @return  Clock frequency; If the clock is invalid, returns 0.
 */
static inline uint32_t CLOCK_GetM7Freq(void)
{
    return CLOCK_GetRootClockFreq(kCLOCK_Root_M7);
}

/*!
 * @brief Get the CCM CPU/core/system frequency.
 *
 * @return  Clock frequency; If the clock is invalid, returns 0.
 */
static inline uint32_t CLOCK_GetM4Freq(void)
{
    return CLOCK_GetRootClockFreq(kCLOCK_Root_M4);
}

/*!
 * @name OSC operations
 * @{
 */

/*!
 * @brief Initialize the external 24MHz clock.
 *
 * This function supports two modes:
 * 1. Use external crystal oscillator.
 * 2. Bypass the external crystal oscillator, using input source clock directly.
 *
 * After this function, please call @ref CLOCK_SetXtal0Freq to inform clock driver
 * the external clock frequency.
 *
 * @param bypassXtalOsc Pass in true to bypass the external crystal oscillator.
 * @note This device does not support bypass external crystal oscillator, so
 * the input parameter should always be false.
 */
void CLOCK_InitExternalClk(bool bypassXtalOsc);

/*!
 * @brief Deinitialize the external 24MHz clock.
 *
 * This function disables the external 24MHz clock.
 *
 * After this function, please call @ref CLOCK_SetXtal0Freq to set external clock
 * frequency to 0.
 */
void CLOCK_DeinitExternalClk(void);

/*!
 * @brief Switch the OSC.
 *
 * This function switches the OSC source for SoC.
 *
 * @param osc   OSC source to switch to.
 */
void CLOCK_SwitchOsc(clock_osc_t osc);

/*!
 * @brief Gets the RTC clock frequency.
 *
 * @return  Clock frequency; If the clock is invalid, returns 0.
 */
static inline uint32_t CLOCK_GetRtcFreq(void)
{
    return 32768U;
}

/*!
 * @brief Initialize the RC oscillator 24MHz clock.
 */
void CLOCK_InitRcOsc24M(void);

/*!
 * @brief Power down the RCOSC 24M clock.
 */
void CLOCK_DeinitRcOsc24M(void);
/* @} */

/*! @brief Enable USB HS clock.
 *
 * This function only enables the access to USB HS prepheral, upper layer
 * should first call the @ref CLOCK_EnableUsbhs0PhyPllClock to enable the PHY
 * clock to use USB HS.
 *
 * @param src  USB HS does not care about the clock source, here must be @ref kCLOCK_UsbSrcUnused.
 * @param freq USB HS does not care about the clock source, so this parameter is ignored.
 * @retval true The clock is set successfully.
 * @retval false The clock source is invalid to get proper USB HS clock.
 */
bool CLOCK_EnableUsbhs0Clock(clock_usb_src_t src, uint32_t freq);

/*! @brief Enable USB HS clock.
 *
 * This function only enables the access to USB HS prepheral, upper layer
 * should first call the @ref CLOCK_EnableUsbhs0PhyPllClock to enable the PHY
 * clock to use USB HS.
 *
 * @param src  USB HS does not care about the clock source, here must be @ref kCLOCK_UsbSrcUnused.
 * @param freq USB HS does not care about the clock source, so this parameter is ignored.
 * @retval true The clock is set successfully.
 * @retval false The clock source is invalid to get proper USB HS clock.
 */
bool CLOCK_EnableUsbhs1Clock(clock_usb_src_t src, uint32_t freq);

/*! @brief Disable USB HS PHY PLL clock.
 *
 * This function disables USB HS PHY PLL clock.
 */
void CLOCK_DisableUsbhs1PhyPllClock(void);

/* @} */

/*!
 * @brief Initialize the ARM PLL.
 *
 * This function initialize the ARM PLL with specific settings
 *
 * @param config   configuration to set to PLL.
 */
void CLOCK_InitArmPll(const clock_arm_pll_config_t *config);

/*!
 * @brief De-initialize the ARM PLL.
 */
void CLOCK_DeinitArmPll(void);

/*!
 * @brief Initialize the System PLL1.
 *
 * This function initializes the System PLL1 with specific settings
 *
 * @param config Configuration to set to PLL1.
 */
void CLOCK_InitSysPll1(const clock_sys_pll1_config_t *config);

/*!
 * @brief De-initialize the System PLL1.
 */
void CLOCK_DeinitSysPll1(void);

/*!
 * @brief Initialize the System PLL2.
 *
 * This function initializes the System PLL2 with specific settings
 *
 * @param config Configuration to set to PLL2.
 */
void CLOCK_InitSysPll2(const clock_sys_pll_config_t *config);

/*!
 * @brief De-initialize the System PLL2.
 */
void CLOCK_DeinitSysPll2(void);

/*!
 * @brief Initialize the System PLL3.
 *
 * This function initializes the System PLL3 with specific settings
 *
 * @param config Configuration to set to PLL3.
 */
void CLOCK_InitSysPll3(const clock_sys_pll3_config_t *config);

/*!
 * @brief De-initialize the System PLL3.
 */
void CLOCK_DeinitSysPll3(void);

/*!
 * @brief Calculate corresponding config values per given frequency
 *
 * This function calculates config valudes per given frequency for Audio/Video
 * PLL.
 *
 * @param cfg pll config structure
 * @param freqInMhz target frequency
 */
status_t CLOCK_CalcAvPllFreq(clock_av_pll_config_t *cfg, uint32_t freqInMhz);

/*!
 * @brief Initializes the Audio PLL with Specific Frequency (in Mhz).
 *
 * This function initializes the Audio PLL with specific frequency
 *
 * @param freqInMhz target frequency
 */
status_t CLOCK_InitAudioPllWithFreq(uint32_t freqInMhz);

/*!
 * @brief Initializes the Audio PLL.
 *
 * This function initializes the Audio PLL with specific settings
 *
 * @param config Configuration to set to PLL.
 */
void CLOCK_InitAudioPll(const clock_audio_pll_config_t *config);

/*!
 * @brief De-initialize the Audio PLL.
 */
void CLOCK_DeinitAudioPll(void);

/*!
 * @brief Initializes the Video PLL with Specific Frequency (in Mhz).
 *
 * This function initializes the Video PLL with specific frequency
 *
 * @param freqInMhz target frequency
 */
status_t CLOCK_InitVideoPllWithFreq(uint32_t freqInMhz);

/*!
 * @brief Initialize the video PLL.
 *
 * This function configures the Video PLL with specific settings
 *
 * @param config   configuration to set to PLL.
 */
void CLOCK_InitVideoPll(const clock_video_pll_config_t *config);

/*!
 * @brief De-initialize the Video PLL.
 */
void CLOCK_DeinitVideoPll(void);

/*!
 * @brief Get current PLL output frequency.
 *
 * This function get current output frequency of specific PLL
 *
 * @param pll   pll name to get frequency.
 * @return The PLL output frequency in hertz.
 */
uint32_t CLOCK_GetPllFreq(clock_pll_t pll);

/*!
 * @brief Initialize PLL PFD.
 *
 * This function initializes the System PLL PFD. During new value setting,
 * the clock output is disabled to prevent glitch.
 *
 * @param pll Which PLL of targeting PFD to be operated.
 * @param pfd Which PFD clock to enable.
 * @param pfdFrac The PFD FRAC value.
 * @note It is recommended that PFD settings are kept between 12-35.
 */
void CLOCK_InitPfd(clock_pll_t pll, clock_pfd_t pfd, uint8_t frac);

/*!
 * @brief De-initialize the System PLL PFD.
 *
 * This function disables the System PLL PFD.
 *
 * @param pll Which PLL of targeting PFD to be operated.
 * @param pfd Which PFD clock to disable.
 */
void CLOCK_DeinitPfd(clock_pll_t pll, clock_pfd_t pfd);

/*!
 * @brief Get current PFD output frequency.
 *
 * This function get current output frequency of specific System PLL PFD
 *
 * @param pll Which PLL of targeting PFD to be operated.
 * @param pfd pfd name to get frequency.
 * @return The PFD output frequency in hertz.
 */
uint32_t CLOCK_GetPfdFreq(clock_pll_t pll, clock_pfd_t pfd);

uint32_t CLOCK_GetFreqFromObs(uint32_t obsSigIndex, uint32_t obsIndex);

/*! @brief Enable OSC 24Mhz
 *
 * This function enables OSC 24Mhz.
 */
void CLOCK_EnableOsc24M(void);

/*! @brief Enable PLL LDO
 *
 * This function enables PLL LDO.
 *
 */
void CLOCK_EnablePllLdo(void);

/*! @brief Enable OSC RC 400Mhz
 *
 * This function enables OSC RC 400Mhz.
 */
void CLOCK_EnableOscRc400M(void);

/*! @brief Enable USB HS clock.
 *
 * This function only enables the access to USB HS prepheral, upper layer
 * should first call the @ref CLOCK_EnableUsbhs0PhyPllClock to enable the PHY
 * clock to use USB HS.
 *
 * @param src  USB HS does not care about the clock source, here must be @ref kCLOCK_UsbSrcUnused.
 * @param freq USB HS does not care about the clock source, so this parameter is ignored.
 * @retval true The clock is set successfully.
 * @retval false The clock source is invalid to get proper USB HS clock.
 */
bool CLOCK_EnableUsbhs0Clock(clock_usb_src_t src, uint32_t freq);

/*! @brief Enable USB HS clock.
 *
 * This function only enables the access to USB HS prepheral, upper layer
 * should first call the @ref CLOCK_EnableUsbhs0PhyPllClock to enable the PHY
 * clock to use USB HS.
 *
 * @param src  USB HS does not care about the clock source, here must be @ref kCLOCK_UsbSrcUnused.
 * @param freq USB HS does not care about the clock source, so this parameter is ignored.
 * @retval true The clock is set successfully.
 * @retval false The clock source is invalid to get proper USB HS clock.
 */
bool CLOCK_EnableUsbhs1Clock(clock_usb_src_t src, uint32_t freq);
/*! @brief Enable USB HS PHY PLL clock.
 *
 * This function enables the internal 480MHz USB PHY PLL clock.
 *
 * @param src  USB HS PHY PLL clock source.
 * @param freq The frequency specified by src.
 * @retval true The clock is set successfully.
 * @retval false The clock source is invalid to get proper USB HS clock.
 */
bool CLOCK_EnableUsbhs0PhyPllClock(clock_usb_phy_src_t src, uint32_t freq);

/*! @brief Disable USB HS PHY PLL clock.
 *
 * This function disables USB HS PHY PLL clock.
 */
void CLOCK_DisableUsbhs0PhyPllClock(void);

/*! @brief Enable USB HS PHY PLL clock.
 *
 * This function enables the internal 480MHz USB PHY PLL clock.
 *
 * @param src  USB HS PHY PLL clock source.
 * @param freq The frequency specified by src.
 * @retval true The clock is set successfully.
 * @retval false The clock source is invalid to get proper USB HS clock.
 */
bool CLOCK_EnableUsbhs1PhyPllClock(clock_usb_phy_src_t src, uint32_t freq);

/*! @brief Disable USB HS PHY PLL clock.
 *
 * This function disables USB HS PHY PLL clock.
 */
void CLOCK_DisableUsbhs1PhyPllClock(void);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _FSL_CLOCK_H_ */
