/*
 * Copyright 2017 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

//#include "bl_context.h"
#include "bootloader_common.h"
#include "fsl_assert.h"
#include "fsl_device_registers.h"
//#include "fsl_lpuart.h"
#if BL_ENABLE_CRC_CHECK
#include "bl_app_crc_check.h"
#endif
#include "bl_flexspi.h"
#include "flexspi_nor_flash.h"
#include "fusemap.h"
//#include "peripherals_pinmux.h"
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef FREQ_396MHz
#define FREQ_396MHz (396000000U)
#endif
#ifndef FREQ_480MHz
#define FREQ_480MHz (480000000U)
#endif
#ifndef FREQ_528MHz
#define FREQ_528MHz (528000000U)
#endif
#ifndef FREQ_24MHz
#define FREQ_24MHz (24000000U)
#endif

/*====================== FLEXSPI1 IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPI1B_DQS_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_00
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_04
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_02
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_01
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_03
#define SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_00
#define SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_13

#define SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_14
#define SW_MUX_CTL_PAD_FLEXSPI1A_DQS_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_12
#define SW_MUX_CTL_PAD_FLEXSPI1A_SS0_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_06
#define SW_MUX_CTL_PAD_FLEXSPI1A_SS1_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_05
#define SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_10
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA0_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_09
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA1_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_07
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA2_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_08
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA3_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_11

#define SW_PAD_CTL_PAD_FLEXSPI1B_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_00
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_04
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_02
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_01
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_03
#define SW_PAD_CTL_PAD_FLEXSPI1B_SS0_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_00
#define SW_PAD_CTL_PAD_FLEXSPI1B_SCLK_IDX   kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_13

#define SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_14
#define SW_PAD_CTL_PAD_FLEXSPI1A_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_12
#define SW_PAD_CTL_PAD_FLEXSPI1A_SS0_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_06
#define SW_PAD_CTL_PAD_FLEXSPI1A_SS1_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_05
#define SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX   kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_10
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA0_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_09
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA1_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_07
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA2_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_08
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA3_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_11

#define SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX   kIOMUXC_FLEXSPI_DQS_FA_SELECT_INPUT
#define kFLEXSPI1A_DQS_SRC_GPIO_SD_14 (0u)
#define kFLEXSPI1A_DQS_SRC_GPIO_SD_12 (1u)

#define FLEXSPI1_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0)

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Actual R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Keeper
#define FLEXSPI_SW_PAD_CTL_VAL                                                                      \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(0) | IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Acutal R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Pull
// 100k ohm pull down resistor
#define FLEXSPI_DQS_SW_PAD_CTL_VAL                                                                  \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |   \
     IOMUXC_SW_PAD_CTL_PAD_HYS(1))

/*====================== FLEXSPI Reset IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPI_RESET_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_13
#define SW_PAD_CTL_PAD_FLEXSPI_RESET_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_13
#define FLEXSPI_RESET_PIN_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5)

#define FLEXSPI_RESET_PIN_GPIO              GPIO1
#define FLEXSPI_RESET_PIN_GPIO_SEL          (0) // 0 for GPIO1, 1 for GPIO2
#define FLEXSPI_RESET_PIN_INDEX             (13)

#define FLEXSPI_RESET_PIN_SW_PAD_CTRL_VAL                                                         \
    (IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(0) | \
     IOMUXC_SW_PAD_CTL_PAD_PUS(0))

// GPIO for reset signals
#define FLEXSPI1A_RESET_GPIO_MUX_VAL    IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5)
#define FLEXSPI1A_RESET_GPIO            GPIO2
#define FLEXSPI1A_RESET_GPIO_SS0_B_PIN  (6)
#define FLEXSPI1A_RESET_GPIO_SCLK_PIN   (10)
#define FLEXSPI1A_RESET_GPIO_DATA0_PIN  (9)

#define GPIO_SW_PAD_CTL_VAL                                                                        \
    IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(0) | \
        IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
// static bool is_flexspi_2nd_bootpin(void);

/*******************************************************************************
 * Codes
 ******************************************************************************/

#if BL_FEATURE_SEMC_NAND_MODULE || BL_FEATURE_SEMC_NOR_MODULE
//!@brief Configure IOMUX for SEMC Peripheral
void semc_iomux_config(semc_mem_config_t *config)
{
    uint32_t dataInoutPadCtlValue = SEMC_SW_PAD_CTL_VAL;
    uint32_t addrInputPadCtlValue = SEMC_SW_PAD_CTL_VAL;
    uint32_t rdyOutputPadCtlValue = SEMC_RDY_SW_PAD_CTL_VAL;
    uint32_t ctlInputPadCtlValue = SEMC_SW_PAD_CTL_VAL;
    uint8_t cePortOutputSelection;

    // Pinmux configuration for SEMC DA[15:0] Port (NOR)
    // Pinmux configuration for SEMC D[15:0] Port (NAND)
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA0_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA0_IDX] = dataInoutPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA1_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA1_IDX] = dataInoutPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA2_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA2_IDX] = dataInoutPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA3_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA3_IDX] = dataInoutPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA4_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA4_IDX] = dataInoutPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA5_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA5_IDX] = dataInoutPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA6_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA6_IDX] = dataInoutPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA7_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA7_IDX] = dataInoutPadCtlValue;
    if ((config->deviceMemType == kSemcDeviceMemType_NOR) ||
        ((config->deviceMemType == kSemcDeviceMemType_NAND) && (config->nandMemConfig.ioPortWidth == 16u)))
    {
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA8_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA8_IDX] = dataInoutPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA9_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA9_IDX] = dataInoutPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA10_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA10_IDX] = dataInoutPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA11_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA11_IDX] = dataInoutPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA12_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA12_IDX] = dataInoutPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA13_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA13_IDX] = dataInoutPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA14_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA14_IDX] = dataInoutPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DATA15_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DATA15_IDX] = dataInoutPadCtlValue;
    }

    // Pinmux configuration for SEMC WE,OE,ADV Port (NOR)
    // Pinmux configuration for SEMC WE,RE,ALE Port (NAND)
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR11_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR11_IDX] = ctlInputPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR12_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR12_IDX] = ctlInputPadCtlValue;
    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_BA1_IDX] = SEMC_MUX_VAL;
    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_BA1_IDX] = ctlInputPadCtlValue;

    // Configure DQS pad
    /*
    if (config->readStrobeMode == kSemcDqsMode_LoopbackFromDqsPad)
    {
        // SEMC_DQS
        uint32_t dqsPadCtlValue = SEMC_DQS_SW_PAD_CTL_VAL;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_DQS_IDX] = SEMC_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_DQS_IDX] = dqsPadCtlValue;
    }
    */

    if (config->deviceMemType == kSemcDeviceMemType_NOR)
    {
        // Pinmux configuration for SEMC A[23:16], WAIT Port (NOR)
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_BA0_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_BA0_IDX] = ctlInputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR0_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR0_IDX] = addrInputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR1_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR1_IDX] = addrInputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR2_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR2_IDX] = addrInputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR3_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR3_IDX] = addrInputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR4_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR4_IDX] = addrInputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR5_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR5_IDX] = addrInputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR6_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR6_IDX] = addrInputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR7_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR7_IDX] = addrInputPadCtlValue;

        cePortOutputSelection = config->norMemConfig.cePortOutputSelection;
    }
    else if (config->deviceMemType == kSemcDeviceMemType_NAND)
    {
        // Pinmux configuration for SEMC CLE,R/B Port (NAND)
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_RDY_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_RDY_IDX] = rdyOutputPadCtlValue;
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR9_IDX] = SEMC_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR9_IDX] = ctlInputPadCtlValue;

        cePortOutputSelection = config->nandMemConfig.cePortOutputSelection;
    }

    // Pinmux configuration for SEMC CE Port (NAND/NOR)
    switch (cePortOutputSelection)
    {
        case kSemcCeOutputSelection_MUX_CSX1:
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_CSX1_IDX] = SEMC_CSX123_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_CSX1_IDX] = ctlInputPadCtlValue;
            break;
        case kSemcCeOutputSelection_MUX_CSX2:
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_CSX2_IDX] = SEMC_CSX123_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_CSX2_IDX] = ctlInputPadCtlValue;
            break;
        case kSemcCeOutputSelection_MUX_CSX3:
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_CSX3_IDX] = SEMC_CSX123_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_CSX3_IDX] = ctlInputPadCtlValue;
            break;
        case kSemcCeOutputSelection_MUX_A8:
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_ADDR8_IDX] = SEMC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_ADDR8_IDX] = ctlInputPadCtlValue;
            break;
        case kSemcCeOutputSelection_MUX_RDY:
        case kSemcCeOutputSelection_MUX_CSX0:
        default:
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_SEMC_CSX0_IDX] = SEMC_CSX0_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_SEMC_CSX0_IDX] = ctlInputPadCtlValue;
            break;
    }
}
#endif // #if BL_FEATURE_SEMC_NAND_MODULE || BL_FEATURE_SEMC_NOR_MODULE

#if BL_FEATURE_SPI_NOR_EEPROM_MODULE
//!@brief Gate on the clock for the LPSPI peripheral
void spi_clock_gate_enable(uint32_t instance)
{
    switch (instance)
    {
        case 1:
            CCM->CCGR1 |= CCM_CCGR1_CG0_MASK;
            break;
        case 2:
            CCM->CCGR1 |= CCM_CCGR1_CG1_MASK;
            break;
        case 3:
            CCM->CCGR1 |= CCM_CCGR1_CG2_MASK;
            break;
        case 4:
            CCM->CCGR1 |= CCM_CCGR1_CG3_MASK;
            break;
        default:
            break;
    }
}

//!@brief Control pin for LPSPI Peripheral
void spi_pcs_pin_control(uint32_t instance, uint8_t pcsx, bool isSelected)
{
    uint32_t pcsPinNumber;
    uint32_t pcsDelay_us;
    GPIO_Type *gpio = NULL;
    switch (instance)
    {
        case 1:
            pcsPinNumber = LPSPI1_PCS_GPIO_NUM;
            gpio = LPSPI1_PCS_GPIO;
            break;
        case 2:
            pcsPinNumber = LPSPI2_PCS_GPIO_NUM;
            gpio = LPSPI2_PCS_GPIO;
            break;
        case 3:
            pcsPinNumber = LPSPI3_PCS_GPIO_NUM;
            gpio = LPSPI3_PCS_GPIO;
            break;
        case 4:
            pcsPinNumber = LPSPI4_PCS_GPIO_NUM;
            gpio = LPSPI4_PCS_GPIO;
            break;
        default:
            return;
    }

    if (isSelected)
    {
        /* Clear PCS pin output to logic 0 */
        gpio->DR &= ~(1u << pcsPinNumber);
        pcsDelay_us = kSpiEepromModuleTiming_MinPcsSetupHold_ns;
    }
    else
    {
        /* Set PCS pin output as logic 1 */
        gpio->DR |= (1u << pcsPinNumber);
        pcsDelay_us = kSpiEepromModuleTiming_MinPcsHigh_ns;
    }

    /* Since we manualy control the pcs for LPSPI data transfer, we should make sure
     *  the timing is aligned with A.C. characteristics of EEPROM device*/
    if (pcsDelay_us % 1000)
    {
        pcsDelay_us /= 1000;
    }
    else
    {
        pcsDelay_us = (pcsDelay_us / 1000) + 1;
    }
    microseconds_delay(pcsDelay_us);
}

//!@brief Configure clock for LPSPI peripheral
void spi_clock_config(uint32_t instance, spi_module_clock_freq_t freq)
{
    /* LPSPI Clk source is controlled by CCM->CBCMR, the clk source selected for LPSPI
     *  is USB PLL (480MHz), see hapi_clock_init() in hapi_irom_clock.c */
    uint32_t topClkFreq = 246860000;
    uint32_t podf;

    uint32_t cbcmr = CCM->CBCMR & (~CCM_CBCMR_LPSPI_PODF_MASK);
    podf = topClkFreq / freq;
    cbcmr |= CCM_CBCMR_LPSPI_PODF(podf - 1);
    CCM->CBCMR = cbcmr;
}

//!@brief Configure IOMUX for LPSPI Peripheral
void spi_iomux_config(spi_nor_eeprom_peripheral_config_t *config)
{
    switch (config->spiIndex)
    {
        case 1:
            /* LPSPI1_SCK*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI1_SCK_IDX] = LPSPI1_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI1_SCK_IDX] = LPSPI_SW_PAD_CTL_VAL;

            /* LPSPI1_SIN*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI1_SIN_IDX] = LPSPI1_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI1_SIN_IDX] = LPSPI_SW_PAD_CTL_VAL;
            /* For input pin, we must set corresponding SELECT_INPUT register */
            IOMUXC->SELECT_INPUT[SELECT_INPUT_LPSPI1_SDI_IDX] = LPSPI1_SDI_SELECT_INPUT_VAL;

            /* LPSPI1_SOUT*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI1_SOUT_IDX] = LPSPI1_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI1_SOUT_IDX] = LPSPI_SW_PAD_CTL_VAL;

            /* LPSPI1_PCS */
            /* Note: So far we cannot do separated send and recive operation in one active pcs period in LPSPI driver
             *  To achieve this goal, we should config PCSx pin as GPIO and control it manually. */
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI1_PCS0_IDX] = GPIO_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI1_PCS0_IDX] = GPIO_SW_PAD_CTL_VAL;

            /* Set PCS pin direction as general-purpose output */
            LPSPI1_PCS_GPIO->GDIR |= (1u << LPSPI1_PCS_GPIO_NUM);
            /* Set PCS pin output as logic 1 */
            LPSPI1_PCS_GPIO->DR |= (1u << LPSPI1_PCS_GPIO_NUM);

            break;

        case 2:
            /* LPSPI2_SCK*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI2_SCK_IDX] = LPSPI2_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI2_SCK_IDX] = LPSPI_SW_PAD_CTL_VAL;

            /* LPSPI2_SIN*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI2_SIN_IDX] = LPSPI2_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI2_SIN_IDX] = LPSPI_SW_PAD_CTL_VAL;
            /* For input pin, we must set corresponding SELECT_INPUT register */
            IOMUXC->SELECT_INPUT[SELECT_INPUT_LPSPI2_SDI_IDX] = LPSPI2_SDI_SELECT_INPUT_VAL;

            /* LPSPI2_SOUT*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI2_SOUT_IDX] = LPSPI2_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI2_SOUT_IDX] = LPSPI_SW_PAD_CTL_VAL;

            /* LPSPI2_PCS */
            /* Note: So far we cannot do separated send and recive operation in one active pcs period in LPSPI driver
             *  To achieve this goal, we should config PCSx pin as GPIO and control it manually. */
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI2_PCS0_IDX] = GPIO_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI2_PCS0_IDX] = GPIO_SW_PAD_CTL_VAL;

            /* Set PCS pin direction as general-purpose output */
            LPSPI2_PCS_GPIO->GDIR |= (1u << LPSPI2_PCS_GPIO_NUM);
            /* Set PCS pin output as logic 1 */
            LPSPI2_PCS_GPIO->DR |= (1u << LPSPI2_PCS_GPIO_NUM);

            break;

        case 3:
            /* LPSPI3_SCK*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI3_SCK_IDX] = LPSPI3_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI3_SCK_IDX] = LPSPI_SW_PAD_CTL_VAL;

            /* LPSPI3_SIN*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI3_SIN_IDX] = LPSPI3_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI3_SIN_IDX] = LPSPI_SW_PAD_CTL_VAL;
            /* For input pin, we must set corresponding SELECT_INPUT register */
            IOMUXC->SELECT_INPUT[SELECT_INPUT_LPSPI3_SDI_IDX] = LPSPI3_SDI_SELECT_INPUT_VAL;

            /* LPSPI3_SOUT*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI3_SOUT_IDX] = LPSPI3_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI3_SOUT_IDX] = LPSPI_SW_PAD_CTL_VAL;

            /* LPSPI3_PCS */
            /* Note: So far we cannot do separated send and recive operation in one active pcs period in LPSPI driver
             *  To achieve this goal, we should config PCSx pin as GPIO and control it manually. */
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI3_PCS0_IDX] = GPIO_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI3_PCS0_IDX] = GPIO_SW_PAD_CTL_VAL;

            /* Set PCS pin direction as general-purpose output */
            LPSPI3_PCS_GPIO->GDIR |= (1u << LPSPI3_PCS_GPIO_NUM);
            /* Set PCS pin output as logic 1 */
            LPSPI3_PCS_GPIO->DR |= (1u << LPSPI3_PCS_GPIO_NUM);

            break;

        case 4:
            /* LPSPI4_SCK*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI4_SCK_IDX] = LPSPI4_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI4_SCK_IDX] = LPSPI_SW_PAD_CTL_VAL;

            /* LPSPI4_SIN*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI4_SIN_IDX] = LPSPI4_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI4_SIN_IDX] = LPSPI_SW_PAD_CTL_VAL;
            /* For input pin, we must set corresponding SELECT_INPUT register */
            IOMUXC->SELECT_INPUT[SELECT_INPUT_LPSPI4_SDI_IDX] = LPSPI2_SDI_SELECT_INPUT_VAL;

            /* LPSPI4_SOUT*/
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI4_SOUT_IDX] = LPSPI4_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI4_SOUT_IDX] = LPSPI_SW_PAD_CTL_VAL;

            /* LPSPI4_PCS */
            /* Note: So far we cannot do separated send and recive operation in one active pcs period in LPSPI driver
             *  To achieve this goal, we should config PCSx pin as GPIO and control it manually. */
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_LPSPI4_PCS0_IDX] = GPIO_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_LPSPI4_PCS0_IDX] = GPIO_SW_PAD_CTL_VAL;

            /* Set PCS pin direction as general-purpose output */
            LPSPI4_PCS_GPIO->GDIR |= (1u << LPSPI4_PCS_GPIO_NUM);
            /* Set PCS pin output as logic 1 */
            LPSPI4_PCS_GPIO->DR |= (1u << LPSPI4_PCS_GPIO_NUM);

            break;
        default:
            break;
    }
}
#endif // BL_FEATURE_SPI_NOR_EEPROM_MODULE

//! @brief Return uart clock frequency according to instance
uint32_t get_uart_clock(uint32_t instance)
{
    // LPUART1 clock has been configured to 20MHz in clock_configure
    uint32_t lpuart_clock = 20000000UL;

    return lpuart_clock;
}

/*
uint32_t read_autobaud_pin( uint32_t instance )
{
    switch(instance)
    {
        case 0:
            return (GPIO_RD_PDIR(GPIOB) >> UART0_RX_GPIO_PIN_NUM) & 1;
        case 1:
            return (GPIO_RD_PDIR(GPIOC) >> UART1_RX_GPIO_PIN_NUM) & 1;
        case 2:
            return (GPIO_RD_PDIR(GPIOD) >> UART2_RX_GPIO_PIN_NUM) & 1;
        default:
            return 0;
    }
}
*/
bool is_boot_pin_asserted(void)
{
    // Boot pin for Flash only target
    return false;
}

//!@brief Get Primary boot device type
uint32_t get_primary_boot_device(void)
{
#define MEMORY_DEV_HIGH_MASK 0xC0
#define MEMORY_DEV_HIGH_SHIFT 0x06
#define MEMORY_DEV_LOW_MASK 0x30
#define MEMORY_DEV_LOW_SHIFT 0x04

    uint8_t flash_device = 0xFF;

    uint32_t high_value = (SRC->SBMR1 & MEMORY_DEV_HIGH_MASK) >> MEMORY_DEV_HIGH_SHIFT;
    uint32_t low_value = (SRC->SBMR1 & MEMORY_DEV_LOW_MASK) >> MEMORY_DEV_LOW_SHIFT;

    switch (high_value)
    {
        case 0:
            switch (low_value)
            {
                case 0:
                    flash_device = kBootDevice_FlexSpiNOR; // FlexSPI NOR
                    break;
                case 1:
                    flash_device = kBootDevice_SemcNOR; // SEMC NOR
                    break;
                default:
                    flash_device = kBootDevice_SemcNAND; // SEMC NAND
                    break;
            }
            break;
        case 1:
            flash_device = kBootDevice_MMC_SD; // SD
            break;
        case 2:
            flash_device = kBootDevice_MMC_SD; // MMC/eMMC
            break;
        case 3:
            flash_device = kBootDevice_FlexSpiNAND; // FlexSPI NAND
            break;
    }

    return flash_device;
}

#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
// Set failsafe settings
status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
// This is an example that shows how to override the default pad setting in ROM, for now, the pad setting in ROM is
// idential to below values
// So, below codes are not required.
#if 0
        // See IOMUXC pad setting definitions for more details.
        config->controllerMiscOption |= (1<<kFlexSpiMiscOffset_PadSettingOverrideEnable);
        config->dqsPadSettingOverride = 0x130f1;
        config->sclkPadSettingOverride = 0x10f1;
        config->csPadSettingOverride = 0x10f1;
        config->dataPadSettingOverride = 0x10f1;
#endif
        if (config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad)
        {
            if (config->controllerMiscOption & (1 << kFlexSpiMiscOffset_DdrModeEnable))
            {
                config->dataValidTime[0].time_100ps = 15; // 1.5 ns // 1/4 * cycle of 166MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 30; // 3 ns // 1/2 * cycle of 166MHz DDR
                }
            }
        }
        status = kStatus_Success;

    } while (0);

    return status;
}
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE

void dummy_byte_callback(uint8_t byte)
{
    (void)byte;
}

void debug_init(void) {}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__

void update_available_peripherals() {}

void init_hardware(void)
{
    CLOCK_EnableClock(kCLOCK_UsbOh3);
}

void deinit_hardware(void)
{
    CLOCK_DisableClock(kCLOCK_UsbOh3);
}

//!@brief Write FlexSPI persistent content
status_t flexspi_nor_write_persistent(const uint32_t data)
{
    SRC->GPR[2] = data;

    return kStatus_Success;
}
//!@brief Read FlexSPI persistent content
status_t flexspi_nor_read_persistent(uint32_t *data)
{
    *data = SRC->GPR[2];

    return kStatus_Success;
}

//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue = config->csPadSettingOverride ? config->csPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue = config->dqsPadSettingOverride ? config->dqsPadSettingOverride : FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = config->sclkPadSettingOverride ? config->sclkPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = config->dataPadSettingOverride ? config->dataPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;

//    TRACE("BootROM: flexspi_iomux_config(), instance=%x, csPadCtrl=%x, dqsPadCtrl=%x, sclkPadCtl=%x, dataPadCtl=%x\n", instance, csPadCtlValue,
//          dqsPadCtlValue, sclkPadCtlValue, dataPadCtlValue);

    if (instance == 0)
    {
        // The primary FlexSPI pinmux, support octal Flash and up to 4 QuadSPI NOR Flash
        {
            // Pinmux configuration for FLEXSPI1 PortA
            if (config->sflashA1Size || config->sflashA2Size)
            {
                if (config->sflashA2Size)
                {
                    // FLEXSPI1A_SS1_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SS1_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SS1_B_IDX] = csPadCtlValue;
                }

                // Basic pinmux configuration for FLEXSPI1
                if (config->sflashA1Size)
                {
                    // FLEXSPI1A_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SS0_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPI1A_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX] = \
                    FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPI1A_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA3_IDX] = dataPadCtlValue;

                if (config->sflashPadType == kSerialFlash_8Pads)
                {
                    // FLEXSPI1A_DATA4 / FLEXSPI1B_DATA0
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA5 / FLEXSPI1B_DATA1
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA6 / FLEXSPI1B_DATA2
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA7 / FLEXSPI1B_DATA3
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX] = dataPadCtlValue;
                }

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondDqsPinMux))
                    {
                        // FLEXSPI1A_SEC_DQS
                        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX] = \
                            FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX] = dqsPadCtlValue;
                        IOMUXC->SELECT_INPUT[SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX] = kFLEXSPI1A_DQS_SRC_GPIO_SD_14;
                    }
                    else
                    {
                        // FLEXSPI1A_DQS
                        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DQS_IDX] = \
                            FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DQS_IDX] = dqsPadCtlValue;
                        IOMUXC->SELECT_INPUT[SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX] = kFLEXSPI1A_DQS_SRC_GPIO_SD_12;
                    }
                }

                // Configure Differential Clock pin
                if (flexspi_is_differential_clock_enable(config))
                {
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX] = sclkPadCtlValue;
                }
            }

            // Pinmux configuration for FLEXSPI1 PortB
            if (config->sflashB1Size || config->sflashB2Size)
            {
                // Basic pinmux configuration for FLEXSPI1
                if (config->sflashB1Size)
                {
                    // FLEXSPI1B_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPI1B_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX] = \
                    FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPI1B_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX] = dataPadCtlValue;

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    // FLEXSPI1B_DQS
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DQS_IDX] =
                        FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DQS_IDX] = dqsPadCtlValue;
                }
            }
        }
    }
}

//!@brief Get the hab status.
//habstatus_option_t get_hab_status()
//{
//    if (ROM_OCOTP_SEC_CONFIG_VALUE() & 0x2)
//    {
//        return kHabStatus_Close;
//    }
//    else
//    {
//        return kHabStatus_Open;
//    }
//}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
