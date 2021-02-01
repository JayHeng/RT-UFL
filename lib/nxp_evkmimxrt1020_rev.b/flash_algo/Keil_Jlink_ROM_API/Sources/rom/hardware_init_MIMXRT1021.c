/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

//#include "bl_context.h"
#if BL_FEATURE_RELIABLE_UPDATE
#include "bl_reliable_update.h"
#endif
#include "bootloader_common.h"
#include "fsl_assert.h"
#include "fsl_device_registers.h"
//#include "fsl_lpuart.h"
#if BL_ENABLE_CRC_CHECK
#include "bl_app_crc_check.h"
#endif
#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
#include "bl_flexspi.h"
#include "flexspi_nor_flash.h"
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
#if BL_FEATURE_SPI_NOR_EEPROM_MODULE
#include "microseconds.h"
#include "spi_nor_eeprom_memory.h"
#endif // BL_FEATURE_SPI_NOR_EEPROM_MODULE
#if BL_FEATURE_SEMC_NAND_MODULE || BL_FEATURE_SEMC_NOR_MODULE
#include "bl_semc.h"
#endif // #if BL_FEATURE_SEMC_NAND_MODULE || BL_FEATURE_SEMC_NOR_MODULE
#include "bl_api.h"
#include "fusemap.h"
//#include "peripherals_pinmux.h"

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

/*====================== FLEXSPI IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPIB_DQS_IDX            79
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX          81
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX          84
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX          85
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX          83
#define SW_MUX_CTL_PAD_FLEXSPIB_SS0_B_IDX          78
#define SW_MUX_CTL_PAD_FLEXSPIB_SS1_B_IDX          75
#define SW_MUX_CTL_PAD_FLEXSPIB_SCLK_IDX           82

#define SW_MUX_CTL_PAD_FLEXSPIA_DQS_IDX            86
#define SW_MUX_CTL_PAD_FLEXSPIA_SS0_B_IDX          92
#define SW_MUX_CTL_PAD_FLEXSPIA_SS1_B_IDX          74
#define SW_MUX_CTL_PAD_FLEXSPIA_SCLK_IDX           88
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA0_IDX          89
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA1_IDX          91
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA2_IDX          90
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA3_IDX          87
#define SW_MUX_CTL_PAD_FLEXSPIA_SCLK_B_IDX         82

#define SW_PAD_CTL_PAD_FLEXSPIB_DQS_IDX            79
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX          81
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX          84
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX          85
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX          83
#define SW_PAD_CTL_PAD_FLEXSPIB_SS0_B_IDX          78
#define SW_PAD_CTL_PAD_FLEXSPIB_SS1_B_IDX          75
#define SW_PAD_CTL_PAD_FLEXSPIB_SCLK_IDX           82

#define SW_PAD_CTL_PAD_FLEXSPIA_DQS_IDX            86
#define SW_PAD_CTL_PAD_FLEXSPIA_SS0_B_IDX          92
#define SW_PAD_CTL_PAD_FLEXSPIA_SS1_B_IDX          74
#define SW_PAD_CTL_PAD_FLEXSPIA_SCLK_IDX           88
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA0_IDX          89
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA1_IDX          91
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA2_IDX          90
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA3_IDX          87
#define SW_PAD_CTL_PAD_FLEXSPIA_SCLK_B_IDX         82

#define FLEXSPIA_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIB_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIA_SS1_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS1_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS0_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_DQS_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Actual R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Keeper
#define FLEXSPI_SW_PAD_CTL_VAL    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) |   \
                                     IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | \
                                     IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))


// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Acutal R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Pull
// 100k ohm pull down resistor
#define FLEXSPI_DQS_SW_PAD_CTL_VAL    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) |   \
                                     IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | \
                                     IOMUXC_SW_PAD_CTL_PAD_PUE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |\
                                     IOMUXC_SW_PAD_CTL_PAD_HYS(1) )


/*====================== FLEXSPI Secondary IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX          63
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX           59
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX          60
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX          62
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX          61
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX          58

#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX          63
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX           59
#define SELECT_INPUT_FLEXSPIA_SEC_SCLK_IDX             31
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX          60
#define SELECT_INPUT_FLEXSPIA_SEC_DATA0_IDX            27
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX          62
#define SELECT_INPUT_FLEXSPIA_SEC_DATA1_IDX            28
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX          61
#define SELECT_INPUT_FLEXSPIA_SEC_DATA2_IDX            29
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX          58
#define SELECT_INPUT_FLEXSPIA_SEC_DATA3_IDX            30

#define FLEXSPIA_SEC_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)


/*====================== FLEXSPI Reset IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPI_RESET_IDX              71
#define SW_PAD_CTL_PAD_FLEXSPI_RESET_IDX              71
#define FLEXSPI_RESET_PIN_MUX                         IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5)
#define FLEXSPI_RESET_PIN_SW_PAD_CTRL_VAL             (IOMUXC_SW_PAD_CTL_PAD_DSE(6) |   \
                                                       IOMUXC_SW_PAD_CTL_PAD_PKE(1) | \
                                                       IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))
#define FLEXSPI_RESET_PIN_GPIO                        GPIO1
#define FLEXSPI_RESET_PIN_INDEX                       29
/*************************************
 *  IVT Data
 *************************************/
typedef struct _ivt_
{
    /** @ref hdr with tag #HAB_TAG_IVT, length and HAB version fields
     *  (see @ref data)
     */
    uint32_t hdr;
    /** Absolute address of the first instruction to execute from the
     *  image
     */
    uint32_t entry;
    /** Reserved in this version of HAB: should be NULL. */
    uint32_t reserved1;
    /** Absolute address of the image DCD: may be NULL. */
    uint32_t dcd;
    /** Absolute address of the Boot Data: may be NULL, but not interpreted
     *  any further by HAB
     */
    uint32_t boot_data;
    /** Absolute address of the IVT.*/
    uint32_t self;
    /** Absolute address of the image CSF.*/
    uint32_t csf;
    /** Reserved in this version of HAB: should be zero. */
    uint32_t reserved2;
} ivt;

#define IVT_MAJOR_VERSION 0x4
#define IVT_MAJOR_VERSION_SHIFT 0x4
#define IVT_MAJOR_VERSION_MASK 0xF
#define IVT_MINOR_VERSION 0x1
#define IVT_MINOR_VERSION_SHIFT 0x0
#define IVT_MINOR_VERSION_MASK 0xF

#define IVT_VERSION(major, minor)                                    \
    ((((major)&IVT_MAJOR_VERSION_MASK) << IVT_MAJOR_VERSION_SHIFT) | \
     (((minor)&IVT_MINOR_VERSION_MASK) << IVT_MINOR_VERSION_SHIFT))

/* IVT header */
#define IVT_TAG_HEADER 0xD1 /**< Image Vector Table */
#define IVT_SIZE 0x2000
#define IVT_PAR IVT_VERSION(IVT_MAJOR_VERSION, IVT_MINOR_VERSION)
#define IVT_HEADER (IVT_TAG_HEADER | (IVT_SIZE << 8) | (IVT_PAR << 24))

/* Set resume entry */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern uint32_t __Vectors[];
extern uint32_t Image$$RW_m_config_text$$Base[];
#define IMAGE_ENTRY_ADDRESS ((uint32_t)__Vectors)
#define FLASH_BASE ((uint32_t)Image$$RW_m_config_text$$Base)
#elif defined(__ICCARM__)
extern uint32_t __VECTOR_TABLE[];
extern uint32_t m_boot_hdr_conf_start[];
#define IMAGE_ENTRY_ADDRESS ((uint32_t)__VECTOR_TABLE)
#define FLASH_BASE ((uint32_t)m_boot_hdr_conf_start)
#elif defined(__GNUC__)
extern uint32_t __VECTOR_TABLE[];
extern uint32_t __FLASH_BASE[];
#define IMAGE_ENTRY_ADDRESS ((uint32_t)__VECTOR_TABLE)
#define FLASH_BASE ((uint32_t)__FLASH_BASE)
#endif

#define DCD_ADDRESS dcd_data
#define BOOT_DATA_ADDRESS &boot_data
#define CSF_ADDRESS 0
#define IVT_RSVD (uint32_t)(0x00000000)

/*************************************
 *  Boot Data
 *************************************/
typedef struct _boot_data_
{
    uint32_t start;       /* boot start location */
    uint32_t size;        /* size */
    uint32_t plugin;      /* plugin flag - 1 if downloaded application is plugin */
    uint32_t placeholder; /* placehoder to make even 0x10 size */
} BOOT_DATA_T;

#define FLASH_SIZE BL_FEATURE_FLASH_SIZE
#define PLUGIN_FLAG (uint32_t)0

/* External Variables */
const BOOT_DATA_T boot_data;

#if defined(XIP_BOOT_HEADER_ENABLE) && (XIP_BOOT_HEADER_ENABLE == 1)
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section(".boot_hdr.ivt")))
#elif defined(__ICCARM__)
#pragma location = ".boot_hdr.ivt"
#endif
/*************************************
 *  IVT Data
 *************************************/
const ivt image_vector_table = {
    IVT_HEADER,                    /* IVT Header */
    IMAGE_ENTRY_ADDRESS,           /* Image Entry Function */
    IVT_RSVD,                      /* Reserved = 0 */
    (uint32_t)0,                   /* Address where DCD information is stored */
    (uint32_t)BOOT_DATA_ADDRESS,   /* Address where BOOT Data Structure is stored */
    (uint32_t)&image_vector_table, /* Pointer to IVT Self (absolute address */
    (uint32_t)CSF_ADDRESS,         /* Address where CSF file is stored */
    IVT_RSVD                       /* Reserved = 0 */
};

#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section(".boot_hdr.boot_data")))
#elif defined(__ICCARM__)
#pragma location = ".boot_hdr.boot_data"
#endif
/*************************************
 *  Boot Data
 *************************************/
const BOOT_DATA_T boot_data = {
    FLASH_BASE,  /* boot start location */
    FLASH_SIZE,  /* size */
    PLUGIN_FLAG, /* Plugin flag*/
    0xFFFFFFFF   /* empty - extra data word */
};

#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section(".boot_hdr.conf")))
#elif defined(__ICCARM__)
#pragma location = ".boot_hdr.conf"
#endif
const flexspi_nor_config_t qspiflash_config = {
    .memConfig =
        {
            .tag              = FLEXSPI_CFG_BLK_TAG,
            .version          = FLEXSPI_CFG_BLK_VERSION,
            .readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackFromDqsPad,
            .csHoldTime       = 3u,
            .csSetupTime      = 3u,
            // Enable DDR mode, Wordaddassable, Safe configuration, Differential clock
            .sflashPadType = kSerialFlash_4Pads,
            .serialClkFreq = kFlexSpiSerialClk_133MHz,
            .sflashA1Size  = 8u * 1024u * 1024u,
            .deviceType = kFlexSpiDeviceType_SerialNOR,
            .controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable),
            .lookupTable =
                {
                    // Read LUTs
                    [4 * NOR_CMD_LUT_SEQ_IDX_READ] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
                    [4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04),

                    // Read Status LUTs
                    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04),

                    // Write Enable LUTs
                    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0x00),

                    // Erase Sector LUTs
                    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x18),

                    // Page Program LUTs
                    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x02, RADDR_SDR, FLEXSPI_1PAD, 0x18),
                    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0x00),

                    // Chip Erase LUTs
                    [4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP, FLEXSPI_1PAD, 0x00),

                    // Block Erase LUTs
                    [4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xD8, RADDR_SDR, FLEXSPI_1PAD, 0x18),
                },
        },
    .pageSize           = 256u,
    .sectorSize         = 4u * 1024u,
    .blockSize          = 64u * 1024u,
    .isUniformBlockSize = false,
    .ipcmdSerialClkFreq = kFlexSpiSerialClk_30MHz,
};
#endif

/*******************************************************************************
 * Prototype
 ******************************************************************************/
bool is_flexspi_2nd_bootpin(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if BL_TARGET_FLASH
__ALIGNED(0x400) static uint32_t s_ramVectorTable[0x100];
#endif // BL_TARGET_FLASH

/*******************************************************************************
 * Codes
 ******************************************************************************/

#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE

bool is_flexspi_2nd_bootpin(void)
{
    bool is_2nd_bootpin_selected = false;
    if ((ROM_OCOTP_FLASH_TYPE_VALUE() == 0x07) ||
        (ROM_OCOTP_QSPI_SIP_VALUE() && ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_VALUE()))
    {
        is_2nd_bootpin_selected = true;
    }
    else
    {
        is_2nd_bootpin_selected = false;
    }

    return is_2nd_bootpin_selected;
}

//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue = FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;

    if (flexspi_is_padsetting_override_enable(config))
    {
        csPadCtlValue = config->csPadSettingOverride;
        dqsPadCtlValue = config->dqsPadSettingOverride;
        sclkPadCtlValue = config->sclkPadSettingOverride;
        dataPadCtlValue = config->dataPadSettingOverride;
    }

    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondPinMux))
    {
        // The secondary FlexSPI Pinmux, supports only 1 Flash
        if (config->sflashA1Size > 0)
        {
            // FLEXSPIA_SS0_B
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX] = csPadCtlValue;
            // FLEXSPIA_SCLK
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX] =
                FLEXSPIA_SEC_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX] = sclkPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_SCLK_IDX] = 0x01;

            // FLEXSPIA_DATA0
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA0_IDX] = 0x01;

            // FLEXSPIA_DATA1
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA1_IDX] = 0x01;

            // FLEXSPIA_DATA2
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA2_IDX] = 0x01;

            // FLEXSPIA_DATA3
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA3_IDX] = 0x01;
        }
    }
    else // The primary FlexSPI pinmux, support octal Flash and up to 4 QuadSPI NOR Flash
    {
        // Pinmux configuration for FLEXSPI PortA
        if (config->sflashA1Size || config->sflashA2Size)
        {
            if (config->sflashA2Size)
            {
                // FLEXSPIA_SS1_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SS1_B_IDX] = FLEXSPIA_SS1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SS1_B_IDX] = csPadCtlValue;
            }

            // Basic pinmux configuration for FLEXSPI
            if (config->sflashA1Size)
            {
                // FLEXSPIA_SS0_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SS0_B_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SS0_B_IDX] = csPadCtlValue;
            }

            // FLEXSPIA_SCLK
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SCLK_IDX] = FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SCLK_IDX] = sclkPadCtlValue;

            // FLEXSPIA_DATA0
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA0_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA0_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA1
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA1_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA1_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA2
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA2_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA2_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA3
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA3_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA3_IDX] = dataPadCtlValue;

            if (config->sflashPadType == kSerialFlash_8Pads)
            {
                // FLEXSPIA_DATA4 / FLEXSPIB_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA5 / FLEXSPIB_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA6 / FLEXSPIB_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA7 / FLEXSPIB_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX] = dataPadCtlValue;
            }

            // Configure DQS pad
            if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
            {
                // FLEXSPIA_DQS
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DQS_IDX] =
                    FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DQS_IDX] = dqsPadCtlValue;
            }

            // Configure Differential Clock pin
            if (flexspi_is_differential_clock_enable(config))
            {
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SCLK_B_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SCLK_B_IDX] = sclkPadCtlValue;
            }
        }

        // Pinmux configuration for FLEXSPI PortB
        if (config->sflashB1Size || config->sflashB2Size)
        {
            if (config->sflashB2Size)
            {
                // FLEXSPIB_SS1_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SS1_B_IDX] = FLEXSPIB_SS1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SS1_B_IDX] = csPadCtlValue;
            }

            // Basic pinmux configuration for FLEXSPI
            if (config->sflashB1Size)
            {
                // FLEXSPIB_SS0_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SS0_B_IDX] = FLEXSPIB_SS0_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SS0_B_IDX] = csPadCtlValue;
            }

            // FLEXSPIB_SCLK
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SCLK_IDX] = FLEXSPIB_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SCLK_IDX] = sclkPadCtlValue;

            // FLEXSPIB_DATA0
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX] = dataPadCtlValue;

            // FLEXSPIB_DATA1
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX] = dataPadCtlValue;

            // FLEXSPIB_DATA2
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX] = dataPadCtlValue;

            // FLEXSPIB_DATA3
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX] = dataPadCtlValue;

            // Configure DQS pad
            if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
            {
                // FLEXSPIB_DQS
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DQS_IDX] =
                    FLEXSPIB_DQS_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DQS_IDX] = dqsPadCtlValue;
            }
        }
    }
}
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE

#if BL_FEATURE_SEMC_NAND_MODULE || BL_FEATURE_SEMC_NOR_MODULE
//!@brief Configure IOMUX for SEMC Peripheral
void semc_iomux_config(semc_mem_config_t *config)
{
    uint32_t dataInoutPadCtlValue = SEMC_SW_PAD_CTL_VAL;
    uint32_t addrInputPadCtlValue = SEMC_SW_PAD_CTL_VAL;
    uint32_t rdyOutputPadCtlValue = SEMC_RDY_SW_PAD_CTL_VAL;
    uint32_t ctlInputPadCtlValue = SEMC_SW_PAD_CTL_VAL;
    uint8_t cePortOutputSelection = config->nandMemConfig.cePortOutputSelection;

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
#if defined(SELECT_INPUT_LPSPI3_SDI_IDX)
            /* For input pin, we must set corresponding SELECT_INPUT register */
            IOMUXC->SELECT_INPUT[SELECT_INPUT_LPSPI3_SDI_IDX] = LPSPI3_SDI_SELECT_INPUT_VAL;
#endif
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
            IOMUXC->SELECT_INPUT[SELECT_INPUT_LPSPI4_SDI_IDX] = LPSPI4_SDI_SELECT_INPUT_VAL;

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
#if BL_TARGET_FLASH
    // Set the WAKEUP to GPIO5_IO00
    IOMUXC_SNVS->SW_MUX_CTL_PAD_WAKEUP = IOMUXC_SNVS_SW_MUX_CTL_PAD_WAKEUP_MUX_MODE(5);
    // Enable the Pull-up resistor on the GPIO5_IO00
    IOMUXC_SNVS->SW_PAD_CTL_PAD_WAKEUP = IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP_PUE(1) |
                                         IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP_PUS(3) |
                                         IOMUXC_SNVS_SW_PAD_CTL_PAD_WAKEUP_PKE(1);
    GPIO5->GDIR &= (1u << 0u); // Set GPIO5_IO00 pin to input mode

    uint32_t readCount = 0u;

    // Determine whether the pin is pressed (active low)
    for (uint32_t i = 0; i < 500; i++)
    {
        readCount += (GPIO5->DR & 1) ? 0 : 1;
    }

    if (readCount >= 250)
    {
        return true;
    }
    return false;
#else
    // Boot pin for Flash only target
    return false;
#endif
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
                    flash_device = kBootDevice_MMC_SD; // SD
                    break;
            }
            break;
        case 1:
            flash_device = kBootDevice_SemcNAND; // SEMC NAND
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
                config->dataValidTime[0].time_100ps = 19; // 1.9 ns // 1/4 * cycle of 133MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 38; // 3.8 ns // 1/2 * cycle of 133MHz DDR
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

#if BL_TARGET_FLASH
    uint32_t flashVectorTableStart = SCB->VTOR;
    memcpy(s_ramVectorTable, (void *)flashVectorTableStart, sizeof(s_ramVectorTable));
    SCB->VTOR = (uint32_t)s_ramVectorTable;
#endif

#ifdef _DEBUG
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_07] = 2;
    IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_06] = 2;
    IOMUXC->SW_PAD_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_07] = 0x10f1;
    IOMUXC->SW_PAD_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_06] = 0x10f1;
    lpuart_config_t lpuartConfig;
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = 115200;
    lpuartConfig.enableRx = true;
    lpuartConfig.enableTx = true;

    LPUART_Init(DEBUG_UART, &lpuartConfig, 20000000u);
#endif
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

//!@brief Get the hab status.
//habstatus_option_t get_hab_status(void)
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

void flexspi_update_padsetting(flexspi_mem_config_t *config, uint32_t driveStrength)
{
#define IOMUXC_PAD_SETTING_DSE_SHIFT (3)
#define IOMUXC_PAD_SETTING_DSE_MASK (0x07 << IOMUXC_PAD_SETTING_DSE_SHIFT)
#define IOMUXC_PAD_SETTING_DSE(x) (((x) << IOMUXC_PAD_SETTING_DSE_SHIFT) & IOMUXC_PAD_SETTING_DSE_MASK)
    if (driveStrength)
    {
        config->controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_PadSettingOverrideEnable);
        config->dqsPadSettingOverride =
            (FLEXSPI_DQS_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
        config->sclkPadSettingOverride =
            (FLEXSPI_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
        config->dataPadSettingOverride =
            (FLEXSPI_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);

        config->csPadSettingOverride =
            (FLEXSPI_DQS_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
    }
}

//void normal_mem_init(void)
//{
//    typedef struct
//    {
//        uint32_t dtcmSizeKB;
//        uint32_t itcmSizeKB;
//        uint32_t ocramSizeKB;
//    } flexram_cfg_t;

//    const flexram_cfg_t k_flexramCfgList[] = {
//        { 64, 64, 128 }, { 128, 64, 64 }, { 128, 0, 128 },  { 128, 32, 96 }, { 64, 128, 64 }, { 192, 0, 64 },
//        { 64, 32, 160 }, { 64, 0, 192 },  { 256, 64, 192 }, { 32, 64, 160 }, { 32, 128, 96 }, { 32, 160, 64 },
//        { 0, 128, 128 }, { 32, 32, 192 }, { 0, 192, 64 },   { 0, 0, 256 },
//    };

//    uint32_t ramCfgIndex = ROM_OCOTP_FLEXRAM_CFG_VALUE();

//    uint32_t itcmSize = k_flexramCfgList[ramCfgIndex].itcmSizeKB * 1024u;
//    uint32_t dtcmSize = k_flexramCfgList[ramCfgIndex].dtcmSizeKB * 1024u;
//    uint32_t ocramSize = k_flexramCfgList[ramCfgIndex].ocramSizeKB * 1024u;

//    if (itcmSize < 1)
//    {
//        itcmSize = 1;
//    }
//    if (dtcmSize < 1)
//    {
//        dtcmSize = 1;
//    }
//    if (ocramSize < 1)
//    {
//        ocramSize = 1;
//    }

//    g_memoryMap[0].startAddress = 0;
//    g_memoryMap[0].endAddress = g_memoryMap[0].startAddress + itcmSize - 1;
//    g_memoryMap[1].startAddress = 0x20000000;
//    g_memoryMap[1].endAddress = g_memoryMap[1].startAddress + dtcmSize - 1;
//    g_memoryMap[2].startAddress = 0x20200000;
//    g_memoryMap[2].endAddress = g_memoryMap[2].startAddress + ocramSize - 1;
//}

//status_t target_load_bootloader_config_area(bootloader_configuration_data_t *config)
//{
//    memcpy(config, (void *)kBootloaderConfigAreaAddress, sizeof(bootloader_configuration_data_t));

//    return kStatus_Success;
//}

#if BL_FEATURE_RELIABLE_UPDATE
typedef struct
{
    status_t (*erase)(uint32_t start, uint32_t lengthInbytes);
    status_t (*program)(uint32_t start, uint8_t *src, uint32_t length);
} flash_driver_interface_t;

typedef struct
{
    uint32_t version;
    status_t (*get_update_partition_info)(partition_t *partition);
    status_t (*update_image_state)(uint32_t state);
    status_t (*get_image_state)(uint32_t *state);
    status_t (*update_image_state_user_api)(uint32_t state, flash_driver_interface_t *flashIf);
    status_t (*update_partition_table_user_api)(partition_t *partition,
                                                uint32_t entries,
                                                flash_driver_interface_t *flashIf);
} reliable_update_interface_t;

status_t api_update_swap_meta(swap_meta_t *swap_meta)
{
    status_t status = kStatus_Fail;
    swap_meta_t swap_metas[2];
    for (uint32_t i = 0; i < 2; i++)
    {
        uint32_t meta_addr = BL_FEATURE_SWAP_META_START + i * BL_FEATURE_FLASH_SECTOR_SIZE;
        memcpy((uint8_t *)&swap_metas[i], (void *)meta_addr, sizeof(swap_meta_t));
    }

    uint32_t update_idx = 0;
    if ((kStatus_Success != swap_meta_check(&swap_metas[0])) && (kStatus_Success != swap_meta_check(&swap_metas[1])))
    {
        update_idx = 0;
    }
    else if ((kStatus_Success == swap_meta_check(&swap_metas[0])) &&
             (kStatus_Success != swap_meta_check(&swap_metas[1])))
    {
        update_idx = 1;
    }
    else if ((kStatus_Success != swap_meta_check(&swap_metas[0])) &&
             (kStatus_Success == swap_meta_check(&swap_metas[1])))
    {
        update_idx = 0;
    }
    else if ((kStatus_Success == swap_meta_check(&swap_metas[0])) &&
             (kStatus_Success == swap_meta_check(&swap_metas[1])))
    {
        update_idx = (swap_metas[0].meta_version > swap_metas[1].meta_version) ? 1 : 0;
    }

    uint32_t meta_addr = BL_FEATURE_SWAP_META_START + update_idx * BL_FEATURE_FLASH_SECTOR_SIZE;

    swap_meta->meta_version++;

    flexspi_nor_config_t flashNorConfig;
    memcpy(&flashNorConfig, (void *)BL_FLEXSPI_AMBA_BASE, sizeof(flashNorConfig));

    uint32_t instance = BL_FEATURE_FLEXSPI_NOR_MODULE_PERIPHERAL_INSTANCE;
    __disable_irq();
    status = flexspi_nor_flash_init(instance, &flashNorConfig);
    __enable_irq();
    uint32_t programBuffer[128];
    memset(programBuffer, 0xff, sizeof(programBuffer));

    do
    {
        uint32_t erase_addr = meta_addr - BL_FLEXSPI_AMBA_BASE;
        __disable_irq();
        status = flexspi_nor_flash_erase(instance, &flashNorConfig, erase_addr, BL_FEATURE_FLASH_SECTOR_SIZE);
        __enable_irq();
        if (status != kStatus_Success)
        {
            break;
        }

        memcpy(programBuffer, swap_meta, sizeof(*swap_meta));
        __disable_irq();
        status = flexspi_nor_flash_page_program(instance, &flashNorConfig, erase_addr, programBuffer);
        __enable_irq();
        if (status != kStatus_Success)
        {
            break;
        }

    } while (0);

    return status;
}

status_t app_api_update_swap_meta(swap_meta_t *swap_meta, flash_driver_interface_t *flashIf)
{
    status_t status = kStatus_Fail;
    swap_meta_t swap_metas[2];
    for (uint32_t i = 0; i < 2; i++)
    {
        uint32_t meta_addr = BL_FEATURE_SWAP_META_START + i * BL_FEATURE_FLASH_SECTOR_SIZE;
        memcpy((uint8_t *)&swap_metas[i], (void *)meta_addr, sizeof(swap_meta_t));
    }

    uint32_t update_idx = 0;
    if ((kStatus_Success != swap_meta_check(&swap_metas[0])) && (kStatus_Success != swap_meta_check(&swap_metas[1])))
    {
        update_idx = 0;
    }
    else if ((kStatus_Success == swap_meta_check(&swap_metas[0])) &&
             (kStatus_Success != swap_meta_check(&swap_metas[1])))
    {
        update_idx = 1;
    }
    else if ((kStatus_Success != swap_meta_check(&swap_metas[0])) &&
             (kStatus_Success == swap_meta_check(&swap_metas[1])))
    {
        update_idx = 0;
    }
    else if ((kStatus_Success == swap_meta_check(&swap_metas[0])) &&
             (kStatus_Success == swap_meta_check(&swap_metas[1])))
    {
        update_idx = (swap_metas[0].meta_version > swap_metas[1].meta_version) ? 1 : 0;
    }

    uint32_t meta_addr = BL_FEATURE_SWAP_META_START + update_idx * BL_FEATURE_FLASH_SECTOR_SIZE;

    swap_meta->meta_version++;
    do
    {
        uint32_t erase_addr = meta_addr - BL_FLEXSPI_AMBA_BASE;
        // Ensure that the program operation cannots be interrupted.
        uint32_t regPrimask = 0U;
        regPrimask = __get_PRIMASK();
        __disable_irq();
        status = flashIf->erase(erase_addr, BL_FEATURE_FLASH_SECTOR_SIZE);
        __set_PRIMASK(regPrimask);
        if (status != kStatus_Success)
        {
            break;
        }
        regPrimask = __get_PRIMASK();
        __disable_irq();
        status = flashIf->program(erase_addr, (uint8_t *)swap_meta, sizeof(*swap_meta));
        __set_PRIMASK(regPrimask);
        if (status != kStatus_Success)
        {
            break;
        }

    } while (0);

    return status;
}

status_t app_api_update_boot_meta(bootloader_meta_t *boot_meta, flash_driver_interface_t *flashIf)
{
    status_t status = kStatus_Fail;
    bootloader_meta_t boot_metas[2];
    for (uint32_t i = 0; i < 2; i++)
    {
        uint32_t meta_addr = BL_FEATURE_BOOT_META_START + i * BL_FEATURE_FLASH_SECTOR_SIZE;
        memcpy((uint8_t *)&boot_metas[i], (void *)meta_addr, sizeof(boot_metas[i]));
    }

    uint32_t update_idx = 0;
    if ((kStatus_Success != boot_meta_check(&boot_metas[0])) && (kStatus_Success != boot_meta_check(&boot_metas[1])))
    {
        update_idx = 0;
    }
    else if ((kStatus_Success == boot_meta_check(&boot_metas[0])) &&
             (kStatus_Success != boot_meta_check(&boot_metas[1])))
    {
        update_idx = 1;
    }
    else if ((kStatus_Success != boot_meta_check(&boot_metas[0])) &&
             (kStatus_Success == boot_meta_check(&boot_metas[1])))
    {
        update_idx = 0;
    }
    else if ((kStatus_Success == boot_meta_check(&boot_metas[0])) &&
             (kStatus_Success == boot_meta_check(&boot_metas[1])))
    {
        update_idx = (boot_metas[0].meta_version > boot_metas[1].meta_version) ? 1 : 0;
    }

    uint32_t meta_addr = BL_FEATURE_BOOT_META_START + update_idx * BL_FEATURE_FLASH_SECTOR_SIZE;

    boot_metas->meta_version++;
    do
    {
        uint32_t erase_addr = meta_addr - BL_FLEXSPI_AMBA_BASE;
        // Ensure that the program operation cannots be interrupted.
        uint32_t regPrimask = 0U;
        regPrimask = __get_PRIMASK();
        __disable_irq();
        status = flashIf->erase(erase_addr, BL_FEATURE_FLASH_SECTOR_SIZE);
        __set_PRIMASK(regPrimask);
        if (status != kStatus_Success)
        {
            break;
        }
        regPrimask = __get_PRIMASK();
        __disable_irq();
        status = flashIf->program(erase_addr, (uint8_t *)boot_meta, sizeof(*boot_meta));
        __set_PRIMASK(regPrimask);
        if (status != kStatus_Success)
        {
            break;
        }

    } while (0);

    return status;
}

status_t get_update_partition_info(partition_t *partition)
{
    bootloader_meta_t boot_meta;
    status_t status = load_boot_meta(&boot_meta);
    if (status != kStatus_Success)
    {
        return status;
    }
    memcpy(partition, &boot_meta.partition[kPartition_Secondary], sizeof(*partition));

    return kStatus_Success;
}

status_t update_partition_table_user_api(partition_t *partition, uint32_t entries, flash_driver_interface_t *flashIf)
{
    bootloader_meta_t boot_meta;
    bool has_boot_meta = false;

    if ((flashIf == NULL) || (entries > ARRAY_SIZE(boot_meta.partition)))
    {
        return kStatus_InvalidArgument;
    }

    status_t status = load_boot_meta(&boot_meta);
    if (status == kStatus_Success)
    {
        has_boot_meta = true;
    }

    if (!has_boot_meta)
    {
        memset(&boot_meta, 0, sizeof(boot_meta));
        boot_meta.tag = BOOTLOADER_META_TAG;
        boot_meta.features.enabledPeripherals = 0xffffffffu;
        boot_meta.features.periphDetectTimeout = 0xffffffffu;
        boot_meta.features.wdTimeout = 0xffffffffu;
    }

    boot_meta.patition_entries = entries;
    memcpy(&boot_meta.partition, partition, entries * sizeof(*partition));
    status = app_api_update_boot_meta(&boot_meta, flashIf);

    return status;
}

status_t update_image_state(uint32_t state)
{
    swap_meta_t swap_meta;
    status_t status = load_swap_meta(&swap_meta);
    if (status != kStatus_Success)
    {
        return status;
    }

    if (state == kSwapType_ReadyForTest)
    {
        image_header_t boot_header;
        get_image_header(kPartition_Secondary, &boot_header);
        if (boot_image_check(&boot_header, kPartition_Secondary) == kStatus_Success)
        {
            swap_meta.swap_type = kSwapType_ReadyForTest;
            swap_meta.copy_status = 0;
            swap_meta.swap_progress.swap_offset = 0;
            swap_meta.swap_progress.swap_status = kSwapStage_NotStarted;
            swap_meta.image_info[1].size = boot_header.image_size + boot_header.header_size;
            swap_meta.confirm_info = 0;
            api_update_swap_meta(&swap_meta);
            return kStatus_Success;
        }
        else
        {
            return kStatus_Fail;
        }
    }
    else if (state == kSwapType_Permanent)
    {
        swap_meta.copy_status = 0;
        swap_meta.swap_progress.swap_offset = 0;
        swap_meta.swap_progress.swap_status = kSwapStage_NotStarted;
        swap_meta.confirm_info = kImageConfirm_Okay;
        swap_meta.swap_type = kSwapType_Permanent;
        api_update_swap_meta(&swap_meta);
        return kStatus_Success;
    }
    else
    {
        return kStatus_InvalidArgument;
    }
}

status_t update_image_state_user_api(uint32_t state, flash_driver_interface_t *flashIf)
{
    swap_meta_t swap_meta;
    status_t status = load_swap_meta(&swap_meta);
    if (status != kStatus_Success)
    {
        return status;
    }

    if (state == kSwapType_ReadyForTest)
    {
        image_header_t boot_header;
        get_image_header(kPartition_Secondary, &boot_header);
        if (boot_image_check(&boot_header, kPartition_Secondary) == kStatus_Success)
        {
            swap_meta.swap_type = kSwapType_ReadyForTest;
            swap_meta.copy_status = 0;
            swap_meta.swap_progress.swap_offset = 0;
            swap_meta.swap_progress.swap_status = kSwapStage_NotStarted;
            swap_meta.image_info[1].size = boot_header.image_size + boot_header.header_size;
            swap_meta.confirm_info = 0;
            app_api_update_swap_meta(&swap_meta, flashIf);
            return kStatus_Success;
        }
        else
        {
            return kStatus_Fail;
        }
    }
    else if (state == kSwapType_Permanent)
    {
        swap_meta.copy_status = 0;
        swap_meta.swap_progress.swap_offset = 0;
        swap_meta.swap_progress.swap_status = kSwapStage_NotStarted;
        swap_meta.confirm_info = kImageConfirm_Okay;
        swap_meta.swap_type = kSwapType_Permanent;
        app_api_update_swap_meta(&swap_meta, flashIf);
        return kStatus_Success;
    }
    else
    {
        return kStatus_InvalidArgument;
    }
}

status_t get_image_state(uint32_t *state)
{
    swap_meta_t swap_meta;
    status_t status = load_swap_meta(&swap_meta);
    if (status != kStatus_Success)
    {
        return status;
    }
    *state = swap_meta.swap_type;

    return status;
}

__USED const reliable_update_interface_t g_reliableUpdateAPI = {
    MAKE_VERSION(1, 0, 0),
    .get_update_partition_info = get_update_partition_info,
    .update_image_state = update_image_state,
    .get_image_state = get_image_state,
    .update_image_state_user_api = update_image_state_user_api,
    .update_partition_table_user_api = update_partition_table_user_api,
};
#else
__USED const uint32_t g_reliableUpdateAPI = 0;
#endif //#if BL_FEATURE_RELIABLE_UPDATE

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
