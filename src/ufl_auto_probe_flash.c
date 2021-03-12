/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_auto_probe_flash.h"
#include "ufl_rom_api.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEFAULT_FLASH_BLOCK_SIZE (64*1024U)

#define FLASH_CONFIG_OPT_1BIT_SDR (0x5a5aa5a5)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if defined(UFL_USE_CONST_VAR)
const
#endif
extern flexspi_nor_config_t flashConfig;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*
kFlexspiNorOpt0_ISSI_IS25LP064A       = 0xc0000007
kFlexspiNorOpt0_ISSI_IS26KS512S       = 0xc0233007
kFlexspiNorOpt0_MXIC_MX25UM51245G     = 0xc0403037
kFlexspiNorOpt0_MXIC_MX25UM51345G     = 0xc0403007
kFlexspiNorOpt0_MXIC_MX25UM51345G_2nd = 0xc1503051
kFlexspiNorOpt1_MXIC_MX25UM51345G_2nd = 0x20000014
kFlexspiNorOpt0_Micron_MT35X          = 0xC0603005
kFlexspiNorOpt0_Adesto_AT25SF128A     = 0xc0000007
kFlexspiNorOpt0_Adesto_ATXP032        = 0xc0803007
kFlexspiNorOpt0_Cypress_S26KS512S     = 0xc0233007
kFlexspiNorOpt0_GigaDevice_GD25LB256E = 0xc0000007
kFlexspiNorOpt0_GigaDevice_GD25LT256E = 0xc0000008
kFlexspiNorOpt0_GigaDevice_GD25LX256E = 0xc0600008
kFlexspiNorOpt0_Winbond_W25Q128JV     = 0xc0000207
*/

static const serial_nor_config_option_t s_flashConfigOpt[] = {
    // For Normal Quad, eg. IS25LP064A, GD25LB256E
      // 1st Pinmux, PortA
    {.option0.U = 0xc0000001, .option1.U = 0x00000000},
      // 1st Pinmux, PortB
    {.option0.U = 0xc1000001, .option1.U = 0x20000000},
      // 2nd Pinmux, PortA
    {.option0.U = 0xc1000001, .option1.U = 0x00010000},

    // For Normal Octal, eg. MX25UM51345G
      // 1st Pinmux, PortA
    {.option0.U = 0xc0403001, .option1.U = 0x00000000},
      // 1st Pinmux, PortB, No DQS
    {.option0.U = 0xc1503051, .option1.U = 0x20000014},

    // For Normal HyperBus, eg. S26KS512S, IS26KS512S
    {.option0.U = (0xc0233000 + kSerialNorCfgOption_MaxFreq), .option1.U = 0x00000000},

    // For Normal Octal, eg. MX25UM51245G
    {.option0.U = 0xc0403031, .option1.U = 0x00000000},

    // For Normal Octal, eg. MT35X
    {.option0.U = 0xc0603001, .option1.U = 0x00000000},
    // For Normal Octal, eg. ATXP032
    {.option0.U = 0xc0803001, .option1.U = 0x00000000},

    // For Normal 1-bit SDR
    {.option0.U = FLASH_CONFIG_OPT_1BIT_SDR, .option1.U = 0x00000000},
};

static const flexspi_nor_config_t s_flashConfigPortX = {
    .memConfig = {
        .tag                  = FLEXSPI_CFG_BLK_TAG,
        .version              = FLEXSPI_CFG_BLK_VERSION,
        .readSampleClkSrc     = kFlexSPIReadSampleClk_LoopbackInternally,
        .csHoldTime           = 3,
        .csSetupTime          = 3,
        .columnAddressWidth   = 0,
        .deviceModeCfgEnable  = 0,
        .deviceModeType       = 0,
        .waitTimeCfgCommands  = 0,
        .deviceModeSeq        = {.seqNum = 0,
                                 .seqId  = 0,},
        .deviceModeArg        = 0,
        .configCmdEnable      = 0,
        .configModeType       = {0},
        .configCmdSeqs        = {0},
        .configCmdArgs        = {0},
        .controllerMiscOption = (0),
        .deviceType           = kFlexSpiDeviceType_SerialNOR,
        .sflashPadType        = kSerialFlash_1Pad,
        .serialClkFreq        = kFlexSpiSerialClk_30MHz,
        .lutCustomSeqEnable   = 0,
        .sflashA1Size         = 0x00200000,
        .sflashA2Size         = 0,
        .sflashB1Size         = 0,
        .sflashB2Size         = 0,
        .csPadSettingOverride = 0,
        .sclkPadSettingOverride = 0,
        .dataPadSettingOverride = 0,
        .dqsPadSettingOverride  = 0,
        .timeoutInMs            = 0,
        .commandInterval        = 0,
        .busyOffset             = 0,
        .busyBitPolarity        = 0,
        .lookupTable            = {
            [0]  = 0x08180403,
            [1]  = 0x00002404,
            [4]  = 0x24040405,
            [12] = 0x00000604,
            [20] = 0x081804D8,
            [36] = 0x08180402,
            [37] = 0x00002080,
            [44] = 0x00000460,
        },
    },
    .pageSize           = 0x200,
    .sectorSize         = 0x1000,
    .ipcmdSerialClkFreq = 1,
    .isUniformBlockSize = 0,
    .blockSize          = 0x10000,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

//!@brief Get FlexSPI instance
FLEXSPI_Type *flexspi_get_module_base(uint32_t instance)
{
    return (FLEXSPI_Type *)g_uflTargetDesc.flexspiBaseAddr;
}

static void flexspi_error_handler(uint32_t instance)
{
    flexspi_sw_reset(instance);
    flexspi_wait_idle(instance);
}

status_t ufl_auto_probe(void)
{
    status_t status = kStatus_Success;
    uint32_t instance = g_uflTargetDesc.flexspiInstance;
    ufl_target_desc_t *uflTargetDesc = (ufl_target_desc_t *)&g_uflTargetDesc;

    // If we have found proper option before, we just reuse it.
    if (g_uflTargetDesc.configOption.option0.B.tag == kSerialNorCfgOption_Tag)
    {
        memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));
        status = flexspi_nor_auto_config(instance, (flexspi_nor_config_t *)&flashConfig, (serial_nor_config_option_t *)&g_uflTargetDesc.configOption);
    }
    else
    {
        serial_nor_config_option_t option;
        uint32_t retryCnt = sizeof(s_flashConfigOpt) / sizeof(serial_nor_config_option_t);
        bool isLowerFreqPassed = false;
        bool isHigherFreqFailed = false;

        // Try all kinds of flash config options until we find proper option.
        for (uint32_t idx = 0; idx < retryCnt;)
        {
            register uint32_t delaycnt;
            // Wait until the FLEXSPI is idle
            delaycnt = 10000u;
            while(delaycnt--)
            {
            }
            
            // Check to see whether to use 1bit SDR
            if (s_flashConfigOpt[idx].option0.U == FLASH_CONFIG_OPT_1BIT_SDR)
            {
                // Currenttly only support Port A
                flexspi_nor_config_t *configPort = (flexspi_nor_config_t *)&s_flashConfigPortX;
                configPort->memConfig.sflashA1Size = 0x00200000;
                configPort->memConfig.sflashB1Size = 0x0;
                status = flexspi_nor_flash_init(instance, configPort);
                if (status == kStatus_Success)
                {
                    break;
                }
                continue;
            }

            memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));
            // Only when last option wasn't passed ever, then we will try new option.
            if (!isLowerFreqPassed)
            {
                option.option0.U = s_flashConfigOpt[idx].option0.U;
                option.option1.U = s_flashConfigOpt[idx].option1.U;
            }
            status = flexspi_nor_get_config(instance, (flexspi_nor_config_t *)&flashConfig, &option);
            if (status == kStatus_Success)
            {
                status = flexspi_nor_flash_init(instance, (flexspi_nor_config_t *)&flashConfig);
                if ((status == kStatus_Success) &&
                    (flashConfig.sectorSize != 0))
                {
                    status = flexspi_nor_flash_erase(instance, (flexspi_nor_config_t *)&flashConfig, 0x0, flashConfig.sectorSize);
                    if ((status == kStatus_Success) &&
                        (flashConfig.pageSize != 0))
                    {
                        status = flexspi_nor_flash_page_program(instance, (flexspi_nor_config_t *)&flashConfig, 0x0, (uint32_t *)&flashConfig);
                        if (status == kStatus_Success)
                        {
                            // Only when higher freq of current option wasn't failed ever, then 
                            //   we will try higher freq of current option.
                            if ((!isHigherFreqFailed) &&
                                (option.option1.U == 0) &&
                                (option.option0.B.max_freq < kSerialNorCfgOption_MaxFreq))
                            {
                                isLowerFreqPassed = true;
                                option.option0.B.max_freq++;
                                flexspi_error_handler(instance);
                                continue;
                            }
                            // If we get to max freq or we failed to use higher freq, then we use 
                            //   current freq as final option.
                            else
                            {
                                uflTargetDesc->configOption.option0.U = option.option0.U;
                                uflTargetDesc->configOption.option1.U = option.option1.U;
                                break;
                            }
                        }
                    }
                }
            }
            if (isLowerFreqPassed)
            {
                // Current freq of option is failed, now we need to go back to lower freq.
                isHigherFreqFailed = true;
                option.option0.B.max_freq--;
            }
            else
            {
                // Switch to new option
                idx++;
            }
            flexspi_error_handler(instance);
        }
    }

    // Set default block size if it is 0
    if (flashConfig.blockSize == 0)
    {
        flexspi_nor_config_t *flashCfg = (flexspi_nor_config_t *)&flashConfig;
        flashCfg->blockSize = DEFAULT_FLASH_BLOCK_SIZE;
    }

    return status;
}

