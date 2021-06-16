/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_find_target.h"
#include "ufl_rom_api.h"
#include "ufl_hardware_init.h"
#include "ufl_auto_probe_flash.h"
#include "ufl_flexspi_nor_bsp_imxrt101x.h"
#include "ufl_flexspi_nor_bsp_imxrt102x.h"
#include "ufl_flexspi_nor_bsp_imxrt105x.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void ufl_fill_flash_api(void);
static void ufl_init_hardware(void);
static void ufl_set_target_property(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if defined(UFL_USE_CONST_VAR)
const
#endif
ufl_target_desc_t g_uflTargetDesc = {.imxrtChipId = kChipId_Invalid};
#if defined(UFL_USE_CONST_VAR)
const
#endif
bool g_isFirstTimeInit = true;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void ufl_fill_flash_api(void)
{
    rt_chip_id_t chipId = (rt_chip_id_t)g_uflTargetDesc.imxrtChipId;
    ufl_target_desc_t *uflTargetDesc = (ufl_target_desc_t *)&g_uflTargetDesc;
    switch (chipId)
    {
        case kChipId_RT5xx:
            uflTargetDesc->flashDriver.init             = g_bootloaderTree_imxrt5xx->flexspiNorDriver->init;
            uflTargetDesc->flashDriver.page_program     = g_bootloaderTree_imxrt5xx->flexspiNorDriver->page_program;
            uflTargetDesc->isFlashPageProgram           = true;
            uflTargetDesc->flashDriver.erase_all        = g_bootloaderTree_imxrt5xx->flexspiNorDriver->erase_all;
            uflTargetDesc->flashDriver.erase            = g_bootloaderTree_imxrt5xx->flexspiNorDriver->erase;
            uflTargetDesc->flashDriver.read             = g_bootloaderTree_imxrt5xx->flexspiNorDriver->read;
            uflTargetDesc->flashDriver.set_clock_source = g_bootloaderTree_imxrt5xx->flexspiNorDriver->set_clock_source;
            uflTargetDesc->flashDriver.get_config       = g_bootloaderTree_imxrt5xx->flexspiNorDriver->get_config;
            // It doesn't matter that page size is overrided or not.
            uflTargetDesc->iarCfg.enablePageSizeOverride = true;
            break;

        case kChipId_RT6xx:
            uflTargetDesc->flashDriver.init             = g_bootloaderTree_imxrt6xx->flexspiNorDriver->init;
            uflTargetDesc->flashDriver.page_program     = g_bootloaderTree_imxrt6xx->flexspiNorDriver->page_program;
            uflTargetDesc->isFlashPageProgram           = true;
            uflTargetDesc->flashDriver.erase_all        = g_bootloaderTree_imxrt6xx->flexspiNorDriver->erase_all;
            uflTargetDesc->flashDriver.erase            = g_bootloaderTree_imxrt6xx->flexspiNorDriver->erase;
            uflTargetDesc->flashDriver.read             = g_bootloaderTree_imxrt6xx->flexspiNorDriver->read;
            uflTargetDesc->flashDriver.set_clock_source = g_bootloaderTree_imxrt6xx->flexspiNorDriver->set_clock_source;
            uflTargetDesc->flashDriver.get_config       = g_bootloaderTree_imxrt6xx->flexspiNorDriver->get_config;
            // page size hasn't to be overrided, or downloading will be hung.
            uflTargetDesc->iarCfg.enablePageSizeOverride = false;
            break;

        case kChipId_RT101x:
            uflTargetDesc->flashDriver.init             = flexspi_nor_drv_flash_init;
            uflTargetDesc->flashDriver.page_program     = flexspi_nor_drv_flash_page_program;
            uflTargetDesc->isFlashPageProgram           = true;
            uflTargetDesc->flashDriver.erase_all        = flexspi_nor_drv_flash_erase_all;
            uflTargetDesc->flashDriver.erase            = flexspi_nor_drv_flash_erase;
            uflTargetDesc->flashDriver.read             = flexspi_nor_drv_flash_read;
            uflTargetDesc->flashDriver.set_clock_source = NULL;
            uflTargetDesc->flashDriver.get_config       = flexspi_nor_drv_get_config;
            // page size has to be overrided, or downloading will be hung.
            uflTargetDesc->iarCfg.enablePageSizeOverride = true;

            uflTargetDesc->flexspiBsp.flexspi_iomux_config         = flexspi_iomux_config_rt1010;
            uflTargetDesc->flexspiBsp.flexspi_update_padsetting    = flexspi_update_padsetting_rt1010;
            uflTargetDesc->flexspiBsp.flexspi_clock_config         = flexspi_clock_config_rt1010;
            uflTargetDesc->flexspiBsp.flexspi_set_failsafe_setting = flexspi_set_failsafe_setting_rt1010;
            uflTargetDesc->flexspiBsp.CLOCK_GetCPUFreq             = CLOCK_GetCPUFreq_RT1010;
            uflTargetDesc->flexspiBsp.flexspi_get_max_supported_freq = flexspi_get_max_supported_freq_rt1010;
            uflTargetDesc->flexspiBsp.flexspi_clock_gate_enable    = flexspi_clock_gate_enable_rt1010;
            uflTargetDesc->flexspiBsp.flexspi_clock_gate_disable   = flexspi_clock_gate_disable_rt1010;
            uflTargetDesc->flexspiBsp.flexspi_nor_write_persistent = flexspi_nor_write_persistent_rt1010;
            uflTargetDesc->flexspiBsp.flexspi_get_clock            = flexspi_get_clock_rt1010;
            uflTargetDesc->flexspiBsp.flexspi_nor_read_persistent  = flexspi_nor_read_persistent_rt1010;
            break;

        case kChipId_RT1015:
        case kChipId_RT102x:
        case kChipId_RT1024_SIP:
            uflTargetDesc->flashDriver.init             = flexspi_nor_drv_flash_init;
            uflTargetDesc->flashDriver.page_program     = flexspi_nor_drv_flash_page_program;
            uflTargetDesc->isFlashPageProgram           = true;
            uflTargetDesc->flashDriver.erase_all        = flexspi_nor_drv_flash_erase_all;
            uflTargetDesc->flashDriver.erase            = flexspi_nor_drv_flash_erase;
            uflTargetDesc->flashDriver.read             = flexspi_nor_drv_flash_read;
            uflTargetDesc->flashDriver.set_clock_source = NULL;
            uflTargetDesc->flashDriver.get_config       = flexspi_nor_drv_get_config;
            // page size has to be overrided, or downloading will be hung.
            uflTargetDesc->iarCfg.enablePageSizeOverride = true;

            uflTargetDesc->flexspiBsp.flexspi_iomux_config         = flexspi_iomux_config_rt1020;
            uflTargetDesc->flexspiBsp.flexspi_update_padsetting    = flexspi_update_padsetting_rt1020;
            uflTargetDesc->flexspiBsp.flexspi_clock_config         = flexspi_clock_config_rt1020;
            uflTargetDesc->flexspiBsp.flexspi_set_failsafe_setting = flexspi_set_failsafe_setting_rt1020;
            uflTargetDesc->flexspiBsp.CLOCK_GetCPUFreq             = CLOCK_GetCPUFreq_RT1020;
            uflTargetDesc->flexspiBsp.flexspi_get_max_supported_freq = flexspi_get_max_supported_freq_rt1020;
            uflTargetDesc->flexspiBsp.flexspi_clock_gate_enable    = flexspi_clock_gate_enable_rt1020;
            uflTargetDesc->flexspiBsp.flexspi_clock_gate_disable   = flexspi_clock_gate_disable_rt1020;
            uflTargetDesc->flexspiBsp.flexspi_nor_write_persistent = flexspi_nor_write_persistent_rt1020;
            uflTargetDesc->flexspiBsp.flexspi_get_clock            = flexspi_get_clock_rt1020;
            uflTargetDesc->flexspiBsp.flexspi_nor_read_persistent  = flexspi_nor_read_persistent_rt1020;
            break;

        case kChipId_RT105x:
            uflTargetDesc->flashDriver.init             = flexspi_nor_drv_flash_init;
            uflTargetDesc->flashDriver.page_program     = flexspi_nor_drv_flash_page_program;
            uflTargetDesc->isFlashPageProgram           = true;
            uflTargetDesc->flashDriver.erase_all        = flexspi_nor_drv_flash_erase_all;
            uflTargetDesc->flashDriver.erase            = flexspi_nor_drv_flash_erase;
            uflTargetDesc->flashDriver.read             = flexspi_nor_drv_flash_read;
            uflTargetDesc->flashDriver.set_clock_source = NULL;
            uflTargetDesc->flashDriver.get_config       = flexspi_nor_drv_get_config;
            // page size has to be overrided, or downloading will be hung.
            uflTargetDesc->iarCfg.enablePageSizeOverride = true;

            uflTargetDesc->flexspiBsp.flexspi_iomux_config         = flexspi_iomux_config_rt1050;
            uflTargetDesc->flexspiBsp.flexspi_update_padsetting    = flexspi_update_padsetting_rt1050;
            uflTargetDesc->flexspiBsp.flexspi_clock_config         = flexspi_clock_config_rt1050;
            uflTargetDesc->flexspiBsp.flexspi_set_failsafe_setting = flexspi_set_failsafe_setting_rt1050;
            uflTargetDesc->flexspiBsp.CLOCK_GetCPUFreq             = CLOCK_GetCPUFreq_RT1050;
            uflTargetDesc->flexspiBsp.flexspi_get_max_supported_freq = flexspi_get_max_supported_freq_rt1050;
            uflTargetDesc->flexspiBsp.flexspi_clock_gate_enable    = flexspi_clock_gate_enable_rt1050;
            uflTargetDesc->flexspiBsp.flexspi_clock_gate_disable   = flexspi_clock_gate_disable_rt1050;
            uflTargetDesc->flexspiBsp.flexspi_nor_write_persistent = flexspi_nor_write_persistent_rt1050;
            uflTargetDesc->flexspiBsp.flexspi_get_clock            = flexspi_get_clock_rt1050;
            uflTargetDesc->flexspiBsp.flexspi_nor_read_persistent  = flexspi_nor_read_persistent_rt1050;
            break;

        case kChipId_RT106x:
        case kChipId_RT1064_SIP:
            uflTargetDesc->flashDriver.init             = g_bootloaderTree_imxrt106x->flexspiNorDriver->init;
            uflTargetDesc->flashDriver.page_program     = g_bootloaderTree_imxrt106x->flexspiNorDriver->program;
            uflTargetDesc->isFlashPageProgram           = false;
            uflTargetDesc->flashDriver.erase_all        = g_bootloaderTree_imxrt106x->flexspiNorDriver->erase_all;
            uflTargetDesc->flashDriver.erase            = g_bootloaderTree_imxrt106x->flexspiNorDriver->erase;
            uflTargetDesc->flashDriver.read             = g_bootloaderTree_imxrt106x->flexspiNorDriver->read;
            uflTargetDesc->flashDriver.set_clock_source = NULL;
            uflTargetDesc->flashDriver.get_config       = g_bootloaderTree_imxrt106x->flexspiNorDriver->get_config;
            // page size has to be overrided, or downloading will be hung.
            uflTargetDesc->iarCfg.enablePageSizeOverride = true;
            break;

        case kChipId_RT117x:
            uflTargetDesc->flashDriver.init             = g_bootloaderTree_imxrt117x->flexspiNorDriver->init;
            uflTargetDesc->flashDriver.page_program     = g_bootloaderTree_imxrt117x->flexspiNorDriver->page_program;
            uflTargetDesc->isFlashPageProgram           = true;
            uflTargetDesc->flashDriver.erase_all        = g_bootloaderTree_imxrt117x->flexspiNorDriver->erase_all;
            uflTargetDesc->flashDriver.erase            = g_bootloaderTree_imxrt117x->flexspiNorDriver->erase;
            uflTargetDesc->flashDriver.read             = g_bootloaderTree_imxrt117x->flexspiNorDriver->read;
            uflTargetDesc->flashDriver.set_clock_source = NULL;
            uflTargetDesc->flashDriver.get_config       = g_bootloaderTree_imxrt117x->flexspiNorDriver->get_config;
            // It doesn't matter that page size is overrided or not.
            uflTargetDesc->iarCfg.enablePageSizeOverride = true;
            break;

        case kChipId_Invalid:
        default:
            break;
    }
}

static void ufl_init_hardware(void)
{
    rt_chip_id_t chipId = (rt_chip_id_t)g_uflTargetDesc.imxrtChipId;
    switch (chipId)
    {
        case kChipId_RT5xx:
            ufl_init_hardware_imxrt5xx();
            break;

        case kChipId_RT6xx:
            ufl_init_hardware_imxrt6xx();
            break;

        case kChipId_RT101x:
            ufl_init_hardware_imxrt101x();
            break;

        case kChipId_RT1015:
        case kChipId_RT102x:
        case kChipId_RT1024_SIP:
            ufl_init_hardware_imxrt102x();
            break;

        case kChipId_RT105x:
            ufl_init_hardware_imxrt105x();
            break;

        case kChipId_RT106x:
        case kChipId_RT1064_SIP:
            ufl_init_hardware_imxrt106x();
            break;

        case kChipId_RT117x:
            ufl_init_hardware_imxrt117x();
            break;

        case kChipId_Invalid:
        default:
            break;
    }
}

static void ufl_set_target_property(void)
{
    rt_chip_id_t chipId = (rt_chip_id_t)g_uflTargetDesc.imxrtChipId;
    ufl_target_desc_t *uflTargetDesc = (ufl_target_desc_t *)&g_uflTargetDesc;
    switch (chipId)
    {
        case kChipId_RT5xx:
            uflTargetDesc->flexspiInstance = MIMXRT5XX_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT5XX_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT5XX_1st_FLEXSPI_AMBA_BASE;
            //uflTargetDesc->configOption.option0.U = 0xc0403004;
            //uflTargetDesc->configOption.option1.U = 0x0;
            break;

        case kChipId_RT6xx:
            uflTargetDesc->flexspiInstance = MIMXRT6XX_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT6XX_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT6XX_1st_FLEXSPI_AMBA_BASE;
            //uflTargetDesc->configOption.option0.U = 0xc1503051;
            //uflTargetDesc->configOption.option1.U = 0x20000014;
            break;

        case kChipId_RT101x:
            uflTargetDesc->flexspiInstance = MIMXRT101X_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT101X_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT101X_1st_FLEXSPI_AMBA_BASE;
            //uflTargetDesc->configOption.option0.U = 0xc0000006;
            //uflTargetDesc->configOption.option1.U = 0x0;
            break;

        case kChipId_RT1015:
        case kChipId_RT102x:
            uflTargetDesc->flexspiInstance = MIMXRT102X_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT102X_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT102X_1st_FLEXSPI_AMBA_BASE;
            //uflTargetDesc->configOption.option0.U = 0xc0000006;
            //uflTargetDesc->configOption.option1.U = 0x0;
            break;

        case kChipId_RT1024_SIP:
            uflTargetDesc->flexspiInstance = MIMXRT102X_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT102X_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT102X_1st_FLEXSPI_AMBA_BASE;
            uflTargetDesc->configOption.option0.U = 0xc0000006;
            uflTargetDesc->configOption.option1.U = 0x0;
            break;

        case kChipId_RT105x:
            uflTargetDesc->flexspiInstance = MIMXRT105X_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT105X_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT105X_1st_FLEXSPI_AMBA_BASE;
            //uflTargetDesc->configOption.option0.U = 0xc0233007;
            //uflTargetDesc->configOption.option1.U = 0x0;
            break;

        case kChipId_RT106x:
            uflTargetDesc->flexspiInstance = MIMXRT106X_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT106X_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT106X_1st_FLEXSPI_AMBA_BASE;
            //uflTargetDesc->configOption.option0.U = 0xc0000006;
            //uflTargetDesc->configOption.option1.U = 0x0;
            break;

        case kChipId_RT1064_SIP:
            uflTargetDesc->flexspiInstance = MIMXRT106X_2nd_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT106X_2nd_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT106X_2nd_FLEXSPI_AMBA_BASE;
            uflTargetDesc->configOption.option0.U = 0xc0000006;
            uflTargetDesc->configOption.option1.U = 0x0;
            break;

        case kChipId_RT117x:
            uflTargetDesc->flexspiInstance = MIMXRT117X_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT117X_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT117X_1st_FLEXSPI_AMBA_BASE;
            //uflTargetDesc->configOption.option0.U = 0xc0000006;
            //uflTargetDesc->configOption.option1.U = 0x0;
            break;

        case kChipId_Invalid:
        default:
            break;
    }
}

status_t ufl_full_setup(void)
{
    bool *isFirstTimeInit = (bool *)&g_isFirstTimeInit;
    status_t status = kStatus_Success;

    // ufl_full_setup() is called in Init(), the Init() is called by 
    //   Erase/Program/verify operation everytime.
    // As we have auto probe feature, this featue can be used to find 
    //   proper flash config option.
    // If we have found proper option by auto probe, we don't need to 
    //   do auto probe anymore. so we just need to init g_uflTargetDesc once.
    if (*isFirstTimeInit)
    {
        serial_nor_config_option_t option;
        option.option0.U = g_uflTargetDesc.configOption.option0.U;
        option.option1.U = g_uflTargetDesc.configOption.option1.U;

        memset((void *)&g_uflTargetDesc, 0U, sizeof(ufl_target_desc_t));

        if (option.option0.B.tag == kSerialNorCfgOption_Tag)
        {
            ufl_target_desc_t *uflTargetDesc = (ufl_target_desc_t *)&g_uflTargetDesc;
            uflTargetDesc->configOption.option0.U = option.option0.U;
            uflTargetDesc->configOption.option1.U = option.option1.U;
        }

        *isFirstTimeInit = false;
    }

    {
        rt_chip_id_t chipId = ufl_get_imxrt_chip_id();
        ufl_target_desc_t *uflTargetDesc = (ufl_target_desc_t *)&g_uflTargetDesc;
        uflTargetDesc->imxrtChipId = (uint32_t)chipId;

        ufl_fill_flash_api();
        ufl_init_hardware();
        ufl_set_target_property();

        status = ufl_auto_probe();
    }

    return status;
}

