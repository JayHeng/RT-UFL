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

ufl_target_desc_t g_uflTargetDesc = {.imxrtChipId = kChipId_Invalid};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void ufl_fill_flash_api(void)
{
    rt_chip_id_t chipId = (rt_chip_id_t)g_uflTargetDesc.imxrtChipId;
    switch (chipId)
    {
        case kChipId_RT5xx:
            g_uflTargetDesc.flashDriver.init             = g_bootloaderTree_imxrt5xx->flexspiNorDriver->init;
            g_uflTargetDesc.flashDriver.page_program     = g_bootloaderTree_imxrt5xx->flexspiNorDriver->page_program;
            g_uflTargetDesc.isFlashPageProgram           = true;
            g_uflTargetDesc.flashDriver.erase_all        = g_bootloaderTree_imxrt5xx->flexspiNorDriver->erase_all;
            g_uflTargetDesc.flashDriver.erase            = g_bootloaderTree_imxrt5xx->flexspiNorDriver->erase;
            g_uflTargetDesc.flashDriver.read             = g_bootloaderTree_imxrt5xx->flexspiNorDriver->read;
            g_uflTargetDesc.flashDriver.set_clock_source = g_bootloaderTree_imxrt5xx->flexspiNorDriver->set_clock_source;
            g_uflTargetDesc.flashDriver.get_config       = g_bootloaderTree_imxrt5xx->flexspiNorDriver->get_config;
            break;

        case kChipId_RT6xx:
            g_uflTargetDesc.flashDriver.init             = g_bootloaderTree_imxrt6xx->flexspiNorDriver->init;
            g_uflTargetDesc.flashDriver.page_program     = g_bootloaderTree_imxrt6xx->flexspiNorDriver->page_program;
            g_uflTargetDesc.isFlashPageProgram           = true;
            g_uflTargetDesc.flashDriver.erase_all        = g_bootloaderTree_imxrt6xx->flexspiNorDriver->erase_all;
            g_uflTargetDesc.flashDriver.erase            = g_bootloaderTree_imxrt6xx->flexspiNorDriver->erase;
            g_uflTargetDesc.flashDriver.read             = g_bootloaderTree_imxrt6xx->flexspiNorDriver->read;
            g_uflTargetDesc.flashDriver.set_clock_source = g_bootloaderTree_imxrt6xx->flexspiNorDriver->set_clock_source;
            g_uflTargetDesc.flashDriver.get_config       = g_bootloaderTree_imxrt6xx->flexspiNorDriver->get_config;
            break;

        case kChipId_RT106x:
            g_uflTargetDesc.flashDriver.init             = g_bootloaderTree_imxrt106x->flexspiNorDriver->init;
            g_uflTargetDesc.flashDriver.page_program     = g_bootloaderTree_imxrt106x->flexspiNorDriver->program;
            g_uflTargetDesc.isFlashPageProgram           = false;
            g_uflTargetDesc.flashDriver.erase_all        = g_bootloaderTree_imxrt106x->flexspiNorDriver->erase_all;
            g_uflTargetDesc.flashDriver.erase            = g_bootloaderTree_imxrt106x->flexspiNorDriver->erase;
            g_uflTargetDesc.flashDriver.read             = g_bootloaderTree_imxrt106x->flexspiNorDriver->read;
            g_uflTargetDesc.flashDriver.set_clock_source = NULL;
            g_uflTargetDesc.flashDriver.get_config       = g_bootloaderTree_imxrt106x->flexspiNorDriver->get_config;
            break;

        case kChipId_RT117x:
            g_uflTargetDesc.flashDriver.init             = g_bootloaderTree_imxrt117x->flexspiNorDriver->init;
            g_uflTargetDesc.flashDriver.page_program     = g_bootloaderTree_imxrt117x->flexspiNorDriver->page_program;
            g_uflTargetDesc.isFlashPageProgram           = true;
            g_uflTargetDesc.flashDriver.erase_all        = g_bootloaderTree_imxrt117x->flexspiNorDriver->erase_all;
            g_uflTargetDesc.flashDriver.erase            = g_bootloaderTree_imxrt117x->flexspiNorDriver->erase;
            g_uflTargetDesc.flashDriver.read             = g_bootloaderTree_imxrt117x->flexspiNorDriver->read;
            g_uflTargetDesc.flashDriver.set_clock_source = NULL;
            g_uflTargetDesc.flashDriver.get_config       = g_bootloaderTree_imxrt117x->flexspiNorDriver->get_config;
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

        case kChipId_RT106x:
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
    switch (chipId)
    {
        case kChipId_RT5xx:
            g_uflTargetDesc.flexspiInstance = MIMXRT5XX_1st_FLEXSPI_INSTANCE;
            g_uflTargetDesc.flexspiBaseAddr = MIMXRT5XX_1st_FLEXSPI_BASE;
            g_uflTargetDesc.flashBaseAddr   = MIMXRT5XX_1st_FLEXSPI_AMBA_BASE;
            //g_uflTargetDesc.configOption.option0.U = 0xc0403004;
            //g_uflTargetDesc.configOption.option1.U = 0x0;
            break;

        case kChipId_RT6xx:
            g_uflTargetDesc.flexspiInstance = MIMXRT6XX_1st_FLEXSPI_INSTANCE;
            g_uflTargetDesc.flexspiBaseAddr = MIMXRT6XX_1st_FLEXSPI_BASE;
            g_uflTargetDesc.flashBaseAddr   = MIMXRT6XX_1st_FLEXSPI_AMBA_BASE;
            //g_uflTargetDesc.configOption.option0.U = 0xc1503051;
            //g_uflTargetDesc.configOption.option1.U = 0x20000014;
            break;

        case kChipId_RT106x:
            g_uflTargetDesc.flexspiInstance = MIMXRT106X_1st_FLEXSPI_INSTANCE;
            g_uflTargetDesc.flexspiBaseAddr = MIMXRT106X_1st_FLEXSPI_BASE;
            g_uflTargetDesc.flashBaseAddr   = MIMXRT106X_1st_FLEXSPI_AMBA_BASE;
            //g_uflTargetDesc.configOption.option0.U = 0xc0000006;
            //g_uflTargetDesc.configOption.option1.U = 0x0;
            break;

        case kChipId_RT117x:
            g_uflTargetDesc.flexspiInstance = MIMXRT117X_1st_FLEXSPI_INSTANCE;
            g_uflTargetDesc.flexspiBaseAddr = MIMXRT117X_1st_FLEXSPI_BASE;
            g_uflTargetDesc.flashBaseAddr   = MIMXRT117X_1st_FLEXSPI_AMBA_BASE;
            //g_uflTargetDesc.configOption.option0.U = 0xc0000006;
            //g_uflTargetDesc.configOption.option1.U = 0x0;
            break;

        case kChipId_Invalid:
        default:
            break;
    }
}

status_t ufl_full_setup(void)
{
    static bool isFirstTimeInit = true;
    status_t status = kStatus_Success;

    // ufl_full_setup() is called in Init(), the Init() is called by 
    //   Erase/Program/verify operation everytime.
    // As we have auto probe feature, this featue can be used to find 
    //   proper flash config option.
    // If we have found proper option by auto probe, we don't need to 
    //   do auto probe anymore. so we just need to init g_uflTargetDesc once.
    if (isFirstTimeInit)
    {
        memset((void *)&g_uflTargetDesc, 0U, sizeof(ufl_target_desc_t));
        isFirstTimeInit = false;
    }

    {
        rt_chip_id_t chipId = ufl_get_imxrt_chip_id();
        g_uflTargetDesc.imxrtChipId = (uint32_t)chipId;

        ufl_fill_flash_api();
        ufl_init_hardware();
        ufl_set_target_property();

        status = ufl_auto_probe();
    }

    return status;
}

