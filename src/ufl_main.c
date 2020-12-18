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
            break;

        case kChipId_RT106x:
            uflTargetDesc->flashDriver.init             = g_bootloaderTree_imxrt106x->flexspiNorDriver->init;
            uflTargetDesc->flashDriver.page_program     = g_bootloaderTree_imxrt106x->flexspiNorDriver->program;
            uflTargetDesc->isFlashPageProgram           = false;
            uflTargetDesc->flashDriver.erase_all        = g_bootloaderTree_imxrt106x->flexspiNorDriver->erase_all;
            uflTargetDesc->flashDriver.erase            = g_bootloaderTree_imxrt106x->flexspiNorDriver->erase;
            uflTargetDesc->flashDriver.read             = g_bootloaderTree_imxrt106x->flexspiNorDriver->read;
            uflTargetDesc->flashDriver.set_clock_source = NULL;
            uflTargetDesc->flashDriver.get_config       = g_bootloaderTree_imxrt106x->flexspiNorDriver->get_config;
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

        case kChipId_RT106x:
            uflTargetDesc->flexspiInstance = MIMXRT106X_1st_FLEXSPI_INSTANCE;
            uflTargetDesc->flexspiBaseAddr = MIMXRT106X_1st_FLEXSPI_BASE;
            uflTargetDesc->flashBaseAddr   = MIMXRT106X_1st_FLEXSPI_AMBA_BASE;
            //uflTargetDesc->configOption.option0.U = 0xc0000006;
            //uflTargetDesc->configOption.option1.U = 0x0;
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
        memset((void *)&g_uflTargetDesc, 0U, sizeof(ufl_target_desc_t));
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

