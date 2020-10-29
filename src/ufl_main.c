/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_find_target.h"
#include "ufl_rom_api.h"
#include "ufl_hardware_init.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void ufl_fill_flash_api(rt_chip_id_t chipId);
static void ufl_init_hardware(rt_chip_id_t chipId);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static rt_chip_id_t s_imxrtChipId = kChipId_Invalid;
flexspi_nor_flash_driver_t g_flexspiNorDriver = {.init = NULL};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void ufl_fill_flash_api(rt_chip_id_t chipId)
{
    switch (chipId)
    {
        case kChipId_RT6xx:
            g_flexspiNorDriver.init             = g_bootloaderTree_imxrt6xx->flexspiNorDriver->init;
            g_flexspiNorDriver.page_program     = g_bootloaderTree_imxrt6xx->flexspiNorDriver->page_program;
            g_flexspiNorDriver.erase_all        = g_bootloaderTree_imxrt6xx->flexspiNorDriver->erase_all;
            g_flexspiNorDriver.erase            = g_bootloaderTree_imxrt6xx->flexspiNorDriver->erase;
            g_flexspiNorDriver.read             = g_bootloaderTree_imxrt6xx->flexspiNorDriver->read;
            g_flexspiNorDriver.set_clock_source = g_bootloaderTree_imxrt6xx->flexspiNorDriver->set_clock_source;
            g_flexspiNorDriver.get_config       = g_bootloaderTree_imxrt6xx->flexspiNorDriver->get_config;
            break;

        case kChipId_RT106x:
            g_flexspiNorDriver.init             = g_bootloaderTree_imxrt106x->flexspiNorDriver->init;
            g_flexspiNorDriver.page_program     = g_bootloaderTree_imxrt106x->flexspiNorDriver->page_program;
            g_flexspiNorDriver.erase_all        = g_bootloaderTree_imxrt106x->flexspiNorDriver->erase_all;
            g_flexspiNorDriver.erase            = g_bootloaderTree_imxrt106x->flexspiNorDriver->erase;
            g_flexspiNorDriver.read             = g_bootloaderTree_imxrt106x->flexspiNorDriver->read;
            g_flexspiNorDriver.set_clock_source = NULL;
            g_flexspiNorDriver.get_config       = g_bootloaderTree_imxrt106x->flexspiNorDriver->get_config;
            break;

        default:
            break;
    }
}

static void ufl_init_hardware(rt_chip_id_t chipId)
{
    switch (chipId)
    {
        case kChipId_RT6xx:
            ufl_init_hardware_imxrt6xx();
            break;

        case kChipId_RT106x:
            ufl_init_hardware_imxrt106x();
            break;

        default:
            break;
    }
}

void ufl_full_setup(void)
{
    s_imxrtChipId = ufl_get_imxrt_chip_id();

    ufl_fill_flash_api(s_imxrtChipId);
    ufl_init_hardware(s_imxrtChipId);
}

