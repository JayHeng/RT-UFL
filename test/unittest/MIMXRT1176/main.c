/*
* The Clear BSD License
* Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
* All rights reserved.
*/

#include <stdio.h>
#include "ufl_rom_api.h"
/***********************************************************************************************************************
 *  Definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 *  Variables
 **********************************************************************************************************************/

flexspi_nor_config_t flashConfig = {.pageSize = 0x400};

/***********************************************************************************************************************
 *  Prototypes
 **********************************************************************************************************************/

/***********************************************************************************************************************
 *  Codes
 **********************************************************************************************************************/

int main()
{
    ufl_full_setup();
    uint32_t instance = g_uflTargetDesc.flexspiInstance;
    serial_nor_config_option_t *configOption = &g_uflTargetDesc.configOption;

    memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));

    status_t status = flexspi_nor_auto_config(instance, &flashConfig, configOption);
    if (!status)
    {
        status = flexspi_nor_flash_erase(instance, &flashConfig, 0x1000, flashConfig.sectorSize);
        if (!status)
        {
            while (1);
        }
    }

    while (1);
}
