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

#define FLASH_BASE_ADDR    0x60000000
#define FLEXSPI_INSTANCE_0 (0)
#define FLEXSPI_INSTANCE_SEL FLEXSPI_INSTANCE_0

/***********************************************************************************************************************
 *  Variables
 **********************************************************************************************************************/

flexspi_nor_config_t flashConfig = {.pageSize = 0x100};

/***********************************************************************************************************************
 *  Prototypes
 **********************************************************************************************************************/

/***********************************************************************************************************************
 *  Codes
 **********************************************************************************************************************/

int main()
{
    ufl_full_setup();

    memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));

    serial_nor_config_option_t configOption;
    configOption.option0 = 0xc0000006;
    configOption.option1 = 0;

    status_t status = flexspi_nor_auto_config(FLEXSPI_INSTANCE_SEL, &flashConfig, &configOption);
    if (!status)
    {
        status = flexspi_nor_flash_erase(FLEXSPI_INSTANCE_SEL, &flashConfig, 0x1000, flashConfig.sectorSize);
        if (!status)
        {
            while (1);
        }
    }

    while (1);
}
