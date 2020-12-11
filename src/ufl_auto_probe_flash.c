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



/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern flexspi_nor_config_t flashConfig;

/*******************************************************************************
 * Variables
 ******************************************************************************/



/*******************************************************************************
 * Code
 ******************************************************************************/

status_t ufl_auto_probe(void)
{
    memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));
    status_t status = flexspi_nor_auto_config(g_uflTargetDesc.flexspiInstance, &flashConfig, &g_uflTargetDesc.configOption);

    return status;
}

