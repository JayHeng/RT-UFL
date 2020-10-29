/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_rom_api.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern flexspi_nor_flash_driver_t g_flexspiNorDriver;

/*******************************************************************************
 * Variables
 ******************************************************************************/



/*******************************************************************************
 * Code
 ******************************************************************************/

status_t flexspi_nor_flash_init(uint32_t instance, void *config)
{
    return g_flexspiNorDriver.init(instance, config);
}

status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        void *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src)
{
    return g_flexspiNorDriver.page_program(instance, config, dstAddr, src);
}

status_t flexspi_nor_flash_erase_all(uint32_t instance, void *config)
{
    return g_flexspiNorDriver.erase_all(instance, config);
}

status_t flexspi_nor_get_config(uint32_t instance, void *config, void *option)
{
    return g_flexspiNorDriver.get_config(instance, config, option);
}

status_t flexspi_nor_flash_erase(uint32_t instance, void *config, uint32_t start, uint32_t length)
{
    return g_flexspiNorDriver.erase(instance, config, start, length);
}

status_t flexspi_nor_flash_read(uint32_t instance, void *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
    return g_flexspiNorDriver.read(instance, config, dst, start, bytes);
}

status_t flexspi_nor_set_clock_source(uint32_t clockSrc)
{
    return g_flexspiNorDriver.set_clock_source(clockSrc);
}

status_t flexspi_nor_auto_config(uint32_t instance, void *config, void *option)
{
    // Wait until the FLEXSPI is idle
    register uint32_t delaycnt = 10000u;
    while(delaycnt--)
    {
    }
    status_t status = flexspi_nor_get_config(instance, config, option);
    if (status != 0)
    {
        return status;
    }
    return flexspi_nor_flash_init(instance, config);
}


