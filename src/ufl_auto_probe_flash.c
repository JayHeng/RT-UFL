/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_auto_probe_flash.h"
#include "ufl_rom_api.h"
#include "MIMXRT_FLEXSPI.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

enum
{
    kSerialNorCfgOption_Tag = 0x0c,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern flexspi_nor_config_t flashConfig;

/*******************************************************************************
 * Variables
 ******************************************************************************/

static const serial_nor_config_option_t s_flashConfigOpt[] = {
    {.option0.U = 0xc0000006, .option1.U = 0x00000000},
    {.option0.U = 0xc0403004, .option1.U = 0x00000000},
    {.option0.U = 0xc1503051, .option1.U = 0x20000014},
};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void flexspi_sw_reset(FLEXSPI_Type *base)
{
    //base->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
    base->MCR0 |= 0x1U;
    while (base->MCR0 & 0x1U)
    {
    }
}

// Wait until FlexSPI controller becomes idle
static void flexspi_wait_idle(FLEXSPI_Type *base)
{
    //while (!(base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK))
    while (!(base->STS0 & 0x2U))
    {
    }
}

static void flexspi_error_handler(FLEXSPI_Type *base)
{
    flexspi_sw_reset(base);
    flexspi_wait_idle(base);
}

status_t ufl_auto_probe(void)
{
    status_t status = kStatus_Success;
    uint32_t instance = g_uflTargetDesc.flexspiInstance;

    // If we have found proper option before, we just reuse it.
    if (g_uflTargetDesc.configOption.option0.B.tag == kSerialNorCfgOption_Tag)
    {
        memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));
        status = flexspi_nor_auto_config(instance, &flashConfig, &g_uflTargetDesc.configOption);
    }
    else
    {
        FLEXSPI_Type *base = (FLEXSPI_Type *)g_uflTargetDesc.flexspiBaseAddr;
        const serial_nor_config_option_t *option;
        uint32_t retryCnt = sizeof(s_flashConfigOpt) / sizeof(serial_nor_config_option_t);

        // Try all kinds of flash config options until we find proper option.
        for (uint32_t idx = 0; idx < retryCnt; idx++)
        {
            register uint32_t delaycnt;
            memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));

            // Wait until the FLEXSPI is idle
            delaycnt = 10000u;
            while(delaycnt--)
            {
            }
            option = &s_flashConfigOpt[idx];
            status = flexspi_nor_get_config(instance, &flashConfig, (void *)option);
            if (status != kStatus_Success)
            {
                flexspi_error_handler(base);
                continue;
            }
            status = flexspi_nor_flash_init(instance, &flashConfig);
            if (status != kStatus_Success)
            {
                flexspi_error_handler(base);
                continue;
            }
            status = flexspi_nor_flash_erase(instance, &flashConfig, 0x0, flashConfig.sectorSize);
            if (status != kStatus_Success)
            {
                flexspi_error_handler(base);
                continue;
            }
            status = flexspi_nor_flash_page_program(instance, &flashConfig, 0x0, (uint32_t *)&flashConfig);
            if (status == kStatus_Success)
            {
                g_uflTargetDesc.configOption.option0.U = option->option0.U;
                g_uflTargetDesc.configOption.option1.U = option->option1.U;
                break;
            }
        }
    }

    return status;
}

