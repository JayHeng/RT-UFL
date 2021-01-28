/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "ufl_rom_api.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    g_uflTargetDesc.flexspiBsp.flexspi_iomux_config(instance, config);
}

void flexspi_update_padsetting(flexspi_mem_config_t *config, uint32_t driveStrength)
{
    g_uflTargetDesc.flexspiBsp.flexspi_update_padsetting(config, driveStrength);
}

void flexspi_clock_config(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
{   
    g_uflTargetDesc.flexspiBsp.flexspi_clock_config(instance, freq, sampleClkMode);   
}

status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    
    status = g_uflTargetDesc.flexspiBsp.flexspi_set_failsafe_setting(config);
    
    return status;
}

status_t flexspi_get_max_supported_freq(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;

    status = g_uflTargetDesc.flexspiBsp.flexspi_get_max_supported_freq(instance, freq, clkMode);

    return status;
}

void flexspi_sw_delay_us(uint64_t us)
{
    uint32_t ticks_per_us = g_uflTargetDesc.flexspiBsp.CLOCK_GetCPUFreq() / 1000000;
    while(us--)
    {
        volatile uint32_t ticks = ticks_per_us / 4;
        while(ticks--)
        {
            __NOP();
        }
    }
}

status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    status_t status = kStatus_Success;
    
    status = g_uflTargetDesc.flexspiBsp.flexspi_get_clock(instance, type, freq);

    return status;
}

void flexspi_clock_gate_enable(uint32_t instance)
{
    g_uflTargetDesc.flexspiBsp.flexspi_clock_gate_enable(instance);
}

void flexspi_clock_gate_disable(uint32_t instance)
{
    g_uflTargetDesc.flexspiBsp.flexspi_clock_gate_disable(instance);
}

status_t flexspi_nor_drv_write_persistent(const uint32_t data)
{   
    g_uflTargetDesc.flexspiBsp.flexspi_nor_write_persistent(data);
    
    return kStatus_Success;
}

status_t flexspi_nor_drv_read_persistent(uint32_t *data)
{
    g_uflTargetDesc.flexspiBsp.flexspi_nor_read_persistent(data);

    return kStatus_Success;
}
