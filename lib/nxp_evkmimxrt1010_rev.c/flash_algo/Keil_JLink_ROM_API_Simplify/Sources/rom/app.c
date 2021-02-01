/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "flexspi_hardware_init_rt1010.h"
#include "bl_flexspi.h"
#include "bl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    flexspi_iomux_config_rt1010(instance, config);
}

void flexspi_update_padsetting(flexspi_mem_config_t *config, uint32_t driveStrength)
{
    flexspi_update_padsetting_rt1010(config, driveStrength);
}

void flexspi_clock_config(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
{   
    flexspi_clock_config_rt1010(instance, freq, sampleClkMode);   
}

status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    
    status = flexspi_set_failsafe_setting_rt1010(config);
    
    return status;
}

status_t flexspi_get_max_supported_freq(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;

    status = flexspi_get_max_supported_freq_rt1010(instance, freq, clkMode);

    return status;
}

void flexspi_sw_delay_us(uint64_t us)
{
    uint32_t ticks_per_us = CLOCK_GetCPUFreq_RT1010() / 1000000;
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
    
    status = flexspi_get_clock_rt1010(instance, type, freq);

    return status;
}

void flexspi_clock_gate_enable(uint32_t instance)
{
    flexspi_clock_gate_enable_rt1010(instance);
}

void flexspi_clock_gate_disable(uint32_t instance)
{
    flexspi_clock_gate_disable_rt1010(instance);
}

status_t flexspi_nor_write_persistent(const uint32_t data)
{	
    flexspi_nor_write_persistent_rt1010(data);
	
    return kStatus_Success;
}

status_t flexspi_nor_read_persistent(uint32_t *data)
{
    flexspi_nor_read_persistent_rt1010(data);

    return kStatus_Success;
}

