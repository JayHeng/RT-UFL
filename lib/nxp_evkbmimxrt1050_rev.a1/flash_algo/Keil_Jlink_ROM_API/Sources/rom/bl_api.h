/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __BL_API_H__
#define __BL_API_H__

#include "fsl_device_registers.h"
#include "fsl_clock.h"
#include "flexspi_nor_flash.h"
#include "fsl_rtwdog.h"
#include "fsl_wdog.h"
#include <string.h>

typedef struct
{
    uint32_t version;
    status_t (*init)(uint32_t instance, flexspi_nor_config_t *config);
    status_t (*program)(uint32_t instance, flexspi_nor_config_t *config, uint32_t dst_addr, const uint32_t *src);
    status_t (*erase_all)(uint32_t instance, flexspi_nor_config_t *config);
    status_t (*erase)(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t lengthInBytes);
    status_t (*read)(
        uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t addr, uint32_t lengthInBytes);
    void (*clear_cache)(uint32_t instance);
    status_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
    status_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber);
} flexspi_nor_driver_interface_t;

typedef struct
{
    void (*runBootloader)(void *arg);
    const uint32_t version;
    const char * copyright;
    const uint32_t *reserved0;
    const flexspi_nor_driver_interface_t* flexSpiNorDriver; //!< FlexSPI NOR Flash API
    const uint32_t *reserved1; 
    const uint32_t *reserved2; 
    const uint32_t *reserved3; 
} bootloader_api_entry_t;

enum
{
    kEnterBootloader_Tag                   = 0xEB,
    kEnterBootloader_Mode_Default          = 0,
    kEnterBootloader_Mode_SerialDownloader = 1,

    kEnterBootloader_SerialInterface_Auto = 0,
    kEnterBootloader_SerialInterface_USB  = 1,
    kEnterBootloader_SerialInterface_UART = 2,

    kEnterBootloader_ImageIndex_Max = 3,
};

typedef union
{
    struct
    {
        uint32_t imageIndex : 4;
        uint32_t reserved : 12;
        uint32_t serialBootInterface : 4;
        uint32_t bootMode : 4;
        uint32_t tag : 8;
    } B;
    uint32_t U;
} run_bootloader_ctx_t;

#define g_bootloaderTree (*(bootloader_api_entry_t **)0x0020001c)

void bl_api_init(void);

#endif //__BL_API_H__
