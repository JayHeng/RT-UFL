/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_ROM_API_H_
#define _UFL_ROM_API_H_

#include "ufl_flexspi_nor_flash.h"
#include "ufl_rom_api_imxrt6xx.h"
#include "ufl_rom_api_imxrt106x.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*
 *  Serial NOR configuration block
 */
typedef struct _flexspi_nor_config
{
    uint32_t cfg0[20];
    uint32_t sflashA1Size;               //!< [0x050-0x053] Size of Flash connected to A1
    uint32_t sflashA2Size;               //!< [0x054-0x057] Size of Flash connected to A2
    uint32_t sflashB1Size;               //!< [0x058-0x05b] Size of Flash connected to B1
    uint32_t sflashB2Size;               //!< [0x05c-0x05f] Size of Flash connected to B2
    uint32_t cfg1[88];
    uint32_t pageSize;              //!< Page size of Serial NOR
    uint32_t sectorSize;            //!< Sector size of Serial NOR
    uint32_t cfg2[2];
    uint32_t blockSize;             //!< Block size
    uint32_t cfg3[11];
} flexspi_nor_config_t;

/*
 * Serial NOR Configuration Option
 */
typedef struct _serial_nor_config_option
{
    uint32_t option0;
    uint32_t option1;
} serial_nor_config_option_t;

//!@brief FLEXSPI Flash driver API Interface
typedef struct _flexspi_nor_flash_driver
{
    status_t (*init)(uint32_t instance, void *config);
    status_t (*page_program)(uint32_t instance, void *config, uint32_t dst_addr, const uint32_t *src);
    status_t (*erase_all)(uint32_t instance, void *config);
    status_t (*erase)(uint32_t instance, void *config, uint32_t start, uint32_t lengthInBytes);
    status_t (*read)(uint32_t instance, void *config, uint32_t *dst, uint32_t addr, uint32_t lengthInBytes);
    status_t (*set_clock_source)(uint32_t clockSrc);
    status_t (*get_config)(uint32_t instance, void *config, void *option);
} flexspi_nor_flash_driver_t;

typedef struct _target_desc
{
    uint32_t imxrtChipId;
    uint32_t flexspiInstance;
    uint32_t flashBaseAddr;
    serial_nor_config_option_t configOption;
    flexspi_nor_flash_driver_t flashDriver;
    bool isFlashPageProgram;
} ufl_target_desc_t;

extern ufl_target_desc_t g_uflTargetDesc;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

status_t flexspi_nor_flash_init(uint32_t instance, void *config);

status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        void *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src);

status_t flexspi_nor_flash_erase_all(uint32_t instance, void *config);

status_t flexspi_nor_get_config(uint32_t instance, void *config, void *option);

status_t flexspi_nor_flash_erase(uint32_t instance, void *config, uint32_t start, uint32_t length);

status_t flexspi_nor_flash_read(uint32_t instance, void *config, uint32_t *dst, uint32_t start, uint32_t bytes);

status_t flexspi_nor_set_clock_source(uint32_t clockSrc);

status_t flexspi_nor_auto_config(uint32_t instance, void *config, void *option);

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_ROM_API_H_ */
