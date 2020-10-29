/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_FLEXSPI_NOR_FLASH_IMXRT6XX_H_
#define _UFL_FLEXSPI_NOR_FLASH_IMXRT6XX_H_

#include "ufl_flexspi_nor_flash.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

//!@brief FLEXSPI Flash driver API Interface
typedef struct _flexspi_nor_flash_driver_imxrt6xx
{
    uint32_t version;
    status_t (*init)(uint32_t instance, void *config);
    status_t (*page_program)(uint32_t instance, void *config, uint32_t dstAddr, const uint32_t *src);
    status_t (*erase_all)(uint32_t instance, void *config);
    status_t (*erase)(uint32_t instance, void *config, uint32_t start, uint32_t length);
    status_t (*erase_sector)(uint32_t instance, void *config, uint32_t address);
    status_t (*erase_block)(uint32_t instance, void *config, uint32_t address);
    status_t (*get_config)(uint32_t instance, void *config, void *option);
    status_t (*read)(uint32_t instance, void *config, uint32_t *dst, uint32_t start, uint32_t bytes);
    status_t (*xfer)(uint32_t instance, void *xfer);
    status_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq);
    status_t (*set_clock_source)(uint32_t clockSrc);
    void (*config_clock)(uint32_t instance, uint32_t freqOption, uint32_t sampleClkMode);
} flexspi_nor_flash_driver_imxrt6xx_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/



#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_FLEXSPI_NOR_FLASH_IMXRT6XX_H_ */
