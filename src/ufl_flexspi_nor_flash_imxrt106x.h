/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_FLEXSPI_NOR_FLASH_IMXRT106X_H_
#define _UFL_FLEXSPI_NOR_FLASH_IMXRT106X_H_

#include "ufl_flexspi_nor_flash.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct _flexspi_nor_flash_driver_imxrt106x
{
    uint32_t version;
    status_t (*init)(uint32_t instance, void *config);
    status_t (*page_program)(uint32_t instance, void *config, uint32_t dst_addr, const uint32_t *src);
    status_t (*erase_all)(uint32_t instance, void *config);
    status_t (*erase)(uint32_t instance, void *config, uint32_t start, uint32_t lengthInBytes);
    status_t (*read)(uint32_t instance, void *config, uint32_t *dst, uint32_t addr, uint32_t lengthInBytes);
    void (*clear_cache)(uint32_t instance);
    status_t (*xfer)(uint32_t instance, void *xfer);
    status_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber);
    status_t (*get_config)(uint32_t instance, void *config, void *option);
} flexspi_nor_flash_driver_imxrt106x_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/



#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_FLEXSPI_NOR_FLASH_IMXRT106X_H_ */
