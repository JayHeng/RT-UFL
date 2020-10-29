/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_ROM_API_IMXRT106X_H_
#define _UFL_ROM_API_IMXRT106X_H_

#include "ufl_flexspi_nor_flash_imxrt106x.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RT106X_ROM_API_TREE_ADDR (0x0020001cu)

typedef struct _bootloader_tree_imxrt106x
{
    const uint32_t version;
    const char *copyright;
    void (*runBootloader)(void *arg);
    const uint32_t reserved0;
    const flexspi_nor_flash_driver_imxrt106x_t *flexspiNorDriver;
    const uint32_t reserved1;
    const uint32_t reserved2;
    const uint32_t reserved3;
    const uint32_t reserved4;
    const uint32_t reserved5;
} bootloader_tree_imxrt106x_t;

#define g_bootloaderTree_imxrt106x (*(bootloader_tree_imxrt106x_t **)(RT106X_ROM_API_TREE_ADDR))

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/



#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_ROM_API_IMXRT106X_H_ */
