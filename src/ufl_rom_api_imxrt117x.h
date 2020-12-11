/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_ROM_API_IMXRT117X_H_
#define _UFL_ROM_API_IMXRT117X_H_

#include "ufl_flexspi_nor_flash_imxrt117x.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RT117X_ROM_API_TREE_ADDR (0x0021001cu)

typedef struct _bootloader_tree_imxrt117x
{
    void (*runBootloader)(void *arg);
    uint32_t version;
    const char *copyright;
    const flexspi_nor_flash_driver_imxrt117x_t *flexspiNorDriver;
    const uint32_t reserved0;
    const uint32_t reserved1;
    const uint32_t reserved2;
    const uint32_t reserved3;
    const uint32_t reserved4;
    const uint32_t reserved5;
    const uint32_t reserved6;
    const uint32_t reserved7;
} bootloader_tree_imxrt117x_t;

#define g_bootloaderTree_imxrt117x (*(bootloader_tree_imxrt117x_t **)(RT117X_ROM_API_TREE_ADDR))

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/



#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_ROM_API_IMXRT117X_H_ */
