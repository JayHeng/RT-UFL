/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_FLEXSPI_NOR_FLASH_IMXRT101X_H_
#define _UFL_FLEXSPI_NOR_FLASH_IMXRT101X_H_

#include "ufl_flexspi_nor_flash.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MIMXRT101X_1st_FLEXSPI_INSTANCE      (0)
#define MIMXRT101X_1st_FLEXSPI_BASE          (0x400A0000u)
#define MIMXRT101X_1st_FLEXSPI_AMBA_BASE     (0x60000000U)

#define MIMXRT101X_2nd_FLEXSPI_INSTANCE      (1)
#define MIMXRT101X_2nd_FLEXSPI_BASE          (0x0)
#define MIMXRT101X_2nd_FLEXSPI_AMBA_BASE     (0x0)

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/



#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_FLEXSPI_NOR_FLASH_IMXRT101X_H_ */
