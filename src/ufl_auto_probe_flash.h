/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_AUTO_PROBE_FLASH_H_
#define _UFL_AUTO_PROBE_FLASH_H_

#include "ufl_common.h"
#include "ufl_flexspi_nor_flash.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

enum
{
    kSerialNorCfgOption_MaxFreq = 0x5,
};


/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

status_t ufl_auto_probe(void);

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_AUTO_PROBE_FLASH_H_ */
