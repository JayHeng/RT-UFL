/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_FIND_TARGET_H_
#define _UFL_FIND_TARGET_H_

#include "core_scb.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum _rt_chip_id
{
    kChipId_Invalid = 0U,
    kChipId_RT6xx   = 1U,
    kChipId_RT106x  = 2U,
} rt_chip_id_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

rt_chip_id_t ufl_get_imxrt_chip_id(void);

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_FIND_TARGET_H_ */
