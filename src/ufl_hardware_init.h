/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_HARDWARE_INIT_H_
#define _UFL_HARDWARE_INIT_H_

#include "ufl_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

void ufl_init_hardware_imxrt5xx(void);

void ufl_init_hardware_imxrt6xx(void);

void ufl_init_hardware_imxrt106x(void);

void ufl_init_hardware_imxrt117x(void);

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_HARDWARE_INIT_H_ */
