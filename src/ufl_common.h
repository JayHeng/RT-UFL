/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_COMMON_H_
#define _UFL_COMMON_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Generic status return codes. */
enum
{
    kStatus_Success = 0U,
};

typedef uint32_t status_t;

#define MEM_WriteU16(addr, value)  (*((volatile uint16_t *)(addr)) = value)
#define MEM_ReadU16(addr)          (*((volatile uint16_t *)(addr)))

#define MEM_WriteU32(addr, value)  (*((volatile uint32_t *)(addr)) = value)
#define MEM_ReadU32(addr)          (*((volatile uint32_t *)(addr)))

typedef enum _rt_chip_id
{
    kChipId_Invalid = 0xFFU,
    kChipId_RT5xx   = 1U,
    kChipId_RT6xx   = 2U,
    kChipId_RT106x  = 3U,
    kChipId_RT117x  = 4U,
} rt_chip_id_t;

#define RT_ROM_BASE_CM33 (0x03000000u)
#define RT_ROM_BASE_CM7  (0x00200000u)

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

void ufl_full_setup(void);

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_COMMON_H_ */
