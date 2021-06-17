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
#include "cmsis_compiler.h"
#if defined(CPU_MIMXRT1176DVMAA_cm7)
#include "fsl_common.h"
#endif
#include "MIMXRT_FLEXSPI.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ALIGN_DOWN(x, a) ((x) & -(a))
#define ALIGN_UP(x, a) (-(-(x) & -(a)))

#define FREQ_1MHz (1UL * 1000 * 1000)

/*! @brief Generic status return codes. */
#if !defined(CPU_MIMXRT1176DVMAA_cm7)
/*! @brief Construct a status code value from a group and code number. */
#define MAKE_STATUS(group, code) ((((group)*100) + (code)))

enum
{
    kStatus_Success = MAKE_STATUS(0, 0),
    kStatus_InvalidArgument = MAKE_STATUS(0, 4),
};

enum
{
    kStatusGroup_FLEXSPI = 70,
};

typedef uint32_t status_t;
#endif

#define MEM_WriteU16(addr, value)  (*((volatile uint16_t *)(addr)) = value)
#define MEM_ReadU16(addr)          (*((volatile uint16_t *)(addr)))

#define MEM_WriteU32(addr, value)  (*((volatile uint32_t *)(addr)) = value)
#define MEM_ReadU32(addr)          (*((volatile uint32_t *)(addr)))

typedef enum _rt_chip_id
{
    kChipId_Invalid    = 0xFFU,
    kChipId_RT5xx      = 1U,
    kChipId_RT6xx      = 2U,
    kChipId_RT101x     = 3U,
    kChipId_RT1015     = 4U,
    kChipId_RT102x     = 5U,
    kChipId_RT1024_SIP = 6U,
    kChipId_RT105x     = 7U,
    kChipId_RT106x     = 8U,
    kChipId_RT1064_SIP = 9U,
    kChipId_RT117x     = 10U,
} rt_chip_id_t;

#define RT_ROM_BASE_CM33 (0x03000000u)
#define RT_ROM_BASE_CM7  (0x00200000u)

#define FP_FLAG_ADDR     (0x0000FFFCu)
#define FP_FLAG_RT117X   (0x5AA60FF0u)

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

status_t ufl_full_setup(void);

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_COMMON_H_ */
