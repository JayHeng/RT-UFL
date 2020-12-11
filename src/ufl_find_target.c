/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_find_target.h"
#include "core_scb.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* See Part number of core in Cortex-Mx Generic UG */
#define CORE_ID_CM33 (0xD21)
#define CORE_ID_CM7  (0xC27)

typedef enum _core_type
{
    kCoreType_Invalid = 0U,
    kCoreType_CM33    = 1U,
    kCoreType_CM7     = 2U,
} core_type_t;

typedef struct _rom_fingerprint
{
    uint32_t chipId;
    uint32_t content[3];
} rom_fingerprint_t;

#define ROM_FP_OFFSET1 (0x18000)
#define ROM_FP_OFFSET2 (0x1a000)
#define ROM_FP_OFFSET3 (0x1c000)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static core_type_t ufl_get_core_type(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static const rom_fingerprint_t s_romFingerprint[] = {
    {kChipId_RT5xx,  {0x00000000, 0x669ff643, 0xa8017026} },        // From ROM 2.0rc4
    {kChipId_RT6xx,  {0x09657b04, 0xf2406510, 0x240046a2} },        // From ROM 2.0rc5.1
    {kChipId_RT106x, {0x4608aa04, 0x2a01b132, 0x000000b4} },        // From ROM 1.0rc3
    {kChipId_RT117x, {0xf24a0110, 0x9909a810, 0xf44f6030} },        // From ROM 2.0rc4.1
};

/*******************************************************************************
 * Code
 ******************************************************************************/

static core_type_t ufl_get_core_type(void)
{
    core_type_t coreType = kCoreType_Invalid;
    uint32_t coreid = (SCB->CPUID & SCB_CPUID_PARTNO_Msk) >> SCB_CPUID_PARTNO_Pos;

    switch (coreid)
    {
        case CORE_ID_CM33:
            coreType = kCoreType_CM33;
            break;

        case CORE_ID_CM7:
            coreType = kCoreType_CM7;
            break;

        default:
            break;
    }

    return coreType;
}

rt_chip_id_t ufl_get_imxrt_chip_id(void)
{
    rt_chip_id_t chipId = kChipId_Invalid;
    core_type_t coreType;
    uint32_t rtRomBase = 0;

    coreType = ufl_get_core_type();
    if (kCoreType_CM33 == coreType)
    {
        rtRomBase = RT_ROM_BASE_CM33;
    }
    else if (kCoreType_CM7 == coreType)
    {
        rtRomBase = RT_ROM_BASE_CM7;
    }
    else
    {}

    do
    {
        uint32_t content[3];
        content[0] = *(uint32_t *)(rtRomBase + ROM_FP_OFFSET1);
        content[1] = *(uint32_t *)(rtRomBase + ROM_FP_OFFSET2);
        content[2] = *(uint32_t *)(rtRomBase + ROM_FP_OFFSET3);

        uint32_t idx = sizeof(s_romFingerprint) / sizeof(rom_fingerprint_t);
        while (idx--)
        {
            if (!memcmp(s_romFingerprint[idx].content, content, sizeof(content)))
            {
                chipId = (rt_chip_id_t)s_romFingerprint[idx].chipId;
                break;
            }
        }
    } while (0);

    return chipId;
}

