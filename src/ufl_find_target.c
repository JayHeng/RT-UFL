/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_find_target.h"
#if defined(CPU_MIMXRT1176DVMAA_cm7)
#include "core_cm7.h"
#else
#include "core_scb.h"
#endif
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

#define ROM_FP_BASE_CM33    (0x10000)
#define ROM_FP_BASE_CM7     (0x0)
#define ROM_FP_BASE_RT117X  (0x10000)

#define ROM_FP_OFFSET1 (0x8000)
#define ROM_FP_OFFSET2 (0xa000)
#define ROM_FP_OFFSET3 (0xc000)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static core_type_t ufl_get_core_type(void);
static rt_chip_id_t ufl_check_imxrt_sip(rt_chip_id_t chipId);

/*******************************************************************************
 * Variables
 ******************************************************************************/

// Keep ROM contents of three positions
static const rom_fingerprint_t s_romFingerprint[] = {
    // RT5xx ROM Size 192KB
    {kChipId_RT5xx,  {0x00000000, 0x669ff643, 0xa8017026} },        // From ROM 2.0rc4
    // RT6xx ROM Size 256KB
    {kChipId_RT6xx,  {0x09657b04, 0xf2406510, 0x240046a2} },        // From ROM 2.0rc5.1

    // RT101x ROM Size 64KB
    {kChipId_RT101x, {0x2805eb03, 0xf88d10c9, 0xf810f000} },        // From ROM 1.0rc3
    // RT102x ROM Size 96KB
    {kChipId_RT102x, {0x4038f88d, 0xe9dd9a03, 0x7831688a} },        // From ROM 1.0rc4
    // RT105x ROM Size 96KB
    {kChipId_RT105x, {0x9e016037, 0x2101eb10, 0xf04fd502} },        // From ROM 1.1rc3
    // RT106x ROM Size 128KB
    {kChipId_RT106x, {0xb0893000, 0x80dbf000, 0xf2c44100} },        // From ROM 1.0rc3
    // RT117x ROM Size 256KB
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

static rt_chip_id_t ufl_check_imxrt_sip(rt_chip_id_t chipId)
{
    if (chipId == kChipId_RT102x)
    {
        uint32_t sipBitMask = 0x100000U;
        uint32_t ocotpCfg3 = *(uint32_t *)0x401f4440;
        if (ocotpCfg3 & sipBitMask)
        {
            chipId = kChipId_RT1024_SIP;
        }
    }
    else if (chipId == kChipId_RT106x)
    {
        uint32_t sipEnBitMask = 0x100000U;
        uint32_t ocotpCfg3 = *(uint32_t *)0x401f4440;
        if (ocotpCfg3 & sipEnBitMask)
        {
            chipId = kChipId_RT1064_SIP;
        }
    }

    return chipId;
}

rt_chip_id_t ufl_get_imxrt_chip_id(void)
{
    rt_chip_id_t chipId = kChipId_Invalid;
    core_type_t coreType;
    uint32_t rtRomFpBase = 0;

    // Get cortex-m core type by SCB->CPUID.
    coreType = ufl_get_core_type();
    // For i.MXRTxxx and i.MXRT1xxx, ROM base addresses are different
    if (kCoreType_CM33 == coreType)
    {
        rtRomFpBase = RT_ROM_BASE_CM33;
        rtRomFpBase += ROM_FP_BASE_CM33;
    }
    else if (kCoreType_CM7 == coreType)
    {
        uint32_t rt117xFlag = *(uint32_t *)FP_FLAG_ADDR;
        rtRomFpBase = RT_ROM_BASE_CM7;
        if (rt117xFlag == FP_FLAG_RT117X)
        {
            rtRomFpBase += ROM_FP_BASE_RT117X;
        }
        else
        {
            rtRomFpBase += ROM_FP_BASE_CM7;
        }
    }
    else
    {}

    do
    {
        uint32_t content[3];
        content[0] = *(uint32_t *)(rtRomFpBase + ROM_FP_OFFSET1);
        content[1] = *(uint32_t *)(rtRomFpBase + ROM_FP_OFFSET2);
        content[2] = *(uint32_t *)(rtRomFpBase + ROM_FP_OFFSET3);

        // Find dedicated i.MXRT part number according to ROM contents
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

    chipId = ufl_check_imxrt_sip(chipId);

    return chipId;
}

