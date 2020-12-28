//*****************************************************************************
// lpcx_flash_driver.h
//
// LPCXpresso flash driver header (Messaged)
//*****************************************************************************
//
// Copyright 2014-2015, 2018-2019, 2020 NXP
// All rights reserved.
//
// NXP Confidential. This software is owned or controlled by NXP and may only be 
// used strictly in accordance with the applicable license terms.  
//
// By expressly accepting such terms or by downloading, installing, activating 
// and/or otherwise using the software, you are agreeing that you have read, and 
// that you agree to comply with and are bound by, such license terms.  
// 
// If you do not agree to be bound by the applicable license terms, then you may not 
// retain, install, activate or otherwise use the software.
//*****************************************************************************

#ifndef _LPCX_FLASH_DRIVER_H
#define _LPCX_FLASH_DRIVER_H

/***********************************************************
 *
 *  WARNING: This file is intended to be identical in both the
 *           host debug stub and the target flash driver.
 *           Do not modify one without modifying the other.
 *
 ***********************************************************/
#include <stdint.h>
#include "lpcx_flash_msgif.h"

#define WEAK                 __attribute__ ((weak))
#define FLASHDEV_SECTION     __attribute__ ((section(".FlashDevice")))
#define EXTERN_MEMORY_DEVICE __attribute__ ((section(".MemoryDevice")))

#define VERS 1                             // struct FlashDevice minor version
#define FLASH_DRV_VERS FLASH_DRV_VER(VERS) // FlashDriver Version

#ifdef __cplusplus
 extern "C" {
#endif

// ===========================
// Flash Programming Functions
// ===========================

 /*
  *  Initialize Flash Programming Functions
  *    Return Value:   0 - OK,  1 - Failed
  */
typedef uint32_t Init_fn_t (void);

  /*
   *  De-Initialize Flash Programming Functions
   *    Return Value:   0 - OK,  1 - Failed
   */
typedef uint32_t UnInit_fn_t (void);

  /*
   *  Erase complete Flash Memory
   *    Return Value:   0 - OK,  1 - Failed
   */
typedef uint32_t EraseChip_fn_t (void);

  /*
   *  Erase Sector in Flash Memory
   *    Parameter:      adr:  Sector Address
   *                numsecs: Number of sectors
   *    Return Value:   0 - OK,  1 - Failed
   */
typedef uint32_t EraseSectors_fn_t (uint32_t adr, uint32_t numsecs);

  /*
   *  Program Page in Flash Memory
   *    Parameter:      adr:  Page Start Address
   *                    sz:   Page Size
   *                    buf:  Page Data
   *    Return Value:   0 - OK,  1 - Failed
   */
typedef uint32_t ProgramPage_fn_t (uint32_t adr, uint32_t sz, uint8_t *buf);

/*
 *  Verify Flash Contents
 *    Parameter:      adr:  Start Address
 *                    sz:   Size (in bytes)
 *                    buf:  Data
 *    Return Value:   (adr+sz) - OK, Failed Address
 */
typedef uint32_t Verify_fn_t (uint32_t adr, uint32_t sz, uint8_t *buf);

/*
 *  Checksum Sector Contents in Flash Memory
 *    Parameter:      adr:  Sector Address
 *                    numsecs: Number of sectors
 *                    data:  checksum data
 *    Return Value:   0 - OK,  1 - Failed
 */
typedef uint32_t ChecksumSectors_fn_t (uint32_t adr, uint32_t numsecs, ChecksumData_t *data);

/* NB - weak references to optional driver functions */
     
     Init_fn_t             Init;
     UnInit_fn_t           UnInit;
WEAK EraseChip_fn_t        EraseChip;
     EraseSectors_fn_t     EraseSectors;
     ProgramPage_fn_t      ProgramPage;
WEAK Verify_fn_t           Verify;
#if defined(SECTOR_HASHING)
WEAK ChecksumSectors_fn_t  ChecksumSectors;
#endif

/* weak references to messaged driver function proformas */
     
ServiceMessages_fn_t  ServiceMessages;

/* The flash device instance */
extern FlashDeviceV_t FlashDevice;

     
#ifdef __cplusplus
}
#endif


#endif  // _LPCX_FLASH_DRIVER_H
