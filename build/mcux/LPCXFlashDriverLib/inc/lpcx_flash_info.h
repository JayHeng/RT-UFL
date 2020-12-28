//*****************************************************************************
// lpcx_flash_info.h
//
// LPCXpresso flash device description header
//*****************************************************************************
//
// Copyright 2014-2019, 2020 NXP
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

#ifndef _LPCX_FLASH_INFO_H
#define _LPCX_FLASH_INFO_H

/***********************************************************
 *
 *  WARNING: This file is intended to be identical in both the
 *           host debug stub and the target flash driver.
 *           Do not modify one without modifying the other.
 *
 ***********************************************************/

#include <stdint.h>


#ifndef MEM_FLASH_PTR
#define MEM_FLASH_PTR(type) type *
#endif

/* types of flash device */
#define UNKNOWN    0           // Unknown
#define ONCHIP     1           // On-chip Flash Memory
#define EXT8BIT    2           // External Flash Device on 8-bit  Bus
#define EXT16BIT   3           // External Flash Device on 16-bit Bus
#define EXT32BIT   4           // External Flash Device on 32-bit Bus
#define EXTSPI     5           // External Flash Device on SPI
#define INTIAP     8           // On chip Flash memory using IAP lib
#define INTKFMM    9           // On chip Kinetis Flash Memory Module
#define EXTSPIJ    10          // External device on SPI - SFDP JEDEC

#define SECTOR_NUM 512         // Max Number of Sector Items
#define PAGE_MAX   65536       // Max Page Size for Programming

struct FlashSectors  {
  uint32_t   szSector;         // Sector Size in Bytes
  uint32_t AddrSector;         // Address of Sector
};

#define SECTOR_END 0xFFFFFFFF, 0xFFFFFFFF

/* This definition is compatible with ARM's definition of the same data
   structure distributed in header FlashOS.h
*/
typedef struct   {
   uint16_t      Vers;    // Version Number and Architecture
   char  DevName[64];    // Device Name and Description
   uint16_t   DevType;    // Device Type: ONCHIP, EXT8BIT, EXT16BIT, ...
   uint32_t    DevAdr;    // Default Device Start Address
   uint32_t     szDev;    // Total Size of Device
   uint32_t    szPage;    // Programming Page Size
   uint32_t Extension;    // Extension field (1st byte - optional true page size)
   uint8_t   valEmpty;    // Content of Erased Memory

   uint32_t    toProg;    // Time Out of Program Page Function
   uint32_t   toErase;    // Time Out of Erase Sector Function

   struct FlashSectors sectors[];
} FlashDeviceV_t;


/* Providing Extension field values */

/*! Value for Extension field holding no extension information */
#define FLASH_DRV_UNDEFINED (0)

/*! Value for Extension field holding only the "real" page size as a power of
 *  two - this is assumed to be the same unit that is prototected against
 *  reading in an erased flash which has the OPMAP_ERASED_WO bit set. */
#define FLASH_DRV_REALPAGE_LN2(szln2) (((szln2) & 0xFF) << 0)

/*! Return is the real page size - it is undefined if 1 is returned */
#define FLASH_DRV_GET_REALPAGE(extension) (1<<((extension) & 0xff))

/* We may in the future define additional macros for inclusion here such as:
 *   #define FLASH_DRV_FUTUREDATA(info) (((info) & 0xFF) << 8)
 *   #define FLASH_DRV_GET_FUTUREDATA(extension) (((extension)>>8) & 0xff)
 *
 * The correct way to use these macros in a flash driver is to OR together
 * macro invocatons to provide a value for the Extension field. e.g.
 *     ...
 *     <szPage>,
 *     FLASH_DRV_REALPAGE_LN2(2) | FLASH_DRV_FUTUREDATA(99),
 *     <valEmpty>,
 *     ...
 */


#define FLASH_DRV_SZVER(size_ln2, major, minor) \
	    (0xffff & (((major)<<8) | ((~(size_ln2))<<13) | (minor)))
// size 128 gives 0 size field

// Driver Versions, do not modify!

#define FLASH_DRV_VER(minor) FLASH_DRV_SZVER(6/*64*/,  1, minor)


#endif /* _LPCX_FLASH_INFO_H */
