//*****************************************************************************
// FlashDev.c
// LPCXpresso Flash driver - Flash Device settings
//*****************************************************************************
//
// Copyright 2014-2015, 2018, 2020 NXP
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

#include "lpcx_flash_driver.h"

FLASHDEV_SECTION
FlashDeviceV_t FlashDevice  =  {
   FLASH_DRV_VERS,      // Driver Version, do not modify!
   "MIMXRT_FLEXSPI "__DATE__" "__TIME__,
   EXTSPI,              // Device Type
   0x00000000,          // Device Start Address
   0x40000000,          // Device Size
   256,                 // Programming Page Size
   0,                   // Reserved, must be 0
   0xFF,                // Initial Content of Erased Memory
   3000,                // Program Page Timeout
   15000,               // Erase Sector Timeout
   // Specify Size and Address of Sectors
   {{0x1000, 0},        // Sector sizes
   {SECTOR_END}}
};


