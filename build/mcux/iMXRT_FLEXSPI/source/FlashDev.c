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

#include <flexspi_QSPI_flash.h>
#include "lpcx_flash_driver.h"
#include "fsl_common.h"

FLASHDEV_SECTION
FlashDeviceV_t FlashDevice  =  {
   FLASH_DRV_VERS,				// Driver Version, do not modify!
   "MIMXRT1050-EVK_IS25WP064A "__DATE__" "__TIME__,
   EXTSPI,     			// Device Type
   FlexSPI_AMBA_BASE, 	// Device Start Address
   DEVICE_SIZE,   			// Device Size
   PSEUDO_PAGE_SIZE,    // Programming Page Size
   0,          			// Reserved, must be 0
   0xFF,       			// Initial Content of Erased Memory
   3000,       			// Program Page Timeout
   6000,       			// Erase Sector Timeout
   // Specify Size and Address of Sectors
   {{SECTOR_SIZE, 0},   // Sector sizes
   {SECTOR_END}}
};


