/**************************************************************************//**
 * @file     FlashDev.c
 * @brief    Flash Device Description for New Device Flash
 * @version  V1.0.0
 * @date     10. January 2018
 ******************************************************************************/
/*
 * Copyright (c) 2010-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include "FlashOS.h"        // FlashOS Structures

// MIMXRT595-EVK, MIMXRT685-EVK
/*
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT_FLEXSPI",           // Device Name 
   EXTSPI,                     // Device Type
   0x08000000,                 // Device Start Address
   0x04000000,                 // Device Size is 64 MB (64mB)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   3000,                       // Program Page Timeout 3000 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

   // Specify Size and Address of Sectors
   0x01000, 0x0,               // sectors are 4 KB
   SECTOR_END
};
*/

// MIMXRT1010-EVK
/*
struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT_FLEXSPI",           // Device Name
   EXTSPI,                     // Device Type
   0x60000000,                 // Device Start Address
   0x01000000,                 // Device Size in Bytes (16mB)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   100,                        // Program Page Timeout 100 mSec
   5000,                       // Erase Sector Timeout 5000 mSec

   // Specify Size and Address of Sectors
   0x1000, 0x00000000,         // Sector Size  4kB (256 Sectors)
   SECTOR_END
};
*/

// MIMXRT1050-EVK
/*
struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT_FLEXSPI",           // Device Name
   EXTSPI,                     // Device Type
   0x60000000,                 // Device Start Address
   0x04000000,                 // Device Size in Bytes (64mB)
   512,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   100,                        // Program Page Timeout 100 mSec
   5000,                       // Erase Sector Timeout 5000 mSec

   // Specify Size and Address of Sectors
   0x40000, 0x00000000,        // Sector Size  256kB (256 Sectors)
   SECTOR_END
};
*/

// MIMXRT1020-EVK, MIMXRT1060-EVK
/*
struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT_FLEXSPI",           // Device Name
   EXTSPI,                     // Device Type
   0x60000000,                 // Device Start Address
   0x00800000,                 // Device Size in Bytes (8mB)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   100,                        // Program Page Timeout 100 mSec
   5000,                       // Erase Sector Timeout 5000 mSec

   // Specify Size and Address of Sectors
   0x1000, 0x00000000,         // Sector Size  4kB (256 Sectors)
   SECTOR_END
};
*/

// MIMXRT1170-EVK
/*
struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT_FLEXSPI",           // Device Name
   EXTSPI,                     // Device Type
   0x30000000,                 // Device Start Address
   0x01000000,                 // Device Size in Bytes (16mB)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   100,                        // Program Page Timeout 100 mSec
   15000,                      // Erase Sector Timeout 15000 mSec

   // Specify Size and Address of Sectors
   0x1000, 0x00000000,         // Sector Size  4kB (256 Sectors)
   SECTOR_END
};
*/

struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT_FLEXSPI",           // Device Name
   EXTSPI,                     // Device Type
   0x00000000,                 // Device Start Address
   0x40000000,                 // Device Size in Bytes (1GB)
   FLASH_DRV_PAGE_SIZE,        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   3000,                       // Program Page Timeout 100 mSec
   15000,                      // Erase Sector Timeout 15000 mSec

   // Specify Size and Address of Sectors
   FLASH_DRV_SECTOR_SIZE, 0x00000000,    // Sector Size  4kB (256 Sectors)
   SECTOR_END
};
