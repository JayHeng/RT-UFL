//*****************************************************************************
// lpcx_flash_memdev.c
//
// LPCXpresso flash driver interface (Messaged) - Memory device definition
//*****************************************************************************
//
// Copyright 2014, 2018, 2020 NXP
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

/***********************************************************
 *
 *  WARNING: This file is intended to be identical in both the
 *           host debug stub and the target flash driver.
 *           Do not modify one without modifying the other.
 *
 ***********************************************************/

#include "lpcx_flash_driver.h"

// Symbols defined in the linker script
extern uint32_t   _b_bss;
extern uint32_t   _e_bss;
extern uint32_t   __load_base;
extern uint32_t   __image_size;
extern uint32_t   __cache;
extern uint32_t   __cache_size;   // redefine in instance structure
extern uint32_t   __initial_sp;
extern uint32_t   __stack_size;
extern uint32_t   __opmap_val;

// MemoryDevice Instance (fill it in)
EXTERN_MEMORY_DEVICE
MemoryDeviceMsg_t MemoryDevice =
{
  MEM_FLASH_VER2_MAJ+0x0,       // Version of flash interface
                                // Magic number to identify flash driver
                                // interface
  { 0x01, 0x23, 0x45, 0x00, 0x00, 0x54, 0x32, 0x10 },
  (uint32_t)&__load_base,         // Driver load address
  (uint32_t)&__image_size,        // Size of .text and .data
  (uint32_t)&__cache,             // RAM buffer location
  (uint32_t)&__cache_size,        // RAM buffer size
  (uint32_t)&__initial_sp,        // Stack top
  (uint32_t)&__stack_size,        // Stack size
  (uint32_t)&__opmap_val,        // Bitmap of available operations - 0 = everything there
  &FlashDevice,                 // Flash Device configuration
  ServiceMessages,              // Service mailbox flash operation message
  0                              // Reserved
};

