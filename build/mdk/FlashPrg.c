/**************************************************************************//**
 * @file     FlashPrg.c
 * @brief    Flash Programming Functions adapted for New Device Flash
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
#include "ufl_rom_api.h"

#if defined(UFL_USE_CONST_VAR)
const
#endif
flexspi_nor_config_t flashConfig = {.pageSize = FLASH_DRV_PAGE_SIZE};

/*
extern struct FlashDevice const FlashDevice;
static void update_flash_dev_property(void)
{
    struct FlashDevice *flashDevice = (struct FlashDevice *)((uint32_t)(&FlashDevice));
    flashDevice->DevAdr = g_uflTargetDesc.flashBaseAddr;
    flashDevice->szDev = 0x800000;
    flashDevice->szPage = flashConfig.pageSize;
    flashDevice->sectors[0].szSector = flashConfig.sectorSize;
}
*/

/* 
   Mandatory Flash Programming Functions (Called by FlashOS):
                int Init        (unsigned long adr,   // Initialize Flash
                                 unsigned long clk,
                                 unsigned long fnc);
                int UnInit      (unsigned long fnc);  // De-initialize Flash
                int EraseSector (unsigned long adr);  // Erase Sector Function
                int ProgramPage (unsigned long adr,   // Program Page Function
                                 unsigned long sz,
                                 unsigned char *buf);

   Optional  Flash Programming Functions (Called by FlashOS):
                int BlankCheck  (unsigned long adr,   // Blank Check
                                 unsigned long sz,
                                 unsigned char pat);
                int EraseChip   (void);               // Erase complete Device
      unsigned long Verify      (unsigned long adr,   // Verify Function
                                 unsigned long sz,
                                 unsigned char *buf);

       - BlanckCheck  is necessary if Flash space is not mapped into CPU memory space
       - Verify       is necessary if Flash space is not mapped into CPU memory space
       - if EraseChip is not provided than EraseSector for all sectors is called
*/


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

    // Do this as IAR can accept option from IDE, so initial option may be 
    //  kept in first time initialization, we need to clear option here in case
    //  uninitialized option0 is happened to be 0xcxxx_xxxx 
    ufl_target_desc_t *uflTargetDesc = (ufl_target_desc_t *)&g_uflTargetDesc;
    if (uflTargetDesc->imxrtChipId == kChipId_Invalid)
    {
        uflTargetDesc->configOption.option0.U = 0x0;
        uflTargetDesc->configOption.option1.U = 0x0;
    }

    status_t status = ufl_full_setup();
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0);
    }

    //update_flash_dev_property();
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {

    /* Add your Code */
    return (0);                                  // Finished without Errors
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip (void) {

    uint32_t instance = g_uflTargetDesc.flexspiInstance;
    /*Erase all*/
    status_t status =  flexspi_nor_flash_erase_all(instance, (void *)&flashConfig);
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {

    uint32_t instance = g_uflTargetDesc.flexspiInstance;
    uint32_t baseAddr = g_uflTargetDesc.flashBaseAddr;
    /*Erase Sector*/
    status_t status =  flexspi_nor_flash_erase(instance, (void *)&flashConfig, adr - baseAddr, FLASH_DRV_SECTOR_SIZE);
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

    status_t status = kStatus_Success;
    uint32_t instance = g_uflTargetDesc.flexspiInstance;
    uint32_t baseAddr = g_uflTargetDesc.flashBaseAddr;

    if (g_uflTargetDesc.isFlashPageProgram)
    {
        for(uint32_t size = 0; size < sz; size+=FLASH_DRV_PAGE_SIZE,
                                           buf+=FLASH_DRV_PAGE_SIZE,
                                           adr+=FLASH_DRV_PAGE_SIZE)
        {
            status =  flexspi_nor_flash_page_program(instance, (void *)&flashConfig, adr - baseAddr, (uint32_t *)buf);
            if (status != kStatus_Success)
            {
                return (1);
            }
        }
    }
    else
    {
        status =  flexspi_nor_flash_page_program(instance, (void *)&flashConfig, adr - baseAddr, (uint32_t *)buf);
        if (status != kStatus_Success)
        {
            return (1);
        }
    }

    return (0);
}
