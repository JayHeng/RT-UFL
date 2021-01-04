//*****************************************************************************
// LPCXpresso IDE flash driver - FlashPrg.c
//
// * Flash Programming Functions for SPIFI Flash
//*****************************************************************************
//
// Copyright 2014-2016, 2018-2019, 2020 NXP
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

#include "ufl_rom_api.h"
#include "lpcx_flash_driver.h"

#if defined(UFL_USE_CONST_VAR)
const
#endif
flexspi_nor_config_t flashConfig = {.pageSize = 0x400};

Mailbox_Init_DynInfo_t Mailbox_Init_DynInfo;

// ************************************
// Flash driver functions
// ************************************

/*
 *  Initialize Flash Programming Functions
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t Init (void)
{
    status_t status;

    ufl_target_desc_t *uflTargetDesc = (ufl_target_desc_t *)&g_uflTargetDesc;
    if (uflTargetDesc->imxrtChipId == kChipId_Invalid)
    {
        uflTargetDesc->configOption.option0.U = 0x0;
        uflTargetDesc->configOption.option1.U = 0x0;
    }

    status = ufl_full_setup();
    if (status != kStatus_Success)
    {
        return (-1);
    }
    else
    {
        return (0);
    }
}

// ************************************

/*  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t UnInit(void) {

    /* Do software reset to reset AHB buffer. */
    //FLEXSPI_SoftwareReset(EXAMPLE_FLEXSPI);

    return 0;                               // Finished without Errors
}

// ************************************

#if !defined(DONT_PROVIDE_ERASECHIP)
/*  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed */
uint32_t EraseChip(void) {

    uint32_t instance = g_uflTargetDesc.flexspiInstance;
    /*Erase all*/
    status_t status =  flexspi_nor_flash_erase_all(instance, (void *)&flashConfig);
    if (status != kStatus_Success)
    {
        return (status);
    }
    else
    {
        return (0);
    }
}

#endif


extern uint32_t checkblank(uint32_t adr, uint32_t words, uint32_t blankval);

// ************************************

/*  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseSectors(uint32_t adr, uint32_t numsecs) {

    status_t status = kStatus_Success;
    uint32_t instance = g_uflTargetDesc.flexspiInstance;
    uint32_t baseAddr = g_uflTargetDesc.flashBaseAddr;

    // bail if < flash, can't really happen
    if (adr < baseAddr)
    {
        return (-1);
    }

     // set address to base of flash
    adr = adr - baseAddr;

    for (uint32_t sector = 0; sector < numsecs; sector++)
    {
        status = flexspi_nor_flash_erase(instance, (void *)&flashConfig, adr + (sector * flashConfig.sectorSize), flashConfig.sectorSize);

        if (status != kStatus_Success)
        {
            return (status);
        }
    }

    return (status);
}

/*  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
// The SPIFI implementation of ProgramPage is really a
// ProgramBufferFull routine. The host debugger will send down
// a buffer full of data (of size FlashDevice.szPage as set up
// in Init() routine, and the spifiProgram() routine will then
// program each page from the buffer in turn. Doing this
// dramatically improves download performance compared to sending
// just a single true page of data at a time to the ProgramPage()
// routine - particularly given the small size of SPIFI flash pages.


uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint8_t *buf) {

    status_t status = kStatus_Success;
    uint32_t instance = g_uflTargetDesc.flexspiInstance;
    uint32_t baseAddr = g_uflTargetDesc.flashBaseAddr;

    // bail if < flash, can't really happen
    if (adr < baseAddr)
    {
        return (-1);
    }

    if (g_uflTargetDesc.isFlashPageProgram)
    {
        for(uint32_t size = 0; size < sz; size+=flashConfig.pageSize,
                                           buf+=flashConfig.pageSize,
                                           adr+=flashConfig.pageSize)
        {
            status =  flexspi_nor_flash_page_program(instance, (void *)&flashConfig, adr - baseAddr, (uint32_t *)buf);
            if (status != kStatus_Success)
            {
                return (status);
            }
        }
    }
    else
    {
        status =  flexspi_nor_flash_page_program(instance, (void *)&flashConfig, adr - baseAddr, (uint32_t *)buf);
        if (status != kStatus_Success)
        {
            return (status);
        }
    }

    return (0);
}

// ************************************
/*
 *  Verify Flash Contents
 *    Parameter:      adr:  Start Address
 *                    sz:   Size (in bytes)
 *                    buf:  Data
 *    Return Value:   0 - OK, Failed - Address
 */
#if !defined(DONT_PROVIDE_VERIFY)
uint32_t Verify(uint32_t adr, uint32_t sz, uint8_t *buf) {

	uint32_t status = 0;

	/* Reset to memory mode so we can access the SPIFI via a bus read */
	//FLEXSPI_SoftwareReset(EXAMPLE_FLEXSPI);

	if (adr == FlashDevice.DevAdr)	{	// ignore CRP word - NOT NEEDED HERE
		status = memcmp((void *) adr, buf, sz > 28 ? 28 : sz);
		if (!status && sz > 32) {
			status = memcmp((void *) adr + 32, buf + 32, sz - 32);
		}
	}
	else {
		status = memcmp((void *) adr, buf, sz);
	}
	return (status);
}
#if defined (USE_SMALL_MEMCMP)
int memcmp(const void *a, const void *b, size_t n)
{	const unsigned char *ac = (const unsigned char *)a,
	*bc = (const unsigned char *)b;
	while (n-- > 0)
	{	unsigned char c1,c2; /* unsigned cmp seems more intuitive */
		if ((c1 = *ac++) != (c2 = *bc++)) return c1 - c2;
	}
	return 0;
}
#endif // USE_SMALL_MEMCMP
#endif // DONT_PROVIDE_VERIFY

