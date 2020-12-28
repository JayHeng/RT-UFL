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

#include <flexspi_QSPI_flash.h>
#include <stdint.h>
// For memcmp used in Verify()
#include <string.h>

#include "fsl_common.h"
#include "lpcx_flash_driver.h"



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

    // this from SystemInit, that is skipped in NDEBUG
#ifdef NDEBUG

    if (WDOG1->WCR & WDOG_WCR_WDE_MASK)
    {
        WDOG1->WCR &= ~WDOG_WCR_WDE_MASK;
    }
    if (WDOG2->WCR & WDOG_WCR_WDE_MASK)
    {
        WDOG2->WCR &= ~WDOG_WCR_WDE_MASK;
    }
    RTWDOG->CNT = 0xD928C520U; /* 0xD928C520U is the update key */
    RTWDOG->TOVAL = 0xFFFF;
    RTWDOG->CS = (uint32_t) ((RTWDOG->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;

    /* Disable Systick which might be enabled by bootrom */
    if (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)
    {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    }

    /* Enable instruction and data caches
     * there are problems with power on reset if
     * this is not done */

     SCB_EnableICache();
//       SCB_EnableDCache();

#endif

	status = QSPI_init();
	if (status) {
		return (-1);
	}

	return 0; // Finished without Errors
}

// ************************************

/*  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t UnInit(void) {

    /* Do software reset to reset AHB buffer. */
    FLEXSPI_SoftwareReset(EXAMPLE_FLEXSPI);

	return 0;                               // Finished without Errors
}

// ************************************

#if !defined(DONT_PROVIDE_ERASECHIP)
/*  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed */
uint32_t EraseChip(void) {

    uint32_t sector = 0;
    status_t status = 0;

    for (sector = 0; sector < SECTOR_NUMBER; sector++) {

        status = flexspi_nor_flash_erase_sector(EXAMPLE_FLEXSPI, sector * SECTOR_SIZE);

    	if (status) {
    		return (status);
    	}
    }
	return (status);
}

#endif


extern uint32_t checkblank(uint32_t adr, uint32_t words, uint32_t blankval);

// ************************************

/*  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseSectors(uint32_t adr, uint32_t numsecs) {

    uint32_t sector = 0;
    status_t status = 0;

    if (adr < FlexSPI_AMBA_BASE) {			// bail if < flash, can't really happen
    	return(-1);
    }
    adr = adr - FlexSPI_AMBA_BASE;			// set address to base of flash

    for (sector = 0; sector < numsecs; sector++) {

    	status = flexspi_nor_flash_erase_sector(EXAMPLE_FLEXSPI, adr + (sector * SECTOR_SIZE));

    	if (status) {
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

    status_t status = 0;
    uint32_t page = 0;

    if (adr < FlexSPI_AMBA_BASE) {			// bail if < flash, can't really happen
    	return(-1);
    }
    adr = adr - FlexSPI_AMBA_BASE;			// set address to base of flash

    while (sz) {

    	status = flexspi_nor_flash_page_program(EXAMPLE_FLEXSPI, adr, (void *) (buf + (page * FLASH_PAGE_SIZE)));

    	if (status) {
    		return (status);
    	}

    	sz = sz - FLASH_PAGE_SIZE;		// Decrease remaining size by page
    	adr = adr + FLASH_PAGE_SIZE;	// Update adr base by page
    	page ++;						// Next page

    }
    return (status);
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
	FLEXSPI_SoftwareReset(EXAMPLE_FLEXSPI);

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

