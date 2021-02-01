/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2010 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Flash Programming Functions adapted                   */
/*               for New Device 256kB Flash                            */
/*                                                                     */
/***********************************************************************/

#include "FlashOS.H" // FlashOS Structures
#include "flexspi_nor_flash.h"

#define FLEXSPI_NOR_INSTANCE 0
#define SECTOR_SIZE          (4096)
#define BASE_ADDRESS         (0x60000000)

/* Init this global variable to workaround of the issue to running this flash algo in Segger */
flexspi_nor_config_t config = {1};

void disableWatchdog_rt1020()
{
    WDOG1->WMCR &= ~WDOG_WMCR_PDE_MASK;
    WDOG2->WMCR &= ~WDOG_WMCR_PDE_MASK;

    /* Watchdog disable */

    if (WDOG1->WCR & WDOG_WCR_WDE_MASK)
    {
        WDOG1->WCR &= ~WDOG_WCR_WDE_MASK;
    }
    if (WDOG2->WCR & WDOG_WCR_WDE_MASK)
    {
        WDOG2->WCR &= ~WDOG_WCR_WDE_MASK;
    }
    RTWDOG->CNT   = 0xD928C520U; /* 0xD928C520U is the update key */
    RTWDOG->TOVAL = 0xFFFF;
    RTWDOG->CS    = (uint32_t)((RTWDOG->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;
}

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init(unsigned long adr, unsigned long clk, unsigned long fnc)
{
    status_t status;
    serial_nor_config_option_t option;
    option.option0.U = 0xc0000006; // QuadSPI NOR, Frequency: 100MHz
                                   /* Disable Watchdog Power Down Counter */
		disableWatchdog_rt1020();
	
    status        = flexspi_nor_get_config(FLEXSPI_NOR_INSTANCE, &config, &option);
    if (status != kStatus_Success)
    {
        return (1);
    }
    status = flexspi_nor_flash_init(FLEXSPI_NOR_INSTANCE, &config);
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0); // Finished without Errors
    }
}

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit(unsigned long fnc)
{
    /* Add your Code */
    return (0); // Finished without Errors
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip(void)
{
    status_t status;
    status = flexspi_nor_flash_erase_all(FLEXSPI_NOR_INSTANCE, &config); // Erase all
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0); // Finished without Errors
    }
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector(unsigned long adr)
{
    status_t status;
    adr    = adr - BASE_ADDRESS;
    status = flexspi_nor_flash_erase(FLEXSPI_NOR_INSTANCE, &config, adr, SECTOR_SIZE); // Erase 1 sector
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

int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
    status_t status;
    adr = adr - BASE_ADDRESS;
    // Program data to destination
    status = flexspi_nor_flash_page_program(FLEXSPI_NOR_INSTANCE, &config, adr, (uint32_t *)buf); // program 1 page
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}
