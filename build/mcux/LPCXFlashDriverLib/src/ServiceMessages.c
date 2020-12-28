//*****************************************************************************
// ServiceMessages.c
//
// Polling wrapper for LPCXpresso flash driver (Messaged)
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

/***********************************************************
 *
 *  WARNING: This target flash driver file is intended to
 *           function in conjunction with host debug stub.
 *           Do not modify one without modifying the other.
 *
 ***********************************************************/

#include <lpcx_flash_driver.h>
#include <lpcx_flash_msgif.h>
#include <stdbool.h>
#include <stdint.h>

void ServiceMessages(Mailbox_t *msg) {
	//	__disable_irq();
	__asm volatile ("cpsid i");

	uint32_t initres = 0;
	uint32_t cmd;
	uint32_t status;
	initres = Init();
	msg->result = initres;
	msg->command = FOP_NONE;

	do {
		__asm volatile ("dmb");
		cmd = msg->command;
		int resultCode = 0;

		if (cmd != 0) {
			// Message from host has been received
			status = msg->status = FSTATUS_BUSY;

			// Check if Init() call failed - in which case we don't want
			//  to try to program the flash!
			if (initres != 0) {
				msg->result = initres;
				status |= FSTATUS_INITFAIL;

			} else {
				if (cmd & FOP_ERASE_SECTOR) {
					resultCode = EraseSectors(
							msg->param[FOPARG_ERASESECT_FADDR],
							msg->param[FOPARG_ERASESECT_SECTORS]);
#if !defined (MINRELEASE) || defined (MINRELEASEMASS)
				} else if (cmd & FOP_ERASE_ALL) {
					if (EraseChip)
						resultCode = EraseChip();
#endif
				} else if (cmd & (FOP_PROGRAM | FOP_VERIFY)) {
					if (cmd & FOP_PROGRAM) {
						/* Implement this as 'program and verify' */
						resultCode =
								ProgramPage(
										msg->param[FOPARG_PROGRAMPAGES_FADDR],
										msg->param[FOPARG_PROGRAMPAGES_BYTES],
										(uint8_t *) (msg->param[FOPARG_PROGRAMPAGES_MEMADDR]));
					}
#if !defined (MINRELEASE)
					/* could ignore opcode - it gets handled in FOP_PROGRAM now */
					if (!resultCode && Verify)
						resultCode =
								Verify(msg->param[FOPARG_VERIFY_FADDR],
										msg->param[FOPARG_VERIFY_BYTES],
										(uint8_t *) (msg->param[FOPARG_VERIFY_MEMADDR]));
#endif
#if defined (SECTOR_HASHING)
				} else if (cmd & FOP_CHECKSUM_SECTORS) {
						// Use Verify with a size of 0 just to make sure the flash is in read mode
						if (Verify)
							Verify(0 /*dummy*/, 0, 0);
						resultCode = ChecksumSectors(
								msg->param[FOPARG_CHECKSUMSECT_FADDR],
								msg->param[FOPARG_CHECKSUMSECT_SECTORS],
								(ChecksumData_t *) msg->param[FOPARG_CHECKSUMSECT_MEMADDR]);
#endif
				} else if (cmd & (FOP_TERMINATE | FOP_CLOSE)) {
#if !defined (MINRELEASE)
					UnInit();
#endif
				} else {
					status |= FSTATUS_NOSUPPORT;
					resultCode = cmd;
				}
				msg->result = resultCode;
				if (resultCode != 0)
					status |= FSTATUS_IAP_ERROR;
			}
			msg->status = status;
			msg->command = FOP_NONE;
			/*< release mailbox for reuse by the client */
		}

	} while (cmd != FOP_TERMINATE);
	while (true)
		continue; /* we're not expected to return - even after a terminate message */
}

#if defined (SECTOR_HASHING)
/*
 *  Checksum Sector Contents in Flash Memory
 *    Parameter:      adr:  Sector Address
 *                    numsecs: Number of sectors
 *                    data:  checksum data
 *    Return Value:   0 - OK,  1 - Failed
 *
 * 32 bit Fowler/Noll/Vo FNV-1a hash code
	http://en.wikipedia.org/wiki/Fowler_Noll_Vo_hash
 */
#define FNV1_32_INIT	0x811c9dc5
#define FNV1_32_PRIME	0x01000193
uint32_t ChecksumSectors(uint32_t adr, uint32_t numsecs, ChecksumData_t* data)
{
	struct FlashSectors* pCrtSector = &FlashDevice.sectors[0];
	uint32_t *results = (uint32_t *)&data->results;
	data->algID = FCHECKSUM_FNV1A_32;

	for (uint32_t i = 0; i < numsecs; i++)
	{
		while (((pCrtSector + 1)->AddrSector != 0xFFFFFFFF) &&
				(adr >= FlashDevice.DevAdr + (pCrtSector + 1)->AddrSector))
		{
			pCrtSector++;
		}

		uint32_t hash = FNV1_32_INIT;
		uint32_t size = pCrtSector->szSector;
		uint8_t *p = (uint8_t *)adr;
		while (size-- > 0)
		{
			hash ^= (uint32_t)*p++;
			hash *= FNV1_32_PRIME;
		}
		results[i] = hash;

		adr += pCrtSector->szSector;
	}

	return 0;
}
#endif
