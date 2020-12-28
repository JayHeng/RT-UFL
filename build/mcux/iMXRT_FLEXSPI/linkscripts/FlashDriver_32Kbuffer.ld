/*
 * Linker script for NXP iMXRT SPIFI Flash Driver (Messaged)
*/
 
MEMORY
{
	  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = (64 * 1024)
}

/* stack size :  multiple of 8*/
__stack_size = (4 * 1024);

/* flash image buffer size : multiple of page size*/				
__cache_size = (32 * 1024);			

/* Supported operations bit map
 * 0x40 = New device info available after Init() call
 * This setting must match the actual target flash driver build!
 */
 __opmap_val = 0x1000;

/* Actual placement of flash driver code/data controlled via standard file */
INCLUDE "../../LPCXFlashDriverLib/linker/placement.ld"
