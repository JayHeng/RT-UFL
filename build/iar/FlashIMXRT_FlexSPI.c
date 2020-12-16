/*************************************************************************
 *
 *   Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2018
 *
 *    File name   : Flash.c
 *    Description : iMXRT1060_FlexSPI flash loader
 *
 *    History :
 *    1. Date        : August, 2018
 *       Author      : Stoyan Choynev
 *       Description : Initial revision. The Flash loader uses ROM API.
 *                     It supports several types of flash devices. The devices
 *                     should be connected to the A port of FlexSPI1 or
 *                     FlexSPI2. For more info read IMXRT1060 ROM API
 *                     specification
 *
 *    $Revision: $
 **************************************************************************/
/** include files **/
#include <intrinsics.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include "flash_loader.h"       // The flash loader framework API declarations.
#include "flash_loader_extra.h"
#include "bl_api.h"

/** local definitions **/

/** default settings **/

/** external functions **/

/** external data **/

/** internal functions **/
static uint32_t strToUint(const char *str);
/** public data **/
#pragma data_alignment=4
const flexspi_nor_config_t config_block ;/*= {
	0x46, 0x43, 0x46, 0x42, 0x00, 0x04, 0x01, 0x56, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x04, 0x07, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEB, 0x04, 0x18, 0x0A,
	0x06, 0x32, 0x04, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x05, 0x04, 0x04, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x04, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x20, 0x04, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xD8, 0x04, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x04, 0x18, 0x08,
	0x04, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x60, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};*/
/** private data **/
static uint32_t fspi_instance;
static uint32_t flash_base;
/** public functions **/

/*************************************************************************
 * Function Name: FlashInit
 * Parameters: Flash Base Address
 *
 * Return:  0 - Init Successful
 *          1 - Init Fail
 * Description: Init flash and build layout.
 *
 *************************************************************************/
#if USE_ARGC_ARGV
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags,
                   int argc, char const *argv[])
#else
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags)
#endif /* USE_ARGC_ARGV */
{
uint32_t result = (RESULT_OK);
serial_nor_config_option_t option;
  
  flash_base = (uint32_t) base_of_flash;
  
  if(0x60000000 == flash_base)
  {
    fspi_instance = 0;
  } 
  else
  {
    fspi_instance = 1;
  }

  bl_api_init();
  
  option.option0.U = 0xC0000008;
  option.option1.U = 0x00000000;
  
#if USE_ARGC_ARGV
  for(int i = 0; i < argc; /*i++*/)
  {
    if((strcmp("--Opt0", argv[i]) == 0) && ((i+1) < argc))
    {
      /* Flash clock init */    
      option.option0.U = strToUint(argv[++i]);
      i+=2;
    }
    else if((strcmp("--Opt1", argv[i]) == 0) && ((i+1) < argc))
    {
      /* Flash clock init */    
      option.option1.U = strToUint(argv[++i]);
      i+=2;
    }
    else
    {
      if(strcmp("--Qspi", argv[i]) == 0 )
      {//default option setting
      }
      else if(strcmp("--QspiDDR", argv[i]) == 0 )
      {
        option.option0.U = 0xc0100003;
      }
      else if(strcmp("--Hyper1V8", argv[i]) == 0 )
      {
        option.option0.U = 0xc0233009;
      }
      else if(strcmp("--Hyper3V0", argv[i]) == 0 )
      {
        option.option0.U = 0xc0333006;
      }
      else if(strcmp("--MxicOpiDDR", argv[i]) == 0 )
      {
        option.option0.U = 0xc0333006;
      }
      else if(strcmp("--McrnOct", argv[i]) == 0 )
      {
        option.option0.U = 0xc0600006;
      }
      else if(strcmp("--McrnOpi", argv[i]) == 0 )
      {
        option.option0.U = 0xc0603008;
      }
      else if(strcmp("--McrnOpiDDR", argv[i]) == 0 )
      {
        option.option0.U = 0xc0633008;
      }
      else if(strcmp("--AdstOpi", argv[i]) == 0 )
      {
        option.option0.U = 0xc0803008;
      }
      
      i+=1;
    }
  }
#endif
    
  if (flexspi_nor_get_config(fspi_instance, (flexspi_nor_config_t *) &config_block, &option) !=  kStatus_Success)
  {
    result = (RESULT_ERROR);
  }
  else
  {    
    if (flexspi_nor_flash_init(fspi_instance, (flexspi_nor_config_t *) &config_block) !=  kStatus_Success)
    {
      result = (RESULT_ERROR);
    }
    else
    {
    
      sprintf(LAYOUT_OVERRIDE_BUFFER,"%d 0x%X, %d 0x%X\0",\
              config_block.blockSize/config_block.sectorSize,config_block.sectorSize,\
              config_block.memConfig.sflashA1Size/config_block.blockSize - 1, config_block.blockSize);
      SET_PAGESIZE_OVERRIDE(config_block.pageSize);
      result |= (OVERRIDE_LAYOUT) | (OVERRIDE_PAGESIZE);
      
      if (FLAG_ERASE_ONLY & flags)
      {
        if(flexspi_nor_flash_erase_all(fspi_instance, (flexspi_nor_config_t *) &config_block) !=  kStatus_Success)
        {
          result = (RESULT_ERROR);
        }
        else
        {
          result = (RESULT_ERASE_DONE);
        }
      }      
    }
  }

  return result; 
}

/*************************************************************************
 * Function Name: FlashWrite
 * Parameters: block base address, offet in block, data size, ram buffer
 *             pointer
 * Return: RESULT_OK - Write Successful
 *         RESULT_ERROR - Write Fail
 * Description. Writes data in to NOR
 *************************************************************************/
uint32_t FlashWrite(void *block_start,
                    uint32_t offset_into_block,
                    uint32_t count,
                    char const *buffer)
{

uint32_t result = (RESULT_OK);
uint32_t loadaddr = (uint32_t)block_start-flash_base+offset_into_block;
uint32_t size = config_block.pageSize;

  while(count)
  {
    
    if (flexspi_nor_flash_page_program(fspi_instance, (flexspi_nor_config_t *) &config_block, loadaddr, (uint32_t *)buffer) != kStatus_Success)
    {
      result = (RESULT_ERROR);
      break;
    }
    count -= size;
    loadaddr += size;
    buffer += size;
  }
  
  return result;
}

/*************************************************************************
 * Function Name: FlashErase
 * Parameters:  Block Address, Block Size
 *
 * Return: RESULT_OK - Erase Successful
 *         RESULT_ERROR - Erase Fail
 * Description: Erase block
 *************************************************************************/
uint32_t FlashErase(void *block_start,
                    uint32_t block_size)
{
uint32_t addr = (uint32_t)block_start-flash_base;
uint32_t result = (RESULT_OK);
    if (flexspi_nor_flash_erase(fspi_instance, (flexspi_nor_config_t *) &config_block, addr , block_size) != kStatus_Success)
    {
      result = (RESULT_ERROR);
    }
  
  return result;
}

/** private functions **/
#if USE_ARGC_ARGV
/*************************************************************************
 * String to unsigned integer
 *************************************************************************/
static uint32_t strToUint(const char *str)
{
    uint32_t base = 10, result = 0;
    char str_c;
    /* Check number base */
    if(strlen(str) > 0 && str[0] == '0')
    {
        if(strlen(str) > 1  && toupper(str[1]) == 'X')
        {
            base = 16;
            str += 2;
        }
        else
        {
            base = 8;
            str += 1;
        }
    }
    /* Convert string to unsigned integer */
    while((str_c = toupper(*str++)) != '\0')
    {
        if('0' <= str_c && str_c <= '9')
            result = result * base + str_c - '0';
        else if('A' <= str_c && str_c <= 'F')
            result = result * base + str_c - 'A' + 10;
        else
            return UINT32_MAX;
    }

    return result;
}
#endif

