/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_hardware_init.h"
#include "ufl_rom_api.h"
#include "cmsis_compiler.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/



/*******************************************************************************
 * Code
 ******************************************************************************/

void ufl_init_hardware_imxrt6xx(void)
{
    uint32_t v;

    __disable_irq();
    // Disable cache
    MEM_WriteU32(0x40033800, 0);
    MEM_WriteU32(0x4003301C, 0);
    // PMC->MEMSEQCTRL = 0x1U;
    MEM_WriteU32(0x40135030, 0x1U);
    // Power FFRO
    MEM_WriteU32(0x40002630, 0x10000U);
    // Flexspi SRAM APD/PPD
    MEM_WriteU32(0x40002634, 0xCU);
    // Power SRAM
    MEM_WriteU32(0x40002638, 0xFFFFFFFF);
    MEM_WriteU32(0x4000263C, 0xFFFFFFFF);

    // PMC CTRL APPLYCFG
    MEM_WriteU32(0x4013500C, MEM_ReadU32(0x4013500C) | 1);
    // WAIT PMC update done
    do 
    {
        v = MEM_ReadU32(0x40135004) & 1;
    } while (v);

    // MAINCLKSELA
    MEM_WriteU32(0x40001430, 0x3U);
    // MAINCLKSELB
    MEM_WriteU32(0x40001434, 0x0U);

    flexspi_nor_set_clock_source(kFlexSpiClockSrc_FFRO_Clk);
    //CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI_OTFAD_CLK_MASK;
    MEM_WriteU32(0x40001040, 0x10000U);
    //RSTCTL0->PRSTCTL0_CLR = RSTCTL0_PRSTCTL0_CLR_FLEXSPI_OTFAD_MASK;
    MEM_WriteU32(0x40000070, 0x10000U);

    //IOPCTL->PIO[2][12] = 0x130;
    MEM_WriteU32(0x40004130, 0x130U);
    //CLKCTL1->PSCCTL1_SET = CLKCTL1_PSCCTL1_SET_HSGPIO2_CLK_SET_MASK;
    MEM_WriteU32(0x40021044, 0x4U);
    //RSTCTL1->PRSTCTL1_CLR = RSTCTL1_PRSTCTL1_CLR_HSGPIO2_RST_CLR_MASK;
    MEM_WriteU32(0x40020074, 0x4U);
    // GPIO->DIR[2] = 1 << 12;
    MEM_WriteU32(0x40102008, 0x1000U);
    // GPIO->CLR[2] = 1 << 12;
    MEM_WriteU32(0x40102288, 0x1000U);
    // Delay 400us to reset external flash
    for(uint32_t i =0; i < 6000; i++)
    {
        __NOP();
    }
    // GPIO->SET[2] = 1 << 12;
    MEM_WriteU32(0x40102208, 0x1000U);
    // Clear FLASH status store register
    MEM_WriteU32(0x40002380, 0x0U);
}

