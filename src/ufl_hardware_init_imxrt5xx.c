/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ufl_hardware_init.h"
#include "ufl_rom_api.h"
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

void ufl_init_hardware_imxrt5xx(void)
{
    uint32_t v;

    //CACHE64_CTRL0->CCR = 0;
    MEM_WriteU32(0x40033800, 0);
    //CACHE64_POLSEL0->POLSEL = 0;
    MEM_WriteU32(0x4003301C, 0);
    // PMC->MEMSEQCTRL = 0x101U;
    MEM_WriteU32(0x40135030, 0x101U);
    //SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_FFRO_PD_MASK;
    MEM_WriteU32(0x40002630u, 0x10000U);
    /* Flexspi SRAM APD/PPD */
    MEM_WriteU32(0x40002634u, 0xCU);
    // PDRUNCFG2, 3 CLR
    MEM_WriteU32(0x40002638, 0xFFFFFFFF);
    MEM_WriteU32(0x4000263C, 0xFFFFFFFF);

    // PMC CTRL APPLYCFG
    MEM_WriteU32(0x4013500C, MEM_ReadU32(0x4013500C) | 1);
    // WAIT PMC update done
    do {
      v = MEM_ReadU32(0x40135004) & 1;
    } while (v);

    //  CLKCTL0->FFRODIVOEN = CLKCTL0_FFRODIVOEN_FFRO_DIV1_O_EN_MASK | CLKCTL0_FFRODIVOEN_FFRO_DIV2_O_EN_MASK |
    //                        CLKCTL0_FFRODIVOEN_FFRO_DIV4_O_EN_MASK | CLKCTL0_FFRODIVOEN_FFRO_DIV8_O_EN_MASK |
    //                        CLKCTL0_FFRODIVOEN_FFRO_DIV16_O_EN_MASK;
    MEM_WriteU32(0x40001110u, 0x1FU);
    /* MAINCLKSELA */
    MEM_WriteU32(0x40001430u, 0x3U);
    /* MAINCLKSELB */
    MEM_WriteU32(0x40001434u, 0x0U);

    flexspi_nor_set_clock_source(kFlexSpiClockSrc_FFRO_Clk);
    //CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI0_OTFAD_MASK;
    MEM_WriteU32(0x40001040, 0x10000U);
    //RSTCTL0->PRSTCTL0_CLR = RSTCTL0_PRSTCTL0_CLR_FLEXSPI0_OTFAD_MASK;
    MEM_WriteU32(0x40000070, 0x10000U);
}

