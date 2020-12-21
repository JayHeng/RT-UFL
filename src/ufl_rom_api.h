/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _UFL_ROM_API_H_
#define _UFL_ROM_API_H_

#include "ufl_flexspi_nor_flash.h"
#include "ufl_rom_api_imxrt5xx.h"
#include "ufl_rom_api_imxrt6xx.h"
#include "ufl_rom_api_imxrt106x.h"
#include "ufl_rom_api_imxrt117x.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*
 *  Serial NOR configuration block
 */
typedef struct _flexspi_nor_config
{
    uint32_t cfg0[20];
    uint32_t sflashA1Size;               //!< [0x050-0x053] Size of Flash connected to A1
    uint32_t sflashA2Size;               //!< [0x054-0x057] Size of Flash connected to A2
    uint32_t sflashB1Size;               //!< [0x058-0x05b] Size of Flash connected to B1
    uint32_t sflashB2Size;               //!< [0x05c-0x05f] Size of Flash connected to B2
    uint32_t cfg1[88];
    uint32_t pageSize;              //!< Page size of Serial NOR
    uint32_t sectorSize;            //!< Sector size of Serial NOR
    uint32_t cfg2[2];
    uint32_t blockSize;             //!< Block size
    uint32_t cfg3[11];
} flexspi_nor_config_t;

/*
 * Serial NOR Configuration Option
 */
typedef struct _serial_nor_config_option
{
    union
    {
        struct
        {
            uint32_t max_freq : 4;          //!< Maximum supported Frequency
            uint32_t misc_mode : 4;         //!< miscellaneous mode
            uint32_t quad_mode_setting : 4; //!< Quad mode setting
            uint32_t cmd_pads : 4;          //!< Command pads
            uint32_t query_pads : 4;        //!< SFDP read pads
            uint32_t device_type : 4;       //!< Device type
            uint32_t option_size : 4;       //!< Option size, in terms of uint32_t, size = (option_size + 1) * 4
            uint32_t tag : 4;               //!< Tag, must be 0x0E
        } B;
        uint32_t U;
    } option0;

    union
    {
        struct
        {
            uint32_t dummy_cycles : 8;     //!< Dummy cycles before read
            uint32_t status_override : 8;  //!< Override status register value during device mode configuration
            uint32_t pinmux_group : 4;     //!< The pinmux group selection
            uint32_t dqs_pinmux_group : 4; //!< The DQS Pinmux Group Selection
            uint32_t drive_strength : 4;   //!< The Drive Strength of FlexSPI Pads
            uint32_t flash_connection : 4; //!< Flash connection option: 0 - Single Flash connected to port A, 1 -
            //! Parallel mode, 2 - Single Flash connected to Port B
        } B;
        uint32_t U;
    } option1;
} serial_nor_config_option_t;

//!@brief FLEXSPI Flash driver API Interface
typedef struct _flexspi_nor_flash_driver
{
    status_t (*init)(uint32_t instance, void *config);
    status_t (*page_program)(uint32_t instance, void *config, uint32_t dst_addr, const uint32_t *src);
    status_t (*erase_all)(uint32_t instance, void *config);
    status_t (*erase)(uint32_t instance, void *config, uint32_t start, uint32_t lengthInBytes);
    status_t (*read)(uint32_t instance, void *config, uint32_t *dst, uint32_t addr, uint32_t lengthInBytes);
    status_t (*set_clock_source)(uint32_t clockSrc);
    status_t (*get_config)(uint32_t instance, void *config, void *option);
} flexspi_nor_flash_driver_t;

typedef struct _tool_cfg_iar
{
    bool enablePageSizeOverride;
} ufl_tool_cfg_iar;

typedef struct _target_desc
{
    uint32_t imxrtChipId;
    uint32_t flexspiInstance;
    uint32_t flexspiBaseAddr;
    uint32_t flashBaseAddr;
    serial_nor_config_option_t configOption;
    flexspi_nor_flash_driver_t flashDriver;
    bool isFlashPageProgram;
    ufl_tool_cfg_iar iarCfg;
} ufl_target_desc_t;

#if defined(UFL_USE_CONST_VAR)
const
#endif
extern ufl_target_desc_t g_uflTargetDesc;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /*_cplusplus*/

status_t flexspi_nor_flash_init(uint32_t instance, void *config);

status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        void *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src);

status_t flexspi_nor_flash_erase_all(uint32_t instance, void *config);

status_t flexspi_nor_get_config(uint32_t instance, void *config, void *option);

status_t flexspi_nor_flash_erase(uint32_t instance, void *config, uint32_t start, uint32_t length);

status_t flexspi_nor_flash_read(uint32_t instance, void *config, uint32_t *dst, uint32_t start, uint32_t bytes);

status_t flexspi_nor_set_clock_source(uint32_t clockSrc);

status_t flexspi_nor_auto_config(uint32_t instance, void *config, void *option);

#if defined(__cplusplus)
}
#endif /*_cplusplus*/

#endif /* _UFL_ROM_API_H_ */
