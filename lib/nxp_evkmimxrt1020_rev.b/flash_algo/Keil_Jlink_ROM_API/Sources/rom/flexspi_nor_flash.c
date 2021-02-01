/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "bl_flexspi.h"
#include "flexspi_nor_flash.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

#define MAX_24BIT_ADDRESSING_SIZE (16UL * 1024 * 1024)

#define NOR_CMD_LUT_FOR_IP_CMD 1 //!< 1 Dedicated LUT Sequence Index for IP Command

//!@brief Typical Serial NOR commands supported by most Serial NOR devices
enum
{
    kSerialNorCmd_BasicRead_3B = 0x03,
    kSerialNorCmd_BasicRead_4B = 0x13,
    kSerialNorCmd_PageProgram_1_1_1_3B = 0x02,
    kSerialNorCmd_PageProgram_1_1_1_4B = 0x12,
    kSerialNorCmd_PageProgram_1_4_4_4B = 0x3E,
    kSerialNorCmd_PageProgram_1_1_4_4B = 0x34,
    kSerialNorCmd_Read_SDR_1_4_4_3B = 0xEB,
    kSerialNorCmd_Read_DDR_1_4_4_3B = 0xED,
    kSerialNorCmd_Read_SDR_1_4_4_4B = 0xEC,
    kSerialNorCmd_Read_SDR_1_1_4_4B = 0x6C,
    kSerialNorCmd_Read_DDR_1_4_4_4B = 0xEE,
    kSerialNorCmd_ChipErase = 0x60,
    kSerialNorCmd_WriteEnable = 0x06,
    kSerialNorCmd_WriteStatusReg1 = 0x01,
    kSerialNorCmd_ReadStatusReg1 = 0x05,
    kSerialNorCmd_WriteStatusReg2 = 0x3E,
    kSerialNorCmd_ReadStatusReg2 = 0x3F,
    kSerialNorCmd_ReadFlagReg = 0x70,
    kSerialNorCmd_ReadId = 0x9F,
};

enum
{
    kSerialNorCmd_SE4K_3B = 0x20,
    kSerialNorCmd_SE4K_4B = 0x21,
    kSerialNorCmd_SE64K_3B = 0xD8,
    kSerialNorCmd_SE64K_4B = 0xDC,
};

enum
{
    kSerialNorQpiMode_NotConfig = 0,
    kSerialNorQpiMode_Cmd_0x38 = 1,
    kSerialNorQpiMode_Cmd_0x38_QE = 2,
    kSerialNorQpiMode_Cmd_0x35 = 3,
    kSerialNorQpiMode_Cmd_0x71 = 4,
    kSerialNorQpiMode_Cmd_0x61 = 5,
};

enum
{
    kSerialNorType_StandardSPI, //!< Device that support Standard SPI and Extended SPI mode
    kSerialNorType_HyperBus,    //!< Device that supports HyperBus only
    kSerialNorType_XPI,         //!< Device that works under DPI, QPI or OPI mode
    kSerialNorType_NoCmd, //!< Device that works under No command mode (XIP mode/Performance Enhance mode/continous read
    //! mode)
};

typedef struct _lut_seq
{
    uint32_t lut[4];
} lut_seq_t;

enum
{
    kSerialNOR_IndividualMode = false,
    kSerialNOR_ParallelMode = true,
};

enum
{
    kFlexSpiSerialClk_Update,
    kFlexSpiSerialClk_Restore
};

enum
{
    kSerialFlash_ReadSFDP = 0x5A,
    kSerialFlash_ReadManufacturerId = 0x9F,
};

//!@brief SFDP related definitions
#define SFDP_SIGNATURE 0x50444653 /* ASCII: SFDP */
enum
{
    kSfdp_Version_Major_1_0 = 1,
    kSfdp_Version_Minor_0 = 0, // JESD216
    kSfdp_Version_Minor_A = 5, // JESD216A
    kSfdp_Version_Minor_B = 6, // JESD216B
    kSfdp_Version_Minor_C = 7, // JESD216C

    kSfdp_BasicProtocolTableSize_Rev0 = 36,
    kSfdp_BasicProtocolTableSize_RevA = 64,
    kSfdp_BasicProtocolTableSize_RevB = kSfdp_BasicProtocolTableSize_RevA,
    kSfdp_BasicProtocolTableSize_RevC = 80,
};

typedef struct _sfdp_header
{
    uint32_t signature;
    uint8_t minor_rev;
    uint8_t major_rev;
    uint8_t param_hdr_num;
    uint8_t sfdp_access_protocol; // Defined in JESD216C, reserved for older version
} sfdp_header_t;

enum
{
    kParameterID_BasicSpiProtocol = 0xFF00,
    // New Table added in JESD216B
    kParameterID_SectorMap = 0xFF81,
    kParameterID_4ByteAddressInstructionTable = 0xFF84,
    // New Table added in JESD216C
    kParameterID_xSpiProfile1_0 = 0xFF85,
    kParameterID_xSpiOrofile2_0 = 0xFF86,
    kParameterID_StaCtrlCfgRegMap = 0xFF87,
    kParameterID_OpiEnableSeq = 0xFF09,
};

//!@brief SFDP Parameter Header, see JESD216B doc for more details
typedef struct _sfdp_parameter_header
{
    uint8_t parameter_id_lsb;
    uint8_t minor_rev;
    uint8_t major_rev;
    uint8_t table_length_in_32bit;
    uint8_t parameter_table_pointer[3];
    uint8_t parameter_id_msb;
} sfdp_parameter_header_t;

//!@brief Basic Flash Parameter Table, see JESD216B doc for more details
typedef struct _jedec_flash_param_table
{
    struct
    {
        uint32_t erase_size : 2;
        uint32_t write_granularity : 1;
        uint32_t reserved0 : 2;
        uint32_t unused0 : 3;
        uint32_t erase4k_inst : 8;
        uint32_t support_1_1_2_fast_read : 1;
        uint32_t address_bits : 2;
        uint32_t support_ddr_clocking : 1;
        uint32_t support_1_2_2_fast_read : 1;
        uint32_t supports_1_4_4_fast_read : 1;
        uint32_t support_1_1_4_fast_read : 1;
        uint32_t unused1 : 9;
    } misc;
    uint32_t flash_density;
    struct
    {
        uint32_t dummy_clocks_1_4_4_read : 5;
        uint32_t mode_clocks_1_4_4_read : 3;
        uint32_t inst_1_4_4_read : 8;
        uint32_t dummy_clocks_1_1_4_read : 5;
        uint32_t mode_clocks_1_1_4_read : 3;
        uint32_t inst_1_1_4_read : 8;
    } read_1_4_info;
    struct
    {
        uint32_t dummy_clocks_1_2_2_read : 5;
        uint32_t mode_clocks_1_2_2_read : 3;
        uint32_t inst_1_2_2_read : 8;
        uint32_t dummy_clocks_1_1_2_read : 5;
        uint32_t mode_clocks_1_1_2_read : 3;
        uint32_t inst_1_1_2_read : 8;
    } read_1_2_info;

    struct
    {
        uint32_t support_2_2_2_fast_read : 1;
        uint32_t reserved0 : 3;
        uint32_t support_4_4_4_fast_read : 1;
        uint32_t reserved1 : 27;
    } read_22_44_check;

    struct
    {
        uint32_t reserved0 : 16;
        uint32_t dummy_clocks_2_2_2_read : 5;
        uint32_t mode_clocks_2_2_2_read : 3;
        uint32_t inst_2_2_2_read : 8;
    } read_2_2_info;
    struct
    {
        uint32_t reserved0 : 16;
        uint32_t dummy_clocks_4_4_4_read : 5;
        uint32_t mode_clocks_4_4_4_read : 3;
        uint32_t inst_4_4_4_read : 8;
    } read_4_4_info;

    struct
    {
        uint8_t size;
        uint8_t inst;
    } erase_info[4];

    uint32_t erase_timing;
    struct
    {
        uint32_t reserved0 : 4;
        uint32_t page_size : 4;
        uint32_t reserved1 : 24;
    } chip_erase_progrm_info;

    struct
    {
        uint32_t suspend_resume_spec;
        uint32_t suspend_resume_inst;
    } suspend_resume_info;

    struct
    {
        uint32_t reserved0 : 2;
        uint32_t busy_status_polling : 6;
        uint32_t reserved1 : 24;
    } busy_status_info;

    struct
    {
        uint32_t mode_4_4_4_disable_seq : 4;
        uint32_t mode_4_4_4_enable_seq : 5;
        uint32_t support_mode_0_4_4 : 1;
        uint32_t mode_0_4_4_exit_method : 6;
        uint32_t mode_0_4_4_entry_method : 4;
        uint32_t quad_enable_requirement : 3;
        uint32_t hold_reset_disable : 1;
        uint32_t reserved0 : 8;
    } mode_4_4_info;

    struct
    {
        uint32_t status_reg_write_enable : 7;
        uint32_t reserved0 : 1;
        uint32_t soft_reset_rescue_support : 6;
        uint32_t exit_4_byte_addressing : 10;
        uint32_t enter_4_byte_addrssing : 8;
    } mode_config_info;

    struct
    {
        uint32_t dummy_clocks_1_8_8_read : 5;
        uint32_t mode_clocks_1_8_8_read : 3;
        uint32_t inst_1_8_8_read : 8;
        uint32_t dummy_clocks_1_1_8_read : 5;
        uint32_t mode_clocks_1_1_8_read : 3;
        uint32_t inst_1_1_8_read : 8;
    } read_1_8_info;

    struct
    {
        uint32_t reserved : 18;
        uint32_t output_driver_strength : 5;
        uint32_t jedec_spi_protocol_reset : 1;
        uint32_t dqs_waveform_type_sdr : 2;
        uint32_t dqs_support_in_qpi_sdr : 1;
        uint32_t dqs_support_in_qpi_ddr : 1;
        uint32_t dqs_support_in_opi_str : 1;
        uint32_t cmd_and_extension_in_opi_ddr : 2;
        uint32_t byte_order_in_opi_ddr : 1;
    } xpi_misc_info;

    struct
    {
        uint32_t opi_sdr_disable_seq : 4;
        uint32_t opi_sdr_enable_deq : 5;
        uint32_t support_mode_0_8_8 : 1;
        uint32_t mode_0_8_8_exit_method : 6;
        uint32_t mode_0_8_8_entry_method : 4;
        uint32_t octal_enable_requirement : 3;
        uint32_t reserved : 9;
    } mode_octal_info;

    struct
    {
        uint32_t qpi_sdr_no_dqs : 4;
        uint32_t qpi_sdr_with_dqs : 4;
        uint32_t qpi_ddr_no_dqs : 4;
        uint32_t qpi_ddr_with_dqs : 4;
        uint32_t opi_sdr_no_dqs : 4;
        uint32_t opi_sdr_with_dqs : 4;
        uint32_t opi_ddr_no_dqs : 4;
        uint32_t opi_ddr_with_dqs : 4;
    } max_speed_info_xpi;

} jedec_flash_param_table_t;

//!@brief 4Byte Addressing Instruction Table, see JESD216B doc for more details
typedef struct _jedec_4byte_addressing_inst_table
{
    struct
    {
        uint32_t support_1_1_1_read : 1;
        uint32_t support_1_1_1_fast_read : 1;
        uint32_t support_1_1_2_fast_read : 1;
        uint32_t support_1_2_2_fast_read : 1;
        uint32_t support_1_1_4_fast_read : 1;
        uint32_t support_1_4_4_fast_read : 1;
        uint32_t support_1_1_1_page_program : 1;
        uint32_t support_1_1_4_page_program : 1;
        uint32_t support_1_4_4_page_program : 1;
        uint32_t support_erase_type1_size : 1;
        uint32_t support_erase_type2_size : 1;
        uint32_t support_erase_type3_size : 1;
        uint32_t support_erase_type4_size : 1;
        uint32_t support_1_1_1_dtr_read : 1;
        uint32_t support_1_2_2_dtr_read : 1;
        uint32_t support_1_4_4_dtr_read : 1;
        uint32_t support_volatile_sector_lock_read_cmd : 1;
        uint32_t support_volatile_sector_lock_write_cmd : 1;
        uint32_t support_nonvolatile_sector_lock_read_cmd : 1;
        uint32_t support_nonvolatile_sector_lock_write_cmd : 1;
        uint32_t reserved : 12;
    } cmd_4byte_support_info;

    struct
    {
        uint8_t erase_inst[4];
    } erase_inst_info;
} jedec_4byte_addressing_inst_table_t;

typedef struct _jdec_query_table
{
    uint32_t standard_version; // JESD216 version
    uint32_t flash_param_tbl_size;
    jedec_flash_param_table_t flash_param_tbl;
    bool has_4b_addressing_inst_table;
    jedec_4byte_addressing_inst_table_t flash_4b_inst_tbl;
} jedec_info_table_t;

/*******************************************************************************
 * Local prototypes
 *******************************************************************************/
//!@brief Exit No Command mode
static status_t flexspi_nor_exit_no_cmd_mode(uint32_t instance,
                                             flexspi_nor_config_t *config,
                                             bool isParallelMode,
                                             uint32_t baseAddr);
//!@brief Restore to No Command mode
static status_t flexspi_nor_restore_no_cmd_mode(uint32_t instance,
                                                flexspi_nor_config_t *config,
                                                bool isParallelMode,
                                                uint32_t baseAddr);

//!@brief Send Write Enable command to Serial NOR via FlexSPI
static status_t flexspi_nor_write_enable(uint32_t instance,
                                         flexspi_nor_config_t *config,
                                         bool isParalleMode,
                                         uint32_t baseAddr);

//!@brief Wait until Flash device is idle
static status_t flexspi_nor_wait_busy(uint32_t instance,
                                      flexspi_nor_config_t *config,
                                      bool isParalleMode,
                                      uint32_t baseAddr);

//!@brief Update Serial Clock for IP command execution
static void flexspi_change_serial_clock(uint32_t instance, flexspi_nor_config_t *config, uint32_t operation);

//!@brief Parse SFDP table and generate FlexSPI NOR config block automatically
static status_t parse_sfdp(uint32_t instance,
                           flexspi_nor_config_t *config,
                           jedec_info_table_t *tbl,
                           serial_nor_config_option_t *option);

//!@brief Read SFDP info specified by arguments
static status_t flexspi_nor_read_sfdp(uint32_t instance, uint32_t addr, uint32_t *buffer, uint32_t bytes);

//!@brief Prepare Quad mode enable LUT sequence
static status_t prepare_quad_mode_enable_sequence(uint32_t instance,
                                                  flexspi_nor_config_t *config,
                                                  jedec_info_table_t *tbl,
                                                  serial_nor_config_option_t *option);

//!@brief Probe dummy cycles for DDR read
static status_t probe_dtr_quad_read_dummy_cycles(uint32_t instance,
                                                 flexspi_nor_config_t *config,
                                                 uint32_t *dummy_cycles);

//!@brief Read SFDP Info back
static status_t flexspi_nor_read_sfdp_info(uint32_t instance, jedec_info_table_t *tbl, bool address_shift_enable);

//!@brief Generate FlexSPI NOR Configuration Block by reading SFDP info
static status_t flexspi_nor_generate_config_block_using_sfdp(uint32_t instance,
                                                             flexspi_nor_config_t *config,
                                                             serial_nor_config_option_t *option);

#if FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
// Basic Read instruction for HyperBus
static status_t flexspi_nor_hyperbus_read(uint32_t instance, uint32_t addr, uint32_t *buffer, uint32_t bytes);

// Basic Write Instruction for HyperBus
static status_t flexspi_nor_hyperbus_write(uint32_t instance, uint32_t addr, uint32_t *buffer, uint32_t bytes);

//!@brief Generate FlexSPI Configuration block for HyperFLASH
static status_t flexspi_nor_generate_config_block_hyperflash(uint32_t instance,
                                                             flexspi_nor_config_t *config,
                                                             bool is_1v8);

//!@brief Generate FlexSPI Configuration Block for Macronix Octal Flash
static status_t flexspi_nor_generate_config_block_mxic_octalflash(uint32_t instance,
                                                                  flexspi_nor_config_t *config,
                                                                  serial_nor_config_option_t *option);

//!@brief Generate FlexSPI Configuration block for Micron XCCELA Flash
static status_t flexspi_nor_generate_config_block_micron_octalflash(uint32_t instance,
                                                                    flexspi_nor_config_t *config,
                                                                    serial_nor_config_option_t *option);

//!@brief Generate FlexSPI Configuration Block for Adesto EcoXIP Flash
static status_t flexspi_nor_generate_config_block_adesto_octalflash(uint32_t instance,
                                                                    flexspi_nor_config_t *config,
                                                                    serial_nor_config_option_t *option);
#endif // FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
//!@brief Generate 0-4-4 mode enable sequence, currently only applicable to Micron QuadSPI FLASH
//        For other QuadSPI NOR Flash device, it is not required.
static status_t prepare_0_4_4_mode_enable_sequence(uint32_t instance,
                                                   flexspi_nor_config_t *config,
                                                   jedec_info_table_t *tbl,
                                                   serial_nor_config_option_t *option);
#endif // FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT

/*******************************************************************************
 * Code
 *******************************************************************************/

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_INIT)
// See flexspi_nor_flash.h for more details
status_t flexspi_nor_flash_init(uint32_t instance, flexspi_nor_config_t *config)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        status = flexspi_init(instance, (flexspi_mem_config_t *)config);
        if (status != kStatus_Success)
        {
            break;
        }

        // Configure Lookup table for Read
        flexspi_update_lut(instance, 0, config->memConfig.lookupTable, 1);

        // QE bit is nonvolatile bit, should be programmed only once
        if (config->memConfig.deviceModeType == kDeviceConfigCmdType_QuadEnable)
        {
            config->memConfig.deviceModeCfgEnable = false;
        }

    } while (0);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_INIT)

status_t flexspi_nor_exit_no_cmd_mode(uint32_t instance,
                                      flexspi_nor_config_t *config,
                                      bool isParallelMode,
                                      uint32_t baseAddr)
{
#if !FLEXSPI_FEATURE_HAS_PARALLEL_MODE
    isParallelMode = false;
#endif
    flexspi_xfer_t flashXfer;
    flashXfer.operation = kFlexSpiOperation_Command;
    flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD;
    flashXfer.seqNum = 1;
    flashXfer.isParallelModeEnable = isParallelMode;
    flashXfer.baseAddress = baseAddr;
    flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &config->memConfig.lookupTable[4 * flashXfer.seqId],
                       flashXfer.seqNum);
    flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;

    return flexspi_command_xfer(instance, &flashXfer);
}

static status_t flexspi_nor_restore_no_cmd_mode(uint32_t instance,
                                                flexspi_nor_config_t *config,
                                                bool isParallelMode,
                                                uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        status = flexspi_nor_write_enable(instance, config, isParallelMode, baseAddr);
        if (status != kStatus_Success)
        {
            break;
        }

#if !FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        isParallelMode = false;
#endif

        flexspi_xfer_t flashXfer;
        flashXfer.operation = kFlexSpiOperation_Command;
        flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_RESTORE_NOCMD;
        flashXfer.seqNum = 1;
        flashXfer.isParallelModeEnable = isParallelMode;
        flashXfer.baseAddress = baseAddr;

        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &config->memConfig.lookupTable[4 * flashXfer.seqId],
                           flashXfer.seqNum);
        flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(instance, &flashXfer);
    } while (0);

    return status;
}

status_t flexspi_nor_write_enable(uint32_t instance,
                                  flexspi_nor_config_t *config,
                                  bool isParallelMode,
                                  uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;
    uint32_t lut_tmp[4];
    do
    {
        if (config == NULL)
        {
            break;
        }
#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        if (config->needExitNoCmdMode)
        {
            // Issue exit no command sequence before sending write enable command, in case device is under
            // continuous mode.
            status = flexspi_nor_exit_no_cmd_mode(instance, config, isParallelMode, baseAddr);
            if (status != kStatus_Success)
            {
                break;
            }
        }
#endif

#if FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
        if (config->serialNorType == kSerialNorType_XPI)
        {
            memcpy(lut_tmp, &config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE], 16);
            memcpy(&config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE],
                   &config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_XPI], 16);
        }
#endif

        status = flexspi_device_write_enable(instance, &config->memConfig, isParallelMode, baseAddr);
#if FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
        // Restore LUT
        if (config->serialNorType == kSerialNorType_XPI)
        {
            memcpy(&config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE], lut_tmp, 16);
        }
#endif
        if (status != kStatus_Success)
        {
            break;
        }

    } while (0);

    return status;
}

status_t flexspi_nor_wait_busy(uint32_t instance, flexspi_nor_config_t *config, bool isParallMode, uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;
    uint32_t lut_tmp[4];

    do
    {
        if (config == NULL)
        {
            break;
        }
#if FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
        if (config->serialNorType == kSerialNorType_XPI)
        {
            memcpy(lut_tmp, &config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS], 16);
            memcpy(&config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS],
                   &config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI], 16);
        }
#endif
        status = flexspi_device_wait_busy(instance, &config->memConfig, isParallMode, baseAddr);
        if (status != kStatus_Success)
        {
            break;
        }

    } while (0);

#if FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
    // Restore LUT
    if ((config != NULL) && (config->serialNorType == kSerialNorType_XPI))
    {
        memcpy(&config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS], lut_tmp, 16);
    }
#endif

    return status;
}

void flexspi_change_serial_clock(uint32_t instance, flexspi_nor_config_t *config, uint32_t operation)
{
    do
    {
        if ((config == NULL) || (operation > kFlexSpiSerialClk_Restore))
        {
            break;
        }

        bool isClockChangeRequired = (config->ipcmdSerialClkFreq > 0) ? true : false;
        if (!isClockChangeRequired)
        {
            break;
        }

        bool isDdrModeEnabled =
            config->memConfig.controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable) ? true : false;

        flexspi_wait_idle(instance);

        if (operation == kFlexSpiSerialClk_Update)
        {
            if (config->ipcmdSerialClkFreq)
            {
                flexspi_clock_config(instance, config->ipcmdSerialClkFreq, isDdrModeEnabled);
                // Re-configure DLLCR
                flexspi_configure_dll(instance, &config->memConfig);
            }
            if (config->halfClkForNonReadCmd)
            {
                flexspi_half_clock_control(instance, 1);
            }
        }
        else
        {
            if (config->ipcmdSerialClkFreq)
            {
                flexspi_clock_config(instance, config->memConfig.serialClkFreq, isDdrModeEnabled);
                // Re-configure DLLCR
                flexspi_configure_dll(instance, &config->memConfig);
            }
            if (config->halfClkForNonReadCmd)
            {
                flexspi_half_clock_control(instance, 0);
            }
        }

        // Per IP requirement, wait at least 10 serial clocks to let the Serial Clock output become stable
        uint32_t serial_clock;
        flexspi_get_clock(instance, kFlexSpiClock_SerialRootClock, &serial_clock);
        if (isDdrModeEnabled)
        {
            serial_clock /= 2;
        }

        uint32_t core_clock;
        flexspi_get_clock(instance, kFlexSpiClock_CoreClock, &core_clock);
        // Note: The while loop needs 4 instructions, so the dummy_cnt needs to be divided by 4 to calculate the actual
        //       dummy cylces
        register uint32_t dummy_cnt = 10 * (1 + core_clock / serial_clock) / 4;
        while (dummy_cnt--)
        {
            __NOP();
        }

    } while (0);
}

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FELXSPI_NOR_PROGRAM)
// See flexspi_nor_flash.h for more details
status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src)
{
    status_t status;
    flexspi_xfer_t flashXfer;
    flexspi_mem_config_t *memCfg = (flexspi_mem_config_t *)config;

#if !FLEXSPI_FEATURE_HAS_PARALLEL_MODE
    bool isParallelMode = false;
#else
    bool isParallelMode = flexspi_is_parallel_mode(memCfg);
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE

    // Update serial clock for IP command, for some devices, it cannot erase/program the
    // device with the highest clock for read.
    flexspi_change_serial_clock(instance, config, kFlexSpiSerialClk_Update);

    do
    {
        // Send write enable before executing page program command
        status = flexspi_nor_write_enable(instance, config, isParallelMode, dstAddr);
        if (status != kStatus_Success)
        {
            break;
        }

        // Prepare page program command
        flashXfer.operation = kFlexSpiOperation_Write;
        flashXfer.seqNum = 1;
        flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM;
        if (memCfg->lutCustomSeqEnable && memCfg->lutCustomSeq[NOR_CMD_INDEX_PAGEPROGRAM].seqNum)
        {
            flashXfer.seqId = memCfg->lutCustomSeq[NOR_CMD_INDEX_PAGEPROGRAM].seqId;
            flashXfer.seqNum = memCfg->lutCustomSeq[NOR_CMD_INDEX_PAGEPROGRAM].seqNum;
        }
        flashXfer.baseAddress = dstAddr;
        flashXfer.isParallelModeEnable = isParallelMode;
        flashXfer.txBuffer = (uint32_t *)src;
        flashXfer.txSize = config->pageSize;

        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &config->memConfig.lookupTable[4 * flashXfer.seqId],
                           flashXfer.seqNum);
        flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(instance, &flashXfer);
        if (status != kStatus_Success)
        {
            break;
        }

        // Wait until the program operation completes on Serial NOR Flash side.
        status = flexspi_nor_wait_busy(instance, config, isParallelMode, dstAddr);
        if (status != kStatus_Success)
        {
            break;
        }

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        if (config->needRestoreNoCmdMode)
        {
            status = flexspi_nor_restore_no_cmd_mode(instance, config, isParallelMode, dstAddr);
        }
#endif

    } while (0);

    flexspi_clear_cache(instance);

    // Restore clock for AHB command
    flexspi_change_serial_clock(instance, config, kFlexSpiSerialClk_Restore);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FELXSPI_NOR_PROGRAM)

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_ERASE_ALL)
// See flexspi_nor_flash.h for more details
status_t flexspi_nor_flash_erase_all(uint32_t instance, flexspi_nor_config_t *config)
{
    uint32_t *flashSizeStart = NULL;
    uint32_t currentFlashSize = 0;
    uint32_t baseAddr = 0;
    uint32_t index = 0;
    status_t status = kStatus_Success;
    flexspi_xfer_t flashXfer;
    flexspi_mem_config_t *memCfg = (flexspi_mem_config_t *)config;

    flashXfer.operation = kFlexSpiOperation_Command;
    flashXfer.seqNum = 1;
    flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_CHIPERASE;
    flashXfer.isParallelModeEnable = kSerialNOR_IndividualMode;
    if (memCfg->lutCustomSeqEnable && memCfg->lutCustomSeq[NOR_CMD_INDEX_CHIPERASE].seqNum)
    {
        flashXfer.seqId = memCfg->lutCustomSeq[NOR_CMD_INDEX_CHIPERASE].seqId;
        flashXfer.seqNum = memCfg->lutCustomSeq[NOR_CMD_INDEX_CHIPERASE].seqNum;
    }
    uint32_t chipEraseSeqId = flashXfer.seqId;
    uint32_t chipEraseSeqNum = flashXfer.seqNum;

    // Update serial clock for IP command, for some devices, it cannot erase/program the
    // device with the highest clock for read.
    flexspi_change_serial_clock(instance, config, kFlexSpiSerialClk_Update);

    // Send Chip Erase command to each device if needed.
    baseAddr = 0;
    flashSizeStart = &memCfg->sflashA1Size;
    for (index = 0; index < 4; index++)
    {
        currentFlashSize = *flashSizeStart++;

        if (currentFlashSize > 0)
        {
            status = flexspi_nor_write_enable(instance, config, kSerialNOR_IndividualMode, baseAddr);
            if (status != kStatus_Success)
            {
                break;
            }

            flashXfer.baseAddress = baseAddr;
            flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &config->memConfig.lookupTable[4 * chipEraseSeqId],
                               chipEraseSeqNum);
            flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
            status = flexspi_command_xfer(instance, &flashXfer);
            if (status != kStatus_Success)
            {
                break;
            }
        }

        baseAddr += currentFlashSize;
    }

    if (status == kStatus_Success)
    {
        baseAddr = 0;
        flashSizeStart = &memCfg->sflashA1Size;
        for (index = 0; index < 4; index++)
        {
            currentFlashSize = *flashSizeStart++;
            if (currentFlashSize > 0)
            {
                // Wait until the chip erase operation completes on Serial NOR Flash side.
                status = flexspi_nor_wait_busy(instance, config, kSerialNOR_IndividualMode, baseAddr);
                if (status != kStatus_Success)
                {
                    break;
                }
#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
                if (config->needRestoreNoCmdMode)
                {
                    status = flexspi_nor_restore_no_cmd_mode(instance, config, kSerialNOR_IndividualMode, baseAddr);
                    if (status != kStatus_Success)
                    {
                        break;
                    }
                }
#endif
            }
            baseAddr += currentFlashSize;
        }
    }

    flexspi_clear_cache(instance);

    // Restore clock for AHB command
    flexspi_change_serial_clock(instance, config, kFlexSpiSerialClk_Restore);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_ERASE_ALL)

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_ERASE_SECTOR)
// See flexspi_nor_flash.h for more details.
status_t flexspi_nor_flash_erase_sector(uint32_t instance, flexspi_nor_config_t *config, uint32_t address)
{
    status_t status;
    flexspi_xfer_t flashXfer;
    bool isParallelMode;
    flexspi_mem_config_t *memCfg = (flexspi_mem_config_t *)config;
    isParallelMode = flexspi_is_parallel_mode(memCfg);

    // Update serial clock for IP command, for some devices, it cannot erase/program the
    // device with the highest clock for read.
    flexspi_change_serial_clock(instance, config, kFlexSpiSerialClk_Update);

    do
    {
        status = flexspi_nor_write_enable(instance, config, isParallelMode, address);
        if (status != kStatus_Success)
        {
            break;
        }

        flashXfer.baseAddress = address;
        flashXfer.operation = kFlexSpiOperation_Command;
        flashXfer.seqNum = 1;
        flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;
        flashXfer.isParallelModeEnable = isParallelMode;

        if (memCfg->lutCustomSeqEnable && memCfg->lutCustomSeq[NOR_CMD_INDEX_ERASESECTOR].seqNum)
        {
            flashXfer.seqId = memCfg->lutCustomSeq[NOR_CMD_INDEX_ERASESECTOR].seqId;
            flashXfer.seqNum = memCfg->lutCustomSeq[NOR_CMD_INDEX_ERASESECTOR].seqNum;
        }

        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &config->memConfig.lookupTable[4 * flashXfer.seqId],
                           flashXfer.seqNum);
        flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(instance, &flashXfer);
        if (status != kStatus_Success)
        {
            break;
        }

        // Wait until the sector erase operation completes on Serial NOR Flash side.
        status = flexspi_nor_wait_busy(instance, config, isParallelMode, address);
        if (status != kStatus_Success)
        {
            break;
        }
#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        if (config->needRestoreNoCmdMode)
        {
            status = flexspi_nor_restore_no_cmd_mode(instance, config, isParallelMode, address);
            if (status != kStatus_Success)
            {
                break;
            }
        }
#endif

    } while (0);

    flexspi_clear_cache(instance);

    // Restore clock for AHB command
    flexspi_change_serial_clock(instance, config, kFlexSpiSerialClk_Restore);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_ERASE_SECTOR)

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_ERASE_BLOCK)
status_t flexspi_nor_flash_erase_block(uint32_t instance, flexspi_nor_config_t *config, uint32_t address)
{
    status_t status;
    flexspi_xfer_t flashXfer;
    bool isParallelMode;
    flexspi_mem_config_t *memCfg = (flexspi_mem_config_t *)config;
    isParallelMode = flexspi_is_parallel_mode(memCfg);

    // Update serial clock for IP command, for some devices, it cannot erase/program the
    // device with the highest clock for read.
    flexspi_change_serial_clock(instance, config, kFlexSpiSerialClk_Update);

    do
    {
        status = flexspi_nor_write_enable(instance, config, isParallelMode, address);
        if (status != kStatus_Success)
        {
            break;
        }

        flashXfer.baseAddress = address;
        flashXfer.operation = kFlexSpiOperation_Command;
        flashXfer.seqNum = 1;
        flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK;
        flashXfer.isParallelModeEnable = isParallelMode;

        if (memCfg->lutCustomSeqEnable && memCfg->lutCustomSeq[NOR_CMD_INDEX_ERASEBLOCK].seqNum)
        {
            flashXfer.seqId = memCfg->lutCustomSeq[NOR_CMD_INDEX_ERASEBLOCK].seqId;
            flashXfer.seqNum = memCfg->lutCustomSeq[NOR_CMD_INDEX_ERASEBLOCK].seqNum;
        }

        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &config->memConfig.lookupTable[4 * flashXfer.seqId],
                           flashXfer.seqNum);
        flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(instance, &flashXfer);
        if (status != kStatus_Success)
        {
            break;
        }

        // Wait until the block erase operation completes on Serial NOR Flash side.
        status = flexspi_nor_wait_busy(instance, config, isParallelMode, address);
        if (status != kStatus_Success)
        {
            break;
        }
#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        if (config->needRestoreNoCmdMode)
        {
            status = flexspi_nor_restore_no_cmd_mode(instance, config, isParallelMode, address);
            if (status != kStatus_Success)
            {
                break;
            }
        }
#endif

    } while (0);

    flexspi_clear_cache(instance);

    // Restore clock for AHB command
    flexspi_change_serial_clock(instance, config, kFlexSpiSerialClk_Restore);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_ERASE_BLOCK)

// Read SFDP parameters from specified offset
status_t flexspi_nor_read_sfdp(uint32_t instance, uint32_t addr, uint32_t *buffer, uint32_t bytes)
{
    flexspi_xfer_t flashXfer;
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (buffer == NULL)
        {
            break;
        }
        memset(&flashXfer, 0, sizeof(flashXfer));
        flashXfer.operation = kFlexSpiOperation_Read;
        flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
        flashXfer.seqNum = 1;
        flashXfer.rxBuffer = buffer;
        flashXfer.rxSize = bytes;
        flashXfer.baseAddress = addr;
        status = flexspi_command_xfer(instance, &flashXfer);

    } while (0);

    return status;
}

status_t prepare_quad_mode_enable_sequence(uint32_t instance,
                                           flexspi_nor_config_t *config,
                                           jedec_info_table_t *tbl,
                                           serial_nor_config_option_t *option)
{
    status_t status = kStatus_InvalidArgument;

    // See JESD216B 6.4.18 for more details.
    do
    {
        // Enter Quad mode
        uint32_t enter_quad_mode_option = kSerialNorQuadMode_NotConfig;
        uint32_t lut_seq[4];
        memset(&lut_seq, 0, sizeof(lut_seq));

        // Ideally, we only need one condition here, however, for some Flash devices that actually support JESD216A
        // before the stanadard is publicly released, the JESD minor revsion is still the initial version. That is why
        // we use two conditions to handle below logic.
        if ((tbl->standard_version >= kSfdp_Version_Minor_A) ||
            (tbl->flash_param_tbl_size >= kSfdp_BasicProtocolTableSize_RevA))
        {
            config->memConfig.deviceModeCfgEnable = true;
            switch (tbl->flash_param_tbl.mode_4_4_info.quad_enable_requirement)
            {
                case 1:
                case 4:
                case 5:
                    enter_quad_mode_option = kSerialNorQuadMode_StatusReg2_Bit1;
                    break;
                case 6:
                    enter_quad_mode_option = kSerialNorQuadMode_StatusReg2_Bit1_0x31;
                    break;
                case 2:
                    enter_quad_mode_option = kSerialNorQuadMode_StatusReg1_Bit6;
                    break;
                case 3:
                    enter_quad_mode_option = kSerialNorQuadMode_StatusReg2_Bit7;
                    break;
                default:
                    enter_quad_mode_option = kSerialNorQuadMode_NotConfig;
                    config->memConfig.deviceModeCfgEnable = false;
                    break;
            }
        }
        else
        {
            enter_quad_mode_option = option->option0.B.quad_mode_setting;
        }

        if (enter_quad_mode_option != kSerialNorQuadMode_NotConfig)
        {
            flexspi_xfer_t xfer;
            uint32_t status_val = 0;
            xfer.baseAddress = 0;
            xfer.isParallelModeEnable = false;
            xfer.operation = kFlexSpiOperation_Read;
            xfer.rxBuffer = &status_val;
            xfer.rxSize = 1;
            xfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
            xfer.seqNum = 1;

            switch (enter_quad_mode_option)
            {
                case kSerialNorQuadMode_StatusReg2_Bit1:
                case kSerialNorQuadMode_StatusReg2_Bit1_0x31:
                    lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x35, READ_SDR, FLEXSPI_1PAD, 1);
                    break;
                case kSerialNorQuadMode_StatusReg1_Bit6:
                    lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 1);
                    break;
                case kSerialNorQuadMode_StatusReg2_Bit7:
                    lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x3F, READ_SDR, FLEXSPI_1PAD, 1);
                    break;
                default:
                    break;
            }

            flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &lut_seq[0], 1);

            status = flexspi_command_xfer(instance, &xfer);
            if (status != kStatus_Success)
            {
                break;
            }

            // Override status value if it is required by uers
            if (option->option0.B.option_size && option->option1.B.status_override)
            {
                status_val = option->option1.B.status_override;
            }

            // Do modify-afer-read status and then create Quad mode Enable sequence
            // Enable QE bit only if it is not enabled.
            config->memConfig.deviceModeCfgEnable = false;
            switch (enter_quad_mode_option)
            {
                case kSerialNorQuadMode_StatusReg1_Bit6:
                    if (!(status_val & FLEXSPI_BITMASK(6)))
                    {
                        config->memConfig.lookupTable[4 * 4] = FLEXSPI_LUT_SEQ(
                            CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_WriteStatusReg1, WRITE_SDR, FLEXSPI_1PAD, 0x01);
                        status_val &= (uint8_t)~0x3c; // Clear Block protection
                        status_val |= FLEXSPI_BITMASK(6);
                        config->memConfig.deviceModeCfgEnable = true;
                    }
                    break;
                case kSerialNorQuadMode_StatusReg2_Bit1:
                    if (!(status_val & FLEXSPI_BITMASK(1)))
                    {
                        config->memConfig.lookupTable[4 * 4] = FLEXSPI_LUT_SEQ(
                            CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_WriteStatusReg1, WRITE_SDR, FLEXSPI_1PAD, 0x02);
                        status_val |= FLEXSPI_BITMASK(1);
                        // QE bit will be programmed after status1 register, so need to left shit 8 bit
                        status_val <<= 8;
                        config->memConfig.deviceModeCfgEnable = true;
                    }
                    break;
                case kSerialNorQuadMode_StatusReg2_Bit1_0x31:
                    if (!(status_val & FLEXSPI_BITMASK(1)))
                    {
                        config->memConfig.lookupTable[4 * 4] =
                            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x31, WRITE_SDR, FLEXSPI_1PAD, 0x01);
                        status_val |= FLEXSPI_BITMASK(1);
                        config->memConfig.deviceModeCfgEnable = true;
                    }
                    break;
                case kSerialNorQuadMode_StatusReg2_Bit7:
                    if (!(status_val & FLEXSPI_BITMASK(7)))
                    {
                        config->memConfig.lookupTable[4 * 4] = FLEXSPI_LUT_SEQ(
                            CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_WriteStatusReg2, WRITE_SDR, FLEXSPI_1PAD, 0x02);
                        status_val |= FLEXSPI_BITMASK(7);
                        config->memConfig.deviceModeCfgEnable = true;
                    }
                    break;
                default:
                    config->memConfig.deviceModeCfgEnable = false;
                    break;
            }
            if (config->memConfig.deviceModeCfgEnable)
            {
                config->memConfig.deviceModeSeq.seqNum = 1;
                config->memConfig.deviceModeSeq.seqId = 4;
                config->memConfig.deviceModeArg = status_val;
                config->memConfig.deviceModeType = kDeviceConfigCmdType_QuadEnable;
            }
        }

        status = kStatus_Success;
    } while (0);

    return status;
}

status_t prepare_0_4_4_mode_enable_sequence(uint32_t instance,
                                            flexspi_nor_config_t *config,
                                            jedec_info_table_t *tbl,
                                            serial_nor_config_option_t *option)
{
    status_t status = kStatus_InvalidArgument;
    uint32_t lut_seq[4];

    // See JESD216B 6.4.18 for more details.
    do
    {
        // Enter 0-4-4 mode
        flexspi_xfer_t xfer;
        uint32_t status_val = 0;
        memset(&lut_seq, 0, sizeof(lut_seq));
        xfer.baseAddress = 0;
        xfer.isParallelModeEnable = false;
        xfer.operation = kFlexSpiOperation_Read;
        xfer.rxBuffer = &status_val;
        xfer.rxSize = 1;
        xfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
        xfer.seqNum = 1;
        lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x85, READ_SDR, FLEXSPI_1PAD, 0x01);
        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &lut_seq[0], 1);
        status = flexspi_command_xfer(instance, &xfer);
        if (status != kStatus_Success)
        {
            break;
        }
        status_val &= (uint8_t)~FLEXSPI_BITMASK(3);

        // Do modify-afer-read status and then create 0-4-4 mode entry sequence
        config->memConfig.deviceModeCfgEnable = true;
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_RESTORE_NOCMD] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x81, CMD_SDR, FLEXSPI_1PAD, status_val);
        config->memConfig.deviceModeSeq.seqNum = 1;
        config->memConfig.deviceModeSeq.seqId = NOR_CMD_LUT_SEQ_IDX_RESTORE_NOCMD;
        config->memConfig.deviceModeArg = status_val; // No care
        config->memConfig.deviceModeType = kDeviceConfigCmdType_Spi2NoCmd;
        config->needRestoreNoCmdMode = true;

        status = kStatus_Success;

    } while (0);

    return status;
}

status_t probe_dtr_quad_read_dummy_cycles(uint32_t instance, flexspi_nor_config_t *config, uint32_t *dummy_cycles)
{
    status_t status = kStatus_InvalidArgument;
    bool dummy_cycle_detected = false;
    flexspi_xfer_t flashXfer;
    uint32_t lut_seq[4];
    // This is the reserved region in the first 4KB header, here the region is used for DDR dummy cycle probe
    flashXfer.baseAddress = sizeof(flexspi_nor_config_t);

    flashXfer.isParallelModeEnable = false;
    flashXfer.seqId = 0;
    flashXfer.seqNum = 1;
    do
    {
        if (NULL == config)
        {
            break;
        }

        const uint32_t probe_pattern[4] = { 0x33221100, 0x77665544, 0xbbaa9988, 0xffeeddcc };
        uint32_t buffer[4];
        flashXfer.operation = kFlexSpiOperation_Read;
        flashXfer.rxBuffer = buffer;
        flashXfer.rxSize = sizeof(buffer);
        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &config->memConfig.lookupTable[4 * flashXfer.seqId],
                           flashXfer.seqNum);
        flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(instance, &flashXfer);
        if (status != kStatus_Success)
        {
            break;
        }
        bool need_program_pattern = true;
        if ((buffer[0] != 0xFFFFFFFFu) || (buffer[1] != 0xFFFFFFFFu) || (buffer[2] != 0xFFFFFFFFu) ||
            (buffer[3] != 0xFFFFFFFFu))
        {
            if (memcmp(probe_pattern, buffer, sizeof(buffer)) != 0)
            {
                status = kStatus_FLEXSPINOR_DTRRead_DummyProbeFailed;
                break;
            }
            need_program_pattern = false;
        }

        if (need_program_pattern)
        {
            // Write Enable
            status = flexspi_nor_write_enable(instance, config, false, 0);
            if (status != kStatus_Success)
            {
                break;
            }

            // Program Pattern
            flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM;
            flashXfer.operation = kFlexSpiOperation_Write;
            flashXfer.txBuffer = (uint32_t *)probe_pattern;
            flashXfer.txSize = sizeof(probe_pattern);
            flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, &config->memConfig.lookupTable[4 * flashXfer.seqId],
                               flashXfer.seqNum);
            flashXfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
            status = flexspi_command_xfer(instance, &flashXfer);
            if (status != kStatus_Success)
            {
                break;
            }

            // Read status
            status = flexspi_device_wait_busy(instance, &config->memConfig, false, 0);
            if (status != kStatus_Success)
            {
                break;
            }

            // Read back
            flashXfer.seqId = 0;
            flashXfer.operation = kFlexSpiOperation_Read;
            flashXfer.rxBuffer = buffer;
            flashXfer.rxSize = sizeof(buffer);
            status = flexspi_command_xfer(instance, &flashXfer);
            if (status != kStatus_Success)
            {
                break;
            }
            if (memcmp(probe_pattern, buffer, sizeof(buffer)) != 0)
            {
                status = kStatus_FLEXSPINOR_DTRRead_DummyProbeFailed;
                break;
            }
        }

        // Prepare to probe
        uint32_t max_probe_try = 50;
        uint32_t probe_cnt = 0;
        uint32_t probe_dummy_cycles = 2;
        // Configure clock
        config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable);
        flexspi_clock_config(instance, config->memConfig.serialClkFreq, true);
        while ((!dummy_cycle_detected) && (++probe_cnt < max_probe_try))
        {
            memset(lut_seq, 0, sizeof(lut_seq));
            if (config->memConfig.sflashA1Size > MAX_24BIT_ADDRESSING_SIZE)
            {
                lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_Read_DDR_1_4_4_4B, RADDR_DDR,
                                             FLEXSPI_4PAD, 0x20);
            }
            else
            {
                lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_Read_DDR_1_4_4_3B, RADDR_DDR,
                                             FLEXSPI_4PAD, 0x18);
            }
            lut_seq[1] = FLEXSPI_LUT_SEQ(DUMMY_DDR, FLEXSPI_4PAD, probe_dummy_cycles, READ_DDR, FLEXSPI_4PAD, 0x04);
            flexspi_update_lut(instance, 0, lut_seq, 1);
            flashXfer.seqId = 0;
            flashXfer.operation = kFlexSpiOperation_Read;
            flashXfer.rxBuffer = buffer;
            flashXfer.rxSize = sizeof(buffer);
            status = flexspi_command_xfer(instance, &flashXfer);
            if (status != kStatus_Success)
            {
                break;
            }

            if (memcmp(probe_pattern, buffer, sizeof(buffer)) == 0)
            {
                dummy_cycle_detected = true;
                break;
            }
            else
            {
                probe_dummy_cycles++;
            }
        }

        if (dummy_cycle_detected)
        {
            *dummy_cycles = probe_dummy_cycles;

            break;
        }
    } while (0);

    if (!dummy_cycle_detected)
    {
        status = kStatus_FLEXSPINOR_DTRRead_DummyProbeFailed;
    }

    return status;
}

status_t get_page_sector_block_size_from_sfdp(flexspi_nor_config_t *config,
                                              jedec_info_table_t *tbl,
                                              uint32_t *sector_erase_cmd,
                                              uint32_t *block_erase_cmd)
{
    jedec_flash_param_table_t *param_tbl = &tbl->flash_param_tbl;
    jedec_4byte_addressing_inst_table_t *flash_4b_tbl = &tbl->flash_4b_inst_tbl;

    // Calculate Flash Size
    uint32_t flash_size;
    uint32_t flash_density = tbl->flash_param_tbl.flash_density;

    if (flash_density & (1U << 0x1F))
    {
        // Flash size >= 4G bits
        flash_size = 1U << ((flash_density & ~(1U << 0x1F)) - 3);
    }
    else
    {
        // Flash size < 4G bits
        flash_size = (flash_density + 1) >> 3;
    }

    uint32_t defaultFlashSize = 0;
    uint32_t *flashSizeArray = &config->memConfig.sflashA1Size;

    for (uint32_t i = 0; i < 4; i++)
    {
        if (*flashSizeArray != 0)
        {
            defaultFlashSize = *flashSizeArray;
            break;
        }
        else
        {
            ++flashSizeArray;
        }
    }
    if (defaultFlashSize < 1)
    {
        return kStatus_InvalidArgument;
    }
    *flashSizeArray = flash_size;

    // Calculate Page size
    uint32_t page_size;
    if (tbl->flash_param_tbl_size < kSfdp_BasicProtocolTableSize_RevA)
    {
        config->pageSize = 256;
    }
    else
    {
        page_size = 1u << (param_tbl->chip_erase_progrm_info.page_size);
        config->pageSize = (page_size == (1u << 15)) ? 256 : page_size;
    }

    // Calculate Sector Size;
    uint32_t sector_size = 0xFFFFFFu;
    uint32_t block_size = 0u;
    uint32_t block_erase_type = 0u;
    uint32_t sector_erase_type = 0u;

    for (uint32_t index = 0; index < 4; index++)
    {
        if (param_tbl->erase_info[index].size != 0)
        {
            uint32_t current_erase_size = 1U << param_tbl->erase_info[index].size;
            if (current_erase_size < sector_size)
            {
                sector_size = current_erase_size;
                sector_erase_type = index;
            }
            if ((current_erase_size > block_size) && (current_erase_size < (1024U * 1024U)))
            {
                block_size = current_erase_size;
                block_erase_type = index;
            }
        }
    }

    config->sectorSize = sector_size;
    config->blockSize = block_size;

    if (sector_size == block_size)
    {
        config->isUniformBlockSize = true;
    }
    else
    {
        config->isUniformBlockSize = false;
    }

    if (*flashSizeArray > MAX_24BIT_ADDRESSING_SIZE)
    {
        if (tbl->has_4b_addressing_inst_table)
        {
            *sector_erase_cmd = flash_4b_tbl->erase_inst_info.erase_inst[sector_erase_type];
            *block_erase_cmd = flash_4b_tbl->erase_inst_info.erase_inst[block_erase_type];
        }
        else
        {
            switch (param_tbl->erase_info[sector_erase_type].inst)
            {
                case kSerialNorCmd_SE4K_3B:
                    *sector_erase_cmd = kSerialNorCmd_SE4K_4B;
                    break;
                case kSerialNorCmd_SE64K_3B:
                    *sector_erase_cmd = kSerialNorCmd_SE64K_4B;
                    break;
            }
            switch (param_tbl->erase_info[block_erase_type].inst)
            {
                case kSerialNorCmd_SE4K_3B:
                    *block_erase_cmd = kSerialNorCmd_SE4K_4B;
                    break;
                case kSerialNorCmd_SE64K_3B:
                    *block_erase_cmd = kSerialNorCmd_SE64K_4B;
                    break;
            }
        }
    }
    else
    {
        *sector_erase_cmd = param_tbl->erase_info[sector_erase_type].inst;
        *block_erase_cmd = param_tbl->erase_info[block_erase_type].inst;
    }

    return kStatus_Success;
}

// Parse SFDP parameters and then fill into FlexSPI Serial NOR Configuration Block
status_t parse_sfdp(uint32_t instance,
                    flexspi_nor_config_t *config,
                    jedec_info_table_t *tbl,
                    serial_nor_config_option_t *option)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        jedec_flash_param_table_t *param_tbl = &tbl->flash_param_tbl;
        jedec_4byte_addressing_inst_table_t *flash_4b_tbl = &tbl->flash_4b_inst_tbl;

        // Check whether DDR mode is supported.
        bool support_ddr_mode = false;
        if (option->option0.B.device_type == kSerialNorCfgOption_DeviceType_ReadSFDP_DDR)
        {
            support_ddr_mode = param_tbl->misc.support_ddr_clocking;
            if (!support_ddr_mode)
            {
                break;
            }
        }

        config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackInternally;
        config->memConfig.serialClkFreq = option->option0.B.max_freq;

        uint8_t read_cmd;
        uint32_t dummy_cycles = 0;
        uint8_t mode_cycles = 0;
        uint32_t sector_erase_cmd;
        uint32_t block_erase_cmd;
        uint32_t address_bits = 24;
        uint32_t address_pads = FLEXSPI_1PAD;

        get_page_sector_block_size_from_sfdp(config, tbl, &sector_erase_cmd, &block_erase_cmd);

        if (config->memConfig.sflashA1Size > MAX_24BIT_ADDRESSING_SIZE)
        {
            address_bits = 32;
        }
        uint32_t cmd_pads = option->option0.B.cmd_pads;
        if (cmd_pads == FLEXSPI_1PAD)
        {
            // Prepare Quad Mode enable sequence as needed.
            status = prepare_quad_mode_enable_sequence(instance, config, tbl, option);
            if (status != kStatus_Success)
            {
                break;
            }

            // Determine Read command based on SFDP
            if (param_tbl->misc.supports_1_4_4_fast_read)
            {
                address_pads = FLEXSPI_4PAD;
                mode_cycles = param_tbl->read_1_4_info.mode_clocks_1_4_4_read;
                dummy_cycles = param_tbl->read_1_4_info.dummy_clocks_1_4_4_read;
            }
            else if (param_tbl->misc.support_1_1_4_fast_read)
            {
                mode_cycles = param_tbl->read_1_4_info.mode_clocks_1_1_4_read;
                dummy_cycles = param_tbl->read_1_4_info.dummy_clocks_1_1_4_read;
            }

            // Page Program
            if (address_bits == 32)
            {
                if (tbl->has_4b_addressing_inst_table)
                {
                    if (flash_4b_tbl->cmd_4byte_support_info.support_1_4_4_page_program)
                    {
                        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] = FLEXSPI_LUT_SEQ(
                            CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_PageProgram_1_4_4_4B, RADDR_SDR, FLEXSPI_4PAD, 32);
                        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                            FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_4PAD, 0x04, STOP, FLEXSPI_1PAD, 0);
                    }
                    else if (flash_4b_tbl->cmd_4byte_support_info.support_1_1_4_page_program)
                    {
                        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] = FLEXSPI_LUT_SEQ(
                            CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_PageProgram_1_1_4_4B, RADDR_SDR, FLEXSPI_1PAD, 32);
                        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                            FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_4PAD, 0x04, STOP, FLEXSPI_1PAD, 0);
                    }
                    else // 1_1_1_page_program
                    {
                        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] = FLEXSPI_LUT_SEQ(
                            CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_PageProgram_1_1_1_4B, RADDR_SDR, FLEXSPI_1PAD, 32);
                        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                            FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0);
                    }
                }
                else
                {
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] = FLEXSPI_LUT_SEQ(
                        CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_PageProgram_1_1_1_4B, RADDR_SDR, FLEXSPI_1PAD, 32);
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                        FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0);
                }
            }
            else // Only consider 1-1-1 Program
            {
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] = FLEXSPI_LUT_SEQ(
                    CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_PageProgram_1_1_1_3B, RADDR_SDR, FLEXSPI_1PAD, 24);
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                    FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0);
            }
        }
        else
        {
            break;
        }

        config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackFromDqsPad;

        // Write Enable
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_WriteEnable, STOP, FLEXSPI_1PAD, 0);
        // ReadStatus
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_ReadStatusReg1, READ_SDR, FLEXSPI_1PAD, 0x04);
        // Erase Sector
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, sector_erase_cmd, RADDR_SDR, FLEXSPI_1PAD, address_bits);

        // Erase Block
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, block_erase_cmd, RADDR_SDR, FLEXSPI_1PAD, address_bits);

        // Erase All
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialNorCmd_ChipErase, STOP, FLEXSPI_1PAD, 0);

        // Calculate dummy cycles
        if (option->option0.B.option_size && option->option1.B.dummy_cycles != 0)
        {
            mode_cycles = 0;
            dummy_cycles = option->option1.B.dummy_cycles;
            if (support_ddr_mode)
            {
                dummy_cycles *= 2;
            }
        }
        // Try to do ddr dummy probe for ddr read when the dummy cycle is not provided
        else if (support_ddr_mode)
        {
            if (address_bits == 32)
            {
                // Basic read command
                config->memConfig.lookupTable[0] =
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x13, RADDR_SDR, FLEXSPI_1PAD, 32);
                config->memConfig.lookupTable[1] = FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 1);
            }

            status = flexspi_nor_flash_init(instance, config);
            if (status != kStatus_Success)
            {
                break;
            }
            // Probe dummy_cycles
            status = probe_dtr_quad_read_dummy_cycles(instance, config, &dummy_cycles);
            if (status != kStatus_Success)
            {
                // Cannot probe dummy cycle for DDR read
                break;
            }
            mode_cycles = 0;
        }

        // Generate Flash Read sequence
        if (address_bits == 24)
        {
            if (support_ddr_mode)
            {
                read_cmd = kSerialNorCmd_Read_DDR_1_4_4_3B;
            }
            else if (param_tbl->misc.supports_1_4_4_fast_read)
            {
                read_cmd = param_tbl->read_1_4_info.inst_1_4_4_read;
            }
            else if (param_tbl->misc.support_1_1_4_fast_read)
            {
                read_cmd = param_tbl->read_1_4_info.inst_1_1_4_read;
            }
            else // Use basic read if the Quad Read is not supported
            {
                read_cmd = kSerialNorCmd_BasicRead_3B;
                dummy_cycles = 0;
                mode_cycles = 0;
            }
        }
        else // 32bit addressing mode
        {
            if (support_ddr_mode)
            {
                read_cmd = kSerialNorCmd_Read_DDR_1_4_4_4B;
                address_pads = FLEXSPI_4PAD;
            }
            else if (tbl->has_4b_addressing_inst_table)
            {
                if (flash_4b_tbl->cmd_4byte_support_info.support_1_4_4_fast_read)
                {
                    read_cmd = kSerialNorCmd_Read_SDR_1_4_4_4B;
                    address_pads = FLEXSPI_4PAD;
                }
                else if (flash_4b_tbl->cmd_4byte_support_info.support_1_1_4_fast_read)
                {
                    read_cmd = kSerialNorCmd_Read_SDR_1_1_4_4B;
                    address_pads = FLEXSPI_1PAD;
                }
                else
                {
                    read_cmd = kSerialNorCmd_BasicRead_4B;
                    dummy_cycles = 0;
                    mode_cycles = 0;
                }
            }
            // For device that is only compliant with JESD216
            else if (param_tbl->misc.supports_1_4_4_fast_read)
            {
                read_cmd = kSerialNorCmd_Read_SDR_1_4_4_4B;
            }
            else if (param_tbl->misc.support_1_1_4_fast_read)
            {
                read_cmd = kSerialNorCmd_Read_SDR_1_1_4_4B;
            }
            else
            {
                read_cmd = kSerialNorCmd_BasicRead_4B;
                dummy_cycles = 0;
                mode_cycles = 0;
            }
        }

        // Read LUT
        if (support_ddr_mode)
        {
            config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable);
            config->memConfig.lookupTable[0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, read_cmd, RADDR_DDR, FLEXSPI_4PAD, address_bits);
        }
        else
        {
            config->memConfig.lookupTable[0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, read_cmd, RADDR_SDR, address_pads, address_bits);
        }

        uint32_t enhance_mode = 0x00;
        uint8_t misc_mode = option->option0.B.misc_mode;
        if (misc_mode == kSerialNorEnhanceMode_Disabled)
        {
            // Treat mode cycles as dummy cycles
            dummy_cycles += mode_cycles;
            mode_cycles = 0;
        }
#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        else if (misc_mode == kSerialNorEnhanceMode_0_4_4_Mode)
        {
            // Cannot detect the 0-4-4 mode entry method, disable 0-4-4 mode
            if ((tbl->flash_param_tbl_size < kSfdp_BasicProtocolTableSize_RevA) ||
                (!param_tbl->mode_4_4_info.support_mode_0_4_4))
            {
                dummy_cycles += mode_cycles;
                mode_cycles = 0;
            }
            else
            {
                uint32_t entry_method = param_tbl->mode_4_4_info.mode_0_4_4_entry_method;

                if ((entry_method & 0x01) || (entry_method & 0x04))
                {
                    enhance_mode = 0xA5;
                }
                else if (entry_method & 0x02)
                {
                    status = prepare_0_4_4_mode_enable_sequence(instance, config, tbl, option);
                    if (status != kStatus_Success)
                    {
                        break;
                    }
                    enhance_mode = 0x01;
                }
                // Refer to JESD216B 6.4.18 for more details.
                uint32_t exit_method = param_tbl->mode_4_4_info.mode_0_4_4_exit_method;
                if ((exit_method & 0x02) || (exit_method & 0x08))
                {
                    // Send 8-10 cycles of 0xF depends on addressing mode
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, 0xFF, CMD_SDR, FLEXSPI_4PAD, 0xFF);
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD + 1] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, 0xFF, CMD_SDR, FLEXSPI_4PAD, 0xFF);
                    if (address_bits == 32)
                    {
                        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD + 2] =
                            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, 0xFF, STOP, FLEXSPI_1PAD, 0);
                    }
                }
                else if ((exit_method & 0x01) || (exit_method & 0x10))
                {
                    // Use command to simulate read access to 0 using mode 0x00
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, 0x00, CMD_SDR, FLEXSPI_4PAD, 0x00);
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD + 1] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, 0x00, CMD_SDR, FLEXSPI_4PAD, 0x00);
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD + 2] =
                        FLEXSPI_LUT_SEQ(MODE8_SDR, FLEXSPI_4PAD, 0x00, DUMMY_SDR, FLEXSPI_4PAD, 0x10);
                }
                else
                {
                    break;
                }

                config->needExitNoCmdMode = true;
            }
        }
#endif // FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        else if (misc_mode == kSerialNorEnhanceMode_InternalLoopback)
        {
            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackInternally;
        }
        else
        {
            // Do nothing
        }

        if (mode_cycles == 0)
        {
            if (dummy_cycles > 0)
            {
                if (support_ddr_mode)
                {
                    config->memConfig.lookupTable[1] =
                        FLEXSPI_LUT_SEQ(DUMMY_DDR, FLEXSPI_4PAD, dummy_cycles, READ_DDR, FLEXSPI_4PAD, 0x04);
                }
                else
                {
                    config->memConfig.lookupTable[1] =
                        FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, dummy_cycles, READ_SDR, FLEXSPI_4PAD, 0x04);
                }
            }
            else // Only applicable to basic read command, all other read commands require dummy cycles
            {
                config->memConfig.lookupTable[1] =
                    FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0);
            }
        }
        else
        {
            uint32_t mode_inst;
#if 0 // Comment out this segment because in JESD216A/B, below logic cannot happen, keep the codes here in case it can
      // be used for later JESD216 revision
            if (support_ddr_mode)
            {
                if (mode_cycles == 1)
                {
                    mode_inst = MODE4_DDR;
                    enhance_mode >>= 4;
                }
                else
                {
                    mode_inst = MODE8_DDR;
                }
                config->memConfig.lookupTable[1] =
                    FLEXSPI_LUT_SEQ(mode_inst, FLEXSPI_4PAD, enhance_mode, DUMMY_DDR, FLEXSPI_4PAD, dummy_cycles);
                config->memConfig.lookupTable[2] =
                    FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_4PAD, 0x04, JMP_ON_CS, FLEXSPI_1PAD, 1);
            }
            else
#endif
            {
                if (mode_cycles == 1)
                {
                    mode_inst = MODE4_SDR;
                    enhance_mode >>= 4;
                }
                else
                {
                    mode_inst = MODE8_SDR;
                }
                config->memConfig.lookupTable[1] =
                    FLEXSPI_LUT_SEQ(mode_inst, FLEXSPI_4PAD, enhance_mode, DUMMY_SDR, FLEXSPI_4PAD, dummy_cycles);
                config->memConfig.lookupTable[2] =
                    FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_4PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0);
            }
        }

        // For QuadSPI Flash, only the Read command is DDR instruction, other commands are all DDR instructions,
        // In FlexSPI design, the clock for DDR mode must be configured as 2 * real clock, so when the system is
        // configured to
        // DDR mode, if the non-read commands are executed, the clock is 2 * real clock, this may cause the clock for
        // SDR command
        // to exceed the spec, so, here we introduce the halfClkForNonReadCmd field, once this field is set, FlexSPI
        // Driver
        // will half the clock for all non-read instruction to ensure the clock for these commands meets SPEC.
        if (support_ddr_mode)
        {
            config->halfClkForNonReadCmd = true;
            config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable);
        }
        // Always enable Safe configuration Frequency
        config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);

        status = kStatus_Success;
    } while (0);

    return status;
}

status_t flexspi_nor_read_sfdp_info(uint32_t instance, jedec_info_table_t *tbl, bool address_shift_enable)
{
    status_t status = kStatus_FlexSPINOR_SFDP_NotFound;
    do
    {
        if (tbl == NULL)
        {
            status = kStatus_InvalidArgument;
            break;
        }

        sfdp_header_t sfdp_header;
        uint32_t address;
        status = flexspi_nor_read_sfdp(instance, 0, (uint32_t *)&sfdp_header, sizeof(sfdp_header));
        if (status != kStatus_Success)
        {
            break;
        }

        if (sfdp_header.signature != SFDP_SIGNATURE)
        {
            status = kStatus_FlexSPINOR_SFDP_NotFound;
            break;
        }

        // Save the standard version for later use.
        tbl->standard_version = sfdp_header.minor_rev;

        uint32_t parameter_header_number = sfdp_header.param_hdr_num + 1;

        sfdp_parameter_header_t sfdp_param_hdrs[10];
        uint32_t max_hdr_count = parameter_header_number > 10 ? 10 : parameter_header_number;
        address = 0x08;
        if (address_shift_enable)
        {
            address <<= 8;
        }
        status = flexspi_nor_read_sfdp(instance, address, (uint32_t *)&sfdp_param_hdrs[0],
                                       max_hdr_count * sizeof(sfdp_parameter_header_t));
        if (status != kStatus_Success)
        {
            break;
        }
        memset(tbl, 0, sizeof(*tbl));

        for (uint32_t i = 0; i < max_hdr_count; i++)
        {
            uint32_t parameter_id =
                sfdp_param_hdrs[i].parameter_id_lsb + ((uint32_t)sfdp_param_hdrs[i].parameter_id_msb << 8);

            if ((parameter_id == kParameterID_BasicSpiProtocol) ||
                (parameter_id == kParameterID_4ByteAddressInstructionTable))
            {
                address = 0;
                for (int32_t index = 2; index >= 0; index--)
                {
                    address <<= 8;
                    address |= sfdp_param_hdrs[i].parameter_table_pointer[index];
                }
                uint32_t table_size = sfdp_param_hdrs[i].table_length_in_32bit * sizeof(uint32_t);

                if (address_shift_enable)
                {
                    address <<= 8;
                }

                if (parameter_id == kParameterID_BasicSpiProtocol)
                {
                    // Limit table size to the max supported standard
                    if (table_size > sizeof(jedec_flash_param_table_t))
                    {
                        table_size = sizeof(jedec_flash_param_table_t);
                    }
                    status = flexspi_nor_read_sfdp(instance, address, (uint32_t *)&tbl->flash_param_tbl, table_size);
                    if (status != kStatus_Success)
                    {
                        break;
                    }
                    tbl->flash_param_tbl_size = table_size;
                }
                else if (parameter_id == kParameterID_4ByteAddressInstructionTable)
                {
                    status = flexspi_nor_read_sfdp(instance, address, (uint32_t *)&tbl->flash_4b_inst_tbl, table_size);
                    if (status != kStatus_Success)
                    {
                        break;
                    }
                    tbl->has_4b_addressing_inst_table = true;
                }
            }
            else
            {
                // Unsupported parameter type, ignore
            }
        }

    } while (0);

    return status;
}

status_t flexspi_nor_generate_config_block_using_sfdp(uint32_t instance,
                                                      flexspi_nor_config_t *config,
                                                      serial_nor_config_option_t *option)
{
    status_t status = kStatus_InvalidArgument;

    const lut_seq_t k_sdfp_lut[4] = {
        // Read SFDP LUT sequence for 1 pad instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialFlash_ReadSFDP, RADDR_SDR, FLEXSPI_1PAD, 24),
          FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_1PAD, 8, READ_SDR, FLEXSPI_1PAD, 0xFF), 0, 0 },

        // Read SFDP LUT sequence for 2 pad instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_2PAD, kSerialFlash_ReadSFDP, RADDR_SDR, FLEXSPI_2PAD, 24),
          FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_2PAD, 8, READ_SDR, FLEXSPI_2PAD, 0xFF), 0, 0 },

        // Read SFDP LUT sequence for 4 pad instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, kSerialFlash_ReadSFDP, RADDR_SDR, FLEXSPI_4PAD, 24),
          FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 8, READ_SDR, FLEXSPI_4PAD, 0xFF), 0, 0 },
    };

    do
    {
        if (option->option0.B.query_pads != FLEXSPI_1PAD)
        {
            break;
        }

        config->memConfig.sflashPadType = kSerialFlash_4Pads;
        // Basic read command
        config->memConfig.lookupTable[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x03, RADDR_SDR, FLEXSPI_1PAD, 24);
        config->memConfig.lookupTable[1] = FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 1);

        status = flexspi_nor_flash_init(instance, config);
        if (status != kStatus_Success)
        {
            break;
        }
#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        if (option->option0.B.misc_mode == kSerialNorEnhanceMode_0_4_4_Mode)
        {
            // Try to exit 0-4-4 mode
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, 0xFF, CMD_SDR, FLEXSPI_4PAD, 0xFF);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD + 1] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, 0xFF, CMD_SDR, FLEXSPI_4PAD, 0xFF);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD + 2] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, 0xFF, STOP, FLEXSPI_1PAD, 0);

            status = flexspi_nor_exit_no_cmd_mode(instance, config, false, 0);
            if (status != kStatus_Success)
            {
                break;
            }
        }
#endif // FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT

        // Read SFDP, probe whether the Flash device is present or not.
        jedec_info_table_t jedec_info_tbl;
        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD,
                           (const uint32_t *)&k_sdfp_lut[option->option0.B.query_pads], 1);
        status = flexspi_nor_read_sfdp_info(instance, &jedec_info_tbl, false);
        if (status != kStatus_Success)
        {
            break;
        }

#if 0 // Disable this feature for now, because it introduces side effect sometimes.
        /* If it presents, probe whether there is a valid config block located at the start of NOR FLASH
         *
         * If the block is valid, just read it to the config block buffer, else parse SFDP table to generate
         * config block
         */
        uint32_t buffer[4];
        flexspi_xfer_t xfer;
        xfer.operation = kFlexSpiOperation_Read;
        xfer.seqNum = 1;
        xfer.seqId = NOR_CMD_LUT_SEQ_IDX_READ;
        xfer.isParallelModeEnable = false;
        xfer.baseAddress = 0;
        xfer.rxBuffer = buffer;
        xfer.rxSize = sizeof(buffer);
        status = flexspi_command_xfer(instance, &xfer);
        if (status != kStatus_Success)
        {
            break;
        }
        if (buffer[0] == FLEXSPI_CFG_BLK_TAG)
        {
            xfer.rxBuffer = (uint32_t *)config;
            xfer.rxSize = sizeof(*config);
            status = flexspi_command_xfer(instance, &xfer);
            if (status != kStatus_Success)
            {
                break;
            }

            // Check wither the pageSize and sectorSize argument are valid, if fo, abort sdfp parsing process
            if ((config->pageSize != 0xFFFFFFFF) && (config->sectorSize != 0xFFFFFFFF))
            {
                break;
            }
            else
            {
                // Restore config block to default and re-try sfdp parsing process
                memset(config, 0, sizeof(*config));
                config->memConfig.tag = FLEXSPI_CFG_BLK_TAG;
                config->memConfig.version = FLEXSPI_CFG_BLK_VERSION;
                config->memConfig.serialClkFreq = kFlexSpiSerialClk_SafeFreq;
                config->memConfig.sflashA1Size = MAX_24BIT_ADDRESSING_SIZE;
            }
        }
#endif
        status = parse_sfdp(instance, config, &jedec_info_tbl, option);
        if (status == kStatus_Success)
        {
            if (flexspi_is_parallel_mode(&config->memConfig))
            {
                config->memConfig.sflashB1Size = config->memConfig.sflashA1Size;
                config->pageSize *= 2;
                config->sectorSize *= 2;
                config->blockSize *= 2;
            }
        }

    } while (0);

    return status;
}

status_t flexspi_nor_hyperbus_read(uint32_t instance, uint32_t addr, uint32_t *buffer, uint32_t bytes)
{
    flexspi_xfer_t xfer;
    xfer.operation = kFlexSpiOperation_Read;
    xfer.seqId = 0;
    xfer.seqNum = 1;
    xfer.baseAddress = addr * 2;
    xfer.isParallelModeEnable = false;
    xfer.rxBuffer = buffer;
    xfer.rxSize = bytes;

    status_t status = flexspi_command_xfer(instance, &xfer);

    return status;
}

status_t flexspi_nor_hyperbus_write(uint32_t instance, uint32_t addr, uint32_t *buffer, uint32_t bytes)
{
    flexspi_xfer_t xfer;
    xfer.operation = kFlexSpiOperation_Write;
    xfer.seqId = 1;
    xfer.seqNum = 1;
    xfer.baseAddress = addr * 2;
    xfer.isParallelModeEnable = false;
    xfer.txBuffer = buffer;
    xfer.txSize = bytes;

    status_t status = flexspi_command_xfer(instance, &xfer);

    return status;
}

status_t flexspi_nor_generate_config_block_hyperflash(uint32_t instance, flexspi_nor_config_t *config, bool is_1v8)
{
    status_t status = kStatus_FLEXSPINOR_Flash_NotFound;
    do
    {
        config->memConfig.columnAddressWidth = 3;
        config->memConfig.sflashPadType = kSerialFlash_8Pads;
        config->memConfig.controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable) |
                                                 FLEXSPI_BITMASK(kFlexSpiMiscOffset_WordAddressableEnable) |
                                                 FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);
        config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;

        if (is_1v8)
        {
            config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_DiffClkEnable);
        }
        config->memConfig.lutCustomSeqEnable = true;
        config->memConfig.busyOffset = 15;
        config->memConfig.busyBitPolarity = 1;

        config->pageSize = 512;
        config->sectorSize = 256 * 1024;
        config->blockSize = config->sectorSize;
        config->isUniformBlockSize = true;

        uint32_t lut_seq[4];

        status = flexspi_nor_flash_init(instance, config);
        if (status != kStatus_Success)
        {
            break;
        }

        memset(lut_seq, 0, sizeof(lut_seq));
        // Write
        lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x20, RADDR_DDR, FLEXSPI_8PAD, 0x18);
        lut_seq[1] = FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, WRITE_DDR, FLEXSPI_8PAD, 0x02);
        flexspi_update_lut(instance, 1, lut_seq, 1);

        // Read
        memset(lut_seq, 0, sizeof(lut_seq));
        lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0, RADDR_DDR, FLEXSPI_8PAD, 0x18);
        lut_seq[1] = FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, READ_DDR, FLEXSPI_8PAD, 0x04);
        flexspi_update_lut(instance, 0, lut_seq, 1);

        /*
         * Read ID-CFI Parameters,be aware that data stored on HyperFLASH are 16bit big-endian, so need to be swapped.
         */
        // CFI Entry
        uint32_t data[1] = { 0x9800 };
        status = flexspi_nor_hyperbus_write(instance, 0x555, &data[0], 2);
        if (status != kStatus_Success)
        {
            break;
        }
        // ID-CFI Read
        uint32_t buffer[2];
        // Read Query Unique ASCII String
        status = flexspi_nor_hyperbus_read(instance, 0x10, &buffer[0], sizeof(buffer));
        if (status != kStatus_Success)
        {
            break;
        }
        buffer[1] &= 0xFFFF;
        // Check that the data read out is  unicode "QRY" in big-endian order
        if ((buffer[0] != 0x52005100) || (buffer[1] != 0x5900))
        {
            status = kStatus_FLEXSPINOR_Flash_NotFound;
            break;
        }
        // Read Flash density
        status = flexspi_nor_hyperbus_read(instance, 0x27, &buffer[0], sizeof(buffer));
        if (status != kStatus_Success)
        {
            break;
        }
        buffer[0] &= 0xFFFF;
        buffer[0] >>= 8;
        if ((buffer[0] >= 0x17) && (buffer[0] <= 0x20))
        {
            config->memConfig.sflashA1Size = 1UL << buffer[0];
        }
        else
        {
            status = kStatus_FLEXSPINOR_Flash_NotFound;
            break;
        }

        // ASO Exit
        data[0] = 0xF000;
        status = flexspi_nor_hyperbus_write(instance, 0x0, &data[0], 2);
        if (status != kStatus_Success)
        {
            break;
        }

        // Read
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0, RADDR_DDR, FLEXSPI_8PAD, 0x18);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] =
            FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, DUMMY_DDR, FLEXSPI_8PAD, 0x0c);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 2] =
            FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0);

        // Read Status
        // 0
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 1] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 2] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 3] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x70);
        // 1
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 4] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0, RADDR_DDR, FLEXSPI_8PAD, 0x18);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 5] =
            FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, DUMMY_RWDS_DDR, FLEXSPI_8PAD, 0x0B);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 6] =
            FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04, STOP, FLEXSPI_1PAD, 0x00);

        // Write Enable
        // 0
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 1] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 2] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 3] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        // 1
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 4] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 5] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x55);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 6] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x02);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 7] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x55);

        // Page Program
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 2] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 3] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xA0);
        // 1
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 4] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, RADDR_DDR, FLEXSPI_8PAD, 0x18);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 5] =
            FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, WRITE_DDR, FLEXSPI_8PAD, 0x80);

        // Erase Sector
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 1] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 2] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 3] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x80);
        // 1
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 4] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 5] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 6] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 7] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        // 2
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 8] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 9] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x55);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 10] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x02);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 11] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x55);
        // 3
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 12] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, RADDR_DDR, FLEXSPI_8PAD, 0x18);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 13] =
            FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 14] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x30, STOP, FLEXSPI_1PAD, 0x0);

        // Erase Chip
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 1] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 2] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 3] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x80);
        // 1
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 4] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 5] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 6] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 7] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        // 2
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 8] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 9] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x55);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 10] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x02);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 11] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x55);
        // 3
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 12] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 13] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 14] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 15] =
            FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x10);

        // LUT customized sequence
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_WRITEENABLE].seqNum = 2;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_WRITEENABLE].seqId = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_READSTATUS].seqNum = 2;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_READSTATUS].seqId = NOR_CMD_LUT_SEQ_IDX_READSTATUS;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_PAGEPROGRAM].seqNum = 2;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_PAGEPROGRAM].seqId = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_ERASESECTOR].seqNum = 4;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_ERASESECTOR].seqId = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_CHIPERASE].seqNum = 4;
        config->memConfig.lutCustomSeq[NOR_CMD_INDEX_CHIPERASE].seqId = NOR_CMD_LUT_SEQ_IDX_CHIPERASE;

        // New field introduced since FlexSPI NOR Driver 1.3
        config->serialNorType = kSerialNorType_HyperBus;
        status = kStatus_Success;
    } while (0);

    return status;
}

status_t flexspi_nor_generate_config_block_mxic_octalflash(uint32_t instance,
                                                           flexspi_nor_config_t *config,
                                                           serial_nor_config_option_t *option)
{
    status_t status = kStatus_FLEXSPINOR_Flash_NotFound;
    flexspi_xfer_t xfer;
    uint32_t mfg_id[2];

    bool is_sdr_mode = option->option0.B.device_type == kSerialNorCfgOption_DeviceType_MacronixOctalSDR;

    // SFDP table is not programmed in OctaFlash yet, so use RDID command instead
    const lut_seq_t k_rdid_lut[3] = {
        // Read Identification LUT sequence for 1 pad instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialFlash_ReadManufacturerId, READ_SDR, FLEXSPI_1PAD, 3), 0, 0, 0 },

        // Read Identification LUT sequence for OPI SDR instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, kSerialFlash_ReadManufacturerId, CMD_SDR, FLEXSPI_8PAD, 0x60),
          FLEXSPI_LUT_SEQ(RADDR_SDR, FLEXSPI_8PAD, 0x20, READ_SDR, FLEXSPI_8PAD, 0x4), 0, 0 },

        // Read Identification LUT sequence for OPI DDR instruction
        { FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, kSerialFlash_ReadManufacturerId, CMD_DDR, FLEXSPI_8PAD, 0x60),
          FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, READ_DDR, FLEXSPI_8PAD, 0x4), 0, 0 },
    };

    do
    {
        uint32_t cmd_pads = option->option0.B.cmd_pads;
        uint32_t query_pads = option->option0.B.query_pads;

        // Do parameter check
        if ((cmd_pads != FLEXSPI_1PAD) && (cmd_pads != FLEXSPI_8PAD))
        {
            status = kStatus_InvalidArgument;
            break;
        }

        if ((query_pads != FLEXSPI_1PAD) && (query_pads != FLEXSPI_8PAD))
        {
            status = kStatus_InvalidArgument;
            break;
        }

        if ((query_pads == FLEXSPI_8PAD) && (cmd_pads == FLEXSPI_1PAD))
        {
            status = kStatus_InvalidArgument;
            break;
        }

        if (query_pads == FLEXSPI_1PAD)
        {
            flash_run_context_t run_ctx;
            status = flexspi_nor_read_persistent(&run_ctx.U);
            flexspi_nor_write_persistent(0);
            if (status == kStatus_Success)
            {
                status = flexspi_nor_restore_spi_protocol(instance, config, &run_ctx);
                if (status != kStatus_Success)
                {
                    break;
                }
            }

            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackInternally;
            config->memConfig.controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);
            xfer.rxSize = 3;
        }
        else if (query_pads == FLEXSPI_8PAD)
        {
            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;
            config->memConfig.controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);

            if (option->option0.B.device_type == kSerialNorCfgOption_DeviceType_MacronixOctalDDR)
            {
                config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable);
            }
            xfer.rxSize = 6;
        }

        if ((cmd_pads == FLEXSPI_1PAD) && (query_pads == FLEXSPI_1PAD))
        {
            is_sdr_mode = true;
        }

        if (cmd_pads == FLEXSPI_8PAD)
        {
            config->memConfig.sflashPadType = kSerialFlash_8Pads;
        }
        else
        {
            config->memConfig.sflashPadType = kSerialFlash_1Pad;
        }

        status = flexspi_nor_flash_init(instance, config);
        if (status != kStatus_Success)
        {
            break;
        }

        uint32_t index = 0;
        if (query_pads == FLEXSPI_1PAD)
        {
            index = 0;
        }
        else if (is_sdr_mode)
        {
            index = 1;
        }
        else
        {
            index = 2;
        }
        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, (const uint32_t *)&k_rdid_lut[index], 1);

        xfer.baseAddress = 0;
        xfer.isParallelModeEnable = false;
        xfer.operation = kFlexSpiOperation_Read;
        xfer.seqId = NOR_CMD_LUT_FOR_IP_CMD;
        xfer.seqNum = 1;
        xfer.rxBuffer = &mfg_id[0];
        status = flexspi_command_xfer(instance, &xfer);
        if (status != kStatus_Success)
        {
            break;
        }

        uint8_t *mfg_id_buffer = (uint8_t *)&mfg_id;
        if (mfg_id_buffer[0] != 0xC2)
        {
            break;
        }

        // Get Flash Size
        if (query_pads == FLEXSPI_1PAD)
        {
            config->memConfig.sflashA1Size = ((64UL * 1024UL) << (mfg_id_buffer[2] - 0x30));
        }
        else
        {
            config->memConfig.sflashA1Size = ((64UL * 1024UL) << (mfg_id_buffer[5] - 0x30));
        }
        config->sectorSize = 4UL * 1024; // 4KB sector size
        config->pageSize = 256U;
        config->blockSize = 64UL * 1024; // 64KB block size

        config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;
        config->memConfig.controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);

        bool enableDTR = !is_sdr_mode;

        // Enable OPI mode
        if (query_pads == FLEXSPI_1PAD)
        {
            if (cmd_pads == FLEXSPI_8PAD)
            {
                config->memConfig.deviceModeCfgEnable = true;
                config->memConfig.deviceModeType = kDeviceConfigCmdType_Spi2Xpi;
                config->memConfig.waitTimeCfgCommands = 1; // Wait 100us
                config->memConfig.deviceModeSeq.seqId = 6;
                config->memConfig.deviceModeSeq.seqNum = 1;

                flash_run_context_t run_ctx;
                run_ctx.B.por_mode = kFlashInstMode_ExtendedSpi;
                run_ctx.B.restore_sequence = kRestoreSequence_Send_6699_9966;

                if (enableDTR)
                {
                    run_ctx.B.current_mode = kFlashInstMode_OPI_DDR;
                    config->memConfig.deviceModeArg = 2;
                    config->memConfig.lookupTable[4 * 6 + 0] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x72, CMD_SDR, FLEXSPI_1PAD, 0x00);
                    config->memConfig.lookupTable[4 * 6 + 1] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, CMD_SDR, FLEXSPI_1PAD, 0x00);
                    config->memConfig.lookupTable[4 * 6 + 2] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, WRITE_SDR, FLEXSPI_1PAD, 0x01);

                    // A flag that marks the configuration block to be saved to FLASH need to be 16bit swapped.
                    // Namely D0 D1 D2 D3 -> D1 D0 D3 D2
                    if (option->option0.B.misc_mode == kSerialNorEnhanceMode_DataOrderSwapped)
                    {
                        config->isDataOrderSwapped = true;
                    }
                    else
                    {
                        config->isDataOrderSwapped = false;
                    }
                }
                else
                {
                    run_ctx.B.current_mode = kFlashInstMode_OPI_SDR;
                    // Enable DQS under SDR mode
                    config->memConfig.deviceModeArg = 0x02;
                    config->memConfig.lookupTable[4 * 6 + 0] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x72, CMD_SDR, FLEXSPI_1PAD, 0x00);
                    config->memConfig.lookupTable[4 * 6 + 1] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, CMD_SDR, FLEXSPI_1PAD, 0x02);
                    config->memConfig.lookupTable[4 * 6 + 2] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, WRITE_SDR, FLEXSPI_1PAD, 0x01);

                    // Enable OPI SDR mode
                    config->memConfig.configCmdEnable = 1;
                    config->memConfig.configCmdSeqs[0].seqId = 7;
                    config->memConfig.configCmdSeqs[0].seqNum = 1;
                    config->memConfig.configCmdArgs[0] = 0x01;
                    config->memConfig.lookupTable[4 * 7 + 0] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x72, CMD_SDR, FLEXSPI_1PAD, 0x00);
                    config->memConfig.lookupTable[4 * 7 + 1] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, CMD_SDR, FLEXSPI_1PAD, 0x00);
                    config->memConfig.lookupTable[4 * 7 + 2] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, WRITE_SDR, FLEXSPI_1PAD, 0x01);
                }

                flexspi_nor_write_persistent(run_ctx.U);
            }
        }

        // Write Enable
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0x00);

        // Read Status
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04);

        if (cmd_pads == FLEXSPI_1PAD)
        {
            // No external DQS for single SDR mode, so use loopback from DQS pad instead
            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackFromDqsPad;
            // Read
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x0C, RADDR_SDR, FLEXSPI_1PAD, 0x20);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] =
                FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_1PAD, 0x08, READ_SDR, FLEXSPI_1PAD, 0x04);

            // Program
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x12, RADDR_SDR, FLEXSPI_1PAD, 0x20);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0x00);

            // Erase Sector
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x21, RADDR_SDR, FLEXSPI_1PAD, 0x20);

            // Erase Block
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xDC, RADDR_SDR, FLEXSPI_1PAD, 0x20);

            // Erase Chip
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP, FLEXSPI_1PAD, 0x00);
        }
        else
        {
            uint32_t cmd_inst = enableDTR ? CMD_DDR : CMD_SDR;
            uint32_t addr_inst = enableDTR ? RADDR_DDR : RADDR_SDR;
            uint32_t dummy_inst = enableDTR ? DUMMY_DDR : DUMMY_SDR;
            uint32_t read_inst = enableDTR ? READ_DDR : READ_SDR;
            uint32_t write_inst = enableDTR ? WRITE_DDR : WRITE_SDR;

            // Read
            uint32_t dummy_cycles = 5;
            if (enableDTR)
            {
                dummy_cycles *= 2;
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
                    FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0xEE, cmd_inst, FLEXSPI_8PAD, 0x11);
            }
            else
            {
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
                    FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0xEC, cmd_inst, FLEXSPI_8PAD, 0x13);
            }
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] =
                FLEXSPI_LUT_SEQ(addr_inst, FLEXSPI_8PAD, 0x20, dummy_inst, FLEXSPI_8PAD, dummy_cycles);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 2] =
                FLEXSPI_LUT_SEQ(read_inst, FLEXSPI_8PAD, 0x04, STOP, FLEXSPI_1PAD, 0x00);

            // Write Enable OPI
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_XPI] =
                FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0x06, cmd_inst, FLEXSPI_8PAD, 0xF9);

            // Read Status OPI
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI + 0] =
                FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0x05, cmd_inst, FLEXSPI_8PAD, 0xFA);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI + 1] =
                FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0x00, cmd_inst, FLEXSPI_8PAD, 0x00);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI + 2] =
                FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0x00, cmd_inst, FLEXSPI_8PAD, 0x00);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI + 3] =
                FLEXSPI_LUT_SEQ(read_inst, FLEXSPI_8PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0);

            // Program
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 0] =
                FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0x12, cmd_inst, FLEXSPI_8PAD, 0xED);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                FLEXSPI_LUT_SEQ(addr_inst, FLEXSPI_8PAD, 0x20, write_inst, FLEXSPI_8PAD, 0x04);

            // Erase Sector
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 0] =
                FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0x21, cmd_inst, FLEXSPI_8PAD, 0xDE);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 1] =
                FLEXSPI_LUT_SEQ(addr_inst, FLEXSPI_8PAD, 0x20, STOP, FLEXSPI_1PAD, 0x00);

            // Erase Block
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK + 0] =
                FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0xDC, cmd_inst, FLEXSPI_8PAD, 0x23);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK + 1] =
                FLEXSPI_LUT_SEQ(addr_inst, FLEXSPI_8PAD, 0x20, STOP, FLEXSPI_1PAD, 0x00);

            // Erase Chip
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE] =
                FLEXSPI_LUT_SEQ(cmd_inst, FLEXSPI_8PAD, 0x60, cmd_inst, FLEXSPI_8PAD, 0x9F);

            if (enableDTR)
            {
                config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable);
            }
            config->serialNorType = kSerialNorType_XPI;
        }

    } while (0);

    return status;
}

status_t flexspi_nor_generate_config_block_micron_octalflash(uint32_t instance,
                                                             flexspi_nor_config_t *config,
                                                             serial_nor_config_option_t *option)
{
    status_t status = kStatus_FLEXSPINOR_Flash_NotFound;

    const lut_seq_t k_sdfp_lut[2] = {
        // Read SFDP LUT sequence for 1 pad instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialFlash_ReadSFDP, RADDR_SDR, FLEXSPI_1PAD, 24),
          FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_1PAD, 8, READ_SDR, FLEXSPI_1PAD, 0xFF), 0, 0 },

        // Read SFDP LUT sequence for OPI DDR instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, kSerialFlash_ReadSFDP, RADDR_DDR, FLEXSPI_8PAD, 32),
          //  FLEXSPI_LUT_SEQ(DUMMY_DDR, FLEXSPI_8PAD, 8, READ_DDR, FLEXSPI_8PAD, 0xFF), 0, 0
          FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0xFF, STOP, FLEXSPI_1PAD, 0), 0, 0 },
    };

    bool is_sdr_mode = option->option0.B.device_type == kSerialNorCfgOption_DeviceType_MicronOctalSDR;

    do
    {
        if (option->option0.B.query_pads == FLEXSPI_1PAD)
        {
            flash_run_context_t run_ctx;
            status = flexspi_nor_read_persistent(&run_ctx.U);
            flexspi_nor_write_persistent(0);
            if (status == kStatus_Success)
            {
                status = flexspi_nor_restore_spi_protocol(instance, config, &run_ctx);
                if (status != kStatus_Success)
                {
                    break;
                }
            }

            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackInternally;
            config->memConfig.controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);
        }
        else
        {
            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;
            config->memConfig.controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable) |
                                                     FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);
        }
        config->memConfig.sflashPadType = kSerialFlash_8Pads;

        status = flexspi_nor_flash_init(instance, config);
        if (status != kStatus_Success)
        {
            break;
        }

        if (option->option0.B.query_pads == FLEXSPI_1PAD)
        {
            flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, (const uint32_t *)&k_sdfp_lut[0], 1);
        }
        else if (option->option0.B.query_pads == FLEXSPI_8PAD)
        {
            flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, (const uint32_t *)&k_sdfp_lut[1], 1);
        }
        else
        {
            break;
        }

        config->memConfig.sflashPadType = kSerialFlash_8Pads;

        jedec_info_table_t jedec_info_tbl;
        status = flexspi_nor_read_sfdp_info(instance, &jedec_info_tbl, false);
        if (status != kStatus_Success)
        {
            break;
        }

        // Get Flash Size, Sector size, page size from SFDP
        uint32_t sector_erase_cmd;
        uint32_t block_erase_cmd;
        get_page_sector_block_size_from_sfdp(config, &jedec_info_tbl, &sector_erase_cmd, &block_erase_cmd);

        if (!is_sdr_mode)
        {
            // Update sample clock source and misc option
            config->memConfig.controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable) |
                                                     FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);
            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;
        }
        // Write Enable
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0);

        // Read Status
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04);

        bool opi_mode_enable = false;
        if ((option->option0.B.query_pads == FLEXSPI_8PAD) || (option->option0.B.cmd_pads == FLEXSPI_8PAD))
        {
            opi_mode_enable = true;
        }

        if (!opi_mode_enable)
        {
            if (option->option0.B.cmd_pads == FLEXSPI_1PAD)
            {
                uint32_t address_bits;
                uint32_t page_program_cmd;
                if (config->memConfig.sflashA1Size >= MAX_24BIT_ADDRESSING_SIZE)
                {
                    address_bits = 32;
                    page_program_cmd = 0x8E;
                }
                else
                {
                    address_bits = 24;
                    page_program_cmd = 0xC2;
                }

                // Erase Sector
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 0] =
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, sector_erase_cmd, RADDR_SDR, FLEXSPI_1PAD, address_bits);

                // Erase Block
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK + 0] =
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, block_erase_cmd, RADDR_SDR, FLEXSPI_1PAD, address_bits);

                // Program (Program 1-8-8)
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 0] =
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, page_program_cmd, RADDR_SDR, FLEXSPI_8PAD, address_bits);
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                    FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_8PAD, 0x80, STOP, FLEXSPI_1PAD, 0);

                if (!is_sdr_mode)
                {
                    // Read
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xFD, RADDR_DDR, FLEXSPI_8PAD, 0x20);
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] =
                        FLEXSPI_LUT_SEQ(DUMMY_DDR, FLEXSPI_8PAD, 0x0c, READ_DDR, FLEXSPI_8PAD, 0x04);

                    config->halfClkForNonReadCmd = true;
                }
                else
                {
                    uint32_t read_cmd = 0xCB;
                    if (config->memConfig.sflashA1Size >= MAX_24BIT_ADDRESSING_SIZE)
                    {
                        read_cmd = 0xCC;
                    }
                    // Write Volatile register, set dummy to 12, then the Octal SDR read can reach 133MHz
                    config->memConfig.lookupTable[4 * 6 + 0] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x81, CMD_SDR, FLEXSPI_1PAD, 0x00);
                    config->memConfig.lookupTable[4 * 6 + 1] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, CMD_SDR, FLEXSPI_1PAD, 0x01);
                    config->memConfig.lookupTable[4 * 6 + 2] =
                        FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x1, STOP, FLEXSPI_1PAD, 0);

                    config->memConfig.deviceModeArg = 12;
                    config->memConfig.deviceModeCfgEnable = true;
                    config->memConfig.deviceModeSeq.seqId = 6;
                    config->memConfig.deviceModeSeq.seqNum = 1;
                    // Read
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
                        FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, read_cmd, RADDR_SDR, FLEXSPI_8PAD, address_bits);
                    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] =
                        FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_8PAD, 0x0C, READ_SDR, FLEXSPI_8PAD, 0x04);
                }

                // Erase Chip
                config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 0] =
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP, FLEXSPI_1PAD, 0);
            }
            else
            {
                break;
            }
        }
        else
        {
            // Read status register using Octal DDR read
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, 0x05, READ_DDR, FLEXSPI_8PAD, 0x04);

            // OPI DDR read
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, 0xFD, RADDR_DDR, FLEXSPI_8PAD, 0x20);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] =
                FLEXSPI_LUT_SEQ(DUMMY_DDR, FLEXSPI_8PAD, 0x06, READ_DDR, FLEXSPI_8PAD, 0x04);

            // Write Enable
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_XPI + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, 0x06, STOP, FLEXSPI_1PAD, 0);

            // Erase Sector
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, 0x21, RADDR_DDR, FLEXSPI_8PAD, 0x20);

            // Erase Block
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, 0xDC, RADDR_DDR, FLEXSPI_8PAD, 0x20);

            // Erase Chip
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, 0x60, STOP, FLEXSPI_1PAD, 0);

            // Program
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, 0x12, RADDR_DDR, FLEXSPI_8PAD, 0x20);
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
                FLEXSPI_LUT_SEQ(WRITE_DDR, FLEXSPI_8PAD, 0x04, STOP, FLEXSPI_1PAD, 0);

            if (option->option0.B.query_pads == FLEXSPI_1PAD)
            {
                flash_run_context_t run_ctx;
                run_ctx.B.por_mode = kFlashInstMode_ExtendedSpi;
                run_ctx.B.restore_sequence = kRestoreSequence_Send_66_99;
                run_ctx.B.current_mode = kFlashInstMode_OPI_DDR;
                flexspi_nor_write_persistent(run_ctx.U);

                // Enable OPI mode
                config->memConfig.deviceModeCfgEnable = true;
                config->memConfig.deviceModeType = kDeviceConfigCmdType_Spi2Xpi;
                config->memConfig.deviceModeArg = 0xE7; // Octal DDR mode
                config->memConfig.deviceModeSeq.seqId = 6;
                config->memConfig.deviceModeSeq.seqNum = 1;
                config->memConfig.waitTimeCfgCommands = 1;

                // Write Volatile register
                config->memConfig.lookupTable[4 * 6 + 0] =
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x81, CMD_SDR, FLEXSPI_1PAD, 0x00);
                config->memConfig.lookupTable[4 * 6 + 1] =
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, CMD_SDR, FLEXSPI_1PAD, 0x00);
                config->memConfig.lookupTable[4 * 6 + 2] =
                    FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x1, STOP, FLEXSPI_1PAD, 0);
            }
            config->serialNorType = kSerialNorType_XPI;
        }
    } while (0);

    return status;
}

status_t flexspi_nor_generate_config_block_adesto_octalflash(uint32_t instance,
                                                             flexspi_nor_config_t *config,
                                                             serial_nor_config_option_t *option)
{
    enum
    {
        kSfdp_LutIndex_Sdr_1_1,
        kSfdp_LutIndex_Sdr_4_4,
        kSfdp_LutIndex_Sdr_8_8,
        kSfdp_LutIndex_Ddr_4_4,
        kSfdp_LutIndex_Ddr_8_8
    };

    status_t status = kStatus_FLEXSPINOR_Flash_NotFound;

    bool is_sdr_mode = option->option0.B.device_type == kSerialNorCfgOption_DeviceType_AdestoOctalSDR;

    const lut_seq_t k_sdfp_lut[5] = {
        // Read SFDP LUT sequence for 1 pad instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, kSerialFlash_ReadSFDP, RADDR_SDR, FLEXSPI_1PAD, 24),
          FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_1PAD, 8, READ_SDR, FLEXSPI_1PAD, 0xFF), 0, 0 },

        // Read SFDP LUT sequence for QPI SDR instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, kSerialFlash_ReadSFDP, RADDR_SDR, FLEXSPI_4PAD, 24),
          FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_4PAD, 0xFF, STOP, FLEXSPI_1PAD, 0), 0, 0 },

        // Read SFDP LUT sequence for OPI SDR instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, kSerialFlash_ReadSFDP, RADDR_SDR, FLEXSPI_8PAD, 24),
          FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_8PAD, 0xFF, STOP, FLEXSPI_1PAD, 0), 0, 0 },

        // Read SFDP LUT sequence for QPI DDR instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_4PAD, kSerialFlash_ReadSFDP, RADDR_DDR, FLEXSPI_4PAD, 24),
          FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_4PAD, 0xFF, STOP, FLEXSPI_1PAD, 0), 0, 0 },

        // Read SFDP LUT sequence for OPI DDR instruction
        { FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_8PAD, kSerialFlash_ReadSFDP, RADDR_DDR, FLEXSPI_8PAD, 32),
          FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0xFF, STOP, FLEXSPI_1PAD, 0), 0, 0 },
    };

    do
    {
        bool address_shift_enable = false;
        config->memConfig.sflashPadType = kSerialFlash_4Pads;

        uint32_t query_pads = option->option0.B.query_pads;
        uint32_t cmd_pads = option->option0.B.cmd_pads;

        config->memConfig.controllerMiscOption = FLEXSPI_BITMASK(kFlexSpiMiscOffset_SafeConfigFreqEnable);
        if (query_pads == FLEXSPI_1PAD)
        {
            flash_run_context_t run_ctx;
            status = flexspi_nor_read_persistent(&run_ctx.U);
            // Clear persistent status
            flexspi_nor_write_persistent(kFlashInstMode_ExtendedSpi);
            if (status == kStatus_Success)
            {
                status = flexspi_nor_restore_spi_protocol(instance, config, &run_ctx);
                if (status != kStatus_Success)
                {
                    break;
                }
            }
            config->memConfig.sflashPadType = kSerialFlash_1Pad;
            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackInternally;
        }
        else if (query_pads == FLEXSPI_4PAD)
        {
            config->memConfig.sflashPadType = kSerialFlash_4Pads;
            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;
        }
        else if (query_pads == FLEXSPI_8PAD)
        {
            config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;
            config->memConfig.sflashPadType = kSerialFlash_8Pads;
            if (!is_sdr_mode)
            {
                address_shift_enable = true;
            }
        }
        else
        {
            break;
        }

        if ((cmd_pads != FLEXSPI_1PAD) && (cmd_pads != FLEXSPI_4PAD) && (cmd_pads != FLEXSPI_8PAD))
        {
            break;
        }

        if ((query_pads == FLEXSPI_1PAD) && (cmd_pads == FLEXSPI_1PAD))
        {
            is_sdr_mode = true;
        }

        if (cmd_pads == FLEXSPI_4PAD)
        {
            config->memConfig.sflashPadType = kSerialFlash_4Pads;
        }
        if (cmd_pads == FLEXSPI_8PAD)
        {
            config->memConfig.sflashPadType = kSerialFlash_8Pads;
        }

        if (is_sdr_mode)
        {
            config->memConfig.controllerMiscOption &= (uint32_t)~FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable);
        }
        else
        {
            config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable);
        }

        status = flexspi_nor_flash_init(instance, config);
        if (status != kStatus_Success)
        {
            break;
        }

        uint32_t lutIndex = kSfdp_LutIndex_Sdr_1_1;
        if (is_sdr_mode)
        {
            switch (query_pads)
            {
                case FLEXSPI_4PAD:
                    lutIndex = kSfdp_LutIndex_Sdr_4_4;
                    break;
                case FLEXSPI_8PAD:
                    lutIndex = kSfdp_LutIndex_Sdr_8_8;
                    break;
                default:
                    lutIndex = kSfdp_LutIndex_Sdr_1_1;
                    break;
            }
        }
        else
        {
            switch (query_pads)
            {
                case FLEXSPI_4PAD:
                    lutIndex = kSfdp_LutIndex_Ddr_4_4;
                    break;
                case FLEXSPI_8PAD:
                    lutIndex = kSfdp_LutIndex_Ddr_8_8;
                    break;
                default:
                    lutIndex = kSfdp_LutIndex_Sdr_1_1;
                    break;
            }
        }

        flexspi_update_lut(instance, NOR_CMD_LUT_FOR_IP_CMD, (const uint32_t *)&k_sdfp_lut[lutIndex], 1);

        jedec_info_table_t jedec_info_tbl;
        status = flexspi_nor_read_sfdp_info(instance, &jedec_info_tbl, address_shift_enable);
        if (status != kStatus_Success)
        {
            break;
        }

        // Get Flash Size, Sector size from SFDP
        uint32_t sector_erase_cmd;
        uint32_t block_erase_cmd;
        get_page_sector_block_size_from_sfdp(config, &jedec_info_tbl, &sector_erase_cmd, &block_erase_cmd);

        // Update Read Sampling clock
        config->memConfig.readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;

        // Write Enable
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0);
        // Read status
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04);

        uint32_t addr_pads = cmd_pads;
        uint32_t dummy_pads = cmd_pads;
        uint32_t write_pads = cmd_pads;
        uint32_t read_pads = cmd_pads;
        uint32_t addr_inst = is_sdr_mode ? RADDR_SDR : RADDR_DDR;
        uint32_t dummy_inst = is_sdr_mode ? DUMMY_SDR : DUMMY_DDR;
        uint32_t read_inst = is_sdr_mode ? READ_SDR : READ_DDR;
        uint32_t write_inst = is_sdr_mode ? WRITE_SDR : WRITE_DDR;
        uint32_t dummy_cycles = (is_sdr_mode) ? 6 : 12;

        // Read command
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, cmd_pads, 0x0B, addr_inst, addr_pads, 0x20);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] =
            FLEXSPI_LUT_SEQ(dummy_inst, dummy_pads, dummy_cycles, read_inst, read_pads, 0x04);

        // Erase Sector
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, cmd_pads, 0x20, addr_inst, addr_pads, 0x20);

        // Erase Block
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, cmd_pads, 0xd8, addr_inst, addr_pads, 0x20);

        // Page Program
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, cmd_pads, 0x02, addr_inst, addr_pads, 0x20);
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] =
            FLEXSPI_LUT_SEQ(write_inst, write_pads, 0x80, STOP, FLEXSPI_1PAD, 0);

        // Erase Chip
        config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 0] =
            FLEXSPI_LUT_SEQ(CMD_SDR, cmd_pads, 0x60, STOP, FLEXSPI_1PAD, 0);

        if ((cmd_pads == FLEXSPI_4PAD) || (cmd_pads == FLEXSPI_8PAD))
        {
            // Read status XPI
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, cmd_pads, 0x05, read_inst, read_pads, 0x04);

            // Write Enable XPI
            config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_XPI + 0] =
                FLEXSPI_LUT_SEQ(CMD_SDR, cmd_pads, 0x06, STOP, FLEXSPI_1PAD, 0);

            config->serialNorType = kSerialNorType_XPI;
        }

        // Global Unprotect
        config->memConfig.deviceModeArg = 0x000000;

        if (cmd_pads == FLEXSPI_4PAD)
        {
            // Enable QPI mode
            config->memConfig.deviceModeArg |= 0x040000;
        }
        else if (cmd_pads == FLEXSPI_8PAD)
        {
            // Enable OPI mode
            config->memConfig.deviceModeArg |= 0x080000;
        }

        if (!is_sdr_mode)
        {
            // Enable DDR/DTR mode
            config->memConfig.deviceModeArg |= 0x800000;
        }
        else
        {
            //  Use delay cell settings for OPI SDR mode using external DQS as sampling source.
            config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_UseValidTimeForAllFreq);
            config->memConfig.dataValidTime[0].delay_cells = 1u;
        }

        config->memConfig.lookupTable[4 * 6] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x71, CMD_SDR, FLEXSPI_1PAD, 0x0);
        config->memConfig.lookupTable[4 * 6 + 1] =
            FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x03, STOP, FLEXSPI_1PAD, 0);
        config->memConfig.deviceModeCfgEnable = true;
        if (cmd_pads != FLEXSPI_1PAD)
        {
            config->memConfig.deviceModeType = kDeviceConfigCmdType_Spi2Xpi;
        }
        else
        {
            config->memConfig.deviceModeType = kDeviceConfigCmdType_Generic;
        }
        config->memConfig.deviceModeSeq.seqId = 6;
        config->memConfig.deviceModeSeq.seqNum = 1;
        config->memConfig.waitTimeCfgCommands = 1; // Wait 100us

        flash_run_context_t run_ctx;
        run_ctx.B.por_mode = kFlashInstMode_ExtendedSpi;
        if (is_sdr_mode)
        {
            if (cmd_pads == FLEXSPI_4PAD)
            {
                run_ctx.B.current_mode = kFlashInstMode_QPI_SDR;
            }
            else if (cmd_pads == FLEXSPI_8PAD)
            {
                run_ctx.B.current_mode = kFlashInstMode_OPI_SDR;
            }
            else
            {
                run_ctx.B.current_mode = kFlashInstMode_ExtendedSpi;
            }
        }
        else
        {
            if (cmd_pads == FLEXSPI_4PAD)
            {
                run_ctx.B.current_mode = kFlashInstMode_QPI_DDR;
            }
            else if (cmd_pads == FLEXSPI_8PAD)
            {
                run_ctx.B.current_mode = kFlashInstMode_OPI_DDR;
            }
            else
            {
                run_ctx.B.current_mode = kFlashInstMode_ExtendedSpi;
            }
        }
        run_ctx.B.restore_sequence = kRestoreSequence_Send_06_FF;
        flexspi_nor_write_persistent(run_ctx.U);

        status = kStatus_Success;

    } while (0);

    return status;
}

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_GET_CFG)
// See flexspi_nor_flash.h for more details.
status_t flexspi_nor_get_config(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((config == NULL) || (option == NULL))
        {
            break;
        }

        // Configure the Configuration block to default value
        memset(config, 0, sizeof(flexspi_nor_config_t));
        config->memConfig.serialClkFreq = kFlexSpiSerialClk_SafeFreq;
        config->memConfig.sflashA1Size = MAX_24BIT_ADDRESSING_SIZE;
        config->memConfig.tag = FLEXSPI_CFG_BLK_TAG;
        config->memConfig.version = FLEXSPI_CFG_BLK_VERSION;
        config->memConfig.csHoldTime = 3;
        config->memConfig.csSetupTime = 3;
        config->ipcmdSerialClkFreq = kFlexSpiSerialClk_SafeFreq;

        if (option->option0.B.option_size > 0)
        {
            // Switch to second pinmux group
            if (option->option1.B.pinmux_group == 1)
            {
                config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondPinMux);
            }

            // Change the Pad Drive Strength
            if (option->option1.B.drive_strength)
            {
                flexspi_update_padsetting(&config->memConfig, option->option1.B.drive_strength);
            }
            // Enable parallel mode support
            if (option->option1.B.flash_connection)
            {
                if ((option->option0.B.device_type == kSerialNorCfgOption_DeviceType_ReadSFDP_SDR) ||
                    (option->option0.B.device_type == kSerialNorCfgOption_DeviceType_ReadSFDP_DDR))
                {
                    uint32_t flashConnection = option->option1.B.flash_connection;

                    switch (flashConnection)
                    {
                        default:
                        case kSerialNorConnection_SinglePortA:
                            // This is default setting, do nothing here
                            break;
                        case kSerialNorConnection_Parallel:
                            config->memConfig.controllerMiscOption |=
                                FLEXSPI_BITMASK(kFlexSpiMiscOffset_ParallelEnable);
                            break;
                        case kSerialNorConnection_SinglePortB:
                            config->memConfig.sflashA1Size = 0;
                            config->memConfig.sflashB1Size = MAX_24BIT_ADDRESSING_SIZE;
                            break;
                    }
                }
                else
                {
                    option->option1.B.flash_connection = 0;
                }
            }
        }

        switch (option->option0.B.device_type)
        {
            case kSerialNorCfgOption_DeviceType_ReadSFDP_SDR:
            case kSerialNorCfgOption_DeviceType_ReadSFDP_DDR:
                status = flexspi_nor_generate_config_block_using_sfdp(instance, config, option);
                break;
#if FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
            case kSerialNorCfgOption_DeviceType_HyperFLASH1V8:
                status = flexspi_nor_generate_config_block_hyperflash(instance, config, true);
                break;
            case kSerialNorCfgOption_DeviceType_HyperFLASH3V0:
                status = flexspi_nor_generate_config_block_hyperflash(instance, config, false);
                break;
            case kSerialNorCfgOption_DeviceType_MacronixOctalDDR:
            case kSerialNorCfgOption_DeviceType_MacronixOctalSDR:
                status = flexspi_nor_generate_config_block_mxic_octalflash(instance, config, option);
                break;
            case kSerialNorCfgOption_DeviceType_MicronOctalDDR:
            case kSerialNorCfgOption_DeviceType_MicronOctalSDR:
                status = flexspi_nor_generate_config_block_micron_octalflash(instance, config, option);
                break;
            case kSerialNorCfgOption_DeviceType_AdestoOctalDDR:
            case kSerialNorCfgOption_DeviceType_AdestoOctalSDR:
                status = flexspi_nor_generate_config_block_adesto_octalflash(instance, config, option);
                break;
#endif // FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
            default:
                status = kStatus_InvalidArgument;
                break;
        }

        if (status == kStatus_Success)
        {
            flexspi_set_failsafe_setting(&config->memConfig);
            config->memConfig.serialClkFreq = option->option0.B.max_freq;

            if (option->option0.B.option_size && (option->option1.B.flash_connection == kSerialNorConnection_BothPorts))
            {
                config->memConfig.sflashB1Size = config->memConfig.sflashA1Size;
            }
        }
        config->memConfig.deviceType = kFlexSpiDeviceType_SerialNOR;

    } while (0);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_GET_CFG)

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_ERASE)
// See flexspi_nor_flash.h for more details.
status_t flexspi_nor_flash_erase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length)
{
    uint32_t aligned_start;
    uint32_t aligned_end;

    status_t status = kStatus_InvalidArgument;

    do
    {
        if (config == NULL)
        {
            break;
        }

        aligned_start = ALIGN_DOWN(start, config->sectorSize);
        aligned_end = ALIGN_UP(start + length, config->sectorSize);

        // If the block size and sector size is uniform, just do sector erase
        if (config->isUniformBlockSize || (config->blockSize == 0))
        {
            while (aligned_start < aligned_end)
            {
                status = flexspi_nor_flash_erase_sector(instance, config, aligned_start);
                if (status != kStatus_Success)
                {
                    return status;
                }
                aligned_start += config->sectorSize;
            }
        }
        else // Try do do erase using maximum granularity in order to improve erase performance
        {
            while (aligned_start < aligned_end)
            {
                bool is_addr_block_aligned = !(aligned_start & ~(config->blockSize));
                uint32_t remaining_size = (aligned_end - aligned_start);
                if (is_addr_block_aligned && (remaining_size >= config->blockSize))
                {
                    status = flexspi_nor_flash_erase_block(instance, config, aligned_start);
                    if (status != kStatus_Success)
                    {
                        return status;
                    }
                    aligned_start += config->blockSize;
                }
                else
                {
                    status = flexspi_nor_flash_erase_sector(instance, config, aligned_start);
                    if (status != kStatus_Success)
                    {
                        return status;
                    }
                    aligned_start += config->sectorSize;
                }
            }
        }
    } while (0);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_ERASE)

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_READ)
// See flexspi_nor_flash.h for more details.
status_t flexspi_nor_flash_read(
    uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((config == NULL) || (dst == NULL) || (bytes < 1))
        {
            break;
        }

        flexspi_xfer_t flashXfer;
        bool isParallelMode;

        flexspi_mem_config_t *memCfg = (flexspi_mem_config_t *)config;
        isParallelMode = flexspi_is_parallel_mode(memCfg);
        flashXfer.operation = kFlexSpiOperation_Read;
        flashXfer.seqNum = 1;
        flashXfer.seqId = NOR_CMD_LUT_SEQ_IDX_READ;
        flashXfer.isParallelModeEnable = isParallelMode;

        while (bytes)
        {
            uint32_t readLength = bytes > 65535 ? 65535 : bytes;

            flashXfer.baseAddress = start;
            flashXfer.rxBuffer = dst;
            flashXfer.rxSize = readLength;
            status = flexspi_command_xfer(instance, &flashXfer);
            if (status != kStatus_Success)
            {
                break;
            }
            bytes -= readLength;
            start += readLength;
            dst += readLength / sizeof(uint32_t);
        }
    } while (0);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_NOR_READ)

status_t flexspi_nor_restore_spi_protocol(uint32_t instance, flexspi_nor_config_t *config, flash_run_context_t *run_ctx)
{
    status_t status = kStatus_InvalidArgument;
    flexspi_xfer_t xfer;

    do
    {
        if (run_ctx == NULL)
        {
            break;
        }

        if (run_ctx->B.current_mode == run_ctx->B.por_mode)
        {
            status = kStatus_Success;
            break;
        }

        uint32_t pad = FLEXSPI_8PAD;
        uint32_t cmd_inst = CMD_SDR;

        config->memConfig.sflashPadType = kSerialFlash_8Pads;
        if ((run_ctx->B.current_mode == kFlashInstMode_QPI_DDR) || (run_ctx->B.current_mode == kFlashInstMode_QPI_SDR))
        {
            pad = FLEXSPI_4PAD;
            config->memConfig.sflashPadType = kSerialFlash_4Pads;
        }

        if ((run_ctx->B.current_mode == kFlashInstMode_OPI_DDR) || (run_ctx->B.current_mode == kFlashInstMode_QPI_DDR))
        {
            cmd_inst = CMD_DDR;
        }

        xfer.baseAddress = 0;
        xfer.isParallelModeEnable = false;
        xfer.operation = kFlexSpiOperation_Command;
        xfer.seqId = 1;

        uint32_t lut_seq[8];
        memset(&lut_seq, 0, sizeof(lut_seq));

        switch (run_ctx->B.restore_sequence)
        {
            case kRestoreSequence_Send_06_FF:
                lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_SDR, pad, 0x06, STOP, FLEXSPI_1PAD, 0);
                lut_seq[4] = FLEXSPI_LUT_SEQ(CMD_SDR, pad, 0xFF, STOP, FLEXSPI_1PAD, 0);
                xfer.seqNum = 2;
                break;
            case kRestoreSequence_Send_66_99:
                lut_seq[0] = FLEXSPI_LUT_SEQ(CMD_SDR, pad, 0x66, STOP, FLEXSPI_1PAD, 0);
                lut_seq[4] = FLEXSPI_LUT_SEQ(CMD_SDR, pad, 0x99, STOP, FLEXSPI_1PAD, 0);
                xfer.seqNum = 2;
                break;
            case kRestoreSequence_Send_6699_9966:
                lut_seq[0] = FLEXSPI_LUT_SEQ(cmd_inst, pad, 0x66, cmd_inst, pad, 0x99);
                lut_seq[4] = FLEXSPI_LUT_SEQ(cmd_inst, pad, 0x99, cmd_inst, pad, 0x66);
                if (cmd_inst == CMD_DDR)
                {
                    config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable);
                }
                xfer.seqNum = 2;
                break;
            default:
                xfer.seqNum = 1;
                break;
        }

        config->memConfig.serialClkFreq = kFlexSpiSerialClk_SafeFreq;
        config->memConfig.sflashA1Size = MAX_24BIT_ADDRESSING_SIZE;
        config->memConfig.tag = FLEXSPI_CFG_BLK_TAG;
        config->memConfig.version = FLEXSPI_CFG_BLK_VERSION;
        config->memConfig.csHoldTime = 3;
        config->memConfig.csSetupTime = 3;
        config->ipcmdSerialClkFreq = kFlexSpiSerialClk_SafeFreq;
        config->memConfig.commandInterval = 10;

        status = flexspi_nor_flash_init(instance, config);
        if (status != kStatus_Success)
        {
            break;
        }

        flexspi_update_lut(instance, 1, &lut_seq[0], xfer.seqNum);
        status = flexspi_command_xfer(instance, &xfer);

        // Delay several ms until device is restored to SPI protocol
        for (volatile uint32_t wait_cnt = SystemCoreClock / 1000; wait_cnt; wait_cnt--)
        {
            __NOP();
        }

    } while (0);

    return status;
}
