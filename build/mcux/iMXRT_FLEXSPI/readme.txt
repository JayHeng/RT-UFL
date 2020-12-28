MIMXRT1050-EVK_IS25WP064A LinkServer/CMSIS-DAP Flash Driver
===========================================================

This MCUXpresso IDE LinkServer/CMSIS-DAP QSPI flash driver example 
is based on the 'evkbimxrt1050_flexspi_nor_polling_transfer' example
from the SDK v2.4.0 EVKB-MIMXRT1050 package.

June 2018:

Initial implementation creates a single driver
'MIMXRT1050-EVK_IS25WP064A.cfx' compatible with the IS25WP064A QSPI
device.

The initialisation function -  QSPI_init() uses SDK code virtually
verbatim but excludes MPU and Debug Console setup.

To improve performance the following LUT entry is modified to force the
use of the 64KB block erase 0xD8 (rather than the 0xD7 4KB variant)

        /* Erase Sector  */
        [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xD8,
        kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

The driver also programs memory using 16KB pseudo pages rather than true
256 bytes pages. This allows better usage of USB bandwidth for 
transferring date, with a noticeable benefit for programming speeds.


Requirements for Building
=========================

The use of MCUXpresso IDE v10.2.0 or later is required for building
this project.

The project is created for a Generic (Cortex) M7 MCU and so generated
code will be compatible with the iMXRT MCUs. This also removes any
requirement for a specific SDK to be installed for use.

The LPCXFlashDriverLib project must also be imported into the flash
driver project's workspace. For correct linkage ensure that both debug
and release variants of this project must be manually pre-built.

This driver project uses example and board support code from the SDK
EVKB-IMXRT1050. If an iMXRT10xx flash driver is required for a different
board, then appropriate board files will need to be updated. 

Note that by default, this QSPI Flash is disabled on the MIMXRT1050 EVKB.
To enable the onboard QSPI Flash, the settings need to be changed as 
follows:

Step1:
Remove resistors: R356, R361 - R366. 

Step2:
Solder 0 ohm resistors: R153 - R158.

Step3:
Change the settings on SW7 to boot from QSPI

For more information see the IMXRT1050 EVKB Board Hardware User’s Guide.

Build Configurations
==================== 
 
There are two build configurations within the project:

'Debug'
-------
 This build configuration can be used to generate a test
version of the flash driver that can be run within the MCUXPresso IDE
debug environment. This configuration builds the flash driver wrapped by
an interface that mimics the interface used by MCUXpresso IDE when
programming a project executable/binary into flash.

This configuration can be very useful when adding support for a new
flash device, to ensure that basic operation works, before trying out
the full flash driver.

'MIMXRT1050-EVK_IS25WP064A'
---------------------------
This creates the actual driver 'MIMXRT1050-EVK_IS25WP064A.cfx' in a 
project directory called 'builds'


Linker Scripts
==============

This project forgoes the use of automatic managed linkerscript
generation due its special requirements for image layout.

Instead a directory containing a linker script per build configuration
is supplied. These linker scripts builds link both configurations to use
on-chip SRAM RAM at 0x20000000 and assume 64KB is available.


Launch Configurations
=====================

As supplied a single launch configuration is included for the debug
build configuration. This file contains a single non default setting
within the 'additional options' entry of '--nopacked'. This setting is
required to ensure the debug interface on this MCU is accessed
appropriately, without this, printf characters may not display correctly
within the console output. Should this launch configuration be deleted,
a new one will be re-generated automatically on the next debug
operation. However, the '--nopacked' must be manually added.


Driver Performance
==================

In testing this driver can achieve programming speeds of around (to a
non erased device):

..... 90 KB/s with LPC-Link2 Bridged Firmware 
..... 23 KB/s with OpenSDA DAPlink on board debug probe


Other Supplied Flash Drivers
============================

This example flash driver targets a single QSPI device only.

Most modern QSPI parts can be automatically identified and configured
via a mechanism known as the Serial Flash Discovery Protocol (SFDP).

Supplied with MCUXpresso IDE version 10.2.1 are drivers that uses this
mechanism to enable operation with many QSPI devices that also support
this protocol. This should in most cases mean that a QSPI device 
specific flash driver, such as this one, is not required.

These drivers include the following:

MIMXRT1020_SFDP_QSPI.cfx
MIMXRT1050_SFDP_QSPI.cfx
MIMXRT1050_SFDP_HYPERFLASH.cfx
LPC18_43_SPIFI_SFDP.cfx
LPC546xx_SPIFI_SFDP.cfx
LPC540xx_SPIFI_SFDP.cfx


Directory Structure
===================

The directory structure for this project is similar to a standard SDK
project with the following exceptions.

'QSPIsource' contains configuration and init files for the QSPI device
'builds' will contain the result of the 'MIMXRT1050-EVK_IS25WP064A.cfx'
'linkscripts' contains a linker script for each build configuration
'test' contains the code required for the debug 'test' build.

