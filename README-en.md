# RT-UFL (Universal FlashLoader)

[![GitHub release](https://img.shields.io/github/release/JayHeng/RT-UFL.svg)](https://github.com/JayHeng/RT-UFL/releases/latest) [![GitHub commits](https://img.shields.io/github/commits-since/JayHeng/RT-UFL/v0.5.svg)](https://github.com/JayHeng/RT-UFL/compare/v0.5...master) ![GitHub All Releases](https://img.shields.io/github/downloads/JayHeng/RT-UFL/total.svg) [![GitHub license](https://img.shields.io/github/license/JayHeng/RT-UFL.svg)](https://github.com/JayHeng/RT-UFL/blob/master/LICENSE)

English | [中文](./README.md)

　　RT-UFL is a universal flashloader, one .FLM file for all i.MXRT chips, it should be integrated into Segger driver (or Keil MDK), then you can download and debug MCUXpresso SDK XIP project by J-Link.

　　The main features of RT-UniversalFlashloader：

> * Can run on all i.MXRT chips.
> * Can support any serial NOR Flash that can be used as i.MXRT boot device .
> * Can erase and program Flash connected to ROM specified pinmux.
> * Can auto-detect serial NOR Flash type (QuadSPI, Octal-SPI, Hyperbus).
> * Can auto-detect SFDP version of Flash.
> * Can support Flash without SFDP.
> * Can auto-detect default QE bit status of QuadSPI Flash.
> * Can output necessary information of Flash for boot header.

　　The block diagram of RT-UniversalFlashloader：

![](doc/RT-UniversalFlashloader_Arch.PNG)

　　RT-UniversalFlashloader contains three layers:

> * [API Layer]: API is defined by Tool (Segger, Keil MDK).
> * [Adapter Layer]: The key layer to support different i.MXRT chips and NOR Flash chips.
> * [Driver Layer]: The low-level FlexSPI NOR driver.

　　To develop RT-UniversalFlashloader, we should be aware of below notes:

> * Always use Cortex-M0 instruction set to build final .FLM file.
> * There is no device ID in i.MXRT, we can take use of ROM content to find part number.
> * Most i.MXRT chips contain ROM API, we can use ROM API as low-level NOR driver.
> * Flash auto-probe feature is added in i.MXRT1060 BootROM, we can refer to this feature to develop key flash_auto_probe() function.
> * soft reset command should be issued to reset Flash after every failed attempt in flash_auto_probe()

