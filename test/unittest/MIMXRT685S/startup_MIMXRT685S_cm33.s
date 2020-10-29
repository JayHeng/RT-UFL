;/*****************************************************************************
; * @file:    startup_MIMXRT685S_cm33.s
; * @purpose: CMSIS Cortex-M33 Core Device Startup File
; *           MIMXRT685S_cm33
; * @version: 2.0
; * @date:    2019-11-12
; *----------------------------------------------------------------------------
; *
; Copyright 1997-2016 Freescale Semiconductor, Inc.
; Copyright 2016-2020 NXP
; All rights reserved.
;
; SPDX-License-Identifier: BSD-3-Clause
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)
        SECTION RO:CODE:NOROOT(2)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

;;The vector table is not needed for initialization.
__iar_init$$done

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
__vector_table_0x1c
        DCD     SecureFault_Handler
#if (__ARM_FEATURE_CMSE & 0x2)
        DCD     0x180000 ;Image length
#else
        DCD     sfe(RO) - __vector_table ;Image length
#endif
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     __vector_table  ;Image load address
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     WDT0_IRQHandler  ; Windowed watchdog timer 0 (CM33 watchdog)
        DCD     DMA0_IRQHandler  ; DMA controller 0 (secure or CM33 DMA)
        DCD     GPIO_INTA_IRQHandler  ; GPIO interrupt A
        DCD     GPIO_INTB_IRQHandler  ; GPIO interrupt B
        DCD     PIN_INT0_IRQHandler  ; Pin interrupt 0 or pattern match engine slice 0 int
        DCD     PIN_INT1_IRQHandler  ; Pin interrupt 1 or pattern match engine slice 1 int
        DCD     PIN_INT2_IRQHandler  ; Pin interrupt 2 or pattern match engine slice 2 int
        DCD     PIN_INT3_IRQHandler  ; Pin interrupt 3 or pattern match engine slice 3 int
        DCD     UTICK0_IRQHandler  ; Micro-tick Timer
        DCD     MRT0_IRQHandler  ; Multi-Rate Timer
        DCD     CTIMER0_IRQHandler  ; Standard counter/timer CTIMER0
        DCD     CTIMER1_IRQHandler  ; Standard counter/timer CTIMER1
        DCD     SCT0_IRQHandler  ; SCTimer/PWM
        DCD     CTIMER3_IRQHandler  ; Standard counter/timer CTIMER3
        DCD     FLEXCOMM0_IRQHandler  ; Flexcomm Interface 0 (USART, SPI, I2C, I2S)
        DCD     FLEXCOMM1_IRQHandler  ; Flexcomm Interface 1 (USART, SPI, I2C, I2S)
        DCD     FLEXCOMM2_IRQHandler  ; Flexcomm Interface 2 (USART, SPI, I2C, I2S)
        DCD     FLEXCOMM3_IRQHandler  ; Flexcomm Interface 3 (USART, SPI, I2C, I2S)
        DCD     FLEXCOMM4_IRQHandler  ; Flexcomm Interface 4 (USART, SPI, I2C, I2S)
        DCD     FLEXCOMM5_IRQHandler  ; Flexcomm Interface 5 (USART, SPI, I2C, I2S)
        DCD     FLEXCOMM14_IRQHandler  ; Flexcomm Interface 14 (SPI only)
        DCD     FLEXCOMM15_IRQHandler  ; Flexcomm Interface 15 (I2C only)
        DCD     ADC0_IRQHandler  ; ADC0
        DCD     Reserved39_IRQHandler  ; Reserved interrupt
        DCD     ACMP_IRQHandler  ; Analog comparator
        DCD     DMIC0_IRQHandler  ; Digital microphone and DMIC subsystem
        DCD     Reserved42_IRQHandler  ; Reserved interrupt
        DCD     HYPERVISOR_IRQHandler  ; Hypervisor
        DCD     SECUREVIOLATION_IRQHandler  ; Secure violation
        DCD     HWVAD0_IRQHandler  ; Hardware Voice Activity Detector
        DCD     Reserved46_IRQHandler  ; Reserved interrupt
        DCD     RNG_IRQHandler  ; Random number Generator
        DCD     RTC_IRQHandler  ; RTC alarm and wake-up
        DCD     DSPWAKE_IRQHandler  ; Wake-up from DSP
        DCD     MU_A_IRQHandler  ; Messaging Unit port A for CM33
        DCD     PIN_INT4_IRQHandler  ; Pin interrupt 4 or pattern match engine slice 4 int
        DCD     PIN_INT5_IRQHandler  ; Pin interrupt 5 or pattern match engine slice 5 int
        DCD     PIN_INT6_IRQHandler  ; Pin interrupt 6 or pattern match engine slice 6 int
        DCD     PIN_INT7_IRQHandler  ; Pin interrupt 7 or pattern match engine slice 7 int
        DCD     CTIMER2_IRQHandler  ; Standard counter/timer CTIMER2
        DCD     CTIMER4_IRQHandler  ; Standard counter/timer CTIMER4
        DCD     OS_EVENT_IRQHandler  ; OS event timer
        DCD     FLEXSPI_IRQHandler  ; FLEXSPI interface
        DCD     FLEXCOMM6_IRQHandler  ; Flexcomm Interface 6 (USART, SPI, I2C, I2S)
        DCD     FLEXCOMM7_IRQHandler  ; Flexcomm Interface 7 (USART, SPI, I2C, I2S)
        DCD     USDHC0_IRQHandler  ; USDHC0 (Enhanced SDHC) interrupt request
        DCD     USDHC1_IRQHandler  ; USDHC1 (Enhanced SDHC) interrupt request
        DCD     SGPIO_INTA_IRQHandler  ; Secure GPIO interrupt A
        DCD     SGPIO_INTB_IRQHandler  ; Secure GPIO interrupt B
        DCD     I3C0_IRQHandler  ; I3C interface 0
        DCD     USB_IRQHandler  ; High-speed USB device/host
        DCD     USB_WAKEUP_IRQHandler  ; USB Activity Wake-up Interrupt
        DCD     WDT1_IRQHandler  ; Windowed watchdog timer 1 (HiFi 4 watchdog)
        DCD     USBPHY_DCD_IRQHandler  ; USBPHY DCD
        DCD     DMA1_IRQHandler  ; DMA controller 1 (non-secure or HiFi 4 DMA)
        DCD     PUF_IRQHandler  ; Physical Unclonable Function
        DCD     POWERQUAD_IRQHandler  ; PowerQuad math coprocessor
        DCD     CASPER_IRQHandler  ; Casper cryptographic coprocessor
        DCD     PMC_PMIC_IRQHandler  ; Power management IC
        DCD     HASHCRYPT_IRQHandler  ; Hash-AES unit
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;

        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        CPSID   I               ; Mask interrupts
        LDR     R0, =0xE000ED08
        LDR     R1, =__vector_table
        STR     R1, [R0]
        LDR     R2, [R1]
        MSR     MSP, R2
        LDR     R0, =sfb(CSTACK)
        MSR     MSPLIM, R0
        CPSIE   I               ; Unmask interrupts
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B .

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B .

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B .

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B .

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B .

        PUBWEAK SecureFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SecureFault_Handler
        B .

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B .

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B .

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B .

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B .

        PUBWEAK WDT0_IRQHandler
        PUBWEAK WDT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
WDT0_IRQHandler
        LDR     R0, =WDT0_DriverIRQHandler
        BX      R0
        PUBWEAK DMA0_IRQHandler
        PUBWEAK DMA0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA0_IRQHandler
        LDR     R0, =DMA0_DriverIRQHandler
        BX      R0
        PUBWEAK GPIO_INTA_IRQHandler
        PUBWEAK GPIO_INTA_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
GPIO_INTA_IRQHandler
        LDR     R0, =GPIO_INTA_DriverIRQHandler
        BX      R0
        PUBWEAK GPIO_INTB_IRQHandler
        PUBWEAK GPIO_INTB_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
GPIO_INTB_IRQHandler
        LDR     R0, =GPIO_INTB_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT0_IRQHandler
        PUBWEAK PIN_INT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT0_IRQHandler
        LDR     R0, =PIN_INT0_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT1_IRQHandler
        PUBWEAK PIN_INT1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT1_IRQHandler
        LDR     R0, =PIN_INT1_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT2_IRQHandler
        PUBWEAK PIN_INT2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT2_IRQHandler
        LDR     R0, =PIN_INT2_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT3_IRQHandler
        PUBWEAK PIN_INT3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT3_IRQHandler
        LDR     R0, =PIN_INT3_DriverIRQHandler
        BX      R0
        PUBWEAK UTICK0_IRQHandler
        PUBWEAK UTICK0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
UTICK0_IRQHandler
        LDR     R0, =UTICK0_DriverIRQHandler
        BX      R0
        PUBWEAK MRT0_IRQHandler
        PUBWEAK MRT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
MRT0_IRQHandler
        LDR     R0, =MRT0_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER0_IRQHandler
        PUBWEAK CTIMER0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER0_IRQHandler
        LDR     R0, =CTIMER0_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER1_IRQHandler
        PUBWEAK CTIMER1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER1_IRQHandler
        LDR     R0, =CTIMER1_DriverIRQHandler
        BX      R0
        PUBWEAK SCT0_IRQHandler
        PUBWEAK SCT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SCT0_IRQHandler
        LDR     R0, =SCT0_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER3_IRQHandler
        PUBWEAK CTIMER3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER3_IRQHandler
        LDR     R0, =CTIMER3_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM0_IRQHandler
        PUBWEAK FLEXCOMM0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM0_IRQHandler
        LDR     R0, =FLEXCOMM0_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM1_IRQHandler
        PUBWEAK FLEXCOMM1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM1_IRQHandler
        LDR     R0, =FLEXCOMM1_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM2_IRQHandler
        PUBWEAK FLEXCOMM2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM2_IRQHandler
        LDR     R0, =FLEXCOMM2_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM3_IRQHandler
        PUBWEAK FLEXCOMM3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM3_IRQHandler
        LDR     R0, =FLEXCOMM3_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM4_IRQHandler
        PUBWEAK FLEXCOMM4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM4_IRQHandler
        LDR     R0, =FLEXCOMM4_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM5_IRQHandler
        PUBWEAK FLEXCOMM5_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM5_IRQHandler
        LDR     R0, =FLEXCOMM5_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM14_IRQHandler
        PUBWEAK FLEXCOMM14_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM14_IRQHandler
        LDR     R0, =FLEXCOMM14_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM15_IRQHandler
        PUBWEAK FLEXCOMM15_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM15_IRQHandler
        LDR     R0, =FLEXCOMM15_DriverIRQHandler
        BX      R0
        PUBWEAK ADC0_IRQHandler
        PUBWEAK ADC0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ADC0_IRQHandler
        LDR     R0, =ADC0_DriverIRQHandler
        BX      R0
        PUBWEAK Reserved39_IRQHandler
        PUBWEAK Reserved39_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reserved39_IRQHandler
        LDR     R0, =Reserved39_DriverIRQHandler
        BX      R0
        PUBWEAK ACMP_IRQHandler
        PUBWEAK ACMP_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ACMP_IRQHandler
        LDR     R0, =ACMP_DriverIRQHandler
        BX      R0
        PUBWEAK DMIC0_IRQHandler
        PUBWEAK DMIC0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMIC0_IRQHandler
        LDR     R0, =DMIC0_DriverIRQHandler
        BX      R0
        PUBWEAK Reserved42_IRQHandler
        PUBWEAK Reserved42_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reserved42_IRQHandler
        LDR     R0, =Reserved42_DriverIRQHandler
        BX      R0
        PUBWEAK HYPERVISOR_IRQHandler
        PUBWEAK HYPERVISOR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
HYPERVISOR_IRQHandler
        LDR     R0, =HYPERVISOR_DriverIRQHandler
        BX      R0
        PUBWEAK SECUREVIOLATION_IRQHandler
        PUBWEAK SECUREVIOLATION_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SECUREVIOLATION_IRQHandler
        LDR     R0, =SECUREVIOLATION_DriverIRQHandler
        BX      R0
        PUBWEAK HWVAD0_IRQHandler
        PUBWEAK HWVAD0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
HWVAD0_IRQHandler
        LDR     R0, =HWVAD0_DriverIRQHandler
        BX      R0
        PUBWEAK Reserved46_IRQHandler
        PUBWEAK Reserved46_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reserved46_IRQHandler
        LDR     R0, =Reserved46_DriverIRQHandler
        BX      R0
        PUBWEAK RNG_IRQHandler
        PUBWEAK RNG_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
RNG_IRQHandler
        LDR     R0, =RNG_DriverIRQHandler
        BX      R0
        PUBWEAK RTC_IRQHandler
        PUBWEAK RTC_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
RTC_IRQHandler
        LDR     R0, =RTC_DriverIRQHandler
        BX      R0
        PUBWEAK DSPWAKE_IRQHandler
        PUBWEAK DSPWAKE_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DSPWAKE_IRQHandler
        LDR     R0, =DSPWAKE_DriverIRQHandler
        BX      R0
        PUBWEAK MU_A_IRQHandler
        PUBWEAK MU_A_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
MU_A_IRQHandler
        LDR     R0, =MU_A_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT4_IRQHandler
        PUBWEAK PIN_INT4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT4_IRQHandler
        LDR     R0, =PIN_INT4_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT5_IRQHandler
        PUBWEAK PIN_INT5_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT5_IRQHandler
        LDR     R0, =PIN_INT5_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT6_IRQHandler
        PUBWEAK PIN_INT6_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT6_IRQHandler
        LDR     R0, =PIN_INT6_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT7_IRQHandler
        PUBWEAK PIN_INT7_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT7_IRQHandler
        LDR     R0, =PIN_INT7_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER2_IRQHandler
        PUBWEAK CTIMER2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER2_IRQHandler
        LDR     R0, =CTIMER2_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER4_IRQHandler
        PUBWEAK CTIMER4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER4_IRQHandler
        LDR     R0, =CTIMER4_DriverIRQHandler
        BX      R0
        PUBWEAK OS_EVENT_IRQHandler
        PUBWEAK OS_EVENT_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
OS_EVENT_IRQHandler
        LDR     R0, =OS_EVENT_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXSPI_IRQHandler
        PUBWEAK FLEXSPI_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXSPI_IRQHandler
        LDR     R0, =FLEXSPI_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM6_IRQHandler
        PUBWEAK FLEXCOMM6_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM6_IRQHandler
        LDR     R0, =FLEXCOMM6_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM7_IRQHandler
        PUBWEAK FLEXCOMM7_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM7_IRQHandler
        LDR     R0, =FLEXCOMM7_DriverIRQHandler
        BX      R0
        PUBWEAK USDHC0_IRQHandler
        PUBWEAK USDHC0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USDHC0_IRQHandler
        LDR     R0, =USDHC0_DriverIRQHandler
        BX      R0
        PUBWEAK USDHC1_IRQHandler
        PUBWEAK USDHC1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USDHC1_IRQHandler
        LDR     R0, =USDHC1_DriverIRQHandler
        BX      R0
        PUBWEAK SGPIO_INTA_IRQHandler
        PUBWEAK SGPIO_INTA_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SGPIO_INTA_IRQHandler
        LDR     R0, =SGPIO_INTA_DriverIRQHandler
        BX      R0
        PUBWEAK SGPIO_INTB_IRQHandler
        PUBWEAK SGPIO_INTB_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SGPIO_INTB_IRQHandler
        LDR     R0, =SGPIO_INTB_DriverIRQHandler
        BX      R0
        PUBWEAK I3C0_IRQHandler
        PUBWEAK I3C0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
I3C0_IRQHandler
        LDR     R0, =I3C0_DriverIRQHandler
        BX      R0
        PUBWEAK USB_IRQHandler
        PUBWEAK USB_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USB_IRQHandler
        LDR     R0, =USB_DriverIRQHandler
        BX      R0
        PUBWEAK USB_WAKEUP_IRQHandler
        PUBWEAK USB_WAKEUP_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USB_WAKEUP_IRQHandler
        LDR     R0, =USB_WAKEUP_DriverIRQHandler
        BX      R0
        PUBWEAK WDT1_IRQHandler
        PUBWEAK WDT1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
WDT1_IRQHandler
        LDR     R0, =WDT1_DriverIRQHandler
        BX      R0
        PUBWEAK USBPHY_DCD_IRQHandler
        PUBWEAK USBPHY_DCD_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USBPHY_DCD_IRQHandler
        LDR     R0, =USBPHY_DCD_DriverIRQHandler
        BX      R0
        PUBWEAK DMA1_IRQHandler
        PUBWEAK DMA1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA1_IRQHandler
        LDR     R0, =DMA1_DriverIRQHandler
        BX      R0
        PUBWEAK PUF_IRQHandler
        PUBWEAK PUF_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PUF_IRQHandler
        LDR     R0, =PUF_DriverIRQHandler
        BX      R0
        PUBWEAK POWERQUAD_IRQHandler
        PUBWEAK POWERQUAD_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
POWERQUAD_IRQHandler
        LDR     R0, =POWERQUAD_DriverIRQHandler
        BX      R0
        PUBWEAK CASPER_IRQHandler
        PUBWEAK CASPER_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CASPER_IRQHandler
        LDR     R0, =CASPER_DriverIRQHandler
        BX      R0
        PUBWEAK PMC_PMIC_IRQHandler
        PUBWEAK PMC_PMIC_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PMC_PMIC_IRQHandler
        LDR     R0, =PMC_PMIC_DriverIRQHandler
        BX      R0
        PUBWEAK HASHCRYPT_IRQHandler
        PUBWEAK HASHCRYPT_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
HASHCRYPT_IRQHandler
        LDR     R0, =HASHCRYPT_DriverIRQHandler
        BX      R0
WDT0_DriverIRQHandler
DMA0_DriverIRQHandler
GPIO_INTA_DriverIRQHandler
GPIO_INTB_DriverIRQHandler
PIN_INT0_DriverIRQHandler
PIN_INT1_DriverIRQHandler
PIN_INT2_DriverIRQHandler
PIN_INT3_DriverIRQHandler
UTICK0_DriverIRQHandler
MRT0_DriverIRQHandler
CTIMER0_DriverIRQHandler
CTIMER1_DriverIRQHandler
SCT0_DriverIRQHandler
CTIMER3_DriverIRQHandler
FLEXCOMM0_DriverIRQHandler
FLEXCOMM1_DriverIRQHandler
FLEXCOMM2_DriverIRQHandler
FLEXCOMM3_DriverIRQHandler
FLEXCOMM4_DriverIRQHandler
FLEXCOMM5_DriverIRQHandler
FLEXCOMM14_DriverIRQHandler
FLEXCOMM15_DriverIRQHandler
ADC0_DriverIRQHandler
Reserved39_DriverIRQHandler
ACMP_DriverIRQHandler
DMIC0_DriverIRQHandler
Reserved42_DriverIRQHandler
HYPERVISOR_DriverIRQHandler
SECUREVIOLATION_DriverIRQHandler
HWVAD0_DriverIRQHandler
Reserved46_DriverIRQHandler
RNG_DriverIRQHandler
RTC_DriverIRQHandler
DSPWAKE_DriverIRQHandler
MU_A_DriverIRQHandler
PIN_INT4_DriverIRQHandler
PIN_INT5_DriverIRQHandler
PIN_INT6_DriverIRQHandler
PIN_INT7_DriverIRQHandler
CTIMER2_DriverIRQHandler
CTIMER4_DriverIRQHandler
OS_EVENT_DriverIRQHandler
FLEXSPI_DriverIRQHandler
FLEXCOMM6_DriverIRQHandler
FLEXCOMM7_DriverIRQHandler
USDHC0_DriverIRQHandler
USDHC1_DriverIRQHandler
SGPIO_INTA_DriverIRQHandler
SGPIO_INTB_DriverIRQHandler
I3C0_DriverIRQHandler
USB_DriverIRQHandler
USB_WAKEUP_DriverIRQHandler
WDT1_DriverIRQHandler
USBPHY_DCD_DriverIRQHandler
DMA1_DriverIRQHandler
PUF_DriverIRQHandler
POWERQUAD_DriverIRQHandler
CASPER_DriverIRQHandler
PMC_PMIC_DriverIRQHandler
HASHCRYPT_DriverIRQHandler
DefaultISR
        B .

        END
