; -------------------------------------------------------------------------
;  @file:    startup_MIMXRT1189_cm33.s
;  @purpose: CMSIS Cortex-M33 Core Device Startup File
;            MIMXRT1189_cm33
;  @version: 0.1
;  @date:    2021-3-9
;  @build:   b211130
; -------------------------------------------------------------------------
;
; Copyright 1997-2016 Freescale Semiconductor, Inc.
; Copyright 2016-2021 NXP
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

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__iar_init$$done:              ; The vector table is not needed
                      ; until after copy initialization is done

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler                                   ;NMI Handler
        DCD     HardFault_Handler                             ;Hard Fault Handler
        DCD     MemManage_Handler                             ;MPU Fault Handler
        DCD     BusFault_Handler                              ;Bus Fault Handler
        DCD     UsageFault_Handler                            ;Usage Fault Handler
__vector_table_0x1c
        DCD     SecureFault_Handler                           ;Secure Fault Handler
        DCD     0                                             ;Reserved
        DCD     0                                             ;Reserved
        DCD     0                                             ;Reserved
        DCD     SVC_Handler                                   ;SVCall Handler
        DCD     DebugMon_Handler                              ;Debug Monitor Handler
        DCD     0                                             ;Reserved
        DCD     PendSV_Handler                                ;PendSV Handler
        DCD     SysTick_Handler                               ;SysTick Handler

                                                              ;External Interrupts
        DCD     TMR1_IRQHandler                               ;TMR1 interrupt
        DCD     DGB_TRACE_IRQHandler                          ;Debug Tarce interrupt
        DCD     CTI_TRIGGER_OUT_CM7_IRQHandler                ;CTI trigger outputs from CM7
        DCD     CTI_TRIGGER_OUT_CM33_IRQHandler               ;CTI trigger outputs from CM33
        DCD     TMR5_IRQHandler                               ;TMR5 interrupt
        DCD     TMR6_IRQHandler                               ;TMR6 interrupt
        DCD     TMR7_IRQHandler                               ;TMR7 interrupt
        DCD     TMR8_IRQHandler                               ;TMR8 interrupt
        DCD     CAN1_IRQHandler                               ;CAN1 interrupt
        DCD     CAN1_ERROR_IRQHandler                         ;CAN1 error interrupt
        DCD     GPIO1_0_IRQHandler                            ;GPIO1 interrupt 0
        DCD     GPIO1_1_IRQHandler                            ;GPIO1 interrupt 1
        DCD     Reserved28_IRQHandler                         ;Reserved interrupt
        DCD     LPI2C1_IRQHandler                             ;LPI2C1 interrupt
        DCD     LPI2C2_IRQHandler                             ;LPI2C2 interrupt
        DCD     LPIT1_IRQHandler                              ;LPIT1 interrupt
        DCD     LPSPI1_IRQHandler                             ;LPSPI1 interrupt
        DCD     LPSPI2_IRQHandler                             ;LPSPI2 interrupt
        DCD     LPTMR1_IRQHandler                             ;LPTMR1 interrupt
        DCD     LPUART1_IRQHandler                            ;LPUART1 interrupt
        DCD     LPUART2_IRQHandler                            ;LPUART2 interrupt
        DCD     Reserved37_IRQHandler                         ;Reserved interrupt
        DCD     Reserved38_IRQHandler                         ;Reserved interrupt
        DCD     PWM1_FAULT_IRQHandler                         ;PWM1 fault or reload error interrupt
        DCD     PWM1_0_IRQHandler                             ;PWM1 capture 0, compare 0, or reload 0 interrupt
        DCD     PWM1_1_IRQHandler                             ;PWM1 capture 1, compare 1, or reload 1 interrupt
        DCD     PWM1_2_IRQHandler                             ;PWM1 capture 2, compare 2, or reload 2 interrupt
        DCD     PWM1_3_IRQHandler                             ;PWM1 capture 3, compare 3, or reload 3 interrupt
        DCD     Reserved44_IRQHandler                         ;Reserved interrupt
        DCD     Reserved45_IRQHandler                         ;Reserved interrupt
        DCD     Reserved46_IRQHandler                         ;Reserved interrupt
        DCD     Reserved47_IRQHandler                         ;Reserved interrupt
        DCD     Reserved48_IRQHandler                         ;Reserved interrupt
        DCD     Reserved49_IRQHandler                         ;Reserved interrupt
        DCD     Reserved50_IRQHandler                         ;Reserved interrupt
        DCD     Reserved51_IRQHandler                         ;Reserved interrupt
        DCD     TPM1_IRQHandler                               ;TPM1 interrupt
        DCD     TPM2_IRQHandler                               ;TPM2 interrupt
        DCD     RTWDOG1_IRQHandler                            ;RTWDOG1 interrupt
        DCD     RTWDOG2_IRQHandler                            ;RTWDOG2 interrupt
        DCD     Reserved56_IRQHandler                         ;Reserved interrupt
        DCD     PDM_HWVAD_EVENT_IRQHandler                    ;HWVAD event interrupt
        DCD     PDM_HWVAD_ERROR_IRQHandler                    ;HWVAD error interrupt
        DCD     PDM_EVENT_IRQHandler                          ;PDM event interrupt
        DCD     PDM_ERROR_IRQHandler                          ;PDM error interrupt
        DCD     SAI1_IRQHandler                               ;SAI interrupt
        DCD     CACHE_ECC_IRQHandler                          ;M33 PS Tag/Data Parity Error
        DCD     M33_TCM_ECC_IRQHandler                        ;M33 TCM ECC interrupt
        DCD     M33_TCM_ERROR_IRQHandler                      ;M33 TCM Error interrupt
        DCD     M7_TCM_ECC_IRQHandler                         ;M7 TCM ECC interrupt
        DCD     M7_TCM_ERROR_IRQHandler                       ;M7 TCM Error interrupt
        DCD     CAN2_IRQHandler                               ;CAN2 interrupt
        DCD     CAN2_ERROR_IRQHandler                         ;CAN2 error interrupt
        DCD     FLEXIO1_IRQHandler                            ;FLEXIO1 interrupt
        DCD     FLEXIO2_IRQHandler                            ;FLEXIO2 interrupt
        DCD     FLEXSPI1_IRQHandler                           ;FLEXSPI1 interrupt
        DCD     FLEXSPI2_IRQHandler                           ;FLEXSPI2 interrupt
        DCD     GPIO2_0_IRQHandler                            ;GPIO2 interrupt 0
        DCD     GPIO2_1_IRQHandler                            ;GPIO2 interrupt 1
        DCD     GPIO3_0_IRQHandler                            ;GPIO3 interrupt 0
        DCD     GPIO3_1_IRQHandler                            ;GPIO3 interrupt 1
        DCD     Reserved77_IRQHandler                         ;Reserved interrupt
        DCD     LPI2C3_IRQHandler                             ;LPI2C3 interrupt
        DCD     LPI2C4_IRQHandler                             ;LPI2C4 interrput
        DCD     LPIT2_IRQHandler                              ;LPIT2 interrupt
        DCD     LPSPI3_IRQHandler                             ;LPSPI3 interrupt
        DCD     LPSPI4_IRQHandler                             ;LPSPI4 interrupt
        DCD     LPTMR2_IRQHandler                             ;LPTMR2 interrupt
        DCD     LPUART3_IRQHandler                            ;LPUART3 interrupt
        DCD     LPUART4_IRQHandler                            ;LPUART4 interrupt
        DCD     LPUART5_IRQHandler                            ;LPUART5 interrupt
        DCD     LPUART6_IRQHandler                            ;LPUART6 interrupt
        DCD     ASRC_IRQHandler                               ;ASRC interrupt
        DCD     BBNSM_IRQHandler                              ;BBNSM iterrupt
        DCD     SYS_CTRL1_IRQHandler                          ;System Counter compare interrupt 0 and 1
        DCD     TPM3_IRQHandler                               ;TPM3 interrupt
        DCD     TPM4_IRQHandler                               ;TPM4 interrupt
        DCD     TPM5_IRQHandler                               ;TPM5 interrupt
        DCD     TPM6_IRQHandler                               ;TPM6 interrupt
        DCD     RTWDOG3_IRQHandler                            ;RTWDOG3 interrupt
        DCD     RTWDOG4_IRQHandler                            ;RTWDOG4 interrupt
        DCD     RTWDOG5_IRQHandler                            ;RTWDOG5 interrupt
        DCD     Reserved98_IRQHandler                         ;Reserved interrupt
        DCD     TEMPSENSOR_IRQHandler                         ;TempSensor alarm
        DCD     BBSM_ALARM_IRQHandler                         ;BBSM alarm interrupt
        DCD     Reserved101_IRQHandler                        ;Reserved interrupt
        DCD     USDHC1_IRQHandler                             ;USDHC1
        DCD     USDHC2_IRQHandler                             ;USDHC2
        DCD     Reserved104_IRQHandler                        ;Reserved interrupt
        DCD     TMR2_IRQHandler                               ;TMR2 interrupt
        DCD     LDO_1P0_IRQHandler                            ;LDO 1P8 interrupt
        DCD     MECC1_IRQHandler                              ;MECC1 interrupt
        DCD     MECC2_IRQHandler                              ;MECC2 interrupt
        DCD     ADC1_IRQHandler                               ;ADC1 interrupt
        DCD     DMA_ERROR_IRQHandler                          ;DMA error interrupt
        DCD     DMA_0_IRQHandler                              ;DMA channel 0 interrupt
        DCD     DMA_1_IRQHandler                              ;DMA channel 1 interrupt
        DCD     DMA_2_IRQHandler                              ;DMA channel 2 interrupt
        DCD     DMA_3_IRQHandler                              ;DMA channel 3 interrupt
        DCD     DMA_4_IRQHandler                              ;DMA channel 4 interrupt
        DCD     DMA_5_IRQHandler                              ;DMA channel 5 interrupt
        DCD     DMA_6_IRQHandler                              ;DMA channel 6 interrupt
        DCD     DMA_7_IRQHandler                              ;DMA channel 7 interrupt
        DCD     DMA_8_IRQHandler                              ;DMA channel 8 interrupt
        DCD     DMA_9_IRQHandler                              ;DMA channel 9 interrupt
        DCD     DMA_10_IRQHandler                             ;DMA channel 10 interrupt
        DCD     DMA_11_IRQHandler                             ;DMA channel 11 interrupt
        DCD     DMA_12_IRQHandler                             ;DMA channel 12 interrupt
        DCD     DMA_13_IRQHandler                             ;DMA channel 13 interrupt
        DCD     DMA_14_IRQHandler                             ;DMA channel 14 interrupt
        DCD     DMA_15_IRQHandler                             ;DMA channel 15 interrupt
        DCD     DMA_16_IRQHandler                             ;DMA channel 16 interrupt
        DCD     DMA_17_IRQHandler                             ;DMA channel 17 interrupt
        DCD     DMA_18_IRQHandler                             ;DMA channel 18 interrupt
        DCD     DMA_19_IRQHandler                             ;DMA channel 19 interrupt
        DCD     DMA_20_IRQHandler                             ;DMA channel 20 interrupt
        DCD     DMA_21_IRQHandler                             ;DMA channel 21 interrupt
        DCD     DMA_22_IRQHandler                             ;DMA channel 22 interrupt
        DCD     DMA_23_IRQHandler                             ;DMA channel 23 interrupt
        DCD     DMA_24_IRQHandler                             ;DMA channel 24 interrupt
        DCD     DMA_25_IRQHandler                             ;DMA channel 25 interrupt
        DCD     DMA_26_IRQHandler                             ;DMA channel 26 interrupt
        DCD     DMA_27_IRQHandler                             ;DMA channel 27 interrupt
        DCD     DMA_28_IRQHandler                             ;DMA channel 28 interrupt
        DCD     DMA_29_IRQHandler                             ;DMA channel 29 interrupt
        DCD     DMA_30_IRQHandler                             ;DMA channel 30 interrupt
        DCD     DMA_31_IRQHandler                             ;DMA channel 31 interrupt
        DCD     DMA4_ERROR_IRQHandler                         ;DMA error interrupt
        DCD     DMA4_0_1_IRQHandler                           ;DMA channel 0/1 interrupt
        DCD     DMA4_2_3_IRQHandler                           ;DMA channel 2/3 interrupt
        DCD     DMA4_4_5_IRQHandler                           ;DMA channel 4/5 interrupt
        DCD     DMA4_6_7_IRQHandler                           ;DMA channel 6/7 interrupt
        DCD     DMA4_8_9_IRQHandler                           ;DMA channel 8/9 interrupt
        DCD     DMA4_10_11_IRQHandler                         ;DMA channel 10/11 interrupt
        DCD     DMA4_12_13_IRQHandler                         ;DMA channel 12/13 interrupt
        DCD     DMA4_14_15_IRQHandler                         ;DMA channel 14/15 interrupt
        DCD     DMA4_16_17_IRQHandler                         ;DMA channel 16/17 interrupt
        DCD     DMA4_18_19_IRQHandler                         ;DMA channel 18/19 interrupt
        DCD     DMA4_20_21_IRQHandler                         ;DMA channel 20/21 interrupt
        DCD     DMA4_22_23_IRQHandler                         ;DMA channel 22/23 interrupt
        DCD     DMA4_24_25_IRQHandler                         ;DMA channel 24/25 interrupt
        DCD     DMA4_26_27_IRQHandler                         ;DMA channel 26/27 interrupt
        DCD     DMA4_28_29_IRQHandler                         ;DMA channel 28/29 interrupt
        DCD     DMA4_30_31_IRQHandler                         ;DMA channel 30/31 interrupt
        DCD     Reserved160_IRQHandler                        ;Reserved interrupt
        DCD     Reserved161_IRQHandler                        ;Reserved interrupt
        DCD     Reserved162_IRQHandler                        ;Reserved interrupt
        DCD     Reserved163_IRQHandler                        ;Reserved interrupt
        DCD     Reserved164_IRQHandler                        ;Reserved interrupt
        DCD     Reserved165_IRQHandler                        ;Reserved interrupt
        DCD     Reserved166_IRQHandler                        ;Reserved interrupt
        DCD     Reserved167_IRQHandler                        ;Reserved interrupt
        DCD     LPI2C5_IRQHandler                             ;LPI2C5 interrupt
        DCD     LPI2C6_IRQHandler                             ;LPI2C6 interrupt
        DCD     SAI4_IRQHandler                               ;SAI4 interrupt
        DCD     SPDIF_IRQHandler                              ;SPDIF interrupt
        DCD     LPUART9_IRQHandler                            ;LPUART9 interrupt
        DCD     LPUART10_IRQHandler                           ;LPUART10 interrupt
        DCD     LPUART11_IRQHandler                           ;LPUART11 interrupt
        DCD     LPUART12_IRQHandler                           ;LPUART12 interrupt
        DCD     Reserved176_IRQHandler                        ;Reserved interrupt
        DCD     Reserved177_IRQHandler                        ;Reserved interrupt
        DCD     Reserved178_IRQHandler                        ;Reserved interrupt
        DCD     Reserved179_IRQHandler                        ;Reserved interrupt
        DCD     TMR3_IRQHandler                               ;TMR3 interrupt
        DCD     TMR4_IRQHandler                               ;TMR4 interrupt
        DCD     Reserved182_IRQHandler                        ;Reserved interrupt
        DCD     Reserved183_IRQHandler                        ;Reserved interrupt
        DCD     Reserved184_IRQHandler                        ;Reserved interrupt
        DCD     Reserved185_IRQHandler                        ;Reserved interrupt
        DCD     PWM2_FAULT_IRQHandler                         ;PWM2 fault or reload error interrupt
        DCD     PWM2_0_IRQHandler                             ;PWM2 capture 0, compare 0, or reload 0 interrupt
        DCD     PWM2_1_IRQHandler                             ;PWM2 capture 1, compare 1, or reload 1 interrupt
        DCD     PWM2_2_IRQHandler                             ;PWM2 capture 2, compare 2, or reload 2 interrupt
        DCD     PWM2_3_IRQHandler                             ;PWM2 capture 3, compare 3, or reload 3 interrupt
        DCD     PWM3_FAULT_IRQHandler                         ;PWM3 fault or reload error interrupt
        DCD     PWM3_0_IRQHandler                             ;PWM3 capture 0, compare 0, or reload 0 interrupt
        DCD     PWM3_1_IRQHandler                             ;PWM3 capture 1, compare 1, or reload 1 interrupt
        DCD     PWM3_2_IRQHandler                             ;PWM3 capture 2, compare 2, or reload 2 interrupt
        DCD     PWM3_3_IRQHandler                             ;PWM3 capture 3, compare 3, or reload 3 interrupt
        DCD     PWM4_FAULT_IRQHandler                         ;PWM4 fault or reload error interrupt
        DCD     PWM4_0_IRQHandler                             ;PWM4 capture 0, compare 0, or reload 0 interrupt
        DCD     PWM4_1_IRQHandler                             ;PWM4 capture 1, compare 1, or reload 1 interrupt
        DCD     PWM4_2_IRQHandler                             ;PWM4 capture 2, compare 2, or reload 2 interrupt
        DCD     PWM4_3_IRQHandler                             ;PWM4 capture 3, compare 3, or reload 3 interrupt
        DCD     ENC1_IRQHandler                               ;ENC1 interrupt
        DCD     ENC2_IRQHandler                               ;ENC2 interrupt
        DCD     ENC3_IRQHandler                               ;ENC3 interrupt
        DCD     ENC4_IRQHandler                               ;ENC4 interrupt
        DCD     ADC2_IRQHandler                               ;ADC2 interrupt
        DCD     DCDC_IRQHandler                               ;DCDC interrupt
        DCD     CAN3_IRQHandler                               ;CAN3 interrupt
        DCD     CAN3_ERROR_IRQHandler                         ;CAN3 error interrupt
        DCD     DAC_IRQHandler                                ;DAC interrupt
        DCD     LPSPI5_IRQHandler                             ;LPSPI5 interrupt
        DCD     LPSPI6_IRQHandler                             ;LPSPI6 interrupt
        DCD     LPUART7_IRQHandler                            ;LPUART7 interrupt
        DCD     LPUART8_IRQHandler                            ;LPUART8 interrupt
        DCD     SAI2_IRQHandler                               ;SAI2 interrupt
        DCD     SAI3_IRQHandler                               ;SAI3 interrupt
        DCD     ACMP1_IRQHandler                              ;CMP1 interrupt
        DCD     ACMP2_IRQHandler                              ;CMP2 interrupt
        DCD     ACMP3_IRQHandler                              ;CMP3 interrupt
        DCD     ACMP4_IRQHandler                              ;CMP4 interrupt
        DCD     Reserved220_IRQHandler                        ;Reserved interrupt
        DCD     Reserved221_IRQHandler                        ;Reserved interrupt
        DCD     Reserved222_IRQHandler                        ;Reserved interrupt
        DCD     Reserved223_IRQHandler                        ;Reserved interrupt
        DCD     Reserved224_IRQHandler                        ;Reserved interrupt
        DCD     GPT1_IRQHandler                               ;GPT1 interrupt
        DCD     GPT2_IRQHandler                               ;GPT2 interrupt
        DCD     KPP_IRQHandler                                ;KPP interrupt
        DCD     USBPHY1_IRQHandler                            ;USBPHY1 interrupt
        DCD     USBPHY2_IRQHandler                            ;USBPHY2 interrupt
        DCD     USB_OTG2_IRQHandler                           ;USBOTG2 interrupt
        DCD     USB_OTG1_IRQHandler                           ;USBOTG1 interrupt
        DCD     XSPI_SLV_IRQHandler                           ;XSPI_SLV interrupt
        DCD     NETC_IRQHandler                               ;NETC interrupt
        DCD     Reserved234_IRQHandler                        ;Reserved interrupt
        DCD     Reserved235_IRQHandler                        ;Reserved interrupt
        DCD     Reserved236_IRQHandler                        ;Reserved interrupt
        DCD     Reserved237_IRQHandler                        ;Reserved interrupt
        DCD     Reserved238_IRQHandler                        ;Reserved interrupt
        DCD     Reserved239_IRQHandler                        ;Reserved interrupt
        DCD     SINC1_0_IRQHandler                            ;SINC Filter Glue 1 channel 0
        DCD     SINC1_1_IRQHandler                            ;SINC Filter Glue 1 channel 1
        DCD     SINC1_2_IRQHandler                            ;SINC Filter Glue 1 channel 2
        DCD     SINC1_3_IRQHandler                            ;SINC Filter Glue 1 channel 3
        DCD     SINC2_0_IRQHandler                            ;SINC Filter Glue 2 channel 0
        DCD     SINC2_1_IRQHandler                            ;SINC Filter Glue 2 channel 1
        DCD     SINC2_2_IRQHandler                            ;SINC Filter Glue 2 channel 2
        DCD     SINC2_3_IRQHandler                            ;SINC Filter Glue 2 channel 3
        DCD     GPIO4_0_IRQHandler                            ;GPIO4 interrupt 0
        DCD     GPIO4_1_IRQHandler                            ;GPIO4 interrupt 1
        DCD     GPIO5_0_IRQHandler                            ;GPIO5 interrupt 0
        DCD     GPIO5_1_IRQHandler                            ;GPIO5 interrupt 1
        DCD     GPIO6_0_IRQHandler                            ;GPIO6 interrupt 0
        DCD     GPIO6_1_IRQHandler                            ;GPIO6 interrupt 1
        DCD     Reserved254_IRQHandler                        ;Reserved interrupt
        DCD     Reserved255_IRQHandler                        ;Reserved interrupt
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
        LDR     R0, =SystemInit
        BLX     R0
        CPSIE   I               ; Unmask interrupts
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

        PUBWEAK TMR1_IRQHandler
        PUBWEAK DGB_TRACE_IRQHandler
        PUBWEAK CTI_TRIGGER_OUT_CM7_IRQHandler
        PUBWEAK CTI_TRIGGER_OUT_CM33_IRQHandler
        PUBWEAK TMR5_IRQHandler
        PUBWEAK TMR6_IRQHandler
        PUBWEAK TMR7_IRQHandler
        PUBWEAK TMR8_IRQHandler
        PUBWEAK CAN1_IRQHandler
        PUBWEAK CAN1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CAN1_IRQHandler
        LDR     R0, =CAN1_DriverIRQHandler
        BX      R0

        PUBWEAK CAN1_ERROR_IRQHandler
        PUBWEAK CAN1_ERROR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CAN1_ERROR_IRQHandler
        LDR     R0, =CAN1_ERROR_DriverIRQHandler
        BX      R0

        PUBWEAK GPIO1_0_IRQHandler
        PUBWEAK GPIO1_1_IRQHandler
        PUBWEAK Reserved28_IRQHandler
        PUBWEAK LPI2C1_IRQHandler
        PUBWEAK LPI2C1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPI2C1_IRQHandler
        LDR     R0, =LPI2C1_DriverIRQHandler
        BX      R0

        PUBWEAK LPI2C2_IRQHandler
        PUBWEAK LPI2C2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPI2C2_IRQHandler
        LDR     R0, =LPI2C2_DriverIRQHandler
        BX      R0

        PUBWEAK LPIT1_IRQHandler
        PUBWEAK LPSPI1_IRQHandler
        PUBWEAK LPSPI1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPSPI1_IRQHandler
        LDR     R0, =LPSPI1_DriverIRQHandler
        BX      R0

        PUBWEAK LPSPI2_IRQHandler
        PUBWEAK LPSPI2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPSPI2_IRQHandler
        LDR     R0, =LPSPI2_DriverIRQHandler
        BX      R0

        PUBWEAK LPTMR1_IRQHandler
        PUBWEAK LPUART1_IRQHandler
        PUBWEAK LPUART1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART1_IRQHandler
        LDR     R0, =LPUART1_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART2_IRQHandler
        PUBWEAK LPUART2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART2_IRQHandler
        LDR     R0, =LPUART2_DriverIRQHandler
        BX      R0

        PUBWEAK Reserved37_IRQHandler
        PUBWEAK Reserved38_IRQHandler
        PUBWEAK PWM1_FAULT_IRQHandler
        PUBWEAK PWM1_0_IRQHandler
        PUBWEAK PWM1_1_IRQHandler
        PUBWEAK PWM1_2_IRQHandler
        PUBWEAK PWM1_3_IRQHandler
        PUBWEAK Reserved44_IRQHandler
        PUBWEAK Reserved45_IRQHandler
        PUBWEAK Reserved46_IRQHandler
        PUBWEAK Reserved47_IRQHandler
        PUBWEAK Reserved48_IRQHandler
        PUBWEAK Reserved49_IRQHandler
        PUBWEAK Reserved50_IRQHandler
        PUBWEAK Reserved51_IRQHandler
        PUBWEAK TPM1_IRQHandler
        PUBWEAK TPM2_IRQHandler
        PUBWEAK RTWDOG1_IRQHandler
        PUBWEAK RTWDOG2_IRQHandler
        PUBWEAK Reserved56_IRQHandler
        PUBWEAK PDM_HWVAD_EVENT_IRQHandler
        PUBWEAK PDM_HWVAD_EVENT_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PDM_HWVAD_EVENT_IRQHandler
        LDR     R0, =PDM_HWVAD_EVENT_DriverIRQHandler
        BX      R0

        PUBWEAK PDM_HWVAD_ERROR_IRQHandler
        PUBWEAK PDM_HWVAD_ERROR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PDM_HWVAD_ERROR_IRQHandler
        LDR     R0, =PDM_HWVAD_ERROR_DriverIRQHandler
        BX      R0

        PUBWEAK PDM_EVENT_IRQHandler
        PUBWEAK PDM_EVENT_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PDM_EVENT_IRQHandler
        LDR     R0, =PDM_EVENT_DriverIRQHandler
        BX      R0

        PUBWEAK PDM_ERROR_IRQHandler
        PUBWEAK PDM_ERROR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PDM_ERROR_IRQHandler
        LDR     R0, =PDM_ERROR_DriverIRQHandler
        BX      R0

        PUBWEAK SAI1_IRQHandler
        PUBWEAK SAI1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SAI1_IRQHandler
        LDR     R0, =SAI1_DriverIRQHandler
        BX      R0

        PUBWEAK CACHE_ECC_IRQHandler
        PUBWEAK M33_TCM_ECC_IRQHandler
        PUBWEAK M33_TCM_ERROR_IRQHandler
        PUBWEAK M7_TCM_ECC_IRQHandler
        PUBWEAK M7_TCM_ERROR_IRQHandler
        PUBWEAK CAN2_IRQHandler
        PUBWEAK CAN2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CAN2_IRQHandler
        LDR     R0, =CAN2_DriverIRQHandler
        BX      R0

        PUBWEAK CAN2_ERROR_IRQHandler
        PUBWEAK CAN2_ERROR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CAN2_ERROR_IRQHandler
        LDR     R0, =CAN2_ERROR_DriverIRQHandler
        BX      R0

        PUBWEAK FLEXIO1_IRQHandler
        PUBWEAK FLEXIO1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXIO1_IRQHandler
        LDR     R0, =FLEXIO1_DriverIRQHandler
        BX      R0

        PUBWEAK FLEXIO2_IRQHandler
        PUBWEAK FLEXIO2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXIO2_IRQHandler
        LDR     R0, =FLEXIO2_DriverIRQHandler
        BX      R0

        PUBWEAK FLEXSPI1_IRQHandler
        PUBWEAK FLEXSPI1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXSPI1_IRQHandler
        LDR     R0, =FLEXSPI1_DriverIRQHandler
        BX      R0

        PUBWEAK FLEXSPI2_IRQHandler
        PUBWEAK FLEXSPI2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXSPI2_IRQHandler
        LDR     R0, =FLEXSPI2_DriverIRQHandler
        BX      R0

        PUBWEAK GPIO2_0_IRQHandler
        PUBWEAK GPIO2_1_IRQHandler
        PUBWEAK GPIO3_0_IRQHandler
        PUBWEAK GPIO3_1_IRQHandler
        PUBWEAK Reserved77_IRQHandler
        PUBWEAK LPI2C3_IRQHandler
        PUBWEAK LPI2C3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPI2C3_IRQHandler
        LDR     R0, =LPI2C3_DriverIRQHandler
        BX      R0

        PUBWEAK LPI2C4_IRQHandler
        PUBWEAK LPI2C4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPI2C4_IRQHandler
        LDR     R0, =LPI2C4_DriverIRQHandler
        BX      R0

        PUBWEAK LPIT2_IRQHandler
        PUBWEAK LPSPI3_IRQHandler
        PUBWEAK LPSPI3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPSPI3_IRQHandler
        LDR     R0, =LPSPI3_DriverIRQHandler
        BX      R0

        PUBWEAK LPSPI4_IRQHandler
        PUBWEAK LPSPI4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPSPI4_IRQHandler
        LDR     R0, =LPSPI4_DriverIRQHandler
        BX      R0

        PUBWEAK LPTMR2_IRQHandler
        PUBWEAK LPUART3_IRQHandler
        PUBWEAK LPUART3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART3_IRQHandler
        LDR     R0, =LPUART3_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART4_IRQHandler
        PUBWEAK LPUART4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART4_IRQHandler
        LDR     R0, =LPUART4_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART5_IRQHandler
        PUBWEAK LPUART5_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART5_IRQHandler
        LDR     R0, =LPUART5_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART6_IRQHandler
        PUBWEAK LPUART6_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART6_IRQHandler
        LDR     R0, =LPUART6_DriverIRQHandler
        BX      R0

        PUBWEAK ASRC_IRQHandler
        PUBWEAK ASRC_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ASRC_IRQHandler
        LDR     R0, =ASRC_DriverIRQHandler
        BX      R0

        PUBWEAK BBNSM_IRQHandler
        PUBWEAK SYS_CTRL1_IRQHandler
        PUBWEAK TPM3_IRQHandler
        PUBWEAK TPM4_IRQHandler
        PUBWEAK TPM5_IRQHandler
        PUBWEAK TPM6_IRQHandler
        PUBWEAK RTWDOG3_IRQHandler
        PUBWEAK RTWDOG4_IRQHandler
        PUBWEAK RTWDOG5_IRQHandler
        PUBWEAK Reserved98_IRQHandler
        PUBWEAK TEMPSENSOR_IRQHandler
        PUBWEAK BBSM_ALARM_IRQHandler
        PUBWEAK Reserved101_IRQHandler
        PUBWEAK USDHC1_IRQHandler
        PUBWEAK USDHC1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USDHC1_IRQHandler
        LDR     R0, =USDHC1_DriverIRQHandler
        BX      R0

        PUBWEAK USDHC2_IRQHandler
        PUBWEAK USDHC2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USDHC2_IRQHandler
        LDR     R0, =USDHC2_DriverIRQHandler
        BX      R0

        PUBWEAK Reserved104_IRQHandler
        PUBWEAK TMR2_IRQHandler
        PUBWEAK LDO_1P0_IRQHandler
        PUBWEAK MECC1_IRQHandler
        PUBWEAK MECC2_IRQHandler
        PUBWEAK ADC1_IRQHandler
        PUBWEAK DMA_ERROR_IRQHandler
        PUBWEAK DMA_ERROR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_ERROR_IRQHandler
        LDR     R0, =DMA_ERROR_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_0_IRQHandler
        PUBWEAK DMA_0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_0_IRQHandler
        LDR     R0, =DMA_0_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_1_IRQHandler
        PUBWEAK DMA_1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_1_IRQHandler
        LDR     R0, =DMA_1_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_2_IRQHandler
        PUBWEAK DMA_2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_2_IRQHandler
        LDR     R0, =DMA_2_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_3_IRQHandler
        PUBWEAK DMA_3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_3_IRQHandler
        LDR     R0, =DMA_3_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_4_IRQHandler
        PUBWEAK DMA_4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_4_IRQHandler
        LDR     R0, =DMA_4_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_5_IRQHandler
        PUBWEAK DMA_5_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_5_IRQHandler
        LDR     R0, =DMA_5_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_6_IRQHandler
        PUBWEAK DMA_6_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_6_IRQHandler
        LDR     R0, =DMA_6_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_7_IRQHandler
        PUBWEAK DMA_7_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_7_IRQHandler
        LDR     R0, =DMA_7_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_8_IRQHandler
        PUBWEAK DMA_8_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_8_IRQHandler
        LDR     R0, =DMA_8_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_9_IRQHandler
        PUBWEAK DMA_9_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_9_IRQHandler
        LDR     R0, =DMA_9_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_10_IRQHandler
        PUBWEAK DMA_10_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_10_IRQHandler
        LDR     R0, =DMA_10_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_11_IRQHandler
        PUBWEAK DMA_11_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_11_IRQHandler
        LDR     R0, =DMA_11_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_12_IRQHandler
        PUBWEAK DMA_12_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_12_IRQHandler
        LDR     R0, =DMA_12_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_13_IRQHandler
        PUBWEAK DMA_13_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_13_IRQHandler
        LDR     R0, =DMA_13_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_14_IRQHandler
        PUBWEAK DMA_14_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_14_IRQHandler
        LDR     R0, =DMA_14_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_15_IRQHandler
        PUBWEAK DMA_15_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_15_IRQHandler
        LDR     R0, =DMA_15_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_16_IRQHandler
        PUBWEAK DMA_16_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_16_IRQHandler
        LDR     R0, =DMA_16_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_17_IRQHandler
        PUBWEAK DMA_17_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_17_IRQHandler
        LDR     R0, =DMA_17_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_18_IRQHandler
        PUBWEAK DMA_18_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_18_IRQHandler
        LDR     R0, =DMA_18_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_19_IRQHandler
        PUBWEAK DMA_19_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_19_IRQHandler
        LDR     R0, =DMA_19_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_20_IRQHandler
        PUBWEAK DMA_20_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_20_IRQHandler
        LDR     R0, =DMA_20_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_21_IRQHandler
        PUBWEAK DMA_21_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_21_IRQHandler
        LDR     R0, =DMA_21_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_22_IRQHandler
        PUBWEAK DMA_22_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_22_IRQHandler
        LDR     R0, =DMA_22_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_23_IRQHandler
        PUBWEAK DMA_23_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_23_IRQHandler
        LDR     R0, =DMA_23_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_24_IRQHandler
        PUBWEAK DMA_24_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_24_IRQHandler
        LDR     R0, =DMA_24_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_25_IRQHandler
        PUBWEAK DMA_25_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_25_IRQHandler
        LDR     R0, =DMA_25_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_26_IRQHandler
        PUBWEAK DMA_26_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_26_IRQHandler
        LDR     R0, =DMA_26_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_27_IRQHandler
        PUBWEAK DMA_27_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_27_IRQHandler
        LDR     R0, =DMA_27_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_28_IRQHandler
        PUBWEAK DMA_28_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_28_IRQHandler
        LDR     R0, =DMA_28_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_29_IRQHandler
        PUBWEAK DMA_29_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_29_IRQHandler
        LDR     R0, =DMA_29_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_30_IRQHandler
        PUBWEAK DMA_30_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_30_IRQHandler
        LDR     R0, =DMA_30_DriverIRQHandler
        BX      R0

        PUBWEAK DMA_31_IRQHandler
        PUBWEAK DMA_31_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA_31_IRQHandler
        LDR     R0, =DMA_31_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_ERROR_IRQHandler
        PUBWEAK DMA4_ERROR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_ERROR_IRQHandler
        LDR     R0, =DMA4_ERROR_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_0_1_IRQHandler
        PUBWEAK DMA4_0_1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_0_1_IRQHandler
        LDR     R0, =DMA4_0_1_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_2_3_IRQHandler
        PUBWEAK DMA4_2_3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_2_3_IRQHandler
        LDR     R0, =DMA4_2_3_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_4_5_IRQHandler
        PUBWEAK DMA4_4_5_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_4_5_IRQHandler
        LDR     R0, =DMA4_4_5_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_6_7_IRQHandler
        PUBWEAK DMA4_6_7_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_6_7_IRQHandler
        LDR     R0, =DMA4_6_7_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_8_9_IRQHandler
        PUBWEAK DMA4_8_9_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_8_9_IRQHandler
        LDR     R0, =DMA4_8_9_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_10_11_IRQHandler
        PUBWEAK DMA4_10_11_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_10_11_IRQHandler
        LDR     R0, =DMA4_10_11_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_12_13_IRQHandler
        PUBWEAK DMA4_12_13_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_12_13_IRQHandler
        LDR     R0, =DMA4_12_13_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_14_15_IRQHandler
        PUBWEAK DMA4_14_15_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_14_15_IRQHandler
        LDR     R0, =DMA4_14_15_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_16_17_IRQHandler
        PUBWEAK DMA4_16_17_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_16_17_IRQHandler
        LDR     R0, =DMA4_16_17_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_18_19_IRQHandler
        PUBWEAK DMA4_18_19_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_18_19_IRQHandler
        LDR     R0, =DMA4_18_19_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_20_21_IRQHandler
        PUBWEAK DMA4_20_21_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_20_21_IRQHandler
        LDR     R0, =DMA4_20_21_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_22_23_IRQHandler
        PUBWEAK DMA4_22_23_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_22_23_IRQHandler
        LDR     R0, =DMA4_22_23_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_24_25_IRQHandler
        PUBWEAK DMA4_24_25_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_24_25_IRQHandler
        LDR     R0, =DMA4_24_25_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_26_27_IRQHandler
        PUBWEAK DMA4_26_27_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_26_27_IRQHandler
        LDR     R0, =DMA4_26_27_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_28_29_IRQHandler
        PUBWEAK DMA4_28_29_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_28_29_IRQHandler
        LDR     R0, =DMA4_28_29_DriverIRQHandler
        BX      R0

        PUBWEAK DMA4_30_31_IRQHandler
        PUBWEAK DMA4_30_31_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA4_30_31_IRQHandler
        LDR     R0, =DMA4_30_31_DriverIRQHandler
        BX      R0

        PUBWEAK Reserved160_IRQHandler
        PUBWEAK Reserved161_IRQHandler
        PUBWEAK Reserved162_IRQHandler
        PUBWEAK Reserved163_IRQHandler
        PUBWEAK Reserved164_IRQHandler
        PUBWEAK Reserved165_IRQHandler
        PUBWEAK Reserved166_IRQHandler
        PUBWEAK Reserved167_IRQHandler
        PUBWEAK LPI2C5_IRQHandler
        PUBWEAK LPI2C5_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPI2C5_IRQHandler
        LDR     R0, =LPI2C5_DriverIRQHandler
        BX      R0

        PUBWEAK LPI2C6_IRQHandler
        PUBWEAK LPI2C6_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPI2C6_IRQHandler
        LDR     R0, =LPI2C6_DriverIRQHandler
        BX      R0

        PUBWEAK SAI4_IRQHandler
        PUBWEAK SAI4_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SAI4_IRQHandler
        LDR     R0, =SAI4_DriverIRQHandler
        BX      R0

        PUBWEAK SPDIF_IRQHandler
        PUBWEAK SPDIF_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SPDIF_IRQHandler
        LDR     R0, =SPDIF_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART9_IRQHandler
        PUBWEAK LPUART9_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART9_IRQHandler
        LDR     R0, =LPUART9_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART10_IRQHandler
        PUBWEAK LPUART10_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART10_IRQHandler
        LDR     R0, =LPUART10_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART11_IRQHandler
        PUBWEAK LPUART11_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART11_IRQHandler
        LDR     R0, =LPUART11_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART12_IRQHandler
        PUBWEAK LPUART12_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART12_IRQHandler
        LDR     R0, =LPUART12_DriverIRQHandler
        BX      R0

        PUBWEAK Reserved176_IRQHandler
        PUBWEAK Reserved177_IRQHandler
        PUBWEAK Reserved178_IRQHandler
        PUBWEAK Reserved179_IRQHandler
        PUBWEAK TMR3_IRQHandler
        PUBWEAK TMR4_IRQHandler
        PUBWEAK Reserved182_IRQHandler
        PUBWEAK Reserved183_IRQHandler
        PUBWEAK Reserved184_IRQHandler
        PUBWEAK Reserved185_IRQHandler
        PUBWEAK PWM2_FAULT_IRQHandler
        PUBWEAK PWM2_0_IRQHandler
        PUBWEAK PWM2_1_IRQHandler
        PUBWEAK PWM2_2_IRQHandler
        PUBWEAK PWM2_3_IRQHandler
        PUBWEAK PWM3_FAULT_IRQHandler
        PUBWEAK PWM3_0_IRQHandler
        PUBWEAK PWM3_1_IRQHandler
        PUBWEAK PWM3_2_IRQHandler
        PUBWEAK PWM3_3_IRQHandler
        PUBWEAK PWM4_FAULT_IRQHandler
        PUBWEAK PWM4_0_IRQHandler
        PUBWEAK PWM4_1_IRQHandler
        PUBWEAK PWM4_2_IRQHandler
        PUBWEAK PWM4_3_IRQHandler
        PUBWEAK ENC1_IRQHandler
        PUBWEAK ENC2_IRQHandler
        PUBWEAK ENC3_IRQHandler
        PUBWEAK ENC4_IRQHandler
        PUBWEAK ADC2_IRQHandler
        PUBWEAK DCDC_IRQHandler
        PUBWEAK CAN3_IRQHandler
        PUBWEAK CAN3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CAN3_IRQHandler
        LDR     R0, =CAN3_DriverIRQHandler
        BX      R0

        PUBWEAK CAN3_ERROR_IRQHandler
        PUBWEAK CAN3_ERROR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CAN3_ERROR_IRQHandler
        LDR     R0, =CAN3_ERROR_DriverIRQHandler
        BX      R0

        PUBWEAK DAC_IRQHandler
        PUBWEAK LPSPI5_IRQHandler
        PUBWEAK LPSPI5_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPSPI5_IRQHandler
        LDR     R0, =LPSPI5_DriverIRQHandler
        BX      R0

        PUBWEAK LPSPI6_IRQHandler
        PUBWEAK LPSPI6_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPSPI6_IRQHandler
        LDR     R0, =LPSPI6_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART7_IRQHandler
        PUBWEAK LPUART7_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART7_IRQHandler
        LDR     R0, =LPUART7_DriverIRQHandler
        BX      R0

        PUBWEAK LPUART8_IRQHandler
        PUBWEAK LPUART8_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
LPUART8_IRQHandler
        LDR     R0, =LPUART8_DriverIRQHandler
        BX      R0

        PUBWEAK SAI2_IRQHandler
        PUBWEAK SAI2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SAI2_IRQHandler
        LDR     R0, =SAI2_DriverIRQHandler
        BX      R0

        PUBWEAK SAI3_IRQHandler
        PUBWEAK SAI3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SAI3_IRQHandler
        LDR     R0, =SAI3_DriverIRQHandler
        BX      R0

        PUBWEAK ACMP1_IRQHandler
        PUBWEAK ACMP2_IRQHandler
        PUBWEAK ACMP3_IRQHandler
        PUBWEAK ACMP4_IRQHandler
        PUBWEAK Reserved220_IRQHandler
        PUBWEAK Reserved221_IRQHandler
        PUBWEAK Reserved222_IRQHandler
        PUBWEAK Reserved223_IRQHandler
        PUBWEAK Reserved224_IRQHandler
        PUBWEAK GPT1_IRQHandler
        PUBWEAK GPT2_IRQHandler
        PUBWEAK KPP_IRQHandler
        PUBWEAK USBPHY1_IRQHandler
        PUBWEAK USBPHY2_IRQHandler
        PUBWEAK USB_OTG2_IRQHandler
        PUBWEAK USB_OTG1_IRQHandler
        PUBWEAK XSPI_SLV_IRQHandler
        PUBWEAK XSPI_SLV_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
XSPI_SLV_IRQHandler
        LDR     R0, =XSPI_SLV_DriverIRQHandler
        BX      R0

        PUBWEAK NETC_IRQHandler
        PUBWEAK Reserved234_IRQHandler
        PUBWEAK Reserved235_IRQHandler
        PUBWEAK Reserved236_IRQHandler
        PUBWEAK Reserved237_IRQHandler
        PUBWEAK Reserved238_IRQHandler
        PUBWEAK Reserved239_IRQHandler
        PUBWEAK SINC1_0_IRQHandler
        PUBWEAK SINC1_1_IRQHandler
        PUBWEAK SINC1_2_IRQHandler
        PUBWEAK SINC1_3_IRQHandler
        PUBWEAK SINC2_0_IRQHandler
        PUBWEAK SINC2_1_IRQHandler
        PUBWEAK SINC2_2_IRQHandler
        PUBWEAK SINC2_3_IRQHandler
        PUBWEAK GPIO4_0_IRQHandler
        PUBWEAK GPIO4_1_IRQHandler
        PUBWEAK GPIO5_0_IRQHandler
        PUBWEAK GPIO5_1_IRQHandler
        PUBWEAK GPIO6_0_IRQHandler
        PUBWEAK GPIO6_1_IRQHandler
        PUBWEAK Reserved254_IRQHandler
        PUBWEAK Reserved255_IRQHandler
        PUBWEAK DefaultISR
        SECTION .text:CODE:REORDER:NOROOT(1)
TMR1_IRQHandler
DGB_TRACE_IRQHandler
CTI_TRIGGER_OUT_CM7_IRQHandler
CTI_TRIGGER_OUT_CM33_IRQHandler
TMR5_IRQHandler
TMR6_IRQHandler
TMR7_IRQHandler
TMR8_IRQHandler
CAN1_DriverIRQHandler
CAN1_ERROR_DriverIRQHandler
GPIO1_0_IRQHandler
GPIO1_1_IRQHandler
Reserved28_IRQHandler
LPI2C1_DriverIRQHandler
LPI2C2_DriverIRQHandler
LPIT1_IRQHandler
LPSPI1_DriverIRQHandler
LPSPI2_DriverIRQHandler
LPTMR1_IRQHandler
LPUART1_DriverIRQHandler
LPUART2_DriverIRQHandler
Reserved37_IRQHandler
Reserved38_IRQHandler
PWM1_FAULT_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
Reserved44_IRQHandler
Reserved45_IRQHandler
Reserved46_IRQHandler
Reserved47_IRQHandler
Reserved48_IRQHandler
Reserved49_IRQHandler
Reserved50_IRQHandler
Reserved51_IRQHandler
TPM1_IRQHandler
TPM2_IRQHandler
RTWDOG1_IRQHandler
RTWDOG2_IRQHandler
Reserved56_IRQHandler
PDM_HWVAD_EVENT_DriverIRQHandler
PDM_HWVAD_ERROR_DriverIRQHandler
PDM_EVENT_DriverIRQHandler
PDM_ERROR_DriverIRQHandler
SAI1_DriverIRQHandler
CACHE_ECC_IRQHandler
M33_TCM_ECC_IRQHandler
M33_TCM_ERROR_IRQHandler
M7_TCM_ECC_IRQHandler
M7_TCM_ERROR_IRQHandler
CAN2_DriverIRQHandler
CAN2_ERROR_DriverIRQHandler
FLEXIO1_DriverIRQHandler
FLEXIO2_DriverIRQHandler
FLEXSPI1_DriverIRQHandler
FLEXSPI2_DriverIRQHandler
GPIO2_0_IRQHandler
GPIO2_1_IRQHandler
GPIO3_0_IRQHandler
GPIO3_1_IRQHandler
Reserved77_IRQHandler
LPI2C3_DriverIRQHandler
LPI2C4_DriverIRQHandler
LPIT2_IRQHandler
LPSPI3_DriverIRQHandler
LPSPI4_DriverIRQHandler
LPTMR2_IRQHandler
LPUART3_DriverIRQHandler
LPUART4_DriverIRQHandler
LPUART5_DriverIRQHandler
LPUART6_DriverIRQHandler
ASRC_DriverIRQHandler
BBNSM_IRQHandler
SYS_CTRL1_IRQHandler
TPM3_IRQHandler
TPM4_IRQHandler
TPM5_IRQHandler
TPM6_IRQHandler
RTWDOG3_IRQHandler
RTWDOG4_IRQHandler
RTWDOG5_IRQHandler
Reserved98_IRQHandler
TEMPSENSOR_IRQHandler
BBSM_ALARM_IRQHandler
Reserved101_IRQHandler
USDHC1_DriverIRQHandler
USDHC2_DriverIRQHandler
Reserved104_IRQHandler
TMR2_IRQHandler
LDO_1P0_IRQHandler
MECC1_IRQHandler
MECC2_IRQHandler
ADC1_IRQHandler
DMA_ERROR_DriverIRQHandler
DMA_0_DriverIRQHandler
DMA_1_DriverIRQHandler
DMA_2_DriverIRQHandler
DMA_3_DriverIRQHandler
DMA_4_DriverIRQHandler
DMA_5_DriverIRQHandler
DMA_6_DriverIRQHandler
DMA_7_DriverIRQHandler
DMA_8_DriverIRQHandler
DMA_9_DriverIRQHandler
DMA_10_DriverIRQHandler
DMA_11_DriverIRQHandler
DMA_12_DriverIRQHandler
DMA_13_DriverIRQHandler
DMA_14_DriverIRQHandler
DMA_15_DriverIRQHandler
DMA_16_DriverIRQHandler
DMA_17_DriverIRQHandler
DMA_18_DriverIRQHandler
DMA_19_DriverIRQHandler
DMA_20_DriverIRQHandler
DMA_21_DriverIRQHandler
DMA_22_DriverIRQHandler
DMA_23_DriverIRQHandler
DMA_24_DriverIRQHandler
DMA_25_DriverIRQHandler
DMA_26_DriverIRQHandler
DMA_27_DriverIRQHandler
DMA_28_DriverIRQHandler
DMA_29_DriverIRQHandler
DMA_30_DriverIRQHandler
DMA_31_DriverIRQHandler
DMA4_ERROR_DriverIRQHandler
DMA4_0_1_DriverIRQHandler
DMA4_2_3_DriverIRQHandler
DMA4_4_5_DriverIRQHandler
DMA4_6_7_DriverIRQHandler
DMA4_8_9_DriverIRQHandler
DMA4_10_11_DriverIRQHandler
DMA4_12_13_DriverIRQHandler
DMA4_14_15_DriverIRQHandler
DMA4_16_17_DriverIRQHandler
DMA4_18_19_DriverIRQHandler
DMA4_20_21_DriverIRQHandler
DMA4_22_23_DriverIRQHandler
DMA4_24_25_DriverIRQHandler
DMA4_26_27_DriverIRQHandler
DMA4_28_29_DriverIRQHandler
DMA4_30_31_DriverIRQHandler
Reserved160_IRQHandler
Reserved161_IRQHandler
Reserved162_IRQHandler
Reserved163_IRQHandler
Reserved164_IRQHandler
Reserved165_IRQHandler
Reserved166_IRQHandler
Reserved167_IRQHandler
LPI2C5_DriverIRQHandler
LPI2C6_DriverIRQHandler
SAI4_DriverIRQHandler
SPDIF_DriverIRQHandler
LPUART9_DriverIRQHandler
LPUART10_DriverIRQHandler
LPUART11_DriverIRQHandler
LPUART12_DriverIRQHandler
Reserved176_IRQHandler
Reserved177_IRQHandler
Reserved178_IRQHandler
Reserved179_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
Reserved182_IRQHandler
Reserved183_IRQHandler
Reserved184_IRQHandler
Reserved185_IRQHandler
PWM2_FAULT_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM4_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
CAN3_DriverIRQHandler
CAN3_ERROR_DriverIRQHandler
DAC_IRQHandler
LPSPI5_DriverIRQHandler
LPSPI6_DriverIRQHandler
LPUART7_DriverIRQHandler
LPUART8_DriverIRQHandler
SAI2_DriverIRQHandler
SAI3_DriverIRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved220_IRQHandler
Reserved221_IRQHandler
Reserved222_IRQHandler
Reserved223_IRQHandler
Reserved224_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
KPP_IRQHandler
USBPHY1_IRQHandler
USBPHY2_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XSPI_SLV_DriverIRQHandler
NETC_IRQHandler
Reserved234_IRQHandler
Reserved235_IRQHandler
Reserved236_IRQHandler
Reserved237_IRQHandler
Reserved238_IRQHandler
Reserved239_IRQHandler
SINC1_0_IRQHandler
SINC1_1_IRQHandler
SINC1_2_IRQHandler
SINC1_3_IRQHandler
SINC2_0_IRQHandler
SINC2_1_IRQHandler
SINC2_2_IRQHandler
SINC2_3_IRQHandler
GPIO4_0_IRQHandler
GPIO4_1_IRQHandler
GPIO5_0_IRQHandler
GPIO5_1_IRQHandler
GPIO6_0_IRQHandler
GPIO6_1_IRQHandler
Reserved254_IRQHandler
Reserved255_IRQHandler
DefaultISR
        B DefaultISR

        END
