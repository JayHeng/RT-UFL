#include "bl_api.h"
#include <stdio.h>

//#define MIMXRT1064_SERIES

/***********************************************************************************************************************
 *
 *  Definitions
 *
 **********************************************************************************************************************/
#define FLEXSPI_INSTANCE (0)
#define FLEXSPI2_INSTANCE (1)

#if defined(MIMXRT1064_SERIES)
#define FLASH_BASE_ADDR 0x70000000
#define FLASH_INSTANCE FLEXSPI2_INSTANCE
#else
#define FLASH_BASE_ADDR 0x60000000
#define FLASH_INSTANCE FLEXSPI_INSTANCE
#endif

#define TEST_OFFSET 0x1000
#define TEST_LENGTH 0x100
#define CONFIG_OPTION 0xc0000007

/***********************************************************************************************************************
 *
 *  Variables
 *
 **********************************************************************************************************************/
flexspi_nor_config_t flashConfig;
serial_nor_config_option_t configOption;
uint32_t programBuffer[64];
uint32_t verifyBuffer[64];

/***********************************************************************************************************************
 *
 *  Prototypes
 *
 **********************************************************************************************************************/
void clock_init(void);
void main();

/***********************************************************************************************************************
 *
 *  Codes
 *
 **********************************************************************************************************************/
void clock_init(void)
{
    if (CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_BYPASS_MASK)
    {
        // Configure ARM_PLL
        CCM_ANALOG->PLL_ARM =
            CCM_ANALOG_PLL_ARM_BYPASS(1) | CCM_ANALOG_PLL_ARM_ENABLE(1) | CCM_ANALOG_PLL_ARM_DIV_SELECT(24);
        // Wait Until clock is locked
        while ((CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_LOCK_MASK) == 0)
        {
        }

        // Configure PLL_SYS
        CCM_ANALOG->PLL_SYS &= ~CCM_ANALOG_PLL_SYS_POWERDOWN_MASK;
        // Wait Until clock is locked
        while ((CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_LOCK_MASK) == 0)
        {
        }

        // Configure PFD_528
        CCM_ANALOG->PFD_528 = CCM_ANALOG_PFD_528_PFD0_FRAC(24) | CCM_ANALOG_PFD_528_PFD1_FRAC(24) |
                              CCM_ANALOG_PFD_528_PFD2_FRAC(19) | CCM_ANALOG_PFD_528_PFD3_FRAC(24);

        // Configure USB1_PLL
        CCM_ANALOG->PLL_USB1 =
            CCM_ANALOG_PLL_USB1_DIV_SELECT(0) | CCM_ANALOG_PLL_USB1_POWER(1) | CCM_ANALOG_PLL_USB1_ENABLE(1);
        while ((CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_LOCK_MASK) == 0)
        {
        }
        CCM_ANALOG->PLL_USB1 &= ~CCM_ANALOG_PLL_USB1_BYPASS_MASK;

        // Configure PFD_480
        CCM_ANALOG->PFD_480 = CCM_ANALOG_PFD_480_PFD0_FRAC(35) | CCM_ANALOG_PFD_480_PFD1_FRAC(35) |
                              CCM_ANALOG_PFD_480_PFD2_FRAC(26) | CCM_ANALOG_PFD_480_PFD3_FRAC(15);

        // Configure Clock PODF
        CCM->CACRR = CCM_CACRR_ARM_PODF(1);

        CCM->CBCDR = (CCM->CBCDR & (~(CCM_CBCDR_SEMC_PODF_MASK | CCM_CBCDR_AHB_PODF_MASK | CCM_CBCDR_IPG_PODF_MASK))) |
                     CCM_CBCDR_SEMC_PODF(2) | CCM_CBCDR_AHB_PODF(2) | CCM_CBCDR_IPG_PODF(2);

        // Configure FLEXSPI2 CLOCKS
        CCM->CBCMR =
            (CCM->CBCMR &
             (~(CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK | CCM_CBCMR_FLEXSPI2_CLK_SEL_MASK | CCM_CBCMR_FLEXSPI2_PODF_MASK))) |
            CCM_CBCMR_PRE_PERIPH_CLK_SEL(3) | CCM_CBCMR_FLEXSPI2_CLK_SEL(1) | CCM_CBCMR_FLEXSPI2_PODF(7);

        // Confgiure FLEXSPI CLOCKS
        CCM->CSCMR1 = ((CCM->CSCMR1 & ~(CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK | CCM_CSCMR1_FLEXSPI_PODF_MASK |
                                        CCM_CSCMR1_PERCLK_PODF_MASK | CCM_CSCMR1_PERCLK_CLK_SEL_MASK)) |
                       CCM_CSCMR1_FLEXSPI_CLK_SEL(3) | CCM_CSCMR1_FLEXSPI_PODF(7) | CCM_CSCMR1_PERCLK_PODF(1));

        // Finally, Enable PLL_ARM, PLL_SYS and PLL_USB1
        CCM_ANALOG->PLL_ARM &= ~CCM_ANALOG_PLL_ARM_BYPASS_MASK;
        CCM_ANALOG->PLL_SYS &= ~CCM_ANALOG_PLL_SYS_BYPASS_MASK;
        CCM_ANALOG->PLL_USB1 &= ~CCM_ANALOG_PLL_USB1_BYPASS_MASK;
    }
}

void main(void)
{
    printf("Initializing ROM API...\n");
    bl_api_init();

    clock_init();

    printf("Test Flash driver APIs...\n");

    uint8_t *pBufStart = (uint8_t *)programBuffer;
    for (uint32_t i = 0; i < sizeof(programBuffer); i++)
    {
        *pBufStart++ = (uint8_t)(i & 0xFF);
    }

    configOption.option0.U = CONFIG_OPTION;

    status_t status = flexspi_nor_get_config(FLASH_INSTANCE, &flashConfig, &configOption);
    if (status != kStatus_Success)
    {
        printf("Flash get configuration failed\n");
        return;
    }
    else
    {
        printf("Flash get configuration passed\n");
    }

    status = flexspi_nor_flash_init(FLASH_INSTANCE, &flashConfig);
    if (status != kStatus_Success)
    {
        printf("Flash initialization failed\n");
        return;
    }
    else
    {
        printf("Flash initialization passed\n");
    }

    status = flexspi_nor_flash_erase(FLASH_INSTANCE, &flashConfig, TEST_OFFSET, TEST_LENGTH);
    if (status != kStatus_Success)
    {
        printf("Flash erase failed\n");
        return;
    }
    else
    {
        printf("Flash erase passed\n");
    }

    flexspi_nor_flash_read(FLASH_INSTANCE, &flashConfig, verifyBuffer, TEST_OFFSET, sizeof(programBuffer));
    bool hasErased = true;
    for (uint32_t i = 0; i < ARRAY_SIZE(verifyBuffer); i++)
    {
        if (verifyBuffer[i] != 0xffffffffu)
        {
            hasErased = false;
            break;
        }
    }
    if (hasErased)
    {
        printf("Flash erase verify passed\n");
    }
    else
    {
        printf("Flash erase verify failed\n");
        return;
    }

    status = flexspi_nor_flash_page_program(FLASH_INSTANCE, &flashConfig, TEST_OFFSET, programBuffer);
    if (status != kStatus_Success)
    {
        printf("Flash program failed\n");
        return;
    }
    else
    {
        printf("Flash program passed\n");
    }
    // Verify
    flexspi_nor_flash_read(FLASH_INSTANCE, &flashConfig, verifyBuffer, TEST_OFFSET, sizeof(programBuffer));
    if (memcmp(verifyBuffer, programBuffer, sizeof(programBuffer)) == 0)
    {
        printf("Flash program verify passed\n");
    }
    else
    {
        printf("Flash program verify failed\n");
    }
}
