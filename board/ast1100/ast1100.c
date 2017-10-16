/*
 * Board initialize for ASPEED AST1100
 *
 *
 */

#include <common.h>
#include <command.h>
#include <pci.h>

int board_init (void)
{
    DECLARE_GLOBAL_DATA_PTR;
    unsigned char data;
    unsigned long reg;

    /* AHB Controller */
    *((volatile ulong*) 0x1E600000)  = 0xAEED1A03;	/* unlock AHB controller */ 
    *((volatile ulong*) 0x1E60008C) |= 0x01;		/* map DRAM to 0x00000000 */

    /* Flash Controller */
#ifdef CONFIG_2SPIFLASH
    *((volatile ulong*) 0x16000000) |= 0x00001c20;	/* enable Flash Write */
#else
    *((volatile ulong*) 0x16000000) |= 0x00001c00;	/* enable Flash Write */
#endif    

    /* SCU */
    *((volatile ulong*) 0x1e6e2000) = 0x1688A8A8;	/* unlock SCU */
    reg = *((volatile ulong*) 0x1e6e2008);		/* LHCLK = HPLL/8 */
    reg &= 0x1c0fffff;                                  /* PCLK  = HPLL/8 */
    reg |= 0x61b00000;					/* BHCLK = HPLL/8 */
    *((volatile ulong*) 0x1e6e2008) = reg;     
    reg = *((volatile ulong*) 0x1e6e200c);		/* enable 2D Clk */
    *((volatile ulong*) 0x1e6e200c) &= 0xFFFFFFFD; 
                                                                          
    /* arch number */
    gd->bd->bi_arch_number = MACH_TYPE_AST1100;
                                                                                                                             
    /* adress of boot parameters */
    gd->bd->bi_boot_params = 0x40000100;
                                                                                                                             
    return 0;
}

int dram_init (void)
{
    DECLARE_GLOBAL_DATA_PTR;
                                                                                                                             
    gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
    gd->bd->bi_dram[0].size  = PHYS_SDRAM_1_SIZE;
                                                                                                                 
    return 0;
}


int misc_init_r(void)
{
    unsigned char reg1, reg2;

    /* Show H/W Version */
    reg1 = (unsigned char) (*((ulong*) 0x1e6e207c));
    reg2 = (unsigned char) (*((ulong*) 0x1e6e2070) >> 24);
    puts ("H/W:   ");
#ifdef CONFIG_AST1100_FPGA       
    printf("FPGA130 Rev. %02x, FPGA70 Rev. %02x \n", reg1, reg2);
#else
    printf("Rev. %02x \n", reg1);    
#endif

#ifdef  CONFIG_ASPEED_SLT
    setenv ("verify", "n");
#endif

    if (getenv ("eeprom") == NULL) {
	setenv ("eeprom", "y");
    }
}
