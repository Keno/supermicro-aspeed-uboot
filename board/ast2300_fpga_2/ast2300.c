/*
 * Board initialize for ASPEED AST2100
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
#ifdef CONFIG_PCI
    *((volatile ulong*) 0x1E60008C) |= 0x30;		/* map PCI */
#endif

    /* Flash Controller */
#ifdef	CONFIG_FLASH_AST2300
    *((volatile ulong*) 0x1e620000) |= 0x800f0000;	/* enable Flash Write */     
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
/* enable wide screen. If your video driver does not support wide screen, don't
enable this bit 0x1e6e2040 D[0]*/
    reg = *((volatile ulong*) 0x1e6e2040);
    *((volatile ulong*) 0x1e6e2040) |= 0x01;
                                                                          
    /* arch number */
    gd->bd->bi_arch_number = MACH_TYPE_AST2300_FPGA_2;
                                                                                                                             
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

/*
SCU7C: Silicon Revision ID Register
D[31:24]: Chip ID
0: AST2050/AST2100/AST2150/AST2200/AST3000
1: AST2300

D[23:16] Silicon revision ID for AST2300 generation and later
0: A0
1: A1
2: A2
.
.
.
FPGA revision starts from 0x80


D[11:8] Bounding option

D[7:0] Silicon revision ID for AST2050/AST2100 generation (for software compatible)
0: A0
1: A1
2: A2
3: A3
.
.
FPGA revision starts from 0x08, 8~10 means A0, 11+ means A1, AST2300 should be assigned to 3
*/
int misc_init_r(void)
{
    unsigned int reg1, revision, chip_id;

    /* Show H/W Version */
    reg1 = (unsigned int) (*((ulong*) 0x1e6e207c));
    chip_id = (reg1 & 0xff000000) >> 24;
    revision = (reg1 & 0xff0000) >> 16;

    puts ("H/W:   ");
    if (chip_id == 1) {
    	if (revision >= 0x80) {
    		printf("AST2300 series FPGA Rev. %02x \n", revision);
    	}
    	else {
    		printf("AST2300 series chip Rev. %02x \n", revision);
    	}
    }
    else if (chip_id == 0) {
	printf("AST2050/AST2150 series chip\n");
    }

#ifdef	CONFIG_PCI	
    pci_init ();
#endif

    if (getenv ("verify") == NULL) {
	setenv ("verify", "n");
    }
    if (getenv ("eeprom") == NULL) {
	setenv ("eeprom", "y");
    }
}

#ifdef	CONFIG_PCI
static struct pci_controller hose;

extern void aspeed_init_pci (struct pci_controller *hose);

void pci_init_board(void)
{
    aspeed_init_pci(&hose);
}
#endif
