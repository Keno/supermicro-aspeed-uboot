/*
 * NAND driver for TI DaVinci based boards.
 *
 * Copyright (C) 2007 Sergey Kubushyn <ksi@koi8.net>
 *
 * Based on Linux DaVinci NAND driver by TI. Original copyright follows:
 */

/*
 *
 * linux/drivers/mtd/nand/nand_davinci.c
 *
 * NAND Flash Driver
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   DaVinci board which utilizes the Samsung k9k2g08 part.
 *
 Modifications:
 ver. 1.0: Feb 2005, Vinod/Sudhakar
 -
 *
 */

#include <common.h>
#include <asm/io.h>

#ifdef CONFIG_SYS_USE_NAND
#if !defined(CONFIG_NAND_LEGACY)

#include <nand.h>
#include <asm/arch/nand_defs.h>
#include <asm/arch/emif_defs.h>

extern struct nand_chip nand_dev_desc[CONFIG_SYS_MAX_NAND_DEVICE];

static void nand_davinci_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct		nand_chip *this = mtd->priv;
	u_int32_t	IO_ADDR_W = (u_int32_t)this->IO_ADDR_W;

	IO_ADDR_W &= ~(MASK_ALE|MASK_CLE);

	if (ctrl & NAND_CTRL_CHANGE) {
		if ( ctrl & NAND_CLE )
			IO_ADDR_W |= MASK_CLE;
		if ( ctrl & NAND_ALE )
			IO_ADDR_W |= MASK_ALE;
		this->IO_ADDR_W = (void __iomem *) IO_ADDR_W;
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, this->IO_ADDR_W);
}

/* Set WP on deselect, write enable on select */
static void nand_davinci_select_chip(struct mtd_info *mtd, int chip)
{
#define GPIO_SET_DATA01	0x01c67018
#define GPIO_CLR_DATA01	0x01c6701c
#define GPIO_NAND_WP	(1 << 4)
#ifdef SONATA_BOARD_GPIOWP
	if (chip < 0) {
		REG(GPIO_CLR_DATA01) |= GPIO_NAND_WP;
	} else {
		REG(GPIO_SET_DATA01) |= GPIO_NAND_WP;
	}
#endif
}

#ifdef CONFIG_SYS_NAND_HW_ECC
#ifdef CONFIG_SYS_DAVINCI_BROKEN_ECC
/* Linux-compatible ECC uses MTD defaults. */
/* These layouts are not compatible with Linux or RBL/UBL. */
#ifdef CONFIG_SYS_NAND_LARGEPAGE
static struct nand_ecclayout davinci_nand_ecclayout = {
	.eccbytes = 12,
	.eccpos = {8, 9, 10, 24, 25, 26, 40, 41, 42, 56, 57, 58},
	.oobfree = {
		{.offset = 2, .length = 6},
		{.offset = 12, .length = 12},
		{.offset = 28, .length = 12},
		{.offset = 44, .length = 12},
		{.offset = 60, .length = 4}
	}
};
#elif defined(CONFIG_SYS_NAND_SMALLPAGE)
static struct nand_ecclayout davinci_nand_ecclayout = {
	.eccbytes = 3,
	.eccpos = {0, 1, 2},
	.oobfree = {
		{.offset = 6, .length = 2},
		{.offset = 8, .length = 8}
	}
};
#else
#error "Either CONFIG_SYS_NAND_LARGEPAGE or CONFIG_SYS_NAND_SMALLPAGE must be defined!"
#endif
#endif /* CONFIG_SYS_DAVINCI_BROKEN_ECC */

static void nand_davinci_enable_hwecc(struct mtd_info *mtd, int mode)
{
	emifregs	emif_addr;
	int		dummy;

	emif_addr = (emifregs)DAVINCI_ASYNC_EMIF_CNTRL_BASE;

	dummy = emif_addr->NANDF1ECC;
	dummy = emif_addr->NANDF2ECC;
	dummy = emif_addr->NANDF3ECC;
	dummy = emif_addr->NANDF4ECC;

	emif_addr->NANDFCR |= (1 << 8);
}

static u_int32_t nand_davinci_readecc(struct mtd_info *mtd, u_int32_t region)
{
	u_int32_t	ecc = 0;
	emifregs	emif_base_addr;

	emif_base_addr = (emifregs)DAVINCI_ASYNC_EMIF_CNTRL_BASE;

	if (region == 1)
		ecc = emif_base_addr->NANDF1ECC;
	else if (region == 2)
		ecc = emif_base_addr->NANDF2ECC;
	else if (region == 3)
		ecc = emif_base_addr->NANDF3ECC;
	else if (region == 4)
		ecc = emif_base_addr->NANDF4ECC;

	return(ecc);
}

static int nand_davinci_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	u_int32_t		tmp;
#ifdef CONFIG_SYS_DAVINCI_BROKEN_ECC
	/*
	 * This is not how you should read ECCs on large page Davinci devices.
	 * The region parameter gets you ECCs for flash chips on different chip
	 * selects, not the 4x512 byte pages in a 2048 byte page.
	 *
	 * Preserved for backwards compatibility though.
	 */

	int			region, n;
	struct nand_chip	*this = mtd->priv;

	n = (this->ecc.size/512);

	region = 1;
	while (n--) {
		tmp = nand_davinci_readecc(mtd, region);
		*ecc_code++ = tmp;
		*ecc_code++ = tmp >> 16;
		*ecc_code++ = ((tmp >> 8) & 0x0f) | ((tmp >> 20) & 0xf0);
		region++;
	}
#else
	const int region = 1;

	tmp = nand_davinci_readecc(mtd, region);

	/* Squeeze 4 bytes ECC into 3 bytes by removing RESERVED bits
	 * and shifting. RESERVED bits are 31 to 28 and 15 to 12. */
	tmp = (tmp & 0x00000fff) | ((tmp & 0x0fff0000) >> 4);

	/* Invert so that erased block ECC is correct */
	tmp = ~tmp;

	*ecc_code++ = tmp;
	*ecc_code++ = tmp >>  8;
	*ecc_code++ = tmp >> 16;
#endif /* CONFIG_SYS_DAVINCI_BROKEN_ECC */
	return(0);
}

#ifdef CONFIG_SYS_DAVINCI_BROKEN_ECC
static void nand_davinci_gen_true_ecc(u_int8_t *ecc_buf)
{
	u_int32_t	tmp = ecc_buf[0] | (ecc_buf[1] << 16) | ((ecc_buf[2] & 0xf0) << 20) | ((ecc_buf[2] & 0x0f) << 8);

	ecc_buf[0] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) | P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp));
	ecc_buf[1] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) | P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
	ecc_buf[2] = ~( P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) | P1e(tmp) | P2048o(tmp) | P2048e(tmp));
}

static int nand_davinci_compare_ecc(u_int8_t *ecc_nand, u_int8_t *ecc_calc, u_int8_t *page_data)
{
	u_int32_t	i;
	u_int8_t	tmp0_bit[8], tmp1_bit[8], tmp2_bit[8];
	u_int8_t	comp0_bit[8], comp1_bit[8], comp2_bit[8];
	u_int8_t	ecc_bit[24];
	u_int8_t	ecc_sum = 0;
	u_int8_t	find_bit = 0;
	u_int32_t	find_byte = 0;
	int		is_ecc_ff;

	is_ecc_ff = ((*ecc_nand == 0xff) && (*(ecc_nand + 1) == 0xff) && (*(ecc_nand + 2) == 0xff));

	nand_davinci_gen_true_ecc(ecc_nand);
	nand_davinci_gen_true_ecc(ecc_calc);

	for (i = 0; i <= 2; i++) {
		*(ecc_nand + i) = ~(*(ecc_nand + i));
		*(ecc_calc + i) = ~(*(ecc_calc + i));
	}

	for (i = 0; i < 8; i++) {
		tmp0_bit[i] = *ecc_nand % 2;
		*ecc_nand = *ecc_nand / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp1_bit[i] = *(ecc_nand + 1) % 2;
		*(ecc_nand + 1) = *(ecc_nand + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp2_bit[i] = *(ecc_nand + 2) % 2;
		*(ecc_nand + 2) = *(ecc_nand + 2) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp0_bit[i] = *ecc_calc % 2;
		*ecc_calc = *ecc_calc / 2;
	}

	for (i = 0; i < 8; i++) {
		comp1_bit[i] = *(ecc_calc + 1) % 2;
		*(ecc_calc + 1) = *(ecc_calc + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp2_bit[i] = *(ecc_calc + 2) % 2;
		*(ecc_calc + 2) = *(ecc_calc + 2) / 2;
	}

	for (i = 0; i< 6; i++)
		ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];

	ecc_bit[22] = tmp2_bit[0] ^ comp2_bit[0];
	ecc_bit[23] = tmp2_bit[1] ^ comp2_bit[1];

	for (i = 0; i < 24; i++)
		ecc_sum += ecc_bit[i];

	switch (ecc_sum) {
		case 0:
			/* Not reached because this function is not called if
			   ECC values are equal */
			return 0;
		case 1:
			/* Uncorrectable error */
			MTDDEBUG (MTD_DEBUG_LEVEL0,
			          "ECC UNCORRECTED_ERROR 1\n");
			return(-1);
		case 12:
			/* Correctable error */
			find_byte = (ecc_bit[23] << 8) +
				(ecc_bit[21] << 7) +
				(ecc_bit[19] << 6) +
				(ecc_bit[17] << 5) +
				(ecc_bit[15] << 4) +
				(ecc_bit[13] << 3) +
				(ecc_bit[11] << 2) +
				(ecc_bit[9]  << 1) +
				ecc_bit[7];

			find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];

			MTDDEBUG (MTD_DEBUG_LEVEL0, "Correcting single bit ECC "
			          "error at offset: %d, bit: %d\n",
			          find_byte, find_bit);

			page_data[find_byte] ^= (1 << find_bit);

			return(0);
		default:
			if (is_ecc_ff) {
				if (ecc_calc[0] == 0 && ecc_calc[1] == 0 && ecc_calc[2] == 0)
					return(0);
			}
			MTDDEBUG (MTD_DEBUG_LEVEL0,
			          "UNCORRECTED_ERROR default\n");
			return(-1);
	}
}
#endif /* CONFIG_SYS_DAVINCI_BROKEN_ECC */

static int nand_davinci_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *this = mtd->priv;
#ifdef CONFIG_SYS_DAVINCI_BROKEN_ECC
	int			block_count = 0, i, rc;

	block_count = (this->ecc.size/512);
	for (i = 0; i < block_count; i++) {
		if (memcmp(read_ecc, calc_ecc, 3) != 0) {
			rc = nand_davinci_compare_ecc(read_ecc, calc_ecc, dat);
			if (rc < 0) {
				return(rc);
			}
		}
		read_ecc += 3;
		calc_ecc += 3;
		dat += 512;
	}
#else
	u_int32_t ecc_nand = read_ecc[0] | (read_ecc[1] << 8) |
					  (read_ecc[2] << 16);
	u_int32_t ecc_calc = calc_ecc[0] | (calc_ecc[1] << 8) |
					  (calc_ecc[2] << 16);
	u_int32_t diff = ecc_calc ^ ecc_nand;

	if (diff) {
		if ((((diff >> 12) ^ diff) & 0xfff) == 0xfff) {
			/* Correctable error */
			if ((diff >> (12 + 3)) < this->ecc.size) {
				uint8_t find_bit = 1 << ((diff >> 12) & 7);
				uint32_t find_byte = diff >> (12 + 3);

				dat[find_byte] ^= find_bit;
				MTDDEBUG(MTD_DEBUG_LEVEL0, "Correcting single "
					 "bit ECC error at offset: %d, bit: "
					 "%d\n", find_byte, find_bit);
				return 1;
			} else {
				return -1;
			}
		} else if (!(diff & (diff - 1))) {
			/* Single bit ECC error in the ECC itself,
			   nothing to fix */
			MTDDEBUG(MTD_DEBUG_LEVEL0, "Single bit ECC error in "
				 "ECC.\n");
			return 1;
		} else {
			/* Uncorrectable error */
			MTDDEBUG(MTD_DEBUG_LEVEL0, "ECC UNCORRECTED_ERROR 1\n");
			return -1;
		}
	}
#endif /* CONFIG_SYS_DAVINCI_BROKEN_ECC */
	return(0);
}
#endif /* CONFIG_SYS_NAND_HW_ECC */

static int nand_davinci_dev_ready(struct mtd_info *mtd)
{
	emifregs	emif_addr;

	emif_addr = (emifregs)DAVINCI_ASYNC_EMIF_CNTRL_BASE;

	return(emif_addr->NANDFSR & 0x1);
}

static int nand_davinci_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	while(!nand_davinci_dev_ready(mtd)) {;}
	*NAND_CE0CLE = NAND_STATUS;
	return(*NAND_CE0DATA);
}

static void nand_flash_init(void)
{
	u_int32_t	acfg1 = 0x3ffffffc;
	u_int32_t	acfg2 = 0x3ffffffc;
	u_int32_t	acfg3 = 0x3ffffffc;
	u_int32_t	acfg4 = 0x3ffffffc;
	emifregs	emif_regs;

	/*------------------------------------------------------------------*
	 *  NAND FLASH CHIP TIMEOUT @ 459 MHz                               *
	 *                                                                  *
	 *  AEMIF.CLK freq   = PLL1/6 = 459/6 = 76.5 MHz                    *
	 *  AEMIF.CLK period = 1/76.5 MHz = 13.1 ns                         *
	 *                                                                  *
	 *------------------------------------------------------------------*/
	 acfg1 = 0
		| (0 << 31 )	/* selectStrobe */
		| (0 << 30 )	/* extWait */
		| (1 << 26 )	/* writeSetup	10 ns */
		| (3 << 20 )	/* writeStrobe	40 ns */
		| (1 << 17 )	/* writeHold	10 ns */
		| (1 << 13 )	/* readSetup	10 ns */
		| (5 << 7 )	/* readStrobe	60 ns */
		| (1 << 4 )	/* readHold	10 ns */
		| (3 << 2 )	/* turnAround	?? ns */
		| (0 << 0 )	/* asyncSize	8-bit bus */
		;

	emif_regs = (emifregs)DAVINCI_ASYNC_EMIF_CNTRL_BASE;

	emif_regs->AWCCR |= 0x10000000;
	emif_regs->AB1CR = acfg1;	/* 0x08244128 */;
	emif_regs->AB2CR = acfg2;
	emif_regs->AB3CR = acfg3;
	emif_regs->AB4CR = acfg4;
	emif_regs->NANDFCR = 0x00000101;
}

int board_nand_init(struct nand_chip *nand)
{
	nand->IO_ADDR_R   = (void  __iomem *)NAND_CE0DATA;
	nand->IO_ADDR_W   = (void  __iomem *)NAND_CE0DATA;
	nand->chip_delay  = 0;
	nand->select_chip = nand_davinci_select_chip;
#ifdef CONFIG_SYS_NAND_USE_FLASH_BBT
	nand->options	  = NAND_USE_FLASH_BBT;
#endif
#ifdef CONFIG_SYS_NAND_HW_ECC
	nand->ecc.mode = NAND_ECC_HW;
#ifdef CONFIG_SYS_DAVINCI_BROKEN_ECC
	nand->ecc.layout  = &davinci_nand_ecclayout;
#ifdef CONFIG_SYS_NAND_LARGEPAGE
	nand->ecc.size = 2048;
	nand->ecc.bytes = 12;
#elif defined(CONFIG_SYS_NAND_SMALLPAGE)
	nand->ecc.size = 512;
	nand->ecc.bytes = 3;
#else
#error "Either CONFIG_SYS_NAND_LARGEPAGE or CONFIG_SYS_NAND_SMALLPAGE must be defined!"
#endif
#else
	nand->ecc.size = 512;
	nand->ecc.bytes = 3;
#endif /* CONFIG_SYS_DAVINCI_BROKEN_ECC */
	nand->ecc.calculate = nand_davinci_calculate_ecc;
	nand->ecc.correct  = nand_davinci_correct_data;
	nand->ecc.hwctl  = nand_davinci_enable_hwecc;
#else
	nand->ecc.mode = NAND_ECC_SOFT;
#endif /* CONFIG_SYS_NAND_HW_ECC */

	/* Set address of hardware control function */
	nand->cmd_ctrl = nand_davinci_hwcontrol;

	nand->dev_ready = nand_davinci_dev_ready;
	nand->waitfunc = nand_davinci_waitfunc;

	nand_flash_init();

	return(0);
}

#else
#error "U-Boot legacy NAND support not available for DaVinci chips"
#endif
#endif	/* CONFIG_SYS_USE_NAND */
