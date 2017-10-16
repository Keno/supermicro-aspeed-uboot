/*
 * Board initialize for ASPEED AST2100
 *
 *
 */

#include <common.h>
#include <command.h>
#include <pci.h>
#include "gpio.h"
#include "pwm_fan.h"

int board_init (void)
{
    DECLARE_GLOBAL_DATA_PTR;
    unsigned char data;
    unsigned long reg;

    /* AHB Controller */
    *((volatile ulong*) 0x1E600000)  = 0xAEED1A03;	/* unlock AHB controller */ 
    *((volatile ulong*) 0x1E60008C) &= ~(0x01);		/* map DRAM to 0x00000000 */
    
    //move the relocate action to "do_bootm" in "/common/cmd_bootm.c"
    //*((volatile ulong*) 0x1E60008C) |= 0x01;		/* map DRAM to 0x00000000 */    

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
//Tony enable uart 1 & uart 2 {
void enableUart(void)
{
	printf("COM:   port1 and port2\n");
	unsigned long int	dw_Hicra_Data_ADDR =(u32)(0x1E6E2000); //unlock 
	*((unsigned long int *) (dw_Hicra_Data_ADDR + 0)) = (0x1688a8a8);
	dw_Hicra_Data_ADDR =(u32)(0x1E6E2084); 
	
	*((unsigned long int *) (dw_Hicra_Data_ADDR + 0)) &= (0x0000ffff);
	*((unsigned long int *)(dw_Hicra_Data_ADDR	+ 0)) |= (0xffff0000);
}
//Tony enable uart 1 & uart 2 }

//Tony pwm init {
static inline u32 aspeed_scu_read(u32 reg)
{
	u32 val;
	val = REG(SCUBA, reg);
	return val;
}
static inline void aspeed_scu_write(u32 val, u32 reg) 
{
	//unlock 
	REG(SCUBA, 0x0) = SCU_PROTECT_UNLOCK;
	REG(SCUBA, reg) = val;
}

static inline void ast_pwm_tacho_write(u32 val, u32 reg)
{
	REG(ASPEED_PWM_BASE, reg) = val;
}

static inline u32 ast_pwm_tacho_read(u32 reg)
{
	u32 val = REG(ASPEED_PWM_BASE, reg);
	return val;
}


static void aspeed_scu_multi_func_pwm_tacho(void)
{
	//TODO check
	u32 sts = aspeed_scu_read(AST_SCU_FUN_PIN_CTRL3) &~0xcfffff;
	//enable PWM0~PWM7
	//According toe X10_AST2400_GPIOs_rev112_09032013.xls, pwm0~pwm3, pw7 are used
	printf("Enable PWM0~3, PWM7\n");
	aspeed_scu_write(sts | 0xc0008f, AST_SCU_FUN_PIN_CTRL3);
}	
static void aspeed_scu_init_pwm_tacho(void)
{
	aspeed_scu_write(aspeed_scu_read(AST_SCU_RESET) | SCU_RESET_PWM, AST_SCU_RESET);
	aspeed_scu_write(aspeed_scu_read(AST_SCU_RESET) & ~SCU_RESET_PWM, AST_SCU_RESET);
}

static void ast_pwm_taco_init(void)
{
	//Enable PWM TACH CLK **************************************************
	// Set M/N/O out is 25Khz
	//The PWM frequency = 24Mhz / (16 * 6 * (9 + 1)) = 25Khz
	ast_pwm_tacho_write(0x09430943, AST_PTCR_CLK_CTRL);
	ast_pwm_tacho_write(0x0943, AST_PTCR_CLK_EXT_CTRL);

	//FULL SPEED at initialize 100% pwm A~H
	ast_pwm_tacho_write(0x0, AST_PTCR_DUTY0_CTRL);
	ast_pwm_tacho_write(0x0, AST_PTCR_DUTY1_CTRL);
	ast_pwm_tacho_write(0x0, AST_PTCR_DUTY2_CTRL);
	ast_pwm_tacho_write(0x0, AST_PTCR_DUTY3_CTRL);

	//Set TACO M/N/O initial unit 0x1000, falling , divide 4 , Enable
	ast_pwm_tacho_write(0x10000001, AST_PTCR_TYPEM_CTRL0);
	ast_pwm_tacho_write(0x10000001, AST_PTCR_TYPEN_CTRL0);
	ast_pwm_tacho_write(0x10000001, AST_PTCR_TYPEO_CTRL0);

	// TACO measure period = 24000000 / 2 / 2 / 256 / 4096 / 1 (only enable 1 TACHO) = 5.72Hz, it means that software needs to
	//	 wait at least 0.2 sec to get refreshed TACO value. If you will enable more TACO or require faster response, you have to
	//	 control the clock divisor and the period to be smaller

	//Full Range to do measure unit 0x1000
	ast_pwm_tacho_write(0x10000000, AST_PTCR_TYPEM_CTRL1);
	ast_pwm_tacho_write(0x10000000, AST_PTCR_TYPEN_CTRL1);
	ast_pwm_tacho_write(0x10000000, AST_PTCR_TYPEO_CTRL1);

	//TACO Source Selection, PWMA for fan0~15 
	ast_pwm_tacho_write(0x0, AST_PTCR_TACH_SOURCE);
	ast_pwm_tacho_write(0x0, AST_PTCR_TACH_SOURCE_EXT);

	//PWM A~D -> Disable , type M, 
	//Tacho 0~15 Disable
	//CLK source 24Mhz
	ast_pwm_tacho_write(AST_PTCR_CTRL_CLK_EN, AST_PTCR_CTRL);	
}

static void  ast_set_tacho_en(u8 tacho_ch, u8 enable)
{
	//tacho number enable
	return;
	printf("tacho%d_en is %d",tacho_ch,enable);
	if(enable)
		ast_pwm_tacho_write(ast_pwm_tacho_read(AST_PTCR_CTRL) | AST_PTCR_CTRL_FAN_NUM_EN(tacho_ch),AST_PTCR_CTRL);
	else
		ast_pwm_tacho_write(ast_pwm_tacho_read(AST_PTCR_CTRL) & ~(AST_PTCR_CTRL_FAN_NUM_EN(tacho_ch)),AST_PTCR_CTRL);		
}


static void 
ast_set_pwm_en(u8 pwm_ch, u8 enable)
{	
    switch (pwm_ch) {

	case PWMA:
		if(enable)
			ast_pwm_tacho_write(ast_pwm_tacho_read(AST_PTCR_CTRL) | AST_PTCR_CTRL_PMWA_EN,AST_PTCR_CTRL);
		else
			ast_pwm_tacho_write(ast_pwm_tacho_read(AST_PTCR_CTRL) & ~AST_PTCR_CTRL_PMWA_EN,AST_PTCR_CTRL);
		break;
	case PWMB:
		if(enable)		
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL) | AST_PTCR_CTRL_PMWB_EN),AST_PTCR_CTRL);
		else
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL) & ~AST_PTCR_CTRL_PMWB_EN),AST_PTCR_CTRL);			
		break;
	case PWMC:
		if(enable)
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL) | AST_PTCR_CTRL_PMWC_EN),AST_PTCR_CTRL);
		else
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL) & ~AST_PTCR_CTRL_PMWC_EN),
					AST_PTCR_CTRL);	
		break;
	case PWMD:
		if(enable)
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL) | AST_PTCR_CTRL_PMWD_EN),AST_PTCR_CTRL);
		else
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL) & ~AST_PTCR_CTRL_PMWD_EN),AST_PTCR_CTRL);
		break;
	case PWME:
		if(enable)
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL_EXT) | AST_PTCR_CTRL_PMWE_EN),AST_PTCR_CTRL_EXT);
		else
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL_EXT) & ~AST_PTCR_CTRL_PMWE_EN),AST_PTCR_CTRL_EXT);
		break;
	case PWMF:
		if(enable)
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL_EXT) | AST_PTCR_CTRL_PMWF_EN),AST_PTCR_CTRL_EXT);
		else
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL_EXT) & ~AST_PTCR_CTRL_PMWF_EN),AST_PTCR_CTRL_EXT);
		break;
	case PWMG:
		if(enable)
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL_EXT) | AST_PTCR_CTRL_PMWG_EN),AST_PTCR_CTRL_EXT);
		else
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL_EXT) & ~AST_PTCR_CTRL_PMWG_EN),AST_PTCR_CTRL_EXT);
		break;
	case PWMH:
		if(enable)
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL_EXT) | AST_PTCR_CTRL_PMWH_EN),AST_PTCR_CTRL_EXT);
		else
			ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CTRL_EXT) & ~AST_PTCR_CTRL_PMWH_EN),AST_PTCR_CTRL_EXT);
		break;
	default:
		printf("error channel ast_get_pwm_type %d \n",pwm_ch);
		break;
    }
}

static u8 ast_get_pwm_type(u8 pwm_ch)
{
    u8 tmp=0;

    switch (pwm_ch) {
	case PWMA:
		tmp = AST_PTCR_CTRL_GET_PWMA_TYPE(ast_pwm_tacho_read(AST_PTCR_CTRL));
		break;
	case PWMB:
		tmp = AST_PTCR_CTRL_GET_PWMB_TYPE(ast_pwm_tacho_read(AST_PTCR_CTRL));
		break;
	case PWMC:
		tmp = AST_PTCR_CTRL_GET_PWMC_TYPE(ast_pwm_tacho_read(AST_PTCR_CTRL));
		break;
	case PWMD:
		tmp = AST_PTCR_CTRL_GET_PWMD_TYPE(ast_pwm_tacho_read(AST_PTCR_CTRL));
		break;
	case PWME:
		tmp = AST_PTCR_CTRL_GET_PWME_TYPE(ast_pwm_tacho_read(AST_PTCR_CTRL_EXT));
		break;
	case PWMF:
		tmp = AST_PTCR_CTRL_GET_PWMF_TYPE(ast_pwm_tacho_read(AST_PTCR_CTRL_EXT));
		break;
	case PWMG:
		tmp = AST_PTCR_CTRL_GET_PWMG_TYPE(ast_pwm_tacho_read(AST_PTCR_CTRL_EXT));
		break;
	case PWMH:
		tmp = AST_PTCR_CTRL_GET_PWMH_TYPE(ast_pwm_tacho_read(AST_PTCR_CTRL_EXT));
		break;
	default:
		printf("error channel ast_get_pwm_type %d \n",pwm_ch);
		break;
    }

	return tmp;
}

static void ast_set_pwm_type(u8 pwm_ch, u8 type)
{
    u32 tmp1,tmp2;

	if(type > 0x2)
		return;

	tmp1 = ast_pwm_tacho_read(AST_PTCR_CTRL);
	tmp2 = ast_pwm_tacho_read(AST_PTCR_CTRL_EXT);

    switch (pwm_ch) {
	case PWMA:
		tmp1 &= ~AST_PTCR_CTRL_SET_PWMA_TYPE_MASK;
		tmp1 |= AST_PTCR_CTRL_SET_PWMA_TYPE(type);
		ast_pwm_tacho_write(tmp1, AST_PTCR_CTRL);
		break;
	case PWMB:
		tmp1 &= ~AST_PTCR_CTRL_SET_PWMB_TYPE_MASK;
		tmp1 |= AST_PTCR_CTRL_SET_PWMB_TYPE(type);
		ast_pwm_tacho_write(tmp1, AST_PTCR_CTRL);
		break;
	case PWMC:
		tmp1 &= ~AST_PTCR_CTRL_SET_PWMC_TYPE_MASK;
		tmp1 |= AST_PTCR_CTRL_SET_PWMC_TYPE(type);
		ast_pwm_tacho_write(tmp1, AST_PTCR_CTRL);
		break;
	case PWMD:
		tmp1 &= ~AST_PTCR_CTRL_SET_PWMD_TYPE_MASK;
		tmp1 |= AST_PTCR_CTRL_SET_PWMD_TYPE(type);
		ast_pwm_tacho_write(tmp1, AST_PTCR_CTRL);
		break;
	case PWME:
		tmp2 &= ~AST_PTCR_CTRL_SET_PWME_TYPE_MASK;
		tmp2 |= AST_PTCR_CTRL_SET_PWME_TYPE(type);
		ast_pwm_tacho_write(tmp2, AST_PTCR_CTRL_EXT);
		break;
	case PWMF:
		tmp2 &= ~AST_PTCR_CTRL_SET_PWMF_TYPE_MASK;
		tmp2 |= AST_PTCR_CTRL_SET_PWMF_TYPE(type);
		ast_pwm_tacho_write(tmp2, AST_PTCR_CTRL_EXT);
		break;
	case PWMG:
		tmp2 &= ~AST_PTCR_CTRL_SET_PWMG_TYPE_MASK;
		tmp2 |= AST_PTCR_CTRL_SET_PWMG_TYPE(type);
		ast_pwm_tacho_write(tmp2, AST_PTCR_CTRL_EXT);
		break;
	case PWMH:
		tmp2 &= ~AST_PTCR_CTRL_SET_PWMH_TYPE_MASK;
		tmp2 |= AST_PTCR_CTRL_SET_PWMH_TYPE(type);
		ast_pwm_tacho_write(tmp2, AST_PTCR_CTRL_EXT);
		break;
	default:
		printf("error channel %d \n",pwm_ch);
		break;
    }
}

static u8 ast_get_pwm_clock_unit(u8 pwm_type)
{
    u8 tmp=0;

    switch (pwm_type) {
	case PWM_TYPE_M:
		tmp = (ast_pwm_tacho_read(AST_PTCR_CLK_CTRL) & AST_PTCR_CLK_CTRL_TYPEM_UNIT_MASK) >> AST_PTCR_CLK_CTRL_TYPEM_UNIT;
		break;
	case PWM_TYPE_N:
		tmp = (ast_pwm_tacho_read(AST_PTCR_CLK_CTRL) & AST_PTCR_CLK_CTRL_TYPEN_UNIT_MASK) >> AST_PTCR_CLK_CTRL_TYPEN_UNIT;
		break;
	case PWM_TYPE_O:
		tmp = (ast_pwm_tacho_read(AST_PTCR_CLK_EXT_CTRL) & AST_PTCR_CLK_CTRL_TYPEO_UNIT_MASK) >> AST_PTCR_CLK_CTRL_TYPEO_UNIT;
		break;
	default:
		printf("error channel ast_get_pwm_clock_unit %d \n",pwm_type);
		break;
    }
	return tmp;
}

static void ast_set_pwm_duty_rising(u8 pwm_ch, u8 rising)
{
	u32 tmp=0;
	u32 pwm_type = ast_get_pwm_type(pwm_ch);
	
	if((rising > 0xff) || (rising > ast_get_pwm_clock_unit(pwm_type)))
		return;
	
    switch (pwm_ch) {
	case PWMA:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY0_CTRL);
		tmp &= ~DUTY_CTRL0_PWMA_RISE_POINT_MASK;
		tmp |= rising;
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY0_CTRL);
		break;
	case PWMB:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY0_CTRL);
		tmp &= ~DUTY_CTRL0_PWMB_RISE_POINT_MASK;
		tmp |= (rising << DUTY_CTRL0_PWMB_RISE_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY0_CTRL);
		break;
	case PWMC:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY1_CTRL);
		tmp &= ~DUTY_CTRL1_PWMC_RISE_POINT_MASK;
		tmp |= rising;
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY1_CTRL);
		break;
	case PWMD:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY1_CTRL);
		tmp &= ~DUTY_CTRL1_PWMD_RISE_POINT_MASK;
		tmp |= (rising << DUTY_CTRL1_PWMD_RISE_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY1_CTRL);
		break;
	case PWME:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY2_CTRL);
		tmp &= ~DUTY_CTRL2_PWME_RISE_POINT_MASK;
		tmp |= rising;
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY2_CTRL);
		break;
	case PWMF:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY2_CTRL);
		tmp &= ~DUTY_CTRL2_PWMF_RISE_POINT_MASK;
		tmp |= (rising << DUTY_CTRL2_PWMF_RISE_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY2_CTRL);
		break;
	case PWMG:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY3_CTRL);
		tmp &= ~DUTY_CTRL3_PWMG_RISE_POINT_MASK;
		tmp |= rising;
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY3_CTRL);
		break;
	case PWMH:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY3_CTRL);
		tmp &= ~DUTY_CTRL3_PWMH_RISE_POINT_MASK;
		tmp |= (rising << DUTY_CTRL3_PWMH_RISE_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY3_CTRL);
		break;			

	default:
	    printf("error pwm channel %d with duty \n",pwm_ch);
		break;
    }
}

static void
ast_set_pwm_duty_falling(u8 pwm_ch, u8 falling)
{
	u32 tmp =0;
	u32 pwm_type = ast_get_pwm_type(pwm_ch);
	
	if((falling > 0xff) || (falling > ast_get_pwm_clock_unit(pwm_type)))
		return;
	
    switch (pwm_ch) {
	case PWMA:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY0_CTRL);
		tmp &= ~DUTY_CTRL0_PWMA_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL0_PWMA_FALL_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY0_CTRL);
		break;
	case PWMB:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY0_CTRL);
		tmp &= ~DUTY_CTRL0_PWMB_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL0_PWMB_FALL_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY0_CTRL);
		break;
	case PWMC:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY1_CTRL);
		tmp &= ~DUTY_CTRL1_PWMC_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL1_PWMC_FALL_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY1_CTRL);
		break;
	case PWMD:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY1_CTRL);
		tmp &= ~DUTY_CTRL1_PWMD_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL1_PWMD_FALL_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY1_CTRL);
		break;
	case PWME:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY2_CTRL);
		tmp &= ~DUTY_CTRL2_PWME_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL2_PWME_FALL_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY2_CTRL);
		break;
	case PWMF:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY2_CTRL);
		tmp &= ~DUTY_CTRL2_PWMF_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL2_PWMF_FALL_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY2_CTRL);
		break;
	case PWMG:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY3_CTRL);
		tmp &= ~DUTY_CTRL3_PWMG_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL3_PWMG_FALL_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY3_CTRL);
		break;
	case PWMH:
		tmp = ast_pwm_tacho_read(AST_PTCR_DUTY3_CTRL);
		tmp &= ~DUTY_CTRL3_PWMH_FALL_POINT_MASK;
		tmp |= (falling << DUTY_CTRL3_PWMH_FALL_POINT);
		ast_pwm_tacho_write(tmp, AST_PTCR_DUTY3_CTRL);
		break;			

	default:
	    printf("error pwm channel %d with duty \n",pwm_ch);
		break;
    }

}


static void configure_pwm_dutycycle(int pwm_no,int value,int flag)
{
	if (flag == 0)
		ast_set_pwm_duty_falling(pwm_no,value);
	else
		ast_set_pwm_duty_rising(pwm_no,value);
}

static void ast_set_pwm_clock_unit(u8 pwm_type, u8 unit)
{
	if(unit > 0xff)
		return;
	switch (pwm_type) {
	case PWM_TYPE_M:
	 ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEM_UNIT_MASK) | (unit << AST_PTCR_CLK_CTRL_TYPEM_UNIT),
	 	AST_PTCR_CLK_CTRL);
	 break;
	case PWM_TYPE_N:
	ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEN_UNIT_MASK) | (unit << AST_PTCR_CLK_CTRL_TYPEN_UNIT),
	 	AST_PTCR_CLK_CTRL);
	 break;
	case PWM_TYPE_O:
	ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_EXT_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEO_UNIT_MASK) | (unit << AST_PTCR_CLK_CTRL_TYPEO_UNIT),
	 	AST_PTCR_CLK_EXT_CTRL);
	 break;
	default:
	 printf("error channel ast_get_pwm_type %d \n",pwm_type);
	 break;
	}
}



static void congiure_pwm_unit(int type,int val)
{
	ast_set_pwm_clock_unit(type,val);
}

static void 
ast_set_pwm_clock_division_h(unsigned char pwm_type, unsigned char div_high)
{
	if(div_high > 0xf)
		return;
	switch (pwm_type) {
	case PWM_TYPE_M:
	 ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEM_H_MASK) | (div_high << AST_PTCR_CLK_CTRL_TYPEM_H),
	 	AST_PTCR_CLK_CTRL);
	 break;
	case PWM_TYPE_N:
	ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEN_H_MASK) | (div_high << AST_PTCR_CLK_CTRL_TYPEN_H),
	 	AST_PTCR_CLK_CTRL);
	 break;
	case PWM_TYPE_O:
	ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_EXT_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEO_H_MASK) | (div_high << AST_PTCR_CLK_CTRL_TYPEO_H),
	 	AST_PTCR_CLK_EXT_CTRL);
	 break;
	default:
	 printf("error channel ast_get_pwm_type %d \n",pwm_type);
	 break;
	}

}


ast_set_pwm_clock_division_l(unsigned char  pwm_type, unsigned char div_low)
{
	if(div_low> 0xf)
		return;
	switch (pwm_type) {
	case PWM_TYPE_M:
	 ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEM_L_MASK) | (div_low << AST_PTCR_CLK_CTRL_TYPEM_L),
	 	AST_PTCR_CLK_CTRL);
	 break;
	case PWM_TYPE_N:
	ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEN_L_MASK) | (div_low << AST_PTCR_CLK_CTRL_TYPEN_L),
	 	AST_PTCR_CLK_CTRL);
	 break;
	case PWM_TYPE_O:
	ast_pwm_tacho_write((ast_pwm_tacho_read(AST_PTCR_CLK_EXT_CTRL) & ~AST_PTCR_CLK_CTRL_TYPEO_L_MASK) | (div_low << AST_PTCR_CLK_CTRL_TYPEO_L),
	 	AST_PTCR_CLK_EXT_CTRL);
	 break;
	default:
	 printf("error channel ast_get_pwm_type %d \n",pwm_type);
	 break;
	}
}

static void SetupASTFANDrv(void)
{
   int	 i,num;
   int   pwm_no,type;


	for(i=0;i<5;++i)
		ast_set_tacho_en(i,1);

	for(i=0;i<2;++i)
	{
		pwm_no = i;
		type = 0;
		ast_set_pwm_en(pwm_no,1);
		configure_pwm_dutycycle(i,100,0);
		ast_set_pwm_type(pwm_no,type);
		ast_set_pwm_clock_unit(type,100);
		ast_set_pwm_clock_division_h(type,0);
		ast_set_pwm_clock_division_l(type,5);
		configure_pwm_dutycycle(i,50,1);
	}
}

void PWMInit(void)
{
	printf("PWM:   portA and portB\n");
	aspeed_scu_multi_func_pwm_tacho();
	aspeed_scu_init_pwm_tacho();
	ast_pwm_taco_init();
	SetupASTFANDrv();
}
//Tony pwm init }




void gpio_select_multi_pin (unsigned int port_no,unsigned int pin_no){

	u32 		  ctrl_pattern = 0;
	REG(SCUBA, 0x00) = 0x1688a8a8;	// Unlock SCU registers 
	
	switch(port_no)
	{
		case 0://port A
		{	
			ctrl_pattern = 0x1<< pin_no;
			REG(SCUBA, 0x80) &= ~(ctrl_pattern);

			switch(pin_no)
			{
				case 4:
				case 5:
					ctrl_pattern = 0x1<< 22;
					REG(SCUBA, 0x90) &= ~(ctrl_pattern);
					break;
				case 6:
				case 7:
					ctrl_pattern = 0x1<< 2;
					REG(SCUBA, 0x90) &= ~(ctrl_pattern);
					break;
				default:
					break;	
			}		
			break;
		}

		case 1://port B
		{	
			ctrl_pattern = 0x1<< (pin_no+8);
			REG(SCUBA, 0x80) &= ~(ctrl_pattern);
			break;
		}

		case 2://port C
		{	
			ctrl_pattern = 0x1<< 0;
			REG(SCUBA, 0x90) &= ~(ctrl_pattern);
			ctrl_pattern = 1<< ( (pin_no / 2) +23);
			REG(SCUBA, 0x90) &= ~(ctrl_pattern);
			break;
		}		

		case 3://port D
		{	
			ctrl_pattern = 0x1<< 0;
			REG(SCUBA, 0x90) &= ~(ctrl_pattern);
			ctrl_pattern = 0x1<< ( (pin_no / 2) +8);
			REG(SCUBA, 0x8C) &= ~(ctrl_pattern);
			break;
		}		

		case 4://port E
		{	
			ctrl_pattern = 0x1<< (pin_no + 16);
    					
    		REG(SCUBA, 0x80) &= ~(0x00ff0000);	// Disable UART3
    		REG(SCUBA, 0x8C) &= ~(0x0000f800);	// Disable by pass
   			REG(SCUBA, 0x70) &= ~(0x00600000);	// HW Trap
    		REG(SCUBA, 0x90) &= ~(0x0FC00003);	// Enable C0			
    					
    					
			REG(SCUBA, 0x80) &= ~(ctrl_pattern);
			ctrl_pattern = 0x1<< ( (pin_no / 2) +12);
			REG(SCUBA, 0x8C) &= ~(ctrl_pattern);
			break;
		}		

		case 5://port F
		{	
			ctrl_pattern = 0x1<< (pin_no + 24);
			REG(SCUBA, 0x80) &= ~(ctrl_pattern);
			switch(pin_no)
			{
				case 1:
				case 2:
				case 3:
					ctrl_pattern = 0x1<< ((pin_no-1)+12);
					REG(SCUBA, 0xA4) &= ~(ctrl_pattern);
					break;
				case 4:
				case 5:
				case 6:
					ctrl_pattern = 0x1<< 15;
					REG(SCUBA, 0xA4) &= ~(ctrl_pattern);
					break;
				default:
					break;	
			}		
			break;
		}		

		case 6://port G
		{	
			ctrl_pattern = 0x1<< pin_no;
			REG(SCUBA, 0x84) &= ~(ctrl_pattern);

			switch(pin_no)
			{
				case 4:
					ctrl_pattern = 0x1<< 1;
					REG(SCUBA, 0x2C) &= ~(ctrl_pattern);
					break;
				default:
					break;	
			}		
			break;
		}		

		case 7://port H
		{	
			ctrl_pattern = 0x11 << 6;
			REG(SCUBA, 0x90) &= ~(ctrl_pattern);

			break;
		}		

		case 9://port J
		{	
			ctrl_pattern = 0x1 << (pin_no + 8);
			REG(SCUBA, 0x84) &= ~(ctrl_pattern);

			break;
		}		

		case 10://port K
		{	
			ctrl_pattern = 1<< ( (pin_no / 2) +18);
			REG(SCUBA, 0x90) &= ~(ctrl_pattern);
			break;
		}		

		case 11://port L
		{	
			ctrl_pattern = 0x1 << (pin_no + 16);
			REG(SCUBA, 0x84) &= ~(ctrl_pattern);
			break;
		}		

		case 12://port M
		{	
			ctrl_pattern = 0x1 << (pin_no + 24);
			REG(SCUBA, 0x84) &= ~(ctrl_pattern);
			break;
		}		

		case 13://port N
		{	
			ctrl_pattern = 0x1 << (pin_no + 0);
			REG(SCUBA, 0x88) &= ~(ctrl_pattern);
			break;
		}		

		case 14://port O
		{	
			ctrl_pattern = 0x1 << (pin_no + 8);
			REG(SCUBA, 0x88) &= ~(ctrl_pattern);
			break;
		}		

		case 15://port P
		{	
			switch(pin_no)
			{
				case 0:
				case 1:
				case 2:
				case 3:
				case 6:
				case 7:
					ctrl_pattern = 0x1 << (pin_no + 16);
					REG(SCUBA, 0x88) &= ~(ctrl_pattern);
					break;
				default:
					break;	
			}		
			break;
		}		

		case 16://port Q
		{	
			switch(pin_no)
			{
				case 0:
				case 1:
				case 2:
				case 3:
					ctrl_pattern = 0x1 << ((pin_no / 2) + 16);
					REG(SCUBA, 0x90) &= ~(ctrl_pattern);
					break;
				case 4:
				case 5:
				case 6:
				case 7:
					ctrl_pattern = 0x1 << (((pin_no-4) / 2) + 27);
					REG(SCUBA, 0x90) &= ~(ctrl_pattern);
					break;
				default:
					break;	
			}		
			break;
		}		

		case 17://port R
		{	
			ctrl_pattern = 0x1 << (pin_no + 24);
			REG(SCUBA, 0x88) &= ~(ctrl_pattern);
			break;
		}		

		case 18://port S
		{	
			ctrl_pattern = 0x1 << pin_no;
			REG(SCUBA, 0x8C) &= ~(ctrl_pattern);
			break;
		}		

		case 19://port T
		{	
			ctrl_pattern = 0x1 << pin_no;
			REG(SCUBA, 0xA0) &= ~(ctrl_pattern);
			break;
		}		

		case 20://port U
		{	
			ctrl_pattern = 0x1 << (pin_no+8);
			REG(SCUBA, 0xA0) &= ~(ctrl_pattern);
			break;
		}		

		case 21://port V
		{	
			ctrl_pattern = 0x1 << (pin_no+16);
			REG(SCUBA, 0xA0) &= ~(ctrl_pattern);
			break;
		}		

		case 22://port W
		{	
			ctrl_pattern = 0x1 << (pin_no+24);
			REG(SCUBA, 0xA0) &= ~(ctrl_pattern);
			break;
		}		

		case 23://port X
		{	
			ctrl_pattern = 0x1 << (pin_no+0);
			REG(SCUBA, 0xA4) &= ~(ctrl_pattern);
			break;
		}		

		case 24://port Y
		{	
			ctrl_pattern = 0x1 << (pin_no+8);
			REG(SCUBA, 0xA4) &= ~(ctrl_pattern);
			break;
		}		

		case 25://port Z
		{	
			ctrl_pattern = 0x1 << (pin_no+16);
			REG(SCUBA, 0xA4) &= ~(ctrl_pattern);
			break;
		}		

		case 26://port AA
		{	
			ctrl_pattern = 0x1 << (pin_no+24);
			REG(SCUBA, 0xA4) &= ~(ctrl_pattern);
			break;
		}		

		case 27://port AB
		{	
			ctrl_pattern = 0x1 << (pin_no+0);
			REG(SCUBA, 0xA8) &= ~(ctrl_pattern);
			break;
		}		


		default:
			break;
	}
}

int gpio_ioctl(unsigned char cmd,unsigned long HWInfo,unsigned char *value ){

	unsigned char init_value, init_dir,tolerant_num;
	unsigned char data2 = 0;
	unsigned long gpiodir, dir_shift_num, data_shift_num, int_shift_num, ctrl_pattern = 0x0,tolerant;
	unsigned int  pin_no = 0x0, port_no = 0x0;

	tolerant	= (HWInfo  & 0x00020000) >> 17; 
    init_value  = (HWInfo  & 0x00040000) >> 18;
    init_dir    = (HWInfo  & 0x00080000) >> 19;
    pin_no      = (HWInfo  & 0x00F00000) >> 20;
    port_no     = (HWInfo  & 0xFF000000) >> 24; //port_no LSB
    						 	
	printf("GPIO:  port:%d, pin:%d, dir:%d val:%d tolerant %d\n",port_no ,pin_no ,init_dir ,init_value,tolerant);
	switch(port_no/4)
	{
		case GPIO_PORT_ABCD:
			dir_shift_num  = 0x04;
			data_shift_num = 0x00;
			tolerant_num   = 0x1C;  	
			break;
		case GPIO_PORT_EFGH: 
			dir_shift_num  = 0x24;
			data_shift_num = 0x20;
			tolerant_num   = 0x3C; 	
			break;
		case GPIO_PORT_IJKL:
			dir_shift_num  = 0x74;
			data_shift_num = 0x70;
			tolerant_num   = 0xAC; 	
			break;
		case GPIO_PORT_MNOP:
			dir_shift_num = 0x7C;
			data_shift_num = 0x78;
			tolerant_num   = 0xFC; 	
			break;
		case GPIO_PORT_QRST:
			dir_shift_num = 0x84;
			data_shift_num = 0x80;
			tolerant_num   = 0x12C; 	
			break;
		case GPIO_PORT_UVWX:
			dir_shift_num = 0x8C;
			data_shift_num = 0x88;
			tolerant_num   = 0x15C; 	
			break;
		case GPIO_PORT_Y___:
			dir_shift_num = 0x1E4;
			data_shift_num = 0x1E0;
			tolerant_num   = 0x18C; 	
			break;
		default:
			dir_shift_num = 0xC4;
			data_shift_num = 0xC0;
			printf( "gpio: port_no %x out of range \n",port_no);
			break;
	}

	gpiodir = REG(GPIOBA, dir_shift_num) >> REG_SHIFT(port_no);
    /* Translate an integer into an 16-bit pattern, ex: 3 -> 0000 0000 0000 0100 */
	ctrl_pattern = 1 << pin_no;

	
	switch (cmd)
	{
		case 0:
		{
			data2 = REG(GPIOBA, data_shift_num) >> REG_SHIFT(port_no);
			//printf( " Read port%d pin%d value is %x. \n",port_no ,pin_no ,data2);
			*value = (data2 & ctrl_pattern) >> pin_no;
			//printf( " data_buf[0] is %x. \n",HW_info.data_buf[0]);
					
			//For debug the registers
			if(((HWInfo & 0x1))==0x1)
			{
				printf( "-%lx,%lx-\n",REG(GPIOBA, dir_shift_num) ,REG(GPIOBA, data_shift_num));
			}
			//printf( " Read port%d pin%d value is %x. \n",port_no ,pin_no ,data2);


			break;
		}
		case 1:
		{
    		gpio_select_multi_pin(port_no,pin_no);

			/* change direction */
			if(init_dir==1)  
				gpiodir |= ctrl_pattern;
			else  
				gpiodir &= ~ctrl_pattern;
			REG(GPIOBA, dir_shift_num) &= (~(0x000000FF << (8 * (port_no % 4))));
			REG(GPIOBA, dir_shift_num) |= (unsigned int)(gpiodir << (8 * (port_no % 4)));

			REG(GPIOBA, tolerant_num) |= (ctrl_pattern << (8 * (port_no % 4)));
			/* change data */
			//only direction = output, then can set init value
			if (init_dir == 1)
			{
				if(init_value == 1)
				{
					REG(GPIOBA, data_shift_num) |= (ctrl_pattern << (8 * (port_no % 4)));
				}
				else
				{
					REG(GPIOBA, data_shift_num) &= ~(ctrl_pattern << (8 * (port_no % 4)));
				}
			}
			break;
		}
		default :
			break;
		
	}
	return 0;
}
void GPIOSetup(unsigned long HWInfo){
	unsigned char val;
	gpio_ioctl(1, HWInfo,&val);
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
    else 
    {
    	printf("AST2400 series chip\n");
	}
#ifdef	CONFIG_PCI	
    pci_init ();
#endif
	enableUart();
	PWMInit();
	GPIOSetup(0x045C0000);
  if (getenv ("verify") == NULL) {
    setenv ("verify", "n");
  }
  if (getenv ("eeprom") == NULL) { 
	  setenv ("eeprom", "y");
  }

	return 0;
}

#ifdef	CONFIG_PCI
static struct pci_controller hose;

extern void aspeed_init_pci (struct pci_controller *hose);

void pci_init_board(void)
{
    aspeed_init_pci(&hose);
}
#endif




