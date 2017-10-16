#define _IRQ_C_

#include <common.h>
#include <irq.h>

#ifdef CONFIG_INT_ENABLED

#define MAXIRQNUM                       43
// Interrupt Handler Table
typedef void (*fptr)();  // function pointer
fptr IRQ_HandlerTable[NR_IRQS];
#define ASPEED_VIC_BASE         0x1E6C0000
#define VIC_BASE_VA             ASPEED_VIC_BASE

#define UMVP_READ_REG(r)        (*((volatile unsigned int *) (r)))
#define UMVP_WRITE_REG(r,v)     (*((volatile unsigned int *) (r)) = ((unsigned int)   (v)))

#define DEFAULT_VIC_EDGE	0
#define DEFAULT_VIC_LEVEL	1
#define DEFAULT_VIC_LOW		0
#define DEFAULT_VIC_HIGH	1

struct  _DEFAULT_VIC_TABLE {
    unsigned int	vic_type;
    unsigned int	vic_trigger;	
};
	
struct _DEFAULT_VIC_TABLE default_vic[] = {
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT31: AHBC      */	    	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT30: PCI       */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT29: 2D        */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT28: TACHO     */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT27: WDT ALARM */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT26: RTC ALARM */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT25: RTC MIN   */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT24: RTC HOUR  */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT23: RTC DAY   */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT22: RTC SEC   */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT21: SCU       */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT20: GPIO      */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT19: SMC       */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT18: TIMER3    */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT17: TIMER2    */	
    { DEFAULT_VIC_EDGE,  DEFAULT_VIC_HIGH },		/* INT16: TIMER1    */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT15: PECI      */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT14: TBD       */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT13: USB11     */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT12: I2C       */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT11: I2S       */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT10: UART2     */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 9: UART1     */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 8: LPC       */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 7: VIDEO     */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 6: MDMA      */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 5: USB2      */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 4: CRYPTO    */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 3: MAC2      */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 2: MAC1      */	
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 1: MIC       */		
    { DEFAULT_VIC_LEVEL, DEFAULT_VIC_HIGH },		/* INT 0: SDRAM     */
};

static void vic_reset(void)
{
    unsigned int i, vic_type=0, vic_trigger=0;
    	
    UMVP_WRITE_REG(VIC_BASE_VA + ASPEED_VIC_INT_SEL_OFFSET,      0x00000000);
    UMVP_WRITE_REG(VIC_BASE_VA + ASPEED_VIC_ENABLE_SET_OFFSET,   0x00000000);
    UMVP_WRITE_REG(VIC_BASE_VA + ASPEED_VIC_ENABLE_CLEAR_OFFSET, 0xFFFFFFFF);
    UMVP_WRITE_REG(VIC_BASE_VA + ASPEED_VIC_EDGE_CLEAR_OFFSET,   0xFFFFFFFF);

    /* set default VIC settings */
    for (i=0; i<NR_IRQS; i++)
    {
    	vic_type   <<= 1;
    	vic_trigger<<= 1;
        vic_type    |= default_vic[i].vic_type;
        vic_trigger |= default_vic[i].vic_trigger;       
        
        IRQ_HandlerTable[i] = 0;
    }
    UMVP_WRITE_REG(VIC_BASE_VA + ASPEED_VIC_SENSE_OFFSET, vic_type);
    UMVP_WRITE_REG(VIC_BASE_VA + ASPEED_VIC_EVENT_OFFSET, vic_trigger);
    
    
}

void aspeed_init_irq(void)
{
	unsigned int i;

	vic_reset();

//	for (i = 0; i < NR_IRQS; i++) 
//	{
//		/* apply VIC1 mask (VIC means VIC1) */
//		if ((1 << i) & ASPEED_VIC_VALID_INTMASK) 
//		{
//			set_irq_chip(i, &sc_chip);
//			set_irq_handler(i, do_level_IRQ);
//			set_irq_flags(i, IRQF_VALID);
//		}
//	}
}

void irq_handler(void)
{
	volatile unsigned int irq;
	int i;
	
	irq = UMVP_READ_REG(VIC_BASE_VA + ASPEED_VIC_STATUS_OFFSET);

//	printf ("irq_handler: irq = 0x%x !! \n", irq);

	for (i=0; i<NR_IRQS; i++)
	{
		if((irq & (0x1<<i)) && (IRQ_HandlerTable[i] != 0))
			(*IRQ_HandlerTable[i])();
	}
	
  	//if((irq & (0x1<<8)) && (IRQ_HandlerTable[8] != 0))
    //	(*IRQ_HandlerTable[8])();

//	printf ("Leave irq_handler: irq = 0x%x !! \n", UMVP_READ_REG(VIC_BASE_VA + ASPEED_VIC_STATUS_OFFSET));	
}

void SetISR(unsigned long int vector, void (*handler)())
{
 	IRQ_HandlerTable[vector] = handler;
}

void vic_enable_intr(int irq)
{
	register unsigned int regVal;

	if (irq < NR_IRQS)
	{
		regVal = UMVP_READ_REG(VIC_BASE_VA + ASPEED_VIC_ENABLE_SET_OFFSET);
		regVal |= (1 << irq);
		UMVP_WRITE_REG(VIC_BASE_VA + ASPEED_VIC_ENABLE_SET_OFFSET, regVal);
	}
}

void vic_disable_intr(int irq)
{
	register unsigned int regVal;

	if (irq < NR_IRQS)
	{
		regVal = UMVP_READ_REG(VIC_BASE_VA + ASPEED_VIC_ENABLE_CLEAR_OFFSET);
		regVal |= (1 << irq);
		UMVP_WRITE_REG(VIC_BASE_VA + ASPEED_VIC_ENABLE_CLEAR_OFFSET, regVal);
	}
}



#endif
