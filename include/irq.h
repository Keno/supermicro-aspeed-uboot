#ifndef _IRQ_H_
#define _IRQ_H_

#include <config.h>

#ifdef _IRQ_C_

#define NR_IRQS                         (MAXIRQNUM + 1)

/* Interrupt numbers and bit positions */
/* on VIC1 */
#define INT_ECC				0
#define INTMASK_ECC			(1 << INT_ECC)

#define INT_MIC				1
#define INTMASK_MIC			(1 << INT_MIC)

#define INT_MAC1           		2
#define INTMASK_MAC1       		(1 << INT_MAC1)

#define INT_MAC2           		3
#define INTMASK_MAC2       		(1 << INT_MAC2)

#define INT_HAC           		4
#define INTMASK_HAC       		(1 << INT_HAC)

#define INT_USB20           		5
#define INTMASK_USB20       		(1 << INT_USB20)

#define INT_MDMA           		6
#define INTMASK_MDMA       		(1 << INT_MDMA)

#define INT_VIDEO           		7
#define INTMASK_VIDEO       		(1 << INT_VIDEO)

#define INT_LPC				8
#define INTMASK_LPC			(1 << INT_LPC)

#define INT_UARTINT0            	9
#define INTMASK_UARTINT0        	(1 << INT_UARTINT0)

#define INT_UARTINT1            	10
#define INTMASK_UARTINT1        	(1 << INT_UARTINT1)

#define INT_I2S           	        11
#define INTMASK_I2S       		(1 << INT_I2S)

#define INT_I2C				12
#define INTMASK_I2C			(1 << INT_I2C)

#define INT_USB11           		13
#define INTMASK_USB11       		(1 << INT_USB11)

#define INT_ADPCM           		14
#define INTMASK_ADPDM       		(1 << INT_ADPCM)

#if 0
#define INT_INTA           		15
#define INTMASK_INTA       		(1 << INT_INTA)
#endif

#define INT_TIMERINT0           	16
#define INTMASK_TIMERINT0       	(1 << INT_TIMERINT0)

#define INT_TIMERINT1           	17
#define INTMASK_TIMERINT1       	(1 << INT_TIMERINT1)

#define INT_TIMERINT2           	18
#define INTMASK_TIMERINT2       	(1 << INT_TIMERINT2)

#define INT_FLASH		      	19
#define INTMASK_FLASH       		(1 << INT_FLASH)

#define INT_GPIO           		20
#define INTMASK_GPIO       		(1 << INT_GPIO)

#define INT_SCU          		21
#define INTMASK_SCU      		(1 << INT_SCU)

#define INT_RTC_SEC			22
#define INTMASK_RTC_SEC			(1 << INT_RTC_SEC)

#define INT_RTC_DAY			23
#define INTMASK_RTC_DAY			(1 << INT_RTC_DAY)

#define INT_RTC_HOUR			24
#define INTMASK_RTC_HOUR		(1 << INT_RTC_HOUR)

#define INT_RTC_MIN			25
#define INTMASK_RTC_MIN			(1 << INT_RTC_MIN)

#define INT_RTC_ALARM			26
#define INTMASK_RTC_ALARM		(1 << INT_RTC_ALARM)

#define INT_WDT           		27
#define INTMASK_WDT        		(1 << INT_WDT)

#define INT_TACHO           		28
#define INTMASK_TACHO       		(1 << INT_TACHO)

#define INT_2D           		29
#define INTMASK_2D       		(1 << INT_2D)

#define INT_PCI           		30
#define INTMASK_PCI       		(1 << INT_PCI)

#define INT_AHBC			31
#define INTMASK_AHBC			(1 << INT_AHBC)

/* Mask of valid system controller interrupts */
/* VIC means VIC1 */
#define ASPEED_VIC_VALID_INTMASK	( \
					 INTMASK_ECC | \
                                         INTMASK_MIC | \
                                         (INTMASK_MAC1 | INTMASK_MAC2) | \
                                         INTMASK_HAC | \
                                         INTMASK_USB20 | \
                                         INTMASK_MDMA | \
                                         INTMASK_VIDEO | \
                                         INTMASK_LPC | \
                                         (INTMASK_UARTINT0 | INTMASK_UARTINT1) | \
                                         INTMASK_I2S | \
                                         INTMASK_I2C | \
                                         INTMASK_USB11 | \
                                         INTMASK_ADPDM | \
                                         (INTMASK_TIMERINT0 | INTMASK_TIMERINT1 | INTMASK_TIMERINT2) | \
                                         INTMASK_FLASH | \
                                         INTMASK_GPIO | \
                                         INTMASK_SCU | \
					 (INTMASK_RTC_SEC | INTMASK_RTC_DAY | INTMASK_RTC_HOUR | INTMASK_RTC_MIN | INTMASK_RTC_ALARM) | \
                                         INTMASK_WDT | \
                                         INTMASK_TACHO | \
                                         INTMASK_2D | \
                                         INTMASK_PCI | \
                                         INTMASK_AHBC \
					)


/* Interrupt Controllers */
#define ASPEED_VIC_STATUS_OFFSET	0x00
#define ASPEED_VIC_RAW_STATUS_OFFSET    0x08
#define ASPEED_VIC_INT_SEL_OFFSET	0x0C
#define ASPEED_VIC_ENABLE_SET_OFFSET    0x10
#define ASPEED_VIC_ENABLE_CLEAR_OFFSET  0x14
#define ASPEED_VIC_SENSE_OFFSET         0x24
#define ASPEED_VIC_BOTH_EDGE_OFFSET	0x28
#define ASPEED_VIC_EVENT_OFFSET         0x2C
#define ASPEED_VIC_EDGE_CLEAR_OFFSET    0x38

#define ASPEED_VIC_STATUS               (ASPEED_VIC_BASE+ASPEED_VIC_STATUS_OFFSET)
#define ASPEED_VIC_RAW_STATUS           (ASPEED_VIC_BASE+ASPEED_VIC_RAW_STATUS_OFFSET)
#define ASPEED_VIC_ENABLE_SET           (ASPEED_VIC_BASE+ASPEED_VIC_ENABLE_SET_OFFSET)
#define ASPEED_VIC_ENABLE_CLEAR         (ASPEED_VIC_BASE+ASPEED_VIC_ENABLE_CLEAR_OFFSET)
#define ASPEED_VIC_SENSE                (ASPEED_VIC_BASE+ASPEED_VIC_SENSE_OFFSET)
#define ASPEED_VIC_BOTH_EDGE		(ASPEED_VIC_BASE+ASPEED_VIC_BOTH_EDGE_OFFSET)
#define ASPEED_VIC_EVENT                (ASPEED_VIC_BASE+ASPEED_VIC_EVENT_OFFSET)
#define ASPEED_VIC_EDGE_CLEAR           (ASPEED_VIC_BASE+ASPEED_VIC_EDGE_CLEAR_OFFSET)


#else

extern void aspeed_init_irq(void);
extern void irq_handler(void);
extern void SetISR(unsigned long int vector, void (*handler)());
extern void vic_enable_intr(int irq);
extern void vic_disable_intr(int irq);

#endif  // End of #ifdef _KCS_C_  
#endif  // End of #define _KCS_H_