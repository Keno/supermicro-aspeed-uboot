#ifndef _GPIO_H_
#define _GPIO_H_

#define GPIO_PORT_ABCD				0
#define GPIO_PORT_EFGH				1
#define GPIO_PORT_IJKL				2
#define GPIO_PORT_MNOP				3
#define GPIO_PORT_QRST				4
#define GPIO_PORT_UVWX				5
#define GPIO_PORT_Y___				6
#define GPIOABCD_INT_STS			0x18
#define GPIOEFGH_INT_STS			0x38
#define GPIOIJKL_INT_STS			0xA8
#define GPIOMNOP_INT_STS			0xF8
#define GPIOQRST_INT_STS			0x128
#define GPIOUVWX_INT_STS			0x158
#define GPIOY____INT_STS			0x188

#define SCUBA  0x1E6E2000
#define GPIOBA 0x1E780000
#define REG(x, shift_num)	((*(volatile unsigned long int *)(x + shift_num)))
#define REG_SHIFT(port)		((port % 4) * 8)
#endif