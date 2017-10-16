/*
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#if defined(CONFIG_CMD_NET) && defined(CONFIG_NET_MULTI) && defined(CONFIG_FARADAYNIC)

#include <malloc.h>
#include <net.h>
#include <pci.h>


/*
SCU88 D[31]: MAC1 MDIO
SCU88 D[30]: MAC1 MDC
SCU90 D[2]:  MAC2 MDC/MDIO
SCU80 D[0]:  MAC1 Link
SCU80 D[1]:  MAC2 Link
*/
#define pci_find_devices         NULL
#define pci_read_config_dword    NULL
#if	defined(CONFIG_AST1300)
#define SCU_BASE                 CONFIG_SCUREG_BASE
#else
#define SCU_BASE                 0x1E6E2000
#endif
#define SCU_RESET_CONTROL        0x04
#define SCU_CLOCK_SELECTION      0x08
#define SCU_CLOCK_CONTROL        0x0C
#define SCU_MAC_CLOCK_DELAY      0x48
#define SCU_SCRATCH_REGISTER     0x40
#define SCU_HARDWARE_TRAPPING    0x70
#define SCU_PIN_MUX              0x74
#define SCU_MULTIFUNCTION_PIN_CTL1_REG     0x80
#define SCU_MULTIFUNCTION_PIN_CTL3_REG     0x88
#define SCU_MULTIFUNCTION_PIN_CTL5_REG     0x90
#define MAC_INTERFACE            0x1C0
#define GMII                     0x0
#define MII                      0x40
#define MAC1_CLOCK_ENABLE        (1 << 20)
#define MAC2_CLOCK_ENABLE        (1 << 21)
#define MAC_AHB_CLOCK_DIVIDER    (0x07 << 16)
#if defined(CONFIG_AST2300_FPGA_2) || defined(CONFIG_AST2300)
#define MAC1_MDIO                (1 << 31)
#define MAC1_MDC                 (1 << 30)
#define MAC1_PHY_LINK            (1 << 0)
#define MAC2_MDC_MDIO            (1 << 2)
#define MAC1_PHY_LINK            (1 << 0)
#define MAC2_PHY_LINK            (1 << 1)
#else
#define MAC2_MDC_MDIO            (1 << 20)
#define MAC2_MII                 (1 << 21)
#define MAC1_PHY_LINK            (1 << 25)
#define MAC2_PHY_LINK            (1 << 26)
#endif

#if	defined(CONFIG_AST1300)
unsigned int faradaynic_iobase[1] = {CONFIG_MACREG_BASE};
#else
unsigned int faradaynic_iobase[1] = {0x1E660000};
#endif

#undef DEBUG_SROM
#undef DEBUG_SROM2

#undef UPDATE_SROM

/* PCI Registers.
 */
#define PCI_CFDA_PSM		0x43

#define CFRV_RN		0x000000f0	/* Revision Number */

#define WAKEUP		0x00		/* Power Saving Wakeup */
#define SLEEP		0x80		/* Power Saving Sleep Mode */

#define DC2114x_BRK	0x0020		/* CFRV break between DC21142 & DC21143 */

/* MAC chip register */
#define ISR_REG				0x00				// interrups status register
#define IER_REG				0x04				// interrupt maks register
#define MAC_MADR_REG		        0x08				// MAC address (Most significant)
#define MAC_LADR_REG		        0x0c				// MAC address (Least significant)

#define MAHT0_REG			0x10				// Multicast Address Hash Table 0 register
#define MAHT1_REG			0x14				// Multicast Address Hash Table 1 register
#define TXPD_REG			0x18				// Transmit Poll Demand register
#define RXPD_REG			0x1c				// Receive Poll Demand register
#define TXR_BADR_REG		        0x20				// Transmit Ring Base Address register
#define RXR_BADR_REG		        0x24				// Receive Ring Base Address register

#define HPTXPD_REG			0x28	//
#define HPTXR_BADR_REG		        0x2c	//

#define ITC_REG				0x30				// interrupt timer control register
#define APTC_REG			0x34				// Automatic Polling Timer control register
#define DBLAC_REG			0x38				// DMA Burst Length and Arbitration control register

#define DMAFIFOS_REG		        0x3c	//
#define FEAR_REG			0x44	//
#define TPAFCR_REG			0x48	//
#define RBSR_REG			0x4c	//for NC Body
#define MACCR_REG			0x50				// MAC control register
#define MACSR_REG			0x54				// MAC status register
#define PHYCR_REG			0x60				// PHY control register
#define PHYDATA_REG			0x64				// PHY Write Data register
#define FCR_REG				0x68				// Flow Control register
#define BPR_REG				0x6c				// back pressure register
#define WOLCR_REG			0x70				// Wake-On-Lan control register
#define WOLSR_REG			0x74				// Wake-On-Lan status register
#define WFCRC_REG			0x78				// Wake-up Frame CRC register
#define WFBM1_REG			0x80				// wake-up frame byte mask 1st double word register
#define WFBM2_REG			0x84				// wake-up frame byte mask 2nd double word register
#define WFBM3_REG			0x88				// wake-up frame byte mask 3rd double word register
#define WFBM4_REG			0x8c				// wake-up frame byte mask 4th double word register


// --------------------------------------------------------------------
//		MACCR_REG
// --------------------------------------------------------------------

#define SW_RST_bit			(1UL<<31)				// software reset/
#define DIRPATH_bit			(1UL<<21)	
#define RX_IPCS_FAIL_bit	(1UL<<20)	//
#define RX_TCPCS_FAIL_bit	(1UL<<19)	//
#define SPEED_100M_MODE_bit     (1UL<<19)               
#define RX_UDPCS_FAIL_bit	(1UL<<18)	//
#define RX_BROADPKT_bit		(1UL<<17)				// Receiving broadcast packet
#define RX_MULTIPKT_bit		(1UL<<16)				// receiving multicast packet
#define RX_HT_EN_bit		(1UL<<15)
#define RX_ALLADR_bit		(1UL<<14)				// not check incoming packet's destination address
#define JUMBO_LF_bit		(1UL<<13)	//
#define RX_RUNT_bit			(1UL<<12)				// Store incoming packet even its length is les than 64 byte
#define CRC_CHK_bit			(1UL<<11)	//
#define CRC_APD_bit			(1UL<<10)				// append crc to transmit packet
#define GMAC_MODE_bit		(1UL<<9)	//
#define FULLDUP_bit			(1UL<<8)				// full duplex
#define ENRX_IN_HALFTX_bit	(1UL<<7)	//
#define LOOP_EN_bit			(1UL<<6)				// Internal loop-back
#define HPTXR_EN_bit		(1UL<<5)	//
#define REMOVE_VLAN_bit		(1UL<<4)	//
#define RXMAC_EN_bit		(1UL<<3)				// receiver enable
#define TXMAC_EN_bit		(1UL<<2)				// transmitter enable
#define RXDMA_EN_bit		(1UL<<1)				// enable DMA receiving channel
#define TXDMA_EN_bit		(1UL<<0)				// enable DMA transmitting channel

//---------------------------------------------------
//  PHY R/W Register Bit
//---------------------------------------------------
#define MIIWR                   (1UL<<27)
#define MIIRD                   (1UL<<26)
#define MDC_CYCTHR              0x34
#define PHY_SPEED_MASK          0xC000
#define PHY_DUPLEX_MASK         0x2000
#define SPEED_1000M             0x02
#define SPEED_100M              0x01
#define SPEED_10M               0x00
#define DUPLEX_FULL             0x01
#define DUPLEX_HALF             0x00
#define RESOLVED_BIT            0x800

#define PHY_SPEED_DUPLEX_MASK   0x01E0
#define PHY_100M_DUPLEX         0x0100
#define PHY_100M_HALF           0x0080
#define PHY_10M_DUPLEX          0x0040
#define PHY_10M_HALF            0x0020



/* Descriptor bits.
 */
#define TXDMA_OWN	0x80000000	/* Own Bit */
#define RXPKT_RDY       0x00000000      
#define RXPKT_STATUS    0x80000000
//#define EDORR		0x00008000	/* Receive End Of Ring */
#define EDORR		0x40000000	/* Receive End Of Ring */
#define LRS		0x10000000	/* Last Descriptor */
#define FRS		0x20000000	/* First Descriptor */
#define RD_ES		0x00008000	/* Error Summary */
//#define EDOTR		0x00008000	/* Transmit End Of Ring */
#define EDOTR		0x40000000	/* Transmit End Of Ring */
#define T_OWN		0x80000000	/* Own Bit */
#define LTS		0x10000000	/* Last Segment */
#define FTS		0x20000000	/* First Segment */
#define CRC_ERR         0x00080000
#define TD_ES		0x00008000	/* Error Summary */
#define TD_SET		0x08000000	/* Setup Packet */
#define RX_ERR          0x00040000
#define FTL             0x00100000
#define RUNT            0x00200000
#define RX_ODD_NB       0x00400000

#define POLL_DEMAND	1
#define RESET_DE4X5(dev) {\
    int i;\
    i=INL(dev, MACCR_REG);\
    udelay(1000);\
    OUTL(dev, i | SW_RST_bit | SPEED_100M_MODE_bit, MACCR_REG);\
    for (; (INL(dev, MACCR_REG ) & SW_RST_bit) != 0; ) {udelay(1000);}\
    OUTL(dev, 0, IER_REG );	\
}

#define START_MAC(dev) {\
    s32 omr; \
    omr = INL(dev, MACCR_REG);\
    omr |= RXMAC_EN_bit | TXMAC_EN_bit | RXDMA_EN_bit | TXDMA_EN_bit;\
    OUTL(dev, omr, MACCR_REG);		/* Enable the TX and/or RX */\
}

#define STOP_MAC(dev) {\
    s32 omr; \
    omr = INL(dev, MACCR_REG);\
    omr &= ~(RXMAC_EN_bit | TXMAC_EN_bit | RXDMA_EN_bit | TXDMA_EN_bit);\
    OUTL(dev, omr, MACCR_REG);		/* Disable the TX and/or RX */ \
}

#define NUM_RX_DESC PKTBUFSRX
#define NUM_TX_DESC 1			/* Number of TX descriptors   */
#define RX_BUFF_SZ  PKTSIZE_ALIGN
#define TX_BUFF_SZ  1514

#define TOUT_LOOP   1000000
#define PHY_LOOP    250
#define ETH_ALEN	6
#define NCSI_LOOP   1500000
#define RETRY_COUNT     1

struct de4x5_desc {
	volatile s32 status;
	u32 des1;
	u32 reserved;
	u32 buf;	
};

//PHY Information
#define PHYID_VENDOR_MASK		0xfffffc00
#define PHYID_VENDOR_MODEL_MASK		0xfffffff0
#define PHYID_VENDOR_MARVELL		0x01410c00
#define PHYID_VENDOR_BROADCOM		0x00406000
#define PHYID_VENDOR_REALTEK		0x001cc800
#define PHYID_RTL8201EL			0x001cc810
#define PHYID_RTL8211			0x001cc910

//NCSI define & structure
//NC-SI Command Packet
typedef struct {
//Ethernet Header
    unsigned char  DA[6];
    unsigned char  SA[6];
    unsigned short EtherType;						//DMTF NC-SI
//NC-SI Control Packet
    unsigned char  MC_ID;						//Management Controller should set this field to 0x00
    unsigned char  Header_Revision;					//For NC-SI 1.0 spec, this field has to set 0x01
    unsigned char  Reserved_1;						//Reserved has to set to 0x00
    unsigned char  IID;							//Instance ID
    unsigned char  Command;
    unsigned char  Channel_ID;
    unsigned short Payload_Length;					//Payload Length = 12 bits, 4 bits are reserved
    unsigned long  Reserved_2;
    unsigned long  Reserved_3;
}  NCSI_Command_Packet;

unsigned char  Payload_Data[16];
unsigned char  Payload_Pad[4] = {0x00, 0x00, 0x00, 0x00};
unsigned long  Payload_Checksum = 0x00000000;


//Command and Response Type
#define	CLEAR_INITIAL_STATE			0x00			//M
#define	SELECT_PACKAGE				0x01			//M
#define	DESELECT_PACKAGE			0x02			//M
#define	ENABLE_CHANNEL				0x03			//M
#define	DISABLE_CHANNEL				0x04			//M
#define	RESET_CHANNEL				0x05			//M
#define	ENABLE_CHANNEL_NETWORK_TX		0x06			//M
#define	DISABLE_CHANNEL_NETWORK_TX		0x07			//M
#define	AEN_ENABLE				0x08
#define	SET_LINK				0x09			//M
#define	GET_LINK_STATUS				0x0A			//M
#define	SET_VLAN_FILTER				0x0B			//M
#define	ENABLE_VLAN				0x0C			//M
#define	DISABLE_VLAN				0x0D			//M
#define	SET_MAC_ADDRESS				0x0E			//M
#define	ENABLE_BROADCAST_FILTERING		0x10			//M
#define	DISABLE_BROADCAST_FILTERING		0x11			//M
#define	ENABLE_GLOBAL_MULTICAST_FILTERING	0x12
#define	DISABLE_GLOBAL_MULTICAST_FILTERING	0x13
#define	SET_NCSI_FLOW_CONTROL			0x14
#define	GET_VERSION_ID				0x15			//M
#define	GET_CAPABILITIES			0x16			//M
#define	GET_PARAMETERS				0x17			//M
#define	GET_CONTROLLER_PACKET_STATISTICS	0x18
#define	GET_NCSI_STATISTICS			0x19
#define	GET_NCSI_PASS_THROUGH_STATISTICS	0x1A

//NC-SI Response Packet
typedef struct {
    unsigned char  DA[6];
    unsigned char  SA[6];
    unsigned short EtherType;						//DMTF NC-SI
//NC-SI Control Packet
    unsigned char  MC_ID;						//Management Controller should set this field to 0x00
    unsigned char  Header_Revision;					//For NC-SI 1.0 spec, this field has to set 0x01
    unsigned char  Reserved_1;						//Reserved has to set to 0x00
    unsigned char  IID;							//Instance ID
    unsigned char  Command;
    unsigned char  Channel_ID;
    unsigned short Payload_Length;					//Payload Length = 12 bits, 4 bits are reserved
    unsigned short  Reserved_2;
    unsigned short  Reserved_3;
    unsigned short  Reserved_4;
    unsigned short  Reserved_5;
    unsigned short  Response_Code;
    unsigned short  Reason_Code;
    unsigned char   Payload_Data[64];
}  NCSI_Response_Packet;

NCSI_Command_Packet NCSI_Request;
NCSI_Command_Packet NCSI_RequestReceive;
NCSI_Response_Packet NCSI_Respond;
NCSI_Response_Packet NCSI_RespondSend;

//Standard Response Code
#define	COMMAND_COMPLETED			0x00
#define	COMMAND_FAILED				0x01
#define	COMMAND_UNAVAILABLE			0x02
#define	COMMAND_UNSUPPORTED			0x03

//Standard Reason Code
#define	NO_ERROR				0x0000
#define	INTERFACE_INITIALIZATION_REQUIRED	0x0001
#define	PARAMETER_IS_INVALID			0x0002
#define	CHANNEL_NOT_READY			0x0003
#define	PACKAGE_NOT_READY			0x0004
#define	INVALID_PAYLOAD_LENGTH			0x0005
#define	UNKNOWN_COMMAND_TYPE			0x7FFF


struct AEN_Packet {
//Ethernet Header
    unsigned char  DA[6];
    unsigned char  SA[6];						//Network Controller SA = FF:FF:FF:FF:FF:FF
    unsigned short EtherType;						//DMTF NC-SI
//AEN Packet Format
    unsigned char  MC_ID;						//Network Controller should set this field to 0x00
    unsigned char  Header_Revision;					//For NC-SI 1.0 spec, this field has to set 0x01
    unsigned char  Reserved_1;						//Reserved has to set to 0x00
//    unsigned char  IID = 0x00;						//Instance ID = 0 in Network Controller
//    unsigned char  Command = 0xFF;					//AEN = 0xFF
    unsigned char  Channel_ID;
//    unsigned short Payload_Length = 0x04;				//Payload Length = 4 in Network Controller AEN Packet
    unsigned long  Reserved_2;
    unsigned long  Reserved_3;
    unsigned char  AEN_Type;
//    unsigned char  Reserved_4[3] = {0x00, 0x00, 0x00};
    unsigned long  Optional_AEN_Data;
    unsigned long  Payload_Checksum;
};

//AEN Type
#define	LINK_STATUS_CHANGE			0x0
#define	CONFIGURATION_REQUIRED			0x1
#define	HOST_NC_DRIVER_STATUS_CHANGE		0x2

typedef struct {
	unsigned char Package_ID;
	unsigned char Channel_ID;
	unsigned long Capabilities_Flags;
	unsigned long Broadcast_Packet_Filter_Capabilities;
	unsigned long Multicast_Packet_Filter_Capabilities;
	unsigned long Buffering_Capabilities;
	unsigned long AEN_Control_Support;
} NCSI_Capability;
NCSI_Capability NCSI_Cap;

//SET_MAC_ADDRESS
#define UNICAST		(0x00 << 5)
#define MULTICAST	(0x01 << 5)
#define DISABLE_MAC_ADDRESS_FILTER	0x00
#define ENABLE_MAC_ADDRESS_FILTER	0x01

//GET_LINK_STATUS
#define LINK_DOWN	0
#define LINK_UP		1

static struct de4x5_desc rx_ring[NUM_RX_DESC] __attribute__ ((aligned(32))); /* RX descriptor ring         */
static struct de4x5_desc tx_ring[NUM_TX_DESC] __attribute__ ((aligned(32))); /* TX descriptor ring         */
static int rx_new;                             /* RX descriptor ring pointer */
static int tx_new;                             /* TX descriptor ring pointer */
static unsigned char tx_buffer[NUM_TX_DESC][TX_BUFF_SZ] __attribute__ ((aligned(32)));
static unsigned char rx_buffer[NUM_RX_DESC][RX_BUFF_SZ] __attribute__ ((aligned(32)));


static char rxRingSize;
static char txRingSize;
static unsigned int InstanceID = 0;
static int Retry = 0;

static int   faradaynic_init(struct eth_device* dev, bd_t* bis);
static int   faradaynic_send(struct eth_device* dev, volatile void *packet, int length);
static int   faradaynic_recv(struct eth_device* dev);
static void  faradaynic_halt(struct eth_device* dev);
static void  set_mac_address (struct eth_device* dev, bd_t* bis);
static void  phy_write_register (struct eth_device* dev, u8 PHY_Register, u8 PHY_Address, u16 PHY_Data);
static u16   phy_read_register (struct eth_device* dev, u8 PHY_Register, u8 PHY_Address);
static void  set_mac_control_register(struct eth_device* dev);

#if defined(CONFIG_E500)
#define phys_to_bus(a) (a)
#else
#define phys_to_bus(a)	pci_phys_to_mem((pci_dev_t)dev->priv, a)
#endif

static int INL(struct eth_device* dev, u_long addr)
{
	return le32_to_cpu(*(volatile u_long *)(addr + dev->iobase));
}

static void OUTL(struct eth_device* dev, int command, u_long addr)
{
	*(volatile u_long *)(addr + dev->iobase) = cpu_to_le32(command);
}


struct eth_device	faradaynic_device[1];

void NCSI_Struct_Initialize(void)
{
	unsigned long i;

	for (i = 0; i < 6; i++) {
		NCSI_Request.DA[i] = 0xFF;
		NCSI_Respond.DA[i] = 0xFF;
		NCSI_Respond.SA[i] = 0xFF;
		NCSI_RespondSend.DA[i] = 0xFF;
		NCSI_RespondSend.SA[i] = 0xFF;
	}
	NCSI_Request.EtherType = 0xF888;
	NCSI_Request.MC_ID = 0;
	NCSI_Request.Header_Revision = 0x01;
	NCSI_Request.Reserved_1 = 0;
	NCSI_Request.Reserved_2 = 0;
	NCSI_Request.Reserved_3 = 0;
	NCSI_Respond.EtherType = 0xF888;
	NCSI_Respond.MC_ID = 0;
	NCSI_Respond.Header_Revision = 0x01;
	NCSI_Respond.Reserved_1 = 0;
	NCSI_Respond.Reserved_2 = 0;
	NCSI_Respond.Reserved_3 = 0;
	NCSI_RespondSend.EtherType = 0xF888;
	NCSI_RespondSend.MC_ID = 0;
	NCSI_RespondSend.Header_Revision = 0x01;
	NCSI_RespondSend.Reserved_1 = 0;
	NCSI_RespondSend.Reserved_2 = 0;
	NCSI_RespondSend.Reserved_3 = 0;

}

int faradaynic_initialize(bd_t *bis)
{
	int             	card_number = 0;
	unsigned int		iobase, SCURegister;
	struct eth_device* 	dev;

#if defined(CONFIG_AST2300_FPGA_2) || defined(CONFIG_AST2300)
//AST2300
//MAC1 CLOCK/RESET/PHY_LINK/MDC_MDIO in SCU
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_CLOCK_CONTROL));
	*(volatile u_long *)(SCU_BASE + SCU_CLOCK_CONTROL) = cpu_to_le32(SCURegister & ~(MAC1_CLOCK_ENABLE));
//Add Clock Selection in AST2300 A1, Please check the datasheet for more detail
//The current sample code uses 0: H-PLL/2 because all EVBs have RGMII interface
//        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_CLOCK_SELECTION));
//	*(volatile u_long *)(SCU_BASE + SCU_CLOCK_SELECTION) = cpu_to_le32(SCURegister & ~(MAC_AHB_CLOCK_DIVIDER));
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_RESET_CONTROL));
	*(volatile u_long *)(SCU_BASE + SCU_RESET_CONTROL) = cpu_to_le32(SCURegister & ~(0x800));
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL3_REG));
	*(volatile u_long *)(SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL3_REG) = cpu_to_le32(SCURegister | (MAC1_MDIO | MAC1_MDC));
//        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_MAC_CLOCK_DELAY));
//Currently we use fix value in MAC timing on EVB
//	*(volatile u_long *)(SCU_BASE + SCU_MAC_CLOCK_DELAY) = CONFIG_MAC_INTERFACE_CLOCK_DELAY;
#ifdef CONFIG_MAC1_PHY_LINK_INTERRUPT
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL1_REG));
	*(volatile u_long *)(SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL1_REG) = cpu_to_le32(SCURegister | (MAC1_PHY_LINK));
#endif

//MAC2 CLOCK/RESET/PHY_LINK/MDC_MDIO
#ifdef CONFIG_MAC2_ENABLE
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_CLOCK_CONTROL));
	*(volatile u_long *)(SCU_BASE + SCU_CLOCK_CONTROL) = cpu_to_le32(SCURegister & ~(MAC2_CLOCK_ENABLE));
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_RESET_CONTROL));
	*(volatile u_long *)(SCU_BASE + SCU_RESET_CONTROL) = cpu_to_le32(SCURegister & ~(0x1000));
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL5_REG));
	*(volatile u_long *)(SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL5_REG) = cpu_to_le32(SCURegister | (MAC2_MDC_MDIO));
#endif
#ifdef CONFIG_MAC2_PHY_LINK_INTERRUPT
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL1_REG));
	*(volatile u_long *)(SCU_BASE + SCU_MULTIFUNCTION_PIN_CTL1_REG) = cpu_to_le32(SCURegister | (MAC2_PHY_LINK));
#endif
#else
//AST1100/AST2050/AST2100
//MAC1 RESET/PHY_LINK in SCU
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_RESET_CONTROL));
	*(volatile u_long *)(SCU_BASE + SCU_RESET_CONTROL) = cpu_to_le32(SCURegister & ~(0x800));
#ifdef CONFIG_MAC1_PHY_LINK_INTERRUPT
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_PIN_MUX));
	*(volatile u_long *)(SCU_BASE + SCU_PIN_MUX) = cpu_to_le32(SCURegister | (MAC1_PHY_LINK));
#endif

//MAC2
#ifdef CONFIG_MAC2_ENABLE
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_RESET_CONTROL));
	*(volatile u_long *)(SCU_BASE + SCU_RESET_CONTROL) = cpu_to_le32(SCURegister & ~(0x1000));
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_PIN_MUX));
	*(volatile u_long *)(SCU_BASE + SCU_PIN_MUX) = cpu_to_le32(SCURegister | (MAC2_MDC_MDIO));
#endif
#ifdef CONFIG_MAC2_MII_ENABLE
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_PIN_MUX));
	*(volatile u_long *)(SCU_BASE + SCU_PIN_MUX) = cpu_to_le32(SCURegister | (MAC2_MII));
#endif
#ifdef CONFIG_MAC2_PHY_LINK_INTERRUPT
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_PIN_MUX));
	*(volatile u_long *)(SCU_BASE + SCU_PIN_MUX) = cpu_to_le32(SCURegister | (MAC2_PHY_LINK));
#endif
#endif

	iobase = faradaynic_iobase[card_number];

	dev = &faradaynic_device[card_number];
	

	sprintf(dev->name, "faradaynic#%d", card_number);

	dev->iobase = iobase;

        if (CONFIG_MAC1_PHY_SETTING >= 1) {
//NCSI Struct Initialize
		NCSI_Struct_Initialize();
	}
//Set Scratch register (0x1E6E2040 D[15:14])(0x1E6E2041 D[7:6]) to inform kernel MAC1 driver
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_SCRATCH_REGISTER));
	*(volatile u_long *)(SCU_BASE + SCU_SCRATCH_REGISTER) = cpu_to_le32((SCURegister & ~(0xc000)) | (CONFIG_MAC1_PHY_SETTING << 14));
//Set Scratch register (0x1E6E2040 D[13:12])(0x1E6E2041 D[5:4]) to inform kernel MAC2 driver
        SCURegister = le32_to_cpu(*(volatile u_long *)(SCU_BASE + SCU_SCRATCH_REGISTER));
	*(volatile u_long *)(SCU_BASE + SCU_SCRATCH_REGISTER) = cpu_to_le32((SCURegister & ~(0x3000)) | (CONFIG_MAC2_PHY_SETTING << 12));


	dev->init   = faradaynic_init;
	dev->halt   = faradaynic_halt;
	dev->send   = faradaynic_send;
	dev->recv   = faradaynic_recv;

	/* Ensure we're not sleeping. */
	if (CONFIG_MAC1_PHY_SETTING >= 1) {
		udelay(2000000); //2.0 sec
	}
	else {
		udelay(10 * 1000);
	}


	dev->init(dev, bis);

	eth_register(dev);
	

	return card_number;
}

void Calculate_Checksum(unsigned char *buffer_base, int Length)
{
	unsigned int i, CheckSum = 0;
	unsigned int Data, Data1;

	for (i = 0; i < ((Length - 14) / 2); i++) {
		Data = buffer_base[i * 2];
		Data1 = buffer_base[i * 2 + 1];
		CheckSum += ((Data << 8) + Data1);
	}
	Payload_Checksum = (~(CheckSum) + 1); //2's complement
//Inverse for insert into buffer
	Data = (Payload_Checksum & 0xFF000000) >> 24;
	Data1 = (Payload_Checksum & 0x000000FF) << 24;
	Payload_Checksum = (Payload_Checksum & 0x00FFFF00) + Data + Data1;
	Data = (Payload_Checksum & 0x00FF0000) >> 8;
	Data1 = (Payload_Checksum & 0x0000FF00) << 8;
	Payload_Checksum = (Payload_Checksum & 0xFF0000FF) + Data + Data1;
}

void copy_data (int Length)
{
	memcpy ((unsigned char *)(tx_ring[tx_new].buf + 30), &Payload_Data, Length);
	Calculate_Checksum((unsigned char *)(tx_ring[tx_new].buf + 14), 30 + Length);
	memcpy ((unsigned char *)(tx_ring[tx_new].buf + 30 + Length), &Payload_Checksum, 4);
}

void NCSI_Rx (struct eth_device* dev)
{
	unsigned long status, length, i = 0;

 	do {
		status = (s32)le32_to_cpu(rx_ring[rx_new].status);
		if ((status & RXPKT_STATUS) != 0) {
			if (status & FRS) {
			length = (le32_to_cpu(rx_ring[rx_new].status) & 0x3FFF);
				length = length < sizeof(NCSI_Respond) ? length : sizeof(NCSI_Respond);
			memcpy (&NCSI_Respond, (unsigned char *)rx_ring[rx_new].buf, length);
				if (NCSI_Respond.EtherType == 0xF888 && NCSI_Respond.Command >= 0x80) {
				//	printf ("NCSI rsp iid %x, command = %x, status = %lx, code = %x, reason = %x\n",
			//			NCSI_Respond.IID, NCSI_Respond.Command, status, NCSI_Respond.Response_Code, NCSI_Respond.Reason_Code);
					rx_ring[rx_new].status &= cpu_to_le32(0x7FFFFFFF);
					rx_new = (rx_new + 1) % rxRingSize;
					break;
				} else if (NCSI_Respond.EtherType == 0xF888) {
					length = length < sizeof(NCSI_RequestReceive) ? length : sizeof(NCSI_RequestReceive);
					memcpy (&NCSI_RequestReceive, (unsigned char *)rx_ring[rx_new].buf, length);
					printf ("NCSI req iid %x, command = %x\n", NCSI_RequestReceive.IID, NCSI_RequestReceive.Command);
					NCSI_RespondSend.IID = NCSI_RequestReceive.IID;
					NCSI_RespondSend.Command = NCSI_RequestReceive.Command + 0x80;
					NCSI_RespondSend.Channel_ID = NCSI_RequestReceive.Channel_ID;
					NCSI_RespondSend.Payload_Length = 0;
					memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_RespondSend, 30);
					copy_data (NCSI_RespondSend.Payload_Length);
					faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_RespondSend.Payload_Length + 4);
					rx_ring[rx_new].status &= cpu_to_le32(0x7FFFFFFF);
					rx_new = (rx_new + 1) % rxRingSize;
					continue;
				} else {
					printf ("Not NCSI command, EtherType=%x\n", NCSI_Respond.EtherType);
					rx_ring[rx_new].status &= cpu_to_le32(0x7FFFFFFF);
					rx_new = (rx_new + 1) % rxRingSize;
					continue;
				}
		}
			printf("this package is not FRS\n");
		rx_ring[rx_new].status &= cpu_to_le32(0x7FFFFFFF);
		rx_new = (rx_new + 1) % rxRingSize;
			continue;
	}
		i++;
	} while (i <= NCSI_LOOP);
	if (i > NCSI_LOOP) {
		printf ("error: cannot receive, %lx\n", status);
	}

}

void DeSelect_Package (struct eth_device* dev, int Package_ID)
{
	unsigned long Combined_Channel_ID;

//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = DESELECT_PACKAGE;
	Combined_Channel_ID = (Package_ID << 5) + 0x1F; //Internal Channel ID = 0x1F, 0x1F means all channel
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (DESELECT_PACKAGE | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

int Select_Package (struct eth_device* dev, int Package_ID)
{
	unsigned long Combined_Channel_ID, Found = 0;

//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = SELECT_PACKAGE;
	Combined_Channel_ID = (Package_ID << 5) + 0x1F; //Internal Channel ID = 0x1F
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = (4 << 8);
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	NCSI_Request.Payload_Length = 4;
        memset ((void *)Payload_Data, 0, 4);
	Payload_Data[3] = 1; //Arbitration Disable
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (SELECT_PACKAGE | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		Found = 0;
		InstanceID--;
	}
	else {
		Retry = 0;
		Found = 1;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;

    return Found;
}

void DeSelect_Active_Package (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = DESELECT_PACKAGE;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + 0x1F; //Internal Channel ID = 0x1F, 0x1F means all channel
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (DESELECT_PACKAGE | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

int Select_Active_Package (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID, Found = 0;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = SELECT_PACKAGE;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + 0x1F; //Internal Channel ID = 0x1F
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = (4 << 8);
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	NCSI_Request.Payload_Length = 4;
        memset ((void *)Payload_Data, 0, 4);
	Payload_Data[3] = 1; //Arbitration Disable
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (SELECT_PACKAGE | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		Found = 0;
		InstanceID--;
	}
	else {
		Retry = 0;
		Found = 1;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;

    return Found;
}

int Clear_Initial_State (struct eth_device* dev, int Channel_ID)
{
	unsigned long Combined_Channel_ID, Found = 0;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = CLEAR_INITIAL_STATE;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + Channel_ID; //Internal Channel ID = 0
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (CLEAR_INITIAL_STATE | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		Found = 0;
		InstanceID--;
	}
	else {
		Retry = 0;
		Found = 1;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;

    return Found;
}

void Get_Version_ID (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = GET_VERSION_ID;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (GET_VERSION_ID | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

void Get_Capabilities (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = GET_CAPABILITIES;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (GET_CAPABILITIES | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
		NCSI_Cap.Capabilities_Flags = NCSI_Respond.Payload_Data[0];
		NCSI_Cap.Broadcast_Packet_Filter_Capabilities = NCSI_Respond.Payload_Data[1];
		NCSI_Cap.Multicast_Packet_Filter_Capabilities = NCSI_Respond.Payload_Data[2];
		NCSI_Cap.Buffering_Capabilities = NCSI_Respond.Payload_Data[3];
		NCSI_Cap.AEN_Control_Support = NCSI_Respond.Payload_Data[4];
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

void Enable_Set_MAC_Address (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID, i;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = SET_MAC_ADDRESS;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = (8 << 8);
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	NCSI_Request.Payload_Length = 8;
	for (i = 0; i < 6; i++) {
		Payload_Data[i] = NCSI_Request.SA[i];
	}
	Payload_Data[6] = 1; //MAC Address Num = 1 --> address filter 1, fixed in sample code
	Payload_Data[7] = UNICAST + 0 + ENABLE_MAC_ADDRESS_FILTER; //AT + Reserved + E
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (SET_MAC_ADDRESS | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

void Enable_Broadcast_Filter (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = ENABLE_BROADCAST_FILTERING;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = (4 << 8);
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	NCSI_Request.Payload_Length = 4;
        memset ((void *)Payload_Data, 0, 4);
        Payload_Data[3] = 0xF; //ARP, DHCP, NetBIOS
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (ENABLE_BROADCAST_FILTERING | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

void Enable_AEN (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = AEN_ENABLE;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = (8 << 8);
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	NCSI_Request.Payload_Length = 8;
        memset ((void *)Payload_Data, 0, 8);
        Payload_Data[3] = 0x00; //MC ID
        Payload_Data[7] = 0x01; //Link Status only
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (AEN_ENABLE | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

void Enable_Network_TX (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = ENABLE_CHANNEL_NETWORK_TX;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (ENABLE_CHANNEL_NETWORK_TX | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

void Disable_Network_TX (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = DISABLE_CHANNEL_NETWORK_TX;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (DISABLE_CHANNEL_NETWORK_TX | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

void Enable_Channel (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = ENABLE_CHANNEL;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (ENABLE_CHANNEL | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

void Disable_Channel (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = DISABLE_CHANNEL;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = (4 << 8);
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	NCSI_Request.Payload_Length = 4;
        memset ((void *)Payload_Data, 0, 4);
        Payload_Data[3] = 0x1; //ALD
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (DISABLE_CHANNEL | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

int Get_Link_Status (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = GET_LINK_STATUS;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = 0;
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (GET_LINK_STATUS | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
    if (NCSI_Respond.Payload_Data[3] & 0x40) {
    	return (NCSI_Respond.Payload_Data[3] & 0x01); //Link Up or Not
    }
    else {
    	return 0; //Auto Negotiate did not finish
    }
}

void Set_Link (struct eth_device* dev)
{
	unsigned long Combined_Channel_ID;
//TX
    do {
	InstanceID++;
	NCSI_Request.IID = InstanceID;
	NCSI_Request.Command = SET_LINK;
	Combined_Channel_ID = (NCSI_Cap.Package_ID << 5) + NCSI_Cap.Channel_ID;
	NCSI_Request.Channel_ID = Combined_Channel_ID;
	NCSI_Request.Payload_Length = (8 << 8);
	memcpy ((unsigned char *)tx_ring[tx_new].buf, &NCSI_Request, 30);
	NCSI_Request.Payload_Length = 8;
        memset ((void *)Payload_Data, 0, 8);
        Payload_Data[2] = 0x02; //full duplex
        Payload_Data[3] = 0x04; //100M, auto-disable
	copy_data (NCSI_Request.Payload_Length);
	faradaynic_send (dev, (void *)tx_ring[tx_new].buf, 30 + NCSI_Request.Payload_Length + 4);
//RX
	NCSI_Rx(dev);
	if ((NCSI_Respond.IID != InstanceID) || (NCSI_Respond.Command != (SET_LINK | 0x80)) || (NCSI_Respond.Response_Code != COMMAND_COMPLETED)) {
		printf ("Retry: Command = %x, Response_Code = %x\n", NCSI_Request.Command, NCSI_Respond.Response_Code);
		Retry++;
		InstanceID--;
	}
	else {
		Retry = 0;
	}
    } while ((Retry != 0) && (Retry <= RETRY_COUNT));
    Retry = 0;
}

static int faradaynic_init(struct eth_device* dev, bd_t* bis)
{
	unsigned long	i, Package_Found = 0, Channel_Found = 0, Re_Send = 0, Link_Status;

	RESET_DE4X5(dev);
	RESET_DE4X5(dev);   // do software reset again, notice in aspeed  page 177
	set_mac_address (dev, bis);
        set_mac_control_register (dev);

	for (i = 0; i < NUM_RX_DESC; i++) {
		rx_ring[i].status = cpu_to_le32(RXPKT_RDY + RX_BUFF_SZ);
		rx_ring[i].buf = (u32)(&rx_buffer[i]);
		rx_ring[i].reserved = 0;
	}

	for (i=0; i < NUM_TX_DESC; i++) {
		tx_ring[i].status = 0;
		tx_ring[i].des1 = 0;
		tx_ring[i].buf = (u32)(&tx_buffer[i]);
		tx_ring[i].reserved = 0;
	}

	rxRingSize = NUM_RX_DESC;
	txRingSize = NUM_TX_DESC;

	rx_ring[rxRingSize - 1].status |= cpu_to_le32(EDORR);
	tx_ring[txRingSize - 1].status |= cpu_to_le32(EDOTR);

	OUTL(dev, ((u32) &tx_ring), TXR_BADR_REG);
	OUTL(dev, ((u32) &rx_ring), RXR_BADR_REG);

	START_MAC(dev);

	tx_new = 0;
	rx_new = 0;

	if (CONFIG_MAC1_PHY_SETTING >= 1) {
//NCSI Start
//DeSelect Package/ Select Package
		for (i = 0; i < 4; i++) {
			DeSelect_Package (dev, i);
			Package_Found = Select_Package (dev, i);
			if (Package_Found == 1) {
//AST2100/AST2050/AST1100 supports 1 package only in current firmware version
				NCSI_Cap.Package_ID = i;
//				Package_Found = 0;
				break;
			}
		}
		if (Package_Found != 0) {
//Initiali State
			for (i = 0; i < 2; i++) { //Suppose 2 channels in current version, You could modify it to 0x1F to support 31 channels
				Channel_Found = Clear_Initial_State(dev, i);
				if (Channel_Found == 1) {
					NCSI_Cap.Channel_ID = i;
					printf ("Found NCSI Network Controller at (%d, %d)\n", NCSI_Cap.Package_ID, NCSI_Cap.Channel_ID);
//Get Version and Capabilities
					Get_Version_ID(dev);
					Get_Capabilities(dev);
					Select_Active_Package(dev);
//Configuration
					Enable_Set_MAC_Address(dev);
					Enable_Broadcast_Filter(dev);
//Enable TX	
					Enable_Network_TX(dev);
//Enable Channel
					Enable_Channel(dev);
//Get Link Status
Re_Get_Link_Status:
					Link_Status = Get_Link_Status(dev);
					if (Link_Status == LINK_UP) {
						printf ("Using NCSI Network Controller (%d, %d)\n", NCSI_Cap.Package_ID, NCSI_Cap.Channel_ID);
						break;
					}
					else if ((Link_Status == LINK_DOWN) && (Re_Send < 2)) {
						Re_Send++;
						goto Re_Get_Link_Status;
					}
//Disable TX	
					Disable_Network_TX(dev);
//Disable Channel
//					Disable_Channel(dev);
					Re_Send = 0;
					Channel_Found = 0;
				}
			}
	    	}
	}
	return 1;
}

static int faradaynic_send(struct eth_device* dev, volatile void *packet, int length)
{
	int		status = -1, oldlength = 0, fail = 0;
	int		i;

	if (length <= 0) {
		printf("%s: bad packet size: %d\n", dev->name, length);
		goto Done;
	}


	for(i = 0; (tx_ring[tx_new].status & cpu_to_le32(TXDMA_OWN)) == 0x80000000; i++) {
		if (i >= TOUT_LOOP) {
			printf("%s: tx error buffer not ready, status=%x, %x\n", dev->name, tx_ring[tx_new].status, INL(dev, MACCR_REG ));
                        fail = 1;
			goto Done;
		}
	}
        

        if (length < 60) {
            oldlength = length;
//            memset ((void *)cpu_to_le32((u32) (packet + length)), 0, 60 - length);
            length = 60;
        }
	tx_ring[tx_new].buf    = cpu_to_le32(((u32) packet));
	tx_ring[tx_new].status   &= (~(0x3FFF));
	tx_ring[tx_new].status   |= cpu_to_le32(LTS | FTS | length);
	tx_ring[tx_new].status |= cpu_to_le32(TXDMA_OWN);
        
	OUTL(dev, POLL_DEMAND, TXPD_REG);

	for (i = 0; (tx_ring[tx_new].status & cpu_to_le32(TXDMA_OWN)) == 0x80000000; i++) 
	{
		if (i >= TOUT_LOOP) 
		{
			printf(".%s: tx buffer not readyi, status=%x, %x\n", dev->name, tx_ring[tx_new].status, INL(dev, MACCR_REG ));
                        fail = 1;
			goto Done;
		}
	}

        if (fail != 1) {
            status = oldlength;
        }

 Done:
    tx_new = (tx_new+1) % NUM_TX_DESC;

	return status;
}

static int faradaynic_recv(struct eth_device* dev)
{
	s32		status;
	int		length    = 0;
	int i;

	for ( ; ; ) 
	{
		status = (s32)le32_to_cpu(rx_ring[rx_new].status);

		if ((status & RXPKT_STATUS) == 0) {
			break;
		}

		if (status & LRS) {
			/* Valid frame status.
			 */
			if (status & (RX_ERR | CRC_ERR | FTL | RUNT | RX_ODD_NB)) {

				/* There was an error.
				 */
				printf("RX error status = 0x%08X\n", status);
			} else {
				/* A valid frame received.
				 */
				length = (le32_to_cpu(rx_ring[rx_new].status) & 0x3FFF);
				/* Pass the packet up to the protocol
				 * layers.
				 */
#ifdef NET_DEBUG				 
				 printf ("[%s] Frame length %d Data :\n\n",__FUNCTION__,length-4 );
				 for (i =0 ;i < (length-4 ); i++)
				 {
				 	if (i != 0 && (i % 16) == 0)
				 		printf("\n");
				 	printf("%2x ",rx_buffer[rx_new][i]);
				 }
				 printf("\n");
#endif					 
				NetReceive(rx_buffer[rx_new], length - 4);
			}

			/* Change buffer ownership for this frame, back
			 * to the adapter.
			 */
			rx_ring[rx_new].status &= cpu_to_le32(0x7FFFFFFF);
//			rx_ring[rx_new].status = cpu_to_le32(RXPKT_RDY);
		}

		/* Update entry information.
		 */
		rx_new = (rx_new + 1) % rxRingSize;
	}

	return length;
}

static void faradaynic_halt(struct eth_device* dev)
{
	STOP_MAC(dev);
}

static void set_mac_address (struct eth_device* dev, bd_t* bis)
{
	unsigned char  mac_address[6]; // 6 bytes mac address
        char  ethaddress[20]; // string for setenv function
	char *s;
        int i, env;                // env variable 0: eeprom, 1: environment parameters

        s = getenv ("eeprom");
        env = (s && (*s == 'y')) ? 0 : 1;

	if (env == 0) {
            env = 1;
	    eeprom_init ();
	    eeprom_read (0xA0, 0, mac_address, 6);

            for (i = 0; i < 6; i++) {
                if (mac_address[i] != 0xFF) {
                    env = 0;	//Suppose not all 0xFF is valid
                }
            }
        }

        if (env == 0) { // EEPROM
            sprintf (ethaddress, "%02X:%02X:%02X:%02X:%02X:%02X", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
            setenv ("ethaddr", ethaddress);
	    OUTL(dev, ((mac_address[2] << 24) | (mac_address[3] << 16) | (mac_address[4] << 8) | mac_address[5]), MAC_LADR_REG);
	    OUTL(dev, ((mac_address[0] << 8) | mac_address[1]), MAC_MADR_REG);
	    if (CONFIG_MAC1_PHY_SETTING >= 1) {
		    for (i = 0; i < 6; i++) {
	    		NCSI_Request.SA[i] = mac_address[i];
	    	    }
	    }
        }
        else { // Environment Parameters
	    OUTL(dev, ((bis->bi_enetaddr[2] << 24) | (bis->bi_enetaddr[3] << 16) | (bis->bi_enetaddr[4] << 8) | bis->bi_enetaddr[5]), MAC_LADR_REG);
	    OUTL(dev, ((bis->bi_enetaddr[0] << 8) | bis->bi_enetaddr[1]), MAC_MADR_REG);
	    if (CONFIG_MAC1_PHY_SETTING >= 1) {
		    for (i = 0; i < 6; i++) {
		    	NCSI_Request.SA[i] = bis->bi_enetaddr[i];
	    	    }
	    }
        }

}


static u16 phy_read_register (struct eth_device* dev, u8 PHY_Register, u8 PHY_Address)
{
       u32 Data, Status = 0, Loop_Count = 0, PHY_Ready = 1;
       u16 Return_Data;

#ifdef	REALTEK_PHY_SUPPORT
       PHY_Address = 0x01;
#endif
//20us * 100 = 2ms > (1 / 2.5Mhz) * 0x34
       OUTL(dev, (PHY_Register << 21) + (PHY_Address << 16) + MIIRD + MDC_CYCTHR, PHYCR_REG);
       do {
            udelay(20);
            Status = (INL (dev, PHYCR_REG) & MIIRD);
            Loop_Count++;
            if (Loop_Count >= 100) {
                PHY_Ready = 0;
                break;
            }
       } while (Status == MIIRD);
       
       if (PHY_Ready == 0) {
           printf ("PHY NOT REDAY ");
           return 0;
       }
       Data = INL (dev, PHYDATA_REG);
       Return_Data = (Data >> 16);
      
       return Return_Data;
}


static void phy_write_register (struct eth_device* dev, u8 PHY_Register, u8 PHY_Address, u16 PHY_Data)
{
       u32 Status = 0, Loop_Count = 0, PHY_Ready = 1;

#ifdef	REALTEK_PHY_SUPPORT
       PHY_Address = 0x01;
#endif
//20us * 100 = 2ms > (1 / 2.5Mhz) * 0x34
       OUTL(dev, PHY_Data, PHYDATA_REG);
       OUTL(dev, (PHY_Register << 21) + (PHY_Address << 16) + MIIWR + MDC_CYCTHR, PHYCR_REG);
       do {
            udelay(20);
            Status = (INL (dev, PHYCR_REG) & MIIWR);
            Loop_Count++;
            if (Loop_Count >= 100) {
                PHY_Ready = 0;
                break;
            }
       } while (Status == MIIWR);
       if (PHY_Ready == 0) {
           printf ("PHY NOT REDAY ");
       }
}

static void set_mac_control_register (struct eth_device* dev)
{
	unsigned long	MAC_CR_Register = 0;
	unsigned long   Loop_Count = 0, PHY_Ready = 1, Chip_ID;
        u16    PHY_Status, PHY_Speed, PHY_Duplex, Resolved_Status = 0, Advertise, Link_Partner;

	if (CONFIG_MAC1_PHY_SETTING >= 1) {
	        MAC_CR_Register = SPEED_100M_MODE_bit | RX_BROADPKT_bit | FULLDUP_bit | RXMAC_EN_bit | RXDMA_EN_bit | TXMAC_EN_bit | TXDMA_EN_bit | CRC_APD_bit;
	}
	else {
	        MAC_CR_Register = SPEED_100M_MODE_bit | FULLDUP_bit | RXMAC_EN_bit | RXDMA_EN_bit | TXMAC_EN_bit | TXDMA_EN_bit | CRC_APD_bit;
	}

	if (CONFIG_MAC1_PHY_SETTING != 2) {
	        Chip_ID = ((phy_read_register (dev, 0x02, 0)) << 16);
	        Chip_ID |= (phy_read_register (dev, 0x03, 0) & 0xffff);
       		 if (((Chip_ID & PHYID_VENDOR_MASK) == PHYID_VENDOR_BROADCOM) ||
       		     ((Chip_ID & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8201EL)) {
	            Advertise = phy_read_register (dev, 0x04, 0);
        	    Link_Partner = phy_read_register (dev, 0x05, 0);
	            Advertise = (Advertise & PHY_SPEED_DUPLEX_MASK);
        	    Link_Partner = (Link_Partner & PHY_SPEED_DUPLEX_MASK);
	            if ((Advertise & Link_Partner) & PHY_100M_DUPLEX) {
        	        MAC_CR_Register |= SPEED_100M_MODE_bit;
	                MAC_CR_Register |= FULLDUP_bit;
        	    }
	            else if ((Advertise & Link_Partner) & PHY_100M_HALF) {
        	        MAC_CR_Register |= SPEED_100M_MODE_bit;
                	MAC_CR_Register &= ~FULLDUP_bit;
	            }
        	    else if ((Advertise & Link_Partner) & PHY_10M_DUPLEX) {
	                MAC_CR_Register &= ~SPEED_100M_MODE_bit;
	                MAC_CR_Register |= FULLDUP_bit;
        	    }
	            else if ((Advertise & Link_Partner) & PHY_10M_HALF) {
        	        MAC_CR_Register &= ~SPEED_100M_MODE_bit;
	                MAC_CR_Register &= ~FULLDUP_bit;
        	    }
	        }
	        else if (((Chip_ID & PHYID_VENDOR_MASK) == PHYID_VENDOR_MARVELL) ||
	        	 ((Chip_ID & PHYID_VENDOR_MODEL_MASK) == PHYID_RTL8211)) {
//Max waiting time = (20 + 2)ms * 250(PHY_LOOP) = 5.5s
	            do {
			udelay (20000);
        	        Resolved_Status = (phy_read_register (dev, 0x11, 0) & RESOLVED_BIT);
	                Loop_Count++;
        	        if (Loop_Count >= PHY_LOOP) {
	                    PHY_Ready = 0;
        	            printf ("PHY NOT READY ");
	                    break;
        	        }
	            } while (Resolved_Status != RESOLVED_BIT);

	            if (PHY_Ready == 1) {
        	        PHY_Status = phy_read_register (dev, 0x11, 0);
                	PHY_Speed = (PHY_Status & PHY_SPEED_MASK) >> 14;
	                PHY_Duplex = (PHY_Status & PHY_DUPLEX_MASK) >> 13;

        	        if (PHY_Speed == SPEED_1000M) {
                	    MAC_CR_Register |= GMAC_MODE_bit;
	                }
	                else {
        	            MAC_CR_Register &= ~GMAC_MODE_bit;
	                    if (PHY_Speed == SPEED_10M) {
        	                MAC_CR_Register &= ~SPEED_100M_MODE_bit;
                	    }
                	}
	                if (PHY_Duplex == DUPLEX_HALF) {
        	            MAC_CR_Register &= ~FULLDUP_bit;
                	}
	            }
//LED Control
//	            if (Chip_ID == 0x1C) {
//        	        PHY_Status = phy_read_register (dev, 0x18, 0);
//	            	phy_write_register (dev, 0x18, 0, (PHY_Status | 0x09));
//	            }
//LED Control D[0], D[6]
//	            if (Chip_ID == 0x141) {
//        	        PHY_Status = phy_read_register (dev, 0x18, 0);
//	            	phy_write_register (dev, 0x18, 0, ((PHY_Status & ~(0x41)) | 0x01));
//	            }
        	}
	}
        OUTL(dev, MAC_CR_Register, MACCR_REG);
}

#endif	/* CFG_CMD_NET && CONFIG_NET_MULTI && CONFIG_FARADAYMAC */
