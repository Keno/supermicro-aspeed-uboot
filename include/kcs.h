#ifndef _KCS_H_
#define _KCS_H_

#define MAX_IMAGE_NUM    30
typedef struct 
{
    unsigned int	dw_BurnAddr;
    unsigned int	dw_EraseSize;
    unsigned int	dw_ImageSize;
    unsigned char	*p_Image;
} St_BURN_IMAGE_INFO;

typedef unsigned char       BYTE;
typedef unsigned short int  WORD;
typedef unsigned long int   DWORD;

/******************************************************************************
*   ENUM        :   e_KCS_ISR_STATE_tag
******************************************************************************/
/**
 *  DECRIPTION  :   The type state of KCS state machine
 *
 *****************************************************************************/
typedef enum 
{
    KCS_ISR_IDLE_STATE = 0,
    KCS_ISR_RXD_STATE,
    KCS_ISR_RXDE_STATE,
    KCS_ISR_TXD_STATE,
    KCS_ISR_TXDE_STATE,
    KCS_ISR_RPS1_STATE,
    KCS_ISR_RPS2_STATE,
    KCS_ISR_ERR_STATE
} KCS_ISR_STATE_tag;


#ifdef _KCS_C_

/*Definition for BTL type*/
#define UBOOT 0
#define WBL 1
#define UBOOT2 2

#define NULL 0

#define NET_FN_CHASSIS          0x00
#define NET_FN_BRIDGE           0x02
#define NET_FN_SE               0x04 
#define NET_FN_APP              0x06
#define NET_FN_FW               0x08
#define NET_FN_STORAGE          0x0A 
#define NET_FN_TRANSPORT        0x0C
#define NET_FN_OEM_WST          0x3E
#define NET_FN_OEM_WST1			0x30

/* KCS interface MISC definition */
#define KCS_ZERO_DATA           0
//#define KCS_PACKAGE_MAX_SIZE    40
#define KCS_PACKAGE_MAX_SIZE    64
#define KCS_SET_ATTEN_BIT       0
/* KCS interface status register bits definition */
#define CMD_DATA            0
#define CMD_COMMAND         1
#define SMS_ATN_SET         1
#define SMS_ATN_CLEAR       0

/* KCS interface state definition */
#define KCS_IDLE      0
#define KCS_READ      1
#define KCS_WRITE     2
#define KCS_ERROR     3

/* KCS interface control codes definition */
#define KCS_CMD_ABORT          0x60
#define KCS_CMD_GET_STATUS     0x60
#define KCS_CMD_WRITE_START    0x61
#define KCS_CMD_WRITE_END      0x62
#define KCS_CMD_READ           0x68

/* KCS interface status codes definition */
#define KCS_STATUS_NO_ERROR                0x0
#define KCS_STATUS_ABORTED_BY_COMMAND      0x01
#define KCS_STATUS_ILLEGAL_CONTROL_CODE    0x02
#define KCS_STATUS_LENGTH_ERROR            0x06
#define KCS_STATUS_UNSPECIFIED_ERROR       0xFF

#define LPC_IRQ                                                          8
#define TOTAL_LPC_CHANNELS                                               3

#define INTC_VIC00        0x00   /** IRQ Status Register */
#define INTC_VIC04        0x04   /** FIQ Status Register */
#define INTC_VIC08        0x08   /** Raw Interrupt Status Register */
#define INTC_VIC0C        0x0C   /** Interrupt Selection Register */
#define INTC_VIC10        0x10   /** Interrupt Enable Register */
#define INTC_VIC14        0x14   /** Interrupt Enable Clear Register */
#define INTC_VIC18        0x18   /** Software Interrupt Register */
#define INTC_VIC1C        0x1C   /** Software Interrupt Clear Register */
#define INTC_VIC20        0x20   /** Protection Enable Register */
#define INTC_VIC24        0x24   /** Interrupt Sensitivity Register */
#define INTC_VIC28        0x28   /** Interrupt Both Edge Trigger Control Register */
#define INTC_VIC2C        0x2C   /** Interrupt Event Register */
#define INTC_VIC38        0x38   /** Edge-Triggered Interrupt Clear Register */

#define CLR_INT14_STATUS		    0x00004000

#define LPC_ADDR                    0x1E789000

#define SIZE_OF_SCU1                    0x0400
#define INT_CTL                     0x1E6C0000

#define KCS_PHY_CHN_NUM				2
#define KCS_PACKAGE_MAX_SIZE    	80

#define KCSSNPWDR			(LPC_ADDR + 0x94)  // KCS LPC Snoop Data Register
#define KCSHICRB			(LPC_ADDR + 0x100)  // Host Interface Control Register B
#define KCSLADR4			(LPC_ADDR + 0x110)  // LPC Channel #4 Address register
#define KCSLSADR1			(LPC_ADDR + 0x120)  // LPC Channel #1 KCS Status Address register


/******************************************************************************
*                   KCS parameters definition for memory allocation
******************************************************************************/
typedef struct
{
    BYTE    b_InitFlag;
    BYTE    b_IsrState;
    BYTE    ab_RxBuf[KCS_PACKAGE_MAX_SIZE];
    BYTE    ab_TxBuf[KCS_PACKAGE_MAX_SIZE];
    BYTE    b_InDataSize;
    BYTE    *pb_OutDataPtr;
    BYTE    b_OutDataSize;
    BYTE    b_StatusCode;
    BYTE    b_LogChanlID;
    BYTE    b_BufferFullFlag;
    DWORD   dW_Addr;
    //TX_QUEUE	*p_ChanlMsgQueGrp;
}St_KcsVarTag;

typedef struct St_KCS_REG
{
    	union                                    // HICR0 offset : 0x00
    	{
            DWORD    DWORD;
            struct 
            {                             
                unsigned char LSCIE:1;           /*              */
                unsigned char LSMIE:1;           /*              */
                unsigned char PMEE:1;            /*              */
                unsigned char SDWNE:1;           /*              */
                unsigned char FGA20E:1;          /*              */
                unsigned char LPC1E:1;           /*              */
                unsigned char LPC2E:1;           /*              */
                unsigned char LPC3E:1;           /*              */
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;          
		}HICR0;

        union                                    // HICR1 offset : 0x04
        {
            DWORD   DWORD;
            struct 
            {                             
                unsigned char LSCIB:1;           /*              */
                unsigned char LSMIB:1;           /*              */
                unsigned char PMEB:1;            /*              */
                unsigned char SDWNB:1;           /*              */
                unsigned char LRSTB:1;           /*              */
                unsigned char IRQBSY:1;          /*              */
                unsigned char CLKREQ:1;          /*              */
                unsigned char LPCBSY:1;          /*              */
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;             
        }HICR1;

        union                                    // HICR2 offset : 0x08
        {
            DWORD   DWORD;
            struct 
            {
                unsigned char ERRIE:1;           /*              */
                unsigned char IBFIE1:1;          /*              */
                unsigned char IBFIE2:1;          /*              */
                unsigned char IBFIE3:1;          /*              */
                unsigned char ABRT:1;            /*              */
                unsigned char SDWN:1;            /*              */
                unsigned char LRST:1;            /*              */
                unsigned char GA20:1;            /*              */
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;           
        }HICR2;

        union                                    // HICR3 offset : 0x0C
        {
            DWORD   DWORD;
            struct 
            {
                unsigned char LSCI:1;            /*              */
                unsigned char LSMI:1;            /*              */
                unsigned char PME:1;             /*              */
                unsigned char LPCPD:1;           /*              */
                unsigned char LRESET:1;          /*              */
                unsigned char SERIRQ:1;          /*              */
                unsigned char CLKRUN:1;          /*              */
                unsigned char LFRAME:1;          /*              */
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;              
        }HICR3;

        union                                    // HICR4 offset : 0x10
        {
            DWORD   DWORD;
            struct 
            {
                unsigned char BTENBL:1;          /*              */
                unsigned char SMICENBL:1;        /*              */
                unsigned char KCSENBL:1;         /*              */
                unsigned char SWENBL:1;          /*              */
                unsigned char wk:3;              /*              */
                unsigned char LADR12SEL:1;       /*              */
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;
        }HICR4;             

        union                                  // LADR3H offset : 0x14
        {
            DWORD   DWORD;                  /*              */
            struct 
            {
                unsigned char ADDR:8;            /*              */
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;                      
        }LADR3H;
        

        union                                  /* LADR3L offset : 0x18       */
        {
            DWORD   DWORD;  
            struct 
            {
                unsigned char ADDR:8;     
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;           
        }LADR3L;   
        
        union                                  /* LADR12H offset : 0x1C       */
        {
            DWORD   DWORD;
            struct 
            {
                unsigned char ADDR:8;            /*              */
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;           
        }LADR12H;        
        
        union                                   /* LADR12L offset : 0x20       */
        {
            DWORD   DWORD;
            struct 
            {
                unsigned char ADDR:8;            /*              */
		        unsigned char Reserved0:8;       /*  Bit  Access */
		        unsigned char Reserved1:8;       /*  Bit  Access */
		        unsigned char Reserved2:8;       /*  Bit  Access */
            }BIT;
        }LADR12L;        
        
        DWORD IDR1;                       /*  offset : 0x24  */
        DWORD IDR2;                       /*  offset : 0x28  */
        DWORD IDR3;                       /*  offset : 0x2C  */
        DWORD ODR1;                       /*  offset : 0x30  */
        DWORD ODR2;                       /*  offset : 0x34  */
        DWORD ODR3;                       /*  offset : 0x38  */
        DWORD STR1;                       /*  offset : 0x3c  */
        DWORD STR2;                       /*  offset : 0x40  */
        DWORD STR3;                       /*  offset : 0x44  */
        
        
        DWORD BTSR0;                      /*  offset : 0x48  */
        DWORD BTSR1;                      /*  offset : 0x4c  */
        DWORD BTCSR0;                     /*  offset : 0x50  */
        DWORD BTCSR1;                     /*  offset : 0x54  */
        DWORD BTCR;                       /*  offset : 0x58  */
        DWORD BTDTR;                      /*  offset : 0x5c  */
        DWORD BTIMSR;                     /*  offset : 0x60  */
        DWORD BTFVSR0;                    /*  offset : 0x64  */
        DWORD BTFVSR1;                    /*  offset : 0x68  */
}St_KCS_REG_TAG;

typedef struct St_KCS_IO_STATUS_REG
{
    union   //IDR
    {
        DWORD    DWORD;
        struct 
        {                             
            unsigned char DATA:8;
	        unsigned char Reserved0:8;       /*  Bit  Access */
	        unsigned char Reserved1:8;       /*  Bit  Access */
	        unsigned char Reserved2:8;       /*  Bit  Access */
        }BIT;
	}IDR;

    DWORD   IDRReserved0;
    DWORD   IDRReserved1;

    union   //ODR
    {
        DWORD    DWORD;
        struct
        {
            unsigned char DATA:8;
	        unsigned char Reserved0:8;       /*  Bit  Access */
	        unsigned char Reserved1:8;       /*  Bit  Access */
	        unsigned char Reserved2:8;       /*  Bit  Access */
        }BIT;
	}ODR;

    DWORD   ODRReserved0;
    DWORD   ODRReserved1;

    union   //STR
    {
        DWORD    DWORD;
        struct
        {
            unsigned char OBF:1;
            unsigned char IBF:1;
            unsigned char SMS_ATN:1;
            unsigned char CD:1;
            unsigned char DBU14:1;
            unsigned char DBU15:1;
            unsigned char SOS:2;
	        unsigned char Reserved0:8;       /*  Bit  Access */
	        unsigned char Reserved1:8;       /*  Bit  Access */
	        unsigned char Reserved2:8;       /*  Bit  Access */
        }BIT;
	}STR;

}St_KCS_IO_STATUS_REG_TAG;


void KCSinit(int, unsigned long int);

void KCSIsr(void);
void Plt_IBFIECtl(BYTE b_ChNo, BYTE b_Ctl);
void Plt_LPCEnlCtl(BYTE b_ChNo, BYTE b_Ctl);
void PltKCS_Put_ADR(BYTE b_KcsNo, DWORD dw_Addr);
void PltKCS_MiscInit(BYTE b_KcsNo);
void PltKCS_PutStatusBit_S0S1(BYTE b_KcsNo, BYTE b_Status);
BYTE PltKCS_ReadIntrptSrc(BYTE b_KcsNo);
void KCS_Potocol_Imp(BYTE b_DevID);
BYTE PltKCS_GetStatusBit_CD(BYTE b_KcsNo);
void PltKCS_Put_ODR(BYTE b_KcsNo, BYTE b_Data);
BYTE PltKCS_Get_IDR(BYTE b_KcsNo);
void KCS_Recv_WriteStartCmd(BYTE b_DevID);
void KCS_Recv_AbortCmd(BYTE b_DevID);
void KCS_Recv_IllegalCmd(BYTE b_DevID);
void KCS_ErrorHandler(BYTE  b_DevID);

BYTE CmdParser(BYTE  b_KcsNo);
void SendBlockInfo(BYTE b_KcsNo);
void ReceiveFWData(BYTE b_KcsNo);
void GetFWUpdateInfo(BYTE b_KcsNo);
void GetFWUpdateStatus(BYTE b_KcsNo);
extern int flash_sect_erase (ulong addr_first, ulong addr_last);

#else
extern unsigned char ucTotalBlk;
extern volatile unsigned char ucBurn;
extern volatile unsigned char ucPercent;
extern unsigned char *ucpUpdateBuf;
extern St_BURN_IMAGE_INFO  BURN_IMAGE_INFO[MAX_IMAGE_NUM];

extern void KCSinit(int, unsigned long int);
extern int flash_raw_write(void * srcAddress, void * destAddress, int size, unsigned char uCountSep, unsigned char image_index);

#endif  // End of #ifdef _KCS_C_  
#endif  // End of #define _KCS_H_
