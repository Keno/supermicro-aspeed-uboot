#define _KCS_C_

#include <common.h>
#include <kcs.h>

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE  1
#endif

#ifdef CONFIG_KCS_SUPPORT

#define FLASH_BASE		 FLASH_BASE_ADDR

St_BURN_IMAGE_INFO  BURN_IMAGE_INFO[MAX_IMAGE_NUM];

St_KcsVarTag at_a_St_KcsVar[TOTAL_LPC_CHANNELS];  //Should  be modify

DWORD 	dw_KCS_Addr[TOTAL_LPC_CHANNELS];
DWORD 	dw_INT_ADDR;
BYTE 	b_ReceiveDataReady = 0;
DWORD 	uiBurnAddr = 0,uiEraseSize = 0,uiImageSize = 0;
DWORD	uiCurtFWDataCnt = 0;

BYTE 	ucImageIndex = 0,ucTotalBlk = 0;
BYTE	ucImageCount = 0;

unsigned char *ucpUpdateBuf = (unsigned char *)0x41000000;

volatile unsigned char ucBurn = 0;
volatile unsigned char ucPercent = 0;

St_KCS_REG_TAG  *St_KCS;
St_KCS_IO_STATUS_REG_TAG *St_KCS_IOS[TOTAL_LPC_CHANNELS];
extern flash_info_t flash_info[];	/* info for FLASH chips */

void KCS_Potocol_Imp(BYTE b_DevID)
{
    BYTE    b_Command = 0;
    BYTE    b_Data = 0; 
    //DWORD   dw_LogicalChannel;
#ifdef DEBUG
    BYTE    i;
#endif

    /* Command in */
    if (CMD_COMMAND == PltKCS_GetStatusBit_CD(b_DevID))
    {
        switch (at_a_St_KcsVar[b_DevID].b_IsrState)
        {
            case KCS_ISR_IDLE_STATE:
            case KCS_ISR_RXDE_STATE:
            case KCS_ISR_RPS1_STATE:
            case KCS_ISR_RPS2_STATE:
            case KCS_ISR_TXD_STATE:
            case KCS_ISR_TXDE_STATE:
                /* BMC sets status to WRITE_STATE */
                PltKCS_PutStatusBit_S0S1(b_DevID, KCS_WRITE);

                /* dummy data to generate OBF (wr_start) */
                PltKCS_Put_ODR(b_DevID, KCS_ZERO_DATA);

                /* get the command byte */
                b_Command = PltKCS_Get_IDR(b_DevID);
                if (KCS_CMD_WRITE_START == b_Command)
                {
                    /* if "WRITE START" command be issued from SMS then set
                    KCS machine state back to receive data state and clear
                    receive buffer size */
                    KCS_Recv_WriteStartCmd(b_DevID);
                }
                else if (KCS_CMD_ABORT == b_Command)
                {
                    /* if "ABORT" command be issued from SMS then set BMC
                    and KCS machine state to IDLE state, and record error
                    code [01] */
            		KCS_Recv_AbortCmd(b_DevID);
                }
                else
                {
                    /* if the command is not expacted then record error
                    code [02] and go to KCS error handler sub-routine */
                    KCS_Recv_IllegalCmd(b_DevID);
                }
                break;

            case KCS_ISR_RXD_STATE:
                /* set BMC status to write state */
                PltKCS_PutStatusBit_S0S1(b_DevID, KCS_WRITE);

	            /* dummy data to generate OBF (wr_end_cmd, wr_start) */
                PltKCS_Put_ODR(b_DevID, KCS_ZERO_DATA);
                b_Command = PltKCS_Get_IDR(b_DevID);
                if (KCS_CMD_WRITE_END == b_Command)
                {
                    /* if " WRITE END" command be issued from SMS then set
                    KCS machine state to receive last byte state */
                    at_a_St_KcsVar[b_DevID].b_IsrState
                    = KCS_ISR_RXDE_STATE;
                }
                else if (KCS_CMD_WRITE_START == b_Command)
                {
                    /* if "WRITE START" command be issued from SMS then set
                    KCS machine state back to receive data state and clear
                    receive buffer size */
                    KCS_Recv_WriteStartCmd(b_DevID);
                }
                else if (KCS_CMD_ABORT == b_Command)
                {
                    /* if "ABORT" command be issued from SMS then set BMC
                    and KCS machine state to IDLE state, and record error
                    code [01] */
            	    KCS_Recv_AbortCmd(b_DevID);
                }
                else
                {
                    /* if the command is not expacted then record error
                    code [02] and go to KCS error handler sub-routine */
            	    KCS_Recv_IllegalCmd(b_DevID);
                }
                break;

	        case KCS_ISR_ERR_STATE:
                /* BMC sets status to WRITE_STATE */
                PltKCS_PutStatusBit_S0S1(b_DevID, KCS_WRITE);

                /* dummy data to generate OBF (error1) */
                PltKCS_Put_ODR(b_DevID, KCS_ZERO_DATA);

                /* get the command byte */
                b_Command = PltKCS_Get_IDR(b_DevID);
                if (KCS_CMD_GET_STATUS == b_Command)
                {
                    /* if "GET STATUS" command be issued from SMS then
                    change KCS machine state to get status phase1 state */
            	    at_a_St_KcsVar[b_DevID].b_IsrState
            	    = KCS_ISR_RPS1_STATE;
                }
                else if (KCS_CMD_WRITE_START == b_Command)
                {
                    /* if "WRITE START" command be issued from SMS then set
                    KCS machine state back to receive data state and clear
                    receive buffer size */
                    KCS_Recv_WriteStartCmd(b_DevID);
                }
                else
                {
                    /* if the command is not expacted then record error
                                code [02] and go to KCS error handler sub-routine */
                	KCS_Recv_IllegalCmd(b_DevID);
                }
	            break;

            default:
                /* if the command is not expacted then record error
                code [FF] and go to KCS error handler sub-routine */
                at_a_St_KcsVar[b_DevID].b_StatusCode
                = KCS_STATUS_UNSPECIFIED_ERROR;

                KCS_ErrorHandler(b_DevID);
                break;

        }  /* end of switch (at_a_St_KcsVar[b_DevID].b_IsrState) */
    }   /* end of if (CMD_COMMAND == PltKCS_GetStatusBit_CD()) */
    else        /* Data in */
    {
        switch (at_a_St_KcsVar[b_DevID].b_IsrState)
        {
            case KCS_ISR_RXD_STATE:
                /* BMC sets status to WRITE_STATE */
                PltKCS_PutStatusBit_S0S1(b_DevID, KCS_WRITE);

            	/* dummy data to generate OBF (wr_data)*/
                PltKCS_Put_ODR(b_DevID, KCS_ZERO_DATA);

                /* get the data byte */
                b_Data = PltKCS_Get_IDR(b_DevID);
                if (at_a_St_KcsVar[b_DevID].b_InDataSize
                    < KCS_PACKAGE_MAX_SIZE)
                {
                    /* save incoming byte from SMS */
                    at_a_St_KcsVar[b_DevID].ab_RxBuf[at_a_St_KcsVar[b_DevID].b_InDataSize++] = b_Data;
                }
	            else
	            {
                    /* if the length of incoming message over 40 bytes then
                    record error code [06] */
	            	at_a_St_KcsVar[b_DevID].b_StatusCode
	            	= KCS_STATUS_LENGTH_ERROR;

                    KCS_ErrorHandler(b_DevID);
	            }
                break;

            case KCS_ISR_RXDE_STATE:
                /* BMC sets status to READ_STATE */
                PltKCS_PutStatusBit_S0S1(b_DevID, KCS_READ);

                /* get the data byte */
                b_Data = PltKCS_Get_IDR(b_DevID);

                /* change KCS machine state to transfer data state */
                at_a_St_KcsVar[b_DevID].b_IsrState
                = KCS_ISR_TXD_STATE;

                if (at_a_St_KcsVar[b_DevID].b_InDataSize
                    < KCS_PACKAGE_MAX_SIZE)
                {

                    /* save the last incoming byte from SMS */
                    at_a_St_KcsVar[b_DevID].ab_RxBuf[at_a_St_KcsVar[b_DevID].b_InDataSize++] = b_Data;
                    b_ReceiveDataReady = 1;
#ifdef DEBUG
		            printk("Receive a KCS message - ");
		            for (i = 0; i < at_a_St_KcsVar[b_DevID].b_InDataSize; i ++)
		            {
		                printk("%x ", at_a_St_KcsVar[b_DevID].ab_RxBuf[i]);
		            }
		            printk("\n");
#endif
		            /* Send event to tell high-level firmware that KCS
		            driver has been got a incoming message from SMS */
		            at_a_St_KcsVar[b_DevID].b_BufferFullFlag = TRUE;
                }
	            else
	            {
	                /* if the length of incoming message over 40 bytes then
                       record error code [06] */
	            	at_a_St_KcsVar[b_DevID].b_StatusCode
	            	= KCS_STATUS_LENGTH_ERROR;

                    KCS_ErrorHandler(b_DevID);
	            }

                break;

            case KCS_ISR_TXD_STATE:

                /* get the data byte */
                b_Data = PltKCS_Get_IDR(b_DevID);
                if (KCS_CMD_READ == b_Data)
                {
                    if (at_a_St_KcsVar[b_DevID].b_OutDataSize > 0)
                    {
                        at_a_St_KcsVar[b_DevID].b_OutDataSize--;

                        if (0 == at_a_St_KcsVar[b_DevID].b_OutDataSize)
                        {
                            /* Transfer End */
                            at_a_St_KcsVar[b_DevID].b_IsrState
                            = KCS_ISR_TXDE_STATE;
                        }

                        /* OBF (read), put one byte of output message to
                        data out register */
                        PltKCS_Put_ODR(b_DevID,
                                         *at_a_St_KcsVar[b_DevID].pb_OutDataPtr++);
                    }
                    else
                    {
                        /* record error code [06] */
                    	at_a_St_KcsVar[b_DevID].b_StatusCode
                    	= KCS_STATUS_LENGTH_ERROR;
                        KCS_ErrorHandler(b_DevID);
                    }
                }
                else
                {
                    /* record error code [02] */
                	at_a_St_KcsVar[b_DevID].b_StatusCode
                	= KCS_STATUS_ILLEGAL_CONTROL_CODE;

                    KCS_ErrorHandler(b_DevID);
                }
                break;

            /* Transfer End */
            case KCS_ISR_TXDE_STATE:

                /* set BMC status to IDLE state */
                PltKCS_PutStatusBit_S0S1(b_DevID, KCS_IDLE);

                /* OBF (read), write a dummy byte to data out register */
                PltKCS_Put_ODR(b_DevID, KCS_ZERO_DATA);

                /* get the data byte */
                b_Data = PltKCS_Get_IDR(b_DevID);
                if (KCS_CMD_READ == b_Data)
                {
                    /* KCS machine state back to idle state */
                    at_a_St_KcsVar[b_DevID].b_IsrState
                    = KCS_ISR_IDLE_STATE;

                    /* record error code [00] */
                    at_a_St_KcsVar[b_DevID].b_StatusCode
                    = KCS_STATUS_NO_ERROR;
                }
                else
                {
                    /* record error code [02] */
                	at_a_St_KcsVar[b_DevID].b_StatusCode
                	= KCS_STATUS_ILLEGAL_CONTROL_CODE;
                    KCS_ErrorHandler(b_DevID);
                }
                break;

            case KCS_ISR_RPS1_STATE:

                /* set BMC status to read state */
                PltKCS_PutStatusBit_S0S1(b_DevID, KCS_READ);
                b_Data = PltKCS_Get_IDR(b_DevID);
                if (KCS_ZERO_DATA == b_Data)
                {
                    PltKCS_PutStatusBit_S0S1(b_DevID, KCS_READ);
                    at_a_St_KcsVar[b_DevID].b_IsrState
                    = KCS_ISR_RPS2_STATE;

                    /* OBF (error2)*/
                    PltKCS_Put_ODR(b_DevID,
                                   at_a_St_KcsVar[b_DevID].b_StatusCode);
                }
                else
                {
                	at_a_St_KcsVar[b_DevID].b_StatusCode
                	= KCS_STATUS_ILLEGAL_CONTROL_CODE;
                    KCS_ErrorHandler(b_DevID);
                }
                break;

            case KCS_ISR_RPS2_STATE:
                PltKCS_PutStatusBit_S0S1(b_DevID, KCS_IDLE);

                /* dummy data to generate OBF (error3)*/
                PltKCS_Put_ODR(b_DevID, KCS_ZERO_DATA);
                b_Data = PltKCS_Get_IDR(b_DevID);
                if (KCS_CMD_READ == b_Data)
                {
                    at_a_St_KcsVar[b_DevID].b_StatusCode
                    = KCS_STATUS_NO_ERROR;
                    at_a_St_KcsVar[b_DevID].b_IsrState
                    = KCS_ISR_IDLE_STATE;
                }
                else
                {
                	at_a_St_KcsVar[b_DevID].b_StatusCode
                	= KCS_STATUS_ILLEGAL_CONTROL_CODE;
                    KCS_ErrorHandler(b_DevID);
                }
                break;

            default:
            	at_a_St_KcsVar[b_DevID].b_StatusCode
            	= KCS_STATUS_UNSPECIFIED_ERROR;
                KCS_ErrorHandler(b_DevID);
                break;

        }  /* end of switch (at_a_St_KcsVar[b_DevID].b_IsrState) */
    }   /* end of else */
}

int flash_raw_write(void * srcAddress, void * destAddress, int size, unsigned char uCountSep, unsigned char image_index)
{
	flash_info_t *info = &flash_info[0];
	int fileSize;
	unsigned int src,dest;
	ulong	erase_start,erase_end;
	int i,blockSize, write_size;
	unsigned char ucCrtCount = 0;
	blockSize = (info->size)/(info->sector_count);
	fileSize=size;
	
	// Write program
	//i = (fileSize) + (sizeof(unsigned int)-(fileSize%sizeof(unsigned int)));//word aligment
	i = (fileSize);

	src=(unsigned int)srcAddress;
	dest=(unsigned int)destAddress;

	while(i)
	{
//		printf ("flash_raw_write: ucCrtCount=%d, src=0x%x, dest=0x%x, uCountSep=0x%x, i=0x%x, !! \n",
//				 ucCrtCount, src, dest, uCountSep, i);
		
		ucCrtCount++;
		
		disable_interrupts ();
		flash_sect_erase((ulong)dest, (ulong)(dest + blockSize - 1));
		if( i < blockSize )
			write_size = i;	//blockSize=i; // Check if > a block size
		else
			write_size = blockSize;
		
		flash_write ((char *)src, (ulong)dest, (ulong)write_size);
		
		enable_interrupts ();
		udelay (10);
		
//		printf ("flash_raw_write over: ucCrtCount = %d, ucPercent = %d !! \n", ucCrtCount, ucPercent);
		
		src+=write_size;
		dest+=write_size;
		i-=write_size;
		
		ucPercent = ((ucCrtCount*100)/(uCountSep*ucTotalBlk)) + (100/ucTotalBlk)*image_index;
//		if(ucCrtCount%uCountSep!=0)
//			ucPercent++;
	}
//	uprintf(" OK!\n");
//	uprintf("Verifing ");
//	// Verify program
//	i=(fileSize&(~0x3))+0x4; //word aligment
//	src=(unsigned int)srcAddress;
//	dest=(unsigned int)destAddress;
//	blockSize=FlashBlockSize((unsigned int)srcAddress);
//	blockSize=blockSize-1;
//	while(i)
//	{
//		if( (i&blockSize)==0x0 )uputchar('.');
//		if( *((volatile unsigned int *)src) != *((volatile unsigned int *)dest) )
//		{
//			uprintf("\nERROR: A:0x%08x W:0x%08x R:0x%08x!!\n",dest,*((volatile unsigned int *)src),*((volatile unsigned int*)dest));
//	//		return -1;
//		}
//		src+=4;
//		dest+=4;
//		i-=4;
//	}
//	printf(" OK!\n");	
//	printf("Programming finished!!\n");	
	return 0;
}

void SendBlockInfo(BYTE b_KcsNo)
{   
	unsigned int uiExecuteAddr = 0;

	uiBurnAddr = at_a_St_KcsVar[b_KcsNo].ab_RxBuf[2];    
    uiBurnAddr = (uiBurnAddr<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[3];
    uiBurnAddr = (uiBurnAddr<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[4];
    uiBurnAddr = (uiBurnAddr<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[5];
    uiBurnAddr = uiBurnAddr;
    //uiBurnAddr = uiBurnAddr;
    
	uiEraseSize = at_a_St_KcsVar[b_KcsNo].ab_RxBuf[6];    
    uiEraseSize = (uiEraseSize<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[7];
    uiEraseSize = (uiEraseSize<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[8];
    uiEraseSize = (uiEraseSize<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[9];
    
	uiImageSize = at_a_St_KcsVar[b_KcsNo].ab_RxBuf[10];    
    uiImageSize = (uiImageSize<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[11];
    uiImageSize = (uiImageSize<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[12];
    uiImageSize = (uiImageSize<<8) + at_a_St_KcsVar[b_KcsNo].ab_RxBuf[13];
    
    ucImageIndex = at_a_St_KcsVar[b_KcsNo].ab_RxBuf[14];
    ucTotalBlk = at_a_St_KcsVar[b_KcsNo].ab_RxBuf[15];

	printf ("[image %lx] uiBurnAddr %lx uiEraseSize %lx uiImageSize %lx\n",ucImageIndex,uiBurnAddr,uiEraseSize,uiImageSize);
	BURN_IMAGE_INFO[ucImageIndex].dw_BurnAddr = uiBurnAddr;
	BURN_IMAGE_INFO[ucImageIndex].dw_EraseSize = uiEraseSize;
	BURN_IMAGE_INFO[ucImageIndex].dw_ImageSize = uiImageSize;

	at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 3;  
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[2] = 0;
} 

void ReceiveFWData(BYTE b_KcsNo)
{    
	int i =0;
	unsigned char ucSize = at_a_St_KcsVar[b_KcsNo].b_InDataSize-3;

	for(i=0;i<ucSize;i++)
	{
		*(ucpUpdateBuf + uiCurtFWDataCnt + uiBurnAddr + i) = at_a_St_KcsVar[b_KcsNo].ab_RxBuf[2+i+1];
	} 
	uiCurtFWDataCnt += ucSize;   
	if(at_a_St_KcsVar[b_KcsNo].ab_RxBuf[2]==1)
	{
		ucImageCount++;
		if(uiCurtFWDataCnt == uiImageSize)
		{
			uiCurtFWDataCnt = 0;
			if(ucImageCount==ucTotalBlk)
			{					
				ucBurn = 1;
				ucImageCount = 0;
			//	printf ("ReceiveFWData: ucBurn=%d !! \n", ucBurn);
			}
		}
	}
	at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 3;
	at_a_St_KcsVar[b_KcsNo].ab_RxBuf[2] = 0;      
} 

void GetFWUpdateInfo(BYTE b_KcsNo)
{  
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[2] = 0;
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[3] = UBOOT2;
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[4] = (unsigned char)(FLASH_BASE >>24); 		
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[5] = (unsigned char)(FLASH_BASE >>16); 	
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[6] = (unsigned char)(FLASH_BASE >>8); 	
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[7] = (unsigned char)(FLASH_BASE); 
	memcpy(ucpUpdateBuf,at_a_St_KcsVar[b_KcsNo].ab_TxBuf,7);		
	at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 8;  
} 

void GetFWUpdateStatus(BYTE b_KcsNo)
{  
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[2] = 0;
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[3] = ucPercent;
	at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 4;  
	printf ("GetFWUpdateStatus : ucPercent = %d !! \n", ucPercent);
}

BYTE CmdParser(BYTE  b_KcsNo)
{
	BYTE i =0;
	switch (at_a_St_KcsVar[b_KcsNo].ab_RxBuf[0]>>2)
	{
		case NET_FN_APP:
		{
			switch (at_a_St_KcsVar[b_KcsNo].ab_RxBuf[1])
			{
				case 0x01:	//Get Device ID
					/* copy data */
				    for (i = 0; i < 20; i ++)
				    {
				        at_a_St_KcsVar[b_KcsNo].ab_TxBuf[i] = i;
				    }
				    at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 20;
				    break;
				case 0x02:	//Cold Reset
					at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 3;
					at_a_St_KcsVar[b_KcsNo].ab_TxBuf[2] = 0;
					//reboot();
					break;
				default:
					at_a_St_KcsVar[b_KcsNo].ab_TxBuf[2] = 0xc1;
					at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 3;
					break;
			}	
			break;	
		}
						
		case NET_FN_OEM_WST:
		{
			switch (at_a_St_KcsVar[b_KcsNo].ab_RxBuf[1])
			{
				case 0x14:	//SendBlockInfo
				{
					SendBlockInfo(b_KcsNo);
					break;					
				}					
				case 0x15:	//ReceiveFWData
				{
					ReceiveFWData(b_KcsNo);
					break;				
				}
				case 0x16:	//GetFWUpdateInfo
				{
					GetFWUpdateInfo(b_KcsNo);
					break;					
				}
				case 0x17:	//GetFWUpdateStatus
				{
					GetFWUpdateStatus(b_KcsNo);
					break;					
				}					
				default:
				{
					at_a_St_KcsVar[b_KcsNo].ab_TxBuf[2] = 0xc1;
					at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 3;
					break;					
				}		
			}
			break;	
		}
		
		default:
		{
			at_a_St_KcsVar[b_KcsNo].ab_TxBuf[2] = 0xc1;
			at_a_St_KcsVar[b_KcsNo].b_OutDataSize = 3;	
			break;	
		}
	}
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[0] = 0;
	at_a_St_KcsVar[b_KcsNo].ab_TxBuf[1] = at_a_St_KcsVar[b_KcsNo].ab_RxBuf[1];
	
	return 1;	
}

void KCSIsr(void)
{
    BYTE b_KcsNo;

    St_KCS->HICR2.BIT.LRST = 0;
    St_KCS->HICR2.BIT.SDWN = 0;
    St_KCS->HICR2.BIT.ABRT = 0;

	//printf ("Enter KCSIsr !! \n");

    for (b_KcsNo = 0; b_KcsNo < 3; b_KcsNo++)
    {
        /* Look-up which KCS channel generate a interrupt */
        if (PltKCS_ReadIntrptSrc(b_KcsNo) == TRUE)
        {
            KCS_Potocol_Imp(b_KcsNo);
            
            /* Handle Command Here */
            if(b_ReceiveDataReady== 1)//mean read a command complete
			{
				CmdParser(b_KcsNo);

			    at_a_St_KcsVar[b_KcsNo].pb_OutDataPtr = at_a_St_KcsVar[b_KcsNo].ab_TxBuf;
			
			    at_a_St_KcsVar[b_KcsNo].b_OutDataSize--;
			    b_ReceiveDataReady = 0;
			    PltKCS_Put_ODR(b_KcsNo, *at_a_St_KcsVar[b_KcsNo].pb_OutDataPtr ++);
            }            
        }
    }
    
	/* clear edge-triggered interrupt */
	*((DWORD *)(dw_INT_ADDR + INTC_VIC38)) = (1 << LPC_IRQ);

    return;
}

void KCSinit(int b_ChNo,unsigned long int dw_DataAddr)
{
//	unsigned short int w_DataAddr = (unsigned short int) KCS_PHY_ADDR;

	SetISR(LPC_IRQ, KCSIsr);
	St_KCS = (St_KCS_REG_TAG *)(LPC_ADDR);
	
	dw_INT_ADDR = (u32)(INT_CTL);
	
    St_KCS_IOS[0] = (St_KCS_IO_STATUS_REG_TAG *) (&(St_KCS->IDR1));
    St_KCS_IOS[1] = (St_KCS_IO_STATUS_REG_TAG *) (&(St_KCS->IDR2));
    St_KCS_IOS[2] = (St_KCS_IO_STATUS_REG_TAG *) (&(St_KCS->IDR3));	
    
    /* set HIF state machine to idle state */
    at_a_St_KcsVar[b_ChNo].b_IsrState = KCS_ISR_IDLE_STATE;

    /* clear error code */
    at_a_St_KcsVar[b_ChNo].b_StatusCode = KCS_STATUS_NO_ERROR;

    /* init sw information */
    at_a_St_KcsVar[b_ChNo].b_InitFlag = TRUE;
    at_a_St_KcsVar[b_ChNo].b_BufferFullFlag = FALSE;    
	
    /* set the isr as level sensitive */
    *((DWORD *) (dw_INT_ADDR + INTC_VIC24)) |= (1 << LPC_IRQ);
    /* set the isr as high level trigger */
    *((DWORD *) (dw_INT_ADDR + INTC_VIC2C)) |= (1 << LPC_IRQ);
    /* set the isr as IRQ interrupt */
    *((DWORD *) (dw_INT_ADDR + INTC_VIC0C)) &= (~(1 << LPC_IRQ));
	/* clear edge-triggered interrupt */
	*((DWORD *)(dw_INT_ADDR + INTC_VIC38)) |= (1 << LPC_IRQ);
    //printk("kcs_drv.c: kcs_init - *((DWORD *) (dw_INT_ADDR + INTC_VIC38)) = 0x%8x.\n", *((DWORD *) (dw_INT_ADDR + INTC_VIC38)));
    
    /* disable IBFIE */
    Plt_IBFIECtl(b_ChNo, 0);
         
    /* disable LPC Channel */
    Plt_LPCEnlCtl(b_ChNo, 0);

    /* clear OBF & dummy read */
    PltKCS_MiscInit(b_ChNo);

    /* enable IBFIE */
    Plt_IBFIECtl(b_ChNo, 1);
          
    /* enable LPC Channel */
    Plt_LPCEnlCtl(b_ChNo, 1);

    /* set KCS initial state */
    PltKCS_PutStatusBit_S0S1(b_ChNo, 3);

    /* Save KCS base address and offset */
    at_a_St_KcsVar[b_ChNo].dW_Addr = dw_DataAddr;
    
    dw_KCS_Addr[b_ChNo] = dw_DataAddr;
    
	/* write KCS address */
	PltKCS_Put_ADR(b_ChNo, dw_KCS_Addr[b_ChNo]);

	/* enable LPC isr */
	*((DWORD *)(dw_INT_ADDR + INTC_VIC10)) |= (1 << LPC_IRQ);
	
}

void Plt_IBFIECtl(BYTE b_ChNo, BYTE b_Ctl)
{
    if (0 == b_Ctl)
    {
        St_KCS->HICR2.DWORD &= (DWORD) (~(0x02 << b_ChNo));
    }
    else if (1 == b_Ctl)
    {
        St_KCS->HICR2.DWORD |= (DWORD) (0x02 << b_ChNo);
    }
}

void Plt_LPCEnlCtl(BYTE b_ChNo, BYTE b_Ctl)
{
    if (0 == b_Ctl)
    {
        St_KCS->HICR0.DWORD &= (DWORD) (~(0x20 << b_ChNo));
    }
    else if (1 == b_Ctl)
    {
        St_KCS->HICR0.DWORD |= (DWORD) (0x20 << b_ChNo);
    }
}

void PltKCS_Put_ADR(BYTE b_KcsNo, DWORD dw_Addr)
{
    BYTE 	b_AddrHi = (BYTE) (dw_Addr >> 24);
    BYTE 	b_AddrLo = (BYTE) (dw_Addr >> 16);
	WORD 	w_Addr;
	DWORD	dw_Data;
    
    switch (b_KcsNo)
    {
        case 0:
			dw_Data  					= 	*(DWORD *) KCSHICRB;
			dw_Data 					|=	0x10;
			*(DWORD *) KCSHICRB 			=	dw_Data;
            St_KCS->HICR4.BIT.LADR12SEL = 	0;
            St_KCS->LADR12H.BIT.ADDR 	= 	b_AddrHi;
            St_KCS->LADR12L.BIT.ADDR 	= 	b_AddrLo;
			
			b_AddrHi 					=	(BYTE) (dw_Addr >>8);
			b_AddrLo 					=	(BYTE) (dw_Addr);
			w_Addr						=   (b_AddrHi << 8) | (b_AddrLo);
			
			dw_Data						=   *(DWORD *)KCSLSADR1;
			dw_Data						&=  0xffff0000;
			dw_Data						|=  w_Addr;
			*(DWORD *)KCSLSADR1			=	dw_Data;	


            break;
       case 1:
            St_KCS->HICR4.BIT.LADR12SEL = 1;
            St_KCS->LADR12H.BIT.ADDR = b_AddrHi;
            St_KCS->LADR12L.BIT.ADDR = b_AddrLo;
            break;
       case 2:
			St_KCS->HICR4.BIT.KCSENBL	= 1;
            St_KCS->LADR3H.BIT.ADDR = b_AddrHi;
            St_KCS->LADR3L.BIT.ADDR = b_AddrLo;
            break;          
	   case 3:
			w_Addr		=	(b_AddrHi << 8) | b_AddrLo;
			*(DWORD *) KCSLADR4   = (WORD) w_Addr;

			b_AddrHi 	= 	(BYTE)(dw_Addr >> 8);
			b_AddrLo 	= 	(BYTE)dw_Addr;
			w_Addr		=	(b_AddrHi << 8) | b_AddrLo;
			*(DWORD *) (KCSLADR4 + 2)   = (WORD) w_Addr;

			break;

    }    
}

void PltKCS_MiscInit(BYTE b_KcsNo)
{
    BYTE b_Dummy;

    /* Clear output buffer full bit */
    St_KCS_IOS[b_KcsNo]->STR.BIT.OBF = 0;
    /* Dummy read IDR register */
    b_Dummy = St_KCS_IOS[b_KcsNo]->IDR.BIT.DATA;
}

void PltKCS_PutStatusBit_S0S1(BYTE b_KcsNo, BYTE b_Status)
{
    St_KCS_IOS[b_KcsNo]->STR.BIT.SOS = b_Status;
}    

BYTE PltKCS_ReadIntrptSrc(BYTE b_KcsNo)
{
    if (St_KCS_IOS[b_KcsNo]->STR.BIT.IBF)
        return (TRUE);
    else
        return (FALSE);
}

BYTE PltKCS_GetStatusBit_CD(BYTE b_KcsNo)
{
    return (St_KCS_IOS[b_KcsNo]->STR.BIT.CD);
}

void PltKCS_Put_ODR(BYTE b_KcsNo, BYTE b_Data)
{
    St_KCS_IOS[b_KcsNo]->ODR.BIT.DATA = b_Data;
}

BYTE PltKCS_Get_IDR(BYTE b_KcsNo)
{
    return (St_KCS_IOS[b_KcsNo]->IDR.BIT.DATA);
}

void KCS_Recv_WriteStartCmd(BYTE b_DevID)
{
    at_a_St_KcsVar[b_DevID].b_IsrState
    = KCS_ISR_RXD_STATE;

    at_a_St_KcsVar[b_DevID].b_InDataSize = 0;

}

void KCS_Recv_AbortCmd(BYTE b_DevID)
{
    PltKCS_PutStatusBit_S0S1(b_DevID, KCS_IDLE);
    at_a_St_KcsVar[b_DevID].b_IsrState
    = KCS_ISR_IDLE_STATE;

	at_a_St_KcsVar[b_DevID].b_StatusCode
	= KCS_STATUS_ABORTED_BY_COMMAND;
}

void KCS_Recv_IllegalCmd(BYTE b_DevID)
{
    at_a_St_KcsVar[b_DevID].b_StatusCode
    = KCS_STATUS_ILLEGAL_CONTROL_CODE;

    KCS_ErrorHandler(b_DevID);
}

void KCS_ErrorHandler(
                       /** Channel number */
                       BYTE  b_DevID
                     )
{
    BYTE    b_DummyRead;

    /* do dummy reading data in register for clearing interrupt flag */
    b_DummyRead = PltKCS_Get_IDR(b_DevID);

    /* set BMC state to ERROR state */
    PltKCS_PutStatusBit_S0S1(b_DevID, KCS_ERROR);
    at_a_St_KcsVar[b_DevID].b_IsrState = KCS_ISR_ERR_STATE;
}



#endif
