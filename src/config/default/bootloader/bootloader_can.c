/*******************************************************************************
  CAN Bootloader Source File

  File Name:
    bootloader_can.c

  Summary:
    This file contains source code necessary to execute CAN bootloader.

  Description:
    This file contains source code necessary to execute CAN bootloader.
    It implements bootloader protocol which uses CAN peripheral to download
    application firmware into internal flash from HOST.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************
#include "../src/config/default/bootloader/bootloader_can.h"
#include "definitions.h"
#include "bootloader_common.h"
//#include "bootloader_can.h"
//#include "GenericTypeDefs.h"
#include "../src/config/default/GenericTypeDefs.h" 
#include "../peripheral/uart/plib_uart4.h"
#include <device.h>
void WriteHexRecord2Flash(uint8_t* HexRecord, UINT totalHexRecLen);
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

#define ADDR_OFFSET              1
#define SIZE_OFFSET              1
#define CRC_OFFSET               1

#define HEADER_CMD_OFFSET        0
#define HEADER_SEQ_OFFSET        1
#define HEADER_MAGIC_OFFSET      2
#define HEADER_SIZE_OFFSET       3

#define CRC_SIZE                 4
#define HEADER_SIZE              4
#define OFFSET_SIZE              4
#define SIZE_SIZE                4
#define MAX_DATA_SIZE            60


#define HEADER_MAGIC             0xE2
#define CAN_FILTER_ID            (0x190)  //0x45A

/* Standard identifier id[28:18]*/
#define WRITE_ID(id)             (id << 18U)
#define READ_ID(id)              (id >> 18U)

#define WORDS(x)                 ((int)((x) / sizeof(uint32_t)))

#define OFFSET_ALIGN_MASK        (~ERASE_BLOCK_SIZE + 1)
#define SIZE_ALIGN_MASK          (~PAGE_SIZE + 1)

/* Compare Value to achieve a 100Ms Delay */
#define TIMER_COMPARE_VALUE     (CORE_TIMER_FREQUENCY / 10)

/* CAN Tx FIFO size */
#define CAN_TX_FIFO_BUFFER_SIZE             16U
#define CAN_MSG_LENG                        8
#define TX_BUFF_SIZE                        20
#define RESET                               0
#define FRAMEWORK_BUFF_SIZE					1000
#define CMD_ID_TX_LEN                       1
#define BOARD_INFO_CMD                      0xAA
#define BOARD_SW_BTL_CMD                    0xAB
#define BTL_START_BYTE                      0x01
#define BTL_DLE_BYTE                        0x10
#define BTL_END_BYTE                        0x04


static BOOL RunApplication = FALSE;
DWORD HexFileStatus = 0xffff;
TxMsgs TxMessage;
TxMsgs * p_Transmit;	
typedef struct 
{
	UINT8 RecDataLen;
	DWORD_VAL Address;
	UINT8 RecType;
	UINT8* Data;
	UINT8 CheckSum;	
	DWORD_VAL ExtSegAddress;
	DWORD_VAL ExtLinAddress;
}T_HEX_RECORD;

enum
{
    BL_CMD_UNLOCK       = 0xA0,
    BL_CMD_DATA         = 0xA1,
    BL_CMD_VERIFY       = 0xA2,
    BL_CMD_RESET        = 0xA3,
    BL_CMD_READ_VERSION = 0xA6,
};

enum
{
    BTL_CMD_READ_VERSION       = 0x01,
    BTL_CMD_ERASE              = 0x02,
    BTL_CMD_PROGRAM            = 0x03,
    BTL_CMD_RUN                = 0x05,
    BTL_COMPORT                = 0x06,
    BTL_HEX_LOAD_FINISH        = 0x07, 
    BTL_CMD_CCU_BOARD          = 0xB5,
    
}e_OLD_BTL_CMD;

enum
{
    BL_RESP_OK          = 0x50,
    BL_RESP_ERROR       = 0x51,
    BL_RESP_INVALID     = 0x52,
    BL_RESP_CRC_OK      = 0x53,
    BL_RESP_CRC_FAIL    = 0x54,
    BL_RESP_SEQ_ERROR   = 0x55
};

// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

//static uint8_t  CACHE_ALIGN flash_data[PAGE_SIZE];
//static uint32_t flash_addr, flash_size, flash_ptr;

//static uint32_t begin, end;
//static uint32_t unlock_begin, unlock_end;

static uint8_t rx_msg[HEADER_SIZE + MAX_DATA_SIZE];


//static uint8_t data_seq = 0;


uint8_t RxFrameValid = false;

typedef struct
{
	uint32_t Len;
	uint8_t Data[FRAMEWORK_BUFF_SIZE];
	
}T_FRAME;

static T_FRAME RxBuff;
static T_FRAME TxBuff;

typedef struct {
    char sw1: 1;
    char sw2: 1;
    char sw3: 1;
    char sw4: 1;
    char sw_reserved: 4;
}DIPSWITCH;

union{
    char CoolingUnitNumber;
    DIPSWITCH CoolingUnitNumber_Bits;
}DIP_SWITCH;



// *****************************************************************************
// *****************************************************************************
// Section: Bootloader Local Functions
// *****************************************************************************
// *****************************************************************************
void System_Start(void){
    p_Transmit = &TxMessage;
     /**********************************************************
 * set Cooling unit number 
 ***********************************************************/
    DIP_SWITCH.CoolingUnitNumber_Bits.sw1 = DIP_SW_1_Get(); 
    DIP_SWITCH.CoolingUnitNumber_Bits.sw2 = DIP_SW_2_Get(); 
    DIP_SWITCH.CoolingUnitNumber_Bits.sw3 = DIP_SW_3_Get(); 
    DIP_SWITCH.CoolingUnitNumber_Bits.sw4 = DIP_SW_4_Get();
    
    
    uint8_t tx_message[8] = {1,2,3,4,5,6,7,8};
    tx_message[0] = DIP_SWITCH.CoolingUnitNumber;
    CAN1_MessageTransmit(0x100, 8U, &tx_message[0], 0U, CAN_MSG_TX_DATA_FRAME);
}

bool ValidAppPresent(void)
{
	DWORD *AppPtr;
	AppPtr = (DWORD *)USER_APP_RESET_ADDRESS;
	if(*AppPtr == 0xFFFFFFFF)
	{
		return 0;   //FALSE;
	}
	else
	{
		return 1;   //TRUE;
	}
}
/* Function to program received application firmware data into internal flash */
//static void flash_write(void)
//{
//    flash_addr = TEST_ADD;
//    if (0U == (flash_addr % ERASE_BLOCK_SIZE))
//    {
//        /* Erase the Current sector */
//        (void)NVM_PageErase(flash_addr);
//
//        while(NVM_IsBusy() == true)
//        {
//        }
//    }
//
//    /* Write Page */
//    (void)NVM_RowWrite((void *)&flash_data[0], flash_addr);
//
//    while (NVM_IsBusy() == true) 
//    {
//    }
//}

/**
 * Static table used for the table_driven implementation.
 *****************************************************************************/
static const uint16_t crc_table[16] = 
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

/********************************************************************
* Function: 	CalculateCrc()
*
* Precondition: 
*
* Input: 		Data pointer and data length
*
* Output:		CRC.
*
* Side Effects:	None.
*
* Overview:     Calculates CRC for the given data and len
*
*			
* Note:		 	None.
********************************************************************/	
uint16_t CalculateCrc(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint16_t crc = 0;
    
    while(len--)
    {
        i = (crc >> 12) ^ (*data >> 4);
	    crc = crc_table[i & 0x0F] ^ (crc << 4);
	    i = (crc >> 12) ^ (*data >> 0);
	    crc = crc_table[i & 0x0F] ^ (crc << 4);
	    data++;
	} 

    return (crc & 0xFFFF);
}

/********************************************************************
* Function: 	BuildRxFrame()
*
* Precondition: 
*
* Input: 		Pointer to Rx Data and Rx byte length.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview: 	Builds rx frame and checks CRC.
*
*			
* Note:		 	None.
********************************************************************/
void BuildRxFrame(uint8_t *RxData, uint16_t RxLen)
{
	static bool Escape = RESET;//FALSE;
	WORD_VAL crc;
	
	
	while((RxLen > 0) && (!RxFrameValid)) // Loop till len = 0 or till frame is valid
	{
		RxLen--;
        
		WD_RESET_Toggle();
        
		if(RxBuff.Len >= sizeof(RxBuff.Data))
		{
			RxBuff.Len = RESET;
		}	
		
		switch(*RxData)
		{
			
			case BTL_START_BYTE:// SOH: //Start of header
				if(Escape)
				{
					// Received byte is not SOH, but data.
					RxBuff.Data[RxBuff.Len++] = *RxData;
					// Reset Escape Flag.
					Escape = RESET;//FALSE;
				}
				else
				{
					// Received byte is indeed a SOH which indicates start of new frame.
					RxBuff.Len = RESET;				
				}		
				break;
				
			case BTL_END_BYTE: //EOT: // End of transmission
				if(Escape)
				{
					// Received byte is not EOT, but data.
					RxBuff.Data[RxBuff.Len++] = *RxData;
					// Reset Escape Flag.
					Escape = RESET;//FALSE;
				}
				else
				{
					// Received byte is indeed a EOT which indicates end of frame.
					// Calculate CRC to check the validity of the frame.
					if(RxBuff.Len > 1)
					{
//						crc.byte.LB = RxBuff.Data[RxBuff.Len-2];
//						crc.byte.HB = RxBuff.Data[RxBuff.Len-1];
                        crc.v[0] =  RxBuff.Data[RxBuff.Len-2];
                        crc.v[1] = RxBuff.Data[RxBuff.Len-1];
						if((CalculateCrc(RxBuff.Data, (uint32_t)(RxBuff.Len-2)) == crc.Val) && (RxBuff.Len > 2))
						{
							// CRC matches and frame received is valid.
							RxFrameValid = TRUE;//TRUE;
											
						}
					}
				}							
				break;
				
				
		    case BTL_DLE_BYTE: //DLE: // Escape character received.
				if(Escape)
				{
					// Received byte is not ESC but data.
					RxBuff.Data[RxBuff.Len++] = *RxData;
					// Reset Escape Flag.
					Escape = RESET;//FALSE;					
				}
				else
				{
					// Received byte is an escape character. Set Escape flag to escape next byte.
					Escape = TRUE;//TRUE;					
				}	
				break;
			
			default: // Data field.
			    RxBuff.Data[RxBuff.Len++] = *RxData;
			    // Reset Escape Flag.
			    Escape = RESET;//FALSE;
				break;	
			
		}
		//Increment the pointer.
		RxData++;		
	}	
}	

/********************************************************************
* Function: 	WriteHexRecord2Flash()
*
* Precondition: 
*
* Input: 		HexRecord buffer.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview:     Writes hex record to flash.
*
*			
* Note:		 	None.
********************************************************************/	
void WriteHexRecord2Flash(uint8_t* HexRecord, UINT totalHexRecLen)
{
	static T_HEX_RECORD HexRecordSt;
	UINT8 Checksum = 0;
	UINT i;
	UINT WrData;
//	UINT RdData;
	void* ProgAddress;
//	UINT Result = 0;
	UINT nextRecStartPt = 0;
//    uint32_t count = 0;
	//UINT8 temp[4];
//        volatile UINT8 test = 0;
//       	void* pFlash;
	while(totalHexRecLen>=5) // A hex record must be atleast 5 bytes. (1 Data Len byte + 1 rec type byte+ 2 address bytes + 1 crc)
	{
		HexRecord = &HexRecord[nextRecStartPt];
		HexRecordSt.RecDataLen = HexRecord[0];
		HexRecordSt.RecType = HexRecord[3];	
		HexRecordSt.Data = &HexRecord[4];
		WD_RESET_Toggle();
		//Determine next record starting point.
		nextRecStartPt = HexRecordSt.RecDataLen + 5;	
		
		// Decrement total hex record length by length of current record.
		totalHexRecLen = totalHexRecLen - nextRecStartPt;
        //LenghtCount = totalHexRecLen;
		
		// Hex Record checksum check.
		Checksum = 0;
		for(i = 0; i < HexRecordSt.RecDataLen + 5; i++)
		{
			Checksum += HexRecord[i];
		}	
		
	    if(Checksum != 0)
	    {
           // Trasmit_CAN_BTL_Response(0x08);
		    //Error. Hex record Checksum mismatch.
		} 
		else
		{
			// Hex record checksum OK.
			switch(HexRecordSt.RecType)
			{
				case DATA_RECORD:  //Record Type 00, data record.
					HexRecordSt.Address.byte.MB = 0;
					HexRecordSt.Address.byte.UB = 0;
					HexRecordSt.Address.byte.HB = HexRecord[1];
					HexRecordSt.Address.byte.LB = HexRecord[2];
					
					// Derive the address.
					HexRecordSt.Address.Val = HexRecordSt.Address.Val + HexRecordSt.ExtLinAddress.Val + HexRecordSt.ExtSegAddress.Val;
							
					while(HexRecordSt.RecDataLen) // Loop till all bytes are done.
					{
											
						// Convert the Physical address to Virtual address. 
						ProgAddress = PA_TO_KVA0(HexRecordSt.Address.Val);

						// Make sure we are not writing boot area and device configuration bits.
#if 1 
						if(((ProgAddress >= (void *)APP_FLASH_BASE_ADDRESS) && (ProgAddress <= (void *)APP_FLASH_END_ADDRESS))
						   && ((ProgAddress < (void*)DEV_CONFIG_REG_BASE_ADDRESS) || (ProgAddress > (void*)DEV_CONFIG_REG_END_ADDRESS)))
#else 
                        if(((ProgAddress >= (void *)APP_FLASH_BASE_ADDRESS) && (ProgAddress <= (void *)APP_FLASH_END_ADDRESS)))  
#endif
                        {
							if(HexRecordSt.RecDataLen < 4)
							{
								
								// Sometimes record data length will not be in multiples of 4. Appending 0xFF will make sure that..
								// we don't write junk data in such cases.
								WrData = 0xFFFFFFFF;
								memcpy(&WrData, HexRecordSt.Data, HexRecordSt.RecDataLen);	
							}
							else
							{	
								memcpy(&WrData, HexRecordSt.Data, 4);
							}		
							// Write the data into flash.	
							//Result = NVMemWriteWord(ProgAddress, WrData);
                            WD_RESET_Toggle();
                            NVM_WordWrite( WrData,(uint32_t)ProgAddress);
							// Assert on error. This must be caught during debug phase.	
                            while (NVM_IsBusy() == true); 
//                          for(count = 0; count < 90; count++){
//                          // eat 5-star do nothing
//                          }
//                            UART4_Write("Yes",3);
							//ASSERT(Result==0);
						}else {
//                            UART4_Write("NO",2);
                        }
//						if((ProgAddress >= (void *)0x9FC00000) && (ProgAddress <= (void *)0x9FC02FEF))
//						{//This is prob as address falls under the boot loader
//							error_flag = 10;
//                          
//						}
						
						// Increment the address.
						HexRecordSt.Address.Val += 4;
						// Increment the data pointer.
						HexRecordSt.Data += 4;
						// Decrement data len.
						if(HexRecordSt.RecDataLen > 3)
						{
							HexRecordSt.RecDataLen -= 4;
						}	
						else
						{
							HexRecordSt.RecDataLen = 0;
						}	
					}

                          
					break;
				
				case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.
				    HexRecordSt.ExtSegAddress.byte.MB = 0;
					HexRecordSt.ExtSegAddress.byte.UB = HexRecordSt.Data[0];
					HexRecordSt.ExtSegAddress.byte.HB = HexRecordSt.Data[1];
					HexRecordSt.ExtSegAddress.byte.LB = 0;
					// Reset linear address.
					HexRecordSt.ExtLinAddress.Val = 0;
					break;
					
				case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address. 
					HexRecordSt.ExtLinAddress.byte.MB = HexRecordSt.Data[0];
					HexRecordSt.ExtLinAddress.byte.UB = HexRecordSt.Data[1];
					HexRecordSt.ExtLinAddress.byte.HB = 0;
					HexRecordSt.ExtLinAddress.byte.LB = 0;
					// Reset segment address.
					HexRecordSt.ExtSegAddress.Val = 0;
					break;
					
				case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
				default: 
                    
					HexRecordSt.ExtSegAddress.Val = 0;
					HexRecordSt.ExtLinAddress.Val = 0;
					break;
			}		
		}	
	}//while(1)	
		
}

/*******************************************************
 *
 *******************************************************/
UCHAR8 Forming_CAN_TxMsg(UINT16 Msg_ID, TxMsgs * ptr, UINT8 *Data, UINT8 count){
 UCHAR8 status = RESET;

    if (ptr != NULL) {
        memset(&ptr->TxMsg[0],RESET,CAN_MSG_LENG);
        switch (Msg_ID) {
            case CAN_FILTER_ID:
                ptr->TxMsgID = CAN_FILTER_ID;
                ptr->TxMsg[0] = Data[count++];
                ptr->TxMsg[1] = Data[count++];
                ptr->TxMsg[2] = Data[count++];
                ptr->TxMsg[3] = Data[count++];
                ptr->TxMsg[4] = Data[count++];
                ptr->TxMsg[5] = Data[count++];
                ptr->TxMsg[6] = Data[count++];
                ptr->TxMsg[7] = Data[count];
                status = TRUE;
                break;
            default:
                status = FALSE; //ERROR_CAN_ID;
                break;
        }
    }
    
    return status;
}

#if 1
unsigned char GetTransmitFrame(unsigned char* Buff)
{
//  1. Initialise BuffLen = 0                                               //    
	INT BuffLen     = RESET;
	WORD_VAL crc    = {RESET};
	UINT8 i         = RESET;
//  2. If TxBuff.Len > 0                                                    //	
	if(TxBuff.Len) 
	{
//      1. Clear output buffer                                              //        
        memset(&Buff[0],RESET,CAN_MSG_LENG);
//      2. Calculate CRC of TxBuff.Data                                     //.
		crc.Val = CalculateCrc(TxBuff.Data, (UINT32)TxBuff.Len);
//      3. Append CRC (LB, HB) to TxBuff.Data                               //        
		TxBuff.Data[TxBuff.Len++] = crc.byte.LB;
		TxBuff.Data[TxBuff.Len++] = crc.byte.HB; 	
//      4. Insert SOH at start of Buff                                      //			
		Buff[BuffLen++] = BTL_START_BYTE;
//      5. For each byte in TxBuff.Data                                     //		
		for(i = RESET; i < TxBuff.Len; i++)
		{
//          - If byte == SOH/EOT/DLE                                        //            
			if((TxBuff.Data[i] == BTL_END_BYTE) || (TxBuff.Data[i] == BTL_START_BYTE)
				|| (TxBuff.Data[i] == BTL_DLE_BYTE))
			{
//              ? Insert DLE (escape character)                             //
				Buff[BuffLen++] = BTL_DLE_BYTE;			
			}
//          - Copy actual data byte into Buff                               //            
			Buff[BuffLen++] = TxBuff.Data[i];
		} 
//      6. Append EOT to mark end of frame                                  //
		Buff[BuffLen++] = BTL_END_BYTE;
//      7. Reset TxBuff.Len to 0 (buffer consumed)                          //		
		TxBuff.Len = RESET; // Purge this buffer, no more required.
	}		
	return(BuffLen); // Return buffer length.
}
#endif 
/*******************************************************************
 * Process follow old Bootloader V1.0
 *********************************************************************/
void process_oldcommand(void)
{
    uint8_t cmd = TxBuff.Data[0] = RxBuff.Data[0];
    //          - Extract requested FDB ID and dispenser number                  //
    UINT8 requestedID   = RESET;
    UINT8 reqDispNum    = RESET;
    uint8_t i           = RESET;   
    uint16_t btlVersion = bootloader_GetVersion();
    void* pFlash        = NULL;
    
    switch (cmd) {
        case BTL_CMD_READ_VERSION:
            TxBuff.Len = (CMD_ID_TX_LEN + 2); 
            
            TxBuff.Data[1] = (uint8_t) ((btlVersion >> 8) & 0xFFU); //Bootloader Major ver
            TxBuff.Data[2] = (uint8_t) (btlVersion & 0xFFU); //Bootloader minor ver
       
            break;
        case BTL_CMD_ERASE: //ERASE Cmd
            //          - Erase boot status flag page                                    //            
            (void)NVM_PageErase(BOOT_FLAG_STATUS_ADDR);
//          - Mark erase start                                               //            
            (void)NVM_WordWrite(0x01, BOOT_FLAG_STATUS_ADDR);  // Indicate flash erase start
            /* Process */
            pFlash = (void*)FLASH_START;
            for (i = RESET; i < ((FLASH_LENGTH + 1) / ERASE_BLOCK_SIZE); i++) { //
                /* Erase the Current sector */
                WD_RESET_Toggle();
                (void) NVM_PageErase((uint32_t) pFlash + (i * ERASE_BLOCK_SIZE));
                while (NVM_IsBusy() == true);

            }
            (void)NVM_WordWrite(0x01, BOOT_FLAG);  // Indicate flash erase start
            (void)NVM_WordWrite(0x01, BOOT_FLAG_STATUS_ADDR);  // Indicate flash erase start
            /* Response */
            TxBuff.Len = CMD_ID_TX_LEN ; 

            break;
        case BTL_CMD_PROGRAM:
            
            WriteHexRecord2Flash(&RxBuff.Data[1], RxBuff.Len - 3);
            
            WD_RESET_Toggle();
            TxBuff.Len = CMD_ID_TX_LEN; 

            break;
        case BTL_CMD_CCU_BOARD:


            requestedID = RxBuff.Data[1] & 0x0F; //Extract FDB Number from byte 
            reqDispNum = (RxBuff.Data[1] >> 4 );

            TxBuff.Len = (CMD_ID_TX_LEN + 2);            
            if (RxBuff.Data[2] == BOARD_INFO_CMD)
                {
//              - Prepare response frame                                     //                    
                    TxBuff.Data[1] = ((reqDispNum << 4 ) | requestedID);
                    TxBuff.Data[2] = 02;//RxBuff.Data[2];
                    TxBuff.Data[3] = 0xFF;
                    TxBuff.Len = (CMD_ID_TX_LEN + 2 + 1);   
                }
            else if(RxBuff.Data[2] == BOARD_SW_BTL_CMD)
            {
                TxBuff.Data[1] = ((reqDispNum << 4 ) | requestedID);
                TxBuff.Data[2] = RxBuff.Data[2];
            }
                else
                {
//          - Else ? Clear TxBuff length                                              //                    
                    TxBuff.Len = RESET;
                }

            break;
        
        case 4:
            break;
        case BTL_CMD_RUN: //RUN cmd
//          - Check boot flag status                                         //
            memcpy((void *)&HexFileStatus, (void *)BOOT_FLAG_STATUS_ADDR, sizeof(HexFileStatus));
//          - If valid, erase boot flag                                      //
            if (HexFileStatus != 0x01)
            {
                RunApplication = TRUE;
                WD_RESET_Toggle();
                NVM_PageErase(BOOT_FLAG);
                while (NVM_IsBusy() == true);

                bootloader_TriggerReset();
                RunApplication = TRUE;
            }
            break;
        case BTL_COMPORT:
            TxBuff.Len = CMD_ID_TX_LEN ;
            break;
        case BTL_HEX_LOAD_FINISH:
            //          - Erase boot status flag page                                    //
            (void)NVM_PageErase(BOOT_FLAG_STATUS_ADDR);
            break;
        default:
            break;
    }
 
}

/* Function to receive application firmware via CAN1 */
static void CAN1_task(void)
{
    uint32_t status         = RESET;
    uint32_t rx_messageID   = RESET;
    uint8_t rx_messageLength = RESET;
    
    uint8_t count           = RESET;
    uint8_t retStatus       = RESET;
    uint8_t sendLen         = RESET;
    uint8_t bytesLeft       = RESET;
    uint8_t framedBuff[TX_BUFF_SIZE] = {RESET};
    
    CAN_MSG_RX_ATTRIBUTE msgFrameAttr = CAN_MSG_RX_DATA_FRAME;

    if (CAN1_InterruptGet(1U, CAN_FIFO_INTERRUPT_RXNEMPTYIF_MASK))
    {

        /* Check CAN1 Status */
        status = (uint32_t)CAN1_ErrorGet();
        if (status == CAN_ERROR_NONE)
        {
            (void)memset(rx_msg, RESET, sizeof(rx_msg));

            /* Receive FIFO 1 New Message */
            if(CAN1_MessageReceive(&rx_messageID, &rx_messageLength, rx_msg, NULL, 1U, &msgFrameAttr) == true)
            {
                /* Check CAN1 Status */
                status = (uint32_t)CAN1_ErrorGet();
                if (status == CAN_ERROR_NONE)
                {
                    //Ready with 8 byte data
                    /* Build Receive Frame */
                    
                    BuildRxFrame(rx_msg, rx_messageLength);
                    if(RxFrameValid == TRUE){
                        process_oldcommand();
//                          3. Get transmit frame buffer                    //                        
                        bytesLeft = GetTransmitFrame(framedBuff);
                        // Now send TxBuff.Data in segments of 8 bytes
                        count = RESET;
//                          4. While bytesLeft > 0                          //                        
                        while (bytesLeft > RESET)
                        {
//                              4.1. Form CAN Transmit message              //
                            retStatus = Forming_CAN_TxMsg(CAN_FILTER_ID, p_Transmit, framedBuff, count);
                            UINT32 delaycount = RESET;
                            //TODO - add debug here, are we not going into retStatus success? PRINT retStatus
                            if (retStatus == TRUE)
                            {
//                                  1. Determine send length (max 8 bytes)  //                                
                                sendLen = (bytesLeft > CAN_MSG_LENG) ? CAN_MSG_LENG : bytesLeft;
//                                  2. Transmit CAN message (extended frame)//                                
                                CAN1_MessageTransmit(p_Transmit->TxMsgID, sendLen, p_Transmit->TxMsg, 0U, CAN_MSG_TX_DATA_FRAME);
//                                  3. Update counters                      //
                                count += sendLen;
                                bytesLeft -= sendLen;
//                                  4. Small delay loop                     //
                                for (delaycount = RESET; delaycount < 200000; delaycount++) {
                                    // delay loop
                                }
                            }
                            else
                            {
//                              1. Abort transmission                    //
                                break;
                            }
                        } 
                        WD_RESET_Toggle();
                        RxFrameValid = RESET;
                    }else{
                        WD_RESET_Toggle();
                    }
                }
                else
                {
                    
                }
            }
        }

    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Bootloader Global Functions
// *****************************************************************************
// *****************************************************************************

void bootloader_CAN_Tasks(void)
{
    CAN1_task();
}


/********************************************************************
* Function: 	ExitFirmwareUpgradeMode()
*
* Precondition: 
*
* Input: 		Void
*
* Output:		True if firmware upgrade mode has to be exited.
*
* Side Effects:	None.
*
* Overview:     This function returns true if firmware mode has to be exited.
*
*			
* Note:		 	None.
********************************************************************/
bool ExitFirmwareUpgradeMode(void)
{
	return RunApplication;
}