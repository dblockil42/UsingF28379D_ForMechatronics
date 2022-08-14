//###########################################################################
//
//
// FILE:    Shared_Boot.c
//
//
// TITLE:   Boot loader shared functions
//
// Functions:
//
//     void VerifyData(void)
//
//###########################################################################
//
// $Release Date:  $
// $Copyright:
// Copyright (C) 2013-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "c2_bootrom.h"
#include "F021_F2837xD_C28x.h"
#include "flash_programming_c28.h" // Flash API example header file

//
// Defines
//
#define NO_ERROR                    0x1000
#define BLANK_ERROR                 0x2000
#define VERIFY_ERROR                0x3000
#define PROGRAM_ERROR               0x4000
#define COMMAND_ERROR               0x5000
#define BUFFER_SIZE                 0x80   // because of ECC, must be multiple
                                           // of 64 bits, or 4 words,
                                           // BUFFER_SIZE % 4 == 0

//
// Globals
//

//
// GetWordData is a pointer to the function that interfaces to the peripheral.
// Each loader assigns this pointer to it's particular GetWordData function.
//
extern uint16fptr GetWordData;

typedef struct
{
   Uint16 status;
   Uint32 address;
   Uint16 flashAPIError;
   Uint32 flashAPIFsmStatus;
}  StatusCode;
extern StatusCode statusCode;

//
// Function Prototypes
//
extern void SCI_SendChecksum(void);
void VerifyData(void);
extern Uint32 GetLongData(void);
extern void ReadReservedFn(void);
extern Uint16 SCIA_GetWordData(void);
extern void setFlashAPIError(Fapi_StatusType status);

//
// VerifyData - This routine copies multiple blocks of data from the host
//              and verifies that data with the flash in the device.
//              An error is returned if the flash is not verified.
//
//              Multiple blocks of data are copied until a block
//              size of 00 00 is encountered.
//
void VerifyData()
{
    statusCode.status = NO_ERROR;
    statusCode.address = 0x12346578;

    struct HEADER {
     Uint16 BlockSize;
     Uint32 DestAddr;
    } BlockHeader;

    Uint16 wordData;
    Uint16 i,j,k;
    Uint16 Buffer[BUFFER_SIZE];
    Uint16 miniBuffer[4];
    int fail = 0;
    Uint16 wordsVerified = 0;

    assert(BUFFER_SIZE % 4 == 0);

    //
    // Assign GetWordData to the SCI-A version of the
    // function. GetWordData is a pointer to a function.
    //
    GetWordData = SCIA_GetWordData;

    //
    // If the KeyValue was invalid, abort the load
    // and return the flash entry point.
    //
    if(SCIA_GetWordData() != 0x08AA)
    {
        statusCode.status = VERIFY_ERROR;
        statusCode.address = FLASH_ENTRY_POINT;
    }

    ReadReservedFn(); //reads and discards 8 reserved words

    //Uint32 EntryAddr = GetLongData();
    GetLongData();

    //
    // Send checksum to satisfy before we begin
    //
#if checksum_enable
    SCI_SendChecksum();
#endif

    //
    // Get the size in words of the first block
    //
    BlockHeader.BlockSize = (*GetWordData)();

    //
    // While the block size is > 0 copy the data
    // to the DestAddr.  There is no error checking
    // as it is assumed the DestAddr is a valid
    // memory location
    //

    EALLOW;
    while(BlockHeader.BlockSize != (Uint16)0x0000)
    {
       Fapi_StatusType oReturnCheck;
       Fapi_FlashStatusWordType oFlashStatusWord;
       //Fapi_FlashStatusType oFlashStatus;
       BlockHeader.DestAddr = GetLongData();
       for(i = 0; i < BlockHeader.BlockSize; i += 0)
       {
           if(BlockHeader.BlockSize < BUFFER_SIZE)
           {
               for(j = 0; j < BlockHeader.BlockSize; j++)
               {
                   wordData = (*GetWordData)();
                   Buffer[j] = wordData;
                   i++;
               }
               for(j = BlockHeader.BlockSize; j < BUFFER_SIZE; j++)
               {
                   Buffer[j] = 0xFFFF;
               }
           }
           else //BlockHeader.BlockSize >= BUFFER_SIZE
           {
               if((BlockHeader.BlockSize - i) < BUFFER_SIZE) //less than one
                                                             //BUFFER_SIZE left
               {
                   //
                   // fill Buffer with rest of data
                   //
                   for(j = 0; j < BlockHeader.BlockSize - i; j++)
                   {
                       wordData = (*GetWordData)();
                       Buffer[j] = wordData;
                   }
                   i += j; //increment i outside here so it doesn't
                           //affect loop above
                   for(; j < BUFFER_SIZE; j++)//fill the rest with 0xFFFF
                   {
                       Buffer[j] = 0xFFFF;
                   }
               }
               else
               {
                   //
                   // fill up like normal, up to BUFFER_SIZE
                   //
                   for(j = 0; j < BUFFER_SIZE; j++)
                   {
                       wordData = (*GetWordData)();
                       Buffer[j] = wordData;
                       i++;
                   }
               }
           }

           for(k = 0; k < (BUFFER_SIZE / 4); k++)
           {
               miniBuffer[0] = Buffer[k * 4 + 0];
               miniBuffer[1] = Buffer[k * 4 + 1];
               miniBuffer[2] = Buffer[k * 4 + 2];
               miniBuffer[3] = Buffer[k * 4 + 3];

              //
               //check if there are still words left to verify
               //
               if(wordsVerified < BlockHeader.BlockSize)
               {
                    while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);
                    //oFlashStatus = Fapi_getFsmStatus();
                    for(j = 0; j < 4; j += 2)
                    {
                       Uint32 toVerify = miniBuffer[j+1];
                       toVerify = toVerify << 16;
                       toVerify |= miniBuffer[j];
                       if(fail == 0)
                       {
                           oReturnCheck = Fapi_doVerify((uint32 *)(BlockHeader.DestAddr+j),
                                                        1,
                                                        (uint32 *)(&toVerify),
                                                        &oFlashStatusWord);
                           if(oReturnCheck != Fapi_Status_Success)
                           {
                               if(fail == 0) //first fail
                               {
                                    statusCode.status = VERIFY_ERROR;
                                    statusCode.address = oFlashStatusWord.au32StatusWord[0];
                                    setFlashAPIError(oReturnCheck);
                                    //
                                    // FMSTAT is not checked for Verify
                                    //
                                    statusCode.flashAPIFsmStatus = 0;
                               }
                               fail++;
                           }
                       }
                    } //for j; for Fapi_doVerify
               } //check if there are still words left to verify
               BlockHeader.DestAddr += 0x4;
               wordsVerified += 0x4;
           } //for(int k); loads miniBuffer with Buffer elements
#if checksum_enable
           SCI_SendChecksum();
#endif
       }
       //
       //get the size of the next block
       //
       BlockHeader.BlockSize = (*GetWordData)();
       wordsVerified = 0;
    }
    EDIS;
    return;
}

//
// End of file
//
