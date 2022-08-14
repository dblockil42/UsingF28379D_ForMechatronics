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
//     void   CopyData(void)
//     Uint32 GetLongData(void)
//     void ReadReservedFn(void)
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
#include "c1_bootrom.h"
#include "F021_F2837xD_C28x.h"
#include "flash_programming_c28.h" // Flash API example header file

//
// Globals
//
uint16fptr GetWordData;   // GetWordData is a pointer to the function that
                          // interfaces to the peripheral. Each loader assigns
                          // this pointer to it's particular GetWordData
                          // function.

unsigned char erasedAlready[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//
// Function Prototypes
//
Uint32 GetLongData(void);
void CopyData(void);
void ReadReservedFn(void);
Uint32 FindSector(Uint32 address);
Uint16 FindSize(Uint32 address);
void Example_Error(Fapi_StatusType status);

//
// CopyData - This routine copies multiple blocks of data from the host
//            to the specified RAM locations.  There is no error
//            checking on any of the destination addresses.
//            That is it is assumed all addresses and block size
//            values are correct.
//
//            Multiple blocks of data are copied until a block
//            size of 00 00 is encountered.
//
void CopyData()
{
    struct HEADER {
     Uint16 BlockSize;
     Uint32 DestAddr;
    } BlockHeader;

    Uint16 wordData;
    Uint16 i;
    Uint16 j;
    Uint16 Buffer[8];
    Uint32 sectorAddress;
    Uint16 sectorSize;

    for(i = 0; i < 14; i++)
    {
        erasedAlready[i] = 0;
    }

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
       volatile Fapi_FlashStatusType oFlashStatus;
       BlockHeader.DestAddr = GetLongData();

       for(i = 0; i < BlockHeader.BlockSize; i += 0)
       {
           for(j = 0; j < 8; j++)
           {
               if(i == BlockHeader.BlockSize)
               {
                   Buffer[j] = 0xFFFF;
               }
               else
               {
                   wordData = (*GetWordData)();
                   Buffer[j] = wordData;
                   i++;
               }
           }
           //
           //check that Buffer is not already all erased data
           //
           if(!((Buffer[0] == 0xFFFF) && (Buffer[1] == 0xFFFF) &&
		        (Buffer[2] == 0xFFFF) && (Buffer[3] == 0xFFFF) &&
		        (Buffer[4] == 0xFFFF) && (Buffer[5] == 0xFFFF) &&
                (Buffer[6] == 0xFFFF) && (Buffer[7] == 0xFFFF)))
           {
               //
               //clean out flash banks if needed
               //
               sectorAddress = FindSector(BlockHeader.DestAddr);
               if(sectorAddress != 0xdeadbeef)
               {
                    sectorSize = FindSize(sectorAddress);
                    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                            (uint32 *)sectorAddress);
                    while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);		
                    oReturnCheck = Fapi_doBlankCheck((uint32 *)sectorAddress,
                            sectorSize,
                            &oFlashStatusWord);

                    if(oReturnCheck != Fapi_Status_Success)
                    {
                        Example_Error(oReturnCheck);
                    }
               }
               //
               //program 8 words at once, 128-bits
               //
               oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)BlockHeader.DestAddr,
                                                       Buffer,
                                                       sizeof(Buffer),
                                                       0,
                                                       0,
                                                       Fapi_AutoEccGeneration);

               while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

               if(oReturnCheck != Fapi_Status_Success)
               {
                  Example_Error(oReturnCheck);
               }
               oFlashStatus = Fapi_getFsmStatus();

               for(j = 0; j < 8; j += 2)
               {
                  Uint32 toVerify = Buffer[j+1];
                  toVerify = toVerify << 16;
                  toVerify |= Buffer[j];
                  oReturnCheck = Fapi_doVerify((uint32 *)(BlockHeader.DestAddr+j),
                                               1,
                                               (uint32 *)(&toVerify),
                                               &oFlashStatusWord);
                  if(oReturnCheck != Fapi_Status_Success)
                  {
                     Example_Error(oReturnCheck);
                  }
               }
           }
           BlockHeader.DestAddr += 0x8;
       }
       //
       //get the size of the next block
       //
       BlockHeader.BlockSize = (*GetWordData)();
    }
    EDIS;
}

//
// FindSector - This routine finds what sector the mentioned address
//              is a part of.
//
Uint32 FindSector(Uint32 address)
{
    if((address >= Bzero_SectorA_start) &&
       (address <= Bzero_SectorA_End) && (erasedAlready[0] == 0))
    {
        erasedAlready[0] = 1;
        return (Uint32)Bzero_SectorA_start;
    }
    else if((address >= Bzero_SectorB_start) &&
            (address <= Bzero_SectorB_End) && (erasedAlready[1] == 0))
    {
        erasedAlready[1] = 1;
        return (Uint32)Bzero_SectorB_start;
    }
    else if((address >= Bzero_SectorC_start) &&
            (address <= Bzero_SectorC_End) && (erasedAlready[2] == 0))
    {
        erasedAlready[2] = 1;
        return (Uint32)Bzero_SectorC_start;
    }
    else if((address >= Bzero_SectorD_start) &&
            (address <= Bzero_SectorD_End) && (erasedAlready[3] == 0))
    {
        erasedAlready[3] = 1;
        return (Uint32)Bzero_SectorD_start;
    }
    else if((address >= Bzero_SectorE_start) &&
            (address <= Bzero_SectorE_End) && (erasedAlready[4] == 0))
    {
        erasedAlready[4] = 1;
        return (Uint32)Bzero_SectorE_start;
    }
    else if((address >= Bzero_SectorF_start) &&
            (address <= Bzero_SectorF_End) && (erasedAlready[5] == 0))
    {
        erasedAlready[5] = 1;
        return (Uint32)Bzero_SectorF_start;
    }
    else if((address >= Bzero_SectorG_start) &&
            (address <= Bzero_SectorG_End) && (erasedAlready[6] == 0))
    {
        erasedAlready[6] = 1;
        return (Uint32)Bzero_SectorG_start;
    }
    else if((address >= Bzero_SectorH_start) &&
            (address <= Bzero_SectorH_End) && (erasedAlready[7] == 0))
    {
        erasedAlready[7] = 1;
        return (Uint32)Bzero_SectorH_start;
    }
    else if((address >= Bzero_SectorI_start) &&
            (address <= Bzero_SectorI_End) && (erasedAlready[8] == 0))
    {
        erasedAlready[8] = 1;
        return (Uint32)Bzero_SectorI_start;
    }
    else if((address >= Bzero_SectorJ_start) &&
            (address <= Bzero_SectorJ_End) && (erasedAlready[9] == 0))
    {
        erasedAlready[9] = 1;
        return (Uint32)Bzero_SectorJ_start;
    }
    else if((address >= Bzero_SectorK_start) &&
            (address <= Bzero_SectorK_End) && (erasedAlready[10] == 0))
    {
        erasedAlready[10] = 1;
        return (Uint32)Bzero_SectorK_start;
    }
    else if((address >= Bzero_SectorL_start) &&
            (address <= Bzero_SectorL_End) && (erasedAlready[11] == 0))
    {
        erasedAlready[11] = 1;
        return (Uint32)Bzero_SectorL_start;
    }
    else if((address >= Bzero_SectorM_start) &&
            (address <= Bzero_SectorM_End) && (erasedAlready[12] == 0))
    {
        erasedAlready[12] = 1;
        return (Uint32)Bzero_SectorM_start;
    }
    else if((address >= Bzero_SectorN_start) &&
            (address <= Bzero_SectorN_End) && (erasedAlready[13] == 0))
    {
        erasedAlready[13] = 1;
        return (Uint32)Bzero_SectorN_start;
    }
    else
    {
        return 0xdeadbeef; //a proxy address to signify that it is not
                           //a flash sector
    }
}

//
// FindSector - This routine finds the size of the sector under use.
//
Uint16 FindSize(Uint32 address)
{
    if(address == Bzero_SectorA_start)
    {
        return Bzero_16KSector_u32length;
    }
    else if(address == Bzero_SectorB_start)
    {
        return Bzero_16KSector_u32length;
    }
    else if(address == Bzero_SectorC_start)
    {
        return Bzero_16KSector_u32length;
    }
    else if(address == Bzero_SectorD_start)
    {
        return Bzero_16KSector_u32length;
    }
    else if(address == Bzero_SectorE_start)
    {
        return Bzero_64KSector_u32length;
    }
    else if(address == Bzero_SectorF_start)
    {
        return Bzero_64KSector_u32length;
    }
    else if(address == Bzero_SectorG_start)
    {
        return Bzero_64KSector_u32length;
    }
    else if(address == Bzero_SectorH_start)
    {
        return Bzero_64KSector_u32length;
    }
    else if(address == Bzero_SectorI_start)
    {
        return Bzero_64KSector_u32length;
    }
    else if(address == Bzero_SectorJ_start)
    {
        return Bzero_64KSector_u32length;
    }
    else if(address == Bzero_SectorK_start)
    {
        return Bzero_16KSector_u32length;
    }
    else if(address == Bzero_SectorL_start)
    {
        return Bzero_16KSector_u32length;
    }
    else if(address == Bzero_SectorM_start)
    {
        return Bzero_16KSector_u32length;
    }
    else if(address == Bzero_SectorN_start)
    {
        return Bzero_16KSector_u32length;
    }
    //
    //no other possible case
    //
    return 0xbeef;
}

//
// GetLongData - This routine fetches a 32-bit value from the peripheral
//               input stream.
//
Uint32 GetLongData()
{
    Uint32 longData;

    //
    // Fetch the upper 1/2 of the 32-bit value
    //
    longData = ((Uint32)(*GetWordData)() << 16);

    //
    // Fetch the lower 1/2 of the 32-bit value
    //
    longData |= (Uint32)(*GetWordData)();

    return longData;
}

//
// Read_ReservedFn - This function reads 8 reserved words in the header.
//                   None of these reserved words are used by the
//                   this boot loader at this time, they may be used in
//                   future devices for enhancements.  Loaders that use
//                   these words use their own read function.
//
void ReadReservedFn()
{
    Uint16 i;

    //
    // Read and discard the 8 reserved words.
    //
    for(i = 1; i <= 8; i++)
    {
       GetWordData();
    }
    return;
}

//
// End of file
//
