//###########################################################################
//
// FILE:    Shared_Erase.c
//
// TITLE:   Erase shared functions
//
// Functions:
//
//     void SCI_Erase(Uint16 command, Uint32 sectors)
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
#include "F2837xD_Gpio_defines.h"
#include "F2837xD_GlobalPrototypes.h"
#include "F021_F2837xD_C28x.h"
#include "flash_programming_c28.h" // Flash API example header file
#include "Shared_Erase.h"
#include "F2837xD_Ipc_drivers.h"
#include "Types.h"

//
// Defines
//
#define C1C2_BROM_IPC_EXECUTE_BOOTMODE_CMD  0x00000013
#define C1C2_BROM_BOOTMODE_BOOT_FROM_SCI    0x00000001
#define C1C2_BROM_BOOTMODE_BOOT_FROM_RAM    0x0000000A
#define C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH  0x0000000B
#define NO_ERROR                            0x1000
#define BLANK_ERROR                         0x2000
#define VERIFY_ERROR                        0x3000
#define PROGRAM_ERROR                       0x4000
#define COMMAND_ERROR                       0x5000
#define INCORRECT_DATA_BUFFER_LENGTH         0x7000
#define INCORRECT_ECC_BUFFER_LENGTH          0x8000
#define DATA_ECC_BUFFER_LENGTH_MISMATCH      0x9000
#define FEATURE_NOT_AVAILABLE                0xB000
#define FAILURE                              0xE000
#define NOT_RECOGNIZED                       0xF000
//
// Globals
//
typedef struct
{
   Uint16 status;
   Uint32 address;
   Uint16 flashAPIError;
   Uint32 flashAPIFsmStatus;
}  StatusCode;
extern StatusCode statusCode;

const Uint32 sectAddress[14] = { Bzero_SectorA_start,
                                  Bzero_SectorB_start,
                                  Bzero_SectorC_start,
                                  Bzero_SectorD_start,
                                  Bzero_SectorE_start,
                                  Bzero_SectorF_start,
                                  Bzero_SectorG_start,   //FLASH_SIZE 0x6
                                  Bzero_SectorH_start,
                                  Bzero_SectorI_start,
                                  Bzero_SectorJ_start,
                                  Bzero_SectorK_start,
                                  Bzero_SectorL_start,
                                  Bzero_SectorM_start,
                                  Bzero_SectorN_start };  //FLASH_SIZE 0x7

const Uint16 sectSize[14] = {     Bzero_16KSector_u32length,
                                Bzero_16KSector_u32length,
                                Bzero_16KSector_u32length,
                                Bzero_16KSector_u32length,
                                Bzero_64KSector_u32length,
                                Bzero_64KSector_u32length,
                                Bzero_64KSector_u32length,   //FLASH_SIZE 0x6
                                Bzero_64KSector_u32length,
                                Bzero_64KSector_u32length,
                                Bzero_64KSector_u32length,
                                Bzero_16KSector_u32length,
                                Bzero_16KSector_u32length,
                                Bzero_16KSector_u32length,
                                Bzero_16KSector_u32length }; //FLASH_SIZE 0x7

//
// Function Prototypes
//
void Shared_Erase(Uint32 sectors);
extern void Example_Error(Fapi_StatusType status);
extern void assignSharedRAMstoCPU2(void);
extern void Assign_SCIA_IO_CPU2(Uint32 BootMode);
extern void SendACK(void);
extern void SendNAK(void);
extern void setFlashAPIError(Fapi_StatusType status);

//
// Shared_Erase - This routine takes the 32-bit sectors variable as
//                a parameter.  Each bit corresponds to a sector,
//                starting with bit 0 and sector A.  This routine
//                attempts to erase the sectors specified.
//
void Shared_Erase(Uint32 sectors)
{
    statusCode.status = NO_ERROR;
    statusCode.address = 0x12345678;
    int i = 0;
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusWordType oFlashStatusWord;
    Fapi_FlashStatusType oFlashStatus;

    int fail = 0;
    EALLOW;
    for(i = 0; i < 14; i++)
    {
        if((sectors & 0x00000001) == 0x00000001 && fail == 0)
        {
            oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                    (uint32 *)(sectAddress[i]));
            while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
            {
            }        
            oReturnCheck = Fapi_doBlankCheck((uint32 *)(sectAddress[i]),
                                             sectSize[i],
                                             &oFlashStatusWord);
            oFlashStatus = Fapi_getFsmStatus();
            if(oReturnCheck != Fapi_Status_Success || oFlashStatus != 0)
            {
                if(fail == 0) //first fail
                {
                    statusCode.status = BLANK_ERROR;
                    statusCode.address = oFlashStatusWord.au32StatusWord[0];
                    setFlashAPIError(oReturnCheck);
                    statusCode.flashAPIFsmStatus = oFlashStatus;
                }
                fail++;
            }
        }
        sectors = sectors >> 1;
    }
    EDIS;
    return;
}

//
// End of file
//
