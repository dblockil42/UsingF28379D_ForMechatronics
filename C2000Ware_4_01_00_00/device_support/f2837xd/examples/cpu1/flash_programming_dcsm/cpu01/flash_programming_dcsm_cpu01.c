//###########################################################################
//
// FILE:   flash_programming_dcsm_cpu01.c
//
// TITLE:  Flash Programming with DCSM.
//
//! \addtogroup cpu01_example_list
//! <h1> Flash Programming with DCSM </h1>
//!
//! This example programs secure memory
//!
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

#include "F28x_Project.h"
#include <string.h>
#include "flash_programming_dcsm_c28.h"
#include "F021_F2837xD_C28x.h"

//
// Defines
//
#define  WORDS_IN_FLASH_BUFFER    0xFF

#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
        #define ramFuncSection ".TI.ramfunc"
    #else
        #define ramFuncSection "ramfuncs"
    #endif
#endif

//
// Globals
//
#pragma DATA_SECTION(Buffer,"BufferDataSection");
uint16   Buffer[WORDS_IN_FLASH_BUFFER + 1];
uint32   *Buffer32 = (uint32 *)Buffer;

//
// Function Prototypes
//
void Example_Error(Fapi_StatusType status);
void Example_Done(void);
void Example_CallFlashAPI(void);

//
// Main
//
void main(void)
{
    //
    // Initialize System Control. Enable Peripheral Clocks.
    //
    InitSysCtrl();

    //
    // Disable CPU interrupts.
    //
    DINT;

    //
    // Initialize PIE control registers to default state.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags.
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize PIE vector table with pointers to the default
    // Interrupt Service Routines (ISR).
    //
    InitPieVectTable();

    //
    // Call the Flash API functions for this example.
    //
    Example_CallFlashAPI();
}

//
// Example_CallFlashAPI - This function will interface to the flash API.
//                        Flash API functions used in this function are
//                        executed from RAM.
//
#pragma CODE_SECTION(Example_CallFlashAPI, ramFuncSection);
void Example_CallFlashAPI(void)
{
    uint32 u32Index = 0UL;
    uint16 i = 0U;
    Fapi_StatusType oReturnCheck;
    volatile Fapi_FlashStatusType oFlashStatus;
    Fapi_FlashStatusWordType oFlashStatusWord;

    //
    // Grab flash semaphore for Zone1 to enable access to flash registers.
    //
    EALLOW;
    DcsmCommonRegs.FLSEM.all = 0xA501;
    EDIS;

    //
    // Call flash initialization to setup flash waitstates.
    // This function must reside in RAM.
    //
    InitFlash();

    //
    // Disable ECC.
    //
    EALLOW;
    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0x0;

    //
    // This function is required to initialize the Flash API based on system
    // frequency before any other Flash API operation can be performed.
    //
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 120);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors.
        //
        Example_Error(oReturnCheck);
    }

    //
    // Fapi_setActiveFlashBank function sets the flash bank and FMC for further
    // flash operations to be performed on the bank.
    //
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors.
        //
        Example_Error(oReturnCheck);
    }

    //
    // Erase Sector C.
    //
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)Bzero_SectorC_start);

    //
    // Wait until FSM is done with erase sector operation.
    //
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
    {
    }

    //
    // Verify that Sector C is erased. The erase step itself does verification
    // as it goes. This verify is a second verification that can be done.
    //
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_SectorC_start,
                   Bzero_16KSector_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors.
        // If erase command fails, use Fapi_getFsmStatus() function to get the
        // FMSTAT register contents to see if any of the EV bit, ESUSP bit,
        // CSTAT bit or VOLTSTAT bit is set. Refer to API documentation for
        // more details.
        //
        Example_Error(oReturnCheck);
    }

    //
    // Erase Sector B.
    //
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)Bzero_SectorB_start);

    //
    // Wait until FSM is done with erase sector operation.
    //
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
    {
    }

    //
    // Verify that Sector B is erased. The erase step itself does verification
    // as it goes. This verify is a second verification that can be done.
    //
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_SectorB_start,
                   Bzero_16KSector_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors.
        // If erase command fails, use Fapi_getFsmStatus() function to get the
        // FMSTAT register contents to see if any of the EV bit, ESUSP bit,
        // CSTAT bit or VOLTSTAT bit is set. Refer to API documentation for
        // more details.
        //
        Example_Error(oReturnCheck);
    }

    //
    // A data buffer of max 8 words can be supplied to the program function.
    // Each word is programmed until the whole buffer is programmed or a
    // problem is found. However, to program a buffer that has more than 8
    // words, program function can be called in a loop to program 8 words for
    // each loop iteration until the whole buffer is programmed.
    //
    // For example, program 0xFF bytes in flash Sector C along with auto-
    // generated ECC.
    //

    //
    // For example fill a buffer with data to program into the flash.
    //
    for(i=0;i<=WORDS_IN_FLASH_BUFFER;i++)
    {
        Buffer[i] = i;
    }

    //
    // Program buffer into Sector C with ECC.
    //
    for(i=0, u32Index = Bzero_SectorC_start;
       (u32Index < (Bzero_SectorC_start + WORDS_IN_FLASH_BUFFER))
       && (oReturnCheck == Fapi_Status_Success); i+= 8, u32Index+= 8)
    {
        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,
                       8,
                       0,
                       0,
                       Fapi_AutoEccGeneration);

        //
        // Wait until FSM is done with program operation.
        //
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
        {
        }

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors.
            //
            Example_Error(oReturnCheck);
        }

        //
        // Read FMSTAT register contents to know the status of FSM after
        // program command for any debug.
        //
        oFlashStatus = Fapi_getFsmStatus();

        //
        // Verify the values programmed. The program step itself does
        // verification as it goes. This verify is a second verification that
        // can be done.
        //
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                       4,
                       Buffer32+(i/2),
                       &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors.
            //
            Example_Error(oReturnCheck);
        }
    }

    //
    // Program buffer into Sector B with out ECC.
    // Disable ECC so that an error is not generated when reading flash contents
    // without ECC programmed.
    //
    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0x0;

    for(i=0, u32Index = Bzero_SectorB_start;
       (u32Index < (Bzero_SectorB_start + WORDS_IN_FLASH_BUFFER))
       && (oReturnCheck == Fapi_Status_Success); i+= 8, u32Index+= 8)
    {
        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,
                       Buffer+i,
                       8,
                       0,
                       0,
                       Fapi_DataOnly);

        //
        // Wait until FSM is done with program operation.
        //
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy)
        {
        }

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors.
            //
            Example_Error(oReturnCheck);
        }

        //
        // Read FMSTAT register contents to know the status of FSM
        // after program command for any debug.
        //
        oFlashStatus = Fapi_getFsmStatus();

        //
        // Verify the values programmed. The program step itself does
        // verification as it goes. This verify is a second verification that
        // can be done.
        //
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                       4,
                       Buffer32+(i/2),
                       &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors.
            //
            Example_Error(oReturnCheck);
        }
    }

    //
    // Erase the sectors that we have programmed above.
    // Erase Sector C.
    //
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)Bzero_SectorC_start);

    //
    // Wait until FSM is done with erase sector operation.
    //
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
    {
    }

    //
    // Verify that Sector C is erased. The erase step itself does verification
    // as it goes. This verify is a second verification that can be done.
    //
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_SectorC_start,
                   Bzero_16KSector_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors.
        // If erase command fails, use Fapi_getFsmStatus() function to get the
        // FMSTAT register contents to see if any of the EV bit, ESUSP bit,
        // CSTAT bit or VOLTSTAT bit is set. Refer to API documentation for
        // more details.
        //
        Example_Error(oReturnCheck);
    }

    //
    // Erase Sector B.
    //
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)Bzero_SectorB_start);

    //
    // Wait until FSM is done with erase sector operation.
    //
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
    {
    }

    //
    // Verify that Sector B is erased. The erase step itself does verification
    // as it goes. This verify is a second verification that can be done.
    //
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_SectorB_start,
                   Bzero_16KSector_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors.
        // If erase command fails, use Fapi_getFsmStatus() function to get the
        // FMSTAT register contents to see if any of the EV bit, ESUSP bit,
        // CSTAT bit or VOLTSTAT bit is set. Refer to API documentation for
        // more details.
        //
        Example_Error(oReturnCheck);
    }

    //
    // Enable ECC.
    //
    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0xA;

    //
    // Release flash semaphore.
    //
    DcsmCommonRegs.FLSEM.all = 0xA500;

    EDIS;

    //
    // Example is done here.
    //
    Example_Done();
}

//
// Example_Error - For this example, if an error is found just stop here
//
#pragma CODE_SECTION(Example_Error,ramFuncSection);
void Example_Error(Fapi_StatusType status)
{
    //
    // Error code will be in the status parameter.
    // ESTOP0.
    //
    __asm("    ESTOP0");
}

//
// Example_Done - For this example, once we are done just stop here
//
#pragma CODE_SECTION(Example_Error,ramFuncSection);
void Example_Done(void)
{
    //
    // ESTOP0.
    //
    __asm("    ESTOP0");

    for(;;)
    {
    }
}

//
// End of file
//
