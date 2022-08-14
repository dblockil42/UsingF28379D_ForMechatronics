//###########################################################################
//
//
// FILE:    Fapi_UserDefinedFunctions.c
//
//
// Description: Contains all user defined functions that the Fapi
//              functions use.
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

//
// Uncomment the appropriate include file for your device
//
//#include "F021_FMC_BE.h"
//#include "F021_FMC_LE.h"
//#include "F021_Concerto_C28x.h"
//#include "F021_Concerto_Cortex.h"
#include "F021_F2837xD_C28x.h"

#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000 
        #define ramFuncSection ".TI.ramfunc"
    #else
        #define ramFuncSection "ramfuncs"
    #endif
#endif

//
// Fapi_serviceWatchdogTimer - Set watchdog timer
//
#pragma CODE_SECTION(Fapi_serviceWatchdogTimer,ramFuncSection);
Fapi_StatusType Fapi_serviceWatchdogTimer(void)
{
   //
   // User to add their own watchdog servicing code here
   //
   return(Fapi_Status_Success);
}

//
// Fapi_setupEepromSectorEnable - Enable flash bank sectors
//
#pragma CODE_SECTION(Fapi_setupEepromSectorEnable,ramFuncSection);
Fapi_StatusType Fapi_setupEepromSectorEnable(void)
{
   //
   // Value must be 0xFFFF to enable erase and programming of the
   // EEPROM bank, 0 to disable
   //
   Fapi_GlobalInit.m_poFlashControlRegisters->Fbse.u32Register = 0xFFFF;

   //
   // Enables sectors 32-63 for bank and sector erase
   //
   FAPI_WRITE_LOCKED_FSM_REGISTER(Fapi_GlobalInit.m_poFlashControlRegisters->FsmSector.u32Register, 0x0U);

   //
   // Enables sectors 0-31 for bank and sector erase
   //
   FAPI_WRITE_LOCKED_FSM_REGISTER(Fapi_GlobalInit.m_poFlashControlRegisters->FsmSector1.u32Register, 0x0U);

   //
   // Enables sectors 32-63 for bank and sector erase
   //
   FAPI_WRITE_LOCKED_FSM_REGISTER(Fapi_GlobalInit.m_poFlashControlRegisters->FsmSector2.u32Register, 0x0U);

   return(Fapi_Status_Success);
}

//
// Fapi_setupBankSectorEnable - Enable flash bank sectors
//
#pragma CODE_SECTION(Fapi_setupBankSectorEnable,ramFuncSection);
Fapi_StatusType Fapi_setupBankSectorEnable(void)
{
   //
   // Enable sectors 0-15 for erase and programming
   //
   Fapi_GlobalInit.m_poFlashControlRegisters->Fbse.u32Register = 0xFFFF;
   FAPI_WRITE_LOCKED_FSM_REGISTER(Fapi_GlobalInit.m_poFlashControlRegisters->FsmSector.u32Register, 0x0U);

   return(Fapi_Status_Success);
}

//
// End of file
//
