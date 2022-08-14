//###########################################################################
//
// FILE:   F2837xD_usb_flash_kernel_cpu02.c
//
// TITLE:  Firmware Upgrade using USB Kernel for Dual Upgrade for F2837xD.
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
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include <string.h>
#include "flash_programming_c28.h" // Flash API example header file
#include "c2_bootrom.h"
#include "F021_F2837xD_C28x.h"

//
// Function Prototypes
//
void Init_Flash_Sectors(void);
extern void Example_Error(Fapi_StatusType status);
Uint32 Secondary_Boot(void);

//
// Main
//
uint32_t main(void)
{
//
// Step 1. Initialize System Control:
// Enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
//    InitSysCtrl(); // PLL would be locked by CPU1 and so would be
                     // the rest of clockinits

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
//    InitGpio();

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
//    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
//    InitPieVectTable();

//
// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
//
    InitFlash();

    IpcRegs.IPCBOOTSTS = C2_BOOTROM_BOOTSTS_SYSTEM_READY;
    while(IpcRegs.IPCSTS.bit.IPC15 == 0){}
    IpcRegs.IPCSET.bit.IPC15 = 1;

    EINT; //enable global interrupt INTM
    ERTM; //enable global realtime interrupt

//
//  Gain pump semaphore
//
    SeizeFlashPump();

    Init_Flash_Sectors();

    uint32_t EntryAddr;
    EntryAddr = Secondary_Boot();

//
// Leave control over flash pump
//
    ReleaseFlashPump();

//
// IPC flag 5 is meant for exit condition
// IPC flag 10  is meant for data
//
    IpcRegs.IPCSET.bit.IPC5 = 1;
    while(IpcRegs.IPCSTS.bit.IPC5 == 1){}

    asm(" NOP");
    asm(" NOP");

    while(IpcRegs.IPCSTS.bit.IPC5 == 0){}

    asm(" NOP");
    asm(" NOP");

//
// Load entry address of application into RPC: return program counter
//
    return EntryAddr;
}

//
// Init_Flash_Sectors - Initialize flash API and active flash bank sectors
//
void Init_Flash_Sectors(void)
{
    EALLOW;
    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0x0;
    Fapi_StatusType oReturnCheck;

    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 150);

    if(oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    if(oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0xA;
    EDIS;
}

//
// Example_Error - For this example, if an error is found just stop here
//
#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
        #pragma CODE_SECTION(Example_Error,".TI.ramfunc");
    #else
        #pragma CODE_SECTION(Example_Error,"ramfuncs");
    #endif
#endif
void Example_Error(Fapi_StatusType status)
{
    //
    //  Error code will be in the status parameter
    //
    __asm("    ESTOP0");
}

//
// End of file
//
