//###########################################################################
//
// FILE:   F2837xD_usb_flash_kernels_cpu01.c
//
// TITLE:  Firmware Upgrade Kernels using USB for F2837xD.
//
//! \addtogroup dual_example_list
//! <h1>Firmware Upgrade Kernels using USB for Single or Dual Upgrade</h1>
//!
//! \b Build \b Configuration: \b DUAL
//!
//! In this example, we set up a USB connection with a host, receive a
//! binary application for CPU01 in sci8 format to run on the device and
//! program it into Flash.  Then CPU01 receiver a CPU02 kernel and loads that
//! into Shared RAM. This kernel should be linked to run from RAMGS2 and
//! RAMGS3.  CPU01 then boots CPU02 with an IPC message and tells it to branch
//! to address $0x0000E000$. CPU01 continues to receive another binary
//! application to be run in CPU02 Flash and it transmits the binary
//! application to CPU02 through IPC. CPU02 reads the application from IPC and
//! programs it into Flash.  After CPU01 and CPU02 complete, they both branch
//! to their respective applications programmed in their respective Flash
//! Banks.
//!
//! \b Build \b Configuration: \b CPU01_RAM
//!
//! In this example, we set up a USB connection with a host, receive a
//! binary application for CPU01 in hex boot format to run on the device and
//! program it into Flash.
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

//
// Included Files
//
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include <string.h>
#include "flash_programming_c28.h" // Flash API example header file
#include "c1_bootrom.h"
#include "F021_F2837xD_C28x.h"

//
// Defines
//
#define C1C2_BROM_IPC_EXECUTE_BOOTMODE_CMD    0x00000013
#define C1C2_BROM_BOOTMODE_BOOT_FROM_RAM      0x0000000A
#define C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH    0x0000000B
#define IPC_BRANCH                            0x00000011

//
// Function Prototypes
//
#ifdef DUAL
Uint32 USB_Boot(Uint16 bootMode);
void USB_Boot2(void);
void assignSharedRAMstoCPU2(void);
#endif
extern void Example_Error(Fapi_StatusType status);
void Init_Flash_Sectors(void);

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
    InitSysCtrl(); //PLL activates

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio();

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
    InitPieCtrl();

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
    InitPieVectTable();
    InitFlash();
#ifdef DUAL
    InitIpc();
#endif

//
// Gain pump semaphore
//
    SeizeFlashPump();

    Init_Flash_Sectors();

//
// Loads CPU01 application into FLASH
//
    uint32_t EntryAddr = USB_Boot(USB_BOOT);

//
// Leave control over flash pump
//
    ReleaseFlashPump();

#ifdef DUAL
    EALLOW;

    DevCfgRegs.CPU2RESCTL.all = 0xA5A50001;
    __asm("  RPT #5 || NOP");
    DevCfgRegs.CPU2RESCTL.all = 0xA5A50000;

    while(1)
    {
        uint32_t bootStatus = IPCGetBootStatus() & 0x0000000F;

        if (bootStatus == C2_BOOTROM_BOOTSTS_SYSTEM_READY)
        {
            break;
        }
        //
        //if CPU2 is already booted, then fail
        //
        else if (bootStatus == C2_BOOTROM_BOOTSTS_C2TOC1_BOOT_CMD_ACK)
        {
            Example_Error(Fapi_Error_Fail);
        }
    }

    //
    // Loop until CPU02 control system IPC flags 1 and 32 are available
    //
    while(IPCLtoRFlagBusy(IPC_FLAG0) | IPCLtoRFlagBusy(IPC_FLAG31)){}

    IpcRegs.IPCSET.all = 0x80000001; //(CPU1 to CPU2 IPC flag register)
    EDIS;

    Uint16 x;
    EALLOW;
    //
    //Hand control of thet GSxRAM over to CPU2
    //
    assignSharedRAMstoCPU2();

    for(x = 0; x < 32767/2; x++)
    {
            asm(" NOP");
            asm(" NOP");
    }

    IpcRegs.IPCSET.bit.IPC15 = 1; //setting the flag to allow CPU2 to run
    for(x = 0; x < 32767/2; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }

    while(IpcRegs.IPCFLG.bit.IPC15 == 0){}

    //
    //Handshake data to CPU2 for application
    //
    USB_Boot2();

    while(IpcRegs.IPCSTS.bit.IPC5 != 1)
    {
        //
        //continues until CPU2 application is finished
        //
    }

    IpcRegs.IPCCLR.bit.IPC5 = 1; //clearing the acknowledgement flag
    EDIS;

    IpcRegs.IPCSET.bit.IPC5 = 1;
#endif
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
// assignSharedRAMstoCPU2 - Assign shared RAM to CPU2 control
//
#ifdef DUAL
void assignSharedRAMstoCPU2(void)
{
    EALLOW;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS2 = 1; //give GS RAM access to CPU02
    MemCfgRegs.GSxMSEL.bit.MSEL_GS3 = 1;

    IpcRegs.IPCSENDADDR = 0x0000E000; //Global shared RAMs 2 and 3
    IpcRegs.IPCSENDCOM = IPC_BRANCH;  //tells CPU2 to branch to address
    IpcRegs.IPCSET.all = 0x80000001;  //(CPU1 to CPU2 IPC flag register)
    EDIS;
}
#endif

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
    // Error code will be in the status parameter
    //
    __asm("    ESTOP0");
}

//
// End of file
//
