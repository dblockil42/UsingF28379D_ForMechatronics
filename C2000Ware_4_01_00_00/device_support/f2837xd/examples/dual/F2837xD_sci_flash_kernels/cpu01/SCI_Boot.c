//###########################################################################
//
// FILE:    SCI_Boot.c
//
// TITLE:   SCI Boot mode routines
//
// Functions:
//
//     Uint32 SCI_Boot(void)
//     inline void SCIA_Init(void)
//     inline void SCIA_AutobaudLock(void)
//     Uint32 SCIA_GetWordData(void)
//
// Notes:
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
#include "F2837xD_Ipc_drivers.h"
#include "Types.h"

//
// Defines
//
#define NO_ERROR                            0x1000
#define BLANK_ERROR                         0x2000
#define VERIFY_ERROR                        0x3000
#define PROGRAM_ERROR                       0x4000
#define COMMAND_ERROR                       0x5000
#define C1C2_BROM_IPC_EXECUTE_BOOTMODE_CMD  0x00000013
#define C1C2_BROM_BOOTMODE_BOOT_FROM_SCI    0x00000001
#define C1C2_BROM_BOOTMODE_BOOT_FROM_RAM    0x0000000A
#define C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH  0x0000000B

//
// Globals
//
typedef struct
{
   Uint16 status;
   Uint32 address;
}  StatusCode;
extern StatusCode statusCode;

//
// Function Prototypes
//
extern Uint16 SCIA_GetWordData(void);
extern Uint32 CopyData(void);
Uint32 GetLongData(void);
extern void ReadReservedFn(void);
extern void Example_Error(Fapi_StatusType status);
Uint32 SCI_Boot(Uint32 BootMode);
void Boot_CPU2(Uint32 BootMode);
void assignSharedRAMstoCPU2(void);
void Assign_SCIA_IO_CPU2(Uint32 BootMode);

//
// SCI_Boot - This module is the main SCI boot routine.
//            It will load code via the SCI-A port.
//
//            It will return a entry point address back
//            to the InitBoot routine which in turn calls
//            the ExitBoot routine.
//
Uint32 SCI_Boot(Uint32 BootMode)
{
    statusCode.status = NO_ERROR;
    statusCode.address = 0x12346578;

    Uint32 EntryAddr;

    //
    // Assign GetWordData to the SCI-A version of the
    // function. GetWordData is a pointer to a function.
    //
    GetWordData = SCIA_GetWordData;

    //
    // If the KeyValue was invalid, abort the load
    // and return the flash entry point.
    //
    if (SCIA_GetWordData() != 0x08AA)
    {
        statusCode.status = VERIFY_ERROR;
        statusCode.address = FLASH_ENTRY_POINT;
    }

    ReadReservedFn(); //reads and discards 8 reserved words

    EntryAddr = GetLongData();

    CopyData();

    Uint16 x = 0;
    for(x = 0; x < 32676; x++){}
    for(x = 0; x < 32676; x++){}

    return EntryAddr;
}

//
// Boot_CPU2 - Boot up CPU2
//
void Boot_CPU2(Uint32 BootMode)
{
    //
    // Leave control over flash pump
    //
    ReleaseFlashPump();

    EALLOW;
    while(1)
    {
        uint32_t bootStatus = IPCGetBootStatus() & 0x0000000F;

        //
        // if CPU2 is already booted, then fail
        //
        if (bootStatus == C2_BOOTROM_BOOTSTS_SYSTEM_READY)
        {
            break;
        }
        else if(bootStatus == C2_BOOTROM_BOOTSTS_C2TOC1_BOOT_CMD_ACK)
        {
            Example_Error(Fapi_Error_Fail);
        }
    }

    //
    // Loop until CPU02 control system IPC flags 1 and 32 are available
    //
    while(IPCLtoRFlagBusy(IPC_FLAG0) | IPCLtoRFlagBusy(IPC_FLAG31)){}

    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    DevCfgRegs.SOFTPRES7.bit.SCI_A = 1;

    int a = 0;
    for(a = 0; a < 32676; a++){}

    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;
    DevCfgRegs.SOFTPRES7.bit.SCI_A = 0;

    Assign_SCIA_IO_CPU2(BootMode);
    assignSharedRAMstoCPU2();

    //
    // CPU1 to CPU2 IPC Boot Mode Register
    //
    IpcRegs.IPCBOOTMODE = C1C2_BROM_BOOTMODE_BOOT_FROM_SCI;

    //
    // CPU1 to CPU2 IPC Command Register
    //
    IpcRegs.IPCSENDCOM = C1C2_BROM_IPC_EXECUTE_BOOTMODE_CMD;

    IpcRegs.IPCSET.all = 0x80000001; //(CPU1 to CPU2 IPC flag register)
    EDIS;

    EALLOW;
    while(IpcRegs.IPCSTS.bit.IPC5 != 1)
    {
        //
        //continues until CPU2 application is finished
        //
    }

    IpcRegs.IPCACK.bit.IPC5 = 1; //clearing the acknowledgement flag
    EDIS;
}

//
// Assign_SCIA_IO_CPU2 - Assign SCIA module to CPU2 control
//
void Assign_SCIA_IO_CPU2(Uint32 BootMode)
{
    EALLOW;
    DevCfgRegs.CPUSEL5.bit.SCI_A = 1;    //SCIA connected to CPU2
    ClkCfgRegs.CLKSEM.all = 0xA5A50000;  //Allows CPU2 bootrom to take control
                                         //of clock configuration registers
    ClkCfgRegs.LOSPCP.all = 0x0007;

    if((BootMode & 0xF0) == 0x0)
    {
        //
        //OPTION 1
        //
        GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);
        GPIO_SetupPinMux(84,GPIO_MUX_CPU2,5);
        GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_ASYNC);
        GPIO_SetupPinMux(85,GPIO_MUX_CPU2,5);
    }
    else
    {
        //
        //OPTION 2
        //
        GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);
        GPIO_SetupPinMux(29,GPIO_MUX_CPU2,1);
        GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_ASYNC);
        GPIO_SetupPinMux(28,GPIO_MUX_CPU2,1);
    }
    EDIS;
}

//
// assignSharedRAMstoCPU2 - Assign shared RAM GS2/GS3 to CPU2
//
void assignSharedRAMstoCPU2(void)
{
    EALLOW;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS2 = 1; //give GS RAM access to CPU02
    MemCfgRegs.GSxMSEL.bit.MSEL_GS3 = 1;
    EDIS;
}

//
// End of file
//
