//###########################################################################
//
// FILE:    Secondary_Boot.c
//
// TITLE:   Secondary Boot mode routines
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
#include "F2837xD_device.h"

//
// Function Prototypes
//
Uint16 S_GetWordData(void);
extern void CopyData(void);
Uint32 GetLongData(void);
extern void ReadReservedFn(void);

//
// Secondary_Boot - Perform secondary boot routine
//
Uint32 Secondary_Boot(void)
{
   Uint32 EntryAddr;

   //
   // GetWordData is a pointer to a function.
   //
   GetWordData = S_GetWordData;

   EntryAddr = GetLongData();

   CopyData();

   Uint16 x;
   for(x = 0; x < 32767; x++)
   {
       asm(" NOP");
       asm(" NOP");
   }

   return EntryAddr;
}

//
// S_GetWordData - Since this is run on CPU2, it does not have
//                 native access to the USB stream. So we will
//                 have to use MSGxRAM or IPCSENDDATA register.
//                 IPCSENDDATA is a 32-bit register; this is used
//                 in echoback.
//                 To read, we use IPCRECVDATA. Flag number is
//                 IPC10.
//
Uint16 S_GetWordData()
{
   Uint16 wordData;
   Uint32 readData; //keeper of data

   wordData = 0x0000;

   while(IpcRegs.IPCSTS.bit.IPC10 == 0)
   {
       if(IpcRegs.IPCFLG.bit.IPC10 == 0)
       {
           IpcRegs.IPCSET.bit.IPC10 = 1;
       }
   }

   readData = IpcRegs.IPCRECVDATA;
   IpcRegs.IPCACK.bit.IPC10 = 1;
   IpcRegs.IPCSET.bit.IPC10 = 1;

   wordData = readData & 0xffff;

   return wordData;
}

//
// End of file
//
