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
// $TI Release:  $
// $Release Date:  $
// $Copyright:
// Copyright (C) 2009-2022 Texas Instruments Incorporated - http://www.ti.com/
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
#include "Boot.h"

//
// Function Prototypes
//
inline void SCIA_Init(void);
inline void SCIA_AutobaudLock(void);
Uint16 SCIA_GetWordData(void);
Uint16 SCIA_GetOnlyWordData(void);

//
// External functions
//
extern void CopyData(void);
Uint32 GetLongData(void);
extern void ReadReservedFn(void);
extern unsigned int checksum;

//
// SCI_Boot - This module is the main SCI boot routine. It will load code
// via the SCI-A port. It will return a entry point address back to the 
// InitBoot routine which in turn calls the ExitBoot routine.
//
Uint32
SCI_Boot()
{
    Uint32 EntryAddr;

    //
    // Assign GetWordData to the SCI-A version of the function. GetOnlyWordData
    // is a pointer to a function. This version doesn't send echo back each 
    // character.
    //
    GetOnlyWordData = SCIA_GetOnlyWordData;

    SCIA_Init();
    SCIA_AutobaudLock();
    checksum = 0;

    //
    // If the KeyValue was invalid, abort the load and return the flash entry
    // point.
    //
    if (SCIA_GetOnlyWordData() != 0x08AA)
    {
        return FLASH_ENTRY_POINT;
    }

    ReadReservedFn();

    EntryAddr = GetLongData();
    CopyData();

    return EntryAddr;
}

//
// SCIA_Init -  Initialize the SCI-A port for communications with the host
//
inline void
SCIA_Init()
{
    //
    // Enable the SCI-A clocks
    //
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK=1;
    SysCtrlRegs.LOSPCP.all = 0x0002;
    SciaRegs.SCIFFTX.all=0x8000;
    
    //
    // 1 stop bit, No parity, 8-bit character, No loopback
    //
    SciaRegs.SCICCR.all = 0x0007;
    
    //
    // Enable TX, RX, Use internal SCICLK
    //
    SciaRegs.SCICTL1.all = 0x0003;
    
    //
    // Disable RxErr, Sleep, TX Wake, Diable Rx Interrupt, Tx Interrupt
    //
    SciaRegs.SCICTL2.all = 0x0000;
    
    //
    // Relinquish SCI-A from reset
    //
    SciaRegs.SCICTL1.all = 0x0023;
    
    //
    // Enable pull-ups on SCI-A pins
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
    //GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;
    GpioCtrlRegs.GPAPUD.all &= 0xCFFFFFFF;
    
    //
    // Enable the SCI-A pins
    //
    //GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
    //GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;
    GpioCtrlRegs.GPAMUX2.all |= 0x05000000;
    
    //
    // Input qual for SCI-A RX is asynch
    //
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
    EDIS;
    return;
}

//
// SCIA_AutobaudLock - Perform autobaud lock with the host. Note that if 
// autobaud never occurs the program will hang in this routine as there
// is no timeout mechanism included.
//
inline void
SCIA_AutobaudLock()
{
    Uint16 byteData;

    //
    // Must prime baud register with >= 1
    //
    SciaRegs.SCILBAUD = 1;
    
    //
    // Prepare for autobaud detection. Set the CDC bit to enable autobaud 
    // detection and clear the ABD bit
    //
    SciaRegs.SCIFFCT.bit.CDC = 1;
    SciaRegs.SCIFFCT.bit.ABDCLR = 1;
    
    //
    // Wait until we correctly read an 'A' or 'a' and lock
    //
    while(SciaRegs.SCIFFCT.bit.ABD != 1)
    {
        
    }
    
    //
    // After autobaud lock, clear the ABD and CDC bits
    //
    SciaRegs.SCIFFCT.bit.ABDCLR = 1;
    SciaRegs.SCIFFCT.bit.CDC = 0;
    
    while(SciaRegs.SCIRXST.bit.RXRDY != 1)
    {
        
    }
    byteData = SciaRegs.SCIRXBUF.bit.RXDT;
    SciaRegs.SCITXBUF = byteData;

    return;
}

//
// SCIA_GetWordData - This routine fetches two bytes from the SCI-A port and 
// puts them together to form a single 16-bit value.  It is assumed that the 
// host is sending the data in the order LSB followed by MSB.
//
Uint16
SCIA_GetWordData()
{
    Uint16 wordData;
    Uint16 byteData;

    wordData = 0x0000;
    byteData = 0x0000;

    //
    // Fetch the LSB and verify back to the host
    //
    while(SciaRegs.SCIRXST.bit.RXRDY != 1)
    {
        
    }
    wordData =  (Uint16)SciaRegs.SCIRXBUF.bit.RXDT;
    SciaRegs.SCITXBUF = wordData;

    //
    // Fetch the MSB and verify back to the host
    //
    while(SciaRegs.SCIRXST.bit.RXRDY != 1)
    {
        
    }
    byteData =  (Uint16)SciaRegs.SCIRXBUF.bit.RXDT;
    SciaRegs.SCITXBUF = byteData;

    checksum += wordData + byteData;

    //
    // form the wordData from the MSB:LSB
    //
    wordData |= (byteData << 8);

    return wordData;
}

//
// SCIA_GetOnlyWordData - 
//
Uint16
SCIA_GetOnlyWordData()
{
    Uint16 wordData;
    Uint16 byteData;

    wordData = 0x0000;
    byteData = 0x0000;

    //
    // Fetch the LSB and verify back to the host
    //
    while(SciaRegs.SCIRXST.bit.RXRDY != 1)
    {
        
    }
    wordData =  (Uint16)SciaRegs.SCIRXBUF.bit.RXDT;
    //SciaRegs.SCITXBUF = wordData;

    //
    // Fetch the MSB and verify back to the host
    //
    while(SciaRegs.SCIRXST.bit.RXRDY != 1)
    {
        
    }
    byteData =  (Uint16)SciaRegs.SCIRXBUF.bit.RXDT;
    //SciaRegs.SCITXBUF = byteData;

    checksum += wordData + byteData;

    //
    // form the wordData from the MSB:LSB
    //
    wordData |= (byteData << 8);

    return wordData;
}

//
// End of File
//

