//###########################################################################
//
// FILE:    USB_Boot.c
//
// TITLE:   USB Boot mode routines
//
// Functions:
//
//     Uint32 USB_Boot(void)
//     Uint32 USB_GetWordData(void)
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
#include "c1_bootrom.h"
#include "F2837xD_Gpio_defines.h"
#include "USB_Boot_Funcs.h"
#include "Types.h"
#include "flash_programming_c28.h" // Flash API example header file

//
// Defines
//
#define wait while(IpcRegs.IPCSTS.bit.IPC10 == 0){}
#define acknowledge IpcRegs.IPCACK.bit.IPC10 = 1;
#define set while(IpcRegs.IPCFLG.bit.IPC10 == 0){IpcRegs.IPCSET.bit.IPC10 = 1;}

//
// Function Prototypes
//
Uint16 USB_GetWordData(void);
void USB_Init(void);
Uint32 USB_Boot(Uint16 bootMode);
void Example_Error(Fapi_StatusType status);

#ifdef DUAL
void USB_Boot2(void);
void CopyDatatoSharedRAM(void);
#endif

extern void CopyData(void);
extern Uint32 GetLongData(void);
extern void ReadReservedFn(void);
extern Uint16 *g_UsbRxBuffer;
extern Uint16 g_UsbRxDataLength;
extern Uint16 g_UsbRxPacketLength;

//
// USB_Boot - Run USB boot procedure
//
Uint32 USB_Boot(Uint16 bootMode)
{
    Uint32 EntryAddr;

    Uint16 x = 0;
    for(x = 0; x < 32767; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }

    if(DevCfgRegs.DC12.bit.USB_A == 0) //uncomment when USB BootROM available
    {
        return FLASH_ENTRY_POINT;
    }

    //
    // Assign GetWordData to the USB version of the
    // function. GetWordData is a pointer to a function.
    //
    GetWordData = USB_GetWordData;

    //
    // USB Boot
    // USB0DM - GPIO42
    // USB0DP - GPIO43
    // (the mode implemented is USB device type, Device Firmware Upgrade Class)
    //
    USB_Init();

    if(GetWordData() != 0x08AA)
    {
        Example_Error(Fapi_Error_Fail);
    }
    ReadReservedFn();

    for(x = 0; x < 32767; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }
    EntryAddr = GetLongData();
    CopyData();

    //
    //reset USB peripheral
    //
    for(x = 0; x < 32767; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }
#if 0
    CpuSysRegs.PCLKCR11.bit.USB_A = 0;
    DevCfgRegs.SOFTPRES11.bit.USB_A = 1;

    for(x = 0; x < 1000; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }
    DevCfgRegs.SOFTPRES11.bit.USB_A = 0;
    CpuSysRegs.PCLKCR11.bit.USB_A = 1;
#endif

#ifdef DUAL
    if(GetWordData() != 0x08AA)
    {
        Example_Error(Fapi_Error_Fail);
    }
    ReadReservedFn();

    for(x = 0; x < 32767; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }
    GetLongData();

    //
    //Copy data into CPU2 shared RAM
    //
    CopyDatatoSharedRAM();
#endif

#ifndef DUAL
    //
    //Disconnect from the bus, disable USB interrupts, and
    //reset the USB module. But first, wait for any ongoing
    //transfers to complete.
    //
    Uint32 disconnectDelay;

    for (disconnectDelay = 0; disconnectDelay < 10000; disconnectDelay++)
    {;}

    USBREG8(USB_O_POWER) &= ~USB_POWER_SOFTCONN;

    EALLOW;
    PieCtrlRegs.PIEIER9.bit.INTx15 = 0;
    IER &= ~M_INT9;
    DevCfgRegs.SOFTPRES11.bit.USB_A = 1;
    DevCfgRegs.SOFTPRES11.bit.USB_A = 0;
    EDIS;

    //
    //Bypass and disable the main and aux PLLs
    //
    EALLOW;
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;
    ClkCfgRegs.SYSPLLMULT.bit.IMULT = 0;
    ClkCfgRegs.SYSPLLCTL1.bit.PLLEN = 0;
    ClkCfgRegs.AUXPLLCTL1.bit.PLLCLKEN = 0;
    ClkCfgRegs.AUXPLLMULT.bit.IMULT = 0;
    ClkCfgRegs.AUXPLLCTL1.bit.PLLEN = 0;
    EDIS;
#endif

    return EntryAddr;
}

#ifdef DUAL
//
// USB_Boot2 - Execute USB boot 2 procedure
//
void USB_Boot2()
{
    Uint32 longData;
    Uint16 wordData;
    GetWordData = USB_GetWordData;
    Uint32 disconnectDelay;

    Uint16 x = 0;
    for(x = 0; x < 32767; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }
#if 0
    CpuSysRegs.PCLKCR11.bit.USB_A = 0;
    DevCfgRegs.SOFTPRES11.bit.USB_A = 1;
    for(x = 0; x < 1000; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }
    DevCfgRegs.SOFTPRES11.bit.USB_A = 0;
    CpuSysRegs.PCLKCR11.bit.USB_A = 1;
#endif

    if(GetWordData() != 0x08AA)
    {
        Example_Error(Fapi_Error_Fail);
    }

    ReadReservedFn();
    for(x = 0; x < 32767; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }

    wait; //entry address
    longData = GetLongData(); //entry address MSW
    IpcRegs.IPCSENDDATA = (longData & 0xffff0000) >> 16;
    acknowledge;
    set;
    wait;
    acknowledge;
    wait;
    IpcRegs.IPCSENDDATA = longData & 0xffff; //entry address LSW
    acknowledge;
    set;
    wait;
    acknowledge;

    struct HEADER {
        Uint16 BlockSize;
        Uint32 DestAddr;
    } BlockHeader;

    BlockHeader.BlockSize = (*GetWordData)();

    wait;

    while(BlockHeader.BlockSize != (Uint16)0x0000)
    {
        wordData = BlockHeader.BlockSize; //sending blocksize
        IpcRegs.IPCSENDDATA = wordData;
        acknowledge;
        set;
        wait;
        acknowledge;

        wait;
        longData = GetLongData(); //sending destination address MSW
        IpcRegs.IPCSENDDATA = (longData & 0xffff0000) >> 16;
        acknowledge;
        set;
        wait;
        acknowledge;

        wait;
        IpcRegs.IPCSENDDATA = longData & 0xffff; //sending dest address LSW
        acknowledge;
        set;
        wait;
        acknowledge;

        for(x = 0; x < BlockHeader.BlockSize; x++)
        {
            wait;
            wordData = (*GetWordData)();
            IpcRegs.IPCSENDDATA = wordData;
            acknowledge;
            set;
            wait;
            acknowledge;
        }

        wait;
        BlockHeader.BlockSize = (*GetWordData)();
    }

    wordData = BlockHeader.BlockSize; //send the last 0x000000
    IpcRegs.IPCSENDDATA = wordData;
    acknowledge;
    set;
    wait;
    acknowledge;

    for(x = 0; x < 32767; x++)
    {
        asm(" NOP");
        asm(" NOP");
    }

#if 0
    EntryAddr = TI_OTP_C1BROM_ESCAPE_POINT_13;
    if((EntryAddr != 0xFFFFFFFF) && (EntryAddr != 0x00000000))
    {
        //
        //if OTP is programmed, then call OTP function
        //
        ((void (*)(void))EntryAddr)();
    }
#endif

    //
    //Disconnect from the bus, disable USB interrupts, and
    //reset the USB module. But first, wait for any ongoing
    //transfers to complete.
    //
    for (disconnectDelay = 0; disconnectDelay < 10000; disconnectDelay++)
    {;}

    USBREG8(USB_O_POWER) &= ~USB_POWER_SOFTCONN;

    EALLOW;
    PieCtrlRegs.PIEIER9.bit.INTx15 = 0;
    IER &= ~M_INT9;
    DevCfgRegs.SOFTPRES11.bit.USB_A = 1;
    DevCfgRegs.SOFTPRES11.bit.USB_A = 0;
    EDIS;

    //
    //Bypass and disable the main and aux PLLs
    //
    EALLOW;
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;
    ClkCfgRegs.SYSPLLMULT.bit.IMULT = 0;
    ClkCfgRegs.SYSPLLCTL1.bit.PLLEN = 0;
    ClkCfgRegs.AUXPLLCTL1.bit.PLLCLKEN = 0;
    ClkCfgRegs.AUXPLLMULT.bit.IMULT = 0;
    ClkCfgRegs.AUXPLLCTL1.bit.PLLEN = 0;
    EDIS;

#if 0
    EntryAddr = TI_OTP_C1BROM_ESCAPE_POINT_13;
    if((EntryAddr != 0xFFFFFFFF) && (EntryAddr != 0x00000000))
    {
        //
        // if OTP is programmed, then call OTP function
        //
        ((void (*)(void))EntryAddr)();
    }
#endif
}
#endif

//
// USB_Init -  Jump starts some vital set-up for USB protocol.
//
void USB_Init()
{
    //
    //enable USB controller clock
    //initialize three layers of interrupt enables
    //peripheral interrupt, PIE interrupt, EINT/DINT
    //
    EALLOW;
    //ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 0x0;    //Use INTOSC2 (~10 MHz)
                                                       //as the main PLL
                                                       //clock source
    ClkCfgRegs.SYSPLLMULT.all = 12;                    //Set IMULT to 12,
                                                       //clear FMULT
    ClkCfgRegs.SYSPLLCTL1.bit.PLLEN = 1;               //Enable the main PLL
    ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 0;      //Set PLLSYSCLKDIV to 1
    while (ClkCfgRegs.SYSPLLSTS.bit.LOCKS != 1) {;}    //Wait for the PLL to
                                                       //lock
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1;            //Turn off the main
                                                       //PLL bypass

    //ClkCfgRegs.CLKSRCCTL2.bit.AUXOSCCLKSRCSEL = 0x1; //Use XTAL (20 MHz) as
                                                       //the aux PLL clock
                                                       //source
    ClkCfgRegs.AUXPLLMULT.all = 6;                     //Set IMULT to 6, clear
                                                       //FMULT - 120MHz
    ClkCfgRegs.AUXPLLCTL1.bit.PLLEN = 1;               //Enable the aux PLL
    ClkCfgRegs.AUXCLKDIVSEL.bit.AUXPLLDIV = 2/2;       //Set AUXPLLDIV to 2
    while (ClkCfgRegs.AUXPLLSTS.bit.LOCKS != 1) {;}    //Wait for the PLL
                                                       //to lock
    ClkCfgRegs.AUXPLLCTL1.bit.PLLCLKEN = 1;            //Turn off aux PLL
                                                       //bypass
    DevCfgRegs.DC12.bit.USB_A = 1;     //feature enabled: device only
    CpuSysRegs.PCLKCR11.bit.USB_A = 1; //module clock turned on

#if 0
    EntryAddr = TI_OTP_C1BROM_ESCAPE_POINT_13;
    if((EntryAddr != 0xffffffff) && (EntryAddr != 0x00000000))
    {
        //
        // if OTP is programmed, then call OTP function
        //
        ((void (*)(void))EntryAddr)();
    }
#endif

    //
    //Connect the PHY to the GPIO pins by setting the GPBAMSEL
    //bits for GPIOs 42 and 43. VBUS and ID are now de-spec'd
    //due to the lack of a 5V fail-safe ESD structure, so
    //GPIOs 46 and 47 are not muxed out.
    //
    GpioCtrlRegs.GPBAMSEL.bit.GPIO42 = 1;
    GpioCtrlRegs.GPBAMSEL.bit.GPIO43 = 1;

    IER = 0x0000;
    IFR = 0x0000;
    //c1brom_enable_pie_in_boot(0);
    PieVectTable.USBA_INT = &UsbIntHandler;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  //enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx15 = 1; //enable USBA interrupt(CPU1 only)
    EDIS;
    IER |= M_INT9;
    EINT;

#if 0
    EntryAddr = TI_OTP_C1BROM_ESCAPE_POINT_13;
    if((EntryAddr != 0xFFFFFFFF) && (EntryAddr != 0x00000000))
    {
        //
        //if OTP is programmed, then call OTP function
        //
        ((void (*)(void))EntryAddr)();
    }
#endif

    //
    //Reset the USB driver's global variables
    //
    ResetUsbDriver();

    //
    //Force USB device mode by setting DEVMODOTG and DEVMOD
    //
    USBREG32(USB_O_GPCS) = 0x3;

    //
    //Clear active interrupts
    //
    USBREG16(USB_O_TXIS);
    USBREG16(USB_O_RXIS);
    USBREG8(USB_O_IS);

    //
    //Set up endpoint 1 for bulk transfers with a 64-byte FIFO
    //
    USBREG8(USB_O_EPIDX) = 1;
    USBREG8(USB_O_RXFIFOSZ) = 0x03;
    USBREG16(USB_O_RXFIFOADD) = 0x100;
    USBREG8(USB_O_RXCSRH1) = 0x40;

    //
    //Enable USB interrupts for EP0 transmit/receive, EP1 receive,
    //disconnection, and reset
    //
    USBREG16(USB_O_TXIE) = 0x0001;
    USBREG16(USB_O_RXIE) = 0x0002;
    USBREG8(USB_O_IE) = (USB_IE_DISCON | USB_IE_RESET);

    //
    //Attach the USB PHY to the bus
    //
    USBREG8(USB_O_POWER) |= USB_POWER_SOFTCONN;
}

//
// CopyDatatoSharedRAM -  Copy data to shared RAM
//
#ifdef DUAL
void CopyDatatoSharedRAM()
{
    struct HEADER {
     Uint16 BlockSize;
     Uint32 DestAddr;
    } BlockHeader;

    Uint16 wordData;
    Uint16 i;

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
    while(BlockHeader.BlockSize != (Uint16)0x0000)
    {
      BlockHeader.DestAddr = GetLongData();

      for(i = 1; i <= BlockHeader.BlockSize; i++)
      {
          wordData = (*GetWordData)();
          *(Uint16 *)BlockHeader.DestAddr++ = wordData;
      }

      //
      // Get the size of the next block
      //
      BlockHeader.BlockSize = (*GetWordData)();
    }
    return;
}
#endif

//
// USB_GetWordData - Since this is run on CPU2, it does not have
//                   native access to the USB stream. So we will
//                   have to use MSGxRAM or IPCSENDDATA register.
//                   IPCSENDDATA is a 32-bit register; this is used
//                   in echoback.
//                   To read, we use IPCRECVDATA. Flag number is IPC10.
//
Uint16 USB_GetWordData()
{
    Uint16 retVal;

    //
    //Wait for the USB receive buffer to be refilled
    //
    while (g_UsbRxBuffer == 0) {;}

    //
    //Read the next data value, update the buffer pointer and length,
    //and do end-of-packet handling.
    //
    retVal = *g_UsbRxBuffer++;
    g_UsbRxPacketLength -= 2;

    if(g_UsbRxPacketLength == 0)
    {
        g_UsbRxBuffer = 0;
        AckEp1();
    }

    return retVal;
}

//
// AckUsbInterrupt - Acknowledge the USB interrupt. Defining a separate
//                   function for this keeps the driver library
//                   device-independent.
//
void AckUsbInterrupt()
{
    //
    //Soprano, USBA interrupt INT9.15
    //
    PieCtrlRegs.PIEACK.bit.ACK9 = 0x1; //group 9
}

//
// End of file
//
