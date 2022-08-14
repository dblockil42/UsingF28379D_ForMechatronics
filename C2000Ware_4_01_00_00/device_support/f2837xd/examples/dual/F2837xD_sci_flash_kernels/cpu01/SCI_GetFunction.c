//###########################################################################
//
//
// FILE:    SCI_GetFunction.c
//
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
#include "F2837xD_Examples.h"
#include "F2837xD_Gpio_defines.h"
#include "F2837xD_GlobalPrototypes.h"
#include "Shared_Erase.h"
#include "Shared_Verify.h"
#include "F2837xD_dcsm.h"

//
// Defines
//
#define DFU_CPU1                    0x0100
#define DFU_CPU2                    0x0200
#define ERASE_CPU1                  0x0300
#define ERASE_CPU2                  0x0400
#define VERIFY_CPU1                 0x0500
#define VERIFY_CPU2                 0x0600
#define CPU1_UNLOCK_Z1              0x000A
#define CPU1_UNLOCK_Z2              0x000B
#define CPU2_UNLOCK_Z1              0x000C
#define CPU2_UNLOCK_Z2              0x000D
#define RUN_CPU1                    0x000E
#define RESET_CPU1                  0x000F
#define RUN_CPU1_BOOT_CPU2          0x0004
#define RESET_CPU1_BOOT_CPU2        0x0007
#define RUN_CPU2                    0x0010
#define RESET_CPU2                  0x0020

#define NO_ERROR                    0x1000
#define BLANK_ERROR                 0x2000
#define VERIFY_ERROR                0x3000
#define PROGRAM_ERROR               0x4000
#define COMMAND_ERROR               0x5000
#define UNLOCK_ERROR                0x6000

#define INCORRECT_DATA_BUFFER_LENGTH         0x7000
#define INCORRECT_ECC_BUFFER_LENGTH          0x8000
#define DATA_ECC_BUFFER_LENGTH_MISMATCH      0x9000
#define FEATURE_NOT_AVAILABLE                0xB000
#define FAILURE                              0xE000
#define NOT_RECOGNIZED                       0xF000

#define ACK                         0x2D
#define NAK                         0xA5

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
StatusCode statusCode;

Uint16 checksum;

//
// Function Prototypes
//
Uint16 SCIA_GetWordData(void);
Uint16 SCIA_GetOnlyWordData(void);
void SendACK(void);
void SendNAK(void);
inline Uint16 SCIA_GetACK(void);
inline void SCIA_Flush(void);
void SCIA_Init(Uint32  BootMode);
void SCIA_AutobaudLock(void);
void SCI_Pinmux_Option1(void);
void SCI_Pinmux_Option2(void);
Uint32 SCI_GetFunction(Uint32  BootMode);
Uint16 SCI_GetPacket(Uint16* length, Uint16* data);
Uint16 SCI_SendPacket(Uint16 command, Uint16 status, Uint16 length,
                      Uint16* data1, Uint16 flashAPIError, Uint16* data2);
void SCI_SendWord(Uint16 word);
void SCI_SendChecksum(void);
extern void Boot_CPU2(Uint32 BootMode);
extern Uint32 SCI_Boot(Uint32 BootMode);
extern void Shared_Erase(Uint32 sectors);
extern void CopyData(void);
extern void VerifyData(void);
extern Uint32 GetLongData(void);
extern void ReadReservedFn(void);


//
// SCI_GetFunction - This function first initializes SCIA and performs
//                   an autobaud lock. It contains a while loop waiting on
//                   commands from the host.  It processes each
//                   command and sends a response except for Run and
//                   Reset commands.  On Run the kernel exits and branches
//                   to the Entry Point.  On Reset, the kernel exits the
//                   while loop and does a WatchDog Time-out.
//
Uint32 SCI_GetFunction(Uint32  BootMode)
{
    Uint32 EntryAddr;
    Uint16 command;
    Uint16 data[10]; // 16*10 = 128 + 32
    Uint16 length;

    //
    // read CCNF0 register to check if SCI is enabled or not
    //
    if((DevCfgRegs.DC8.bit.SCI_A != 0x01))
    {
        return 0xFFFFFFFF;
    }

    //
    // Assign GetWordData to the SCI-A version of the
    // function. GetWordData is a pointer to a function.
    //
    GetWordData = SCIA_GetWordData;

    //
    // Initialize the SCI-A port for communications
    // with the host.
    //
    // parameter SCI_BOOT for GPIO84,85; parameter SCI_BOOT_ALTERNATE
    // for GPIO28,29
    //
    SCIA_Init(BootMode);
    SCIA_AutobaudLock();

    command = SCI_GetPacket(&length, data);

    while(command != RESET_CPU1)
    {
        //
        // Reset the statusCode.
        //
        statusCode.status = NO_ERROR;
        statusCode.address = 0x12345678;
        checksum = 0;

        //
        // CPU1_UNLOCK_Z1
        //
        if(command == CPU1_UNLOCK_Z1)
        {
            Uint32 password0 = (Uint32)data[0] | ((Uint32)data[1] << 16);
            Uint32 password1 = (Uint32)data[2] | ((Uint32)data[3] << 16);
            Uint32 password2 = (Uint32)data[4] | ((Uint32)data[5] << 16);
            Uint32 password3 = (Uint32)data[6] | ((Uint32)data[7] << 16);

            //
            // Unlock the device
            //
            DcsmZ1Regs.Z1_CSMKEY0 = password0;
            DcsmZ1Regs.Z1_CSMKEY1 = password1;
            DcsmZ1Regs.Z1_CSMKEY2 = password2;
            DcsmZ1Regs.Z1_CSMKEY3 = password3;

            if(DcsmZ1Regs.Z1_CR.bit.UNSECURE == 0) //0 = Locked
            {
                statusCode.status = UNLOCK_ERROR;
            }
        }
        //
        // CPU1_UNLOCK_Z2
        //
        else if(command == CPU1_UNLOCK_Z2)
        {
            Uint32 password0 = (Uint32)data[0] | ((Uint32)data[1] << 16);
            Uint32 password1 = (Uint32)data[2] | ((Uint32)data[3] << 16);
            Uint32 password2 = (Uint32)data[4] | ((Uint32)data[5] << 16);
            Uint32 password3 = (Uint32)data[6] | ((Uint32)data[7] << 16);

            //
            // Unlock the device
            //
            DcsmZ2Regs.Z2_CSMKEY0 = password0;
            DcsmZ2Regs.Z2_CSMKEY1 = password1;
            DcsmZ2Regs.Z2_CSMKEY2 = password2;
            DcsmZ2Regs.Z2_CSMKEY3 = password3;

            if(DcsmZ2Regs.Z2_CR.bit.UNSECURE == 0) //0 = Locked
            {
                statusCode.status = UNLOCK_ERROR;
            }
        }
        //
        // DFU_CPU1
        //
        else if(command == DFU_CPU1)
        {
            EntryAddr = SCI_Boot(BootMode);//loads application into CPU1 FLASH
            if(statusCode.status == NO_ERROR)
            {
                statusCode.address = EntryAddr;
            }
        }
        //
        // ERASE_CPU1
        //
        else if(command == ERASE_CPU1)
        {
            Uint32 sectors = (Uint32)(((Uint32)data[1] << 16) |
                                      (Uint32)data[0]);
            Shared_Erase(sectors);
        }
        //
        // VERIFY_CPU1
        //
        else if(command == VERIFY_CPU1)
        {
            VerifyData();
        }
        //
        // RUN_CPU1
        //
        else if(command == RUN_CPU1)
        {
            EntryAddr = (Uint32)( ((Uint32)data[1] << 16) | (Uint32)data[0]);
            return(EntryAddr);
        }
        //
        // RUN_CPU1_BOOT_CPU2
        //
        else if(command == RUN_CPU1_BOOT_CPU2)
        {
            Boot_CPU2(BootMode);
            EntryAddr = (Uint32)( ((Uint32)data[1] << 16) | (Uint32)data[0]);
            return(EntryAddr);
        }
        //
        // RESET_CPU1_BOOT_CPU2
        //
        else if(command == RESET_CPU1_BOOT_CPU2)
        {
            Boot_CPU2(BootMode);
            break;//break the loop so it goes to reset
        }
        //
        // COMMAND_ERROR
        //
        else
        {
            statusCode.status = COMMAND_ERROR;
        }
        //
        // send the packet and if NAK send again.
        //
        while(SCI_SendPacket(command, statusCode.status, 12,
                             (Uint16*)&statusCode.address, statusCode.flashAPIError,
                             (Uint16*)&statusCode.flashAPIFsmStatus))
        {}

        command = SCI_GetPacket(&length, data); //get next packet

    }
    //
    // RESET_CPU1
    //
    // Reset with WatchDog Timeout
    //
    EALLOW;
    WdRegs.SCSR.all = 0;    //enable WDRST
    WdRegs.WDCR.all = 0x28; //enable WD
    EDIS;
    while(1){}
}

//
// SCI_SendPacket - Sends a Packet to the host which contains
//                  status in the data and address.  It sends the
//                  statusCode global variable contents.  It then waits
//                  for an ACK or NAK from the host.
//
Uint16 SCI_SendPacket(Uint16 command, Uint16 status, Uint16 length,
                      Uint16* data1, Uint16 flashAPIError, Uint16* data2)
{
    int i;

    SCIA_Flush();
    DELAY_US(100000);
    SCI_SendWord(0x1BE4);
    SCI_SendWord(length);

    checksum = 0;
    SCI_SendWord(command);
    SCI_SendWord(status);

    for(i = 0; i < 2; i++)
    {
        SCI_SendWord(*(data1 + i));
    }

    SCI_SendWord(flashAPIError);

    for(i = 0; i < 2; i++)
    {
       SCI_SendWord(*(data2 + i));
    }
    
    SCI_SendChecksum();
    SCI_SendWord(0xE41B);

    //
    // Receive an ACK or NAK
    //
    return SCIA_GetACK();
}

//
// SCIA_GetACK - Gets 1-byte ACK from the host.
//
inline Uint16 SCIA_GetACK()
{
    Uint16 wordData;
    while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }

    wordData =  (Uint16)SciaRegs.SCIRXBUF.bit.SAR;
    if(wordData != ACK)
    {
        return(1);
    }

    return(0);
}

//
// SCIA_SendChecksum - Sends the Global checksum value
//
void SCI_SendChecksum()
{
    SciaRegs.SCITXBUF.bit.TXDT = (checksum & 0xFF); //LSB
    SCIA_Flush();
    SCIA_GetACK();

    SciaRegs.SCITXBUF.bit.TXDT = (checksum >> 8) & 0xFF; //MSB
    SCIA_Flush();
    SCIA_GetACK();
}

//
// SCIA_SendWord - Sends a Uint16 word.
//
void SCI_SendWord(Uint16 word)
{
    SciaRegs.SCITXBUF.bit.TXDT = (word & 0xFF); //LSB
    checksum += word & 0xFF;
    SCIA_Flush();
    SCIA_GetACK();

    SciaRegs.SCITXBUF.bit.TXDT = (word>>8) & 0xFF; //MSB
    checksum += word>>8 & 0xFF;
    SCIA_Flush();
    SCIA_GetACK();
}

//
// SCIA_Init - Initialize the SCI-A port for communications
//             with the host.
//
void SCIA_Init(Uint32  BootMode)
{
    //
    // Enable the SCI-A clocks
    //
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    ClkCfgRegs.LOSPCP.all = 0x0007;
    SciaRegs.SCIFFTX.all = 0x8000;

    //
    // 1 stop bit, No parity, 8-bit character
    // No loopback
    //
    SciaRegs.SCICCR.all = 0x0007;

    //
    // Enable TX, RX, Use internal SCICLK
    //
    SciaRegs.SCICTL1.all = 0x0003;

    //
    // Disable RxErr, Sleep, TX Wake,
    // Disable Rx Interrupt, Tx Interrupt
    //
    SciaRegs.SCICTL2.all = 0x0000;

    //
    // Relinquish SCI-A from reset
    //
    SciaRegs.SCICTL1.all = 0x0023;
    EDIS;

    //
    //Configure GPIO84 as SCITXDA (Output pin)
    //Configure GPIO85 as SCIRXDA (Input pin)
    //
    if((BootMode & 0xF0) == 0x0)
    {
        //
        //Configure GPIO84 as SCITXDA (Output pin)
        //Configure GPIO85 as SCIRXDA (Input pin)
        //
        SCI_Pinmux_Option1();
    }
    else
    {
        //
        //Configure GPIO29 as SCITXDA (Output pin)
        //Configure GPIO28 as SCIRXDA (Input pin)
        //
        SCI_Pinmux_Option2();
    }
    return;
}

//
// SCIA_AutobaudLock - Perform autobaud lock with the host.
//                     Note that if autobaud never occurs
//                     the program will hang in this routine as there
//                     is no timeout mechanism included.
//
void SCIA_AutobaudLock(void)
{
    Uint16 byteData;

    //
    // Must prime baud register with >= 1
    //
    SciaRegs.SCIHBAUD.bit.BAUD = 0;
    SciaRegs.SCILBAUD.bit.BAUD = 1;

    //
    // Prepare for autobaud detection
    // Set the CDC bit to enable autobaud detection
    // and clear the ABD bit
    //
    SciaRegs.SCIFFCT.bit.CDC = 1;
    SciaRegs.SCIFFCT.bit.ABDCLR = 1;

    //
    // Wait until we correctly read an
    // 'A' or 'a' and lock
    //
    while(SciaRegs.SCIFFCT.bit.ABD != 1) {}

    //
    // After autobaud lock, clear the ABD and CDC bits
    //
    SciaRegs.SCIFFCT.bit.ABDCLR = 1;
    SciaRegs.SCIFFCT.bit.CDC = 0;

    while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
    byteData = SciaRegs.SCIRXBUF.bit.SAR;
    SciaRegs.SCITXBUF.bit.TXDT = byteData;

    return;
}

//
// SCIA_GetWordData - This routine fetches two bytes from the SCI-A
//                    port and puts them together to form a single
//                    16-bit value.  It is assumed that the host is
//                    sending the data in the order LSB followed by MSB.
//
Uint16 SCIA_GetWordData(void)
{
   Uint16 wordData;
   Uint16 byteData;

   wordData = 0x0000;
   byteData = 0x0000;

   //
   // Fetch the LSB and verify back to the host
   //
   while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
   wordData =  (Uint16)SciaRegs.SCIRXBUF.bit.SAR;

#if !checksum_enable
   SciaRegs.SCITXBUF.bit.TXDT = wordData;
#endif

   //
   // Fetch the MSB and verify back to the host
   //
   while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
   byteData =  (Uint16)SciaRegs.SCIRXBUF.bit.SAR;

#if !checksum_enable
   SciaRegs.SCITXBUF.bit.TXDT = byteData;
#endif
#if checksum_enable
   checksum += wordData + byteData;
#endif

   //
   // form the wordData from the MSB:LSB
   //
   wordData |= (byteData << 8);

   return wordData;
}

//
// SCIA_GetOnlyWordData - This routine fetches two bytes from the SCI-A
//                        port and puts them together to form a single
//                        16-bit value.  It is assumed that the host is
//                        sending the data in the order LSB followed by MSB.
//
Uint16 SCIA_GetOnlyWordData(void)
{
   Uint16 wordData;
   Uint16 byteData;

   wordData = 0x0000;
   byteData = 0x0000;

   //
   // Fetch the LSB and verify back to the host
   //
   while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
   wordData = (Uint16)SciaRegs.SCIRXBUF.bit.SAR;
   //SciaRegs.SCITXBUF.bit.TXDT = wordData;

   //
   // Fetch the MSB and verify back to the host
   //
   while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
   byteData =  (Uint16)SciaRegs.SCIRXBUF.bit.SAR;
   //SciaRegs.SCITXBUF.bit.TXDT = byteData;

   checksum += wordData + byteData;

   //
   // form the wordData from the MSB:LSB
   //
   wordData |= (byteData << 8);

   return wordData;
}

//
// SCI_GetPacket - This routine receives the packet, returns the
//                 command and puts the data length in Uin16* length
//                 and data in Uint16* data
//
Uint16 SCI_GetPacket(Uint16* length, Uint16* data)
{
    if(SCIA_GetOnlyWordData() != 0x1BE4)
    {
        SendNAK();
        return(100); //start packet error
    }

    *length = SCIA_GetOnlyWordData();

    checksum = 0; //checksum of command and data
    Uint16 command = SCIA_GetOnlyWordData();

    int i = 0;
    for(i = 0; i < (*length)/2; i++)
    {
        *(data+i) = SCIA_GetOnlyWordData();
    }

    Uint16 dataChecksum = checksum;
    if(dataChecksum != SCIA_GetOnlyWordData())
    {
        SendNAK();
        return(101); //checksum error
    }
    if(SCIA_GetOnlyWordData() != 0xE41B)
    {
        SendNAK();
        return(102); //end packet error
    }

    SendACK();
    return(command);
}

//
// SendACK - This routine transmits ACK.
//
void SendACK(void)
{
    while(!SciaRegs.SCICTL2.bit.TXRDY)
    {
    }

    SciaRegs.SCITXBUF.bit.TXDT = ACK;
    SCIA_Flush();
}

//
// SendNAK - This routine transmits NAK.
//
void SendNAK(void)
{
    while(!SciaRegs.SCICTL2.bit.TXRDY)
    {
    }

    SciaRegs.SCITXBUF.bit.TXDT = NAK;
    SCIA_Flush();
}

//
// SCIA_Flush - This routine flushes SCIA.
//
inline void SCIA_Flush(void)
{
    while(!SciaRegs.SCICTL2.bit.TXEMPTY)
    {
    }
}

//
// SCI_Pinmux_Option1 - This routine configures GPIO84 and GPIO85
//                      as SCI pins :-
//                       1) Configure GPIO84 as SCITXDA pin
//                       2) Configure GPIO85 as SCIRXDA pin
//                       3) Configure GPIO85 as asynchronous pin
//
void SCI_Pinmux_Option1(void)
{
    //
    // Configure GPIO84 as SCITXDA (Output pin)
    // Configure GPIO85 as SCIRXDA (Input pin)
    // Configure GPIO85 (SCIRXRA) as async pin
    //
    EALLOW;
    GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(84,GPIO_MUX_CPU1,5);
    GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(85,GPIO_MUX_CPU1,5);
    EDIS;
}

//
// SCI_Pinmux_Option2 - This routine configures GPIO84 and GPIO85
//                      as SCI pins :-
//                       1) Configure GPIO29 as SCITXDA pin
//                       2) Configure GPIO28 as SCIRXDA pin
//                       3) Configure GPIO28 as asynchronous pin
//
void SCI_Pinmux_Option2(void)
{
    //
    // Configure GPIO29 as SCITXDA (Output pin)
    // Configure GPIO28 as SCIRXDA (Input pin)
    // Configure GPIO28 (SCIRXRA) as async pin
    //
    EALLOW;
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(29,GPIO_MUX_CPU1,1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(28,GPIO_MUX_CPU1,1);
    EDIS;
}

//
// End of file
//
