//###########################################################################
//
// FILE:   USB_Boot_Funcs.h
//
// Description: USB boot loader support definitions for Soprano
//
//###########################################################################
//
// $Release Date: $
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

#ifndef __USB_BOOT_FUNCS_H__
#define __USB_BOOT_FUNCS_H__

//
// Included Files
//
#include "c1_bootrom.h"
#include "USB_Regs.h"
#include "USB_Structs.h"

//
// Defines
//
#define true                   1
#define false                  0
#define EP0_MAX_PACKET_SIZE   64   // Max packet size for endpoint 0

//
// Globals
//
typedef enum
{
    USB_STATE_IDLE,     //Waiting for a request from the host controller
    USB_STATE_TX,       //Sending data to the host due to an IN request
    USB_STATE_RX,       //Receiving data from the host due to an OUT request
    USB_STATE_STALL,    //Waiting for the host to acknowledge a stall condition
    USB_STATE_STATUS    //Waiting for the host to acknowledge completion of an
                        //IN or OUT request
} eEndpointState;

typedef enum
{
    USB_MORE_DATA,      //Do not set the DATAEND bit
    USB_END_DATA        //Set the DATAEND bit
} eDataEnd;

//
// Function Prototypes
//
interrupt void UsbIntHandler();
void AckUsbInterrupt();
static void UsbEndpoint0StateMachine();
static eEndpointState HandleControlRequest(const sUsbRequestPacket *req);
static void CopyStringDescriptor(const sUsbStringDescriptor *strDesc,
                                 Uint16 *buffer);
void ResetUsbDriver();
void TxDataEp0(const Uint16 *data, Uint16 length8, eDataEnd dataEnd);
Uint16 RxDataEp0(Uint16 *data);
static void StallEp0();
void AckEp0(eDataEnd dataEnd);
Uint16 RxDataEp1(Uint16 *data);
void AckEp1();

#endif //__USB_BOOT_FUNCS_H__

//
// End of file
//
