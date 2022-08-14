//###########################################################################
//
//
// FILE:    USB_Boot_Funcs.c
//
//
// Description: USB boot loader support functions for Soprano -- mainly the
// interrupt handler and its sub-functions.
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
#include "USB_Boot_Funcs.h"

//
// Globals
//
static eEndpointState g_UsbEp0State;
static Uint16 g_UsbAddress;
static Uint16 *g_UsbTxData;
static Uint16 g_UsbTxLength;
static Uint16 rxBuffer[64/2];  // 64 bytes for control and bulk transactions
Uint16 *g_UsbRxBuffer;
Uint16 g_UsbRxPacketLength;
Uint16 g_UsbRxDataLength;

//
// UsbIntHandler - Main USB interrupt handler
//
interrupt void UsbIntHandler()
{
    Uint16 txStatus, rxStatus, rxLength, genStatus;

    txStatus = USBREG16(USB_O_TXIS);
    rxStatus = USBREG16(USB_O_RXIS);
    genStatus = USBREG8(USB_O_IS);

    //
    // Check for a reset from the host or device disconnection. If we find one,
    // reset any global variables.
    //
    if((genStatus & USB_IE_RESET) || (genStatus & USB_IE_DISCON))
    {
        ResetUsbDriver();
    }

    //
    // If all else is in order, check for control transfer activity on
    // endpoint 0
    //
    if(txStatus & 0x0001)
    {
        UsbEndpoint0StateMachine();
    }

    //
    // Check for received boot loader data on endpoint 1
    //
    if(rxStatus & 0x0002)
    {
        if(g_UsbRxPacketLength == 0)
        {
            rxLength = RxDataEp1(rxBuffer);
            g_UsbRxPacketLength = rxLength;
            g_UsbRxBuffer = rxBuffer;
        }
    }

    //
    // Acknowledge the interrupt
    //
    AckUsbInterrupt();
}

//
// UsbEndpoint0StateMachine - USB endpoint 0 state machine.
//                            Here's a state transition diagram:
//
//   USB_STATE_IDLE -*--> USB_STATE_TX -*-> USB_STATE_STATUS -*->USB_STATE_IDLE
//                   |                                        |
//                   |--> USB_STATE_STALL ---------->---------
//
static void UsbEndpoint0StateMachine()
{
    Uint16 epStatus;

    //
    // Get the current status of endpoint 0
    //
    epStatus = USBREG8(USB_O_CSRL0);

    //
    // Transit the states
    //
    switch (g_UsbEp0State)
    {
        case USB_STATE_IDLE:
            //
            // If there's a new packet waiting, handle it
            //
            if(epStatus & USB_CSRL0_RXRDY)
            {
                RxDataEp0(rxBuffer);
                g_UsbEp0State = HandleControlRequest((sUsbRequestPacket *)rxBuffer);
            }
            break;
        case USB_STATE_TX:
            //
            // Continue sending data to the host
            //
            if (g_UsbTxLength > EP0_MAX_PACKET_SIZE)
            {
                TxDataEp0(g_UsbTxData, EP0_MAX_PACKET_SIZE, USB_MORE_DATA);
                g_UsbTxData += EP0_MAX_PACKET_SIZE/2;
                g_UsbTxLength -= EP0_MAX_PACKET_SIZE;
            }
            else
            {
                TxDataEp0(g_UsbTxData, g_UsbTxLength, USB_END_DATA);
                g_UsbEp0State = USB_STATE_IDLE;
            }
            break;
        case USB_STATE_STALL:
            //
            // If a stall handshake has been completely sent, clear the status
            // bit and go idle
            //
            if(epStatus & USB_CSRL0_STALLED)
            {
                USBREG8(USB_O_CSRL0) &= ~USB_CSRL0_STALLED;
                g_UsbEp0State = USB_STATE_IDLE;
            }
            break;
        case USB_STATE_STATUS:
            //
            // If there's a pending address, write it to the register
            //
            USBREG8(USB_O_FADDR) = g_UsbAddress;

            //
            // If there's a new packet waiting, handle it, otherwise
            // go back to the idle state.
            //
            if (epStatus & USB_CSRL0_RXRDY)
            {
                RxDataEp0(rxBuffer);
                HandleControlRequest((sUsbRequestPacket *)rxBuffer);
            }
            else
            {
                g_UsbEp0State = USB_STATE_IDLE;
            }
            break;
        default:
            //
            // Error out?
            //
            break;
    }
}

//
// HandleControlRequest - Handle a control request
//
static eEndpointState HandleControlRequest(const sUsbRequestPacket *req)
{
    Uint16 stringBuf[50];

    if(req->bmRequestType_bRequest.reqType == USB_REQTYPE_STD)
    {
        //
        // Standard requests
        //
        switch(req->bmRequestType_bRequest.bRequest)
        {
            case USB_REQ_GET_DESCRIPTOR:
                switch (req->wValue >> 8)
                {
                    case USB_DTYPE_DEVICE:
                        g_UsbTxData = (Uint16 *)&loaderDeviceDescriptor;
                        g_UsbTxLength = loaderDeviceDescriptor.bDescriptorType_bLength & 0xFF;
                        break;
                    case USB_DTYPE_CONFIG:
                        CombineUsbDescriptors(stringBuf);
                        g_UsbTxData = stringBuf;
                        g_UsbTxLength = stringBuf[1];
                        break;
                    case USB_DTYPE_STRING:
                        //
                        // Check for an invalid string index
                        //
                        if((req->wValue & 0xFF) > 4)
                        {
                            StallEp0();
                            return USB_STATE_STALL;
                        }

                        CopyStringDescriptor(stringDescriptors[req->wValue & 0xFF], stringBuf);
                        g_UsbTxData = stringBuf;
                        g_UsbTxLength = stringBuf[0] & 0xFF;
                        break;
                    case USB_DTYPE_INTERFACE:
                    default:
                        StallEp0();
                        return USB_STATE_STALL;
                }

                AckEp0(USB_MORE_DATA);

                if (g_UsbTxLength > req->wLength)
                {
                    g_UsbTxLength = req->wLength;
                }

                if (g_UsbTxLength > EP0_MAX_PACKET_SIZE)
                {
                    TxDataEp0(g_UsbTxData, EP0_MAX_PACKET_SIZE, USB_MORE_DATA);
                    g_UsbTxData += EP0_MAX_PACKET_SIZE/2;
                    g_UsbTxLength -= EP0_MAX_PACKET_SIZE;

                    return USB_STATE_TX;
                }
                else
                {
                    TxDataEp0(g_UsbTxData, g_UsbTxLength, USB_END_DATA);
                    return USB_STATE_IDLE;
                }
                //
                //Always returns elsewhere -- no fall-through
                //
            case USB_REQ_SET_ADDRESS:
                //
                //Ack the request, but the address can't be set until the host
                //sends a zero-length packet to follow up. Save it for later.
                //
                AckEp0(USB_END_DATA);
                g_UsbAddress = req->wValue;
                return USB_STATE_STATUS;
            case USB_REQ_GET_STATUS:
                //
                //The only relevant status bit is the bit 0 (self-powered), so
                //just send that.
                //
                AckEp0(USB_MORE_DATA);
                stringBuf[0] = 0x0001;
                TxDataEp0(stringBuf, 2, USB_END_DATA);
                return USB_STATE_IDLE;
            case USB_REQ_SET_CONFIG:
                //
                //There's only one configuration, so cheat and just ack
                //the request
                //
                AckEp0(USB_END_DATA);
                return USB_STATE_IDLE;
            case USB_REQ_CLEAR_FEATURE:
            case USB_REQ_SET_FEATURE:
            case USB_REQ_SET_DESCRIPTOR:
            case USB_REQ_GET_CONFIG:
            case USB_REQ_GET_INTERFACE:
            case USB_REQ_SET_INTERFACE:
            default:
                StallEp0();
                return USB_STATE_STALL;
        }
    }
    else
    {
        //
        //Unknown request
        //
        StallEp0();
        return USB_STATE_STALL;
    }
}

//
// CopyStringDescriptor - Copy a string descriptor into a temporary buffer
//
static void CopyStringDescriptor(const sUsbStringDescriptor *strDesc,
                                 Uint16 *buffer)
{
    Uint16 strLen, c;

    strLen = (strDesc->bDescriptorType_bLength & 0xFF) - 2;
    buffer[0] = strDesc->bDescriptorType_bLength;

    for(c = 0; c < strLen; c++)
    {
        __byte((int *)buffer, c+2) = __byte((int *)strDesc->bString, c);
    }
}

//
// ResetUsbDriver - Reset all of the global variables. This is also called by
//                  USB_Boot() since we don't get initialization for free in
//                  the boot ROM.
//
void ResetUsbDriver()
{
    g_UsbEp0State = USB_STATE_IDLE;
    g_UsbAddress = 0;
    g_UsbTxData = 0;
    g_UsbTxLength = 0;

    g_UsbRxBuffer = 0;
    g_UsbRxPacketLength = 0;
    g_UsbRxDataLength = 0;
}

//
// TxDataEp0 - Transmit data from USB endpoint 0
//
void TxDataEp0(const Uint16 *data, Uint16 length8, eDataEnd dataEnd)
{
    Uint16 c;

    //
    //Write the data to the FIFO
    //
    for (c = 0; c < length8; c++)
    {
        USBREG8(USB_O_FIFO0) = __byte((int *)data, c);
    }

    //
    //Kick off the transmission. If there's no more data, set the end bit.
    //
    USBREG8(USB_O_CSRL0) = (dataEnd == USB_END_DATA) ?
                           (USB_CSRL0_TXRDY |
                            USB_CSRL0_DATAEND) : USB_CSRL0_TXRDY;
}

//
// RxDataEp0 - Receive data from USB endpoint 0
//
Uint16 RxDataEp0(Uint16 *data)
{
    Uint16 length8, c;

    length8 = USBREG8(USB_O_COUNT0);

    for(c = 0; c < length8; c++)
    {
        __byte((int *)data, c) = USBREG8(USB_O_FIFO0);
    }

    return length8;
}

//
// AckEp0 - Send an ACK from USB endpoint 0 with an optional DATAEND
//
void AckEp0(eDataEnd dataEnd)
{
    USBREG8(USB_O_CSRL0) = (dataEnd == USB_END_DATA) ?
                           (USB_CSRL0_RXRDYC |
                            USB_CSRL0_DATAEND) : USB_CSRL0_RXRDYC;
}

//
// StallEp0 - Stall USB endpoint 0
//
static void StallEp0()
{
    USBREG8(USB_O_CSRL0) = (USB_CSRL0_STALL | USB_CSRL0_RXRDYC);
}

//
// RxDataEp1 - Receive data from USB endpoint 1
//
Uint16 RxDataEp1(Uint16 *data)
{
    Uint16 length8, c;

    length8 = USBREG8(USB_O_RXCOUNT1);

    for(c = 0; c < length8; c++)
    {
        __byte((int *)data, c) = USBREG8(USB_O_FIFO1);
    }

    return length8;
}

//
// AckEp1 - Send an ACK from USB endpoint 1
//
void AckEp1()
{
    USBREG8(USB_O_RXCSRL1) = 0x00;
}

//
// End of file
//
