//###########################################################################
//
// FILE:   USB_Structs.h
//
// Description: USB descriptor structure definitions for Soprano
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

#ifndef __USB_STRUCTS_H__
#define __USB_STRUCTS_H__

//
// Included Files
//
#include "c1_bootrom.h"

//
// Defines
//
#define USB_DTYPE_DEVICE           0x01
#define USB_DTYPE_CONFIG           0x02
#define USB_DTYPE_STRING           0x03
#define USB_DTYPE_INTERFACE        0x04
#define USB_DTYPE_ENDPOINT         0x05
#define USB_DTYPE_DEVICE_QUAL      0x06
#define USB_DTYPE_OSPEED_CONF      0x07
#define USB_DTYPE_INTERFACE_PWR    0x08

//
// Globals
//
typedef struct
{
    Uint16 bDescriptorType_bLength;           //[7:0]  Descriptor length in bytes (18 for device)
                                              //[15:8] Descriptor type (0x01 for device)
    Uint16 bcdUSB;                            //USB specification number -- 0x0110 for low/full speed
    Uint16 bDeviceSubClass_bDeviceClass;      //[7:0]  Subclass code
                                              //[15:8] Class code -- 0x00 for interface-specific, 0xFF for vendor-specific
    Uint16 bMaxPacketSize_bDeviceProtocol;    //[7:0]  Protocol code
                                              //[15:8] Maximum packet size for EP0 (64 for Soprano)
    Uint16 idVendor;                          //Vendor ID code -- 0x1CBE for TI
    Uint16 idProduct;                         //Product ID code -- assigned by manufacturer
    Uint16 bcdDevice;                         //Device release number
    Uint16 iProduct_iManufacturer;            //[7:0]  Index of manufacturer string descriptor
                                              //[15:8] Index of product string descriptor
    Uint16 bNumConfigurations_iSerialNumber;  //[7:0]  Index of serial number string descriptor
                                              //[15:8] Number of possible configurations
} sUsbDeviceDescriptor;

typedef struct
{
    Uint16 bDescriptorType_bLength;             //[7:0]  Descriptor length in bytes (9+1 for configuration)
                                                //[15:8] Descriptor type (0x02 for configuration)
    Uint16 wTotalLength;                        //Total length of config, interface, and endpoint descriptors
    Uint16 bConfigurationValue_bNumInterfaces;  //[7:0]  Number of interfaces in this descriptor
                                                //[15:8] ID number of this configuration
    Uint16 bmAttributes_iConfiguration;         //[7:0]  Index of configuration string descriptor
                                                //[15]   Reserved, set to 1
                                                //[14]   Self-powered
                                                //[13]   Remote wake-up
                                                //[12:8] Reserved, set to 0
    Uint16 padding_bMaxPower;                   //[7:0]  Max power consumption in 2mA units
                                                //[15:8] Unused -- only present for alignment
} sUsbConfigDescriptor;

typedef struct
{
    Uint16 bDescriptorType_bLength;                 //[7:0]  Descriptor length in bytes (9+1 for interface)
                                                    //[15:8] Descriptor type (0x04 for interface)
    Uint16 bAlternateSetting_bInterfaceNumber;      //[7:0]  Alternate interface number for SetInterface
                                                    //[15:8] ID number of this interface
    Uint16 bInterfaceClass_bNumEndpoints;           //[7:0]  Number of endpoints used by this interface
                                                    //[15:8] Interface-specific class code
    Uint16 bInterfaceProtocol_bInterfaceSubclass;   //[7:0]  Interface-specific subclass code
                                                    //[15:8] Interface-specific protocol code
    Uint16 padding_iInterface;                      //[7:0]  Index of interface string descriptor
                                                    //[15:8] Unused -- only present for alignment
} sUsbInterfaceDescriptor;

typedef struct
{
    Uint16 bDescriptorType_bLength;             //[7:0]  Descriptor length in bytes (7+1 for endpoint)
                                                //[15:8] Descriptor type (0x05 for interface)
    struct
    {
        //bEndpointAddress
        Uint16 endpointNumber   : 4;            //Endpoint number
        Uint16 rsvd1            : 3;            //Reserved -- set to 0
        Uint16 direction        : 1;            //0: OUT, 1: IN (ignored for control endpoints)

        //bmAttributes
        Uint16 transferType     : 2;            //0x0: Control, 0x1: Isochronous, 0x2: Bulk, 0x3: Interrupt
        Uint16 syncMode         : 2;            //Synchronization mode (isochronous only)
        Uint16 usageType        : 2;            //Usage type (isochronous only)
        Uint16 rsvd2            : 2;            //Reserved
    } bmAttributes_bEndpointAddress;

    Uint16 wMaxPacketSize;                      //Maximum packet size this endpoint can send or receive
    Uint16 padding_bInterval;                   //[7:0]  Interval for polling data transfers (ignored for bulk and control)
                                                //[15:0] Unused -- only present for alignment
} sUsbEndpointDescriptor;

typedef struct
{
    Uint16 bDescriptorType_bLength;             //[7:0]  Descriptor length in bytes (2+L for string)
                                                //[15:8] Descriptor type (0x03 for string)
    const char *bString;                        //Pointer to the actual string
} sUsbStringDescriptor;

//
//Standard control request identifiers
//
#define USB_REQ_GET_STATUS        0x00
#define USB_REQ_CLEAR_FEATURE     0x01
#define USB_REQ_SET_FEATURE       0x03
#define USB_REQ_SET_ADDRESS       0x05
#define USB_REQ_GET_DESCRIPTOR    0x06
#define USB_REQ_SET_DESCRIPTOR    0x07
#define USB_REQ_GET_CONFIG        0x08
#define USB_REQ_SET_CONFIG        0x09
#define USB_REQ_GET_INTERFACE     0x0A
#define USB_REQ_SET_INTERFACE     0x0B

//
//Control request types
//
#define USB_REQTYPE_STD           0x0
#define USB_REQTYPE_CLASS         0x1
#define USB_REQTYPE_VENDOR        0x2

//
//Control request recipients
//
#define USB_REQREC_DEVICE         0x0
#define USB_REQREC_INTERFACE      0x1
#define USB_REQREC_ENDPOINT       0x2
#define USB_REQREC_OTHER          0x3

//
//Control request data packet format (received from host during enumeration)
//
typedef struct
{
    struct
    {
        Uint16 recipient    : 5;      //[4:0]  Recipient (0: device, 1: interface, 2: endpoint, 3: other)
        Uint16 reqType      : 2;      //[6:5]  Request type (00: standard, 01: class-specific, 10: vendor-specific)
        Uint16 dataDir      : 1;      //[7]    Direction of data flow (0: OUT, 1: IN)
        Uint16 bRequest     : 8;      //[15:8] Request identifier
    } bmRequestType_bRequest;
    Uint16 wValue;                    //Extra request-specific information
    Uint16 wIndex;                    //Extra request-specific information
    Uint16 wLength;                   //Max number of bytes expected in the follow-up data packet (0: no data packet)
} sUsbRequestPacket;

//
//Implementations for use in the control request handler
//
extern const sUsbDeviceDescriptor loaderDeviceDescriptor;
extern const sUsbStringDescriptor langDescriptor;
extern const sUsbStringDescriptor mfgDescriptor;
extern const sUsbStringDescriptor prodDescriptor;
extern const sUsbStringDescriptor serialDescriptor;
extern const sUsbStringDescriptor loaderStringDescriptor;
extern const sUsbStringDescriptor * const stringDescriptors[];

//
//Function prototypes
//
void CombineUsbDescriptors(Uint16 *buffer);

#endif //__USB_STRUCTS_H__

//
// End of file
//
