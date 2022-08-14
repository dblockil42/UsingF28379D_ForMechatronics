//###########################################################################
//
//
// FILE:    USB_Structs.c
//
//
// Description: USB Descriptor implementations for the Soprano boot loader
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
#include "USB_Structs.h"

//
// Globals
//

//
//Device descriptor for the boot loader
//
const sUsbDeviceDescriptor loaderDeviceDescriptor =
{
    USB_DTYPE_DEVICE<<8 | 2*sizeof(sUsbDeviceDescriptor),
    0x0110,
    0x00<<8 | 0xFF,
    64<<8 | 0x00,
    0x1CBE,
    0x00FF,
    0x0001,
    2<<8 | 1,
    1<<8 | 3
};

//
//Configuration, interface, and functional descriptors for the loader.
//The wTotalLength must be set in software based on the total set of
//descriptors sent to the host.
//
const sUsbConfigDescriptor loaderConfigDescriptor =
{
    USB_DTYPE_CONFIG<<8 | (2*sizeof(sUsbConfigDescriptor) - 1),
    0,
    1<<8 | 1,
    0xC0<<8 | 0,
    150
};
const sUsbInterfaceDescriptor loaderInterfaceDescriptor =
{
    USB_DTYPE_INTERFACE<<8 | (2*sizeof(sUsbInterfaceDescriptor) - 1),
    0<<8 | 0,
    0xFF<<8 | 1,
    0x00<<8 | 0x00,
    4
};
const sUsbEndpointDescriptor loaderEndpointDescriptor =
{
    USB_DTYPE_ENDPOINT<<8 | (2*sizeof(sUsbEndpointDescriptor) - 1),
    {
        1, 0, 0,     //Endpoint 1, OUT type
        2, 0, 0, 0   //Bulk mode transfers
    },
    64,              //Max packet size in bytes (64 is max for bulk transfers)
    0
};

//
//Combine the configuration, interface, and functional descriptors into a
//single block, stripping out the 16-bit alignment padding in the process.
//
void CombineUsbDescriptors(Uint16 *buffer)
{
    Uint16 b, d, byteLength;

    b = d = 0;
    byteLength = loaderConfigDescriptor.bDescriptorType_bLength & 0xFF;
    while(byteLength--)
    {
        __byte((int *)buffer, b) = __byte((int *)&loaderConfigDescriptor, d);
        b++; d++;
    }

    d = 0;
    byteLength = loaderInterfaceDescriptor.bDescriptorType_bLength & 0xFF;
    while(byteLength--)
    {
        __byte((int *)buffer, b) = __byte((int *)&loaderInterfaceDescriptor, d);
        b++; d++;
    }

    d = 0;
    byteLength = loaderEndpointDescriptor.bDescriptorType_bLength & 0xFF;
    while(byteLength--)
    {
        __byte((int *)buffer, b) = __byte((int *)&loaderEndpointDescriptor, d);
        b++; d++;
    }

    //
    //Set wTotalLength
    //
    buffer[1] = (loaderConfigDescriptor.bDescriptorType_bLength & 0xFF) +
                (loaderInterfaceDescriptor.bDescriptorType_bLength & 0xFF) +
                (loaderEndpointDescriptor.bDescriptorType_bLength & 0xFF);
}

//
//String descriptor strings. These are variable-length and thus cannot be
//part of a structure.
//
const char langID = 0x0409;    //US English
const char manufacturerString[] = "Texas Instruments";
const char productString[] = "TMS320F2837xD USB Boot Loader";
const char serialString[] = "2";
const char loaderString[] = "C2000-BOOT";

//
//String descriptor structures. Each consists of a header followed by a
//pointer to the actual string. The full descriptor must be constructed
//at run-time by copying the header and the string into a temporary buffer.
//
const sUsbStringDescriptor langDescriptor =
{
    USB_DTYPE_STRING<<8 | (2 + 2*sizeof(langID)),
    &langID
};
const sUsbStringDescriptor mfgDescriptor =
{
    USB_DTYPE_STRING<<8 | (2 + 2*sizeof(manufacturerString)),
    manufacturerString
};
const sUsbStringDescriptor prodDescriptor =
{
    USB_DTYPE_STRING<<8 | (2 + 2*sizeof(productString)),
    productString
};
const sUsbStringDescriptor serialDescriptor =
{
    USB_DTYPE_STRING<<8 | (2 + 2*sizeof(serialString)),
    serialString
};
const sUsbStringDescriptor loaderStringDescriptor =
{
    USB_DTYPE_STRING<<8 | (2 + 2*sizeof(loaderString)),
    loaderString
};

//
//String descriptor array. Used to provide indexing for descriptor requests.
//This needs to be a constant array of constant pointers to keep it out of
//the .cinit section.
//
const sUsbStringDescriptor * const stringDescriptors[] =
{
    &langDescriptor,
    &mfgDescriptor,
    &prodDescriptor,
    &serialDescriptor,
    &loaderStringDescriptor
};

//
// End of file
//
