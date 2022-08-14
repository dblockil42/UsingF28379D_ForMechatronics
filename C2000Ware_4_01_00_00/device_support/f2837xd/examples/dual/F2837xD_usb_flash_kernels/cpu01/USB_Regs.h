//###########################################################################
//
// FILE:   USB_Regs.h
//
// Description: USB register definitions and helper macros for Soprano
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

#ifndef __USB_REGS_H__
#define __USB_REGS_H__

//
// Defines
//
#define USB0_BASE_ADDR    0x00040000    //Soprano
//#define USB0_BASE_ADDR    0x4000    //Octave
#define USBREG32(r)    (*(volatile unsigned long *)(USB0_BASE_ADDR + (r)))
#define USBREG16(r)    (*(volatile unsigned short *)(USB0_BASE_ADDR + (r)))
#define USBREG8(r)    __byte((int *)(USB0_BASE_ADDR + (r)), 0)

#define USB_O_FADDR             0x00000000  // USB Device Functional Address
#define USB_O_POWER             0x00000001  // USB Power
#define USB_O_TXIS              0x00000002  // USB Transmit Interrupt Status
#define USB_O_RXIS              0x00000004  // USB Receive Interrupt Status
#define USB_O_TXIE              0x00000006  // USB Transmit Interrupt Enable
#define USB_O_RXIE              0x00000008  // USB Receive Interrupt Enable
#define USB_O_IS                0x0000000A  // USB General Interrupt Status
#define USB_O_IE                0x0000000B  // USB Interrupt Enable
#define USB_O_FRAME             0x0000000C  // USB Frame Value
#define USB_O_EPIDX             0x0000000E  // USB Endpoint Index
#define USB_O_TEST              0x0000000F  // USB Test Mode

#define USB_O_FIFO0             0x00000020  // USB FIFO Endpoint 0
#define USB_O_FIFO1             0x00000024  // USB FIFO Endpoint 1
#define USB_O_FIFO2             0x00000028  // USB FIFO Endpoint 2
#define USB_O_FIFO3             0x0000002C  // USB FIFO Endpoint 3

#define USB_O_DEVCTL            0x00000060  // USB Device Control
#define USB_O_TXFIFOSZ          0x00000062  // USB Transmit Dynamic FIFO Sizing
#define USB_O_RXFIFOSZ          0x00000063  // USB Receive Dynamic FIFO Sizing
#define USB_O_TXFIFOADD         0x00000064  // USB Transmit FIFO Start Address
#define USB_O_RXFIFOADD         0x00000066  // USB Receive FIFO Start Address
#define USB_O_CONTIM            0x0000007A  // USB Connect Timing
#define USB_O_FSEOF             0x0000007D  // USB Full-Speed Last Transaction to End of Frame Timing
#define USB_O_LSEOF             0x0000007E  // USB Low-Speed Last Transaction to End of Frame Timing

#define USB_O_TXFUNCADDR0       0x00000080  // USB Transmit Functional Address Endpoint 0
#define USB_O_TXHUBADDR0        0x00000082  // USB Transmit Hub Address Endpoint 0
#define USB_O_TXHUBPORT0        0x00000083  // USB Transmit Hub Port Endpoint 0
#define USB_O_TXFUNCADDR1       0x00000088  // USB Transmit Functional Address Endpoint 1
#define USB_O_TXHUBADDR1        0x0000008A  // USB Transmit Hub Address Endpoint 1
#define USB_O_TXHUBPORT1        0x0000008B  // USB Transmit Hub Port Endpoint 1
#define USB_O_RXFUNCADDR1       0x0000008C  // USB Receive Functional Address Endpoint 1
#define USB_O_RXHUBADDR1        0x0000008E  // USB Receive Hub Address Endpoint 1
#define USB_O_RXHUBPORT1        0x0000008F  // USB Receive Hub Port Endpoint 1
#define USB_O_TXFUNCADDR2       0x00000090  // USB Transmit Functional Address Endpoint 2
#define USB_O_TXHUBADDR2        0x00000092  // USB Transmit Hub Address Endpoint 2
#define USB_O_TXHUBPORT2        0x00000093  // USB Transmit Hub Port Endpoint 2
#define USB_O_RXFUNCADDR2       0x00000094  // USB Receive Functional Address Endpoint 2
#define USB_O_RXHUBADDR2        0x00000096  // USB Receive Hub Address Endpoint 2
#define USB_O_RXHUBPORT2        0x00000097  // USB Receive Hub Port Endpoint 2
#define USB_O_TXFUNCADDR3       0x00000098  // USB Transmit Functional Address Endpoint 3
#define USB_O_TXHUBADDR3        0x0000009A  // USB Transmit Hub Address Endpoint 3
#define USB_O_TXHUBPORT3        0x0000009B  // USB Transmit Hub Port Endpoint 3
#define USB_O_RXFUNCADDR3       0x0000009C  // USB Receive Functional Address Endpoint 3
#define USB_O_RXHUBADDR3        0x0000009E  // USB Receive Hub Address Endpoint 3
#define USB_O_RXHUBPORT3        0x0000009F  // USB Receive Hub Port Endpoint 3

#define USB_O_CSRL0             0x00000102  // USB Control and Status Endpoint 0 Low
#define USB_O_CSRH0             0x00000103  // USB Control and Status Endpoint 0 High
#define USB_O_COUNT0            0x00000108  // USB Receive Byte Count Endpoint 0
#define USB_O_TYPE0             0x0000010A  // USB Type Endpoint 0
#define USB_O_NAKLMT            0x0000010B  // USB NAK Limit
#define USB_O_TXMAXP1           0x00000110  // USB Maximum Transmit Data Endpoint 1
#define USB_O_TXCSRL1           0x00000112  // USB Transmit Control and Status Endpoint 1 Low
#define USB_O_TXCSRH1           0x00000113  // USB Transmit Control and Status Endpoint 1 High
#define USB_O_RXMAXP1           0x00000114  // USB Maximum Receive Data Endpoint 1
#define USB_O_RXCSRL1           0x00000116  // USB Receive Control and Status Endpoint 1 Low
#define USB_O_RXCSRH1           0x00000117  // USB Receive Control and Status Endpoint 1 High
#define USB_O_RXCOUNT1          0x00000118  // USB Receive Byte Count Endpoint 1
#define USB_O_TXTYPE1           0x0000011A  // USB Host Transmit Configure Type Endpoint 1
#define USB_O_TXINTERVAL1       0x0000011B  // USB Host Transmit Interval Endpoint 1
#define USB_O_RXTYPE1           0x0000011C  // USB Host Configure Receive Type Endpoint 1
#define USB_O_RXINTERVAL1       0x0000011D  // USB Host Receive Polling Interval Endpoint 1
#define USB_O_TXMAXP2           0x00000120  // USB Maximum Transmit Data Endpoint 2
#define USB_O_TXCSRL2           0x00000122  // USB Transmit Control and Status Endpoint 2 Low
#define USB_O_TXCSRH2           0x00000123  // USB Transmit Control and Status Endpoint 2 High
#define USB_O_RXMAXP2           0x00000124  // USB Maximum Receive Data Endpoint 2
#define USB_O_RXCSRL2           0x00000126  // USB Receive Control and Status Endpoint 2 Low
#define USB_O_RXCSRH2           0x00000127  // USB Receive Control and Status Endpoint 2 High
#define USB_O_RXCOUNT2          0x00000128  // USB Receive Byte Count Endpoint 2
#define USB_O_TXTYPE2           0x0000012A  // USB Host Transmit Configure Type Endpoint 2
#define USB_O_TXINTERVAL2       0x0000012B  // USB Host Transmit Interval Endpoint 2
#define USB_O_RXTYPE2           0x0000012C  // USB Host Configure Receive Type Endpoint 2
#define USB_O_RXINTERVAL2       0x0000012D  // USB Host Receive Polling Interval Endpoint 2
#define USB_O_TXMAXP3           0x00000130  // USB Maximum Transmit Data Endpoint 3
#define USB_O_TXCSRL3           0x00000132  // USB Transmit Control and Status Endpoint 3 Low
#define USB_O_TXCSRH3           0x00000133  // USB Transmit Control and Status Endpoint 3 High
#define USB_O_RXMAXP3           0x00000134  // USB Maximum Receive Data Endpoint 3
#define USB_O_RXCSRL3           0x00000136  // USB Receive Control and Status Endpoint 3 Low
#define USB_O_RXCSRH3           0x00000137  // USB Receive Control and Status Endpoint 3 High
#define USB_O_RXCOUNT3          0x00000138  // USB Receive Byte Count Endpoint 3
#define USB_O_TXTYPE3           0x0000013A  // USB Host Transmit Configure Type Endpoint 3
#define USB_O_TXINTERVAL3       0x0000013B  // USB Host Transmit Interval Endpoint 3
#define USB_O_RXTYPE3           0x0000013C  // USB Host Configure Receive Type Endpoint 3
#define USB_O_RXINTERVAL3       0x0000013D  // USB Host Receive Polling Interval Endpoint 3
#define USB_O_RQPKTCOUNT1       0x00000304  // USB Request Packet Count in Block Transfer Endpoint 1
#define USB_O_RQPKTCOUNT2       0x00000308  // USB Request Packet Count in Block Transfer Endpoint 2
#define USB_O_RQPKTCOUNT3       0x0000030C  // USB Request Packet Count in Block Transfer Endpoint 3

#define USB_O_RXDPKTBUFDIS      0x00000340  // USB Receive Double Packet Buffer Disable
#define USB_O_TXDPKTBUFDIS      0x00000342  // USB Transmit Double Packet Buffer Disable
#define USB_O_EPC               0x00000400  // USB External Power Control
#define USB_O_EPCRIS            0x00000404  // USB External Power Control Raw Interrupt Status
#define USB_O_EPCIM             0x00000408  // USB External Power Control Interrupt Mask
#define USB_O_EPCISC            0x0000040C  // USB External Power Control Interrupt Status and Clear
#define USB_O_DRRIS             0x00000410  // USB Device RESUME Raw Interrupt Status
#define USB_O_DRIM              0x00000414  // USB Device RESUME Interrupt Mask
#define USB_O_DRISC             0x00000418  // USB Device RESUME Interrupt Status and Clear
#define USB_O_GPCS              0x0000041C  // USB General-Purpose Control and Status
#define USB_O_IDVISC            0x0000444C  // USB ID Valid Detect Interrupt status and clear
#define USB_O_DMASEL            0x00000450  // USB DMA Select

//
//Bit definitions for USB_O_POWER
//
#define USB_POWER_ISOUP         0x00000080  // Isochronous Update
#define USB_POWER_SOFTCONN      0x00000040  // Soft Connect/Disconnect
#define USB_POWER_RESET         0x00000008  // RESET Signalling
#define USB_POWER_RESUME        0x00000004  // RESUME Signalling
#define USB_POWER_SUSPEND       0x00000002  // SUSPEND Mode
#define USB_POWER_PWRDNPHY      0x00000001  // Power Down PHY

//
//Bit definitions for USB_O_IE
//
#define USB_IE_VBUSERR          0x00000080  // Enable VBUS Error Interrupt
#define USB_IE_SESREQ           0x00000040  // Enable Session Request
#define USB_IE_DISCON           0x00000020  // Enable Disconnect Interrupt
#define USB_IE_CONN             0x00000010  // Enable Connect Interrupt
#define USB_IE_SOF              0x00000008  // Enable Start-of-Frame Interrupt
#define USB_IE_RESET            0x00000004  // Enable RESET Interrupt
#define USB_IE_RESUME           0x00000002  // Enable RESUME Interrupt
#define USB_IE_SUSPND           0x00000001  // Enable SUSPEND Interrupt

//
//Bit definitions for USB_O_CSRL0
//
#define USB_CSRL0_RXRDY         0x01        // Received packet is ready
#define USB_CSRL0_TXRDY         0x02        // Transmit FIFO is ready for transmission
#define USB_CSRL0_STALLED       0x04        // Stall handshake has been sent
#define USB_CSRL0_DATAEND       0x08        // Set to indicate no more TX/RX data
#define USB_CSRL0_SETEND        0x10        // Control transfer ended prematurely
#define USB_CSRL0_STALL         0x20        // Send a stall
#define USB_CSRL0_RXRDYC        0x40        // Clear the RXRDY bit
#define USB_CSRL0_SETENDC       0x80        // Clear the SETEND bit

#endif //__USB_REGS_H__

//
// End of file
//
