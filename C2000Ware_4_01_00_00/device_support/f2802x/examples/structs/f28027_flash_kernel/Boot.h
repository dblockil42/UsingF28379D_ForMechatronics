//###########################################################################
//
// FILE:   Boot.h
//
// TITLE:  Boot ROM Definitions.
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

#ifndef TMS320X2802X_BOOT_H
#define TMS320X2802X_BOOT_H

#include "F2802x_Device.h"
#include "f2802x_examples.h"

//
// Define for the length of the programming buffer
//
#define PROG_BUFFER_LENGTH 0x400

//
// Boot Modes
//

//
//           GPIO37    GPIO34   TRSTn
//            TDO      CMP2OUT
// Mode EMU    x         x        1      Emulator connected
// Mode 0      0         0        0      Parallel I/O
// Mode 1      0         1        0      SCI
// Mode 2      1         0        0      wait
// Mode 3      1         1        0      "Get Mode"
//
#define OTP_KEY             0x3D7BFE
#define OTP_BMODE           0x3D7BFF
#define KEY_VAL             0x55AA

#define PARALLEL_BOOT       0x0000
#define SCI_BOOT            0x0001
#define WAIT_BOOT           0x0002
#define GET_BOOT            0x0003

#define SPI_BOOT            0x0004
#define I2C_BOOT            0x0005
#define OTP_BOOT            0x0006
#define RAM_BOOT            0x000A
#define FLASH_BOOT          0x000B

//
// Fixed boot entry points
//
#define FLASH_ENTRY_POINT 0x3F7FF6
#define OTP_ENTRY_POINT   0x3D7800
#define RAM_ENTRY_POINT   0x000000

#define DIVSEL_BY_4             0
#define DIVSEL_BY_2             2
#define DIVSEL_BY_1             3

#define ERROR                   1
#define NO_ERROR                0
#define EIGHT_BIT               8
#define SIXTEEN_BIT            16
#define EIGHT_BIT_HEADER   0x08AA
#define SIXTEEN_BIT_HEADER 0x10AA

//
// Typedefs
//
typedef Uint16 (* uint16fptr)();

extern  uint16fptr GetWordData;
extern  uint16fptr GetOnlyWordData;
extern  void (*Flash_CallbackPtr) (void);
extern  Uint32 Flash_CPUScaleFactor;
#define Device_cal (void   (*)(void))0x3D7C80
#define Get_mode   (Uint16 (*)(void))0x3D7CC0

//
// Defines configured by the boot ROM
//
#define BORTRIM (Uint16 *)0x0986

#endif  // end of TMS320x2802x_BOOT_H definition

//
// End of File
//

