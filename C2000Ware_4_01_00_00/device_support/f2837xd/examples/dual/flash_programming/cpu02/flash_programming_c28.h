//###########################################################################
//
// FILE:   flash_programming_c28.h
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

#define Bzero_SectorN_start         0xBE000                       //Start Address : Sector 0 of Bank Under Test
#define Bzero_SectorN_End           0xBFFFF                       //Start Address : Sector 0 of Bank Under Test

#define Bzero_SectorM_start         0xBC000                       //Start Address : Sector 1 of Bank Under Test
#define Bzero_SectorM_End           0xBDFFF                       //Start Address : Sector 1 of Bank Under Test

#define Bzero_SectorL_start         0xBA000                       //Start Address : Sector 2 of Bank Under Test
#define Bzero_SectorL_End           0xBBFFF                       //Start Address : Sector 2 of Bank Under Test

#define Bzero_SectorK_start         0xB8000                       //Start Address : Sector 3 of Bank Under Test
#define Bzero_SectorK_End           0xB9FFF                       //Start Address : Sector 3 of Bank Under Test

#define Bzero_SectorJ_start         0xB0000                       //Start Address : Sector 4 of Bank Under Test
#define Bzero_SectorJ_End           0xB7FFF                       //Start Address : Sector 4 of Bank Under Test

#define Bzero_SectorI_start         0xA8000                       //Start Address : Sector 5 of Bank Under Test
#define Bzero_SectorI_End           0xAFFFF                       //Start Address : Sector 5 of Bank Under Test

#define Bzero_SectorH_start         0xA0000                       //Start Address : Sector 6 of Bank Under Test
#define Bzero_SectorH_End           0xA7FFF                       //Start Address : Sector 6 of Bank Under Test

#define Bzero_SectorG_start         0x98000
#define Bzero_SectorG_End           0x9FFFF

#define Bzero_SectorF_start         0x90000
#define Bzero_SectorF_End           0x97FFF

#define Bzero_SectorE_start         0x88000
#define Bzero_SectorE_End           0x8FFFF

#define Bzero_SectorD_start         0x86000
#define Bzero_SectorD_End           0x87FFF

#define Bzero_SectorC_start         0x84000
#define Bzero_SectorC_End           0x85FFF

#define Bzero_SectorB_start         0x82000
#define Bzero_SectorB_End           0x83FFF

#define Bzero_SectorA_start         0x80000
#define Bzero_SectorA_End           0x81FFF


#define Bzero_16KSector_u32length   0x1000
#define Bzero_64KSector_u32length   0x4000

//
// End of file
//
