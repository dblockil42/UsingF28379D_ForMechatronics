//#############################################################################
//
// FILE: fsk_corr_detector.h
//
// TITLE: Header file for FSK detect functions, part of fsk_corr_detect_lib
//
//#############################################################################
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
//#############################################################################

#ifndef FSK_CORR_DETECT_INCLUDE_H
#define FSK_CORR_DETECT_INCLUDE_H

#ifndef     MATH_TYPE
#define     MATH_TYPE      1   //FLOAT_MATH
#endif
#include <stdint.h>
#include "IQmathLib.h"

#define RX_MESSAGE_SIZE 33  // Number of bits to receive

typedef struct{
	float mark_freq;
	float space_freq;
	float isr_freq;
	float bit_freq;
	float detection_threshold;
	int16_t bit_detected; // +1 mark, -1 space, 0 zero energy or unspecified frequency
}FSK_CORR_DETECTOR;

//
// global counter variables
//
extern uint16_t mark_counter, space_counter, zero_counter, bit_sample_counter;
//extern uint16_t mark_detect[RX_MESSAGE_SIZE], space_detect[RX_MESSAGE_SIZE], n; // Used for debug

extern void FSK_CORR_DETECTOR_INIT( volatile FSK_CORR_DETECTOR *obj);

extern void FSK_CORR_DETECTOR_BITRATE_RUN(volatile FSK_CORR_DETECTOR *obj);

extern void FSK_CORR_DETECTOR_RUN(volatile float adc_value);

extern void FSK_CORR_DETECTOR_OverSampl_RUN(volatile FSK_CORR_DETECTOR *obj);

#endif
