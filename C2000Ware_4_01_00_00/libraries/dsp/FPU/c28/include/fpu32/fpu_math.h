#ifndef _FPU_MATH_H_
#define _FPU_MATH_H_
//#############################################################################
//! \file   include/fpu_math.h
//!
//! \brief  Prototypes and Definitions for the C28x FPU Library
//! \author Vishal Coelho
//! \date   n/a
//
//  Group: 			C2000
//  Target Family:	F2837x
//
//#############################################################################
//
//
// $Copyright: Copyright (C) 2022 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

//*****************************************************************************
// includes
//*****************************************************************************
#include "fpu_types.h"

//!
//! \defgroup DSP_MATH_F32 Math Operations

//!
//! \addtogroup DSP_MATH_F32
// @{
    
#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
// function prototypes
//*****************************************************************************
//! \brief Fast Square Root.
//!
//! This function is an inline optimized fast square root function using
//! two iterations of the Newton Raphson method to achieve an accurate result.
//! \param x input variable
//! \attention
//! -# Performance is best with -o2, -mn compiler options (cgtools v6.0.1)
//! -# A single invocation of the __ffsqrtf function takes 22 cycles to 
//! complete.
//! Inspection of the generated assembly code would reveal 11 NOP's used as 
//! delay slots between instructions. If the user were to chain back-to-back 
//! invocations of the __ffsqrtf function, and then subsequently use the 
//! results in either arithmetic or assignment statements, the compiler will 
//! interleave the instructions of both functions, effectively resulting in 11 
//! cycles per function call. The compiler will not interleave the instructions
//! of back-to-back functions if their results are subsequently used in logical
//!  statements.
//! -# This function uses a Newton-Raphson approximation of the square root 
//! which does not work correctly for very small numbers (~1.175494351E-38). In 
//! such cases the user can call the sqrt() from the run time support library. 
//! It should be noted that bounds checking is not included in this routine.
//! 
//! <table>
//! <caption id="multi_row">Performance Data</caption>
//! <tr><th> Function   <th> Cycles 
//! <tr><td> __ffsqrtf  <td> 22
//! </table>
//
inline static float __ffsqrtf(const float x)
{
    float dst,tmp2, tmp3;

    dst = __eisqrtf32(x);
    tmp2 = x * 0.5;
    tmp3 = dst * dst;
    tmp3 = tmp3 * tmp2;
    tmp3 = 1.5 - tmp3;
    dst = dst * tmp3;
    tmp3 = dst * tmp2;
    tmp3 = dst * tmp3;
    tmp3 = 1.5 - tmp3;
    dst = dst * tmp3;
    dst = x * dst;
    return dst;
}
// @} //addtogroup

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of _FPU_MATH_H_

// End of File
