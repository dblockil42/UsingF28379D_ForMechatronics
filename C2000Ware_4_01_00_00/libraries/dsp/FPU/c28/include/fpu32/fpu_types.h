#ifndef _FPU_TYPES_H_
#define _FPU_TYPES_H_

//#############################################################################
//! \file   include/fpu_types.h
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
#include <stdint.h>
#include "float.h"

//!
//! \defgroup DSP_TYPES_F32 Single Precision DSP Library Data Types

//!
//! \addtogroup DSP_TYPES_F32
// @{
    
#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
// defines
//*****************************************************************************
//! Set to 1U if you would like to use legacy names from v1.50.00.00 of the 
//! DSP library; set to 0U (default) to use the new naming convention.
//! It is important to note that not all module elements were updated from the
//! old naming scheme.
//! Rebuild the library and examples after changing this value
//!
#define USE_LEGACY_NAMES    (0U)

//*****************************************************************************
// typedefs
//*****************************************************************************

#ifndef FPU_TYPES
#define FPU_TYPES

//! \brief Complex Float data type for the single precision DSP library
//!
typedef struct {
    //! dat[0] is the real portion while dat[1] is the imaginary part.
    float dat[2]; 
} complex_float;
#endif //FPU_TYPES

// @} //addtogroup

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of _FPU_TYPES_H_

// End of File
