//#############################################################################
//! FILE: cla_cfft.h
//!
//! TITLE: Prototypes and definitions for Complex FFT on CLA
//
//
///#############################################################################
//
//
// $Copyright: Copyright (C) 2022 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

#ifndef _CLA_CFFT_H_
#define _CLA_CFFT_H_

//
// Includes
//

//!
//! \defgroup CLA_DSP_FFT Fast Fourier Transform (CLA)

//!
//! \ingroup CLA_DSP_FFT
// @{

#ifdef __cplusplus
extern "C" {
#endif

//
// Defines
//

//
// Typedefs
//


//
// Globals
//

//! \brief CLA twiddle factors
//!
extern const float *cla_twiddleFactors;

//! \brief CLA bit reversal tables
//!
extern const float *cla_bitReversalTable;

//
// Function prototypes
//

//! \brief Runs the Complex FFT routine (1024 points)
//! 
//! \attention 
//! -# This is an in-place algorithm
//! -# The input/output buffer must be aligned to a 12-bit address, usually the
//!   starting address of one of the CLA data RAMs
//! -# The complex data has real-first ordering i.e. the real part occupies the lower 
//!   double word
//! -# This function is not re-entrant as it uses global variable to store temporary
//!   values. It also expects the FFT buffer to be global (to both the C28 and CLA)
//!   and to be named \b "IOBuffer". If the user desires to change the name, the macro
//!   IOBUFFER must be altered in the source assembly to reflect the new name and the
//!   code rebuilt
//!
//! \return FFT of the input in the I/O buffer
//
extern void CLA_CFFT_run1024Pt();

//! \brief Runs the Complex FFT routine (512 points)
//!
//! \attention
//! -# This is an in-place algorithm
//! -# The input/output buffer must be aligned to a 12-bit address, usually the
//!   starting address of one of the CLA data RAMs
//! -# The complex data has real-first ordering i.e. the real part occupies the lower
//!   double word
//! -# This function is not re-entrant as it uses global variable to store temporary
//!   values. It also expects the FFT buffer to be global (to both the C28 and CLA)
//!   and to be named \b "IOBuffer". If the user desires to change the name, the macro
//!   IOBUFFER must be altered in the source assembly to reflect the new name and the
//!   code rebuilt
//!
//! \return FFT of the input in the I/O buffer
//
extern void CLA_CFFT_run512Pt();
extern void CLA_CFFT_run256Pt();

//!
//! \brief Unpack the 512-point complex FFT output to get the FFT of a 1024 point
//!         real sequence
//!
//! In order to get the FFT of a real N-point sequence, we treat the input as
//! an N/2-point complex sequence, take its complex FFT, use the following
//! properties to get the N-pt Fourier transform of the real sequence
//!
//! \f[
//! FFT_{n}(k,f) = FFT_{N/2}(k,f_{e})+e^{\frac{-j2{\pi}k}{N}}FFT_{N/2}(k,f_{o})
//! \f]
//!
//! where \f$f_{e}\f$ is the even elements, \f$f_{o}\f$ the odd elements,
//! k = 0 to \f$\frac{N}{2}-1\f$ and
//!
//! \f[ F_{e}(k) = \frac{Z(k) + Z(\frac{N}{2}-k)^{\ast}}{2} \f]
//! \f[ F_{o}(k) = -j\frac{Z(k) - Z(\frac{N}{2}-k)^{\ast}}{2} \f]
//!
//! We get the first N/2 points of the FFT by combining the above two equations
//! \f[ F(k) = F_{e}(k) + e^{\frac{-j2{\pi}k}{N}}F_{o}(k) \f]
//!
//! \attention
//! -# This is an off-place algorithm
//! -# Since this function follows an FFT the input buffer must have been 
//!   aligned to a 12-bit address, usually the starting address of one of the 
//!   CLA data RAMs
//! -# The complex data has real-first ordering i.e. the real part occupies
//!    the lower double word
//! -# This function expects the FFT buffers to be global (to both 
//!    the C28 and CLA) and to be named "IOBuffer" and "IOBuffer2" respectively.
//!    If the user desires to  change the name, the macros I_BUFFER and O_BUFFER
//!    must be altered in the source assembly to reflect the new name and the 
//!    code rebuilt
//! -# In the loops the code does two extra reads beyond the end of the
//!    twiddle factor table. Ensure that atleast 4 words after the twiddle
//!    factor table are within CLA accessible data RAM. If using the tables in
//!    Data ROM this is not an issue as the bit reversal tables follow the
//!    twiddle table and you end up reading the first two entries of that
//!    table instead
//! \sa http://www.engineeringproductivitytools.com/stuff/T0001/PT10.HTM for
//! the entire derivation
//
extern void CLA_CFFT_unpack512Pt();
extern void CLA_CFFT_unpack256Pt();

#ifdef __cplusplus
}
#endif // extern "C"

// @} //ingroup

#endif //end of _CLA_CFFT_H_ definition
