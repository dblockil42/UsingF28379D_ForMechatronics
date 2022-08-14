***********************************************************************
* File: ClaTasks.asm
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
***********************************************************************

***********************************************************************
* Include variables and constants that will be shared in the C28x C-code
* and CLA assembly code.  This is accomplished by using .cdecls to include
* a C-code header file that contains these variables and constants.
***********************************************************************

      .cdecls   "Lab.h"


***********************************************************************
* Include an MDEBUGSTOP (CLA breakpoint) as the first instruction
* of each task - set CLA_DEBUG to 1.  Use any other value to leave out
* the MDEBUGSTOP instruction.
***********************************************************************

CLA_DEBUG	.set	0	;set to 1 to enable CLA breakpoints


***********************************************************************
* Function: Cla1Prog section
*
* Description: Contains CLA FIR filter for lab exercise
***********************************************************************

       .sect	"Cla1Prog"
       .align	2	;section even aligned - CLA instructions 32-bit 


***********************************************************************
*      C L A   T A S K   1
***********************************************************************
_Cla1Task1:

       .if CLA_DEBUG == 1
        MDEBUGSTOP
       .endif

***********************************************************************
* CLA Task 1
*
* This task is triggered at the end of conversion with an ADC interrupt.
* The ADC RESULT0 register is read as soon as it is available. It will then
* run a FIR filter and places the result into ClaFilter. The CPU will take
* an interrupt at the end of the task. It will log the ADC RESULT0 register
* for comparison as well as the CLA generated ClaFilter value.
***********************************************************************

; X and A are arrays of 32-bit float (i.e. 2 words)
; Use these defines to make the code easier to read

_X4  .set _xDelay+8  
_X3  .set _xDelay+6  
_X2  .set _xDelay+4  
_X1  .set _xDelay+2  
_X0  .set _xDelay+0  

_A4  .set _coeffs+8  
_A3  .set _coeffs+6  
_A2  .set _coeffs+4  
_A1  .set _coeffs+2  
_A0  .set _coeffs+0 

; CLA 5-tap FIR Filter
;
; Coefficients A[0, 1, 2, 3, 4]
; Data         X[0, 1, 2, 3, 4] (Delay Line - X[0] is newest value)
;
; Equations
;
; Y = A4 * X4      First Calculation of sum of products.
; X4 = X3          X4 can now be updated, because it has been used.
; Y = Y + A3 * X3  Second product, 1st add.
; X3 = X2          X3 update
; Y = Y + A2 * X2  Third product, 2nd add.
; X2 = X1          X2 update
; Y = Y + A1 * X1  Fourth product, 3rd add.
; X1 = X0          X1 update
; Y = Y = A0 * X0
;
    MMOV32     MR0,@_X4                           ;1 Load MR0 with X4
    MMOV32     MR1,@_A4                           ;2 Load MR1 with A4
    MNOP                                          ;3 Wait till I8 to read result
    MNOP                                          ;4 Wait till I8 to read result
    MNOP                                          ;5 Wait till I8 to read result
    MNOP                                          ;6 Wait till I8 to read result
    MNOP                                          ;7 Wait till I8 to read result
    MUI16TOF32 MR2,  @_AdcaResultRegs.ADCRESULT0  ;8 Read ADCRESULT0 and convert to float

    MMPYF32    MR2, MR1, MR0                      ; MR2 (Y) = MR1 (A4) * MR0 (X4)
 || MMOV32     @_X0, MR2

    MMOVD32    MR0,@_X3                           ; Load MR0 with X3, Load X4 with X3
    MMOV32     MR1,@_A3                           ; Load MR1 with A3

    MMPYF32    MR3, MR1, MR0                      ; MR3 (Y) = MR1 (A3) * MR0 (X3)
 || MMOV32     MR1,@_A2                           ; Load MR1 with A2
    MMOVD32    MR0,@_X2                           ; Load MR0 with X2, Load X3 with X2

    MMACF32    MR3, MR2, MR2, MR1, MR0            ; MR3 = A3*X3 + A4*X4
 || MMOV32     MR1,@_A1                           ; MR2 = MR1 (A2) * MR0 (X2)
    MMOVD32    MR0,@_X1                           ; Load MR0 with X1, Load X2 with X1

    MMACF32    MR3, MR2, MR2, MR1, MR0            ; MR3 = A2*X2 + (A3*X3 + A4*X4)
 || MMOV32     MR1,@_A0                           ; MR2 = MR1 (A1) * MR0 (X1)
    MMOVD32    MR0,@_X0                           ; Load MR0 with X0, Load X1 with X0 

    MMACF32    MR3, MR2, MR2, MR1, MR0            ; MR3 = A1*X1 + (A2*X2 +A3*X3 + A4*X4)
 || MMOV32     MR1,@_A0                           ; MR2 = MR1 (A0) * MR0 (X0)

    MADDF32    MR3, MR3, MR2                      ; MR3 = A0*X0 + (A1*X1 + A2*X2 +A3*X3 + A4*X4)

    MF32TOUI16 MR2, MR3                           ; Convert back to Uint16 value
    MMOV16     @_ClaFilteredOutput, MR2           ; Output
    MSTOP                                         ; End task
    MNOP                                          ; for pipeline protection
    MNOP                                          ; for pipeline protection
    MNOP                                          ; for pipeline protection


    .if 0      ; 1 = Use TASK2-8 from this file; 0 = Don't use
***********************************************************************
*      C L A   T A S K   2
***********************************************************************          
_Cla1Task2:
    MSTOP                                    ; End task
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection


***********************************************************************
*      C L A   T A S K   3
***********************************************************************
_Cla1Task3:
    MSTOP                                    ; End task
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection


***********************************************************************
*      C L A   T A S K   4
***********************************************************************
_Cla1Task4:
    MSTOP                                    ; End task
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection


***********************************************************************
*      C L A   T A S K   5
***********************************************************************
_Cla1Task5:
    MSTOP                                    ; End task
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection


***********************************************************************
*      C L A   T A S K   6
***********************************************************************
_Cla1Task6:
    MSTOP                                    ; End task
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection


***********************************************************************
*      C L A   T A S K   7
***********************************************************************
_Cla1Task7:  
    MSTOP                                    ; End task
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection


***********************************************************************
*      C L A   T A S K   8
***********************************************************************
_Cla1Task8:

       .if CLA_DEBUG == 1
        MDEBUGSTOP
       .endif

***********************************************************************
* CLA Task 8
*
* This task initializes the filter input delay line (X0 to X4) to zero.
***********************************************************************

    MMOVIZ       MR0, #0.0                   ; Load MR0 with 0
    MUI16TOF32   MR0, MR0                    ; Convert Unit16 to Float32
    MMOV32       @_X0, MR0                   ; Load X0 with MR0
    MMOV32       @_X1, MR0                   ; Load X1 with MR0
    MMOV32       @_X2, MR0                   ; Load X2 with MR0
    MMOV32       @_X3, MR0                   ; Load X3 with MR0
    MMOV32       @_X4, MR0                   ; Load X4 with MR0
    MSTOP                                    ; End task
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection
    MNOP                                     ; for pipeline protection


***********************************************************************
    .endif

    .end 
