/**********************************************************************
* File: lab.h
* Device: TMS320F2837xD
* Author: C2000 Technical Training, Texas Instruments
* Description: Include file for workshop lab exercises.  Include this
* file in all C-source files.
**********************************************************************/

#ifndef LAB_H
#define LAB_H


//---------------------------------------------------------------------------
// Constant Definitions
//
#define ADC_BUF_LEN         50              // ADC buffer length
#define ADC_SAMPLE_PERIOD   1999            // 1999 = 50 kHz sampling w/ 100 MHz ePWM clock
#define PWM_HALF_PERIOD     25000           // period/2 for 2 kHz symmetric PWM w/ 100 MHz ePWM clock
#define PWM_DUTY_CYCLE      18750           // 25% duty cycle
#define PWM_MIN_DUTY		22500			// 10% duty cycle for PWM modulation
#define PWM_MAX_DUTY		2500			// 90% duty cycle for PWM modulation
#define PWM_STEP			10				// Step size change for PWM modulation
#define FILTER_LEN          5               // filter length
#define SINE_PTS            25              // number of point in sine wave


//---------------------------------------------------------------------------
// Include Standard C Language Header Files
// (Header file <string.h> not supported by CLA compiler)
//
#if !defined(__TMS320C28XX_CLA__)
    #include <string.h>
#endif


//---------------------------------------------------------------------------
// Include any other Header Files
//
#include "F2837xD_Cla_typedefs.h"        // CLA type definitions
#include "F2837xD_device.h"              // F2837xD header file peripheral address definitions
#include "F2837xD_Adc_defines.h"         // ADC definitions
#include "F2837xD_defaultisr.h"          // ISR definitions
#include "F2837xD_Pie_defines.h"         // PIE definitions


//---------------------------------------------------------------------------
// Function Prototypes
//
extern void AdcSetMode(Uint16, Uint16, Uint16);
extern void CalAdcINL(Uint16);
extern void DelayUs(Uint16);
extern void InitAdca(void);
extern void InitCla(void);
extern void InitDacb(void);
extern void InitDma(void);
extern void InitECap(void);
extern void InitEPwm(void);
extern void InitFlash(void);
extern void InitGpio(void);
extern void InitPieCtrl(void);
extern void InitSysCtrl(void);
extern void InitWatchdog(void);
extern void InitXbar();
extern void SetDBGIER(Uint16);
extern void UserInit(void);


//---------------------------------------------------------------------------
// CLA Function Prototypes
//
extern interrupt void Cla1Task1();
extern interrupt void Cla1Task2();
extern interrupt void Cla1Task3();
extern interrupt void Cla1Task4();
extern interrupt void Cla1Task5();
extern interrupt void Cla1Task6();
extern interrupt void Cla1Task7();
extern interrupt void Cla1Task8();


//---------------------------------------------------------------------------
// Global symbols defined in the linker command file
//
extern Uint16 cla1Funcs_loadstart;
extern Uint16 cla1Funcs_loadsize;
extern Uint16 cla1Funcs_runstart;
extern Uint16 secureRamFuncs_loadstart;
extern Uint16 secureRamFuncs_loadsize;
extern Uint16 secureRamFuncs_runstart;
extern Uint16 Cla1Prog_Start;


//---------------------------------------------------------------------------
// Global Variables References
//
extern float32 xDelay[FILTER_LEN];
extern float32 coeffs[FILTER_LEN];
extern Uint16 AdcBuf[ADC_BUF_LEN];
extern Uint16 AdcBufFiltered[ADC_BUF_LEN];
extern Uint16 AdcBufRaw[2*ADC_BUF_LEN];
extern Uint16 ClaFilteredOutput;
extern Uint16 DacOffset;
extern Uint16 DacOutput;
extern Uint32 PwmDuty;
extern Uint32 PwmPeriod;
extern Uint16 AdcResult;
extern Uint16 DacData;
extern Uint16 SineData;
extern Uint16 DEBUG_TOGGLE;
extern Uint16 SINE_ENABLE;
extern Uint16 PWM_MODULATE;
extern int QuadratureTable[SINE_PTS];
extern const struct PIE_VECT_TABLE PieVectTableInit;    // PieVectTableInit is always extern


//---------------------------------------------------------------------------
// Macros
//

// The following pointer to a function call calibrates the ADC reference, 
// DAC offset, and internal oscillators
#define Device_cal (void   (*)(void))0x070282

// The following pointers to functions calibrate the ADC linearity.  Use this
// in the AdcSetMode(...) function only
#define CalAdcaINL (void   (*)(void))0x0703B4
#define CalAdcbINL (void   (*)(void))0x0703B2
#define CalAdccINL (void   (*)(void))0x0703B0
#define CalAdcdINL (void   (*)(void))0x0703AE

// The following pointer to a function call looks up the ADC offset trim for a
// given condition. Use this in the AdcSetMode(...) function only.
#define GetAdcOffsetTrimOTP (Uint16 (*)(Uint16 OTPoffset))0x0703AC


//---------------------------------------------------------------------------
#endif  // end of LAB_H definition


//--- end of file -----------------------------------------------------
