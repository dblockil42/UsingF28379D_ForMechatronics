//###########################################################################
//
// FILE:   buffdac_sine_cpu01.c
//
// TITLE:  Buffered DAC Sine Wave Output Example for F2837xD.
//
//! \addtogroup cpu01_example_list
//! <h1> Buffered DAC Sine (buffdac_sine) </h1>
//!
//! This example generates a sine wave on the buffered DAC output,
//! DACOUTA/ADCINA0 (HSEC Pin 9) and uses the default DAC reference setting
//! of VDAC.
//!
//! When the DAC reference is set to VDAC, an external reference voltage
//! must be applied to the VDAC pin. This can accomplished by connecting a
//! jumper wire from 3.3V to ADCINB0 (HSEC pin 12).
//!
//! Run the included .js file to add the watch variables.
//! This example uses the SGEN module. Documentation for the SGEN module can be
//! found in the SGEN library directory.
//!
//! The generated waveform can be adjusted with the following variables while
//! running:
//! - \b waveformGain \b : Adjust the magnitude of the waveform.
//! Range is from 0.0 to 1.0. The default value of 0.8003 centers the waveform
//! within the linear range of the DAC
//! - \b waveformOffset \b : Adjust the offset of the waveform.
//!    Range is from -1.0 to 1.0. The default value of 0 centers the waveform
//! - \b outputFreq_hz \b : Adjust the output frequency of the waveform.
//! Range is from 0 to maxOutputFreq_hz
//! - \b maxOutputFreq_hz \b : Adjust the max output frequency of the waveform.
//! Range - See SGEN module documentation for how this affects other parameters
//!
//! The generated waveform can be adjusted with the following variables/macros
//! but require recompile:
//! - \b samplingFreq_hz \b : Adjust the rate at which the DAC is updated.
//! Range - See SGEN module documentation for how this affects other parameters
//! - \b SINEWAVE_TYPE \b : The type of sine generated.
//! Range - LOW_THD_SINE, HIGH_PRECISION_SINE
//! - \b REFERENCE \b : The reference for the DAC.
//! Range - REFERENCE_VDAC, REFERENCE_VREF
//! - \b CPUFREQ_MHZ \b : The cpu frequency. This does not set the cpu
//!                       frequency.
//! Range - See device data manual
//! - \b DAC_NUM \b : The DAC to use.
//! Range - DACA, DACB, DACC
//!
//! The following variables give additional information about the generated
//! waveform:
//! See SGEN module documentation for details
//! - \b freqResolution_hz \b
//! - \b maxOutput_lsb \b : Maximum value written to the DAC.
//! - \b minOutput_lsb \b : Minimum value written to the DAC.
//! - \b pk_to_pk_lsb \b : Magnitude of generated waveform.
//! - \b cpuPeriod_us \b : Period of cpu.
//! - \b samplingPeriod_us \b : The rate at which the DAC is updated.
//! Note that samplingPeriod_us has to be greater than the DAC settling time.
//! - \b interruptCycles \b : Interrupt duration in cycles.
//! - \b interruptDuration_us \b : Interrupt duration in uS.
//! - \b sgen \b : The SGEN module instance.
//! - \b DataLog \b : Circular log of writes to the DAC.
//!
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
#include "F28x_Project.h"
#include "sgen.h"         // Signal Generation Headerfile

//
// Defines
//
#define DLOG_SIZE             1024
#define REFERENCE_VDAC        0
#define REFERENCE_VREF        1
#define LOW_THD_SINE          0
#define HIGH_PRECISION_SINE   1
#define DACA                  1
#define DACB                  2
#define DACC                  3
#define SINEWAVE_TYPE         LOW_THD_SINE
#define REFERENCE             REFERENCE_VDAC
#define CPUFREQ_MHZ           200
#define DAC_NUM               DACA

//
// Globals
//
Uint16 DataLog[DLOG_SIZE];
#pragma DATA_SECTION(DataLog, "DLOG");
volatile struct DAC_REGS* DAC_PTR[4] = {0x0,&DacaRegs,&DacbRegs,&DaccRegs};
Uint32 samplingFreq_hz = 200000;
Uint32 outputFreq_hz = 1000;
Uint32 maxOutputFreq_hz = 5000;
float waveformGain = 0.8003; // Range 0.0 -> 1.0
float waveformOffset = 0;    // Range -1.0 -> 1.0

#if SINEWAVE_TYPE==LOW_THD_SINE //initialize sine wave type
SGENTI_1 sgen = SGENTI_1_DEFAULTS;
#elif SINEWAVE_TYPE==HIGH_PRECISION_SINE
SGENHP_1 sgen = SGENHP_1_DEFAULTS;
#endif

Uint16 sgen_out = 0;
Uint16 ndx = 0;
float freqResolution_hz = 0;
float cpuPeriod_us = 0;
Uint32 interruptCycles = 0;
float interruptDuration_us = 0;
float samplingPeriod_us = 0;
Uint16 maxOutput_lsb = 0;
Uint16 minOutput_lsb = 0;
Uint16 pk_to_pk_lsb = 0;

//
// Function Prototypes
//
static inline void dlog(Uint16 value);
static inline void setFreq(void);
static inline void setGain(void);
static inline void setOffset(void);
static inline Uint16 getMax(void);
static inline Uint16 getMin(void);
void configureDAC(Uint16 dac_num);
void configureWaveform(void);
interrupt void cpu_timer0_isr(void);

//
// Main
//
void main(void)
{
//
// Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Clear all interrupts and initialize PIE vector table:
//
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

//
// Map Cpu Timer0 interrupt function to the PIE vector table
//
    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

//
// Initialize variables
//
    cpuPeriod_us = (1.0/CPUFREQ_MHZ);
    samplingPeriod_us = (1000000.0/samplingFreq_hz);

//
// Initialize datalog
//
    for(ndx=0; ndx<DLOG_SIZE; ndx++)
    {
        DataLog[ndx] = 0;
    }
    ndx = 0;

//
// Configure DAC
//
    configureDAC(DAC_NUM);

//
// Configure Waveform
//
    configureWaveform();

//
// Initialize Cpu Timers
//
    InitCpuTimers();

//
// Configure Cpu Timer0 to interrupt at specified sampling frequency
//
    ConfigCpuTimer(&CpuTimer0, CPUFREQ_MHZ, 1000000.0/samplingFreq_hz);

//
// Start Cpu Timer0
//
    CpuTimer0Regs.TCR.all = 0x4000;

//
// Enable interrupt
//
    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    EINT;
    ERTM;

    while(1)
    {
        setFreq();   // Set Output Frequency and Max Output Frequency
        setGain();   // Set Magnitude of Waveform
        setOffset(); // Set Offset of Waveform
        maxOutput_lsb = getMax();
        minOutput_lsb = getMin();
        pk_to_pk_lsb = maxOutput_lsb - minOutput_lsb;
    }
}

//
// dlog - Circular DataLog. DataLog[0] contains the next index to
//        be overwritten
//
static inline void dlog(Uint16 value)
{
    DataLog[ndx] = value;
    if(++ndx == DLOG_SIZE)
    {
        ndx = 0;
    }
    DataLog[0] = ndx;
}

//
// setFreq - Set the SINE frequency in SGEN
//
static inline void setFreq(void)
{
#if SINEWAVE_TYPE==LOW_THD_SINE
    //
    // Range(Q0) = 0x0000 -> 0x7FFF, step_max(Q0) =
    // (Max_Freq_hz*0x10000)/Sampling_Freq_hz
    //
    sgen.step_max = (maxOutputFreq_hz*0x10000)/samplingFreq_hz;

    //
    // Range(Q15) = 0x0000 -> 0x7FFF, freq(Q15) =
    // (Required_Freq_hz/Max_Freq_hz)*0x8000
    //
    sgen.freq = ((float)outputFreq_hz/maxOutputFreq_hz)*0x8000;
#elif SINEWAVE_TYPE==HIGH_PRECISION_SINE
    //
    // Range(Q0) = 0x00000000 -> 0x7FFFFFFF, step_max(Q0) =
    // (Max_Freq_hz*0x100000000)/Sampling_Freq_hz
    //
    sgen.step_max = (maxOutputFreq_hz*0x100000000)/samplingFreq_hz;

    //
    // Range(Q31) = 0x00000000 -> 0x7FFFFFFF, freq(Q31) =
    // (Required_Freq_hz/Max_Freq_hz)*0x80000000
    //
    sgen.freq = ((float)outputFreq_hz/maxOutputFreq_hz)*0x80000000;
#endif

    freqResolution_hz = (float)maxOutputFreq_hz/sgen.step_max;
}

//
// setGain - Set the gain in SGEN
//
static inline void setGain(void)
{
    sgen.gain = waveformGain * 0x7FFF;   // Range(Q15) = 0x0000 -> 0x7FFF
}

//
// setOffset - Set the offset in SGEN
//
static inline void setOffset(void)
{
    sgen.offset = waveformOffset * 0x7FFF; // Range(Q15) = 0x8000 -> 0x7FFF
}

//
// getMax - Get the max value in the data log
//
static inline Uint16 getMax(void)
{
    Uint16 index = 0;
    Uint16 tempMax = 0;

    for(index=1; index<DLOG_SIZE; index++)
    {
        if(tempMax<DataLog[index])
        {
            tempMax = DataLog[index];
        }
    }

    return tempMax;
}

//
// getMin - Get the min value in the data log
//
static inline Uint16 getMin(void)
{
    Uint16 index = 0;
    Uint16 tempMin = 0xFFFF;

    for(index=1; index<DLOG_SIZE; index++)
    {
        if(tempMin>DataLog[index])
        {
            tempMin = DataLog[index];
        }
    }

    return tempMin;
}

//
// configureDAC - Enable and configure the requested DAC module
//
void configureDAC(Uint16 dac_num)
{
    EALLOW;

    DAC_PTR[dac_num]->DACCTL.bit.DACREFSEL = REFERENCE;
    DAC_PTR[dac_num]->DACOUTEN.bit.DACOUTEN = 1;
    DAC_PTR[dac_num]->DACVALS.all = 0;

    DELAY_US(10); // Delay for buffered DAC to power up

    EDIS;
}

//
// configureWaveform - Configure the SINE waveform
//
void configureWaveform(void)
{
    sgen.alpha = 0; // Range(16) = 0x0000 -> 0xFFFF
    setFreq();
    setGain();
    setOffset();
}

//
// cpu_timer0_isr - Timer ISR that writes the sine value to DAC, log the sine
//                  value, compute the next sine value, and calculate interrupt
//                  duration
//
interrupt void cpu_timer0_isr(void)
{
    //
    // Start Cpu Timer1 to indicate begin of interrupt
    //
    CpuTimer1Regs.TCR.all = 0x0000;

    //
    // Write current sine value to buffered DAC
    //
    DAC_PTR[DAC_NUM]->DACVALS.all = sgen_out;

    //
    // Log current sine value
    //
    dlog(sgen_out);

    //
    // Compute next sine value
    //
    sgen.calc(&sgen);

    //
    // Scale next sine value
    //
    sgen_out = (sgen.out + 32768) >> 4;

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    //
    // Stop Cpu Timer1 to indicate end of interrupt
    //
    CpuTimer1Regs.TCR.all = 0x0010;

    //
    // Calculate interrupt duration in cycles
    //
    interruptCycles = 0xFFFFFFFFUL - CpuTimer1Regs.TIM.all;

    //
    // Calculate interrupt duration in micro seconds
    //
    interruptDuration_us = cpuPeriod_us * interruptCycles;

    //
    // Reload Cpu Timer1
    //
    CpuTimer1Regs.TCR.all = 0x0030;
}

//
// End of file
//
