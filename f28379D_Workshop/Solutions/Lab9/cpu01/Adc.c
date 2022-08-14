/**********************************************************************
* File: Adc.c -- Solution File
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"				// Main include file


/**********************************************************************
* Function: InitAdca()
*
* Description: Initializes ADC-A on the F28x7x
**********************************************************************/
void InitAdca(void)
{
	asm(" EALLOW");						// Enable EALLOW protected register access

//--- Reset the ADC.  This is good programming practice.
	DevCfgRegs.SOFTPRES13.bit.ADC_A = 1;	// ADC is reset
	DevCfgRegs.SOFTPRES13.bit.ADC_A = 0;	// ADC is released from reset

//--- Configure the ADC base registers
	AdcaRegs.ADCCTL1.all = 0x0004;		// Main ADC configuration
// bit 15-14     00:     reserved
// bit 13        0:      ADCBSY, ADC busy, read-only
// bit 12        0:      reserved
// bit 11-8      0's:    ADCBSYCHN, ADC busy channel, read-only
// bit 7         0:      ADCPWDNZ, ADC power down, 0=powered down, 1=powered up
// bit 6-3       0000:   reserved
// bit 2         1:      INTPULSEPOS, INT pulse generation, 0=start of conversion, 1=end of conversion
// bit 1-0       00:     reserved

	AdcaRegs.ADCCTL2.all = 0x0006;		// ADC clock configuration
// bit 15-8      0's:    reserved
// bit 7         0:      SIGNALMODE, configured by AdcSetMode() below to get calibration correct
// bit 6         0:      RESOLUTION, configured by AdcSetMode() below to get calibration correct
// bit 5-4       00:     reserved
// bit 3-0       0110:   PRESCALE, ADC clock prescaler.  0110=CPUCLK/4

	AdcaRegs.ADCBURSTCTL.all = 0x0000;
// bit 15        0:      BURSTEN, 0=burst mode disabled, 1=burst mode enabled
// bit 14-12     000:    reserved
// bit 11-8      0000:   BURSTSIZE, 0=1 SOC converted (don't care)
// bit 7-6       00:     reserved
// bit 5-0       000000: BURSTTRIGSEL, 00=software only (don't care)

//--- Call AdcSetMode() to configure the resolution and signal mode.
//    This also performs the correct ADC calibration for the configured mode.
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

//--- SOC0 configuration
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7;		// Trigger using ePWM2-ADCSOCA
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;			// Convert channel ADCINA0 (Ch. 0)
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19;			// Acquisition window set to (19+1)=20 cycles (100 ns with 200 MHz SYSCLK)

	AdcaRegs.ADCINTSOCSEL1.bit.SOC0 = 0;		// No ADC interrupt triggers SOC0 (TRIGSEL field determines trigger)

	AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 0;	// All SOCs handled in round-robin mode

//--- ADCA1 interrupt configuration
	AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1;		// Interrupt pulses regardless of flag state
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;		// Enable the interrupt in the ADC
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;		// EOC0 triggers the interrupt

//--- Enable the ADC interrupt
//	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;			// Enable ADCA1 interrupt in PIE group 1
	IER |= 0x0001;								// Enable INT1 in IER to enable PIE group

//--- Finish up
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;			// Power up the ADC
	DelayUs(1000);								// Wait 1 ms after power-up before using the ADC
	asm(" EDIS");								// Disable EALLOW protected register access

} // end InitAdc()


//--- end of file -----------------------------------------------------
