/**********************************************************************
* File: EPwm.c -- Solution File
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"				// Main include file


/**********************************************************************
* Function: InitEPwm()
*
* Description: Initializes the Enhanced PWM modules on the F28x7x
**********************************************************************/
void InitEPwm(void)
{
asm(" EALLOW");						// Enable EALLOW protected register access

	// Configure the prescaler to the ePWM modules.  Max ePWM input clock is 100 MHz.
	ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;		// EPWMCLK divider from PLLSYSCLK.  0=/1, 1=/2

	// Must disable the clock to the ePWM modules if you want all ePWM modules synchronized.
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

	asm(" EDIS");						// Disable EALLOW protected register access


//---------------------------------------------------------------------
//--- Configure ePWM2 to trigger ADC SOCA at a 50 kHz rate
//---------------------------------------------------------------------
	asm(" EALLOW");							// Enable EALLOW protected register access
	DevCfgRegs.SOFTPRES2.bit.EPWM2 = 1;		// ePWM2 is reset
	DevCfgRegs.SOFTPRES2.bit.EPWM2 = 0;		// ePWM2 is released from reset
	asm(" EDIS");							// Disable EALLOW protected register access

	EPwm2Regs.TBCTL.bit.CTRMODE = 0x3;		// Disable the timer

	EPwm2Regs.TBCTL.all = 0xC033;			// Configure timer control register
// bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
// bit 13        0:      PHSDIR, 0 = count down after sync event
// bit 12-10     000:    CLKDIV, 000 => TBCLK = HSPCLK/1
// bit 9-7       000:    HSPCLKDIV, 000 => HSPCLK = EPWMCLK/1
// bit 6         0:      SWFSYNC, 0 = no software sync produced
// bit 5-4       11:     SYNCOSEL, 11 = sync-out disabled
// bit 3         0:      PRDLD, 0 = reload PRD on counter=0
// bit 2         0:      PHSEN, 0 = phase control disabled
// bit 1-0       11:     CTRMODE, 11 = timer stopped (disabled)

	EPwm2Regs.TBCTR = 0x0000;				// Clear timer counter
	EPwm2Regs.TBPRD = ADC_SAMPLE_PERIOD;	// Set timer period
	EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;		// Set timer phase

	EPwm2Regs.ETPS.all = 0x0100;			// Configure SOCA
// bit 15-14     00:     EPWMxSOCB, read-only
// bit 13-12     00:     SOCBPRD, don't care
// bit 11-10     00:     EPWMxSOCA, read-only
// bit 9-8       01:     SOCAPRD, 01 = generate SOCA on first event
// bit 7-4       0000:   reserved
// bit 3-2       00:     INTCNT, don't care
// bit 1-0       00:     INTPRD, don't care

	EPwm2Regs.ETSEL.all = 0x0A00;			// Enable SOCA to ADC
// bit 15        0:      SOCBEN, 0 = disable SOCB
// bit 14-12     000:    SOCBSEL, don't care
// bit 11        1:      SOCAEN, 1 = enable SOCA
// bit 10-8      010:    SOCASEL, 010 = SOCA on PRD event
// bit 7-4       0000:   reserved
// bit 3         0:      INTEN, 0 = disable interrupt
// bit 2-0       000:    INTSEL, don't care

	EPwm2Regs.TBCTL.bit.CTRMODE = 0x0;		// Enable the timer in count up mode


//---------------------------------------------------------------------
//--- Configure ePWM1 for 2 kHz symmetric PWM on EPWM1A pin    
//---------------------------------------------------------------------
	asm(" EALLOW");							// Enable EALLOW protected register access
	DevCfgRegs.SOFTPRES2.bit.EPWM1 = 1;		// ePWM1 is reset
	DevCfgRegs.SOFTPRES2.bit.EPWM1 = 0;		// ePWM1 is released from reset
	asm(" EDIS");							// Disable EALLOW protected register access

	EPwm1Regs.TBCTL.bit.CTRMODE = 0x3;		// Disable the timer

	EPwm1Regs.TBCTL.all = 0xC033;			// Configure timer control register
// bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
// bit 13        0:      PHSDIR, 0 = count down after sync event
// bit 12-10     000:    CLKDIV, 000 => TBCLK = HSPCLK/1
// bit 9-7       000:    HSPCLKDIV, 000 => HSPCLK = EPWMCLK/1
// bit 6         0:      SWFSYNC, 0 = no software sync produced
// bit 5-4       11:     SYNCOSEL, 11 = sync-out disabled
// bit 3         0:      PRDLD, 0 = reload PRD on counter=0
// bit 2         0:      PHSEN, 0 = phase control disabled
// bit 1-0       11:     CTRMODE, 11 = timer stopped (disabled)

	EPwm1Regs.TBCTR = 0x0000;				// Clear timer counter
	EPwm1Regs.TBPRD = PWM_HALF_PERIOD;		// Set timer period
	EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;		// Set timer phase

	EPwm1Regs.CMPA.bit.CMPA = PWM_DUTY_CYCLE;	// Set PWM duty cycle

	EPwm1Regs.CMPCTL.all = 0x0002;			// Compare control register
// bit 15-10     0's:    reserved
// bit 9         0:      SHDWBFULL, read-only
// bit 8         0:      SHDWAFULL, read-only
// bit 7         0:      reserved
// bit 6         0:      SHDWBMODE, don't care
// bit 5         0:      reserved
// bit 4         0:      SHDWAMODE, 0 = shadow mode
// bit 3-2       00:     LOADBMODE, don't care
// bit 1-0       10:     LOADAMODE, 10 = load on zero or PRD match

	EPwm1Regs.AQCTLA.all = 0x0060;		// Action-qualifier control register A
// bit 15-12     0000:   reserved
// bit 11-10     00:     CBD, 00 = do nothing
// bit 9-8       00:     CBU, 00 = do nothing
// bit 7-6       01:     CAD, 01 = clear
// bit 5-4       10:     CAU, 10 = set
// bit 3-2       00:     PRD, 00 = do nothing
// bit 1-0       00:     ZRO, 00 = do nothing

	EPwm1Regs.AQSFRC.all = 0x0000;		// Action-qualifier s/w force register
// bit 15-8      0's:    reserved
// bit 7-6       00:     RLDCSF, 00 = reload AQCSFRC on zero
// bit 5         0:      OTSFB, 0 = do not initiate a s/w forced event on output B
// bit 4-3       00:     ACTSFB, don't care
// bit 2         0:      OTSFA, 0 = do not initiate a s/w forced event on output A
// bit 1-0       00:     ACTSFA, don't care

	EPwm1Regs.AQCSFRC.all = 0x0000;		// Action-qualifier continuous s/w force register
// bit 15-4      0's:    reserved
// bit 3-2       00:     CSFB, 00 = forcing disabled
// bit 1-0       00:     CSFA, 00 = forcing disabled

	EPwm1Regs.DBCTL.bit.OUT_MODE = 0;	// Deadband disabled
	EPwm1Regs.PCCTL.bit.CHPEN = 0;		// PWM chopper unit disabled
	EPwm1Regs.TZDCSEL.all = 0x0000;		// All trip zone and DC compare actions disabled

	EPwm1Regs.TBCTL.bit.CTRMODE = 0x2;	// Enable the timer in count up/down mode

//---------------------------------------------------------------------
//--- Enable the clocks to the ePWM module.                   
//--- Note: this should be done after all ePWM modules are configured
//--- to ensure synchronization between the ePWM modules.
//---------------------------------------------------------------------
	asm(" EALLOW");							// Enable EALLOW protected register access
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;	// TBCLK to ePWM modules enabled
	asm(" EDIS");							// Disable EALLOW protected register access

} // end InitEPwm()


//--- end of file -----------------------------------------------------
