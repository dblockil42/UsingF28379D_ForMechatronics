/**********************************************************************
* File: SysCtrl.c
* Devices: TMS320F2837x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"						// Main include file


/**********************************************************************
* Function: InitSysCtrl()
* Description: Initializes the F2837x CPU
**********************************************************************/
void InitSysCtrl(void)
{
volatile Uint16 i;						// General purpose Uint16
volatile int16 dummy;					// General purpose volatile int16

	asm(" EALLOW");						// Enable EALLOW protected register access

//--- Call the Device_cal() function located in reserved OTP.  Device_cal is
//    a macro defined in the file Lab.h (as may be the case for the code being
//    used).  This macro simply defines Device_cal to be a function pointer to
//    the correct address in the reserved OTP.  Note that the device cal function
//    is automatically called by the bootloader.  A call to this function is included
//    here just in case the bootloader is bypassed during development.


	CpuSysRegs.PCLKCR13.bit.ADC_A = 1;					// Enable ADC-A clock
	CpuSysRegs.PCLKCR13.bit.ADC_B = 1;					// Enable ADC-B clock
	CpuSysRegs.PCLKCR13.bit.ADC_C = 1;					// Enable ADC-C clock
	CpuSysRegs.PCLKCR13.bit.ADC_D = 1;					// Enable ADC-D clock
//    (*Device_cal)();									// Call the calibration function
	CpuSysRegs.PCLKCR13.bit.ADC_A = 0;					// Disable ADC-A clock
	CpuSysRegs.PCLKCR13.bit.ADC_B = 0;					// Disable ADC-B clock
	CpuSysRegs.PCLKCR13.bit.ADC_C = 0;					// Disable ADC-C clock
	CpuSysRegs.PCLKCR13.bit.ADC_D = 0;					// Disable ADC-D clock

//--- Configure the clock sources

// Make sure missing clock detect has not triggered
	if (ClkCfgRegs.MCDCR.bit.MCLKSTS != 1)
	{													// PLL is not running in limp mode
		// Set the clock source.  We are running from INTOSC2 by default after a reset.
		ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 0; 	// Set clock source to INTOSC2 (default)
		ClkCfgRegs.CLKSRCCTL1.bit.XTALOFF=1;        	// Turn off XTALOSC
		ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 0;	// PLLSYSCLK divider from OSCCLK or PLLCLK.  0=/1 (run at 10 MHz)

		// Adjust the PLL for 400 MHz output frequency (from 10 MHz INTOSC2).
		// Must write to FMULT and IMULT fields together in SYSPLLMULT register.
		// Writing either FMULT or IMULT to a non-zero value also sets SYSPLLCTL1.PLLEN=1 (SYSPLL enable).

		ClkCfgRegs.SYSPLLMULT.all = 0x00000028;
// bit 31-10     0's:    reserved
// bit 9-8        00:    FMULT, fractional multiplier.  00=0.00, 01=0.25, 10=0.50, 11=0.75
// bit 7           0:    reserved
// bit 6-0   0101000:    IMULT, integer multiplier = 0x28 = 40

		// Wait for PLL to lock.
		// During this time the CPU is still running from INTOSC2/PLLSYSCLKDIV.
		// Once the PLL is stable we must manually switch over to use the PLL output.
		// Code is not required to sit and wait for the PLL to lock.  However, 
		// if the code does anything that is timing critical (e.g. something that
		// relies on the CPU clock frequency to be at speed), then it is best to wait
		// until PLL lock is complete.  The watchdog should be disabled before this loop
		// or fed within the loop (as is done here).
		while(ClkCfgRegs.SYSPLLSTS.bit.LOCKS != 1)		// Wait for LOCKS bit to set
		{
			WdRegs.WDKEY.all = 0x0055;					// Service the watchdog while waiting
			WdRegs.WDKEY.all = 0x00AA;					//   in case the user enabled it.
		}

		// After the PLL has locked, we want to switch to the PLL output but ramp the
		// PLLSYSCLKDIV slowly so that we avoid large inrush currents.
		// In this example, I switch through /8, /4, /3, and finally /2 dividers
		// with a small time delay in between each to let the power supply settle.
		// This is only an example.  The amount of time and divider increments that you
		// need to wait depends on the power supply feeding the processor (i.e., how
		// much voltage droop occurs due to the inrush currents, and how long it takes
		// the voltage regulators to recover).

		ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 4;	// Set initial divider at /8
		ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1;			// Switch over to use PLL output, 400 MHz /8 = 50 MHz
		DelayUs(20/8);									// Wait 20 us (just an example).  Remember we're running
														// at /8 speed here, so divide function argument by 8.
		ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 2;	// Change to /4 divider
		DelayUs(20/4);									// Wait 20 us (just an example)
		ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 1;	// Change to /2 divider
	}
	else												// Missing clock detect has triggered
	{
	// INTOSC2 has failed and the device is running in PLL limp mode.
	// User should replace the below with a call to an appropriate function,
	// for example shutdown the system (since something is very wrong!).
		asm(" ESTOP0");


	}

//--- Enable the clocks to the peripherals

	CpuSysRegs.PCLKCR0.bit.CLA1 = 1;					// CLA1 clock enabled
	CpuSysRegs.PCLKCR0.bit.DMA = 1;						// DMA clock enabled
	CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;				// CPU Timer0 clock enabled
	CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;				// CPU Timer1 clock enabled
	CpuSysRegs.PCLKCR0.bit.CPUTIMER2 = 1;				// CPU Timer2 clock enabled
	CpuSysRegs.PCLKCR0.bit.HRPWM = 1;					// HRPWM clock enabled
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;				// TBCLKSYNC bit is handled separately in InitEPwm()

	CpuSysRegs.PCLKCR1.bit.EMIF1 = 1;					// EMIF1 clock enabled
	CpuSysRegs.PCLKCR1.bit.EMIF2 = 1;					// EMIF2 clock enabled

	CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;					// ePWM1 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;					// ePWM2 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;					// ePWM3 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;					// ePWM4 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;					// ePWM5 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;					// ePWM6 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;					// ePWM7 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;					// ePWM8 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM9 = 1;					// ePWM9 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM10 = 1;					// ePWM10 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM11 = 1;					// ePWM11 clock enabled
	CpuSysRegs.PCLKCR2.bit.EPWM12 = 1;					// ePWM12 clock enabled

	CpuSysRegs.PCLKCR3.bit.ECAP1 = 1;					// eCAP1 clock enabled
	CpuSysRegs.PCLKCR3.bit.ECAP2 = 1;					// eCAP2 clock enabled
	CpuSysRegs.PCLKCR3.bit.ECAP3 = 1;					// eCAP3 clock enabled
	CpuSysRegs.PCLKCR3.bit.ECAP4 = 1;					// eCAP4 clock enabled
	CpuSysRegs.PCLKCR3.bit.ECAP5 = 1;					// eCAP5 clock enabled
	CpuSysRegs.PCLKCR3.bit.ECAP6 = 1;					// eCAP6 clock enabled

	CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;					// eQEP1 clock enabled
	CpuSysRegs.PCLKCR4.bit.EQEP2 = 1;					// eQEP2 clock enabled
	CpuSysRegs.PCLKCR4.bit.EQEP3 = 1;					// eQEP3 clock enabled

	CpuSysRegs.PCLKCR6.bit.SD1 = 1;						// Sigma-Delta1 clock enabled
	CpuSysRegs.PCLKCR6.bit.SD2 = 1;						// Sigma-Delta2 clock enabled

	CpuSysRegs.PCLKCR7.bit.SCI_A = 1;					// SCI-A clock enabled
	CpuSysRegs.PCLKCR7.bit.SCI_B = 1;					// SCI-B clock enabled
	CpuSysRegs.PCLKCR7.bit.SCI_C = 1;					// SCI-C clock enabled
	CpuSysRegs.PCLKCR7.bit.SCI_D = 1;					// SCI-D clock enabled

	CpuSysRegs.PCLKCR8.bit.SPI_A = 1;					// SPI-A clock enabled
	CpuSysRegs.PCLKCR8.bit.SPI_B = 1;					// SPI-B clock enabled
	CpuSysRegs.PCLKCR8.bit.SPI_C = 1;					// SPI-C clock enabled

	CpuSysRegs.PCLKCR9.bit.I2C_A = 1;					// I2C-A clock enabled
	CpuSysRegs.PCLKCR9.bit.I2C_B = 1;					// I2C-B clock enabled

	CpuSysRegs.PCLKCR10.bit.CAN_A = 1;					// CAN-A clock enabled
	CpuSysRegs.PCLKCR10.bit.CAN_B = 1;					// CAN-B clock enabled

	CpuSysRegs.PCLKCR11.bit.McBSP_A = 1;				// McBSP-A clock enabled
	CpuSysRegs.PCLKCR11.bit.McBSP_B = 1;				// McBSP-B clock enabled
	CpuSysRegs.PCLKCR11.bit.USB_A = 1;					// USB-A clock enabled

	CpuSysRegs.PCLKCR12.bit.uPP_A = 1;					// uPP-A clock enabled

	CpuSysRegs.PCLKCR13.bit.ADC_A = 1;					// ADC1 clock enabled
	CpuSysRegs.PCLKCR13.bit.ADC_B = 1;					// ADC2 clock enabled
	CpuSysRegs.PCLKCR13.bit.ADC_C = 1;					// ADC3 clock enabled
	CpuSysRegs.PCLKCR13.bit.ADC_D = 1;					// ADC4 clock enabled

	CpuSysRegs.PCLKCR14.bit.CMPSS1 = 1;					// Comparator sub-system 1 clock enabled
	CpuSysRegs.PCLKCR14.bit.CMPSS2 = 1;					// Comparator sub-system 2 clock enabled
	CpuSysRegs.PCLKCR14.bit.CMPSS3 = 1;					// Comparator sub-system 3 clock enabled
	CpuSysRegs.PCLKCR14.bit.CMPSS4 = 1;					// Comparator sub-system 4 clock enabled
	CpuSysRegs.PCLKCR14.bit.CMPSS5 = 1;					// Comparator sub-system 5 clock enabled
	CpuSysRegs.PCLKCR14.bit.CMPSS6 = 1;					// Comparator sub-system 6 clock enabled
	CpuSysRegs.PCLKCR14.bit.CMPSS7 = 1;					// Comparator sub-system 7 clock enabled
	CpuSysRegs.PCLKCR14.bit.CMPSS8 = 1;					// Comparator sub-system 8 clock enabled

	CpuSysRegs.PCLKCR16.bit.DAC_A = 1;					// DAC-A clock enabled
	CpuSysRegs.PCLKCR16.bit.DAC_B = 1;					// DAC-B clock enabled
	CpuSysRegs.PCLKCR16.bit.DAC_C = 1;					// DAC-C clock enabled

//--- Configure access to the common peripherals
	CpuSysRegs.SECMSEL.bit.PF1SEL = 0;					// PF1 (ePWM,eCAP,eQEP,CMPSS,DAC,SDFM) access, 0=CLA, 1=DMA
	CpuSysRegs.SECMSEL.bit.PF2SEL = 0;					// PF2 (SPI, McBSP, UPP) access, 0=CLA, 1=DMA

//--- Configure the low-power modes
	CpuSysRegs.LPMCR.bit.LPM = 0;		// LPM bits set IDLE mode (default)

//--- Finish up
	asm(" EDIS");						// Disable EALLOW protected register access

} // end InitSysCtrl()


//--- end of file -----------------------------------------------------
