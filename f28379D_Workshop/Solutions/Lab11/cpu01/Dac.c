/**********************************************************************
* File: Dac.c
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"						// Main include file


/**********************************************************************
* Function: InitDacb()
*
* Description: Initializes DAC-B for the F28x7x
**********************************************************************/
void InitDacb(void)
{
	asm(" EALLOW");						// Enable EALLOW protected register access

//--- Configure DAC-B control registers
    DacbRegs.DACCTL.all = 0x0001;
// bit 15-8      0's:    reserved
// bit 7-4    0000:      DAC PWMSYNC select, not used since LOADMODE=0
// bit 3         0:      reserved
// bit 2         0:      LOADMODE, DACVALA load mode, 0=next SYSCLK, 1=next PWMSYNC specified by SYNCSEL
// bit 1         0:      reserved
// bit 0         1:      DACREFSEL, DAC reference select, 0=VDAC/VSSA, 1=ADC VREFHI/VREFLO

//--- Set DAC-B output to mid-range
    DacbRegs.DACVALS.all = 0x0800;         // DACVALS = bits 11-0, bits 15-12 reserved 

//--- Enable DAC-B output
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;    // DAC output enable, 0=disable, 1=enable

//--- DAC-B lock control register
    DacbRegs.DACLOCK.all = 0x0000;         // Write a 1 to lock (cannot be cleared once set)

	asm(" EDIS");						// Disable EALLOW protected register access

} // end of InitDac()


//--- end of file -----------------------------------------------------
