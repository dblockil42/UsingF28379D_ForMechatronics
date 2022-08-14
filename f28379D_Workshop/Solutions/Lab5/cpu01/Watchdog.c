/**********************************************************************
* File: Watchdog.c -- Solution File
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"						// Main include file


/**********************************************************************
* Function: InitWatchdog()
*
* Description: Initializes Watchdog Timer for the F28x7x
**********************************************************************/
void InitWatchdog(void)
{
	asm(" EALLOW");						// Enable EALLOW protected register access

//--- Disable the Watchdog Timer
	WdRegs.WDCR.all = 0x0068;
// bit 15-7      0's:    reserved
// bit 6         1:      WDDIS, 1=disable WD
// bit 5-3       101:    WDCHK, WD check bits, always write as 101b
// bit 2-0       000:    WDPS, WD prescale bits, 000: WDCLK=OSCCLK/512/1

//--- System and Control Register
	WdRegs.SCSR.all = 0x0000;
// bit 15-3      0's:    reserved
// bit 2         0:      WDINTS, WD interrupt status bit (read-only)
// bit 1         0:      WDENINT, 0=WD causes reset, 1=WD causes WDINT
// bit 0         0:      WDOVERRIDE, write 1 to disable disabling of the WD (clear-only)

	WdRegs.WDWCR.all = 0x0000;
// bit 15-9      0's:    reserved
// bit 8         0:      FIRSTKEY (read-only)
// bit 7-0       0x00:   MIN, minimum service interval - 0x00 is no minimum

	asm(" EDIS");						// Disable EALLOW protected register access

//--- Enable the Watchdog interrupt
	PieCtrlRegs.PIEIER1.bit.INTx8 = 1;	// Enable WAKEINT (LPM/WD) in PIE group #1
	IER |= 0x0001;						// Enable INT1 in IER to enable PIE group 1

} // end of InitWatchdog()


//--- end of file -----------------------------------------------------
