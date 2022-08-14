/**********************************************************************
* File: ECap.c -- Solution File
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"						// Main include file


/**********************************************************************
* Function: InitECap()
*
* Description: Initializes the Enhanced Capture modules on the F28x7x
**********************************************************************/
void InitECap(void)
{
//---------------------------------------------------------------------
//--- Configure eCAP1 unit for capture                        
//---------------------------------------------------------------------
	asm(" EALLOW");							// Enable EALLOW protected register access
	DevCfgRegs.SOFTPRES3.bit.ECAP1 = 1;		// eCAP1 is reset
	DevCfgRegs.SOFTPRES3.bit.ECAP1 = 0;		// eCAP1 is released from reset
	asm(" EDIS");

	ECap1Regs.ECEINT.all = 0;					// Disable all eCAP interrupts
	ECap1Regs.ECCTL1.bit.CAPLDEN = 0;			// Disabled loading of capture results
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;			// Stop the counter

	ECap1Regs.TSCTR = 0;						// Clear the counter
	ECap1Regs.CTRPHS = 0;						// Clear the counter phase register

	ECap1Regs.ECCTL2.all = 0x0096;				// ECAP control register 2
// bit 15-11     00000:  reserved
// bit 10        0:      APWMPOL, don't care
// bit 9         0:      CAP/APWM, 0 = capture mode, 1 = APWM mode
// bit 8         0:      SWSYNC, 0 = no action (no s/w synch)
// bit 7-6       10:     SYNCO_SEL, 10 = disable sync out signal
// bit 5         0:      SYNCI_EN, 0 = disable Sync-In
// bit 4         1:      TSCTRSTOP, 1 = enable counter
// bit 3         0:      RE-ARM, 0 = don't re-arm, 1 = re-arm
// bit 2-1       11:     STOP_WRAP, 11 = wrap after 4 captures
// bit 0         0:      CONT/ONESHT, 0 = continuous mode

	ECap1Regs.ECCTL1.all = 0xC144;				// ECAP control register 1
// bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
// bit 13-9      00000:  PRESCALE, 00000 = divide by 1
// bit 8         1:      CAPLDEN, 1 = enable capture results load
// bit 7         0:      CTRRST4, 0 = do not reset counter on CAP4 event
// bit 6         1:      CAP4POL, 0 = rising edge, 1 = falling edge
// bit 5         0:      CTRRST3, 0 = do not reset counter on CAP3 event
// bit 4         0:      CAP3POL, 0 = rising edge, 1 = falling edge
// bit 3         0:      CTRRST2, 0 = do not reset counter on CAP2 event
// bit 2         1:      CAP2POL, 0 = rising edge, 1 = falling edge
// bit 1         0:      CTRRST1, 0 = do not reset counter on CAP1 event
// bit 0         0:      CAP1POL, 0 = rising edge, 1 = falling edge

	ECap1Regs.ECEINT.all = 0x0008;				// Enable desired eCAP interrupts
// bit 15-8      0's:    reserved
// bit 7         0:      CTR=CMP, 0 = compare interrupt disabled
// bit 6         0:      CTR=PRD, 0 = period interrupt disabled
// bit 5         0:      CTROVF, 0 = overflow interrupt disabled
// bit 4         0:      CEVT4, 0 = event 4 interrupt disabled
// bit 3         1:      CEVT3, 1 = event 3 interrupt enabled
// bit 2         0:      CEVT2, 0 = event 2 interrupt disabled
// bit 1         0:      CEVT1, 0 = event 1 interrupt disabled
// bit 0         0:      reserved

	PieCtrlRegs.PIEIER4.bit.INTx1 = 1;	// Enable ECAP1_INT in PIE group 4
	IER |= 0x0008;						// Enable INT4 in IER to enable PIE group 4

} // end InitECap()


//--- end of file -----------------------------------------------------
