/**********************************************************************
* File: Main_11_cpu2.c -- Solution File for Lab 11_cpu2
* Devices: TMS320F28379D
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"                        // Main include file

//--- Global Variables
Uint16 AdcBuf[ADC_BUF_LEN];             // ADC buffer allocation
Uint16 SineData;                        // Sine table data to CPU1 via IPC0


/**********************************************************************
* Function: main()
*
* Description: Main function for C28x workshop labs
**********************************************************************/
void main(void)
{
//--- CPU Initialization
	InitPieCtrl();						// Initialize and enable the PIE (FILE: PieCtrl.c)
	InitWatchdog();						// Initialize the Watchdog Timer (FILE: WatchDog.c)

//--- Enable global interrupts
	asm(" CLRC INTM, DBGM");			// Enable global interrupts and realtime debug

//--- Clear IPC1 Flag
    IpcRegs.IPCCLR.bit.IPC1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

//--- Enable IPC PIE interrupts
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1;		// Enable IPC1 ISR
	IER |= 0x0001;                          // Enable INT1 in IER to enable PIE group 1

//--- Let CPU1 know that CPU2 is ready
	IpcRegs.IPCSET.bit.IPC17 = 1;		// Set IPC17 to release CPU1

//--- Enable global interrupts
	asm(" CLRC INTM, DBGM");			// Enable global interrupts and realtime debug

//--- Main Loop
	while(1)							// endless loop - wait for an interrupt
	{
		asm(" NOP");
	}


} //end of main()


/*** end of file *****************************************************/
