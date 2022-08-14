/**********************************************************************
* File: Main_11_cpu1.c -- Solution File for Lab 11_cpu1
* Devices: TMS320F28379D
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"                        // Main include file

//--- Global Variables
Uint16 DacData;                           // DAC value from CPU2 via IPC0
Uint16 AdcResult;                         // ADC result to CPU2 via IPC1


/**********************************************************************
* Function: main()
*
* Description: Main function for C28x workshop labs
**********************************************************************/
void main(void)
{
//--- CPU Initialization
	InitSysCtrl();						// Initialize the CPU (FILE: SysCtrl.c)
	InitGpio();							// Initialize the shared GPIO pins (FILE: Gpio.c)
	InitPieCtrl();						// Initialize and enable the PIE (FILE: PieCtrl.c)
	InitWatchdog();						// Initialize the Watchdog Timer (FILE: WatchDog.c)

//--- Peripheral Initialization
	InitAdca();							// Initialize the ADC-A (FILE: Adc.c)
	InitDacb();                         // Initialize the DAC-B (File: Dac.c)
	InitEPwm();							// Initialize the EPwm (FILE: EPwm.c) 

//--- Clear IPC Flags
	IpcRegs.IPCCLR.all = 0xFFFFFFFF;

//--- Enable IPC PIE interrupts
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;	    // IPC0 ISR
    IER |= 0x0001;					    	// Enable INT1 in IER to enable PIE group 1

//--- Wait here until CPU2 is ready
	while (IpcRegs.IPCSTS.bit.IPC17 == 0) ;	// Wait for CPU2 to set IPC17
	IpcRegs.IPCACK.bit.IPC17 = 1;			// Acknowledge and clear IPC17

//--- Enable global interrupts
	asm(" CLRC INTM, DBGM");			// Enable global interrupts and realtime debug

//--- Main Loop
	while(1)							// endless loop - wait for an interrupt
	{
		asm(" NOP");
	}


} //end of main()


/*** end of file *****************************************************/
