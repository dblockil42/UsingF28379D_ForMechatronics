/**********************************************************************
* File: Gpio.c -- Solution File
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"						// Main include file


/**********************************************************************
* Function: InitGpio()
*
* Description: Initializes the shared GPIO pins on the F28x7x
**********************************************************************/
void InitGpio(void)
{
	asm(" EALLOW");								// Enable EALLOW protected register access

//--- Unlock the GPxLOCK register bits for all ports
	GpioCtrlRegs.GPACR.all   = 0x00000000;
	GpioCtrlRegs.GPBCR.all   = 0x00000000;
	GpioCtrlRegs.GPCCR.all   = 0x00000000;
	GpioCtrlRegs.GPDCR.all   = 0x00000000;
	GpioCtrlRegs.GPECR.all   = 0x00000000;
	GpioCtrlRegs.GPFCR.all   = 0x00000000;

//--- Disable the register locks for all ports
	GpioCtrlRegs.GPALOCK.all = 0x00000000;
	GpioCtrlRegs.GPBLOCK.all = 0x00000000;
	GpioCtrlRegs.GPCLOCK.all = 0x00000000;
	GpioCtrlRegs.GPDLOCK.all = 0x00000000;
	GpioCtrlRegs.GPELOCK.all = 0x00000000;
	GpioCtrlRegs.GPFLOCK.all = 0x00000000;

//--- Group A pins
	GpioCtrlRegs.GPACTRL.all  = 0x00000000;		// QUALPRD = PLLSYSCLK for all group A GPIO
	GpioCtrlRegs.GPAQSEL1.all = 0x00000000;		// Synchronous qualification for all group A GPIO 0-15
	GpioCtrlRegs.GPAQSEL2.all = 0x00000000;		// Synchronous qualification for all group A GPIO 16-31
	GpioCtrlRegs.GPADIR.all   = 0x00000000;		// All GPIO are inputs
	GpioCtrlRegs.GPAPUD.all   = 0x00000000;		// All pullups enabled
	GpioCtrlRegs.GPAINV.all   = 0x00000000;		// No inputs inverted
	GpioCtrlRegs.GPAODR.all   = 0x00000000;		// All outputs normal mode (no open-drain outputs)
	GpioCtrlRegs.GPACSEL1.all = 0x00000000;		// GPIO 0-7   \.
	GpioCtrlRegs.GPACSEL2.all = 0x00000000;		// GPIO 8-15   |. GPIODAT/SET/CLEAR/TOGGLE reg. master:
	GpioCtrlRegs.GPACSEL3.all = 0x00000000;		// GPIO 16-23  |. 0=CPU1, 1=CPU1.CLA1, 2=CPU2, 3=CPU2.CLA1
	GpioCtrlRegs.GPACSEL4.all = 0x00000000;		// GPIO 24-31 /.

	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;		// 0|0=GPIO  0|1=EPWM1A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1;		// 1|0=GPIO  1|1=rsvd         1|2=SDAA         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;		// 0|0=GPIO  0|1=EPWM1B       0|2=rsvd         0|3=MFSRB
	GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCLA         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;		// 0|0=GPIO  0|1=EPWM2A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR1  1|2=SDAB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;		// 0|0=GPIO  0|1=EPWM2B       0|2=OUTPUTXBAR2  0|3=MCLKRB
	GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR2  1|2=SCLB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;		// 0|0=GPIO  0|1=EPWM3A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR3  1|2=CANTXA       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;		// 0|0=GPIO  0|1=EPWM3B       0|2=MFSRA        0|3=OUTPUTXBAR3
	GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=CANRXA       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;		// 0|0=GPIO  0|1=EPWM4A       0|2=OUTPUTXBAR4  0|3=EPWMSYNCO
	GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 0;		// 1|0=GPIO  1|1=EQEP3A       1|2=CANTXB       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;		// 0|0=GPIO  0|1=EPWM4B       0|2=MCLKRA       0|3=OUTPUTXBAR5
	GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 0;		// 1|0=GPIO  1|1=EQEP3B       1|2=CANRXB       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;		// 0|0=GPIO  0|1=EPWM5A       0|2=CANTXB       0|3=ADCSOCAO
	GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 0;		// 1|0=GPIO  1|1=EQEP3S       1|2=SCITXDA      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;		// 0|0=GPIO  0|1=EPWM5B       0|2=SCITXDB      0|3=OUTPUTXBAR6
	GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 0;		// 1|0=GPIO  1|1=EQEP3I       1|2=SCIRXDA      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;		// 0|0=GPIO  0|1=EPWM6A       0|2=CANRXB       0|3=ADCSOCBO
	GpioCtrlRegs.GPAMUX1.bit.GPIO10  = 0;		// 1|0=GPIO  1|1=EQEP1A       1|2=SCITXDB      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-WAIT
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;		// 0|0=GPIO  0|1=EPWM6B       0|2=SCIRXDB      0|3=OUTPUTXBAR7
	GpioCtrlRegs.GPAMUX1.bit.GPIO11  = 0;		// 1|0=GPIO  1|1=EQEP1B       1|2=SCIRXDB      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-STRT
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;		// 0|0=GPIO  0|1=EPWM7A       0|2=CANTXB       0|3=MDXB
	GpioCtrlRegs.GPAMUX1.bit.GPIO12  = 0;		// 1|0=GPIO  1|1=EQEP1S       1|2=SCITXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-ENA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;		// 0|0=GPIO  0|1=EPWM7B       0|2=CANRXB       0|3=MDRB
	GpioCtrlRegs.GPAMUX1.bit.GPIO13  = 0;		// 1|0=GPIO  1|1=EQEP1I       1|2=SCIRXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-D7
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;		// 0|0=GPIO  0|1=EPWM8A       0|2=SCITXDB      0|3=MCLKXB
	GpioCtrlRegs.GPAMUX1.bit.GPIO14  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=OUTPUTXBAR3  1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-D6
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;		// 0|0=GPIO  0|1=EPWM8B       0|2=SCIRXDB      0|3=MFSXB
	GpioCtrlRegs.GPAMUX1.bit.GPIO15  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=OUTPUTXBAR4  1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-D5
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 0;		// 0|0=GPIO  0|1=SPISIMOA     0|2=CANTXB       0|3=OUTPUTXBAR7
	GpioCtrlRegs.GPAMUX2.bit.GPIO16  = 0;		// 1|0=GPIO  1|1=EPWM9A       1|2=rsvd         1|3=SD1_D1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-D4
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 0;		// 0|0=GPIO  0|1=SPISOMIA     0|2=CANRXB       0|3=OUTPUTXBAR8
	GpioCtrlRegs.GPAMUX2.bit.GPIO17  = 0;		// 1|0=GPIO  1|1=EPWM9B       1|2=rsvd         1|3=SD1_C1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-D3
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = 0;		// 0|0=GPIO  0|1=SPICLKA      0|2=SCITXDB      0|3=CANRXA
	GpioCtrlRegs.GPAMUX2.bit.GPIO18  = 0;		// 1|0=GPIO  1|1=EPWM10A      1|2=rsvd         1|3=SD1_D2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-D2
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 0;		// 0|0=GPIO  0|1=SPISTEAn     0|2=SCIRXDB      0|3=CANTXA
	GpioCtrlRegs.GPAMUX2.bit.GPIO19  = 0;		// 1|0=GPIO  1|1=EPWM10B      1|2=rsvd         1|3=SD1_C2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-D1
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0;		// 0|0=GPIO  0|1=EQEP1A       0|2=MDXA         0|3=CANTXB
	GpioCtrlRegs.GPAMUX2.bit.GPIO20  = 0;		// 1|0=GPIO  1|1=EPWM11A      1|2=rsvd         1|3=SD1_D3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-D0
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0;		// 0|0=GPIO  0|1=EQEP1B       0|2=MDRA         0|3=CANRXB
	GpioCtrlRegs.GPAMUX2.bit.GPIO21  = 0;		// 1|0=GPIO  1|1=EPWM11B      1|2=rsvd         1|3=SD1_C3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=UPP-CLK
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = 0;		// 0|0=GPIO  0|1=EQEP1S       0|2=MCLKXA       0|3=SCITXDB
	GpioCtrlRegs.GPAMUX2.bit.GPIO22  = 0;		// 1|0=GPIO  1|1=EPWM12A      1|2=SPICLKB      1|3=SD1_D4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 0;		// 0|0=GPIO  0|1=EQEP1I       0|2=MFSXA        0|3=SCIRXDB
	GpioCtrlRegs.GPAMUX2.bit.GPIO23  = 0;		// 1|0=GPIO  1|1=EPWM12B      1|2=SPISTEBn     1|3=SD1_C4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO24 = 0;		// 0|0=GPIO  0|1=OUTPUTXBAR1  0|2=EQEP2A       0|3=MDXB
	GpioCtrlRegs.GPAMUX2.bit.GPIO24  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPISIMOB     1|3=SD2_D1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = 0;		// 0|0=GPIO  0|1=OUTPUTXBAR2  0|2=EQEP2B       0|3=MDRB
	GpioCtrlRegs.GPAMUX2.bit.GPIO25  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPISOMIB     1|3=SD2_C1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0;		// 0|0=GPIO  0|1=OUTPUTXBAR3  0|2=EQEP2I       0|3=MCLKXB
	GpioCtrlRegs.GPAMUX2.bit.GPIO26  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR3  1|2=SPICLKB      1|3=SD2_D2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO27 = 0;		// 0|0=GPIO  0|1=OUTPUTXBAR4  0|2=EQEP2S       0|3=MFSXB
	GpioCtrlRegs.GPAMUX2.bit.GPIO27  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR4  1|2=SPISTEBn     1|3=SD2_C2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO28 = 0;		// 0|0=GPIO  0|1=SCIRXDA      0|2=EM1CS4n      0|3=rsvd
	GpioCtrlRegs.GPAMUX2.bit.GPIO28  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR5  1|2=EQEP3A       1|3=SD2_D3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO29 = 0;		// 0|0=GPIO  0|1=SCITXDA      0|2=EM1SDCKE     0|3=rsvd
	GpioCtrlRegs.GPAMUX2.bit.GPIO29  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR6  1|2=EQEP3B       1|3=SD2_C3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO30 = 0;		// 0|0=GPIO  0|1=CANRXA       0|2=EM1CLK       0|3=rsvd
	GpioCtrlRegs.GPAMUX2.bit.GPIO30  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR7  1|2=EQEP3S       1|3=SD2_D4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;		// 0|0=GPIO  0|1=CANTXA       0|2=EM1WEn       0|3=rsvd
	GpioCtrlRegs.GPAMUX2.bit.GPIO31  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR8  1|2=EQEP3I       1|3=SD2_C4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------

//--- Group B pins
	GpioCtrlRegs.GPBCTRL.all  = 0x00000000;		// QUALPRD = PLLSYSCLK for all group B GPIO
	GpioCtrlRegs.GPBQSEL1.all = 0x00000000;		// Synchronous qualification for all group B GPIO 32-47
	GpioCtrlRegs.GPBQSEL2.all = 0x00000000;		// Synchronous qualification for all group B GPIO 48-63
	GpioCtrlRegs.GPBDIR.all   = 0x00000000;		// All group B GPIO are inputs
	GpioCtrlRegs.GPBPUD.all   = 0x00000000;		// All group B pullups enabled
	GpioCtrlRegs.GPBINV.all   = 0x00000000;		// No inputs inverted
	GpioCtrlRegs.GPBODR.all   = 0x00000000;		// All outputs normal mode (no open-drain outputs)
	GpioCtrlRegs.GPBCSEL1.all = 0x00000000;		// GPIO 32-39 \.
	GpioCtrlRegs.GPBCSEL2.all = 0x00000000;		// GPIO 40-47  |. GPIODAT/SET/CLEAR/TOGGLE reg. master:
	GpioCtrlRegs.GPBCSEL3.all = 0x00000000;		// GPIO 48-55  |. 0=CPU1, 1=CPU1.CLA1, 2=CPU2, 3=CPU2.CLA1
	GpioCtrlRegs.GPBCSEL4.all = 0x00000000;		// GPIO 56-63 /.

	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO32 = 0;		// 0|0=GPIO  0|1=SDAA         0|2=EM1CS0n      0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO32  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO33 = 0;		// 0|0=GPIO  0|1=SCLA         0|2=EM1RNW       0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO33  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;		// 0|0=GPIO  0|1=OUTPUTXBAR1  0|2=EM1CS2n      0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO34  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SDAB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO35 = 0;		// 0|0=GPIO  0|1=SCIRXDA      0|2=EM1CS3n      0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO35  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCLB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO36 = 0;		// 0|0=GPIO  0|1=SCITXDA      0|2=EM1WAIT      0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO36  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=CANRXA       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO37 = 0;		// 0|0=GPIO  0|1=OUTPUTXBAR2  0|2=EM1OEn       0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO37  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=CANTXA       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO38 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A0        0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO38  = 0;		// 1|0=GPIO  1|1=SCITXDC      1|2=CANTXB       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO39 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A1        0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO39  = 0;		// 1|0=GPIO  1|1=SCIRXDC      1|2=CANRXB       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A2        0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO40  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SDAB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A3        0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO41  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCLB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBAMSEL.bit.GPIO42 = 0;		// 0=digital function, 1=USB0DM
	GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO42  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SDAA         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SCITXDA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBAMSEL.bit.GPIO43 = 0;		// 0=digital function, 1=USB0DP
	GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO43  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCLA         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SCIRXDA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO44 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A4        0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO44  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO45 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A5        0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO45  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO46 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A6        0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO46  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX1.bit.GPIO47 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A7        0|3=rsvd
	GpioCtrlRegs.GPBMUX1.bit.GPIO47  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO48 = 0;		// 0|0=GPIO  0|1=OUTPUTXBAR3  0|2=EM1A8        0|3=rsvd
	GpioCtrlRegs.GPBMUX2.bit.GPIO48  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDA      1|3=SD1_D1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO49 = 0;		// 0|0=GPIO  0|1=OUTPUTXBAR4  0|2=EM1A9        0|3=rsvd
	GpioCtrlRegs.GPBMUX2.bit.GPIO49  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDA      1|3=SD1_C1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO50 = 0;		// 0|0=GPIO  0|1=EQEP1A       0|2=EM1A10       0|3=rsvd
	GpioCtrlRegs.GPBMUX2.bit.GPIO50  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPISIMOC     1|3=SD1_D2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO51 = 0;		// 0|0=GPIO  0|1=EQEP1B       0|2=EM1A11       0|3=rsvd
	GpioCtrlRegs.GPBMUX2.bit.GPIO51  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPISOMIC     1|3=SD1_C2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO52 = 0;		// 0|0=GPIO  0|1=EQEP1S       0|2=EM1A12       0|3=rsvd
	GpioCtrlRegs.GPBMUX2.bit.GPIO52  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPICLKC      1|3=SD1_D3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO53 = 0;		// 0|0=GPIO  0|1=EQEP1I       0|2=EM1D31       0|3=EM2D15
	GpioCtrlRegs.GPBMUX2.bit.GPIO53  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPISTECn     1|3=SD1_C3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 0;		// 0|0=GPIO  0|1=SPISIMOA     0|2=EM1D30       0|3=EM2D14
	GpioCtrlRegs.GPBMUX2.bit.GPIO54  = 0;		// 1|0=GPIO  1|1=EQEP2A       1|2=SCITXDB      1|3=SD1_D4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 0;		// 0|0=GPIO  0|1=SPISOMIA     0|2=EM1D29       0|3=EM2D13
	GpioCtrlRegs.GPBMUX2.bit.GPIO55  = 0;		// 1|0=GPIO  1|1=EQEP2B       1|2=SCIRXDB      1|3=SD1_C4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO56 = 0;		// 0|0=GPIO  0|1=SPICLKA      0|2=EM1D28       0|3=EM2D12
	GpioCtrlRegs.GPBMUX2.bit.GPIO56  = 0;		// 1|0=GPIO  1|1=EQEP2S       1|2=SCITXDC      1|3=SD2_D1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO57 = 0;		// 0|0=GPIO  0|1=SPISTEAn     0|2=EM1D27       0|3=EM2D11
	GpioCtrlRegs.GPBMUX2.bit.GPIO57  = 0;		// 1|0=GPIO  1|1=EQEP2I       1|2=SCIRXDC      1|3=SD2_C1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = 0;		// 0|0=GPIO  0|1=MCLKRA       0|2=EM1D26       0|3=EM2D10
	GpioCtrlRegs.GPBMUX2.bit.GPIO58  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR1  1|2=SPICLKB      1|3=SD2_D2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISIMOA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 0;		// 0|0=GPIO  0|1=MFSRA        0|2=EM1D25       0|3=EM2D9
	GpioCtrlRegs.GPBMUX2.bit.GPIO59  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR2  1|2=SPISTEBn     1|3=SD2_C2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISOMIA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 0;		// 0|0=GPIO  0|1=MCLKRB       0|2=EM1D24       0|3=EM2D8
	GpioCtrlRegs.GPBMUX2.bit.GPIO60  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR3  1|2=SPISIMOB     1|3=SD2_D3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPICLKA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = 0;		// 0|0=GPIO  0|1=MFSRB        0|2=EM1D23       0|3=EM2D7
	GpioCtrlRegs.GPBMUX2.bit.GPIO61  = 0;		// 1|0=GPIO  1|1=OUTPUTXBAR4  1|2=SPISOMIB     1|3=SD2_C3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISTEAn
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO62 = 0;		// 0|0=GPIO  0|1=SCIRXDC      0|2=EM1D22       0|3=EM2D6
	GpioCtrlRegs.GPBMUX2.bit.GPIO62  = 0;		// 1|0=GPIO  1|1=EQEP3A       1|2=CANRXA       1|3=SD2_D4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = 0;		// 0|0=GPIO  0|1=SCITXDC      0|2=EM1D21       0|3=EM2D5
	GpioCtrlRegs.GPBMUX2.bit.GPIO63  = 0;		// 1|0=GPIO  1|1=EQEP3B       1|2=CANTXA       1|3=SD2_C4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISIMOB
	//-----------------------------------------------------------------------------------------------------

//--- Group C pins
	GpioCtrlRegs.GPCCTRL.all  = 0x00000000;		// QUALPRD = PLLSYSCLK for all group pinsC GPIO
	GpioCtrlRegs.GPCQSEL1.all = 0x00000000;		// Synchronous qualification for all group C GPIO 64-79
	GpioCtrlRegs.GPCQSEL2.all = 0x00000000;		// Synchronous qualification for all group C GPIO 80-95
	GpioCtrlRegs.GPCDIR.all   = 0x00000000;		// All GPIO are inputs
	GpioCtrlRegs.GPCPUD.all   = 0x00000000;		// All pullups enabled
	GpioCtrlRegs.GPCINV.all   = 0x00000000;		// No inputs inverted
	GpioCtrlRegs.GPCODR.all   = 0x00000000;		// All outputs normal mode (no open-drain outputs)
	GpioCtrlRegs.GPCCSEL1.all = 0x00000000;		// GPIO 64-71 \.
	GpioCtrlRegs.GPCCSEL2.all = 0x00000000;		// GPIO 72-79  |. GPIODAT/SET/CLEAR/TOGGLE reg. master:
	GpioCtrlRegs.GPCCSEL3.all = 0x00000000;		// GPIO 80-87  |. 0=CPU1, 1=CPU1.CLA1, 2=CPU2, 3=CPU2.CLA1
	GpioCtrlRegs.GPCCSEL4.all = 0x00000000;		// GPIO 88-95 /.

	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D20       0|3=EM2D4
	GpioCtrlRegs.GPCMUX1.bit.GPIO64  = 0;		// 1|0=GPIO  1|1=EQEP3S       1|2=SCIRXDA      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISOMIB
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D19       0|3=EM2D3
	GpioCtrlRegs.GPCMUX1.bit.GPIO65  = 0;		// 1|0=GPIO  1|1=EQEP3I       1|2=SCITXDA      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPICLKB
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D18       0|3=EM2D2
	GpioCtrlRegs.GPCMUX1.bit.GPIO66  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SDAB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISTEBn
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO67 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D17       0|3=EM2D1
	GpioCtrlRegs.GPCMUX1.bit.GPIO67  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO68 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D16       0|3=EM2D0
	GpioCtrlRegs.GPCMUX1.bit.GPIO68  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO69 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D15       0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO69  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCLB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISIMOC
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO70 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D14       0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO70  = 0;		// 1|0=GPIO  1|1=CANRXA       1|2=SCITXDB      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISOMIC
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO71 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D13       0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO71  = 0;		// 1|0=GPIO  1|1=CANTXA       1|2=SCIRXDB      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPICLKC
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO72 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D12       0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO72  = 0;		// 1|0=GPIO  1|1=CANTXB       1|2=SCITXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=SPISTECn
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO73 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D11       0|3=XCLKOUT
	GpioCtrlRegs.GPCMUX1.bit.GPIO73  = 0;		// 1|0=GPIO  1|1=CANRXB       1|2=SCIRXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO74 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D10       0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO74  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO75 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D9        0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO75  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO76 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D8        0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO76  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO77 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D7        0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO77  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO78 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D6        0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO78  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=EQEP2A       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX1.bit.GPIO79 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D5        0|3=rsvd
	GpioCtrlRegs.GPCMUX1.bit.GPIO79  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=EQEP2B       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO80 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D4        0|3=rsvd
	GpioCtrlRegs.GPCMUX2.bit.GPIO80  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=EQEP2S       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO81 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D3        0|3=rsvd
	GpioCtrlRegs.GPCMUX2.bit.GPIO81  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=EQEP2I       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO82 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D2        0|3=rsvd
	GpioCtrlRegs.GPCMUX2.bit.GPIO82  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO83 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D1        0|3=rsvd
	GpioCtrlRegs.GPCMUX2.bit.GPIO83  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPCMUX2.bit.GPIO84  = 0;		// 1|0=GPIO  1|1=SCITXDA      1|2=MDXB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=MDXA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1D0        0|3=rsvd
	GpioCtrlRegs.GPCMUX2.bit.GPIO85  = 0;		// 1|0=GPIO  1|1=SCIRXDA      1|2=MDRB         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=MDRA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO86 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A13       0|3=EM1CAS
	GpioCtrlRegs.GPCMUX2.bit.GPIO86  = 0;		// 1|0=GPIO  1|1=SCITXDB      1|2=MCLKXB       1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=MCLKXA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO87 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A14       0|3=EM1RAS
	GpioCtrlRegs.GPCMUX2.bit.GPIO87  = 0;		// 1|0=GPIO  1|1=SCIRXDB      1|2=MFSXB        1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=MFSXA
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO88 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A15       0|3=EM1DQM0
	GpioCtrlRegs.GPCMUX2.bit.GPIO88  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO89 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A16       0|3=EM1DQM1
	GpioCtrlRegs.GPCMUX2.bit.GPIO89  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO90 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A17       0|3=EM1DQM2
	GpioCtrlRegs.GPCMUX2.bit.GPIO90  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO91 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A18       0|3=EM1DQM3
	GpioCtrlRegs.GPCMUX2.bit.GPIO91  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SDAA         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO92 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=EM1A19       0|3=EM1BA1
	GpioCtrlRegs.GPCMUX2.bit.GPIO92  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCLA         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO93 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=Ersvd        0|3=EM1BA0
	GpioCtrlRegs.GPCMUX2.bit.GPIO93  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO94 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPCMUX2.bit.GPIO94  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPCGMUX2.bit.GPIO95 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPCMUX2.bit.GPIO95  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------

//--- Group D pins
	GpioCtrlRegs.GPDCTRL.all  = 0x00000000;		// QUALPRD = PLLSYSCLK for all group D GPIO
	GpioCtrlRegs.GPDQSEL1.all = 0x00000000;		// Synchronous qualification for all group D GPIO 96-111
	GpioCtrlRegs.GPDQSEL2.all = 0x00000000;		// Synchronous qualification for all group D GPIO 112-127
	GpioCtrlRegs.GPDDIR.all   = 0x00000000;		// All GPIO are inputs
	GpioCtrlRegs.GPDPUD.all   = 0x00000000;		// All pullups enabled
	GpioCtrlRegs.GPDINV.all   = 0x00000000;		// No inputs inverted
	GpioCtrlRegs.GPDODR.all   = 0x00000000;		// All outputs normal mode (no open-drain outputs)
	GpioCtrlRegs.GPDCSEL1.all = 0x00000000;		// GPIO 96-103  \.
	GpioCtrlRegs.GPDCSEL2.all = 0x00000000;		// GPIO 104-111  |. GPIODAT/SET/CLEAR/TOGGLE reg. master:
	GpioCtrlRegs.GPDCSEL3.all = 0x00000000;		// GPIO 112-119  |. 0=CPU1, 1=CPU1.CLA1, 2=CPU2, 3=CPU2.CLA1
	GpioCtrlRegs.GPDCSEL4.all = 0x00000000;		// GPIO 120-127 /.

	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO96 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2DQM1
	GpioCtrlRegs.GPDMUX1.bit.GPIO96  = 0;		// 1|0=GPIO  1|1=EQEP1A       1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO97 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2DQM0
	GpioCtrlRegs.GPDMUX1.bit.GPIO97  = 0;		// 1|0=GPIO  1|1=EQEP1B       1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO98 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A0
	GpioCtrlRegs.GPDMUX1.bit.GPIO98  = 0;		// 1|0=GPIO  1|1=EQEP1S       1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO99 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A1
	GpioCtrlRegs.GPDMUX1.bit.GPIO99  = 0;		// 1|0=GPIO  1|1=EQEP1I       1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO100 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A2
	GpioCtrlRegs.GPDMUX1.bit.GPIO100  = 0;		// 1|0=GPIO  1|1=EQEP2A       1|2=SPISIMOC     1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO101 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A3
	GpioCtrlRegs.GPDMUX1.bit.GPIO101  = 0;		// 1|0=GPIO  1|1=EQEP2B       1|2=SPISOMIC     1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO102 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A4
	GpioCtrlRegs.GPDMUX1.bit.GPIO102  = 0;		// 1|0=GPIO  1|1=EQEP2S       1|2=SPICLKC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO103 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A5
	GpioCtrlRegs.GPDMUX1.bit.GPIO103  = 0;		// 1|0=GPIO  1|1=EQEP2I       1|2=SPISTECn     1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO104 = 0;		// 0|0=GPIO  0|1=SDAA         0|2=rsvd         0|3=EM2A6
	GpioCtrlRegs.GPDMUX1.bit.GPIO104  = 0;		// 1|0=GPIO  1|1=EQEP3A       1|2=SCITXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO105 = 0;		// 0|0=GPIO  0|1=SCLA         0|2=rsvd         0|3=EM2A7
	GpioCtrlRegs.GPDMUX1.bit.GPIO105  = 0;		// 1|0=GPIO  1|1=EQEP3B       1|2=SCIRXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO106 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A8
	GpioCtrlRegs.GPDMUX1.bit.GPIO106  = 0;		// 1|0=GPIO  1|1=EQEP3S       1|2=SCITXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO107 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A9
	GpioCtrlRegs.GPDMUX1.bit.GPIO107  = 0;		// 1|0=GPIO  1|1=EQEP3I       1|2=SCIRXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO108 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A10
	GpioCtrlRegs.GPDMUX1.bit.GPIO108  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO109 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2A11
	GpioCtrlRegs.GPDMUX1.bit.GPIO109  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO110 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2WAIT
	GpioCtrlRegs.GPDMUX1.bit.GPIO110  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX1.bit.GPIO111 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2BA0
	GpioCtrlRegs.GPDMUX1.bit.GPIO111  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO112 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2BA1
	GpioCtrlRegs.GPDMUX2.bit.GPIO112  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO113 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2CAS
	GpioCtrlRegs.GPDMUX2.bit.GPIO113  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO114 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2RAS
	GpioCtrlRegs.GPDMUX2.bit.GPIO114  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO115 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2CS0n
	GpioCtrlRegs.GPDMUX2.bit.GPIO115  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO116 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2CS2n
	GpioCtrlRegs.GPDMUX2.bit.GPIO116  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO117 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2SDCKE
	GpioCtrlRegs.GPDMUX2.bit.GPIO117  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO118 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2CLK
	GpioCtrlRegs.GPDMUX2.bit.GPIO118  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO119 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2RNW
	GpioCtrlRegs.GPDMUX2.bit.GPIO119  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO120 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2WEn
	GpioCtrlRegs.GPDMUX2.bit.GPIO120  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=USB0PFLT
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO121 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=EM2OEn
	GpioCtrlRegs.GPDMUX2.bit.GPIO121  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=USB0EPEN
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO122 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPDMUX2.bit.GPIO122  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPISIMOC     1|3=SD1_D1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO123 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPDMUX2.bit.GPIO123  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPISOMIC     1|3=SD1_C1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO124 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPDMUX2.bit.GPIO124  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPICLKC      1|3=SD1_D2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO125 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPDMUX2.bit.GPIO125  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SPISTECn     1|3=SD1_C2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO126 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPDMUX2.bit.GPIO126  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD1_D3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPDGMUX2.bit.GPIO127 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPDMUX2.bit.GPIO127  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD1_C3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------

//--- Group E pins
	GpioCtrlRegs.GPECTRL.all  = 0x00000000;		// QUALPRD = PLLSYSCLK for all group E GPIO
	GpioCtrlRegs.GPEQSEL1.all = 0x00000000;		// Synchronous qualification for all group E GPIO 128-143
	GpioCtrlRegs.GPEQSEL2.all = 0x00000000;		// Synchronous qualification for all group E GPIO 144-159
	GpioCtrlRegs.GPEDIR.all   = 0x00000000;		// All GPIO are inputs
	GpioCtrlRegs.GPEPUD.all   = 0x00000000;		// All pullups enabled
	GpioCtrlRegs.GPEINV.all   = 0x00000000;		// No inputs inverted
	GpioCtrlRegs.GPEODR.all   = 0x00000000;		// All outputs normal mode (no open-drain outputs)
	GpioCtrlRegs.GPECSEL1.all = 0x00000000;		// GPIO 128-135 \.
	GpioCtrlRegs.GPECSEL2.all = 0x00000000;		// GPIO 136-143  |. GPIODAT/SET/CLEAR/TOGGLE reg. master:
	GpioCtrlRegs.GPECSEL3.all = 0x00000000;		// GPIO 144-151  |. 0=CPU1, 1=CPU1.CLA1, 2=CPU2, 3=CPU2.CLA1
	GpioCtrlRegs.GPECSEL4.all = 0x00000000;		// GPIO 152-159 /.

	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO128 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO128  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD1_D4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO129 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO129  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD1_C4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO130 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO130  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD2_D1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO131 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO131  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD2_C1
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO132 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO132  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD2_D2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO133 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO133  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD2_C2
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO134 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO134  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=SD2_D3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO135 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO135  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDA      1|3=SD2_C3
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO136 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO136  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDA      1|3=SD2_D4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO137 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO137  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDB      1|3=SD2_C4
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO138 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO138  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDB      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO139 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO139  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO140 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO140  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDC      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO141 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO141  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCIRXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO142 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO142  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=SCITXDD      1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX1.bit.GPIO143 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX1.bit.GPIO143  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO144 = 0;		// 0|0=GPIO  0|1=rsvd         0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO144  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO145 = 0;		// 0|0=GPIO  0|1=EPWM1A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO145  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO146 = 0;		// 0|0=GPIO  0|1=EPWM1B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO146  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO147 = 0;		// 0|0=GPIO  0|1=EPWM2A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO147  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO148 = 0;		// 0|0=GPIO  0|1=EPWM2B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO148  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO149 = 0;		// 0|0=GPIO  0|1=EPWM3A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO149  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO150 = 0;		// 0|0=GPIO  0|1=EPWM3B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO150  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO151 = 0;		// 0|0=GPIO  0|1=EPWM4A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO151  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO152 = 0;		// 0|0=GPIO  0|1=EPWM4B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO152  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO153 = 0;		// 0|0=GPIO  0|1=EPWM5A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO153  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO154 = 0;		// 0|0=GPIO  0|1=EPWM5B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO154  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO155 = 0;		// 0|0=GPIO  0|1=EPWM6AA      0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO155  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO156 = 0;		// 0|0=GPIO  0|1=EPWM6B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO156  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO157 = 0;		// 0|0=GPIO  0|1=EPWM7A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO157  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO158 = 0;		// 0|0=GPIO  0|1=EPWM7B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO158  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPEGMUX2.bit.GPIO159 = 0;		// 0|0=GPIO  0|1=EPWM8A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPEMUX2.bit.GPIO159  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------

//--- Group F pins
	GpioCtrlRegs.GPFCTRL.all  = 0x00000000;		// QUALPRD = PLLSYSCLK for all group F GPIO
	GpioCtrlRegs.GPFQSEL1.all = 0x00000000;		// Synchronous qualification for all group F GPIO 160-168
	GpioCtrlRegs.GPFDIR.all   = 0x00000000;		// All GPIO are inputs
	GpioCtrlRegs.GPFPUD.all   = 0x00000000;		// All pullups enabled
	GpioCtrlRegs.GPFINV.all   = 0x00000000;		// No inputs inverted
	GpioCtrlRegs.GPFODR.all   = 0x00000000;		// All outputs normal mode (no open-drain outputs)
	GpioCtrlRegs.GPFCSEL1.all = 0x00000000;		// GPIO 160-167 \. GPIODAT/SET/CLEAR/TOGGLE reg. master:
	GpioCtrlRegs.GPFCSEL2.all = 0x00000000;		// GPIO 168     /. 0=CPU1, 1=CPU1.CLA1, 2=CPU2, 3=CPU2.CLA1

	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO160 = 0;		// 0|0=GPIO  0|1=EPWM8B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO160  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO161 = 0;		// 0|0=GPIO  0|1=EPWM9A       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO161  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO162 = 0;		// 0|0=GPIO  0|1=EPWM9B       0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO162  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO163 = 0;		// 0|0=GPIO  0|1=EPWM10A      0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO163  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO164 = 0;		// 0|0=GPIO  0|1=EPWM10B      0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO164  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO165 = 0;		// 0|0=GPIO  0|1=EPWM11A      0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO165  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO166 = 0;		// 0|0=GPIO  0|1=EPWM11B      0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO166  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO167 = 0;		// 0|0=GPIO  0|1=EPWM12A      0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO167  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------
	GpioCtrlRegs.GPFGMUX1.bit.GPIO168 = 0;		// 0|0=GPIO  0|1=EPWM12B      0|2=rsvd         0|3=rsvd
	GpioCtrlRegs.GPFMUX1.bit.GPIO168  = 0;		// 1|0=GPIO  1|1=rsvd         1|2=rsvd         1|3=rsvd
												// 2|0=GPIO  2|1=rsvd         2|2=rsvd         2|3=rsvd
												// 3|0=GPIO  3|1=rsvd         3|2=rsvd         3|3=rsvd
	//-----------------------------------------------------------------------------------------------------

//--- Select pin configurations
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;			// GPIO34 is an output (connected to LED)
	GpioDataRegs.GPBSET.bit.GPIO34 = 1;			// GPIO34 pin is set to 1 (turn LED on)
	GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;			// GPIO19 is an output (used in Lab 6)
	GpioDataRegs.GPASET.bit.GPIO19 = 1;			// GPIO19 pin is set to 1 (used in Lab 6 - pin high)
	GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;			// GPIO18 is an output (used in Lab 6 - pin toggle)
    GpioCtrlRegs.GPACSEL4.bit.GPIO31 = 2;       // GPIO31 is controlled by CPU2 (used in Lab 11)
	GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;			// GPIO31 is an output (used in Lab 11 - connected to LED)
	GpioDataRegs.GPASET.bit.GPIO31 = 1;			// GPIO31 pin is set to 1 (used in Lab 11 - turn LED on)

//--- Enable the register locks for all ports
	GpioCtrlRegs.GPALOCK.all = 0xFFFFFFFF;
	GpioCtrlRegs.GPBLOCK.all = 0xFFFFFFFF;
	GpioCtrlRegs.GPCLOCK.all = 0xFFFFFFFF;
	GpioCtrlRegs.GPDLOCK.all = 0xFFFFFFFF;
	GpioCtrlRegs.GPELOCK.all = 0xFFFFFFFF;
	GpioCtrlRegs.GPFLOCK.all = 0xFFFFFFFF;

//--- Lock the GPxLOCK register bits for all ports
	GpioCtrlRegs.GPACR.all   = 0xFFFFFFFF;
	GpioCtrlRegs.GPBCR.all   = 0xFFFFFFFF;
	GpioCtrlRegs.GPCCR.all   = 0xFFFFFFFF;
	GpioCtrlRegs.GPDCR.all   = 0xFFFFFFFF;
	GpioCtrlRegs.GPECR.all   = 0xFFFFFFFF;
	GpioCtrlRegs.GPFCR.all   = 0xFFFFFFFF;

//--- Finish up
		asm(" EDIS");								// Disable EALLOW protected register access

} // end InitGpio()


//--- end of file -----------------------------------------------------
