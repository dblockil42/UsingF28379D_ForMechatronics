/**********************************************************************
* File: DefaultIsr_5.c
* Devices: TMS320F2837xD
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"						// Main include file


//---------------------------------------------------------------------
interrupt void TIMER1_ISR(void)						// 0x000D1A  CPU Timer1
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void TIMER2_ISR(void)						// 0x000D1C  CPU Timer2
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DATALOG_ISR(void)					// 0x000D1E  CPU data logging interrupt
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void RTOS_ISR(void)						// 0x000D20  CPU RTOS interrupt
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EMU_ISR(void) 						// 0x000D22  CPU emulation interrupt
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void NMI_ISR(void)						// 0x000D24  XNMI interrupt
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ILLEGAL_ISR(void)					// 0x000D26  Illegal operation trap
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER1_ISR(void)						// 0x000D28  Software interrupt #1
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER2_ISR(void)						// 0x000D2A  Software interrupt #2
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER3_ISR(void)						// 0x000D2C  Software interrupt #3
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER4_ISR(void)						// 0x000D2E  Software interrupt #4
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER5_ISR(void)						// 0x000D30  Software interrupt #5
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER6_ISR(void)						// 0x000D32  Software interrupt #6
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER7_ISR(void)						// 0x000D34  Software interrupt #7
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER8_ISR(void)						// 0x000D36  Software interrupt #8
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER9_ISR(void)						// 0x000D38  Software interrupt #9
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER10_ISR(void)						// 0x000D3A  Software interrupt #10
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER11_ISR(void)						// 0x000D3C  Software interrupt #11
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void USER12_ISR(void)						// 0x000D3E  Software interrupt #12
{
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}


//=====================================================================
// ISRs for PIE vectors x.1 - x.8
//=====================================================================

//---------------------------------------------------------------------
interrupt void ADCA1_ISR(void)						// PIE1.1 @ 0x000D40  ADC-A interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCB1_ISR(void)						// PIE1.2 @ 0x000D42  ADC-B interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCC1_ISR(void)						// PIE1.3 @ 0x000D44  ADC-C interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void XINT1_ISR(void)						// PIE1.4 @ 0x000D46  External interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void XINT2_ISR(void)						// PIE1.5 @ 0x000D48  External interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCD1_ISR(void)						// PIE1.6 @ 0x000D4A  ADC-D interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void TIMER0_ISR(void)						// PIE1.7 @ 0x000D4C  CPU TIMER 0 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void WAKE_ISR(void)						// PIE1.8 @ 0x000D4E  WAKE interrupt (LPM/WD)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM1_TZ_ISR(void)					// PIE2.1 @ 0x000D50  ePWM1 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM2_TZ_ISR(void)					// PIE2.2 @ 0x000D52  ePWM2 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM3_TZ_ISR(void)					// PIE2.3 @ 0x000D54  ePWM3 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM4_TZ_ISR(void)					// PIE2.4 @ 0x000D56  ePWM4 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM5_TZ_ISR(void)					// PIE2.5 @ 0x000D58  ePWM5 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM6_TZ_ISR(void)					// PIE2.6 @ 0x000D5A  ePWM6 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM7_TZ_ISR(void)					// PIE2.7 @ 0x000D5C  ePWM7 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM8_TZ_ISR(void)					// PIE2.8 @ 0x000D5E  ePWM8 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM1_ISR(void)						// PIE3.1 @ 0x000D60  ePWM1 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM2_ISR(void)						// PIE3.2 @ 0x000D62  ePWM2 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM3_ISR(void)						// PIE3.3 @ 0x000D64  ePWM3 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM4_ISR(void)						// PIE3.4 @ 0x000D66  ePWM4 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM5_ISR(void)						// PIE3.5 @ 0x000D68  ePWM5 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM6_ISR(void)						// PIE3.6 @ 0x000D6A  ePWM6 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM7_ISR(void)						// PIE3.7 @ 0x000D6C  ePWM7 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM8_ISR(void)						// PIE3.8 @ 0x000D6E  ePWM8 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ECAP1_ISR(void)						// PIE4.1 @ 0x000D70  eCAP1 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ECAP2_ISR(void)						// PIE4.2 @ 0x000D72  eCAP2 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ECAP3_ISR(void)						// PIE4.3 @ 0x000D74  eCAP3 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ECAP4_ISR(void)						// PIE4.4 @ 0x000D76  eCAP4 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ECAP5_ISR(void)						// PIE4.5 @ 0x000D78  eCAP5 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ECAP6_ISR(void)						// PIE4.6 @ 0x000D7A  eCAP6 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE4.7 @ 0x000D7C reserved
													// PIE4.8 @ 0x000D7E reserved

//---------------------------------------------------------------------
interrupt void EQEP1_ISR(void)						// PIE5.1 @ 0x000D80  eQEP1 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;			// Must acknowledge the PIE group
 
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EQEP2_ISR(void)						// PIE5.2 @ 0x000D82  eQEP2 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EQEP3_ISR(void)						// PIE5.3 @ 0x000D84  eQEP3 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE5.4 @ 0x000D86 reserved
													// PIE5.5 @ 0x000D88 reserved
													// PIE5.6 @ 0x000D8A reserved
													// PIE5.7 @ 0x000D8C reserved
													// PIE5.8 @ 0x000D8E reserved

//---------------------------------------------------------------------
interrupt void SPIA_RX_ISR(void)					// PIE6.1 @ 0x000D90  SPI-A RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SPIA_TX_ISR(void)					// PIE6.2 @ 0x000D92  SPI-A TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SPIB_RX_ISR(void)					// PIE6.3 @ 0x000D94  SPI-B RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SPIB_TX_ISR(void)					// PIE6.4 @ 0x000D96  SPI-B TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void MCBSPA_RX_ISR(void)					// PIE6.5 @ 0x000D98  McBSP-A RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void MCBSPA_TX_ISR(void)					// PIE6.6 @ 0x000D9A  McBSP-A TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void MCBSPB_RX_ISR(void)					// PIE6.7 @ 0x000D9C  McBSP-B RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void MCBSPB_TX_ISR(void)					// PIE6.8 @ 0x000D9E  McBSP-B TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DMA_CH1_ISR(void)					// PIE7.1 @ 0x000DA0  DMA channel 1 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DMA_CH2_ISR(void)					// PIE7.2 @ 0x000DA2  DMA channel 2 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DMA_CH3_ISR(void)					// PIE7.3 @ 0x000DA4  DMA channel 3 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DMA_CH4_ISR(void)					// PIE7.4 @ 0x000DA6  DMA channel 4 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DMA_CH5_ISR(void)					// PIE7.5 @ 0x000DA8  DMA channel 5 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DMA_CH6_ISR(void)					// PIE7.6 @ 0x000DAA  DMA channel 6 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE7.7 @ 0x000DAC reserved
													// PIE7.8 @ 0x000DAE reserved

//---------------------------------------------------------------------
interrupt void I2CA_ISR(void)						// PIE8.1 @ 0x000DB0  I2C-A RX interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void I2CA_FIFO_ISR(void)					// PIE8.2 @ 0x000DB2  I2C-A RX interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------

interrupt void I2CB_ISR(void)						// PIE8.3 @ 0x000DB4  I2C-B RX interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void I2CB_FIFO_ISR(void)					// PIE8.4 @ 0x000DB6  I2C-B RX interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SCIC_RX_ISR(void)					// PIE8.5 @ 0x000DB8  SCI-C RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SCIC_TX_ISR(void)					// PIE8.6 @ 0x000DBA  SCI-C TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SCID_RX_ISR(void)					// PIE8.7 @ 0x000DBC  SCI-D RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SCID_TX_ISR(void)					// PIE8.8 @ 0x000DBE  SCI-D TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SCIA_RX_ISR(void)					// PIE9.1 @ 0x000DC0  SCI-A RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SCIA_TX_ISR(void)					// PIE9.2 @ 0x000DC2  SCI-A TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SCIB_RX_ISR(void)					// PIE9.3 @ 0x000DC4  SCI-B RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SCIB_TX_ISR(void)					// PIE9.4 @ 0x000DC6  SCI-B TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CANA0_ISR(void)	 					// PIE9.5 @ 0x000DC8  CAN-A interrupt #0
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CANA1_ISR(void)	 					// PIE9.6 @ 0x000DCA  CAN-A interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CANB0_ISR(void)	 					// PIE9.7 @ 0x000DCC  CAN-B interrupt #0
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CANB1_ISR(void)	 					// PIE9.8 @ 0x000DCE  CAN-B interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCA_EVT_ISR(void)					// PIE10.1 @ 0x000DD0 ADC-A event interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCA2_ISR(void)						// PIE10.2 @ 0x000DD2 ADC-A interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCA3_ISR(void)						// PIE10.3 @ 0x000DD4 ADC-A interrupt #3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCA4_ISR(void)						// PIE10.4 @ 0x000DD6 ADC-A interrupt #4
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCB_EVT_ISR(void)					// PIE10.5 @ 0x000DD8 ADC-B event interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCB2_ISR(void)						// PIE10.6 @ 0x000DDA ADC-B interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCB3_ISR(void)						// PIE10.7 @ 0x000DDC ADC-B interrupt #3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCB4_ISR(void)						// PIE10.8 @ 0x000DDE ADC-B interrupt #4
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA1_1_ISR(void)						// PIE11.1 @ 0x000DE0 CLA1 interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA1_2_ISR(void)						// PIE11.2 @ 0x000DE2 CLA1 interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA1_3_ISR(void)						// PIE11.3 @ 0x000DE4 CLA1 interrupt #3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA1_4_ISR(void)						// PIE11.4 @ 0x000DE6 CLA1 interrupt #4
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA1_5_ISR(void)						// PIE11.5 @ 0x000DE8 CLA1 interrupt #5
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA1_6_ISR(void)						// PIE11.6 @ 0x000DEA CLA1 interrupt #6
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA1_7_ISR(void)						// PIE11.7 @ 0x000DEC CLA1 interrupt #7
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA1_8_ISR(void)						// PIE11.8 @ 0x000DEE CLA1 interrupt #8
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void XINT3_ISR(void)						// PIE12.1 @ 0x000DF0  External interrupt #3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void XINT4_ISR(void)						// PIE12.2 @ 0x000DF2  External interrupt #4
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void XINT5_ISR(void)						// PIE12.3 @ 0x000DF4  External interrupt #5
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
													// PIE12.4 @ 0x000DF6  reserved
													// PIE12.5 @ 0x000DF8  reserved

//---------------------------------------------------------------------
interrupt void VCU_ISR(void)						// PIE12.6 @ 0x000DFA  VCU interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void FPU_OVERFLOW_ISR(void)				// PIE12.7 @ 0x000DFC  FPU overflow interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void FPU_UNDERFLOW_ISR(void)				// PIE12.8 @ 0x000DFE  FPU underflow interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}


//=====================================================================
// ISRs for PIE vectors x.9 - x.16
//=====================================================================

//---------------------------------------------------------------------
													// PIE1.9  @ 0x000E00  reserved
													// PIE1.10 @ 0x000E02  reserved
													// PIE1.11 @ 0x000E04  reserved
													// PIE1.12 @ 0x000E06  reserved

//---------------------------------------------------------------------
interrupt void IPC0_ISR(void)						// PIE1.13 @ 0x000E08  Inter-processor communication interrupt #0
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void IPC1_ISR(void)						// PIE1.14 @ 0x000E0A Inter-processor communication interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void IPC2_ISR(void)						// PIE1.15 @ 0x000E0C  Inter-processor communication interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void IPC3_ISR(void)						// PIE1.16 @ 0x000E0E  Inter-processor communication interrupt #3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void EPWM9_TZ_ISR(void)					// PIE2.9 @ 0x000E10  ePWM9 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM10_TZ_ISR(void)					// PIE2.10 @ 0x000E12  ePWM10 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM11_TZ_ISR(void)					// PIE2.11 @ 0x000E14  ePWM11 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM12_TZ_ISR(void)					// PIE2.12 @ 0x000E16  ePWM12 Trip Zone interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;			// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE2.13 @ 0x000E18  reserved
													// PIE2.14 @ 0x000E1A  reserved
													// PIE2.15 @ 0x000E1C  reserved
													// PIE2.16 @ 0x000E1E  reserved

//---------------------------------------------------------------------
interrupt void EPWM9_ISR(void)						// PIE3.9 @ 0x000E20  ePWM9 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM10_ISR(void)						// PIE3.10 @ 0x000E22  ePWM10 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM11_ISR(void)						// PIE3.11 @ 0x000E24  ePWM11 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void EPWM12_ISR(void)						// PIE3.12 @ 0x000E26  ePWM12 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;			// Must acknowledge the PIE group
  
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE3.13 @ 0x000E28  reserved
													// PIE3.14 @ 0x000E2A  reserved
													// PIE3.15 @ 0x000E2C  reserved
													// PIE3.16 @ 0x000E2E  reserved

//---------------------------------------------------------------------
													// PIE4.9  @ 0x000E30 reserved
													// PIE4.10 @ 0x000E32 reserved
													// PIE4.11 @ 0x000E34 reserved
													// PIE4.12 @ 0x000E36 reserved
													// PIE4.13 @ 0x000E38 reserved
													// PIE4.14 @ 0x000E3A reserved
													// PIE4.15 @ 0x000E3C reserved
													// PIE4.16 @ 0x000E3E reserved

//---------------------------------------------------------------------
interrupt void SD1_ISR(void)						// PIE5.9 @ 0x000E40  Sigma-Delta 1 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;			// Must acknowledge the PIE group
 
// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SD2_ISR(void)						// PIE5.10 @ 0x000E42  Sigma-Delta 2 interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE5.11 @ 0x000E44 reserved
													// PIE5.12 @ 0x000E46 reserved
													// PIE5.13 @ 0x000E48 reserved
													// PIE5.14 @ 0x000E4A reserved
													// PIE5.15 @ 0x000E4C reserved
													// PIE5.16 @ 0x000E4E reserved

//---------------------------------------------------------------------
interrupt void SPIC_RX_ISR(void)					// PIE6.9 @ 0x000E50  SPI-C RX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void SPIC_TX_ISR(void)					// PIE6.10 @ 0x000E52  SPI-C TX interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE6.11 @ 0x000E54 reserved
													// PIE6.12 @ 0x000E56 reserved
													// PIE6.13 @ 0x000E58 reserved
													// PIE6.14 @ 0x000E5A reserved
													// PIE6.15 @ 0x000E5C reserved
													// PIE6.16 @ 0x000E5E reserved

//---------------------------------------------------------------------
													// PIE7.9  @ 0x000E60 reserved
													// PIE7.10 @ 0x000E62 reserved
													// PIE7.11 @ 0x000E64 reserved
													// PIE7.12 @ 0x000E66 reserved
													// PIE7.13 @ 0x000E68 reserved
													// PIE7.14 @ 0x000E6A reserved
													// PIE7.15 @ 0x000E6C reserved
													// PIE7.16 @ 0x000E6E reserved

//---------------------------------------------------------------------
													// PIE8.9  @ 0x000E70 reserved
													// PIE8.10 @ 0x000E72 reserved
													// PIE8.11 @ 0x000E74 reserved
													// PIE8.12 @ 0x000E76 reserved
													// PIE8.13 @ 0x000E78 reserved
													// PIE8.14 @ 0x000E7A reserved

//---------------------------------------------------------------------
#ifdef CPU1
interrupt void UPPA_ISR(void)						// PIE8.15 @ 0x000E7C  uPPA interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}
#endif
								
//---------------------------------------------------------------------
													// PIE8.16 @ 0x000E7E reserved

//---------------------------------------------------------------------
interrupt void DCANC_1_ISR(void)					// PIE9.9 @ 0x000E80  DCAN-C interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DCANC_2_ISR(void)					// PIE9.10 @ 0x000E82  DCAN-C interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DCAND_1_ISR(void)					// PIE9.11 @ 0x000E84  DCAN-D interrupt #1
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void DCAND_2_ISR(void)					// PIE9.12 @ 0x000E86  DCAN-D interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE9.13 @ 0x000E88 reserved
													// PIE9.14 @ 0x000E8A reserved

//---------------------------------------------------------------------
#ifdef CPU1
interrupt void USBA_ISR(void)						// PIE9.15 @ 0x000E8C  USB interrupt A
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;			// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}
#endif

//---------------------------------------------------------------------
													// PIE9.16 @ 0x000E8E reserved

//---------------------------------------------------------------------
interrupt void ADCC_EVT_ISR(void)					// PIE10.9 @ 0x000E90 ADC-C event interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCC2_ISR(void)						// PIE10.10 @ 0x000E92 ADC-C interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCC3_ISR(void)						// PIE10.11 @ 0x000E94 ADC-C interrupt #3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCC4_ISR(void)						// PIE10.12 @ 0x000E96 ADC-C interrupt #4
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCD_EVT_ISR(void)					// PIE10.13 @ 0x000E98 ADC-D event interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCD2_ISR(void)						// PIE10.14 @ 0x000E9A ADC-D interrupt #2
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCD3_ISR(void)						// PIE10.15 @ 0x000E9C ADC-D interrupt #3
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void ADCD4_ISR(void)						// PIE10.16 @ 0x000E9E ADC-D interrupt #4
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
													// PIE11.9  @ 0x000EA0 reserved
													// PIE11.10 @ 0x000EA2 reserved
													// PIE11.11 @ 0x000EA4 reserved
													// PIE11.12 @ 0x000EA6 reserved
													// PIE11.13 @ 0x000EA8 reserved
													// PIE11.14 @ 0x000EAA reserved
													// PIE11.15 @ 0x000EAC reserved
													// PIE11.16 @ 0x000EAE reserved

//---------------------------------------------------------------------
interrupt void EMIF_ERROR_ISR(void)					// PIE12.9 @ 0x000EB0  EMIF error interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void RAM_CORRECTABLE_ERROR_ISR(void)  	// PIE12.10 @ 0x000EB2  RAM correctable error interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void FLASH_CORRECTABLE_ERROR_ISR(void)	// PIE12.11 @ 0x000EB4  Flash correctable error interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void RAM_ACCESS_VIOLATION_ISR(void)		// PIE12.12 @ 0x000EB6  RAM access violation interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void SYS_PLL_SLIP_ISR(void)				// PIE12.13 @ 0x000EB8  System PLL slip interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void AUX_PLL_SLIP_ISR(void)				// PIE12.14 @ 0x000EBA  Auxiliary PLL slip  interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void CLA_OVERFLOW_ISR(void)				// PIE12.15 @ 0x000EBC  CLA overflow interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}     

//---------------------------------------------------------------------
interrupt void CLA_UNDERFLOW_ISR(void)				// PIE12.16 @ 0x000EBE  CLA underflow interrupt
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;		// Must acknowledge the PIE group

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}

//---------------------------------------------------------------------
interrupt void PIE_RESERVED_ISR(void)				// Reserved PIE vectors
{
// This ISR is for reserved PIE vectors.  It should never be reached by
// properly executing code.  If you get here, it means something is wrong.

// Next two lines for debug only - remove after inserting your ISR
	asm (" ESTOP0");								// Emulator Halt instruction
	while(1);
}


//--- end of file -----------------------------------------------------
