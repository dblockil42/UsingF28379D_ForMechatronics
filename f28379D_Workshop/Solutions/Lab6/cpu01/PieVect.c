/**********************************************************************
* File: PieVect.c
* Devices: TMS320F2837xD
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"						// Main include file


/**********************************************************************
* The PIE vector initialization table for the F2837xD
**********************************************************************/
const struct PIE_VECT_TABLE PieVectTableInit = {

//--- Base vectors
    PIE_RESERVED_ISR,               // 0x000D00  reserved
    PIE_RESERVED_ISR,               // 0x000D02  reserved
    PIE_RESERVED_ISR,               // 0x000D04  reserved
    PIE_RESERVED_ISR,               // 0x000D06  reserved
    PIE_RESERVED_ISR,               // 0x000D08  reserved
    PIE_RESERVED_ISR,               // 0x000D0A  reserved
    PIE_RESERVED_ISR,               // 0x000D0C  reserved
    PIE_RESERVED_ISR,               // 0x000D0E  reserved
    PIE_RESERVED_ISR,               // 0x000D10  reserved
    PIE_RESERVED_ISR,               // 0x000D12  reserved
    PIE_RESERVED_ISR,               // 0x000D14  reserved
    PIE_RESERVED_ISR,               // 0x000D16  reserved
    PIE_RESERVED_ISR,               // 0x000D18  reserved
    TIMER1_ISR,                     // 0x000D1A  CPU Timer1 interrupt
    TIMER2_ISR,                     // 0x000D1C  CPU Timer2 interrupt
    DATALOG_ISR,                    // 0x000D1E  CPU data logging interrupt
    RTOS_ISR,                       // 0x000D20  CPU RTOS interrupt
    EMU_ISR,                        // 0x000D22  CPU emulation interrupt
    NMI_ISR,                        // 0x000D24  NMI - XNMI interrupt
    ILLEGAL_ISR,                    // 0x000D26  ILLEGAL - illegal operation trap
    USER1_ISR,                      // 0x000D28  USER1 - software interrupt #1
    USER2_ISR,                      // 0x000D2A  USER2 - software interrupt #2
    USER3_ISR,                      // 0x000D2C  USER3 - software interrupt #3
    USER4_ISR,                      // 0x000D2E  USER4 - software interrupt #4
    USER5_ISR,                      // 0x000D30  USER5 - software interrupt #5
    USER6_ISR,                      // 0x000D32  USER6 - software interrupt #6
    USER7_ISR,                      // 0x000D34  USER7 - software interrupt #7
    USER8_ISR,                      // 0x000D36  USER8 - software interrupt #8
    USER9_ISR,                      // 0x000D38  USER9 - software interrupt #9
    USER10_ISR,                     // 0x000D3A  USER10 - software interrupt #10
    USER11_ISR,                     // 0x000D3C  USER11 - software interrupt #11
    USER12_ISR,                     // 0x000D3E  USER12 - software interrupt #12

//--- Core interrupt #1 re-map, PIE 1.1 - 1.8
    ADCA1_ISR,                      // 0x000D40  ADC-A interrupt #1
    ADCB1_ISR,                      // 0x000D40  ADC-B interrupt #1
    ADCC1_ISR,                      // 0x000D40  ADC-C interrupt #1
    XINT1_ISR,                      // 0x000D46  External interrupt #1
    XINT2_ISR,                      // 0x000D48  External interrupt #2
    ADCD1_ISR,                      // 0x000D4A  ADC-D interrupt #1
    TIMER0_ISR,                     // 0x000D4C  CPU TIMER 0 interrupt
    WAKE_ISR,                       // 0x000D4E  WAKE interrupt (LPM/WD)

//--- Core interrupt #2 re-map, PIE 2.1 - 2.8
    EPWM1_TZ_ISR,                   // 0x000D50  EPWM1 trip zone interrupt
    EPWM2_TZ_ISR,                   // 0x000D52  EPWM2 trip zone interrupt
    EPWM3_TZ_ISR,                   // 0x000D54  EPWM3 trip zone interrupt
    EPWM4_TZ_ISR,                   // 0x000D56  EPWM4 trip zone interrupt
    EPWM5_TZ_ISR,                   // 0x000D58  EPWM5 trip zone interrupt
    EPWM6_TZ_ISR,                   // 0x000D5A  EPWM6 trip zone interrupt
    EPWM7_TZ_ISR,                   // 0x000D5C  EPWM7 trip zone interrupt
    EPWM8_TZ_ISR,                   // 0x000D5E  EPWM8 trip zone interrupt

//--- Core interrupt #3 re-map, PIE 3.1 - 3.8
    EPWM1_ISR,                      // 0x000D60  EPWM1 interrupt
    EPWM2_ISR,                      // 0x000D62  EPWM2 interrupt
    EPWM3_ISR,                      // 0x000D64  EPWM3 interrupt
    EPWM4_ISR,                      // 0x000D66  EPWM4 interrupt
    EPWM5_ISR,                      // 0x000D68  EPWM5 interrupt
    EPWM6_ISR,                      // 0x000D6A  EPWM6 interrupt
    EPWM7_ISR,                      // 0x000D6C  EPWM7 interrupt
    EPWM8_ISR,                      // 0x000D6E  EPWM8 interrupt

//--- Core interrupt #4 re-map, PIE 4.1 - 4.8
    ECAP1_ISR,                      // 0x000D70  ECAP1 interrupt
    ECAP2_ISR,                      // 0x000D72  ECAP2 interrupt
    ECAP3_ISR,                      // 0x000D74  ECAP3 interrupt
    ECAP4_ISR,                      // 0x000D76  ECAP4 interrupt
    ECAP5_ISR,                      // 0x000D78  ECAP5 interrupt
    ECAP6_ISR,                      // 0x000D7A  ECAP6 interrupt
    PIE_RESERVED_ISR,               // 0x000D7C  reserved
    PIE_RESERVED_ISR,               // 0x000D7E  reserved

//--- Core interrupt #5 re-map, PIE 5.1 - 5.8
    EQEP1_ISR,                      // 0x000D80  EQEP1 interrupt
    EQEP2_ISR,                      // 0x000D82  EQEP2 interrupt
    EQEP3_ISR,                      // 0x000D84  EQEP3 interrupt
    PIE_RESERVED_ISR,               // 0x000D86  reserved
    PIE_RESERVED_ISR,               // 0x000D88  reserved
    PIE_RESERVED_ISR,               // 0x000D8A  reserved
    PIE_RESERVED_ISR,               // 0x000D8C  reserved
    PIE_RESERVED_ISR,               // 0x000D8E  reserved

//--- Core interrupt #6 re-map, PIE 6.1 - 6.8
    SPIA_RX_ISR,                    // 0x000D90  SPI-A RX interrupt
    SPIA_TX_ISR,                    // 0x000D92  SPI-A TX interrupt
    SPIB_RX_ISR,                    // 0x000D94  SPI-B RX interrupt
    SPIB_TX_ISR,                    // 0x000D96  SPI-B TX interrupt
    MCBSPA_RX_ISR,                  // 0x000D98  McBSP-A RX interrupt
    MCBSPA_TX_ISR,                  // 0x000D9A  McBSP-A TX interrupt
    MCBSPB_RX_ISR,                  // 0x000D9C  McBSP-B RX interrupt
    MCBSPB_TX_ISR,                  // 0x000D9E  McBSP-B TX interrupt

//--- Core interrupt #7 re-map, PIE 7.1 - 7.8
    DMA_CH1_ISR,                    // 0x000DA0  DMA channel 1 interrupt
    DMA_CH2_ISR,                    // 0x000DA2  DMA channel 2 interrupt
    DMA_CH3_ISR,                    // 0x000DA4  DMA channel 3 interrupt
    DMA_CH4_ISR,                    // 0x000DA6  DMA channel 4 interrupt
    DMA_CH5_ISR,                    // 0x000DA8  DMA channel 5 interrupt
    DMA_CH6_ISR,                    // 0x000DAA  DMA channel 6 interrupt
    PIE_RESERVED_ISR,               // 0x000DAC  reserved
    PIE_RESERVED_ISR,               // 0x000DAE  reserved

//--- Core interrupt #8 re-map, PIE 8.1 - 8.8
    I2CA_ISR,                       // 0x000DB0  I2C-A RX interrupt
    I2CA_FIFO_ISR,                  // 0x000DB2  I2C-A TX interrupt
    I2CB_ISR,                       // 0x000DB4  I2C-B RX interrupt
    I2CB_FIFO_ISR,                  // 0x000DB6  I2C-B TX interrupt
    SCIC_RX_ISR,                    // 0x000DB8  SCI-C RX interrupt
    SCIC_TX_ISR,                    // 0x000DBA  SCI-C TX interrupt
    SCID_RX_ISR,                    // 0x000DBC  SCI-D RX interrupt
    SCID_TX_ISR,                    // 0x000DBE  SCI-D TX interrupt

//--- Core interrupt #9 re-map, PIE 9.1 - 9.8
    SCIA_RX_ISR,                    // 0x000DC0  SCI-A RX interrupt
    SCIA_TX_ISR,                    // 0x000DC2  SCI-A TX interrupt
    SCIB_RX_ISR,                    // 0x000DC4  SCI-B RX interrupt
    SCIB_TX_ISR,                    // 0x000DC6  SCI-B TX interrupt
    CANA0_ISR,                      // 0x000DC8  CAN-A interrupt #0
    CANA1_ISR,                      // 0x000DCA  CAN-A interrupt #1
    CANB0_ISR,                      // 0x000DCC  CAN-B interrupt #0
    CANB1_ISR,                      // 0x000DCE  CAN-B interrupt #1

//--- Core interrupt #10 re-map, PIE 10.1 - 10.8
    ADCA_EVT_ISR,                   // 0x000DD0  ADC-A event interrupt
    ADCA2_ISR,                      // 0x000DD2  ADC-A interrupt #2
    ADCA3_ISR,                      // 0x000DD4  ADC-A interrupt #3
    ADCA4_ISR,                      // 0x000DD6  ADC-A interrupt #4
    ADCB_EVT_ISR,                   // 0x000DD8  ADC-B event interrupt
    ADCB2_ISR,                      // 0x000DDA  ADC-B interrupt #2
    ADCB3_ISR,                      // 0x000DDC  ADC-B interrupt #3
    ADCB4_ISR,                      // 0x000DDE  ADC-B interrupt #4

//--- Core interrupt #11 re-map, PIE 11.1 - 11.8
    CLA1_1_ISR,                     // 0x000DE0  CLA1 interrupt #1
    CLA1_2_ISR,                     // 0x000DE2  CLA1 interrupt #2
    CLA1_3_ISR,                     // 0x000DE4  CLA1 interrupt #3
    CLA1_4_ISR,                     // 0x000DE6  CLA1 interrupt #4
    CLA1_5_ISR,                     // 0x000DE8  CLA1 interrupt #5
    CLA1_6_ISR,                     // 0x000DEA  CLA1 interrupt #6
    CLA1_7_ISR,                     // 0x000DEC  CLA1 interrupt #7
    CLA1_8_ISR,                     // 0x000DEE  CLA1 interrupt #8

//--- Core interrupt #12 re-map, PIE 12.1 - 12.8
    XINT3_ISR,                      // 0x000DF0  External interrupt #3
    XINT4_ISR,                      // 0x000DF2  External interrupt #4
    XINT5_ISR,                      // 0x000DF4  External interrupt #5
    PIE_RESERVED_ISR,               // 0x000DF6  reserved
    PIE_RESERVED_ISR,               // 0x000DF8  reserved
    VCU_ISR,                        // 0x000DFA  VCU interrupt
    FPU_OVERFLOW_ISR,               // 0x000DFC  FPU overflow interrupt
    FPU_UNDERFLOW_ISR,              // 0x000DFE  FPU underflow interrupt

//--- Core interrupt #1 re-map, PIE 1.9 - 1.16
    PIE_RESERVED_ISR,               // 0x000E00  reserved
    PIE_RESERVED_ISR,               // 0x000E02  reserved
    PIE_RESERVED_ISR,               // 0x000E04  reserved
    PIE_RESERVED_ISR,               // 0x000E06  reserved
    IPC0_ISR,           	        // 0x000E08  IPC interrupt #0
    IPC1_ISR,       	            // 0x000E0A  IPC interrupt #1
    IPC2_ISR,   	                // 0x000E0C  IPC interrupt #2
    IPC3_ISR, 	                    // 0x000E0E  IPC interrupt #3

//--- Core interrupt #2 re-map, PIE 2.9 - 2.16
    EPWM9_TZ_ISR,                   // 0x000E10  ePWM9 trip zone interrupt
    EPWM10_TZ_ISR,                  // 0x000E12  ePWM10 trip zone interrupt
    EPWM11_TZ_ISR,                  // 0x000E14  ePWM11 trip zone interrupt
    EPWM12_TZ_ISR,                  // 0x000E16  ePWM12 trip zone interrupt
    PIE_RESERVED_ISR,               // 0x000E18  reserved
    PIE_RESERVED_ISR,               // 0x000E1A  reserved
    PIE_RESERVED_ISR,               // 0x000E1C  reserved
    PIE_RESERVED_ISR,               // 0x000E1E  reserved

//--- Core interrupt #3 re-map, PIE 3.9 - 3.16
    EPWM9_ISR,                      // 0x000E20  ePWM9 interrupt
    EPWM10_ISR,                     // 0x000E22  ePWM10 interrupt
    EPWM11_ISR,                     // 0x000E24  ePWM11 interrupt
    EPWM12_ISR,                     // 0x000E26  ePWM12 interrupt
    PIE_RESERVED_ISR,               // 0x000E28  reserved
    PIE_RESERVED_ISR,               // 0x000E2A  reserved
    PIE_RESERVED_ISR,               // 0x000E2C  reserved
    PIE_RESERVED_ISR,               // 0x000E2E  reserved

//--- Core interrupt #4 re-map, PIE 4.9 - 4.16
    PIE_RESERVED_ISR,               // 0x000E30  reserved
    PIE_RESERVED_ISR,               // 0x000E32  reserved
    PIE_RESERVED_ISR,               // 0x000E34  reserved
    PIE_RESERVED_ISR,               // 0x000E36  reserved
    PIE_RESERVED_ISR,               // 0x000E38  reserved
    PIE_RESERVED_ISR,               // 0x000E3A  reserved
    PIE_RESERVED_ISR,               // 0x000E3C  reserved
    PIE_RESERVED_ISR,               // 0x000E3E  reserved

//--- Core interrupt #5 re-map, PIE 5.9 - 5.16
    SD1_ISR,                        // 0x000E40  Sigma-delta #1 interrupt
    SD2_ISR,                        // 0x000E42  Sigma-delta #2 interrupt
    PIE_RESERVED_ISR,               // 0x000E44  reserved
    PIE_RESERVED_ISR,               // 0x000E46  reserved
    PIE_RESERVED_ISR,               // 0x000E48  reserved
    PIE_RESERVED_ISR,               // 0x000E4A  reserved
    PIE_RESERVED_ISR,               // 0x000E4C  reserved
    PIE_RESERVED_ISR,               // 0x000E4E  reserved

//--- Core interrupt #6 re-map, PIE 6.9 - 6.16
    SPIC_RX_ISR,                    // 0x000E50  SPI-C RX interrupt
    SPIC_TX_ISR,                    // 0x000E52  SPI-C TX interrupt
    PIE_RESERVED_ISR,               // 0x000E54  reserved
    PIE_RESERVED_ISR,               // 0x000E56  reserved
    PIE_RESERVED_ISR,               // 0x000E58  reserved
    PIE_RESERVED_ISR,               // 0x000E5A  reserved
    PIE_RESERVED_ISR,               // 0x000E5C  reserved
    PIE_RESERVED_ISR,               // 0x000E5E  reserved

//--- Core interrupt #7 re-map, PIE 7.9 - 7.16
    PIE_RESERVED_ISR,               // 0x000E60  reserved
    PIE_RESERVED_ISR,               // 0x000E62  reserved
    PIE_RESERVED_ISR,               // 0x000E64  reserved
    PIE_RESERVED_ISR,               // 0x000E66  reserved
    PIE_RESERVED_ISR,               // 0x000E68  reserved
    PIE_RESERVED_ISR,               // 0x000E6A  reserved
    PIE_RESERVED_ISR,               // 0x000E6C  reserved
    PIE_RESERVED_ISR,               // 0x000E6E  reserved

//--- Core interrupt #8 re-map, PIE 8.9 - 8.16
    PIE_RESERVED_ISR,               // 0x000E70  reserved
    PIE_RESERVED_ISR,               // 0x000E72  reserved
    PIE_RESERVED_ISR,               // 0x000E74  reserved
    PIE_RESERVED_ISR,               // 0x000E76  reserved
    PIE_RESERVED_ISR,               // 0x000E78  reserved
    PIE_RESERVED_ISR,               // 0x000E7A  reserved
#ifdef CPU1
	UPPA_ISR,                       // 0x000E7C  UPPA interrupt
#elif defined(CPU2)
    PIE_RESERVED_ISR,               // 0x000E7C  reserved
#else
	#error "Either 'CPU1' or 'CPU2' must be defined in the project"
#endif
    PIE_RESERVED_ISR,               // 0x000E7E  reserved

//--- Core interrupt #9 re-map, PIE 9.9 - 9.16
    PIE_RESERVED_ISR,               // 0x000E80  reserved
    PIE_RESERVED_ISR,               // 0x000E82  reserved
    PIE_RESERVED_ISR,               // 0x000E84  reserved
    PIE_RESERVED_ISR,               // 0x000E86  reserved
    PIE_RESERVED_ISR,               // 0x000E88  reserved
    PIE_RESERVED_ISR,               // 0x000E8A  reserved
#ifdef CPU1
	USBA_ISR,                       // 0x000E8C  USBA interrupt
#elif defined(CPU2)
    PIE_RESERVED_ISR,               // 0x000E8C  reserved
#else
	#error "Either 'CPU1' or 'CPU2' must be defined in the project"
#endif
    PIE_RESERVED_ISR,               // 0x000E8E  reserved

//--- Core interrupt #10 re-map, PIE 10.9 - 10.16
    ADCC_EVT_ISR,                   // 0x000E90  ADC-C event interrupt
    ADCC2_ISR,                      // 0x000E92  ADC-C interrupt #2
    ADCC3_ISR,                      // 0x000E94  ADC-C interrupt #3
    ADCC4_ISR,                      // 0x000E96  ADC-C interrupt #4
    ADCD_EVT_ISR,                   // 0x000E98  ADC-D event interrupt
    ADCD2_ISR,                      // 0x000E9A  ADC-D interrupt #2
    ADCD3_ISR,                      // 0x000E9C  ADC-D interrupt #3
    ADCD4_ISR,                      // 0x000E9E  ADC-D interrupt #4

//--- Core interrupt #11 re-map, PIE 11.9 - 11.16
    PIE_RESERVED_ISR,               // 0x000EA0  reserved
    PIE_RESERVED_ISR,               // 0x000EA2  reserved
    PIE_RESERVED_ISR,               // 0x000EA4  reserved
    PIE_RESERVED_ISR,               // 0x000EA6  reserved
    PIE_RESERVED_ISR,               // 0x000EA8  reserved
    PIE_RESERVED_ISR,               // 0x000EAA  reserved
    PIE_RESERVED_ISR,               // 0x000EAC  reserved
    PIE_RESERVED_ISR,               // 0x000EAE  reserved

//--- Core interrupt #12 re-map, PIE 12.9 - 12.16
    EMIF_ERROR_ISR,                 // 0x000EB0  EMIF error interrupt
    RAM_CORRECTABLE_ERROR_ISR,      // 0x000EB2  RAM correctable error interrupt
    FLASH_CORRECTABLE_ERROR_ISR,    // 0x000EB4  Flash correctable error interrupt
    RAM_ACCESS_VIOLATION_ISR,       // 0x000EB6  RAM access violation interrupt
    SYS_PLL_SLIP_ISR,               // 0x000EB8  System PLL slip interrupt
    AUX_PLL_SLIP_ISR,               // 0x000EBA  Auxiliary PLL slip interrupt
    CLA_OVERFLOW_ISR,              	// 0x000EBC  CLA overflow interrupt
    CLA_UNDERFLOW_ISR              	// 0x000EBE  CLA underflow interrupt

}; // end PieVectTableInit{}


//--- end of file -----------------------------------------------------
