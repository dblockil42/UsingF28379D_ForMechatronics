#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "F28x_Project.h"     // Device Headerfile and Examples Include File

//LED Code Fragments

//LED1 Off use SET.bit to turn on LED
//GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

//LED2 Off use SET.bit to turn on LED
//GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

//LED3 Off use SET.bit to turn on LED
//GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

//LED4 Off use SET.bit to turn on LED
//GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

//LED5 Off use SET.bit to turn on LED
//GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

//LED6 Off use SET.bit to turn on LED
//GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

//LED7 Off use SET.bit to turn on LED
//GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

//LED8 Off use SET.bit to turn on LED
//GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

//LED9 Off use SET.bit to turn on LED
//GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

//LED10 Off use SET.bit to turn on LED
//GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

//LED11 Off use SET.bit to turn on LED
//GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

//LED12 Off use SET.bit to turn on LED
//GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

//LED13 Off use SET.bit to turn on LED
//GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

//LED14 Off use SET.bit to turn on LED
//GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

//LED15 Off use SET.bit to turn on LED
//GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

//LED16 Off use SET.bit to turn on LED
//GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

// Column 1 Off  Use SET.bit for On and TOGGLE.bit to toggle On/Off
//GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
//GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
//GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
//GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
//GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

// Column 2 Off  Use SET.bit for On and TOGGLE.bit to toggle On/Off
//GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
//GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
//GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
//GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

// Column 3 Off  Use SET.bit for On and TOGGLE.bit to toggle On/Off
//GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
//GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

// ROW 1 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
//GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
//GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

// ROW 2 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
//GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
//GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

// ROW 3 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
//GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
//GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

// ROW 4 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
//GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
//GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

// ROW 5 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
//GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;
//GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
//GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

//
// XXX		16B 
// XXX		27C
// XXX		38D
// XXX		49E
// XXXX		5AF
//                      1 2 3 4 5 6 7 8 9 A B C D E F
//uint16_t LED_ [15] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

// 0
// 000		000 
// 0X0		070
// 0X0		080
// 0X0		090
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_0[15] = {1,1,1,1,1,1,0,0,0,1,1,1,1,1,1};

// 1
// X0X		10B 
// 00X		00C
// X0X		30D
// X0X		40E
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_1[15] = {0,1,0,0,1,1,1,1,1,1,0,0,0,0,1};

// 2
// 000		000 
// XX0		270
// 000		000
// 0XX		09E
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_2[15] = {1,0,1,1,1,1,0,1,0,1,1,1,1,0,1};

// 3
// 000		000 
// XX0		270
// 000		000
// XX0		490
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_3[15] = {1,0,1,0,1,1,0,1,0,1,1,1,1,1,1};

// 4
// 0X0		060 
// 0X0		070
// 000		000
// XX0		490
// XX0X		5A0
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_4[15] = {1,1,1,0,0,0,0,1,0,0,1,1,1,1,1};

// 5
// 000		000 
// 0XX		07C
// 000		000
// XX0		490
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_5[15] = {1,1,1,0,1,1,0,1,0,1,1,0,1,1,1};

// 6
// 000		000 
// 0XX		07C
// 000		000
// 0X0		090
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_6[15] = {1,1,1,1,1,1,0,1,0,1,1,0,1,1,1};

// 7
// 000		000 
// XX0		270
// XX0		380
// XX0		490
// XX0X		5A0
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_7[15] = {1,0,0,0,0,1,0,0,0,0,1,1,1,1,1};

// 8
// 000		000 
// 0X0		070
// 000		000
// 0X0		090
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_8[15] = {1,1,1,1,1,1,0,1,0,1,1,1,1,1,1};

// 9
// 000		000 
// 0X0		070
// 000		000
// XX0		490
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_9[15] = {1,1,1,0,1,1,0,1,0,1,1,1,1,1,1};

// A
// 000		000 
// 0X0		070
// 000		000
// 0X0		090
// 0X0X		0A0
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_A[15] = {1,1,1,1,1,1,0,1,0,0,1,1,1,1,1};

// B
// 00X		00B 
// 0X0		070
// 00X		00D
// 0X0		090
// 00XX		00F
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_B[15] = {1,1,1,1,1,1,0,1,0,1,0,1,0,1,0};

// C
// 000		000 
// 0XX		07C
// 0XX		08D
// 0XX		09E
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_C[15] = {1,1,1,1,1,1,0,0,0,1,1,0,0,0,1};

// D
// 00X		00B 
// 0X0		070
// 0X0		080
// 0X0		090
// 00XX		00F
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_D[15] = {1,1,1,1,1,1,0,0,0,1,0,1,1,1,0};

// E
// 000		000 
// 0XX		07C
// 000		000
// 0XX		09E
// 000X		000
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_E[15] = {1,1,1,1,1,1,0,1,0,1,1,0,1,0,1};

// F
// 000		000 
// 0XX		07C
// 000		000
// 0XX		09E
// 0XXX		0AF
//                    1 2 3 4 5 6 7 8 9 A B C D E F 
uint16_t LED_F[15] = {1,1,1,1,1,1,0,1,0,0,1,0,1,0,0};



void setLEDLetter(uint16_t * letter) {

	if (letter[0] == 0) {
		GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	} else {
		GpioDataRegs.GPASET.bit.GPIO22 = 1;		
	}

	if (letter[1] == 0) {
		GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
	} else {
		GpioDataRegs.GPCSET.bit.GPIO94 = 1;		
	}

	if (letter[2] == 0) {
		GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
	} else {
		GpioDataRegs.GPCSET.bit.GPIO95 = 1;
	}

	if (letter[3] == 0) {
		GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
	} else {
		GpioDataRegs.GPDSET.bit.GPIO97 = 1;
	}

	if (letter[4] == 0) {
		GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;
	} else {
		GpioDataRegs.GPDSET.bit.GPIO111 = 1;
	}

	if (letter[5] == 0) {
		GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
	} else {
		GpioDataRegs.GPESET.bit.GPIO130 = 1;
	}

	if (letter[6] == 0) {
		GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
	} else {
		GpioDataRegs.GPESET.bit.GPIO131 = 1;
	}

	if (letter[7] == 0) {
		GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
	} else {
		GpioDataRegs.GPASET.bit.GPIO25 = 1;
	}

	if (letter[8] == 0) {
		GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
	} else {
		GpioDataRegs.GPASET.bit.GPIO26 = 1;
	}

	if (letter[9] == 0) {
		GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
	} else {
		GpioDataRegs.GPASET.bit.GPIO27 = 1;
	}

	if (letter[10] == 0) {
		GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
	} else {
		GpioDataRegs.GPBSET.bit.GPIO60 = 1;
	}

	if (letter[11] == 0) {
		GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
	} else {
		GpioDataRegs.GPBSET.bit.GPIO61 = 1;
	}

	if (letter[12] == 0) {
		GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;
	} else {
		GpioDataRegs.GPESET.bit.GPIO157 = 1;
	}

	if (letter[13] == 0) {
		GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	} else {
		GpioDataRegs.GPESET.bit.GPIO158 = 1;
	}

	if (letter[14] == 0) {
		GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;
	} else {
		GpioDataRegs.GPESET.bit.GPIO159 = 1;
	}

}


void displayLEDletter(uint16_t num) {
	num = num&0xF;  // only look at bottom 4 bits so number between 0 and 15 or 0x0 and 0xF
	if (num == 0) {
		setLEDLetter(LED_0);
	} else if (num == 1) {
		setLEDLetter(LED_1);
	} else if (num == 2) {
		setLEDLetter(LED_2);
	} else if (num == 3) {
		setLEDLetter(LED_3);
	} else if (num == 4) {
		setLEDLetter(LED_4);
	} else if (num == 5) {
		setLEDLetter(LED_5);
	} else if (num == 6) {
		setLEDLetter(LED_6);
	} else if (num == 7) {
		setLEDLetter(LED_7);
	} else if (num == 8) {
		setLEDLetter(LED_8);
	} else if (num == 9) {
		setLEDLetter(LED_9);
	} else if (num == 10) {
		setLEDLetter(LED_A);
	} else if (num == 11) {
		setLEDLetter(LED_B);
	} else if (num == 12) {
		setLEDLetter(LED_C);
	} else if (num == 13) {
		setLEDLetter(LED_D);
	} else if (num == 14) {
		setLEDLetter(LED_E);
	} else if (num == 15) {
		setLEDLetter(LED_F);
	}
}
