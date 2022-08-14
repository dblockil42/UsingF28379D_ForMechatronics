***********************************************************************
* File: CodeStartBranch.asm
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
***********************************************************************

WD_DISABLE	.set	1		;set to 1 to disable WD, else set to 0

    .ref _c_int00
    .def code_start

***********************************************************************
* Function: codestart section
*
* Description: Branch to code starting point
***********************************************************************
    .sect "codestart"
code_start:
    .if WD_DISABLE == 1
        LB wd_disable       ;Branch to watchdog disable code
    .else
        LB _c_int00         ;Branch to start of boot.asm in RTS library
    .endif

;end codestart section


***********************************************************************
* Function: wd_disable
*
* Description: Disables the watchdog timer
***********************************************************************
    .if WD_DISABLE == 1

	.text
wd_disable:
	EALLOW					;Enable EALLOW protected register access
    MOVZ DP, #7029h>>6      ;Set data page for WDCR register
    MOV @7029h, #0068h      ;Set WDDIS bit in WDCR to disable WD
	EDIS					;Disable EALLOW protected register access
    LB _c_int00             ;Branch to start of boot.asm in RTS library

    .endif

;end wd_disable
***********************************************************************


    .end                    ; end of file CodeStartBranch.asm