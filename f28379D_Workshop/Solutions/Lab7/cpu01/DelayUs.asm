**********************************************************************
* File: DelayUs.asm
* Devices: TMS320F2837x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************

WDKEY	.set	0x7025

**********************************************************************
* Function: DelayUs()
* Description: Implements a time delay
* DSP: TMS320F2837x
* Include files: none
* Function Prototype: void DelayUs(unsigned int)
* Useage: DelayUs(Usec);
* Input Parameters: unsigned int Usec = time delay in microseconds
* Return Value: none
* Notes:
*   1) The execution time of this routine is based upon a 200 MHz
*      CPUCLK.  It also assumes that the function executes out of
*      internal RAM.  If executing out of internal flash or external
*      memory, the execution speed will be slightly slower.
*      However, the inner loop of this function is essentially
*      invariant to the memory it is running in.  Therefore, even
*      when running in flash memory, the basic loop time will be
*      only slightly longer than 1 us.
*
*   2) The outer loop of this function is interruptible (i.e., every
*      1 us).  The user should disable interrupts before calling the
*      function if they need an exact delay time, as opposed to a
*      minimum delay time.
*
*   3) The constant LOOP_COUNT at the top of the code below controls
*      the number of delay cycles.  By default, it is setup for 200 MHz
*      device operation.  It can be adjusted by the user for other CPU
*      frequencies and the function then re-compiled according to the
*      following equation:
*
*      LOOP_COUNT = (delay * f_cpu) - 12
*
*         where:  delay = desired delay in us
*                 f_cpu = CPU frequency in MHz.
*
*      Example: delay = 1 us, f_cpu = 200 MHz  ==> LOOP_COUNT = 188
*
**********************************************************************

LOOP_COUNT   .set    188

       .def _DelayUs
       .text

_DelayUs:
        MOVB AH, #0                   ;Zero AH
        PUSH ST1                      ;Save ST1 to preserve EALLOW setting

DelayUs1:                             ;Outer loop

;Service the watchdog in case it is active
        EALLOW                        ;(1 cycle)
        MOVZ DP, #(WDKEY>>6)          ;(1 cycle)
        MOV @WDKEY, #0x0055           ;(1 cycle)
        MOV @WDKEY, #0x00AA           ;(1 cycle)
        EDIS                          ;(1 cycle)

;Proceed with the inner loop
        RPT #LOOP_COUNT               ;Inner loop (1 cycle)
     || NOP                           ;(LOOP_COUNT + 1 cycles)

        SUBB ACC,#1                   ;Decrement outer loop counter (1 cycle)
        BF DelayUs1, GT               ;Branch for outer loop (4 cycles)

;Finish up
        POP ST1                       ;Restore ST1
        LRETR                         ;Return

;end of function DelayUs()
**********************************************************************

       .end
;end of file DelayUs.asm
