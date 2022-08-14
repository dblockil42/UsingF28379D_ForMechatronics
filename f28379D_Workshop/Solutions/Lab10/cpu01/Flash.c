/**********************************************************************
* File: Flash.c
* Devices: TMS320F2837x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"						// Main include file


/**********************************************************************
* Function: InitFlash()
* Description: Initializes the F2837xD flash timing registers.
* Notes:
*  1) This function MUST be executed out of RAM.  Executing it out of
*     OTP/FLASH will produce unpredictable results.
*
*  2) The flash registers are code security module protected.  Therefore,
*     you must either run this function from secure RAM, or you must
*     first unlock the CSM.  Note that unlocking the CSM as part of the
*     program flow can compromise the code security.
*
* 3) The latest datasheet for the particular device of interest should
*    be consulted to confirm the flash timing specifications.
**********************************************************************/
#pragma CODE_SECTION(InitFlash, "secureRamFuncs")
void InitFlash(void)
{
	asm(" EALLOW");									// Enable EALLOW protected register access

    //At reset bank and pump are in sleep
    //A Flash access will power up the bank and pump automatically
    //After a Flash access, bank and pump go to low power mode (configurable in FBFALLBACK/FPAC1 registers)-
    //if there is no further access to flash
    //Power up Flash bank and pump and this also sets the fall back mode of flash and pump as active
    Flash0CtrlRegs.FPAC1.bit.PMPPWR = 0x1;
    Flash0CtrlRegs.FBFALLBACK.bit.BNKPWR0 = 0x3;

    //Disable Cache and prefetch mechanism before changing wait states
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.DATA_CACHE_EN = 0;
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.PREFETCH_EN = 0;

    //Set waitstates according to frequency
    //                CAUTION
    //Minimum waitstates required for the flash operating
    //at a given CPU rate must be characterized by TI.
    //Refer to the datasheet for the latest information.
    Flash0CtrlRegs.FRDCNTL.bit.RWAIT = 0x3;			// Setting for 200 MHz

    //Enable Cache and prefetch mechanism to improve performance
    //of code executed from Flash.
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.DATA_CACHE_EN = 1;
    Flash0CtrlRegs.FRD_INTF_CTRL.bit.PREFETCH_EN = 1;

    //At reset, ECC is enabled
    //If it is disabled by application software and if application again wants to enable ECC
    Flash0EccRegs.ECC_ENABLE.bit.ENABLE = 0xA;

    asm(" EDIS");									// Disable EALLOW protected register access

// Force a complete pipeline flush to ensure that the write to the last register
// configured occurs before returning.  Safest thing is to wait 8 full cycles.

    asm(" RPT #6 || NOP");

} // end of InitFlash()


//--- end of file -----------------------------------------------------
