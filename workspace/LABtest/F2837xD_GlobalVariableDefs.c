//###########################################################################
//
// FILE:    F2837xD_globalvariabledefs.c
//
// TITLE:   F2837xD Global Variables and Data Section Pragmas.
//
//###########################################################################
//
// $Release Date:  $
// $Copyright:
// Copyright (C) 2013-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#include "F2837xD_device.h"     // F2837xD Headerfile Include File

//---------------------------------------------------------------------------
// Define Global Peripheral Variables:
//

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AccessProtectionRegsFile")
#else
#pragma DATA_SECTION(AccessProtectionRegs,"AccessProtectionRegsFile");
#endif
volatile struct ACCESS_PROTECTION_REGS AccessProtectionRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdcaRegsFile")
#else
#pragma DATA_SECTION(AdcaRegs,"AdcaRegsFile");
#endif
volatile struct ADC_REGS AdcaRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdcbRegsFile")
#else
#pragma DATA_SECTION(AdcbRegs,"AdcbRegsFile");
#endif
volatile struct ADC_REGS AdcbRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdccRegsFile")
#else
#pragma DATA_SECTION(AdccRegs,"AdccRegsFile");
#endif
volatile struct ADC_REGS AdccRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdcdRegsFile")
#else
#pragma DATA_SECTION(AdcdRegs,"AdcdRegsFile");
#endif
volatile struct ADC_REGS AdcdRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdcaResultRegsFile")
#else
#pragma DATA_SECTION(AdcaResultRegs,"AdcaResultRegsFile");
#endif
volatile struct ADC_RESULT_REGS AdcaResultRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdcbResultRegsFile")
#else
#pragma DATA_SECTION(AdcbResultRegs,"AdcbResultRegsFile");
#endif
volatile struct ADC_RESULT_REGS AdcbResultRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdccResultRegsFile")
#else
#pragma DATA_SECTION(AdccResultRegs,"AdccResultRegsFile");
#endif
volatile struct ADC_RESULT_REGS AdccResultRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdcdResultRegsFile")
#else
#pragma DATA_SECTION(AdcdResultRegs,"AdcdResultRegsFile");
#endif
volatile struct ADC_RESULT_REGS AdcdResultRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AnalogSubsysRegsFile")
#else
#pragma DATA_SECTION(AnalogSubsysRegs,"AnalogSubsysRegsFile");
#endif
volatile struct ANALOG_SUBSYS_REGS AnalogSubsysRegs;
#endif // ifdef CPU1

#if __TI_COMPILER_VERSION__ >= 16006000
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CanaRegsFile")
#else
#pragma DATA_SECTION(CanaRegs,"CanaRegsFile");
#endif
volatile struct CAN_REGS CanaRegs;
#endif // __TI_COMPILER_VERSION__ >= 16006000

#if __TI_COMPILER_VERSION__ >= 16006000
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CanbRegsFile")
#else
#pragma DATA_SECTION(CanbRegs,"CanbRegsFile");
#endif
volatile struct CAN_REGS CanbRegs;
#endif // __TI_COMPILER_VERSION__ >= 16006000

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cla1RegsFile")
#else
#pragma DATA_SECTION(Cla1Regs,"Cla1RegsFile");
#endif
volatile struct CLA_REGS Cla1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb1DataExchRegsFile")
#else
#pragma DATA_SECTION(Clb1DataExchRegs,"Clb1DataExchRegsFile");
#endif
volatile struct CLB_DATA_EXCHANGE_REGS Clb1DataExchRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb2DataExchRegsFile")
#else
#pragma DATA_SECTION(Clb2DataExchRegs,"Clb2DataExchRegsFile");
#endif
volatile struct CLB_DATA_EXCHANGE_REGS Clb2DataExchRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb3DataExchRegsFile")
#else
#pragma DATA_SECTION(Clb3DataExchRegs,"Clb3DataExchRegsFile");
#endif
volatile struct CLB_DATA_EXCHANGE_REGS Clb3DataExchRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb4DataExchRegsFile")
#else
#pragma DATA_SECTION(Clb4DataExchRegs,"Clb4DataExchRegsFile");
#endif
volatile struct CLB_DATA_EXCHANGE_REGS Clb4DataExchRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb1LogicCfgRegsFile")
#else
#pragma DATA_SECTION(Clb1LogicCfgRegs,"Clb1LogicCfgRegsFile");
#endif
volatile struct CLB_LOGIC_CONFIG_REGS Clb1LogicCfgRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb2LogicCfgRegsFile")
#else
#pragma DATA_SECTION(Clb2LogicCfgRegs,"Clb2LogicCfgRegsFile");
#endif
volatile struct CLB_LOGIC_CONFIG_REGS Clb2LogicCfgRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb3LogicCfgRegsFile")
#else
#pragma DATA_SECTION(Clb3LogicCfgRegs,"Clb3LogicCfgRegsFile");
#endif
volatile struct CLB_LOGIC_CONFIG_REGS Clb3LogicCfgRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb4LogicCfgRegsFile")
#else
#pragma DATA_SECTION(Clb4LogicCfgRegs,"Clb4LogicCfgRegsFile");
#endif
volatile struct CLB_LOGIC_CONFIG_REGS Clb4LogicCfgRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb1LogicCtrlRegsFile")
#else
#pragma DATA_SECTION(Clb1LogicCtrlRegs,"Clb1LogicCtrlRegsFile");
#endif
volatile struct CLB_LOGIC_CONTROL_REGS Clb1LogicCtrlRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb2LogicCtrlRegsFile")
#else
#pragma DATA_SECTION(Clb2LogicCtrlRegs,"Clb2LogicCtrlRegsFile");
#endif
volatile struct CLB_LOGIC_CONTROL_REGS Clb2LogicCtrlRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb3LogicCtrlRegsFile")
#else
#pragma DATA_SECTION(Clb3LogicCtrlRegs,"Clb3LogicCtrlRegsFile");
#endif
volatile struct CLB_LOGIC_CONTROL_REGS Clb3LogicCtrlRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Clb4LogicCtrlRegsFile")
#else
#pragma DATA_SECTION(Clb4LogicCtrlRegs,"Clb4LogicCtrlRegsFile");
#endif
volatile struct CLB_LOGIC_CONTROL_REGS Clb4LogicCtrlRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ClbXbarRegsFile")
#else
#pragma DATA_SECTION(ClbXbarRegs,"ClbXbarRegsFile");
#endif
volatile struct CLB_XBAR_REGS ClbXbarRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ClkCfgRegsFile")
#else
#pragma DATA_SECTION(ClkCfgRegs,"ClkCfgRegsFile");
#endif
volatile struct CLK_CFG_REGS ClkCfgRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cmpss1RegsFile")
#else
#pragma DATA_SECTION(Cmpss1Regs,"Cmpss1RegsFile");
#endif
volatile struct CMPSS_REGS Cmpss1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cmpss2RegsFile")
#else
#pragma DATA_SECTION(Cmpss2Regs,"Cmpss2RegsFile");
#endif
volatile struct CMPSS_REGS Cmpss2Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cmpss3RegsFile")
#else
#pragma DATA_SECTION(Cmpss3Regs,"Cmpss3RegsFile");
#endif
volatile struct CMPSS_REGS Cmpss3Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cmpss4RegsFile")
#else
#pragma DATA_SECTION(Cmpss4Regs,"Cmpss4RegsFile");
#endif
volatile struct CMPSS_REGS Cmpss4Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cmpss5RegsFile")
#else
#pragma DATA_SECTION(Cmpss5Regs,"Cmpss5RegsFile");
#endif
volatile struct CMPSS_REGS Cmpss5Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cmpss6RegsFile")
#else
#pragma DATA_SECTION(Cmpss6Regs,"Cmpss6RegsFile");
#endif
volatile struct CMPSS_REGS Cmpss6Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cmpss7RegsFile")
#else
#pragma DATA_SECTION(Cmpss7Regs,"Cmpss7RegsFile");
#endif
volatile struct CMPSS_REGS Cmpss7Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Cmpss8RegsFile")
#else
#pragma DATA_SECTION(Cmpss8Regs,"Cmpss8RegsFile");
#endif
volatile struct CMPSS_REGS Cmpss8Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CpuTimer0RegsFile")
#else
#pragma DATA_SECTION(CpuTimer0Regs,"CpuTimer0RegsFile");
#endif
volatile struct CPUTIMER_REGS CpuTimer0Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CpuTimer1RegsFile")
#else
#pragma DATA_SECTION(CpuTimer1Regs,"CpuTimer1RegsFile");
#endif
volatile struct CPUTIMER_REGS CpuTimer1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CpuTimer2RegsFile")
#else
#pragma DATA_SECTION(CpuTimer2Regs,"CpuTimer2RegsFile");
#endif
volatile struct CPUTIMER_REGS CpuTimer2Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CpuSysRegsFile")
#else
#pragma DATA_SECTION(CpuSysRegs,"CpuSysRegsFile");
#endif
volatile struct CPU_SYS_REGS CpuSysRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DacaRegsFile")
#else
#pragma DATA_SECTION(DacaRegs,"DacaRegsFile");
#endif
volatile struct DAC_REGS DacaRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DacbRegsFile")
#else
#pragma DATA_SECTION(DacbRegs,"DacbRegsFile");
#endif
volatile struct DAC_REGS DacbRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DaccRegsFile")
#else
#pragma DATA_SECTION(DaccRegs,"DaccRegsFile");
#endif
volatile struct DAC_REGS DaccRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DcsmCommonRegsFile")
#else
#pragma DATA_SECTION(DcsmCommonRegs,"DcsmCommonRegsFile");
#endif
volatile struct DCSM_COMMON_REGS DcsmCommonRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DcsmZ1RegsFile")
#else
#pragma DATA_SECTION(DcsmZ1Regs,"DcsmZ1RegsFile");
#endif
volatile struct DCSM_Z1_REGS DcsmZ1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DcsmZ2RegsFile")
#else
#pragma DATA_SECTION(DcsmZ2Regs,"DcsmZ2RegsFile");
#endif
volatile struct DCSM_Z2_REGS DcsmZ2Regs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DevCfgRegsFile")
#else
#pragma DATA_SECTION(DevCfgRegs,"DevCfgRegsFile");
#endif
volatile struct DEV_CFG_REGS DevCfgRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DmaClaSrcSelRegsFile")
#else
#pragma DATA_SECTION(DmaClaSrcSelRegs,"DmaClaSrcSelRegsFile");
#endif
volatile struct DMA_CLA_SRC_SEL_REGS DmaClaSrcSelRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("DmaRegsFile")
#else
#pragma DATA_SECTION(DmaRegs,"DmaRegsFile");
#endif
volatile struct DMA_REGS DmaRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ECap1RegsFile")
#else
#pragma DATA_SECTION(ECap1Regs,"ECap1RegsFile");
#endif
volatile struct ECAP_REGS ECap1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ECap2RegsFile")
#else
#pragma DATA_SECTION(ECap2Regs,"ECap2RegsFile");
#endif
volatile struct ECAP_REGS ECap2Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ECap3RegsFile")
#else
#pragma DATA_SECTION(ECap3Regs,"ECap3RegsFile");
#endif
volatile struct ECAP_REGS ECap3Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ECap4RegsFile")
#else
#pragma DATA_SECTION(ECap4Regs,"ECap4RegsFile");
#endif
volatile struct ECAP_REGS ECap4Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ECap5RegsFile")
#else
#pragma DATA_SECTION(ECap5Regs,"ECap5RegsFile");
#endif
volatile struct ECAP_REGS ECap5Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ECap6RegsFile")
#else
#pragma DATA_SECTION(ECap6Regs,"ECap6RegsFile");
#endif
volatile struct ECAP_REGS ECap6Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Emif1ConfigRegsFile")
#else
#pragma DATA_SECTION(Emif1ConfigRegs,"Emif1ConfigRegsFile");
#endif
volatile struct EMIF1_CONFIG_REGS Emif1ConfigRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Emif2ConfigRegsFile")
#else
#pragma DATA_SECTION(Emif2ConfigRegs,"Emif2ConfigRegsFile");
#endif
volatile struct EMIF2_CONFIG_REGS Emif2ConfigRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Emif1RegsFile")
#else
#pragma DATA_SECTION(Emif1Regs,"Emif1RegsFile");
#endif
volatile struct EMIF_REGS Emif1Regs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Emif2RegsFile")
#else
#pragma DATA_SECTION(Emif2Regs,"Emif2RegsFile");
#endif
volatile struct EMIF_REGS Emif2Regs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm1RegsFile")
#else
#pragma DATA_SECTION(EPwm1Regs,"EPwm1RegsFile");
#endif
volatile struct EPWM_REGS EPwm1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm2RegsFile")
#else
#pragma DATA_SECTION(EPwm2Regs,"EPwm2RegsFile");
#endif
volatile struct EPWM_REGS EPwm2Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm3RegsFile")
#else
#pragma DATA_SECTION(EPwm3Regs,"EPwm3RegsFile");
#endif
volatile struct EPWM_REGS EPwm3Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm4RegsFile")
#else
#pragma DATA_SECTION(EPwm4Regs,"EPwm4RegsFile");
#endif
volatile struct EPWM_REGS EPwm4Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm5RegsFile")
#else
#pragma DATA_SECTION(EPwm5Regs,"EPwm5RegsFile");
#endif
volatile struct EPWM_REGS EPwm5Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm6RegsFile")
#else
#pragma DATA_SECTION(EPwm6Regs,"EPwm6RegsFile");
#endif
volatile struct EPWM_REGS EPwm6Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm7RegsFile")
#else
#pragma DATA_SECTION(EPwm7Regs,"EPwm7RegsFile");
#endif
volatile struct EPWM_REGS EPwm7Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm8RegsFile")
#else
#pragma DATA_SECTION(EPwm8Regs,"EPwm8RegsFile");
#endif
volatile struct EPWM_REGS EPwm8Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm9RegsFile")
#else
#pragma DATA_SECTION(EPwm9Regs,"EPwm9RegsFile");
#endif
volatile struct EPWM_REGS EPwm9Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm10RegsFile")
#else
#pragma DATA_SECTION(EPwm10Regs,"EPwm10RegsFile");
#endif
volatile struct EPWM_REGS EPwm10Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm11RegsFile")
#else
#pragma DATA_SECTION(EPwm11Regs,"EPwm11RegsFile");
#endif
volatile struct EPWM_REGS EPwm11Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm12RegsFile")
#else
#pragma DATA_SECTION(EPwm12Regs,"EPwm12RegsFile");
#endif
volatile struct EPWM_REGS EPwm12Regs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwmXbarRegsFile")
#else
#pragma DATA_SECTION(EPwmXbarRegs,"EPwmXbarRegsFile");
#endif
volatile struct EPWM_XBAR_REGS EPwmXbarRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EQep1RegsFile")
#else
#pragma DATA_SECTION(EQep1Regs,"EQep1RegsFile");
#endif
volatile struct EQEP_REGS EQep1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EQep2RegsFile")
#else
#pragma DATA_SECTION(EQep2Regs,"EQep2RegsFile");
#endif
volatile struct EQEP_REGS EQep2Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EQep3RegsFile")
#else
#pragma DATA_SECTION(EQep3Regs,"EQep3RegsFile");
#endif
volatile struct EQEP_REGS EQep3Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Flash0CtrlRegsFile")
#else
#pragma DATA_SECTION(Flash0CtrlRegs,"Flash0CtrlRegsFile");
#endif
volatile struct FLASH_CTRL_REGS Flash0CtrlRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Flash0EccRegsFile")
#else
#pragma DATA_SECTION(Flash0EccRegs,"Flash0EccRegsFile");
#endif
volatile struct FLASH_ECC_REGS Flash0EccRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("FlashPumpSemaphoreRegsFile")
#else
#pragma DATA_SECTION(FlashPumpSemaphoreRegs,"FlashPumpSemaphoreRegsFile");
#endif
volatile struct FLASH_PUMP_SEMAPHORE_REGS FlashPumpSemaphoreRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("GpioCtrlRegsFile")
#else
#pragma DATA_SECTION(GpioCtrlRegs,"GpioCtrlRegsFile");
#endif
volatile struct GPIO_CTRL_REGS GpioCtrlRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("GpioDataRegsFile")
#else
#pragma DATA_SECTION(GpioDataRegs,"GpioDataRegsFile");
#endif
volatile struct GPIO_DATA_REGS GpioDataRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("I2caRegsFile")
#else
#pragma DATA_SECTION(I2caRegs,"I2caRegsFile");
#endif
volatile struct I2C_REGS I2caRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("I2cbRegsFile")
#else
#pragma DATA_SECTION(I2cbRegs,"I2cbRegsFile");
#endif
volatile struct I2C_REGS I2cbRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("InputXbarRegsFile")
#else
#pragma DATA_SECTION(InputXbarRegs,"InputXbarRegsFile");
#endif
volatile struct INPUT_XBAR_REGS InputXbarRegs;
#endif // ifdef CPU1

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("IpcRegsFile")
#else
#pragma DATA_SECTION(IpcRegs,"IpcRegsFile");
#endif
volatile struct IPC_REGS_CPU1 IpcRegs;
#endif // ifdef CPU1

#ifdef CPU2
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("IpcRegsFile")
#else
#pragma DATA_SECTION(IpcRegs,"IpcRegsFile");
#endif
volatile struct IPC_REGS_CPU2 IpcRegs;
#endif // ifdef CPU2

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("MemoryErrorRegsFile")
#else
#pragma DATA_SECTION(MemoryErrorRegs,"MemoryErrorRegsFile");
#endif
volatile struct MEMORY_ERROR_REGS MemoryErrorRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("MemCfgRegsFile")
#else
#pragma DATA_SECTION(MemCfgRegs,"MemCfgRegsFile");
#endif
volatile struct MEM_CFG_REGS MemCfgRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("McbspaRegsFile")
#else
#pragma DATA_SECTION(McbspaRegs,"McbspaRegsFile");
#endif
volatile struct McBSP_REGS McbspaRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("McbspbRegsFile")
#else
#pragma DATA_SECTION(McbspbRegs,"McbspbRegsFile");
#endif
volatile struct McBSP_REGS McbspbRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("NmiIntruptRegsFile")
#else
#pragma DATA_SECTION(NmiIntruptRegs,"NmiIntruptRegsFile");
#endif
volatile struct NMI_INTRUPT_REGS NmiIntruptRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("OutputXbarRegsFile")
#else
#pragma DATA_SECTION(OutputXbarRegs,"OutputXbarRegsFile");
#endif
volatile struct OUTPUT_XBAR_REGS OutputXbarRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("PieCtrlRegsFile")
#else
#pragma DATA_SECTION(PieCtrlRegs,"PieCtrlRegsFile");
#endif
volatile struct PIE_CTRL_REGS PieCtrlRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("PieVectTableFile")
#else
#pragma DATA_SECTION(PieVectTable,"PieVectTableFile");
#endif
volatile struct PIE_VECT_TABLE PieVectTable;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("RomPrefetchRegsFile")
#else
#pragma DATA_SECTION(RomPrefetchRegs,"RomPrefetchRegsFile");
#endif
volatile struct ROM_PREFETCH_REGS RomPrefetchRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("RomWaitStateRegsFile")
#else
#pragma DATA_SECTION(RomWaitStateRegs,"RomWaitStateRegsFile");
#endif
volatile struct ROM_WAIT_STATE_REGS RomWaitStateRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("SciaRegsFile")
#else
#pragma DATA_SECTION(SciaRegs,"SciaRegsFile");
#endif
volatile struct SCI_REGS SciaRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ScibRegsFile")
#else
#pragma DATA_SECTION(ScibRegs,"ScibRegsFile");
#endif
volatile struct SCI_REGS ScibRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ScicRegsFile")
#else
#pragma DATA_SECTION(ScicRegs,"ScicRegsFile");
#endif
volatile struct SCI_REGS ScicRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("ScidRegsFile")
#else
#pragma DATA_SECTION(ScidRegs,"ScidRegsFile");
#endif
volatile struct SCI_REGS ScidRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Sdfm1RegsFile")
#else
#pragma DATA_SECTION(Sdfm1Regs,"Sdfm1RegsFile");
#endif
volatile struct SDFM_REGS Sdfm1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Sdfm2RegsFile")
#else
#pragma DATA_SECTION(Sdfm2Regs,"Sdfm2RegsFile");
#endif
volatile struct SDFM_REGS Sdfm2Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("SpiaRegsFile")
#else
#pragma DATA_SECTION(SpiaRegs,"SpiaRegsFile");
#endif
volatile struct SPI_REGS SpiaRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("SpibRegsFile")
#else
#pragma DATA_SECTION(SpibRegs,"SpibRegsFile");
#endif
volatile struct SPI_REGS SpibRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("SpicRegsFile")
#else
#pragma DATA_SECTION(SpicRegs,"SpicRegsFile");
#endif
volatile struct SPI_REGS SpicRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("SyncSocRegsFile")
#else
#pragma DATA_SECTION(SyncSocRegs,"SyncSocRegsFile");
#endif
volatile struct SYNC_SOC_REGS SyncSocRegs;
#endif // ifdef CPU1

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("UppRegsFile")
#else
#pragma DATA_SECTION(UppRegs,"UppRegsFile");
#endif
volatile struct UPP_REGS UppRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("WdRegsFile")
#else
#pragma DATA_SECTION(WdRegs,"WdRegsFile");
#endif
volatile struct WD_REGS WdRegs;

#ifdef CPU1
//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("XbarRegsFile")
#else
#pragma DATA_SECTION(XbarRegs,"XbarRegsFile");
#endif
volatile struct XBAR_REGS XbarRegs;
#endif // ifdef CPU1

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("XintRegsFile")
#else
#pragma DATA_SECTION(XintRegs,"XintRegsFile");
#endif
volatile struct XINT_REGS XintRegs;



//===========================================================================
// End of file.
//===========================================================================


