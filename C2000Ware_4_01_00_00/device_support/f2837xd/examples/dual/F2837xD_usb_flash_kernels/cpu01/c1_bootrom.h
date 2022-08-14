//###########################################################################
//
// FILE:   c_bootrom.h
//
// TITLE:  C-BootROM Definitions.
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


#ifndef C_BOOTROM_H_
#define C_BOOTROM_H_

//
// Included Files
//
#include "F2837xD_device.h"
//#include "driver_incs.h"
//#include "c1_bootrom_ipc_commands.h"

//
// Defines
//

#ifndef IS_TRUE
#define IS_TRUE        1
#endif

#ifndef IS_FALSE
#define IS_FALSE       0
#endif

#ifndef IS_SUCCESS
#define IS_SUCCESS    IS_TRUE
#endif

#ifndef IS_FAILURE
#define IS_FAILURE    IS_FALSE
#endif

extern  Uint16 EmuKey;
extern  Uint16 EmuBMode;

#define KEY_VAL                 0x5A

#define STAND_ALONE_BOOT_TYPE   1
#define EMU_BOOT_TYPE           2
#define HIB_BOOT_TYPE           3

#define EMULATE_TRUE_BOOT       0xFF         // follow stand-alone boot with emulator connected
#define EMULATE_BOOTPIN_MODE    0xFE        // Get the boot pin info from EMUBMODE CTRL reg and follow it

#define PARALLEL_BOOT           0x0

#define SCI_BOOT                0x1
#define SCI_BOOT_ALTERNATE      0x81

#define WAIT_BOOT               0x2
#define GET_BOOT                0x3

#define SPI_BOOT                0x4
#define SPI_BOOT_ALTERNATE      0x84

#define I2C_BOOT                0x5
#define I2C_BOOT_ALTERNATE      0x85

//#define OTP_BOOT                0x6
#define CAN_BOOT                0x07
#define CAN_BOOT_ALTERNATE      0x87
#define CAN_BOOT_TEST_DEFAULT   0x47  //enables tx of two packets for bit rate calcs - default io
#define CAN_BOOT_TEST_ALT       0xC7  //enables tx of two packets for bit rate calcs  - alternate io

#define RAM_BOOT                0xA
#define FLASH_BOOT              0xB

#define USB_BOOT                0xC
#define USB_BOOT_ALTERNATE      0x8C  //reserved for now - defaults to flash

//
// Fixed boot entry points:
//
#define FLASH_ENTRY_POINT      0x080000
#define RAM_ENTRY_POINT        0x000000
//#define OTP_ENTRY_POINT        0x070BFE    //end of CPU2 mapped to CPU1

#define ERROR                   1
#define NO_ERROR                0

#define BROM_EIGHT_BIT_HEADER           0x08AA

//
//#define TI_OTP_PARTID_L                    *(volatile uint32_t *)(0x70200)
//#define TI_OTP_PARTID_H                    *(volatile uint32_t *)(0x70202)
//
//
//#define TI_OTP_DCX_REG_BASE_ADDRESS        0x70204
//#define TI_OTP_REG_DC00                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS)
//#define TI_OTP_REG_DC01                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 2)
//#define TI_OTP_REG_DC02                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 4)
//#define TI_OTP_REG_DC03                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 6)
//#define TI_OTP_REG_DC04                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 8)
//#define TI_OTP_REG_DC05                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 10)
//#define TI_OTP_REG_DC06                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 12)
//#define TI_OTP_REG_DC07                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 14)
//#define TI_OTP_REG_DC08                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 16)
//#define TI_OTP_REG_DC09                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 18)
//#define TI_OTP_REG_DC10                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 20)
//#define TI_OTP_REG_DC11                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 22)
//#define TI_OTP_REG_DC12                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 24)
//#define TI_OTP_REG_DC13                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 26)
//#define TI_OTP_REG_DC14                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 28)
//#define TI_OTP_REG_DC15                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 30)
//#define TI_OTP_REG_DC16                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 32)
//#define TI_OTP_REG_DC17                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 34)
//#define TI_OTP_REG_DC18                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 36)
//#define TI_OTP_REG_DC19                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 38)
//#define TI_OTP_REG_DC20                    *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 40)
//#define TI_OTP_REG_PERCNF1                *(volatile uint32_t *)(TI_OTP_DCX_REG_BASE_ADDRESS + 42)
//

typedef Uint16 (* uint16fptr)();
extern  uint16fptr GetWordData;

//
//#define DEVICE_CAL_LOCATION        0x70280
//#define C1BROM_DEVCAL (void   (*)(void))(DEVICE_CAL_LOCATION)
//
//#define OTP_CPU_ID_VERSION                (Uint16)(*(volatile Uint16 *)(0x7026D))
//
//// bit 1,0 = 01 means enable PLL, else don't
//// bit 7:2 = PLL divider to use - default will be all 1's, but PLL wont be used when default
//// bit 9,8 = 01 means, ENABLE IPC in WAIT boot mode, else nope.
//// bits 31:24 = 0x5A means the WORD is good/valid else not valid or not good
//
//#define OTP_BOOT_CONFIGURE_WORD            (Uint32)(*(volatile Uint32 *)(Uint32)(0x703EE))
//
//
//#define OTP_BOOT_ESCAPE_TABLE_END        0x703EC
//
////extern void otp_func_refs();
//#define TI_OTP_C1BROM_ESCAPE_POINT_15            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-28))
//#define TI_OTP_C1BROM_ESCAPE_POINT_14            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-26))
//#define TI_OTP_C1BROM_ESCAPE_POINT_13            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-24))
//#define TI_OTP_C1BROM_ESCAPE_POINT_12            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-22))
//#define TI_OTP_C1BROM_ESCAPE_POINT_11            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-20))
//#define TI_OTP_C1BROM_ESCAPE_POINT_10            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-18))
//#define TI_OTP_C1BROM_ESCAPE_POINT_9            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-16))
//#define TI_OTP_C1BROM_ESCAPE_POINT_8            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-14))
//#define TI_OTP_C1BROM_ESCAPE_POINT_7            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-12))
//#define TI_OTP_C1BROM_ESCAPE_POINT_6            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-10))
//#define TI_OTP_C1BROM_ESCAPE_POINT_5            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-8))
//#define TI_OTP_C1BROM_ESCAPE_POINT_4            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-6))
//#define TI_OTP_C1BROM_ESCAPE_POINT_3            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-4))
//#define TI_OTP_C1BROM_ESCAPE_POINT_2            (Uint32)(*(volatile Uint32 *)((Uint32)(OTP_BOOT_ESCAPE_TABLE_END)-2))
//#define TI_OTP_C1BROM_ESCAPE_POINT_1            (Uint32)(*(volatile Uint32 *)(Uint32)(OTP_BOOT_ESCAPE_TABLE_END))
//
//
//#define TI_OTP_PMM_LC_TRIM_KEY                    (Uint16)(*(volatile Uint16 *)(Uint32)(0x70274))
//#define TI_OTP_PMM_LC_BGSLOPE_VAL_IREF_TRIM        (Uint16)(*(volatile Uint16 *)(Uint32)(0x70270))
//#define TI_OTP_PMM_LC_VMON_TRIM                    (Uint16)(*(volatile Uint16 *)(Uint32)(0x70271))
//#define TI_OTP_PMM_LC_VREG_TRIM                    (Uint32)(*(volatile Uint32 *)(Uint32)(0x70272))
//
//#define TI_OTP_OSC_TRIM_KEY                        (Uint16)(*(volatile Uint16 *)(Uint32)(0x70275))
//#define TI_OTP_OSC_REF_TRIM                        (Uint32)(*(volatile Uint32 *)(Uint32)(0x70276))
//#define TI_OTP_OSC1_TRIM                        (Uint32)(*(volatile Uint32 *)(Uint32)(0x70278))
//#define TI_OTP_OSC2_TRIM                        (Uint32)(*(volatile Uint32 *)(Uint32)(0x7027A))
//
//
///*internal Marcos used by C1-BootROM*/
//
///* MACRO to ACK IPC command
// * should acknowledge both IPC Flag31 and IPC Flag0 if the IPC command is completed successfully
// */
//#define C1BROM_C2C1_IPC_COMMAND_ACK                    IpcRegs.IPCACK.all = 0x80000001
//
//
///* MACROS to NAK asn IPC command with error code
// * Aria will acknowledge only IPC Flag1 if the command is invalid/resulted in error or cannot be executed
// */
//
//#define C1BROM_C2C1_IPC_COMMAND_NAK(error_code)        IpcRegs.IPCACK.bit.IPC0 = 0x01; \
//                                                    IpcRegs.IPCLOCALREPLY = (Uint32)error_code;
//
///*Macros to ACK PIE IPC interrupt*/
//
//#define C1BROM_C2C1_IPC_INT1_ACK                    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1
//
///*macros to check if IPC flags are set properly*/
//
//#define C1BROM_CHECK_C2C1_IPC_COMMAND_FLAGS_SET        ((IpcRegs.IPCSTS.bit.IPC0) && (IpcRegs.IPCSTS.bit.IPC31))
//
//
////typedefs
//
///*below data structure stores the current mtoa ipc command being processed by C-BootROM*/
//
//typedef struct current_c2c1_ipc_status
//{
//    command_status_t        status;        /*status of the current command*/
//    uint32_t                command;    /*current command value*/
//}cur_c2c1_ipc_cmd_sts_t;
//
//
///*function prototypes*/
//extern command_status_t c1brom_c2c1_ipc_set_bits_16(void);
//extern command_status_t c1brom_c2c1_ipc_set_bits_32(void);
//extern command_status_t c1brom_c2c1_ipc_clear_bits_16(void);
//extern command_status_t c1brom_c2c1_ipc_clear_bits_32(void);
//extern command_status_t c1brom_c2c1_ipc_write_data_16(void);
//extern command_status_t c1brom_c2c1_ipc_write_data_32(void);
//extern command_status_t c1brom_c2c1_ipc_read_data_16(void);
//extern command_status_t c1brom_c2c1_ipc_read_data_32(void);
//
//extern command_status_t c1brom_c2c1_ipc_set_bits_protected_16(void);
//extern command_status_t c1brom_c2c1_ipc_set_bits_protected_32(void);
//extern command_status_t c1brom_c2c1_ipc_clear_bits_protected_16(void);
//extern command_status_t c1brom_c2c1_ipc_clear_bits_protected_32(void);
//extern command_status_t c1brom_c2c1_ipc_write_data_protected_16(void);
//extern command_status_t c1brom_c2c1_ipc_write_data_protected_32(void);
//extern command_status_t c1brom_c2c1_ipc_read_data_protected_16(void);
//extern command_status_t c1brom_c2c1_ipc_read_data_protected_32(void);
//
//void c1brom_init_pie_control(void);
//void c1brom_init_pie_vect_table(void);
//
//extern interrupt void c1brom_itrap_isr(void);
//extern interrupt void c1brom_c2c1_ipc_int1_isr(void);
//extern interrupt void c1brom_handle_nmi();
//extern void c1brom_pie_vect_mismatch_handler();
//extern void c1brom_handle_nmi_at_start();
//extern void c1brom_enable_pie_in_boot(uint16_t mode);


//extern uint16_t c1brom_read_otp_bootmode();
//extern uint16_t c1brom_evaluate_bootmode(uint16_t bootmode, uint16_t bootmode_type);
//extern uint16_t c1brom_read_otp_bootpinconfig();
//extern uint16_t c1brom_decode_bootpins(uint16_t pinconfig);

//
// Function Prototypes
//
extern Uint32 I2C_Boot(Uint32  BootMode);
extern Uint32 SCI_Boot(Uint32  BootMode);
extern Uint32 SPI_Boot(Uint32  BootMode);
extern Uint32 DCAN_Boot(Uint32  BootMode);
extern Uint32 SPI_Alternate_IO_Boot();
extern Uint32 Parallel_Boot();
extern Uint32 USB_Boot(Uint16 bootMode);
extern Uint16 SelectBootMode(void);

#endif //C_BOOTROM_H_

//
// End of file
//
