//###########################################################################
//
// FILE:   can_loopback_interrupts.c
//
// TITLE:  Example to demonstrate basic CAN setup and use.
//
//! \addtogroup cpu01_example_list
//! <h1>CAN External Loopback with Interrupts (can_loopback_interrupts)</h1>
//!
//! This example, using driverlib, shows the basic setup of CAN in order to 
//! transmit and receive messages on the CAN bus.  The CAN peripheral is 
//! configured to transmit messages with a specific CAN ID.  A message is then 
//! transmitted once per second, using a simple delay loop for timing.  The 
//! message that is sent is a 4 byte message that contains an incrementing 
//! pattern.  A CAN interrupt handler is used to confirm message transmission 
//! and count the number of messages that have been sent.
//!
//! This example sets up the CAN controller in External Loopback test mode.
//! Data transmitted is visible on the CAN0TX pin and can be received with
//! an appropriate mailbox configuration.
//!
//! This example uses the following interrupt handlers:\n
//! - INT_CANA0 - CANIntHandler
//!
//
//###########################################################################
//
// $Release Date: $
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

//
// Included Files
//
#include "F28x_Project.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "driverlib/can.h"

//
// Globals
//
volatile unsigned long g_ulTxMsgCount = 0; // A counter that keeps track of the
                                           // number of times the TX interrupt
                                           // has occurred, which should match
                                           // the number of TX messages that
                                           // were sent.
volatile unsigned long g_ulRxMsgCount = 0;
unsigned long u32CanAErrorStatus;
volatile unsigned long g_bErrFlag = 0;     // A flag to indicate that some
                                           // transmission error occurred.
tCANMsgObject sTXCANMessage;
tCANMsgObject sRXCANMessage;
unsigned char ucTXMsgData[4] ={ 0x00, 0x00, 0x00, 0x00 };
unsigned char ucRXMsgData[4];
unsigned long u32CntTXMsgData = 0x12345678;

//
// CANIntHandler - This function is the interrupt handler for the CAN
//                 peripheral. It checks for the cause of the interrupt, and
//                 maintains a count of all messages that have been
//                 transmitted.
//
interrupt void
CANIntHandler(void)
{
    unsigned long ulStatus;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ulStatus = CANIntStatus(CANA_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ulStatus == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ulStatus = CANStatusGet(CANA_BASE, CAN_STS_CONTROL);

        //
        //Check to see if an error occurred.
        //
        if(((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 7) &&
           ((ulStatus  & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 0))
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            g_bErrFlag = 1;
        }
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ulStatus == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANA_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulTxMsgCount++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // receiving messages.
    //
    else if(ulStatus == 2)
    {
        //
        // Get the received message
        //
        CANMessageGet(CANA_BASE, 2, &sRXCANMessage, true);

        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CANA_BASE, 2);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ulRxMsgCount++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    CANGlobalIntClear(CANA_BASE, CAN_GLB_INT_CANINT0);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

//
// Main
//
int main(void)
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xD_Gpio.c file and
    // illustrates how to set the GPIO to its default state.
    //
    InitGpio();
    GPIO_SetupPinMux(73, GPIO_MUX_CPU1, 3); // GPIO 73 - XCLKOUT
    GPIO_SetupPinOptions(73, GPIO_OUTPUT, GPIO_PUSHPULL); // SYSCLOCK/8 = 25MHz
    GPIO_SetupPinMux(30, GPIO_MUX_CPU1, 1); // GPIO30 - CANRXA
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 1); // GPIO31 - CANTXA
    GPIO_SetupPinOptions(30, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);

    //
    // Initialize the CAN controller
    //
    CANInit(CANA_BASE);

    //
    // Setup CAN to be clocked off the PLL output clock (500kHz CAN-Clock)
    //
    CANClkSourceSelect(CANA_BASE, 0);

    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() is used to determine the clock rate that
    // is used for clocking the CAN peripheral.  This can be replaced with a
    // fixed value if you know the value of the system clock, saving the extra
    // function call.  For some parts, the CAN peripheral is clocked by a fixed
    // 8 MHz regardless of the system clock in which case the call to
    // SysCtlClockGet() should be replaced with 8000000.  Consult the data
    // sheet for more information about CAN peripheral clocking.
    //
    CANBitRateSet(CANA_BASE, 200000000, 500000);

    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    CANIntEnable(CANA_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    // Register interrupt handler in RAM vector table
    //
    EALLOW;
    PieVectTable.CANA0_INT = CANIntHandler;
    EDIS;

    //
    // Enable the CAN interrupt on the processor (PIE).
    //
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;
    IER |= 0x0100;   // M_INT9
    EINT;

    //
    // Enable test mode and select external loopback
    //
    HWREG(CANA_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
    HWREG(CANA_BASE + CAN_O_TEST) = CAN_TEST_EXL;

    //
    // Enable the CAN for operation.
    //
    CANEnable(CANA_BASE);

    //
    // Enable CAN Global Interrupt line0
    //
    CANGlobalIntEnable(CANA_BASE, CAN_GLB_INT_CANINT0);

    //
    // Initialize the message object that will be used for sending CAN
    // messages.  The message will be 4 bytes that will contain an incrementing
    // value.  Initially it will be set to 0x12345678.
    //
    ucTXMsgData[0] = (u32CntTXMsgData>>24) & 0xFF;
    ucTXMsgData[1] = (u32CntTXMsgData>>16) & 0xFF;
    ucTXMsgData[2] = (u32CntTXMsgData>>8) & 0xFF;
    ucTXMsgData[3] = (u32CntTXMsgData) & 0xFF;
    sTXCANMessage.ui32MsgID = 1;                      // CAN message ID - use 1
    sTXCANMessage.ui32MsgIDMask = 0;                  // no mask needed for TX
    sTXCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;  // enable interrupt on TX
    sTXCANMessage.ui32MsgLen = sizeof(ucTXMsgData);   // size of message is 8
    sTXCANMessage.pucMsgData = ucTXMsgData;           // ptr to message content

    //
    // Initialize the message object that will be used for receiving CAN
    // messages.
    //
    *(unsigned long *)ucRXMsgData = 0;
    sRXCANMessage.ui32MsgID = 1;                      // CAN message ID - use 1
    sRXCANMessage.ui32MsgIDMask = 0;                  // no mask needed for TX
    sRXCANMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE;  // enable interrupt on RX
    sRXCANMessage.ui32MsgLen = sizeof(ucRXMsgData);   // size of message is 8
    sRXCANMessage.pucMsgData = ucRXMsgData;           // ptr to message content

    //
    // Setup the message object being used to receive messages
    //
    CANMessageSet(CANA_BASE, 2, &sRXCANMessage, MSG_OBJ_TYPE_RX);

    //
    // Send the CAN message using object number 1 (not the same thing as
    // CAN ID, which is also 1 in this example).  This function will cause
    // the message to be transmitted right away.
    //
    ucTXMsgData[0] = (u32CntTXMsgData>>24) & 0xFF;
    ucTXMsgData[1] = (u32CntTXMsgData>>16) & 0xFF;
    ucTXMsgData[2] = (u32CntTXMsgData>>8) & 0xFF;
    ucTXMsgData[3] = (u32CntTXMsgData) & 0xFF;

    //
    // Enter loop to send messages.  A new message will be sent once per
    // second.  The 4 bytes of message content will be treated as an unsigned
    // long and incremented by one each time.
    //
    for(;;)
    {
        //
        // Check the error flag to see if errors occurred
        //
        if(g_bErrFlag)
        {
            asm("   ESTOP0");
        }

        if(g_ulTxMsgCount == g_ulRxMsgCount)
        {
            //
            // Transmit Message
            //
            CANMessageSet(CANA_BASE, 1, &sTXCANMessage, MSG_OBJ_TYPE_TX);
        }
        else
        {
            g_bErrFlag = 1;
        }

        //
        // Now wait 1 second before continuing
        //
        DELAY_US(1000*1000);

        //
        // Increment the value in the transmitted message data.
        //
        ucTXMsgData[0] += 0x1;
        ucTXMsgData[1] += 0x1;
        ucTXMsgData[2] += 0x1;
        ucTXMsgData[3] += 0x1;
        
        //
        // Reset data if exceeds a byte
        //
        if(ucTXMsgData[0] > 0xFF)
        {
            ucTXMsgData[0] = 0;  
        }
        if(ucTXMsgData[1] > 0xFF)
        {
            ucTXMsgData[1] = 0; 
        }     
        if(ucTXMsgData[2] > 0xFF)
        {
            ucTXMsgData[2] = 0; 
        }     
        if(ucTXMsgData[3] > 0xFF)
        {
            ucTXMsgData[3] = 0; 
        }             
    }
}

//
// End of file
//
