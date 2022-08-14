//#############################################################################
//
// FILE: boost_afe031_f28379d_rx_main.c
//
// TITLE: FSK Receiver on the AFE031
//
//#############################################################################
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
//#############################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include "afe031_config.h"
#include "fsk_corr_detector.h"
#include "fsk_packetization.h"

//
// Enable ADC interrupt service nesting, per TI-WIKI page found here:
// http://processors.wiki.ti.com/index.php/Interrupt_Nesting_on_C28x
//
#include "boostxl_afe031_f28379d_rx_isr.h"
#define ISRS_GROUP1 (M_INT1|M_INT2|M_INT3|M_INT4|M_INT5|M_INT6|M_INT7|M_INT8)

//
// Platform-dependent GPIO
//
#ifdef _LAUNCHXL_F28379D
#define GPIO_RED_LED_LP    34
#define GPIO_BLUE_LED_LP   31
#define GPIO_RED_LED2_BP   4
#define GPIO_BLUE_LED1_BP  5
#endif

//
// Defines
//
#define EPWM1_TIMER_TBPRD  333          // EPwm1 Period register, 333 ==> 300kHz
#define EPWM2_TIMER_TBPRD  10667        // EPwm2 Period register, 10666 ==> 586Hz
#define FSK_BIT_DETECTION_THRESHOLD 0.1 // Bit detection threshold value
#define RX_MESSAGE_TIMEOUT  2000000     // Timeout in uSeconds

#define ADC_PU_SCALE_FACTOR (float)(1.0 / 4096.0)   // ADC Scaling Factor
#define SCALE_ADC_INPUT(adc_input)  (((float)(adc_input))*ADC_PU_SCALE_FACTOR)*2.0

//
// Globals
//
float input;    // adc_input holder variable
volatile FSK_CORR_DETECTOR FSK_struct1; // FSK structure
int16_t rxMessage[RX_MESSAGE_SIZE];  // Buffer to hold received message
volatile uint16_t msgFull;     // Flag to signify that the message buffer is full
uint16_t  message_index;
int16_t packet[NUMBER_OF_WORDS];    // Buffer to hold received packet
int16_t packet_sum;

//
// Function Prototypes
//

// Initialization Prototypes
void AFE_InitGpio(void);
void ConfigureADC(void);
void SetupADCEpwm(Uint16 channel);
void initEPWM1(void);
void initEPWM2(void);
void InitEPwm1Gpio(void);
void initADCSOC(void);

// Receive Handling Prototypes
void Start_Receiving(void);
void Stop_Receiving(void);
void Visual_Indication(void);

// ISR Prototypes
__interrupt void adc_sample_signal(void);
__interrupt void epwm2_isr(void);
__interrupt void cpu_timer2_isr(void);

#ifdef _FLASH
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;
#endif

//
// Main
//
void main(void)
{
    //
    // Initialize System Control:
    //
    InitSysCtrl();

    //
    // For Flash mode, copy designated functions to RAM
    //
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    //
    // Initialize LaunchPad GPIO
    //
    InitGpio();

    //
    // Initialize BoosterPack GPIO
    // IMPORTANT: Wait a second to ensure
    // host PC has enumerated the LaunchPad
    // COM port before AFE is initialized!
    //
    DELAY_US(1000000);
    AFE_InitGpio();

    //
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table
    //
    InitPieVectTable();

    //
    // Map ISR functions
    //
    EALLOW;
    PieVectTable.ADCA1_INT = &adc_sample_signal; // ISR for ADCA
    PieVectTable.EPWM2_INT = &epwm2_isr;         // ISR for EPwm2
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;   // ISR for CPU_timer2
    EDIS;

    //
    //  Initialize CPU Timer
    //
    InitCpuTimers();

    //
    // Configure CPU Timer 2 for desired timeout (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer2, 200, RX_MESSAGE_TIMEOUT);

    //
    // Configure the ADC and power it up
    //
    ConfigureADC();

    //
    // Sync EPwms
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    //
    // Configure ePWMs
    //
    initEPWM1();
    initEPWM2();

    EALLOW;
    InputXbarRegs.INPUT5SELECT = 500; // Setting to a GPIO number above what exists on device, to avoid interference.
    EPwm1Regs.TBCTL.bit.PHSEN = 1;
    EPwm2Regs.TBCTL.bit.PHSEN = 1;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 0;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Sync EPwms
    EPwm1Regs.TBCTL.bit.SWFSYNC = 1; // Sync PWM1 and PWM2 clocks
    EDIS;

    //
    // Setup the ADC for ePWM triggered conversions on channel 1
    //
    SetupADCEpwm(2);    // Boosterpack RX connected to ADCINA2
                        // this ADC_Input may only work on LaunchPad revision Ver. 2 and later
                        // If using older LaunchPad use a different ADCIN

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;  // Enable int1 (EPwm1)
    IER |= M_INT3;  // Enable int3 (EPwm2)
    IER |= M_INT14; //  Enable in14 (CPU_Timer2)

    //
    // Enable PIE interrupt for EPwms
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // EPwm1/ADC
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  // EPwm2

    //
    //Configure AFE
    //
    HAL_afe031Init();

    //
    // Disable AFE TX
    //
    HAL_afe031_txDisable(); // make sure PA is disabled

    //
    // Enable AFE RX mode
    //
    HAL_afe031_rxEnable();

    //
    // NOTE: Enable INT output/flags after initializing DAC/PA.
    //
    HAL_afe031_cfgInt();

    //
    // NOTE: Enable interrupts after initializing AFE031.
    //
    EINT;
    ERTM;

    //
    // GPIO toggle for measuring ISR frequencies, will be half the frequency witnessed if probed
    //
    EALLOW;
    GpioCtrlRegs.GPEMUX1.bit.GPIO139 = 0;   // LP Pin 43, ADC Sampling ISR
    GpioCtrlRegs.GPEDIR.bit.GPIO139 = 1;
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 0;    // LP Pin 47, Oversampling Bit-rate ISR
    GpioCtrlRegs.GPCDIR.bit.GPIO65 = 1;
    EDIS;

    //
    // Set FSK Correlation Detector parameters
    //
    FSK_struct1.bit_freq = 586;
    FSK_struct1.isr_freq = 300000;     // ADC Sampling frequency
    FSK_struct1.mark_freq = 131250;    // Mark Frequency Detected
    FSK_struct1.space_freq = 143750;   // Space Frequency Detected
    FSK_struct1.detection_threshold = FSK_BIT_DETECTION_THRESHOLD; // Set threshold to meet input signal
    // FSK_struct1.bit_detected ==> Watch variable showing detected bit

    FSK_CORR_DETECTOR_INIT(&FSK_struct1); // Initialize FSK structure

    //
    // Initialize received message buffer
    //
    for(message_index = 0; message_index < RX_MESSAGE_SIZE; message_index++)
    {
        rxMessage[message_index] = 0;
    }
    message_index = 0;
    msgFull = 0;

    //
    // Take input indefinitely
    //
    while(1)
    {
        //
        // Begin receiving data
        //
        Start_Receiving();

        //
        // wait for message to be received, timeout if not received within set time
        //
        while(!msgFull){}

        msgFull = 0;    // Clear the msgFull flag

        //
        // Stop receiving data
        //
        Stop_Receiving();

        //
        // Packetize the received message, clears the received message buffer
        //
        Packetize(rxMessage, packet);

        //
        // LED indication of received packet
        //
        Visual_Indication();

        //
        // software breakpoint after receiving full message
        //
        //asm("   ESTOP0"); // Uncomment to receive only one packet
    }
}

//
// Function to Initialize GPIOs/LEDs on BOOSTXL-AFE031
//
void AFE_InitGpio()
{
    // Enable register access
    EALLOW;

    // LaunchPad LEDs
    GPIO_SetupPinMux(GPIO_RED_LED_LP, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(GPIO_RED_LED_LP, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(GPIO_BLUE_LED_LP, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(GPIO_BLUE_LED_LP, GPIO_OUTPUT, GPIO_PUSHPULL);

    // BoosterPack LEDs
    GPIO_SetupPinMux(GPIO_RED_LED2_BP, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(GPIO_RED_LED2_BP, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(GPIO_BLUE_LED1_BP, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(GPIO_BLUE_LED1_BP, GPIO_OUTPUT, GPIO_PUSHPULL);

    // Disable register access
    EDIS;
}

//
// ConfigureADC - Write ADC configurations and power up ADC A
//
void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // 12 bit res

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// initEPWM1 - Function to configure ePWM1 to generate the SOC.
//
void initEPWM1(void)
{
    EALLOW;

    EPwm1Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event

    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;        // Set EPwm1 Timer period

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_FREEZE;    // Freeze counter

    //
    // Setup TBCLK
    //

    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Setup counter mode
    //
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    // 300kHz == TB_DIV1;   586Hz == TB_DIV2
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;       // 300kHz == TB_DIV1;   586Hz == TB_DIV4;

    //
    // Setup shadowing
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on period event
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on every event

    EDIS;
}

//
// initEPWM2 - Function to configure ePWM1 to generate the SOC.
//
void initEPWM2(void)
{
    EALLOW;

    EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;        // Set EPwm1 Timer period

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_FREEZE;    // Freeze counter

    //
    // Setup TBCLK
    //
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Setup counter mode
    //
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;    // 300kHz == TB_DIV1;   586Hz == TB_DIV2
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV4;       // 300kHz == TB_DIV1;   586Hz == TB_DIV4;

    //
    // Setup shadowing
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on period event
    EPwm2Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on every event

    EDIS;
}

//
// SetupADCEpwm - Setup ADC EPWM acquisition window
//
void SetupADCEpwm(Uint16 channel)
{
    Uint16 acqps;

    //
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  // SOC0 will convert pin A2 ==> pin 29 on LaunchXL-F28379D
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;  // sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    // trigger on ePWM1 SOCA/C
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // make sure INT1 flag is cleared
    EDIS;
}

//
// ADC Sampling ISR function
//
#pragma CODE_SECTION(adc_sample_signal,".TI.ramfunc");
__interrupt void adc_sample_signal(void)
{
    //
    // Retrieve ADC input value and scale (0 to 2)
    //
    input = SCALE_ADC_INPUT(AdcaResultRegs.ADCRESULT0);

    //
    // Pass ADC sample to FSK correlation function
    //
    FSK_CORR_DETECTOR_RUN(input);

    //
    // Clear the interrupt flag and issue ACK
    //
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    //
    // Toggle gpio pin for measuring frequency, debug
    //
    //GpioDataRegs.GPETOGGLE.bit.GPIO139 = 1;  // LP Pin 43, uncomment and probe for frequency test
}

//
// Bit-decision ISR function
//
#pragma CODE_SECTION(epwm2_isr,".TI.ramfunc");
__interrupt void epwm2_isr(void)
{
    //
    // Enable ADC ISR Nesting
    //
    uint16_t TempPIEIER;
    TempPIEIER = PieCtrlRegs.PIEIER2.all;
    IER |= M_INT1;
    IER &= MINT1;                         // Set "global" priority
    PieCtrlRegs.PIEIER2.all &= MG1_1;     // Set "group" priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;      // Enable PIE interrupts
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    //
    // Run the FSK Correlation Detector Function
    //
    FSK_CORR_DETECTOR_OverSampl_RUN(&FSK_struct1);

    //
    // See if a mark or space bit is detected
    //
    if(FSK_struct1.bit_detected != 0)
    {
        rxMessage[message_index++] = FSK_struct1.bit_detected; // Save the detected bit in the message buffer

        FSK_struct1.bit_detected = 0; // Clear the detected bit member

        //
        // Set flags when message buffer is full
        //
        if(RX_MESSAGE_SIZE <= message_index)
        {
            message_index = 0;
            msgFull = 1;
        }
    }

    //
    // Clear INT flag for EPwm2
    //
    EPwm2Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    //
    // Restore registers saved for ADC ISR Nesting
    //
    DINT;
    PieCtrlRegs.PIEIER2.all = TempPIEIER;

    //
    // Toggle gpio pin for measuring frequency, debug purposes
    //
    //GpioDataRegs.GPCTOGGLE.bit.GPIO65 = 1; // LP Pin 47
}

//
// cpu_timer2_isr CPU Timer2 ISR, for message timeout
//
#pragma CODE_SECTION(cpu_timer2_isr,".TI.ramfunc");
__interrupt void cpu_timer2_isr(void)
{
    //
    // Set msgFull flag to 1 to timeout
    //
    msgFull = 1;

    //
    // Clear INT flags for EPwm2 and ADC
    //
    EPwm2Regs.ETCLR.bit.INT = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
}

//
//  Begin Receiving input function
//
void Start_Receiving(void)
{
    //
    // Start CPU Timer2 for Message Timeout
    //
    StartCpuTimer2();

    //
    // Start ePWMs to start ISRs
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // Enable SOCA, start of conversion
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // 300kHz == TB_COUNT_UP;   586Hz == TB_COUNT_UPDOWN
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // 300kHz == TB_COUNT_UP;   586Hz == TB_COUNT_UPDOWN
}

//
//  Stop Receiving input function
//
void Stop_Receiving(void)
{
    //
    // Stop CPU Timer2 for Message Timeout
    //
    StopCpuTimer2();

    //
    // Reset CPU Timer2 for Message Timeout
    //
    ReloadCpuTimer2();

    //
    // Stop ePWMs to stop ISRs
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
}

//
// Function for Visual indication of packet received
//
void Visual_Indication(void)
{
    //
    // Turn both LEDs off
    //
    GPIO_WritePin(GPIO_RED_LED2_BP, 1);
    GPIO_WritePin(GPIO_BLUE_LED1_BP, 1);

    //
    // W1,W1,W1 received
    //
    if(packet_sum == packet_1)
    {
        // Toggle blue LED
        GPIO_WritePin(GPIO_BLUE_LED1_BP, 0);
        DELAY_US(250000);
        GPIO_WritePin(GPIO_BLUE_LED1_BP, 1);
    }
    //
    // W0,W0,W0 received
    //
    else if(packet_sum == packet_0)
    {
        // Toggle red LED
        GPIO_WritePin(GPIO_RED_LED2_BP, 0);
        DELAY_US(250000);
        GPIO_WritePin(GPIO_RED_LED2_BP, 1);
    }
    //
    // Continuous Mark
    //
    else if(packet_sum == packet_m)
    {
        // Turn blue LED on
        GPIO_WritePin(GPIO_BLUE_LED1_BP, 0);
    }
    //
    // Continuous Space
    //
    else if(packet_sum == packet_s)
    {
        // Turn red LED on
        GPIO_WritePin(GPIO_RED_LED2_BP, 0);
    }
    //
    // Zero energy or unspecified packet received
    //
    else
    {
        // Leave both LEDs off
    }
}

//
// End of file
//
