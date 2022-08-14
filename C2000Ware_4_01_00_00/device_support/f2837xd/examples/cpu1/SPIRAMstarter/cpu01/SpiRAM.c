#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "F28379dSerial.h"
#include "SpiRAM.h"

uint16_t SpiRAM_total_data = NUM_PACKET*NUM_VAR*PACKET_SIZE;
typedef union {
    uint32_t address;
    uint16_t parts[2];
    float value;
} float_int;
//
// Globals
//
#pragma DATA_SECTION(spiBuf, ".my_arrs");    // map the TX data to memory
uint16_t spiBuf[201];
uint16_t spi_receive_Buf[201];
float_int to_send[100];     // Send data buffer
// in the data stream to check received data
float_int write_address; //the address to write to
float_int read_result;
float_int read_address;
volatile Uint16 *DMASource;
volatile Uint16 *DMAreceivedest;
int16_t DMAcount = 0;
uint32_t SpiRAM_total_send = 0; //total number send
uint16_t SpiRAM_dummy = 0;//for spi buf
uint16_t SpiRAM_tmp1=0;
uint16_t SpiRAM_tmp2=0;
uint32_t SpiRAM_time_counter = 0; //for generating different floats for testing
char SpiRAM_send_sci[4]; //send the float
uint32_t SpiRAM_spi_total_read = 0; //total number of spi reads
uint16_t SpiRAM_transmit_done = 0;
uint16_t SpiRAM_spi_state = 0;
uint32_t SpiRAM_read_packet_count = 0; //the location to read the packet
uint16_t SpiRAM_spi_read_count = 0; //number of total spi read
uint16_t SpiRAM_buf_len = 0; //number of things in the buffer
uint32_t SpiRAM_time_entered = 0; //for assigning address for the writing with DMA
uint16_t SpiRAM_temp = 0;;
//
// InitDma - DMA setup for both TX and RX channels.
//
void InitDma()
{
    //
    // Initialize DMA
    //
    EALLOW;

    //
    // Perform a hard reset on DMA
    //
    DmaRegs.DMACTRL.bit.HARDRESET = 1;
    __asm (" nop"); // one NOP required after HARDRESET

    //
    // Allow DMA to run free on emulation suspend
    //
    DmaRegs.DEBUGCTRL.bit.FREE = 1;

    EDIS;

    DMASource = (volatile Uint16 *)spiBuf;
    DMAreceivedest = (volatile Uint16 *) spi_receive_Buf;

    EALLOW;
    //source and destination setting
    //    DmaRegs.CH4.SRC_BEG_ADDR_SHADOW = (Uint32)DMASource;
    DmaRegs.CH4.SRC_ADDR_SHADOW = (Uint32)&SpiaRegs.SPIRXBUF;
    //    DmaRegs.CH4.DST_BEG_ADDR_SHADOW = (Uint32)&SpiaRegs.SPITXBUF;
    DmaRegs.CH4.DST_ADDR_SHADOW = (Uint32)DMAreceivedest;

    // burst setting
    DmaRegs.CH4.BURST_SIZE.all = BURST;
    DmaRegs.CH4.SRC_BURST_STEP = 0; //src is spiBuf
    DmaRegs.CH4.DST_BURST_STEP = 1; //dest is SPITXBUF

    //Transfer setting
    DmaRegs.CH4.TRANSFER_SIZE = TRANSFER;
    DmaRegs.CH4.SRC_TRANSFER_STEP = 0; //src is spiBuf
    DmaRegs.CH4.DST_TRANSFER_STEP = 1; //dest is SPITXBUF
    EDIS;

    EALLOW;
    DmaRegs.CH4.MODE.all = 0x8304; //good
    // bit 15        1:      CHINTE, 0=interrupt disabled, 1=interrupt enabled
    // bit 14        0:      DATASIZE, 0=16-bit, 1=32-bit
    // bit 13-12     00:     reserved
    // bit 11        0:      CONTINUOUS, 0=stop, 1=re-init after transfer complete
    // bit 10        0:      ONESHOT, 0=one burst on trigger, 1=all bursts on trigger
    // bit 9         1:      CHINTMODE, 0=start of transfer, 1=end of transfer
    // bit 8         1:      PERINTE, peripheral interrupt trigger enable, 0=disabled, 1=enabled
    // bit 7         0:      OVRINTE, overflow interrupt enable, 0=disabled, 1=enabled
    // bit 6-5       00:     reserved
    // bit 4-0       00005:  Set to channel number

    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH4 = 110; //SPIARX
    DmaRegs.CH4.CONTROL.all = 0x0090; //good
    // bit 15        0:      reserved
    // bit 14        0:      OVRFLG, overflow flag, read-only
    // bit 13        0:      RUNSTS, run status, read-only
    // bit 12        0;      BURSTSTS, burst status, read-only
    // bit 11        0:      TRANSFERSTS, transfer status, read-only
    // bit 10-9      00:     reserved
    // bit 8         0:      PERINTFLG, read-only
    // bit 7         1:      ERRCLR, error clear, 0=no action, 1=clear SYNCERR bit
    // bit 6-5       00:     reserved
    // bit 4         1:      PERINTCLR, periph event clear, 0=no action, 1=clear periph event
    // bit 3         0:      PERINTFRC, periph event force, 0=no action, 1=force periph event
    // bit 2         0:      SOFTRESET, 0=no action, 1=soft reset the channel
    // bit 1         0:      HALT, 0=no action, 1=halt the channel
    // bit 0         0:      RUN, 0=no action, 1=enable the channel
    EDIS;

    EALLOW;
    //source and destination setting
    //    DmaRegs.CH5.SRC_BEG_ADDR_SHADOW = (Uint32)DMASource;
    DmaRegs.CH5.SRC_ADDR_SHADOW = (Uint32)DMASource;
    //    DmaRegs.CH5.DST_BEG_ADDR_SHADOW = (Uint32)&SpiaRegs.SPITXBUF;
    DmaRegs.CH5.DST_ADDR_SHADOW = (Uint32)&SpiaRegs.SPITXBUF;

    // burst setting
    DmaRegs.CH5.BURST_SIZE.all = BURST;
    DmaRegs.CH5.SRC_BURST_STEP = 1; //src is spiBuf
    DmaRegs.CH5.DST_BURST_STEP = 0; //dest is SPITXBUF

    //Transfer setting
    DmaRegs.CH5.TRANSFER_SIZE = TRANSFER;
    DmaRegs.CH5.SRC_TRANSFER_STEP = 1; //src is spiBuf
    DmaRegs.CH5.DST_TRANSFER_STEP = 0; //dest is SPITXBUF
    EDIS;

    EALLOW;
    DmaRegs.CH5.MODE.all = 0x0305; //good
    // bit 15        1:      CHINTE, 0=interrupt disabled, 1=interrupt enabled
    // bit 14        0:      DATASIZE, 0=16-bit, 1=32-bit
    // bit 13-12     00:     reserved
    // bit 11        0:      CONTINUOUS, 0=stop, 1=re-init after transfer complete
    // bit 10        0:      ONESHOT, 0=one burst on trigger, 1=all bursts on trigger
    // bit 9         1:      CHINTMODE, 0=start of transfer, 1=end of transfer
    // bit 8         1:      PERINTE, peripheral interrupt trigger enable, 0=disabled, 1=enabled
    // bit 7         0:      OVRINTE, overflow interrupt enable, 0=disabled, 1=enabled
    // bit 6-5       00:     reserved
    // bit 4-0       00005:  Set to channel number

    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH5 = 109; //SPIATX
    DmaRegs.CH5.CONTROL.all = 0x0090; //good
    // bit 15        0:      reserved
    // bit 14        0:      OVRFLG, overflow flag, read-only
    // bit 13        0:      RUNSTS, run status, read-only
    // bit 12        0;      BURSTSTS, burst status, read-only
    // bit 11        0:      TRANSFERSTS, transfer status, read-only
    // bit 10-9      00:     reserved
    // bit 8         0:      PERINTFLG, read-only
    // bit 7         1:      ERRCLR, error clear, 0=no action, 1=clear SYNCERR bit
    // bit 6-5       00:     reserved
    // bit 4         1:      PERINTCLR, periph event clear, 0=no action, 1=clear periph event
    // bit 3         0:      PERINTFRC, periph event force, 0=no action, 1=force periph event
    // bit 2         0:      SOFTRESET, 0=no action, 1=soft reset the channel
    // bit 1         0:      HALT, 0=no action, 1=halt the channel
    // bit 0         0:      RUN, 0=no action, 1=enable the channel
    EDIS;
}

float results[200];
__interrupt void SPIA_isr(void){
    if (SpiRAM_spi_state == 1) { //this state empty the spirxbuf, write 4 things to spitxbuf to get the data at specified location
        SpiRAM_spi_state = 2;

        GpioDataRegs.GPASET.bit.GPIO19 = 1;
        SpiRAM_dummy = SpiaRegs.SPIRXBUF;
        SpiaRegs.SPIFFRX.bit.RXFFIL = 4;

        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1; //slave select low
        read_address.address = (0x03000000L) | (SpiRAM_read_packet_count*4L*PACKET_SIZE);
        SpiaRegs.SPITXBUF = read_address.parts[1];
        SpiaRegs.SPITXBUF = read_address.parts[0];
        SpiaRegs.SPITXBUF = 0x0300;
        SpiaRegs.SPITXBUF = 0x0000;
        SpiaRegs.SPITXBUF = 0x0000;
        SpiaRegs.SPITXBUF = 0x0000;
    } else if (SpiRAM_spi_state == 2) { //this state read 4 things back, only the latter two are important; then send 2 things to continuously get data
        SpiRAM_spi_state = 3;
        SpiRAM_dummy = SpiaRegs.SPIRXBUF;
        SpiRAM_dummy = SpiaRegs.SPIRXBUF;
        SpiRAM_tmp1 = SpiaRegs.SPIRXBUF;
        SpiRAM_tmp2 = SpiaRegs.SPIRXBUF;
        read_result.parts[0] = SpiRAM_tmp1;//order is flipped
        read_result.parts[1] = SpiRAM_tmp2;
        SpiRAM_send_sci[0] = read_result.parts[0] & 0xFF;
        SpiRAM_send_sci[1] = (read_result.parts[0]>>8) & 0xFF;
        SpiRAM_send_sci[2] = read_result.parts[1] & 0xFF;
        SpiRAM_send_sci[3] = (read_result.parts[1]>>8) & 0xFF;
        serial_sendSCIA(&SerialA, SpiRAM_send_sci, 4);
        SpiaRegs.SPIFFRX.bit.RXFFIL = 2;
        SpiaRegs.SPITXBUF = 0x0000;
        SpiaRegs.SPITXBUF = 0x0000;
        SpiRAM_spi_read_count++;
        SpiRAM_spi_total_read++;
    } else if (SpiRAM_spi_state == 3) { //this state read 2 things and send two things, till the end
        SpiRAM_tmp1 = SpiaRegs.SPIRXBUF;
        SpiRAM_tmp2 = SpiaRegs.SPIRXBUF;
        read_result.parts[0] = SpiRAM_tmp1;//order is flipped
        read_result.parts[1] = SpiRAM_tmp2;
        SpiRAM_send_sci[0] = read_result.parts[0] & 0xFF;
        SpiRAM_send_sci[1] = (read_result.parts[0]>>8) & 0xFF;
        SpiRAM_send_sci[2] = read_result.parts[1] & 0xFF;
        SpiRAM_send_sci[3] = (read_result.parts[1]>>8) & 0xFF;
        serial_sendSCIA(&SerialA, SpiRAM_send_sci, 4);
        //            if (SpiRAM_spi_read_count < 100) {
        results[SpiRAM_spi_read_count%200] = read_result.value;
        //            }
        SpiRAM_spi_read_count++;
        SpiRAM_spi_total_read++;

        if (SpiRAM_spi_read_count < PACKET_SIZE){
            SpiaRegs.SPIFFRX.bit.RXFFIL = 2;
            SpiaRegs.SPITXBUF = 0x0000;
            SpiaRegs.SPITXBUF = 0x0000;

        } else {
            SpiRAM_spi_state = 10;
            GpioDataRegs.GPASET.bit.GPIO19 = 1;
        }
    } else if (SpiRAM_spi_state == 10) {//this state empty the spirxbuf
        SpiRAM_dummy =  SpiaRegs.SPIRXBUF;
        SpiRAM_dummy =  SpiaRegs.SPIRXBUF;
        SpiRAM_dummy =  SpiaRegs.SPIRXBUF;
        SpiRAM_dummy =  SpiaRegs.SPIRXBUF;
        SpiaRegs.SPIFFRX.bit.RXFFIL = 1;

    }
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

//
// local_D_INTCH5_ISR - DMA Channel 5 ISR
//
uint32_t ch5count = 0;
__interrupt void DMA_ISR(void)
{

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7; // ACK to receive more interrupts

//    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    ch5count++;
    return;
}

void setupSpia(void) {

    //set up for SPIRAM
    // When DMA is being used to transfer data from 28379D to SPIRAM best to use SPISTEA mode for chip select.
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 1);  // Set as SPISTEA
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);  // Make GPIO19 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO19 = 1;  //Initially Set GPIO19/SS High so SPIRAM is not selected

    GPIO_SetupPinMux(58, GPIO_MUX_CPU1, 15);  //Set GPIO58 pin to SPISIMOA
    GPIO_SetupPinMux(59, GPIO_MUX_CPU1, 15);  //Set GPIO59 pin to SPISOMIA
    GPIO_SetupPinMux(18, GPIO_MUX_CPU1, 1);  //Set GPIO18 pin to SPICLKA

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Set I/O pin to asynchronous mode recommended for SPI

    EDIS;

    EALLOW;

    SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in Reset

    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;  //This happens to be the mode for both the DAN28027 and
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  //The MPU-9250,  Mode 01.
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master
    SpiaRegs.SPICCR.bit.SPICHAR = 15;  // Set to transmit and receive 16 bits each write to SPITXBUF
    SpiaRegs.SPICTL.bit.TALK = 1;  // Enable transmission
    SpiaRegs.SPIPRI.bit.FREE = 1;  // Free run, continue SPI operation
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;  // Disables the SPI interrupt

    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 2; // Set SCLK bit rate to 1 MHz so 1us period.  SPI base clock is
    // 50MHZ.  And this setting divides that base clock to create SCLK period
    SpiaRegs.SPISTS.all = 0x0000;  // Clear status flags just in case they are set for some reason

    SpiaRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;    // Enable SPI FIFO enhancements
    SpiaRegs.SPIFFTX.bit.TXFIFO =  0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpiaRegs.SPIFFTX.bit.TXFFIENA =1;

    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;   // Disable the RX FIFO Interrupt because first DMA will be used to transfer to SPIRAM

    SpiaRegs.SPIFFCT.bit.TXDLY = 0; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip

    SpiaRegs.SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset

    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;    // Release transmit FIFO from reset.
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;    // Enables SPI interrupt.  !! I don't think this is needed.  Need to Test

    SpiaRegs.SPIFFRX.all=0x2040;             // RX FIFO enabled, clear FIFO int
    SpiaRegs.SPIFFRX.bit.RXFFIL = FIFO_LVL;  // Set RX FIFO level

    SpiaRegs.SPIFFTX.all=0xE040;             // FIFOs enabled, TX FIFO released,
    SpiaRegs.SPIFFTX.bit.TXFFIL = FIFO_LVL;  // Set TX FIFO level
    //    SpiaRegs.SPIFFTX.bit.TXFFIL = 1; //Interrupt Level to 16 words or more received into FIFO causes interrupt.  This is just the initial setting for the register.  Will be changed below

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1; //slave select low
    SpiaRegs.SPITXBUF = 0x0140; //write0x01 sequential mode(100 0000)

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    SpiRAM_temp = SpiaRegs.SPIRXBUF;
    DELAY_US(20);

}

//function to generate things to write into the spiram
float toreturn = 0;
float generate_float(uint32_t time) {
    if (time % 5 == 0) {
        toreturn = time*0.1+0.001;
    } else if (time % 5 == 1) {
        toreturn = time*0.01;
    } else if (time % 5 == 2) {
        toreturn = time*0.001;
    } else if (time % 5 == 3){
        toreturn = time*0.0001;
    } else if (time % 5 == 4) {
        toreturn = time*0.00001;
    }
    return toreturn;
}

//function to write 4 floats into the SPIRAM
void saveData(float val1, float val2, float val3, float val4, float val5){
    uint16_t i;
    if (SpiRAM_transmit_done == 0) {
        to_send[SpiRAM_buf_len].value = val1;
        to_send[SpiRAM_buf_len+1].value = val2;
        to_send[SpiRAM_buf_len+2].value = val3;
        to_send[SpiRAM_buf_len+3].value = val4;
        to_send[SpiRAM_buf_len+4].value = val5;
        SpiRAM_buf_len = SpiRAM_buf_len + NUM_VAR;
        SpiRAM_total_send = SpiRAM_total_send + NUM_VAR;
        SpiRAM_time_counter = SpiRAM_time_counter + NUM_VAR;
        if (SpiRAM_buf_len == 100) {
            // saveData can be restarted by sending command from Matlab.  So make sure when sending data to SPIRAM, DMA needs STE enabled for SS
            GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 1);  // Set as SPISTEA for DMA
            SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;  // Disable SPIRXFF interrupt because using DMA here

            SpiRAM_buf_len = 0;

            write_address.address = (0x02000000L) | (SpiRAM_time_entered*2L*(TRANSFER-1)); //num of floats*4 is the number to multiply
            spiBuf[0] = write_address.parts[1];
            spiBuf[1] = write_address.parts[0];

            for (i = 0; i < 100; i++) {
                spiBuf[2*i+2] = to_send[i].parts[0];
                spiBuf[2*i+3] = to_send[i].parts[1];
            }
//            GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
            EALLOW;
            DmaRegs.CH5.CONTROL.bit.RUN = 1;
            DmaRegs.CH4.CONTROL.bit.RUN = 1;
            EDIS;     // Start SPI TX DMA channel
            SpiRAM_time_entered++;
        }
    }
    if (SpiRAM_total_send > SpiRAM_total_data) {
        SpiRAM_transmit_done = 1;
        SpiRAM_total_send = 0;
        GpioDataRegs.GPFSET.bit.GPIO160 = 1;
    }
}
