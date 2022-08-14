#ifndef __SPIRAM_H__
#define __SPIRAM_H__


/*
 * ############################################################################
 * Things need to change:
 * Define number of data points per variable  Default is 6000 points per variable
 * Change NUM_POINTS if you would like to save more data values per variable.
 * But if NUM_POINTS * 5(variables) greater than 65400 only 65400 points will be saved
 * LED16 Turns after all the data has been collected.  NOTE!!!!!  The given MATLAB functions only
 * work when LED16 is ON (Data collection done).
 * ############################################################################
 */

// NUM_POINTS must be divisible by 200 for given code to work correctly
#define NUM_POINTS 6000
#define BURST         (FIFO_LVL-1)    // burst size should be less than 8
#define TRANSFER      201              // [(MEM_BUFFER_SIZE/FIFO_LVL)-1] [num of floats*2+1]
#define FIFO_LVL      1             // FIFO Interrupt Level
#define NUM_VAR       5  //number of variables to send
#define PACKET_SIZE   200
//number of 200 floats to save
#define NUM_PACKET    (NUM_POINTS/PACKET_SIZE)

__interrupt void DMA_ISR(void);
__interrupt void SPIA_isr(void);

void InitDma();
void setupSpia(void);
float generate_float(uint32_t time);
void saveData(float val1, float val2, float val3, float val4, float val5);
#endif /* __SPIRAM_H__ */


