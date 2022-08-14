#ifndef F28027SERIAL_H_
#define F28027SERIAL_H_
#include <buffer.h>


#define SYSCLKOUT_KHZ 60000

#define LSPCLK_KHZ (SYSCLKOUT_KHZ/((SysCtrlRegs.LOSPCP.bit.LSPCLK == 0) ? 1 : (SysCtrlRegs.LOSPCP.bit.LSPCLK*2)))

#define LSPCLK_HZ (LSPCLK_KHZ*1000L)

typedef struct serial_s {
	volatile struct buffer_s TX;
	volatile struct SCI_REGS *sci;
	void (*got_data)(struct serial_s *s, char data);
} serial_t;


extern serial_t SerialA;

uint16_t init_serial(serial_t *s, Uint32 baud, void (*got_func)(serial_t *s, char data));
void uninit_serial(serial_t *s);
uint16_t serial_send(serial_t *s, char *data, Uint16 len);
uint16_t serial_printf(serial_t *s, char *fmt, ...);

__interrupt void TXAINT_data_sent(void);
__interrupt void RXAINT_recv_ready(void);


#endif /* F28027SERIAL_H_ */
