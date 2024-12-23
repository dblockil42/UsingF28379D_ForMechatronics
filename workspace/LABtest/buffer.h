
#ifndef __BUFFER_H__
#define __BUFFER_H__

#define BUF_SIZESCIA		(1024)
#define BUF_SIZESCIB		(136)
#define BUF_SIZESCIC		(136)
#define BUF_SIZESCID		(512)

// For SCIA
typedef volatile struct bufferSCIA_s {
	volatile char buf[BUF_SIZESCIA];
	volatile Uint16 head, tail, size;
} bufferSCIA_t;

extern inline void init_bufferSCIA(bufferSCIA_t *b)
{
	b->size = b->head = b->tail = 0;
}
#define buf_clearSCIA(b)			init_bufferSCIA(b)

#ifdef _FLASH
#pragma CODE_SECTION(buf_writeSCIA_1, ".TI.ramfunc");
#endif
extern inline uint16_t buf_writeSCIA_1(bufferSCIA_t *b, char data)
{
	if (b->size == BUF_SIZESCIA) return 2;
	b->buf[b->head] = data;
	b->head = (b->head+1)%BUF_SIZESCIA;
	b->size++;
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_readSCIA_1, ".TI.ramfunc");
#endif
extern inline uint16_t buf_readSCIA_1(bufferSCIA_t *b, Uint16 offset, char *data)
{
	if (b->size == 0 || b->size < offset) return 3;
	*data = b->buf[ (b->tail+offset)%BUF_SIZESCIA ];
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_removeSCIA, ".TI.ramfunc");
#endif
extern inline uint16_t buf_removeSCIA(bufferSCIA_t *b, Uint16 len)
{
	len = (len > b->size) ? b->size : len;
	b->tail = (b->tail+len)%BUF_SIZESCIA;
	b->size -= len;
	return len;
}

// For SCIB
typedef volatile struct bufferSCIB_s {
	volatile char buf[BUF_SIZESCIB];
	volatile Uint16 head, tail, size;
} bufferSCIB_t;

extern inline void init_bufferSCIB(bufferSCIB_t *b)
{
	b->size = b->head = b->tail = 0;
}
#define buf_clearSCIB(b)			init_bufferSCIB(b)

#ifdef _FLASH
#pragma CODE_SECTION(buf_writeSCIB_1, ".TI.ramfunc");
#endif
extern inline uint16_t buf_writeSCIB_1(bufferSCIB_t *b, char data)
{
	if (b->size == BUF_SIZESCIB) return 2;
	b->buf[b->head] = data;
	b->head = (b->head+1)%BUF_SIZESCIB;
	b->size++;
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_readSCIB_1, ".TI.ramfunc");
#endif
extern inline uint16_t buf_readSCIB_1(bufferSCIB_t *b, Uint16 offset, char *data)
{
	if (b->size == 0 || b->size < offset) return 3;
	*data = b->buf[ (b->tail+offset)%BUF_SIZESCIB ];
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_removeSCIB, ".TI.ramfunc");
#endif
extern inline uint16_t buf_removeSCIB(bufferSCIB_t *b, Uint16 len)
{
	len = (len > b->size) ? b->size : len;
	b->tail = (b->tail+len)%BUF_SIZESCIB;
	b->size -= len;
	return len;
}

// For SCIC
typedef volatile struct bufferSCIC_s {
	volatile char buf[BUF_SIZESCIC];
	volatile Uint16 head, tail, size;
} bufferSCIC_t;



extern inline void init_bufferSCIC(bufferSCIC_t *b)
{
	b->size = b->head = b->tail = 0;
}
#define buf_clearSCIC(b)			init_bufferSCIC(b)

#ifdef _FLASH
#pragma CODE_SECTION(buf_writeSCIC_1, ".TI.ramfunc");
#endif
extern inline uint16_t buf_writeSCIC_1(bufferSCIC_t *b, char data)
{
	if (b->size == BUF_SIZESCIC) return 2;
	b->buf[b->head] = data;
	b->head = (b->head+1)%BUF_SIZESCIC;
	b->size++;
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_readSCIC_1, ".TI.ramfunc");
#endif
extern inline uint16_t buf_readSCIC_1(bufferSCIC_t *b, Uint16 offset, char *data)
{
	if (b->size == 0 || b->size < offset) return 3;
	*data = b->buf[ (b->tail+offset)%BUF_SIZESCIC ];
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_removeSCIC, ".TI.ramfunc");
#endif
extern inline uint16_t buf_removeSCIC(bufferSCIC_t *b, Uint16 len)
{
	len = (len > b->size) ? b->size : len;
	b->tail = (b->tail+len)%BUF_SIZESCIC;
	b->size -= len;
	return len;
}

// For SCID
typedef volatile struct bufferSCID_s {
	volatile char buf[BUF_SIZESCID];
	volatile Uint16 head, tail, size;
} bufferSCID_t;

extern inline void init_bufferSCID(bufferSCID_t *b)
{
	b->size = b->head = b->tail = 0;
}
#define buf_clearSCID(b)			init_bufferSCID(b)

#ifdef _FLASH
#pragma CODE_SECTION(buf_writeSCID_1, ".TI.ramfunc");
#endif
extern inline uint16_t buf_writeSCID_1(bufferSCID_t *b, char data)
{
	if (b->size == BUF_SIZESCID) return 2;
	b->buf[b->head] = data;
	b->head = (b->head+1)%BUF_SIZESCID;
	b->size++;
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_readSCID_1, ".TI.ramfunc");
#endif
extern inline uint16_t buf_readSCID_1(bufferSCID_t *b, Uint16 offset, char *data)
{
	if (b->size == 0 || b->size < offset) return 3;
	*data = b->buf[ (b->tail+offset)%BUF_SIZESCID ];
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_removeSCID, ".TI.ramfunc");
#endif
extern inline uint16_t buf_removeSCID(bufferSCID_t *b, Uint16 len)
{
	len = (len > b->size) ? b->size : len;
	b->tail = (b->tail+len)%BUF_SIZESCID;
	b->size -= len;
	return len;
}

#endif /* __BUFFER_H__ */


