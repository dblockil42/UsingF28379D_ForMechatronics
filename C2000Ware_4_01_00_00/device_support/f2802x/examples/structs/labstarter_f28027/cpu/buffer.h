
#ifndef __BUFFER_H__
#define __BUFFER_H__

#define BUF_SIZE		(108)

typedef volatile struct buffer_s {
	volatile char buf[BUF_SIZE];
	volatile Uint16 head, tail, size;
} buffer_t;



extern inline void init_buffer(buffer_t *b)
{
	b->size = b->head = b->tail = 0;
}
#define buf_clear(b)			init_buffer(b)

#ifdef _FLASH
#pragma CODE_SECTION(buf_write_1, "ramfuncs");
#endif
extern inline uint16_t buf_write_1(buffer_t *b, char data)
{
	if (b->size == BUF_SIZE) return 2;
	b->buf[b->head] = data;
	b->head = (b->head+1)%BUF_SIZE;
	b->size++;
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_read_1, "ramfuncs");
#endif
extern inline uint16_t buf_read_1(buffer_t *b, Uint16 offset, char *data)
{
	if (b->size == 0 || b->size < offset) return 3;
	*data = b->buf[ (b->tail+offset)%BUF_SIZE ];
	return 0;
}

#ifdef _FLASH
#pragma CODE_SECTION(buf_remove, "ramfuncs");
#endif
extern inline uint16_t buf_remove(buffer_t *b, Uint16 len)
{
	len = (len > b->size) ? b->size : len;
	b->tail = (b->tail+len)%BUF_SIZE;
	b->size -= len;
	return len;
}

#endif /* __BUFFER_H__ */


