#pragma once
#include "typedefs.h"

#ifdef RADIO_DEBUG_RX_PACKET
#define RING_BUFFER_SIZE 256
#else
#define RING_BUFFER_SIZE 64
#endif

typedef struct
{
    volatile i8 data[RING_BUFFER_SIZE];
    volatile i8 cur_ind;
    volatile i8 end_ind;
} Ring_Buffer;

i8 rb_bytes_available(volatile Ring_Buffer * buf);

i8 rb_write_str(volatile const char * str, volatile Ring_Buffer * buf);

void rb_write(volatile const i8 * data, volatile i8 size, volatile Ring_Buffer * buf);

i8 rb_read(volatile i8 * dest_buf, volatile i8 max_size, volatile Ring_Buffer * source_buf);

i8 rb_read_byte(volatile Ring_Buffer * buf);

i8 rb_read_str(volatile char * str, volatile i8 str_buffer_max_size, volatile Ring_Buffer * source_buf);

void rb_flush(volatile Ring_Buffer * buf);

void rb_clear(volatile Ring_Buffer * buf);

void rb_write_byte(volatile i8 byte, volatile Ring_Buffer * buf);
