#pragma once
#include "typedefs.h"

#ifdef RADIO_DEBUG_RX_PACKET
#define RING_BUFFER_SIZE 256
#else
#define RING_BUFFER_SIZE 128
#endif

typedef struct
{
    i8 data[RING_BUFFER_SIZE];
    i8 cur_ind;
    i8 end_ind;
} Ring_Buffer;

i8 rb_bytes_available(Ring_Buffer * buf);

i8 rb_write_str(const char * str, Ring_Buffer * buf);

void rb_write(i8 * data, i8 size, Ring_Buffer * buf);

i8 rb_read(i8 * dest_buf, i8 max_size, Ring_Buffer * source_buf);

i8 rb_read_str(char * str, i8 str_buffer_max_size, Ring_Buffer * source_buf);

void rb_flush(Ring_Buffer * buf);

void rb_clear(Ring_Buffer * buf);

void rb_write_byte(i8 byte, Ring_Buffer * buf);
