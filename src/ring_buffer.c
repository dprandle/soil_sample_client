#include "ring_buffer.h"

i8 rb_bytes_available(volatile Ring_Buffer * buf)
{
    if (buf->end_ind < buf->cur_ind)
        return buf->end_ind + (RING_BUFFER_SIZE - buf->cur_ind);
    else
        return buf->end_ind - buf->cur_ind;
}

i8 rb_write_str(volatile const char * str, volatile Ring_Buffer * buf)
{
    i8 ind = 0;
    while (str[ind] != '\0')
    {
        rb_write_byte(str[ind], buf);
        ++ind;
    }
    return ind;
}

void rb_write(volatile const i8 * data, volatile i8 size, volatile Ring_Buffer * buf)
{
    for (i8 i = 0; i < size; ++i)
        rb_write_byte(data[i], buf);
}

i8 rb_read(volatile i8 * dest_buf, volatile i8 max_size, volatile Ring_Buffer * source_buf)
{
    i8 ret = 0;
    while (source_buf->cur_ind != source_buf->end_ind && ret < max_size)
    {
        dest_buf[ret] = source_buf->data[source_buf->cur_ind];
        ++source_buf->cur_ind;

        // Reset back to zero if needed
        if (source_buf->cur_ind == RING_BUFFER_SIZE)
            source_buf->cur_ind = 0;

        ++ret;
    }
    return ret;
}

i8 rb_read_str(volatile char * str, volatile i8 str_buffer_max_size, volatile Ring_Buffer * source_buf)
{
    i8 ret = 0;
    while (source_buf->data[source_buf->cur_ind] != '\0' && ret < str_buffer_max_size)
    {
        str[ret] = source_buf->data[source_buf->cur_ind];
        ++source_buf->cur_ind;

        // Reset back to zero if needed
        if (source_buf->cur_ind == RING_BUFFER_SIZE)
            source_buf->cur_ind = 0;

        ++ret;
    }
    return ret;
}

i8 rb_read_byte(volatile Ring_Buffer * buf)
{
    i8 ret = buf->data[buf->cur_ind];
    ++buf->cur_ind;
    if (buf->cur_ind == RING_BUFFER_SIZE)
        buf->cur_ind = 0;
    return ret;
}

void rb_write_byte(volatile i8 byte, volatile Ring_Buffer * buf)
{
    buf->data[buf->end_ind] = byte;
    ++buf->end_ind;

    // Wrap around if index exceeds max size of buffer
    if (buf->end_ind == RING_BUFFER_SIZE)
        buf->end_ind = 0;
}

void rb_flush(volatile Ring_Buffer * buf)
{
    buf->cur_ind = buf->end_ind;
}

void rb_clear(volatile Ring_Buffer * buf)
{
    for (int i = 0; i < RING_BUFFER_SIZE; ++i)
        buf->data[i] = 0;
    buf->cur_ind = 0;
    buf->end_ind = 0;
}
