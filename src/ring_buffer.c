#include "ring_buffer.h"

i8 rb_bytes_available(Ring_Buffer * buf)
{
    if (buf->end_ind < buf->cur_ind)
        return buf->end_ind + (RING_BUFFER_SIZE - buf->cur_ind);
    else
        return buf->end_ind - buf->cur_ind;
}

i8 rb_write_str(const char * str, Ring_Buffer * buf)
{
    i8 ind = 0;
    while (str[ind] != '\0')
    {
        rb_write_byte(str[ind], buf);
        ++ind;
    }
    return ind;
}

void rb_write(i8 * data, i8 size, Ring_Buffer * buf)
{
    for (i8 i = 0; i < size; ++i)
        rb_write_byte(data[i], buf);
}

i8 rb_read(i8 * dest_buf, i8 max_size, Ring_Buffer * source_buf)
{
    i8 ret = 0;
    while (rb_bytes_available(source_buf) != 0 && ret < max_size)
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

i8 rb_read_str(char * str, i8 str_buffer_max_size, Ring_Buffer * source_buf)
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

void rb_write_byte(i8 byte, Ring_Buffer * buf)
{
    buf->data[buf->end_ind] = byte;
    ++buf->end_ind;

    // Wrap around if index exceeds max size of buffer
    if (buf->end_ind == RING_BUFFER_SIZE)
        buf->end_ind = 0;
}
