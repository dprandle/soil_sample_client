#pragma once

#include "typedefs.h"

#define BC_UART_RX_BUF_SIZE 32
#define BC_UART_TX_BUF_SIZE 64

extern int BC_DOUBLE_PRINT_PRECISION;

typedef struct
{
    i8 tx_buffer[BC_UART_TX_BUF_SIZE];
    i8 rx_buffer[BC_UART_RX_BUF_SIZE];
    i8 tx_cur_ind;
    i8 tx_end_ind;
    i8 rx_cur_ind;
    i8 rx_end_ind;
} Backchannel_UART;

extern Backchannel_UART bcuart;

void bc_uart_init();

void bc_uart_tx_str(const char * str);

void bc_uart_tx_byte(i8 byte);

void bc_uart_rx_byte(i8 byte);

void bc_uart_shutdown();

static void _uart_init();

static void _pin_init();

static void _send_next();

static void _add_byte_to_buffer(i8 byte);
