#pragma once

#include "typedefs.h"

#define BC_UART_RX_BUF_SIZE 32
#define BC_UART_TX_BUF_SIZE 32

typedef struct
{
    ui8 tx_buffer[BC_UART_TX_BUF_SIZE];
    ui8 rx_buffer[BC_UART_RX_BUF_SIZE];
} Backchannel_UART;

extern Backchannel_UART bcuart;

void backchannel_uart_init();
void backchannel_uart_shutdown();
