#pragma once

#include "typedefs.h"

#define BC_UART_RX_BUF_SIZE 32
#define BC_UART_TX_BUF_SIZE 64
#define COMMAND_SIZE 2
#define COMMAND_COUNT 2

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
extern void (*CHECK_FOR_COMMAND_FUNC)(void);
extern char COMMANDS[COMMAND_COUNT][COMMAND_SIZE];
extern void (*COMMAND_FUNC[COMMAND_COUNT])(void);

void bc_print(const char * str);

void bc_uart_init();

void bc_uart_tx_str(const char * str);

void bc_uart_tx_buffer(i8 * data, i8 size);

void bc_uart_tx_byte(i8 byte);

void bc_uart_shutdown();

static void _command_AB();

static void _command_BA();

static void _check_command();

static void _uart_init();

static void _pin_init();

static void _send_next();

static void _add_byte_to_tx_buffer(i8 byte);