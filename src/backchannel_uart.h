#pragma once

#include "typedefs.h"

#define BC_UART_RX_BUF_SIZE 32
#define BC_UART_TX_BUF_SIZE 64
#define COMMAND_SIZE 2
#define COMMAND_COUNT 1

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

/// Print the passed string to UART followed by CR and LF
void bc_print(const char * str);

/// Initialize UART and pins for UART
void bc_uart_init();

/// Send the string str on UART - adds to the bcuart buffer
void bc_uart_tx_str(const char * str);

/// Send data on uart - should be kept less than BC_UART_TX_BUFFER_SIZE or else will
/// wrap around.
void bc_uart_tx_buffer(i8 * data, i8 size);

void bc_uart_tx_byte(i8 byte);

void bc_uart_shutdown();

static void _radio_write();

static void _check_command();

static void _uart_init();

static void _pin_init();

static void _send_next();

static void _add_byte_to_tx_buffer(i8 byte);