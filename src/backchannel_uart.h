#pragma once

#include "typedefs.h"
#include "ring_buffer.h"

#define COMMAND_SIZE 2
#define COMMAND_COUNT 5

extern Ring_Buffer bc_tx;
extern Ring_Buffer bc_rx;
extern int TX_MODE;

extern void (*CHECK_FOR_COMMAND_FUNC)(void);
extern char COMMANDS[COMMAND_COUNT][COMMAND_SIZE];
extern void (*COMMAND_FUNC[COMMAND_COUNT])(void);

void bc_init();

void bc_print(const char * str);

void bc_print_crlf(const char * str);

void bc_print_byte(i8 byte, i8 base);

void bc_print_raw(i8 byte);

static void _radio_get_config();

static void _radio_power_up_rx();

static void _radio_power_up_tx();

static void _radio_get_freq_channel();

static void _radio_power_down();

static void _check_command();

static void _uart_init();

static void _pin_init();

static inline void _send_next();