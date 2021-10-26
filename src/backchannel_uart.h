#pragma once

#include "typedefs.h"
#include "ring_buffer.h"

#define COMMAND_SIZE 2
#define COMMAND_COUNT 1

extern Ring_Buffer bc_tx;
extern Ring_Buffer bc_rx;

extern void (*CHECK_FOR_COMMAND_FUNC)(void);
extern char COMMANDS[COMMAND_COUNT][COMMAND_SIZE];
extern void (*COMMAND_FUNC[COMMAND_COUNT])(void);

void bc_init();

void bc_print(const char * str);

void bc_print_crlf(const char * str);

void bc_print_raw(i8 byte);

void bc_print_byte(i8 byte, i8 base);

void bc_print_int(i16 val, i8 base);

static void _radio_write();

static void _check_command();

static void _uart_init();

static void _pin_init();

static void _send_next();