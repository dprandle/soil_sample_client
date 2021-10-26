#include <msp430fr2311.h>
#include <string.h>

#include <stdlib.h>

//#include "util.h"
#include "radio_nrf24l01p.h"
#include "backchannel_uart.h"

void (*CHECK_FOR_COMMAND_FUNC)(void)       = 0;
char COMMANDS[COMMAND_COUNT][COMMAND_SIZE] = {{'C', 'F'}};
void (*COMMAND_FUNC[COMMAND_COUNT])(void)  = {_radio_write};

Ring_Buffer bc_tx = {};
Ring_Buffer bc_rx = {};

void _radio_write()
{
    bc_print_crlf("--- Exec ---");
    radio_nRF24L01P_read_register(EN_AA);
    bc_print_crlf("\n\r--- Done ---");
}

void _uart_init()
{
    // Set source clock to SMCLCK
    UCA0CTLW0 |= UCSSEL1; // could also set UCSSEL0 - SMCLK is selected with 0x0080 or 0x00C0

    // Prescalar as selected for this SMCLK
    UCA0BRW = 0x0003;

    // Set UCBRS to AD (high byte), set UCBRF to 6 (high nibble of low byte)
    // And enable oversampling with UCOS16 (lowest bit)
    UCA0MCTLW = 0xAD61;

    // Enable UART
    UCA0CTLW0 &= ~UCSWRST;

    // Enable TX and RX interrupts
    UCA0IE = UCRXIE | UCTXIE;
    UCA0IFG &= ~(UCRXIFG | UCTXIFG);
}

void _pin_init()
{
    // Enable UCA0TXD on pin 1.7
    P1SEL1 &= ~BIT7; // Set bit 7 to 0
    P1SEL0 |= BIT7;  // Set bit 7 to 1

    // Enable UCA0RXD on pin 1.6
    P1SEL1 &= ~BIT6; // Set bit 6 to 0
    P1SEL0 |= BIT6;  // Set bit 6 to 1
}

void bc_init()
{
    _pin_init();
    _uart_init();
}

void bc_print_crlf(const char * str)
{
    bc_print(str);
    bc_print("\r\n");
}

void bc_print(const char * str)
{
    i8 cnt = rb_write_str(str, &bc_tx);
    if (cnt == rb_bytes_available(&bc_tx))
        _send_next();
}

void bc_print_byte(i8 byte, i8 base)
{
    i8 buf[8];
    itoa(byte,buf,base);
    i8 cnt = rb_write_str(buf, &bc_tx);
    if (cnt == rb_bytes_available(&bc_tx))
        _send_next();
}

void bc_print_raw(i8 byte)
{
    rb_write(&byte, 1, &bc_tx);
    if (rb_bytes_available(&bc_tx) == 1)
        _send_next();
}

void bc_print_int(i16 val, i8 base)
{
    char buf[16];
    itoa(val,buf,base);
    i8 cnt = rb_write_str(buf, &bc_tx);
    if (cnt == rb_bytes_available(&bc_tx))
        _send_next();
}

void _check_command()
{
    i8 cur_ind             = 0;
    void (*func_ptr)(void) = 0;
    while (bc_rx.cur_ind != bc_rx.end_ind)
    {
        for (i8 i = 0; i < COMMAND_COUNT; ++i)
        {
            if (COMMANDS[i][cur_ind] == bc_rx.data[bc_rx.cur_ind])
                func_ptr = COMMAND_FUNC[i];
        }

        ++cur_ind;
        ++bc_rx.cur_ind;
        if (bc_rx.cur_ind == RING_BUFFER_SIZE)
            bc_rx.cur_ind = 0;
        
        if (cur_ind == COMMAND_SIZE || !func_ptr)
        {
            bc_rx.cur_ind = bc_rx.end_ind;
            if (func_ptr)
                func_ptr();
        }
    }
}

void _send_next()
{
    i8 b;
    if (rb_read(&b, 1, &bc_tx))
        UCA0TXBUF = b;
}

__interrupt_vec(EUSCI_A0_VECTOR) void uart_backchannel_ISR(void)
{
    i8 byte = 0;
    switch (UCA0IV)
    {
    case (UCIV__NONE):
        break;
    case (UCIV__UCRXIFG):
        
        byte = UCA0RXBUF;
        rb_write(&byte, 1, &bc_rx);
        
        // Echo with newline if \r
        bc_print_raw(byte);
        
        if (byte == '\r')
        {
            bc_print_raw('\n');
            CHECK_FOR_COMMAND_FUNC = _check_command;
            LPM4_EXIT;
        }
        break;
    case (UCIV__UCTXIFG):
        _send_next();
        break;
    case (UCIV__UCSTTIFG):
        break;
    case (UCIV__UCTXCPTIFG):
        break;
    default:
        break;
    }
}
