#include <msp430fr2311.h>
#include <string.h>

#include "backchannel_uart.h"

Backchannel_UART bcuart                = {};
void (*CHECK_FOR_COMMAND_FUNC)(void) = 0;
char COMMANDS[COMMAND_COUNT][COMMAND_SIZE] = {{'A','B'},{'B','A'}};
void (*COMMAND_FUNC[COMMAND_COUNT])(void) = {_command_AB, _command_BA};

void _command_AB()
{
    bc_print("AB_Func!");
}

void _command_BA()
{
    bc_print("BA_Func!");
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

void bc_uart_init()
{
    bcuart.tx_cur_ind = 0;
    bcuart.rx_cur_ind = 0;
    bcuart.tx_end_ind = 0;
    bcuart.rx_end_ind = 0;

    memset(bcuart.tx_buffer, 0, BC_UART_TX_BUF_SIZE);
    memset(bcuart.rx_buffer, 0, BC_UART_RX_BUF_SIZE);
    _pin_init();
    _uart_init();
}

void bc_print(const char * str)
{
    bc_uart_tx_str(str);
    bc_uart_tx_str("\r\n");
}

void bc_uart_tx_str(const char * str)
{
    i8 ready_to_send = (bcuart.tx_cur_ind == bcuart.tx_end_ind);

    const char * cur = str;
    while (*cur != '\0')
    {
        _add_byte_to_tx_buffer(*cur);
        ++cur;
    }

    if (ready_to_send)
        _send_next();
}

void bc_uart_tx_buffer(i8 * data, i8 size)
{
    i8 ready_to_send = (bcuart.tx_cur_ind == bcuart.tx_end_ind);

    for (i8 i = 0; i < size; ++i)
        _add_byte_to_tx_buffer(data[i]);

    if (ready_to_send)
        _send_next();
}

void bc_uart_tx_byte(i8 byte)
{
    i8 ready_to_send = (bcuart.tx_cur_ind == bcuart.tx_end_ind);
    _add_byte_to_tx_buffer(byte);

    if (ready_to_send)
        _send_next();
}

void _add_byte_to_tx_buffer(i8 byte)
{
    bcuart.tx_buffer[bcuart.tx_end_ind] = byte;
    ++bcuart.tx_end_ind;

    // Wrap around if index exceeds max size of buffer
    if (bcuart.tx_end_ind == BC_UART_TX_BUF_SIZE)
        bcuart.tx_end_ind = 0;
}

void _check_command()
{
    i8 cur_ind = 0;
    void (*func_ptr)(void) = 0;
    while (bcuart.rx_cur_ind != bcuart.rx_end_ind)
    {
        for (i8 i = 0; i < COMMAND_COUNT; ++i)
        {
            if (COMMANDS[i][cur_ind] == bcuart.rx_buffer[bcuart.rx_cur_ind])
                func_ptr = COMMAND_FUNC[i];
        }

        ++cur_ind;
        ++bcuart.rx_cur_ind;
        if (bcuart.rx_cur_ind == BC_UART_RX_BUF_SIZE)
            bcuart.rx_cur_ind = 0;
        if (cur_ind == COMMAND_SIZE || !func_ptr)
        {
            bcuart.rx_cur_ind = bcuart.rx_end_ind;
            if (func_ptr)
                func_ptr();
        }
    }
}

void _add_byte_to_rx_buffer(i8 byte)
{
    // For now just echo byte back
    bc_uart_tx_byte(byte);
    if (byte == '\r')
        bc_uart_tx_byte('\n');

    bcuart.rx_buffer[bcuart.rx_end_ind] = byte;
    ++bcuart.rx_end_ind;

    // Wrap around if index exceeds max size of buffer
    if (bcuart.rx_end_ind == BC_UART_RX_BUF_SIZE)
        bcuart.rx_end_ind = 0;
}

void _send_next()
{
    if (bcuart.tx_cur_ind != bcuart.tx_end_ind)
    {
        i8 b = bcuart.tx_buffer[bcuart.tx_cur_ind];
        ++bcuart.tx_cur_ind;

        // Reset back to zero if needed
        if (bcuart.tx_cur_ind == BC_UART_TX_BUF_SIZE)
            bcuart.tx_cur_ind = 0;

        UCA0TXBUF = b;
    }
}

__interrupt_vec(EUSCI_A0_VECTOR) void uart_backchannel_ISR(void)
{
    switch (UCA0IV)
    {
    case (UCIV__NONE):
        break;
    case (UCIV__UCRXIFG):
        _add_byte_to_rx_buffer(UCA0RXBUF);
        if (UCA0RXBUF == '\r')
        {
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
