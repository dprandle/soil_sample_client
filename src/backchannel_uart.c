#include <msp430.h>
#include <stdlib.h>

#include "radio_nrf24l01p.h"
#include "backchannel_uart.h"

void (*CHECK_FOR_COMMAND_FUNC)(void) = 0;
char COMMANDS[COMMAND_COUNT][COMMAND_SIZE] = {
    {'C', 'I'}, 
    {'R', 'X'}, 
    {'T', 'X'}, 
    {'c', 'f'}, 
    {'C', 'H'},
    {'c', 'h'}, 
    {'R', 'F'}, 
    {'C', 'W'}, 
    {'r', 'f'}, 
    {'R', 'T'}, 
    {'r', 't'}, 
    {'A', 'W'}, 
    {'a', 'w'}, 
    {'P', 'E'}, 
    {'p', 'e'}, 
    {'R', 'A'}, 
    {'r', 'a'}, 
    {'T', 'A'}, 
    {'t', 'a'}, 
    {'A', 'A'}, 
    {'a', 'a'}, 
    {'D', 'P'}, 
    {'d', 'p'}, 
    {'F', 'T'}, 
    {'f', 't'}, 
    {'f', 'i'}, 
    {'p', 's'}, 
    {'r', 'p'}};

void (*COMMAND_FUNC[COMMAND_COUNT])(void) = {
    _radio_clear_interrupts, 
    _radio_set_config_power_up_rx, 
    _radio_set_config_power_up_tx, 
    _radio_get_config,
    _radio_set_freq_channel,
    _radio_get_freq_channel,
    _radio_set_rf_setup_normal,
    _radio_set_rf_setup_carrier,
    _radio_get_rf_setup,
    _radio_set_retransmission,
    _radio_get_retransmission,
    _radio_set_address_width,
    _radio_get_address_width,
    _radio_set_rx_pipe_enable,
    _radio_get_rx_pipe_enable,
    _radio_set_rx_pipe_0_address,
    _radio_get_rx_pipe_0_address,
    _radio_set_tx_address,
    _radio_get_tx_address,
    _radio_set_auto_ack,
    _radio_get_auto_ack,
    _radio_set_pipe_dynamic_payload,
    _radio_get_pipe_dynamic_payload,
    _radio_set_features,
    _radio_get_features,
    _radio_get_fifo_status,
    _radio_get_packet_stats, 
    _radio_get_rx_power};

Ring_Buffer bc_tx = {};
Ring_Buffer bc_rx = {};
int         TX_MODE = 0;

void _radio_clear_interrupts()
{
    bc_print_crlf("ClearInts");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_STATUS, NRF24L01P_RX_DR | NRF24L01P_TX_DS | NRF24L01P_MAX_RT);
}


void _radio_set_config_power_up_rx()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("RX Mode");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_CONFIG, NRF24L01P_EN_CRC | NRF24L01P_PWR_UP | NRF24L01P_PRIM_RX);
    P1OUT |= BIT3;
}

void _radio_set_config_power_up_tx()
{
    P1OUT &= ~BIT3;
    TX_MODE = 1;
    bc_print_crlf("TX Mode");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_CONFIG, NRF24L01P_EN_CRC | NRF24L01P_PWR_UP);
}

void _radio_get_config()
{
    bc_print_crlf("Get CFG");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_CONFIG, 1);
}


void _radio_set_freq_channel()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("Set CH");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_RF_CH, 101);
}

void _radio_get_freq_channel()
{
    bc_print_crlf("Get CH");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_RF_CH, 1);
}


void _radio_set_rf_setup_normal()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("Setup RF");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_RF_SETUP, NRF24L01P_RF_PWR_3);
}

void _radio_set_rf_setup_carrier()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("CW");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_RF_SETUP, NRF24L01P_CONT_WAVE | NRF24L01P_PLL_LOCK | NRF24L01P_RF_PWR_3);
    P1OUT |= BIT3;
}

void _radio_get_rf_setup()
{
    bc_print_crlf("Get RF");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_RF_SETUP, 1);
}


void _radio_set_retransmission()
{
    bc_print_crlf("Set ARETR");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_SETUP_RETR, NRF24L01P_ARD_500 | 10);
}

void _radio_get_retransmission()
{
    bc_print_crlf("Get ARETR");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_SETUP_RETR, 1);
}


void _radio_set_address_width()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("Set AddrW");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_SETUP_RETR, NRF24L01P_AW_3);
}

void _radio_get_address_width()
{
    bc_print_crlf("Get AddrW");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_SETUP_RETR, 1);
}


void _radio_set_rx_pipe_enable()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("Set PipeEn");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_EN_RXADDR, NRF24L01P_ERX_P0);
}

void _radio_get_rx_pipe_enable()
{
    bc_print_crlf("Get PipeEn");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_EN_RXADDR, 1);
}


void _radio_set_rx_pipe_0_address()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("Set P0 Addr");
    radio_nRF24L01P_write_register_data(NRF24L01P_ADDR_RX_ADDR_P0, "node1", 5);
}

void _radio_get_rx_pipe_0_address()
{
    bc_print_crlf("Get P0 Addr");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_RX_ADDR_P0, 5);
}


void _radio_set_tx_address()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("Set TX Addr");
    radio_nRF24L01P_write_register_data(NRF24L01P_ADDR_TX_ADDR, "node1", 5);
}

void _radio_get_tx_address()
{
    bc_print_crlf("Get TX Addr");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_TX_ADDR, 5);
}


void _radio_set_auto_ack()
{
    P1OUT &= ~BIT3;

    bc_print_crlf("Set AA");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_EN_AA, NRF24L01P_ENAA_P0);
}

void _radio_get_auto_ack()
{
    bc_print_crlf("Get AA");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_EN_AA, 1);
}


void _radio_set_pipe_dynamic_payload()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("Set DynP");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_DYNPD, 0);
}

void _radio_get_pipe_dynamic_payload()
{
    bc_print_crlf("Get DynP");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_DYNPD, 1);
}


void _radio_set_features()
{
    P1OUT &= ~BIT3;
    bc_print_crlf("Set Feat");
    radio_nRF24L01P_write_register(NRF24L01P_ADDR_FEATURE, 0);
}

void _radio_get_features()
{
    bc_print_crlf("Get Feat");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_FEATURE, 1);
}


void _radio_get_fifo_status()
{
    bc_print_crlf("Get FIFO_St");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_FIFO_STATUS, 1);
}

void _radio_get_packet_stats()
{
    bc_print_crlf("Get PcktSts");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_OBSERVE_TX, 1);
}

void _radio_get_rx_power()
{
    bc_print_crlf("Get RxPwr");
    radio_nRF24L01P_read_register(NRF24L01P_ADDR_RPD, 1);
}



void _uart_init()
{
    // Set source clock to SMCLCK
    UCA0CTLW0 |= UCSSEL1; // could also set UCSSEL0 - SMCLK is selected with 0x0080 or 0x00C0

    // The following is for 9600 Baud with 4 MHz clock
    // Prescalar as selected for this SMCLK
    UCA0BRW = 26;

    // Set UCBRS to B6 (high byte), set UCBRF to 0 (high nibble of low byte)
    // And enable oversampling with UCOS16 (lowest bit)
    UCA0MCTLW = 0xB601;

    // Enable UART
    UCA0CTLW0 &= ~UCSWRST;

    // Enable TX and RX interrupts
    UCA0IE = UCRXIE | UCTXIE;
    UCA0IFG &= ~(UCRXIFG | UCTXIFG);
}

void _pin_init()
{
    // Enable UCA0TXD on pin 1.0
    P1SEL0 |= BIT0;
    SYSCFG2 &= ~ADCPCTL0;

    // Enable UCA0RXD on pin 1.1
    P1SEL0 |= BIT1; // Set bit 6 to 1
    SYSCFG2 &= ~ADCPCTL1;
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
    itoa(byte, buf, base);
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

void _check_command()
{
    i8 cur_ind = 0;
    i8 data_cnt = rb_bytes_available(&bc_rx);
    i8 start_ind = bc_rx.cur_ind;

    i64 running_mask = -1;
    while (bc_rx.cur_ind != bc_rx.end_ind)
    {
        if (bc_rx.data[bc_rx.cur_ind] == '\n' || bc_rx.data[bc_rx.cur_ind] == '\r')
        {
            bc_rx.cur_ind = bc_rx.end_ind;
            break;
        }

        i64 cur_mask = 0;
        for (i8 i = 0; i < COMMAND_COUNT; ++i)
        {
            if (COMMANDS[i][cur_ind] == bc_rx.data[bc_rx.cur_ind])
                cur_mask |= ((i64)1) << i;
        }

        running_mask &= cur_mask;

        ++cur_ind;
        ++bc_rx.cur_ind;
        if (bc_rx.cur_ind == RING_BUFFER_SIZE)
            bc_rx.cur_ind = 0;
    }

    if (running_mask > 0 && running_mask != -1)
    {
        TX_MODE = 0;
        cur_ind = 0;
        while ((running_mask >> cur_ind) > 1)
            ++cur_ind;
        
        COMMAND_FUNC[cur_ind]();
    }
    else if (TX_MODE)
    {
        rb_write_byte(NRF24L01P_CMD_W_TX_PAYLOAD, &rad_tx);
        while (start_ind != bc_rx.end_ind)
        {
            rb_write_byte(bc_rx.data[start_ind], &rad_tx);
            ++start_ind;
            if (start_ind == RING_BUFFER_SIZE)
                start_ind = 0;
        }
        radio_nRF24L01P_burst_spi_tx();
        P1OUT |= BIT3;
        __delay_cycles(200);
        P1OUT &= ~BIT3;
    }
}

inline void _send_next()
{
    static i8 b = 0;
    b = bc_tx.data[bc_tx.cur_ind];
    ++bc_tx.cur_ind;
    if (bc_tx.cur_ind == RING_BUFFER_SIZE)
        bc_tx.cur_ind = 0;
    UCA0TXBUF = b;
}

__interrupt_vec(USCI_A0_VECTOR) void uart_backchannel_ISR(void)
{
    i8 byte = 0;
    switch (UCA0IV)
    {
    case (USCI_NONE):
        break;
    case (USCI_UART_UCRXIFG):
        byte = UCA0RXBUF;
        rb_write_byte(byte, &bc_rx);

        // Echo with newline if \r
        if (byte == '\r')
        {
            bc_print("\r\n");
            CHECK_FOR_COMMAND_FUNC = _check_command;
            LPM4_EXIT;
        }
        else
        {
            bc_print_raw(byte);
        }
        break;
    case (USCI_UART_UCTXIFG):
        if (bc_tx.cur_ind != bc_tx.end_ind)
            _send_next();
        break;
    case (USCI_UART_UCSTTIFG):
        break;
    case (USCI_UART_UCTXCPTIFG):
        break;
    default:
        break;
    }
}
