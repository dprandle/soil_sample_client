#include <msp430.h>
#include <string.h>
#include <stdlib.h>

#include "radio_nrf24l01p.h"
#include "backchannel_uart.h"

#define COMMAND_SIZE 2

volatile Ring_Buffer bc_tx = {};
volatile Ring_Buffer bc_rx = {};

volatile void (*CHECK_FOR_COMMAND_FUNC)(void) = 0;

#ifndef RADIO_DEBUG_SPI
#define COMMAND_COUNT 1

char COMMANDS[COMMAND_COUNT][COMMAND_SIZE] = {{'s', 'i'}};

void (*COMMAND_FUNC[COMMAND_COUNT])(void) = {_sample_func};

inline void _sample_func()
{
    bc_print_crlf("Sample");
}
#else
#define COMMAND_COUNT 29
char COMMANDS[COMMAND_COUNT][COMMAND_SIZE] = {{'C', 'I'}, {'R', 'X'}, {'T', 'X'}, {'c', 'f'}, {'C', 'H'}, {'c', 'h'}, {'R', 'F'}, {'C', 'W'}, {'r', 'f'}, {'R', 'T'},
                                              {'r', 't'}, {'A', 'W'}, {'a', 'w'}, {'P', 'E'}, {'p', 'e'}, {'R', 'A'}, {'r', 'a'}, {'T', 'A'}, {'t', 'a'}, {'A', 'A'},
                                              {'a', 'a'}, {'D', 'P'}, {'d', 'p'}, {'F', 'T'}, {'f', 't'}, {'f', 'i'}, {'p', 's'}, {'r', 'p'}, {'E', 'N'}};

void (*COMMAND_FUNC[COMMAND_COUNT])(void) = {
    _radio_clear_interrupts,      _radio_set_config_power_up_rx,   _radio_set_config_power_up_tx,   _radio_get_config,
    _radio_set_freq_channel,      _radio_get_freq_channel,         _radio_set_rf_setup_normal,      _radio_set_rf_setup_carrier,
    _radio_get_rf_setup,          _radio_set_retransmission,       _radio_get_retransmission,       _radio_set_address_width,
    _radio_get_address_width,     _radio_set_rx_pipe_enable,       _radio_get_rx_pipe_enable,       _radio_set_rx_pipe_0_address,
    _radio_get_rx_pipe_0_address, _radio_set_tx_address,           _radio_get_tx_address,           _radio_set_auto_ack,
    _radio_get_auto_ack,          _radio_set_pipe_dynamic_payload, _radio_get_pipe_dynamic_payload, _radio_set_features,
    _radio_get_features,          _radio_get_fifo_status,          _radio_get_packet_stats,         _radio_get_rx_power,
    _radio_toggle_enable};

const char * debug_tmr_packet = "abcdefghijklmnopqrstuvwxyz[];',.";
char debug_rcv_packet[RADIO_PAYLOAD_SIZE] = {};

void tx_packet_sent()
{
    bc_print_crlf("Packet Sent");
    P1OUT &= ~BIT4;
}

void rx_packet_rcvd()
{
    bc_print_crlf("Packet Received");
    bzero(debug_rcv_packet, RADIO_PAYLOAD_SIZE);
    radio_clock_out(debug_rcv_packet, RADIO_PAYLOAD_SIZE);
    bc_print_crlf(debug_rcv_packet);
}


void _radio_clear_interrupts()
{
    bc_print("ClearInts: ");    
    i8 ret = radio_write_register(NRF24L01P_ADDR_STATUS, NRF24L01P_RX_DR | NRF24L01P_TX_DS | NRF24L01P_MAX_RT);
    bc_print_byte(ret,16);
    bc_print("\n\r");
}

void _radio_set_config_power_up_rx()
{
    radio_set_pckt_rx_cb(rx_packet_rcvd);
    bc_print("RX Mode: ");
    i8 ret = radio_configure(RADIO_RX);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_set_config_power_up_tx()
{
    radio_set_pckt_tx_cb(tx_packet_sent);
    bc_print("TX Packet: ");
    i8 ret = radio_configure(RADIO_TX);
    bc_print_byte(ret,16);
    bc_print("\n\r");
    radio_clock_in(debug_tmr_packet, RADIO_PAYLOAD_SIZE);
    radio_enable_pulse();
}

void _radio_get_config()
{
    bc_print("Get CFG: ");
    radio_read_register(NRF24L01P_ADDR_CONFIG, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_freq_channel()
{
    bc_print("Set CH: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_RF_CH, 101);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_freq_channel()
{
    bc_print("Get CH: ");
    radio_read_register(NRF24L01P_ADDR_RF_CH, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");

}

void _radio_set_rf_setup_normal()
{
    bc_print("Setup RF: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_RF_SETUP, NRF24L01P_RF_PWR_3);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_set_rf_setup_carrier()
{
    bc_print("CW: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_RF_SETUP, NRF24L01P_CONT_WAVE | NRF24L01P_PLL_LOCK | NRF24L01P_RF_PWR_3);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_rf_setup()
{
    bc_print("Get RF: ");
    radio_read_register(NRF24L01P_ADDR_RF_SETUP, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_retransmission()
{
    bc_print("Set ARETR: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_SETUP_RETR, NRF24L01P_ARD_500 | 10);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_retransmission()
{
    bc_print("Get ARETR: ");
    radio_read_register(NRF24L01P_ADDR_SETUP_RETR, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_address_width()
{
    bc_print("Set AddrW: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_SETUP_RETR, NRF24L01P_AW_3);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_address_width()
{
    bc_print("Get AddrW: ");
    radio_read_register(NRF24L01P_ADDR_SETUP_RETR, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_rx_pipe_enable()
{
    bc_print("Set PipeEn: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_EN_RXADDR, NRF24L01P_ERX_P0);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_rx_pipe_enable()
{
    bc_print("Get PipeEn: ");
    radio_read_register(NRF24L01P_ADDR_EN_RXADDR, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_rx_pipe_0_address()
{
    bc_print("Set P0 Addr: ");
    i8 ret = radio_write_register_data(NRF24L01P_ADDR_RX_ADDR_P0, "node1", 5);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_rx_pipe_0_address()
{
    bc_print("Get P0 Addr: ");
    radio_read_register(NRF24L01P_ADDR_RX_ADDR_P0, 5);
    for (int i = 0; i < 5; ++i)
        bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_tx_address()
{
    bc_print("Set TX Addr: ");
    i8 ret = radio_write_register_data(NRF24L01P_ADDR_TX_ADDR, "node1", 5);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_tx_address()
{
    bc_print("Get TX Addr: ");
    radio_read_register(NRF24L01P_ADDR_TX_ADDR, 5);
    for (int i = 0; i < 5; ++i)
        bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_auto_ack()
{
    bc_print("Set AA: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_EN_AA, NRF24L01P_ENAA_P0);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_auto_ack()
{
    bc_print("Get AA: ");
    radio_read_register(NRF24L01P_ADDR_EN_AA, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_pipe_dynamic_payload()
{
    bc_print("Set DynP: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_DYNPD, 0);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_pipe_dynamic_payload()
{
    bc_print("Get DynP: ");
    radio_read_register(NRF24L01P_ADDR_DYNPD, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_set_features()
{
    bc_print("Set Feat: ");
    i8 ret = radio_write_register(NRF24L01P_ADDR_FEATURE, 0);
    bc_print_byte(ret,16);
    bc_print("\n\r");

}

void _radio_get_features()
{
    bc_print("Get Feat: ");
    radio_read_register(NRF24L01P_ADDR_FEATURE, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_get_fifo_status()
{
    bc_print("Get FIFO_St: ");
    radio_read_register(NRF24L01P_ADDR_FIFO_STATUS, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_get_packet_stats()
{
    bc_print("Get PcktSts: ");
    radio_read_register(NRF24L01P_ADDR_OBSERVE_TX, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_get_rx_power()
{
    bc_print("Get RxPwr: ");
    radio_read_register(NRF24L01P_ADDR_RPD, 1);
    bc_print_byte(rb_read_byte(&rad_rx),16);
    bc_print("\n\r");
}

void _radio_toggle_enable()
{
    P1OUT ^= BIT3;
    if ((P1OUT & BIT3) == BIT3)
        bc_print_crlf("Enabled");
    else
        bc_print_crlf("Disabled");
}
#endif

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
    UCA0IFG &= ~UCRXIFG;
    UCA0IE = UCRXIE;
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

void bc_update()
{
    if (CHECK_FOR_COMMAND_FUNC)
    {
        CHECK_FOR_COMMAND_FUNC();
        CHECK_FOR_COMMAND_FUNC = 0;
    }
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
    rb_write_str(str, &bc_tx);
    _transmit_burst();
}

void bc_print_byte(i8 byte, i8 base)
{
    i8 buf[8];
    itoa(byte, buf, base);
    rb_write_str(buf, &bc_tx);
    _transmit_burst();
}

void bc_print_int(i16 byte, i8 base)
{
    i8 buf[16];
    itoa(byte, buf, base);
    rb_write_str(buf, &bc_tx);
    _transmit_burst();
}

void bc_print_raw(i8 byte)
{
    rb_write_byte(byte, &bc_tx);
    _transmit_burst();
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

    if (running_mask > 0 && running_mask != -1 && cur_ind == COMMAND_SIZE)
    {
        cur_ind = 0;
        while ((running_mask >> cur_ind) > 1)
            ++cur_ind;

        COMMAND_FUNC[cur_ind]();
    }
}

static void _transmit_burst()
{
    // While TX IFG is set and there is still data to send in the ring buffer, clear the TX interrupt flag
    // and send the next byte. Once no more data, this will end.
    while (bc_tx.cur_ind != bc_tx.end_ind)
    {
        UCA0IFG &= ~UCTXIFG;
        UCA0TXBUF = rb_read_byte(&bc_tx);
        while (!(UCA0IFG & UCTXIFG))
            ;
    }
}

__interrupt_vec(USCI_A0_VECTOR) void uart_backchannel_ISR(void)
{
    volatile i8 byte = 0;
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
        break;
    case (USCI_UART_UCSTTIFG):
        break;
    case (USCI_UART_UCTXCPTIFG):
        break;
    default:
        break;
    }
}
