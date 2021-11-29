#include <msp430.h>

#include "node_control.h"
#include "radio_nrf24l01p.h"
#include "backchannel_uart.h"
#include "rtc.h"

#define RADIO_MAX_QUEUE_SIZE 14

#define RADIO_UPDATE_TX_RX 0x01
#define RADIO_UPDATE_SPI   0x02

static volatile Ring_Buffer rad_tx = {};
volatile Ring_Buffer rad_rx = {};

static volatile Packet_Callback rx_cback = 0;
static volatile Packet_Callback tx_cback = 0;

static volatile i8 do_update = 0;
static volatile i8 current_config = RADIO_NOT_CONFIGURED;

void _spi_init()
{
    // Set source clock to SMCLK - other bit (UCSSEL0) is don't care
    UCB0CTLW0 |= UCSSEL1;

    // Second half of clock instead of first
    UCB0CTLW0 |= UCCKPH;

    // For this radio, its MSB first
    UCB0CTLW0 |= UCMSB;

    // Set as SPI master
    UCB0CTLW0 |= UCMST;

    // Set bitclock = SM clock - radio requires 0-10 Mbps (0? Thats what the datasheet says - would be interesting)
    // Doesn't specify in datasheet for MSP default values - so explicitly setting to zero
    UCB0BRW = 2;

    // Enable SPI
    UCB0CTLW0 &= ~UCSWRST;

    // Enable TX and RX interrupts, then clear the interrupt flags
    //UCB0IE |= (UCTXIE | UCRXIE);
    //UCB0IFG &= ~(UCTXIFG); // | UCRXIFG);
}

void _pins_init()
{
    // You must either use all P1 SPI pins, or all P2 SPI pins - you can't mix
    // Choose which ones are used by setting USSCIBRMP = 0 for P1 and USSCIBRMP = 1 for P2 in SYSCFG2 register

    // Set P5.0 to CSN for Radio. Tried to use STE - it works for SPI speeds significantly less than MCLK: success of multi byte transfers become dependent
    // on how many cycles your code takes to run - this is because the TX buffer is one byte. The interrupt triggers, you add another, but all of that stuff
    // takes time and if the SPI clock is anywhere close to MCLK, the STE will go high again between multi byte transfers. Using GPIO gives straight forward
    // control. Set high initially - pull low on transmit.
    P5DIR |= BIT0;
    P5OUT |= BIT0;
    LCDPCTL2 &= ~LCDS32;
    P5SEL0 &= ~BIT0;

    // Set P5.1 to USB0CLK (SPI)
    LCDPCTL2 &= ~LCDS33;
    P5SEL0 |= BIT1;

    // Set P5.2 to USB0MOSI (SPI)
    LCDPCTL2 &= ~LCDS34;
    P5SEL0 |= BIT2;

    // Set P5.3 to USB0MISO (SPI)
    // Set a pull up resistor as the radio doesn't drive the pin very well
    LCDPCTL2 &= ~LCDS35;
    P5SEL0 |= BIT3;
    P5DIR &= ~BIT3;
    P5REN |= BIT3;
    P5OUT |= BIT3;

    // Set P1.3 to CE (output)
    // CE is active high - initialize to low
    SYSCFG2 &= ~ADCPCTL3;
    P1SEL0 &= ~BIT3;
    P1DIR |= BIT3;
    P1OUT &= ~BIT3;

    // Set P1.6 to IRQ (input) with ADC off
    // IRQ is active low - we need interrupt to trigger on falling edge
    // Enable pullup resistor
    // Finally enable interrupt, and clear the flag
    SYSCFG2 &= ~ADCPCTL6;
    P1SEL0 &= ~BIT6;
    P1DIR &= ~BIT6;
    P1IES |= BIT6;
    P1REN |= BIT6;
    P1OUT |= BIT6;
    P1IE |= BIT6;
    P1IFG &= ~BIT6;
}

void radio_clock_in(volatile const u8 * data, volatile u8 size)
{
    rb_write_byte(NRF24L01P_CMD_W_TX_PAYLOAD, &rad_tx);
    rb_write(data, size, &rad_tx);
    radio_burst_spi_tx();
}

void radio_clock_out(volatile u8 * data, volatile u8 size)
{
    radio_burst_spi_rx(NRF24L01P_CMD_R_RX_PAYLOAD, RADIO_PAYLOAD_SIZE);
    rb_read(data, size, &rad_rx);
}

i8 radio_get_tx_or_rx()
{
    return current_config;
}

void radio_set_pckt_rx_cb(volatile Packet_Callback cback)
{
    rx_cback = cback;
}

Packet_Callback radio_get_pckt_rx_cb()
{
    return rx_cback;
}

void radio_set_pckt_tx_cb(volatile Packet_Callback cback)
{
    tx_cback = cback;
}

Packet_Callback radio_get_pckt_tx_cb()
{
    return tx_cback;
}

i8 radio_configure(i8 tx_or_rx)
{
    current_config = tx_or_rx;
    if (tx_or_rx)
        return radio_write_register(NRF24L01P_ADDR_CONFIG, NRF24L01P_EN_CRC | NRF24L01P_PWR_UP);
    else
        return radio_write_register(NRF24L01P_ADDR_CONFIG, NRF24L01P_EN_CRC | NRF24L01P_PWR_UP | NRF24L01P_PRIM_RX);
}

void radio_enable()
{
    P1OUT |= BIT3;
}

void radio_disable()
{
    P1OUT &= ~BIT3;
}

void radio_enable_pulse()
{
    P1OUT |= BIT3;
    __delay_cycles(DELAY_OF_11_uS);
    P1OUT &= ~BIT3;
}

i8 radio_flush(i8 tx_or_rx)
{
    i8 cmd = NRF24L01P_CMD_FLUSH_RX;
    if (tx_or_rx)
        cmd = NRF24L01P_CMD_FLUSH_TX;

    rb_write_byte(cmd, &rad_tx);
    radio_burst_spi_tx_rx();
    return rb_read_byte(&rad_rx);
}

i8 radio_clear_interrupts()
{
    return radio_write_register(NRF24L01P_ADDR_STATUS, NRF24L01P_RX_DR | NRF24L01P_TX_DS | NRF24L01P_MAX_RT);
}

void radio_init()
{
    _pins_init();
    _spi_init();

    radio_disable();

    radio_flush(RADIO_TX);

    radio_flush(RADIO_RX);

    radio_clear_interrupts();

    // Set payload length for RX pipe 0 to 32 bytes
    i8 ret1 = radio_write_register(NRF24L01P_ADDR_RX_PW_P0, RADIO_PAYLOAD_SIZE);

    // Turn off auto-retransmission
    i8 ret2 = radio_write_register(NRF24L01P_ADDR_SETUP_RETR, 0);

    // Turn off auto-ack for everything
    i8 ret3 = radio_write_register(NRF24L01P_ADDR_EN_AA, 0);

    // Set the channel
    i8 ret4 = radio_write_register(NRF24L01P_ADDR_RF_CH, RF_CHANNEL);

    bc_print("Set PL Width: ");
    bc_print_byte(ret1,16);
    bc_print("\n\rTurn of RETR: ");
    bc_print_byte(ret2,16);
    bc_print("\n\rDisable ENAA: ");
    bc_print_byte(ret3,16);
    bc_print("\n\rSet RF CH: ");
    bc_print_byte(ret4,16);
    bc_print("\n\r");
}

i8 radio_write_register(i8 regaddr, i8 byte)
{
    rb_write_byte(NRF24L01P_CMD_W_REGISTER | regaddr, &rad_tx);
    rb_write_byte(byte, &rad_tx);
    radio_burst_spi_tx_rx();
    i8 ret = rb_read_byte(&rad_rx);
    rb_flush(&rad_rx);
    return ret;
}

i8 radio_write_register_data(i8 regaddr, i8 * data, i8 size)
{
    rb_write_byte(NRF24L01P_CMD_W_REGISTER | regaddr, &rad_tx);
    rb_write(data, size, &rad_tx);
    radio_burst_spi_tx_rx();
    i8 ret = rb_read_byte(&rad_rx);
    rb_flush(&rad_rx);
    return ret;
}

i8 radio_read_register(i8 regaddr, i8 nbytes)
{
    radio_burst_spi_rx(NRF24L01P_CMD_R_REGISTER | regaddr, nbytes);
    return rb_bytes_available(&rad_rx);
}

void radio_update()
{
    if ((do_update & RADIO_UPDATE_TX_RX) == RADIO_UPDATE_TX_RX)
    {
        do_update &= ~RADIO_UPDATE_TX_RX;
        if (!current_config)
        {
            if (rx_cback)
                rx_cback();
            else
                bc_print_crlf("No RX CB");
        }
        else
        {
            if (tx_cback)
                tx_cback();
            else
                bc_print_crlf("No TX CB");
        }
        i8 ret = radio_clear_interrupts();
#ifdef RADIO_DEBUG_SPI
        bc_print("CI: ");
        bc_print_byte(ret, 16);
        bc_print("\n\r");
#endif
    }
}

static void radio_burst_spi_tx()
{
    // Set CSN to enable radio
    P5OUT &= ~BIT0;

    // While TX IFG is set and there is still data to send in the ring buffer, clear the TX interrupt flag
    // and send the next byte. Once no more data, this will end.
    UCB0IFG &= ~UCRXIFG;
    while (rad_tx.cur_ind != rad_tx.end_ind)
    {
        UCB0IFG &= ~UCTXIFG;
        UCB0TXBUF = rb_read_byte(&rad_tx);
        while (!(UCB0IFG & UCTXIFG))
            ;
    }

    UCB0IFG &= ~UCRXIFG;
    while (!(UCB0IFG & UCRXIFG))
        ;

    // Set CSN again to indicate we are done
    P5OUT |= BIT0;
}

static void radio_burst_spi_tx_rx()
{
    // Set CSN to enable radio
    P5OUT &= ~BIT0;

    // While TX IFG is set and there is still data to send in the ring buffer, clear the TX interrupt flag
    // and send the next byte. Once no more data, this will end.
    UCB0IFG &= ~UCRXIFG;
    while (rad_tx.cur_ind != rad_tx.end_ind)
    {
        UCB0TXBUF = rb_read_byte(&rad_tx);
        while (!(UCB0IFG & UCRXIFG))
            ;
        rb_write_byte(UCB0RXBUF, &rad_rx);
        UCB0IFG &= ~UCRXIFG;
    }

    // Set CSN again to indicate we are done
    P5OUT |= BIT0;
}

void radio_burst_spi_rx(i8 cmd_address, i8 nbytes)
{
    // Set CSN to enable radio
    P5OUT &= ~BIT0;

    // While RX IFG is set and we haven't reached nbytes yet, clear the RX interrupt flag
    // and send the next byte. Once no more data, this will end.
    i8 np1_bytes = nbytes + 1;

    UCB0IFG &= ~UCRXIFG;
    while (np1_bytes)
    {
        // First time, use cmd address, fromcg then on use NOP
        UCB0TXBUF = cmd_address;
        while (!(UCB0IFG & UCRXIFG))
            ;

        if (np1_bytes <= nbytes)
            rb_write_byte(UCB0RXBUF, &rad_rx);
        else
            cmd_address = NRF24L01P_CMD_NOP;

        UCB0IFG &= ~UCRXIFG;
        --np1_bytes;
    }

    // Set CSN again to indicate we are done
    P5OUT |= BIT0;
}

__interrupt_vec(PORT1_VECTOR) void port_2_isr()
{
    switch (P1IV)
    {
    case (P1IV_NONE):
        break;
    case (P1IV_P1IFG0):
        break;
    case (P1IV_P1IFG1):
        break;
    case (P1IV_P1IFG2):
        break;
    case (P1IV_P1IFG3):
        break;
    case (P1IV_P1IFG4):
        break;
    case (P1IV_P1IFG5):
        break;
    case (P1IV_P1IFG6):
        do_update |= RADIO_UPDATE_TX_RX;
        LPM4_EXIT;
        break;
    case (P1IV_P1IFG7):
        break;
    default:
        break;
    }
}