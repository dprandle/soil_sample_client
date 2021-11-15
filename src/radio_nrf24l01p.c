#include <msp430.h>

#include "radio_nrf24l01p.h"
#include "backchannel_uart.h"

Ring_Buffer rad_tx = {};
Ring_Buffer rad_rx = {};

static i8 current_command             = -1;
static i8 command_register            = -1;
static i8 response_bytes_received     = 0;
static i8 response_bytes_expected     = 0;
void (*HANDLE_RADIO_RX_COMMAND)(void) = 0;

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
    UCB0IE |= (UCTXIE | UCRXIE);
    UCB0IFG &= ~(UCTXIFG | UCRXIFG);
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
    P5SEL0 &= !BIT0;

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

void radio_nRF24L01P_init()
{
    _pins_init();
    _spi_init();

    // Just for now - power up radio
}

void radio_nRF24L01P_write_register(i8 regaddr, i8 * data, i8 size)
{
    static i8 cmd_write = W_REGISTER;
    rb_write(&cmd_write, 1, &rad_tx);
    rb_write(data, size, &rad_tx);
    if (rb_bytes_available(&rad_tx) == size + 1)
    {
        command_register        = regaddr;
        current_command         = cmd_write;
        response_bytes_received = 0;
        response_bytes_expected = size + 1;
        P1OUT &= ~BIT0;
        _send_next();
    }
}

void radio_nRF24L01P_read_register(i8 regaddr)
{
    static i8 cmdword[] = {W_TX_PAYLOAD, 0x55, 0X55, 0X55, 0X55, 0X55, 0X55, 0X55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
                           0x55,         0X55, 0X55, 0X55, 0X55, 0X55, 0X55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    //cmdword[0] = regaddr;
    rb_write(cmdword, 33, &rad_tx);
    if (rb_bytes_available(&rad_tx) == 33)
    {
        radio_nRF24L01P_burst_spi_tx();
        // scooby = 8;
        // P1OUT &= ~BIT0;
        // _send_next();
    }
}

void radio_nRF24L01P_rx_byte(i8 byte)
{}

inline void _send_next()
{
    static i8 b = 0;
    b           = rad_tx.data[rad_tx.cur_ind];
    ++rad_tx.cur_ind;
    if (rad_tx.cur_ind == RING_BUFFER_SIZE)
        rad_tx.cur_ind = 0;
    UCB0TXBUF = b;
}

void radio_nRF24L01P_burst_spi_tx()
{
    bc_print_crlf("Here");

    // Disable TX and RX interrupts
    UCB0IE &= ~(UCTXIE | UCRXIE);

    // Set CSN to enable radio
    P1OUT &= ~BIT0;

    // While TX IFG is set and there is still data to send in the ring buffer, clear the TX interrupt flag
    // and send the next byte. Once no more data, this will end.
    do
    {
        // Clear both TX and RX flags
        UCB0IFG &= ~(UCTXIFG | UCRXIFG);
        _send_next();
    } while (rad_tx.cur_ind != rad_tx.end_ind && (UCB0IFG & UCTXIFG));

    // Re-enable the interrupt flags, first clearing the TX flag - want to run the RX ISR so don't clear it
    UCB0IFG &= ~UCTXIFG;

    // Set that we are expecting 1 byte, let the RX ISR clear the CSN
    UCB0IE |= (UCTXIE | UCRXIE);

    // Wait until the last byte is done sending before changing CSN back. Essentially wait until the RX interrupt is run
    // before putting the CSN back.
    while (UCB0IFG & UCRXIFG)
        ;

    // Set CSN again to indicate we are done
    P1OUT |= BIT0;
}

__interrupt_vec(PORT2_VECTOR) void port_2_isr()
{
    switch (P2IV)
    {
    case (P2IV_NONE):
        break;
    case (P2IV_P2IFG0):
        bc_print_crlf("IRQ p2.0");
        break;
    case (P2IV_P2IFG1):
        //bc_uart_tx_str("IRQ P2.1!");
        break;
    case (P2IV_P2IFG2):
        //bc_uart_tx_str("IRQ P2.2!");
        break;
    case (P2IV_P2IFG3):
        //bc_uart_tx_str("IRQ P2.3!");
        break;
    case (P2IV_P2IFG4):
        //bc_uart_tx_str("IRQ P2.4!");
        break;
    case (P2IV_P2IFG5):
        //bc_uart_tx_str("IRQ P2.5!");
        break;
    case (P2IV_P2IFG6):
        //bc_uart_tx_str("IRQ P2.6!");
        break;
    case (P2IV_P2IFG7):
        //bc_uart_tx_str("IRQ P2.7!");
        break;
    default:
        //bc_uart_tx_str("IRQ None!");
        break;
    }
}

void _check_rx_radio()
{
    bc_print("Doing something..\n\r");
    while (rad_rx.cur_ind != rad_rx.end_ind)
    {
        ++rad_rx.cur_ind;
        if (rad_rx.cur_ind == RING_BUFFER_SIZE)
            rad_rx.cur_ind = 0;
    }
}

__interrupt_vec(USCI_B0_VECTOR) void spi_isr()
{
    static i8 b = 0;
    char      buff[3];
    switch (UCB0IV)
    {
    case (USCI_NONE):
        break;
    case (USCI_SPI_UCRXIFG):
        b = UCB0RXBUF;

        if (response_bytes_expected != 0)
        {
            ++response_bytes_received;
            if (response_bytes_received == 1)
            {
                bc_print("\nStatus:");
                bc_print_byte(b, 16);
            }
            rb_write_byte(b, &rad_rx);

            if (response_bytes_received == response_bytes_expected)
            {
                P1OUT |= BIT0;
                HANDLE_RADIO_RX_COMMAND = _check_rx_radio;
                LPM4_EXIT;
            }
        }
        else if (response_bytes_received > 0)
        {
            bc_print("UH OH!");
            response_bytes_received = 0;
        }
        break;
    case (USCI_SPI_UCTXIFG):
        if (rad_tx.cur_ind != rad_tx.end_ind)
            _send_next();
        break;
    default:
        break;
    }
}
