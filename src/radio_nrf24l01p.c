#include <msp430.h>
#include <stdlib.h>

#include "radio_nrf24l01p.h"
#include "backchannel_uart.h"

Ring_Buffer rad_tx = {};
Ring_Buffer rad_rx = {};

void _spi_init()
{
    // Set source clock to SMCLK - other bit (UCSSEL0) is don't care
    UCB0CTLW0 |= UCSSEL1;

    UCB0CTLW0 |= UCCKPH;
    //UCB0CTLW0 |= UCCKPL;

    // For this radio, its MSB first
    UCB0CTLW0 |= UCMSB;

    // Set as SPI master
    UCB0CTLW0 |= UCMST;

    // // For this radio, slave enabled active low
    // UCB0CTLW0 |= UCMODE_2;

    // Use STE to connect to CSN on radio - slave enable mode
    // UCB0CTLW0 |= UCSTEM;

    // Set bitclock = SM clock - radio requires 0-10 Mbps (0? Thats what the datasheet says - would be interesting)
    // Doesn't specify in datasheet for MSP default values - so explicitly setting to zero
    UCB0BRW = 100;

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

    // Set P1.0 to CSN for Radio. Tried to use STE - it works for SPI speeds significantly less than MCLK: success of multi byte transfers become dependent
    // on how many cycles your code takes to run - this is because the TX buffer is one byte. The interrupt triggers, you add another, but all of that stuff
    // takes time and if the SPI clock is anywhere close to MCLK, the STE will go high again between multi byte transfers. Using GPIO gives straight forward
    // control. Set high initially - pull low on transmit.
    P1DIR |= BIT0;
    P1OUT |= BIT0;

    // P1SEL1 &= ~BIT0;
    // P1SEL0 |= BIT0;


    // Set P1.1 to USB0CLK (SPI): P2SEL1.3 = 0 and P2SEL0.3 = 1
    P1SEL1 &= ~BIT1;
    P1SEL0 |= BIT1;

    // Set P1.3 to USB0SOMI (SPI): P1SEL1.3 = 0 and P1SEL0.3 = 1
    P1SEL1 &= ~BIT3;
    P1SEL0 |= BIT3;
    P1REN |= BIT3;
    //P1DIR &= ~BIT3;
    P1OUT |= BIT3;

    // Set P1.2 to USB0SIMO (SPI): P1SEL1.2 = 0 and P1SEL0.2 = 1
    P1SEL1 &= ~BIT2;
    P1SEL0 |= BIT2;

    // Set P2.1 to CE (output): P2SEL1.1 = 0 and P2SEL0.1 = 0 and P2DIR.1 = 1
    // CE is active high - initialize to low
    P2SEL1 &= ~BIT1;
    P2SEL0 &= ~BIT1;
    P2DIR |= BIT1;
    P2OUT &= ~BIT1;

    // Set P2.0 to IRQ (input): P2SEL1.0 = 0 and P2SEL0.0 = 0 and P2DIR.0 = 0
    // IRQ is active low - we need interrupt to trigger on falling edge - set P2IFG.0=1
    // Enable pullup resistor - first enable with P2REN.0=1 then set as pull up with P2OUT.0=1
    // Finally enable interrupt with P2IE.0=1, then clear the flag that all of this may have made
    P2SEL1 &= ~BIT0;
    P2SEL0 &= ~BIT0;
    P2DIR &= ~BIT0;
    P2IES |= BIT0;
    P2REN |= BIT0;
    P2OUT |= BIT0;
    P2IE |= BIT0;
    P2IFG &= ~BIT0;
}

void radio_nRF24L01P_init()
{
    _pins_init();
    _spi_init();
}

void radio_nRF24L01P_read_register(i8 regaddr)
{
    static i8 cmdword[2] = {0x01, 0xFF};
    //cmdword[0] = regaddr;
    rb_write(cmdword, 2, &rad_tx);
    if (rb_bytes_available(&rad_tx) == 2)
    {
        P1OUT &= ~BIT0;
        _send_next();
    }
}

void radio_nRF24L01P_rx_byte(i8 byte)
{}

void _send_next()
{
    static i8 b = 0;
    if (rad_tx.cur_ind != rad_tx.end_ind)
    {
        b = rad_tx.data[rad_tx.cur_ind];
        ++rad_tx.cur_ind;
        if (rad_tx.cur_ind == RING_BUFFER_SIZE)
            rad_tx.cur_ind = 0;
        UCB0TXBUF = b;
    }
    else
    {
        P1OUT |= BIT0;
    }
}

__interrupt_vec(PORT2_VECTOR) void port_2_isr()
{
    switch (P2IV)
    {
    case (P2IV__P2IFG0):
        bc_print_crlf("IRQ p2.0");
        break;
    case (P2IV__P2IFG1):
        //bc_uart_tx_str("IRQ P2.1!");
        break;
    case (P2IV__P2IFG2):
        //bc_uart_tx_str("IRQ P2.2!");
        break;
    case (P2IV__P2IFG3):
        //bc_uart_tx_str("IRQ P2.3!");
        break;
    case (P2IV__P2IFG4):
        //bc_uart_tx_str("IRQ P2.4!");
        break;
    case (P2IV__P2IFG5):
        //bc_uart_tx_str("IRQ P2.5!");
        break;
    case (P2IV__P2IFG6):
        //bc_uart_tx_str("IRQ P2.6!");
        break;
    case (P2IV__P2IFG7):
        //bc_uart_tx_str("IRQ P2.7!");
        break;
    default:
        //bc_uart_tx_str("IRQ None!");
        break;
    }
}

__interrupt_vec(EUSCI_B0_VECTOR) void spi_isr()
{
    static i8 b = 0;
    char buff[3];
    switch (UCB0IV)
    {
    case (UCIV__NONE):
        break;
    case (UCIV__UCRXIFG):
        b = UCB0RXBUF;
        itoa(b, buff, 16);
        bc_print_crlf(buff);
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
