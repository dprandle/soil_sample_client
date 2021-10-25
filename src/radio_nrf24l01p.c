#include <msp430fr2311.h>
#include <stdlib.h>

#include "radio_nrf24l01p.h"
#include "backchannel_uart.h"

Radio_nRF24L01P radio = {};


void _spi_init()
{
    // Set source clock to SMCLK - other bit (UCSSEL0) is don't care
    UCB0CTLW0 |= UCSSEL1;
    
    // For this radio, its MSB first
    UCB0CTLW0 |= UCMSB;

    // Set as SPI master
    UCB0CTLW0 |= UCMST;

    // For this radio, slave enabled active low
    UCB0CTLW0 |= UCMODE_2;

    // Use STE to connect to CSN on radio - slave enable mode
    UCB0CTLW0 |= UCSTEM;

    // Set bitclock = SM clock - radio requires 0-10 Mbps (0? Thats what the datasheet says - would be interesting)
    // Doesn't specify in datasheet for MSP default values - so explicitly setting to zero
    UCB0BRW = 0;

    // Enable SPI
    UCB0CTLW0 &= ~UCSWRST;

    // Enable TX and RX interrupts, then clear the interrupt flags
    UCB0IE |= (UCTXIE | UCRXIE);
    UCB0IFG &= ~(UCTXIFG | UCRXIFG);
}

void _pins_init()
{
    // Set P1.0 to UCB0STE (SPI): P1SEL1.0 = 0 and P1SEL0.0 = 1
    P1SEL1 &= ~BIT0;
    P1SEL0 |= BIT0;

    // Set P2.3 to USB0CLK (SPI): P2SEL1.3 = 0 and P2SEL0.3 = 1
    P2SEL1 &= ~BIT3;
    P2SEL0 |= BIT3;

    // Set P1.3 to USB0SOMI (SPI): P1SEL1.3 = 0 and P1SEL0.3 = 1
    P1SEL1 &= ~BIT3;
    P1SEL0 |= BIT3;

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

void radio_nRF24L01P_tx_byte(i8 byte)
{
    UCB0TXBUF = byte;
}

void radio_nRF24L01P_rx_byte(i8 byte)
{

}

void radio_nRF24L01P_shutdown(){}

__interrupt_vec(PORT2_VECTOR) void port_2_isr()
{
    switch(P2IV)
    {
        case(P2IV__P2IFG0):
            bc_uart_tx_str("IRQ p2.0");
            break;
        case(P2IV__P2IFG1):
            //bc_uart_tx_str("IRQ P2.1!");
            break;
        case(P2IV__P2IFG2):
            //bc_uart_tx_str("IRQ P2.2!");
            break;
        case(P2IV__P2IFG3):
            //bc_uart_tx_str("IRQ P2.3!");
            break;
        case(P2IV__P2IFG4):
            //bc_uart_tx_str("IRQ P2.4!");
            break;
        case(P2IV__P2IFG5):
            //bc_uart_tx_str("IRQ P2.5!");
            break;
        case(P2IV__P2IFG6):
            //bc_uart_tx_str("IRQ P2.6!");
            break;
        case(P2IV__P2IFG7):
            //bc_uart_tx_str("IRQ P2.7!");
            break;
        default:
            //bc_uart_tx_str("IRQ None!");
            break;
    }
}

__interrupt_vec(EUSCI_B0_VECTOR) void spi_isr()
{
    char buf[5];
    switch (UCB0IV)
    {
    case (UCIV__NONE):
        break;
    case (UCIV__UCRXIFG):
        bc_uart_tx_str("Received byte:");
        itoa(UCB0RXBUF, buf, 16);
        bc_print(buf);
        break;
    case (UCIV__UCTXIFG):
        break;
    case (UCIV__UCSTTIFG):
        break;
    case (UCIV__UCTXCPTIFG):
        break;
    default:
        break;
    }
}
