#include <msp430fr2311.h>
#include "radio_nrf24l01p.h"

Radio_nRF24L01P radio = {};

void radio_nRF24L01P_init()
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

    // Set P1.3 to USB0SIMO (SPI): P1SEL1.2 = 0 and P1SEL0.2 = 1
    P1SEL1 &= ~BIT2;
    P1SEL0 |= BIT2;

    // Set P2.1 to CE (output): P2SEL1.1 = 0 and P2SEL0.1 = 0 and P2DIR.1 = 1
    // CE is active high - initialize to low
    P2SEL1 &= ~BIT1;
    P2SEL0 &= ~BIT1;
    P2DIR |= BIT1;
    P2OUT &= ~BIT1;

    // Set P2.0 to IRQ (input): P2SEL1.0 = 0 and P2SEL0.0 = 0 and P2DIR.0 = 0
    // IRQ is active low
    P2SEL1 &= ~BIT0;
    P2SEL0 &= ~BIT0;
    P2DIR &= ~BIT0;
    P2
    P2OUT &= ~BIT1;

}

void radio_nRF24L01P_tx_byte(i8 byte)
{

}

void radio_nRF24L01P_rx_byte(i8 byte)
{

}

void radio_nRF24L01P_shutdown()
{

}
