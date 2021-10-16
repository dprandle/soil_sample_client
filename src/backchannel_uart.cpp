#include <msp430.h>
#include "backchannel_uart.h"

void Backchannel_UART::init()
{
    // Set source clock to SMCLCK
    UCA0CTLW0 |= UCSSEL1; // could also set UCSSEL0 - SMCLK is selected with 0x0080 or 0x00C0

    // Prescalar as selected for this SMCLK
    UCA0BRW = 0x0003;

    // Set UCBRS to AD (high byte), set UCBRF to 6 (high nibble of low byte)
    // And enable oversampling with UCOS16 (lowest bit)
    UCA0MCTLW = 0xAD61;

    // Enable UCA0TXD on pin 1.7
    P1SEL1 &= ~BIT7; // Set bit 7 to 0
    P1SEL0 |= BIT7;  // Set bit 7 to 1

    // Enable UART
    UCA0CTLW0 &= ~UCSWRST;
}

void Backchannel_UART::shutdown()
{
}