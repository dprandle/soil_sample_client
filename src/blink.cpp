#include <msp430.h>
#include <stdio.h>

#include "typedefs.h"
#include "backchannel_uart.h"

void reset_clocks()
{
    // Set the DCOTRIM bits back to default and the DCORSEL0 to 2 MHz (also default)
    // and set the modulation to enabled
    CSCTL1 = DCOFTRIM0 | DCOFTRIM1 | DCORSEL0 | DISMOD;

    // Set FLLD to 1 (devide DCOCLK by 2) as default, set FLLN to default value
    // This results in DCOCLK = 2.097152 MHz clock, and DCOCLKDIV of 1.048600 MHz
    CSCTL2 = FLLD0 | 0x001F;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;     // Disable the GPIO power-on default high-impedance mode
                              // to activate previously configured port settings
    P1DIR |= 0x01;            // Set P1.0 to output direction

    reset_clocks();
    Backchannel_UART bcuart;
    bcuart.init();

    // Set MCLK to generate on P2.6 - this is P2SEL.x = 01 and P2DIR.x = 1
    P2DIR |= 0x0040;
    P2SEL1 &= ~0x0040; // Set bit 6 to zero
    P2SEL0 |= 0x0040;  // Set bit 6 to one

    char c = 'a';
    for (;;)
    {
        volatile unsigned int i; // volatile to prevent optimization

        P1OUT ^= 0x01; // Toggle P1.0 using exclusive-OR
        UCA0TXBUF = c;
        ++c;
        if (c > 'z')
            c = 'a';

        i = 60000; // SW Delay
        do
            i--;
        while (i != 0);
    }
}

/*
All interrupts are set the positive edge which is with RFIES = 0 - this is the default condition
*/
#pragma vector = USCI_A0_VECTOR
__interrupt void uart_backchannel_IRQ(void)
{
    switch (UCA0IV)
    {
    case (UCIV__NONE):
        break;
    case (UCIV__UCRXIFG):
        break;
    case (UCIV__UCTXIFG):
        break;
    case (UCIV__UCSTTIFG):
        break;
    case (UCIV__UCTXCPTIFG):
        break;
    }
}
