#include <msp430.h>

#include "node_control.h"
#include "backchannel_uart.h"
#include "radio_nrf24l01p.h"

Node_Control nctrl;

void node_control_init()
{
    _setup_clocks();

    // 105 ms seems to be needed here for reset - 150 for safeness
    // Otherwise UART gets all messed up... not sure why
    _generate_mclk_on_pin();

    bc_init();
    radio_nRF24L01P_init();
    _EINT();
    bc_print_crlf("\n\n\rInitialized");
}

void node_control_run()
{
    while (1)
    {
        if (CHECK_FOR_COMMAND_FUNC)
        {
            CHECK_FOR_COMMAND_FUNC();
            CHECK_FOR_COMMAND_FUNC = 0;
        }

        bc_print_crlf("Entering LPM4");
        LPM4;
        __delay_cycles(160);
        bc_print_crlf("Leaving LPM4");
    }
}

void node_control_shutdown()
{}

void _generate_mclk_on_pin()
{
    // Set MCLK to generate on P2.6 - this is P2SEL.x = 01 and P2DIR.x = 1
    P2DIR |= 0x0040;
    P2SEL1 &= ~0x0040; // Set bit 6 to zero
    P2SEL0 |= 0x0040;  // Set bit 6 to one
}

void _setup_clocks()
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // This should really be after
    PM5CTL0 &= ~LOCKLPM5;

    /*
    This section is to get the DCO clock set correctly quickly, so going in to LPM doesn't prevent it from getting FLL lock
    */
    // Clear DCO and OFIFG flags before disabling FLL
    CSCTL7 &= ~DCOFFG;
    SFRIFG1 &= ~OFIFG;

    // Turn off the FLL
    __bis_SR_register(SCG0);

    // Select REFO as source clock
    CSCTL3 &= ~SELREF;
    CSCTL3 |= SELREF_1;

    // Clear the DCO and MOD registers
    CSCTL0 = 0;
    
    // Unset all dcorsel bits, then set for 16 MHz range
    CSCTL1 &= ~DCORSEL;
    CSCTL1 |= DCORSEL_5;

    // We want FLLD at 1 (16 MHz operation) and FLLN at 487
    // This produces DCOCLK and DCOCLKDIV of (FLLN + 1)(REFO) = (487 + 1)(32768) = 15.990784 MHz
    CSCTL2 &= ~(FLLD | FLLN);
    CSCTL2 |= (FLLD_0 | 0x01E7);

    // We don't need so high for SMCLK - divide by 4.. would divide by 8 but baud rate doesn't work
    CSCTL5 |= DIVS_2;

    // A few no ops to make the above take affect
    _no_operation();
    _no_operation();
    _no_operation();

    // Re-enable FLL
    __bic_SR_register(SCG0);

    // Poll until the FLL has reached a lock!
    while ((CSCTL7 & FLLUNLOCK) || (CSCTL7 & DCOFFG))
    {
        // Clear OSC fault flags
        CSCTL7 &= ~DCOFFG;

        // Clear OFIFG fault flag
        SFRIFG1 &= ~OFIFG;
    }
}
