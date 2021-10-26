#include <msp430fr2311.h>
#include "node_control.h"
#include "backchannel_uart.h"
#include "radio_nrf24l01p.h"

Node_Control nctrl;

void node_control_init()
{

    _setup_clocks();

    // 105 ms seems to be needed here for reset - 150 for safeness
    // Otherwise UART gets all messed up... not sure why
    __delay_cycles(150000);
    _generate_mclk_on_pin();

    bc_init();
    radio_nRF24L01P_init();
    _EINT();
    bc_print_crlf("\n\n\r---- Init complete ----");
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
        __delay_cycles(250);
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

    // Set the DCOTRIM bits back to default and the DCORSEL0 to 2 MHz (also default)
    // and set the modulation to enabled
    CSCTL1 = DCOFTRIM0 | DCOFTRIM1 | DCORSEL0 | DISMOD;

    // Set FLLD to 1 (devide DCOCLK by 2) as default, set FLLN to default value
    // This results in DCOCLK = 2.097152 MHz clock, and DCOCLKDIV of 1.048600 MHz
    CSCTL2 = FLLD0 | 0x001F;
}
