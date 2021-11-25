#include <msp430.h>

#include "node_control.h"
#include "backchannel_uart.h"
#include "radio_nrf24l01p.h"
#include "rtc.h"

Node_Control nctrl;

void _clock_in_payload()
{
    rb_write_str("asdfghjklzxcvbnmqwertyuiop[];,./", &rad_tx);
    radio_burst_spi_tx(NRF24L01P_CMD_W_TX_PAYLOAD);
    bc_print_crlf("TX Packet");
}

void frame_start()
{
    ++nctrl.cur_frame.cur_timeslot;
    if (nctrl.cur_frame.cur_timeslot == nctrl.cur_frame.our_timeslot)
    {
        // The frame after this HAS to be TX - only one rx frame and def not end of frame.. so...
        radio_configure(RADIO_TX);
        _clock_in_payload();
        rtc_set_callback(frame_tx_timeslot);
        rtc_set_tick_cycles(nctrl.t.tx_to_rx);
    }
    else
    {
        rtc_set_callback(frame_rx_timeslot_begin);
        rtc_set_tick_cycles(nctrl.t.rx_on + 2*nctrl.src_t.frame_extra_listen);
    }
}

void frame_prep_start()
{
    rtc_set_interrupt_tick_count(0);
    rtc_set_callback(frame_start);
    toggel_pin_next_isr = 1;
    if (nctrl.cur_frame.cur_timeslot == 1)
        rtc_set_tick_cycles(nctrl.t.rx_packet_to_tx);
    else
        rtc_set_tick_cycles(nctrl.t.rx_packet_to_rx - nctrl.src_t.frame_extra_listen);

}

void frame_end()
{
    toggel_pin_next_isr = 1;
    //P1OUT &= ~BIT5;
    rtc_set_interrupt_tick_count(nctrl.sleep_frame_count * nctrl.timeslots_per_frame - 2);
    rtc_set_callback(frame_prep_start);
    ++nctrl.cur_frame.ind;
    nctrl.cur_frame.cur_timeslot = 0;    
}

void frame_rx_timeslot_end()
{
    P1OUT &= ~BIT4;
    radio_disable();

    // Increment to next timeslot
    ++nctrl.cur_frame.cur_timeslot;
    if (nctrl.cur_frame.cur_timeslot == nctrl.cur_frame.our_timeslot) // TX
    {
        radio_configure(RADIO_TX);
        _clock_in_payload();
        rtc_set_callback(frame_tx_timeslot);

        if ((nctrl.cur_frame.cur_timeslot+1) > nctrl.timeslots_per_frame)
            rtc_set_tick_cycles(nctrl.t.tx_to_end_frame);
        else
            rtc_set_tick_cycles(nctrl.t.tx_to_rx);
    }
    else if (nctrl.cur_frame.cur_timeslot > nctrl.timeslots_per_frame) // END Frame
    {
        rtc_set_callback(frame_end);
        //toggel_pin_next_isr = 1;
        rtc_set_tick_cycles(nctrl.t.timeslot);
    }
    else // RX
    {
        rtc_set_callback(frame_rx_timeslot_begin);
        rtc_set_tick_cycles(nctrl.t.rx_on);
    }
}

void frame_rx_timeslot_begin()
{
    // Turn the RX on - should already be configured for RX
    P1OUT |= BIT4;
    radio_enable();

    // Set the callback to the end of the timeslot assuming we receive no packet
    rtc_set_callback(frame_rx_timeslot_end);

    u8 extra_listen = 0;
    if (nctrl.cur_frame.cur_timeslot == 1)
        extra_listen = nctrl.src_t.frame_extra_listen;

    // Set the timer for after the callback..
    // If we receive a packet, we are gonna restart the timer
    if ((nctrl.cur_frame.cur_timeslot+1) == nctrl.cur_frame.our_timeslot)
        rtc_set_tick_cycles(nctrl.t.rx_no_packet_to_tx - extra_listen);
    else if ((nctrl.cur_frame.cur_timeslot+1) > nctrl.timeslots_per_frame)
        rtc_set_tick_cycles(nctrl.t.rx_no_packet_to_end_frame - extra_listen);
    else
        rtc_set_tick_cycles(nctrl.t.rx_no_packet_to_rx - extra_listen);
}

void frame_tx_timeslot()
{
    P1OUT |= BIT4;
    radio_enable_pulse();

    // Increment to next timeslot
    ++nctrl.cur_frame.cur_timeslot;

    // The next timeslot will either be RX or end of frame
    if (nctrl.cur_frame.cur_timeslot <= nctrl.timeslots_per_frame)
    {
        // Set the callback that should be called next - the time counting to this callback is currently ticking
        rtc_set_callback(frame_rx_timeslot_begin);

        // Set the time for AFTER the above callback is called - so the RTC will transfer to the new time
        // We know the next thing is to turn the RX on
        rtc_set_tick_cycles(nctrl.t.rx_on);
    }
    else
    {
        rtc_set_callback(frame_end);
        rtc_set_tick_cycles(nctrl.t.timeslot);
    }
}

void turn_rx_off()
{
    if (!radio_startup_synced())
    {
        // Setup TX root node packet
        nctrl.cur_frame.ind = 0;
        nctrl.cur_frame.our_timeslot = 1;
        nctrl.cur_frame.cur_timeslot = 1;
        nctrl.cur_frame.timeslots[nctrl.cur_frame.our_timeslot-1].src_addr = 0x01;
        nctrl.cur_frame.timeslots[nctrl.cur_frame.our_timeslot-1].dest_addr = 0x00;
        
        radio_disable();
        radio_configure(RADIO_TX);
        bc_print_crlf("No SYNC TX");

        rtc_set_mode(RTC_MODE_REPEAT);
        rtc_set_tick_cycles(nctrl.t.tx_to_rx);
        rtc_set_interrupt_tick_count(0);
        rtc_start();
        P1OUT |= BIT5;
        frame_tx_timeslot();
    }
}

static void _recalc_rtc_derived_from_source()
{
    nctrl.t.rx_on = (nctrl.src_t.settle + 2*nctrl.src_t.listen) - 1;
    nctrl.t.rx_no_packet_to_rx = (nctrl.src_t.timeslot - (nctrl.src_t.settle + 2*nctrl.src_t.listen)) - 1;
    nctrl.t.rx_no_packet_to_tx = (nctrl.src_t.timeslot - (nctrl.src_t.settle + nctrl.src_t.listen)) - 1;
    nctrl.t.rx_no_packet_to_end_frame = (nctrl.src_t.timeslot - nctrl.src_t.listen)- 1;
    nctrl.t.rx_packet_to_rx = (nctrl.src_t.timeslot - (nctrl.src_t.settle + nctrl.src_t.listen)) - 1;
    nctrl.t.rx_packet_to_tx = (nctrl.src_t.timeslot - nctrl.src_t.settle) - 1;
    nctrl.t.tx_to_rx = (nctrl.src_t.timeslot - nctrl.src_t.listen) - 1;
    nctrl.t.tx_to_end_frame = (nctrl.src_t.timeslot + nctrl.src_t.settle) - 1;
    nctrl.t.timeslot = nctrl.src_t.timeslot - 1;
}

Frame_Info * get_frame_info()
{
    return &nctrl.cur_frame;
}

static void _setup_rtc()
{
    rtc_init();

    // Wait in RX mode for 4 seconds
    rtc_set_interrupt_tick_count(4);

    // Startup wait as RX for packet time
    rtc_set_interrupt_tick_count(nctrl.timeslots_per_frame * nctrl.startup_listen_frame_count); // 16 * 4 = 64 ticks
    rtc_set_mode(RTC_MODE_ONE_SHOT);
    rtc_set_callback(turn_rx_off);
}

void node_control_init()
{
    _setup_clocks();
    _setup_pins();
    _setup_rtc();

    bc_init();
    radio_init();

    // Setup control parameters
    nctrl.timeslots_per_frame = 8;
    nctrl.startup_listen_frame_count = 4; // Wait 4 * 2 == 8 seconds
    nctrl.sleep_frame_count = 2;

    nctrl.src_t.timeslot = 4096;//CRYSTAL_FREQ / 8; // 125 ms
    nctrl.src_t.settle = 5; // 152.59 uS
    double exact_calc = CRYSTAL_PPM * 0.000001 * 2.0 * nctrl.src_t.timeslot * nctrl.timeslots_per_frame;
    nctrl.src_t.listen = (u8)(exact_calc + ROUND_THRESHOLD);
    nctrl.src_t.frame_extra_listen = (u8)(exact_calc * nctrl.sleep_frame_count + ROUND_THRESHOLD) - nctrl.src_t.listen;
    _recalc_rtc_derived_from_source();

    _EINT();
    bc_print("\n\n\rTl, Tlframe, pckt: ");
    bc_print_byte(nctrl.src_t.listen, 10);
    bc_print_raw(',');
    bc_print_byte(nctrl.src_t.frame_extra_listen, 10);
    bc_print_raw(',');
    bc_print_int(nctrl.t.rx_packet_to_tx, 10);
    bc_print_crlf("\n\rInitialized");
}

void node_control_run()
{
    radio_configure(RADIO_RX);
    radio_enable();
    rtc_start();

    while (1)
    {
        bc_update();
        radio_update();
        rtc_update();
        LPM4;
    }
}

void node_control_shutdown()
{}

void _setup_pins()
{
    // SMCLK on P8.0
    P8SEL0 |= BIT0;
    SYSCFG2 &= ~ADCPCTL8;
    P8DIR |= BIT0;

    // ACLK on P8.1
    P8SEL0 |= BIT1;
    SYSCFG2 &= ~ADCPCTL9;
    P8DIR |= BIT1;

    // Use P1.5 to show Frame start and end
    SYSCFG2 &= ~ADCPCTL5;
    P1SEL0 &= ~BIT5;
    P1DIR |= BIT5;
    P1OUT &= ~BIT5;

    // Use P1.4 to show timeslot activity
    SYSCFG2 &= ~ADCPCTL4;
    P1SEL0 &= ~BIT4;
    P1DIR |= BIT4;
    P1OUT &= ~BIT4;

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
    CSCTL3 &= ~(SELREF0 | SELREF1);
    CSCTL3 |= SELREF_1;

    // Clear the DCO and MOD registers
    CSCTL0 = 0;

    // Unset all dcorsel bits, then set for 16 MHz range
    CSCTL1 &= ~(DCORSEL0 | DCORSEL1 | DCORSEL2);
    CSCTL1 |= DCORSEL_5;

    // We want FLLD at 1 (16 MHz operation) and FLLN at 487
    // This produces DCOCLK and DCOCLKDIV of (FLLN + 1)(REFO) = (487 + 1)(32768) = 15.990784 MHz
    CSCTL2 = (FLLD_0 | 0x01E7);

    // We don't need so high for SMCLK - divide by 4.. would divide by 8 but baud rate doesn't work
    CSCTL5 |= DIVS_2;

    // A few no ops to make the above take affect
    _no_operation();
    _no_operation();
    _no_operation();

    // Re-enable FLL
    __bic_SR_register(SCG0);

    // Poll until the FLL has reached a lock!
    while ((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) || (CSCTL7 & DCOFFG))
    {
        // Clear OSC fault flags
        CSCTL7 &= ~DCOFFG;

        // Clear OFIFG fault flag
        SFRIFG1 &= ~OFIFG;
    }


    // Setup XT1 for crystal
    P4SEL0 |= BIT1;
    P4SEL0 |= BIT2;

    // Clear SELA bits - use XT1clock as SELA
    CSCTL4 &= ~0x0F00;
    //CSCTL6 |= XT1BYPASS;

    //CSCTL6 &= ~XT1AUTOOFF;
    //CSCTL4 |= 0x0200;

    //CSCTL5 &= ~VLOAUTOOFF;

    // Clear DIVA - need to measure XT1
    // CSCTL6 &= ~0x0F00;
    // //CSCTL6 |= 0x0100;
    // CSCTL6 |= BIT4;
    // CSCTL6 &= ~XT1AUTOOFF;
}
