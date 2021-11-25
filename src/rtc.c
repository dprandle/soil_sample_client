#include <msp430.h>
#include "rtc.h"

const i8 RTC_MODE_REPEAT = 0;
const i8 RTC_MODE_ONE_SHOT = 1;

static RTC_Callback cback = 0;
static i8 timer_triggered = 0;
static i8 timer_mode = RTC_MODE_REPEAT;
static i16 interrupt_tick_count = 0;
static i16 current_tick_count = 0;

u8 toggel_pin_next_isr = 0;

void rtc_init()
{
    // Select XT1CLK as source - our external oscillator - the most accurate thing we have available
    // Enable interrupt
    rtc_stop();
    rtc_reset();
    i8 dummy_read = RTCIV;
    RTCCTL |= RTCIE;
}

void rtc_update()
{
    if (timer_triggered && cback)
    {
        if (timer_mode == RTC_MODE_ONE_SHOT)
        {
            rtc_stop();
            rtc_reset();
        }
        timer_triggered = 0;
        cback();
    }
}

void rtc_set_callback(RTC_Callback callback)
{
    cback = callback;
}

void rtc_start()
{
    // First clear the bits, then set the right ones
    RTCCTL &= ~RTCSS_3;
    RTCCTL |= RTCSS_2;
}

void rtc_stop()
{
    // Clear the clock source bits
    RTCCTL &= ~RTCSS_3;
}

void rtc_reset()
{
    // Set the reset bit
    current_tick_count = 0;
    RTCCTL |= RTCSR;
}

void rtc_set_tick_cycles(i16 value)
{
    current_tick_count = 0;
    RTCMOD = value;
}

void rtc_set_interrupt_tick_count(i16 count)
{
    current_tick_count = 0;
    interrupt_tick_count = count;
}

i16 rtc_get_interrupt_tick_count()
{
    return interrupt_tick_count;
}

i16 rtc_get_tick_cycles()
{
    return RTCMOD;
}

i16 rtc_get_elapsed()
{
    return RTCCNT;
}

void rtc_set_mode(i8 timer_mode_)
{
    timer_mode = timer_mode_;
}

i8 rtc_get_mode()
{
    return timer_mode;
}

__interrupt_vec(RTC_VECTOR) void rtc_isr(void)
{
    i8 dummyread = RTCIV;
    if (toggel_pin_next_isr)
    {
        P1OUT ^= BIT5;
        toggel_pin_next_isr = 0;
    }
    ++current_tick_count;
    if (cback && (current_tick_count >= interrupt_tick_count)) // >= in case interrupt_tick_count changed
    {
        current_tick_count = 0;
        timer_triggered = 1;
        LPM4_EXIT;
    }
}
