#pragma once

#include "typedefs.h"

extern const i8 RTC_MODE_REPEAT;
extern const i8 RTC_MODE_ONE_SHOT;

extern volatile u8 toggel_pin_next_isr;

typedef void (*RTC_Callback)(void);

void rtc_init();

void rtc_set_cb(RTC_Callback callback);

void rtc_start();

void rtc_stop();

void rtc_reset();

void rtc_set_interrupt_tick_count(i16 count);

i16 rtc_get_interrupt_tick_count();

void rtc_set_tick_cycles(i16 value);

i16 rtc_get_tick_cycles();

i16 rtc_get_elapsed();

void rtc_set_mode(i8 timer_mode);

i8 rtc_get_mode();

void rtc_update();
