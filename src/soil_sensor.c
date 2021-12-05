#include <msp430.h>
#include "soil_sensor.h"

static volatile u16 latest_sample = 0;
static volatile Soil_Sample_Callback cback = 0;
static volatile i8 sample_complete_update = 0;

void soil_sensor_init()
{
    // Set TSample to 64 cycles
    ADCCTL0 |= ADCSHT_4;

    // Turn on the ADC
    ADCCTL0 |= ADCON;

    // Enable Pulse Sample Mode - to start a sample just set ADCSC bit
    ADCCTL1 |= ADCSHP;

    // Reduce sampling rate - save power
    ADCCTL2 |= ADCSR;

    // Enable sample complete interrupt
    ADCIE |= ADCIE0;

    // Set Channel 8 input on ADC
    // and enable ADC on P8.0
    ADCMCTL0 |= ADCINCH_8;
    SYSCFG2 |= ADCPCTL8;
}

void soil_sensor_set_cb(Soil_Sample_Callback cb)
{
    cback = cb;
}

void soil_sensor_trigger_sample()
{
    ADCCTL0 |= (ADCENC | ADCSC);
}

void soil_sensor_update()
{
    if (sample_complete_update)
    {
        sample_complete_update = 0;
        cback();
    }
}

u16 soil_sensor_get_latest_sample()
{
    return latest_sample;
}

__interrupt_vec(ADC_VECTOR) void adc_ISR(void)
{
    switch (ADCIV)
    {
        case (ADCIV_NONE):
            break;
        case (ADCIV_ADCOVIFG):
            break;
        case (ADCIV_ADCTOVIFG):
            break;
        case (ADCIV_ADCHIIFG):
            break;
        case (ADCIV_ADCLOIFG):
            break;
        case (ADCIV_ADCINIFG):
            break;
        case (ADCIV_ADCIFG):
            latest_sample = ADCMEM0;
            if (cback)
            {
                sample_complete_update = 1;
                LPM4_EXIT;
            }
            break;
    }
}