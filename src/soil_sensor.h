#pragma once
#include "typedefs.h"

typedef void (*Soil_Sample_Callback)(void);

void soil_sensor_init();

void soil_sensor_update();

void soil_sensor_set_cb(Soil_Sample_Callback cb);

void soil_sensor_trigger_sample();

u16 soil_sensor_get_latest_sample();