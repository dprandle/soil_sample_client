#pragma once

#include "typedefs.h"

#define MAX_NODE_HOPS   5
#define PACKET_PAYLOAD_BYTE_SIZE  32
#define DATARATE 2000000
#define PACKET_BYTE_SIZE  39
#define CRYSTAL_PPM     20
#define CRYSTAL_FREQ    32768
#define ROUND_THRESHOLD 0.75
#define MAX_TIMESLOTS_PER_FRAME 32
#define OUR_TIMESLOT_DATA nctrl.cur_frame.timeslots[nctrl.cur_frame.our_timeslot-1]
#define CUR_TIMESLOT_DATA nctrl.cur_frame.timeslots[nctrl.cur_frame.cur_timeslot-1]

typedef struct
{
    i8 src_addr;
    i8 dest_addr;
    i16 data;
} Node_Data;

typedef union
{
    struct
    {
        i8 frame_ind;
        i8 timeslot;
        i16 timeslot_mask;
        i8 total_node_count;
        i8 future1;
        i16 future2;
        Node_Data data;
        Node_Data fwd[MAX_NODE_HOPS];
    };
    char raw_data[PACKET_PAYLOAD_BYTE_SIZE];
} Timeslot_Packet;

typedef struct
{
    i8 timeslot_mask;
    Node_Data data;
} Timeslot_Info;

typedef struct
{
    u8 rx_on;
    u16 rx_packet_to_tx;
    u16 rx_packet_to_rx;
    u16 rx_no_packet_to_tx;
    u16 rx_no_packet_to_rx;
    u16 rx_no_packet_to_end_frame;
    u16 tx_to_rx;
    u16 tx_to_end_frame;
    u16 timeslot;
} RTC_Set_Values;

typedef struct
{
    u16 timeslot;
    u8 settle;
    u8 listen;
    u8 drift_listen;
    u8 frame_extra_drift_listen;
    u8 packet_listen;
    u8 tx_to_rx_measured_delay;
} RTC_Cycle_Source;

typedef struct
{
    u8 our_timeslot;
    u8 ind;
    u8 cur_timeslot;
    Timeslot_Info timeslots[MAX_TIMESLOTS_PER_FRAME];
} Frame_Info;

typedef struct
{
    u8 timeslots_per_frame;
    u8 startup_listen_frame_count;
    u8 sleep_frame_count;
    u8 total_node_count;

    RTC_Cycle_Source src_t;
    RTC_Set_Values t;
    Frame_Info cur_frame;
} Node_Control;

static void _setup_clocks();
static void _setup_pins();

#ifndef RADIO_DEBUG_SPI
static void _recalc_rtc_derived_from_source();
static void _setup_rtc();
static void _clock_in_payload();
static void _clock_out_payload();
static void _set_short_listen();
static void _set_long_listen();
void frame_start();
void frame_end();
void frame_rx_timeslot_end();
void frame_rx_timeslot_begin();
void frame_tx_timeslot();
#endif

void node_control_init();
void node_control_run();
void node_control_shutdown();

void node_start_frame();
