#pragma once

#include "typedefs.h"

#define RADIO_NRF24L01P_RX_BUF_SIZE 32
#define RADIO_NRF24L01P_TX_BUF_SIZE 64

typedef struct
{
    i8 tx_buffer[RADIO_NRF24L01P_TX_BUF_SIZE];
    i8 rx_buffer[RADIO_NRF24L01P_RX_BUF_SIZE];
    i8 tx_cur_ind;
    i8 tx_end_ind;
    i8 rx_cur_ind;
    i8 rx_end_ind;
} Radio_nRF24L01P;

extern Radio_nRF24L01P radio;

void radio_nRF24L01P_init();

void radio_nRF24L01P_tx_byte(i8 byte);

void radio_nRF24L01P_rx_byte(i8 byte);

void radio_nRF24L01P_shutdown();
