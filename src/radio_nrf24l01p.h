#pragma once

#include "typedefs.h"
#include "ring_buffer.h"

// Commands
#define NRF24L01P_CMD_R_REGISTER    0x00 /* last 4 bits will indicate reg. address */
#define NRF24L01P_CMD_W_REGISTER    0x20 /* last 4 bits will indicate reg. address */
#define NRF24L01P_CMD_REGISTER_MASK 0x1F
#define NRF24L01P_CMD_R_RX_PAYLOAD  0x61
#define NRF24L01P_CMD_W_TX_PAYLOAD  0xA0
#define NRF24L01P_CMD_FLUSH_TX      0xE1
#define NRF24L01P_CMD_FLUSH_RX      0xE2
#define NRF24L01P_CMD_REUSE_TX_PL   0xE3
#define NRF24L01P_CMD_ACTIVATE      0x50
#define NRF24L01P_CMD_R_RX_PL_WID   0x60
#define NRF24L01P_CMD_NOP           0xFF

// Register addresses
#define NRF24L01P_ADDR_CONFIG      0x00
#define NRF24L01P_ADDR_EN_AA       0x01
#define NRF24L01P_ADDR_EN_RXADDR   0x02
#define NRF24L01P_ADDR_SETUP_AW    0x03
#define NRF24L01P_ADDR_SETUP_RETR  0x04
#define NRF24L01P_ADDR_RF_CH       0x05
#define NRF24L01P_ADDR_RF_SETUP    0x06
#define NRF24L01P_ADDR_STATUS      0x07
#define NRF24L01P_ADDR_OBSERVE_TX  0x08
#define NRF24L01P_ADDR_RPD         0x09
#define NRF24L01P_ADDR_RX_ADDR_P0  0x0A
#define NRF24L01P_ADDR_RX_ADDR_P1  0x0B
#define NRF24L01P_ADDR_RX_ADDR_P2  0x0C
#define NRF24L01P_ADDR_RX_ADDR_P3  0x0D
#define NRF24L01P_ADDR_RX_ADDR_P4  0x0E
#define NRF24L01P_ADDR_RX_ADDR_P5  0x0F
#define NRF24L01P_ADDR_TX_ADDR     0x10
#define NRF24L01P_ADDR_RX_PW_P0    0x11
#define NRF24L01P_ADDR_RX_PW_P1    0x12
#define NRF24L01P_ADDR_RX_PW_P2    0x13
#define NRF24L01P_ADDR_RX_PW_P3    0x14
#define NRF24L01P_ADDR_RX_PW_P4    0x15
#define NRF24L01P_ADDR_RX_PW_P5    0x16
#define NRF24L01P_ADDR_FIFO_STATUS 0x17
#define NRF24L01P_ADDR_DYNPD       0x1C
#define NRF24L01P_ADDR_FEATURE     0x1D

// CONFIG register fields
#define NRF24L01P_MASK_RX_DR_SHIFT  6
#define NRF24L01P_MASK_RX_DR        0x40
#define NRF24L01P_MASK_TX_DS_SHIFT  5
#define NRF24L01P_MASK_TX_DS        0x20
#define NRF24L01P_MASK_MAX_RT_SHIFT 4
#define NRF24L01P_MASK_MAX_RT       0x10
#define NRF24L01P_EN_CRC_SHIFT      3
#define NRF24L01P_EN_CRC            0x08
#define NRF24L01P_CRCO_SHIFT        2
#define NRF24L01P_CRCO              0x04
#define NRF24L01P_PWR_UP_SHIFT      1
#define NRF24L01P_PWR_UP            0x02
#define NRF24L01P_PRIM_RX           0x01

// EN_AA register fields
#define NRF24L01P_ENAA_P5_SHIFT 5
#define NRF24L01P_ENAA_P5       0x20
#define NRF24L01P_ENAA_P4_SHIFT 4
#define NRF24L01P_ENAA_P4       0x10
#define NRF24L01P_ENAA_P3_SHIFT 3
#define NRF24L01P_ENAA_P3       0x08
#define NRF24L01P_ENAA_P2_SHIFT 2
#define NRF24L01P_ENAA_P2       0x04
#define NRF24L01P_ENAA_P1_SHIFT 1
#define NRF24L01P_ENAA_P1       0x02
#define NRF24L01P_ENAA_P0       0x01

// EN_RXADDR register fields
#define NRF24L01P_ERX_P5_SHIFT 5
#define NRF24L01P_ERX_P5       0x20
#define NRF24L01P_ERX_P4_SHIFT 4
#define NRF24L01P_ERX_P4       0x10
#define NRF24L01P_ERX_P3_SHIFT 3
#define NRF24L01P_ERX_P3       0x08
#define NRF24L01P_ERX_P2_SHIFT 2
#define NRF24L01P_ERX_P2       0x04
#define NRF24L01P_ERX_P1_SHIFT 1
#define NRF24L01P_ERX_P1       0x02
#define NRF24L01P_ERX_P0       0x01

// SETUP_AW register fields (AW_0 illegal)
#define NRF24L01P_AW_1 0x01
#define NRF24L01P_AW_2 0x02
#define NRF24L01P_AW_3 0x03
#define NRF24L01P_AW   AW_3

// SETUP_RETR register fields
#define NRF24L01P_ARD_SHIFT 4
#define NRF24L01P_ARD_250   0x00
#define NRF24L01P_ARD_500   0x10
#define NRF24L01P_ARD_750   0x20
#define NRF24L01P_ARD_1000  0x30
#define NRF24L01P_ARD_1250  0x40
#define NRF24L01P_ARD_1500  0x50
#define NRF24L01P_ARD_1750  0x60
#define NRF24L01P_ARD_2000  0x70
#define NRF24L01P_ARD_2250  0x80
#define NRF24L01P_ARD_2500  0x90
#define NRF24L01P_ARD_2750  0xA0
#define NRF24L01P_ARD_3000  0xB0
#define NRF24L01P_ARD_3250  0xC0
#define NRF24L01P_ARD_3500  0xD0
#define NRF24L01P_ARD_3750  0xE0
#define NRF24L01P_ARD_4000  0xF0
#define NRF24L01P_ARD       ARD_4000
#define NRF24L01P_ARC_OFF   0x00
#define NRF24L01P_ARC_MAX   0xFF

// RF_CH register fields
#define NRF24L01P_RF_CH_MAX 0x7F

// RF_SETUP register fields
#define NRF24L01P_CONT_WAVE_SHIFT  7
#define NRF24L01P_CONT_WAVE        0x80
#define NRF24L01P_RF_DR_LOW_SHIFT  5
#define NRF24L01P_RF_DR_LOW        0x20
#define NRF24L01P_PLL_LOCK_SHIFT   4
#define NRF24L01P_PLL_LOCK         0x10
#define NRF24L01P_RF_DR_HIGH_SHIFT 3
#define NRF24L01P_RF_DR_HIGH_2Mb   0x08
#define NRF24L01P_RF_PWR_SHIFT     1
#define NRF24L01P_RF_PWR_0         0x00
#define NRF24L01P_RF_PWR_1         0x02
#define NRF24L01P_RF_PWR_2         0x04
#define NRF24L01P_RF_PWR_3         0x06
#define NRF24L01P_RF_PWR           RF_PWR_3

// STATUS register fields
#define NRF24L01P_RX_DR_SHIFT   6
#define NRF24L01P_RX_DR         0x40
#define NRF24L01P_TX_DS_SHIFT   5
#define NRF24L01P_TX_DS         0x20
#define NRF24L01P_MAX_RT_SHIFT  4
#define NRF24L01P_MAX_RT        0x10
#define NRF24L01P_RX_P_NO_SHIFT 1
#define NRF24L01P_RX_P_NO_0     0x00
#define NRF24L01P_RX_P_NO_1     0x02
#define NRF24L01P_RX_P_NO_2     0x04
#define NRF24L01P_RX_P_NO_3     0x06
#define NRF24L01P_RX_P_NO_4     0x08
#define NRF24L01P_RX_P_NO_5     0x0A
#define NRF24L01P_RX_P_NO_EMPTY 0x0E
#define NRF24L01P_RX_P_NO       RX_P_NO_EMPTY
#define NRF24L01P_TX_FULL       0x01

// OBSERVE_TX register fields
#define NRF24L01P_PLOS_CNT_SHIFT 4
#define NRF24L01P_PLOS_CNT       0xF0
#define NRF24L01P_ARC_CNT        0x0F

// FIFO_STATUS register fields
#define NRF24L01P_TX_REUSE_SHIFT  6
#define NRF24L01P_TX_REUSE        0x40
#define NRF24L01P_FIFO_FULL_SHIFT 5
#define NRF24L01P_FIFO_FULL       0x20
#define NRF24L01P_TX_EMPTY_SHIFT  4
#define NRF24L01P_TX_EMPTY        0x10
#define NRF24L01P_RX_FULL_SHIFT   1
#define NRF24L01P_RX_FULL         0x02
#define NRF24L01P_RX_EMPTY_SHIFT  0
#define NRF24L01P_RX_EMPTY        0x01

/* DYNPD register fields */
#define NRF24L01P_DPL_P5_SHIFT 5
#define NRF24L01P_DPL_P5       0x20
#define NRF24L01P_DPL_P4_SHIFT 4
#define NRF24L01P_DPL_P4       0x10
#define NRF24L01P_DPL_P3_SHIFT 3
#define NRF24L01P_DPL_P3       0x08
#define NRF24L01P_DPL_P2_SHIFT 2
#define NRF24L01P_DPL_P2       0x04
#define NRF24L01P_DPL_P1_SHIFT 1
#define NRF24L01P_DPL_P1       0x02
#define NRF24L01P_DPL_P0_SHIFT 0
#define NRF24L01P_DPL_P0       0x01

#define NRF24L01P_FEATURE_EN_DPL     0x04
#define NRF24L01P_FEATURE_EN_ACK_PAY 0x02
#define NRF24L01P_FEATURE_EN_DYN_ACK 0x01

#define RF_CHANNEL 101
#define RADIO_PAYLOAD_SIZE 32

#define RADIO_TX 1
#define RADIO_RX 0
#define RADIO_NOT_CONFIGURED -1

typedef void (*Packet_Callback)(void);

void radio_init();

void radio_update();

void radio_configure(i8 tx_or_rx);

i8 radio_get_tx_or_rx();

void radio_enable();

void radio_disable();

void radio_clock_in(u8 * data, u8 size);

void radio_clock_out(u8 * data, u8 size);

void radio_enable_pulse();

void radio_clear_interrupts();

void radio_set_pckt_rx_cb(Packet_Callback cback);

Packet_Callback radio_get_pckt_rx_cb();

void radio_set_pckt_tx_cb(Packet_Callback cback);

Packet_Callback radio_get_pckt_tx_cb();

/// 1 for TX and 0 for RX
void radio_flush(i8 tx_or_rx);

/// Transmit all data pending in rad_tx as fast as possible, ignoring
/// all RX bytes shifted back
void radio_burst_spi_tx(i8 cmd_address);

/// Get nbytes as fast as possible, by polling the rx register
void radio_burst_spi_rx(i8 cmd_address, i8 nbytes);

void radio_write_register_data(i8 regaddr, i8 * data, i8 size);

void radio_write_register(i8 regaddr, i8 byte);

void radio_read_register(i8 regaddr, i8 nbytes);

static inline void _add_command(i8 cmd, i8 expected);

static inline void _send_next();

static void _spi_init();

static void _pins_init();

#ifdef RADIO_DEBUG_SPI
static inline void _print_rx_buf(i8 base);
#endif
