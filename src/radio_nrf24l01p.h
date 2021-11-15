#pragma once

#include "typedefs.h"
#include "ring_buffer.h"

// Commands
#define R_REGISTER    0x00 /* last 4 bits will indicate reg. address */
#define W_REGISTER    0x20 /* last 4 bits will indicate reg. address */
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define NOP           0xFF

// Register addresses
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C

// CONFIG register fields
#define MASK_RX_DR_SHIFT  6
#define MASK_RX_DR        0x40
#define MASK_TX_DS_SHIFT  5
#define MASK_TX_DS        0x20
#define MASK_MAX_RT_SHIFT 4
#define MASK_MAX_RT       0x10
#define EN_CRC_SHIFT      3
#define EN_CRC            0x08
#define CRCO_SHIFT        2
#define CRCO              0x04
#define PWR_UP_SHIFT      1
#define PWR_UP            0x02
#define PRIM_RX           0x01

// EN_AA register fields
#define ENAA_P5_SHIFT 5
#define ENAA_P5       0x20
#define ENAA_P4_SHIFT 4
#define ENAA_P4       0x10
#define ENAA_P3_SHIFT 3
#define ENAA_P3       0x08
#define ENAA_P2_SHIFT 2
#define ENAA_P2       0x04
#define ENAA_P1_SHIFT 1
#define ENAA_P1       0x02
#define ENAA_P0       0x01

// EN_RXADDR register fields
#define ERX_P5_SHIFT 5
#define ERX_P5       0x20
#define ERX_P4_SHIFT 4
#define ERX_P4       0x10
#define ERX_P3_SHIFT 3
#define ERX_P3       0x08
#define ERX_P2_SHIFT 2
#define ERX_P2       0x04
#define ERX_P1_SHIFT 1
#define ERX_P1       0x02
#define ERX_P0       0x01

// SETUP_AW register fields (AW_0 illegal)
#define AW_1 0x01
#define AW_2 0x02
#define AW_3 0x03
#define AW   AW_3

// SETUP_RETR register fields
#define ARD_SHIFT 4
#define ARD_250   0x00
#define ARD_500   0x10
#define ARD_750   0x20
#define ARD_1000  0x30
#define ARD_1250  0x40
#define ARD_1500  0x50
#define ARD_1750  0x60
#define ARD_2000  0x70
#define ARD_2250  0x80
#define ARD_2500  0x90
#define ARD_2750  0xA0
#define ARD_3000  0xB0
#define ARD_3250  0xC0
#define ARD_3500  0xD0
#define ARD_3750  0xE0
#define ARD_4000  0xF0
#define ARD       ARD_4000
#define ARC_OFF   0x00
#define ARC_MAX   0xFF

// RF_CH register fields
#define RF_CH_MAX 0x7F

// RF_SETUP register fields
#define CONT_WAVE_SHIFT  7
#define CONT_WAVE        0x80
#define RF_DR_LOW_SHIFT  5
#define RF_DR_LOW        0x20
#define PLL_LOCK_SHIFT   4
#define PLL_LOCK         0x10
#define RF_DR_HIGH_SHIFT 3
#define RF_DR_HIGH_2Mb   0x08
#define RF_PWR_SHIFT     1
#define RF_PWR_0         0x00
#define RF_PWR_1         0x02
#define RF_PWR_2         0x04
#define RF_PWR_3         0x06
#define RF_PWR           RF_PWR_3

// STATUS register fields
#define RX_DR_SHIFT   6
#define RX_DR         0x40
#define TX_DS_SHIFT   5
#define TX_DS         0x20
#define MAX_RT_SHIFT  4
#define MAX_RT        0x10
#define RX_P_NO_SHIFT 1
#define RX_P_NO_0     0x00
#define RX_P_NO_1     0x02
#define RX_P_NO_2     0x04
#define RX_P_NO_3     0x06
#define RX_P_NO_4     0x08
#define RX_P_NO_5     0x0A
#define RX_P_NO_EMPTY 0x0E
#define RX_P_NO       RX_P_NO_EMPTY
#define TX_FULL       0x01

// OBSERVE_TX register fields
#define PLOS_CNT_SHIFT 4
#define PLOS_CNT       0xF0
#define ARC_CNT        0x0F

// FIFO_STATUS register fields
#define TX_REUSE_SHIFT  6
#define TX_REUSE        0x40
#define FIFO_FULL_SHIFT 5
#define FIFO_FULL       0x20
#define TX_EMPTY_SHIFT  4
#define TX_EMPTY        0x10
#define RX_FULL_SHIFT   1
#define RX_FULL         0x02
#define RX_EMPTY_SHIFT  0
#define RX_EMPTY        0x01

/* DYNPD register fields */
#define DPL_P5_SHIFT 5
#define DPL_P5       0x20
#define DPL_P4_SHIFT 4
#define DPL_P4       0x10
#define DPL_P3_SHIFT 3
#define DPL_P3       0x08
#define DPL_P2_SHIFT 2
#define DPL_P2       0x04
#define DPL_P1_SHIFT 1
#define DPL_P1       0x02
#define DPL_P0_SHIFT 0
#define DPL_P0       0x01

extern Ring_Buffer rad_tx;
extern Ring_Buffer rad_rx;
extern void (*HANDLE_RADIO_RX_COMMAND)(void);

void radio_nRF24L01P_init();

/// Transmit all data pending in rad_tx as fast as possible, ignoring
/// all RX bytes shifted back
void radio_nRF24L01P_burst_spi_tx();

void radio_nRF24L01P_write_register(i8 regaddr, i8 * data, i8 size);

void radio_nRF24L01P_read_register(i8 regaddr);

void radio_nRF24L01P_rx_byte(i8 byte);

static inline void _send_next();

static void _spi_init();

static void _pins_init();

static void _check_rx_radio();
