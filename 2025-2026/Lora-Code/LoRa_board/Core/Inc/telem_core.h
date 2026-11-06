/* Telemetry logic: CAN decoding, timeouts, events, packet building, batch scheduling and the
 * LoRa receive parser. No HAL: compiled into the firmware and into the host tests. */
#ifndef TELEM_CORE_H
#define TELEM_CORE_H

#include <stdint.h>
#include "telem_config.h"
#include "telemetry_schema.h"

#define CAN_EXT_FLAG 0x80000000u   /* id bit 31 = extended, as in the radio packets */

typedef struct {
    uint32_t id;      /* 11- or 29-bit ID (no flag bit) */
    uint8_t ext;      /* 1 = 29-bit */
    uint8_t dlc;      /* 0..8 */
    uint8_t data[8];
    uint32_t t_ms;    /* HAL_GetTick() when received */
} can_frame_t;

/* DIAG counters filled by the board code and the core */
typedef struct {
    uint32_t can_rx_total;
    uint32_t can_rx_unused;
    uint8_t can_tec;
    uint8_t can_rec;
    uint8_t can_busoff_count;
    uint8_t can_rx_overflow;       /* saturating; board increments via telem_note_rx_overflow() */
    uint16_t lora_channel_errors;
    uint16_t lora_power_on;
    uint16_t cmd_bad;
    uint8_t events_dropped;        /* saturating */
    uint32_t writes_skipped;       /* not on the radio: a due write that was refused or missed */
} telem_counters_t;

extern telem_counters_t telem_cnt;

uint16_t ts_crc16(const uint8_t *p, uint16_t n);

/* Boot: the first write is due 1 s after `now` (the module prints "Power on" ~0.5 s after boot). */
void telem_init(uint32_t now);

/* Main loop, for each received CAN frame (oldest first). */
void telem_on_can(const can_frame_t *f);

/* Main loop, each pass: CAN error state from the controller. */
void telem_on_can_state(uint8_t tec, uint8_t rec, uint8_t error_passive, uint8_t bus_off, uint32_t now);

/* Board: the CAN receive ring overflowed. */
void telem_note_rx_overflow(void);

/* Main loop, each pass: 50 ms samples, the mid-period FAST, timeouts and events. */
void telem_poll(uint32_t now);

/* Byte from the car LoRa module. Returns 1 when `cmd` (24 B) holds a command that passed the
 * sync, type, CRC and key checks (bad candidates are counted and skipped). Also counts the
 * module's "channel Error" and "Power on" lines. */
int telem_lora_rx_byte(uint8_t b, uint32_t now, uint8_t cmd[TS_CMD_SIZE]);

/* Main loop: every byte from the car module. Passes good commands to diag_core when
 * TELEM_BAMO_DIAG != 0; flag 0 ignores them (still counted by the parser). */
void telem_lora_feed(uint8_t b, uint32_t now);

/* Is a write due (deadline t0 + 200 ms x k reached)? */
int telem_write_due(uint32_t now);

/* Transport rule check (BUILD_INSTRUCTIONS 5.3): refuse while the previous DMA is running,
 * less than 120 ms after the previous write, or over 250 B. */
int telem_tx_allowed(uint32_t now, uint32_t last_write_ms, int have_last, int dma_busy, uint16_t len);

/* Build the due write (header + 3 x 64 B) into buf; returns its length (195). */
uint16_t telem_build_write(uint32_t now, uint8_t *buf);

/* A due write could not be sent: move to the next deadline. */
void telem_write_skipped(uint32_t now);

/* Test hooks */
void telem_push_event(uint8_t code, uint16_t value, uint32_t t);
uint16_t telem_valid_now(uint32_t now);
uint32_t telem_write_count(void);

#endif /* TELEM_CORE_H */
