/* Diagnostics logic: command validation and safety rules, session state, repeating reads,
 * captures and DIAG_RESP. No HAL: frames go out through the send function given to diag_init(),
 * so the host tests can inject a fake. Compiled only when TELEM_BAMO_DIAG != 0. */
#ifndef DIAG_CORE_H
#define DIAG_CORE_H

#include <stdint.h>
#include "telem_core.h"

#define DIAG_MAX_REPEAT_READS   3      /* repeating reads started by diagnostics */
#define DIAG_MIN_PERIOD_MS      20
#define DIAG_RATE_PER_S         20     /* CAN frames per second from diagnostics */
#define DIAG_SESSION_TIMEOUT_MS 10000u
#define DIAG_N_CAPTURE_SLOTS    4
#define DIAG_CAPTURE_RING       16

/* Returns 0 when the frame was queued for transmission. */
typedef int (*diag_can_send_fn)(const can_frame_t *f);

void diag_init(diag_can_send_fn send);

/* Every received CAN frame: live-inverter inputs and captures. */
void diag_on_can(const can_frame_t *f);

/* A command that already passed sync/type/CRC/key (telem_lora_rx_byte). */
void diag_on_command(const uint8_t cmd[TS_CMD_SIZE], uint32_t now);

/* Main loop: ends the session after 10 s without a valid command. */
void diag_poll(uint32_t now);

uint8_t diag_session_active(void);
int diag_inverter_live(uint32_t now);
void diag_last_ack(uint8_t *seq, uint8_t *status);   /* FAST bytes 52-53 */
int diag_resp_pending(void);
void diag_build_resp(ts_diag_resp_t *r, uint32_t now);
uint16_t diag_cmd_executed(void);
uint16_t diag_capture_dropped(void);
uint8_t diag_n_repeat_reads(void);

#endif /* DIAG_CORE_H */
