/* Hardware glue for the LoRa telemetry board (HAL CAN, LL USART2 + DMA, IWDG). */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "telem_core.h"

void board_can_start(void);                 /* filter (accept all, FIFO0), notifications, start */
int board_can_pop(can_frame_t *f);          /* 1 = a received frame was copied out of the ring */
uint8_t board_can_take_overflows(void);     /* receive-ring and FIFO overruns since the last call */
void board_can_state(uint8_t *tec, uint8_t *rec, uint8_t *error_passive, uint8_t *bus_off);
int board_can_send(const can_frame_t *f);   /* diagnostics: 0 = queued in a TX mailbox */

void board_uart_start(void);                /* RX interrupt + TX DMA request */
int board_uart_pop(uint8_t *b);             /* 1 = a byte from the LoRa module */
int board_tx_busy(void);                    /* the previous DMA write is still running */
void board_tx_start(const uint8_t *p, uint16_t n);
void board_usart2_irq(void);                /* called from USART2_IRQHandler */

void board_iwdg_start(void);                /* independent watchdog, ~500 ms */
void board_iwdg_refresh(void);

#endif /* BOARD_H */
