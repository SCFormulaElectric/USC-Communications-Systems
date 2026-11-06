/* Hardware glue (BUILD_INSTRUCTIONS.md 5.1-5.3): CAN receive interrupt into a ring buffer,
 * USART2 at 115200 with one-shot DMA transmit (DMA1 channel 4, from the CubeMX F042 DMA table)
 * and RXNE-interrupt receive into a ring, CAN error state, and the independent watchdog. */
#include <string.h>
#include "main.h"
#include "board.h"

extern CAN_HandleTypeDef hcan;

/* ------------------------------------------------------------------ CAN receive ring
 * Single producer (CAN interrupt) / single consumer (main loop): head is written only by the
 * interrupt, tail only by main; free-running 8-bit indices, 32 entries. */
#define CAN_RING 32u
static can_frame_t can_ring[CAN_RING];
static volatile uint8_t can_head, can_tail, can_ovf;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    CAN_RxHeaderTypeDef rh;
    uint8_t d[8];
    while (HAL_CAN_GetRxFifoFillLevel(h, CAN_RX_FIFO0)) {
        if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &rh, d) != HAL_OK) break;
        if ((uint8_t)(can_head - can_tail) >= CAN_RING) {   /* ring full: count and drop */
            if (can_ovf < 0xFFu) can_ovf++;
            continue;
        }
        can_frame_t *f = &can_ring[can_head % CAN_RING];
        f->ext = rh.IDE == CAN_ID_EXT;
        f->id = f->ext ? rh.ExtId : rh.StdId;
        f->dlc = (uint8_t)(rh.DLC > 8 ? 8 : rh.DLC);
        memcpy(f->data, d, 8);
        f->t_ms = HAL_GetTick();
        can_head++;
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h)
{
    if (h->ErrorCode & HAL_CAN_ERROR_RX_FOV0) {   /* hardware FIFO overrun */
        if (can_ovf < 0xFFu) can_ovf++;
    }
    HAL_CAN_ResetError(h);
}

/* Per build flag; only while the controller is in initialization mode. */
void board_can_apply_mode(void)
{
#if TELEM_BAMO_DIAG == 0
    hcan.Instance->BTR |= CAN_BTR_SILM;      /* silent: the controller can't transmit (no ACK, no error frames) */
#else
    hcan.Instance->MCR |= CAN_MCR_ABOM;      /* recover from bus-off by itself */
    hcan.Instance->MCR &= ~CAN_MCR_NART;     /* retransmit diagnostic frames after lost arbitration */
#endif
}

/* Returns 1 when CAN is running. Never stops the board: leaving initialization mode needs 11
 * recessive bits on CAN RX, which fails while the transceiver is unpowered or the bus is held
 * dominant. main.c retries every second; telemetry keeps running meanwhile. */
int board_can_start(void)
{
    CAN_FilterTypeDef fc;
    if (HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY) {    /* a previous start failed */
        if (HAL_CAN_Init(&hcan) != HAL_OK) return 0;
        board_can_apply_mode();
    }
    memset(&fc, 0, sizeof fc);
    fc.FilterBank = 0;
    fc.FilterMode = CAN_FILTERMODE_IDMASK;
    fc.FilterScale = CAN_FILTERSCALE_32BIT;
    fc.FilterIdHigh = 0;                  /* mask 0: accept every frame, standard and extended */
    fc.FilterMaskIdHigh = 0;
    fc.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    fc.FilterActivation = ENABLE;
    fc.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &fc) != HAL_OK) return 0;
    if (HAL_CAN_Start(&hcan) != HAL_OK) return 0;
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
        return 0;
    return 1;
}

int board_can_pop(can_frame_t *f)
{
    uint8_t t = can_tail;
    if (t == can_head) return 0;
    *f = can_ring[t % CAN_RING];
    can_tail = (uint8_t)(t + 1);
    return 1;
}

uint8_t board_can_take_overflows(void)
{
    __disable_irq();
    uint8_t n = can_ovf;
    can_ovf = 0;
    __enable_irq();
    return n;
}

void board_can_state(uint8_t *tec, uint8_t *rec, uint8_t *error_passive, uint8_t *bus_off)
{
    uint32_t esr = hcan.Instance->ESR;
    *tec = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    *rec = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    *error_passive = (esr & CAN_ESR_EPVF) != 0;
    *bus_off = (esr & CAN_ESR_BOFF) != 0;
}

int board_can_send(const can_frame_t *f)
{
    CAN_TxHeaderTypeDef th;
    uint32_t mb;
    if (hcan.Instance->ESR & CAN_ESR_BOFF) return -1;           /* TX_FAIL while bus-off */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) return -1;
    memset(&th, 0, sizeof th);
    th.IDE = f->ext ? CAN_ID_EXT : CAN_ID_STD;
    if (f->ext) th.ExtId = f->id; else th.StdId = f->id;
    th.RTR = CAN_RTR_DATA;
    th.DLC = f->dlc;
    return HAL_CAN_AddTxMessage(&hcan, &th, (uint8_t *)f->data, &mb) == HAL_OK ? 0 : -1;
}

/* ------------------------------------------------------------------ USART2 (LoRa module) */
#define UART_RING 128u
static uint8_t uart_ring[UART_RING];
static volatile uint8_t uart_head, uart_tail;

void board_usart2_irq(void)
{
    uint32_t isr = USART2->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;   /* else RXNE stops */
    if (isr & USART_ISR_RXNE) {
        uint8_t b = (uint8_t)USART2->RDR;
        if ((uint8_t)(uart_head - uart_tail) < UART_RING) {
            uart_ring[uart_head % UART_RING] = b;
            uart_head++;
        }
    }
}

void board_uart_start(void)
{
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, 0, (uint32_t)&USART2->TDR,
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_USART_EnableDMAReq_TX(USART2);
    LL_USART_EnableIT_RXNE(USART2);
    NVIC_SetPriority(USART2_IRQn, 1);     /* below CAN: the CAN FIFO is only 3 frames deep */
    NVIC_EnableIRQ(USART2_IRQn);
}

int board_uart_pop(uint8_t *b)
{
    uint8_t t = uart_tail;
    if (t == uart_head) return 0;
    *b = uart_ring[t % UART_RING];
    uart_tail = (uint8_t)(t + 1);
    return 1;
}

int board_tx_busy(void)
{
    return LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_4) && LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_4) != 0;
}

void board_tx_start(const uint8_t *p, uint16_t n)
{
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
    LL_DMA_ClearFlag_GI4(DMA1);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)p);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, n);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

/* ------------------------------------------------------------------ watchdog */
void board_iwdg_start(void)
{
    __HAL_RCC_DBGMCU_CLK_ENABLE();
    __HAL_DBGMCU_FREEZE_IWDG();            /* don't reset while halted in the debugger */
    IWDG->KR = 0xCCCCu;                    /* start (also starts the ~40 kHz LSI) */
    IWDG->KR = 0x5555u;                    /* unlock PR/RLR */
    IWDG->PR = 3u;                         /* /32 -> ~1.25 kHz */
    IWDG->RLR = 625u;                      /* ~500 ms (LSI 30-50 kHz: ~400-670 ms) */
    while (IWDG->SR) { }
    IWDG->KR = 0xAAAAu;
}

void board_iwdg_refresh(void) { IWDG->KR = 0xAAAAu; }
