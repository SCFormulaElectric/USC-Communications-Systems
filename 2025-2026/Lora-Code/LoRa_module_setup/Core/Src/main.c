/* One-time setup of the DX-LR02 LoRa module that is wired to the telemetry board's STM32
 * (USART2 on PA2/PA3), for boards where the module can't be reached from a PC.
 *
 * Flash this, let it run (~10-40 s), then flash the telemetry firmware again.
 *
 *  1. Finds the module's UART baud rate: '+++' at 9600, 115200, 57600, 38400, 19200, 4800, 2400,
 *     1200 (up to 3 passes). '+++' toggles AT/data mode, so "Exit AT" means it was already in
 *     AT mode and is toggled back.
 *  2. Saves AT+HELP (the old settings) in g_before.
 *  3. Sets MODE1, LEVEL7 (SF5), CHANNEL03, MAC 0a,02, POWE22 and baud 115200 (AT+BAUD8, takes
 *     effect at the reset). NEVER sends AT+SF12. Then AT+RESET.
 *  4. Checks the new settings at 115200 (g_after), leaves AT mode.
 *  5. LED on PA5:  short flash per baud tried = working
 *                  steady ON       = done, module set up (g_result 1)
 *                  fast blink 10 Hz = no answer at any baud: check power/wiring (g_result 2)
 *                  slow blink 1 Hz  = answered, but the settings didn't check out (g_result 3)
 *  6. On success it sends a text report over the radio every 3 s (to MAC 0a,01, channel 03): the
 *     PC module prints it on its COM port (listen.py).
 * Everything is also in RAM for the debugger: g_result, g_found_baud, g_before, g_after, g_log.
 *
 * Bare metal (CMSIS only), HSI 8 MHz, SysTick 1 ms. No HAL, no interrupts except SysTick. */
#include <stdint.h>
#include <string.h>
#include "stm32f0xx.h"

#define F_CPU 8000000u

volatile uint32_t g_ms;
volatile uint32_t g_result;          /* 0 running, 1 OK, 2 no answer, 3 settings didn't check out */
volatile uint32_t g_found_baud;      /* baud the module answered at before setup */
volatile uint32_t g_attempts;
char g_before[640];                  /* AT+HELP before */
char g_after[640];                   /* AT+HELP after */
char g_log[900];                     /* commands and the start of each reply */
char g_report[2][240];               /* the radio report */

static char rx[640];
static const uint32_t k_bauds[] = {9600, 115200, 57600, 38400, 19200, 4800, 2400, 1200};

void SysTick_Handler(void) { g_ms++; }
static void delay_ms(uint32_t d) { uint32_t t = g_ms; while (g_ms - t < d) { } }
static void led(int on) { if (on) GPIOA->BSRR = 1u << 5; else GPIOA->BRR = 1u << 5; }

/* ------------------------------------------------------------------ text helpers (no printf) */
static void cat(char *dst, uint16_t size, const char *s)
{
    uint16_t n = (uint16_t)strlen(dst);
    while (*s && n + 1 < size) dst[n++] = *s++;
    dst[n] = 0;
}

static void cat_u32(char *dst, uint16_t size, uint32_t v)
{
    char t[11];
    int i = 10;
    t[i] = 0;
    do { t[--i] = (char)('0' + v % 10); v /= 10; } while (v);
    cat(dst, size, &t[i]);
}

/* Copy the value after `key` (up to the end of the line) from an AT+HELP reply. */
static void field(char *dst, uint16_t size, const char *src, const char *key)
{
    const char *p = strstr(src, key);
    if (!p) { cat(dst, size, "?"); return; }
    p += strlen(key);
    uint16_t n = (uint16_t)strlen(dst);
    while (*p && *p != '\r' && *p != '\n' && n + 1 < size) dst[n++] = *p++;
    dst[n] = 0;
}

static void log_line(const char *cmd, const char *reply)
{
    cat(g_log, sizeof g_log, cmd);
    cat(g_log, sizeof g_log, " -> ");
    char shortr[40];
    uint16_t i = 0;
    for (const char *p = reply; *p && i < sizeof shortr - 1; p++)
        shortr[i++] = (*p == '\r' || *p == '\n') ? ' ' : *p;
    shortr[i] = 0;
    cat(g_log, sizeof g_log, i ? shortr : "(no reply)");
    cat(g_log, sizeof g_log, "\n");
}

/* ------------------------------------------------------------------ USART2 (polling) */
static void uart_init(uint32_t baud)
{
    USART2->CR1 = 0;
    USART2->BRR = (F_CPU + baud / 2) / baud;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart_clear(void)
{
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_PECF;
    while (USART2->ISR & USART_ISR_RXNE) (void)USART2->RDR;
}

static void uart_write(const uint8_t *p, uint16_t n)
{
    while (n--) {
        while (!(USART2->ISR & USART_ISR_TXE)) { }
        USART2->TDR = *p++;
    }
    while (!(USART2->ISR & USART_ISR_TC)) { }
}

/* Collect bytes until `quiet` ms pass without one (after the first), or `timeout` ms in total. */
static uint16_t uart_read(char *buf, uint16_t max, uint32_t timeout, uint32_t quiet)
{
    uint16_t n = 0;
    uint32_t t0 = g_ms, last = 0;
    int got = 0;
    while (g_ms - t0 < timeout) {
        uint32_t isr = USART2->ISR;
        if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
            USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
        if (isr & USART_ISR_RXNE) {
            char c = (char)USART2->RDR;
            if (n + 1 < max) buf[n++] = c;
            last = g_ms;
            got = 1;
        } else if (got && g_ms - last >= quiet) {
            break;
        }
    }
    buf[n] = 0;
    return n;
}

/* ------------------------------------------------------------------ AT commands */
static void at(const char *cmd, uint32_t timeout)
{
    uart_clear();
    uart_write((const uint8_t *)cmd, (uint16_t)strlen(cmd));
    uart_write((const uint8_t *)"\r\n", 2);
    uart_read(rx, sizeof rx, timeout, 250);
    log_line(cmd, rx);
}

static int has(const char *pat) { return strstr(rx, pat) != 0; }

/* '+++' toggles: 1 = now in AT mode, 0 = now in data mode, -1 = no answer */
static int toggle(void)
{
    at("+++", 900);
    if (has("Entry")) return 1;
    if (has("Exit")) return 0;
    return -1;
}

static int enter_at(void)
{
    for (int i = 0; i < 2; i++) {
        int r = toggle();
        if (r == 1) return 1;
        if (r < 0) return 0;
        delay_ms(200);                       /* it was in AT mode and just left it: toggle again */
    }
    return 0;
}

static int leave_at(void)
{
    for (int i = 0; i < 2; i++) {
        int r = toggle();
        if (r == 0) return 1;
        if (r < 0) return 0;
        delay_ms(200);
    }
    return 0;
}

/* Baud rate the module answers '+++' at (left in AT mode), or 0. */
static uint32_t find_module(void)
{
    for (int pass = 0; pass < 3; pass++) {
        for (unsigned i = 0; i < sizeof k_bauds / sizeof k_bauds[0]; i++) {
            led(1);
            uart_init(k_bauds[i]);
            delay_ms(30);
            led(0);
            g_attempts++;
            if (enter_at()) return k_bauds[i];
            delay_ms(400);                   /* let any garbage from a wrong baud time out */
        }
    }
    return 0;
}

static int cmd_ok(const char *cmd)
{
    at(cmd, 1500);
    return has("OK");
}

static int settings_ok(const char *help)
{
    return strstr(help, "MODE:1") && strstr(help, "LEVEL:7") && strstr(help, "Spreading Factor:5") &&
           strstr(help, "MAC:0a,02") && strstr(help, ">> 03") && strstr(help, "Power:22dBm");
}

/* ------------------------------------------------------------------ report over the radio */
static void summary(char *dst, uint16_t size, const char *title, const char *help, uint32_t baud)
{
    dst[0] = 0;
    cat(dst, size, title);
    cat(dst, size, " baud ");
    cat_u32(dst, size, baud);
    cat(dst, size, " | MODE ");
    field(dst, size, help, "MODE:");
    cat(dst, size, " | LEVEL ");
    field(dst, size, help, "LEVEL:");
    cat(dst, size, " | SF ");
    field(dst, size, help, "Spreading Factor:");
    cat(dst, size, " | MAC ");
    field(dst, size, help, "MAC:");
    cat(dst, size, " | freq ");
    field(dst, size, help, "Frequency:");
    cat(dst, size, " | power ");
    field(dst, size, help, "Power:");
    cat(dst, size, " | CRC ");
    field(dst, size, help, "CRC:");
    cat(dst, size, " | fw ");
    field(dst, size, help, "+VERSION=");
    cat(dst, size, "\r\n");
}

static void radio_send(const char *text)
{
    static const uint8_t hdr[3] = {0x0A, 0x01, 0x03};     /* to the PC module, channel 03 */
    uint16_t n = (uint16_t)strlen(text);
    if (n > 240) n = 240;                                 /* one write <= 250 B incl. header */
    uart_write(hdr, 3);
    uart_write((const uint8_t *)text, n);
}

/* ------------------------------------------------------------------ main */
static uint32_t run_setup(void)
{
    uint32_t baud = find_module();
    if (!baud) return 2;
    g_found_baud = baud;

    at("AT+HELP", 3000);
    if (!g_before[0]) memcpy(g_before, rx, sizeof g_before);

    cmd_ok("AT+MODE1");
    cmd_ok("AT+LEVEL7");                  /* LEVEL 7 = SF5; resets an earlier AT+SF12 */
    cmd_ok("AT+CHANNEL03");
    cmd_ok("AT+MAC0a,02");
    cmd_ok("AT+POWE22");
    if (baud != 115200) cmd_ok("AT+BAUD8");   /* 115200, applied at the reset */
    at("AT+RESET", 1500);
    delay_ms(2500);                       /* reboot: prints "Power on" ~0.5 s later */

    uart_init(115200);
    delay_ms(50);
    if (!enter_at()) {
        uint32_t b = find_module();       /* not at 115200: find out where it went */
        if (!b) return 2;
        at("AT+HELP", 3000);
        memcpy(g_after, rx, sizeof g_after);
        leave_at();
        return 3;
    }
    at("AT+HELP", 3000);
    memcpy(g_after, rx, sizeof g_after);
    at("AT+BAUD", 1500);
    int ok = settings_ok(g_after) && has("+BAUD=8");
    leave_at();
    return ok ? 1u : 3u;
}

int main(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    /* PA5 LED output; PA2 = USART2_TX, PA3 = USART2_RX (AF1, pull-up on RX) */
    GPIOA->MODER = (GPIOA->MODER & ~((3u << 10) | (3u << 4) | (3u << 6))) | (1u << 10) | (2u << 4) | (2u << 6);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFu << 8) | (0xFu << 12))) | (1u << 8) | (1u << 12);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3u << 6)) | (1u << 6);
    SysTick_Config(F_CPU / 1000u);

    delay_ms(1500);                       /* the module boots with the board */
    for (int attempt = 0; attempt < 2 && g_result != 1; attempt++)
        g_result = run_setup();

    if (g_result == 1) {
        summary(g_report[0], sizeof g_report[0], "LORA MODULE SETUP OK. BEFORE:", g_before, g_found_baud);
        summary(g_report[1], sizeof g_report[1], "AFTER:", g_after, 115200);
        led(1);
        uart_init(115200);
        for (;;) {                        /* report every 3 s; writes >= 120 ms apart, <= 250 B */
            radio_send(g_report[0]);
            delay_ms(600);
            radio_send(g_report[1]);
            delay_ms(2400);
        }
    }
    for (;;) {                            /* 2: fast blink (no answer)   3: slow blink (check failed) */
        led(1);
        delay_ms(g_result == 2 ? 50 : 500);
        led(0);
        delay_ms(g_result == 2 ? 50 : 500);
    }
}
