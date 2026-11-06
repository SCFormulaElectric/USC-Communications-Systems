/* Host tests for telem_core.c and diag_core.c (BUILD_INSTRUCTIONS.md section 8.1).
 * Built three times by run_host_tests.py: -DTELEM_BAMO_DIAG=0, 1 and 2.
 * A simulated main loop steps the clock 1 ms at a time, feeds CAN frames and LoRa bytes,
 * and records every 195 B write. */
#include <stdio.h>
#include <string.h>
#include "telem_core.h"
#if TELEM_BAMO_DIAG
#include "diag_core.h"
#endif

/* ------------------------------------------------------------------ mini test framework */
static int g_checks, g_fails;
static const char *g_test = "";
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; \
    printf("  FAIL [%s] %s:%d: %s\n", g_test, __FILE__, __LINE__, #c); } } while (0)
#define CHECK_EQ(a, b) do { long long _a = (long long)(a), _b = (long long)(b); g_checks++; \
    if (_a != _b) { g_fails++; printf("  FAIL [%s] %s:%d: %s == %s (got %lld, want %lld)\n", \
        g_test, __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)
#define TEST(name) static void name(void)
#define RUN(name) do { int _f = g_fails; g_test = #name; name(); \
    printf("%-44s %s\n", #name, g_fails == _f ? "ok" : "FAILED"); } while (0)

/* ------------------------------------------------------------------ simulation harness */
static uint32_t T;
#define MAX_W 400
static uint8_t W[MAX_W][TS_WRITE_SIZE];
static uint32_t WT[MAX_W];
static int NW;

static int NSENT, SEND_FAIL;
#if TELEM_BAMO_DIAG
static can_frame_t SENT[128];
static int fake_send(const can_frame_t *f)
{
    if (SEND_FAIL) return -1;
    if (NSENT < 128) SENT[NSENT] = *f;
    NSENT++;
    return 0;
}
#endif

static void reset(void)
{
    T = 0;
    telem_init(T);
#if TELEM_BAMO_DIAG
    diag_init(fake_send);
#endif
    NW = 0; NSENT = 0; SEND_FAIL = 0;
}

static void can_rx(uint32_t id, uint8_t ext, uint8_t dlc, const uint8_t *d)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id; f.ext = ext; f.dlc = dlc; f.t_ms = T;
    memcpy(f.data, d, dlc);
    telem_on_can(&f);
#if TELEM_BAMO_DIAG
    diag_on_can(&f);
#endif
}
#define CAN(id, ...)  do { const uint8_t _d[] = {__VA_ARGS__}; can_rx(id, 0, sizeof _d, _d); } while (0)
#define CANX(id, ...) do { const uint8_t _d[] = {__VA_ARGS__}; can_rx(id, 1, sizeof _d, _d); } while (0)

static void step_to(uint32_t t)
{
    while (T < t) {
        T++;
        telem_poll(T);
#if TELEM_BAMO_DIAG
        diag_poll(T);
#endif
        if (telem_write_due(T)) {
            uint16_t n = telem_build_write(T, NW < MAX_W ? W[NW] : W[MAX_W - 1]);
            CHECK_EQ(n, TS_WRITE_SIZE);
            if (NW < MAX_W) WT[NW] = T;
            NW++;
        }
    }
}

static const ts_packet_t *PK(int w, int k) { return (const ts_packet_t *)(W[w] + 3 + TS_PKT_SIZE * k); }
static uint8_t TYPE(int w, int k) { return PK(w, k)->hdr.ver_type & 0x0F; }

/* The most recent FAST packet in the recorded writes (any slot, any build flag). */
static __attribute__((unused)) const ts_fast_t *latest_fast(void)
{
    for (int w = NW - 1; w >= 0; w--)
        for (int k = 2; k >= 0; k--)
            if (TYPE(w, k) == TS_TYPE_FAST) return &PK(w, k)->p.fast;
    return 0;
}

static void lora_bytes(const uint8_t *b, int n) { for (int i = 0; i < n; i++) telem_lora_feed(b[i], T); }

static void make_cmd(uint8_t *c, uint8_t seq, uint8_t code, const uint8_t *args, int nargs)
{
    memset(c, 0, TS_CMD_SIZE);
    c[0] = 0xA5; c[1] = 0x5A; c[2] = 0x18; c[3] = seq; c[4] = 0x5A; c[5] = 0xC3; c[6] = code;
    if (nargs) memcpy(c + 7, args, nargs);
    uint16_t crc = ts_crc16(c, 22);
    c[22] = (uint8_t)crc; c[23] = (uint8_t)(crc >> 8);
}

static void cmd(uint8_t seq, uint8_t code, const uint8_t *args, int nargs)
{
    uint8_t c[TS_CMD_SIZE];
    make_cmd(c, seq, code, args, nargs);
    lora_bytes(c, TS_CMD_SIZE);
}
#define CMD(seq, code, ...) do { const uint8_t _a[] = {__VA_ARGS__}; cmd(seq, code, _a, sizeof _a); } while (0)

#if TELEM_BAMO_DIAG
static uint8_t last_st(void) { uint8_t s, st; diag_last_ack(&s, &st); return st; }
#endif

/* Collect EVENT records from packet 3 of the writes from index `from`. */
static int collect_events(int from, uint8_t *codes, uint16_t *values, int max)
{
    int n = 0;
    for (int w = from; w < NW && w < MAX_W; w++)
        for (int k = 0; k < 3; k++)
            if (TYPE(w, k) == TS_TYPE_EVENT) {
                const ts_event_t *e = &PK(w, k)->p.event;
                for (int i = 0; i < e->count && n < max; i++) { codes[n] = e->records[i].code; values[n] = e->records[i].value; n++; }
            }
    return n;
}

static int has_event(int from, uint8_t code, int value)
{
    uint8_t c[400]; uint16_t v[400];
    int n = collect_events(from, c, v, 400);
    for (int i = 0; i < n; i++) if (c[i] == code && (value < 0 || v[i] == value)) return 1;
    return 0;
}

/* ------------------------------------------------------------------ tests: CRC, transport */
TEST(crc_test_vector)
{
    CHECK_EQ(ts_crc16((const uint8_t *)"123456789", 9), 0x29B1);
    CHECK_EQ(TS_CRC_CHECK, 0x29B1);
}

TEST(tx_allowed_rules)
{
    CHECK(telem_tx_allowed(1000, 0, 0, 0, 195));          /* first write */
    CHECK(!telem_tx_allowed(1000, 0, 0, 1, 195));         /* DMA still running */
    CHECK(!telem_tx_allowed(1119, 1000, 1, 0, 195));      /* 119 ms after the previous */
    CHECK(telem_tx_allowed(1120, 1000, 1, 0, 195));       /* 120 ms */
    CHECK(telem_tx_allowed(1200, 1000, 1, 0, 250));
    CHECK(!telem_tx_allowed(1200, 1000, 1, 0, 251));      /* over 250 B */
}

TEST(batch_layout_and_timing)
{
    reset();
    step_to(1000 + 200 * 24);
    CHECK_EQ(NW, 25);
    uint8_t seq = PK(0, 0)->hdr.seq;
    for (int w = 0; w < NW; w++) {
        CHECK_EQ(WT[w], 1000 + 200 * w);                    /* deadlines t0 + 200 ms x k */
        CHECK_EQ(W[w][0], 0x0A); CHECK_EQ(W[w][1], 0x01); CHECK_EQ(W[w][2], 0x03);
        for (int k = 0; k < 3; k++) {
            const ts_packet_t *p = PK(w, k);
            CHECK_EQ(p->hdr.sync[0], 0xA5); CHECK_EQ(p->hdr.sync[1], 0x5A);
            CHECK_EQ(p->hdr.ver_type >> 4, 1);
            CHECK_EQ(p->hdr.seq, (uint8_t)(seq + 3 * w + k));   /* one counter for all packets */
            CHECK_EQ(p->crc, ts_crc16((const uint8_t *)p, 62));
        }
#if TELEM_BAMO_DIAG != 2
        int n = w % 5;
        CHECK_EQ(TYPE(w, 0), TS_TYPE_FAST);
        CHECK_EQ(TYPE(w, 1), TS_TYPE_FAST);
        CHECK_EQ(PK(w, 0)->hdr.uptime_ms, WT[w] - 100);     /* packet 1 = FAST from 100 ms earlier */
        CHECK_EQ(PK(w, 1)->hdr.uptime_ms, WT[w]);           /* packet 2 = FAST built now */
        if (n == 0) CHECK_EQ(TYPE(w, 2), TS_TYPE_THERM);
        else if (n == 1) CHECK_EQ(TYPE(w, 2), TS_TYPE_DIAG);
        else CHECK(TYPE(w, 2) == TS_TYPE_DIAG || TYPE(w, 2) == TS_TYPE_EVENT);
#else
        int fast = 0;
        for (int k = 0; k < 3; k++) fast += TYPE(w, k) == TS_TYPE_FAST;
        CHECK_EQ(fast, (w % 5) == 0 ? 1 : 0);                /* diag-first: FAST once per second */
        if (w % 5 == 0) CHECK_EQ(TYPE(w, 0), TS_TYPE_FAST);
#endif
    }
}

TEST(first_write_one_second_after_boot)
{
    reset();
    step_to(999);
    CHECK_EQ(NW, 0);
    step_to(1000);
    CHECK_EQ(NW, 1);
}

TEST(skipped_write_moves_to_next_deadline)
{
    reset();
    step_to(999);
    telem_write_skipped(1000);           /* e.g. DMA busy at the deadline */
    step_to(1199);
    CHECK_EQ(NW, 0);
    step_to(1200);
    CHECK_EQ(NW, 1);
    CHECK_EQ(WT[0], 1200);
}

/* ------------------------------------------------------------------ tests: CAN decoding */
#if TELEM_BAMO_DIAG != 2
TEST(fast_fields_decode_with_byte_order)
{
    reset();
    step_to(1149);
    /* sampled values: set before the 1150 sample, change before the 1200 sample */
    CAN(0x364, 0x00, 0x64);                      /* brake1 100 (BE) */
    CAN(0x365, 0x01, 0x2C);                      /* brake2 300 */
    CAN(0x201, 0x90, 0x34, 0x12);                /* torque cmd 0x1234 (LE) */
    step_to(1175);
    CAN(0x364, 0x12, 0x34);                      /* brake1 0x1234 (BE) */
    CAN(0x365, 0xAB, 0xCD);                      /* brake2 0xABCD */
    CAN(0x201, 0x90, 0xFF, 0x7F);                /* torque cmd 32767 = 100% */
    CAN(0x473, 0xFF, 0x38);                      /* commanded torque -200 = -20.0 Nm (BE) */
    CAN(0x181, 0x30, 0xFF, 0x7F, 0x00);          /* speed 32767 */
    CAN(0x181, 0xEB, 0x10, 0x27, 0x00);          /* DC bus 10000 */
    CAN(0x181, 0x49, 0x99, 0x24, 0x00);          /* motor temp 9369 */
    CAN(0x181, 0x4A, 0x00, 0x80, 0x00);          /* IGBT -32768 */
    CAN(0x181, 0x20, 0xF4, 0x01, 0x00);          /* I actual 500 */
    CAN(0x181, 0x40, 0x01, 0x80, 0x00);          /* KERN_STATUS 0x8001 */
    CAN(0x301, 0x01, 0x02, 0x03, 0x04, 0x55, 0x06, 0x07, 0x00);
    CAN(0x302, 0x0A, 0x0B, 0x0C, 0x0D, 0x3C, 0x14, 0x00, 0x00);
    CAN(0x303, 0xDE, 0xAD, 0xBE, 0xEF);
    CANX(0x1839F380, 0x11, 0x22, 0x2D, 0x44, 0x55, 0x66, 0x77, 0x88);   /* max cell 45 */
    CAN(0x588, 0x07);
    step_to(1200);
    const ts_fast_t *f = &PK(NW - 1, 1)->p.fast;          /* FAST built at 1200 */
    CHECK_EQ(WT[NW - 1], 1200);
    CHECK_EQ(f->brake1, 0x1234);
    CHECK_EQ(f->brake1_prev, 100);
    CHECK_EQ(f->brake2, 0xABCD);
    CHECK_EQ(f->brake2_prev, 300);
    CHECK_EQ(f->torque_cmd, 32767);
    CHECK_EQ(f->torque_cmd_prev, 0x1234);
    CHECK_EQ(f->torque_req, -200);
    CHECK_EQ(f->speed, 32767);
    CHECK_EQ(f->dc_bus, 10000);
    CHECK_EQ(f->motor_temp, 9369);
    CHECK_EQ(f->igbt_temp, -32768);
    CHECK_EQ(f->i_actual, 500);
    CHECK_EQ(f->kern_status, 0x8001);
    CHECK_EQ(f->pack_current, 0x0102);
    CHECK_EQ(f->pack_voltage, 0x0304);
    CHECK_EQ(f->pack_soc, 0x55);
    CHECK_EQ(f->pack_ocv, 0x0607);
    CHECK_EQ(f->soc16, 0x0A0B);
    CHECK_EQ(f->pack_resistance, 0x0C0D);
    CHECK_EQ(f->bms_high_temp, 60);
    CHECK_EQ(f->bms_low_temp, 20);
    CHECK_EQ(f->bms_fault_flags, 0xDEADBEEFu);
    CHECK_EQ(f->max_cell_temp, 45);
    CHECK_EQ(f->bms_fault_num, 7);
    CHECK_EQ(f->reserved29, 0);
    /* 0x66 is the other DC bus register */
    CAN(0x181, 0x66, 0x39, 0x30, 0x00);
    step_to(1400);
    CHECK_EQ(PK(NW - 1, 1)->p.fast.dc_bus, 12345);
}
#endif

#if TELEM_BAMO_DIAG != 2
TEST(fast_frame_count_and_ack_default)
{
    reset();
    step_to(1100);
    for (int i = 0; i < 37; i++) CAN(0x123, 0x00);        /* 37 frames in (1100, 1200] */
    step_to(1300);
    const ts_fast_t *f = &PK(NW - 1, 1)->p.fast;          /* built at 1200 */
    CHECK_EQ(WT[NW - 1], 1200);
    CHECK_EQ(f->can_frames_100ms, 37);
    CHECK_EQ(f->ack_seq, 0);
    CHECK_EQ(f->ack_status, 0xFF);                        /* no command yet */
    CHECK_EQ(telem_cnt.can_rx_unused, 37);                /* 0x123 isn't used by anything */
}
#endif

TEST(therm_and_diag_payloads)
{
    reset();
    step_to(500);
    for (int k = 0; k < 6; k++) CANX(0x1839F380u + k, (uint8_t)(k + 1), 2, 3, 4, 5, 6, 7, (uint8_t)(0x80 + k));
    CAN(0x181, 0xC8, 0x70, 0x17, 0x00);                   /* Nmax 6000 */
    CAN(0x181, 0x25, 0x10, 0x00, 0x00);
    CAN(0x181, 0xD9, 0xFE, 0xFF, 0x00);                   /* -2 */
    CAN(0x481, 0x12, 0x34);
    CAN(0x666, 0x56, 0x78);
    CANX(0x18EEFF80, 0x00);
    CAN(0x476, 0x00, 0x41);                               /* VCU motor temp: bytes kept as received */
    step_to(1000 + 200 * 4);
    int got_therm = 0, got_diag = 0;
    for (int w = 0; w < NW; w++) {
        if (TYPE(w, 2) == TS_TYPE_THERM) {
            const ts_therm_t *t = &PK(w, 2)->p.therm;
            CHECK_EQ(t->f380[0], 1); CHECK_EQ(t->f385[0], 6); CHECK_EQ(t->f383[7], 0x83);
            CHECK_EQ(t->seen, 0x7F);                      /* bits 0-5: 0x1839F380-385, bit 6: 0x476 */
            CHECK_EQ(t->motor_temp_vcu[0], 0x00);
            CHECK_EQ(t->motor_temp_vcu[1], 0x41);
            got_therm = 1;
        }
        if (TYPE(w, 2) == TS_TYPE_DIAG) {
            const ts_diag_t *g = &PK(w, 2)->p.diag;
            CHECK_EQ(g->fw_version, TS_FW_VERSION);
            CHECK_EQ(g->build_flag, TELEM_BAMO_DIAG);
            CHECK_EQ(g->nmax, 6000);
            CHECK_EQ(g->ramp, 16);
            CHECK_EQ(g->i_200pc, -2);
            CHECK_EQ(g->reg_seen, (1 << 0) | (1 << 1) | (1 << 6));
            CHECK_EQ(g->raw_481, 0x3412);                 /* bytes copied: low byte = B0 */
            CHECK_EQ(g->raw_666, 0x7856);
            CHECK_EQ(g->raw_seen, 0x03);
            CHECK_EQ(g->can_rx_total, 13);
            CHECK_EQ(g->can_rx_unused, 0);
            got_diag = 1;
        }
    }
    CHECK(got_therm);
    CHECK(got_diag);
    /* THERM 'seen' clears 2 s after the last frame */
    step_to(500 + 2001 + 1000);
    for (int w = NW - 5; w < NW; w++) if (TYPE(w, 2) == TS_TYPE_THERM) CHECK_EQ(PK(w, 2)->p.therm.seen, 0);
}

/* ------------------------------------------------------------------ tests: valid / status */
TEST(valid_bits_clear_after_timeouts)
{
    static const uint16_t to[TS_N_VALID_BITS] = TS_VALID_TIMEOUTS_MS;
    reset();
    T = 5000;
    CAN(0x301, 0, 0, 0, 0, 0, 0, 0);
    CAN(0x302, 0, 0, 0, 0, 0, 0);
    CAN(0x303, 0, 0, 0, 0);
    CANX(0x1839F380, 0, 0, 0);
    CAN(0x181, 0x30, 0, 0, 0);
    CAN(0x181, 0x49, 0, 0, 0);
    CAN(0x181, 0x20, 0, 0, 0);
    CAN(0x181, 0x40, 0, 0, 0);
    CAN(0x181, 0xEB, 0, 0, 0);
    CAN(0x364, 0, 0);
    CAN(0x365, 0, 0);
    CAN(0x473, 0, 0);
    CAN(0x201, 0x90, 0, 0);
    CAN(0x471, 1);
    CAN(0x588, 0);
    CANX(0x18EEFF80, 0);
    CHECK_EQ(telem_valid_now(5000), 0xFFFF);
    for (int b = 0; b < TS_N_VALID_BITS; b++) {
        CHECK(telem_valid_now(5000 + to[b]) & (1u << b));
        CHECK(!(telem_valid_now(5000 + to[b] + 1) & (1u << b)));
    }
    /* bit 5 also from 0x4A, bit 8 also from 0x66 */
    reset();
    T = 100;
    CAN(0x181, 0x4A, 0, 0, 0);
    CAN(0x181, 0x66, 0, 0, 0);
    CHECK_EQ(telem_valid_now(100), (1u << 5) | (1u << 8));
}

#if TELEM_BAMO_DIAG != 2
TEST(status_bits)
{
    reset();
    step_to(1050);
    CAN(0x470, 0x01);                          /* red light */
    CAN(0x471, 0x01);                          /* BSPD OK */
    CAN(0x468, 0x00);
    CAN(0x475, 0x01);                          /* R2D */
    CAN(0x201, 0x51, 0x00, 0x00);              /* inverter enable */
    CAN(0x777, 0x05, 0x00);                    /* dash button */
    telem_on_can_state(10, 130, 1, 0, T);      /* error passive */
    step_to(1100);                             /* mid FAST at 1100 */
    uint16_t st;
    step_to(1200);
    st = PK(NW - 1, 0)->p.fast.status;         /* the 1100 FAST */
    CHECK(st & TS_STATUS_RED_LIGHT);
    CHECK(st & TS_STATUS_BSPD_OK);
    CHECK(st & TS_STATUS_BMS_FAULT_MSG);
    CHECK(!(st & TS_STATUS_IMD_FAULT_MSG));
    CHECK(st & TS_STATUS_R2D);
    CHECK(st & TS_STATUS_INV_ENABLED);
    CHECK(st & TS_STATUS_DASH_BUTTON);
    CHECK(st & TS_STATUS_CAN_ERROR);
    st = PK(NW - 1, 1)->p.fast.status;         /* the 1200 FAST */
    CHECK(!(st & TS_STATUS_DASH_BUTTON));      /* only "since the previous FAST" */
    CHECK(!(st & TS_STATUS_BMS_FAULT_MSG));    /* 0x468 older than 100 ms */
    CAN(0x201, 0x51, 0x04, 0x00);              /* disable */
    CAN(0x470, 0x00);
    step_to(1400);
    st = PK(NW - 1, 1)->p.fast.status;
    CHECK(!(st & TS_STATUS_INV_ENABLED));
    CHECK(!(st & TS_STATUS_RED_LIGHT));
}
#endif

/* ------------------------------------------------------------------ tests: events */
TEST(events_fire_on_edges)
{
    reset();
    step_to(900);
    CAN(0x475, 0x00);                          /* first observation: no event */
    CAN(0x471, 0x01);
    step_to(910);
    CAN(0x475, 0x01);                          /* R2D on */
    CAN(0x471, 0x00);                          /* BSPD trip */
    step_to(920);
    CAN(0x475, 0x00);                          /* R2D off */
    CAN(0x471, 0x01);                          /* BSPD clear */
    CAN(0x468, 0x00);                          /* starts arriving */
    CAN(0x469, 0x00);
    CAN(0x588, 0x03);                          /* BMS fault number changed */
    CAN(0x201, 0x51, 0x04, 0x00);              /* inverter enable/disable command */
    CAN(0x201, 0x51, 0x04, 0x00);              /* same value: no second event */
    CAN(0x7E3, 0x01, 0x02);                    /* BMS clear */
    CAN(0x201, 0x8E, 0x00, 0x00);              /* Bamocar error clear */
    CAN(0x777, 0x34, 0x12);                    /* dash button */
    telem_on_can_state(0, 0, 1, 0, T);         /* error passive */
    telem_on_can_state(255, 0, 1, 1, T);       /* bus-off */
    {
        const uint8_t po[] = "noise Power on\r\n";
        lora_bytes(po, sizeof po - 1);
    }
    step_to(4000);                             /* 0x468/0x469 stop; sources time out */
    CHECK(has_event(0, TS_EVT_R2D_ON, -1));
    CHECK(has_event(0, TS_EVT_R2D_OFF, -1));
    CHECK(has_event(0, TS_EVT_BSPD_TRIP, -1));
    CHECK(has_event(0, TS_EVT_BSPD_CLEAR, -1));
    CHECK(has_event(0, TS_EVT_BMS_FAULT_MSG_START, -1));
    CHECK(has_event(0, TS_EVT_BMS_FAULT_MSG_STOP, -1));
    CHECK(has_event(0, TS_EVT_IMD_FAULT_MSG_START, -1));
    CHECK(has_event(0, TS_EVT_IMD_FAULT_MSG_STOP, -1));
    CHECK(has_event(0, TS_EVT_BMS_FAULT_CHANGED, 3));
    CHECK(has_event(0, TS_EVT_INV_ENABLE_CMD, 0x0004));
    CHECK(has_event(0, TS_EVT_BMS_CLEAR_SENT, 0x0201));
    CHECK(has_event(0, TS_EVT_BAMO_ERR_CLEAR_SENT, -1));
    CHECK(has_event(0, TS_EVT_DASH_BUTTON, 0x1234));
    CHECK(has_event(0, TS_EVT_CAN_ERROR_PASSIVE, -1));
    CHECK(has_event(0, TS_EVT_CAN_BUS_OFF, -1));
    CHECK(has_event(0, TS_EVT_LORA_POWER_ON, -1));
    CHECK(has_event(0, TS_EVT_SOURCE_BACK, TS_VB_GEVCU_471));
    CHECK(has_event(0, TS_EVT_SOURCE_TIMEOUT, TS_VB_GEVCU_471));
    CHECK(has_event(0, TS_EVT_SOURCE_BACK, TS_VB_BMS_588));
    CHECK(has_event(0, TS_EVT_SOURCE_TIMEOUT, TS_VB_BMS_588));
    uint8_t c[400]; uint16_t v[400];
    int n = collect_events(0, c, v, 400), inv = 0, r2d_on = 0;
    for (int i = 0; i < n; i++) { inv += c[i] == TS_EVT_INV_ENABLE_CMD; r2d_on += c[i] == TS_EVT_R2D_ON; }
    CHECK_EQ(inv, 1);
    CHECK_EQ(r2d_on, 1);
    CHECK_EQ(telem_cnt.can_busoff_count, 1);
    CHECK_EQ(telem_cnt.lora_power_on, 1);
}

#if TELEM_BAMO_DIAG != 2
TEST(event_records_age)
{
    reset();
    step_to(1300);
    CAN(0x7E3, 0x09, 0x00);                    /* at 1300; first EVENT slot is write 2 at 1400 */
    step_to(1400);
    CHECK_EQ(TYPE(NW - 1, 2), TS_TYPE_EVENT);
    const ts_event_t *e = &PK(NW - 1, 2)->p.event;
    CHECK_EQ(e->count, 1);
    CHECK_EQ(e->records[0].code, TS_EVT_BMS_CLEAR_SENT);
    CHECK_EQ(e->records[0].age_ms, 100);
    CHECK_EQ(e->records[0].value, 9);
}
#endif

TEST(event_ring_drops_oldest)
{
    reset();
    for (int i = 0; i < 40; i++) telem_push_event((uint8_t)(100 + i), (uint16_t)i, 0);
    CHECK_EQ(telem_cnt.events_dropped, 8);
    step_to(1000 + 200 * 15);
    uint8_t c[400]; uint16_t v[400];
    int n = collect_events(0, c, v, 400);
    CHECK_EQ(n, 33);                           /* drop record + the newest 32 */
    CHECK_EQ(c[0], TS_EVT_EVENTS_DROPPED);
    CHECK_EQ(v[0], 8);
    CHECK_EQ(v[1], 8);                         /* oldest kept is #8 */
    CHECK_EQ(v[32], 39);
    for (int w = 0; w < NW; w++) if (TYPE(w, 2) == TS_TYPE_DIAG) { CHECK_EQ(PK(w, 2)->p.diag.events_dropped, 8); break; }
}

/* ------------------------------------------------------------------ tests: LoRa parser */
TEST(lora_parser_bad_crc_key_and_text)
{
    reset();
    uint8_t c[TS_CMD_SIZE], b[TS_CMD_SIZE];
    make_cmd(c, 1, TS_CMD_PING, 0, 0);
    uint8_t got[TS_CMD_SIZE];
    int n = 0;
    /* good command after garbage */
    const uint8_t junk[] = {0x00, 0xA5, 0x11, 0xA5, 0xA5, 0x5A, 0x01};
    for (unsigned i = 0; i < sizeof junk; i++) n += telem_lora_rx_byte(junk[i], T, got);
    for (int i = 0; i < TS_CMD_SIZE; i++) n += telem_lora_rx_byte(c[i], T, got);
    /* (the A5 5A in the junk opens a candidate that fails its CRC; the parser recovers) */
    CHECK_EQ(n, 1);
    CHECK(memcmp(got, c, TS_CMD_SIZE) == 0);
    uint16_t bad0 = telem_cnt.cmd_bad;
    memcpy(b, c, sizeof b); b[22] ^= 1;                     /* bad CRC */
    n = 0;
    for (int i = 0; i < TS_CMD_SIZE; i++) n += telem_lora_rx_byte(b[i], T, got);
    CHECK_EQ(n, 0);
    CHECK_EQ(telem_cnt.cmd_bad, bad0 + 1);
    make_cmd(b, 2, TS_CMD_PING, 0, 0); b[5] = 0xC4;          /* bad key (CRC recomputed) */
    uint16_t crc = ts_crc16(b, 22); b[22] = (uint8_t)crc; b[23] = (uint8_t)(crc >> 8);
    n = 0;
    for (int i = 0; i < TS_CMD_SIZE; i++) n += telem_lora_rx_byte(b[i], T, got);
    CHECK_EQ(n, 0);
    CHECK_EQ(telem_cnt.cmd_bad, bad0 + 2);
    /* module text */
    const uint8_t txt[] = "Power on\r\nchannel Error:66\r\nchannel Error:31\r\n";
    for (unsigned i = 0; i < sizeof txt - 1; i++) telem_lora_rx_byte(txt[i], T, got);
    CHECK_EQ(telem_cnt.lora_power_on, 1);
    CHECK_EQ(telem_cnt.lora_channel_errors, 2);
#if TELEM_BAMO_DIAG
    /* bad commands are not acknowledged */
    reset();
    lora_bytes(b, TS_CMD_SIZE);
    CHECK_EQ(last_st(), TS_ST_NONE);
    CHECK_EQ(diag_cmd_executed(), 0);
    CHECK(!diag_session_active());
#endif
}

#if TELEM_BAMO_DIAG == 0
TEST(flag0_ignores_commands)
{
    reset();
    step_to(900);
    CMD(1, TS_CMD_PING);
    CMD(2, TS_CMD_BAMO_WRITE, 0x35, 2, 0x10, 0x00, 0, 0);
    step_to(1400);
    const ts_fast_t *f = &PK(NW - 1, 1)->p.fast;
    CHECK_EQ(f->ack_seq, 0);
    CHECK_EQ(f->ack_status, 0xFF);
    CHECK(!(f->status & TS_STATUS_DIAG_SESSION));
    CHECK_EQ(telem_cnt.cmd_bad, 0);
    for (int w = 0; w < NW; w++) for (int k = 0; k < 3; k++) CHECK(TYPE(w, k) != TS_TYPE_DIAG_RESP);
}
#endif

/* ------------------------------------------------------------------ tests: diagnostics */
#if TELEM_BAMO_DIAG
static const uint8_t k_blocked[] = {0x90, 0x31, 0x6E, 0x51, 0x68, 0x84, 0xDC};

TEST(diag_blocked_registers_write_and_send_frame)
{
    reset();
    T = 100;
    uint8_t seq = 1;
    for (unsigned i = 0; i < sizeof k_blocked; i++) {
        CMD(seq++, TS_CMD_BAMO_WRITE, k_blocked[i], 2, 0x01, 0x00, 0, 0);
        CHECK_EQ(last_st(), TS_ST_REJ_REGID);
        CMD(seq++, TS_CMD_BAMO_WRITE, k_blocked[i], 4, 0x01, 0x00, 0, 0);
        CHECK_EQ(last_st(), TS_ST_REJ_REGID);
        CMD(seq++, TS_CMD_SEND_FRAME, 0x01, 0x02, 0, 0, 3, k_blocked[i], 0x01, 0x00);
        CHECK_EQ(last_st(), TS_ST_REJ_REGID);
    }
    CHECK_EQ(NSENT, 0);
    /* allowed register: 2-byte and 4-byte writes */
    CMD(seq++, TS_CMD_BAMO_WRITE, 0x35, 2, 0x34, 0x12, 0, 0);
    CHECK_EQ(last_st(), TS_ST_OK);
    CHECK_EQ(NSENT, 1);
    CHECK_EQ(SENT[0].id, 0x201); CHECK_EQ(SENT[0].ext, 0); CHECK_EQ(SENT[0].dlc, 3);
    CHECK_EQ(SENT[0].data[0], 0x35); CHECK_EQ(SENT[0].data[1], 0x34); CHECK_EQ(SENT[0].data[2], 0x12);
    CMD(seq++, TS_CMD_BAMO_WRITE, 0x35, 4, 0x78, 0x56, 0x34, 0x12);
    CHECK_EQ(last_st(), TS_ST_OK);
    CHECK_EQ(SENT[1].dlc, 5);
    CHECK_EQ(SENT[1].data[4], 0x12);
    CMD(seq++, TS_CMD_BAMO_WRITE, 0x35, 3, 0, 0, 0, 0);       /* bad size */
    CHECK_EQ(last_st(), TS_ST_BAD_ARGS);
    CMD(seq++, TS_CMD_BAMO_WRITE, 0x3D, 2, 0x35, 0x00, 0, 0); /* a read disguised as a write */
    CHECK_EQ(last_st(), TS_ST_BAD_ARGS);
    CMD(seq++, TS_CMD_SEND_FRAME, 0x01, 0x02, 0, 0, 3, 0x35, 0x01, 0x00);   /* SEND_FRAME, allowed reg */
    CHECK_EQ(last_st(), TS_ST_OK);
    CMD(seq++, TS_CMD_SEND_FRAME, 0x00, 0x01, 0, 0, 1, 0x00);              /* other ID */
    CHECK_EQ(last_st(), DIAG_ANY_ID ? TS_ST_OK : TS_ST_REJ_ID);
    CMD(seq++, TS_CMD_SEND_FRAME, 0x01, 0x02, 0, 0, 9, 0x35);              /* dlc 9 */
    CHECK_EQ(last_st(), TS_ST_BAD_ARGS);
    CMD(seq++, TS_CMD_SEND_FRAME, 0x01, 0x02, 0, 0x80, 1, 0x35);           /* extended 0x201: not the Bamocar */
    CHECK_EQ(last_st(), DIAG_ANY_ID ? TS_ST_OK : TS_ST_REJ_ID);
    CMD(seq++, 0x55);                                                       /* unknown code */
    CHECK_EQ(last_st(), TS_ST_BAD_ARGS);
}

TEST(diag_reads_and_repeating_limit)
{
    reset();
    T = 100;
    uint8_t seq = 1;
    CMD(seq++, TS_CMD_BAMO_READ, 0x30, 0);                  /* GEVCU's repeating registers */
    CHECK_EQ(last_st(), TS_ST_REJ_REGID);
    CMD(seq++, TS_CMD_BAMO_READ, 0x49, 0xFF);
    CHECK_EQ(last_st(), TS_ST_REJ_REGID);
    CMD(seq++, TS_CMD_BAMO_READ, 0x4A, 100);
    CHECK_EQ(last_st(), TS_ST_REJ_REGID);
    CMD(seq++, TS_CMD_SEND_FRAME, 0x01, 0x02, 0, 0, 3, 0x3D, 0x30, 0x00);  /* same via SEND_FRAME */
    CHECK_EQ(last_st(), TS_ST_REJ_REGID);
    CHECK_EQ(NSENT, 0);
    CMD(seq++, TS_CMD_BAMO_READ, 0x8F, 0);                  /* one-shot */
    CHECK_EQ(last_st(), TS_ST_OK);
    CHECK_EQ(SENT[0].dlc, 3); CHECK_EQ(SENT[0].data[0], 0x3D); CHECK_EQ(SENT[0].data[1], 0x8F); CHECK_EQ(SENT[0].data[2], 0);
    CMD(seq++, TS_CMD_BAMO_READ, 0xC8, 19);                 /* period below 20 ms */
    CHECK_EQ(last_st(), TS_ST_BAD_ARGS);
    CMD(seq++, TS_CMD_BAMO_READ, 0xC8, 100);
    CHECK_EQ(last_st(), TS_ST_OK);
    CMD(seq++, TS_CMD_BAMO_READ, 0xC4, 20);
    CHECK_EQ(last_st(), TS_ST_OK);
    CMD(seq++, TS_CMD_SEND_FRAME, 0x01, 0x02, 0, 0, 3, 0x3D, 0xC5, 254);   /* repeating via SEND_FRAME */
    CHECK_EQ(last_st(), TS_ST_OK);
    CHECK_EQ(diag_n_repeat_reads(), 3);
    CMD(seq++, TS_CMD_BAMO_READ, 0xC6, 100);                /* 4th */
    CHECK_EQ(last_st(), TS_ST_REJ_LIMIT);
    CMD(seq++, TS_CMD_BAMO_READ, 0xC8, 200);                /* changing an existing one is fine */
    CHECK_EQ(last_st(), TS_ST_OK);
    CHECK_EQ(diag_n_repeat_reads(), 3);
    CMD(seq++, TS_CMD_BAMO_READ, 0xC8, 0xFF);               /* stop frees a slot */
    CHECK_EQ(last_st(), TS_ST_OK);
    CHECK_EQ(SENT[NSENT - 1].data[2], 0xFF);
    CHECK_EQ(diag_n_repeat_reads(), 2);
    CMD(seq++, TS_CMD_BAMO_READ, 0xC6, 100);
    CHECK_EQ(last_st(), TS_ST_OK);
}

TEST(diag_inverter_live_rules)
{
    uint8_t seq = 1;
    int cond;
    for (cond = 0; cond < 4; cond++) {
        reset();
        T = 20000;
        if (cond == 0) CAN(0x201, 0x51, 0x00, 0x00);        /* enabled */
        if (cond == 1) CAN(0x475, 0x01);                    /* R2D */
        if (cond == 2) CAN(0x473, 0x00, 0x10);              /* torque request */
        if (cond == 3) CAN(0x181, 0x40, 0x01, 0x00, 0x00);  /* KERN_STATUS bit 0 */
        CHECK(diag_inverter_live(T));
        CMD(seq++, TS_CMD_BAMO_WRITE, 0x35, 2, 0x10, 0x00, 0, 0);
        CHECK_EQ(last_st(), TS_ST_REJ_INV_ENABLED);
        CMD(seq++, TS_CMD_SEND_FRAME, 0x01, 0x02, 0, 0, 3, 0x35, 0x10, 0x00);
        CHECK_EQ(last_st(), TS_ST_REJ_INV_ENABLED);
        CMD(seq++, TS_CMD_BAMO_READ, 0xC8, 100);            /* repeating read */
        CHECK_EQ(last_st(), TS_ST_REJ_INV_ENABLED);
        CMD(seq++, TS_CMD_BAMO_READ, 0xC8, 0xFF);           /* stop is not a one-shot read */
        CHECK_EQ(last_st(), TS_ST_REJ_INV_ENABLED);
        CHECK_EQ(NSENT, 0);
        CMD(seq++, TS_CMD_BAMO_READ, 0xC8, 0);              /* one-shot read is allowed */
        CHECK_EQ(last_st(), TS_ST_OK);
        CMD(seq++, TS_CMD_PING);
        CHECK_EQ(last_st(), TS_ST_OK);
        CMD(seq++, TS_CMD_CAPTURE_SET, 0, 0x81, 0x01, 0, 0, 0xFF, 0x07, 0, 0, 10);
        CHECK_EQ(last_st(), TS_ST_OK);
        CMD(seq++, TS_CMD_CAPTURE_CLEAR, 0xFF);
        CHECK_EQ(last_st(), TS_ST_OK);
        CMD(seq++, TS_CMD_BAMO_WRITE, 0x90, 2, 0, 0, 0, 0); /* blocked register still says REJ_REGID */
        CHECK_EQ(last_st(), TS_ST_REJ_REGID);
        CMD(seq++, TS_CMD_STOP_ALL);
        CHECK_EQ(last_st(), TS_ST_OK);
    }
    /* windows expire */
    reset(); T = 1000; CAN(0x475, 0x01);
    CHECK(diag_inverter_live(11000)); CHECK(!diag_inverter_live(11001));
    reset(); T = 1000; CAN(0x473, 0x00, 0x00);
    CHECK(diag_inverter_live(3000)); CHECK(!diag_inverter_live(3001));
    reset(); T = 1000; CAN(0x181, 0x40, 0x01, 0x00, 0x00);
    CHECK(diag_inverter_live(3000)); CHECK(!diag_inverter_live(3001));
    reset(); T = 1000; CAN(0x181, 0x40, 0x00, 0x00, 0x00);  /* bit 0 clear */
    CHECK(!diag_inverter_live(1000));
    reset(); T = 1000; CAN(0x475, 0x00);                    /* R2D off */
    CHECK(!diag_inverter_live(1000));
    reset(); T = 1000; CAN(0x201, 0x51, 0x00, 0x00); CAN(0x201, 0x51, 0x04, 0x00);  /* enabled then disabled */
    CHECK(!diag_inverter_live(1000));
    CMD(1, TS_CMD_BAMO_WRITE, 0x35, 2, 0x10, 0x00, 0, 0);
    CHECK_EQ(last_st(), TS_ST_OK);
}

TEST(diag_rate_limit)
{
    reset();
    T = 1000;
    uint8_t seq = 1;
    for (int i = 0; i < 20; i++) {
        CMD(seq++, TS_CMD_BAMO_READ, 0x8F, 0);
        CHECK_EQ(last_st(), TS_ST_OK);
    }
    CMD(seq++, TS_CMD_BAMO_READ, 0x8F, 0);                  /* 21st in the same instant */
    CHECK_EQ(last_st(), TS_ST_REJ_RATE);
    CHECK_EQ(NSENT, 20);
    T += 49;
    CMD(seq++, TS_CMD_BAMO_READ, 0x8F, 0);
    CHECK_EQ(last_st(), TS_ST_REJ_RATE);
    T += 1;                                                 /* one token back every 50 ms */
    CMD(seq++, TS_CMD_BAMO_READ, 0x8F, 0);
    CHECK_EQ(last_st(), TS_ST_OK);
    /* sustained: at most 20 per second */
    reset();
    T = 1000;
    int ok = 0;
    for (int i = 0; i < 200; i++) {          /* one attempt every 10 ms for 2 s */
        CMD((uint8_t)(i + 1), TS_CMD_BAMO_READ, 0x8F, 0);
        ok += last_st() == TS_ST_OK;
        T += 10;
    }
    CHECK(ok <= 20 + 40);                    /* 20 burst + 2 s x 20/s */
    CHECK(ok >= 20 + 39);
}

TEST(diag_session_timeout_stops_reads)
{
    reset();
    step_to(500);
    CMD(1, TS_CMD_BAMO_READ, 0xC8, 100);
    CMD(2, TS_CMD_BAMO_READ, 0xC4, 50);
    CMD(3, TS_CMD_CAPTURE_SET, 0, 0x81, 0x01, 0, 0, 0xFF, 0x07, 0, 0, 10);
    CHECK(diag_session_active());
    CHECK_EQ(NSENT, 2);
    step_to(500 + 10000);                    /* exactly 10 s: still open */
    CHECK(diag_session_active());
    step_to(500 + 10001);
    CHECK(!diag_session_active());
    CHECK_EQ(NSENT, 4);
    int stops = 0;
    for (int i = 2; i < NSENT; i++)
        if (SENT[i].id == 0x201 && SENT[i].dlc == 3 && SENT[i].data[0] == 0x3D && SENT[i].data[2] == 0xFF &&
            (SENT[i].data[1] == 0xC8 || SENT[i].data[1] == 0xC4)) stops++;
    CHECK_EQ(stops, 2);
    CHECK_EQ(diag_n_repeat_reads(), 0);
    CAN(0x181, 0xC8, 1, 2, 3);               /* captures were cleared */
    CHECK(!diag_resp_pending());
    /* the status bit follows the session */
    CMD(4, TS_CMD_PING);
    step_to(T + 1000);
    CHECK(latest_fast()->status & TS_STATUS_DIAG_SESSION);
    CMD(5, TS_CMD_STOP_ALL);
    CHECK(!diag_session_active());
    step_to(T + 1000);
    CHECK(!(latest_fast()->status & TS_STATUS_DIAG_SESSION));
}

TEST(diag_same_sequence_not_run_twice)
{
    reset();
    T = 100;
    CMD(7, TS_CMD_BAMO_WRITE, 0x35, 2, 0x01, 0x00, 0, 0);
    CHECK_EQ(NSENT, 1);
    CHECK_EQ(diag_cmd_executed(), 1);
    CMD(7, TS_CMD_BAMO_WRITE, 0x35, 2, 0x01, 0x00, 0, 0);   /* PC resend */
    CMD(7, TS_CMD_BAMO_WRITE, 0x35, 2, 0x01, 0x00, 0, 0);
    CHECK_EQ(NSENT, 1);
    CHECK_EQ(diag_cmd_executed(), 1);
    CHECK_EQ(last_st(), TS_ST_OK);
    ts_diag_resp_t r;
    memset(&r, 0, sizeof r);
    diag_build_resp(&r, T);
    CHECK_EQ(r.n_acks, 3);                                 /* acknowledged each time */
    CHECK_EQ(r.acks[2].seq, 7); CHECK_EQ(r.acks[2].status, TS_ST_OK);
    /* a rejected command repeated keeps its stored status */
    CMD(8, TS_CMD_BAMO_WRITE, 0x90, 2, 0, 0, 0, 0);
    CMD(8, TS_CMD_BAMO_WRITE, 0x35, 2, 0, 0, 0, 0);         /* same seq, different body: not run */
    CHECK_EQ(last_st(), TS_ST_REJ_REGID);
    CHECK_EQ(NSENT, 1);
    /* FAST bytes 52-53 carry the acknowledgement */
    step_to(2400);
    CHECK_EQ(latest_fast()->ack_seq, 8);
    CHECK_EQ(latest_fast()->ack_status, TS_ST_REJ_REGID);
}

TEST(diag_tx_fail)
{
    reset();
    T = 100;
    SEND_FAIL = 1;
    CMD(1, TS_CMD_BAMO_READ, 0x8F, 0);
    CHECK_EQ(last_st(), TS_ST_TX_FAIL);
    CMD(2, TS_CMD_BAMO_READ, 0xC8, 100);
    CHECK_EQ(last_st(), TS_ST_TX_FAIL);
    CHECK_EQ(diag_n_repeat_reads(), 0);                    /* not counted as started */
}

TEST(diag_captures_and_diag_resp)
{
    reset();
    step_to(900);
    CMD(1, TS_CMD_CAPTURE_SET, 0, 0x81, 0x01, 0, 0, 0xFF, 0x07, 0, 0, 2);  /* 0x181, 2 frames/s */
    CHECK_EQ(last_st(), TS_ST_OK);
    CMD(2, TS_CMD_CAPTURE_SET, 4, 0, 0, 0, 0, 0, 0, 0, 0, 2);                /* bad slot */
    CHECK_EQ(last_st(), TS_ST_BAD_ARGS);
    CMD(3, TS_CMD_CAPTURE_SET, 1, 0x80, 0xF3, 0x39, 0x98, 0xF0, 0xFF, 0xFF, 0x9F, 5);  /* ext 0x1839F380, mask */
    CHECK_EQ(last_st(), TS_ST_OK);
    CAN(0x181, 0xC8, 0x70, 0x17, 0x00);
    CAN(0x181, 0x8F, 0x20, 0x00, 0x00, 0x00, 0x00);
    CAN(0x181, 0x25, 0x01, 0x00, 0x00);       /* 3rd within 1 s: over the slot rate */
    CAN(0x182, 0x00);                         /* no match */
    CANX(0x1839F383, 0x01, 0x02);             /* matches slot 1 via the mask */
    CAN(0x183, 0x00);
    step_to(1400);
    int frames = 0, acks = 0, dropped = 0;
    uint32_t ids[8] = {0};
    for (int w = 0; w < NW; w++)
        for (int k = 0; k < 3; k++)
            if (TYPE(w, k) == TS_TYPE_DIAG_RESP) {
                const ts_diag_resp_t *r = &PK(w, k)->p.diag_resp;
                for (int i = 0; i < r->n_frames; i++) if (frames < 8) ids[frames++] = r->frames[i].id;
                acks += r->n_acks;
                dropped += r->cap_dropped;
            }
    CHECK_EQ(frames, 3);
    CHECK_EQ(ids[0], 0x181); CHECK_EQ(ids[1], 0x181);
    CHECK_EQ(ids[2], 0x1839F383u | CAN_EXT_FLAG);
    CHECK_EQ(acks, 3);
    CHECK_EQ(dropped, 1);
    CHECK_EQ(diag_capture_dropped(), 1);
    /* the capture ring holds 16 */
    reset();
    T = 100;
    CMD(1, TS_CMD_CAPTURE_SET, 0, 0x81, 0x01, 0, 0, 0xFF, 0x07, 0, 0, 255);
    for (int i = 0; i < 20; i++) CAN(0x181, (uint8_t)i, 0, 0);
    CHECK_EQ(diag_capture_dropped(), 4);
    CMD(2, TS_CMD_CAPTURE_CLEAR, 0);
    CAN(0x181, 0x01, 0, 0);
    CHECK_EQ(diag_capture_dropped(), 4);
    CMD(3, TS_CMD_CAPTURE_CLEAR, 9);
    CHECK_EQ(last_st(), TS_ST_BAD_ARGS);
}

TEST(diag_resp_is_sent_in_batches)
{
    reset();
    step_to(1300);
    CMD(1, TS_CMD_PING);
    step_to(1600);
    int found = 0;
    for (int w = 0; w < NW; w++)
        for (int k = 0; k < 3; k++)
            if (TYPE(w, k) == TS_TYPE_DIAG_RESP) {
                found = 1;
#if TELEM_BAMO_DIAG == 2
                CHECK_EQ(k, 0);               /* diag-first: DIAG_RESP fills slot 1 */
#else
                CHECK_EQ(k, 2);
                CHECK(w % 5 >= 2);
#endif
            }
    CHECK(found);
}
#endif /* TELEM_BAMO_DIAG */

#if TELEM_BAMO_DIAG == 2
TEST(flag2_fast_once_per_second_with_values)
{
    reset();
    step_to(1990);
    CAN(0x181, 0x30, 0xFF, 0x7F, 0x00);
    CAN(0x364, 0x12, 0x34);
    CAN(0x470, 0x01);
    for (int i = 0; i < 5; i++) CAN(0x123, 0x00);
    step_to(2000);
    CHECK_EQ(TYPE(NW - 1, 0), TS_TYPE_FAST);
    const ts_fast_t *f = &PK(NW - 1, 0)->p.fast;
    CHECK_EQ(PK(NW - 1, 0)->hdr.uptime_ms, 2000);
    CHECK_EQ(f->speed, 32767);
    CHECK_EQ(f->brake1, 0x1234);
    CHECK(f->status & TS_STATUS_RED_LIGHT);
    CHECK_EQ(f->can_frames_100ms, 8);         /* the 8 frames at 1990 are in the (1900, 2000] window */
    CHECK_EQ(f->ack_status, 0xFF);
}

TEST(flag2_fill_order)
{
    reset();
    step_to(1000 + 200 * 9);
    int therm = 0, diag = 0;
    for (int w = 0; w < NW; w++)
        for (int k = 0; k < 3; k++) {
            therm += TYPE(w, k) == TS_TYPE_THERM;
            diag += TYPE(w, k) == TS_TYPE_DIAG;
        }
    CHECK(therm > 0 && diag > 0);             /* filler alternates THERM / DIAG */
    CHECK(therm - diag <= 1 && diag - therm <= 1);
    reset();
    step_to(1300);
    telem_push_event(TS_EVT_DASH_BUTTON, 1, T);
    CMD(1, TS_CMD_PING);
    step_to(1400);                            /* write 2 (n = 2): DIAG_RESP, EVENT, filler */
    CHECK_EQ(TYPE(NW - 1, 0), TS_TYPE_DIAG_RESP);
    CHECK_EQ(TYPE(NW - 1, 1), TS_TYPE_EVENT);
}
#endif

/* ------------------------------------------------------------------ main */
int main(void)
{
    printf("== host tests, TELEM_BAMO_DIAG=%d ==\n", TELEM_BAMO_DIAG);
    RUN(crc_test_vector);
    RUN(tx_allowed_rules);
    RUN(batch_layout_and_timing);
    RUN(first_write_one_second_after_boot);
    RUN(skipped_write_moves_to_next_deadline);
#if TELEM_BAMO_DIAG != 2
    RUN(fast_fields_decode_with_byte_order);
#endif
#if TELEM_BAMO_DIAG != 2
    RUN(fast_frame_count_and_ack_default);
#endif
    RUN(therm_and_diag_payloads);
    RUN(valid_bits_clear_after_timeouts);
#if TELEM_BAMO_DIAG != 2
    RUN(status_bits);
#endif
    RUN(events_fire_on_edges);
#if TELEM_BAMO_DIAG != 2
    RUN(event_records_age);
#endif
    RUN(event_ring_drops_oldest);
    RUN(lora_parser_bad_crc_key_and_text);
#if TELEM_BAMO_DIAG == 0
    RUN(flag0_ignores_commands);
#endif
#if TELEM_BAMO_DIAG
    RUN(diag_blocked_registers_write_and_send_frame);
    RUN(diag_reads_and_repeating_limit);
    RUN(diag_inverter_live_rules);
    RUN(diag_rate_limit);
    RUN(diag_session_timeout_stops_reads);
    RUN(diag_same_sequence_not_run_twice);
    RUN(diag_tx_fail);
    RUN(diag_captures_and_diag_resp);
    RUN(diag_resp_is_sent_in_batches);
#endif
#if TELEM_BAMO_DIAG == 2
    RUN(flag2_fast_once_per_second_with_values);
    RUN(flag2_fill_order);
#endif
    printf("%d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
