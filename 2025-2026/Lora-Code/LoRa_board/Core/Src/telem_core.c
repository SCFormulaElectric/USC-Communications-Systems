/* Telemetry logic (BUILD_INSTRUCTIONS.md sections 3 and 5). No HAL includes. */
#include <string.h>
#include "telem_core.h"
#if TELEM_BAMO_DIAG
#include "diag_core.h"
#endif

telem_counters_t telem_cnt;

/* ------------------------------------------------------------------ helpers */
static uint16_t be16(const uint8_t *d) { return (uint16_t)((d[0] << 8) | d[1]); }
static uint16_t le16(const uint8_t *d) { return (uint16_t)(d[0] | (d[1] << 8)); }
static uint32_t be32(const uint8_t *d)
{
    return ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
}
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
/* true when `t` is at most `win` ms before `now` (t may be a little after now: ISR timestamps) */
static int within(uint32_t now, uint32_t t, uint32_t win) { return (int32_t)(now - t) <= (int32_t)win; }
static int reached(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }

uint16_t ts_crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = TS_CRC_INIT;
    while (n--) {
        crc ^= (uint16_t)(*p++ << 8);
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ TS_CRC_POLY) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ------------------------------------------------------------------ state */
static const uint16_t k_timeout[TS_N_VALID_BITS] = TS_VALID_TIMEOUTS_MS;
static const uint8_t k_diag_regs[7] = {0xC8, 0x25, 0x4D, 0xC4, 0xC5, 0xC6, 0xD9};
static const char k_ce[] = "channel Error";
static const char k_po[] = "Power on";

#define EV_RING 32
typedef struct { uint32_t t; uint16_t value; uint8_t code; } ev_t;

static struct {
    uint32_t last_seen[TS_N_VALID_BITS];
    uint16_t ever;              /* valid source seen at least once */
    uint16_t valid_prev;
    /* latest values */
    uint16_t brake1, brake2;
    int16_t torque_cmd;
    uint16_t s_brake1[2], s_brake2[2];   /* [0] = latest 50 ms sample, [1] = 50 ms earlier */
    int16_t s_torque[2];
    int16_t torque_req, speed, dc_bus, motor_temp, igbt_temp, i_actual;
    uint16_t kern_status, pack_current, pack_voltage, pack_ocv, pack_res, soc16;
    uint8_t pack_soc, max_cell, bms_fault, bms_hi, bms_lo;
    uint32_t fault_flags;
    /* status inputs */
    uint8_t red, bspd, bspd_known, r2d, r2d_known, inv_en, inv_known, dash;
    uint8_t on468, on469, can_ep, can_boff;
    uint16_t inv_val;
    uint32_t t468, t469;
    /* THERM / DIAG */
    uint8_t therm[6][8];
    uint32_t therm_t[6];
    uint8_t therm_ever, raw_ever;
    int16_t regs[7];
    uint16_t reg_seen;
    uint8_t raw[3][2];
    uint32_t raw_t[3];
    uint8_t vcu_mt[2];          /* 0x476 B0-B1 as received (VCU-relayed motor temperature) */
    uint8_t vcu_mt_ever;
    uint32_t vcu_mt_t;
    /* frames per 100 ms */
    uint16_t frames_cur, frames_last100;
    /* scheduling */
    uint32_t next_write, next_mid, next_sample, next_100;
    uint32_t write_count;
    uint8_t seq, filler;
    uint8_t have_mid;
    uint32_t mid_uptime;
    ts_fast_t mid;
    /* LoRa receive parser */
    uint8_t rx[TS_CMD_SIZE];
    uint8_t rx_n, m_ce, m_po;
    /* events */
    ev_t ev[EV_RING];
    uint8_t ev_head, ev_n;
    uint16_t ev_dropped_pending;
} S;

/* ------------------------------------------------------------------ events */
void telem_push_event(uint8_t code, uint16_t value, uint32_t t)
{
    if (S.ev_n == EV_RING) {                 /* full: drop the oldest and count it */
        S.ev_head = (uint8_t)((S.ev_head + 1) % EV_RING);
        S.ev_n--;
        if (S.ev_dropped_pending < 0xFFFFu) S.ev_dropped_pending++;
        if (telem_cnt.events_dropped < 0xFFu) telem_cnt.events_dropped++;
    }
    ev_t *e = &S.ev[(S.ev_head + S.ev_n) % EV_RING];
    e->t = t; e->code = code; e->value = value;
    S.ev_n++;
}

static int events_waiting(void) { return S.ev_n || S.ev_dropped_pending; }

static void seen(uint8_t bit, uint32_t t)
{
    S.last_seen[bit] = t;
    S.ever |= (uint16_t)(1u << bit);
}

uint16_t telem_valid_now(uint32_t now)
{
    uint16_t v = 0;
    for (uint8_t i = 0; i < TS_N_VALID_BITS; i++)
        if (((S.ever >> i) & 1u) && within(now, S.last_seen[i], k_timeout[i]))
            v |= (uint16_t)(1u << i);
    return v;
}

uint32_t telem_write_count(void) { return S.write_count; }

/* ------------------------------------------------------------------ init */
void telem_init(uint32_t now)
{
    memset(&S, 0, sizeof S);
    memset(&telem_cnt, 0, sizeof telem_cnt);
    uint32_t t0 = now + 1000u;               /* first write 1 s after boot */
    S.next_write = t0;
    S.next_mid = t0 - 100u;                  /* FAST for packet 1 of the first write */
    S.next_sample = t0 - 150u;               /* samples on the t0 + 50 ms x k grid */
    S.next_100 = t0;
}

/* ------------------------------------------------------------------ CAN decoding */
static void decode_bamo(const uint8_t *d, uint32_t t)      /* 0x181, dlc >= 3 */
{
    int16_t v = (int16_t)le16(d + 1);
    switch (d[0]) {
    case 0x30: S.speed = v; seen(TS_VB_BAMO_SPEED, t); return;
    case 0x49: S.motor_temp = v; seen(TS_VB_BAMO_TEMPS, t); return;
    case 0x4A: S.igbt_temp = v; seen(TS_VB_BAMO_TEMPS, t); return;
    case 0x20: S.i_actual = v; seen(TS_VB_BAMO_CURRENT, t); return;
    case 0x40: S.kern_status = (uint16_t)v; seen(TS_VB_BAMO_STATUS, t); return;
    case 0xEB:
    case 0x66: S.dc_bus = v; seen(TS_VB_BAMO_DCBUS, t); return;
    default:
        for (uint8_t i = 0; i < 7; i++)
            if (d[0] == k_diag_regs[i]) { S.regs[i] = v; S.reg_seen |= (uint16_t)(1u << i); }
        return;
    }
}

static void decode_201(const uint8_t *d, uint8_t n, uint32_t t)   /* GEVCU -> Bamocar */
{
    if (n < 1) return;
    if (d[0] == 0x90 && n >= 3) {
        S.torque_cmd = (int16_t)le16(d + 1);
        seen(TS_VB_TORQUE_CMD, t);
    } else if (d[0] == 0x51 && n >= 3) {
        uint16_t v = le16(d + 1);
        if (!S.inv_known || v != S.inv_val) telem_push_event(TS_EVT_INV_ENABLE_CMD, v, t);
        S.inv_known = 1;
        S.inv_val = v;
        S.inv_en = (v & 0x0004u) == 0;        /* 51 00 00 = enable, 51 04 00 = disable */
    } else if (d[0] == 0x8E) {
        telem_push_event(TS_EVT_BAMO_ERR_CLEAR_SENT, 0, t);
    }
}

void telem_on_can(const can_frame_t *f)
{
    const uint8_t *d = f->data;
    uint8_t n = f->dlc > 8 ? 8 : f->dlc;
    uint32_t t = f->t_ms;
    telem_cnt.can_rx_total++;
    S.frames_cur++;

    if (f->ext) {
        if (f->id >= 0x1839F380u && f->id <= 0x1839F385u) {
            uint8_t k = (uint8_t)(f->id - 0x1839F380u);
            memset(S.therm[k], 0, 8);
            memcpy(S.therm[k], d, n);
            S.therm_t[k] = t;
            S.therm_ever |= (uint8_t)(1u << k);
            if (k == 0) {
                seen(TS_VB_THERM_380, t);
                if (n >= 3) S.max_cell = d[2];
            }
        } else if (f->id == 0x18EEFF80u) {
            seen(TS_VB_THERM_HB, t);
        } else {
            telem_cnt.can_rx_unused++;
        }
        return;
    }

    switch (f->id) {
    case 0x181: if (n >= 3) decode_bamo(d, t); return;
    case 0x201: decode_201(d, n, t); return;
    case 0x301:
        seen(TS_VB_BMS_301, t);
        if (n >= 2) S.pack_current = be16(d);
        if (n >= 4) S.pack_voltage = be16(d + 2);
        if (n >= 5) S.pack_soc = d[4];
        if (n >= 7) S.pack_ocv = be16(d + 5);
        return;
    case 0x302:
        seen(TS_VB_BMS_302, t);
        if (n >= 2) S.soc16 = be16(d);
        if (n >= 4) S.pack_res = be16(d + 2);
        if (n >= 5) S.bms_hi = d[4];
        if (n >= 6) S.bms_lo = d[5];
        return;
    case 0x303:
        seen(TS_VB_BMS_303, t);
        if (n >= 4) S.fault_flags = be32(d);
        return;
    case 0x364: seen(TS_VB_BRAKE1, t); if (n >= 2) S.brake1 = be16(d); return;
    case 0x365: seen(TS_VB_BRAKE2, t); if (n >= 2) S.brake2 = be16(d); return;
    case 0x468:
        S.t468 = t;
        if (!S.on468) { S.on468 = 1; telem_push_event(TS_EVT_BMS_FAULT_MSG_START, 0, t); }
        return;
    case 0x469:
        S.t469 = t;
        if (!S.on469) { S.on469 = 1; telem_push_event(TS_EVT_IMD_FAULT_MSG_START, 0, t); }
        return;
    case 0x470: if (n >= 1) S.red = d[0] != 0; return;
    case 0x471:
        seen(TS_VB_GEVCU_471, t);
        if (n >= 1) {
            if (S.bspd_known && S.bspd == 1 && d[0] == 0) telem_push_event(TS_EVT_BSPD_TRIP, 0, t);
            if (S.bspd_known && S.bspd == 0 && d[0] == 1) telem_push_event(TS_EVT_BSPD_CLEAR, 0, t);
            S.bspd = d[0];
            S.bspd_known = 1;
        }
        return;
    case 0x473: seen(TS_VB_TORQUE_REQ, t); if (n >= 2) S.torque_req = (int16_t)be16(d); return;
    case 0x475:
        if (n >= 1) {
            uint8_t on = d[0] != 0;
            if (S.r2d_known && on != S.r2d) telem_push_event(on ? TS_EVT_R2D_ON : TS_EVT_R2D_OFF, 0, t);
            S.r2d = on;
            S.r2d_known = 1;
        }
        return;
    case 0x588:
        seen(TS_VB_BMS_588, t);
        if (n >= 1) {
            if (d[0] != S.bms_fault) telem_push_event(TS_EVT_BMS_FAULT_CHANGED, d[0], t);
            S.bms_fault = d[0];
        }
        return;
    case 0x476:                         /* motor temperature relayed by the VCU, 2 bytes */
        if (n >= 2) { S.vcu_mt[0] = d[0]; S.vcu_mt[1] = d[1]; }
        S.vcu_mt_t = t;
        S.vcu_mt_ever = 1;
        return;
    case 0x777:
        S.dash = 1;
        telem_push_event(TS_EVT_DASH_BUTTON, n >= 2 ? le16(d) : (n ? d[0] : 0), t);
        return;
    case 0x7E3:
        telem_push_event(TS_EVT_BMS_CLEAR_SENT, n >= 2 ? le16(d) : (n ? d[0] : 0), t);
        return;
    case 0x481: case 0x666: case 0x321: {
        uint8_t k = f->id == 0x481 ? 0 : (f->id == 0x666 ? 1 : 2);
        if (n >= 2) { S.raw[k][0] = d[0]; S.raw[k][1] = d[1]; }
        S.raw_t[k] = t;
        S.raw_ever |= (uint8_t)(1u << k);
        return;
    }
    default:
        telem_cnt.can_rx_unused++;
        return;
    }
}

void telem_note_rx_overflow(void)
{
    if (telem_cnt.can_rx_overflow < 0xFFu) telem_cnt.can_rx_overflow++;
}

void telem_on_can_state(uint8_t tec, uint8_t rec, uint8_t error_passive, uint8_t bus_off, uint32_t now)
{
    telem_cnt.can_tec = tec;
    telem_cnt.can_rec = rec;
    if (bus_off && !S.can_boff) {
        if (telem_cnt.can_busoff_count < 0xFFu) telem_cnt.can_busoff_count++;
        telem_push_event(TS_EVT_CAN_BUS_OFF, 0, now);
    }
    if (error_passive && !S.can_ep) telem_push_event(TS_EVT_CAN_ERROR_PASSIVE, 0, now);
    S.can_boff = bus_off != 0;
    S.can_ep = error_passive != 0;
}

/* ------------------------------------------------------------------ packet builders */
static void build_fast(ts_fast_t *f, uint32_t now)
{
    uint16_t st = 0;
    memset(f, 0, sizeof *f);
    f->valid = telem_valid_now(now);
    if (S.red) st |= TS_STATUS_RED_LIGHT;
    if (S.bspd_known && S.bspd == 1) st |= TS_STATUS_BSPD_OK;
    if (S.on468 && within(now, S.t468, 100)) st |= TS_STATUS_BMS_FAULT_MSG;
    if (S.on469 && within(now, S.t469, 100)) st |= TS_STATUS_IMD_FAULT_MSG;
    if (S.r2d) st |= TS_STATUS_R2D;
    if (S.inv_known && S.inv_en) st |= TS_STATUS_INV_ENABLED;
    if (S.dash) st |= TS_STATUS_DASH_BUTTON;
    S.dash = 0;
#if TELEM_BAMO_DIAG
    if (diag_session_active()) st |= TS_STATUS_DIAG_SESSION;
#endif
    if (S.can_ep || S.can_boff) st |= TS_STATUS_CAN_ERROR;
    f->status = st;
    f->brake1 = S.s_brake1[0];
    f->brake1_prev = S.s_brake1[1];
    f->brake2 = S.s_brake2[0];
    f->brake2_prev = S.s_brake2[1];
    f->torque_cmd = S.s_torque[0];
    f->torque_cmd_prev = S.s_torque[1];
    f->torque_req = S.torque_req;
    f->speed = S.speed;
    f->dc_bus = S.dc_bus;
    f->pack_current = S.pack_current;
    f->pack_voltage = S.pack_voltage;
    f->pack_soc = S.pack_soc;
    f->max_cell_temp = S.max_cell;
    f->bms_fault_num = S.bms_fault;
    f->motor_temp = S.motor_temp;
    f->igbt_temp = S.igbt_temp;
    f->i_actual = S.i_actual;
    f->kern_status = S.kern_status;
    f->bms_fault_flags = S.fault_flags;
    f->pack_ocv = S.pack_ocv;
    f->pack_resistance = S.pack_res;
    f->bms_high_temp = S.bms_hi;
    f->bms_low_temp = S.bms_lo;
    f->soc16 = S.soc16;
    f->can_frames_100ms = S.frames_last100;
#if TELEM_BAMO_DIAG
    uint8_t aseq, ast;
    diag_last_ack(&aseq, &ast);
    f->ack_seq = aseq;
    f->ack_status = ast;
#else
    f->ack_seq = 0;
    f->ack_status = TS_ST_NONE;
#endif
}

static void build_therm(ts_therm_t *t, uint32_t now)
{
    memcpy(t->f380, S.therm, sizeof S.therm);     /* f380..f385 are contiguous (static asserts) */
    uint8_t seen_bits = 0;
    for (uint8_t k = 0; k < 6; k++)
        if (((S.therm_ever >> k) & 1u) && within(now, S.therm_t[k], 2000)) seen_bits |= (uint8_t)(1u << k);
    if (S.vcu_mt_ever && within(now, S.vcu_mt_t, 2000)) seen_bits |= (uint8_t)(1u << 6);
    t->seen = seen_bits;
    t->motor_temp_vcu[0] = S.vcu_mt[0];               /* bytes as received; the PC decodes them */
    t->motor_temp_vcu[1] = S.vcu_mt[1];
}

static void build_diag(ts_diag_t *g, uint32_t now)
{
    g->fw_version = TS_FW_VERSION;
    g->build_flag = TELEM_BAMO_DIAG;
    g->build_opts = (DIAG_ALLOW_WRITES ? TS_OPT_ALLOW_WRITES : 0u) | (DIAG_ANY_ID ? TS_OPT_ANY_ID : 0u);
    g->can_rx_total = telem_cnt.can_rx_total;
    g->can_rx_unused = telem_cnt.can_rx_unused;
    g->can_tec = telem_cnt.can_tec;
    g->can_rec = telem_cnt.can_rec;
    g->can_busoff_count = telem_cnt.can_busoff_count;
    g->can_rx_overflow = telem_cnt.can_rx_overflow;
    g->lora_channel_errors = telem_cnt.lora_channel_errors;
    g->lora_power_on = telem_cnt.lora_power_on;
    g->cmd_bad = telem_cnt.cmd_bad;
#if TELEM_BAMO_DIAG
    g->cmd_executed = diag_cmd_executed();
    g->capture_dropped = diag_capture_dropped();
#endif
    g->nmax = S.regs[0];
    g->ramp = S.regs[1];
    g->i_max = S.regs[2];
    g->i_max_pk = S.regs[3];
    g->i_con_eff = S.regs[4];
    g->i_device = S.regs[5];
    g->i_200pc = S.regs[6];
    g->reg_seen = S.reg_seen;
    g->raw_481 = (uint16_t)(S.raw[0][0] | (S.raw[0][1] << 8));
    g->raw_666 = (uint16_t)(S.raw[1][0] | (S.raw[1][1] << 8));
    g->raw_321 = (uint16_t)(S.raw[2][0] | (S.raw[2][1] << 8));
    uint8_t rs = 0;
    for (uint8_t k = 0; k < 3; k++)
        if (((S.raw_ever >> k) & 1u) && within(now, S.raw_t[k], 2000)) rs |= (uint8_t)(1u << k);
    g->raw_seen = rs;
    g->events_dropped = telem_cnt.events_dropped;
}

static uint16_t age_ms(uint32_t uptime, uint32_t t)
{
    int32_t a = (int32_t)(uptime - t);
    if (a < 0) return 0;
    return a > 0xFFFF ? 0xFFFFu : (uint16_t)a;
}

static void build_event(ts_event_t *e, uint32_t uptime)
{
    uint8_t c = 0;
    if (S.ev_dropped_pending) {
        e->records[c].age_ms = 0;
        e->records[c].code = TS_EVT_EVENTS_DROPPED;
        e->records[c].value = S.ev_dropped_pending;
        S.ev_dropped_pending = 0;
        c++;
    }
    while (c < 10 && S.ev_n) {
        ev_t *v = &S.ev[S.ev_head];
        e->records[c].age_ms = age_ms(uptime, v->t);
        e->records[c].code = v->code;
        e->records[c].value = v->value;
        S.ev_head = (uint8_t)((S.ev_head + 1) % EV_RING);
        S.ev_n--;
        c++;
    }
    e->count = c;
}

/* Header + CRC for a packet whose payload is already at pkt + 8. */
static void finalize(uint8_t *pkt, uint8_t type, uint32_t uptime)
{
    pkt[0] = TS_SYNC0;
    pkt[1] = TS_SYNC1;
    pkt[2] = (uint8_t)((TS_PKT_VERSION << 4) | type);
    pkt[3] = S.seq++;
    put32(pkt + 4, uptime);
    uint16_t crc = ts_crc16(pkt, TS_CRC_OFF);
    pkt[TS_CRC_OFF] = (uint8_t)crc;
    pkt[TS_CRC_OFF + 1] = (uint8_t)(crc >> 8);
}

/* One packet of the given type at pkt (64 B, cleared first). */
static void emit(uint8_t *pkt, uint8_t type, uint32_t now)
{
    ts_packet_t *p = (ts_packet_t *)pkt;   /* packed: alignment 1 */
    memset(pkt, 0, TS_PKT_SIZE);
    switch (type) {
    case TS_TYPE_FAST: build_fast(&p->p.fast, now); break;
    case TS_TYPE_THERM: build_therm(&p->p.therm, now); break;
    case TS_TYPE_EVENT: build_event(&p->p.event, now); break;
#if TELEM_BAMO_DIAG
    case TS_TYPE_DIAG_RESP: diag_build_resp(&p->p.diag_resp, now); break;
#endif
    default: type = TS_TYPE_DIAG; build_diag(&p->p.diag, now); break;
    }
    finalize(pkt, type, now);
}

#if TELEM_BAMO_DIAG == 2
/* Diagnostics-first fill order: DIAG_RESP, EVENT, then THERM/DIAG alternating as filler. */
static uint8_t fill_type(void)
{
    if (diag_resp_pending()) return TS_TYPE_DIAG_RESP;
    if (events_waiting()) return TS_TYPE_EVENT;
    S.filler ^= 1u;
    return S.filler ? TS_TYPE_THERM : TS_TYPE_DIAG;
}
#else
static uint8_t slot3_type(uint8_t n)
{
    if (n == 0) return TS_TYPE_THERM;
    if (n == 1) return TS_TYPE_DIAG;
    if (events_waiting()) return TS_TYPE_EVENT;
#if TELEM_BAMO_DIAG
    if (diag_resp_pending()) return TS_TYPE_DIAG_RESP;
#endif
    return TS_TYPE_DIAG;
}
#endif

/* ------------------------------------------------------------------ scheduling */
static void take_sample(void)
{
    S.s_brake1[1] = S.s_brake1[0]; S.s_brake1[0] = S.brake1;
    S.s_brake2[1] = S.s_brake2[0]; S.s_brake2[0] = S.brake2;
    S.s_torque[1] = S.s_torque[0]; S.s_torque[0] = S.torque_cmd;
}

static void check_valid_edges(uint32_t now)
{
    uint16_t v = telem_valid_now(now);
    uint16_t ch = v ^ S.valid_prev;
    for (uint8_t i = 0; i < TS_N_VALID_BITS; i++)
        if ((ch >> i) & 1u)
            telem_push_event(((v >> i) & 1u) ? TS_EVT_SOURCE_BACK : TS_EVT_SOURCE_TIMEOUT, i, now);
    S.valid_prev = v;
}

void telem_poll(uint32_t now)
{
    while (reached(now, S.next_sample)) {
        take_sample();
        S.next_sample += 50u;
        check_valid_edges(now);
    }
    while (reached(now, S.next_100)) {
        S.frames_last100 = S.frames_cur;
        S.frames_cur = 0;
        S.next_100 += 100u;
    }
    if (S.on468 && !within(now, S.t468, 100)) { S.on468 = 0; telem_push_event(TS_EVT_BMS_FAULT_MSG_STOP, 0, now); }
    if (S.on469 && !within(now, S.t469, 100)) { S.on469 = 0; telem_push_event(TS_EVT_IMD_FAULT_MSG_STOP, 0, now); }
#if TELEM_BAMO_DIAG != 2
    if (reached(now, S.next_mid)) {            /* FAST that becomes packet 1 of the next write */
        build_fast(&S.mid, now);
        S.mid_uptime = now;
        S.have_mid = 1;
        do S.next_mid += TS_WRITE_PERIOD_MS; while (reached(now, S.next_mid));
    }
#endif
}

int telem_write_due(uint32_t now) { return reached(now, S.next_write); }

int telem_tx_allowed(uint32_t now, uint32_t last_write_ms, int have_last, int dma_busy, uint16_t len)
{
    if (dma_busy) return 0;
    if (len > TS_MAX_WRITE_SIZE) return 0;
    if (have_last && (int32_t)(now - last_write_ms) < TS_MIN_WRITE_GAP_MS) return 0;
    return 1;
}

static void advance_deadline(uint32_t now)
{
    S.next_write += TS_WRITE_PERIOD_MS;
    while (reached(now, S.next_write)) {       /* missed whole periods: skip them */
        S.next_write += TS_WRITE_PERIOD_MS;
        telem_cnt.writes_skipped++;
    }
}

void telem_write_skipped(uint32_t now)
{
    telem_cnt.writes_skipped++;
    S.have_mid = 0;
    advance_deadline(now);
}

uint16_t telem_build_write(uint32_t now, uint8_t *buf)
{
    uint8_t *p = buf + 3;
    uint8_t n = (uint8_t)(S.write_count % 5u);
    buf[0] = TS_HDR_TO_PC_0;
    buf[1] = TS_HDR_TO_PC_1;
    buf[2] = TS_HDR_TO_PC_2;
#if TELEM_BAMO_DIAG == 2
    uint8_t slot = 0;
    if (n == 0) { emit(p, TS_TYPE_FAST, now); slot = 1; }
    for (; slot < TS_PACKETS_PER_WRITE; slot++) emit(p + slot * TS_PKT_SIZE, fill_type(), now);
#else
    /* packet 1: the FAST built 100 ms earlier (rebuilt now only if it is missing) */
    if (S.have_mid) {
        memset(p, 0, TS_PKT_SIZE);
        memcpy(p + TS_PAYLOAD_OFF, &S.mid, sizeof S.mid);
        finalize(p, TS_TYPE_FAST, S.mid_uptime);
    } else {
        emit(p, TS_TYPE_FAST, now);
    }
    emit(p + TS_PKT_SIZE, TS_TYPE_FAST, now);            /* packet 2: FAST built now */
    emit(p + 2 * TS_PKT_SIZE, slot3_type(n), now);       /* packet 3: by write index */
#endif
    S.have_mid = 0;
    S.write_count++;
    advance_deadline(now);
    return TS_WRITE_SIZE;
}

/* ------------------------------------------------------------------ LoRa receive */
static uint8_t match(uint8_t *st, const char *pat, uint8_t len, uint8_t b)
{
    if (b == (uint8_t)pat[*st]) {
        if (++*st == len) { *st = 0; return 1; }
    } else {
        *st = (b == (uint8_t)pat[0]) ? 1 : 0;
    }
    return 0;
}

static int cmd_ok(const uint8_t *c)
{
    return c[2] == TS_CMD_VER_TYPE
        && c[4] == (uint8_t)(CMD_KEY >> 8) && c[5] == (uint8_t)(CMD_KEY & 0xFFu)
        && ts_crc16(c, TS_CMD_CRC_OFF) == le16(c + TS_CMD_CRC_OFF);
}

int telem_lora_rx_byte(uint8_t b, uint32_t now, uint8_t cmd[TS_CMD_SIZE])
{
    if (match(&S.m_ce, k_ce, sizeof k_ce - 1, b) && telem_cnt.lora_channel_errors < 0xFFFFu)
        telem_cnt.lora_channel_errors++;
    if (match(&S.m_po, k_po, sizeof k_po - 1, b)) {
        if (telem_cnt.lora_power_on < 0xFFFFu) telem_cnt.lora_power_on++;
        telem_push_event(TS_EVT_LORA_POWER_ON, 0, now);
    }
    if (S.rx_n == 0) {
        if (b == TS_SYNC0) S.rx[S.rx_n++] = b;
        return 0;
    }
    if (S.rx_n == 1) {
        if (b == TS_SYNC1) S.rx[S.rx_n++] = b;
        else S.rx_n = (b == TS_SYNC0) ? 1 : 0;
        return 0;
    }
    S.rx[S.rx_n++] = b;
    if (S.rx_n < TS_CMD_SIZE) return 0;
    if (cmd_ok(S.rx)) {
        memcpy(cmd, S.rx, TS_CMD_SIZE);
        S.rx_n = 0;
        return 1;
    }
    if (telem_cnt.cmd_bad < 0xFFFFu) telem_cnt.cmd_bad++;
    /* move ahead: restart at the next A5 5A (or a trailing A5) inside the candidate */
    uint8_t i = 1;
    while (i < TS_CMD_SIZE && !(S.rx[i] == TS_SYNC0 && (i == TS_CMD_SIZE - 1 || S.rx[i + 1] == TS_SYNC1))) i++;
    S.rx_n = (uint8_t)(TS_CMD_SIZE - i);
    memmove(S.rx, S.rx + i, S.rx_n);
    return 0;
}

void telem_lora_feed(uint8_t b, uint32_t now)
{
    uint8_t cmd[TS_CMD_SIZE];
    if (!telem_lora_rx_byte(b, now, cmd)) return;
#if TELEM_BAMO_DIAG
    diag_on_command(cmd, now);
#endif
    /* flag 0: commands are parsed (and bad ones counted) but ignored */
}
