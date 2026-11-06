/* Diagnostics logic (BUILD_INSTRUCTIONS.md 3.7, 3.8, 5.6). No HAL includes.
 * Check order for a command: arguments -> build allows it -> CAN ID -> blocked register ->
 * inverter live -> repeating-read limit -> rate limit -> CAN send. */
#include "telem_config.h"
#if TELEM_BAMO_DIAG
#include <string.h>
#include "diag_core.h"

#define BAMO_RX_ID 0x201u      /* frames to the Bamocar */
#define REG_READ   0x3Du

static const uint8_t k_write_blocked[TS_N_WRITE_BLOCKED] = TS_WRITE_BLOCKED_REGS;
static const uint8_t k_read_blocked[TS_N_READ_BLOCKED] = TS_READ_BLOCKED_REGS;

typedef struct { uint8_t used, ext, rate; uint32_t id, mask, win_t; uint8_t win_n; } cap_slot_t;
typedef struct { uint32_t t, id; uint8_t dlc; uint8_t data[8]; } cap_frame_t;

static struct {
    diag_can_send_fn send;
    /* session */
    uint8_t active;
    uint32_t last_cmd_t;
    uint8_t have_last, last_seq, last_status;
    uint16_t executed;
    /* acks waiting for a DIAG_RESP */
    uint8_t ack_seq[3], ack_st[3], ack_n;
    /* repeating reads started by diagnostics */
    uint8_t rd_reg[DIAG_MAX_REPEAT_READS], rd_n;
    /* rate limit: token bucket, 20 tokens, one back every 50 ms */
    uint8_t tokens;
    uint32_t tok_t;
    /* live-inverter inputs */
    uint8_t inv_known, inv_en, have_r2d, have_473, have_kern, kern_bit0;
    uint32_t t_r2d, t_473, t_kern;
    /* captures */
    cap_slot_t slot[DIAG_N_CAPTURE_SLOTS];
    cap_frame_t ring[DIAG_CAPTURE_RING];
    uint8_t ring_head, ring_n, drop_since;
    uint16_t drop_total;
} D;

static int within(uint32_t now, uint32_t t, uint32_t win) { return (int32_t)(now - t) <= (int32_t)win; }
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int in_list(uint8_t r, const uint8_t *l, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) if (l[i] == r) return 1;
    return 0;
}

void diag_init(diag_can_send_fn send)
{
    memset(&D, 0, sizeof D);
    D.send = send;
    D.tokens = DIAG_RATE_PER_S;
}

/* ------------------------------------------------------------------ inputs */
int diag_inverter_live(uint32_t now)
{
    if (D.inv_known && D.inv_en) return 1;                            /* last 0x201/0x51 enabled */
    if (D.have_r2d && within(now, D.t_r2d, 10000)) return 1;          /* 0x475 B0 = 1 in 10 s */
    if (D.have_473 && within(now, D.t_473, 2000)) return 1;           /* 0x473 in 2 s */
    if (D.have_kern && D.kern_bit0 && within(now, D.t_kern, 2000)) return 1;  /* KERN bit 0 */
    return 0;
}

static void capture(const can_frame_t *f)
{
    for (uint8_t s = 0; s < DIAG_N_CAPTURE_SLOTS; s++) {
        cap_slot_t *c = &D.slot[s];
        if (!c->used || c->ext != f->ext || ((f->id ^ c->id) & c->mask)) continue;
        if (!within(f->t_ms, c->win_t, 999)) { c->win_t = f->t_ms; c->win_n = 0; }
        if (c->win_n >= c->rate || D.ring_n == DIAG_CAPTURE_RING) {   /* over the slot rate, or ring full */
            if (D.drop_since < 0xFFu) D.drop_since++;
            if (D.drop_total < 0xFFFFu) D.drop_total++;
            return;
        }
        c->win_n++;
        cap_frame_t *r = &D.ring[(D.ring_head + D.ring_n) % DIAG_CAPTURE_RING];
        r->t = f->t_ms;
        r->id = f->id | (f->ext ? CAN_EXT_FLAG : 0u);
        r->dlc = f->dlc > 8 ? 8 : f->dlc;
        memcpy(r->data, f->data, 8);
        D.ring_n++;
        return;                                  /* first matching slot only */
    }
}

void diag_on_can(const can_frame_t *f)
{
    const uint8_t *d = f->data;
    if (!f->ext) {
        if (f->id == BAMO_RX_ID && f->dlc >= 3 && d[0] == 0x51) {
            D.inv_known = 1;
            D.inv_en = (d[1] & 0x04u) == 0;      /* value bit 2 clear = enabled */
        } else if (f->id == 0x475 && f->dlc >= 1 && d[0] == 1) {
            D.have_r2d = 1; D.t_r2d = f->t_ms;
        } else if (f->id == 0x473) {
            D.have_473 = 1; D.t_473 = f->t_ms;
        } else if (f->id == 0x181 && f->dlc >= 3 && d[0] == 0x40) {
            D.have_kern = 1; D.t_kern = f->t_ms; D.kern_bit0 = d[1] & 1u;
        }
    }
    capture(f);
}

/* ------------------------------------------------------------------ sending */
static void refill(uint32_t now)
{
    uint32_t n = (uint32_t)(now - D.tok_t) / (1000u / DIAG_RATE_PER_S);
    if (!n) return;
    D.tok_t += n * (1000u / DIAG_RATE_PER_S);
    D.tokens = (uint8_t)((D.tokens + n) > DIAG_RATE_PER_S ? DIAG_RATE_PER_S : D.tokens + n);
}

static uint8_t send_frame(uint32_t id, uint8_t ext, uint8_t dlc, const uint8_t *data, uint32_t now, int limited)
{
    can_frame_t f;
    memset(&f, 0, sizeof f);
    f.id = id; f.ext = ext; f.dlc = dlc; f.t_ms = now;
    memcpy(f.data, data, dlc);
    if (limited) {
        refill(now);
        if (!D.tokens) return TS_ST_REJ_RATE;
        D.tokens--;
    }
    return D.send && D.send(&f) == 0 ? TS_ST_OK : TS_ST_TX_FAIL;
}

static int rd_find(uint8_t reg)
{
    for (uint8_t i = 0; i < D.rd_n; i++) if (D.rd_reg[i] == reg) return i;
    return -1;
}

static void rd_remove(uint8_t reg)
{
    int i = rd_find(reg);
    if (i < 0) return;
    D.rd_reg[i] = D.rd_reg[--D.rd_n];
}

/* Stops every repeating read diagnostics started and clears the captures (not rate limited). */
static void stop_everything(uint32_t now)
{
    for (uint8_t i = 0; i < D.rd_n; i++) {
        uint8_t d[3] = {REG_READ, D.rd_reg[i], 0xFF};
        send_frame(BAMO_RX_ID, 0, 3, d, now, 0);
    }
    D.rd_n = 0;
    memset(D.slot, 0, sizeof D.slot);
    D.ring_n = 0;
}

static void end_session(uint32_t now)
{
    stop_everything(now);
    D.active = 0;
}

/* ------------------------------------------------------------------ commands */
static uint8_t do_read(uint8_t reg, uint8_t period, uint32_t now)
{
    if (in_list(reg, k_read_blocked, TS_N_READ_BLOCKED)) return TS_ST_REJ_REGID;
    if (period >= 1 && period < DIAG_MIN_PERIOD_MS) return TS_ST_BAD_ARGS;
    if (period != 0 && diag_inverter_live(now)) return TS_ST_REJ_INV_ENABLED;
    int idx = rd_find(reg);
    if (period != 0 && period != 0xFF && idx < 0 && D.rd_n >= DIAG_MAX_REPEAT_READS) return TS_ST_REJ_LIMIT;
    uint8_t d[3] = {REG_READ, reg, period};
    uint8_t st = send_frame(BAMO_RX_ID, 0, 3, d, now, 1);
    if (st != TS_ST_OK) return st;
    if (period == 0xFF) rd_remove(reg);
    else if (period != 0 && idx < 0) D.rd_reg[D.rd_n++] = reg;
    return TS_ST_OK;
}

static uint8_t do_write(const uint8_t *a, uint32_t now)
{
    uint8_t reg = a[0], size = a[1];
    if (size != 2 && size != 4) return TS_ST_BAD_ARGS;
    if (reg == REG_READ) return TS_ST_BAD_ARGS;           /* a read request: use BAMO_READ */
    if (!DIAG_ALLOW_WRITES) return TS_ST_REJ_DISABLED;
    if (in_list(reg, k_write_blocked, TS_N_WRITE_BLOCKED)) return TS_ST_REJ_REGID;
    if (diag_inverter_live(now)) return TS_ST_REJ_INV_ENABLED;
    uint8_t d[5] = {reg, a[2], a[3], a[4], a[5]};
    return send_frame(BAMO_RX_ID, 0, (uint8_t)(1 + size), d, now, 1);
}

static uint8_t do_send(const uint8_t *a, uint32_t now)
{
    uint32_t raw = get32(a);
    uint8_t ext = (raw & CAN_EXT_FLAG) != 0;
    uint32_t id = raw & ~CAN_EXT_FLAG;
    uint8_t dlc = a[4];
    const uint8_t *data = a + 5;
    if (dlc > 8 || (ext ? id > 0x1FFFFFFFu : id > 0x7FFu)) return TS_ST_BAD_ARGS;
    int to_bamo = !ext && id == BAMO_RX_ID;
    if (!to_bamo && !DIAG_ANY_ID) return TS_ST_REJ_ID;
    if (to_bamo) {
        if (dlc == 0) return TS_ST_BAD_ARGS;
        if (data[0] == REG_READ) {                         /* a read: BAMO_READ rules */
            if (dlc < 3) return TS_ST_BAD_ARGS;
            return do_read(data[1], data[2], now);
        }
        if (!DIAG_ALLOW_WRITES) return TS_ST_REJ_DISABLED;
        if (in_list(data[0], k_write_blocked, TS_N_WRITE_BLOCKED)) return TS_ST_REJ_REGID;
    } else if (!DIAG_ALLOW_WRITES) {
        return TS_ST_REJ_DISABLED;
    }
    if (diag_inverter_live(now)) return TS_ST_REJ_INV_ENABLED;
    return send_frame(id, ext, dlc, data, now, 1);
}

static uint8_t do_capture_set(const uint8_t *a)
{
    uint8_t s = a[0], rate = a[9];
    if (s >= DIAG_N_CAPTURE_SLOTS || rate == 0) return TS_ST_BAD_ARGS;
    uint32_t id = get32(a + 1), mask = get32(a + 5);
    cap_slot_t *c = &D.slot[s];
    c->used = 1;
    c->ext = (id & CAN_EXT_FLAG) != 0;
    c->id = id & ~CAN_EXT_FLAG;
    c->mask = mask & ~CAN_EXT_FLAG;
    c->rate = rate;
    c->win_n = 0;
    return TS_ST_OK;
}

static uint8_t do_capture_clear(uint8_t s)
{
    if (s == 0xFF) { memset(D.slot, 0, sizeof D.slot); return TS_ST_OK; }
    if (s >= DIAG_N_CAPTURE_SLOTS) return TS_ST_BAD_ARGS;
    memset(&D.slot[s], 0, sizeof D.slot[s]);
    return TS_ST_OK;
}

/* While the inverter may be live, only PING, CAPTURE_SET/CLEAR, STOP_ALL and one-shot
 * BAMO_READ get through: do_read/do_write/do_send return REJ_INV_ENABLED for the rest. */
static uint8_t execute(uint8_t code, const uint8_t *a, uint32_t now)
{
    switch (code) {
    case TS_CMD_PING: return TS_ST_OK;
    case TS_CMD_BAMO_READ: return do_read(a[0], a[1], now);
    case TS_CMD_BAMO_WRITE: return do_write(a, now);
    case TS_CMD_SEND_FRAME: return do_send(a, now);
    case TS_CMD_CAPTURE_SET: return do_capture_set(a);
    case TS_CMD_CAPTURE_CLEAR: return do_capture_clear(a[0]);
    case TS_CMD_STOP_ALL: stop_everything(now); return TS_ST_OK;
    default: return TS_ST_BAD_ARGS;
    }
}

static void queue_ack(uint8_t seq, uint8_t st)
{
    if (D.ack_n == 3) {                          /* keep the newest three */
        D.ack_seq[0] = D.ack_seq[1]; D.ack_st[0] = D.ack_st[1];
        D.ack_seq[1] = D.ack_seq[2]; D.ack_st[1] = D.ack_st[2];
        D.ack_n = 2;
    }
    D.ack_seq[D.ack_n] = seq;
    D.ack_st[D.ack_n] = st;
    D.ack_n++;
}

void diag_on_command(const uint8_t cmd[TS_CMD_SIZE], uint32_t now)
{
    const ts_cmd_t *c = (const ts_cmd_t *)cmd;
    uint8_t seq = c->seq;
    D.active = 1;                                /* any valid command starts / keeps a session */
    D.last_cmd_t = now;
    if (D.have_last && seq == D.last_seq) {      /* a repeat: acknowledge again, don't run it */
        queue_ack(seq, D.last_status);
        return;
    }
    uint8_t st = execute(c->code, c->args, now);
    D.have_last = 1;
    D.last_seq = seq;
    D.last_status = st;
    if (D.executed < 0xFFFFu) D.executed++;
    queue_ack(seq, st);
    if (c->code == TS_CMD_STOP_ALL) D.active = 0;  /* STOP_ALL ends the session */
}

void diag_poll(uint32_t now)
{
    if (D.active && !within(now, D.last_cmd_t, DIAG_SESSION_TIMEOUT_MS)) end_session(now);
}

/* ------------------------------------------------------------------ outputs */
uint8_t diag_session_active(void) { return D.active; }
uint16_t diag_cmd_executed(void) { return D.executed; }
uint16_t diag_capture_dropped(void) { return D.drop_total; }
uint8_t diag_n_repeat_reads(void) { return D.rd_n; }

void diag_last_ack(uint8_t *seq, uint8_t *status)
{
    *seq = D.have_last ? D.last_seq : 0;
    *status = D.have_last ? D.last_status : TS_ST_NONE;
}

int diag_resp_pending(void) { return D.ack_n || D.ring_n || D.drop_since; }

void diag_build_resp(ts_diag_resp_t *r, uint32_t now)
{
    r->n_acks = D.ack_n;
    for (uint8_t i = 0; i < D.ack_n; i++) { r->acks[i].seq = D.ack_seq[i]; r->acks[i].status = D.ack_st[i]; }
    D.ack_n = 0;
    uint8_t n = 0;
    while (n < 3 && D.ring_n) {
        cap_frame_t *f = &D.ring[D.ring_head];
        int32_t age = (int32_t)(now - f->t);
        r->frames[n].age_ms = age < 0 ? 0 : (age > 0xFFFF ? 0xFFFFu : (uint16_t)age);
        r->frames[n].id = f->id;
        r->frames[n].dlc = f->dlc;
        memcpy(r->frames[n].data, f->data, 8);
        D.ring_head = (uint8_t)((D.ring_head + 1) % DIAG_CAPTURE_RING);
        D.ring_n--;
        n++;
    }
    r->n_frames = n;
    r->cap_dropped = D.drop_since;
    D.drop_since = 0;
}

#endif /* TELEM_BAMO_DIAG */
