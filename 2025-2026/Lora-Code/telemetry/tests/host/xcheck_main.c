/* Cross-check writer (BUILD_INSTRUCTIONS.md 8.3).
 *   xcheck <packets.bin> <scenario.bin>
 * packets.bin : one 64 B packet per payload type, every field filled by xcheck_fill.c (generated
 *               from the YAML), then each command code's 24 B packet with filled arguments.
 * scenario.bin: the 3 packets of the 1200 ms write after feeding telem_core real CAN frames
 *               from section 3 of the brief (decoded and checked by tests/test_xcheck.py). */
#include <stdio.h>
#include <string.h>
#include "telem_core.h"

void xcheck_fill_fast(ts_fast_t *x);
void xcheck_fill_therm(ts_therm_t *x);
void xcheck_fill_diag(ts_diag_t *x);
void xcheck_fill_event(ts_event_t *x);
void xcheck_fill_diag_resp(ts_diag_resp_t *x);
void xcheck_fill_args_bamo_read(ts_args_bamo_read_t *x);
void xcheck_fill_args_bamo_write(ts_args_bamo_write_t *x);
void xcheck_fill_args_send_frame(ts_args_send_frame_t *x);
void xcheck_fill_args_capture_set(ts_args_capture_set_t *x);
void xcheck_fill_args_capture_clear(ts_args_capture_clear_t *x);

static void put_packet(FILE *f, ts_packet_t *p, uint8_t type)
{
    p->hdr.sync[0] = TS_SYNC0;
    p->hdr.sync[1] = TS_SYNC1;
    p->hdr.ver_type = (uint8_t)((TS_PKT_VERSION << 4) | type);
    p->hdr.seq = type;
    p->hdr.uptime_ms = 0x01020304u * type;
    p->crc = ts_crc16((const uint8_t *)p, TS_CRC_OFF);
    fwrite(p, TS_PKT_SIZE, 1, f);
}

static void put_cmd(FILE *f, ts_cmd_t *c, uint8_t code)
{
    c->sync[0] = TS_SYNC0;
    c->sync[1] = TS_SYNC1;
    c->ver_type = TS_CMD_VER_TYPE;
    c->seq = code;
    c->key[0] = TS_CMD_KEY0;
    c->key[1] = TS_CMD_KEY1;
    c->code = code;
    c->crc = ts_crc16((const uint8_t *)c, TS_CMD_CRC_OFF);
    fwrite(c, TS_CMD_SIZE, 1, f);
}

static uint32_t T;
static void can(uint32_t id, uint8_t ext, uint8_t dlc, const uint8_t *d)
{
    can_frame_t fr;
    memset(&fr, 0, sizeof fr);
    fr.id = id; fr.ext = ext; fr.dlc = dlc; fr.t_ms = T;
    memcpy(fr.data, d, dlc);
    telem_on_can(&fr);
}
#define CAN(id, ...)  do { const uint8_t _d[] = {__VA_ARGS__}; can(id, 0, sizeof _d, _d); } while (0)
#define CANX(id, ...) do { const uint8_t _d[] = {__VA_ARGS__}; can(id, 1, sizeof _d, _d); } while (0)

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: xcheck <packets.bin> <scenario.bin>\n"); return 2; }
    FILE *f = fopen(argv[1], "wb");
    if (!f) return 2;
    ts_packet_t p;
    memset(&p, 0, sizeof p); xcheck_fill_fast(&p.p.fast); put_packet(f, &p, TS_TYPE_FAST);
    memset(&p, 0, sizeof p); xcheck_fill_therm(&p.p.therm); put_packet(f, &p, TS_TYPE_THERM);
    memset(&p, 0, sizeof p); xcheck_fill_diag(&p.p.diag); put_packet(f, &p, TS_TYPE_DIAG);
    memset(&p, 0, sizeof p); xcheck_fill_event(&p.p.event); put_packet(f, &p, TS_TYPE_EVENT);
    memset(&p, 0, sizeof p); xcheck_fill_diag_resp(&p.p.diag_resp); put_packet(f, &p, TS_TYPE_DIAG_RESP);
    ts_cmd_t c;
    memset(&c, 0, sizeof c); put_cmd(f, &c, TS_CMD_PING);
    memset(&c, 0, sizeof c); xcheck_fill_args_bamo_read((ts_args_bamo_read_t *)c.args); put_cmd(f, &c, TS_CMD_BAMO_READ);
    memset(&c, 0, sizeof c); xcheck_fill_args_bamo_write((ts_args_bamo_write_t *)c.args); put_cmd(f, &c, TS_CMD_BAMO_WRITE);
    memset(&c, 0, sizeof c); xcheck_fill_args_send_frame((ts_args_send_frame_t *)c.args); put_cmd(f, &c, TS_CMD_SEND_FRAME);
    memset(&c, 0, sizeof c); xcheck_fill_args_capture_set((ts_args_capture_set_t *)c.args); put_cmd(f, &c, TS_CMD_CAPTURE_SET);
    memset(&c, 0, sizeof c); xcheck_fill_args_capture_clear((ts_args_capture_clear_t *)c.args); put_cmd(f, &c, TS_CMD_CAPTURE_CLEAR);
    memset(&c, 0, sizeof c); put_cmd(f, &c, TS_CMD_STOP_ALL);
    fclose(f);

    /* scenario: real frames through telem_core */
    uint8_t w[TS_WRITE_SIZE];
    T = 0;
    telem_init(T);
    for (T = 1; T <= 1200; T++) {
        if (T == 1100) {
            CAN(0x364, 0x12, 0x34);
            CAN(0x201, 0x90, 0xFF, 0x7F);                 /* torque command 100% */
            CAN(0x473, 0xFF, 0x38);                       /* -20.0 Nm */
            CAN(0x181, 0x30, 0xFF, 0x7F, 0x00);           /* speed raw 32767 */
            CAN(0x181, 0x49, 0x99, 0x24, 0x00);           /* motor temp raw 9369 = 0 degC */
            CAN(0x181, 0xC8, 0x70, 0x17, 0x00);           /* Nmax 6000 */
            CAN(0x301, 0x01, 0x02, 0x03, 0x04, 0x55, 0x06, 0x07, 0x00);
            CAN(0x303, 0xDE, 0xAD, 0xBE, 0xEF);
            CANX(0x1839F380, 0x11, 0x22, 0x2D, 0x44, 0x55, 0x66, 0x77, 0x88);
            CAN(0x470, 0x01);
            CAN(0x471, 0x01);
        }
        telem_poll(T);
        if (telem_write_due(T)) telem_build_write(T, w);
    }
    f = fopen(argv[2], "wb");
    if (!f) return 2;
    fwrite(w + 3, TS_PKT_SIZE, 3, f);                     /* the write at 1200 ms */
    fclose(f);
    return 0;
}
