/* Host-side unit tests for App/app/dock.c — the UV-K5 V3 dock protocol core.
 *
 * The "definition of done" for the F2 port is byte-compatibility with
 * radio-server's FirmwareFakeSerial + Uvk5Decoder
 * (radio_server/backends/uvk5/, tests/test_uvk5_transport.py). These cases
 * mirror that fake's acceptance / dispatch / reply rules and pin one
 * byte-exact reply vector as an independent oracle.
 *
 * Build & run:  make -C tests/host run     (or: cc -I App tests/host/test_dock.c App/app/dock.c)
 */

#include "app/dock.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- tiny test framework ---- */
static int g_checks = 0, g_fail = 0;
#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            printf("  FAIL: %s (line %d)\n", (msg), __LINE__);              \
            g_fail++;                                                       \
        }                                                                   \
    } while (0)

/* ---- fake HAL ---- */
static uint16_t g_regs[256];
static uint8_t  g_cap[2048];
static uint16_t g_caplen;
static int      g_reads, g_writes;

/* TX-state spy (F5). g_ev logs the *order* of events: 'w' on a REG_30 write,
 * '1'/'0' on a tx_set(on/off) callback — so "1w" proves the PA edge is driven
 * BEFORE the register write completes. */
static int      g_tx_calls, g_tx_last;   /* g_tx_last: 1=on, 0=off, -1=none yet */
static char     g_ev[128];
static int      g_evn;

static void ev_push(char c)
{
    if (g_evn < (int)sizeof(g_ev) - 1) g_ev[g_evn++] = c;
}
static uint16_t hal_read(void *u, uint16_t reg)
{
    (void)u; g_reads++;
    return g_regs[reg & 0xFF];
}
static void hal_write(void *u, uint16_t reg, uint16_t val)
{
    (void)u; g_writes++;
    if ((reg & 0xFF) == 0x30) ev_push('w');
    g_regs[reg & 0xFF] = val;
}
static void hal_send(void *u, const uint8_t *buf, uint16_t len)
{
    (void)u;
    if ((uint32_t)g_caplen + len <= sizeof(g_cap)) {
        memcpy(g_cap + g_caplen, buf, len);
        g_caplen = (uint16_t)(g_caplen + len);
    }
}
static void hal_tx(void *u, bool on)
{
    (void)u; g_tx_calls++; g_tx_last = on ? 1 : 0;
    ev_push(on ? '1' : '0');
}
/* set-VFO spy (0x0873). Records the last decoded channel and how many times the
 * firmware would have been asked to apply one — the count is what proves a
 * malformed or out-of-range frame was REFUSED rather than quietly passed on to
 * the radio's own VFO struct.
 *
 * g_vfo_force_status makes the fake binding refuse the way the real one does
 * for the two conditions only it can see (band, tone). It then ALSO fills in
 * frequencies, deliberately, so the tests can prove dock.c blanks them: a
 * non-zero status must never ship a channel the radio is not on. */
static int        g_vfo_calls;
static dock_vfo_t g_vfo_last;
static uint8_t    g_vfo_force_status;

static void hal_set_vfo(void *u, const dock_vfo_t *vfo, dock_vfo_applied_t *out)
{
    (void)u; g_vfo_calls++; g_vfo_last = *vfo;
    if (g_vfo_force_status != DOCK_VFO_APPLIED) {
        out->status       = g_vfo_force_status;
        out->rx_hz        = 0xDEADBEEFu;
        out->tx_hz        = 0xFEEDFACEu;
        out->ctcss_tenths = 0x1234u;
        out->power        = 0x7Fu;
        return;
    }
    /* Mirrors uart.c's DOCK_POWER_MAP. The wire's 0/1/2 are NOT the firmware's
     * OUTPUT_POWER_* values (that enum is USER, LOW1..LOW5, MID, HIGH), and
     * dock.c must pass whatever the binding reports straight through rather
     * than assuming the two scales agree. */
    static const uint8_t FAKE_POWER_MAP[3] = { 1u, 6u, 7u };   /* LOW1, MID, HIGH */
    out->status       = DOCK_VFO_APPLIED;
    out->rx_hz        = vfo->rx_hz;
    out->tx_hz        = (vfo->direction == DOCK_OFFSET_ADD) ? vfo->rx_hz + vfo->offset_hz
                      : (vfo->direction == DOCK_OFFSET_SUB) ? vfo->rx_hz - vfo->offset_hz
                      : vfo->rx_hz;
    out->ctcss_tenths = vfo->ctcss_tenths;
    out->power        = FAKE_POWER_MAP[vfo->power];
}
/* set-modulation spy (0x0877). Same shape and same job as the set-VFO spy: the
 * CALL COUNT is what proves a refused frame never reached the radio.
 *
 * g_mod_force_readback lets a test make the fake report a modulation OTHER than the
 * one it was handed. That is not a contrived case — it is the whole reason 0x0878
 * carries a read-back at all, and the 0x0874 `power` bug (wire 2 "high" landing on
 * OUTPUT_POWER_LOW2) is what a reply that merely echoes the request looks like. */
static int     g_mod_calls;
static uint8_t g_mod_last;
static uint8_t g_mod_force_status;    /* non-zero: refuse the way a HAL might */
static int     g_mod_force_readback;  /* <0 = report what was asked for */
static int     g_mod_force_raw;       /* <0 = derive from the readback */
static uint8_t g_mod_force_flags;

static void hal_set_modulation(void *u, uint8_t wire_mod, dock_mod_applied_t *out)
{
    (void)u; g_mod_calls++; g_mod_last = wire_mod;
    if (g_mod_force_status != DOCK_MOD_APPLIED) {
        /* Fill in a plausible-looking modulation on a refusal, deliberately, so the
         * tests can prove dock.c overwrites it with DOCK_MOD_UNKNOWN. A binding that
         * forgets must not be able to publish a modulation the radio is not on. */
        out->status     = g_mod_force_status;
        out->modulation = DOCK_MOD_FM;
        out->raw        = 0u;
        out->flags      = DOCK_MOD_FLAG_TX_OK;
        return;
    }
    const uint8_t applied = (g_mod_force_readback < 0)
                          ? wire_mod : (uint8_t)g_mod_force_readback;
    out->status     = DOCK_MOD_APPLIED;
    out->modulation = applied;
    out->raw        = (g_mod_force_raw < 0) ? applied : (uint8_t)g_mod_force_raw;
    out->flags      = g_mod_force_flags;
}

/* set-FM spy (0x0879). Unlike the modulation spy, which echoes what it is handed,
 * this one MODELS A RECEIVER: it keeps its own on/off, frequency and band, and the
 * reply is read out of that model. That is deliberate. 0x087A exists to report what
 * the radio is ON rather than what it was told, and a fake that echoes cannot tell
 * those two apart — every read-back assertion would pass against a dock.c that simply
 * copied the request back.
 *
 * The three statuses only the radio side can reach are modelled here rather than
 * forced, because that is where they genuinely live: ERR_OFF (tune with the receiver
 * off) and ERR_TX / ERR_BAND, which need the firmware's own FUNCTION_* state and the
 * BK1080 driver's limit tables. */
static int       g_fm_calls;
static dock_fm_t g_fm_last;
static uint8_t   g_fm_force_status;   /* non-zero: refuse the way a HAL might */
static bool      g_fm_on;             /* what the modelled receiver is doing */
static uint32_t  g_fm_freq_hz;        /* retained across off, as the firmware does */
static uint8_t   g_fm_band;
static uint8_t   g_fm_flags;

static void hal_set_fm(void *u, const dock_fm_t *fm, dock_fm_applied_t *out)
{
    (void)u; g_fm_calls++; g_fm_last = *fm;

    if (g_fm_force_status != DOCK_FM_APPLIED) {
        /* Plausible values on a refusal, deliberately, so the tests can prove dock.c
         * overwrites them. Both of these are the answer a host most wants to believe:
         * "the receiver is off" and "band 0". A binding that forgets to blank must not
         * be able to publish either. */
        out->status  = g_fm_force_status;
        out->state   = DOCK_FM_STATE_OFF;
        out->freq_hz = 103200000u;
        out->band    = 0u;
        /* BOTH flag bits set, so the blanking test can prove dock.c clears the whole
         * byte rather than the one bit that existed when it was written. */
        out->flags   = DOCK_FM_FLAG_TX_OK | DOCK_FM_FLAG_FM_BLOCKS_TX;
        return;
    }

    if (fm->action == DOCK_FM_TUNE && !g_fm_on) {
        out->status = DOCK_FM_ERR_OFF;   /* dock.c cannot see this; only the radio can */
        return;                          /* and dock.c blanks the rest */
    }

    if (fm->action == DOCK_FM_OFF) {
        g_fm_on = false;
    } else {
        g_fm_on      = true;
        g_fm_freq_hz = fm->freq_hz;
        g_fm_band    = fm->band;
    }

    out->status  = DOCK_FM_APPLIED;
    out->state   = g_fm_on ? DOCK_FM_STATE_ON : DOCK_FM_STATE_OFF;
    out->freq_hz = g_fm_freq_hz;         /* the firmware keeps it across an off */
    out->band    = g_fm_band;
    out->flags   = g_fm_flags;
}

static const dock_hal_t HAL = {
    hal_read, hal_write, hal_send, NULL, hal_tx, hal_set_vfo, hal_set_modulation,
    hal_set_fm
};
/* A build with no radio-side binding at all — 0x0873 must still answer. */
static const dock_hal_t HAL_NO_VFO = {
    hal_read, hal_write, hal_send, NULL, hal_tx, NULL, hal_set_modulation,
    hal_set_fm
};
/* A build carrying the set-VFO binding but not the set-modulation one — an F6
 * firmware's shape. 0x0877 must still answer, with ERR_NO_HAL. */
static const dock_hal_t HAL_NO_MOD = {
    hal_read, hal_write, hal_send, NULL, hal_tx, hal_set_vfo, NULL,
    hal_set_fm
};
/* A build with the set-modulation binding but not the set-FM one — an F7 firmware's
 * shape, and also a Fusion-preset-without-ENABLE_FMRADIO build. 0x0879 must still
 * answer, with ERR_NO_HAL, rather than fall through to the silence that means
 * "this firmware does not have the command at all". */
static const dock_hal_t HAL_NO_FM = {
    hal_read, hal_write, hal_send, NULL, hal_tx, hal_set_vfo, hal_set_modulation,
    NULL
};

/* Decode the one 0x0874 reply in the capture buffer. False unless exactly one
 * well-formed reply is there, so "sent nothing" can never read as a pass. */
#define VFO_REPLY_FRAME_LEN 24u   /* AB CD | size:2 | (4 + 12 + 2) | DC BA */

static bool last_vfo_reply(dock_vfo_applied_t *out)
{
    if (g_caplen != VFO_REPLY_FRAME_LEN) return false;
    uint8_t body[4 + 12 + 2];
    memcpy(body, g_cap + 4, sizeof(body));
    dock_obfuscate(body, (uint16_t)sizeof(body));
    if ((uint16_t)(body[0] | (body[1] << 8)) != DOCK_REPLY_SET_VFO) return false;
    if ((uint16_t)(body[2] | (body[3] << 8)) != 12u) return false;
    out->status = body[4];
    out->power  = body[5];
    out->rx_hz  = (uint32_t)body[6]  | ((uint32_t)body[7] << 8)
                | ((uint32_t)body[8] << 16) | ((uint32_t)body[9] << 24);
    out->tx_hz  = (uint32_t)body[10] | ((uint32_t)body[11] << 8)
                | ((uint32_t)body[12] << 16) | ((uint32_t)body[13] << 24);
    out->ctcss_tenths = (uint16_t)(body[14] | (body[15] << 8));
    return true;
}

/* Decode the one 0x0878 reply in the capture buffer. Same "exactly one well-formed
 * frame or false" rule as last_vfo_reply — silence must never read as a pass. */
#define MOD_REPLY_FRAME_LEN 16u   /* AB CD | size:2 | (4 + 4 + 2) | DC BA */

static bool last_mod_reply(dock_mod_applied_t *out)
{
    if (g_caplen != MOD_REPLY_FRAME_LEN) return false;
    uint8_t body[4 + 4 + 2];
    memcpy(body, g_cap + 4, sizeof(body));
    dock_obfuscate(body, (uint16_t)sizeof(body));
    if ((uint16_t)(body[0] | (body[1] << 8)) != DOCK_REPLY_SET_MOD) return false;
    if ((uint16_t)(body[2] | (body[3] << 8)) != 4u) return false;
    out->status     = body[4];
    out->modulation = body[5];
    out->raw        = body[6];
    out->flags      = body[7];
    return true;
}

/* Decode the one 0x087A reply in the capture buffer. Same "exactly one well-formed
 * frame or false" rule as the two above — silence must never read as a pass. */
#define FM_REPLY_FRAME_LEN 20u   /* AB CD | size:2 | (4 + 8 + 2) | DC BA */

static bool last_fm_reply(dock_fm_applied_t *out)
{
    if (g_caplen != FM_REPLY_FRAME_LEN) return false;
    uint8_t body[4 + 8 + 2];
    memcpy(body, g_cap + 4, sizeof(body));
    dock_obfuscate(body, (uint16_t)sizeof(body));
    if ((uint16_t)(body[0] | (body[1] << 8)) != DOCK_REPLY_SET_FM) return false;
    if ((uint16_t)(body[2] | (body[3] << 8)) != 8u) return false;
    out->status  = body[4];
    out->state   = body[5];
    out->freq_hz = (uint32_t)body[6] | ((uint32_t)body[7] << 8)
                 | ((uint32_t)body[8] << 16) | ((uint32_t)body[9] << 24);
    out->band    = body[10];
    out->flags   = body[11];
    return true;
}

static dock_ctx_t ctx;

static void reset(void)
{
    memset(g_regs, 0, sizeof(g_regs));
    g_caplen = 0; g_reads = 0; g_writes = 0;
    g_tx_calls = 0; g_tx_last = -1; g_evn = 0; memset(g_ev, 0, sizeof(g_ev));
    g_vfo_calls = 0; memset(&g_vfo_last, 0, sizeof(g_vfo_last));
    g_vfo_force_status = DOCK_VFO_APPLIED;
    g_mod_calls = 0; g_mod_last = 0xEE;
    g_mod_force_status = DOCK_MOD_APPLIED;
    g_mod_force_readback = -1; g_mod_force_raw = -1;
    g_mod_force_flags = DOCK_MOD_FLAG_TX_OK;
    g_fm_calls = 0; memset(&g_fm_last, 0, sizeof(g_fm_last));
    g_fm_force_status = DOCK_FM_APPLIED;
    /* The modelled receiver starts OFF on band 0 with nothing tuned — a radio nobody
     * has put into broadcast FM, which is the state every real one boots into
     * (board.c powers the BK1080 down at BOARD_Init). */
    g_fm_on = false; g_fm_freq_hz = 0u; g_fm_band = 0u;
    g_fm_flags = DOCK_FM_FLAG_TX_OK;
    dock_init(&ctx, &HAL);
}

/* Build an obfuscated COMMAND frame with a real CRC (what radio-server sends).
 * payload = [opcode:u16][param_len:u16][params]; frame Size = 4 + param_len. */
static uint16_t build_cmd(uint8_t *out, uint16_t opcode,
                          const uint8_t *params, uint16_t plen)
{
    uint8_t body[DOCK_MAX_PAYLOAD + 2];
    uint16_t size = (uint16_t)(4 + plen);
    body[0] = (uint8_t)(opcode & 0xFF); body[1] = (uint8_t)(opcode >> 8);
    body[2] = (uint8_t)(plen & 0xFF);   body[3] = (uint8_t)(plen >> 8);
    memcpy(body + 4, params, plen);
    uint16_t crc = dock_crc16(body, size);
    body[size] = (uint8_t)(crc & 0xFF); body[size + 1] = (uint8_t)(crc >> 8);
    dock_obfuscate(body, (uint16_t)(size + 2));

    uint16_t n = 0;
    out[n++] = 0xAB; out[n++] = 0xCD;
    out[n++] = (uint8_t)(size & 0xFF); out[n++] = (uint8_t)(size >> 8);
    memcpy(out + n, body, size + 2); n = (uint16_t)(n + size + 2);
    out[n++] = 0xDC; out[n++] = 0xBA;
    return n;
}

static void feed(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) dock_rx_byte(&ctx, buf[i]);
}

/* 0x0879 params: [action:u8][freq_hz:u32 LE][band:u8]. */
static uint16_t p_fm(uint8_t *p, uint8_t action, uint32_t hz, uint8_t band)
{
    p[0] = action;
    p[1] = (uint8_t)(hz & 0xFF);
    p[2] = (uint8_t)((hz >> 8) & 0xFF);
    p[3] = (uint8_t)((hz >> 16) & 0xFF);
    p[4] = (uint8_t)((hz >> 24) & 0xFF);
    p[5] = band;
    return DOCK_SET_FM_PARAM_LEN;
}

/* little-endian param builders */
static uint16_t p_read(uint8_t *p, const uint16_t *regs, uint16_t count)
{
    uint16_t n = 0;
    p[n++] = (uint8_t)(count & 0xFF); p[n++] = (uint8_t)(count >> 8);
    for (uint16_t i = 0; i < count; i++) {
        p[n++] = (uint8_t)(regs[i] & 0xFF); p[n++] = (uint8_t)(regs[i] >> 8);
    }
    return n;
}
static uint16_t p_write(uint8_t *p, const uint16_t *pairs, uint16_t count)
{
    uint16_t n = 0;
    p[n++] = (uint8_t)(count & 0xFF); p[n++] = (uint8_t)(count >> 8);
    for (uint16_t i = 0; i < count; i++) {
        p[n++] = (uint8_t)(pairs[i * 2] & 0xFF);     p[n++] = (uint8_t)(pairs[i * 2] >> 8);
        p[n++] = (uint8_t)(pairs[i * 2 + 1] & 0xFF); p[n++] = (uint8_t)(pairs[i * 2 + 1] >> 8);
    }
    return n;
}

int main(void)
{
    uint8_t frame[512], params[512];
    uint16_t flen, plen;

    /* 1. Connect probe: ReadRegisters([0x30]) -> exactly one 0x0951 reply,
     *    byte-exact against a hand-computed golden (independent oracle). */
    reset();
    g_regs[0x30] = 0xC1FE;
    { uint16_t r[] = { 0x30 }; plen = p_read(params, r, 1); }
    flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
    feed(frame, flen);
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x08, 0x00,
            0x47, 0x65, 0x10, 0xE6, 0x1E, 0x91, 0xF3, 0x81, 0xDE, 0xCA,
            0xDC, 0xBA,
        };
        CHECK(g_reads == 1, "connect probe: exactly one register read");
        CHECK(g_caplen == sizeof(golden), "connect probe: reply length");
        CHECK(g_caplen == sizeof(golden) &&
              memcmp(g_cap, golden, sizeof(golden)) == 0,
              "connect probe: byte-exact 0x0951 reply vector");
    }

    /* 2. WriteRegisters -> no reply, register store updated. */
    reset();
    { uint16_t pr[] = { 0x30, 0xC1FE }; plen = p_write(params, pr, 1); }
    flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen);
    feed(frame, flen);
    CHECK(g_writes == 1 && g_regs[0x30] == 0xC1FE, "write: store updated");
    CHECK(g_caplen == 0, "write: no reply");

    /* 3. ReadRegisters of several -> one 0x0951 per register, in order. */
    reset();
    g_regs[0x38] = 0x1111; g_regs[0x39] = 0x2222; g_regs[0x33] = 0x3333;
    { uint16_t r[] = { 0x38, 0x39, 0x33 }; plen = p_read(params, r, 3); }
    flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
    feed(frame, flen);
    CHECK(g_reads == 3, "multi-read: three reads");
    CHECK(g_caplen == 3 * 16, "multi-read: three framed replies");
    /* second reply's register field (offset 16 + 4, de-obfuscated) == 0x39 */
    if (g_caplen == 48) {
        uint8_t body[10];
        memcpy(body, g_cap + 16 + 4, 10);            /* 2nd frame, payload at +4 */
        dock_obfuscate(body, 10);
        /* payload = [opcode:u16][param_len:u16][reg:u16][value:u16] */
        uint16_t op  = (uint16_t)(body[0] | (body[1] << 8));
        uint16_t reg = (uint16_t)(body[4] | (body[5] << 8));
        uint16_t val = (uint16_t)(body[6] | (body[7] << 8));
        CHECK(op == DOCK_REPLY_REG_INFO && reg == 0x39 && val == 0x2222,
              "multi-read: 2nd reply opcode/reg/value");
    }

    /* 4. Bad command CRC -> dropped: no dispatch, no reply. */
    reset();
    { uint16_t r[] = { 0x30 }; plen = p_read(params, r, 1); }
    flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
    frame[6] ^= 0xFF;                 /* corrupt an obfuscated payload byte */
    feed(frame, flen);
    CHECK(g_reads == 0 && g_caplen == 0, "bad CRC: dropped");

    /* 5. Bad footer -> dropped. */
    reset();
    { uint16_t r[] = { 0x30 }; plen = p_read(params, r, 1); }
    flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
    frame[flen - 1] = 0x00;           /* break 0xBA */
    feed(frame, flen);
    CHECK(g_reads == 0 && g_caplen == 0, "bad footer: dropped");

    /* 6. Zero-size frame -> dropped, and a following good frame still parses. */
    reset();
    g_regs[0x30] = 0x1234;
    {
        uint8_t junk[] = { 0xAB, 0xCD, 0x00, 0x00, 0xDC, 0xBA };
        feed(junk, sizeof(junk));
        CHECK(g_caplen == 0, "zero-size: dropped");
        { uint16_t r[] = { 0x30 }; plen = p_read(params, r, 1); }
        flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
        feed(frame, flen);
        CHECK(g_reads == 1, "recover after zero-size");
    }

    /* 7. Oversize (Size > 254) -> dropped. */
    reset();
    {
        uint8_t big[8] = { 0xAB, 0xCD, 0xFF, 0x00 /* Size=255 */, 0, 0, 0xDC, 0xBA };
        feed(big, sizeof(big));
        CHECK(g_reads == 0 && g_caplen == 0, "oversize: dropped");
    }

    /* 8. Leading garbage + bad 2nd-preamble byte, then a good frame. */
    reset();
    g_regs[0x30] = 0x55AA;
    {
        uint8_t noise[] = { 0x00, 0x11, 0xAB, 0x00, 0xAB, 0x22 };  /* AB not followed by CD */
        feed(noise, sizeof(noise));
        { uint16_t r[] = { 0x30 }; plen = p_read(params, r, 1); }
        flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
        feed(frame, flen);
        CHECK(g_reads == 1, "resync past garbage / bad 2nd preamble");
    }

    /* 9. Frame split one byte at a time still parses (streaming). */
    reset();
    g_regs[0x30] = 0xABCD;
    { uint16_t r[] = { 0x30 }; plen = p_read(params, r, 1); }
    flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
    feed(frame, flen);               /* feed() already delivers byte-by-byte */
    CHECK(g_reads == 1 && g_caplen == 16, "streaming byte-by-byte");

    /* 10. Full-control: 0x0870 sets flag; register R/W works while in it;
     *     0x0871 clears it. (Matches the fake's full_control flag + shared dispatch.) */
    reset();
    flen = build_cmd(frame, DOCK_CMD_ENTER_HW, params, 0);
    feed(frame, flen);
    CHECK(ctx.full_control, "0x0870 enters full-control");
    g_regs[0x30] = 0x0BEE;
    { uint16_t r[] = { 0x30 }; plen = p_read(params, r, 1); }
    flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
    feed(frame, flen);
    CHECK(g_reads == 1 && ctx.full_control, "read works inside full-control");
    flen = build_cmd(frame, DOCK_CMD_EXIT_HW, params, 0);
    feed(frame, flen);
    CHECK(!ctx.full_control, "0x0871 exits full-control");

    /* 11. Malformed frame immediately followed by a good one, single buffer. */
    reset();
    g_regs[0x51] = 0x9999;
    {
        uint8_t buf[600]; uint16_t n = 0;
        uint16_t bad = build_cmd(frame, DOCK_CMD_READ_REGS, params, 0); /* count=0 read: valid but no reply */
        (void)bad;
        /* a truly malformed lead: AB CD with wrong footer */
        uint8_t junk[] = { 0xAB, 0xCD, 0x04, 0x00, 1, 2, 3, 4, 5, 6, 0x00, 0x00 };
        memcpy(buf + n, junk, sizeof(junk)); n = (uint16_t)(n + sizeof(junk));
        { uint16_t r[] = { 0x51 }; plen = p_read(params, r, 1); }
        flen = build_cmd(frame, DOCK_CMD_READ_REGS, params, plen);
        memcpy(buf + n, frame, flen); n = (uint16_t)(n + flen);
        feed(buf, n);
        CHECK(g_reads == 1, "good frame after malformed in same buffer");
    }

    /* ===== F5: REG_30 TX-state seam (drives the physical PA) ===== */
    /* The app's TX word is 0xC1FE (ENABLE_TX_DSP, bit1, set); its RX word is
     * 0xBFF1 (TX_DSP clear). dock.c edge-detects that bit and calls tx_set()
     * BEFORE completing the REG_30 write. */

    /* 12. Key: write REG_30=0xC1FE -> tx_set(true) fires once, before the write. */
    reset();
    { uint16_t pr[] = { 0x30, 0xC1FE }; plen = p_write(params, pr, 1); }
    flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen);
    feed(frame, flen);
    CHECK(g_tx_calls == 1 && g_tx_last == 1, "key: tx_set(true) fires once");
    CHECK(strcmp(g_ev, "1w") == 0, "key: PA engaged BEFORE the REG_30 write");
    CHECK(ctx.tx_on && g_regs[0x30] == 0xC1FE, "key: tx_on set, REG_30 written");

    /* 13. Key then un-key: each edge drives the PA before its write ("1w0w"). */
    reset();
    { uint16_t pr[] = { 0x30, 0xC1FE }; plen = p_write(params, pr, 1); }
    flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen); feed(frame, flen);
    { uint16_t pr[] = { 0x30, 0xBFF1 }; plen = p_write(params, pr, 1); }
    flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen); feed(frame, flen);
    CHECK(g_tx_calls == 2 && g_tx_last == 0, "un-key: tx_set(false) fires");
    CHECK(strcmp(g_ev, "1w0w") == 0, "each PA edge precedes its REG_30 write");
    CHECK(!ctx.tx_on, "un-key: tx_on cleared");

    /* 14. Edge-detect: repeated same-state key writes fire tx_set once. */
    reset();
    for (int k = 0; k < 3; k++) {
        uint16_t pr[] = { 0x30, 0xC1FE }; plen = p_write(params, pr, 1);
        flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen); feed(frame, flen);
    }
    CHECK(g_tx_calls == 1 && g_writes == 3, "repeat key: one PA engage, three writes");
    CHECK(strcmp(g_ev, "1www") == 0, "repeat key: PA edge only on the first write");

    /* 15. Writes to other registers never touch the TX state. */
    reset();
    { uint16_t pr[] = { 0x38, 0xC1FE, 0x36, 0x7FFF }; plen = p_write(params, pr, 2); }
    flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen);
    feed(frame, flen);
    CHECK(g_tx_calls == 0 && !ctx.tx_on, "non-REG_30 writes never key");

    /* 16. Fail-safe: a dangling key is dropped at 0x0870 enter and at 0x0871 exit. */
    reset();
    { uint16_t pr[] = { 0x30, 0xC1FE }; plen = p_write(params, pr, 1); }        /* key, no un-key */
    flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen); feed(frame, flen);
    flen = build_cmd(frame, DOCK_CMD_ENTER_HW, params, 0); feed(frame, flen);   /* 0x0870 */
    CHECK(g_tx_calls == 2 && g_tx_last == 0 && !ctx.tx_on, "0x0870 enter clears a stale key");
    { uint16_t pr[] = { 0x30, 0xC1FE }; plen = p_write(params, pr, 1); }        /* key again inside */
    flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen); feed(frame, flen);
    CHECK(g_tx_calls == 3 && ctx.tx_on, "re-key inside full-control");
    flen = build_cmd(frame, DOCK_CMD_EXIT_HW, params, 0); feed(frame, flen);    /* 0x0871 */
    CHECK(g_tx_calls == 4 && g_tx_last == 0 && !ctx.tx_on, "0x0871 exit drops the PA");

    /* ---- 0x0873 set-VFO -------------------------------------------------
     *
     * This is the one command whose whole point is to outlive the dock session,
     * so its failure modes are different in kind from the register commands:
     * a register write that goes wrong is undone by the next 0x0871, while a
     * VFO written wrong is what the radio transmits on after we walk away.
     * Hence "refuse" rather than "clamp" everywhere below. */

    /* K0PRA: receive 448.525, transmit 5 MHz down, 100.0 Hz, wide, high power. */
    static const uint8_t K0PRA[DOCK_SET_VFO_PARAM_LEN] = {
        0xC8, 0xF2, 0xBB, 0x1A,   /* rx_hz  448 525 000 */
        0x40, 0x4B, 0x4C, 0x00,   /* offset   5 000 000 */
        0xE8, 0x03,               /* ctcss tenths 1000 = 100.0 Hz */
        DOCK_OFFSET_SUB,          /* direction */
        0x00,                     /* wide */
        0x02,                     /* high power */
    };
    dock_vfo_applied_t rep;

    /* 17. A well-formed repeater channel decodes field for field. */
    reset();
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(g_vfo_calls == 1, "0x0873: a well-formed channel is applied once");
    CHECK(g_vfo_last.rx_hz == 448525000u, "0x0873: rx frequency decoded");
    CHECK(g_vfo_last.offset_hz == 5000000u, "0x0873: offset decoded");
    CHECK(g_vfo_last.ctcss_tenths == 1000u, "0x0873: CTCSS decoded in tenths");
    CHECK(g_vfo_last.direction == DOCK_OFFSET_SUB, "0x0873: direction decoded");
    CHECK(g_vfo_last.narrow == 0 && g_vfo_last.power == 2, "0x0873: bandwidth/power decoded");
    CHECK(g_writes == 0, "0x0873: writes no registers itself");
    CHECK(last_vfo_reply(&rep), "0x0874: exactly one well-formed reply");
    CHECK(rep.status == DOCK_VFO_APPLIED, "0x0874: applied");
    CHECK(rep.rx_hz == 448525000u && rep.tx_hz == 443525000u,
          "0x0874: reports BOTH legs, so the caller learns where it will radiate");
    CHECK(rep.ctcss_tenths == 1000u, "0x0874: reports the tone actually set");
    CHECK(rep.power == 7u,
          "0x0874: reports the RADIO's power level, not the 0/1/2 that was sent");

    /* 17b. Byte-exact reply vector — an oracle independent of this file's own
     *      builders, and the thing radio-server's decoder is written against. */
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x10, 0x00, 0x62, 0x64, 0x18, 0xE6,
            0x2E, 0x96, 0xC5, 0xB2, 0x9A, 0x2F, 0x5D, 0xE7,
            0x7C, 0x19, 0x01, 0x83, 0xE9, 0x93, 0xDC, 0xBA,
        };
        CHECK(g_caplen == sizeof(golden), "0x0874: golden reply length");
        CHECK(g_caplen == sizeof(golden) &&
              memcmp(g_cap, golden, sizeof(golden)) == 0, "0x0874: byte-exact reply");
    }

    /* 17c. The power byte is a conduit, not a copy of the request: asking for
     *      mid (1) must come back as the radio's own MID, not as 1. */
    reset();
    {
        uint8_t v[DOCK_SET_VFO_PARAM_LEN];
        memcpy(v, K0PRA, sizeof(v));
        v[12] = 1;                                 /* wire "mid" */
        flen = build_cmd(frame, DOCK_CMD_SET_VFO, v, sizeof(v));
        feed(frame, flen);
    }
    CHECK(last_vfo_reply(&rep) && rep.power == 6u,
          "0x0874: the wire's mid maps onto the radio's own scale, and is reported as such");

    /* 18. A simplex channel is expressible: no offset, no tone. */
    reset();
    {
        uint8_t v[DOCK_SET_VFO_PARAM_LEN] = {
            0x40, 0x5E, 0x92, 0x1A,   /* 445 800 000 */
            0, 0, 0, 0,               /* no offset */
            0, 0,                     /* no tone */
            DOCK_OFFSET_NONE, 0, 1,
        };
        flen = build_cmd(frame, DOCK_CMD_SET_VFO, v, sizeof(v));
        feed(frame, flen);
    }
    CHECK(g_vfo_calls == 1 && g_vfo_last.rx_hz == 445800000u, "0x0873: simplex channel applies");
    CHECK(g_vfo_last.ctcss_tenths == 0 && g_vfo_last.direction == DOCK_OFFSET_NONE,
          "0x0873: no tone and no offset survive as zero, not as garbage");
    CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_APPLIED
          && rep.rx_hz == 445800000u && rep.tx_hz == 445800000u,
          "0x0874: simplex reports the same frequency on both legs");

    /* 19. A truncated payload is refused, not read past. The frame is otherwise
     *     valid, so nothing but the length check stands between a short frame
     *     and reading whatever follows it in the RX buffer. It still answers:
     *     a caller that mis-sized its own frame most needs to be told. */
    reset();
    {
        uint8_t v[DOCK_SET_VFO_PARAM_LEN - 1] = { 0 };
        flen = build_cmd(frame, DOCK_CMD_SET_VFO, v, sizeof(v));
        feed(frame, flen);
    }
    CHECK(g_vfo_calls == 0, "0x0873: a short payload is refused");
    CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_ERR_SHORT,
          "0x0874: a short payload is REPORTED, not swallowed");

    /* 20. Out-of-range fields are refused rather than clamped, and each names
     *     its own reason. A bad direction byte silently treated as "simplex"
     *     would transmit on the repeater's OUTPUT — on top of the machine, and
     *     on top of whoever it is repeating. */
    {
        uint8_t v[DOCK_SET_VFO_PARAM_LEN];
        memcpy(v, K0PRA, sizeof(v));

        reset();
        v[10] = 0x07;                             /* direction: not NONE/ADD/SUB */
        flen = build_cmd(frame, DOCK_CMD_SET_VFO, v, sizeof(v));
        feed(frame, flen);
        CHECK(g_vfo_calls == 0, "0x0873: an unknown offset direction is refused");
        CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_ERR_DIRECTION,
              "0x0874: the direction is named as the reason");

        reset();
        memcpy(v, K0PRA, sizeof(v));
        v[11] = 0x05;                             /* bandwidth: neither wide nor narrow */
        flen = build_cmd(frame, DOCK_CMD_SET_VFO, v, sizeof(v));
        feed(frame, flen);
        CHECK(g_vfo_calls == 0, "0x0873: an unknown bandwidth is refused");
        CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_ERR_FIELD,
              "0x0874: a bad bandwidth is reported");

        reset();
        memcpy(v, K0PRA, sizeof(v));
        v[12] = 0x09;                             /* power: off the end of the scale */
        flen = build_cmd(frame, DOCK_CMD_SET_VFO, v, sizeof(v));
        feed(frame, flen);
        CHECK(g_vfo_calls == 0, "0x0873: an unknown power level is refused");
        CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_ERR_FIELD,
              "0x0874: a bad power level is reported");
    }

    /* 21. Refused inside full-control. Applying a VFO mid-dock would call
     *     RADIO_SetupRegisters underneath a host that believes it owns the
     *     synthesiser — the "adopt whatever you find" fault ADR 0132 removed. */
    reset();
    flen = build_cmd(frame, DOCK_CMD_ENTER_HW, params, 0); feed(frame, flen);
    g_caplen = 0;                                  /* drop the enter's own traffic */
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(g_vfo_calls == 0, "0x0873: refused while the host holds full-control");
    CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_ERR_BUSY,
          "0x0874: full-control is named, so the caller can retry after 0x0871");

    flen = build_cmd(frame, DOCK_CMD_EXIT_HW, params, 0); feed(frame, flen);
    g_caplen = 0;
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(g_vfo_calls == 1, "0x0873: accepted once full-control is released");
    CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_APPLIED,
          "0x0874: applied after full-control is released");

    /* 22. Never keys. This command runs while the radio is a radio, so if it
     *     could touch the TX state it would key one outside the dock's own
     *     fail-safe seams, with nothing tracking it. */
    CHECK(g_tx_calls == 0 && !ctx.tx_on, "0x0873: no PA activity of its own");

    /* 23. A build with no radio-side binding still answers. Silence here would
     *     be indistinguishable from a radio that is not listening at all. */
    reset();
    dock_init(&ctx, &HAL_NO_VFO);
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_ERR_NO_HAL,
          "0x0874: a missing set_vfo binding is reported, not silent");

    /* 24. A rejection never carries frequencies. The fake binding fills them in
     *     on purpose; dock.c must blank them, so no caller can ever read a
     *     channel off a reply that says the radio is not on one. This is the
     *     whole contract in one case: status first, and it is authoritative. */
    reset();
    g_vfo_force_status = DOCK_VFO_ERR_BAND;
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_ERR_BAND,
          "0x0874: an out-of-band refusal from the radio side is reported");
    CHECK(rep.rx_hz == 0 && rep.tx_hz == 0 && rep.ctcss_tenths == 0 && rep.power == 0,
          "0x0874: a non-zero status ships no frequencies, whatever the HAL wrote");

    reset();
    g_vfo_force_status = DOCK_VFO_ERR_TONE;
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_ERR_TONE
          && rep.rx_hz == 0 && rep.ctcss_tenths == 0,
          "0x0874: an unresolvable tone refuses the whole tune, on frequency or not");

    /* ================= 0x0877 set-modulation / 0x0878 (F7) ================= */

    dock_mod_applied_t mrep;
    static const uint8_t MOD_AM[1] = { DOCK_MOD_AM };

    /* 25. Byte-exact COMMAND vector. Not a test of dock.c's dispatch — it exercises
     *     this file's own builder — but it is the artifact a third-party client is
     *     implemented against, and the thing that catches a client which mis-sizes
     *     param_len, byte-swaps the opcode, or CRCs the obfuscated bytes instead of
     *     the plaintext. Cross-checked against a separately written framer that
     *     reproduces PROTOCOL.md's published 0x0951 and 0x0874 vectors exactly. */
    reset();
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x05, 0x00,
            0x61, 0x64, 0x15, 0xE6, 0x2F, 0x11, 0xD5,
            0xDC, 0xBA,
        };
        flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
        CHECK(flen == sizeof(golden), "0x0877: golden command length");
        CHECK(flen == sizeof(golden) && memcmp(frame, golden, sizeof(golden)) == 0,
              "0x0877: byte-exact command vector");
    }

    /* 26. Byte-exact REPLY vector, the oracle radio-server's decoder is written
     *     against. AM applies but cannot transmit on this build, so flags = 0 —
     *     which is exactly the case a host most needs to read correctly. */
    reset();
    g_mod_force_flags = 0;                       /* AM: the radio will not key */
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x08, 0x00,
            0x6E, 0x64, 0x10, 0xE6, 0x2E, 0x90, 0x0C, 0x40, 0xDE, 0xCA,
            0xDC, 0xBA,
        };
        CHECK(g_caplen == sizeof(golden), "0x0878: golden reply length");
        CHECK(g_caplen == sizeof(golden) &&
              memcmp(g_cap, golden, sizeof(golden)) == 0,
              "0x0878: byte-exact reply vector");
    }

    /* 27. A truncated payload is refused before params[0] is ever read. This is
     *     what makes an EMPTY 0x0877 a safe firmware-level probe: the length check
     *     is the first branch, so the frame cannot move the radio, and any 0x0878
     *     at all answers "this firmware is F7 or later". */
    reset();
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, params, 0);
    feed(frame, flen);
    CHECK(g_mod_calls == 0, "0x0877: an empty payload is refused, not read past");
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_ERR_SHORT,
          "0x0878: a short payload is REPORTED, so the probe gets an answer");
    CHECK(mrep.modulation == DOCK_MOD_UNKNOWN && mrep.raw == DOCK_MOD_UNKNOWN,
          "0x0878: a refusal names no modulation — 0xFF, never 0 (0 is FM)");

    /* 28. Out of range is refused, never clamped. USB's number is reserved but the
     *     value is not accepted at F7, and 0x09 is not a modulation at all. A clamp
     *     would leave the radio demodulating something nobody asked for while the
     *     reply said it worked. */
    reset();
    {
        uint8_t v[1] = { DOCK_MOD_USB };
        flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, v, sizeof(v));
        feed(frame, flen);
    }
    CHECK(g_mod_calls == 0, "0x0877: reserved-but-unaccepted USB is refused");
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_ERR_FIELD,
          "0x0878: USB refusal is named");

    reset();
    {
        uint8_t v[1] = { 0x09 };
        flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, v, sizeof(v));
        feed(frame, flen);
    }
    CHECK(g_mod_calls == 0, "0x0877: nonsense is refused, not folded into range");
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_ERR_FIELD
          && mrep.modulation == DOCK_MOD_UNKNOWN,
          "0x0878: an off-scale value refuses and names no modulation");

    /* 29. Refused inside full-control, for the same reason 0x0873 is: applying it
     *     would run RADIO_SetupRegisters underneath a host that believes it owns
     *     the synthesiser. Ordering matters — SHORT is checked before BUSY, and
     *     BUSY before the value — so a caller learns the most fundamental problem
     *     first rather than the last one to be looked at. */
    reset();
    flen = build_cmd(frame, DOCK_CMD_ENTER_HW, params, 0); feed(frame, flen);
    g_caplen = 0;
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(g_mod_calls == 0, "0x0877: refused while the host holds full-control");
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_ERR_BUSY,
          "0x0878: full-control is named, so the caller can retry after 0x0871");

    flen = build_cmd(frame, DOCK_CMD_EXIT_HW, params, 0); feed(frame, flen);
    g_caplen = 0;
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(g_mod_calls == 1, "0x0877: accepted once full-control is released");
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_APPLIED,
          "0x0878: applied after full-control is released");

    /* 30. A REFUSAL MUST NOT MOVE THE STICKY VALUE. The single most important case
     *     here. Set AM (applied), hold full-control, ask for FM (refused BUSY),
     *     release, then tune — the tune must still carry AM. If the sticky value
     *     were committed during decode rather than after the radio actually took
     *     it, that refused FM would silently become what every later 0x0873
     *     applies: the exact silent-revert bug this mechanism removes, in mirror
     *     image, and invisible because the refusal itself was reported correctly. */
    reset();
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_APPLIED,
          "0x0878: AM applied (sticky-refusal setup)");

    g_caplen = 0;
    flen = build_cmd(frame, DOCK_CMD_ENTER_HW, params, 0); feed(frame, flen);
    {
        uint8_t v[1] = { DOCK_MOD_FM };
        g_caplen = 0;
        flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, v, sizeof(v));
        feed(frame, flen);
    }
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_ERR_BUSY,
          "0x0878: the FM request is refused while full-control is held");

    flen = build_cmd(frame, DOCK_CMD_EXIT_HW, params, 0); feed(frame, flen);
    g_caplen = 0;
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(g_vfo_last.modulation == DOCK_MOD_AM,
          "0x0873: a REFUSED set-modulation never became the sticky value");

    /* 31. A valid frame reaches the binding exactly once, and does nothing else.
     *     NOTE: this stays green against a stub that skips every check — it
     *     discriminates against a dispatch that never calls the binding or calls it
     *     twice, not against one that fails to validate. Do not read it as evidence
     *     the refusals work; that is what 27-30 are for. */
    reset();
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(g_mod_calls == 1 && g_mod_last == DOCK_MOD_AM,
          "0x0877: a valid frame reaches the binding exactly once, with the value sent");
    CHECK(g_writes == 0 && g_reads == 0 && g_vfo_calls == 0,
          "0x0877: no register traffic and no set-VFO of its own");
    CHECK(g_tx_calls == 0 && !ctx.tx_on, "0x0877: no PA activity of its own");

    /* 32. THE POINT OF THE STICKY VALUE. Set AM, then tune — the tune must carry
     *     AM into the binding. Without this, uart.c's Dock_ApplyVfo writes
     *     MODULATION_FM literally and a tune silently undoes the modulation, with
     *     nothing on the wire saying so. Sending the two as separate frames is not
     *     just awkward, it is unreliable: this link drops frames (ADR 0131), so a
     *     dropped set-modulation after a tune leaves the radio on the right channel
     *     in the wrong demodulator. */
    reset();
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    g_caplen = 0;
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(g_vfo_calls == 1 && g_vfo_last.modulation == DOCK_MOD_AM,
          "0x0873: carries the modulation 0x0877 set, so a tune cannot revert it");
    CHECK(last_vfo_reply(&rep) && rep.status == DOCK_VFO_APPLIED,
          "0x0874: the tune still applies normally with a sticky modulation");

    /* 33. Backward compatibility, stated as a test. A host that never sends 0x0877
     *     must see F6 behaviour exactly — so the sticky value is seeded FM, from a
     *     constant. Seeding it by reading the radio's current modulation would pass
     *     this only by luck and would be the ADR 0132 "adopt whatever state you
     *     find" fault: a repeater channel tuned in whatever the front panel was
     *     last left on. */
    reset();
    flen = build_cmd(frame, DOCK_CMD_SET_VFO, K0PRA, sizeof(K0PRA));
    feed(frame, flen);
    CHECK(g_vfo_calls == 1 && g_vfo_last.modulation == DOCK_MOD_FM,
          "0x0873: FM without a prior 0x0877 — an F6 host sees no change at all");

    /* 34. The reply REPORTS, it does not ECHO. The fake is told to come back with
     *     FM after being asked for AM; the reply must say FM while the sticky value
     *     holds the AM that was requested and accepted. This is the 0x0874 `power`
     *     bug in a new field: the wire's "high" landed on OUTPUT_POWER_LOW2 and a
     *     reply that echoed the request would have confirmed it perfectly. */
    reset();
    g_mod_force_readback = DOCK_MOD_FM;
    g_mod_force_raw      = 0;
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_APPLIED
          && mrep.modulation == DOCK_MOD_FM,
          "0x0878: reports the read-back, not the value it was handed");
    CHECK(ctx.modulation == DOCK_MOD_AM,
          "0x0877: the sticky value is the accepted request, reported independently");

    /* 35. A modulation the wire cannot name survives as UNKNOWN. On a build with
     *     ENABLE_BYP_RAW_DEMODULATORS the radio has modes this protocol has no word
     *     for, and the firmware enum's numbering shifts underneath them. Collapsing
     *     one onto index 0 would report FM — a specific, wrong, believable answer.
     *     `raw` still carries the radio's own value, which is the only thing that
     *     can tell BYP from RAW at a bench at 2 a.m. */
    reset();
    g_mod_force_readback = DOCK_MOD_UNKNOWN;
    g_mod_force_raw      = 3;                    /* MODULATION_BYP on such a build */
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_APPLIED
          && mrep.modulation == DOCK_MOD_UNKNOWN && mrep.raw == 3,
          "0x0878: an unnameable modulation stays UNKNOWN and keeps its raw value");

    /* 36. An F6-shaped build — set-VFO bound, set-modulation not — still answers.
     *     Silence is what a pre-F7 firmware does, and it is how a host detects the
     *     level; a firmware that HAS the opcode but no binding must not imitate it. */
    reset();
    dock_init(&ctx, &HAL_NO_MOD);
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(g_caplen == MOD_REPLY_FRAME_LEN, "0x0878: a missing binding still replies");
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_ERR_NO_HAL,
          "0x0878: a missing set_modulation binding is reported, not silent");

    /* 37. A rejection never describes a modulation — enforced in dock.c, not left
     *     to each binding. The fake writes FM and TX_OK on its refusal on purpose.
     *     Blanking to 0 here (0x0874 blanks its frequencies to 0) would answer
     *     "refused, and by the way you are on FM, and you can transmit" — which is
     *     worse than 0x0874's case, because 0 Hz is obviously not a channel and 0
     *     IS a valid modulation. */
    reset();
    g_mod_force_status = DOCK_MOD_ERR_FIELD;
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(last_mod_reply(&mrep) && mrep.status == DOCK_MOD_ERR_FIELD,
          "0x0878: a refusal from the radio side is reported");
    CHECK(mrep.modulation == DOCK_MOD_UNKNOWN && mrep.raw == DOCK_MOD_UNKNOWN
          && mrep.flags == 0,
          "0x0878: a non-zero status ships no modulation and no TX claim, whatever the HAL wrote");

    /* 38. An unknown opcode is still answered with silence. This is not a detail —
     *     it is the whole firmware-level detection mechanism: a new host sending
     *     0x0877 to a pre-F7 radio must get nothing back, so "no reply" means "does
     *     not have it". A default: that started replying would break every level
     *     probe at once, including 0x0873's. */
    reset();
    flen = build_cmd(frame, 0x087Fu, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    CHECK(g_caplen == 0, "unknown opcode: still no reply, so absence stays detectable");

    /* ================= 0x0879 / 0x087A — broadcast FM, the BK1080 (F8) =========
     *
     * A different chip from everything above. These tests are about three things the
     * other opcodes do not have to care about: a frequency raster that must be
     * refused rather than rounded, a band field the firmware would silently clamp,
     * and a receiver whose state the reply has to READ rather than echo.
     */
    dock_fm_applied_t frep;
    const uint32_t HZ_1032 = 103200000u;   /* 103.2 MHz — on the 100 kHz raster */

    /* 39. Byte-exact COMMAND vector. As with test 25 this exercises the harness's own
     *     builder rather than dock.c's dispatch — it is the artifact a third-party
     *     client is implemented against, and it is published in PROTOCOL.md. Derived
     *     from a reference framer written separately from dock.c, which reproduces the
     *     two already-published F7 vectors byte-for-byte before being trusted here. */
    reset();
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x0A, 0x00,
            0x6F, 0x64, 0x12, 0xE6, 0x2F, 0x91, 0xB8, 0x66, 0x27, 0x35, 0x9A, 0x85,
            0xDC, 0xBA,
        };
        plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
        flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
        CHECK(flen == sizeof(golden), "0x0879: golden command length");
        CHECK(flen == sizeof(golden) && memcmp(frame, golden, sizeof(golden)) == 0,
              "0x0879: byte-exact command vector");
    }

    /* 40. Byte-exact REPLY vector, for the same reason and with the same provenance. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x0C, 0x00,
            0x6C, 0x64, 0x1C, 0xE6, 0x2E, 0x90, 0x0D, 0xF5, 0x07, 0x33, 0xD5, 0x41,
            0xEC, 0xFC,
            0xDC, 0xBA,
        };
        CHECK(g_caplen == sizeof(golden), "0x087A: golden reply length");
        CHECK(g_caplen == sizeof(golden) &&
              memcmp(g_cap, golden, sizeof(golden)) == 0,
              "0x087A: byte-exact reply vector");
    }

    /* 41. Short payload is refused before a single field is decoded, which is what
     *     makes an EMPTY 0x0879 a safe firmware-level probe: it cannot move the
     *     receiver, and it cannot take the speaker away from the station. */
    reset();
    {
        /* Both halves of the probe exchange are PUBLISHED in PROTOCOL.md, so both are
         * pinned here rather than left as a derivation nobody checked. */
        static const uint8_t probe[] = {
            0xAB, 0xCD, 0x04, 0x00, 0x6F, 0x64, 0x14, 0xE6, 0x8D, 0x89, 0xDC, 0xBA,
        };
        flen = build_cmd(frame, DOCK_CMD_SET_FM, params, 0);
        CHECK(flen == sizeof(probe) && memcmp(frame, probe, sizeof(probe)) == 0,
              "0x0879: byte-exact empty-payload probe vector");
    }
    feed(frame, flen);
    CHECK(g_fm_calls == 0, "0x0879: an empty payload is refused, not read past");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_SHORT,
          "0x087A: a short payload is REPORTED, so the probe gets an answer");
    CHECK(frep.state == DOCK_FM_STATE_UNKNOWN && frep.band == DOCK_FM_BAND_UNKNOWN
          && frep.freq_hz == 0u,
          "0x087A: a refusal names no state and no band — 0xFF, never 0 (0 is OFF, and band 0)");
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x0C, 0x00,
            0x6C, 0x64, 0x1C, 0xE6, 0x2F, 0x6E, 0x0D, 0x40, 0x21, 0x35, 0x2A, 0x40,
            0xEC, 0xFC,
            0xDC, 0xBA,
        };
        CHECK(g_caplen == sizeof(golden) &&
              memcmp(g_cap, golden, sizeof(golden)) == 0,
              "0x087A: byte-exact probe-refusal vector");
    }

    /* 42. Full-control is named rather than ignored, and released cleanly. */
    reset();
    flen = build_cmd(frame, DOCK_CMD_ENTER_HW, params, 0); feed(frame, flen);
    g_caplen = 0;
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_calls == 0, "0x0879: refused while the host holds full-control");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_BUSY,
          "0x087A: full-control is named, so the caller can retry after 0x0871");

    flen = build_cmd(frame, DOCK_CMD_EXIT_HW, params, 0); feed(frame, flen);
    g_caplen = 0;
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_calls == 1, "0x0879: accepted once full-control is released");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_APPLIED,
          "0x087A: applied after full-control is released");

    /* 43. THE ONE THIS OPCODE EXISTS TO GET RIGHT: a frequency off the 100 kHz raster
     *     is REFUSED, not rounded. 103.25 MHz is not "nearly 103.2" — on the broadcast
     *     band the next raster step is a whole different station, and a host that asked
     *     for one and silently got the other has no way to find out. 0x0873 truncates
     *     sub-10 Hz detail because 10 Hz of a repeater channel is nothing; this is not
     *     that, and the difference is the reason this is a separate opcode. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, 103250000u, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_calls == 0, "0x0879: an off-raster frequency never reaches the radio");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_FIELD,
          "0x087A: an off-raster frequency is refused, never rounded to the nearest station");
    CHECK(!g_fm_on, "0x0879: a refused frequency leaves the receiver where it was");

    /* 44. A band number the firmware's two-bit field would silently truncate. Band 4
     *     assigned to `uint8_t FM_Band : 2` becomes band 0 with no diagnostic — the
     *     radio lands on 87.5-108 while the host believes otherwise. Refused here, on
     *     the wire's own scale, before any binding sees it. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 4u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_calls == 0, "0x0879: a band the firmware would truncate never reaches it");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_FIELD,
          "0x087A: an out-of-range band is refused, not clamped to 0");

    /* 45. An action this firmware does not accept. Same shape as 0x0877's USB refusal:
     *     the number is refused now and can be accepted later, additively. */
    reset();
    plen = p_fm(params, 3u, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_calls == 0, "0x0879: an unknown action never reaches the radio");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_FIELD,
          "0x087A: an unknown action is named");

    /* 46. Refused while the DOCK ITSELF holds the key. This is the guard the radio's
     *     own ACTION_FM cannot provide: it tests gCurrentFunction, and Dock_ForceTx
     *     keys by writing REG_30 without ever entering FUNCTION_Select, so the
     *     firmware's own check sees an idle radio mid-over. ctx->tx_on is the only
     *     state that knows, and it lives here — which is also why this is the one new
     *     refusal a host test can prove. Turning the receiver on here would raise the
     *     speaker amp into a live mic and flip the LNA GPIOs mid-transmission. */
    reset();
    {
        static const uint16_t key_on[] = { 0x30u, 0xC1FEu };
        plen = p_write(params, key_on, 1);
        flen = build_cmd(frame, DOCK_CMD_WRITE_REGS, params, plen);
        feed(frame, flen);
    }
    CHECK(ctx.tx_on, "0x0850: the key is up, so the FM refusal below is the real case");
    g_caplen = 0;
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_calls == 0, "0x0879: refused while the dock holds the transmitter keyed");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_TX,
          "0x087A: keyed is its OWN status, not BUSY — BUSY already means full-control");

    /* 47. TUNE is not a cheaper ON. A host stepping across the band must never be able
     *     to switch the station deaf by accident, so tuning a receiver that is off is
     *     refused rather than promoted. Only the radio side can see this, so it is the
     *     HAL that reports it — and dock.c still blanks the reply. */
    reset();
    plen = p_fm(params, DOCK_FM_TUNE, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_calls == 1, "0x0879: TUNE reaches the radio — only it knows if FM is on");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_OFF,
          "0x087A: TUNE on a receiver that is off is refused, not promoted to an ON");
    CHECK(frep.state == DOCK_FM_STATE_UNKNOWN && frep.freq_hz == 0u,
          "0x087A: and that refusal still ships no state claim");

    /* 48. A frequency outside the named band's own limits. dock.c cannot check this —
     *     the limits live in the BK1080 driver's tables and a second copy here would be
     *     a drift hazard — so it is a HAL verdict, reported through the same reply.
     *     64.0 MHz is legal on band 3 and below band 0's floor of 87.5. */
    reset();
    g_fm_force_status = DOCK_FM_ERR_BAND;
    plen = p_fm(params, DOCK_FM_ON, 64000000u, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_BAND,
          "0x087A: out-of-band is reported by the radio side, which owns the limits");

    /* 49. A build without the binding still answers. An F7 firmware, or a Fusion build
     *     with ENABLE_FMRADIO off, must say ERR_NO_HAL rather than fall through to the
     *     silence that means "this firmware does not have 0x0879 at all". Those two are
     *     different facts and a host acts differently on them. */
    reset();
    dock_init(&ctx, &HAL_NO_FM);
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_caplen == FM_REPLY_FRAME_LEN, "0x087A: a missing binding still replies");
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_NO_HAL,
          "0x087A: a missing set_fm binding is reported, not silent");

    /* 50. A rejection never describes the receiver — enforced in dock.c, not left to
     *     each binding. The fake writes "off, band 0, 103.2, and you can transmit" on
     *     its refusal on purpose: every one of those is what a host wants to hear, and
     *     none of them may be published by a frame that says it refused. */
    reset();
    g_fm_force_status = DOCK_FM_ERR_BAND;
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_BAND,
          "0x087A: a refusal from the radio side is reported");
    CHECK(frep.state == DOCK_FM_STATE_UNKNOWN && frep.band == DOCK_FM_BAND_UNKNOWN
          && frep.freq_hz == 0u && frep.flags == 0,
          "0x087A: a non-zero status ships no state, band, frequency or TX claim, whatever the HAL wrote");

    /* 51. A valid frame reaches the binding exactly once, and does nothing else.
     *     NOTE, as for test 31: this stays green against a stub that skips every check.
     *     It discriminates against a dispatch that never calls the binding or calls it
     *     twice, not against one that fails to validate. Tests 41-48 are that. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 2u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_calls == 1 && g_fm_last.action == DOCK_FM_ON
          && g_fm_last.freq_hz == HZ_1032 && g_fm_last.band == 2u,
          "0x0879: a valid frame reaches the binding exactly once, with the values sent");
    CHECK(g_writes == 0 && g_reads == 0 && g_vfo_calls == 0 && g_mod_calls == 0,
          "0x0879: no register traffic, no set-VFO and no set-modulation of its own");
    CHECK(g_tx_calls == 0 && !ctx.tx_on, "0x0879: no PA activity of its own");

    /* 52. The reply REPORTS, it does not echo. The modelled receiver refuses to leave
     *     87.5 — the way a radio whose band was changed under it would — and the reply
     *     must carry where it actually is, not where it was sent. This is 0x0874's
     *     doctrine and the reason the fake models a receiver instead of echoing. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    g_fm_freq_hz = 87500000u;                     /* it drifted back, as radios do */
    g_caplen = 0;
    plen = p_fm(params, DOCK_FM_TUNE, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_APPLIED,
          "0x087A: a TUNE on a running receiver applies");
    CHECK(frep.freq_hz == HZ_1032 && frep.state == DOCK_FM_STATE_ON,
          "0x087A: the frequency is read back from the receiver, in the same Hz that were sent");

    /* 53. OFF turns it off and says so, and the receiver keeps the frequency it was on
     *     — exactly as gEeprom.FM_FrequencyPlaying does across an off. A `state` of OFF
     *     is the host's only signal that the station can hear its own channel again. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.state == DOCK_FM_STATE_ON,
          "0x087A: ON reports ON — the station is now deaf to its own channel");
    g_caplen = 0;
    plen = p_fm(params, DOCK_FM_OFF, 0u, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_APPLIED
          && frep.state == DOCK_FM_STATE_OFF,
          "0x087A: OFF reports OFF");
    CHECK(frep.freq_hz == HZ_1032,
          "0x087A: and the receiver keeps its frequency across an off, as the firmware does");

    /* 53b. OFF is never refused for a field it does not use. Turning the receiver off
     *      is how a host gives the station its ears back, so a stale frequency or an
     *      out-of-range band in a payload whose action is OFF must not block it — both
     *      of which would be refused outright on an ON. Same instinct as radio-server's
     *      ADR 0151: the direction that restores the radio always stays available. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(g_fm_on, "0x0879: the receiver is on, so the OFF below is the real case");
    g_caplen = 0;
    plen = p_fm(params, DOCK_FM_OFF, 103250000u, 7u);   /* both fields are junk */
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_APPLIED
          && frep.state == DOCK_FM_STATE_OFF,
          "0x087A: OFF is not refused for an off-raster frequency or a bad band it ignores");
    CHECK(!g_fm_on, "0x0879: and the receiver really did stop");

    /* 54. The TX_OK flag is carried and is NOT about broadcast FM. It reports whether
     *     the radio will key its own transmit path — fed by the BK4819 demodulator,
     *     which the BK1080 does not touch. A host must be able to read "playing
     *     broadcast FM" and "will transmit normally" off the same frame, because that
     *     combination is the actual state of this radio and it is the dangerous one. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.state == DOCK_FM_STATE_ON
          && (frep.flags & DOCK_FM_FLAG_TX_OK) != 0,
          "0x087A: deaf and still able to transmit is representable, because it is real");

    reset();
    g_fm_flags = 0u;                              /* the BK4819 is on AM */
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && (frep.flags & DOCK_FM_FLAG_TX_OK) == 0,
          "0x087A: and a radio that cannot key says so on this frame too");

    /* 55. The F7 and F6 goldens are unmoved. 0x0879 added a member to dock_hal_t and a
     *     case to the dispatch; neither may shift a byte of a frame already shipped and
     *     already implemented against by radio-server's frames.py. */
    reset();
    g_mod_force_flags = 0;                        /* as test 26 — AM will not key */
    flen = build_cmd(frame, DOCK_CMD_SET_MODULATION, MOD_AM, sizeof(MOD_AM));
    feed(frame, flen);
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x08, 0x00,
            0x6E, 0x64, 0x10, 0xE6, 0x2E, 0x90, 0x0C, 0x40, 0xDE, 0xCA,
            0xDC, 0xBA,
        };
        CHECK(g_caplen == sizeof(golden) &&
              memcmp(g_cap, golden, sizeof(golden)) == 0,
              "0x0878: F7's golden reply is byte-identical after F8");
    }

    /* 56. And an unknown opcode adjacent to the new one is still silent, so the
     *     firmware-level probe keeps working one level up. */
    reset();
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, 0x087Bu, params, plen);
    feed(frame, flen);
    CHECK(g_caplen == 0, "0x087B: unknown opcode beside 0x0879 is still answered with silence");

    /* =====================================================================
     * F9 — the radio refuses to transmit while deaf (0x087A flags bit 1)
     *
     * The interlock itself lives in RADIO_PrepareTX and its predicate in
     * dock_tx_interlock.h; neither is reachable from here, and the predicate has its
     * own three-way-compiled test (tests/host/test_interlock.c). What THESE cases pin
     * is the wire: that dock.c carries the second bit, blanks it with the rest, and
     * keeps it independent of bit 0 — the property a host's will_key rule depends on.
     * ===================================================================== */

    /* 57. Bit 1 is carried on an applied reply. This is an interlocked image reporting
     *     a running BK1080: the station is deaf AND the radio will now refuse its own
     *     PTT, which is the state F9 exists to create and to publish. */
    reset();
    g_fm_flags = DOCK_FM_FLAG_TX_OK | DOCK_FM_FLAG_FM_BLOCKS_TX;
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.state == DOCK_FM_STATE_ON
          && (frep.flags & DOCK_FM_FLAG_FM_BLOCKS_TX) != 0,
          "0x087A: a build whose interlock is refusing says so on the wire");

    /* 58. The two bits are INDEPENDENT, and all four combinations are real states of
     *     some image the fork publishes. This is the case that matters most to a host,
     *     because the naive read — "bit 0 says it will key" — is wrong on exactly one
     *     of them. The rule is will_key = TX_OK && !FM_BLOCKS_TX.
     *
     *     TX_OK=1 BLOCKS=0  FM demodulator, no interlock or no broadcast FM -> keys
     *     TX_OK=1 BLOCKS=1  FM demodulator, interlocked image, receiver running -> REFUSES
     *     TX_OK=0 BLOCKS=0  the radio is on AM (F7's refusal), receiver idle -> refuses
     *     TX_OK=0 BLOCKS=1  on AM *and* deaf: two independent reasons, both reported */
    {
        static const uint8_t combos[4] = {
            DOCK_FM_FLAG_TX_OK,
            DOCK_FM_FLAG_TX_OK | DOCK_FM_FLAG_FM_BLOCKS_TX,
            0u,
            DOCK_FM_FLAG_FM_BLOCKS_TX,
        };
        for (unsigned i = 0; i < 4; i++) {
            reset();
            g_fm_flags = combos[i];
            plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
            flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
            feed(frame, flen);
            CHECK(last_fm_reply(&frep) && frep.flags == combos[i],
                  "0x087A: every TX_OK/FM_BLOCKS_TX combination survives the wire intact");
        }
    }

    /* 59. Bit 1 blanks with everything else on a refusal — and note WHICH WAY that
     *     fails. flags blanks to 0, so a refused frame reports NOT-blocked. That is
     *     deliberate and it matches the sentinel rule above rather than fighting it: a
     *     refusal measured nothing, and an unmeasured field must never lock a
     *     transmitter. Reporting "blocked" here would let a lost frame stop a station.
     *     (Same rule radio-server ADR 0158 pinned in both directions for tx_ok.) */
    reset();
    g_fm_force_status = DOCK_FM_ERR_BAND;         /* the HAL sets BOTH bits; see hal_set_fm */
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_ERR_BAND && frep.flags == 0u,
          "0x087A: a refusal blanks BOTH flag bits, and blanks toward 'not blocked'");

    /* 60. Byte-exact F9 reply vector, flags = 0x03. Derived from the same independent
     *     reference framer as F8's, which was re-run this cycle and reproduced BOTH
     *     published 0x087A vectors and the published 0x0879 command byte-for-byte
     *     before this new output was trusted. It differs from F8's published ON vector
     *     at exactly one byte — offset 15, the obfuscated flags byte. */
    reset();
    g_fm_flags = DOCK_FM_FLAG_TX_OK | DOCK_FM_FLAG_FM_BLOCKS_TX;
    plen = p_fm(params, DOCK_FM_ON, HZ_1032, 0u);
    flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
    feed(frame, flen);
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x0C, 0x00,
            0x6C, 0x64, 0x1C, 0xE6, 0x2E, 0x90, 0x0D, 0xF5, 0x07, 0x33, 0xD5, 0x43,
            0xEC, 0xFC,
            0xDC, 0xBA,
        };
        CHECK(g_caplen == sizeof(golden), "0x087A: F9 golden reply length");
        CHECK(g_caplen == sizeof(golden) &&
              memcmp(g_cap, golden, sizeof(golden)) == 0,
              "0x087A: byte-exact F9 reply vector (flags = TX_OK | FM_BLOCKS_TX)");
    }

    /* 61. The 0x0879 OFF command, byte-exact — and this one is overdue rather than new.
     *     PROTOCOL.md published four 0x0879/0x087A vectors at F8 and NONE of them was
     *     the frame radio-server actually sends, which makes the spec misleading rather
     *     than merely incomplete: an implementer working from it has no pinned example
     *     of the only direction that gives a deaf station its ears back. Derived from
     *     the reference framer AND cross-checked against radio-server's own
     *     ClearBroadcastFm().to_frame(), which agree byte-for-byte.
     *     (radio-server ADR 0158 R6.) */
    reset();
    {
        static const uint8_t golden[] = {
            0xAB, 0xCD, 0x0A, 0x00,
            0x6F, 0x64, 0x12, 0xE6, 0x2E, 0x91, 0x0D, 0x40, 0x21, 0x35, 0x6E, 0x13,
            0xDC, 0xBA,
        };
        plen = p_fm(params, DOCK_FM_OFF, 0u, 0u);
        flen = build_cmd(frame, DOCK_CMD_SET_FM, params, plen);
        CHECK(flen == sizeof(golden), "0x0879: OFF golden command length");
        CHECK(flen == sizeof(golden) && memcmp(frame, golden, sizeof(golden)) == 0,
              "0x0879: byte-exact OFF command vector — the frame the host actually sends");
    }
    feed(frame, flen);
    CHECK(last_fm_reply(&frep) && frep.status == DOCK_FM_APPLIED
          && frep.state == DOCK_FM_STATE_OFF,
          "0x087A: and that frame is the one that turns the receiver off");

    /* ---- report ---- */
    printf("dock host tests: %d checks, %d failures\n", g_checks, g_fail);
    if (g_fail) { printf("RESULT: FAIL\n"); return 1; }
    printf("RESULT: PASS\n");
    return 0;
}
