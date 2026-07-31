/* Host-side tests for App/app/dock_tx_interlock.h — the ONE predicate that decides
 * whether a running BK1080 blocks transmit.
 *
 * WHY THIS FILE EXISTS SEPARATELY FROM test_dock.c
 *
 * The interlock is build-conditional, so the 0x087A flag that reports it must report
 * what THIS IMAGE actually does rather than a constant. A flag that lied on any build
 * the fork publishes would be worse than no flag at all: a host would refuse to key a
 * radio that keys fine, or trust a radio that is about to transmit deaf.
 *
 * Guaranteeing that is a matter of having exactly ONE place where the build condition
 * lives — the predicate — and then proving that place behaves correctly under EVERY
 * preprocessor shape the fork ships. That is what this file does: the Makefile compiles
 * it three times, once per shape, and runs all three.
 *
 * It is a separate binary because preprocessor state is per-translation-unit; one main()
 * cannot be compiled two ways at once. Note also what this does NOT prove: that
 * RADIO_PrepareTX actually calls the predicate. radio.c is not host-compilable, so that
 * seam is covered by the `check-fm-interlock` grep in the Makefile, which is a grep and
 * is labelled as one.
 *
 * Build & run:  make -C tests/host run
 */

#include "app/dock_tx_interlock.h"

#include <stdbool.h>
#include <stdio.h>

/* ---- tiny test framework, same shape as test_dock.c ---- */
static int g_checks = 0, g_fail = 0;
#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            printf("  FAIL: %s (line %d)\n", (msg), __LINE__);              \
            g_fail++;                                                       \
        }                                                                   \
    } while (0)

#ifdef ENABLE_FMRADIO
/* The firmware's own flag (App/app/fm.c:36). This test owns the storage, which is the
 * whole reason the predicate lives in a header with no includes: the one firmware symbol
 * it needs is a plain bool, so a host can supply it and exercise the real code rather
 * than a copy of it. */
bool gFmRadioMode;
#endif

int main(void)
{
#if defined(ENABLE_FMRADIO) && defined(ENABLE_DOCK_FM_TX_INTERLOCK)
    const char *build = "Fusion: ENABLE_FMRADIO + ENABLE_DOCK_FM_TX_INTERLOCK";

    gFmRadioMode = false;
    CHECK(!Dock_BroadcastFmBlocksTx(),
          "interlocked build, receiver idle: nothing is blocked");

    gFmRadioMode = true;
    CHECK(Dock_BroadcastFmBlocksTx(),
          "interlocked build, receiver running: transmit is blocked");

    /* The predicate is the live state, not a latch: giving the radio its ears back must
     * give the transmitter back in the same instant, with no restart and no host. This
     * is exactly what the host-side gate (radio-server ADR 0158) cannot do. */
    gFmRadioMode = false;
    CHECK(!Dock_BroadcastFmBlocksTx(),
          "interlocked build: clearing broadcast FM un-blocks immediately — it does not latch");

#elif defined(ENABLE_FMRADIO)
    const char *build = "Broadcast/Basic/Game: ENABLE_FMRADIO, no interlock";

    gFmRadioMode = false;
    CHECK(!Dock_BroadcastFmBlocksTx(),
          "no-interlock build, receiver idle: nothing is blocked");

    /* THE LOAD-BEARING ONE. This is a stock F4HWN edition with no dock and no host. It
     * transmits while playing broadcast FM, exactly as upstream does, and the flag on
     * 0x087A must therefore report NOT-blocked. Reporting blocked here would be a lie
     * about a radio that is about to key, which is the dangerous direction. */
    gFmRadioMode = true;
    CHECK(!Dock_BroadcastFmBlocksTx(),
          "no-interlock build, receiver running: still not blocked, because this image really does key");

#else
    const char *build = "Custom/Bandscope/RescueOps: no ENABLE_FMRADIO";

    /* No BK1080 driver is linked into these images at all, so there is no second
     * receiver to be deaf to. The predicate must compile and answer false without
     * referring to gFmRadioMode, which does not exist here. */
    CHECK(!Dock_BroadcastFmBlocksTx(),
          "no-BK1080 build: there is no second receiver, so nothing can block on one");
#endif

    printf("interlock predicate [%s]: %d checks, %d failures\n", build, g_checks, g_fail);
    if (g_fail) { printf("RESULT: FAIL\n"); return 1; }
    printf("RESULT: PASS\n");
    return 0;
}
