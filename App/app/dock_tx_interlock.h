/* Copyright 2026 kbennett2000
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef APP_DOCK_TX_INTERLOCK_H
#define APP_DOCK_TX_INTERLOCK_H

#include <stdbool.h>

/* F9 — does a running broadcast-FM receiver block transmit on THIS BUILD?
 *
 * ONE PREDICATE, THREE CONSUMERS, AND THAT IS THE WHOLE POINT OF THE FILE.
 *
 *   1. RADIO_PrepareTX (App/radio.c) — the gate that actually refuses the key.
 *   2. Dock_SetFm      (App/app/uart.c) — the 0x087A flag that REPORTS the refusal.
 *   3. tests/host/test_interlock.c — compiled three times, once per build shape.
 *
 * F7 reported its TX_OK flag through Dock_ModulationCanTx (uart.c), which is a hand
 * copy of the gate in radio.c. Two statements of one rule, and nothing makes them
 * agree. That was accepted once. Doing it again would mean a host could be told the
 * radio will key while the radio has already decided otherwise — so the rule is stated
 * once, here, and both the enforcement and the report read it.
 *
 * WHY THE CONDITION IS IN THIS FUNCTION RATHER THAN AT THE CALL SITES.
 *
 * The interlock is build-conditional (see below), so "will this radio key?" has a
 * different answer in two images of the same firmware. A host cannot see a build flag
 * from the far end of a serial cable — that is the same reason DOCK_MOD_FLAG_TX_OK
 * exists at all (dock.h, ADR 0149/0150). Because the #if lives here and nowhere else,
 * an image without the interlock reports "not blocked" automatically and truthfully:
 * it really will key while playing broadcast FM, exactly as upstream F4HWN does. A
 * flag that lied on any build this fork publishes would be worse than no flag.
 *
 * WHY THIS HEADER HAS NO FIRMWARE INCLUDES.
 *
 * The single symbol it needs is a plain bool, so it is declared rather than included:
 * that keeps the header free of the firmware tree (the spirit of AGENTS.md guardrail 4,
 * which keeps dock.c host-testable) and it is what lets tests/host compile this exact
 * code under every preprocessor shape instead of testing a copy of it. <stdbool.h> is
 * a freestanding C header, not a firmware one.
 *
 * WHAT IT DELIBERATELY IS NOT.
 *
 * Not a latch. It reads live state, so clearing broadcast FM at the front panel gives
 * the transmitter back in the same instant — no restart, no host, no round trip. The
 * host-side gate (radio-server ADR 0158) cannot do that and says so.
 *
 * Not a guard on the DOCK's own keying. A host that keys by writing REG_30 over 0x0850
 * never enters RADIO_PrepareTX (Dock_ForceTx, uart.c) and is NOT covered. That is
 * recorded rather than fixed: 0x0850 sends no reply, so a refusal there would be a host
 * that keys, radiates nothing, and is told nothing.
 */

#ifdef ENABLE_FMRADIO
/* App/app/fm.c:36, declared in App/app/fm.h:33. Redeclared rather than included so this
 * header stays include-free; the declaration is identical and the compiler checks it
 * against fm.h in every translation unit that has both. */
extern bool gFmRadioMode;
#endif

static inline bool Dock_BroadcastFmBlocksTx(void)
{
#if defined(ENABLE_FMRADIO) && defined(ENABLE_DOCK_FM_TX_INTERLOCK)
    /* gFmRadioMode is the firmware's ONLY authority on whether the BK1080 is running.
     * There is no chip read-back to prefer: gIsInitBK1080 is file-static to bk1080.c,
     * is never cleared, and means "the register table was loaded once since boot". */
    return gFmRadioMode;
#else
    return false;
#endif
}

#endif /* APP_DOCK_TX_INTERLOCK_H */
