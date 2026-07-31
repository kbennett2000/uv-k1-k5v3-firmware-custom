# Host-side tests for the dock protocol core (App/app/dock.c).
# Pure C, no firmware/hardware dependencies — builds and runs on the host.
#
#   make -C tests/host run

CC      ?= cc
CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -O2
INCLUDES = -I../../App

SRC = test_dock.c ../../App/app/dock.c

test_dock: $(SRC) ../../App/app/dock.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SRC)

# F9 — the broadcast-FM transmit interlock predicate, compiled ONCE PER BUILD SHAPE the
# fork publishes and run three times. The predicate is build-conditional, so the 0x087A
# flag that reports it must report what THIS IMAGE does; a constant would be a lie on
# some published build. Preprocessor state is per-translation-unit, so one binary cannot
# cover two shapes — hence three.
INTERLOCK_SRC = test_interlock.c
INTERLOCK_H   = ../../App/app/dock_tx_interlock.h
INTERLOCK_BINS = test_interlock_fusion test_interlock_broadcast test_interlock_nofm

test_interlock_fusion: $(INTERLOCK_SRC) $(INTERLOCK_H)
	$(CC) $(CFLAGS) $(INCLUDES) -DENABLE_FMRADIO -DENABLE_DOCK_FM_TX_INTERLOCK -o $@ $(INTERLOCK_SRC)

test_interlock_broadcast: $(INTERLOCK_SRC) $(INTERLOCK_H)
	$(CC) $(CFLAGS) $(INCLUDES) -DENABLE_FMRADIO -o $@ $(INTERLOCK_SRC)

test_interlock_nofm: $(INTERLOCK_SRC) $(INTERLOCK_H)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(INTERLOCK_SRC)

# Every binary runs even when an earlier one fails, and the exit status is the OR. An
# `|| exit 1` here would stop at the first failure and hide the rest — which it did, on a
# mutation run during this cycle: a bad predicate failed the Fusion build and the other
# two compilations were never executed at all, so the run under-reported its own evidence.
run: test_dock $(INTERLOCK_BINS) check-fm-restore check-fm-interlock
	@rc=0; ./test_dock || rc=1; \
	 for b in $(INTERLOCK_BINS); do ./$$b || rc=1; done; \
	 exit $$rc

# A grep, not a test, and it is here because the real test is not expressible.
#
# RADIO_SetupRegisters() opens with AUDIO_AudioPathOff() and gEnableSpeaker = false and
# does not clear gFmRadioMode, so any dock glue path ending in it leaves a running
# BK1080 playing to a dead amplifier. Dock_RestoreFmAudio() undoes that and must follow
# EVERY such call in uart.c's dock block.
#
# uart.c is not host-compiled — it includes firmware headers, which is the whole point
# of keeping dock.c pure (AGENTS.md guardrail 4) — so no check in test_dock.c can see
# these functions at all. This counts call sites instead. It cannot tell you the restore
# WORKS; it can only tell you someone added a fourth RADIO_SetupRegisters and forgot it.
UART_C = ../../App/app/uart.c

check-fm-restore:
	@setups=$$(grep -c '^[[:space:]]*RADIO_SetupRegisters(' $(UART_C)); \
	 restores=$$(grep -c '^[[:space:]]*Dock_RestoreFmAudio();' $(UART_C)); \
	 if [ "$$setups" != "$$restores" ]; then \
	   echo "FAIL: uart.c has $$setups RADIO_SetupRegisters call(s) in the dock block"; \
	   echo "      but $$restores Dock_RestoreFmAudio() call(s)."; \
	   echo "      Every RADIO_SetupRegisters in dock glue must be followed by one, or a"; \
	   echo "      running broadcast receiver is left muted with no timer to recover it."; \
	   exit 1; \
	 fi; \
	 echo "dock glue: $$setups RADIO_SetupRegisters, $$restores Dock_RestoreFmAudio — paired"

# A second grep, and for the same reason as the one above: the thing that needs checking
# is not host-compilable.
#
# F9's interlock has two halves that MUST read the same predicate — the gate that refuses
# the key (RADIO_PrepareTX, radio.c) and the 0x087A flag that reports the refusal to a
# host (Dock_SetFm, uart.c). If they ever stop agreeing, a host is told the radio will key
# while the radio has already decided otherwise. test_interlock.c proves the predicate
# itself behaves under every build shape; nothing host-side can prove it is CALLED, because
# neither radio.c nor uart.c compiles off-target.
#
# So this counts call sites. It cannot tell you the gate WORKS; it can tell you someone
# rewrote one half in terms of gFmRadioMode directly and left the other reading the
# predicate — which is exactly how F7's Dock_ModulationCanTx became a hand copy of the AM
# gate rather than a shared rule.
#
# Comment lines are filtered out before counting: both call sites are documented in
# comments that name the predicate, and a gate that counted those would pass for the
# wrong reason (or fail whenever someone edits a comment).
RADIO_C = ../../App/radio.c
NOT_A_COMMENT = grep -v '^[[:space:]]*\(//\|\*\|/\*\)'

check-fm-interlock:
	@gate=$$(grep 'Dock_BroadcastFmBlocksTx()' $(RADIO_C) | $(NOT_A_COMMENT) | wc -l); \
	 flag=$$(grep 'Dock_BroadcastFmBlocksTx()' $(UART_C) | $(NOT_A_COMMENT) | wc -l); \
	 def=$$(grep -c 'static inline bool Dock_BroadcastFmBlocksTx' $(INTERLOCK_H)); \
	 if [ "$$gate" != "1" ] || [ "$$flag" != "1" ] || [ "$$def" != "1" ]; then \
	   echo "FAIL: the broadcast-FM interlock predicate is not shared as intended."; \
	   echo "      radio.c gate call sites: $$gate (want 1)"; \
	   echo "      uart.c  flag call sites: $$flag (want 1)"; \
	   echo "      dock_tx_interlock.h definitions: $$def (want 1)"; \
	   echo "      The refusal and the flag that reports it must read ONE predicate, or a"; \
	   echo "      host can be told the radio will key after the radio has refused."; \
	   exit 1; \
	 fi; \
	 echo "fm interlock: 1 gate (radio.c), 1 flag (uart.c), 1 definition — shared"

clean:
	rm -f test_dock $(INTERLOCK_BINS)

.PHONY: run clean check-fm-restore check-fm-interlock
