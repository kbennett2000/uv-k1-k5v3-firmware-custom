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

run: test_dock check-fm-restore
	./test_dock

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

clean:
	rm -f test_dock

.PHONY: run clean check-fm-restore
