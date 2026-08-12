# =============================================================================
#  LidarAGL — host unit-test build (NO hardware required)
# -----------------------------------------------------------------------------
#  Compiles the PURE-logic firmware modules together with the test mains using
#  the host's gcc. Define UNIT_TEST so config.h skips its hardware-only region.
#
#  Usage:
#      make            # build + run every test
#      make clean
#
#  If you don't have 'make', the equivalent one-liners are in README.md and the
#  CI snippet below; e.g.:
#      gcc -I../main -DUNIT_TEST ../main/robust.c ../main/sensor_profile.c \
#          test_robust.c -lm -o run_robust && ./run_robust
# =============================================================================

CC      ?= gcc
CFLAGS  := -I../main -DUNIT_TEST -std=gnu11 -Wall -Wextra -O2
LDLIBS  := -lm
SRC     := ../main

# Pure modules under test (must never include esp_*/driver/freertos).
PURE := $(SRC)/robust.c $(SRC)/sensor_profile.c $(SRC)/state_machine.c \
        $(SRC)/audio_math.c $(SRC)/lwnx.c $(SRC)/range_filter.c \
        $(SRC)/boot_buffer.c

TESTS := run_robust run_state_machine run_audio_math run_lwnx \
         run_range_filter run_boot_buffer run_flight

.PHONY: all clean
all: $(TESTS)
	@echo "---- running tests ----"
	@for t in $(TESTS); do echo "## $$t"; ./$$t || exit 1; done
	@echo "---- all tests passed ----"

run_robust:        test_robust.c        $(PURE)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

run_state_machine: test_state_machine.c $(PURE)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

run_audio_math:    test_audio_math.c    $(PURE)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

run_lwnx:          test_lwnx.c          $(PURE)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

run_range_filter:  test_range_filter.c  $(PURE)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

run_boot_buffer:   test_boot_buffer.c   $(PURE)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

run_flight:        test_flight.c        $(PURE)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -f $(TESTS) $(addsuffix .exe,$(TESTS))
