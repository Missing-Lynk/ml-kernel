# Statically-linked bring-up test tools for the open kernel's drivers, cross-compiled for the
# aarch64 goggle. Each tool exercises a driver through its userspace ABI (sysfs / dev nodes).
# Most tools are a single self-contained .c; the display tools additionally link the shared
# drm_util.c helper (DRM/KMS plumbing + cache maintenance, not driver code).
#
# Binaries go into build/ (git-ignored), so the source dir stays clean.
#
#   make                 # build every tool into build/
#   make led_test        # build one (-> build/led_test)
#   make clean           # remove build/
#
# Override the toolchain if yours is named differently:
#   make CROSS_COMPILE=aarch64-none-linux-gnu-

CROSS_COMPILE ?= aarch64-linux-gnu-
CC            := $(CROSS_COMPILE)gcc
CFLAGS        := -static -O2 -Wall -Wextra
BUILD         := build

# Shared library (not a standalone tool) and the tools that link it.
LIB     := drm_util.c
DISPLAY := display_test overlay_test display_demo display_bounce display_orient i420_test yuv_demo prime_test

SRCS  := $(wildcard *.c)
TOOLS := $(filter-out $(LIB:.c=),$(SRCS:.c=))
BINS  := $(addprefix $(BUILD)/,$(TOOLS))

all: $(BINS)

# Display tools: link the shared helper. This static pattern rule takes precedence over the
# generic one below for these targets.
$(addprefix $(BUILD)/,$(DISPLAY)): $(BUILD)/%: %.c $(LIB) drm_util.h | $(BUILD)
	$(CC) $(CFLAGS) $< $(LIB) -o $@

# Everything else: a single self-contained source.
$(BUILD)/vblank_rate: LDLIBS += -lm
$(BUILD)/%: %.c | $(BUILD)
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD):
	mkdir -p $(BUILD)

# Convenience: `make led_test` builds build/led_test (the bare tool name is a phony alias).
$(TOOLS): %: $(BUILD)/%

clean:
	rm -rf $(BUILD)

.PHONY: all clean $(TOOLS)
