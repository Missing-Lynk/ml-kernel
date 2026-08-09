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
# -MMD -MP emits a build/<tool>.d listing every header the tool actually included, which is
# what makes a header edit rebuild the binary. Without it a changed uapi header (several tools
# include ../modules/*.h) leaves a stale binary compiled against the old struct layout - the
# exact ABI mismatch these tools exist to detect.
CFLAGS        := -static -O2 -Wall -Wextra -MMD -MP
BUILD         := build

# Shared library (not a standalone tool) and the tools that link it.
LIB     := drm_util.c
DISPLAY := display_test overlay_test display_demo display_bounce display_orient i420_test yuv_demo prime_test

SRCS  := $(wildcard *.c)
TOOLS := $(filter-out $(LIB:.c=),$(SRCS:.c=))
BINS  := $(addprefix $(BUILD)/,$(TOOLS))
DEPS  := $(BINS:=.d)

all: $(BINS)

# Display tools: link the shared helper. This static pattern rule takes precedence over the
# generic one below for these targets.
#
# Both local headers are listed explicitly because the generated .d does not cover these
# targets: two sources in one gcc call means the second translation unit's dep file overwrites
# the first, so build/<tool>.d ends up describing drm_util.c alone. The cost of naming them is
# that any display tool rebuilds when either header changes, which is the safe direction.
$(addprefix $(BUILD)/,$(DISPLAY)): $(BUILD)/%: %.c $(LIB) drm_util.h drm_glyphs.h | $(BUILD)
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

# After the rules, so a missing .d on the first build is not an error (-include).
-include $(DEPS)

.PHONY: all clean $(TOOLS)
