# Makefile - developer entry points for the open kernel tree.
#
# Lint every in-repo C source against kernel style, and drive the build scripts.
# `make` (no target) prints this help. The kernel Image is built by scripts/build.sh
# (pinned, hermetic, reproducible); these targets just wrap it.

LINT    := scripts/lint.sh
BUILD   := scripts/build.sh
MODULES := modules/build.sh

# Devices -> board names. Each devices/<name>/ holds that board's DTS + its config-fragment
# list, so config (and thus the Image) is per-board now; a board target builds that device's
# Image + DTB (BOARD selects the device dir).
BOARD_NAMES := $(notdir $(wildcard devices/*))

.PHONY: help lint build verify modules boards $(addprefix board-,$(BOARD_NAMES))

help:
	@echo "developer targets:"
	@echo "  make lint             checkpatch every in-repo kernel C source"
	@echo "  make build            reproducible Image + DTB (default betafpv-vr04-goggle; BOARD=<name> to pick)"
	@echo "  make verify           build twice, confirm the Image is bit-reproducible"
	@echo "  make modules          out-of-tree + =m modules (modules/build.sh)"
	@echo "  make boards           list devices (devices/<name>/) and their model names"
	@echo "  make board-<name>     build just that device (Image + DTB; names from 'make boards')"
	@echo ""
	@echo "  STRICT=1 make lint    add checkpatch --strict;  FAST=1 make build  incremental build"

lint:
	@$(LINT)

build:
	@$(BUILD)

verify:
	@$(BUILD) verify

modules:
	@$(MODULES)

boards:
	@echo "Available devices (name -> model):"
	@for d in devices/*/; do \
	  n=$$(basename $$d); \
	  m=$$(sed -n 's/.*model *= *"\(.*\)".*/\1/p' $$d*.dts | head -1); \
	  printf "  %-24s %s\n" "$$n" "$$m"; \
	done

# One target per device: build that device's Image + DTB (BOARD selects devices/<name>/).
# Static pattern rule (explicit target list) so it composes with .PHONY; unknown names get
# make's own "No rule to make target" - run `make boards` for the valid names.
$(addprefix board-,$(BOARD_NAMES)): board-%:
	@BOARD=$* $(BUILD)
