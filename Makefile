# Makefile - developer entry points for the open kernel tree.
#
# Lint every in-repo C source against kernel style, and drive the build scripts.
# `make` (no target) prints this help. The kernel Image is built by scripts/build.sh
# (pinned, hermetic, reproducible); these targets just wrap it.

LINT    := scripts/lint.sh
BUILD   := scripts/build.sh
MODULES := modules/build.sh

# Board DTS files -> board names. The Image is board-neutral; only the DTB differs per
# board, so a board target = the shared Image plus that board's DTB.
BOARD_DTS   := $(wildcard dts/*.dts)
BOARD_NAMES := $(patsubst dts/%.dts,%,$(BOARD_DTS))

.PHONY: help lint build verify modules boards $(addprefix board-,$(BOARD_NAMES))

help:
	@echo "developer targets:"
	@echo "  make lint             checkpatch every in-repo kernel C source"
	@echo "  make build            reproducible kernel Image + all board DTBs (scripts/build.sh)"
	@echo "  make verify           build twice, confirm the Image is bit-reproducible"
	@echo "  make modules          out-of-tree + =m modules (modules/build.sh)"
	@echo "  make boards           list board DTS files and their model names"
	@echo "  make board-<name>     Image + just one board's DTB (names from 'make boards')"
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
	@echo "Available boards (dts basename -> model):"
	@for d in $(BOARD_DTS); do \
	  n=$$(basename $$d .dts); \
	  m=$$(sed -n 's/.*model *= *"\(.*\)".*/\1/p' $$d | head -1); \
	  printf "  %-18s %s\n" "$$n" "$$m"; \
	done

# One target per board: build the shared Image plus only that board's DTB. Static pattern
# rule (explicit target list) so it composes with .PHONY; unknown names get make's own
# "No rule to make target" - run `make boards` for the valid names.
$(addprefix board-,$(BOARD_NAMES)): board-%:
	@BOARDS=$* $(BUILD)
