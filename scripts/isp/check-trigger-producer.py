#!/usr/bin/env python3
"""Check the AEC trigger-scalar producer against vendor captures and device sweeps.

Gamma, DRC, cm and cm2 key on a scalar running 0 to 550, not on the linear gain the
noise and demosaic ladders take. The scalar is the exposure-table index the current
luma would need in order to reach its target:

    ev(i)  = table[i].gain_q8 * table[i].line_count
    ratio  = ev(exp_index) / ev(exp_index - 1)
    scalar = floor(exp_index + log(target / current_luma) / log(ratio))

The ratio is taken backwards so it stays defined at the table's last entry, which is
where a lens-covered capture sits.

--session pairs, per capture, the scalar interval inverted from the cm/cm2 bank
against the AE state in the process heap at that moment. The heaps are proprietary
and are not in the repository.

--sweep checks a device sweep of the parameter against the producer, taking the
exposure table from the tuning blob.

    kernel/scripts/isp/check-trigger-producer.py --session out/au-vendor-session
"""

import argparse
import math
import pathlib
import struct
import sys

from blob_layout import Layout

_LAY = Layout.load()

# The AE state block, anchored off the generated exposure table rather than a
# fixed address: the heap base moves between capture sessions.
HEAP_BASE = 0x50C000
STATE_VA = 0xED07A0
STATE_OFF = STATE_VA - HEAP_BASE
STATE_TABLE = STATE_OFF + 0x68
STATE_AE = STATE_OFF + 0x4000

OFF_EXP_INDEX = STATE_AE + 4300
OFF_LUX_INDEX = STATE_AE + 4312
OFF_LUMA_TARGET = STATE_AE + 4512
OFF_CURRENT_LUMA = STATE_AE + 4596

SIG_LEN = 512

# The shipped exposure table, keyed off the layout so the offset lives in one place.
_TABLE = _LAY["ae_exposure_table"]
BLOB_TABLE_OFF = _TABLE.offset
BLOB_TABLE_COUNT = _TABLE.count
BLOB_TABLE_STRIDE = _TABLE.stride

# Scalar intervals inverted from each capture's cm/cm2 bank by
# check-cm-ladder.py and check-cm2-ladder.py, quoted here as the measurement.
PAIRS = (
    ("bright", "heap-bright.bin", None),
    ("live", "heap-live.bin", (290.0, 330.0)),
    ("ambient2", "heap-ambient2.bin", (307.5, 360.0)),
    ("covered", "heap-covered.bin", (397.5, 452.25)),
)


def anchor(heap: bytes, signature: bytes) -> int:
    """Byte shift of this heap's AE state against the reference layout."""
    hits = []
    i = heap.find(signature)

    while i >= 0:
        hits.append(i)
        i = heap.find(signature, i + 1)

    if not hits:
        sys.exit("exposure table signature not found; heap layout changed")

    return min(hits, key=lambda h: abs(h - STATE_TABLE)) - STATE_TABLE


def exposure_table(heap: bytes, shift: int) -> list[tuple[int, int]]:
    """The generated exposure table as {gain Q8, line count}, to its terminator."""
    out: list[tuple[int, int]] = []

    for i in range(4096):
        gain, lines = struct.unpack_from("<II", heap, STATE_TABLE + shift + i * 8)

        if gain == 0 or lines == 0:
            break

        out.append((gain, lines))

    return out


def predict(table: list[tuple[int, int]], exp_index: int, target: float,
            luma: float) -> float:
    """The trigger scalar, before truncation."""
    i = max(1, min(exp_index, len(table) - 1))
    here = table[i][0] * table[i][1]
    prev = table[i - 1][0] * table[i - 1][1]

    if prev <= 0 or here <= prev or luma <= 0.0 or target <= 0.0:
        return float(exp_index)

    return exp_index + math.log(target / luma) / math.log(here / prev)


def blob_table(blob: bytes) -> list[tuple[int, int]]:
    """The exposure table as {gain Q8, line count}, straight from the tuning file."""
    out: list[tuple[int, int]] = []

    for i in range(BLOB_TABLE_COUNT):
        gain, lines = struct.unpack_from("<II", blob,
                                         BLOB_TABLE_OFF + i * BLOB_TABLE_STRIDE)

        out.append((gain, lines))

    return out


def check_sweep(sweep: pathlib.Path, tuning: pathlib.Path) -> int:
    """Verify a device sweep: every sampled scalar against the producer."""
    table = blob_table(tuning.read_bytes())
    rows = []

    with sweep.open() as f:
        for line in f:
            parts = line.strip().split(",")

            if len(parts) != 6 or parts[0] == "uptime_s":
                continue

            try:
                rows.append((float(parts[0]), int(parts[1]), int(parts[2]),
                             float(parts[3]), float(parts[4])))
            except ValueError:
                continue

    if not rows:
        sys.exit(f"{sweep}: no usable samples")

    ceiling = len(table) - 1
    saturated = [r for r in rows if r[2] >= ceiling]
    deltas: dict[int, int] = {}

    for _up, scalar, exp_index, luma, target in rows:
        d = scalar - math.floor(predict(table, exp_index, target, luma))
        deltas[d] = deltas.get(d, 0) + 1

    # A sweep samples the parameter and the decision log separately, so a row can
    # pair a scalar with a neighbouring frame's luma. The scalar is truncated, so
    # that skew moves it by one count and no more. Anything past one count is the
    # producer disagreeing, not the sampler.
    bad = sum(n for d, n in deltas.items() if abs(d) > 1)
    span = (min(r[1] for r in rows), max(r[1] for r in rows))

    print(f"{len(rows)} samples, scalar {span[0]}..{span[1]}, "
          f"index {min(r[2] for r in rows)}..{max(r[2] for r in rows)}")
    print("delta (measured - predicted):")

    for d in sorted(deltas):
        print(f"  {d:+3d}  {deltas[d]:4d}")

    print(f"{deltas.get(0, 0)}/{len(rows)} exact, "
          f"{sum(n for d, n in deltas.items() if abs(d) <= 1)}/{len(rows)} within "
          f"the one-count sampling skew, {bad} beyond it")

    if not saturated:
        print("no sample reached the table ceiling, so this sweep does not "
              "separate the producer from exp_index; cover the lens for longer")
    else:
        best = max(saturated, key=lambda r: r[1])
        print(f"saturated at index {ceiling}: {len(saturated)} samples, scalar "
              f"reached {best[1]}, {best[1] - ceiling} counts past the ceiling; "
              f"exp_index as the producer would be wrong by that margin")

    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", type=pathlib.Path,
                    help="the au-vendor-session capture directory")
    ap.add_argument("--sweep", type=pathlib.Path,
                    help="a device sweep CSV from glue/camera/au-tone-test.sh")
    ap.add_argument("--tuning", type=pathlib.Path,
                    help="the sensor tuning blob, required with --sweep")
    args = ap.parse_args()

    if args.sweep:
        if not args.tuning:
            sys.exit("--sweep needs --tuning")

        return check_sweep(args.sweep, args.tuning)

    if not args.session:
        sys.exit("one of --session or --sweep is required")

    ref_path = args.session / "heap-live.bin"

    if not ref_path.exists():
        sys.exit(f"{ref_path} not found; the heaps are not in the repository")

    ref = ref_path.read_bytes()
    signature = ref[STATE_TABLE:STATE_TABLE + SIG_LEN]

    failures: list[str] = []
    rows: list[tuple[str, int, float, float, float, int]] = []

    for label, name, interval in PAIRS:
        path = args.session / name

        if not path.exists():
            print(f"{label}: {name} absent, skipped")
            continue

        heap = path.read_bytes()
        shift = anchor(heap, signature)
        table = exposure_table(heap, shift)

        exp_index = struct.unpack_from("<I", heap, OFF_EXP_INDEX + shift)[0]
        lux = struct.unpack_from("<f", heap, OFF_LUX_INDEX + shift)[0]
        luma = struct.unpack_from("<f", heap, OFF_CURRENT_LUMA + shift)[0]
        target = float(struct.unpack_from("<I", heap, OFF_LUMA_TARGET + shift)[0])

        raw = predict(table, exp_index, target, luma)
        got = math.floor(raw)
        rows.append((label, exp_index, luma, lux, raw, got))

        if got != int(lux):
            failures.append(f"{label}: producer gives {got}, vendor holds "
                            f"{int(lux)} (raw {raw:.3f})")

        # The register-derived interval is the independent measurement: it comes
        # from the colour bank, never from the heap.
        if interval is not None:
            lo, hi = interval

            if not lo <= lux <= hi:
                failures.append(f"{label}: vendor scalar {lux:.0f} outside the "
                                f"interval [{lo}, {hi}] its own cm/cm2 bank implies")

            if lo <= exp_index <= hi:
                print(f"  note: exp_index {exp_index} also falls inside "
                      f"{label}'s interval, so this pair does not separate them")

    print(f"\n{'capture':10} {'exp_index':>9} {'luma':>8} {'vendor':>7} "
          f"{'raw':>9} {'floor':>6}")

    for label, exp_index, luma, lux, raw, got in rows:
        print(f"{label:10} {exp_index:9d} {luma:8.3f} {lux:7.0f} {raw:9.3f} "
              f"{got:6d}")

    for line in failures:
        print(line)

    if failures:
        return 1

    print(f"\nthe shipped producer reproduces the vendor trigger scalar on all "
          f"{len(rows)} paired captures")

    return 0


if __name__ == "__main__":
    sys.exit(main())
