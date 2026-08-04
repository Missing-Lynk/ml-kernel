"""
Vendor-image helpers shared by the ISP generator and checker scripts.

Covers the two structures every script that reads libmpp_service.so needs: the
VMA-to-file mapping, and the ISP-init template array. Nothing here parses an
MMIO trace.
"""

import struct
import sys

# The second LOAD segment maps file 0x4004d0 at 0x4104d0. Every VMA this tree
# quotes is in that segment, so VMA - 0x10000 is its file offset.
VMA_TO_FILE = 0x10000

# ISP-init template array: the contiguous {u64 source, u64 length} descriptor
# list the service copies its static payloads from. See docs/camera-stack.md.
TEMPLATE_ARRAY_VMA = 0x472600
TEMPLATE_ENTRIES = 56
TEMPLATE_ENTRY_BYTES = 16


def lib_slice(lib: bytes, vma: int, length: int, what: str) -> bytes:
    """
    `length` bytes of `lib` at `vma`, or exit naming `what`.

    Python truncates an out-of-range slice silently, so the bounds are checked
    here rather than left to the caller's unpack.
    """
    off = vma - VMA_TO_FILE
    if off < 0 or off + length > len(lib):
        sys.exit(f"{what}: VMA 0x{vma:x} + 0x{length:x} lies outside the library")

    return lib[off:off + length]


def template_entry(lib: bytes, index: int,
                   source: str | None = None) -> tuple[int, int]:
    """
    (source VMA, length) of one ISP-init template descriptor.

    `source` prefixes any error, for callers that accept more than one file.
    """
    where = f"{source}: " if source else ""
    if not 0 <= index < TEMPLATE_ENTRIES:
        sys.exit(f"{where}template entry {index} is outside the "
                 f"{TEMPLATE_ENTRIES}-entry array")

    raw = lib_slice(lib, TEMPLATE_ARRAY_VMA + index * TEMPLATE_ENTRY_BYTES,
                    TEMPLATE_ENTRY_BYTES, f"{where}template array entry {index}")

    return struct.unpack("<QQ", raw)


def template_payload(lib: bytes, index: int, expect_length: int,
                     source: str | None = None) -> bytes:
    """
    The bytes one template descriptor points at, its length asserted.

    A length mismatch means the array layout moved, which is what the callers
    exist to catch, so it exits rather than returning a short payload.
    """
    where = f"{source}: " if source else ""
    src, length = template_entry(lib, index, source)
    if length != expect_length:
        sys.exit(f"{where}template entry {index} is 0x{length:x} bytes, expected "
                 f"0x{expect_length:x}; the array layout has changed")

    return lib_slice(lib, src, length, f"{where}template entry {index} payload")
