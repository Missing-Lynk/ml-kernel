"""
The sensor tuning blob's layout, and typed access to a loaded blob.

`blob-layout.toml` beside this file is the mapping: every field recovered from
nt99235_tuning_preview_fpv.bin.

    from blob_layout import Layout, Blob

    lay = Layout.load()                       # the mapping alone, no blob needed
    lay['rnr_payload'].offset                 # 0x7a6c
    lay.stage('rnr')                          # every rnr section

    blob = Blob.open(path)                    # mapping + bytes
    blob.u32('ae_damping')                    # 50
    blob.f32('ae_log_ladder')                 # 77.893997
    blob.records('ae_exposure_table')         # 366 slices of 8 bytes
    blob.array('rnr_bands')                   # 32 floats

Section offsets are absolute. A `header` section also carries `fields`, the displacements within
it that the C side adds; `field_offset` resolves the two.
"""

import struct
import sys
import tomllib
from collections.abc import Iterator, Sequence
from dataclasses import dataclass, field
from pathlib import Path

LAYOUT_PATH = Path(__file__).resolve().parent / "blob-layout.toml"

# `count` elements of `elem`.
ARRAY_KINDS = frozenset({"array", "bands"})
# `count` records of `stride` bytes.
RECORD_KINDS = frozenset({"record_array", "payload", "page"})

ELEM_SIZE = {"u32": 4, "f32": 4}


@dataclass(frozen=True)
class Section:
    """One recovered region of the blob."""

    name: str
    stage: str
    offset: int
    kind: str
    elem: str | None = None
    count: int | None = None
    stride: int | None = None
    length: int | None = None
    doc: str = ""
    fields: dict[str, int] = field(default_factory=dict)

    @property
    def size(self) -> int:
        """Bytes the section spans."""
        if self.length is not None:
            return self.length

        if self.kind in RECORD_KINDS:
            return (self.count or 1) * (self.stride or 0)

        if self.kind in ARRAY_KINDS:
            return (self.count or 1) * ELEM_SIZE[self.elem or "u32"]

        return ELEM_SIZE.get(self.elem or "u32", 4)

    @property
    def end(self) -> int:
        return self.offset + self.size

    def field_offset(self, name: str) -> int:
        """The absolute offset of a named field within a `header` section."""
        if name not in self.fields:
            raise KeyError(f"{self.name} has no field {name!r}; it has {sorted(self.fields)}")

        return self.offset + self.fields[name]


class Layout:
    """The mapping: offsets, extents, provenance. Needs no blob."""

    def __init__(self, size: int, sensors: Sequence[str], sections: Sequence[Section]) -> None:
        self.size = size
        self.sensors = tuple(sensors)
        self._sections = tuple(sections)
        self._by_name = {s.name: s for s in sections}
        if len(self._by_name) != len(self._sections):
            seen: set[str] = set()
            dupes = sorted({s.name for s in sections if s.name in seen or seen.add(s.name)})
            raise ValueError(f"blob-layout.toml has duplicate section names: {dupes}")

    @classmethod
    def load(cls, path: Path = LAYOUT_PATH) -> Layout:
        try:
            raw = tomllib.loads(path.read_text())
        except FileNotFoundError:
            sys.exit(f"{path}: not found")

        meta = raw.get("meta", {})
        sections = [
            Section(
                name=s["name"],
                stage=s.get("stage", "?"),
                offset=s["offset"],
                kind=s["kind"],
                elem=s.get("elem"),
                count=s.get("count"),
                stride=s.get("stride"),
                length=s.get("length"),
                doc=s.get("doc", ""),
                fields=s.get("fields", {}),
            )
            for s in raw.get("section", ())
        ]

        return cls(meta.get("size", 0xD6C58), meta.get("sensors", ()), sections)

    def __getitem__(self, name: str) -> Section:
        if name not in self._by_name:
            raise KeyError(f"no section {name!r} in blob-layout.toml")

        return self._by_name[name]

    def __contains__(self, name: str) -> bool:
        return name in self._by_name

    def __iter__(self) -> Iterator[Section]:
        return iter(self._sections)

    def __len__(self) -> int:
        return len(self._sections)

    def stage(self, stage: str) -> list[Section]:
        return [s for s in self._sections if s.stage == stage]

    def stages(self) -> list[str]:
        return sorted({s.stage for s in self._sections})

    def gates(self) -> list[Section]:
        return [s for s in self._sections if s.kind == "gate"]

    def anchors(self) -> list[tuple[str, int]]:
        """(stage, offset) for every stage's enable word, ascending.

        A ladder's enable word is its gate: rnr and lnr are anchored by their header's `enable`
        field, not by a `gate` section. drc, gamma, cm and cm2 carry both at the same offset.
        """
        found: dict[int, str] = {}
        for s in self._sections:
            if s.kind == "gate":
                found.setdefault(s.offset, s.stage)
            elif "enable" in s.fields:
                found.setdefault(s.field_offset("enable"), s.stage)

        return sorted(((stage, off) for off, stage in found.items()), key=lambda p: p[1])

    def spans(self) -> list[tuple[int, int, str]]:
        """(start, length, name) for every section, ascending."""
        return sorted((s.offset, s.size, s.name) for s in self._sections)

    def at(self, offset: int) -> list[Section]:
        """Every section covering an absolute offset, and every header that names it.

        0x79e0 appears nowhere in the tree; the kernel reads it as AR_ISP_RNR_BLOB_HEADER +
        AR_ISP_RNR_HDR_MODE.
        """
        hit = [s for s in self._sections if s.offset <= offset < s.end]
        named = [s for s in self._sections
                 if s.fields and any(s.field_offset(f) == offset for f in s.fields)]

        # Deduped by name: Section carries a dict and is not hashable.
        seen: dict[str, Section] = {s.name: s for s in hit + named}

        return sorted(seen.values(), key=lambda s: (s.offset, s.name))


class Blob:
    """A loaded tuning file, read through the mapping."""

    def __init__(self, data: bytes, layout: Layout | None = None) -> None:
        self.layout = layout or Layout.load()
        if len(data) != self.layout.size:
            raise ValueError(f"{len(data)} bytes, not the {self.layout.size} a tuning blob has")

        self.data = data

    @classmethod
    def open(cls, path: Path | str, layout: Layout | None = None) -> Blob:
        p = Path(path)
        if not p.exists():
            sys.exit(f"{p}: not found")

        return cls(p.read_bytes(), layout)

    def raw(self, name: str) -> bytes:
        s = self.layout[name]

        return self.data[s.offset:s.end]

    def u32(self, name: str, index: int = 0) -> int:
        s = self.layout[name]

        return struct.unpack_from("<I", self.data, s.offset + index * 4)[0]

    def f32(self, name: str, index: int = 0) -> float:
        s = self.layout[name]

        return struct.unpack_from("<f", self.data, s.offset + index * 4)[0]

    def field(self, name: str, field_name: str) -> int:
        """One named word of a `header` section."""
        return struct.unpack_from("<I", self.data, self.layout[name].field_offset(field_name))[0]

    def array(self, name: str) -> list[int] | list[float]:
        s = self.layout[name]
        spec = "<f" if s.elem == "f32" else "<I"

        return [struct.unpack_from(spec, self.data, s.offset + i * 4)[0]
                for i in range(s.count or 0)]

    def records(self, name: str) -> list[bytes]:
        s = self.layout[name]
        stride = s.stride or 0

        return [self.data[s.offset + i * stride:s.offset + (i + 1) * stride]
                for i in range(s.count or 0)]

    def gate(self, name: str) -> bool:
        return self.u32(name) == 1
