#!/usr/bin/env python3
"""
Recover each ISP submodule's enable/bypass gate from libmpp_service.so.

Every submodule in the vendor service is one C file exposing the same method
set; the `__func__` strings `isp_sub_enable`, `isp_sub_disable`,
`isp_sub_process_reg_update` and `isp_sub_set_ctl` appear once per file, which
is what attributes a function to a module without any symbol table.

Two structures carry the answer, both read from code:

  1. A module's init calls ar_dev_pa2va() and stores register pointers into
     its private struct, which the caller reaches as [handle + 464]. The
     slots are NOT at fixed displacements: cnf holds the top bank at +552 and
     its own at +568, de3d holds the top bank at +560 and its own at +576,
     and qgg stores only its own bank and has no top pointer. So the map is
     derived per module by init_map(), and a fixed layout is not assumed.

     Other slots hold a second ISP instance, reached only when
     get_env() == 0x2304 and get_isp_work_mode() == 3. This device is an
     AR9311 and takes neither branch; those slots are absent from the derived
     map, so they drop out on their own.

  2. The module gates itself two ways, and both are walked:

       a bit, read-modify-written through a slot pointer. `orr` sets, `and`
       of an inverted mask clears. A bit written BOTH ways is a gate; one
       only ever set is configuration.

       a whole word, `str wzr` to turn the stage off and a computed value to
       turn it on. cm, wb, hdr_lsc and lms gate this way, and an orr/and-only
       detector misses them entirely.

     Which way means "on" is not in the instruction: some stages carry an
     enable bit and some carry a bypass bit. Polarity comes from pairing the
     gate against the tuning file and a live capture, below.

The same modules read their gate flag out of the tuning file through a fixed
chain, so `--blob` reports what the shipped tuning actually asks for:

    bl     isp_get_tuning_manager
    ldr    x0, [x0, #544]           the per-sensor tuning array
    umaddl x1, wIdx, wStride, x0    sensor index * 0x3b1e8
    ldr    xB, [x1, #24]            that sensor's tuning image
    add    xB, xB, #N, lsl #12      optional page part of the offset
    ldr    wF, [xB, #M]             the flag

giving file offset (N << 12) + M. gen-ccm.py carries two of these by hand
(GATE_CCM1 0x253fc, GATE_CCM2 0x2595c); this derives them, and the rest.

With `--capture` as well, each gate gets three independent readings: the bit
the library writes, what the tuning file asks for, and what the register holds
on a streaming vendor unit. Where the tuning flag and the live bit agree the
gate is enable-high, where they disagree it is bypass-high. A stage whose
three readings do not line up is printed with an empty verdict rather than
resolved by preference.

The library, tuning file and capture are not in the repository.

    kernel/scripts/isp/isp-gates.py \\
        --lib out/air-gather/vendor-root/usr/lib/libmpp_service.so \\
        --blob out/air-gather/camera/nt99235_tuning_preview_fpv.bin \\
        --capture out/au-chain-slota-live/slotA.txt
"""

import argparse
import bisect
import importlib.util
import re
import struct
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

# The module private struct, reached as [handle + 464]. Its pointer slots are
# NOT at fixed displacements: see init_map().
PRIVATE = 464

# The tuning-manager chain.
TUNING_ARRAY = 544
IMAGE_PTR = 24

# Instructions whose first operand is a source. Without this the compare and
# branch that guard an enable look like writes and discard the pointer the
# next instruction dereferences.
READS_ONLY = {
    'cbz', 'cbnz', 'tbz', 'tbnz', 'cmp', 'cmn', 'tst', 'ccmp', 'ccmn',
    'str', 'strb', 'strh', 'stur', 'sturb', 'sturh', 'stp', 'stnp',
    'ret', 'br', 'blr', 'b', 'bl', 'svc', 'nop',
}

INSN = re.compile(r'^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$')
MEM = re.compile(r'^\[(x\d+)(?:, #(\d+))?\]$')
REG = re.compile(r'^[wx]\d+$')
SRC_FILE = re.compile(r'^[\w/.-]*isp_sub_([\w]+)\.c$')

# Source-file basename to the name gen-isp-library.py uses for the same bank.
ALIAS = {
    'lee_lnr': 'lnr', 'raw_his_stats': 'raw_hist_stats',
    'rgb_his_stats': 'rgb_hist_stats', 'gtm2': 'ltm', 'birnr': 'rnr',
    'face_rro_stats': 'rro_face_stats',
}


def split_args(text):
    """Split an operand list on commas that are not inside [] brackets."""
    out, depth, cur = [], 0, ''
    for ch in text:
        if ch == '[':
            depth += 1
        elif ch == ']':
            depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def imm(tok):
    """The value of a `#0x...` or `#123` operand, masked to 32 bits."""
    tok = tok.strip()
    if not tok.startswith('#'):
        return None
    tok = tok[1:]
    try:
        value = int(tok, 16) if tok.startswith('0x') else int(tok)
    except ValueError:
        return None
    return value & 0xFFFFFFFF


class Library:
    """A disassembled libmpp_service.so, indexed by function."""

    def __init__(self, path):
        self.path = path
        self.data = Path(path).read_bytes()
        self._load_segments()
        self._load_functions()
        self._disassemble()
        self._attribute()

    # -- ELF ------------------------------------------------------------

    def _load_segments(self):
        """VMA to file-offset ranges, from the program headers."""
        self.segments = []
        out = subprocess.run(['readelf', '-lW', self.path],
                             capture_output=True, text=True, check=True).stdout
        for m in re.finditer(
                r'^\s+LOAD\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+0x[0-9a-f]+\s+'
                r'0x([0-9a-f]+)', out, re.M):
            off, vma, size = (int(m.group(i), 16) for i in (1, 2, 3))
            self.segments.append((vma, vma + size, off - vma))

    def cstr(self, vma, maxlen=300):
        """The NUL-terminated string at `vma`, or '' if it is not one."""
        for lo, hi, delta in self.segments:
            if lo <= vma < hi:
                off = vma + delta
                end = self.data.find(b'\0', off, off + maxlen)
                if end < 0:
                    return ''
                return self.data[off:end].decode('ascii', 'replace')
        return ''

    def _load_functions(self):
        """Exact function bounds from the FDEs in .eh_frame."""
        self.fdes = []
        out = subprocess.run(['readelf', '--debug-dump=frames', self.path],
                             capture_output=True, text=True, check=True).stdout
        for m in re.finditer(r'FDE cie=\w+ pc=([0-9a-f]+)\.\.([0-9a-f]+)', out):
            self.fdes.append((int(m.group(1), 16), int(m.group(2), 16)))
        self.fdes.sort()
        self.starts = [f[0] for f in self.fdes]

    def func_of(self, addr):
        i = bisect.bisect_right(self.starts, addr) - 1
        if i >= 0 and self.fdes[i][0] <= addr < self.fdes[i][1]:
            return self.fdes[i][0]
        return None

    # -- disassembly ----------------------------------------------------

    def _disassemble(self):
        """objdump the whole image once, bucketed by containing function."""
        for tool in ('aarch64-linux-gnu-objdump', 'objdump'):
            try:
                out = subprocess.run(
                    [tool, '-d', '--no-show-raw-insn', self.path],
                    capture_output=True, text=True, check=True).stdout
                break
            except (FileNotFoundError, subprocess.CalledProcessError):
                continue
        else:
            sys.exit('no objdump able to read an aarch64 image was found')

        self.body = defaultdict(list)
        self.plt = {}
        for line in out.splitlines():
            m = INSN.match(line)
            if not m:
                continue
            addr = int(m.group(1), 16)
            rest = m.group(3)
            call = re.match(r'^([0-9a-f]+) <([\w.]+)@plt>', rest.split('\t')[-1]
                            if '\t' in rest else rest)
            if m.group(2) == 'bl' and call:
                self.plt.setdefault(call.group(2), call.group(1))
            func = self.func_of(addr)
            if func is None:
                continue
            args = rest.split('//')[0].split('<')[0].strip()
            self.body[func].append((addr, m.group(2), split_args(args)))

    def _attribute(self):
        """Function to source file, via the adrp/add pairs it materialises."""
        pending, self.file_of = {}, {}
        for func, insns in self.body.items():
            pending.clear()
            for addr, op, a in insns:
                if op == 'adrp' and len(a) == 2:
                    try:
                        pending[a[0]] = int(a[1], 16)
                    except ValueError:
                        pending.pop(a[0], None)
                elif op == 'add' and len(a) == 3 and a[2].startswith('#') \
                        and a[1] in pending:
                    value = imm(a[2])
                    if value is None:
                        continue
                    target = pending[a[1]] + value
                    pending[a[0]] = target
                    m = SRC_FILE.match(self.cstr(target))
                    if m:
                        self.file_of.setdefault(func, set()).add(m.group(1))

        self.by_module = defaultdict(set)
        for func, files in self.file_of.items():
            if len(files) == 1:
                self.by_module[next(iter(files))].add(func)


def init_map(lib, func):
    """
    {private-struct slot: ISP register offset} for one module's init.

    The slots are NOT at fixed displacements across modules. cnf holds the top
    bank at +552 and its own bank at +568; de3d holds the top bank at +560 and
    its own at +576, and reading de3d's +552 as a top pointer yields registers
    that do not exist. So the map is derived per module from the init that
    calls ar_dev_pa2va, and a module whose init is not found is skipped rather
    than assumed.
    """
    slots = {}
    priv = {}      # register -> displacement into the private struct
    va = {}        # register -> offset from ar_dev_pa2va()
    konst = {}

    for _addr, op, a in lib.body.get(func, []):
        if op == 'bl':
            if a and a[0] == lib.plt.get('ar_dev_pa2va'):
                va = {'x0': 0}
                konst.clear()
            continue

        if op == 'ldr' and len(a) == 2:
            m = MEM.match(a[1])
            priv.pop(a[0], None)
            va.pop(a[0], None)
            konst.pop(a[0], None)
            if m and int(m.group(2) or 0) == PRIVATE:
                priv[a[0]] = 0
            continue

        if op == 'mov' and len(a) == 2 and a[1].startswith('#'):
            va.pop(a[0], None)
            priv.pop(a[0], None)
            value = imm(a[1])
            konst[a[0]] = value
            if value is None:
                konst.pop(a[0], None)
            continue

        if op == 'add' and len(a) in (3, 4):
            # A bank literal reaches the add either whole, through a register,
            # or pre-shifted as `#0x5, lsl #12`, which a plain-immediate scan
            # misses (wb 0x5000, raw_hist, drc, isp_input 0x7000).
            shift = 12 if len(a) == 4 and 'lsl #12' in a[3] else 0
            if len(a) == 4 and not shift:
                va.pop(a[0], None)
                priv.pop(a[0], None)
                konst.pop(a[0], None)
                continue
            src, off = a[1], None
            if a[2].startswith('#'):
                off = imm(a[2])
            elif a[2] in konst:
                off = konst[a[2]]
            if off is not None:
                off <<= shift
            dst = a[0]
            base_va, base_priv = va.get(src), priv.get(src)
            va.pop(dst, None)
            priv.pop(dst, None)
            konst.pop(dst, None)
            if off is None:
                continue
            if base_va is not None:
                va[dst] = base_va + off
            elif base_priv is not None:
                priv[dst] = base_priv + off
            continue

        if op in ('str', 'stp'):
            m = MEM.match(a[-1])
            if m and m.group(1) in priv:
                at = priv[m.group(1)] + int(m.group(2) or 0)
                for i, reg in enumerate(a[:-1]):
                    if reg in va:
                        slots[at + 8 * i] = va[reg]
            continue

        if op.startswith('b.') or op in READS_ONLY:
            continue

        if a and REG.match(a[0]):
            va.pop(a[0], None)
            priv.pop(a[0], None)
            konst.pop(a[0], None)

    return slots


def name_of(slots, slot):
    """A label for a slot: the top bank, its +4 companion, or the own bank."""
    base = slots[slot]
    if base == 0:
        return 'top'
    if base == 4:
        return 'top+4'
    return 'bank'


def module_slots(lib, mod):
    """The slot map for a module, from whichever of its functions is the init."""
    best = {}
    for func in sorted(lib.by_module[mod]):
        slots = init_map(lib, func)
        # Not every init installs a top pointer: qgg stores only its own bank,
        # so requiring offset 0 would drop it. Take the richest map instead.
        if len(slots) > len(best):
            best = slots
    return best


def gates(lib, func, slots):
    """
    (slot, displacement, mask, set|clear) for each RMW through a slot pointer.

    Only pointers loaded out of the module private struct count, so another
    struct with a field at the same displacement cannot pass for a register
    base. The walk is straight-line: it ignores branches, which is safe here
    because the load, the mask and the store of one gate are contiguous.
    """
    out = []
    priv, ptr, val, konst = set(), {}, {}, {}

    def kill(reg):
        priv.discard(reg)
        ptr.pop(reg, None)
        konst.pop(reg, None)
        for alias in (reg, 'w' + reg[1:], 'x' + reg[1:]):
            val.pop(alias, None)

    for _addr, op, a in lib.body.get(func, []):
        if op == 'ldr' and len(a) == 2:
            m = MEM.match(a[1])
            if not m:
                kill(a[0])
                continue
            base, disp = m.group(1), int(m.group(2) or 0)
            kill(a[0])
            if disp == PRIVATE:
                priv.add(a[0])
            elif base in priv and disp in slots:
                ptr[a[0]] = disp
            elif base in ptr and a[0].startswith('w'):
                val[a[0]] = (ptr[base], base, disp, None, None)

        elif op == 'mov' and len(a) == 2 and a[1].startswith('#'):
            kill(a[0])
            value = imm(a[1])
            if value is not None:
                konst[a[0]] = value

        elif op in ('orr', 'and', 'bic') and len(a) == 3 and a[1] in val:
            slot, preg, disp, _mask, _kind = val[a[1]]
            value = imm(a[2]) if a[2].startswith('#') else konst.get(a[2])
            kill(a[0])
            if value is None:
                continue
            if op == 'orr':
                mask, kind = value, 'set'
            elif op == 'and':
                mask, kind = (~value) & 0xFFFFFFFF, 'clear'
            else:
                mask, kind = value, 'clear'
            val[a[0]] = (slot, preg, disp, mask, kind)

        elif op == 'str' and len(a) == 2 and a[0] in val:
            m = MEM.match(a[1])
            slot, preg, disp, mask, kind = val[a[0]]
            if m and kind and m.group(1) == preg \
                    and int(m.group(2) or 0) == disp:
                out.append((slot, disp, mask, kind))

        elif op.startswith('b.') or op in READS_ONLY:
            continue

        elif a and REG.match(a[0]):
            kill(a[0])
            if op in ('ldp', 'ldnp') and len(a) > 1 and REG.match(a[1]):
                kill(a[1])

    return out


def word_gates(lib, func, slots):
    """
    (slot, displacement, zero|value) for whole-word writes through a slot.

    Not every stage gates on a bit. `cm` and `wb` write their whole gate word,
    `str wzr` to turn the stage off and a computed value to turn it on, so a
    detector that only looks for orr/and misses them entirely.
    """
    out = []
    priv, ptr = set(), {}
    for _addr, op, a in lib.body.get(func, []):
        if op == 'ldr' and len(a) == 2:
            m = MEM.match(a[1])
            priv.discard(a[0])
            ptr.pop(a[0], None)
            if not m:
                continue
            base, disp = m.group(1), int(m.group(2) or 0)
            if disp == PRIVATE:
                priv.add(a[0])
            elif base in priv and disp in slots:
                ptr[a[0]] = disp

        elif op == 'str' and len(a) == 2:
            m = MEM.match(a[1])
            if m and m.group(1) in ptr and a[0].startswith('w'):
                out.append((ptr[m.group(1)], int(m.group(2) or 0),
                            'zero' if a[0] == 'wzr' else 'value'))

        elif op.startswith('b.') or op in READS_ONLY:
            continue

        elif a and REG.match(a[0]):
            priv.discard(a[0])
            ptr.pop(a[0], None)

    return out


def tuning_offsets(lib, func):
    """
    File offsets of the tuning-image fields this function TESTS.

    A module reads many tuning fields; only a gate flag is branched on. Taking
    the first read instead picks up payload fields and lands on offsets that
    contradict the live register (acm, af_stats), so a field only counts once
    a cbz/cbnz/tbz/cmp consumes the register it was loaded into.
    """
    out, state = [], {}
    pending = {}      # register -> offset it was loaded from
    tm = lib.plt.get('isp_get_tuning_manager')
    if tm is None:
        return out

    for _addr, op, a in lib.body.get(func, []):
        if op == 'bl':
            if a and a[0] == tm:
                state['x0'] = ('tm', 0)
            continue

        if op == 'ldr' and len(a) == 2:
            m = MEM.match(a[1])
            if not m:
                state.pop(a[0], None)
                continue
            base, disp = m.group(1), int(m.group(2) or 0)
            src = state.get(base)
            state.pop(a[0], None)
            if src is None:
                continue
            kind, off = src
            if kind == 'tm' and disp == TUNING_ARRAY:
                state[a[0]] = ('arr', 0)
            elif kind == 'arr' and disp == IMAGE_PTR:
                state[a[0]] = ('img', 0)
            elif kind == 'img':
                if a[0].startswith('w'):
                    pending[a[0]] = off + disp
                else:
                    state[a[0]] = ('img', off + disp)
            continue

        if op == 'umaddl' and len(a) == 4:
            # The sensor-index multiply keeps us inside the same array.
            if state.get(a[3], (None,))[0] == 'arr':
                state[a[0]] = ('arr', 0)
            else:
                state.pop(a[0], None)
            continue

        if op == 'add' and len(a) in (3, 4) and a[2].startswith('#'):
            src = state.get(a[1])
            value = imm(a[2])
            shift = 12 if len(a) == 4 and 'lsl #12' in a[3] else 0
            if src and src[0] == 'img' and value is not None:
                state[a[0]] = ('img', src[1] + (value << shift))
            else:
                state.pop(a[0], None)
            continue

        if op in ('cbz', 'cbnz', 'tbz', 'tbnz', 'cmp', 'ccmp') and a:
            if a[0] in pending:
                out.append(pending[a[0]])
            continue

        if op.startswith('b.') or op in READS_ONLY:
            continue

        if a and REG.match(a[0]):
            state.pop(a[0], None)
            pending.pop(a[0], None)

    return out


def load_banks():
    """Module to bank, from the validated map in gen-isp-library.py."""
    path = Path(__file__).with_name('gen-isp-library.py')
    spec = importlib.util.spec_from_file_location('gen_isp_library', path)
    mod = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(path.parent))
    spec.loader.exec_module(mod)

    banks = {}
    for _entry, name, bank, _src in mod.MODULES:
        banks.setdefault(name, bank)
    for _entry, name, bank in mod.UNINSTALLED:
        banks.setdefault(name, bank)
    return banks


def read_capture(path):
    """ISP offset to word, from a `SECTION isp-XX` register dump."""
    regs, section = {}, None
    for line in Path(path).read_text().splitlines():
        m = re.match(r'^SECTION isp-([0-9a-f]{2})\s*$', line)
        if m:
            section = int(m.group(1), 16) << 8
            continue
        if line.startswith('SECTION'):
            section = None
            continue
        m = re.match(r'^\+0x([0-9a-f]{4}): ((?:[0-9a-f]{8} ?)+)', line)
        if m and section is not None:
            off = int(m.group(1), 16)
            for i, word in enumerate(m.group(2).split()):
                regs[section + off + 4 * i] = int(word, 16)
    return regs


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lib', required=True, help='libmpp_service.so')
    ap.add_argument('--blob', help='vendor tuning file')
    ap.add_argument('--capture', help='slot-A register capture')
    ap.add_argument('--all', action='store_true',
                    help='also show writes that are not two-way gates')
    args = ap.parse_args()

    lib = Library(args.lib)
    banks = load_banks()
    blob = Path(args.blob).read_bytes() if args.blob else None
    live = read_capture(args.capture) if args.capture else {}

    print(f'{"stage":16s} {"gate":8s} {"bit":4s} {"live":11s} {"="} '
          f'{"tuning":12s} {"gate":5s} verdict')
    print('-' * 78)

    unresolved, missing, drift = [], [], []
    for mod in sorted(lib.by_module):
        slots = module_slots(lib, mod)
        if not slots:
            missing.append(mod)
            continue
        bank = banks.get(ALIAS.get(mod, mod))
        own = {v for v in slots.values() if v not in (0, 4)}
        if bank is not None and own and bank not in own:
            drift.append((mod, bank, sorted(own)))

        setm = defaultdict(int)
        clrm = defaultdict(int)
        for func in sorted(lib.by_module[mod]):
            for slot, disp, mask, kind in gates(lib, func, slots):
                reg = slots[slot] + disp
                key = (reg, name_of(slots, slot))
                if kind == 'set':
                    setm[key] |= mask
                else:
                    clrm[key] |= mask

        # A gate bit is one the module both sets and clears. The two masks
        # need not match: the top-level stages set their own bit and clear it
        # together with a companion, so intersect rather than compare.
        # Whole-word gates: the same register written both zero and non-zero.
        words = defaultdict(set)
        for func in sorted(lib.by_module[mod]):
            for slot, disp, kind in word_gates(lib, func, slots):
                words[(slots[slot] + disp, name_of(slots, slot))].add(kind)

        found = []
        for key, kinds in sorted(words.items()):
            if len(kinds) == 2:
                found.append((key[0], 0, key[1] + ' word'))
        for key in sorted(set(setm) | set(clrm)):
            both = setm[key] & clrm[key]
            for bit in range(32):
                if both >> bit & 1:
                    found.append((key[0], 1 << bit, key[1]))
            if args.all:
                rest = (setm[key] | clrm[key]) & ~both
                for bit in range(32):
                    if rest >> bit & 1:
                        found.append((key[0], 1 << bit, key[1] + '?'))
        if not found:
            continue

        offsets = []
        if blob is not None:
            for func in sorted(lib.by_module[mod]):
                offsets += [o for o in tuning_offsets(lib, func)
                            if o % 4 == 0 and o + 4 <= len(blob)]
        # Offsets past 0xb0000 sit outside the per-stage tuning region and
        # have not been shown to be gates, so a lower one is preferred.
        low = [o for o in offsets if o < 0xB0000]
        off = (low or offsets or [None])[0]
        gate = struct.unpack_from('<I', blob, off)[0] \
            if off is not None else None

        for reg, mask, where in sorted(found):
            bit = mask.bit_length() - 1
            word = live.get(reg)
            shown = f'0x{word:08x}' if word is not None else '--'
            if word is None:
                state = '-'
            elif mask:
                state = str(1 if word & mask else 0)
            else:
                state = str(1 if word else 0)
            gtxt = '--' if gate is None else str(gate)
            verdict = ''
            if state in '01' and gtxt in ('0', '1'):
                verdict = 'enable-high' if state == gtxt else 'bypass-high'
            elif state in '01' or gtxt in ('0', '1'):
                unresolved.append(mod)
            offtxt = f'0x{off:06x}' if off is not None else '--'
            print(f'{mod:16s} 0x{reg:04x}   {"word" if not mask else bit:<4} {shown:11s} {state} '
                  f'{offtxt:12s} {gtxt:5s} {verdict}   [{where}]')


    if missing:
        print(f'\nno init found, slot map unknown: {", ".join(sorted(missing))}')
    for mod, bank, own in drift:
        seen = ', '.join(f'0x{o:04x}' for o in own)
        print(f'{mod}: gen-isp-library.py says bank 0x{bank:04x}, '
              f'its init installs {seen}')
    if unresolved:
        print(f'\nno verdict (one reading missing): '
              f'{", ".join(sorted(set(unresolved)))}')


if __name__ == '__main__':
    main()
