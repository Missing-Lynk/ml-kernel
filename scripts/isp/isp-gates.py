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
                out.append((slot, disp, mask, kind, _addr))

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
                            'zero' if a[0] == 'wzr' else 'value', _addr))

        elif op.startswith('b.') or op in READS_ONLY:
            continue

        elif a and REG.match(a[0]):
            priv.discard(a[0])
            ptr.pop(a[0], None)

    return out


def tuning_tests(lib, func):
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

    for addr, op, a in lib.body.get(func, []):
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
                out.append((pending[a[0]], addr, op))
            continue

        if op.startswith('b.') or op in READS_ONLY:
            continue

        if a and REG.match(a[0]):
            state.pop(a[0], None)
            pending.pop(a[0], None)

    return out


def tuning_offsets(lib, func):
    """Just the file offsets, for callers that do not need the test site."""
    return [off for off, _addr, _op in tuning_tests(lib, func)]


# Branches that fall through as well as jumping, and those that do not.
COND_BRANCH = {'cbz', 'cbnz', 'tbz', 'tbnz'}
NO_FALLTHROUGH = {'b', 'br', 'ret'}
# The branch taken when the tested register is zero.
ZERO_ON_TAKEN = {'cbz', 'tbz'}


def successors(lib, func):
    """{address: [addresses that can follow it]} for one function."""
    out = {}
    insns = lib.body.get(func, [])
    for i, (addr, op, a) in enumerate(insns):
        nxt = insns[i + 1][0] if i + 1 < len(insns) else None
        targets = []
        if op in COND_BRANCH or op.startswith('b.') or op == 'b':
            try:
                targets.append(int(a[-1], 16))
            except (ValueError, IndexError):
                pass

        if op not in NO_FALLTHROUGH and nxt is not None:
            targets.append(nxt)

        out[addr] = targets
    return out


def reachable(succ, start):
    """Every address reachable from `start`, following the successor map."""
    seen, stack = set(), [start]
    while stack:
        addr = stack.pop()
        if addr in seen or addr not in succ:
            continue

        seen.add(addr)
        stack.extend(succ[addr])

    return seen


def flag_paths(lib, func, branch_addr, branch_op):
    """
    (addresses reached only when the flag is zero, only when it is non-zero).

    Which way a gate bit means "on" is not in the instruction: the top-level
    register holds enable bits and bypass bits side by side. It is in the
    branch. acm reaches `orr w0, w0, #0x1` on its flag-ZERO path, so that bit
    turns the stage off, while ccm1 reaches `orr w0, w0, #0x200000` on its
    flag-NON-ZERO path, so that one turns it on. Reading it here keeps the
    capture out of the derivation and leaves it free to falsify the result.

    Reachability, not control dependence. Control dependence is the textbook
    relation and it was tried: it classifies nothing here, because each arm
    branches again internally (a null check, a work-mode test) so the gate
    write does not post-dominate the arm it sits in. Exclusive reachability
    is weaker but decides these functions, and a site both arms can reach is
    still declined rather than guessed.
    """
    succ = successors(lib, func)
    insns = lib.body.get(func, [])
    taken, fall = None, None
    for i, (addr, _op, a) in enumerate(insns):
        if addr != branch_addr:
            continue

        try:
            taken = int(a[-1], 16)
        except (ValueError, IndexError):
            return set(), set()
        fall = insns[i + 1][0] if i + 1 < len(insns) else None
        break

    if taken is None or fall is None:
        return set(), set()

    if branch_op in ZERO_ON_TAKEN:
        zero_start, nonzero_start = taken, fall
    else:
        zero_start, nonzero_start = fall, taken

    zero = reachable(succ, zero_start)
    nonzero = reachable(succ, nonzero_start)

    # Where the two paths rejoin says nothing, so only the exclusive parts do.
    return zero - nonzero, nonzero - zero


def cached_flags(lib, func):
    """
    {private-struct offset: tuning offset} for flags this function caches.

    A module usually reads its flag in set_ctl and writes the gate bit in
    process_reg_update, so the two are not in the same function and the branch
    that decides the bit tests a cached copy. ccm1 is the shape: set_ctl loads
    the flag at 0x253fc and stores it into the instance, process_reg_update
    branches on that field.
    """
    out, pending = {}, {}
    state = {}
    tm = lib.plt.get('isp_get_tuning_manager')
    priv = set()
    for _addr, op, a in lib.body.get(func, []):
        if op == 'bl':
            if a and a[0] == tm:
                state['x0'] = ('tm', 0)

            continue

        if op == 'ldr' and len(a) == 2:
            m = MEM.match(a[1])
            state.pop(a[0], None)
            pending.pop(a[0], None)

            if not m:
                continue

            base, disp = m.group(1), int(m.group(2) or 0)
            if disp == PRIVATE:
                priv.add(a[0])
                continue

            src = state.get(base)
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
            state[a[0]] = ('arr', 0) if state.get(a[3], (None,))[0] == 'arr' \
                else state.pop(a[0], None)
            continue

        if op == 'add' and len(a) in (3, 4) and a[2].startswith('#'):
            src, value = state.get(a[1]), imm(a[2])
            shift = 12 if len(a) == 4 and 'lsl #12' in a[3] else 0
            if src and src[0] == 'img' and value is not None:
                state[a[0]] = ('img', src[1] + (value << shift))
            else:
                state.pop(a[0], None)
            continue

        if op == 'str' and len(a) == 2 and a[0] in pending:
            m = MEM.match(a[1])
            if m and m.group(1) in priv:
                out[int(m.group(2) or 0)] = pending[a[0]]
            continue

        if op.startswith('b.') or op in READS_ONLY:
            continue

        if a and REG.match(a[0]):
            state.pop(a[0], None)
            pending.pop(a[0], None)
            priv.discard(a[0])

    return out


def cached_tests(lib, func, fields):
    """(branch address, op) for branches on a cached flag field."""
    out, held, priv = [], {}, set()
    for addr, op, a in lib.body.get(func, []):
        if op == 'ldr' and len(a) == 2:
            m = MEM.match(a[1])
            held.pop(a[0], None)
            priv.discard(a[0])
            if not m:
                continue

            base, disp = m.group(1), int(m.group(2) or 0)
            if disp == PRIVATE:
                priv.add(a[0])
            elif base in priv and disp in fields and a[0].startswith('w'):
                held[a[0]] = disp
            continue

        if op in ('cbz', 'cbnz') and a and a[0] in held:
            out.append((addr, op))
            continue

        if op.startswith('b.') or op in READS_ONLY:
            continue

        if a and REG.match(a[0]):
            held.pop(a[0], None)
            priv.discard(a[0])

    return out


def derive_polarity(lib, mod, slots):
    """
    {(register, mask): 'enable' | 'bypass'} for the gates this module writes.

    Derived from the branch alone: a bit the module SETS on the path where its
    tuning flag is non-zero is an enable, one it sets on the flag-zero path is
    a bypass. Nothing here reads a capture, which leaves the capture free to
    falsify the result instead of producing it.
    """
    out = {}
    fields = {}
    for func in sorted(lib.by_module[mod]):
        fields.update(cached_flags(lib, func))

    for func in sorted(lib.by_module[mod]):
        sites = gates(lib, func, slots)
        if not sites:
            continue

        tests = [(addr, op) for _off, addr, op in tuning_tests(lib, func)]
        tests += cached_tests(lib, func, fields)
        for branch_addr, branch_op in tests:
            zero, nonzero = flag_paths(lib, func, branch_addr, branch_op)
            for slot, disp, mask, kind, addr in sites:
                if kind != 'set' or slot not in slots:
                    continue

                if addr in nonzero:
                    sense = 'enable'
                elif addr in zero:
                    sense = 'bypass'
                else:
                    continue

                out.setdefault((slots[slot] + disp, mask), sense)

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


def collect(lib, banks, blob, live, show_all=False):
    """
    Every module's gates, as records the report and the header both read.

    Per stage: its bank, the tuning offset its gate flag comes from, and one
    entry per gate with the mask the enable path sets, the mask the disable
    path clears, and the polarity. `missing` and `drift` carry the two ways a
    module can fail to produce a slot map at all.
    """
    stages, missing, drift = [], [], []

    for mod in sorted(lib.by_module):
        slots = module_slots(lib, mod)
        if not slots:
            missing.append(mod)
            continue

        sense = derive_polarity(lib, mod, slots)
        bank = banks.get(ALIAS.get(mod, mod))
        own = {v for v in slots.values() if v not in (0, 4)}
        if bank is not None and own and bank not in own:
            drift.append((mod, bank, sorted(own)))

        setm = defaultdict(int)
        clrm = defaultdict(int)
        # Kept per instruction as well as unioned. The disable path clears the
        # gate bit together with a companion in one write, and which companion
        # travels with which gate is only visible before the union: dpc gates
        # three bits of 0x0c10 and attributing all three to the first would
        # invent a mask the library never writes.
        clr_insns = defaultdict(list)
        for func in sorted(lib.by_module[mod]):
            for slot, disp, mask, kind, _at in gates(lib, func, slots):
                reg = slots[slot] + disp
                key = (reg, name_of(slots, slot))
                if kind == 'set':
                    setm[key] |= mask
                else:
                    clrm[key] |= mask
                    clr_insns[key].append(mask)

        # A gate bit is one the module both sets and clears. The two masks
        # need not match: the top-level stages set their own bit and clear it
        # together with a companion, so intersect rather than compare.
        # Whole-word gates: the same register written both zero and non-zero.
        words = defaultdict(set)
        for func in sorted(lib.by_module[mod]):
            for slot, disp, kind, _at in word_gates(lib, func, slots):
                words[(slots[slot] + disp, name_of(slots, slot))].add(kind)

        found = []
        for key, kinds in sorted(words.items()):
            if len(kinds) == 2:
                found.append((key[0], 0, key[1] + ' word', key))

        for key in sorted(set(setm) | set(clrm)):
            both = setm[key] & clrm[key]
            for bit in range(32):
                if both >> bit & 1:
                    found.append((key[0], 1 << bit, key[1], key))

            if show_all:
                rest = (setm[key] | clrm[key]) & ~both
                for bit in range(32):
                    if rest >> bit & 1:
                        found.append((key[0], 1 << bit, key[1] + '?', key))

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

        entries = []
        for reg, mask, where, key in sorted(found):
            word = live.get(reg)
            if word is None:
                state = None
            elif mask:
                state = 1 if word & mask else 0
            else:
                state = 1 if word else 0

            # Polarity from the branch is the answer; the paired reading is
            # only a check on it, and it mislabels any stage that is not
            # running because that bank reads zero either way.
            code = sense.get((reg, mask))
            paired = None
            if state is not None and gate in (0, 1):
                paired = 'enable' if state == gate else 'bypass'

            bit = mask.bit_length() - 1
            clear = 0
            for m in clr_insns.get(key, ()):
                if mask and m >> bit & 1:
                    clear |= m

            entries.append({
                'reg': reg,
                'mask': mask,
                'set_mask': mask or setm.get(key, 0),
                'clear_mask': clear,
                'where': where,
                'live': word,
                'state': state,
                'code': code,
                'paired': paired,
            })

        stages.append({
            'stage': mod,
            'bank': bank,
            'blob_off': off,
            'blob_gate': gate,
            'gates': entries,
        })

    return stages, missing, drift


def report(stages, missing, drift):
    """The human table: one line per gate, three readings and a verdict."""
    print(f'{"stage":16s} {"gate":8s} {"bit":4s} {"live":11s} {"="} '
          f'{"tuning":12s} {"gate":5s} verdict')
    print('-' * 78)

    unresolved = []
    for st in stages:
        off, gate = st['blob_off'], st['blob_gate']
        offtxt = f'0x{off:06x}' if off is not None else '--'
        gtxt = '--' if gate is None else str(gate)

        for g in st['gates']:
            state = '-' if g['state'] is None else str(g['state'])
            shown = f'0x{g["live"]:08x}' if g['live'] is not None else '--'
            code, paired = g['code'], g['paired']
            verdict = f'{code}*' if code else ''
            if paired:
                if code and code != paired:
                    verdict = f'{code}* (paired says {paired})'
                elif not code:
                    verdict = f'{paired} (paired)'
            elif g['state'] is not None or gtxt in ('0', '1'):
                unresolved.append(st['stage'])

            bit = 'word' if not g['mask'] else g['mask'].bit_length() - 1
            print(f'{st["stage"]:16s} 0x{g["reg"]:04x}   {bit:<4} {shown:11s} '
                  f'{state} {offtxt:12s} {gtxt:5s} {verdict}   [{g["where"]}]')

    if missing:
        print(f'\nno init found, slot map unknown: {", ".join(sorted(missing))}')
    for mod, bank, own in drift:
        seen = ', '.join(f'0x{o:04x}' for o in own)
        print(f'{mod}: gen-isp-library.py says bank 0x{bank:04x}, '
              f'its init installs {seen}')
    if unresolved:
        print(f'\nno verdict (one reading missing): '
              f'{", ".join(sorted(set(unresolved)))}')


# Stages left out of the generated table.
#
# The _v1 variants share their sibling's tuning flag and their banks read zero
# on a streaming vendor unit, so the shipped pipeline runs the non-v1 pair;
# two stages behind one flag is a way to gate both by accident. hdr owns four
# banks and its init installs 0x1fe0 rather than the 0x1c00 the bank map
# records, so its gate is not resolved against a one-bank slot map.
EXCLUDE = ('ltm_v1', 'drc_v1', 'gamma_v1', 'hdr')

# Stages that own a DMA table fetch. A fetch stage toggled mid-stream risks
# stalling the fetch rather than bypassing the stage, so the flag travels with
# the table and the decision is the driver's.
FETCH = ('lsc', 'ltm', 'gtm2', 'gamma', 'drc')

KINDS = {
    None: 'AR_ISP_GATE_UNKNOWN',
    'enable': 'AR_ISP_GATE_BIT_ENABLE',
    'bypass': 'AR_ISP_GATE_BIT_BYPASS',
}

HEADER_DOC = '''/* SPDX-License-Identifier: GPL-2.0 */
/* Generated by kernel/scripts/isp/isp-gates.py --header. Do not edit. */
/*
 * Which register and bit gates each ISP submodule, and where the tuning file
 * decides it.
 *
 * Recovered from libmpp_service.so by kernel/scripts/isp/isp-gates.py; see
 * plans/isp-stage-gates.md for the mechanism. Nothing here is inferred from a
 * capture: the register and the masks come from the library's own read-modify
 * writes, and the polarity from which side of the tuning-flag branch the set
 * sits on. Each row carries how its polarity was settled, `code` where the
 * branch gives it directly and `paired` where it comes from the tuning flag
 * and a live register agreeing.
 *
 * set_mask is what the enable path sets; clear_mask is what the disable path
 * clears, which for the top-level stages is the gate bit together with a
 * companion the vendor always clears with it (ccm1 23, drc 20, gamma 30).
 * What those companions select is not recovered, so they are carried exactly
 * as the library writes them rather than narrowed to a guess.
 *
 * blob_gate is the offset of the stage's gate flag in the sensor tuning file,
 * or 0 for the stages that read no flag at all. Those carry AR_ISP_STAGE_NO_BLOB
 * and their state is whatever the configure path produced: no gate state is
 * invented here that the library was not seen to decide.
 */

#ifndef AR_ISP_GATES_H
#define AR_ISP_GATES_H

/*
 * How a stage is turned off. A bit gate is read-modify-written and its
 * polarity says which way means running; a word gate is written zero to stop
 * the stage and a computed value to start it, so its "on" cannot be
 * synthesised from this table.
 */
#define AR_ISP_GATE_UNKNOWN\\t\\t0
#define AR_ISP_GATE_BIT_ENABLE\\t\\t1
#define AR_ISP_GATE_BIT_BYPASS\\t\\t2
#define AR_ISP_GATE_WORD\\t\\t3

/* Owns a DMA table fetch. */
#define AR_ISP_STAGE_FETCH\\t\\t0x01
/* Reads no tuning flag, so the file cannot say what it should be. */
#define AR_ISP_STAGE_NO_BLOB\\t\\t0x02

struct ar_isp_gate {
\\tu16 reg;
\\tu32 set_mask;
\\tu32 clear_mask;
\\tu8 kind;
};

struct ar_isp_stage {
\\tconst char *name;
\\tu16 bank;
\\tu32 blob_gate;
\\tconst struct ar_isp_gate *gates;
\\tu8 n_gates;
\\tu8 flags;
};
'''


def emit_header(stages, out):
    """The generated table, in the same envelope as the other vendor tables."""
    lines = [HEADER_DOC.replace('\\t', '\t')]

    kept = [st for st in stages if st['stage'] not in EXCLUDE and st['gates']]

    for st in kept:
        lines.append(f'static const struct ar_isp_gate '
                     f'ar_isp_gates_{st["stage"]}[] = {{')
        for g in st['gates']:
            if not g['mask']:
                kind = 'AR_ISP_GATE_WORD'
                source = 'word'
            else:
                kind = KINDS[g['code'] or g['paired']]
                source = 'code' if g['code'] else \
                    ('paired' if g['paired'] else 'unresolved')
            lines.append(f'\t{{ 0x{g["reg"]:04x}, 0x{g["set_mask"]:08x}, '
                         f'0x{g["clear_mask"]:08x}, {kind} }},\t/* {source} */')
        lines.append('};\n')

    lines.append('static const struct ar_isp_stage ar_isp_stages[] = {')
    for st in kept:
        flags = []
        if st['stage'] in FETCH:
            flags.append('AR_ISP_STAGE_FETCH')

        if st['blob_off'] is None:
            flags.append('AR_ISP_STAGE_NO_BLOB')

        off = st['blob_off'] or 0
        bank = st['bank'] if st['bank'] is not None else st['gates'][0]['reg']
        lines.append(f'\t{{ "{st["stage"]}", 0x{bank:04x}, 0x{off:06x}, '
                     f'ar_isp_gates_{st["stage"]}, '
                     f'{len(st["gates"])}, {" | ".join(flags) or "0"} }},')

    lines.append('};\n')
    lines.append('#endif /* AR_ISP_GATES_H */')

    Path(out).write_text('\n'.join(lines) + '\n')
    n = sum(len(st['gates']) for st in kept)
    print(f'{out}: {len(kept)} stages, {n} gates', file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lib', required=True, help='libmpp_service.so')
    ap.add_argument('--blob', help='vendor tuning file')
    ap.add_argument('--capture', help='slot-A register capture')
    ap.add_argument('--all', action='store_true',
                    help='also show writes that are not two-way gates')
    ap.add_argument('--header', metavar='PATH',
                    help='write the generated stage table here instead of '
                         'printing the report')
    args = ap.parse_args()

    lib = Library(args.lib)
    banks = load_banks()
    blob = Path(args.blob).read_bytes() if args.blob else None
    live = read_capture(args.capture) if args.capture else {}

    stages, missing, drift = collect(lib, banks, blob, live, args.all)

    if args.header:
        emit_header(stages, args.header)
        return

    report(stages, missing, drift)


if __name__ == '__main__':
    main()
