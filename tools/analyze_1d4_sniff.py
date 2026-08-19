#!/usr/bin/env python3
"""
Analyze known-good 0x1D4 sniff frames from serial logs.

Goals:
1) Verify CRC reproduction confidence from captured good frames.
2) Isolate active bitfields (bits that actually change).
3) Rank likely counter fields for replay experiments.
4) Build a CRC-valid, clock-continuous replay loop and emit firmware header.

Input formats accepted:
- Lines containing: [VCM-1D4-SNIFF] ... data=AA BB CC DD EE FF 11 22
- Any line containing at least 8 hex byte tokens (AA..FF); the last 8 are used.

Usage examples:
    python tools/analyze_1d4_sniff.py --input 1d4_sniff.log
    python tools/analyze_1d4_sniff.py --input 1d4_sniff.log --emit-template
    python tools/analyze_1d4_sniff.py --input 1d4_sniff.log --emit-loop
    python tools/analyze_1d4_sniff.py --input 1d4_sniff.log --emit-loop-header \
            --output-header include/Leaf1d4ReplaySeries.h
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import re
import statistics
from typing import Dict, List, Optional, Sequence, Tuple

HEX_BYTE_RE = re.compile(r"(?<![0-9A-Fa-f])([0-9A-Fa-f]{2})(?![0-9A-Fa-f])")
VCM_DATA_RE = re.compile(r"data\s*=\s*([0-9A-Fa-f]{2}(?:\s+[0-9A-Fa-f]{2}){7})")
HEX_GROUP_RE = re.compile(r"[0-9A-Fa-f]+")


@dataclasses.dataclass
class Frame:
    raw: bytes  # 8 bytes

    @property
    def payload7(self) -> bytes:
        return self.raw[:7]

    @property
    def crc_rx(self) -> int:
        return self.raw[7]

    @property
    def clock(self) -> int:
        # Intel bitfield 38|2 (in 8-byte frame)
        return extract_intel_unsigned(self.raw, 38, 2)


def extract_intel_unsigned(data: bytes, start_bit: int, bit_len: int) -> int:
    value = 0
    for i in range(bit_len):
        bit = start_bit + i
        byte_idx = bit // 8
        bit_idx = bit % 8
        bit_val = (data[byte_idx] >> bit_idx) & 0x01
        value |= (bit_val << i)
    return value


def crc8_msb(data: bytes, poly: int = 0x1D, init: int = 0xFF, xor_out: int = 0xFF) -> int:
    crc = init & 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            msb = (crc & 0x80) != 0
            crc = (crc << 1) & 0xFF
            if msb:
                crc ^= poly
    return crc ^ xor_out


def compute_1d4_crc_base(payload7: bytes) -> int:
    # Base model learned in firmware path: CRC8(MSB, poly=0x1D, init/xor=0xFF) over [idLo + payload7]
    p = bytes([0xD4]) + payload7
    return crc8_msb(p)


def parse_frame_from_line(line: str) -> Optional[Frame]:
    # Fast path for canonical "data=AA BB ... HH" formatting.
    m = VCM_DATA_RE.search(line)
    if m:
        parts = m.group(1).split()
        if len(parts) == 8:
            try:
                return Frame(bytes(int(p, 16) for p in parts))
            except ValueError:
                return None

    # Robust path for raw logs where spacing can collapse, e.g. last bytes as "01E4".
    if "data=" in line:
        data_tail = line.split("data=", 1)[1]
        groups = HEX_GROUP_RE.findall(data_tail)
        parsed: List[int] = []
        for g in groups:
            if len(g) < 2 or (len(g) % 2) != 0:
                continue
            for i in range(0, len(g), 2):
                parsed.append(int(g[i:i + 2], 16))
                if len(parsed) >= 8:
                    break
            if len(parsed) >= 8:
                break
        if len(parsed) >= 8:
            return Frame(bytes(parsed[:8]))

    all_hex = HEX_BYTE_RE.findall(line)
    if len(all_hex) < 8:
        return None

    try:
        vals = [int(x, 16) for x in all_hex[-8:]]
    except ValueError:
        return None

    return Frame(bytes(vals))


def load_frames(path: str) -> List[Frame]:
    frames: List[Frame] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            fr = parse_frame_from_line(line)
            if fr is not None and len(fr.raw) == 8:
                frames.append(fr)
    return frames


def derive_clock_residue(frames: Sequence[Frame]) -> Dict[int, int]:
    # For each clock bin, find residue maximizing match where
    # crc_rx == base_crc ^ residue
    residues: Dict[int, int] = {}
    by_clock: Dict[int, List[Frame]] = collections.defaultdict(list)
    for fr in frames:
        by_clock[fr.clock].append(fr)

    for clk, subset in by_clock.items():
        hist = collections.Counter()
        for fr in subset:
            base = compute_1d4_crc_base(fr.payload7)
            hist[fr.crc_rx ^ base] += 1
        residues[clk] = hist.most_common(1)[0][0]
    return residues


def crc_match_stats(frames: Sequence[Frame], residues: Dict[int, int]) -> Tuple[int, int, float]:
    ok = 0
    total = 0
    for fr in frames:
        clk = fr.clock
        if clk not in residues:
            continue
        calc = compute_1d4_crc_base(fr.payload7) ^ residues[clk]
        total += 1
        if calc == fr.crc_rx:
            ok += 1
    pct = (100.0 * ok / total) if total else 0.0
    return ok, total, pct


def bit_variance_report(frames: Sequence[Frame]) -> List[Tuple[int, int]]:
    # Returns list of (bit_index, transitions_count) for bits that vary across sample set.
    # bit_index is absolute in 64-bit frame, little-endian per byte bit order.
    if not frames:
        return []

    vals = [fr.raw for fr in frames]
    varying: List[Tuple[int, int]] = []
    for bit in range(64):
        byte_idx = bit // 8
        bit_idx = bit % 8
        seq = [((v[byte_idx] >> bit_idx) & 1) for v in vals]
        if any(x != seq[0] for x in seq[1:]):
            transitions = sum(1 for i in range(1, len(seq)) if seq[i] != seq[i - 1])
            varying.append((bit, transitions))
    varying.sort(key=lambda x: (-x[1], x[0]))
    return varying


def extract_field_value_intel(frames: Sequence[Frame], start: int, width: int) -> List[int]:
    return [extract_intel_unsigned(fr.raw, start, width) for fr in frames]


def counter_score(values: Sequence[int], width: int) -> float:
    if len(values) < 3:
        return 0.0
    mod = 1 << width
    good = 0
    total = 0
    for i in range(1, len(values)):
        prev = values[i - 1]
        cur = values[i]
        total += 1
        if cur == ((prev + 1) % mod):
            good += 1
    return good / total if total else 0.0


def counter_step_stats(values: Sequence[int], width: int) -> Tuple[int, int, int, int, float]:
    if len(values) < 2:
        return 0, 0, 0, 0, 0.0

    mod = 1 << width
    step_one = 0
    wraps = 0
    other = 0
    total = 0

    for i in range(1, len(values)):
        prev = values[i - 1]
        cur = values[i]
        expected = (prev + 1) % mod
        total += 1
        if cur == expected:
            step_one += 1
            if prev == mod - 1 and cur == 0:
                wraps += 1
        else:
            other += 1

    ratio = (step_one / total) if total else 0.0
    return step_one, wraps, other, total, ratio


def rank_counter_candidates(frames: Sequence[Frame], max_width: int = 8) -> List[Tuple[int, int, float, int]]:
    # Candidate tuple: (start_bit, width, score, unique_count)
    candidates: List[Tuple[int, int, float, int]] = []
    for width in range(1, max_width + 1):
        for start in range(0, 64 - width + 1):
            vals = extract_field_value_intel(frames, start, width)
            uniq = len(set(vals))
            if uniq < min(3, (1 << width)):
                continue
            score = counter_score(vals, width)
            if score >= 0.40:
                candidates.append((start, width, score, uniq))
    candidates.sort(key=lambda x: (-x[2], -x[3], x[1], x[0]))
    return candidates


def counter_continuity_report(frames: Sequence[Frame], max_width: int = 8) -> List[Tuple[int, int, float, int, int, int, int, int]]:
    # Candidate tuple: (start_bit, width, step_one_ratio, step_one_count, wrap_count, other_count, total_steps, unique_count)
    report: List[Tuple[int, int, float, int, int, int, int, int]] = []
    for width in range(1, max_width + 1):
        for start in range(0, 64 - width + 1):
            values = extract_field_value_intel(frames, start, width)
            unique_count = len(set(values))
            if unique_count < min(3, (1 << width)):
                continue
            step_one_count, wrap_count, other_count, total_steps, step_one_ratio = counter_step_stats(values, width)
            if total_steps > 0 and step_one_ratio >= 0.40:
                report.append((start, width, step_one_ratio, step_one_count, wrap_count, other_count, total_steps, unique_count))
    report.sort(key=lambda x: (-x[2], -x[7], -x[3], x[1], x[0]))
    return report


def format_bytes(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def emit_template(frames: Sequence[Frame], residues: Dict[int, int]) -> None:
    if not frames:
        return
    # Use most common frame bytes as template baseline.
    counter = collections.Counter(fr.raw for fr in frames)
    tpl = bytearray(counter.most_common(1)[0][0])

    clk = extract_intel_unsigned(bytes(tpl), 38, 2)
    base = compute_1d4_crc_base(bytes(tpl[:7]))
    tpl[7] = base ^ residues.get(clk, 0)

    print("\nTemplate candidate (most frequent observed frame with recalculated CRC):")
    print(f"  bytes: {format_bytes(bytes(tpl))}")
    print(f"  clock: {clk}")
    print(f"  crc  : 0x{tpl[7]:02X}")


def is_crc_valid(frame: Frame, residues: Dict[int, int]) -> bool:
    clk = frame.clock
    if clk not in residues:
        return False
    calc = compute_1d4_crc_base(frame.payload7) ^ residues[clk]
    return calc == frame.crc_rx


def trim_clock_continuous(frames: Sequence[Frame], residues: Dict[int, int]) -> List[Frame]:
    # Keep only CRC-valid frames first.
    valid = [fr for fr in frames if is_crc_valid(fr, residues)]
    if not valid:
        return []

    # Collapse adjacent duplicates of exactly same bytes.
    collapsed: List[Frame] = [valid[0]]
    for fr in valid[1:]:
        if fr.raw != collapsed[-1].raw:
            collapsed.append(fr)

    best: List[Frame] = []
    for i in range(len(collapsed)):
        run = [collapsed[i]]
        expect = (collapsed[i].clock + 1) & 0x03
        for j in range(i + 1, len(collapsed)):
            clk = collapsed[j].clock
            if clk == run[-1].clock:
                # Ignore repeated same-clock samples while seeking next step.
                continue
            if clk == expect:
                run.append(collapsed[j])
                expect = (expect + 1) & 0x03
            else:
                break
        if len(run) > len(best):
            best = run
    return best


def shortest_period(frames: Sequence[Frame]) -> int:
    n = len(frames)
    if n == 0:
        return 0
    for p in range(1, n + 1):
        ok = True
        for i in range(n):
            if frames[i].raw != frames[i % p].raw:
                ok = False
                break
        if ok:
            return p
    return n


def derive_loop_series(cont_run: Sequence[Frame], max_frames: int) -> List[Frame]:
    if not cont_run:
        return []
    period = shortest_period(cont_run)
    loop = list(cont_run[:period])
    if len(loop) > max_frames:
        loop = loop[:max_frames]
    return loop


def emit_loop(loop: Sequence[Frame]) -> None:
    print("\nClock-continuous replay loop:")
    if not loop:
        print("  none")
        return
    for idx, fr in enumerate(loop):
        print(f"  idx={idx:02d} clk={fr.clock} frame={format_bytes(fr.raw)}")


def emit_loop_header(loop: Sequence[Frame], out_path: str) -> None:
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("#pragma once\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write("namespace MetaSense::Leaf1d4Replay {\n\n")
        f.write(f"constexpr uint8_t kFrameCount = {len(loop)}U;\n\n")
        f.write("constexpr uint8_t kFrames[kFrameCount][8] = {\n")
        for fr in loop:
            row = ", ".join(f"0x{b:02X}U" for b in fr.raw)
            f.write(f"    {{{row}}},\n")
        f.write("};\n\n")
        f.write("} // namespace MetaSense::Leaf1d4Replay\n")


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze known-good 0x1D4 sniff frames")
    ap.add_argument("--input", required=True, help="Path to serial/sniff log file")
    ap.add_argument("--min-frames", type=int, default=50, help="Minimum frames required for meaningful stats")
    ap.add_argument("--emit-template", action="store_true", help="Emit a baseline replay template frame")
    ap.add_argument("--emit-loop", action="store_true", help="Emit clock-continuous, CRC-valid replay loop")
    ap.add_argument("--emit-loop-header", action="store_true", help="Write include/Leaf1d4ReplaySeries.h from replay loop")
    ap.add_argument("--output-header", default="include/Leaf1d4ReplaySeries.h", help="Path for generated replay header")
    ap.add_argument("--max-loop-frames", type=int, default=16, help="Upper bound for emitted loop frame count")
    args = ap.parse_args()

    frames = load_frames(args.input)
    print(f"Loaded frames: {len(frames)}")
    if len(frames) < args.min_frames:
        print(f"WARNING: fewer than {args.min_frames} frames; confidence may be low")

    if not frames:
        print("No parsable 8-byte frames found in input.")
        return 2

    residues = derive_clock_residue(frames)
    ok, total, pct = crc_match_stats(frames, residues)

    print("\nCRC model:")
    print("  base = CRC8-MSB(poly=0x1D, init=0xFF, xorOut=0xFF) over [0xD4 + payload7]")
    print("  final = base XOR residue_by_clock")
    print("  residue_by_clock:")
    for clk in sorted(residues):
        print(f"    clock {clk}: 0x{residues[clk]:02X}")
    print(f"  match: {ok}/{total} ({pct:.2f}%)")

    varying = bit_variance_report(frames)
    print("\nActive bits (frame bits that vary):")
    if not varying:
        print("  none")
    else:
        # Print top 32 by transition activity to keep output readable.
        for bit, trans in varying[:32]:
            print(f"  bit {bit:02d}: transitions={trans}")

    print("\nLikely counter-field candidates (Intel bit order, ranked):")
    cands = rank_counter_candidates(frames)
    if not cands:
        print("  none above threshold")
    else:
        for start, width, score, uniq in cands[:20]:
            print(f"  start={start:02d} width={width} score={score:.3f} unique={uniq}")

    print("\nCounter continuity check (does the field advance by +1 each frame?):")
    continuity = counter_continuity_report(frames)
    if not continuity:
        print("  none above threshold")
    else:
        for start, width, ratio, step_one_count, wrap_count, other_count, total_steps, uniq in continuity[:20]:
            verdict = "YES" if ratio == 1.0 else "NO"
            print(f"  start={start:02d} width={width} step1={step_one_count}/{total_steps} ({ratio:.3f}) wraps={wrap_count} other={other_count} unique={uniq} {verdict}")

    # Known expected clock field reminder for this protocol.
    clock_values = extract_field_value_intel(frames, 38, 2)
    print("\nKnown field sanity checks:")
    print(f"  clock(38|2) unique={sorted(set(clock_values))} avg={statistics.mean(clock_values):.3f}")

    if args.emit_template:
        emit_template(frames, residues)

    if args.emit_loop or args.emit_loop_header:
        continuous = trim_clock_continuous(frames, residues)
        loop = derive_loop_series(continuous, max_frames=args.max_loop_frames)
        print("\nLoop derivation:")
        print(f"  crc_valid_frames={sum(1 for fr in frames if is_crc_valid(fr, residues))}")
        print(f"  longest_clock_continuous_run={len(continuous)}")
        print(f"  loop_period={len(loop)}")
        emit_loop(loop)
        if args.emit_loop_header:
            emit_loop_header(loop, args.output_header)
            print(f"\nWrote replay header: {args.output_header}")

    print("\nNext-step recommendation:")
    print("  1) Freeze all non-essential payload bits from a known-good template.")
    print("  2) Prefer replay-loop mode generated from contiguous valid captures.")
    print("  3) Introduce torque field changes last, one variable at a time.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
