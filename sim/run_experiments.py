#!/usr/bin/env python3
"""Runs every Monte Carlo configuration in parallel shards and prints Markdown tables.

Usage: python3 sim/run_experiments.py [--scale 0.1]
--scale shrinks every shape count for a quick pass.
"""
import argparse
import os
import re
import subprocess
import sys
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(ROOT, "build", "huddle_sim")
SEED = 1

# (section, label, binary, shapes, extra args)
CONFIGS = [
    ("sync", "Paper model (per-step exchange)", "huddle_sim", 40000, ["--verify-holes"]),
    ("control", "Negative control: Algorithm 6 disabled", "huddle_sim", 2000, ["--no-delay"]),
    ("ambiguity", "Midpoint rounded up instead of down", "huddle_sim_roundup", 10000, []),
    ("ambiguity", "Remark 1 special flank signal removed", "huddle_sim_no_remark1", 10000, []),
    ("ambiguity", "Same-step dockers blind to each other at init", "huddle_sim", 10000,
     ["--blind-simultaneous"]),
    ("lossy", "drop 10%, delay <= 1, unknown = free", "huddle_sim", 5000, ["--lossy", "0.1", "1", "free"]),
    ("lossy", "drop 30%, delay <= 3, unknown = free", "huddle_sim", 5000, ["--lossy", "0.3", "3", "free"]),
    ("lossy", "drop 60%, delay <= 6, unknown = free", "huddle_sim", 5000, ["--lossy", "0.6", "6", "free"]),
    ("lossy", "drop 90%, delay <= 8, unknown = free", "huddle_sim", 5000, ["--lossy", "0.9", "8", "free"]),
    ("lossy", "drop 10%, delay <= 1, unknown = null", "huddle_sim", 5000, ["--lossy", "0.1", "1", "null"]),
    ("lossy", "drop 30%, delay <= 3, unknown = null", "huddle_sim", 5000, ["--lossy", "0.3", "3", "null"]),
    ("latency", "docking latency 1 step, no recheck", "huddle_sim", 5000, ["--latency", "1", "off"]),
    ("latency", "docking latency 3 steps, no recheck", "huddle_sim", 5000, ["--latency", "3", "off"]),
    ("latency", "latency 3 + drop 30%, delay <= 3, unknown = free", "huddle_sim", 5000,
     ["--latency", "3", "off", "--lossy", "0.3", "3", "free"]),
]

FIELDS = ("trials complete unreachable hole deadlock misposition off_target step_limit steps "
          "aborted raised withdrawn suppressed size_sum size_max shapes").split()


def run_shard(binary, offset, shapes, extra):
    cmd = [os.path.join(ROOT, "build", binary), "--seed", str(SEED), "--offset", str(offset),
           "--shapes", str(shapes)] + extra
    out = subprocess.run(cmd, capture_output=True, text=True)
    if out.returncode not in (0, 1):
        sys.exit(f"{' '.join(cmd)} exited {out.returncode}: {out.stderr}")
    rows, failures = [], []
    for line in out.stdout.splitlines():
        if line.startswith("RESULT"):
            k = int(re.search(r" k=(\d+)", line).group(1))
            values = {f: int(v) for f, v in re.findall(r" (\w+)=(\d+)", line) if f in FIELDS}
            rows.append((k, values))
        elif line.startswith("FAIL"):
            failures.append(line)
    return rows, failures


def run_config(binary, shapes, extra, workers):
    per = -(-shapes // workers)
    shards = [(i * per, min(per, shapes - i * per)) for i in range(workers) if i * per < shapes]
    totals = defaultdict(lambda: defaultdict(int))
    failures = []
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for rows, fails in pool.map(lambda s: run_shard(binary, s[0], s[1], extra), shards):
            failures += fails
            for k, values in rows:
                for f, v in values.items():
                    totals[k][f] = max(totals[k][f], v) if f == "size_max" else totals[k][f] + v
    return totals, failures


def pct(part, whole):
    return f"{100.0 * part / whole:.2f}%" if whole else "-"


def summarize(label, totals, per_k):
    rows = []
    groups = [(str(k), [totals[k]]) for k in sorted(totals)] if per_k else [
        ("1-4", [totals[k] for k in sorted(totals)])]
    for k_label, group in groups:
        t = defaultdict(int)
        for g in group:
            for f, v in g.items():
                t[f] = max(t[f], v) if f == "size_max" else t[f] + v
        other = t["misposition"] + t["off_target"] + t["step_limit"]
        mean_steps = t["steps"] / t["complete"] if t["complete"] else 0
        rows.append(f"| {label} | {k_label} | {t['trials']:,} | {t['complete']:,} "
                    f"({pct(t['complete'], t['trials'])}) | {t['unreachable']:,} | {t['hole']:,} | "
                    f"{t['deadlock']:,} | {other:,} | {mean_steps:.1f} | "
                    f"{pct(t['suppressed'], t['raised'] + t['suppressed'])} | {t['withdrawn']:,} |")
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--workers", type=int, default=os.cpu_count() or 4)
    args = parser.parse_args()

    header = ("| Configuration | k | Trials | Complete | Unreachable | Hole | Deadlock | Other | "
              "Mean steps | Signals suppressed | Signals withdrawn |\n"
              "|---|---|---|---|---|---|---|---|---|---|---|")
    section_rows = defaultdict(list)
    shape_note = None
    for section, label, binary, shapes, extra in CONFIGS:
        shapes = max(1, int(shapes * args.scale))
        totals, failures = run_config(binary, shapes, extra, args.workers)
        section_rows[section] += summarize(label, totals, per_k=section == "sync")
        if section == "sync":
            any_k = totals[min(totals)]
            shape_note = (f"{shapes:,} shapes, mean size {any_k['size_sum'] / shapes:.1f} cells, "
                          f"largest {any_k['size_max']} cells")
        print(f"done: {label} ({len(failures)} failures)", file=sys.stderr)
        for line in failures[:3]:
            print(f"  {line}", file=sys.stderr)

    print(f"Seed {SEED}. Sync run: {shape_note}.\n")
    for section in ("sync", "control", "ambiguity", "lossy", "latency"):
        print(header)
        print("\n".join(section_rows[section]))
        print()


if __name__ == "__main__":
    main()
