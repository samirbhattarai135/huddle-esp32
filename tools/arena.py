#!/usr/bin/env python3
"""Arena: emulates the physical world for a Huddle demo on ESP32 boards.

The arena owns ground truth (which board sits on which cell). Each step it
reads every board's signaled walls, picks a signaled opening at random,
"docks" the next free board there by telling the board and its new neighbors
which wall now touches which MAC address, then checks that the board computed
its own position correctly and that no opening became unreachable
(Definition 1). The boards decide everything else themselves and talk to each
other over ESP-NOW.

Hardware:  python3 tools/arena.py --ports /dev/cu.usbserial-1 /dev/cu.usbserial-2 ...
Host only: python3 tools/arena.py --fake 7      (runs build/fake_node, same node.c)
           python3 tools/arena.py --fake 40 --random --drop 0.3
"""
import argparse
import os
import queue
import secrets
import random
import subprocess
import sys
import threading
import time

UNIT = [(0, 1), (-1, 1), (-1, 0), (0, -1), (1, -1), (1, 0)]
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FAKE_NODE = os.path.join(ROOT, "build", "fake_node")

# Seven cells, two columns of growth plus a right-hand pair: the root must
# grow its own column, then seed both neighbors (cf. Fig. 2 of the paper).
DEMO_SHAPE = [(0, 0), (0, 1), (0, 2), (-1, 1), (-1, 2), (1, -1), (1, 0)]


def add(cell, wall):
    return (cell[0] + UNIT[wall][0], cell[1] + UNIT[wall][1])


class Board:
    """One board: a line-oriented link (serial port or fake_node process)."""

    def __init__(self, name, write_line, read_line):
        self.name = name
        self._write_line = write_line
        self.lines = queue.Queue()
        self.mac = None
        self.signals = 0
        self.init = None
        threading.Thread(target=self._reader, args=(read_line,), daemon=True).start()

    def _reader(self, read_line):
        while True:
            line = read_line()
            if line is None:
                return
            if line:
                self.lines.put(line)

    def send(self, line):
        self._write_line(line + "\n")

    def pump(self, until=None, timeout=5.0, radio=None):
        """Handles incoming lines; returns the first one starting with `until`."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if until:
                    raise TimeoutError(f"{self.name}: no '{until}' within {timeout}s")
                return None
            try:
                line = self.lines.get(timeout=min(remaining, 0.05) if not until else remaining)
            except queue.Empty:
                if until:
                    continue
                return None
            if line.startswith("SIG "):
                self.signals = int(line.split()[1])
            elif line.startswith("INIT "):
                parts = line.split()
                self.init = (int(parts[1]), int(parts[2]))
                print(f"  {self.name}: {line}")
            elif line.startswith("MAC "):
                self.mac = line.split()[1]
            elif line.startswith("TX ") and radio is not None:
                _, receiver, payload = line.split()
                radio.append((self.mac, receiver, payload))
            elif line.startswith("ERR"):
                print(f"  {self.name}: {line}")
            if until and line.startswith(until):
                return line


def open_fake(index):
    mac = f"02:00:00:00:{index >> 8:02x}:{index & 0xFF:02x}"
    proc = subprocess.Popen([FAKE_NODE, mac], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            text=True, bufsize=1)

    def write(line):
        proc.stdin.write(line)
        proc.stdin.flush()

    def read():
        line = proc.stdout.readline()
        return line.strip() if line else None

    return Board(f"node{index}", write, read)


def open_serial(port):
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial is required for hardware runs: python3 -m pip install pyserial")
    link = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(2.0)  # opening the port resets most ESP32 dev boards

    def write(line):
        link.write(line.encode())

    def read():
        return link.readline().decode(errors="replace").strip()

    return Board(port, write, read)


def random_shape(size, rng):
    """Random hole-free shape grown from (0, 0), the root's cell."""
    cells = {(0, 0)}
    while len(cells) < size:
        base = rng.choice(sorted(cells))
        cell = add(base, rng.randrange(6))
        if cell in cells or max(abs(cell[0]), abs(cell[1])) > 15:
            continue
        if not has_hole(cells | {cell}):
            cells.add(cell)
    return sorted(cells)


def has_hole(occupied, shape=None):
    """True if some empty cell (of `shape`, if given) is enclosed by `occupied`."""
    cells = set(occupied) | set(shape or ())
    ps = [c[0] for c in cells]
    qs = [c[1] for c in cells]
    lo_p, hi_p, lo_q, hi_q = min(ps) - 1, max(ps) + 1, min(qs) - 1, max(qs) + 1
    outside = {(lo_p, lo_q)}
    stack = [(lo_p, lo_q)]
    while stack:
        cell = stack.pop()
        for w in range(6):
            n = add(cell, w)
            if lo_p <= n[0] <= hi_p and lo_q <= n[1] <= hi_q and n not in occupied and n not in outside:
                outside.add(n)
                stack.append(n)
    candidates = shape if shape is not None else [
        (p, q) for p in range(lo_p, hi_p + 1) for q in range(lo_q, hi_q + 1)]
    return any(c not in occupied and c not in outside for c in candidates)


def render(shape, placed):
    xs = [3 * p for p, _ in shape]
    ys = [2 * q + p for p, q in shape]
    width, height = max(xs) - min(xs) + 1, max(ys) - min(ys) + 1
    canvas = [[" "] * width for _ in range(height)]
    for p, q in shape:
        mark = "." if (p, q) not in placed else ("@" if (p, q) == (0, 0) else "o")
        canvas[max(ys) - (2 * q + p)][3 * p - min(xs)] = mark
    for row in canvas:
        print("    " + "".join(row).rstrip())


class Arena:
    def __init__(self, boards, shape, args, rng):
        self.boards = boards
        self.shape = shape
        self.shape_set = set(shape)
        self.args = args
        self.rng = rng
        self.fake = args.fake is not None
        self.placed = {}  # cell -> Board
        self.session_key = secrets.token_bytes(16).hex()  # fresh per run, never stored

    def settle(self):
        """Lets signals and status words propagate before reading signals."""
        if not self.fake:
            end = time.monotonic() + self.args.settle
            while time.monotonic() < end:
                for board in self.boards:
                    board.pump(timeout=0.02)
            return
        for _ in range(self.args.ticks):
            radio = []
            for board in self.placed.values():
                board.send("TICK")
            for board in self.placed.values():
                board.pump(until="TICKED", radio=radio)
            by_mac = {b.mac: b for b in self.boards}
            for sender, receiver, payload in radio:
                if self.rng.random() >= self.args.drop:
                    by_mac[receiver].send(f"RX {sender} {payload}")

    def dock(self, cells):
        free = [b for b in self.boards if b not in self.placed.values()]
        new = dict(zip(cells, free))
        self.placed.update(new)
        # Docking boards learn all their occupied walls before any neighbor
        # starts sending to them.
        for cell, board in new.items():
            for w in range(6):
                if add(cell, w) in self.placed:
                    self.attach(board, w, self.placed[add(cell, w)])
        for cell, board in new.items():
            for w in range(6):
                neighbor = add(cell, w)
                if neighbor in self.placed and neighbor not in new:
                    self.attach(self.placed[neighbor], (w + 3) % 6, board)
        return new

    def attach(self, board, wall, other):
        board.send(f"ATTACH {wall} {other.mac}")
        board.pump(until="OK ATTACH")

    def check_reachability(self):
        for cell in self.shape:
            if cell in self.placed:
                continue
            degree = sum(add(cell, w) in self.placed for w in range(6))
            if degree > 3:
                return f"opening {cell} has {degree} attached neighbors"
        if has_hole(set(self.placed), self.shape):
            return "the assembly encloses an empty target cell"
        return None

    def run(self):
        for board in self.boards:
            board.send("RESET")
            board.pump(until="OK RESET")
            board.send(f"KEY {self.session_key}")
            board.pump(until="OK KEY")
            board.send("WHO")
            board.pump(until="MAC")
            for p, q in self.shape:
                board.send(f"CELL {p} {q}")
            board.send("SHAPE_DONE")
            board.pump(until="OK SHAPE")
        root = self.boards[0]
        self.placed[(0, 0)] = root
        root.send("ROOT")
        root.pump(until="INIT")

        for step in range(1, 10 * len(self.shape)):
            if len(self.placed) == len(self.shape):
                print(f"Complete: {len(self.placed)} boards in {step - 1} steps.")
                return 0
            if len(self.placed) == len(self.boards):
                print("Out of boards before the shape was complete.")
                return 1
            self.settle()
            openings = set()
            for cell, board in self.placed.items():
                for w in range(6):
                    if board.signals >> w & 1:
                        target = add(cell, w)
                        if target not in self.shape_set or target in self.placed:
                            print(f"FAIL: {board.name} at {cell} signals wall {w} toward {target}")
                            return 1
                        openings.add(target)
            if not openings:
                print("FAIL: deadlock, no board is signaling")
                render(self.shape, self.placed)
                return 1
            picks = self.rng.sample(sorted(openings), min(self.args.k, len(openings)))
            new = self.dock(picks)
            for cell, board in new.items():
                self.wait_for_init(board)
                if board.init != cell:
                    print(f"FAIL: {board.name} docked at {cell} but computed {board.init}")
                    return 1
            print(f"step {step}: docked at {', '.join(map(str, picks))}")
            problem = self.check_reachability()
            if problem:
                print(f"FAIL: {problem}")
                return 1
            if not self.fake or len(self.shape) <= 12:
                render(self.shape, self.placed)
        print("FAIL: step limit")
        return 1

    def wait_for_init(self, board):
        if not self.fake:
            board.pump(until="INIT", timeout=10.0)
            return
        for _ in range(50):
            if board.init:
                return
            self.settle()
        raise TimeoutError(f"{board.name} never initialized")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--ports", nargs="+", help="serial ports, root board first")
    mode.add_argument("--fake", type=int, metavar="N", help="run N fake_node processes instead")
    parser.add_argument("--shape", help='target cells "p,q p,q ..." including 0,0 (default: 7-cell demo)')
    parser.add_argument("--random", action="store_true", help="random hole-free shape, one cell per board")
    parser.add_argument("--k", type=int, default=1, help="dockings per step")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--settle", type=float, default=1.0, help="hardware: seconds between steps")
    parser.add_argument("--ticks", type=int, default=2, help="fake: node steps between dockings")
    parser.add_argument("--drop", type=float, default=0.0, help="fake: radio frame drop probability")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    count = args.fake if args.fake is not None else len(args.ports)
    if args.shape:
        shape = [tuple(int(v) for v in cell.split(",")) for cell in args.shape.split()]
    elif args.random:
        shape = random_shape(count, rng)
    else:
        shape = DEMO_SHAPE
    if (0, 0) not in shape or has_hole(set(shape)):
        sys.exit("shape must contain the root cell 0,0 and have no holes")
    if count < len(shape):
        sys.exit(f"{len(shape)} cells need {len(shape)} boards, got {count}")

    if args.fake is not None and not os.path.exists(FAKE_NODE):
        sys.exit("build the host node first: make build/fake_node")
    boards = [open_fake(i) for i in range(count)] if args.fake is not None else [
        open_serial(p) for p in args.ports]
    print(f"Target: {len(shape)} cells, {count} boards.")
    render(shape, {})
    return Arena(boards, shape, args, rng).run()


if __name__ == "__main__":
    sys.exit(main())
