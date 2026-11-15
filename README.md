# huddle-esp32

A from-scratch reimplementation of **Huddle** (Chin, Meng, Chen, Bassett, Ivanov,
["Huddle: Parallel Shape Assembly using Decentralized, Minimalistic Robots"](https://www.amazon.science/publications/huddle-parallel-shape-assembly-using-decentralized-minimalistic-robots),
Amazon Robotics, DARS 2026), plus a check of what happens when the neighbor
link is not the clean per-step exchange the paper assumes, plus firmware that
runs the algorithm on ESP32 boards talking over ESP-NOW.

Status: the C implementation and the Monte Carlo results below are real and
reproducible with the commands in this file. The firmware compiles and its
logic has been run on the host. **It has not been flashed or run on physical
boards yet; the hardware demo is pending.**

A diagram-first walk-through of the design is in
[`docs/architecture.html`](docs/architecture.html); GitHub shows it as source,
so download it or open it from a local clone.

## The paper in one paragraph

Hexagonal robots build a target shape S, given as a set of hex cells with no
holes, starting from one root robot. In-assembly robots "signal" on some of
their walls, and passing-by robots dock at signaled openings. Each robot knows
only its own walls (null, free or occupied) and, once per step, two integers
from each attached neighbor: a 16-bit wall-status word and a 32-bit word
holding its position and the unit vector of the wall facing you. Algorithms 1-4
run once at docking and fix the robot's role (which flanks it nucleates) and
growth direction. Algorithms 5-6 run every step and choose the walls to signal:
fore-aft walls first, flanks only after, and a fore-aft signal is delayed
while the neighbor in the parent column still has that wall free. The paper
proves that every opening stays reachable (an empty target cell never has more
than 3 attached neighbors) and the assembly never encloses a hole. It reports
160,000 Monte Carlo trials (40,000 random shapes, 1-4 simultaneous attachments
per step, up to more than 250 robots), all successful, and a 107-robot
Isaac Sim run. Real robots are listed as future work.

## What this repository adds

1. `core/`: Algorithms 1-6 in portable C99, about 300 lines. Pure functions
   over a small per-robot struct; no heap, no globals. The same file builds for
   the host and for the ESP32.
2. `sim/`: a Monte Carlo harness that reproduces the paper's correctness
   experiment and checks Definitions 1 and 2 after every step.
3. A study of the assumption the paper makes silently: that every robot
   receives every neighbor's status every step. The harness drops and delays
   status messages, and separately lets docking robots arrive several steps
   after they picked a signal. It also replays old status words and injects
   forged ones, to show what the neighbor link has to defend against.
4. `firmware/`: an Arduino sketch where each ESP32 is one in-assembly robot.
   ESP-NOW frames carry the paper's two integers plus a sequence number and a
   SipHash-2-4 tag under a per-run session key. Physical docking is emulated by
   `tools/arena.py`, which drives 6-7 boards from one laptop.

## Results

All numbers come from `make experiments` (seed 1). Every configuration is
4 trials per shape (k = 1, 2, 3, 4 attachments per step). Shapes are random
hole-free hex shapes of 10 to 300 cells.

Columns: *Unreachable* is a Definition 1 violation (some empty target cell
adjacent to the assembly has 4 or more attached neighbors after a step).
*Hole* is a Definition 2 violation. *Deadlock* means nothing signals but the
shape is not complete. *Other* counts a robot computing the wrong position,
signaling toward a non-target cell, or hitting the step limit. A trial stops
at its first violation. *Mean steps* is over completed trials. *Signals
suppressed* is the share of wall-steps where a robot held a signal back only
because its view of a neighbor was stale. *Signals withdrawn* counts a signal
that was on in one step and off in the next while its wall was still free.

Seed 1. Sync run: 40,000 shapes, mean size 155.4 cells, largest 300 cells.

| Configuration | k | Trials | Complete | Unreachable | Hole | Deadlock | Other | Mean steps | Signals suppressed | Signals withdrawn |
|---|---|---|---|---|---|---|---|---|---|---|
| Paper model (per-step exchange) | 1 | 40,000 | 40,000 (100.00%) | 0 | 0 | 0 | 0 | 154.4 | 0.00% | 0 |
| Paper model (per-step exchange) | 2 | 40,000 | 40,000 (100.00%) | 0 | 0 | 0 | 0 | 79.0 | 0.00% | 0 |
| Paper model (per-step exchange) | 3 | 40,000 | 40,000 (100.00%) | 0 | 0 | 0 | 0 | 55.1 | 0.00% | 0 |
| Paper model (per-step exchange) | 4 | 40,000 | 40,000 (100.00%) | 0 | 0 | 0 | 0 | 44.0 | 0.00% | 0 |

| Configuration | k | Trials | Complete | Unreachable | Hole | Deadlock | Other | Mean steps | Signals suppressed | Signals withdrawn |
|---|---|---|---|---|---|---|---|---|---|---|
| Negative control: Algorithm 6 disabled | 1-4 | 8,000 | 1,641 (20.51%) | 6,359 | 0 | 0 | 0 | 24.5 | 0.00% | 0 |

| Configuration | k | Trials | Complete | Unreachable | Hole | Deadlock | Other | Mean steps | Signals suppressed | Signals withdrawn |
|---|---|---|---|---|---|---|---|---|---|---|
| Midpoint rounded up instead of down | 1-4 | 40,000 | 40,000 (100.00%) | 0 | 0 | 0 | 0 | 82.6 | 0.00% | 0 |
| Remark 1 special flank signal removed | 1-4 | 40,000 | 40,000 (100.00%) | 0 | 0 | 0 | 0 | 82.6 | 0.00% | 0 |
| Same-step dockers blind to each other at init | 1-4 | 40,000 | 40,000 (100.00%) | 0 | 0 | 0 | 0 | 82.6 | 0.00% | 0 |

| Configuration | k | Trials | Complete | Unreachable | Hole | Deadlock | Other | Mean steps | Signals suppressed | Signals withdrawn |
|---|---|---|---|---|---|---|---|---|---|---|
| drop 10%, delay <= 1, unknown = free | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 7.74% | 0 |
| drop 30%, delay <= 3, unknown = free | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 16.54% | 0 |
| drop 60%, delay <= 6, unknown = free | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 25.90% | 0 |
| drop 90%, delay <= 8, unknown = free | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 35.88% | 0 |
| drop 10%, delay <= 1, unknown = null | 1-4 | 20,000 | 6,775 (33.88%) | 13,225 | 0 | 0 | 0 | 43.1 | 0.92% | 45,607 |
| drop 30%, delay <= 3, unknown = null | 1-4 | 20,000 | 4,997 (24.98%) | 15,003 | 0 | 0 | 0 | 29.7 | 1.22% | 30,028 |

| Configuration | k | Trials | Complete | Unreachable | Hole | Deadlock | Other | Mean steps | Signals suppressed | Signals withdrawn |
|---|---|---|---|---|---|---|---|---|---|---|
| docking latency 1 step, no recheck | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 95.6 | 0.00% | 0 |
| docking latency 3 steps, no recheck | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 138.8 | 0.00% | 0 |
| latency 3 + drop 30%, delay <= 3, unknown = free | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 138.8 | 13.83% | 0 |

| Configuration | k | Trials | Complete | Unreachable | Hole | Deadlock | Other | Mean steps | Signals suppressed | Signals withdrawn |
|---|---|---|---|---|---|---|---|---|---|---|
| replay 50% of link-steps, overwrite | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 0.00% | 731,873 |
| replay 50% of link-steps, monotone merge | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 0.00% | 0 |
| replay 100% of link-steps, overwrite | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 0.00% | 618,832 |
| replay 100% of link-steps, monotone merge | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 0.00% | 0 |
| drop 30%, delay <= 3, replay 50%, overwrite | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 19.70% | 468,254 |
| drop 30%, delay <= 3, replay 50%, monotone merge | 1-4 | 20,000 | 20,000 (100.00%) | 0 | 0 | 0 | 0 | 82.7 | 11.35% | 0 |

| Configuration | k | Trials | Complete | Unreachable | Hole | Deadlock | Other | Mean steps | Signals suppressed | Signals withdrawn |
|---|---|---|---|---|---|---|---|---|---|---|
| forged frames 0.1% of link-steps and inits, no authentication | 1-4 | 20,000 | 17,072 (85.36%) | 252 | 0 | 0 | 2,676 | 78.1 | 0.00% | 852 |
| forged frames 1% of link-steps and inits, no authentication | 1-4 | 20,000 | 5,705 (28.52%) | 891 | 0 | 7 | 13,397 | 46.6 | 0.00% | 3,440 |
| forged frames 1%, monotone merge, no authentication | 1-4 | 20,000 | 5,681 (28.41%) | 1,089 | 0 | 7 | 13,223 | 44.8 | 0.00% | 80 |

Replay rows (5,000 shapes, k = 1..4, seed 1): in each link step, with the
stated probability, the receiver is handed an older status word that the
neighbor really sent. "Overwrite" replaces the robot's view with it, as a
receiver with no sequence check would. "Monotone merge" applies
`huddle_merge_status_word`. Forgery rows (same shapes): each link step has the
stated chance of a false status word (every free wall claimed occupied, or
everything null, half each), and each initialization has the same chance of a
false pose word (wrong sender cell and wall within 3 cells). In *Other*, the
failures are mostly robots that computed the wrong cell from a false pose
word. Nothing authenticates these frames.

### What the tables say

- **Paper model: 160,000 / 160,000 trials complete**, no reachability or hole
  violation, matching the paper. In the main run the Euler-characteristic hole
  check was cross-checked against a flood fill after every step of every
  trial, and every robot's wall status was compared with the ground truth.
- **The checker catches failures.** With Algorithm 6 switched off, 79% of
  trials hit an unreachable opening (usually within the first few dozen
  steps). This is the failure in Fig. 3b of the paper, and it shows the harness
  is not passing everything by construction.
- **The ambiguous readings I tested do not matter** (each 40,000 / 40,000):
  rounding the midpoint up instead of down, deleting the Remark 1 special
  flank signal (the paper says it is optional; that holds), and having robots
  that dock in the same step not see each other at initialization.
- **Lossy links are safe if "never heard from" is treated as "all walls
  free".** Wall statuses only move from free to occupied, and Algorithm 6 only
  delays a signal when it sees "free". So a stale status word can only cause
  extra delay, never a missing delay. The one dangerous state is a neighbor
  that has docked but whose first message has not arrived. If the robot
  assumes that neighbor's walls are free, every configuration passes, up to
  90% frame loss with up to 8 steps of delay (20,000 / 20,000). If it assumes
  the walls are null (no information, so no delay), 66% of trials fail at
  just 10% loss and 75% fail at 30% loss. The firmware uses the safe default.
- **The cost of loss is suppressed signals, not steps, in this model.** At 90%
  loss about 36% of signal opportunities are held back. Completion steps do
  not change because the simulator always has a robot ready for any opening and
  openings outnumber k. With a limited supply of passing robots this would
  slow assembly down. I have not modeled that.
- **No signal was ever withdrawn in any safe configuration.** Once a wall
  starts signaling it keeps signaling until it is occupied. Because of that, a
  robot that commits to a signal and docks 1 or 3 steps later never lands
  somewhere it should not, even without rechecking the signal on arrival
  (20,000 / 20,000 each, also when combined with 30% loss). This is an
  observation from these runs, not a proof.

- **Replayed old status words do not break assembly, with or without the
  merge** (20,000 / 20,000 in every replay row, including 100% replay and
  replay on top of 30% loss). This is the argument above, now measured: an old
  word is pointwise "more free", so it can only add delay. Without the merge a
  replay makes signals flicker (731,873 withdrawn signals at 50% replay, versus
  0 with the merge). The merge makes the view a ratchet and removes that
  flicker. In the lossy row the merge also lowers the suppressed share (11.35%
  against 19.70%), because a replayed word can be newer than the last one that
  got through. With the merge on and no replay or forgery the results are
  identical to without it (checked on 2,000 shapes, sync and lossy).
- **Forged frames, with no authentication, break assembly.** At 0.1% of link
  steps and initializations, 14.64% of trials fail; at 1%, 71.48%. Unreachable
  openings come from a false "occupied" or "null" word that removes a delay
  (252 and 891 trials). Most of the rest are false pose words that put a robot
  in the wrong cell (the 2,676 and 13,397 in *Other*). The merge does not fix
  forged status: a false "occupied" is the direction it accepts, and it then
  sticks (1,089 unreachable trials against 891). The merge is for replay; the
  tag is for forgery.

One finished assembly (94 robots, k = 2):

```
Final assembly, shape 0 (94 robots, k=2):
                   o                
                                    
                   o                
                      L     o       
                         o     o    
                      o     R       
             o     o     R          
                      R             
             o     o           o    
          o     o     o     o       
       o     o     o     o     o    
          o     o     o     o       
       o     o     o     o     o    
    o     L     o     o     o       
       L     o     o     o     o    
    o     o     L     o     o     o 
       o     L     B     o     R    
    o     o     o     o     R       
       o     o     o     R     o    
          o     o     R     o     o 
             o     o     o     R    
          o     o     o     o       
       o     o     o     o     o    
          L     o     o     o       
       L     o     @     o     o    
    o                 o             
                   o                
    @ root, L/R/B left/right/both-flank nucleus, o regular, . unfilled target
```

## Reproduce

Requirements: a C99 compiler, Python 3, and for the firmware `arduino-cli`
with the `esp32:esp32` core (built here with 3.3.10).

```sh
make test          # unit tests (core, SipHash vectors, frame auth), checker self-test,
                   # 1,200-trial sim, negative control, replay and forgery runs,
                   # -O0/-O2 agreement, host run of the firmware logic
make experiments   # all tables above; a few minutes on an 8-core laptop
make firmware      # arduino-cli compile --fqbn esp32:esp32:esp32 --library core firmware/huddle_node

# one-off runs
./build/huddle_sim --shapes 1000                       # paper model, k = 1..4
./build/huddle_sim --shapes 1000 --lossy 0.3 3 free    # drop, max delay, unknown policy
./build/huddle_sim --shapes 1000 --latency 3 off       # docking latency, recheck on/off
./build/huddle_sim --shapes 1000 --no-delay            # negative control
./build/huddle_sim --shapes 1000 --replay 0.5 --merge  # old words replayed, monotone merge
./build/huddle_sim --shapes 1000 --forge 0.01          # false status and pose words
./build/huddle_sim --shapes 40 --k 2 --render --min-size 60 --max-size 110
```

`huddle_sim` prints one `FAIL` line per failed trial (shape index, k, step)
and exits non-zero if any trial failed. `--render` draws the first failure,
or one finished assembly; `HUDDLE_DUMP=1` adds every robot's role and wall
status.

## Firmware

`firmware/huddle_node/` is one robot. `node.c` holds the arena protocol, the
neighbor link and the Huddle state, with no Arduino code in it, so the same
file also runs on the host as `build/fake_node`. `huddle_node.ino` adds
ESP-NOW, USB serial and the LED.

- **Wire format.** One 18-byte ESP-NOW unicast per attached neighbor per step
  (every 100 ms), all fields little-endian:

  | Bytes | Field |
  |---|---|
  | 0-1 | `uint16` wall-status word: 2 bits per wall, 0 null, 1 free, 2 occupied; wall w at bits 2w..2w+1 |
  | 2-5 | `uint32` pose word: bits 0-11 p, 12-23 q, 24-25 du, 26-27 dv, all two's complement, where (du, dv) is the unit vector of the sender's wall that faces the receiver; bits 28-31 zero |
  | 6-9 | `uint32` sender sequence number, first frame is 1, one counter per sender across all its peers |
  | 10-17 | `uint64` tag = SipHash-2-4(session key, sender MAC \|\| receiver MAC \|\| bytes 6-9 \|\| bytes 0-5) |

  Bytes 0-5 are the paper's two integers, unchanged. Nothing else goes over
  the radio. A receiver drops a frame unless all of these hold: it has a
  session key; the pose word decodes to one of the six walls; that wall has an
  attached neighbor whose bound MAC equals the source MAC; the tag verifies
  (compared in constant time); and the sequence number is greater than the last
  one accepted on that wall. A tag covers both MACs, so a frame cannot be
  reflected or redirected to another board. `node.rejected` counts drops.
- **Monotone merge.** A robot keeps, per wall of each neighbor, the highest
  status it has heard: only free -> occupied is accepted, and a received null
  or invalid field is ignored (`huddle_merge_status_word` in `core/`). The first
  authenticated word from a neighbor is taken as is; until it arrives the
  robot assumes all walls free, as before.
- **Session key.** `tools/arena.py` draws a fresh 128-bit key per run with
  `secrets.token_bytes(16)` and sends `KEY <32 hex>` to every board over USB
  serial after `RESET` and before `ATTACH`. A board keeps it in RAM only (no NVS,
  no flash), echoes only `OK KEY`, forgets it on `RESET` or power loss, and
  neither sends nor accepts frames until it has one. The sequence counter
  restarts with the key. A board that reboots mid-run has no key and must be
  re-keyed and re-attached by the arena.
- **Initialization.** A docked board does nothing until the first frame from a
  bound neighbor arrives. It then sets its position to sender + (du, dv), exactly as
  in the paper, and runs Algorithms 1-4.
- **Emulated docking.** The arena owns the ground truth. It tells a board
  `ATTACH <wall> <neighbor MAC>` for each wall that now touches a neighbor (the
  stand-in for the wall-occupancy sensor), and tells the neighbors the same.
  It then waits for the board's `INIT p q ...` line and fails the run if the
  board computed the wrong cell. Only `ATTACH` binds a wall to a MAC: a frame
  is dropped unless its source MAC is the one bound to the wall its unit
  vector says it came from, and a board never initializes from an unbound
  frame. A frame that arrives before its `ATTACH` line is dropped; the
  neighbor resends it next step.
- **ESP-NOW encryption is not used.** Per-peer LMK encryption
  (`esp_now_peer_info_t.encrypt`/`lmk`) is left off. Nothing here is secret
  (the frames carry cell coordinates and wall states), integrity comes from the
  tag, and I did not want to add an untested radio path before any board has
  been flashed. See Security.
- **Output.** `SIG <mask> <p> <q>` on serial whenever the set of signaling walls
  changes. LED on GPIO 2: off = not in the assembly, solid = signaling,
  slow blink = in the assembly with nothing to signal. The pin, step period and
  Wi-Fi channel are constants at the top of the sketch.
- **Size** (`make firmware`, esp32:esp32 3.3.10, ESP32 Dev Module): 921,520
  bytes of flash (70% of 1,310,720) and 45,712 bytes of static RAM (13% of
  327,680). The robot state (`node` struct, which includes the key and
  sequence counters) is 280 bytes on the host build.

Running it on boards (not yet done; the arena sends each board its session
key, so a board that is not driven by `arena.py` stays silent):

```sh
python3 -m pip install pyserial
for p in /dev/cu.usbserial-*; do
  arduino-cli compile --upload -p "$p" --fqbn esp32:esp32:esp32 --library core firmware/huddle_node
done
python3 tools/arena.py --ports /dev/cu.usbserial-A /dev/cu.usbserial-B ...   # first port = root
```

The default target is a 7-cell shape in which the root grows its own column
and then seeds a column on each side; `--shape "0,0 0,1 ..."` sets another.
Without boards, `python3 tools/arena.py --fake 7` runs the same arena against
seven `fake_node` processes, relaying their frames itself (`--drop` loses a
fraction of them). `make test` runs the 7-node demo and a 40-node random shape
with 30% frame loss. In six informal runs at 60% loss with k = 3, a build of
`node.c` that treated unheard neighbors as null failed once with an
unreachable opening; the shipped build passed all six.

## Security

Threats to the neighbor link and what the firmware does about each. The
simulator's forgery rows are measured without authentication; they show what
the tag has to prevent. They are not a with-authentication result, which the
simulator cannot give (it would only test its own assumption). The with-tag
behavior is covered by `test/test_node.c`.

- **Replayed or forged frames.** CVE-2024-42483 and CVE-2024-42484 (Nozomi
  Networks, 2024) describe a flaw in the anti-replay cache of Espressif's
  `esp-now` component, fixed in 2.5.2: the cache could be flushed, so encrypted
  frames could be replayed. This firmware does not depend on that cache. A
  frame is accepted only with a valid tag and a sequence number above the last
  accepted one for its wall, so a replay is dropped by the application whether
  or not the radio layer filtered it. **Unconfirmed:** whether the arduino-esp32
  3.3.10 `esp_now.h` path used here contains that component. In the installed
  package `esp_now.h` comes from the ESP-IDF `esp_wifi` component and the
  library is `libespnow.a`; there is no `esp-now` component directory, and
  `strings` finds no replay-related text in the archive. That does not show
  that the code is unaffected.
- **Spoofed source MAC.** The MAC check binds a wall to the address given by
  `ATTACH`, but a MAC is not secret. It is kept as a filter, and the tag is
  the control that counts: a forger needs the session key.
- **Key extraction.** Espressif advisory AR2026-005 (2026-06-10) reports
  voltage fault injection recovering AES keys on the ESP32-S3, notes that the
  base ESP32 has no glitch detector, and recommends short-lived or session
  keys. So there is no long-term key: the key is generated per run on the
  laptop, held in RAM, and lost at power-off. The tag key is a SipHash key,
  not the ESP-NOW LMK, but the same advice applies. An attacker who glitches a
  board during a run can still read that run's key from RAM.
- **Monotone merge** (see Firmware) limits what a stale or replayed word can do
  even before the sequence check. It does nothing against a forged "occupied"
  word, since free -> occupied is the allowed direction; that is what the tag
  is for.

Still open:

- No secure boot or flash encryption is configured. Anyone with USB access can
  reflash a board or dump its flash.
- A physical glitching attack on the base ESP32 is not mitigated, only made
  less valuable by short-lived keys.
- Key distribution over USB serial is trusted: the laptop and the cable are
  inside the trust boundary.
- No ESP-NOW LMK encryption, so a listener can read cell coordinates and wall
  states. SipHash-2-4 with a 64-bit tag is a MAC, not encryption.
- The tag is 64 bits, so a blind forger succeeds with probability 2^-64 per
  frame. A flood of bad frames costs the receiver one SipHash per frame.
- Mid-run reboots are not handled: a rebooted board has no key until the arena
  re-keys it.

## Deviations from the paper

- **Shape input.** The paper gives each robot the perimeter X_b, finds the
  inclusion-minimal cycles with Johnson's algorithm and uses ray casting to
  decide which neighboring cells are inside. A wall is a connection wall
  exactly when the cell it faces is in S, and Algorithm 3 needs only the
  column segments of S. For a hole-free shape the perimeter determines S, so I
  give each robot S directly as a bitmap and skip the reconstruction. The
  classification is the same; the step that turns a perimeter into S is not
  implemented.
- **Motion and sensing are abstracted** as in the paper's Monte Carlo
  experiment: a docking robot appears in the chosen opening instantly (or after
  a fixed latency in the latency runs), and wall occupancy is sensed exactly.
- **Orientation.** The paper's joining robots rotate their wall labels using
  the attracting robot's unit vector. In the simulator and in the arena every
  robot is already north-up, so that transform is not exercised.
- **Monte Carlo details the paper does not give**, chosen here: shape sizes
  uniform in 10..300 cells (mean 155); shapes grown from one cell, picking
  candidates uniformly, preferring few neighbors (thin, branchy shapes) or
  preferring many (compact blobs), a third of shapes each, rejecting any cell
  that would close a hole; the root placed on a uniformly random cell; each
  step docks min(k, number of signaled cells) distinct signaled cells chosen
  uniformly; the docking robot receives its pose word from one signaling
  neighbor.

## Ambiguities and the reading I chose

1. **Midpoint rounding.** Fig. 2 says the midpoint is "rounded down". I use
   floor((q_lo + q_hi) / 2), so rows -3..-2 give -3, not -2 (C's `/` would
   give -2; there is a unit test for this). Rounding up also passed 40,000
   shapes.
2. **"Free flank wall is between 2 null walls" (Algorithm 2).** I read this as:
   the two walls adjacent to it on the hexagon (w-1 and w+1) are both null. For
   FL that means F and RL are null, i.e. this robot is the only cell of its
   column segment that touches the left segment, and it touches it diagonally.
   This matches Fig. 3a(iii).
3. **Statuses in Algorithms 2 and 4 depend on time.** Whether a flank wall is
   free or already occupied at docking changes the result. Both run once, with
   the occupancy sensed at the instant of docking, and are never recomputed.
4. **Which adjacent segment Algorithm 3 looks at.** Line 7 says "identify target
   segment Q_(p+a)" in the singular. A column can touch several segments of the
   next column, so I evaluate every segment of column p+a that touches the
   robot's own segment and nucleate if any of them picks this robot.
5. **What "connects to Q_p" means for m\* (Algorithm 3, line 15).** I read it as
   a same-row connection: (p+a, q\*) is in the adjacent segment and (p, q\*) is
   in the robot's own segment. This is what Fig. 3a(ii) shows: (-1, 3) touches
   (0, 2) diagonally, yet m\* is (-1, 2). The candidate rows form an interval
   that m lies outside of, so the nearest row is unique. If the two segments
   only touch diagonally there is no such row; Algorithm 3 then returns false
   and Algorithm 2 (reading 2) supplies the nucleus.
6. **Algorithm 5, line 4, "all s_i = occupied".** Null walls are never occupied,
   so I read it as "no wall is free". The output is the same either way.
7. **Algorithm 5, line 12.** "w is between (2 occupied or (1 null fore-aft and 1
   occupied)) walls": the two hexagon-adjacent walls of w are both occupied, or
   one is a null F/R wall and the other is occupied.
8. **Simultaneous docking.** With k > 1, robots docking in the same step see
   each other as occupied walls at initialization. The other reading (they do
   not) produced identical results on 40,000 shapes.
9. **Edges.** Definition 1 counts attachments. I treat every pair of adjacent
   occupied cells as attached.
10. **Wire encoding.** The paper gives the widths (16 and 32 bits) but not the
    field layout; the layout above is mine. Positions are limited to
    -2048..2047.
11. **Messages not yet received.** The paper assumes the per-step exchange
    always succeeds, so it never says what Algorithm 6 should assume about a
    neighbor it has not heard from. This matters (see Results); I use "all
    walls free".

## Things that went wrong along the way

- **A compiler bug, not an algorithm bug.** The first runs showed deadlocks.
  The cause was Apple clang 21.0.0's loop vectorizer miscompiling the loop that
  shifts a shape and computes its bounding box at -O2, so robots saw a
  different shape than the simulator. `tools/clang_vectorizer_repro.c` is a
  25-line reproduction (prints `-2 -1 2 3` at -O0 and `-8 -4 8 8` at -O2; I
  could not find undefined behavior in it). The Makefile now builds with
  `-fno-vectorize`, and `make test` checks that -O0 and -O2 builds print
  identical results.
- Before that was found I added a check that every robot's wall status matches
  the world after each step. It stays on in `--verify-holes` runs.

## Remaining work

- Flash 6-7 ESP32 boards and run `tools/arena.py --ports ...`. Record ESP-NOW
  frame loss and whether every board initializes to the right cell. Check that
  frames with a wrong tag really are dropped on the radio, not only in the host
  tests.
- Configure secure boot and flash encryption, and decide whether to turn on
  ESP-NOW LMK encryption with a key derived from the session key.
- Replace the arena's `ATTACH` lines with real sensing (reed switches or hall
  sensors on each wall) and the serial `SIG` lines with directional IR, which
  is where the paper's signals live.
- Exercise the orientation transform: let docked boards have arbitrary
  rotations and derive their labels from the received unit vector.
- Model a limited supply of passing robots, so that suppressed signals show up
  as completion time.
- Implement the perimeter-to-shape step (cycle finding plus ray casting) if
  robots should carry only X_b.

## Layout

```
core/huddle.[ch]           Algorithms 1-6, wire format, monotone status merge
test/test_core.c           cases from Figs. 2 and 3, rounding, wire format, merge
test/test_node.c           SipHash vectors, frame layout, tamper/replay/key/MAC rejection
sim/huddle_sim.c           Monte Carlo harness and checkers
sim/run_experiments.py     runs every configuration in parallel, prints the tables
firmware/huddle_node/      ESP32 sketch (huddle_node.ino), portable logic (node.[ch]), SipHash-2-4 (siphash.[ch])
tools/fake_node.c          host stand-in for one board
tools/arena.py             world emulator for real boards or fake nodes
tools/clang_vectorizer_repro.c
```
