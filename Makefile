CC ?= cc
# Apple clang 21's loop vectorizer miscompiles a min/max loop in the shape
# generator (tools/clang_vectorizer_repro.c), so it stays off.
NOVEC := $(shell $(CC) -fno-vectorize -x c -c /dev/null -o /dev/null 2>/dev/null && echo -fno-vectorize || echo -fno-tree-vectorize)
CFLAGS ?= -std=c99 -O2 $(NOVEC) -Wall -Wextra -pedantic
CORE := core/huddle.c core/huddle.h
SIM := sim/huddle_sim.c
FQBN := esp32:esp32:esp32

all: build/huddle_sim build/test_core

build:
	mkdir -p build

build/huddle_sim: $(CORE) $(SIM) | build
	$(CC) $(CFLAGS) -Icore core/huddle.c $(SIM) -o $@

build/huddle_sim_O0: $(CORE) $(SIM) | build
	$(CC) -std=c99 -O0 -Icore core/huddle.c $(SIM) -o $@

build/huddle_sim_roundup: $(CORE) $(SIM) | build
	$(CC) $(CFLAGS) -DHUDDLE_MIDPOINT_ROUND_UP=1 -Icore core/huddle.c $(SIM) -o $@

build/huddle_sim_no_remark1: $(CORE) $(SIM) | build
	$(CC) $(CFLAGS) -DHUDDLE_SPECIAL_FLANK_SIGNAL=0 -Icore core/huddle.c $(SIM) -o $@

build/test_core: $(CORE) test/test_core.c | build
	$(CC) $(CFLAGS) -Icore core/huddle.c test/test_core.c -o $@

# Fails if the algorithm, the checker, or the build breaks.
test: build/test_core build/huddle_sim build/huddle_sim_O0
	./build/test_core
	./build/huddle_sim --shapes 300 --verify-holes > build/sync.txt || (cat build/sync.txt; exit 1)
	grep RESULT build/sync.txt
	./build/huddle_sim --shapes 100 --lossy 0.3 3 free > build/lossy.txt || (cat build/lossy.txt; exit 1)
	@echo "negative control: Algorithm 6 disabled must produce unreachable openings"
	! ./build/huddle_sim --shapes 50 --no-delay > /dev/null
	@echo "determinism: -O0 and -O2 builds must agree"
	./build/huddle_sim_O0 --shapes 100 > build/o0.txt
	./build/huddle_sim --shapes 100 > build/o2.txt
	cmp build/o0.txt build/o2.txt
	@echo "make test: OK"

experiments: build/huddle_sim build/huddle_sim_roundup build/huddle_sim_no_remark1
	python3 sim/run_experiments.py

firmware:
	arduino-cli compile --fqbn $(FQBN) --library core firmware/huddle_node

clean:
	rm -rf build

.PHONY: all test experiments firmware clean
