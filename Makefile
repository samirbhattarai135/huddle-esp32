CC ?= cc
# Apple clang 21's loop vectorizer miscompiles a min/max loop in the shape
# generator (tools/clang_vectorizer_repro.c), so it stays off.
NOVEC := $(shell $(CC) -fno-vectorize -x c -c /dev/null -o /dev/null 2>/dev/null && echo -fno-vectorize || echo -fno-tree-vectorize)
CFLAGS ?= -std=c99 -O2 $(NOVEC) -Wall -Wextra -pedantic
CORE := core/huddle.c core/huddle.h
SIM := sim/huddle_sim.c
FQBN := esp32:esp32:esp32

NODE := firmware/huddle_node/node.c firmware/huddle_node/node.h firmware/huddle_node/siphash.c firmware/huddle_node/siphash.h

all: build/huddle_sim build/test_core build/test_node build/fake_node

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

build/fake_node: $(CORE) $(NODE) tools/fake_node.c | build
	$(CC) $(CFLAGS) -Icore -Ifirmware/huddle_node core/huddle.c firmware/huddle_node/node.c firmware/huddle_node/siphash.c tools/fake_node.c -o $@

build/test_node: $(CORE) $(NODE) test/test_node.c | build
	$(CC) $(CFLAGS) -Icore -Ifirmware/huddle_node core/huddle.c firmware/huddle_node/node.c firmware/huddle_node/siphash.c test/test_node.c -o $@

build/test_core: $(CORE) test/test_core.c | build
	$(CC) $(CFLAGS) -Icore core/huddle.c test/test_core.c -o $@

# Fails if the algorithm, the checker, or the build breaks.
test: build/test_core build/test_node build/huddle_sim build/huddle_sim_O0 build/fake_node
	./build/test_core
	./build/test_node
	./build/huddle_sim --self-test
	./build/huddle_sim --shapes 300 --verify-holes > build/sync.txt || (cat build/sync.txt; exit 1)
	grep RESULT build/sync.txt
	./build/huddle_sim --shapes 100 --lossy 0.3 3 free > build/lossy.txt || (cat build/lossy.txt; exit 1)
	@echo "negative control: Algorithm 6 disabled must produce unreachable openings"
	! ./build/huddle_sim --shapes 50 --no-delay > /dev/null
	@echo "replay with and without the monotone merge, forged frames fail without authentication"
	./build/huddle_sim --shapes 200 --replay 0.5 --merge > /dev/null
	./build/huddle_sim --shapes 200 --replay 0.5 > /dev/null
	! ./build/huddle_sim --shapes 50 --forge 0.01 > /dev/null
	@echo "determinism: -O0 and -O2 builds must agree"
	./build/huddle_sim_O0 --shapes 100 > build/o0.txt
	./build/huddle_sim --shapes 100 > build/o2.txt
	cmp build/o0.txt build/o2.txt
	@echo "firmware node logic on the host: 7-board demo, then 40 nodes with 30% frame loss"
	python3 tools/arena.py --fake 7 > build/arena7.txt || (cat build/arena7.txt; exit 1)
	python3 tools/arena.py --fake 40 --random --drop 0.3 --k 2 > build/arena40.txt || (cat build/arena40.txt; exit 1)
	tail -1 build/arena40.txt
	@echo "make test: OK"

experiments: build/huddle_sim build/huddle_sim_roundup build/huddle_sim_no_remark1
	python3 sim/run_experiments.py

firmware:
	arduino-cli compile --fqbn $(FQBN) --library core firmware/huddle_node

clean:
	rm -rf build

.PHONY: all test experiments firmware clean
