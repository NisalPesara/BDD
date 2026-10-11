#!/bin/sh
# Builds and runs every desktop test (needs g++). From the ppg_esp32 folder:  sh test/run_tests.sh
set -e
cd "$(dirname "$0")"
OUT=${TMPDIR:-/tmp}/ppg_tests; mkdir -p "$OUT"
echo "== unit tests";            g++ -std=c++17 -O2 -Wall -Wextra -I.. native_test.cpp -o "$OUT/native" && "$OUT/native"
echo "== end-to-end sketch";     g++ -std=c++17 -O2 -Wall -Wextra -Istubs -I.. sketch_test.cpp -o "$OUT/sketch" && (cd "$OUT" && ./sketch)
echo "== minor-noise libraries"; g++ -std=c++17 -O2 -Wall -I.. test_ppgnr.cpp -o "$OUT/ppgnr" && "$OUT/ppgnr" | tail -1
echo "== ANC (REL9)";            g++ -std=c++17 -O2 -I.. -DANC_MODE=1 anc_test.cpp -o "$OUT/anc" && "$OUT/anc"
