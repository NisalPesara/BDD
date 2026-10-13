#!/bin/sh
# Builds and runs every desktop test (needs g++; on Windows add -D_USE_MATH_DEFINES for M_PI). From the ppg_esp32 folder:  sh test/run_tests.sh
# The sketch scenario sends b itself (manual reference) -> built with AUTO_BASELINE=0.
set -e
cd "$(dirname "$0")"
OUT=${TMPDIR:-/tmp}/ppg_tests; mkdir -p "$OUT"
echo "== unit tests";            g++ -std=c++17 -O2 -Wall -Wextra -I.. native_test.cpp -o "$OUT/native" && "$OUT/native"
echo "== end-to-end sketch (report mode)";   g++ -std=c++17 -O2 -Wall -Wextra -DAUTO_BASELINE=0 -DOUTPUT_MODE=0 -Istubs -I.. sketch_test.cpp -o "$OUT/sketch_report" && (cd "$OUT" && ./sketch_report)
echo "== end-to-end sketch (teleplot mode)"; g++ -std=c++17 -O2 -Wall -Wextra -DAUTO_BASELINE=0 -Istubs -I.. sketch_test.cpp -o "$OUT/sketch_teleplot" && (cd "$OUT" && ./sketch_teleplot)
echo "== LED-current lock";      g++ -std=c++17 -O2 -Wall -Wextra -Istubs -I.. agc_test.cpp -o "$OUT/agc" && "$OUT/agc"
echo "== minor-noise libraries"; g++ -std=c++17 -O2 -Wall -I.. test_ppgnr.cpp -o "$OUT/ppgnr" && "$OUT/ppgnr" | tail -1
echo "== ANC (REL9)";            g++ -std=c++17 -O2 -I.. -DANC_MODE=1 anc_test.cpp -o "$OUT/anc" && "$OUT/anc"
echo "== ANC (REL6)";            g++ -std=c++17 -O2 -I.. -DANC_MODE=2 anc_test.cpp -o "$OUT/anc6" && "$OUT/anc6"
