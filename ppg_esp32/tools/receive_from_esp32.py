#!/usr/bin/env python3
"""
Record the purified PPG streamed by the ESP32 and save it as CSV for your
wave-properties (pyPPG / own) code.
    pip install pyserial pandas
    python receive_from_esp32.py COM5 60        # port, seconds   (Linux: /dev/ttyUSB0)
"""
import sys, time, serial, pandas as pd

port = sys.argv[1]; secs = float(sys.argv[2]) if len(sys.argv) > 2 else 60
ser = serial.Serial(port, 460800, timeout=1)
rows, t0 = [], time.time()
while time.time() - t0 < secs:
    line = ser.readline().decode(errors="ignore").strip()
    if not line or line.startswith("#"): continue
    p = line.split(",")
    if len(p) != 8: continue
    try: rows.append([float(v) for v in p])
    except ValueError: pass
df = pd.DataFrame(rows, columns=["t_ms","ppg","vpg","apg","jpg","sqi_good","beat","bpm"])
df.to_csv("esp32_ppg_clean.csv", index=False)
print(f"saved {len(df)} samples -> esp32_ppg_clean.csv  ({(df.sqi_good==1).mean()*100:.1f}% flagged good)")

# ---- feed your detector: only keep good-quality samples, keep 125 Hz timing ----
# signal.v  = df.ppg.values      (already band-pass filtered + smoothed on the ESP32)
# signal.fs = 125
# If you reuse the pyPPG notebook, either skip its preprocessing (assign signal.ppg / vpg /
# apg / jpg from the CSV columns) or accept a second, mild filtering pass.
