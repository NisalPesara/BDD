#!/usr/bin/env python3
"""
Record the ESP32 output and turn the "Name: value unit" reports into CSV tables.

    pip install pyserial
    python receive_from_esp32.py COM5 120                 # port, seconds  (Linux: /dev/ttyACM0)
    python receive_from_esp32.py COM5 120 --baseline      # also send 'b' after 15 s (capture baseline)
    python receive_from_esp32.py --parse session_raw.txt  # only convert an existing log

Output files (prefix 'session' by default):
    session_raw.txt     everything the board printed
    session_pulses.csv  one row per PULSE REPORT  (all features, flow ratios, flow status, SpO2)
    session_status.csv  one row per STATUS block  (flow status, signal quality, HR, SpO2, FSR, motion, NIRS)
    session_plot.csv    plotter-mode lines (PPG, VPG/10, APG/100, Beat, SQI) if the board was in 'p' mode
"""
import argparse, csv, re, sys, time

LINE = re.compile(r"^([^:#]+):\s*(\S+)(?:\s+(.*))?$")


def parse(lines):
    pulses, status, plot = [], [], []
    cur, kind = None, None
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("==================== PULSE #"):
            cur, kind = {"pulse": line.split("#")[1].split()[0]}, "pulse"
            continue
        if line.startswith("---- STATUS"):
            if cur and kind == "status":
                status.append(cur)
            cur, kind = {"t_s": line.split("t=")[1].split()[0]}, "status"
            continue
        if line.startswith("====") and cur and kind == "pulse":
            pulses.append(cur); cur, kind = None, None
            continue
        if line.startswith("PPG:"):                                   # plotter line
            plot.append(dict(kv.split(":", 1) for kv in line.split(",") if ":" in kv))
            continue
        if cur is not None:
            m = LINE.match(line)
            if m:
                name, value, rest = m.group(1).strip(), m.group(2), m.group(3) or ""
                # text values (Flow status, Dicrotic notch ...) keep the whole text
                try:
                    float(value)
                    cur[name] = value
                except ValueError:
                    cur[name] = (value + " " + rest).strip() if value != "n/a" else ""
    if cur and kind == "status":
        status.append(cur)
    return pulses, status, plot


def write(rows, path):
    if not rows:
        return 0
    cols = []
    for r in rows:
        for k in r:
            if k not in cols:
                cols.append(k)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    return len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?")
    ap.add_argument("seconds", nargs="?", type=float, default=60)
    ap.add_argument("--prefix", default="session")
    ap.add_argument("--baseline", action="store_true", help="send 'b' 15 s after start")
    ap.add_argument("--parse", help="convert an existing raw log instead of recording")
    a = ap.parse_args()

    if a.parse:
        lines = open(a.parse, encoding="utf-8", errors="ignore").read().splitlines()
    else:
        if not a.port:
            sys.exit("give a serial port, e.g. COM5 or /dev/ttyACM0")
        import serial
        ser = serial.Serial(a.port, 460800, timeout=0.5)
        lines, t0, sent = [], time.time(), False
        with open(a.prefix + "_raw.txt", "w", encoding="utf-8") as raw:
            while time.time() - t0 < a.seconds:
                if a.baseline and not sent and time.time() - t0 > 15:
                    ser.write(b"b"); sent = True
                l = ser.readline().decode(errors="ignore")
                if l:
                    raw.write(l); lines.append(l)
                    if "Flow status" in l or "EVENT" in l:
                        print(l.strip())
        ser.close()

    p, s, q = parse(lines)
    print(f"pulses: {write(p, a.prefix + '_pulses.csv')}  status: {write(s, a.prefix + '_status.csv')}  "
          f"plot samples: {write(q, a.prefix + '_plot.csv')}")


if __name__ == "__main__":
    main()
