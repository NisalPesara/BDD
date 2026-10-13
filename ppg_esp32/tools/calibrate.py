"""OBSOLETE (2026-10-12): m14 now shows the relative optical pulse-strength index x, which has no calibration
table and no >FlowLevel / >FlowScore output. This script only prints that notice and exits; kept for reference.

Blood-pressure-cuff calibration of the 10-level flow score (m14_flow_level.h).

Cuff on the UPPER ARM, PPG probe on the wrist of the same arm, arm resting. The script guides you through
known cuff pressures, records the firmware's flow score at each, and prints the calibration table
(FL_CAL_IN / FL_CAL_OUT) for config.h that maps each measured score onto the level that pressure should give.

  python tools/calibrate.py               # COM3, 460800 baud (close Teleplot first)
  python tools/calibrate.py --port COM5

SAFETY: never keep the cuff above your systolic pressure for more than 60 s. Release at once if you feel
pain, strong tingling or numbness. Stop with Ctrl+C at any time (then deflate the cuff).
The firmware must be in Teleplot mode (the default after a reset).
"""
import argparse
import csv
import statistics
import sys
import threading
import time

# (name, instruction, seconds, target level or None = not used for the table)
STAGES = [
    ("0 mmHg",     "Cuff fully DEFLATED. Keep still.",                                   60, 1),
    ("60 mmHg",    "Inflate the cuff to about 60 mmHg and hold it there. Keep still.",    45, 3),
    ("100 mmHg",   "Inflate the cuff to about 100 mmHg and hold it there. Keep still.",   45, 6),
    ("occluded",   "Inflate ABOVE your systolic pressure (about 180 mmHg). MAX 45 s!",     45, 10),
    ("release",    "DEFLATE the cuff completely now. Keep still (recovery check).",       60, None),
]
SETTLE_S = 10          # first seconds of each stage are not used (pressure / tissue settling)


def reader(ser, rows, stop):
    rec = {}
    while not stop.is_set():
        line = ser.readline().decode(errors="replace").strip()
        if not line.startswith(">") or ":" not in line:
            continue
        key, val = line[1:].split(":", 1)
        val = val.split("|")[0]
        if key == "FlowLevel" and "FlowLevel" in rec:     # one block per second starts with >FlowLevel
            rec["t"] = time.time(); rows.append(rec); rec = {}
        if key in ("FlowLevel", "FlowLive", "FlowScore", "relAmp", "relPI", "detRate", "ampCV", "tplCorr", "HR"):
            try: rec[key] = float(val)
            except ValueError: pass
        elif key == "FlowState":
            rec[key] = val


def countdown(seconds, rows, label):
    end = time.time() + seconds
    while time.time() < end:
        r = rows[-1] if rows else {}
        sys.stdout.write("\r  %-9s %3d s left | level %-2s score %-5s HR %-4s   " % (
            label, int(end - time.time()), int(r.get("FlowLevel", 0)) or "-",
            ("%.2f" % r["FlowScore"]) if "FlowScore" in r else "-", ("%.0f" % r["HR"]) if "HR" in r else "-"))
        sys.stdout.flush(); time.sleep(0.5)
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--csv", default="calibration_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
    a = ap.parse_args()
    sys.exit("calibrate.py is obsolete: the firmware no longer sends >FlowLevel / >FlowScore and has no FL_CAL table.\n"
             "The LEDs now show the relative optical pulse-strength index x (m14_flow_level.h), which uses no\n"
             "calibration. To log x during cuff stages use: python tools/flow_viewer.py --csv cuff_log.csv")

    import serial
    ser = serial.Serial(); ser.port, ser.baudrate, ser.timeout = a.port, a.baud, 0.3
    ser.dtr = False; ser.rts = False
    ser.open()
    rows, stop = [], threading.Event()
    threading.Thread(target=reader, args=(ser, rows, stop), daemon=True).start()

    print(__doc__.split("SAFETY")[0].strip()); print()
    input("Cuff on the upper arm (deflated), probe on the wrist, arm resting. Press Enter to start ... ")
    print("\nResetting and capturing the baseline - keep completely still.")
    ser.write(b"r")
    t0 = time.time()
    while time.time() - t0 < 150:
        if rows and rows[-1].get("FlowLevel", 0) > 0 and rows[-1].get("FlowLive", 0):
            break
        sys.stdout.write("\r  waiting for baseline ... %3d s" % (time.time() - t0)); sys.stdout.flush(); time.sleep(0.5)
    print()
    if not (rows and rows[-1].get("FlowLevel", 0) > 0):
        print("No baseline after 150 s (motion? probe?). Fix the probe and run again."); stop.set(); return
    print("Baseline captured.\n")

    marks = []
    try:
        for name, text, secs, target in STAGES:
            print("STAGE %s: %s" % (name, text))
            input("  Press Enter when the cuff is at this pressure ... ")
            start = time.time()
            countdown(secs, rows, name)
            marks.append((name, start, time.time(), target))
    except KeyboardInterrupt:
        print("\nStopped - DEFLATE THE CUFF.")
    stop.set(); time.sleep(0.5); ser.close()

    with open(a.csv, "w", newline="") as f:
        w = csv.writer(f)
        keys = ["FlowLevel", "FlowLive", "FlowScore", "relAmp", "relPI", "detRate", "ampCV", "tplCorr", "HR", "FlowState"]
        w.writerow(["time_s", "stage"] + keys)
        for r in rows:
            st = next((m[0] for m in marks if m[1] <= r["t"] <= m[2]), "")
            w.writerow(["%.1f" % (r["t"] - rows[0]["t"]), st] + [r.get(k, "") for k in keys])
    print("\nRaw data saved to", a.csv)

    print("\n stage      | target | median score | median level | relAmp | beats | samples")
    pts = []
    for name, s, e, target in marks:
        sel = [r for r in rows if s + SETTLE_S <= r["t"] <= e and "FlowScore" in r]
        if not sel:
            print(" %-10s | %6s | no data" % (name, target)); continue
        sc = statistics.median(r["FlowScore"] for r in sel)
        lv = statistics.median(r.get("FlowLevel", 0) for r in sel)
        ra = [r["relAmp"] for r in sel if "relAmp" in r]
        dr = [r["detRate"] for r in sel if "detRate" in r]
        print(" %-10s | %6s | %12.3f | %12.1f | %6s | %5s | %d" % (name, target if target else "-", sc, lv,
              "%.2f" % statistics.median(ra) if ra else "-", "%.2f" % statistics.median(dr) if dr else "-", len(sel)))
        if target: pts.append((sc, (10 - target) / 10.0 + 0.05))   # target level -> centre of its score band

    pts.sort()
    mono = all(pts[i][1] <= pts[i + 1][1] for i in range(len(pts) - 1))
    if len(pts) < 2:
        print("\nNot enough stages measured for a table."); return
    if not mono:
        print("\nWARNING: the measured score did not fall steadily with pressure (motion or probe shift during a"
              " stage?). The table below is unreliable - repeat the calibration.")
    cin = [0.0] + [round(p[0], 3) for p in pts] + [1.0]
    cout = [0.0] + [round(p[1], 3) for p in pts] + [1.0]
    for i in range(1, len(cin)):                       # keep the table strictly increasing
        if cin[i] <= cin[i - 1]: cin[i] = round(cin[i - 1] + 0.001, 3)
        if cout[i] < cout[i - 1]: cout[i] = cout[i - 1]
    print("\nPaste into config.h (replacing the two FL_CAL lines):\n")
    print("static const float FL_CAL_IN[]  = {%s};" % ", ".join("%.3ff" % v for v in cin))
    print("static const float FL_CAL_OUT[] = {%s};" % ", ".join("%.3ff" % v for v in cout))


if __name__ == "__main__":
    main()
