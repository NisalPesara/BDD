"""Live viewer of the relative pulse level (the 10-LED output) of the ESP32 PPG monitor.

Top graph: x = time, y = integer levels 1..10 drawn as a step graph with a shaded band per level, plus a
separate UNKNOWN row (level 0). Bottom graph: relativeAmplitude and relativePI together with PulseIndex = the
continuous x the LEDs use (all three from the firmware - nothing is recalculated here; gaps = unavailable). Higher level = stronger relative optical pulsatility (pulse strength x
compared with the 'b' reference). It is a relative pulse level - NOT a blood-flow percentage.
A 10-LED column on the right copies the hardware LEDs (all off = UNKNOWN).

The firmware must be in Teleplot mode (default after boot; 'p' cycles modes). It reads
>BloodFlowLevel (10 Hz) and the once-per-second >PulseIndex, >relativeAmplitude, >relativePI,
>pulseDetectionRate, >amplitudeCV, >templateCorrelation, >IR_DC  (-1 / -2 = unavailable).
Close Teleplot / the serial monitor first: only one program can open the port.

  python tools/flow_viewer.py                    # COM3, 460800 baud
  python tools/flow_viewer.py --port COM5 --window 300 --csv level_log.csv
  python tools/flow_viewer.py --demo             # simulated data, no board needed
"""
import argparse
import csv
import math
import queue
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

TITLE = "Relative pulse level \u2014 not blood-flow percentage"
SLOW = ("PulseIndex", "relativeAmplitude", "relativePI", "pulseDetectionRate", "amplitudeCV",
        "templateCorrelation", "IR_DC")
# level 1 (weakest pulse vs reference) red -> level 10 (as strong as the reference) green; 0 = UNKNOWN grey
COLORS = [None, "#a50026", "#d73027", "#f46d43", "#fdae61", "#fee08b",
          "#d9ef8b", "#a6d96a", "#66bd63", "#1a9850", "#006837"]
UNKNOWN_COLOR = "#9e9e9e"
LED_OFF = "#2b2b2b"


def level_name(lv):
    if lv <= 0:
        return "UNKNOWN"
    return "x %.1f-%.1f of reference" % ((lv - 1) / 10, lv / 10)


def color_of(lv):
    return COLORS[lv] if 1 <= lv <= 10 else UNKNOWN_COLOR


def serial_reader(port, baud, q, stop):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.5
    s.dtr = False; s.rts = False                     # do not reset the board
    s.open()
    while not stop.is_set():
        line = s.readline().decode(errors="replace").strip()
        if not line.startswith(">") or ":" not in line:
            continue
        key, val = line[1:].split(":", 1)
        if key != "BloodFlowLevel" and key not in SLOW:
            continue
        try:
            q.put((time.time(), key, float(val.split("|")[0])))
        except ValueError:
            pass
    s.close()


def demo_reader(q, stop):
    """Reference -> slow restriction -> no detectable pulse (UNKNOWN) -> release, 10x faster than real time.
    Same rules as the firmware: x = 0.75 x + 0.25 clamp(relativePI) per pulse; UNKNOWN clears x."""
    t0 = time.time(); x = None; next_pulse = 0.0; last_slow = -1.0; rel = None
    while not stop.is_set():
        t = (time.time() - t0) * 10
        if t < 30:    rpi = 1.0
        elif t < 120: rpi = 1.0 - (t - 30) / 90 * 0.9
        elif t < 150: rpi = None                       # pulse too weak to detect -> UNKNOWN, not "zero"
        else:         rpi = min(1.0, 0.1 + (t - 150) / 20)
        moving = 60 < t < 66                           # a motion episode -> UNKNOWN
        if rpi is None or moving:
            x = None; rel = None
        elif t >= next_pulse:                          # one new pulse every 0.8 s
            next_pulse = t + 0.8
            rel = max(0.0, rpi * (1 + 0.04 * math.sin(7 * t)))
            target = min(1.0, rel)
            x = target if x is None else 0.75 * x + 0.25 * target
        level = 0 if x is None else min(10, 1 + int(math.floor(10 * x)))
        now = time.time()
        q.put((now, "BloodFlowLevel", level))
        if t - last_slow >= 10:                        # 1 s of firmware time
            last_slow = t
            for k, v in (("PulseIndex", x), ("relativeAmplitude", rel), ("relativePI", rel),
                         ("pulseDetectionRate", None if rel is None else 1.0), ("amplitudeCV", None if rel is None else 0.03),
                         ("templateCorrelation", None if rel is None else 0.98), ("IR_DC", 180000)):
                q.put((now, k, (-2.0 if k == "templateCorrelation" else -1.0) if v is None else v))
        if t > 200:
            t0 = time.time(); x = None; next_pulse = 0.0; last_slow = -1.0
        time.sleep(0.01)                               # 10x faster: 100 ms of firmware time


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--window", type=float, default=120, help="seconds shown")
    ap.add_argument("--csv", help="also log every BloodFlowLevel sample (+ the latest index values) to this CSV file")
    ap.add_argument("--demo", action="store_true", help="simulated data, no board")
    ap.add_argument("--snapshot", help="(with --demo) run N seconds, save a PNG and exit", nargs=2,
                    metavar=("SECONDS", "FILE"))
    a = ap.parse_args()

    q, stop = queue.Queue(), threading.Event()
    target = (lambda: demo_reader(q, stop)) if a.demo else (lambda: serial_reader(a.port, a.baud, q, stop))
    threading.Thread(target=target, daemon=True).start()

    logf = open(a.csv, "w", newline="") if a.csv else None
    logw = csv.writer(logf) if logf else None
    if logw:
        logw.writerow(["time_s", "level", "meaning"] + list(SLOW))

    data = []                                          # (t, level)
    ratios = {k: [] for k in ("relativeAmplitude", "relativePI", "PulseIndex")}   # (t, value or nan)
    latest = {}
    t_start = None

    plt.rcParams["toolbar"] = "None"
    fig = plt.figure(figsize=(13, 8.5), facecolor="#111111")
    ax = fig.add_axes([0.23, 0.38, 0.60, 0.50], facecolor="#181818")
    led_ax = fig.add_axes([0.86, 0.38, 0.10, 0.50], facecolor="#111111")
    rax = fig.add_axes([0.23, 0.07, 0.60, 0.24], facecolor="#181818", sharex=ax)
    rax.set_ylim(0, 1.5); rax.axhline(1.0, color="#666666", lw=0.8, ls=":")
    rax.set_ylabel("x reference", color="#cccccc"); rax.set_xlabel("time (s)", color="#cccccc")
    rax.tick_params(colors="#cccccc"); rax.grid(color="#333333", lw=0.6)
    for sp in rax.spines.values(): sp.set_color("#444444")
    rlines = {k: rax.plot([], [], color=c, lw=w, label=lbl)[0] for k, c, w, lbl in (
        ("relativeAmplitude", "#5fa8ff", 1.2, "relativeAmplitude"), ("relativePI", "#ffb347", 1.2, "relativePI"),
        ("PulseIndex", "#ffffff", 2.2, "PulseIndex x (drives the LEDs)"))}
    rax.legend(loc="upper left", fontsize=8, facecolor="#222222", labelcolor="#dddddd", edgecolor="#444444", ncol=3)
    fig.text(0.23, 0.955, TITLE, color="#dddddd", fontsize=12, style="italic")
    info = fig.text(0.23, 0.91, "waiting for data ...", color="white", fontsize=15, weight="bold")
    sub = fig.text(0.23, 0.875, "", color="#bbbbbb", fontsize=9.5)

    for lv in range(1, 11):                            # one shaded band per level
        ax.axhspan(lv - 0.5, lv + 0.5, color=COLORS[lv], alpha=0.13, lw=0)
        ax.axhline(lv - 0.5, color="#2e2e2e", lw=0.6)
    ax.axhspan(-0.9, -0.1, color=UNKNOWN_COLOR, alpha=0.12, lw=0)       # separate UNKNOWN row
    ax.axhline(0.5, color="#888888", lw=1.4)
    ax.axhline(-0.1, color="#888888", lw=0.8, ls=":")
    ax.set_ylim(-0.9, 10.5)
    ax.set_yticks([-0.5] + list(range(1, 11)))
    ax.set_yticklabels(["0  UNKNOWN"] + ["%d  %s" % (lv, level_name(lv)) for lv in range(1, 11)], fontsize=9)
    ax.tick_params(colors="#cccccc")
    for lbl, lv in zip(ax.get_yticklabels(), [0] + list(range(1, 11))):   # after tick_params, which resets them
        lbl.set_color(color_of(lv)); lbl.set_weight("bold")
    ax.tick_params(labelbottom=False)
    ax.set_ylabel("relative pulse level (10 = reference strength)", color="#cccccc")
    for sp in ax.spines.values(): sp.set_color("#444444")
    ax.grid(axis="x", color="#333333", lw=0.6)

    steps = LineCollection([], linewidths=5, capstyle="butt")
    risers = LineCollection([], linewidths=1.2, colors="#777777")
    ax.add_collection(risers); ax.add_collection(steps)
    cursor, = ax.plot([], [], "o", ms=11, mec="white", mew=1.5)

    led_ax.set_xlim(0, 1); led_ax.set_ylim(-0.9, 10.5); led_ax.axis("off")
    # scatter markers stay round whatever the axes' aspect ratio
    leds = led_ax.scatter([0.38] * 10, list(range(1, 11)), s=900, c=[LED_OFF] * 10, edgecolors="#555555")
    for lv in range(1, 11):
        led_ax.text(0.85, lv, str(lv), color="#999999", va="center", fontsize=10)
    led_ax.text(0.38, 10.75, "LEDs", color="#cccccc", ha="center", fontsize=10)

    def y_of(lv):                                      # level 0 is drawn inside the UNKNOWN row
        return -0.5 if lv <= 0 else lv

    def fmt(k, v):
        if v is None or v in (-1.0, -2.0):
            return "n/a"
        return {"PulseIndex": "%.3f", "relativeAmplitude": "%.3f", "relativePI": "%.3f", "pulseDetectionRate": "%.2f",
                "amplitudeCV": "%.3f", "templateCorrelation": "%.3f", "IR_DC": "%.0f"}[k] % v

    def update(_):
        nonlocal t_start
        while not q.empty():
            tw, key, val = q.get()
            if t_start is None: t_start = tw
            if key != "BloodFlowLevel":
                latest[key] = val
                if key in ratios:                      # -1 = unavailable -> gap in the line
                    ratios[key].append((tw - t_start, float("nan") if val == -1.0 else val))
                continue
            lv = int(round(val))
            data.append((tw - t_start, lv))
            if logw:
                logw.writerow([f"{tw - t_start:.2f}", lv, level_name(lv)] + [latest.get(k, "") for k in SLOW])
                logf.flush()
        if not data:
            return
        now = data[-1][0]
        t0 = max(0.0, now - a.window)
        segs, cols, rise = [], [], []
        pts = [d for d in data if d[0] >= t0 - 1]
        for i, (t, lv) in enumerate(pts):              # step graph: hold each value until the next sample
            t_next = pts[i + 1][0] if i + 1 < len(pts) else t + 0.1
            segs.append([(t, y_of(lv)), (t_next, y_of(lv))])
            cols.append(color_of(lv))
            if i + 1 < len(pts) and pts[i + 1][1] != lv:
                rise.append([(t_next, y_of(lv)), (t_next, y_of(pts[i + 1][1]))])
        steps.set_segments(segs); steps.set_colors(cols); risers.set_segments(rise)
        ax.set_xlim(t0, max(t0 + a.window, now + 1))
        for k, ln in rlines.items():
            pts_r = [r for r in ratios[k] if r[0] >= t0 - 1]
            ln.set_data([r[0] for r in pts_r], [r[1] for r in pts_r])

        lv = data[-1][1]
        cursor.set_data([now + 0.1], [y_of(lv)]); cursor.set_color(color_of(lv))
        leds.set_facecolor([COLORS[i] if i <= lv else LED_OFF for i in range(1, 11)])
        if lv > 0:
            info.set_text(f"Level {lv}/10 : {level_name(lv)}")
        else:
            info.set_text("UNKNOWN : not measurable now (all LEDs off)")
        info.set_color(color_of(lv) if lv > 0 else "white")
        sub.set_text("   ".join(f"{lbl} {fmt(k, latest.get(k))}" for k, lbl in (
            ("PulseIndex", "x"), ("relativeAmplitude", "relAmp"), ("relativePI", "relPI"),
            ("pulseDetectionRate", "beats"), ("amplitudeCV", "ampCV"), ("templateCorrelation", "shape"),
            ("IR_DC", "IR DC"))))

    anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    fig.canvas.manager.set_window_title("Relative pulse level")
    if a.snapshot:
        secs, path = float(a.snapshot[0]), a.snapshot[1]
        end = time.time() + secs
        while time.time() < end:
            update(0); plt.pause(0.3)
        update(0); fig.savefig(path, dpi=90, facecolor=fig.get_facecolor())
    else:
        plt.show()
    stop.set()
    if logf: logf.close()
    _ = anim


if __name__ == "__main__":
    main()
