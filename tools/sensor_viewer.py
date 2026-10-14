"""
Live sensor viewer for the IDP robot. Runs on the PC, NOT on the Pico.

    pip install pyserial
    python tools/sensor_viewer.py

1. Upload the robot code (config.py, sensors/, libs/, tests/) to the Pico.
2. Close MicroPico's terminal / disconnect MicroPico: only one program can
   use the serial port at a time.
3. Pick the port (the Pico is detected automatically) and press Connect.

The viewer interrupts whatever is running on the Pico and starts
tests/stream_sensors.py with the pins typed in the "Wiring" row, so you can
test any wiring without editing config.py. Leave a field empty to disable that
sensor group. Press Apply after changing the wiring.

Tabs: "Overview" shows every sensor; "Distance (ToF)" shows each VL53L0X
with its I2C bus scan, a continuous distance-vs-time curve (solid = filtered,
dashed = raw), noise statistics and an offset calibration helper;
"Magnetic" shows the Hall sensor voltage as a continuous curve with its
zero-field baseline and detection thresholds.

Command line options set the starting values, e.g. only the pin monitor and IR:
    python tools/sensor_viewer.py --line "" --crash "" --connect
    python tools/sensor_viewer.py --tab tof --connect
    python tools/sensor_viewer.py --tab mag --connect --record log.txt
"""

import argparse
import json
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is missing: pip install pyserial")
    sys.exit(1)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import config  # noqa: E402  (pure Python, safe to import on the PC)

PICO_VID = 0x2E8A
BG = "#f4f4f4"
ON = "#2ecc71"
OFF = "#3b3b3b"
RED = "#e74c3c"
AMBER = "#f39c12"
GREY = "#bdbdbd"
FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 10, "bold")
FONT_BIG = ("Segoe UI", 16, "bold")
MONO = ("Consolas", 9)


def find_ports():
    """Return (port names, index of the most likely Pico)."""
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    names = [p.device for p in ports]
    best = 0
    for i, p in enumerate(ports):
        if p.vid == PICO_VID:
            best = i
            break
    return names, best


def parse_pins(text):
    text = text.replace(" ", "")
    if not text:
        return []
    return [int(x) for x in text.split(",") if x != ""]


class Link:
    """Serial connection with a background reader thread feeding a queue."""

    def __init__(self, port, out):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.out = out
        self.alive = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.ser.read(4096)
            except (serial.SerialException, OSError) as e:
                if self.alive:
                    self.out.put(("error", "serial error: %s" % e))
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.out.put(("line", line.decode("utf-8", "replace").rstrip("\r")))

    def start_stream(self, args):
        self.ser.write(b"\r\x03\x03")          # Ctrl-C twice: stop whatever is running
        time.sleep(0.3)
        self.ser.write(b"\x04")                # Ctrl-D: soft reset, so freshly uploaded files are used
        time.sleep(1.2)
        self.ser.write(b"\x03\x03")            # stop main.py, which the soft reset starts again
        time.sleep(0.3)
        cmd = "from tests.stream_sensors import stream; stream(%s)\r" % args
        self.ser.write(cmd.encode())

    def stop_stream(self):
        try:
            self.ser.write(b"\x03")
        except (serial.SerialException, OSError):
            pass

    def close(self):
        self.stop_stream()
        self.alive = False
        time.sleep(0.15)
        try:
            self.ser.close()
        except (serial.SerialException, OSError):
            pass


class Lamp:
    """A coloured dot with a caption underneath."""

    def __init__(self, parent, caption, size=22):
        self.frame = tk.Frame(parent, bg=BG)
        self.canvas = tk.Canvas(self.frame, width=size + 4, height=size + 4, bg=BG, highlightthickness=0)
        self.dot = self.canvas.create_oval(2, 2, size + 2, size + 2, fill=GREY, outline="#777")
        self.canvas.pack()
        self.label = tk.Label(self.frame, text=caption, bg=BG, font=("Segoe UI", 8), justify="center")
        self.label.pack()

    def set(self, colour, caption=None):
        self.canvas.itemconfig(self.dot, fill=colour)
        if caption is not None:
            self.label.config(text=caption)


CURVE_COLOURS = ("#1f77b4", "#ff7f0e", "#2ca02c", "#d62728")
STATUS_TEXT = {11: "valid", 4: "no target"}


def status_text(status):
    if status is None:
        return "-"
    return "%d (%s)" % (status, STATUS_TEXT.get(status, "check"))


def stats(values):
    """(mean, standard deviation, min, max, n) of a list of numbers, or None if empty."""
    n = len(values)
    if n == 0:
        return None
    mean = sum(values) / float(n)
    sd = (sum((v - mean) ** 2 for v in values) / n) ** 0.5
    return mean, sd, min(values), max(values), n


def suggest_offset(raws, true_mm):
    """Calibration: (offset_mm, mean_raw, sd, n) so that raw + offset == true distance."""
    s = stats(raws)
    if s is None:
        return None
    mean, sd, _, _, n = s
    return int(round(true_mm - mean)), mean, sd, n


class ToFPage(tk.Frame):
    """Distance tab: one card per sensor, a continuous distance-vs-time curve and calibration."""

    def __init__(self, parent):
        tk.Frame.__init__(self, parent, bg=BG, padx=8, pady=4)
        self.hist = {}       # name -> list of (time, available, filtered mm, raw mm, status)
        self.counts = {}     # name -> list of (time, sample_count) for the rate
        self.cards = {}
        self.scan = {}
        self.calib = None    # (name, true_mm, end_time, raws)
        self.max_mm = config.TOF_MAX_VALID_MM
        self.paused = False
        self._build()

    def _build(self):
        self.cards_frame = tk.Frame(self, bg=BG)
        self.cards_frame.pack(fill="x", pady=(2, 6))
        tk.Label(self.cards_frame, text="Not streaming yet. Connect, tick ToF and press Apply.",
                 bg=BG, fg="#777", font=FONT).pack(anchor="w")

        ctrl = tk.Frame(self, bg=BG)
        ctrl.pack(fill="x")
        tk.Label(ctrl, text="Range (mm)", bg=BG, font=FONT).pack(side="left")
        self.range_var = tk.StringVar(value="1200")
        box = ttk.Combobox(ctrl, textvariable=self.range_var, values=("300", "600", "1200", "2000"),
                           width=6, state="readonly")
        box.pack(side="left", padx=(4, 12))
        box.bind("<<ComboboxSelected>>", lambda e: self.redraw())
        tk.Label(ctrl, text="Time window (s)", bg=BG, font=FONT).pack(side="left")
        self.window_var = tk.StringVar(value="10")
        box = ttk.Combobox(ctrl, textvariable=self.window_var, values=("5", "10", "30"), width=4, state="readonly")
        box.pack(side="left", padx=(4, 12))
        box.bind("<<ComboboxSelected>>", lambda e: self.redraw())
        self.raw_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctrl, text="show raw readings (dashed)", variable=self.raw_var,
                        command=self.redraw).pack(side="left", padx=6)
        self.pause_btn = ttk.Button(ctrl, text="Pause", command=self.toggle_pause)
        self.pause_btn.pack(side="left", padx=6)

        self.chart = tk.Canvas(self, bg="white", highlightthickness=1, highlightbackground="#999", height=300)
        self.chart.pack(fill="both", expand=True, pady=6)
        self.chart.bind("<Configure>", lambda e: self.redraw())

        cal = tk.LabelFrame(self, text="Offset calibration", bg=BG, font=FONT_BOLD, padx=8, pady=6)
        cal.pack(fill="x")
        tk.Label(cal, text="Put a flat target exactly this far from the sensor window, then press "
                           "Calibrate (averages 2 s of valid raw readings).", bg=BG, font=FONT).pack(anchor="w")
        row = tk.Frame(cal, bg=BG)
        row.pack(fill="x", pady=4)
        tk.Label(row, text="sensor", bg=BG, font=FONT).pack(side="left")
        self.cal_sensor = ttk.Combobox(row, width=8, state="readonly")
        self.cal_sensor.pack(side="left", padx=(4, 12))
        tk.Label(row, text="true distance (mm)", bg=BG, font=FONT).pack(side="left")
        self.cal_true = tk.StringVar(value="100")
        ttk.Entry(row, textvariable=self.cal_true, width=6).pack(side="left", padx=4)
        ttk.Button(row, text="Calibrate", command=self.start_calibration).pack(side="left", padx=8)
        self.cal_result = tk.Label(cal, text="", bg=BG, font=FONT_BOLD, justify="left", anchor="w")
        self.cal_result.pack(anchor="w")

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")
        if not self.paused:
            self.redraw()

    def _bus_text(self, name):
        row = self.scan.get(name)
        if row is None:
            return ""
        _, bus_id, sda, scl, addr, found = row
        where = "I2C%s  SDA (D) GP%d = pin %s,  SCL (C) GP%d = pin %s" % (
            "soft" if bus_id is None else bus_id, sda, config.physical_pin(sda),
            scl, config.physical_pin(scl))
        if found is None:
            state = "bus scan failed"
        elif addr in found:
            state = "bus scan: found 0x%02X - wiring OK" % addr
        elif found:
            state = "bus scan: found %s but not 0x%02X" % (", ".join("0x%02X" % a for a in found), addr)
        else:
            state = "bus scan: nothing found - check + / - / C / D wiring"
        return where + "\n" + state

    def on_hello(self, h):
        self.max_mm = h.get("max_mm", self.max_mm)
        for w in self.cards_frame.winfo_children():
            w.destroy()
        self.cards = {}
        self.hist = {}
        self.counts = {}
        self.calib = None
        self.scan = dict((row[0], row) for row in h.get("scan", []))
        names = h.get("tof", [])
        if not names:
            tk.Label(self.cards_frame, text="ToF is off. Tick 'ToF' in the Wiring row and press Apply.",
                     bg=BG, fg="#777", font=FONT).pack(anchor="w")
            self.cal_sensor["values"] = ()
            self.redraw()
            return
        for i, name in enumerate(names):
            card = tk.LabelFrame(self.cards_frame, text=" %s " % name, bg=BG, font=FONT_BOLD,
                                 fg=CURVE_COLOURS[i % 4], padx=10, pady=6)
            card.pack(side="left", fill="both", expand=True, padx=(0 if i == 0 else 6, 0))
            big = tk.Label(card, text="-", font=("Segoe UI", 28, "bold"), bg=BG, anchor="w")
            big.pack(anchor="w")
            info = tk.Label(card, text="", font=FONT, bg=BG, justify="left", anchor="w")
            info.pack(anchor="w")
            bus = tk.Label(card, text=self._bus_text(name), font=("Segoe UI", 9), bg=BG, fg="#555",
                           justify="left", anchor="w")
            bus.pack(anchor="w", pady=(4, 0))
            self.cards[name] = (big, info)
            self.hist[name] = []
            self.counts[name] = []
        self.cal_sensor["values"] = names
        self.cal_sensor.current(0)
        self.redraw()

    def on_sample(self, rows, now=None):
        if now is None:
            now = time.time()
        for row in rows:
            name, available, mm, raw, status, errors = row[:6]
            samples = row[6] if len(row) > 6 else None
            if name not in self.cards:
                continue
            hist = self.hist[name]
            hist.append((now, available, mm, raw, status))
            while hist and now - hist[0][0] > 31.0:
                hist.pop(0)
            rate = None
            if samples is not None:
                counts = self.counts[name]
                counts.append((now, samples))
                while len(counts) > 2 and now - counts[0][0] > 1.5:
                    counts.pop(0)
                if len(counts) > 1 and counts[-1][0] > counts[0][0]:
                    rate = (counts[-1][1] - counts[0][1]) / (counts[-1][0] - counts[0][0])

            big, info = self.cards[name]
            if not available:
                big.config(text="n/a", fg=RED)
            elif mm is None:
                big.config(text="no target", fg="#777")
            else:
                big.config(text="%d mm" % mm, fg="black")
            recent = [h[2] for h in hist if h[1] and h[2] is not None and now - h[0] <= 2.0]
            s = stats(recent)
            text = "raw %s mm    status %s\n" % ("-" if raw is None else raw, status_text(status))
            text += "rate %s Hz    I2C errors %d\n" % ("-" if rate is None else "%.0f" % rate, errors)
            if s is None:
                text += "last 2 s: no valid readings"
            else:
                text += "last 2 s: mean %.0f   sd %.1f   min %d   max %d   (n=%d)" % s
            info.config(text=text)

            if self.calib is not None and self.calib[0] == name and available \
                    and status == 11 and raw is not None and raw < 8190:
                self.calib[3].append(raw)
        if self.calib is not None and now >= self.calib[2]:
            self.finish_calibration()
        if not self.paused:
            self.redraw(now)

    def start_calibration(self):
        name = self.cal_sensor.get()
        if not name:
            self.cal_result.config(text="No ToF sensor is streaming.", fg=RED)
            return
        try:
            true_mm = float(self.cal_true.get())
        except ValueError:
            self.cal_result.config(text="Enter the true distance in mm.", fg=RED)
            return
        self.calib = (name, true_mm, time.time() + 2.0, [])
        self.cal_result.config(text="Collecting 2 s of readings from %s ..." % name, fg="black")

    def finish_calibration(self):
        name, true_mm, _, raws = self.calib
        self.calib = None
        if len(raws) < 5:
            self.cal_result.config(text="Only %d valid readings - is the target in view and within range?"
                                        % len(raws), fg=RED)
            return
        offset, mean, sd, n = suggest_offset(raws, true_mm)
        self.cal_result.config(
            text="%s: mean raw %.1f mm (sd %.1f, n=%d) at %.0f mm  ->  set \"offset_mm\": %d for \"%s\" "
                 "in config.TOF_SENSORS" % (name, mean, sd, n, true_mm, offset, name), fg="#1e8449")

    def redraw(self, now=None):
        c = self.chart
        if now is None:
            now = time.time()
        w = max(c.winfo_width(), 200)
        h = max(c.winfo_height(), 120)
        c.delete("all")
        left, right, top, bottom = 56, 16, 28, 28
        pw = w - left - right
        ph = h - top - bottom
        rng = float(self.range_var.get())
        win = float(self.window_var.get())

        step = 50 if rng <= 300 else 100 if rng <= 600 else 200 if rng <= 1200 else 500
        y = 0
        while y <= rng:
            py = top + ph - ph * y / rng
            c.create_line(left, py, left + pw, py, fill="#e6e6e6")
            c.create_text(left - 6, py, text="%d" % y, anchor="e", font=("Segoe UI", 8))
            y += step
        c.create_text(12, top + ph / 2, text="distance (mm)", angle=90, font=("Segoe UI", 8))
        tick = 1 if win <= 10 else 5
        for s in range(0, int(win) + 1, tick):
            px = left + pw - pw * s / win
            c.create_line(px, top, px, top + ph, fill="#f2f2f2")
            c.create_text(px, top + ph + 12, text=("-%d s" % s) if s else "now", font=("Segoe UI", 8))
        c.create_rectangle(left, top, left + pw, top + ph, outline="#999")

        def point(t, v):
            v = min(max(v, 0), rng)
            return left + pw - pw * (now - t) / win, top + ph - ph * v / rng

        for i, (name, hist) in enumerate(self.hist.items()):
            colour = CURVE_COLOURS[i % 4]
            filtered = []
            raw_pts = []
            for t, available, mm, raw, status in hist:
                if now - t > win or not available:
                    continue
                # "No target" is drawn at the top edge, so the curve stays continuous.
                filtered.extend(point(t, rng if mm is None else mm))
                if raw is not None:
                    raw_pts.extend(point(t, rng if raw >= 8190 else raw))
            if self.raw_var.get() and len(raw_pts) >= 4:
                c.create_line(*raw_pts, fill=colour, width=1, dash=(3, 3))
            if len(filtered) >= 4:
                c.create_line(*filtered, fill=colour, width=3, smooth=True)
            lx = left + 10 + i * 120
            c.create_line(lx, top - 14, lx + 24, top - 14, fill=colour, width=3)
            c.create_text(lx + 30, top - 14, text=name, anchor="w", font=FONT)
        c.create_text(left + pw, top - 14, anchor="e", fill="#777", font=("Segoe UI", 8),
                      text="solid = filtered distance, dashed = raw, top edge = no target / beyond range")


def classify_hall(mvs):
    """Guess the sensor kind from recent readings (mV)."""
    if len(mvs) < 10:
        return "collecting readings..."
    low = sum(1 for v in mvs if v < 400)
    high = sum(1 for v in mvs if v > 2900)
    spread = max(mvs) - min(mvs)
    if low + high >= 0.9 * len(mvs):
        return "looks DIGITAL: output only near 0 V / 3.3 V"
    if spread >= 200:
        return "looks ANALOG: voltage changes continuously with the field"
    return "steady in-between voltage: analog sensor with no field nearby (bring the magnet within 1 cm)"


class MagPage(tk.Frame):
    """Magnetic tab: live value, detection state and a continuous voltage-vs-time curve."""

    def __init__(self, parent):
        tk.Frame.__init__(self, parent, bg=BG, padx=8, pady=4)
        self.hist = []          # (time, mV, detected)
        self.setup = None       # [pin, mode, baseline, on_mv, off_mv]
        self.baseline = None
        self.paused = False
        self._build()

    def _build(self):
        card = tk.LabelFrame(self, text=" magnetic sensor ", bg=BG, font=FONT_BOLD, padx=10, pady=6)
        card.pack(fill="x", pady=(2, 6))
        left = tk.Frame(card, bg=BG)
        left.pack(side="left", fill="y")
        self.big = tk.Label(left, text="-", font=("Segoe UI", 28, "bold"), bg=BG, anchor="w")
        self.big.pack(anchor="w")
        self.lamp = Lamp(left, "no magnet", 30)
        self.lamp.frame.pack(anchor="w", pady=(2, 0))
        self.info = tk.Label(card, text="Not streaming yet. Connect, then the magnetic sensor starts with "
                                        "the 'Mag' pin of the Wiring row.", font=FONT, bg=BG,
                             justify="left", anchor="nw")
        self.info.pack(side="left", fill="both", expand=True, padx=20)

        ctrl = tk.Frame(self, bg=BG)
        ctrl.pack(fill="x")
        tk.Label(ctrl, text="Time window (s)", bg=BG, font=FONT).pack(side="left")
        self.window_var = tk.StringVar(value="10")
        box = ttk.Combobox(ctrl, textvariable=self.window_var, values=("5", "10", "30"), width=4, state="readonly")
        box.pack(side="left", padx=(4, 12))
        box.bind("<<ComboboxSelected>>", lambda e: self.redraw())
        self.pause_btn = ttk.Button(ctrl, text="Pause", command=self.toggle_pause)
        self.pause_btn.pack(side="left", padx=6)
        tk.Label(ctrl, text="Press Apply with no magnet nearby to re-zero the baseline.", bg=BG, fg="#777",
                 font=FONT).pack(side="left", padx=10)

        self.chart = tk.Canvas(self, bg="white", highlightthickness=1, highlightbackground="#999", height=320)
        self.chart.pack(fill="both", expand=True, pady=6)
        self.chart.bind("<Configure>", lambda e: self.redraw())

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")
        if not self.paused:
            self.redraw()

    def on_hello(self, h):
        self.setup = h.get("mag")
        self.hist = []
        if not self.setup:
            self.baseline = None
            self.big.config(text="off", fg="#777")
            self.info.config(text="Magnetic sensor is off. Put its GP number in the 'Mag' field and press Apply.")
        else:
            pin, mode, baseline, on_mv, off_mv = self.setup
            self.baseline = baseline
            self.info.config(text="GP%d (pin %s), read as %s" % (pin, config.physical_pin(pin), mode))
        self.redraw()

    def on_sample(self, row, now=None):
        if now is None:
            now = time.time()
        if not self.setup:
            return
        value, detected, field, polarity, detections, baseline = row
        pin, mode, _, on_mv, off_mv = self.setup
        self.baseline = baseline
        if mode == "analog":
            mv = value
        else:
            mv = 3300 if value else 0
        self.hist.append((now, mv, detected))
        while self.hist and now - self.hist[0][0] > 31.0:
            self.hist.pop(0)
        recent = [h[1] for h in self.hist if now - h[0] <= 3.0]
        self.big.config(text="%d mV" % mv if mode == "analog" else ("LOW" if not value else "HIGH"),
                        fg="black")
        self.lamp.set(RED if detected else OFF, "MAGNET" if detected else "no magnet")
        lines = ["GP%d (pin %s), read as %s" % (pin, config.physical_pin(pin), mode)]
        if mode == "analog":
            lines.append("baseline (zero field) %s mV    field %+d mV    polarity %s" % (
                baseline, field if field is not None else 0,
                {1: "+ (one pole)", -1: "- (other pole)"}.get(polarity, "-")))
            lines.append("detect when |field| >= %d mV, release below %d mV" % (on_mv, off_mv))
        lines.append("detections so far: %d" % detections)
        if recent:
            lines.append("last 3 s: min %d  max %d  mean %d mV" % (
                min(recent), max(recent), sum(recent) // len(recent)))
        lines.append(classify_hall(recent))
        self.info.config(text="\n".join(lines))
        if not self.paused:
            self.redraw(now)

    def redraw(self, now=None):
        c = self.chart
        if now is None:
            now = time.time()
        w = max(c.winfo_width(), 200)
        h = max(c.winfo_height(), 120)
        c.delete("all")
        left, right, top, bottom = 56, 16, 28, 28
        pw = w - left - right
        ph = h - top - bottom
        rng = 3300.0
        win = float(self.window_var.get())
        for y in range(0, 3301, 500):
            py = top + ph - ph * y / rng
            c.create_line(left, py, left + pw, py, fill="#e6e6e6")
            c.create_text(left - 6, py, text="%d" % y, anchor="e", font=("Segoe UI", 8))
        c.create_text(12, top + ph / 2, text="voltage (mV)", angle=90, font=("Segoe UI", 8))
        tick = 1 if win <= 10 else 5
        for s in range(0, int(win) + 1, tick):
            px = left + pw - pw * s / win
            c.create_line(px, top, px, top + ph, fill="#f2f2f2")
            c.create_text(px, top + ph + 12, text=("-%d s" % s) if s else "now", font=("Segoe UI", 8))
        c.create_rectangle(left, top, left + pw, top + ph, outline="#999")

        def ypos(v):
            return top + ph - ph * min(max(v, 0), rng) / rng

        if self.setup and self.setup[1] == "analog" and self.baseline is not None:
            on_mv = self.setup[3]
            c.create_line(left, ypos(self.baseline), left + pw, ypos(self.baseline), fill="#555", dash=(6, 4))
            c.create_text(left + 4, ypos(self.baseline) - 8, text="baseline", anchor="w", fill="#555",
                          font=("Segoe UI", 8))
            for v in (self.baseline + on_mv, self.baseline - on_mv):
                c.create_line(left, ypos(v), left + pw, ypos(v), fill=RED, dash=(2, 4))
            c.create_text(left + 4, ypos(self.baseline + on_mv) - 8, text="detect threshold", anchor="w",
                          fill=RED, font=("Segoe UI", 8))
        points = []
        for t, mv, detected in self.hist:
            if now - t > win:
                continue
            points.extend((left + pw - pw * (now - t) / win, ypos(mv)))
        if len(points) >= 4:
            c.create_line(*points, fill="#8e44ad", width=3, smooth=True)
        c.create_line(left + 10, top - 14, left + 34, top - 14, fill="#8e44ad", width=3)
        c.create_text(left + 40, top - 14, text="sensor output", anchor="w", font=FONT)


class Viewer(tk.Tk):
    def __init__(self, options=None):
        tk.Tk.__init__(self)
        self.options = options
        self.title("IDP sensor viewer")
        self.configure(bg=BG)
        self.geometry("1180x760")
        self.minsize(980, 640)
        self.q = queue.Queue()
        self.link = None
        self.record = None
        if options is not None and options.record:
            self.record = open(options.record, "a", encoding="utf-8")
        self.last_sample = 0.0
        self.monitor = []
        self.pin_lamps = []
        self.pin_last = []
        self.pin_changes = []
        self.tof_rows = {}
        self.max_mm = config.TOF_MAX_VALID_MM
        self._build()
        self.refresh_ports()
        if options is not None:
            if options.line is not None:
                self.line_var.set(options.line)
            if options.crash is not None:
                self.crash_var.set(options.crash)
            if options.ir is not None:
                self.ir_var.set(options.ir)
            if options.no_tof:
                self.tof_var.set(False)
            if options.port:
                self.port_var.set(options.port)
            if options.mag is not None:
                self.mag_var.set(options.mag)
            if options.tab == "tof":
                self.notebook.select(self.tof_page)
            elif options.tab == "mag":
                self.notebook.select(self.mag_page)
            if options.connect:
                self.after(300, self.toggle_connect)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(30, self.poll)

    # ------------------------------------------------------------------ layout
    def _box(self, parent, title):
        return tk.LabelFrame(parent, text=title, bg=BG, font=FONT_BOLD, padx=8, pady=6)

    def _build(self):
        top = tk.Frame(self, bg=BG, padx=8, pady=6)
        top.pack(fill="x")
        tk.Label(top, text="Port", bg=BG, font=FONT).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=12, state="readonly")
        self.port_box.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left", padx=2)
        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=6)
        self.status = tk.Label(top, text="not connected", bg=BG, font=FONT_BOLD, fg="#555")
        self.status.pack(side="left", padx=10)

        wiring = tk.Frame(self, bg=BG, padx=8)
        wiring.pack(fill="x")
        tk.Label(wiring, text="Wiring (GP numbers):", bg=BG, font=FONT_BOLD).pack(side="left")
        self.line_var = tk.StringVar(value="%d,%d,%d,%d" % (
            config.LINE_OUTER_LEFT_PIN, config.LINE_INNER_LEFT_PIN,
            config.LINE_INNER_RIGHT_PIN, config.LINE_OUTER_RIGHT_PIN))
        self.crash_var = tk.StringVar(value="%d,%d" % (config.CRASH_LEFT_PIN, config.CRASH_RIGHT_PIN))
        self.ir_var = tk.StringVar(value=str(config.IR_RX_PIN))
        self.mag_var = tk.StringVar(value=str(config.MAG_PIN))
        self.tof_var = tk.BooleanVar(value=True)
        for label, var, width in (("line L->R", self.line_var, 14), ("crash L,R", self.crash_var, 8),
                                  ("IR", self.ir_var, 4), ("Mag", self.mag_var, 4)):
            tk.Label(wiring, text=label, bg=BG, font=FONT).pack(side="left", padx=(10, 2))
            ttk.Entry(wiring, textvariable=var, width=width).pack(side="left")
        ttk.Checkbutton(wiring, text="ToF (pins from config.py)", variable=self.tof_var).pack(side="left", padx=10)
        ttk.Button(wiring, text="Apply", command=self.apply).pack(side="left", padx=6)
        tk.Label(wiring, text="(empty field = sensor group off)", bg=BG, fg="#777", font=FONT).pack(side="left")

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=4)
        body = tk.Frame(self.notebook, bg=BG, padx=8, pady=4)
        self.notebook.add(body, text="  Overview  ")
        self.tof_page = ToFPage(self.notebook)
        self.notebook.add(self.tof_page, text="  Distance (ToF)  ")
        self.mag_page = MagPage(self.notebook)
        self.notebook.add(self.mag_page, text="  Magnetic  ")
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(2, weight=1)

        # Pin monitor
        pins = self._box(body, "Pin monitor  (green = HIGH, dark = LOW, number = level changes)")
        pins.grid(row=0, column=0, columnspan=2, sticky="ew", pady=4)
        self.pins_frame = tk.Frame(pins, bg=BG)
        self.pins_frame.pack(fill="x")
        tk.Label(pins, text="Unused pins have the pull-up on: an unconnected pin reads a steady HIGH. "
                            "A connected sensor makes its pin change.", bg=BG, fg="#666",
                 font=("Segoe UI", 8)).pack(anchor="w")

        # Line
        line = self._box(body, "Line sensors (SEN0017)")
        line.grid(row=1, column=0, sticky="nsew", padx=(0, 4), pady=4)
        self.line_canvas = tk.Canvas(line, width=420, height=150, bg=BG, highlightthickness=0)
        self.line_canvas.pack()
        self.line_cells = []
        for i, name in enumerate(("outer L", "inner L", "inner R", "outer R")):
            x = 20 + i * 100
            cell = self.line_canvas.create_rectangle(x, 10, x + 80, 60, fill=GREY, outline="#777")
            self.line_canvas.create_text(x + 40, 72, text=name, font=FONT)
            self.line_cells.append(cell)
        self.line_canvas.create_line(20, 110, 400, 110, fill="#888", width=2)
        for v, label in ((-1, "-1 (line left)"), (0, "0"), (1, "+1 (line right)")):
            x = 210 + v * 190
            self.line_canvas.create_line(x, 104, x, 116, fill="#888")
            self.line_canvas.create_text(x, 128, text=label, font=("Segoe UI", 8))
        self.err_marker = self.line_canvas.create_oval(204, 104, 216, 116, fill=AMBER, outline="")
        self.line_info = tk.Label(line, text="-", bg=BG, font=FONT, justify="left")
        self.line_info.pack(anchor="w")

        # ToF
        tof = self._box(body, "Distance (VL53L0X)")
        tof.grid(row=1, column=1, sticky="nsew", padx=(4, 0), pady=4)
        self.tof_frame = tk.Frame(tof, bg=BG)
        self.tof_frame.pack(fill="both", expand=True)

        # Crash + IR + log
        lower = tk.Frame(body, bg=BG)
        lower.grid(row=2, column=0, columnspan=2, sticky="nsew")
        lower.columnconfigure(2, weight=1)
        lower.rowconfigure(0, weight=1)

        crash = self._box(lower, "Crash (SEN0138)")
        crash.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=4)
        self.crash_lamps = [Lamp(crash, "left", 36), Lamp(crash, "right", 36)]
        for lamp in self.crash_lamps:
            lamp.frame.pack(side="left", padx=10)

        ir = self._box(lower, "IR receiver (TSOP4838)")
        ir.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        self.ir_lamp = Lamp(ir, "signal", 36)
        self.ir_lamp.frame.pack(side="left", padx=10)
        self.ir_info = tk.Label(ir, text="-", bg=BG, font=FONT, justify="left", wraplength=220)
        self.ir_info.pack(side="left")

        logbox = self._box(lower, "Events / messages from the Pico")
        logbox.grid(row=0, column=2, sticky="nsew", padx=(4, 0), pady=4)
        self.log = ScrolledText(logbox, height=10, font=MONO, wrap="word")
        self.log.pack(fill="both", expand=True)
        self.log.tag_config("err", foreground=RED)
        self.log.tag_config("ev", foreground="#1f6fb2")
        ttk.Button(logbox, text="Clear", command=lambda: self.log.delete("1.0", "end")).pack(anchor="e")

    # --------------------------------------------------------------- actions
    def refresh_ports(self):
        names, best = find_ports()
        self.port_box["values"] = names
        if names:
            self.port_box.current(best)

    def stream_args(self):
        try:
            line = parse_pins(self.line_var.get())
            crash = parse_pins(self.crash_var.get())
            ir = parse_pins(self.ir_var.get())
            mag = parse_pins(self.mag_var.get())
        except ValueError:
            self.write_log("Wiring fields must be GP numbers separated by commas", "err")
            return None
        all_pins = line + crash + ir + mag
        for gp in set(all_pins):
            if all_pins.count(gp) > 1:
                self.write_log("Warning: GP%d is used by more than one sensor" % gp, "err")
            if gp in config.BOARD_PINS:
                self.write_log("Warning: GP%d is a robot board pin (%s)" % (gp, config.BOARD_PINS[gp]), "err")
        return "line_pins=%r, crash_pins=%r, ir_pin=%d, use_tof=%r, mag_pin=%d" % (
            line, crash, ir[0] if ir else -1, bool(self.tof_var.get()), mag[0] if mag else -1)

    def toggle_connect(self):
        if self.link is not None:
            self.disconnect()
            return
        port = self.port_var.get()
        if not port:
            self.write_log("No serial port found. Is the Pico plugged in?", "err")
            return
        args = self.stream_args()
        if args is None:
            return
        try:
            self.link = Link(port, self.q)
        except (serial.SerialException, OSError) as e:
            self.write_log("Cannot open %s: %s\n(Close MicroPico / other serial terminals first.)" % (port, e), "err")
            self.link = None
            return
        self.connect_btn.config(text="Disconnect")
        self.status.config(text="connected to %s, starting..." % port, fg="#555")
        self.last_sample = time.time()
        self.link.start_stream(args)

    def apply(self):
        if self.link is None:
            return
        args = self.stream_args()
        if args is None:
            return
        self.link.stop_stream()
        time.sleep(0.3)
        self.link.start_stream(args)
        self.status.config(text="restarting with new wiring...", fg="#555")

    def disconnect(self):
        if self.link is not None:
            self.link.close()
            self.link = None
        self.connect_btn.config(text="Connect")
        self.status.config(text="not connected", fg="#555")

    def on_close(self):
        self.disconnect()
        if self.record is not None:
            self.record.close()
            self.record = None
        self.destroy()

    def write_log(self, text, tag=None):
        self.log.insert("end", text + "\n", tag)
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 600:
            self.log.delete("1.0", "%d.0" % (lines - 500))
        self.log.see("end")

    # ------------------------------------------------------------ data in
    def poll(self):
        latest = None
        try:
            while True:
                kind, text = self.q.get_nowait()
                if self.record is not None and kind == "line":
                    self.record.write("%.3f %s\n" % (time.time(), text))
                if kind == "error":
                    self.write_log(text, "err")
                    self.disconnect()
                elif text.startswith("@S "):
                    try:
                        msg = json.loads(text[3:])
                    except ValueError:
                        continue
                    for ev in msg.get("ev", []):
                        self.write_log(time.strftime("%H:%M:%S ") + ev, "ev")
                    latest = msg
                elif text.startswith("@H "):
                    try:
                        self.on_hello(json.loads(text[3:]))
                    except ValueError:
                        pass
                elif text.startswith("@E"):
                    self.write_log("stream stopped")
                elif text.strip() and not text.startswith(">>>") and "stream(" not in text:
                    bad = "Error" in text or "Traceback" in text or "File " in text
                    self.write_log(text, "err" if bad else None)
        except queue.Empty:
            pass
        if latest is not None:
            self.last_sample = time.time()
            self.on_sample(latest)
        if self.link is not None and time.time() - self.last_sample > 2.0:
            self.status.config(text="no data - is the code uploaded? see messages", fg=RED)
        self.after(30, self.poll)

    @staticmethod
    def role(roles, gp):
        if gp in roles:
            return roles[gp]
        if gp in config.BOARD_PINS:
            return "board pin"
        return "free"

    def on_hello(self, h):
        self.status.config(text="streaming", fg="#1e8449")
        self.write_log("stream started: line %s, crash %s, IR %s, ToF %s" % (
            h.get("line"), h.get("crash"), h.get("ir"), h.get("tof")))
        for e in h.get("errors", []):
            self.write_log("Pico: " + e, "err")
        self.max_mm = h.get("max_mm", self.max_mm)
        self.tof_page.on_hello(h)
        self.mag_page.on_hello(h)
        # pin monitor lamps
        for lamp in self.pin_lamps:
            lamp.frame.destroy()
        self.monitor = h.get("monitor", [])
        phys = h.get("phys", [None] * len(self.monitor))
        roles = {}
        for i, gp in enumerate(h.get("line", [])):
            roles[gp] = ("line OL", "line IL", "line IR", "line OR")[i]
        for i, gp in enumerate(h.get("crash", [])):
            roles[gp] = ("crash L", "crash R")[i]
        if h.get("ir", -1) >= 0:
            roles[h["ir"]] = "IR"
        if h.get("mag"):
            roles[h["mag"][0]] = "magnet"
        self.pin_lamps = []
        for i, gp in enumerate(self.monitor):
            caption = "GP%d\npin %s\n%s" % (gp, phys[i], self.role(roles, gp))
            lamp = Lamp(self.pins_frame, caption)
            lamp.frame.grid(row=0, column=i, padx=3)
            self.pin_lamps.append(lamp)
        self.pin_last = [None] * len(self.monitor)
        self.pin_changes = [0] * len(self.monitor)
        self.pin_captions = [self.role(roles, gp) for gp in self.monitor]
        self.pin_phys = phys
        # ToF rows
        for row in self.tof_rows.values():
            for w in row:
                w.destroy()
        self.tof_rows = {}
        for r, name in enumerate(h.get("tof", [])):
            title = tk.Label(self.tof_frame, text=name, bg=BG, font=FONT_BOLD, width=6, anchor="w")
            value = tk.Label(self.tof_frame, text="-", bg=BG, font=FONT_BIG, width=9, anchor="e")
            bar = tk.Canvas(self.tof_frame, width=260, height=22, bg="white", highlightthickness=1,
                            highlightbackground="#999")
            fill = bar.create_rectangle(0, 0, 0, 22, fill="#3498db", outline="")
            info = tk.Label(self.tof_frame, text="", bg=BG, font=("Segoe UI", 8), anchor="w")
            title.grid(row=2 * r, column=0, sticky="w")
            value.grid(row=2 * r, column=1)
            bar.grid(row=2 * r, column=2, padx=6)
            info.grid(row=2 * r + 1, column=0, columnspan=3, sticky="w", pady=(0, 8))
            bar.fill = fill
            self.tof_rows[name] = (title, value, bar, info)
        if not h.get("tof"):
            lbl = tk.Label(self.tof_frame, text="ToF off", bg=BG, fg="#777", font=FONT)
            lbl.grid(row=0, column=0)
            self.tof_rows["_off"] = (lbl,)

    def on_sample(self, m):
        # pins
        for i, level in enumerate(m.get("p", [])):
            if i >= len(self.pin_lamps):
                break
            if self.pin_last[i] is not None and level != self.pin_last[i]:
                self.pin_changes[i] += 1
            self.pin_last[i] = level
            caption = "GP%d\npin %s\n%s\n%d" % (self.monitor[i], self.pin_phys[i],
                                               self.pin_captions[i], self.pin_changes[i])
            self.pin_lamps[i].set(ON if level else OFF, caption)
        # line
        line = m.get("line")
        if line:
            bits, err, state, in_junction, exits = line
            for i, mask in enumerate((8, 4, 2, 1)):
                self.line_canvas.itemconfig(self.line_cells[i], fill="white" if bits & mask else "black")
            x = 210 + max(-1.0, min(1.0, err)) * 190
            self.line_canvas.coords(self.err_marker, x - 6, 104, x + 6, 116)
            self.line_info.config(text="error %+.2f   state %s   %s   junctions passed %d" % (
                err, state, "ON JUNCTION" if in_junction else "", exits))
        else:
            for cell in self.line_cells:
                self.line_canvas.itemconfig(cell, fill=GREY)
            self.line_info.config(text="line sensors off (needs 4 pins)")
        # ToF
        if "tof" in m:
            self.tof_page.on_sample(m["tof"])
        if "mag" in m:
            self.mag_page.on_sample(m["mag"])
        for row in m.get("tof", []):
            name, available, mm, raw, status, errors = row[:6]
            row = self.tof_rows.get(name)
            if row is None:
                continue
            _, value, bar, info = row
            if not available:
                value.config(text="n/a", fg=RED)
                bar.coords(bar.fill, 0, 0, 0, 22)
            elif mm is None:
                value.config(text="no target", fg="#777")
                bar.coords(bar.fill, 0, 0, 0, 22)
            else:
                value.config(text="%d mm" % mm, fg="black")
                bar.coords(bar.fill, 0, 0, 260 * min(mm, self.max_mm) / float(self.max_mm), 22)
            info.config(text="raw %s   status %s   I2C errors %s" % (raw, status, errors))
        # crash
        crash = m.get("crash")
        if crash:
            for lamp, (raw, pressed, hits), side in zip(self.crash_lamps, crash, ("left", "right")):
                lamp.set(RED if pressed else ON, "%s\n%s\nraw %d  hits %d" % (
                    side, "PRESSED" if pressed else "released", raw, hits))
        else:
            for lamp, side in zip(self.crash_lamps, ("left", "right")):
                lamp.set(GREY, "%s\noff" % side)
        # IR
        ir = m.get("ir")
        if ir:
            present, raw, edges = ir
            self.ir_lamp.set(AMBER if present else OFF, "PRESENT" if present else "absent")
            self.ir_info.config(text="output %d   edges %d" % (raw, edges))
        else:
            self.ir_lamp.set(GREY, "off")
            self.ir_info.config(text="IR off")


def main():
    parser = argparse.ArgumentParser(description="Live IDP sensor viewer")
    parser.add_argument("--port", help="serial port, e.g. COM8 (default: auto-detect the Pico)")
    parser.add_argument("--line", help='line sensor GPs left to right, e.g. "18,19,20,21"; "" = off')
    parser.add_argument("--crash", help='crash sensor GPs "left,right"; "" = off')
    parser.add_argument("--ir", help='IR receiver GP; "" = off')
    parser.add_argument("--no-tof", action="store_true", help="do not start the ToF sensors")
    parser.add_argument("--connect", action="store_true", help="connect immediately")
    parser.add_argument("--mag", help='magnetic sensor GP (an ADC pin shows the voltage); "" = off')
    parser.add_argument("--tab", choices=("overview", "tof", "mag"), default="overview", help="tab shown first")
    parser.add_argument("--record", help="append every line received from the Pico to this file")
    Viewer(parser.parse_args()).mainloop()


if __name__ == "__main__":
    main()
