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
"Grab / Servo" (ADDED 2026-10-23) is for calibrating the gripper: the
Left / Right arrow keys move the selected servo (1 / 2 selects it) while the
grab distance, front distance and magnet readings are shown next to it, and
positions can be saved and copied as config lines (see ServoPage).

Command line options set the starting values, e.g. only the pin monitor and IR:
    python tools/sensor_viewer.py --line "" --crash "" --connect
    python tools/sensor_viewer.py --tab tof --connect
    python tools/sensor_viewer.py --tab mag --connect --record log.txt
    python tools/sensor_viewer.py --tab servo --connect
"""

import argparse
import json
import math
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
                chunk = self.ser.read(256)
            except (serial.SerialException, OSError) as e:
                if self.alive:
                    self.out.put(("error", "serial error: %s" % e))
                break

            if not chunk:
                # Don't lose output that has no newline, such as a REPL prompt.
                if buf:
                    self.out.put((
                        "line",
                        buf.decode("utf-8", "replace").rstrip("\r")
                    ))
                    buf = b""
                continue

            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.out.put((
                    "line",
                    line.decode("utf-8", "replace").rstrip("\r")
                ))

    def start_stream(self, args):
        """Interrupt the current program, clear cached project modules,
        and start the sensor stream without resetting the Pico."""
        try:
            # Stop the current program and allow the REPL to recover.
            self.ser.write(b"\x03\x03")
            time.sleep(1.0)

            # Clear project imports in a single REPL command.
            purge = (
                "import sys; "
                "[sys.modules.pop(k, None) for k in list(sys.modules) "
                "if k.split('.')[0] in "
                "('tests', 'sensors', 'control', 'actuators', 'libs', 'config')]"
                "\r"
            )
            self.ser.write(purge.encode("utf-8"))
            time.sleep(0.5)

            # Start streaming.
            cmd = (
                "from tests.stream_sensors import stream; "
                "stream(%s)\r" % args
            )
            self.ser.write(cmd.encode("utf-8"))

        except (serial.SerialException, OSError) as e:
            self.out.put(("error", "Could not start sensor stream: %s" % e))

    def send(self, text):
        """Send a command to the running stream."""
        try:
            self.ser.write((text + "\n").encode("utf-8"))
        except (serial.SerialException, OSError) as e:
            self.out.put(("error", "Could not send command: %s" % e))

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

    
def start_stream(self, args):
    # Stop any currently running program.
    self.ser.write(b"\r\x03\x03")
    time.sleep(0.5)

    # Clear cached project modules without resetting the Pico.
    purge = (
        "import sys\r"
        "for _m in [k for k in sys.modules if k.split('.')[0] in "
        "('tests', 'sensors', 'control', 'actuators', 'libs', 'config')]: "
        "del sys.modules[_m]\r"
        "\r"
    )
    self.ser.write(purge.encode())
    time.sleep(1.0)

    # Start the sensor stream separately.
    cmd = "from tests.stream_sensors import stream; stream(%s)\r" % args
    self.ser.write(cmd.encode())


    def send(self, text):
        """ADDED (2026-10-23): type one command line to the running stream,
        e.g. "servo1 135" (see tests/stream_sensors.py)."""
        try:
            self.ser.write((text + "\n").encode())
        except (serial.SerialException, OSError):
            pass

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


class ServoPage(tk.Frame):
    """ADDED (2026-10-23): gripper calibration tab.

    Move the gripper servos with the keyboard while watching the grab distance
    sensor, the front distance sensor and the magnet sensor:
        Left / Right         selected servo -/+ one step
        Shift + Left / Right five steps at once
        1 / 2                select servo1 / servo2 (or click its card)
        x                    selected servo limp (no pulses)
        r                    safe reset (config.SERVO_RESET; with none set:
                             straight to the middle of the safe range).
                             A limp servo always starts with this reset: the
                             first arrow key press on it sends the reset.
                             No angle outside config.SERVO_SAFE_RANGE is ever
                             sent. (ADDED 2026-10-23)
        Enter                LOCK the selected servo at its angle (grab
                             confirmed): it keeps holding, and the keys,
                             "go to", "limp" and "Move there" cannot move it.
                             The angle is saved as position "grab". Enter
                             again unlocks. (ADDED 2026-10-23)
    The keys only work while this tab is shown and no text field has the
    cursor. "Save position" remembers the current angles (e.g. "open",
    "closed", "lifted"); "Copy as config" puts them on the clipboard as a
    Python dict for config.py.
    """

    STEPS = (1, 2, 5, 10)

    def __init__(self, parent, send, log):
        tk.Frame.__init__(self, parent, bg=BG, padx=8, pady=6, takefocus=1)
        self.send = send       # send(text) -> True if the command went to the Pico
        self.log = log         # log(text, tag) writes to the viewer's message box
        self.names = []        # servo names the Pico reported, e.g. ["servo1", "servo2"]
        self.pins = {}         # name -> GP number
        self.safe = {}         # name -> (low, high) safe range reported by the Pico
        self.reset_paths = {}  # name -> (start, end) of its reset, or None (middle of the safe range)
        self.resetting = {}    # name -> time a reset was sent; its keys wait until the Pico reports an angle
        self.target = {}       # name -> angle the arrow keys step from (degrees)
        self.reported = {}     # name -> last angle the Pico reports (None = limp)
        self.limits = {}       # name -> (min StringVar, max StringVar)
        self.cards = {}        # name -> (frame, canvas, big label, info label)
        self.selected = None
        self.locked = {}       # name -> angle it is locked at (Enter key, grab confirmed)
        self.free = False      # ADDED (2026-10-26): calibration mode (see on_hello)
        self.goto_vars = {}    # name -> the card's "go to" field
        self.saved = []        # [(label, {name: angle}, grab mm)]
        self.grab_mm = None
        self._build()

    # ------------------------------------------------------------ layout
    def _build(self):
        # ADDED (2026-10-26): red banner, shown only in calibration mode
        self.banner = tk.Label(self, bg="#fde2e1", fg=RED, font=FONT_BOLD, justify="left", anchor="w", text=(
            "CALIBRATION MODE (gripper hardware changed): the safe ranges and resets in config.py are NOT used.\n"
            "Only the 270 deg limit (2.5 ms pulse) is kept. A limp servo starts straight at the angle in its "
            "'go to' field: set a safe one first. Find the new limits, save them, then put them in config.py."))
        self.help = tk.Label(self, bg=BG, font=FONT, justify="left", anchor="w", text=(
            "Keys:  Left / Right = selected servo -/+ step     Shift + Left / Right = 5 steps     "
            "1 / 2 = select servo     x = limp     r = safe reset     Enter = LOCK / unlock the grab angle\n"
            "The servos are limp until the first key press, which moves the servo to the middle of its "
            "safe range (config.SERVO_SAFE_RANGE; no angle outside it is ever sent).\n"
            "After typing in a field, press Enter or click a servo card to get the keys back."))
        self.help.pack(fill="x", pady=(0, 4))

        steps = tk.Frame(self, bg=BG)
        steps.pack(fill="x")
        tk.Label(steps, text="Step:", bg=BG, font=FONT_BOLD).pack(side="left")
        self.step_var = tk.IntVar(value=2)
        for s in self.STEPS:
            ttk.Radiobutton(steps, text="%d deg" % s, value=s, variable=self.step_var,
                            takefocus=0).pack(side="left", padx=4)

        middle = tk.Frame(self, bg=BG)
        middle.pack(fill="both", expand=True, pady=4)
        self.cards_frame = tk.Frame(middle, bg=BG)
        self.cards_frame.pack(side="left", fill="y")
        self.no_servo = tk.Label(self.cards_frame, bg=BG, fg="#777", font=FONT,
                                 text="Not streaming yet. Connect first.")
        self.no_servo.pack(padx=10, pady=20)

        # live sensor readings next to the servos
        side = tk.LabelFrame(middle, text="Sensors for aiming", bg=BG, font=FONT_BOLD, padx=10, pady=6)
        side.pack(side="left", fill="both", expand=True, padx=(8, 0))
        tk.Label(side, text="grab distance (%s)" % config.GRAB_TOF, bg=BG, font=FONT).pack(anchor="w")
        self.grab_label = tk.Label(side, text="-", bg=BG, font=("Segoe UI", 28, "bold"))
        self.grab_label.pack(anchor="w")
        self.grab_state = tk.Label(side, text="", bg=BG, font=FONT_BOLD)
        self.grab_state.pack(anchor="w")
        tk.Label(side, text="config: box seen < %d mm, in reach <= %d mm" % (
            config.GRAB_SEE_MM, config.GRAB_REACH_MM), bg=BG, fg="#666", font=("Segoe UI", 8)).pack(anchor="w")
        tk.Label(side, text="front distance (%s)" % config.FRONT_TOF, bg=BG, font=FONT).pack(anchor="w", pady=(10, 0))
        self.front_label = tk.Label(side, text="-", bg=BG, font=FONT_BIG)
        self.front_label.pack(anchor="w")
        tk.Label(side, text="magnet", bg=BG, font=FONT).pack(anchor="w", pady=(10, 0))
        self.mag_label = tk.Label(side, text="-", bg=BG, font=FONT_BIG)
        self.mag_label.pack(anchor="w")

        # saved positions
        saved = tk.LabelFrame(self, text="Saved positions (double-click = move there)", bg=BG,
                              font=FONT_BOLD, padx=8, pady=4)
        saved.pack(fill="x", pady=(4, 0))
        row = tk.Frame(saved, bg=BG)
        row.pack(fill="x")
        tk.Label(row, text="name", bg=BG, font=FONT).pack(side="left")
        self.label_var = tk.StringVar(value="open")
        entry = ttk.Entry(row, textvariable=self.label_var, width=14)
        entry.pack(side="left", padx=4)
        entry.bind("<Return>", lambda e: (self.save_position(), self.focus_set()))
        ttk.Button(row, text="Save position", command=self.save_position).pack(side="left", padx=4)
        ttk.Button(row, text="Move there", command=self.go_saved).pack(side="left", padx=4)
        ttk.Button(row, text="Delete", command=self.delete_saved).pack(side="left", padx=4)
        ttk.Button(row, text="Copy as config", command=self.copy_config).pack(side="left", padx=4)
        self.listbox = tk.Listbox(saved, height=5, font=MONO, activestyle="none")
        self.listbox.pack(fill="x", pady=4)
        self.listbox.bind("<Double-Button-1>", lambda e: self.go_saved())

    def _card(self, name):
        """One servo: a dial (black needle = angle on the Pico, amber = target,
        blue arc = allowed range), the angle, limits and buttons."""
        frame = tk.LabelFrame(self.cards_frame, text="%s  (GP%s)" % (name, self.pins.get(name, "?")),
                              bg=BG, font=FONT_BOLD, padx=6, pady=4, highlightthickness=3,
                              highlightbackground=BG, highlightcolor=BG)
        canvas = tk.Canvas(frame, width=230, height=150, bg=BG, highlightthickness=0)
        canvas.grid(row=0, column=0, rowspan=4)
        big = tk.Label(frame, text="limp", bg=BG, font=("Segoe UI", 22, "bold"), width=7)
        big.grid(row=0, column=1, columnspan=3)
        info = tk.Label(frame, text="", bg=BG, fg="#555", font=("Segoe UI", 8))
        info.grid(row=1, column=1, columnspan=3)
        safe_lo, safe_hi = self.safe.get(name, (0, 270))
        lo, hi = tk.StringVar(value=str(safe_lo)), tk.StringVar(value=str(safe_hi))
        tk.Label(frame, text="limits", bg=BG, font=FONT).grid(row=2, column=1, sticky="e")
        for column, var in ((2, lo), (3, hi)):
            limit_entry = ttk.Entry(frame, textvariable=var, width=5)
            limit_entry.grid(row=2, column=column)
            # Enter: back to the arrow keys (the dial redraws as you type)
            limit_entry.bind("<Return>", lambda e: self.focus_set())
            var.trace_add("write", lambda *args: self.draw(name))
        goto = tk.StringVar(value=str(self.middle(name)))
        self.goto_vars[name] = goto      # ADDED (2026-10-26): calibration mode starts from it
        go_entry = ttk.Entry(frame, textvariable=goto, width=5)
        go_entry.grid(row=3, column=2)
        go_entry.bind("<Return>", lambda e: (self.go_to(name, goto.get()), self.focus_set()))
        ttk.Button(frame, text="go to", width=6, takefocus=0,
                   command=lambda: self.go_to(name, goto.get())).grid(row=3, column=1)
        ttk.Button(frame, text="limp", width=6, takefocus=0,
                   command=lambda: self.limp(name)).grid(row=3, column=3)
        # ADDED (2026-10-23): the safe reset (see reset())
        ttk.Button(frame, text="reset", width=6, takefocus=0,
                   command=lambda: self.key("r", False) if self.free else (self.busy(name) or self.reset(name))
                   ).grid(row=4, column=1)
        for w in (frame, canvas, big, info):
            w.bind("<Button-1>", lambda e: self.select(name))
        frame.pack(fill="x", pady=4)
        self.limits[name] = (lo, hi)
        self.cards[name] = (frame, canvas, big, info)

    # ------------------------------------------------------------ actions
    def select(self, name):
        if name not in self.cards:
            return
        self.selected = name
        for n, (frame, _, _, _) in self.cards.items():
            colour = "#1f6fb2" if n == name else BG
            frame.config(highlightbackground=colour, highlightcolor=colour)
        self.focus_set()          # so the arrow keys do not go to a text field

    def middle(self, name):
        """ADDED (2026-10-23): middle of the servo's safe range; the first key
        press moves from here."""
        lo, hi = self.safe.get(name, (0, 270))
        return (lo + hi) // 2

    def limit_range(self, name):
        """(min, max) typed in the card, kept inside the servo's safe range
        (CHANGED 2026-10-23: was 0 .. 270)."""
        safe_lo, safe_hi = self.safe.get(name, (0, 270))
        lo_var, hi_var = self.limits[name]
        try:
            lo = max(safe_lo, min(safe_hi, int(float(lo_var.get()))))
            hi = max(safe_lo, min(safe_hi, int(float(hi_var.get()))))
        except ValueError:
            return safe_lo, safe_hi
        return (lo, hi) if lo <= hi else (hi, lo)

    def is_locked(self, name):
        """True (and a message) if `name` is locked, so it must not move."""
        if name in self.locked:
            self.log("%s is LOCKED at %d deg - press Enter to unlock it first" % (name, self.locked[name]), "err")
            return True
        return False

    def toggle_lock(self, name):
        """ADDED (2026-10-23): Enter key. Lock the servo at its present angle
        once the grab is right (it keeps the angle; nothing here can move it),
        or unlock it again."""
        if name in self.locked:
            del self.locked[name]
            self.log("%s unlocked" % name)
        else:
            if self.reported.get(name) is None:
                self.log("%s is limp: move it to the grab angle first, then press Enter" % name, "err")
                return
            # the last angle sent (the Pico's report can be one sample behind)
            angle = self.target.get(name, self.reported[name])
            self.locked[name] = angle
            self.log("%s LOCKED at %d deg (grab angle, saved as 'grab'). Enter again unlocks." % (name, angle), "ev")
            self.save_position(label="grab")
        self.draw(name)

    def reset(self, name):
        """ADDED (2026-10-23): ask the Pico for the servo's safe reset
        (config.SERVO_RESET, or with none set: straight to the middle of the
        safe range). A limp servo can only be started this way.
        Keys for this servo wait until the Pico reports its angle again."""
        if self.is_locked(name):
            return
        if not self.send("%s reset" % name):
            self.log("not connected: servo command not sent", "err")
            return
        path = self.reset_paths.get(name)
        if path is None:
            self.target[name] = self.middle(name)
            self.log("%s reset: straight to %d deg (middle of its safe range)" % (name, self.target[name]), "ev")
        else:
            self.target[name] = path[1]
            self.log("%s reset: %d -> %d deg, slowly (about 1.6 s)" % (name, path[0], path[1]), "ev")
        self.resetting[name] = time.time()
        self.draw(name)

    def busy(self, name):
        """ADDED (2026-10-23): True while a reset sent for `name` is still
        running on the Pico (no angle reported yet, at most 4 s)."""
        sent = self.resetting.get(name)
        if sent is None:
            return False
        if self.reported.get(name) is not None or time.time() - sent > 4.0:
            del self.resetting[name]
            return False
        return True

    def move(self, name, angle):
        if self.is_locked(name) or self.busy(name):
            return
        # ADDED (2026-10-23): a limp servo is started with its safe reset,
        # never with a jump to an angle (its real position is unknown).
        # CHANGED (2026-10-26): in calibration mode there is no known safe
        # reset: it starts at the angle typed in its "go to" field instead.
        if self.reported.get(name) is None:
            if self.free:
                self.start_free(name)
            else:
                self.reset(name)
            return
        lo, hi = self.limit_range(name)
        angle = max(lo, min(hi, int(round(angle))))
        self.target[name] = angle
        if not self.send("%s %d" % (name, angle)):
            self.log("not connected: servo command not sent", "err")
        self.draw(name)

    def start_free(self, name):
        """ADDED (2026-10-26): calibration mode, limp servo: send the angle in
        its "go to" field (inside the limits typed in the card); the Pico
        starts the servo straight there."""
        try:
            angle = float(self.goto_vars[name].get())
        except ValueError:
            self.log("type a start angle in the 'go to' field of %s first" % name, "err")
            return
        lo, hi = self.limit_range(name)
        angle = max(lo, min(hi, int(round(angle))))
        if not self.send("%s %d" % (name, angle)):
            self.log("not connected: servo command not sent", "err")
            return
        self.target[name] = angle
        self.resetting[name] = time.time()     # keys wait until the Pico reports the angle
        self.log("%s starts at %d deg (calibration mode)" % (name, angle), "ev")
        self.draw(name)

    def go_to(self, name, text):
        try:
            angle = float(text)
        except ValueError:
            self.log("'go to' needs a number of degrees", "err")
            return
        self.select(name)
        if self.free and self.reported.get(name) is None:
            self.goto_vars[name].set(str(int(round(angle))))   # ADDED (2026-10-26)
        self.move(name, angle)

    def limp(self, name):
        if self.is_locked(name):      # limp would drop the box
            return
        self.send("%s off" % name)

    def key(self, keysym, shift):
        """A key pressed while this tab is shown (see Viewer.on_servo_key)."""
        if keysym in ("1", "2"):
            index = int(keysym) - 1
            if index < len(self.names):
                self.select(self.names[index])
            return
        if self.selected is None:
            return
        if keysym in ("Return", "KP_Enter"):
            self.toggle_lock(self.selected)
            return
        if keysym in ("x", "X"):
            self.limp(self.selected)
            return
        if keysym in ("r", "R"):                 # ADDED (2026-10-23)
            if self.free:
                self.log("calibration mode: no safe reset is known - use 'go to'", "err")
            elif not self.busy(self.selected):
                self.reset(self.selected)
            return
        sign = -1 if keysym == "Left" else 1
        step = self.step_var.get() * (5 if shift else 1)
        start = self.target.get(self.selected, self.middle(self.selected))
        self.move(self.selected, start + sign * step)

    def save_position(self, label=None):
        if not self.names:
            return
        if label is None:
            label = self.label_var.get().strip() or "position %d" % (len(self.saved) + 1)
        angles = dict((n, self.reported.get(n)) for n in self.names)
        angles.update(self.locked)     # a locked servo: the angle it is locked at
        self.saved.append((label, angles, self.grab_mm))
        text = "%-12s " % label + "  ".join(
            "%s %s" % (n, "limp" if a is None else "%d deg" % a) for n, a in sorted(angles.items()))
        text += "   grab %s" % ("-" if self.grab_mm is None else "%d mm" % self.grab_mm)
        self.listbox.insert("end", text)

    def _picked(self):
        picked = self.listbox.curselection()
        return picked[0] if picked else None

    def go_saved(self):
        i = self._picked()
        if i is None:
            return
        for name, angle in self.saved[i][1].items():
            if angle is not None and name in self.cards:
                self.move(name, angle)

    def delete_saved(self):
        i = self._picked()
        if i is not None:
            del self.saved[i]
            self.listbox.delete(i)

    def copy_config(self):
        if not self.saved:
            self.log("nothing saved yet", "err")
            return
        lines = ["# Gripper servo angles (degrees), measured with tools/sensor_viewer.py (Grab / Servo tab)",
                 "GRIPPER_ANGLES = {"]
        for label, angles, grab in self.saved:
            body = ", ".join('"%s": %s' % (n, a) for n, a in sorted(angles.items()))
            note = "" if grab is None else "    # grab distance %d mm" % grab
            lines.append('    "%s": {%s},%s' % (label, body, note))
        lines.append("}")
        text = "\n".join(lines)
        self.clipboard_clear()
        self.clipboard_append(text)
        self.log("copied to the clipboard:\n" + text)

    # ------------------------------------------------------------ data in
    def on_hello(self, h):
        servos = h.get("servo")
        if servos is None:
            servos = []
            self.log("The Pico did not report any servos: upload the new tests/stream_sensors.py", "err")
        names = [row[0] for row in servos]
        self.pins = dict((row[0], row[1]) for row in servos)
        # CHANGED (2026-10-23): each row also carries the servo's safe range
        # (config.SERVO_SAFE_RANGE); the limits in the card can only narrow it.
        self.safe = dict((row[0], (row[2], row[3]) if len(row) >= 4 else (0, 270)) for row in servos)
        # ADDED (2026-10-23): reset path (config.SERVO_RESET) or None per servo
        self.reset_paths = dict((row[0], row[4] if len(row) >= 5 else None) for row in servos)
        self.resetting = {}
        self.reported = dict((n, None) for n in names)   # a new stream starts with limp servos
        # ADDED (2026-10-26): calibration mode (stream started with servo_free=True)
        self.free = bool(h.get("servo_free"))
        if self.free:
            self.banner.pack(fill="x", pady=(0, 4), before=self.help)
        else:
            self.banner.pack_forget()
        if self.locked:
            # the restarted stream has let every servo go limp: a lock means nothing now
            self.log("stream restarted: the servos are limp, locks cleared", "err")
            self.locked = {}
        if names != self.names:
            for frame, _, _, _ in self.cards.values():
                frame.destroy()
            self.cards = {}
            self.limits = {}
            self.names = names
            for n in names:
                self._card(n)
            if names:
                self.no_servo.pack_forget()
                self.select(names[0])
            else:
                self.no_servo.config(text="No servos on the Pico.")
                self.no_servo.pack(padx=10, pady=20)
        for n in names:
            self.draw(n)

    def on_sample(self, m):
        for name, angle in m.get("servo", []):
            if self.reported.get(name, "?") != angle:
                self.reported[name] = angle
                self.draw(name)
        for row in m.get("tof", []):
            name, available, mm = row[:3]
            text = "n/a" if not available else ("no target" if mm is None else "%d mm" % mm)
            if name == config.GRAB_TOF:
                self.grab_mm = mm if available else None
                self.grab_label.config(text=text)
                if self.grab_mm is None:
                    self.grab_state.config(text="", fg="black")
                elif self.grab_mm <= config.GRAB_REACH_MM:
                    self.grab_state.config(text="IN REACH", fg="#1e8449")
                elif self.grab_mm < config.GRAB_SEE_MM:
                    self.grab_state.config(text="box seen", fg=AMBER)
                else:
                    self.grab_state.config(text="nothing close", fg="#777")
            elif name == config.FRONT_TOF:
                self.front_label.config(text=text)
        mag = m.get("mag")
        if mag:
            field, polarity = mag[2], mag[3]
            self.mag_label.config(text="%s mV  %s" % (field, polarity or ""))

    def draw(self, name):
        if name not in self.cards:
            return
        _, canvas, big, info = self.cards[name]
        lo, hi = self.limit_range(name)
        angle = self.reported.get(name)
        target = self.target.get(name)
        big.config(text="limp" if angle is None else "%d deg" % angle, fg="#777" if angle is None else "black")
        info.config(text="target %s   limits %d-%d" % ("-" if target is None else "%d" % target, lo, hi))
        if name in self.locked:
            big.config(text="%d LOCKED" % self.locked[name], fg=RED)
            info.config(text="grab angle locked - Enter unlocks")
        # The dial: 0 deg at the bottom left, 135 at the top, 270 at the bottom
        # right. Screen angle (counter-clockwise from 3 o'clock) = 225 - servo angle.
        canvas.delete("all")
        cx, cy, r = 115, 85, 62
        box = (cx - r, cy - r, cx + r, cy + r)
        canvas.create_arc(box, start=-45, extent=270, style="arc", width=10, outline="#dddddd")
        canvas.create_arc(box, start=225 - hi, extent=hi - lo, style="arc", width=10, outline="#9ec9ec")
        for a in (0, 90, 180, 270):
            t = math.radians(225 - a)
            canvas.create_text(cx + (r + 16) * math.cos(t), cy - (r + 16) * math.sin(t), text=str(a),
                               font=("Segoe UI", 8), fill="#666")
        needle = RED if name in self.locked else "black"
        for value, colour, width in ((target, AMBER, 2), (angle, needle, 4)):
            if value is None:
                continue
            t = math.radians(225 - value)
            canvas.create_line(cx, cy, cx + (r - 6) * math.cos(t), cy - (r - 6) * math.sin(t),
                               fill=colour, width=width, capstyle="round")
        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="black", outline="")


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
            elif options.tab == "servo":                 # ADDED (2026-10-23)
                self.notebook.select(self.servo_page)
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
        # ADDED (2026-10-23): gripper calibration tab (servos on the arrow keys).
        self.servo_page = ServoPage(self.notebook, self.send_command, self.write_log)
        self.notebook.add(self.servo_page, text="  Grab / Servo  ")
        # The keys are bound on the window AND on the notebook: when the notebook
        # (the row of tabs) has the keyboard focus, its own Left / Right binding
        # would switch tabs; the notebook binding runs first and stops that while
        # the servo tab is shown.
        for key in ("<Left>", "<Right>", "<Shift-Left>", "<Shift-Right>", "<Key-1>", "<Key-2>", "<Key-x>", "<Key-r>",
                    "<Return>", "<KP_Enter>"):
            self.bind(key, self.on_servo_key)
            self.notebook.bind(key, self.on_servo_key)
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)
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

        crash = self._box(lower, "Crash (SEN0138) + push button")
        crash.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=4)
        self.crash_lamps = [Lamp(crash, "left", 36), Lamp(crash, "right", 36)]
        for lamp in self.crash_lamps:
            lamp.frame.pack(side="left", padx=10)
        # ADDED (2026-10-23): the push button (config.PUSH_BUTTON_PIN). It is read
        # by the pin monitor (internal pull-up, pressed = LOW).
        self.push_lamp = Lamp(crash, "push", 36)
        self.push_lamp.frame.pack(side="left", padx=10)

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
        args = "line_pins=%r, crash_pins=%r, ir_pin=%d, use_tof=%r, mag_pin=%d" % (
            line, crash, ir[0] if ir else -1, bool(self.tof_var.get()), mag[0] if mag else -1)
        # ADDED (2026-10-26): --servo-free = servo calibration mode on the Pico
        if getattr(self.options, "servo_free", False):
            args += ", servo_free=True"
        return args

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

    def send_command(self, text):
        """ADDED (2026-10-23): send a command line to the Pico's stream (servo
        commands). Returns False when not connected."""
        if self.link is None:
            return False
        self.link.send(text)
        return True

    def on_servo_key(self, event):
        """ADDED (2026-10-23): arrow / 1 / 2 / x keys for the Grab / Servo tab.
        Ignored on the other tabs and while a text field has the cursor."""
        if self.notebook.select() != str(self.servo_page):
            return None
        focus = self.focus_get()
        if focus is not None and focus.winfo_class() in ("Entry", "TEntry", "TCombobox", "Spinbox"):
            return None
        self.servo_page.key(event.keysym, bool(event.state & 0x1))   # 0x1 = Shift held
        return "break"

    def on_tab_changed(self, event):
        # ADDED (2026-10-23): give the servo tab the keyboard focus when it is
        # opened, so the arrow keys go to the servos and not to a text field.
        if self.notebook.select() == str(self.servo_page):
            self.after(20, self.servo_page.focus_set)

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
        self.servo_page.on_hello(h)     # ADDED (2026-10-23)
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
        roles.setdefault(config.PUSH_BUTTON_PIN, "push btn")   # ADDED (2026-10-23)
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
        self.servo_page.on_sample(m)     # ADDED (2026-10-23)
        # ADDED (2026-10-23): push button lamp, from the pin monitor's level of
        # PUSH_BUTTON_PIN (pull-up: released = HIGH, pressed = LOW).
        levels = m.get("p", [])
        if config.PUSH_BUTTON_PIN in self.monitor and self.monitor.index(config.PUSH_BUTTON_PIN) < len(levels):
            pushed = levels[self.monitor.index(config.PUSH_BUTTON_PIN)] == 0
            self.push_lamp.set(RED if pushed else ON, "push GP%d\n%s" % (
                config.PUSH_BUTTON_PIN, "PRESSED" if pushed else "released"))
        else:
            self.push_lamp.set(GREY, "push\nnot monitored")
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
    parser.add_argument("--tab", choices=("overview", "tof", "mag", "servo"), default="overview",
                        help="tab shown first (servo = Grab / Servo calibration, added 2026-10-23)")
    parser.add_argument("--record", help="append every line received from the Pico to this file")
    # ADDED (2026-10-26)
    parser.add_argument("--servo-free", action="store_true",
                        help="servo CALIBRATION mode: ignore the safe ranges / resets in config.py "
                             "(after a hardware change); only the 270 deg limit stays")
    Viewer(parser.parse_args()).mainloop()


if __name__ == "__main__":
    main()
