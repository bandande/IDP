"""
Pictures of the simulated table and of a run (matplotlib, saved as PNG).

draw_arena(track, png)                  the table, with node names
draw_run(track, world, notes, png)      the table + the robot's path coloured
                                        by time, robot outline at start / end,
                                        numbered markers where the robot code
                                        printed something
"""

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
from matplotlib.collections import LineCollection   # noqa: E402
from matplotlib.patches import Polygon, Rectangle   # noqa: E402

FLOOR = "#1d1f22"
LINE = "#f4f4f0"


def _segment_poly(p, q, w):
    (px, py), (qx, qy) = p, q
    dx, dy = qx - px, qy - py
    n = math.hypot(dx, dy) or 1.0
    ox, oy = -dy / n * w / 2, dx / n * w / 2
    ex, ey = dx / n * w / 2, dy / n * w / 2      # square line caps
    return [(px - ex + ox, py - ey + oy), (qx + ex + ox, qy + ey + oy),
            (qx + ex - ox, qy + ey - oy), (px - ex - ox, py - ey - oy)]


def _draw_table(ax, track, labels):
    ax.set_facecolor("#9aa0a6")
    ax.add_patch(Rectangle((0, 0), track.width, track.length, color=FLOOR, zorder=0))
    if track.ramp:
        x0, y0, x1, y1 = track.ramp
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color="#55595e", zorder=1))
        ax.plot([x0, x0], [y0, y1], color="#f2c80f", lw=2, zorder=2)
        ax.plot([x1, x1], [y0, y1], color="#f2c80f", lw=2, zorder=2)
        ax.text(x1 + 0.01, (y0 + y1) / 2, "ramp", color="#f2c80f", fontsize=7, rotation=90, va="center", zorder=6)
    for pad in track.pads:
        cx, cy = pad["centre"]
        s = pad["size"]
        ax.add_patch(Rectangle((cx - s / 2, cy - s / 2), s, s, color=pad["colour"], zorder=2))
    for p, q in track.segments():
        ax.add_patch(Polygon(_segment_poly(p, q, track.line_w), closed=True, color=LINE, zorder=3, lw=0))
    for rack in track.racks:
        x0, y0, x1, y1 = rack["rect"]
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=rack["colour"], alpha=0.9, zorder=4))
        for i in range(1, len(rack["slots"])):
            yy = (rack["slots"][i - 1] + rack["slots"][i]) / 2
            ax.plot([x0, x1], [yy, yy], color="black", lw=0.6, zorder=5)
    if track.gate:
        gx, gy = track.gate
        for dx in (-0.09, 0.09):
            ax.add_patch(Rectangle((gx + dx - 0.012, gy - 0.012), 0.024, 0.024, color="#f2c80f", ec="black", zorder=5))
        ax.text(gx + 0.11, gy, "gate", color="#f2c80f", fontsize=7, va="center", zorder=6)
    if labels:
        for name, (x, y) in track.nodes.items():
            kind = track.kinds[name]
            if kind == "slot" or name in ("BOX_NW", "BOX_NE", "BOX_SW", "BOX_SE"):
                continue     # too crowded: the rungs and the box crossings are labelled
            ax.text(x + 0.012, y + 0.012, name, color="#ff9f43", fontsize=6.5, zorder=7,
                    bbox=dict(facecolor="black", alpha=0.45, lw=0, pad=0.6))
    ax.set_xlim(-0.02, track.width + 0.02)
    ax.set_ylim(-0.02, track.length + 0.02)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m, east)")
    ax.set_ylabel("y (m, north)")


def draw_arena(track, png, labels=True):
    fig, ax = plt.subplots(figsize=(1.4 + 9.0 * track.width / track.length, 10))
    _draw_table(ax, track, labels)
    ax.set_title("Simulated table (topology from the video; lengths approximate)", fontsize=9)
    fig.tight_layout()
    fig.savefig(png, dpi=130)
    plt.close(fig)


def _robot_outline(ax, world, x, y, th, colour, alpha=1.0):
    s = world.spec
    c, s_ = math.cos(th), math.sin(th)
    for front, back, half in ((s.body_front, s.body_back, s.body_half_width),
                              (s.wheel_half_length, s.wheel_half_length, s.wheel_half_width)):
        pts = []
        for f, l in ((front, half), (front, -half), (-back, -half), (-back, half)):
            pts.append((x + f * c - l * s_, y + f * s_ + l * c))
        ax.add_patch(Polygon(pts, closed=True, fill=False, ec=colour, lw=1.4, alpha=alpha, zorder=9))
    sx = [x + s.sensor_ahead * c - lat * s_ for lat in s.sensor_lat]
    sy = [y + s.sensor_ahead * s_ + lat * c for lat in s.sensor_lat]
    ax.plot(sx, sy, "o", color=colour, ms=2.5, alpha=alpha, zorder=9)


def draw_run(track, world, notes, png, title=""):
    """notes: [(t, x, y, text)] printed by the robot code (numbered on the map)."""
    fig, (ax, axt) = plt.subplots(1, 2, figsize=(14, 10), gridspec_kw={"width_ratios": [1.0, 0.9]})
    _draw_table(ax, track, labels=True)
    tr = world.trace
    if len(tr) > 1:
        pts = [(r[1], r[2]) for r in tr]
        segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
        lc = LineCollection(segs, cmap="viridis", lw=2.2, zorder=8)
        lc.set_array([r[0] for r in tr[:-1]])
        ax.add_collection(lc)
        cb = fig.colorbar(lc, ax=ax, fraction=0.035, pad=0.01)
        cb.set_label("time (s)")
        # path of the sensor-row centre too (thin), it is what follows the line
        a = world.spec.sensor_ahead
        ax.plot([r[1] + a * math.cos(r[3]) for r in tr], [r[2] + a * math.sin(r[3]) for r in tr],
                color="#ff6b6b", lw=0.7, alpha=0.8, zorder=8)
        t0, x0, y0, th0 = tr[0][:4]
        _robot_outline(ax, world, x0, y0, th0, "#7bed9f")
    _robot_outline(ax, world, world.x, world.y, world.th, "#ff4757")
    for name, x0, y0, x1, y1 in getattr(world, "_box_rects", []):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color="#c8a165", ec="black", zorder=6))
    for pad in track.pads:
        pole = getattr(world, "pad_poles", {}).get(pad["name"])
        if pole:
            cx, cy = pad["centre"]
            ax.text(cx, cy, pole[0].upper(), color="black", fontsize=9, ha="center", va="center",
                    weight="bold", zorder=7)
    for i, (t, x, y, text) in enumerate(notes, 1):
        ax.plot(x, y, "o", color="#ffa502", ms=4, zorder=10)
        ax.text(x + 0.01, y - 0.03, str(i), color="#ffa502", fontsize=7, zorder=10)
    extra = ""
    if getattr(world, "light", None):
        extra = "   [traffic light %gs red / %gs green]" % world.light[:2]
    ax.set_title((title or "simulated run") + extra, fontsize=9)
    axt.axis("off")
    lines = ["green outline = start, red outline = end; thick = rear axle path,",
             "thin red = sensor-row path; orange numbers = printed lines:", ""]
    for i, (t, x, y, text) in enumerate(notes, 1):
        lines.append("%3d  %6.2fs  %s" % (i, t, text[:70]))
    if world.stopped:
        lines += ["", "end: %s" % world.stopped]
    axt.text(0, 1, "\n".join(lines[:95]), va="top", family="monospace", fontsize=7.2)
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    plt.close(fig)
