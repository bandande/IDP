"""
The competition table for the simulator.

The white lines come from sim/track_map.py: the table as named nodes with
positions (metres) and the straight lines between them. (The robot itself
uses MAP.py, which only knows which node is next to which; the simulator
needs real positions.) This file adds the line width, the T-bars, the
coloured pads, the racks, the ramp and the traffic light, for the sensors
and the pictures.

Coordinates: metres, origin = south-west corner of the table, x = east,
y = north (see sim/track_map.py).
"""

import math

import numpy as np

from sim import track_map

LINE_W = 0.020            # white tape width
TBAR = 0.12               # T-bar length at the north ends
PAD_SIZE = 0.10           # coloured pad square
RACK_DEPTH = 0.12         # rack, east-west
RACK_TOUCH = 0.03         # the robot may touch a rack (to put a box in): only pushing
                          # more than this into it counts as a collision. (The gap between
                          # a rung end and the rack is a guess, so this is generous.)
PAD_COLOURS = {"PAD_GREEN": "#1fa36a", "PAD_RED": "#d62f4b",
               "PAD_YELLOW": "#f2d21b", "PAD_BLUE": "#2b7bd6"}

# Other names accepted by the simulator options (--boxes, --poles): the
# teammate's old MAP.py names and short pad colours.
TEAM_NAMES = {"GREEN_1": "PAD_GREEN", "GREEN_2": "PAD_RED", "RED_1": "PAD_YELLOW", "RED_2": "PAD_BLUE",
              "GREEN": "PAD_GREEN", "RED": "PAD_RED", "YELLOW": "PAD_YELLOW", "BLUE": "PAD_BLUE"}
for _k in range(1, 7):
    TEAM_NAMES["LEFT_BOTTOM_DROP_%d" % _k] = "ORANGE_W%d" % _k
    TEAM_NAMES["LEFT_TOP_DROP_%d" % _k] = "ORANGE_E%d" % _k
    TEAM_NAMES["RIGHT_TOP_DROP_%d" % _k] = "PURPLE_W%d" % _k
    TEAM_NAMES["RIGHT_BOTTOM_DROP_%d" % _k] = "PURPLE_E%d" % _k


def sim_name(name):
    """sim/track_map.py node name for a simulator name, a MAP.py name or a pad colour."""
    return TEAM_NAMES.get(name, name)


class Track:
    """White lines plus the objects on the table. white_at(x, y) is what a
    line sensor sees there."""

    def __init__(self, width, length, line_w):
        self.width = width
        self.length = length
        self.line_w = line_w
        self.nodes = {}        # name -> (x, y)
        self.kinds = {}        # name -> "start", "box", "corner", "junction", "pad", "slot", "end"
        self.edges = []        # (a, b, p, q): line from point p to point q, joining nodes a and b
        self.extra = []        # (p, q): white lines that are not map lines (T-bars, pad outlines)
        self.pads = []         # {"name", "colour", "centre", "size"}
        self.racks = []        # {"name", "colour", "rect": (x0, y0, x1, y1), "slots": [y, ...]}
        self.ramp = None       # (x0, y0, x1, y1)
        self.gate = None       # (x, y): the traffic light at the ramp entrance
        self.pad_fill_white = True
        self._mask = None
        self._res = None

    def segments(self):
        """All white straight lines as (p, q)."""
        for a, b, p, q in self.edges:
            yield p, q
        for p, q in self.extra:
            yield p, q

    def neighbours(self):
        return track_map.NEIGHBOURS

    def rasterize(self, res=0.001):
        """Precompute a white/black bitmap (res metres per pixel) for fast sensing."""
        nx, ny = int(math.ceil(self.width / res)), int(math.ceil(self.length / res))
        mask = np.zeros((ny, nx), dtype=bool)
        half = self.line_w / 2
        for p, q in self.segments():
            _paint_segment(mask, res, p, q, half)
        if self.pad_fill_white:
            for pad in self.pads:
                cx, cy = pad["centre"]
                s = pad["size"] / 2
                mask[max(0, int((cy - s) / res)):int((cy + s) / res),
                     max(0, int((cx - s) / res)):int((cx + s) / res)] = True
        self._mask, self._res = mask, res
        return mask

    def white_at(self, x, y):
        if self._mask is None:
            self.rasterize()
        ix, iy = int(x / self._res), int(y / self._res)
        if 0 <= iy < self._mask.shape[0] and 0 <= ix < self._mask.shape[1]:
            return bool(self._mask[iy, ix])
        return False

    def obstacle_at(self, x, y):
        """Name of what is at (x, y) that the robot must not drive into, or None."""
        if not (0 <= x <= self.width and 0 <= y <= self.length):
            return "table edge"
        for rack in self.racks:
            x0, y0, x1, y1 = rack["rect"]
            m = RACK_TOUCH
            if x0 + m <= x <= x1 - m and y0 + m <= y <= y1 - m:
                return rack["name"]
        return None


def _paint_segment(mask, res, p, q, half):
    (px, py), (qx, qy) = p, q
    x0, x1 = min(px, qx) - half, max(px, qx) + half
    y0, y1 = min(py, qy) - half, max(py, qy) + half
    i0, i1 = max(0, int(x0 / res)), min(mask.shape[1], int(math.ceil(x1 / res)) + 1)
    j0, j1 = max(0, int(y0 / res)), min(mask.shape[0], int(math.ceil(y1 / res)) + 1)
    if i0 >= i1 or j0 >= j1:
        return
    xs = (np.arange(i0, i1) + 0.5) * res
    ys = (np.arange(j0, j1) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    dx, dy = qx - px, qy - py
    ll = dx * dx + dy * dy
    t = np.clip(((gx - px) * dx + (gy - py) * dy) / ll, 0.0, 1.0) if ll > 0 else 0.0
    d2 = (gx - px - t * dx) ** 2 + (gy - py - t * dy) ** 2
    mask[j0:j1, i0:i1] |= d2 <= half * half


def _kind(name):
    if name == "START":
        return "start"
    if name.startswith("BOX_"):
        return "box"
    if name.startswith("CORNER_"):
        return "corner"
    if name in track_map.PADS:
        return "pad"
    if name in track_map.SLOTS:
        return "slot"
    if name.endswith("_END"):
        return "end"
    return "junction"


def build_arena(pad_fill_white=True):
    """The table: lines from track_map.py plus pads, racks, ramp and T-bars."""
    t = Track(track_map.TABLE_WIDTH, track_map.TABLE_LENGTH, LINE_W)
    t.pad_fill_white = pad_fill_white
    for name, (x, y) in track_map.NODES.items():
        t.nodes[name] = (x, y)
        t.kinds[name] = _kind(name)
    for a, b in track_map.LINES:
        t.edges.append((a, b, track_map.NODES[a], track_map.NODES[b]))

    # T-bars across the three north ends
    for end in ("W_END", "S_END", "E_END"):
        x, y = track_map.NODES[end]
        t.extra.append(((x - TBAR / 2, y), (x + TBAR / 2, y)))

    # coloured pads (white outline + coloured square) centred on the pad nodes
    s = PAD_SIZE / 2
    for pad in track_map.PADS:
        cx, cy = track_map.NODES[pad]
        t.pads.append({"name": pad, "colour": PAD_COLOURS[pad], "centre": (cx, cy), "size": PAD_SIZE})
        t.extra += [((cx - s, cy - s), (cx + s, cy - s)), ((cx + s, cy - s), (cx + s, cy + s)),
                    ((cx + s, cy + s), (cx - s, cy + s)), ((cx - s, cy + s), (cx - s, cy - s))]

    # racks: one slot per rung, centred between the two rung ends that face it
    rung_y = track_map.RUNG_Y
    pitch = rung_y[1] - rung_y[0]
    y0, y1 = rung_y[0] - pitch / 2, rung_y[-1] + pitch / 2
    for name, colour, west_end, east_end in (("orange rack", "#f07a1a", "ORANGE_W1", "ORANGE_E1"),
                                             ("purple rack", "#7a3fc0", "PURPLE_W1", "PURPLE_E1")):
        xc = (track_map.NODES[west_end][0] + track_map.NODES[east_end][0]) / 2
        t.racks.append({"name": name, "colour": colour,
                        "rect": (xc - RACK_DEPTH / 2, y0, xc + RACK_DEPTH / 2, y1), "slots": list(rung_y)})

    # ramp on the spine between the start box and the first crossing; the
    # traffic light stands at its entrance
    xs = track_map.X_SPINE
    ramp_y0 = track_map.NODES["BOX_N"][1] + 0.07
    ramp_y1 = track_map.NODES["S1"][1] - 0.14
    t.ramp = (xs - 0.08, ramp_y0, xs + 0.08, ramp_y1)
    t.gate = (xs, ramp_y0)
    t.rasterize()
    return t


def describe(track):
    """Text listing of the topology (for checking against the real table)."""
    lines = []
    for name in track.nodes:
        x, y = track.nodes[name]
        nb = track_map.NEIGHBOURS[name]
        links = ", ".join("%s: %s" % (k, nb[k]) for k in ("north", "east", "south", "west") if k in nb)
        lines.append("%-10s %-8s (%.2f, %.2f)  %s" % (name, track.kinds[name], x, y, links))
    return "\n".join(lines)


if __name__ == "__main__":
    import argparse
    import os
    parser = argparse.ArgumentParser(description="Draw the simulated table and list its topology.")
    parser.add_argument("--png", default="sim_arena.png", help="output image")
    parser.add_argument("--no-labels", action="store_true")
    args = parser.parse_args()
    from sim.render import draw_arena
    track = build_arena()
    print(describe(track))
    draw_arena(track, args.png, labels=not args.no_labels)
    print("saved", os.path.abspath(args.png))
