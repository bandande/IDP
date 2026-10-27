"""
The simulated world: the table, the robot's motion and its sensors.

The robot is a differential drive: two driven wheels on the REAR axle, the
row of four line sensors at the front (see RobotSpec). The robot code runs
unchanged on top of the fake machine / utime modules; they call into the
World for time, inputs and outputs:
  * time only moves when the code sleeps or touches the hardware/clock;
    advance() then integrates the motion in small steps,
  * motor DIR pins and PWM duty set the wheel speeds (with a lag),
  * line sensor pins read the table under each sensor,
  * crash sensor pins read the simulated buttons (press() / scheduled).
The world stops the run (raises SimStop) at the time limit, or when the
robot's body hits a rack or leaves the table.
"""

import math


class SimStop(BaseException):
    """Ends a simulation run. BaseException, so `except Exception` in the
    robot code does not swallow it."""

    def __init__(self, reason):
        BaseException.__init__(self, reason)
        self.reason = reason


class RobotSpec:
    """Robot geometry and motor model. Lengths in metres.

    Defaults are ESTIMATES from the photo of 2026-10-19 - measure the real
    robot and override them (sim.run --spec key=value ...).
    """

    def __init__(self, **kw):
        # Distance between the two wheel centres (photo: ~0.27 m). 0.26 also
        # makes a 1700 ms spin at 60 % about a quarter turn, as on the table.
        self.track_width = 0.26
        self.sensor_ahead = 0.20     # line sensor row, distance in front of the rear axle
        # Lateral offset of each line sensor's detection spot, LEFT positive,
        # in the order outer-left, inner-left, inner-right, outer-right.
        # The inner pair must both see a centred line (bench test: 0110).
        self.sensor_lat = (0.055, 0.009, -0.009, -0.055)
        # Wheel speed at 100 % PWM (m/s). ~0.15-0.20 matches the teammate's tuning on
        # the real table (main.py, 2026-10-19): at 60 % the robot needs
        # NODE_FORWARD_MS = 1600 ms to move ~sensor_ahead, and a 1700 ms spin
        # at 60 % is about a quarter turn.
        self.vmax = 0.20
        self.tau = 0.06              # wheel speed lag (first-order time constant, s)
        self.deadband = 0.0          # PWM fraction below which a wheel does not turn
        self.left_gain = 1.0         # >1 = that wheel runs faster (mismatched motors)
        self.right_gain = 1.0
        # Body outline for collisions, relative to the rear axle centre: the
        # base plate (long, narrow) plus the wheel strip (short, wide).
        self.body_back = 0.04             # the base plate ends just behind the wheels (photo)
        self.body_front = 0.22
        self.body_half_width = 0.10
        self.wheel_half_width = 0.16
        self.wheel_half_length = 0.04
        for k, v in kw.items():
            if not hasattr(self, k):
                raise ValueError("unknown RobotSpec field %r" % k)
            setattr(self, k, v)


class World:
    def __init__(self, track, spec, pins, pose, t_end_s, step_us=1000, stop_on_collision=True,
                 trace_every_s=0.01):
        """pins: dict with
             line: [outer-left, inner-left, inner-right, outer-right] GPIO numbers
             line_white_level: pin level when a sensor sees white
             left_dir, left_pwm, right_dir, right_pwm, forward_level,
             left_invert, right_invert
             crash: {gpio: released_level, ...}
           pose: (x, y, heading_rad) of the rear-axle centre; heading 0 = east,
                 pi/2 = north (counter-clockwise)."""
        self.track = track
        self.spec = spec
        self.pins = pins
        self.x, self.y, self.th = pose
        self.vl = self.vr = 0.0
        self.now_us = 0
        self._phys_us = 0
        self.step_us = step_us
        self.t_end_us = int(t_end_s * 1e6)
        self.stop_on_collision = stop_on_collision
        self.out_levels = {}            # output pin -> level
        self.duty = {}                  # pwm pin -> duty_u16
        # ADDED (2026-10-27): pwm pin -> (lowest, highest) NON-ZERO duty sent
        # during the whole run. sim/check.py turns the servo pins' values into
        # angles to check that no servo ever left its safe range.
        self.duty_range = {}
        self.inputs = dict(pins.get("crash", {}))   # input pin -> level (buttons)
        self._line_pin_index = {p: i for i, p in enumerate(pins["line"])}
        self.line_white = [False] * 4
        self._schedule = []             # [(t_us, fn)], sorted
        self.trace = []                 # (t, x, y, th, bits, vl, vr)
        self._trace_every_us = int(trace_every_s * 1e6)
        self._next_trace_us = 0
        self.events = []                # (t, kind, text)
        self.distance = 0.0
        self.stopped = None
        self._busy = False              # advance(): no nested event processing
        # Task-sensor world (set with setup_task_world()).
        self.mag_pin = None
        self.mag_baseline_mv = 2150
        self.mag_north_sign = 1
        self.pad_poles = {}             # pad node -> "north" / "south"
        self.boxes = set()              # dead-end nodes with a box in the rack slot ahead
        self._box_rects = []            # (name, x0, y0, x1, y1)
        # ADDED (2026-10-26): a box to pick up on each pad, seen by the front
        # ToF; it is taken away when the gripper closes in front of it and the
        # robot then carries it (its magnet stays at the magnet sensor).
        self._pickup_rects = []         # (pad name, x0, y0, x1, y1)
        self.carrying = None            # pad name of the box the robot holds
        self.grip_pin = None            # servo2 signal pin
        self.grip_closed_duty = None    # duty at or below which servo2 counts as closed
        self.ir_pin = None
        self.light = None               # (red_s, green_s, phase_s) or None
        self._noise = 12345
        self._update_line_sensors()

    def setup_task_world(self, mag_pin=None, pad_poles=None, mag_north_sign=1, boxes=(),
                         ir_pin=None, light=None, ir_when="green", grip_pin=None, grip_closed_duty=None):
        """Magnets in the pick-up pads, boxes already in rack slots, traffic light.

        pad_poles: {"GREEN": "north", ...}; boxes: dead-end node names whose
        rack slot already holds a box; light: (red_s, green_s, phase_s) cycle
        of the traffic light (None = no light); ir_when: the light sends IR
        bursts while it is "green" (our table) or "red"."""
        self.mag_pin = mag_pin
        self.pad_poles = dict(pad_poles or {})
        self.mag_north_sign = mag_north_sign
        self.boxes = set(boxes)
        self._box_rects = [self._box_rect(n) for n in self.boxes]
        self.ir_pin = ir_pin
        self.light = light
        self.ir_when = ir_when
        # ADDED (2026-10-26): boxes to pick up, one per pad (see __init__)
        self._pickup_rects = [self._pickup_rect(pad["name"]) for pad in self.track.pads]
        self.carrying = None
        self.grip_pin = grip_pin
        self.grip_closed_duty = grip_closed_duty
        if ir_pin is not None:
            self.inputs[ir_pin] = 1     # TSOP4838 output idles HIGH

    def add_box(self, node):
        """A box has been put into the rack slot at the end of rung `node`."""
        if node not in self.boxes:
            self.boxes.add(node)
            self._box_rects.append(self._box_rect(node))

    def _box_rect(self, node):
        """A box in the rack slot just beyond the dead end `node`."""
        nb = self.track.neighbours()[node]
        if len(nb) != 1:
            raise ValueError("%s is not a dead end" % node)
        jx, jy = self.track.nodes[list(nb.values())[0]]
        nx, ny = self.track.nodes[node]
        dx, dy = nx - jx, ny - jy
        n = math.hypot(dx, dy)
        dx, dy = dx / n, dy / n
        near, far, half = 0.035, 0.10, 0.04     # box from 3.5 cm to 10 cm past the rung end
        pts = [(nx + dx * d + dy * l, ny + dy * d - dx * l) for d in (near, far) for l in (-half, half)]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return (node, min(xs), min(ys), max(xs), max(ys))

    def _pickup_rect(self, pad):
        """ADDED (2026-10-26): the box to pick up on `pad`: a 5 cm square on
        the pad centre."""
        bx, by = self.track.nodes[pad]
        return (pad, bx - 0.025, by - 0.025, bx + 0.025, by + 0.025)

    # -- time -------------------------------------------------------------
    def advance(self, us):
        if self.stopped is not None:
            raise SimStop(self.stopped)
        self.now_us += us
        while self.now_us - self._phys_us >= self.step_us:
            self._phys_us += self.step_us
            self._step(self.step_us / 1e6)
        if not self._busy:
            # Scheduled events and the IR signal can fire IRQ handlers, which
            # read the clock (= advance() again); do not re-enter from there.
            self._busy = True
            try:
                while self._schedule and self._schedule[0][0] <= self.now_us:
                    fn = self._schedule.pop(0)[1]
                    fn()
                if self.ir_pin is not None:
                    level = self._ir_level()
                    if level != self.inputs.get(self.ir_pin):
                        self.set_input(self.ir_pin, level)
            finally:
                self._busy = False
        if self.now_us >= self.t_end_us:
            self._stop("time limit %.1f s" % (self.t_end_us / 1e6))

    def at(self, t_s, fn):
        """Run fn() once the simulated time reaches t_s."""
        self._schedule.append((int(t_s * 1e6), fn))
        self._schedule.sort(key=lambda e: e[0])

    def t(self):
        return self.now_us / 1e6

    def _stop(self, reason):
        self.stopped = reason
        self.events.append((self.t(), "stop", reason))
        raise SimStop(reason)

    # -- hardware interface used by the fakes --------------------------------
    def pin_input(self, pin):
        i = self._line_pin_index.get(pin)
        if i is not None:
            white = self.line_white[i]
            lvl = self.pins["line_white_level"]
            return lvl if white else 1 - lvl
        return self.inputs.get(pin)

    def pin_output(self, pin, level):
        self.out_levels[pin] = level

    def pwm_output(self, pin, duty):
        self.duty[pin] = duty
        # ADDED (2026-10-27): remember the lowest / highest duty of each pin
        # (0 = no pulses, e.g. a motor switched off or a limp servo: skipped)
        if duty:
            low, high = self.duty_range.get(pin, (duty, duty))
            self.duty_range[pin] = (min(low, duty), max(high, duty))
        # ADDED (2026-10-26): servo2 closing with a pick-up box right in front
        # of the nose (front ToF 80 mm or less, its ray hitting that box)
        # takes the box.
        if (pin == self.grip_pin and duty and self.grip_closed_duty is not None
                and duty <= self.grip_closed_duty and self.carrying is None):
            # FIX (2026-10-27): take the NEAREST box on the ToF's ray, and only
            # if it is 80 mm or closer. Before, the first box in the list that
            # the ray hit at any distance was taken: at PAD_YELLOW (facing
            # west) the ray also hits the PAD_GREEN box across the table, on
            # the same row, and the robot "carried" the green box (and its
            # magnet pole).
            nearest, nearest_d = None, None
            for rect in self._pickup_rects:
                rd = self._ahead_distance(rect)
                if rd is not None and (nearest_d is None or rd < nearest_d):
                    nearest, nearest_d = rect, rd
            if nearest is not None and nearest_d <= 0.080:
                self._pickup_rects.remove(nearest)
                self.carrying = nearest[0]

    def adc_input(self, pin):
        if pin == self.mag_pin:
            mv = self.mag_baseline_mv + self.magnet_field_mv() + self._noise_mv(15)
            return max(0, min(65535, int(mv * 65535 / 3300)))
        return None

    # -- task sensors -------------------------------------------------------
    def nose(self):
        """Front centre of the robot (where the magnet and front ToF sensors sit)."""
        f = self.spec.body_front
        return self.x + f * math.cos(self.th), self.y + f * math.sin(self.th)

    def magnet_field_mv(self):
        """Field at the magnet sensor: +-600 mV within 10 cm of a pad with a magnet."""
        # ADDED (2026-10-26): a carried box keeps its magnet at the sensor
        if self.carrying is not None:
            pole = self.pad_poles.get(self.carrying)
            if pole is not None:
                return 600 * (self.mag_north_sign if pole == "north" else -self.mag_north_sign)
        nx, ny = self.nose()
        for pad in self.track.pads:
            pole = self.pad_poles.get(pad["name"])
            if pole is None:
                continue
            cx, cy = pad["centre"]
            if math.hypot(nx - cx, ny - cy) < 0.10:
                sign = self.mag_north_sign if pole == "north" else -self.mag_north_sign
                return 600 * sign
        return 0

    def _noise_mv(self, amp):
        self._noise = (self._noise * 1103515245 + 12345) & 0x7FFFFFFF
        return (self._noise % (2 * amp + 1)) - amp

    def front_distance_mm(self, max_mm=1200):
        """Front ToF: distance along the heading to a box in a slot or the table edge."""
        ox, oy = self.nose()
        dx, dy = math.cos(self.th), math.sin(self.th)
        best = None
        for name, x0, y0, x1, y1 in self._box_rects + self._pickup_rects:   # CHANGED (2026-10-26)
            if x0 <= ox <= x1 and y0 <= oy <= y1:
                return 15     # pressed against a box: a real VL53L0X reads "very close" (or garbage)
        # CHANGED (2026-10-26): the boxes on the pick-up pads are seen too
        rects = ([r[1:] for r in self._box_rects] + [r[1:] for r in self._pickup_rects]
                 + [(0.0, 0.0, self.track.width, self.track.length)])
        for i, (x0, y0, x1, y1) in enumerate(rects):
            d = _ray_box(ox, oy, dx, dy, x0, y0, x1, y1, inside_exit=(i == len(rects) - 1))
            if d is not None and (best is None or d < best):
                best = d
        if best is None or best * 1000 > max_mm:
            return None
        return int(best * 1000)

    def rear_distance_mm(self, max_mm=1200):
        """ADDED (2026-10-27): rear ToF (config.REAR_TOF, at the robot's tail):
        distance straight backwards to a box in a slot / on a pad or the table
        edge, like front_distance_mm() but from the back of the body."""
        b = self.spec.body_back
        ox, oy = self.x - b * math.cos(self.th), self.y - b * math.sin(self.th)
        dx, dy = -math.cos(self.th), -math.sin(self.th)
        best = None
        rects = ([r[1:] for r in self._box_rects] + [r[1:] for r in self._pickup_rects]
                 + [(0.0, 0.0, self.track.width, self.track.length)])
        for i, (x0, y0, x1, y1) in enumerate(rects):
            d = _ray_box(ox, oy, dx, dy, x0, y0, x1, y1, inside_exit=(i == len(rects) - 1))
            if d is not None and (best is None or d < best):
                best = d
        if best is None or best * 1000 > max_mm:
            return None
        return int(best * 1000)

    def _ahead_distance(self, rect):
        """ADDED (2026-10-26), CHANGED (2026-10-27): distance (m) along the
        front ToF's ray to this box, or None if the ray misses it (it was
        _ahead_hits(), which only said hit / miss)."""
        ox, oy = self.nose()
        if rect[1] <= ox <= rect[3] and rect[2] <= oy <= rect[4]:
            return 0.0          # the nose is pushed into the box (front ToF reads 15 mm then)
        dx, dy = math.cos(self.th), math.sin(self.th)
        return _ray_box(ox, oy, dx, dy, rect[1], rect[2], rect[3], rect[4])

    def light_state(self, t_s=None):
        if self.light is None:
            return None
        red, green, phase = self.light
        t = (self.t() if t_s is None else t_s) + phase
        return "red" if (t % (red + green)) < red else "green"

    def _ir_level(self):
        """Traffic light IR: 600 us bursts every 40 ms while the light is in the
        ir_when state (receiver output LOW during a burst), silent otherwise."""
        if self.light_state() != self.ir_when:
            return 1
        return 0 if (self.now_us % 40000) < 600 else 1

    # -- buttons ------------------------------------------------------------
    def set_input(self, pin, level):
        """Drive an input pin (fires its IRQ handler on an edge)."""
        import machine
        self.inputs[pin] = level
        p = machine.Pin.get(pin)
        if p is not None:
            p.sim_set(level)

    def press(self, pin, t_s, hold_ms=80):
        """Schedule a press of the (active-low) crash sensor on `pin`."""
        released = self.pins.get("crash", {}).get(pin, 1)
        self.at(t_s, lambda: (self.events.append((self.t(), "press", "button GP%d" % pin)),
                              self.set_input(pin, 1 - released)))
        self.at(t_s + hold_ms / 1000.0, lambda: self.set_input(pin, released))

    # -- physics ------------------------------------------------------------
    def _wheel_target(self, side):
        p = self.pins
        duty = self.duty.get(p[side + "_pwm"], 0) / 65535.0
        if duty < self.spec.deadband:
            return 0.0
        forward = self.out_levels.get(p[side + "_dir"], p["forward_level"]) == p["forward_level"]
        if p.get(side + "_invert"):
            forward = not forward
        gain = self.spec.left_gain if side == "left" else self.spec.right_gain
        v = duty * self.spec.vmax * gain
        return v if forward else -v

    def _step(self, dt):
        s = self.spec
        k = 1.0 if s.tau <= 0 else min(1.0, dt / s.tau)
        self.vl += (self._wheel_target("left") - self.vl) * k
        self.vr += (self._wheel_target("right") - self.vr) * k
        v = (self.vl + self.vr) / 2
        w = (self.vr - self.vl) / s.track_width
        # exact integration of a constant (v, w) arc over dt
        if abs(w) < 1e-9:
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
        else:
            th2 = self.th + w * dt
            self.x += v / w * (math.sin(th2) - math.sin(self.th))
            self.y -= v / w * (math.cos(th2) - math.cos(self.th))
            self.th = th2
        self.distance += abs(v) * dt
        self._update_line_sensors()
        if self.now_us >= self._next_trace_us:
            self._next_trace_us += self._trace_every_us
            self.trace.append((self._phys_us / 1e6, self.x, self.y, self.th, self.line_bits(), self.vl, self.vr))
        if self.stop_on_collision:
            hit = self._collision()
            if hit:
                self._stop("robot hit the %s" % hit)

    def sensor_points(self):
        c, s_ = math.cos(self.th), math.sin(self.th)
        a = self.spec.sensor_ahead
        pts = []
        for lat in self.spec.sensor_lat:
            # forward = (c, s), left = (-s, c)
            pts.append((self.x + a * c - lat * s_, self.y + a * s_ + lat * c))
        return pts

    def _update_line_sensors(self):
        self.line_white = [self.track.white_at(px, py) for px, py in self.sensor_points()]

    def line_bits(self):
        """Same bit order as sensors.line_processing: bit 3 = outer-left ... bit 0 = outer-right."""
        b = 0
        for i, w in enumerate(self.line_white):
            if w:
                b |= 1 << (3 - i)
        return b

    def body_outlines(self):
        """The robot's footprint as two rectangles (plate, wheel strip), each a
        list of 4 corner points in world coordinates."""
        s = self.spec
        c, s_ = math.cos(self.th), math.sin(self.th)
        out = []
        for front, back, half in ((s.body_front, s.body_back, s.body_half_width),
                                  (s.wheel_half_length, s.wheel_half_length, s.wheel_half_width)):
            rect = []
            for f, l in ((front, half), (front, -half), (-back, -half), (-back, half)):
                rect.append((self.x + f * c - l * s_, self.y + f * s_ + l * c))
            out.append(rect)
        return out

    def _collision(self):
        for corners in self.body_outlines():
            pts = list(corners)
            for i in range(4):   # edge midpoints too
                (ax, ay), (bx, by) = corners[i], corners[(i + 1) % 4]
                pts.append(((ax + bx) / 2, (ay + by) / 2))
            for px, py in pts:
                hit = self.track.obstacle_at(px, py)
                if hit:
                    return hit
            for rack in self.track.racks:   # rack corner inside the robot body
                x0, y0, x1, y1 = rack["rect"]
                from sim.arena import RACK_TOUCH
                m = RACK_TOUCH                  # touching is allowed (see arena.RACK_TOUCH)
                x0, y0, x1, y1 = x0 + m, y0 + m, x1 - m, y1 - m
                for px, py in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                    if _inside(px, py, corners):
                        return rack["name"]
        return None


def _inside(px, py, poly):
    sign = 0
    for i in range(len(poly)):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % len(poly)]
        cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        if cross != 0:
            if sign == 0:
                sign = 1 if cross > 0 else -1
            elif (cross > 0) != (sign > 0):
                return False
    return True


def _ray_box(ox, oy, dx, dy, x0, y0, x1, y1, inside_exit=False):
    """Distance along the ray (ox, oy) + t (dx, dy) to the box [x0, x1] x [y0, y1].
    inside_exit=True: the ray starts inside and the exit point is wanted (table edge)."""
    tmin, tmax = -1e9, 1e9
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
    if tmax < max(tmin, 0):
        return None
    if inside_exit:
        return tmax if tmax >= 0 else None
    return tmin if tmin >= 0 else None
