import config
import constants
from actuators.gripper import grab_box, grab_finish_box
from sensors.magnetic_confidence_v2 import detect_magnetic_pole


def _speed_at_least_1(speed):
    """A motor speed (%) that is never 0 or less.

    Robot.forward() treats a speed of 0 (or None) as "not given" and then
    drives that wheel at the full DRIVE_SPEED instead. So if a config value
    ever made a steering speed 0 (for example PICKUP_STEER equal to
    PICKUP_FOLLOW_SPEED), the robot would suddenly turn hard the wrong way.
    1 % is so weak that the wheel stands still, which is what was meant."""
    if speed < 1:
        return 1
    return speed


class PickUp:
    def __init__(self, robot, motor3, motor4, servo1, servo2, tasks, leds, uturn_direction="left"):
        """robot: functions.Robot (line sensor attributes, forward/spin/stop).
        uturn_direction: "left" or "right", the way to spin
        for the U-turn - away from the neighbouring box's line (see
        config.PICKUP_UTURN_DIRECTION).
        motor3 = left wheel, motor4 = right wheel.
        servo1 / servo2: actuators.servo.Servo, already in the carry pose
        (gripper_start): servo1 high = arm up, servo2 high = gripper open.
        tasks: control.task_sensors.TaskSensors (box_distance_now(): the
        GRAB sensor at the front, config.GRAB_TOF)."""
        self.robot = robot
        self.motor3 = motor3
        self.motor4 = motor4
        self.servo1 = servo1
        self.servo2 = servo2
        self.tasks = tasks
        self.leds = leds


        # The public state, read by main.py and the simulator check.
        self.phase = "idle"
        self.ok = False
        self.message = ""
        self.last_distance_mm = None
        self.pole = None
        self.level = None
        
        self.no_box=False

        # Internal state (the leading underscore means "only used in here").
        self._phase_start_ms = 0      # now_ms when the present phase began (for its time limit)
        self._confirm_count = 0       # sensor pattern seen this many update() calls in a row
        self._left_speed = config.PICKUP_FOLLOW_SPEED    # the last line-following command,
        self._right_speed = config.PICKUP_FOLLOW_SPEED   # kept while both centre sensors are black
        self._box_gripped = False     # "grab": True once grab_box() has finished
        self._left_line = False       # "u_turn": True once both centre sensors were black
        self._last_reading_ms = 0     # "to_box": now_ms of the last real distance

    
    # -----------------------------------------------------------------
    # The two methods main.py calls
    # -----------------------------------------------------------------
    def start(self, now_ms):
        """Begin the pick-up. Call once, with the robot standing still right
        after it turned from the main line into the box's stub line.
        With no grab sensor it ends at once as "failed"
        (the next update() returns True and the robot never moves)."""
        self.ok = False
        self.message = ""
        self.last_distance_mm = None
        self.pole = None
        self.level = None
        self._box_gripped = False
        self._left_line = False
        self._reset_line_following()
        # The robot does not move yet: the first update() looks at the line
        # sensors and sends the first drive command.
        self._change_phase("to_cross_line", now_ms, "following the box line")
        if self._no_grab_sensor():
            self._fail(now_ms, "no grab distance sensor (%s did not start) - not moving"
                       % config.GRAB_TOF)

    def update(self, now_ms):
        """Do one step of the pick-up. Call every main-loop iteration, AFTER
        the robot's line sensor attributes were refreshed.

        Returns True once the pick-up has finished ("done" or "failed"),
        else False. After it has finished it does nothing and keeps
        returning True."""
        if self.phase == "done" or self.phase == "failed":
            return True
        if self.phase == "idle":
            return False           # start() was not called yet: nothing to do

        # Each phase has its own small method below.
        if self.phase == "to_cross_line":
            self._update_to_cross_line(now_ms)
        elif self.phase == "to_box":
            self._update_to_box(now_ms)
        elif self.phase == "grab":
            self._update_grab(now_ms)
        elif self.phase == "reverse":            
            self._update_reverse(now_ms)
        elif self.phase == "back_to_main":
            self._update_back_to_main(now_ms)

        return self.phase == "done" or self.phase == "failed"

    # -----------------------------------------------------------------
    # One method per phase
    # -----------------------------------------------------------------
    def _update_to_cross_line(self, now_ms):
        """Step 1: follow the stub line until all four sensors see the white
        cross line in front of the box (PICKUP_CROSS_CONFIRM calls in a row,
        so a single noisy read does not count)."""
        r = self.robot
        all_four_white = (r.left_sensor == 1 and r.front_left_sensor == 1
                          and r.front_right_sensor == 1 and r.right_sensor == 1)
        if all_four_white:
            self._confirm_count += 1
        else:
            self._confirm_count = 0

        if self._confirm_count >= config.PICKUP_CROSS_CONFIRM:
            # Cross line found: from here on drive dead straight. The centre
            # sensors are no use for steering any more (the cross line and
            # the box's pad confuse them), and the robot already points
            # along the stub line.
            self._change_phase("to_box", now_ms, "cross line found")
            self._last_reading_ms = now_ms
            
            self._update_to_box(now_ms)
            
            return
        if self._time_is_up(now_ms, config.PICKUP_TO_CROSS_MAX_MS):
            self._fail(now_ms, "no cross line within %d ms" % config.PICKUP_TO_CROSS_MAX_MS)
            return

        self._follow_line()

    def _update_to_box(self, now_ms):
        """Step 2: drive straight until the GRAB distance sensor reads
        GRAB_STOP_MM or less. If that does not happen within
        GRAB_APPROACH_MAX_MS the pick-up fails: the gripper must never close
        on nothing (or on a box in the wrong place).
        It also fails (and stops) when the sensor gives no distance at all
        for config.GRAB_NO_READING_MAX_MS in a row."""
        if self._no_grab_sensor():
            self._fail(now_ms, "no grab distance sensor (%s did not start) - not driving to the box"
                       % config.GRAB_TOF)     # _fail() stops the wheels
            return
        distance = self.tasks.box_distance_now()   # non-blocking, mm or None
        self.last_distance_mm = distance
        


        if distance is not None and distance <= constants.DISTANCE_TO_PICK_UP:
            print(distance)
            # Stop at once, the box is right in front of the gripper.
            self.robot.stop(self.motor3, self.motor4)
            self._change_phase("grab", now_ms, "box %d mm ahead" % distance)
            return


        if distance is not None:
            self._last_reading_ms = now_ms
        elif now_ms - self._last_reading_ms >= config.GRAB_NO_READING_MAX_MS:
            self._fail(now_ms, "no distance from the grab sensor for %d ms (not answering, or no box in range)"
                       % (now_ms - self._last_reading_ms))
            
            return

        if self._time_is_up(now_ms, config.GRAB_APPROACH_MAX_MS):
            self._fail(now_ms, "no box within %d mm after %d ms"
                       % (config.GRAB_STOP_MM, config.GRAB_APPROACH_MAX_MS))
            
            if self._last_reading_ms>=config.GRAB_APPROACH_MAX_MS-100:
                print("CONTINUING")
                self._last_reading_ms = now_ms
                self._change_phase("reverse", now_ms, "no box detected")
                self.no_box=True

            else:
                print("some readings received but still failed")
            return

        # Not there yet: keep driving straight (sent again every call, so
        # the wheels always do what this phase wants).
        self._drive_straight_to_box()
    
        

    def _update_grab(self, now_ms):
        """Step 3: pick the box up, then start the U-turn.

        This phase takes two update() calls:
          1st call: stop, then grab_box() (actuators/gripper.py): servo1 to
                    its low end (arm down), servo2 to its low end (gripper
                    closed, and it stays closed), servo1 to its high end
                    (arm up). The servos only move through Servo.move_slowly,
                    inside config.SERVO_SAFE_RANGE. This WAITS (blocks) for
                    about 1.5 - 2 s while the servos turn.
          2nd call: start the U-turn.
        Why not start the U-turn at the end of the 1st call? Its now_ms was
        taken BEFORE the servos moved, so it is about 2 s out of date. The
        U-turn's minimum time (PICKUP_UTURN_MIN_MS) would then already look
        used up. The 2nd call brings a fresh now_ms."""
        if not self._box_gripped:
            self.robot.stop(self.motor3, self.motor4)   # make sure the wheels stand still
            grab_box(self.servo1, self.servo2)
            
            # Magnetic sensor
            confidence=0
            level=""
            for i in range(5):
                level, confidence = detect_magnetic_pole(
                    adc_pin=26,
                    baseline_data=self.robot.baseline_data,
                    run_time_sec=1+(i*0.2),
                )
                if confidence>0.75:
                    break
                else:
                    print("Confidence too low, try again")
            else:
                self.leds.show(green=True)                        
                self.leds.show(red=True)
            print("level:"+str(level) +"\t confidence:"+ str(confidence))
            
            self.level=level
            self.confidence=confidence
            
            if level == "N":
                # Top means red is on and green is off                
                self.leds.show(green=False)                        
                self.leds.show(red=True)
            else:
                # Bottom means red is off and green is on
                self.leds.show(green=True)                        
                self.leds.show(red=False)
            
            grab_finish_box(self.servo1, self.servo2)
            
            
            
            self.pole = self.tasks.read_pole()
            self._box_gripped = True
            return

        print("reverse")
        self._change_phase("reverse", now_ms, "box held")
        self._left_line = False
        speed = _speed_at_least_1(config.GRAB_REVERSE_SPEED)
        self.robot.forward(self.motor3, self.motor4, speed, speed, reverse=True)

    def _update_reverse(self, now_ms):
        """Step 4: spin on the spot (self.uturn_direction) until the robot faces back along
        the stub line. The U-turn is finished only when ALL of these hold:
          (a) at least PICKUP_UTURN_MIN_MS have passed since the spin began,
          (b) both centre sensors have been black at least once since the
              spin began (so the robot really left the line it started on),
          (c) both centre sensors see white now (the line is found again).
        (a) and (b) stop the U-turn from ending at once, while the centre
        sensors are still on the line the robot started on."""
        r = self.robot


        long_enough = (now_ms - self._phase_start_ms) >= config.PICKUP_REVERSE_MIN_MS   # condition (a)

        if long_enough and (r.left_sensor == 1 or r.right_sensor == 1):
            self.robot.stop(self.motor3, self.motor4)
            self._change_phase("back_to_main", now_ms,
                               "line found again after %d ms" % (now_ms - self._phase_start_ms))
            # Following the line back starts from "straight on".
            self._reset_line_following()
            return

        if self._time_is_up(now_ms, config.PICKUP_UTURN_MAX_MS):
            self._fail(now_ms, "line not found again within %d ms" % config.PICKUP_UTURN_MAX_MS)
            return

        # Keep spinning (sent again every call, like the other phases).
        speed = _speed_at_least_1(config.GRAB_REVERSE_SPEED)
        self.robot.forward(self.motor3, self.motor4, speed, speed, reverse=True)

    def _update_back_to_main(self, now_ms):
        """Step 5: follow the stub line back until BOTH outer sensors see
        white (PICKUP_CROSS_CONFIRM calls in a row): the main line at the
        junction. Then stop: the pick-up is done."""
        r = self.robot
        both_outer_white = r.left_sensor == 1 and r.right_sensor == 1
        if both_outer_white:
            self._confirm_count += 1
        else:
            self._confirm_count = 0

        if self._confirm_count >= config.PICKUP_CROSS_CONFIRM:
            self.robot.stop(self.motor3, self.motor4)
            self.ok = True
            self._change_phase("done", now_ms, None)
            self.message = "back on the main line"
            print(self.tasks.grab_tof.poll())
            return

        if self._time_is_up(now_ms, config.PICKUP_BACK_MAX_MS):
            self._fail(now_ms, "main line not found within %d ms" % config.PICKUP_BACK_MAX_MS)
            return

        speed = _speed_at_least_1(config.GRAB_REVERSE_SPEED)
        self.robot.forward(self.motor3, self.motor4, speed, speed, reverse=True)

    # -----------------------------------------------------------------
    # Small helpers
    # -----------------------------------------------------------------
    def _follow_line(self):
        """Steer along the line with the two centre sensors (1 = white line):
            both white          -> both wheels PICKUP_FOLLOW_SPEED (straight)
            only centre-left    -> the line is drifting to the left: the
                                   left wheel runs PICKUP_STEER slower, so
                                   the robot turns left back onto it
            only centre-right   -> the right wheel runs slower (turn right)
            both black          -> keep the previous command (the robot is
                                   probably just crossing a gap or a corner
                                   of the line; changing course blindly
                                   would make it worse)
        The command is sent every call, so the motors always run it."""
        r = self.robot
        centre_left_white = r.front_left_sensor == 1
        centre_right_white = r.front_right_sensor == 1
        fast = config.PICKUP_FOLLOW_SPEED
        slow = _speed_at_least_1(config.PICKUP_FOLLOW_SPEED - config.PICKUP_STEER)

        if centre_left_white and centre_right_white:
            self._left_speed = fast
            self._right_speed = fast
        elif centre_left_white:
            self._left_speed = slow
            self._right_speed = fast
        elif centre_right_white:
            self._left_speed = fast
            self._right_speed = slow
        # else: both black -> _left_speed / _right_speed stay as they were

        # Never pass 0 or None to Robot.forward (it would use full speed).
        self.robot.forward(self.motor3, self.motor4,
                           _speed_at_least_1(self._left_speed), _speed_at_least_1(self._right_speed))

    def _reset_line_following(self):
        """Forget the last line-following command: start from straight on.
        Used when a line-following phase begins."""
        self._left_speed = config.PICKUP_FOLLOW_SPEED
        self._right_speed = config.PICKUP_FOLLOW_SPEED

    def _drive_straight_to_box(self):
        """Both wheels at GRAB_APPROACH_SPEED: dead straight, no steering."""
        speed = _speed_at_least_1(config.GRAB_APPROACH_SPEED)
        self.robot.forward(self.motor3, self.motor4, speed, speed)

    def _time_is_up(self, now_ms, limit_ms):
        """True when the present phase has been running for limit_ms or longer."""
        return (now_ms - self._phase_start_ms) >= limit_ms

    def _no_grab_sensor(self):
        """True if there is no grab distance sensor
        (TaskSensors leaves tasks.grab_tof as None when it did not start)."""
        return getattr(self.tasks, "grab_tof", None) is None

    def _change_phase(self, new_phase, now_ms, why):
        """Go to `new_phase`: remember when it began (for its time limit),
        clear the "in a row" counter, and print exactly ONE line that starts
        with "pickup: <new phase>" (the simulator check reads these lines).
        `why` is a short reason shown in brackets, or None."""
        self.phase = new_phase
        self._phase_start_ms = now_ms
        self._confirm_count = 0
        if why is None:
            self.message = new_phase
            print("pickup: %s" % new_phase)
        else:
            self.message = why
            print("pickup: %s (%s)" % (new_phase, why))

    def _fail(self, now_ms, reason):
        """End the pick-up as "failed": stop the wheels and say which step
        failed and why. The servos are NOT moved: they keep their position
        (moving the gripper now could drop a box or hit something)."""
        failed_step = self.phase
        self.robot.stop(self.motor3, self.motor4)
        self.ok = False
        self.phase = "failed"
        self._phase_start_ms = now_ms
        self.message = "%s: %s" % (failed_step, reason)
        print("pickup: failed - %s" % self.message)
