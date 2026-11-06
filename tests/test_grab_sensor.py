from utime import sleep_ms

import config
from control.task_sensors import TaskSensors, make_grab_tof


def test_grab_sensor(seconds=None):
    tasks = TaskSensors(grab_tof=make_grab_tof())
    print("grab sensor %s; box seen < %d mm, in reach <= %d mm"
          % (config.GRAB_TOF, config.GRAB_SEE_MM, config.GRAB_REACH_MM))
    loops = 0
    while seconds is None or loops * 200 < seconds * 1000:
        d = tasks.box_distance_now()
        if d is None:
            print("  ---- mm  (nothing in range)")
        else:
            state = "IN REACH" if tasks.box_in_reach(d) else ("box seen" if tasks.box_seen(d) else "-")
            print("  %4d mm  %s" % (d, state))
        sleep_ms(200)
        loops += 1


if __name__ == "__main__":
    test_grab_sensor()
