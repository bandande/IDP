"""
The task, step by step. This is the file to change when the task rules
change; the settings it uses are in config.py ("Main program: the task").

WHAT THE ROBOT DOES (run)
  For every pad in config.PICKUP_ORDER:
    1. Drive to the pad and pick the box up.
    2. Read the box's magnet: north or south pole -> rack level TOP or BOTTOM
       (config.MAG_POLE_LEVEL).
    3. Work out the slots for this box: the rack comes from the pad
       (config.DELIVERY_RACK), the side of the rack from the level
       (config.DELIVERY_SIDE); slots 1 to 6 are tried in order.
    4. Go to the first slot not yet known to be full: to its junction,
       turned to look down its rung (nav.face_slot). Look into the slot with
       the front distance sensor. If a box is already there, remember that
       and try the next slot; otherwise drive up to the rack
       (nav.enter_slot) and put the box down.
  Finally drive back to START and stop in the start box.

  Driving is done by nav.go_to(node) (navigation.py); node names are in
  track_map.py. A press of the start/stop button or a driving problem stops
  the run (main.py handles that).
"""

from utime import sleep_ms

import config
import track_map


def slots_for(pad, level):
    """The rack slots for a box from `pad` with magnet level `level`, in the
    order to try them, e.g. ["ORANGE_W1", "ORANGE_W2", ...]."""
    rack = config.DELIVERY_RACK[pad]
    side = config.DELIVERY_SIDE[(rack, level)]
    return ["%s_%s%d" % (rack, side, number) for number in range(1, len(track_map.RUNG_Y) + 1)]


def pick_up(tasks, pad):
    """At a pad, facing the box: pick it up and read its magnet.
    Returns the rack level ("TOP" or "BOTTOM") the box must go to."""
    print("picking up the box at %s" % pad)
    sleep_ms(config.PICKUP_TIME_MS)        # TODO: close the gripper (servo code goes here)
    pole = tasks.read_pole()
    level = tasks.level_for_pole(pole)
    print("box magnet: %s (strongest field %s mV) -> %s level"
          % (pole if pole else "NOT FOUND", tasks.last_field_mv, level))
    return level


def put_down(slot):
    """In front of a free rack slot: put the box down."""
    print("putting the box down in %s" % slot)
    sleep_ms(config.DROP_TIME_MS)          # TODO: open the gripper (servo code goes here)


def deliver(nav, tasks, pad, level, full_slots):
    """Take the box from `pad` to the first free slot for it. `full_slots` is
    the set of slots known to be full; it is updated. Returns the slot used,
    or None if every slot for this box is full."""
    for slot in slots_for(pad, level):
        if slot in full_slots:
            continue
        nav.face_slot(slot)
        if tasks.slot_occupied():
            print("%s already has a box (%s mm away) - trying the next slot"
                  % (slot, tasks.last_slot_mm))
            full_slots.add(slot)
            continue
        nav.enter_slot(slot)
        put_down(slot)
        full_slots.add(slot)
        return slot
    print("every %s slot for this box is full - keeping the box" % level)
    return None


def run(nav, tasks):
    """The whole task, from the start box back to the start box.
    Returns the number of boxes delivered."""
    full_slots = set()
    delivered = 0
    for pad in config.PICKUP_ORDER:
        nav.go_to(pad)
        level = pick_up(tasks, pad)
        slot = deliver(nav, tasks, pad, level, full_slots)
        if slot is not None:
            delivered += 1
            print("delivered the box from %s to %s (%d so far)" % (pad, slot, delivered))
    nav.go_to("START")
    nav.drive_wheels_onto_node()           # park with the wheels in the start box
    print("mission complete: %d boxes delivered" % delivered)
    return delivered
