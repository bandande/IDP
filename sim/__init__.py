"""
PC simulator for the line-following robot (runs on CPython, not on the Pico;
do not upload this folder to the robot).

  sim/arena.py   the competition table (topology from the bench video)
  sim/world.py   robot motion (rear-wheel differential drive), line sensors,
                 buttons, collisions
  sim/fakes/     fake machine / utime / micropython so the robot code runs
                 unchanged
  sim/render.py  PNG pictures of the table and of a run
  sim/run.py     command line: python -m sim.run --help

Quick start (from the repository root):
    python -m sim.arena --png sim_arena.png     # draw the table
    python -m sim.run --time 40 --png run.png   # run main.py on it
"""
