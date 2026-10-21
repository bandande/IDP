"""
Fake MicroPython modules for the PC simulator.

sim.run puts this folder at the front of sys.path, so the robot code's
`import machine`, `import utime` and `import micropython` load these fakes
unchanged. Do not import them as `sim.fakes.machine` etc. - that would make
second, separate copies.
"""
