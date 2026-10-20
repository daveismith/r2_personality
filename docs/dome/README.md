# Dome device catalogue

Raw `ros2 service call` outputs from the workbench dome (`/r2_dome`, firmware at
r2_domeplayer 92b31f3, astromech_msgs 2.0.0), taken 2026-10-20 through the linked Zenoh
routers. They are the source for `r2_mocks`'s `dome_mock.yaml` catalogue (phase 1 step 2).

- `2026-10-20-workbench-leds-list.txt`: `ListLeds`. Five displays, six surfaces, the 14
  code effects, and 16 clip names from `/data/clips` for each of the three targets
  (`alarm`, `alarm-twinkle`, `colourswap`, `colourwheel`, `fadescroll`, `failure` (18 s,
  does not loop), `flash`, `flipflop`, `flipflop-alt`, `fractal`, `hscan`, `march` (48 s,
  does not loop), `metaballs`, `plasma`, `pulse`, `vscan`).
- `2026-10-20-workbench-magic-list.txt`: `ListMagicPanel` with no panel fitted
  (`link` 0, empty catalogue). The droid's panel, if fitted, needs its own capture.

Measured the same day from the dev container: `RunLedEffect` round trip 8-14 ms, median
10 ms, over ten calls.
