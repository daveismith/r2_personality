# Dome device catalogue

Raw `ros2 service call` outputs from the workbench dome (`/r2_dome`, firmware at
r2_domeplayer 92b31f3, astromech_msgs 2.0.0), taken 2026-10-20 through the linked Zenoh
routers. They are the source for `r2_mocks`'s `dome_mock.yaml` catalogue (phase 1 step 2).

- `2026-10-20-workbench-leds-list.txt`: `ListLeds`. Five displays, six surfaces, the 14
  code effects, and 16 clip names from `/data/clips` for each of the three targets
  (`alarm`, `alarm-twinkle`, `colourswap`, `colourwheel`, `fadescroll`, `failure` (18 s,
  does not loop), `flash`, `flipflop`, `flipflop-alt`, `fractal`, `hscan`, `march` (48 s,
  does not loop), `metaballs`, `plasma`, `pulse`, `vscan`).
- `2026-10-20-workbench-magic-list.txt`: `ListMagicPanel` with the bench's simulated panel
  (the Pico simulator): `link` 1, firmware 0.12.1, protocol 1.1, `has_repeat`,
  `has_brightness`, `has_names`, 58 sequences (ids 0-57) with names and measured lengths.
  The droid's real panel needs its own capture.

Measured the same day from the dev container: `RunLedEffect` round trip 8-14 ms, median
10 ms, over ten calls.

## The droid's own dome (2026-11-10)

`2026-11-10-droid-leds-list.txt` and `2026-11-10-droid-magic-list.txt`: the same two calls
on the droid's dome controller (`/r2_dome` on the body Pi's router, reached from the dev
container through the peered routers), phase 1 step 0's droid part.

- **LEDs**: the same five displays, six surfaces and 14 code effects as the workbench. The
  clips differ: `flash` and `pulse` for all three targets, plus the bench test clips
  (`bench-fast`, `bench-gled`, `bench-once`, `bench-rgb`, `bench-tint`) and `built-in`. The
  workbench's other 14 clips are absent, and the palettes name eight of them (`alarm`,
  `alarm-twinkle`, `colourswap`, `fadescroll`, `flipflop`, `flipflop-alt`, `hscan`, `vscan`),
  so on the droid those picks are dropped with one warning each: playful keeps only its
  Heart sequence, annoyed `flash` and `redalert`, curious `pulse`, `cycle`, `roam` and Eye
  scan, scared `flash`, `redalert` and its sequences, sulking `pulse` and Sad face. Copying
  the 16 clips to the droid's `/data/clips` restores them.
- **Magic Panel**: `link` 1, firmware 0.12.1, protocol 1.1, the same 58 sequences with the
  same names and lengths as the bench simulator, `has_eeprom_config` and `has_orientation`
  besides; `inputs` enabled (rotary), `rotated`, `legacy`, brightness 5 (default 15).
