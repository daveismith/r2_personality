# 2026-11-18 dome floor speed retuned for the new velocity map (phase 2 follow-up)

The velocity map of r2_bringup #10 (2026-11-01, reprinted dome holder: positive k 12.90, p0
11.38, minimum 20 %; negative k 12.46, p0 12.89, minimum 22 %) reached `main` beside phase 2
on 2026-11-18. With it the personality floor of 0.6 rad/s rounds to 19 % and 20 %, both under
the minimums, so the driver sends 0 %: a floor-speed command would not turn the dome. The
new floors are about 0.63 rad/s positive and 0.69 negative.

Run on the droid: body Pi on `main` of `r2_bringup` (`da40713`) and `r2_personality`
(`a191d84`) with the edits below in the working tree, `launch_robot.launch.py`, calm locked,
hands off. Driven from the dev container with `express`; `rec.py` logged every joint state
(10 Hz), personality command, controller command and dome arbiter log line. The dome's MCU
was not on the graph (not needed: the dome drive is on the body).

## What changed

| setting | was | now | why |
|---|---|---|---|
| `defaults.dome.min_speed_rad_s` (`expression.yaml`) and `personality_min_speed_rad_s` (`dome_teleop.launch.py`) | 0.6 | 0.75 | 21 % positive, 22 % negative: above both minimums |
| `profiles.lazy.v_max` | 0.6 | 0.75 | a profile may not top out under the floor (the arbiters refuse to start) |
| `defaults.dome.floor` | `{coast_rad: 0.12, speed_rad_s: 0.35}` | `{coast_rad: 0.0, speed_rad_s: 0.35}` | retry pulses of 0.15 to 0.35 s did not turn the dome (0 to 3 degrees); the motor does nothing for about the first 0.3 s. With `coast_rad` 0 a 10 degree error is a 0.5 s pulse |

The code defaults (`DomeDefaults`, `Floor`, the converter's parameter default) stay at the old
values: the mock's plant still models the old map and the unit tests are built on it.

## Measured

- **Lazy looks of 1.2 rad at the floor, before the pulse change** (`lazy-moves.log`, `an.py`):
  the dome turned every time; cruise about 0.43 rad/s positive and 0.6 negative; the looks
  ended 7 to 11 degrees short and the 0.15 to 0.35 s retry pulses did not close the gap.
- **Two rounds of the phase 2 look and glance set, before the pulse change**
  (`round-before.txt`): 14 moves, final error 1.0 to 10.3 degrees, mean 6.9; one outside the
  0.15 rad (8.6 degree) band after its retries.
- **The same two rounds after it** (`round-after.txt`): 14 moves, final error 0.0 to 8.3
  degrees, mean 3.7; all inside the band. The 2026-10-23 acceptance had 5.7 to 7.3 degrees.
- **Idle glances, `curious --lock`, 45 s**: three whole glances of 62, 67 and 72 degrees
  (palette 29 to 80), returns -8, +1 and -6 degrees, about 6 s each, 14 s apart.
- Every non-zero personality command in the after-round and the idle run was between 0.75
  and 2.00 rad/s (141 in the idle run).

## Open

- The mock's plant (`r2_mocks/dome_plant.py`) and the code defaults still describe the old
  map; the sim has not been re-run with the shipped 0.75 floor (no node tests while the robot
  is connected).
- The floor's `speed_rad_s` is now a pulse-length constant, not the cruise speed.
- David did not watch these moves; judged from the joint telemetry only.
