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

## The simulator, the same day

- **The dome measured again by the method of step 8b** (`constant-speed.txt`, `drive.py`,
  `char.py`; driver and recorder on the body Pi, dome arbiter stopped): the table and what it
  says are in `docs/dome-motion-measurements.md`, "The new velocity map". In short: a dead time
  of about 0.35 s and then a fast rise, achieved speed and coast both different by direction.
- **`r2_mocks` `DomePlant` has the preset `measured_2026_11`** built from that table, and
  `dome_drive_mock` runs it by default (`preset`). `DomePlant()` and the code defaults stay on
  the October plant, which the motion model's unit tests are built on; `test_dome_node.py`
  pins that preset. New unit tests: the preset itself (`test_dome_plant.py`), and the shipped
  settings on it (`test_motion.py`): every look of a 10 degree grid in the three profiles
  ends inside the band (mean 3.0 degrees, against 3.7 on the droid), and the old 0.6 floor
  does not turn it.
- **Gazebo keeps its own floor.** Gazebo's dome is an ideal velocity joint (no dead band, dead
  time or coast). With the droid's floor the looks in the headless sim ended 7 to 15 degrees
  off (the 0.5 s retry pulse overshoots); on an ideal plant in the unit harness 90 of 108
  looks ended outside the band. `r2_sim`'s `launch_sim.launch.py` now generates its
  `expression_config` from the shipped file with `min_speed_rad_s` 0.6 and `floor`
  `{0.12, 0.35}`, and passes the converter 0.6. With that, two rounds in the headless sim
  (`sim-round.txt`): 14 moves, final error 0.1 to 3.0 degrees.
- **Tests** (on `ROS_DOMAIN_ID` 42, the robot stack stopped, so nothing could reach the dome's
  MCU, which was still on domain 0): `r2_mocks` 248 passed, `r2_expression` 771 passed.
  `r2_expression`'s flake8, pep257 and copyright tests now lint the code directories only:
  they had been failing on the probe scripts kept under `docs/sessions/`.
- Not from this work, seen in the same run and left: `r2_sim`
  `test_sim_forwards_wav_backend_and_mock_level_topic` fails (it compares a
  `LaunchConfiguration` with `'wav'`), and the flake8 tests of `r2_bringup` and `r2_sim` fail
  on the launch files' quoting style.

## Open

- The code defaults (`Limits`, `DomeDefaults`, `Floor`, the converter's parameter default) are
  the October values, which now suit Gazebo and the old plant, not the droid; the shipped
  `expression.yaml` and `dome_teleop.launch.py` carry the droid's.
- The floor's `speed_rad_s` is a pulse-length constant, not the cruise speed.
- The shipped `coast` is one line for both directions; the droid's coast differs by
  direction (see the measurements).
- David did not watch these moves; judged from the joint telemetry only.
