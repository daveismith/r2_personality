# Dome motion measurements (phase 2 step 8b)

Measured on the droid on 2026-10-23. They re-fit the coast of plan 2.1 and 4.5 at
personality speeds and set `coast` in `config/expression.yaml` (`c1_s` 0.52, `c2_rad` 0.0)
and the defaults of `r2_mocks`' `DomePlant`.

## Method

- From the dev container, through the converter's Foxglove input: `/cmd_vel_dome_foxglove`
  (the converter scales it by 1.5, so a commanded 0.6, 1.0, 1.5 and 2.0 rad/s to the dome
  was published as 0.4, 0.67, 1.0 and 1.33). The dome arbiter was stopped, so the converter
  was the only source.
- Each speed was published for 3 s, then nothing. The converter's 0.5 s timeout turned the
  silence into a 0 on `/dome_controller/commands`, which came 0.41 to 0.45 s after the last
  Foxglove message.
- Positions: `/dome_state_broadcaster/joint_states` at 10 Hz.
- **Achieved speed**: the mean slope of the position over the last 1 s before the 0.
- **Time to 90 %** (t90): from the first sample that moved to the first sample at or above
  90 % of the achieved speed (the speed from successive positions).
- **Coast**: the position at rest (the first sample after which it did not change) minus
  the position at the converter's 0 on `/dome_controller/commands`. Degrees are the same
  number converted.
- Sampling at 10 Hz: the stop time and the rest time are each known to 0.1 s, so a coast has
  an error of about 0.1 * v rad (0.13 to 0.18 rad at 1.3 to 1.8 rad/s), and a t90 an error
  of 0.1 s.
- The sign convention and home are plan 2.5 (confirmed). The dome turned 13.9 rad through the
  slip ring during the fit.

## Table

| commanded rad/s | achieved rad/s (last 1 s) | t90 | coast rad | coast deg |
|---|---|---|---|---|
| +0.6 | +0.35 | 0.60 s | +0.122 | 7.0 |
| +1.0 | +1.29 | 0.96 s | +0.611 | 35.0 |
| +1.5 | +1.33 | 0.55 s | +0.750 | 43.0 |
| +2.0 | +1.80 | 0.41 s | +0.890 | 51.0 |
| -1.0 | -0.76 | 0.50 s | -0.454 | -26.0 |

No rebound after any stop: the single 8 degree spring-back seen in acceptance item 1 did not
recur.

## Fit

Least squares of the coast against the achieved speed:

    coast = 0.516 s * v - 0.006 rad

so the config uses `c1_s` 0.52 and `c2_rad` 0.0 (the offset is under a third of a degree
and below the sampling error). The fit is on the achieved speed, which is what the dome
was doing when the 0 arrived; the move's early stop uses its measured speed, so the
two match.

## Against the calibration findings

The findings' fit was `0.37 s * v - 4 deg` (0.37 v - 0.07 rad), measured at MinSpeed 20
and mostly higher speeds ("reliable from about 2 rad/s"). At personality speeds it
predicts 0.30 rad at 1.0 rad/s and 0.67 rad at 2.0 rad/s against the 0.52 and 1.04 the
new fit gives, and against 0.61 and 0.89 measured: the old fit would have stopped
the dome 0.2 to 0.3 rad too late, hence the re-fit. The slope is higher and the offset
vanishes. (The findings also record 0.47 to 0.50 s * v on other days; the dome's coast
varies with friction and warm-up, which the move's learned coast scale absorbs.)

## Velocity map

Commanded against achieved: 0.6 gave 0.35, 1.0 gave 1.29, 1.5 gave 1.33, 2.0 gave 1.80, and -1.0
gave -0.76. The map is loose below 2 rad/s, as `calibration_findings.md:139` says
(1.5 rad/s ran 14 to 36 % slow there; here 1.0 ran 29 % fast). Two commands of the same
value can differ in speed. The position loop absorbs it: it is closed on the angle
and uses the measured speed for its early stop. Feel varies, the heading does not.
The one point near the floor (0.6 giving 0.35, with a 0.12 rad coast) is why the plant
models commands under 0.7 rad/s as achieving 0.58 of the command.

## Lag

t90 was 0.4 to 1.0 s (0.41 at 2.0, 0.55 at 1.5, 0.96 at 1.0, 0.60 at 0.6, 0.50 at -1.0),
faster than the findings' 1.5 s step response (which was with the dome on the plate and
at higher speeds, and slower by 2.6 times than without it). The plant's default is
0.6 s to 90 %.

## What came of it

- `r2_expression/config/expression.yaml`: `coast: {c1_s: 0.52, c2_rad: 0.0}`;
  `Coast`'s defaults in `motion.py` and the config docstring agree.
- `r2_mocks` `DomePlant`: lag 0.6 s to 90 %, coast 0.52 v with no offset, the floor
  achieving 0.58 of a command under 0.7 rad/s; the earlier numbers stay as the
  `findings_2026_10` preset.
- Plan 2.1, 4.5, step 8b and the risks.
- Open: the acceleration cap by feel at 2 rad/s; acceptance items 2 to 16 and the CPU
  measurement. Large moves of the `lazy` profile (v_max at the floor, 0.35 rad/s achieved)
  take over 8 s beyond about 120 degrees: watch `max_move_s` for them.

## Position misreads

Also measured on 2026-10-23, from the 10 Hz joint states with the dome turning clockwise at
1.0 rad/s: every time the dome passes about 313 to 331 degrees the reading drops to about
279 degrees for one sample and then jumps forward to about 325 to 331 degrees. The deltas of
consecutive samples were -0.59 then +0.91 rad on one revolution and -0.58 then +0.80 rad on
the next, at the same place. Anticlockwise, and at rest, there were no spikes (every
consecutive delta under 0.2 rad).

A one-sample outlier of 34 degrees wrecks the P-loop: the measured speed reads about -6 rad/s,
the extrapolated position and the early stop follow it, and in one move the loop reversed
direction and ran to its budget. Hence `defaults.dome.max_plausible_rad_s` (4.0, twice the
cap) and rule 9 of `motion.py`: a sample whose implied speed from the last accepted sample
exceeds it is held back, and is dropped unless the next sample agrees with it within the
same speed (a genuine fast push, or a wrap crossing with the firmware's 9-degree step, is
accepted one sample late). `DomePlant(misread_at=(313, 331, 279))` reproduces the fault in the
tests; the grid across the sector lands in the band with the gate and some moves miss it
without.

## Loop tuning

Also 2026-10-23, real dome, calm locked, hands off, with `coast` at the measured 0.52 s * v
and the arbiter learning the coast scale (the first rule 5). Five consecutive manual moves
(joint angles, radians; bearings are the opposite sign):

| move | what happened |
|---|---|
| look to -1.0 | landed 0.3 degrees off |
| look to +1.0 | stopped 7.3 degrees short |
| look to -3.0 (across the wrap, a +2.4 rad move) | overshot by 53 degrees; after one retry ended 15 degrees past |
| snappy look to 0 | overshot 9 degrees, then corrected to 1 degree |
| 0.6 rad glance | went out 1.43 rad (48 degrees of overshoot) and returned to 1 degree |

The arbiter's log showed `move over, coast scale X` with X swinging 0.30, 0.53, 0.19, 0.47,
0.23, 1.19, 0.23, 0.72, 1.25, 1.08 over consecutive moves. The learned scale was noise:
10 Hz samples, the ring misreads, and the velocity map's asymmetry (a commanded 1.0 rad/s
gives about 1.29 clockwise and 0.76 anticlockwise). A low lesson makes the next move stop
late by tens of degrees. And the floor speed's own coast (0.6 commanded, 0.35 to 0.5
achieved, `0.52 * v` = 0.18 to 0.26 rad, 10 to 15 degrees; measured 0.12 at 0.35) exceeds the
old 3 degree settle band, so a retry at the floor cannot converge on a 7 degree error.

Chosen defaults (`defaults.dome`, all configurable):

- `coast.learn` false: the scale stays 1.0. With true: `learn_rate` 0.3 of the way to the
  observed ratio per lesson, kept within `scale_limits` [0.7, 1.4]. The 10 minute fade stays.
- `retries` 2, `final_pulse_below_rad` 0.35, `floor: {coast_rad: 0.12, speed_rad_s: 0.35}`:
  a retry from under 0.35 rad is one pulse at the floor of
  `clamp((|err| - 0.12) / 0.35, 0.1, 0.6)` s, then a 0, settle and check again (so a 0.2 rad
  error gets 0.23 s, which coasts in to about the target).
- `settle_band_rad` 0.10 (5.7 degrees) for every profile at first (per-profile override kept); raised to 0.15 (8.6 degrees) in the shipped yaml after six robot moves ended 4-8 degrees off, see the robot note.

`r2_mocks`' `DomePlant` follows: a command from 0.7 up to 1.25 rad/s achieves `speed_gain`
of itself (1.29 clockwise, 0.76 anticlockwise by default; the measured 1.5 and 2.0 gave 0.89
and 0.90 and stay as the command), and the coast below 0.76 rad/s is the measured 0.12 rad at
0.35 rad/s (the line would say 0.18).

Simulation on that plant (`tuning_grid` in `test/test_motion.py`): look_at moves of 0.6, 1.0
and 2.4 rad, both ways, from eight starts (none, pi, 0 and 2 pi crossed), for snappy, smooth
and lazy, with a `CoastMemory` carried between moves. With learning off or on, every one of
the 144 moves ends within the 0.10 band after at most two retries, with no timeouts (worst
miss 5.5 degrees); with the firmware quirks and the ring misread on top, 143 of 144 (a
snappy 0.6 rad move whose start sits in the 355 to 4 degree gap, 9.6 degrees). To be
confirmed on the droid: item 8b of the plan, the same five moves again.
