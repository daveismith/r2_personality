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
