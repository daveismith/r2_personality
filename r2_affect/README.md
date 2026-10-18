# r2_affect

Layer 2 of R2's personality (architecture §4): a small numeric mood model. Stimuli push
four numbers around, the numbers relax back toward R2's temperament, and the nearest of a
handful of mood prototypes gives the label (`calm`, `curious`, `scared`, ...) that later
layers read. Everything is configured from `config/affect.yaml`.

This step (phase 0, step 2) delivers the pure-Python core:

| file | what |
|---|---|
| `r2_affect/model.py` | `AffectModel`: dynamics, impulses, refractory, classification with hysteresis, force/lock, focus, idle. No ROS imports. |
| `r2_affect/config.py` | `load_config()`: reads and validates `affect.yaml` into frozen dataclasses; every rejection raises `AffectConfigError` naming the key. |
| `config/affect.yaml` | the shipped configuration (plan §5.4). |

The `r2_affect` node (`/stimuli` → `/mood/state`, `/mood/set`, `/mood/reload_config`),
its launch file, the `mood` CLI, the bag replay tool and the Foxglove tuning layout arrive
in later steps of the phase 0 plan (`docs/personality-phase0-plan.md` in `r2_dev`).

## The model

State: `arousal` and `valence` in [-1, 1], `curiosity` and `confidence` in [0, 1]. The
axes and ranges are fixed in code. The model starts at the temperament, labelled with the
nearest mood. Every call takes `now` (seconds, any monotonic clock); the first `tick`
sets the time base. There is no global state, so many models can run in one process.

**Tick** (`tick(now)`, 10 Hz in the node):

1. `dt` = now − previous tick, clamped to `[0, max_dt_s]`; the result says when it was
   clamped (sim paused, clock jump) so the node can warn. `dt` = 0 changes nothing.
2. Unless locked: each axis relaxes toward its temperament, `x += (baseline − x)·dt/τ`
   (explicit Euler, factor capped at 1). After `idle.after_s` without a stimulus, the
   `idle` impulse is applied as a rate, `x += impulse · min(1, (idle_s − after_s)/ramp_s) · dt`.
   Axes are clamped to their ranges.
3. A timed lock past `lock_until` expires; a focus past its ttl expires.
4. Classification: distance to a prototype is Euclidean over the axes **it specifies**,
   each difference divided by the axis's half-range and multiplied by its
   `distance_weights` entry. The nearest prototype (first in file order on a tie) is the
   candidate; the label moves to it only after `d_candidate < d_label − margin` has held
   at every tick for `dwell_s`. `mood_strength = clamp(1 − d_label/strength_radius, 0, 1)`.

**Stimulus** (`stimulus(kind, intensity, confidence, subject, bearing_rad, ttl_s, now)`):

1. `confidence < min_confidence`: dropped, nothing else happens.
2. Otherwise `idle_s` resets, even if the stimulus is dropped below (it is evidence of
   the world).
3. A subject or a bearing makes it the focus for `ttl_s` (`focus.default_ttl_s` when 0),
   replacing the previous focus. This happens even when locked.
4. Unknown kind: dropped (legal, so producers can exist before their tuning). A kind with
   `refractory_s`: dropped if the last *accepted* one is younger than that.
5. Unless locked, `impulse · intensity` (intensity clamped to [0, 1]) is added to each
   axis the kind names, then clamped. The result reports accepted/applied, the reason for
   a drop (`low_confidence`, `unknown_kind`, `refractory`, `locked`) and the actual delta.

**Force and lock** (`set_mood(mood, lock, duration_s, source, now)` → `(accepted, reason)`),
the `SetMood` semantics:

- `mood` set: the prototype's specified axes jump to it, the others stay; the label is
  that mood at once and the dwell timer restarts. Without `lock`, dynamics resume (and a
  lock held by the same source is released).
- `lock`: relax, idle drift and impulses stop until `duration_s` (0: until cleared);
  focus, idle time and refractory keep going. `mood` "" with `lock` locks in place.
- `mood` "" and `lock` false clears, always, from any source.
- While one source holds the lock, any other request from another source is refused and
  the reason names the holder. Unknown labels and negative durations are refused. A
  refused request changes nothing. An empty `source` is recorded as `unknown`.

`reload(config)` swaps the configuration and keeps the state (vector, idle, lock, focus,
refractory history); a label that no longer exists becomes the nearest mood.

## `affect.yaml`

| key | required | default | rule |
|---|---|---|---|
| `temperament` | yes | | every axis, inside its range |
| `relax_tau_s` | yes | | every axis, seconds, > 0 |
| `distance_weights` | no | 1.0 per axis | any subset of axes, >= 0 |
| `strength_radius` | no | 0.5 | > 0 |
| `hysteresis` | no | `{margin: 0.15, dwell_s: 1.5}` | both >= 0 |
| `focus` | no | `{default_ttl_s: 5.0}` | > 0 |
| `idle` | no | `{after_s: 30, ramp_s: 120}` | `after_s` >= 0, `ramp_s` > 0 |
| `impulses` | yes | | kind → `{axis: delta, ..., refractory_s: s}`; `refractory_s` >= 0; the `idle` kind must exist (a rate per second) |
| `moods` | yes | | label → `{axis: centre, ...}`, at least one mood, at least one axis each, inside the ranges; file order breaks ties |

Also rejected: unknown top-level keys, unknown axis names anywhere, unknown keys inside
`hysteresis`/`focus`/`idle`, non-numeric or non-finite values, duplicate keys. Kinds not
in `impulses` are legal to receive.

## Tests

```bash
# from the workspace root, in the dev container
colcon build --symlink-install --packages-select r2_affect
source ./setup.bash
colcon test --packages-select r2_affect && colcon test-result --verbose

# or just the unit tests, no build needed
python3 -m pytest src/r2_personality/r2_affect/test -q
```

The model tests drive `now` by hand with short time constants, so nothing sleeps. The
lint tests (flake8, pep257, copyright) need `/opt/ros/jazzy/setup.bash` sourced.

Apache-2.0.
