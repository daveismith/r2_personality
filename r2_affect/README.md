# r2_affect

Layer 2 of R2's personality (architecture §4): a small numeric mood model. Stimuli push
four numbers around, the numbers relax back toward R2's temperament, and the nearest of a
handful of mood prototypes gives the label (`calm`, `curious`, `scared`, ...) that later
layers read. Everything is configured from `config/affect.yaml`.

Phase 0, step 2 delivered the pure-Python core, step 3 the ROS node around it:

| file | what |
|---|---|
| `r2_affect/model.py` | `AffectModel`: dynamics, impulses, refractory, classification with hysteresis, force/lock, focus, idle. No ROS imports. |
| `r2_affect/config.py` | `load_config()`: reads and validates `affect.yaml` into frozen dataclasses; every rejection raises `AffectConfigError` naming the key. |
| `config/affect.yaml` | the shipped configuration (plan §5.4). |

The bag replay tool and the Foxglove tuning layout arrive in later steps of the phase 0
plan (`docs/personality-phase0-plan.md` in `r2_dev`).

## Running the node

```bash
ros2 launch r2_affect affect.launch.py                       # robot / real time
ros2 launch r2_affect affect.launch.py use_sim_time:=true    # with the simulator
ros2 launch r2_affect affect.launch.py config:=/path/to/affect.yaml rate_hz:=10.0
# or without the launch file
ros2 run r2_affect r2_affect --ros-args -p config:=/path/to/affect.yaml
```

| parameter | default | meaning |
|---|---|---|
| `config` | installed `share/r2_affect/config/affect.yaml` | the YAML to load, and to re-read on reload |
| `rate_hz` | 10.0 | tick and `/mood/state` publish rate |
| `min_confidence` | 0.0 | stimuli below this confidence are dropped |
| `max_dt_s` | 0.5 | longest time step one tick takes (a longer gap is clamped and warned about once) |
| `use_sim_time` | false | follow `/clock`; with the clock stopped the state does not change |

| name | type | |
|---|---|---|
| `/stimuli` | `astromech_msgs/Stimulus` | subscribed, reliable, depth 50 |
| `/mood/state` | `astromech_msgs/MoodState` | published every tick, reliable, depth 10 |
| `/mood/set` | `astromech_msgs/SetMood` | service: force, lock, clear |
| `/mood/reload_config` | `std_srvs/Trigger` | service: re-read the YAML |

All time is the node clock; a stimulus's `header.stamp` is ignored. The node publishes
nothing but `/mood/state` (and `/rosout`, `/parameter_events`). An invalid config at
startup is logged and the node exits with status 1.

Logs: one INFO line per applied stimulus (kind, source, intensity, the change per axis,
the vector after), INFO on label changes (`mood: calm -> curious (d=0.21)`) and on lock
set, cleared or expired, WARN once per unknown kind, once for the reserved `idle` kind
and once for a clamped time step; low-confidence and refractory drops are DEBUG.

## Setting the mood

`SetMood` fields: `mood` (a label from `affect.yaml`, or empty), `lock`, `duration_s`
(0: until cleared), `source` (who asks; the holder of a lock is recorded as `lock_source`).
A force switches the label at once; a lock held by one source is refused for any other
source, with the holder named in the reason; a clear works from any source.

| | `mood` CLI | Foxglove Service Call panel, service `/mood/set` |
|---|---|---|
| force (dynamics resume) | `mood set sleepy` | `{"mood": "sleepy", "lock": false, "duration_s": 0, "source": "foxglove"}` |
| force and lock | `mood set sleepy --lock` | `{"mood": "sleepy", "lock": true, "duration_s": 0, "source": "foxglove"}` |
| lock in place | `mood lock` | `{"mood": "", "lock": true, "duration_s": 0, "source": "foxglove"}` |
| timed lock | `mood set excited --lock --duration 30` or `mood lock --duration 30` | `{"mood": "excited", "lock": true, "duration_s": 30, "source": "foxglove"}` |
| clear | `mood clear` | `{"mood": "", "lock": false, "duration_s": 0, "source": "foxglove"}` |
| watch | `mood watch` | add `/mood/state` to a Raw Messages panel |

Every CLI subcommand takes `--source NAME` (default `mood_cli`) and, like any node,
`--ros-args -p use_sim_time:=true`. `set`, `lock` and `clear` print `accepted` or
`refused: <reason>` and the resulting label, and exit 0 when accepted, 1 otherwise.
`mood watch` prints one line per message: stamp, label (strength), the four axes,
`locked(<source>)` or `unlocked`, `focus=<subject>@<bearing>` and `idle`.

In Foxglove, a Raw Messages panel on `/mood/state` shows `locked` and `lock_source` (and
`lock_until`, zero when the lock lasts until cleared). The response of a Service Call
carries the same state in `result`. A timed lock needs no action to end: it expires on the
first tick after its time and the node logs it.

## Reloading the config

Edit the YAML at the `config` parameter, then:

```bash
ros2 service call /mood/reload_config std_srvs/srv/Trigger
```

The file is read and validated first; only a valid file is swapped in, and the state
(vector, label, lock, focus, idle time) is kept. A broken file answers
`success: false` with the problem (the key and why) and the old config stays in use.

## The model

State: `arousal` and `valence` in [-1, 1], `curiosity` and `confidence` in [0, 1]. The
axes and ranges are fixed in code. The model starts at the temperament, labelled with the
nearest mood. Every call takes `now` (seconds, any monotonic clock); the first `tick`
sets the time base. There is no global state, so many models can run in one process.

**Tick** (`tick(now)`, 10 Hz in the node):

1. `dt` = now − previous tick, clamped to `[0, max_dt_s]`; the result says when it was
   clamped (sim paused, clock jump) so the node can warn. `dt` = 0 changes nothing, not
   even expiry. If the clock went backwards (sim restart), every stored time (lock end,
   focus end, dwell start, refractory) shifts with it, so each falls due at the same
   relative time on the new clock.
2. Unless locked: each axis relaxes toward its temperament, `x += (baseline − x)·dt/τ`
   (explicit Euler, factor capped at 1). After `idle.after_s` without a stimulus, the
   `idle` impulse is applied as a rate, `x += impulse · min(1, (idle_s − after_s)/ramp_s) · dt`.
   Axes are clamped to their ranges.
3. A timed lock expires on the first tick at or after `lock_until`; a focus expires on
   the first tick after its ttl (a tick at exactly the end still shows it).
4. Classification: distance to a prototype is Euclidean over the axes **it specifies**,
   each difference divided by the axis's half-range and multiplied by its
   `distance_weights` entry. The nearest prototype (first in file order on a tie) is the
   candidate; the label moves to it only after `d_candidate < d_label − margin` has held
   at every tick for `dwell_s` with the same candidate (the timer restarts when the
   condition lapses or the candidate changes).
   `mood_strength = clamp(1 − d_label/strength_radius, 0, 1)`.

**Stimulus** (`stimulus(kind, intensity, confidence, subject, bearing_rad, ttl_s, now)`):

0. Kind `idle`: dropped as `reserved_kind`, nothing else happens (that entry is the idle
   drift rate, not a stimulus).
1. `confidence < min_confidence`: dropped, nothing else happens.
2. Otherwise `idle_s` resets, even if the stimulus is dropped below (it is evidence of
   the world).
3. A subject or a bearing makes it the focus for `ttl_s` (`focus.default_ttl_s` when 0,
   negative or infinite). The focus is a single record replaced by the latest stimulus
   that names a subject or a bearing, so a bearing-only stimulus blanks
   `focus_subject`. This happens even when locked.
4. Unknown kind: dropped (legal, so producers can exist before their tuning). A kind with
   `refractory_s`: dropped if the last *accepted* one is younger than that.
5. Unless locked, `impulse · intensity` (intensity clamped to [0, 1]) is added to each
   axis the kind names, then clamped. The result reports accepted/applied, the reason for
   a drop (`reserved_kind`, `low_confidence`, `unknown_kind`, `refractory`, `locked`),
   the actual delta, and `lock_expired` when the stimulus arrived after `lock_until` but
   before the tick that would have expired the lock (it is then applied).

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
- A timed lock that has run out is expired at the start of the call, even if the request
  is then refused; the caller sees that by comparing `state.locked` before and after.

`reload(config)` swaps the configuration and keeps the state (vector, idle, lock, focus,
refractory history); a label that no longer exists becomes the nearest mood.

## `affect.yaml`

| key | required | default | rule |
|---|---|---|---|
| `temperament` | yes | | every axis, inside its range |
| `relax_tau_s` | yes | | every axis, seconds, > 0 |
| `distance_weights` | no | 1.0 per axis | any subset of axes, > 0 |
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
# from the workspace root on the Mac (runs in the dev container)
make build PKGS=r2_affect
make test PKGS=r2_affect

# inside the dev container those are
colcon build --symlink-install --packages-select r2_affect
source ./setup.bash
colcon test --packages-select r2_affect && colcon test-result --verbose

# or just the unit tests, no build needed
python3 -m pytest src/r2_personality/r2_affect/test -q
```

The model tests drive `now` by hand with short time constants, so nothing sleeps. The
lint tests (flake8, pep257, copyright) need `/opt/ros/jazzy/setup.bash` sourced.

Apache-2.0.
