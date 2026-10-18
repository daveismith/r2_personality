# r2_affect

Layer 2 of R2's personality (architecture §4): a small numeric mood model. Stimuli push
four numbers around, the numbers relax back toward R2's temperament, and the nearest of a
handful of mood prototypes gives the label (`calm`, `curious`, `scared`, ...) that later
layers read. Everything is configured from `config/affect.yaml`.

Phase 0, step 2 delivered the pure-Python core, step 3 the ROS node around it, step 6 the
tuning tools (`docs/personality-phase0-plan.md` in `r2_dev`):

| file | what |
|---|---|
| `r2_affect/model.py` | `AffectModel`: dynamics, impulses, refractory, classification with hysteresis, force/lock, focus, idle. No ROS imports. |
| `r2_affect/config.py` | `load_config()`: reads and validates `affect.yaml` into frozen dataclasses; every rejection raises `AffectConfigError` naming the key. |
| `r2_affect/node.py`, `r2_affect/mood_cli.py` | the `r2_affect` node and the `mood` CLI. |
| `r2_affect/replay.py` | `replay`: runs a recorded session's stimuli through the model offline (see [Tuning loop](#tuning-loop)). |
| `config/affect.yaml` | the shipped configuration (plan §5.4). |
| `foxglove/personality-tuning.json` | the Foxglove tuning layout, installed to `share/r2_affect/foxglove/`. |
| `docs/sessions/` | notes from tuning sessions. |

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

`rate_hz`, `min_confidence` and `max_dt_s` are read once at startup (an integer such as
`-p rate_hz:=10` is fine); `ros2 param set` on them is refused with "read at startup;
restart the node". `/mood/state` keeps its 10 Hz heartbeat when the sim clock is paused
(same stamp, same vector). `duration_s` above one day is refused.

All time is the node clock; a stimulus's `header.stamp` is ignored. The node publishes
nothing but `/mood/state` (and `/rosout`, `/parameter_events`). An invalid config at
startup is logged and the node exits with status 1.

Logs: one INFO line per applied stimulus (kind, source, intensity, the change per axis,
the vector after), INFO on label changes (`mood: calm -> curious (d=0.21)`) and on lock
set, cleared or expired, WARN once per unknown kind, once for the reserved `idle` kind
and once for a clamped time step; a refractory drop is INFO, at most once per kind per refractory window (more are DEBUG); low-confidence drops are DEBUG.

## Setting the mood

`SetMood` fields: `mood` (a label from `affect.yaml`, or empty), `lock`, `duration_s`
(0: until cleared), `source` (who asks; the holder of a lock is recorded as `lock_source`).
A force switches the label at once; a lock held by one source is refused for any other
source, with the holder named in the reason; a clear works from any source.

| | `mood` CLI | Foxglove Service Call panel, service `/mood/set` |
|---|---|---|
| force (dynamics resume) | `ros2 run r2_affect mood set sleepy` | `{"mood": "sleepy", "lock": false, "duration_s": 0, "source": "foxglove"}` |
| force and lock | `ros2 run r2_affect mood set sleepy --lock` | `{"mood": "sleepy", "lock": true, "duration_s": 0, "source": "foxglove"}` |
| lock in place | `ros2 run r2_affect mood lock` | `{"mood": "", "lock": true, "duration_s": 0, "source": "foxglove"}` |
| timed lock | `ros2 run r2_affect mood set excited --lock --duration 30` or `ros2 run r2_affect mood lock --duration 30` | `{"mood": "excited", "lock": true, "duration_s": 30, "source": "foxglove"}` |
| clear | `ros2 run r2_affect mood clear` | `{"mood": "", "lock": false, "duration_s": 0, "source": "foxglove"}` |
| watch | `ros2 run r2_affect mood watch` | add `/mood/state` to a Raw Messages panel |

Every CLI subcommand takes `--source NAME` (default `mood_cli`) and, like any node,
`--ros-args -p use_sim_time:=true`. `set`, `lock` and `clear` print `accepted` or
`refused: <reason>` and the resulting label, and exit 0 when accepted, 1 otherwise.
`ros2 run r2_affect mood watch` prints one line per message: stamp, label (strength), the four axes,
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

## Tuning loop

Tuning means poking R2 (or the sim) with stimuli, watching what the mood does, then trying
other values in `affect.yaml` until the transitions look right. The loop:

1. Open the layout in Foxglove and connect to the sim or the robot.
2. Record a session: `ros2 bag record ... /stimuli /mood/state /rosout` (see
   [Recording](#recording) for the flags).
3. Drive a scenario: the Publish panels, `stimulate`, or the robot itself. The acceptance
   script in plan §8 works as one.
4. Check the model reproduces the recording: `ros2 run r2_affect replay <bag> --compare`.
5. Copy `affect.yaml`, edit it, and preview it on the same session:
   `ros2 run r2_affect replay <bag> --config new.yaml --csv new.csv`. Repeat until the
   transitions read right.
6. Put the values into the live `affect.yaml` and apply them without a restart:
   `ros2 service call /mood/reload_config std_srvs/srv/Trigger` (or the Reload panel).
   Then repeat live.
7. Write down what you changed and why in `docs/sessions/` (see its README).

### The Foxglove layout

`foxglove/personality-tuning.json`, installed to
`install/r2_affect/share/r2_affect/foxglove/personality-tuning.json`. In Foxglove, use
**Layouts → Import from file…** and pick it. Then open a Foxglove WebSocket connection:

| | run | connect to |
|---|---|---|
| sim | `ros2 launch r2_sim sim_foxglove.launch.py` (starts `r2_affect` on sim time) | `ws://localhost:8766` |
| robot | `launch_robot.launch.py` on the robot; the dev container's `foxglove` service is the bridge | `ws://localhost:8765` |

The same layout works for both connections.

| panel | what it does |
|---|---|
| Mood axes (Plot) | `arousal`, `valence`, `curiosity`, `confidence` and `mood_strength` from `/mood/state`, y fixed to −1..1, last 120 s |
| Mood label (State Transitions) | `/mood/state.mood` over time, with `locked` under it |
| Mood state (Raw Messages) | the whole `/mood/state`: `locked`, `lock_source`, `lock_until`, `focus_subject`, `focus_bearing_rad`, `idle_s` |
| Stimulus: bump, face_known (dave), button_sequence (LRR), touched (Publish) | each click publishes one `Stimulus` on `/stimuli` with `source: "foxglove"`, `confidence: 1`, `ttl_s: 0`. You can edit the body (intensity, subject, bearing) first. |
| Set mood (Call Service) | `/mood/set`, preset to force and lock `sleepy` as `foxglove`. Clear it with `{"mood": "", "lock": false, "duration_s": 0, "source": "foxglove"}`; the other bodies are in [Setting the mood](#setting-the-mood). |
| Reload config (Call Service) | `/mood/reload_config` with an empty request; the response says `success` and why not |
| r2_affect log (Log) | `/rosout` with the search term `r2_affect`: one INFO line per stimulus (delta and vector after), label changes, lock set/cleared/expired, reloads, warnings. The term matches the logger name or the text, so another node's line that mentions `r2_affect` also shows. |

**Stimuli without a bearing.** `bearing_rad` should be NaN when unknown, but Foxglove's
Publish and Call Service editors parse with `JSON.parse`, which has no NaN. The bodies of
`bump`, `button_sequence` and `touched` use `1e999` instead. `JSON.parse` reads that as
Infinity and sends it as float +inf. `r2_affect` treats any non-finite bearing as no
bearing, just like NaN, so the focus doesn't change. Don't use `0` here: it is a real
bearing (straight ahead) and it would move the focus. This was checked against
`foxglove_bridge` 3.6 with a CDR client publish, which Foxglove uses when the bridge offers
`cdr` first, as this one does. If a client published the same body as JSON, Infinity would
become `null`, and the bridge drops that message. Use `1e999` only in these hand-driven
panels. The adapters publish NaN.

### Recording

```bash
# robot (wall clock everywhere)
ros2 bag record --include-unpublished-topics -o <name> /stimuli /mood/state /rosout
# sim: also --use-sim-time
ros2 bag record --use-sim-time --include-unpublished-topics -o <name> /stimuli /mood/state /rosout
```

Wait for `All requested topics are subscribed` before the first stimulus. Ctrl-C stops the
recording and closes the bag.

- `--include-unpublished-topics`: without it, the recorder subscribes to `/stimuli` only
  after it sees a publisher. `stimulate` publishes one message and exits, so the recorder
  misses its first stimulus. In the dry run, a bump went missing this way and `--compare`
  flagged it as a 0.6 arousal mismatch.
- `--use-sim-time` (sim only): the bag's receive timestamps then come from `/clock`, the
  clock `r2_affect` runs on. Pausing Gazebo while recording pauses the bag's clock too, so
  the replay sees the same time the node did. Without the flag the stamps are wall time,
  and the replay counts a pause as time passing (relaxing toward the temperament) when the
  node was frozen. Even without a pause, Gazebo's real-time factor (about 0.96 in the
  container) makes each replayed `dt` about 4 % longer than the node's. In the dry run, a
  26 s session recorded on wall time compared at 0.013, and one recorded with
  `--use-sim-time` at 0.00006. On the robot the node and the recorder both use wall time,
  so no flag is needed.

### Replaying

```bash
ros2 run r2_affect replay <bag> --compare                          # does the model reproduce the run?
ros2 run r2_affect replay <bag> --config new.yaml --csv new.csv    # what would new values have done?
```

| option | default | meaning |
|---|---|---|
| `bag` | | the bag directory (mcap or sqlite3) |
| `--config` | installed `share/r2_affect/config/affect.yaml` | the YAML to replay with |
| `--csv OUT` | none | one row per tick: `t_rel`, the four axes, `mood`, `mood_strength`, `idle_s`, `locked` (`-` for stdout) |
| `--compare` | off | check against the recorded `/mood/state`; exit 1 if any axis is off by 0.02 or more |
| `--rate-hz` | 10 | tick rate (the node's `rate_hz`) |
| `--max-dt` | 0.5 | the node's `max_dt_s` |
| `--start` | `recorded` | `recorded` starts from the first `/mood/state` in the bag; `temperament` starts from the config's temperament |

The replay works like the node, on the bag's *receive* timestamps (never
`header.stamp`). It ticks every `1/rate_hz`, and applies each stimulus at its receive time,
in order, before the next tick. It prints every stimulus with what it did (the delta per
axis, or why it was dropped), the label transitions, and the end state.

**Where it starts.** The node has usually been running before the recording starts, so by
default the model starts from the first recorded `/mood/state`: the vector, the label, the
idle time, and the lock if there is one. The tick grid starts at that message, in phase
with the node's ticks. A state message can't carry the refractory history, a label change
already in progress, or the focus ttl, so those start empty. A bump less than 2 s before
the recording, or a label in mid-dwell, can therefore differ for a moment. With no
`/mood/state` in the bag, or with `--start temperament`, the model starts at the
temperament and the grid starts at the first message.

**`--compare`** pairs each recorded state with the nearest replayed tick that has seen the
same stimuli. Without that rule, a stimulus landing between a node tick and a replay tick
would show up as a one-tick spike. It prints the largest difference per axis and when it
happened, the number of ticks where the labels disagree, and both lists of transitions.
Near a transition the node and the replay can switch one tick apart, so a few label
disagreements are normal. With the YAML that was running, the difference should be about 0:
the node and the replay run the same code on the same stimuli. A difference of 0.02 or
more fails, and the output names the axis. Common causes are a stimulus missing from the
bag, a different YAML, a wall-clock recording of the sim, or a `SetMood` call.
`/mood/set` calls (force, lock, clear) are not in a bag, so they are not replayed. When a
recorded state is locked, the tool says so, because the comparison means nothing across a
lock. A force without a lock shows up as a mismatch from that moment on.

### Applying the values live

The node re-reads the file at its `config` parameter. With `--symlink-install` that file is
`config/affect.yaml` in this package, so edit it in place (or put a copy anywhere and launch
with `affect_config:=` / `config:=`). Then:

```bash
ros2 service call /mood/reload_config std_srvs/srv/Trigger
```

or use the Reload config panel. The reload keeps the state, and a broken file is refused
with the reason (see [Reloading the config](#reloading-the-config)). Don't commit a tuned
`affect.yaml` without a note in `docs/sessions/`.

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

The node tests start their own node with a unique node name and remapped topics and
services, so they are isolated by remapping and run alongside a live stack (the sim or the
robot with its own `/r2_affect`) on the default domain.

The model tests drive `now` by hand with short time constants, so nothing sleeps. The
lint tests (flake8, pep257, copyright) need `/opt/ros/jazzy/setup.bash` sourced.

Apache-2.0.
