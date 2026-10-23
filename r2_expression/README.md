# r2_expression

Layer 4 of R2's personality (architecture §6): the arbiters that own the dome's LED and
Magic Panel channels. Everyone who wants the lights to do something (the mood palette,
flourishes, later the behaviour tree, an operator) publishes an `ExpressionIntent`; the
arbiter of that channel decides whether it runs, and it is the only ROS client of the dome
device's LED and Magic Panel services. A person at the dome (its console, its touchscreen,
the text topics) holds the lights on the device, and the arbiter yields.

Phase 1, step 3 delivered the pure-Python core and step 4 the renderers and the two arbiter
nodes (`docs/personality-phase1-plan.md` in `r2_dev`); the CLI, panels and bringup follow in
steps 6-7.

| file | what |
|---|---|
| `r2_expression/core.py` | `Arbiter`: channels, claims, leases, priorities, the device's holds and the state-topic fallback, the watchdog, flourish firing. No ROS imports. |
| `r2_expression/palette.py` | label + strength to a style per channel, `ListLeds`/`ListMagicPanel` pick validation, the flourish scheduler. Pure. |
| `r2_expression/config.py` | `load_config()`: reads and validates `expression.yaml` into frozen dataclasses; every rejection raises `ExpressionConfigError` naming the dotted key. |
| `r2_expression/led_renderer.py`, `magic_renderer.py` | a core `Action` to the exact service request, or a local refusal with the device's own string. Pure (message types only). `render_common.py` holds what they share. |
| `r2_expression/motion.py`, `dome_renderer.py` | the dome's motion model (moves, profiles, the coast stop, `CoastMemory`; pure), and a `CALL_VELOCITY` action to the `TwistStamped` for the converter. |
| `r2_expression/arbiter_node.py` | the nodes: a shared base, `DeviceArbiterNode` (entry points `led_arbiter`, `magic_arbiter`) and `DomeArbiterNode` (`dome_arbiter`). |
| `launch/expression.launch.py` | `use_sim_time`, `config`, `device`, `dome` (true) and the dome's four topics; starts the three nodes. |
| `config/expression.yaml` | the shipped settings and a palette for every mood of `r2_affect`'s `affect.yaml`. |

## The rules in a page

Channels: `fld` (surface `fld`, both front boards), its sub-channels `tfld` and `bfld` (one
board each), `rld`, `psi` (surfaces `fpsi` and `rpsi` as one channel) and `magic`. Channels
that share a display overlap (`fld` with `tfld` and `bfld`); claims and runs act across
overlapping channels, because the device replaces any run sharing a display.

1. **Every device request carries `owner: r2_expression` and a priority 1-255, never 0**
   (0 is the legacy value the device never refuses: it would override a person). An
   `Action` built with priority 0 raises. **Nothing ever carries hue**: hue is the droid's
   colour scheme, not a mood knob.
2. **Ladder.** `safety` <= 255, `manual` <= 200, `behavior` <= 100, `idle` <= 10. A source
   above its slot, priority 0, an unknown source or action: dropped and counted.
3. **Claims** (`/expression/claim/led`, `/expression/claim/magic`): accepted unless another owner holds the channel, or
   an overlapping one, at an equal or higher priority (the refusal names the holder). The
   same owner renews; `lease_s` 0 releases (on the channel the claim is on; any priority; an
   owner with no claim there is refused `no claim by '<owner>' on <channel>`);
   leases are capped at `max_lease_s`. Claims
   layer: a lower claim holds again when a higher one goes. Owner `manual` is reserved for
   the auto-claim of rule 4. An accepted claim stops our
   lower runs on the channel at once; lower intents are dropped (and counted) until release.
4. **Manual auto-claim.** A `manual` intent claims its channel (owner `manual`, at its own
   priority, never lowered by a renewal) for `manual_hold_s` after the later of the intent
   and the end of its run. A manual intent passes that claim, never another owner's higher
   one.
5. **Within a level** the newer intent wins, unless the running one is
   `interruptible: false`. A higher-priority running intent blocks lower runs and stops. A
   run the device refuses outright leaves the runs it would have replaced tracked.
6. **Duration cap.** `idle` and `behavior` runs are sent with `min(duration,
   max_duration_ms)`, and `max_duration_ms` instead of 0: a dead arbiter or personality
   leaves nothing running for more than 30 s. `manual` and `safety` are exempt. LED runs end
   on the device; a Magic Panel run is stopped by the arbiter at its duration, once no
   person holds the panel (a refused stop is retried after the device's 5 s hold).
7. **A person at the dome is the device's hold.** A `held by console|ui|text topic` or
   `held: manual` refusal, or `held_by` on the state topic, makes the channel `FOREIGN`
   with the holder as owner and its remaining time as lease (`manual`: forever, -1). A run
   we did not start keeps it `FOREIGN` after the hold until the surface is idle. Idle and
   behaviour intents are dropped while foreign, dark or offline (an operator's are sent: the
   device refuses 1-199 and accepts 200); no stop is ever sent to a surface running
   someone else's run. A hold ends when its time runs out; a refusal's hold covers every
   display the request touched, so overlapping channels see it too.
   `ConfigureMagicPanel` (which takes no priority) is only sent while the panel's
   `held_by` is empty; a panel run `started_by: other` (rotary switch, another master) is
   `FOREIGN` owner `other`.
8. **Palette.** Each palette channel (`fld`, `rld`, `psi`, `magic`) is re-styled at
   priority 10 when the label changes, when the mood goes stale or returns, on reload, and
   whenever the channel becomes free again (a release, the end of a hold, the strip back
   on, the device back, the end of a run of ours above priority 10, whose look brightness
   would otherwise change). Nothing below a running intent's priority touches its channel,
   configure included. Idle intents and flourishes step off a claim at any priority. A display a person left `off` or solid is put back to random.
9. **Watchdog.** `/mood/state` older than `mood_stale_s`: neutral palette, no flourishes,
   `STALE`. No state message for `device_stale_s` (or no catalogue yet, or no Magic Panel
   link): `OFFLINE`. `strip_enabled` false: `DARK`.
10. **Flourishes** fire within the mood's `every_s` as idle intents, so rule 2-7 apply: a
    foreign, dark or offline channel drops and counts them, and so does a claim at any
    priority; none fire while stale. Each arbiter draws from the mood's whole set and plays only its own picks, so
    the two together keep the mood's rate and weights.
11. **Own runs survive a restart.** A run the state topic shows with `source: ros, owner:
    r2_expression` on the first message is adopted with `max_duration_ms` from then, and
    stopped then if still running. The device cannot tell an operator's unbounded manual run
    (`express ... --duration 0`, sent through the arbiter, so `owner r2_expression`) from the
    arbiter's own: after an arbiter restart it is adopted too, becomes an idle-priority run and
    is stopped at `max_duration_ms`.
12. **A sequence ends blank, and nothing unbounded is personality.** A Magic Panel sequence the
    arbiter starts carries `end: blank` unless the intent says `end: leave` (a flourish is
    never left lit). An `idle` or `behavior` intent with `repeat` 0, or naming a sequence the
    catalogue flags as looping (`FLAG_LOOPS`: the random shows), is dropped as `unbounded
    sequence`; a `manual` one is allowed, and stopped at its duration if it has one. A pick in
    `expression.yaml` may not say `repeat: 0`.

Status precedence: `OFFLINE`, `DARK`, `FOREIGN`, `RUNNING`, `CLAIMED`, `STALE`, `IDLE`.

## `expression.yaml`

```yaml
defaults:                  # all optional; these are the defaults
  manual_hold_s: 5.0
  mood_stale_s: 3.0
  max_duration_ms: 30000   # integer
  status_hz: 2.0
  max_lease_s: 60.0
  device_stale_s: 5.0
neutral:                   # required: stale mood, and any mood without an entry
  fld: {drift: calm, brightness: 40}     # a single number here; no flourish
  rld: {drift: calm, brightness: 40}
  psi: {brightness: 40}                  # magic left out: the panel is untouched
moods:                     # required, one entry per affect.yaml mood
  excited:
    fld:   {drift: lively, brightness: [80, 100]}   # drift calm|lively; brightness 0-100
    rld:   {drift: lively, brightness: [80, 100]}   # or [min, max] scaled by mood_strength
    psi:   {brightness: [80, 100]}                  # no drift on the PSIs
    magic: {brightness: 12}                         # 0-15
    flourish:
      every_s: [6, 15]                              # 0 < min <= max
      picks:                                        # weighted; exactly one of effect/clip/sequence
        - {channel: fld, effect: rainbow, weight: 3, duration_ms: 4000}
        - {channel: psi, clip: flash, rgb: 0xff2200, speed: 5, duration_ms: 1200}
        - {channel: magic, sequence: 27, repeat: 1, end: blank, duration_ms: 3000}
```

A channel a mood leaves out takes neutral's style. Pick channels are `fld`, `tfld`, `bfld`,
`rld`, `psi` (effects and clips; no clips on `tfld`/`bfld`) and `magic` (a sequence id or
catalogue name). `duration_ms` is >= 1 and capped at `max_duration_ms`; absent, the cap is
used. Rejected: a `hue` key anywhere, unknown keys or channels, values out of range, drift
on the PSIs or the panel, a pick with no kind or two, `every_s` min above max, a pick with
`duration_ms` 0. Effect, clip and sequence names are checked against the device's
`ListLeds`/`ListMagicPanel` at startup: a pick the device lacks is dropped with one WARN,
so a palette can name clips only some droids carry.

## Palettes and flourishes

`expression.yaml` gives every mood of `affect.yaml` a palette (`moods.<label>`); `neutral`
is used while `/mood/state` is stale and for a label with no entry.

| key | what it does |
|---|---|
| `fld`, `rld`: `drift` | `calm` or `lively` (the device's two presets; about 5 times the activity). No hue: that is the droid's colour scheme. |
| `fld`, `rld`, `psi`: `brightness` | percent; `[min, max]` is placed by `mood_strength` (0 gives min, 1 gives max); a number is fixed. The PSIs have no drift. |
| `magic`: `brightness` | 0-15, sent with `ConfigureMagicPanel` only while the panel is not held by a person. Left out: the panel is untouched. |
| `flourish.every_s` | `[min, max]` seconds from one flourish to the next, drawn at random. |
| `flourish.picks` | weighted; each is a run on one channel: `effect`, `clip` or `sequence`, optional `rgb`, `speed`, `repeat`, `end`, and `duration_ms` (capped at `max_duration_ms`, never 0). |

The palette is applied at priority 10 when the label changes (not when only the strength
moves), when the mood goes stale or returns, and when a channel becomes free again; it is
sent once per change, not every tick. A flourish is an idle intent, so a claim at any
priority, a person's hold, a dark strip, an offline device or a stale mood drops it.

**Floors** (plan 2.6, measured on the strip): the logics are invisible below about 25 %, so
`fld` and `rld` never go under 25 in a mood (neutral is 40); the PSIs stay visible and floor
at 20. Sleepy sits at the floors and sulking just above them, both calm; excited and playful are lively and
bright; annoyed and scared use `alarm`, `flash`, `redalert` and Magic Panel Alert (6) and
Alert long (7); curious uses `roam`, `cycle` and the scans; calm has an occasional `pulse`.
Flourish rates run from every 6-15 s (excited) to every 120-300 s (sleepy).

**Validation.** At startup each arbiter calls `list` on the device and checks every pick
against it: an effect or clip name the device lacks (clips are per target: `fld`, `psi`,
`rld`), or a Magic Panel sequence missing from its catalogue, is dropped with one WARN, and
the other picks keep their weights. A palette can therefore name clips only some droids
carry. The shipped file is checked against the workbench captures in `docs/dome/` by a test.
A palette configure the device does not answer is sent again after `device_stale_s`.

**Previewing with the mock** (the real `/r2_dome` may be on the graph: use private names):

```bash
ros2 run r2_mocks dome_mock --ros-args -r __node:=dome_mock_manual -p state_hz:=10.0
ros2 launch r2_expression expression.launch.py device:=/dome_mock_manual
ros2 topic pub -r 5 /mood/state astromech_msgs/msg/MoodState "{mood: excited, mood_strength: 1.0}"
ros2 topic echo /dome_mock_manual/mock/picture     # drift and brightness per display
ros2 topic echo /expression/status                 # the running flourish, per channel
```

A stale mood returns fld, rld and psi to neutral; the Magic Panel (no `magic` in neutral) keeps
the last mood's brightness. `redalert` follows the audio level, so check it on the lights. Sequence
durations are the catalogue's lengths (a panel run is blanked at `duration_ms`).

Without `r2_affect` running you are the only `/mood/state` publisher; with it running, remap
the arbiters and your publisher to private names as the tests do.

## Running the nodes

```bash
ros2 launch r2_expression expression.launch.py                       # against /r2_dome
ros2 launch r2_expression expression.launch.py device:=/dome_mock use_sim_time:=true
ros2 run r2_expression led_arbiter --ros-args -p device:=/dome_mock   # one node alone
ros2 run r2_expression magic_arbiter --ros-args -p config:=/path/expression.yaml
```

With a real dome on the graph, test against a mock under another name (`ros2 launch r2_mocks
dome_mock.launch.py node_name:=dome_mock`) and pass that name as `device`; remap the
`/expression/*` and `/mood/state` names too if a live stack is running.

**One arbiter pair per graph.** At startup an arbiter looks for another node of its own name
(`/led_arbiter`, `/magic_arbiter`) and, if there is one, logs an ERROR naming it and exits 1:
two pairs would fight over one device with the same owner (a Mac container beside the robot).
`-p allow_duplicate:=true` skips the check (the tests start two on purpose).

Parameters, all read at startup (a later `ros2 param set` is refused, as in `r2_affect`):

| parameter | default | |
|---|---|---|
| `config` | the installed `expression.yaml` | settings and palettes |
| `device` | `/r2_dome` | the dome device node; services are `<device>/leds/...` or `<device>/magic/...` |
| `allow_duplicate` | false | start although another node of this name is on the graph |
| `use_sim_time` | false | node clock follows `/clock`; the 4 Hz tick runs on the steady clock |

| | `led_arbiter` | `magic_arbiter` |
|---|---|---|
| channels | `fld`, `tfld`, `bfld`, `rld`, `psi` | `magic` |
| subscribes | `/expression/intent` (reliable, depth 50), `/mood/state`, `<device>/leds/states` | same intent and mood, `<device>/magic/states` (reliable, depth 10) |
| serves | `/expression/claim/led` | `/expression/claim/magic` |
| clients | `<device>/leds/list`, `run`, `say`, `stop`, `configure` | `<device>/magic/list`, `start`, `stop`, `configure` |
| publishes | `/expression/status` (`ChannelStatus`) | same |

Nothing else is published but `/rosout` and `/parameter_events`; no other client of the device
exists (a test checks both by graph introspection).

### The claim service

The plan names one service, `/expression/claim`, offered by both nodes. **That does not work
on rmw_zenoh**: with two servers on one name a request goes to either of them (measured: about
half the requests for `magic` reached the LED node, which answered `not my channel`). So each
node offers its own: **`/expression/claim/led`** and **`/expression/claim/magic`**. A node still
refuses the other's channels with `not my channel` and `accepted false`, so a client that tries
both is correct. The `express` CLI and the Foxglove panels pick the service by channel.

### Status semantics

`/expression/status` carries one `ChannelStatus` per channel at `status_hz` and at once on a
change of state, owner, priority, active intent, mood, or the `dropped`/`refused` counts (not
for the lease and remaining-time countdowns). The LED node publishes `fld`, `rld` and `psi`
always and `tfld`/`bfld` only while they are not `IDLE` (plus the one message that returns them
to `IDLE`). States and their order of precedence are in the rules above; `OFFLINE` is shown from
the start until the device has answered `list` and sent a state message, and again when no state
message arrives for `device_stale_s`.

### Startup and the single worker

1. The worker waits for `<device>/.../list` to appear on the graph (polled, nothing sent), calls
   it, and retries every 5 s while a call gets no answer. Meanwhile every channel is `OFFLINE`.
2. The catalogue is built (`palette.led_catalogue` / `magic_catalogue`) and given to the core.
   State messages before that are ignored; the first one after it adopts this arbiter's own
   runs (`source ros`, `owner r2_expression`: stopped, plan 4.2), then the current mood's
   palette goes out. The Magic Panel node lists again when `interrogations` moves or the link
   comes back.

All device calls are made by **one worker thread**, from a queue, in the order the core produced
them: one request in flight per device, a 5 s timeout, and a timed-out call is reported to the
core as refused with reason `timeout`. A palette request that fails like this (a timeout, the device offline, a stale drop; not a
hold, not a refusal) is sent again after `device_stale_s`. The device queue is bounded by age: an `idle` or `behavior`
action that waited longer than `device_stale_s` when the worker reaches it (the device was
stalled or slow) is dropped, reported to the core as refused `stale` and logged as
one WARN per burst plus one INFO with the count; `manual` and `safety` actions (priority above
100) and stops are always sent. A run, say or start also replaces one still queued for the same
channel and surface (reported as refused `superseded`, quiet): a device that wedges and
recovers replays at most one run per target. Stops and configures are never replaced. An
exception in a callback (intent, mood, state, claim, tick) or in the worker is logged once a
minute per callback and the node carries on. The executor thread (subscriptions, the claim service,
the tick) never waits on the device; the core is guarded by one lock. Before a request is sent
the renderer checks what it can: text on a PSI, more than two lines, a line of 64 bytes (UTF-8, as the
device's `strlen` counts) or more, a name over 63 bytes (`name too long`: the device never answers it), an owner over 31
bytes; a refusal is local, logged, and reported to the core as the device's refusal would be.

### Logging

INFO: startup, each state change of a channel, each accepted intent and claim, the mood label,
each device refusal that is new (`held by ...` once per hold, not on each repeat), the device
answering `list`. WARNING: each core warning once (unknown mood label, a pick the device lacks),
a request that got no answer, a local refusal, a device refusal other than a hold. DEBUG:
dropped intents, each device call (`fld: run on fld (intent)`), repeats of a hold refusal.
`--ros-args --log-level debug` shows them.

## The dome channel

`dome_arbiter` serves channel `dome` (phase 2 plan 4.3, 5.4). It is the same base node as the
other two (claims, leases, priorities, the manual auto-claim, status, the namesake refusal,
the guarded callbacks, startup parameters) with a different device: **the device is
`dome_twist_converter`** (`r2_bringup`), and the arbiter reaches it only by publishing
`geometry_msgs/TwistStamped` (`twist.angular.z` in rad/s, positive = positive joint) on
`command_topic`. It calls no service and has no catalogue and no worker thread. The converter
is the authority: it ranks the joystick, then Foxglove, then the personality, caps the
personality's speed and acceleration, and stops the dome `personality_timeout_s` (0.5 s) after
the arbiter falls silent, so a dead arbiter cannot leave the dome turning. Nothing here
publishes to `/dome_controller/commands`, `/cmd_vel*` or `/dome/joint_command`, and the
position interface, `~/setup` and the bench services are never touched.

| parameter (startup) | default | |
|---|---|---|
| `config`, `use_sim_time`, `allow_duplicate` | as above | |
| `command_topic` | `/dome/cmd/personality` | the converter's personality input (published, reliable, depth 1) |
| `status_topic` | `/dome/cmd/status` | the converter's `DomeCommandStatus` (reliable, depth 1) |
| `joint_states_topic` | `/dome_state_broadcaster/joint_states` | `sensor_msgs/JointState` (best effort, depth 1) |
| `odom_topic` | `/diff_cont/odom` | `nav_msgs/Odometry` (best effort, depth 1, one message in 3 read) |
| `joint_name` | `dome_joint` | whose `position` is the dome's |

It also takes `/expression/intent`, `/mood/state`, serves **`/expression/claim/dome`** and
publishes `/expression/status`, as the others. The verbs (`look_at`, `glance`, `turn`, `rest`,
`stop`; `track` is refused `not in this phase`) and the motion settings are in
`expression.yaml` (`defaults.dome`, `profiles`, each mood's `dome`).

What the states mean here, beyond the phase 1 ones:

- **`foreign`** (owner `joystick` or `foxglove`, lease 0): the converter's status says a
  person drives the dome. A move in progress is abandoned at once with a 0 (a 0 on the lowest
  input overrides nobody) and is not resumed; the channel is `idle` again when the source is
  `idle` or `personality`, the mood's rest (if any) is re-applied and glances wait
  `after_foreign_s`. There is no attribution on a Twist: the converter's status is the dome's
  `held_by`.
- **`suppressed`** (`STATE_SUPPRESSED`, 7): the body drives faster than `body_speed_m_s` or
  yaws faster than `body_yaw_rad_s`. The move in progress is stopped with a 0, `idle` and
  `behavior` intents are dropped and counted, `manual` and `safety` still run; it clears
  `after_still_s` after the odometry says the body is still.
- **`offline`**: no joint state or no converter status for `device_stale_s` (or never).
  Nothing is sent; intents are dropped. At start the arbiter sends nothing until it has both.
- **`stale`**: the mood went quiet; an `idle` or `behavior` move stops, no glances, `manual`
  still runs.

**Threads, timers and CPU** (rule 11: 1.5 % of a Pi 5 core idle, 4 % in a move). One thread:
every subscription, the claim service and both timers run on the executor, under the lock.
Every velocity the core returns is published at once from the callback that produced it, so a
stop (foreign, suppressed, a claim, a stop intent) leaves the node in the same callback. A
move runs on a **20 Hz timer** (the converter's rate) that the first move action creates and
the end of the move destroys; while it exists it ticks the core and the 4 Hz tick skips its
turn. Idle, the node wakes for the joint state (10 Hz), the converter's status (2 Hz), the
mood (5 Hz), the odometry (one in three read, all received) and the 4 Hz tick, nothing else.
On SIGINT or SIGTERM a node in a move publishes one 0 before it exits and waits up to 0.2 s
for it to be acknowledged (it installs its own signal handlers so the context is still up for
it, and ignores a repeated signal while it closes); a SIGKILL is covered by the converter's
timeout. Measured in the dev container (`test_dome_node.py`, logged not asserted): idle
3.8 % of a container core, against 3.6 % for `led_arbiter` measured the same way (the two
phase 1 arbiters together are 2.74 % on the Pi), so roughly 1.4 % on the Pi: inside the
budget, but an estimate until step 8b measures it there.

**The coast is learned and carried.** Each stop shows how far the dome really ran on after
its 0, and the move scales the coast fit by observed over predicted (motion rule 5). The
arbiter keeps that scale for the next move (`motion.CoastMemory`, `Arbiter.coast_scale`), so
on Gazebo's ideal plant only the first move after a start stops short and creeps; on the real
dome the scale stays near 1. The carried scale fades toward 1.0 with a 10-minute half-life
(the coast drifts with the battery and the temperature, and one odd stop, a hand on the dome
while it settled, should not outlive a quiet spell); a move that never stopped, or that was
stopped (a person, a claim, suppression), teaches nothing. `dome: move over, coast scale X for the next` is logged at INFO after every move.

```bash
ros2 run r2_expression dome_arbiter                              # against the converter
ros2 run r2_expression dome_arbiter --ros-args -p joint_states_topic:=/dome_drive_mock/joint_states
```

## How the nodes use the core (step 4)

```python
from r2_expression.config import load_config
from r2_expression.core import Arbiter, Intent, LED_CHANNELS
from r2_expression.palette import led_catalogue

arb = Arbiter(load_config(path), LED_CHANNELS)          # magic node: ('magic',)
do(arb.set_catalogue(led_catalogue(list_response), now))  # after ListLeds answers
do(arb.led_state(state_msg, now))                        # every /r2_dome/leds/states
do(arb.mood(msg.mood, msg.mood_strength, now))           # every /mood/state
result = arb.intent(intent_msg, now); do(result.actions) # every /expression/intent
claim = arb.claim(req.channel, req.owner, req.priority, req.lease_s, now)
do(claim.actions)                                        # answer with claim.accepted/reason/status
do(arb.tick(now))                                        # a timer, 4 Hz in the node
for status in arb.statuses(now): publish(status)         # status_hz and on change
for line in arb.take_warnings(): log.warn(line)
```

The dome node does the same with `arb.dome_state({'position_rad': p}, now)` (each joint
state), `{'source': status.source}` (each converter status) and `{'body_speed_m_s': v,
'body_yaw_rad_s': w}` (odometry), and ticks at 20 Hz while `arb.moving`; its actions are all
`CALL_VELOCITY` with `{velocity_rad_s}`, published at once (`dome_renderer.render`).

`do(actions)` hands each `Action` to the single worker: `action.call` names the service
(`run`, `say`, `stop`, `configure`, `start`, `magic_stop`, `magic_brightness`) and
`action.request` holds its request fields, priority and owner included. After the reply,
`arb.device_reply(action, accepted, reason, now)` returns a line to log (or `''`); a hold
refusal makes the channel `FOREIGN` and counts in `refused`, and a refusal on one PSI
leaves the other's run in place. Snapshots and intents may be ROS messages or dicts of
their fields. `arb.reload(config, now)` keeps claims, runs, holds and counts and returns
the re-styling actions.

## Driving the lights by hand

Two ways, same messages: the `express` CLI and the Foxglove tab
`foxglove/expression-panels.json` (installed under `share/r2_expression/foxglove`).

| | CLI | Foxglove panel |
|---|---|---|
| run an effect | `express effect fld fire --duration 5` | Publish "fld fire" (`/expression/intent`) |
| Magic Panel sequence | `express sequence magic 44 --duration 3` | Publish "magic sequence 44" |
| drift | `express configure rld --drift lively` | Publish "rld drift lively" |
| stop | `express stop fld` | Publish "stop fld" |
| hold a channel | `express claim fld --owner me --lease 30` / `express release fld` (`magic` for the panel) | Call Service `/expression/claim/led` and `/expression/claim/magic` (a claim and a release panel each) |
| why is it quiet | `express status [--watch]` | State Transitions on `/expression/status{channel=="fld"}.state` |
| see the lights (sim) | `express dome "leds run fld fire"` | Raw Messages `/dome_mock/mock/picture`, Image `/dome_mock/mock/image` |
| turn the dome | `express look 0.8 [--profile snappy]`, `express turn -1.57`, `express rest [0.35]` | Publish "look_at 0.8 rad" (edit the bearing) |
| glance | `express glance [--amplitude 0.5] [--dwell 1.0] [--side left]` | |
| stop the dome | `express stop dome` | Publish "stop dome"; Call Service `/expression/claim/dome` |

`configure --drift` works on `fld`, `tfld`, `bfld` and `rld` (the PSIs have no drift).
Also `clip`, `text "line1" "line2" [--motion loop] [--on-black]`, `configure --brightness N`
(0-100; magic 0-15), `--rgb rrggbb`, `--speed 1-9`, `--hold` (not interruptible), `--id`.
Exit codes: 0 done (claim accepted, intent sent to a subscriber), 1 refused or nothing
answering, 2 bad arguments. An intent is fire-and-forget: one an arbiter drops (a higher
priority holds the channel) shows only as `dropped` in `express status`. `claim` and
`release` call `/expression/claim/led` for `fld`, `tfld`, `bfld`, `rld` and `psi`, and
`/expression/claim/magic` for `magic`; `--claim-service NAME` overrides the choice. A `release`
by an owner that holds nothing on the channel exits 1 and prints the refusal with the status
line, which shows the real holder. `sequence` defaults to `--repeat 1 --end blank`.

**The dome verbs** (`look`, `glance`, `turn`, `rest`, and `stop dome`) send `look_at`, `glance`,
`turn`, `rest` and `stop` intents on channel `dome`. Bearings are radians, positive left, 0
forward, within (-pi, pi]; `turn` takes a change within -2pi to 2pi; `--dwell` is more than 0
seconds; `--amplitude` is in (0, pi]; `--profile` is a bare profile name (default: the mood's).
`claim dome` calls `/expression/claim/dome`. `express status` shows the dome's `suppressed` state
(the body is moving). The Foxglove tab is `foxglove/dome-panels.json`: a `look_at` Publish panel, claim
and release, the dome's state, the joint position, and `/dome/cmd/status.source`.

```bash
express look 0.8 --profile snappy     # 0.8 rad to the left
express glance --side right --dwell 1.5
express stop dome
```

**199 and `--override`.** Every intent goes out as `source: manual` at priority 199, which
yields to a person at the dome (their hold on the device is 200 or the device's own).
`--override` sends 200, which beats that hold and is never refused; the Foxglove panels
send 199 and have no override. `--source idle|behavior` (10 / 100, or `--priority N`) is for
testing the arbiter's ladder only.

**The dome's own console is not routed.** `express dome` and the mock's console topic exist
only in the sim, where the mock is `/dome_mock` and `express dome` sends to it by default
(`--device /r2_dome` for a mock running under the device's own name); on the robot, a line typed at the dome's console (or the touchscreen, or
the panel's rotary switch) holds those lights directly, and the arbiters show the channel as
`foreign` until it ends. The real device has no `mock/console` topic, so `express dome` there prints the "nobody subscribes" note and exits 1.

## Tests

```bash
make test PKGS=r2_expression                     # from the Mac (dev container)
# inside the dev container, from /ws:
colcon build --symlink-install --packages-select r2_expression
colcon test --packages-select r2_expression && colcon test-result --verbose
# or, as the core needs no ROS, straight from the package directory:
cd src/r2_personality/r2_expression && python3 -m pytest -q
```

The tests are named for the rules above; `test/fixtures.py` holds the test config, the
catalogues and snapshot builders, and the shipped YAML is checked against `affect.yaml` and
the workbench captures in `docs/dome/` (skipped when those are not beside the package).
