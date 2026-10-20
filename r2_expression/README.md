# r2_expression

Layer 4 of R2's personality (architecture §6): the arbiters that own the dome's LED and
Magic Panel channels. Everyone who wants the lights to do something (the mood palette,
flourishes, later the behaviour tree, an operator) publishes an `ExpressionIntent`; the
arbiter of that channel decides whether it runs, and it is the only ROS client of the dome
device's LED and Magic Panel services. A person at the dome (its console, its touchscreen,
the text topics) holds the lights on the device, and the arbiter yields.

Phase 1, step 3 delivered the pure-Python core (`docs/personality-phase1-plan.md` in
`r2_dev`); the nodes, renderers, CLI and launch files follow in steps 4-7.

| file | what |
|---|---|
| `r2_expression/core.py` | `Arbiter`: channels, claims, leases, priorities, the device's holds and the state-topic fallback, the watchdog, flourish firing. No ROS imports. |
| `r2_expression/palette.py` | label + strength to a style per channel, `ListLeds`/`ListMagicPanel` pick validation, the flourish scheduler. Pure. |
| `r2_expression/config.py` | `load_config()`: reads and validates `expression.yaml` into frozen dataclasses; every rejection raises `ExpressionConfigError` naming the dotted key. |
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
3. **Claims** (`/expression/claim`): accepted unless another owner holds the channel, or
   an overlapping one, at an equal or higher priority (the refusal names the holder). The
   same owner renews; `lease_s` 0 releases (on the channel the claim is on; any priority);
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
    stopped then if still running.

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
do(arb.tick(now))                                        # a timer, 10 Hz say
for status in arb.statuses(now): publish(status)         # status_hz and on change
for line in arb.take_warnings(): log.warn(line)
```

`do(actions)` hands each `Action` to the single worker: `action.call` names the service
(`run`, `say`, `stop`, `configure`, `start`, `magic_stop`, `magic_brightness`) and
`action.request` holds its request fields, priority and owner included. After the reply,
`arb.device_reply(action, accepted, reason, now)` returns a line to log (or `''`); a hold
refusal makes the channel `FOREIGN` and counts in `refused`, and a refusal on one PSI
leaves the other's run in place. Snapshots and intents may be ROS messages or dicts of
their fields. `arb.reload(config, now)` keeps claims, runs, holds and counts and returns
the re-styling actions.

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
