# r2_stimulus

Layer 1 of R2's personality stack. Producers here *describe* what happened ("a bump of
intensity 0.7 from the left") and never decide what the droid feels; the `r2_affect` node
maps each `kind` to an effect through its `affect.yaml`.

## `stimulate`

```bash
ros2 run r2_stimulus stimulate <kind> [--intensity F] [--confidence F] [--subject S] \
    [--bearing RAD] [--ttl S] [--extra JSON] [--repeat N] [--every S] [--source NAME]
```

Defaults: intensity 1.0, confidence 1.0, subject empty, bearing unknown, ttl 0 (the affect
node's default), repeat 1, every 0.5 s, source `stimulate_cli`. One line is printed per
message sent. The tool waits up to 2 s for a subscriber on `/stimuli`; if there is none it
still publishes once and warns on stderr. `--ros-args -p use_sim_time:=true` works.

```bash
ros2 run r2_stimulus stimulate bump --intensity 0.8
ros2 run r2_stimulus stimulate face_known --subject dave --bearing 0.8
ros2 run r2_stimulus stimulate button_sequence --subject LRR --extra '{"taps":"LRR"}'
ros2 run r2_stimulus stimulate bump --repeat 3 --every 0.2
```

Notes:

- Omitting `--bearing` means *unknown* (NaN). `--bearing 0` is a real direction, straight
  ahead, and sets the affect node's focus.
- `kind` is not validated against `affect.yaml`: `stimulate nonsense` is a legitimate test
  and makes the affect node log one WARN and ignore it.
- Exit code 2 for an empty or whitespace-containing kind, intensity or confidence outside
  [0, 1], `--repeat` below 1, `--every` below 0, or `--extra` that is not a JSON object.

## Adapters

Three nodes turn existing signals into stimuli. Started together, in one process
(`stimulus_adapters`, the three nodes keep their names), by
`ros2 launch r2_stimulus adapters.launch.py [use_sim_time:=true] [config:=PATH] [separate:=true]` (or by
`r2_bringup`'s `personality.launch.py`, which passes `use_sim_time` and its
`stimulus_config` as `config`). Thresholds are ROS
parameters in `config/stimulus.yaml`, one section per node; the pure logic is in
`r2_stimulus/detect.py` (no ROS imports, time passed in).

**Rule: the adapters only listen to drive topics.** Their one publisher (besides
`/rosout` and `/parameter_events`) is `/stimuli`. They never publish to `/cmd_vel*`,
`/diff_cont/*` or `/dome_controller/commands`; the drive is never driven by the
personality stack. `test/test_adapters_pub.py` checks this on the running nodes.

Every message has `source` = the node name, `confidence` 1.0, `bearing_rad` NaN, `ttl_s` 0
(affect default), `header.stamp` from the node clock (sim time with `use_sim_time`). One
INFO line is logged per stimulus.

| node | listens to | emits |
|---|---|---|
| `drive_stimulus` | `/diff_cont/odom` (`nav_msgs/Odometry`, planar speed of twist linear x,y) | `being_driven`, `stopped` |
| `dome_stimulus` | `/dome_state_broadcaster/joint_states` (`dome_joint`), `/dome_controller/commands` (`data[0]` rad/s) | `dome_pushed` |
| `button_stimulus` | `/dome/buttons/sequence` (`DomeButtonSequence`) | `button_sequence` |

`drive_stimulus`: speed = max(planar `|v|`, `|angular.z| * wheel_separation / 2`), so a spin
in place counts as driving (`wheel_separation` 0.572 m, from `my_controllers.yaml`).
`being_driven` (intensity `min(1, peak speed / max_speed_mps)`) at most once per
`being_driven_period_s` (1.0) while speed > `moving_mps` (0.05). diff_cont's odometry ramps
over about 0.2 s, so the first one is emitted only after `confirm_s` (0.25) of continuous
motion and carries the peak speed seen, not the first weak sample; a drive shorter than
that emits nothing, not even `stopped`. `stopped` (intensity 1) once when the speed has been
below `moving_mps` for `stop_after_s` (1.0) after confirmed motion. `max_speed_mps` (1.0)
is a placeholder: set it from the drive's real top speed.

`dome_stimulus`: `dome_pushed` (intensity `min(1, |omega|/max_omega)`, `max_omega` 6.65
rad/s, the measured dome top speed) when the measured dome speed exceeds `pushed_omega`
(0.3 rad/s) while the commanded `|omega|` is below `commanded_eps` (0.02) continuously for
`pushed_after_s` (0.3), at most once per `pushed_period_s` (1.0). A command older than
`command_timeout_s` (0.5) counts as 0. The real dome coasts 0.6 to 1.0 s and up to 74
degrees after a command ends (ros2_roamadome bench results, 2026-10-16), and its position
stream lags, so a push is only reported after `settle_s` (1.5) since the last non-zero
command AND once the windowed speed has been seen at or below `pushed_omega` since then
(the coast is over). A push within about 1.5 s of a command is therefore missed.
The speed is the unwrapped displacement over the last `window_s` (0.4 s) of the 10 Hz,
roughly 1-degree-quantised positions divided by the time spanned (at least `min_span_s`,
0.25 s); the 0 / 2 pi wrap is handled. Quantisation noise is at most about 0.05 rad/s over
the full 0.4 s window and 0.07 rad/s at the 0.25 s minimum, against the 0.3 threshold.
A push is reported about 0.6 s after it starts, not instantly.
Why `commanded_eps` 0.02 is safe: the dome plugin's rad/s to percent mapping
(`velocity_map` in `ros2_roamadome`, parameters in `r2_bringup/urdf/ros2_control.xacro`) sends
0 % for any command whose fitted percent falls below `min_percent` (20 and 22 on this robot,
about 0.44 rad/s) because `velocity_map.below_minimum` is left at its default `zero`. A command
under `commanded_eps` therefore never moves the dome. Revisit if `below_minimum` is ever set to
`minimum`, which would make every non-zero command turn the dome at `min_percent`.

Limitations: any dome motion without a command on `/dome_controller/commands` (bench sweeps
with `dome_bench:=true`, firmware homing or autonomous moves) reads as `dome_pushed`. A
non-finite value in any input is ignored, and a clock jump backwards (sim reset, looping
bag) resets the detector state.

Parameters are read at startup: integers are accepted (`settle_s: 1`), a bool, a non-number
or an out-of-range value exits the node with an error, and a later `ros2 param set` is
refused ("read at startup; restart the node").

`button_stimulus`: one `button_sequence` per message, `subject` = `taps`, intensity
`min(1, len(taps)/6)`, `extra` = `{"value":..,"id":..,"source":..}`. The subscription is
reliable, depth 10.

### CPU

Measured on the Pi 5 (27 min, `ps` lifetime average): `drive_stimulus` 3.8 %, `dome_stimulus`
1.3 %, `button_stimulus` 0.3 %, plus `r2_affect` 1.4 %: 6.8 % of a core against a target of
3 %. The cost is mostly rclpy per message (28 Hz odometry, 10 Hz joint states, 20 Hz
commands) and the deserialisation of `Odometry` with its two 36-double covariances. The fix:

- `drive_stimulus` subscribes raw and deserialises one message in `odom_decimate` (default 1:
  with polling the detector sees every polled sample, about 10 Hz). The detector is unchanged.
- `stimulus_adapters` hosts the three nodes in one process (one executor and thread per node,
  so a wake-up only rebuilds that node's wait set). `adapters.launch.py` starts it by default;
  `separate:=true` starts `drive_stimulus`, `dome_stimulus` and `button_stimulus` as three
  processes, which are also fine to run on their own.
- Polled spinning: each node sleeps `poll_period_s` (default 0.1 s, a parameter of every node;
  0 = block in `spin_once` as before), then runs ready callbacks with a zero timeout until one
  call runs none (at most 32 per poll). The odometry, joint-state and dome-command
  subscriptions are best effort with depth 1, so only the newest sample of each stream is
  processed per poll (the rest are dropped by the middleware, never deserialised). The button
  subscription stays reliable with depth 10 and is drained fully: no sequence is lost (tested
  with five sequences 10 ms apart). Stimuli are therefore up to one `poll_period_s` late, and
  SIGINT/SIGTERM are noticed within one period. The dome detector copes with the 10 Hz stream
  arriving on a 10 Hz poll with skipped samples (`test_detect.py`, source periods 0.095 to
  0.105 s); its thresholds are unchanged.
- In blocking mode the executor wakes every 0.5 s (was 0.2 s) only to notice signals.

In the dev container (synthetic inputs at 28 Hz Odometry, 10 Hz JointState, 20 Hz commands;
steady state over 45 s from /proc ticks; two runs each):

| setup | % of a core |
|---|---|
| old: three processes, blocking, no decimation | 6.9-7.6 |
| `stimulus_adapters`, blocking, `odom_decimate` 5 | 5.6-5.8 |
| `stimulus_adapters`, `poll_period_s` 0.1 | 3.6-3.9 |
| `stimulus_adapters`, `poll_period_s` 0.05 | 5.7-5.9 |
| `drive_stimulus` alone, `poll_period_s` 0.1 | 2.0-2.2 |
| `drive_stimulus` alone, `poll_period_s` 0.05 | 2.9-3.1 |

That is about 0.5 of the old total. A profile of the polled node shows about 2 s of Python in
30 s, mostly start-up; what remains is largely the middleware receiving about 58 messages a
second, which no Python change removes. `poll_period_s` trades latency against CPU roughly
linearly.

Re-measure on the robot with `ps -o pcpu,args -C python3` (or `top -H`) after 10 minutes of
driving.

### Testing

Unit and node tests: `colcon test --packages-select r2_stimulus` (the node test remaps every
topic to private names, so it is safe with a live sim on the graph).

| adapter | in sim (`ros2 launch r2_sim sim_foxglove.launch.py`, adapters start with it) | on the robot |
|---|---|---|
| `drive_stimulus` | (the Foxglove Teleop panel publishes a plain Twist, which `twist_mux` does not take on `/cmd_vel_joy`) `ros2 topic pub -r 10 /cmd_vel_joy geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.3}}}"`; expect `being_driven` about once a second, then `stopped` 1 s after you stop; `angular: {z: 1.0}` instead of linear x spins in place and must not produce `stopped` | drive with the joystick |
| `dome_stimulus` | negative: `ros2 topic pub -r 10 /cmd_vel_dome_foxglove geometry_msgs/msg/Twist "{angular: {z: 1.0}}"` turns the dome and emits nothing; positive: a Gazebo wrench on the dome link with the dome idle (the dome velocity controller may hold the joint; the unit tests are the primary check) | turn the dome by hand with the controller idle (keep hands clear of the gear ring); drive it with the trigger for the negative case |
| `button_stimulus` | `ros2 topic pub --once /dome/buttons/sequence astromech_msgs/msg/DomeButtonSequence "{value: 4, taps: LRR, id: 1, source: 1}"` | tap the dome buttons |

Watch the output with `ros2 topic echo /stimuli` or the adapters' log lines.
