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

Three nodes turn existing signals into stimuli. Started together by
`ros2 launch r2_stimulus adapters.launch.py [use_sim_time:=true] [config:=PATH]` (or by
`r2_bringup`'s `personality.launch.py`, which passes `use_sim_time`). Thresholds are ROS
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

`drive_stimulus`: `being_driven` (intensity `min(1, |v|/max_speed_mps)`) at most once per
`being_driven_period_s` (1.0) while `|v| > moving_mps` (0.05); `stopped` (intensity 1) once
when the speed has been below `moving_mps` for `stop_after_s` (1.0) after having moved.
`max_speed_mps` (1.0) is a placeholder: set it from the drive's real top speed.

`dome_stimulus`: `dome_pushed` (intensity `min(1, |omega|/max_omega)`, `max_omega` 6.65
rad/s, the measured dome top speed) when the measured dome speed exceeds `pushed_omega`
(0.3 rad/s) while the commanded `|omega|` is below `commanded_eps` (0.02) continuously for
`pushed_after_s` (0.3), at most once per `pushed_period_s` (1.0). A command older than
`command_timeout_s` (0.5) counts as 0, and nothing is emitted for `settle_s` (0.5) after a
non-zero command ends, so the dome's own coast is not a push. The speed is computed from the
10 Hz, roughly 1-degree-quantised positions as the unwrapped displacement over the last
`window_s` (0.4 s) of samples divided by the time spanned (at least `min_span_s`, 0.25 s),
so quantisation costs at most about 0.05 rad/s and the 0 / 2 pi wrap is handled. This
reaction is therefore about 0.6 s after a push starts, not instant.

`button_stimulus`: one `button_sequence` per message, `subject` = `taps`, intensity
`min(1, len(taps)/6)`, `extra` = `{"value":..,"id":..,"source":..}`. The subscription is
reliable, depth 10.

### Testing

Unit and node tests: `colcon test --packages-select r2_stimulus` (the node test remaps every
topic to private names, so it is safe with a live sim on the graph).

| adapter | in sim (`ros2 launch r2_sim sim_foxglove.launch.py`, adapters start with it) | on the robot |
|---|---|---|
| `drive_stimulus` | Foxglove Teleop panel on `/cmd_vel_joy`, or `ros2 topic pub -r 10 /cmd_vel_joy geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.3}}}"`; expect `being_driven` about once a second, then `stopped` 1 s after you stop | drive with the joystick |
| `dome_stimulus` | negative: `ros2 topic pub -r 10 /cmd_vel_dome_foxglove geometry_msgs/msg/Twist "{angular: {z: 1.0}}"` turns the dome and emits nothing; positive: a Gazebo wrench on the dome link with the dome idle (the dome velocity controller may hold the joint; the unit tests are the primary check) | turn the dome by hand with the controller idle (keep hands clear of the gear ring); drive it with the trigger for the negative case |
| `button_stimulus` | `ros2 topic pub --once /dome/buttons/sequence astromech_msgs/msg/DomeButtonSequence "{value: 4, taps: LRR, id: 1, source: 1}"` | tap the dome buttons |

Watch the output with `ros2 topic echo /stimuli` or the adapters' log lines.
