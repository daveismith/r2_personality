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

Nodes that turn existing signals (drive odometry, dome motion, dome buttons) into stimuli
arrive in step 7 of the phase 0 plan.
