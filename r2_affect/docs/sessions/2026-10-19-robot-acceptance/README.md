# 2026-10-19 robot acceptance run

Phase 0 acceptance script (r2_dev `docs/personality-phase0-plan.md`, section 8) on the real
droid: `launch_robot.launch.py` on the body Pi, Foxglove on `:8765` with the tuning layout,
the CLIs run from the dev container through the linked Zenoh routers. David drove the panels
and the dome; Claude drove the CLIs and the base (as the operator, from the container; the
personality stack never commands the drive). Recorded on wall time with
`ros2 bag record --include-unpublished-topics --topics /stimuli /mood/state /rosout`:
1575 s, 22 stimuli, 13 998 states. Config: the tuned `affect.yaml` (idle 180/300).

| item | result |
|---|---|
| 1 fresh start | `calm` at temperament; seen again after the item 12 restart |
| 2 bump | applied; `calm -> scared` after the dwell |
| 3 second bump within 2 s | `dropped: refractory` |
| 4 leave it | `excited -> calm` 56 s after the lock was cleared (arousal relaxing from 0.8) |
| 5 face_known dave, bearing 0.8 | focus set; focus bearing updated to −0.5 later while locked |
| 6 force sleepy | label `sleepy` at once |
| 7 force + lock excited | three bumps `(locked, not applied)`; focus still updated |
| 8 lock from another source | refused naming the holder; clear from the other source accepted |
| 9 unknown kind | one WARN |
| 10 reload | not run on the robot (needs the YAML edited on the Pi); passed in the sim |
| 11 idle | `calm -> sleepy` 6.7 min after the last stimulus (tuned values predict 6.6) |
| 12 kill r2_affect | `/mood/state` silent; the other nine nodes unaffected, joint states 10 Hz, odom 28 Hz; restart `calm` at temperament, stimuli applied at once |
| 13 pause Gazebo | sim only |
| 14 CPU after 10 min | **fails**: 6.8 % of a core for the four processes, target 3 % (below) |

Adapters on the robot (plan step 7, robot column):

| check | result |
|---|---|
| dome turned by hand, trigger released | `dome_pushed` at intensity 0.07 then 0.54; the case Gazebo could not produce |
| dome driven from the Foxglove Teleop panel, then coasting | 27 panel twists, 56 non-zero controller commands, 4.9 rad of travel, **no** `dome_pushed` during the turn or the coast |
| base driven 0.2 m/s for 3 s | `being_driven` 0.19, then `stopped` about 1 s after the command ended |
| base spun in place 0.6 rad/s for 3 s | three `being_driven` (spin counts as driving), one `stopped` after, none during |
| dome buttons | not run: the dome device (`/r2_dome`) is not on the robot's graph; its router setting is phase 1 step 0 |

`ros2 run r2_affect replay <this dir> --compare`: 0 label disagreements over 13 998 states;
max diff 0.046 (confidence) at 1200 s, the kill-and-restart of item 12, which the replay
can only approximate as a force. Everything else reproduces.

**Item 14, measured with `ps -eo pcpu` (lifetime average) after 27 minutes:**

| process | CPU |
|---|---|
| `drive_stimulus` | 3.8 % |
| `r2_affect` | 1.4 % |
| `dome_stimulus` | 1.3 % |
| `button_stimulus` | 0.3 % |

`drive_stimulus` deserialises `nav_msgs/Odometry` (two 36-double covariances) at 28 Hz in
Python; `dome_stimulus` takes joint states at 10 Hz and commands at 20 Hz; 0.3 % is the
floor of an idle rclpy process with a 0.2 s executor timeout. Proposed fix, as a follow-up
PR: a raw subscription on `/diff_cont/odom` deserialising one message in five, and the three
adapters hosted in one process; expected total about 2.5 %. `r2_affect` itself is fine.
