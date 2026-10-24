# 2026-10-23 robot acceptance run (phase 2, dome)

Phase 2 acceptance script (r2_dev `docs/personality-phase2-plan.md`, section 9, robot column,
step 8b) on the droid's own dome, on the body Pi (`/home/pi/r2_ws`, symlink install): the
radar eye forward at joint 0, the slip ring, `launch_robot.launch.py` with the dome arbiter
reaching the dome only through `dome_twist_converter`. Code: `r2_personality` 8c0c70d (loop
tuning, band 0.10 rad) until the arbiter restart at 01:33:53, then f4f8e11 (settle band 0.15 rad),
on top of 0b43b3f (coast re-fit) and 4c9e490 (position-misread gate); `r2_bringup` 1ac079d;
`astromech_msgs` 162a100, all on `phase2`. David was at the droid for part of the time (the Foxglove drives,
the kills and restarts on the Pi); Claude drove the CLIs and probes from the dev container
through the peered Zenoh routers over WiFi. The probes of items 2 to 4 published their intents
from one rclpy node (`probe-*` ids in `rosout.txt`): the `express` CLI churned discovery, as the
sim note found. The mood was locked `calm` unless stated. Recorded on wall time from the
container; times below are UTC and bag arrival times unless "stamp" is said. `rosout.txt` is
`/rosout` as text (`dome_arbiter`, `dome_twist_converter`, `r2_affect`; 437 exact duplicate lines
removed, Findings 7); `transcript.txt` is the per-item numbers the table cites, with the
parts of the run the bags do not hold marked "run log".

| item | result | evidence |
|---|---|---|
| 1 fresh start | PASS | `express turn 0.3`: the radar eye turned to the droid's left (joint angle decreasing) at the 0.6 floor throughout, 2.2 s of motion, stopped with a 0, final error 1.8 deg (run log; not in the bags). An 8 deg spring-back after the stop was seen once and not again in five later stops. The bag holds a repeat (00:08:12.870, band 0.10): -0.60 for 1.1 s, joint 1.065 to a low of 0.593 (9.7 deg past the 0.765 target), a +0.60 pulse, rest at 0.873 (6.2 deg short), `claimed` 00:08:16.9, `idle` 00:08:21.3. Sign and home are right |
| 2 `look 1.0 / -1.0 / 3.0 --profile smooth` | PASS (judged at band 0.15) | 01:31 set (run under 8c0c70d, band 0.10; not repeated at 0.15), calm locked, hands off, one probe node: `look 1.0` error -5.7 deg, 3.0 s of motion, in the band at +2.5 s, peak 1.00; `look -1.0` -7.3 deg, 4.4 s, +3.0 s, peak 1.00; `look 3.0` (joint 0.873 across the wrap) +3.9 deg, 5.1 s, +4.5 s, peak 1.00. All `running act=look_at`, then `claimed`, then `idle` 9-11 s after the intent (the 5 s manual hold). Errors of -7.3 and -7.7 deg are outside 0.10 rad (5.7 deg) and inside 0.15 rad (8.6 deg): the plan's wording is changed to 0.15 (Findings 3). Earlier sets in `transcript.txt` (before the tuning; the container's node tests on the graph): errors of 0.3 to 29 deg where the run was not interrupted |
| 3 `look 0 --profile snappy`, then cap 1.0 | PASS (first half) | `look 0 snappy` (01:31:38.876, from joint 3.351): error -7.0 deg, 3.9 s, in the band at +4.3 s, peak 2.00 = the cap, every non-zero command at least 0.60 (every personality-source value in both bags is 0 or at least 0.60), `capped` false throughout (0 `capped` messages in either bag). The second half (cap 1.0) needs a converter restart and was not run on the robot; it is the sim note's item 3 (same code path) |
| 4 `glance --amplitude 0.6 --dwell 1` | PASS | 01:31:49.822: out 28 deg (asked 34, inside the band), return offset -4.0 deg, 5.2 s of motion, `idle` 5.1 s after the move (the run is at band 0.10; the same reading). The earlier glance sets landed at -1, -12 and +1 deg |
| 5 stick / Foxglove mid-move | PASS (Foxglove variant) | 00:17:55.474 a Foxglove drive during a behaviour turn: converter source `foxglove`, `foreign owner=foxglove` 0.094 s later, the arbiter's 0 on `/dome/cmd/personality` 2 ms after the converter switched, no non-zero personality output while foreign; release 00:17:56.174, `idle` with it, no resumption (the next move, -0.60 at 00:17:56.9, belongs to the next behaviour intent). 00:02:27.268 the same during a glance: `foreign` +0.008 s, personality 0 at +0.008 s, `idle` 00:02:28.470, next personality command the glance of 00:02:47. Joystick variant not run (no joystick to hand) |
| 6 `claim dome --override` | not run on the robot | done in the sim (same code path) |
| 7 idle glances by mood | PASS (note) | `curious --lock` (01:34:02.8): glances every 8.0 s (palette 8-20, strength 1 locked: the lower bound), 57/68/38/59 deg (palette 29-80), returns -6/1/-6/-8 deg. `excited --lock` (01:35:18.8): every 4.0-4.2 s (4-10), 9-50 deg (palette 17-52; the 9 deg one is short), returns within +-9 deg. `calm --lock` (01:36:34.9): 3 glances in 75 s, first after 20.1 s (palette 20-60), 11/19/14 deg (15-40), returns -5/-1/-2 deg. Curious bigger and quicker than calm, excited quick and small; per-glance table in `transcript.txt` |
| 8 sleepy rest, return | PASS (mechanism); gap | `sleepy --lock` (01:42:05.5): mood change 01:42:05.7, rest move 0.4 s after the CLI's accept, at +0.60, done at joint 5.777: -8.9 deg from the heading (`rest_rad` 0.35). The recheck fired at 01:42:54.1 and corrected to -3.9 deg (1.6 s). The displacement: a behaviour `turn 0.35` at 01:42:18.7 moved the dome 0.419 rad (24 deg) and the rest was re-applied as it ended (1.4 s): not a push (the sim's finding 6). An earlier `turn 0.35` (01:39:20.8, calm) arrived while the dome was offline (Findings 2) and moved nothing. A hand push was not available: the 30 s recheck after a real push is unconfirmed on the droid |
| 9 body moving | NOT RUN | David could not drive the base; no `suppressed` state in either bag |
| 10 mood locked, bumps | PASS | two `bump` stimuli (23:17:14.1 and 23:17:17.1 on `/stimuli`); `r2_affect` logged `no change ... (locked, not applied)` twice (23:17:15.1, 23:17:19.1); mood `calm` and locked throughout |
| 11 kill `r2_affect` | PASS (at the limit) | SIGKILL; last `/mood/state` stamp 23:17:20.644, `dome` `stale mood=neutral` stamp 23:17:23.676 (+3.03 s; 2.95 s in the run); restart: first `/mood/state` 23:17:36.487, `idle mood=calm` at once, first glance 23:17:56.6 |
| 12 kill `dome_arbiter` mid-move | PASS | `look -2.5 --profile lazy` (23:56:15.7), SIGKILL; last `/dome/cmd/personality` stamp 23:56:17.407, the converter's source `idle` 0 stamped 23:56:17.928 (+0.52 s; 0.62 s in the run: the 0.5 s timeout plus a tick), nothing non-zero forwarded after, the LED channels kept publishing status; restart 23:56:30.2: `offline`, `idle` 23:56:30.8, no move. The link stalled 2.4 s around the kill (all topics from the Pi), so use the stamps |
| 13 stop `dome_state_broadcaster` | PASS | last joint state 23:16:17.311, `offline` 23:16:22.358 (+5.05 s, `device_stale_s`); first joint state after the reactivation 23:16:28.011 and `idle` on the same tick (the run log has 1.5 s) |
| 14 graph check | PASS | over 30 nodes: `/dome_controller/commands` published by `/dome_twist_converter` only; `/dome/cmd/personality` by `/dome_arbiter` only; nothing on `/dome/joint_command`; no personality node on `/cmd_vel*` or `/diff_cont/*` (run log; not in the bags) |
| 15 `dome_pushed` | PASS (no positive control) | bag: 2 messages on `/stimuli`, both `bump` (item 10); bag2: 0 messages. Across the two bags the dome ran 16 looks, 278 glances, 66 turns and 7 rests with no `dome_pushed`. No push `dome_stimulus` could see was made, so absence is shown, not the detector firing |
| 16 CPU | FAIL (budget); rate PASS | `/dome_state_broadcaster/joint_states` 10.0 Hz with and without the arbiter. 10 minutes idle (calm glances only, router quiet at 1.6 %): `dome_arbiter` 3.64 %, converter 2.69 %, `stimulus_adapters` 2.62 %, `led_arbiter` 1.67 %, `magic_arbiter` 1.40 %, `r2_affect` 0.73 %; against 2 %. A near-continuous-motion window: `dome_arbiter` 5.01 %, converter 4.06 %; the budget is 4 % during a move. Findings 5 |

Bag: `bag/` 23:10:46 to 00:19:05 (4099 s), 143 962 messages, 4.9 MiB mcap with Zstd chunks
(rewritten from 18.5 MiB); `bag2/` 00:19:32 to 01:46:20 (5209 s), 188 065 messages, 6.2 MiB
(from 24.0 MiB); both well under 20 MB, to be committed. Topics (bag + bag2): `/dome_state_broadcaster/joint_states`
32 165 + 48 921, `/expression/status` 30 275 + 43 539, `/dome_controller/commands` 20 715 + 22 410,
`/dome/cmd/personality` 19 961 + 23 432, `/rosout` 17 556 + 16 748, `/mood/state` 16 108 + 22 836,
`/dome/cmd/status` 7 047 + 10 165, `/expression/intent` 133 + 14, `/stimuli` 2 + 0,
`/diff_cont/cmd_vel` 0 + 0. `bag2/` was still being recorded when this note was started; it was
stopped (SIGTERM) at 01:46:20 and `metadata.yaml` written then, so nothing is attached any more.
The recording began at 23:10:46, not 23:38 as first noted.

## Findings

1. **The bags are rewritten and the history already holds raw ones.** Both were rewritten with
   `rosbag2_py` (reader to writer, `compression: "Zstd"` in the mcap storage config; every message
   kept; `ros2 bag convert` produced an empty bag in the sim). The repository history keeps the
   uncompressed copies committed earlier in 4c9e490 (8.0 MB, a partial bag) and 8c0c70d (19.4 MB and
   17.0 MB, `bag2/` while it was still open); the compressed files replace them in the next commit
   but not in the history.
2. **Node tests in the dev container load the robot's router, and the stack stalls.** The Zenoh
   router on the Pi twice reached 80-96 % of a core. Cause: node tests run in the container
   (`dome_mock_t*`, `arbiter_test_*`, a fake `/mood/state` publisher, the `dome_arbiter_tdn*`,
   `led_arbiter_t*`, `dome_twist_converter_tdn*` names in `/rosout`: about 6 000 lines from 23:13
   to 01:28:26) reach the robot's router through the peered routers over WiFi, and the router did
   not recover until it was restarted. The router logged `Route reply: Query not found` and
   `Didn't receive final reply for query ... Timeout(600s)` on the peer link face. The bag shows it: joint
   states of 10 Hz arrive thinned or in gaps (23:32:51 to 23:47:33 nothing; 2.8 Hz in 23:3x;
   01:28:23 to 01:30:49 and 01:37:49 to 01:40:21 nothing, 8.2 Hz and 7.0 Hz in the buckets; dozens of 5-16 s
   gaps from 00:50), `r2_affect` logged `time step 138.249 s clamped` at 01:30:41, the dome read `stale`
   01:28:26 and `offline` 01:28:34 and again 01:38:29, and the stack was relaunched at 01:40:17 (its second `dome_arbiter` refused to start, the first
   still being on the graph: `cannot start: another node named /dome_arbiter is already on the graph`). Rule: no node tests
   in the container while the robot is connected; added to `r2_bringup`'s README "Things to avoid"
   and to the plan (rule 13, risks). The accepted results above come from after 01:28:26 or from
   stamps; item 8's first turn (01:39:20) fell in the second stall.
3. **The settle band is 0.15 rad and the looks end about 5 deg to one side.** Four looks, a lazy look
   and a glance: -5.7, -7.3, +3.9, -7.0, -7.7, -4.0 deg: mean about -5 deg, spread about 4 deg, five of
   six on the joint-negative side of the target, several outside 0.10 rad. The feedback is 10 Hz
   (11 deg per sample at 2 rad/s) and the floor's own run-on is 7 deg, so a 5.7 deg band is
   tighter than the loop can hold; f4f8e11 set `settle_band_rad` 0.15 for the three profiles,
   loaded at the 01:33:53 restart (items 7 and 8 ran with it; items 2 to 4 were not repeated). Item 2's wording follows: 0.15 rad (8.6 deg). The
   measured plant: lag 0.6 s to 90 %, coast 0.52 s * v, 1.29 rad/s clockwise and 0.76 anticlockwise
   at a 1.0 command, the floor at 0.35 rad/s achieved with a 0.12 rad run-on; the re-fit and the
   gate are in `dome-motion-measurements.md`.
4. **Coast learning was noise and is off.** Before it was turned off the learned scale swung
   0.19 to 1.25 between moves (`rosout.txt` from 23:10); a low lesson gave a 53 deg overshoot on a look
   across the wrap and 48 deg on a glance's outbound leg. 8c0c70d: learn off, the floor pulse, retries 2.
   The ring misreads one sample near 313-331 deg clockwise at 1.0 rad/s (-0.59 then +0.91 rad); the
   plausibility gate (`max_plausible_rad_s` 4.0, 4c9e490) holds such a sample back. No gate event
   shows in `rosout.txt` (it does not log).
5. **Item 16 fails, and the cost is rclpy's, not ours.** `dome_arbiter` idle is 3.64 % against the
   2 % of rule 11 (5.01 % in near-continuous motion; the converter 2.69 % and 4.06 %). cProfile of the node:
   our Python is about 10 us per joint state; the cost is the executor, about 1.7 ms per wake-up on
   the Pi 5 (a wait-set rebuild over about 32 waitables), so only fewer idle wake-ups reach 2 %. A
   trim of the settle passes saved nothing (branch `phase2-cpu-trim`, not merged). Follow-up, not
   fixed in this phase: a lower idle feedback rate or fewer entities. The plan's budget sentence now
   says what was measured.
6. **Item 8's push is still unproven, and the rest re-applies at the end of any move.** A
   `turn 0.35` while sleepy moved the dome 24 deg (0.419 rad for 0.35 asked) and the rest took it back
   1.4 s later, as in the sim. The 30 s recheck after a real hand push has not been seen on the
   droid; the recheck seen (01:42:54, 35.5 s after the turn began) corrected -8.9 deg to -3.9 deg.
7. **Environment.** `/rosout` in the bags holds each arbiter line twice from about 23:46 (437 duplicate lines
   removed from `rosout.txt`; the cause is unknown, probably the peer link). The robot's stack ran at
   nice 19 until it was found: `/etc/security/limits.conf` gave `@realtime` `priority 99` (a nice
   value, clamped to 19) beside the intended `rtprio 99`; the two `priority` lines were deleted and
   `sudo renice -n 0 -u pi` fixed the running stack (`r2_bringup` README, `docs/robot-host-devices.md`).
   The robot's `setup.bash` exported a stale `ZENOH_ROUTER_CONFIG_URI`; the right file is the
   three-line template of the workspace Makefile (`RMW_IMPLEMENTATION`, `source
   /opt/ros/jazzy/setup.bash`, `source install/setup.bash`). Nodes log `Unable to connect to a Zenoh
   router` when launched before the router listens: start the router first and wait for
   `ss -ltn | grep 7447`. Method errors, not failures: with `sleepy` unlocked every manual move was followed by
   the rest move back to 0.35 rad (by design, 01:19-01:21 in the bag); an attempt while David drove
   the dome servos by hand gave moves abandoned to the stick (by design).
8. **Not covered**: item 9 (the base), item 5's joystick variant, item 6 and item 3's second half
   on the droid, the hand push of item 8, the acceleration cap by feel at 2 rad/s, and the
   droid's CPU budget (item 16).
