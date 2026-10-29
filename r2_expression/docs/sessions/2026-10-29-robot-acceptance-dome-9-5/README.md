# 2026-10-29 robot acceptance run (phase 2, dome): items 9 and 5 (joystick)

The two phase 2 acceptance items left open by the 2026-10-23 run
(`../2026-10-23-robot-acceptance-dome/README.md`, Findings 8): item 9, base-motion
suppression, and the joystick half of item 5. Same setup as that run: the droid's own dome on
the body Pi, `launch_robot.launch.py`, the dome reached only through `dome_twist_converter`,
the code the 2026-10-24 session left on the Pi (`phase2` branches; not re-verified from the
container). David drove the base and the dome trigger from the joystick; Claude drove the
intents from the dev container over the peered Zenoh routers, from one long-lived rclpy node
(`probe.py`: it logs the dome channel, the converter's source, `/dome/cmd/personality`, the
controller command, the drive commands and the arbiter's log lines, and publishes
`ExpressionIntent` from lines appended to `probe_commands.txt`). No node tests ran in the
container (rule 13). The mood was locked `calm` for the whole run and unlocked at the end.
Times are UTC arrival times in the container unless "stamp" is said; `probe.log` is the full
probe output (1997 lines; the `===` lines mark the passes), `bag/` the recording.

| item | result | evidence |
|---|---|---|
| 9 body moving | PASS | Behaviour looks (`look ±1.5 lazy`, source behavior) every 6 s with an idle glance between, so the dome was mid-move when David drove. First attempt (00:43:24): the stick at a third gave 0.158 m/s for one message, nothing suppressed, `drive_stimulus` saw `being_driven` (mood locked, not applied). Second (firm pushes back and forth, 00:44:17 to 00:44:23): three suppression episodes. Onset: joystick command 0.158 m/s arrived 00:44:17.542, the arbiter's 0 on `/dome/cmd/personality` at +2 ms, `dome` `SUPPRESSED` (stamp 17.630), `dome: move over` and `dome: suppressed` logged, the controller command 0 at +12 ms, the look `probe-b1f330a9` (commanded -0.60 since 16.478) abandoned. Dropped and counted: an idle glance at 19.438 (`dropped` 8 to 9) and a behaviour look at 22.439 (9 to 10), both while `SUPPRESSED`. Release: `idle` 1.01, 1.06 and 1.01 s after the last command above the threshold (`after_still_s` 1.0; peaks 0.27, 0.55 and 0.51 m/s). Manual while suppressed (00:45:08 to 00:45:40, manual looks `±1.5 lazy` at 199 every 5 s while David drove back and forth, peaks to 0.86 m/s and 0.56 rad/s): the running manual look was halted at the onset (`dome: suppressed (manual)`, 22.549; by design, a move in progress stops whatever its source), then four manual looks sent while suppressed (23.043, 28.045, 33.038, 38.039) were all accepted and drove the dome (converter source `personality`, -0.60 / +0.60), the channel reading `suppressed` with `owner=manual` throughout (the plan's cosmetic note), while the idle glances between them were dropped (`dropped` 15 to 18); `running (manual)` 1.04 s after the last above-threshold command |
| 5 joystick mid-move | PASS | Two samples during behaviour looks. 00:47:01: converter source `joystick` (stamp 01.290, -0.002 rad/s: a light pull), `dome` `FOREIGN owner=joystick` stamp 01.293 (+3 ms), the arbiter's 0 at +7 ms, `dome: move over` (look `probe-4f41450b` abandoned at -0.60) and the look sent 37 ms earlier dropped (21 to 22); release: the hold's zeros, source `idle` and `dome` `idle` at 03.38, no resumption (the next motion is the next intent at 06.24). 00:48:02: a full pull, source `joystick` -0.819 rad/s (stamp 02.644), `FOREIGN` stamp 02.647 (+3 ms), the arbiter's 0 at +2 ms, look `probe-d802f4eb` abandoned, the look sent during the hold dropped (22 to 23); release: joystick 0 at 05.305, source `idle` and `dome` `idle` at 05.71, no resumption, the next motion the next intent at 08.25. A third pull by David never crossed the joystick deadzone and left no trace |

Bag: `bag/` 00:42:16 to 00:48:38 (382 s), 35 166 messages, 4.6 MiB mcap (under 20 MB, committed
as recorded). Topics: `/cmd_vel_dome` 8 853, `/expression/status` 4 830, `/dome_controller/commands`
4 130, `/dome/cmd/personality` 3 822, `/dome_state_broadcaster/joint_states` 3 819,
`/diff_cont/cmd_vel` 2 864, `/cmd_vel_joy` 2 742, `/mood/state` 1 910, `/rosout` 1 286,
`/dome/cmd/status` 815, `/expression/intent` 63, `/stimuli` 32. The first minute's recording
was discarded: the probe had subscribed to `/dome/cmd/personality` as `Float64MultiArray`
(it is `TwistStamped`, `angular.z`), which made rosbag2 refuse the topic as two-typed.

## Findings

1. **The threshold against this joystick.** `scale_linear` is 0.5 m/s at full stick and the
   suppression threshold 0.15 m/s, so a stick under a third does nothing while the dome keeps
   moving; a spin needs 0.3 rad/s, over half of the 0.5 rad/s yaw scale. Fine for the purpose
   (a creep does not disturb the dome) and left as it is.
2. **A manual move already running is stopped at the onset.** `_dome_guard` halts any move in
   progress when suppression begins, manual included; manual intents sent after that run. The
   plan's wording (item 9: `express look 0` manual "still runs") is what was tested and holds.
3. **The joint misread** of the 2026-10-23 run (Findings 4) showed once more at 00:41:35
   (319° to 279° to 319° in two samples at 0.6 rad/s), held back by the plausibility gate.
4. **Drive stimuli while locked.** `being_driven` and `stopped` from `drive_stimulus` arrived
   with every push (`/stimuli` 32 messages) and were refused by the lock as expected.
5. Remaining open from the 2026-10-23 run, unchanged: item 6 and item 3's cap-1.0 half (sim
   only), item 8's hand push, the acceleration cap by feel, and item 16's CPU budget.
