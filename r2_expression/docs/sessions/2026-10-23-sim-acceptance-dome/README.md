# 2026-10-23 sim acceptance run (phase 2, dome motion)

Phase 2 acceptance script (r2_dev `docs/personality-phase2-plan.md`, section 9, items 2-15;
1 and 16 are robot-only) run against the headless simulator in the dev container
(`ros2 launch r2_sim sim_foxglove.launch.py`) with the shipped `expression.yaml` and
`affect.yaml`, on the `phase2` branches (`r2_personality` 75f0108, `r2_bringup` 1ac079d,
`astromech_msgs` 162a100, `r2_sim` 1f19571). The real workbench dome `/r2_dome` was on the
graph throughout (`/foxglove_bridge` was not): nothing called `/r2_dome/*` and nothing
published to `/dome/*` of the device; the LED and Magic Panel arbiters used the sim's
`/dome_mock`, and the dome arbiter reached the sim dome only through `dome_twist_converter`.
At the start no arbiter, `/r2_affect` or converter from the robot was on the graph. An agent
drove the CLIs (`express`, `mood`, `ros2 topic pub`, `ros2 control`) and rclpy probes
(sim-time logger of the dome status, joint state, `/dome/cmd/personality`,
`/dome_controller/commands`, `/dome/cmd/status`, `/stimuli`; a publisher playing the stick, the
Foxglove teleop panel and the base); nobody drove the Foxglove panels. Recorded on sim time with
`ros2 bag record --use-sim-time --include-unpublished-topics --topics /expression/intent /expression/status /mood/state /dome/cmd/personality /dome/cmd/status /dome_controller/commands /dome_state_broadcaster/joint_states /diff_cont/odom /rosout /stimuli`
(section 9's list plus `/stimuli`, for item 15; `/clock` is not in it). The bag was then
rewritten losslessly with mcap Zstd chunk compression (Findings 1). `rosout.txt` is `/rosout`
as text (the dome, converter, affect, LED/Magic arbiters, stimulus and controller nodes; the
sim's twist_mux "Velocity command timed out" flood filtered out); `transcript.txt` is the
per-item command output, the `/expression/status` lines for `dome`, the converter source
changes and the glance analysis the table cites. All times are sim seconds.

| item | result | evidence |
|---|---|---|
| 1 fresh start | robot only; sim variant PASS | sim variant: `express turn 0.3` (intent 131.557): joint -0.0307 to -0.3367 (decreasing; target -0.3307, error -0.3 deg), command -0.6 for 0.45 s, `running` 131.563 to `claimed` 132.908 (1.35 s, `max_move_s` 8). At the start `dome` was `idle` and the only personality commands were calm idle glances (first at 94.2, then 13-24 s apart) |
| 2 `look 1.0 / -1.0 / 3.0 --profile smooth` | PASS (note) | `look 1.0` (171.243): within 3 deg at +1.0 s, move over +1.75 s, final error 1.5 deg; peak command 0.805, min non-zero 0.6. `look -1.0` (189.479): within 3 deg +2.35 s, over +3.2 s, error 0.06 deg, peak 1.0. `look 3.0` (207.448, joint 1.00 to 3.27 across the +-pi wrap, the short way): +2.7 s, over +3.5 s, error -0.65 deg. All three `running act=look_at`, then `claimed owner=manual lease=5.0` and `idle` 5.0 s after the move (the manual hold). The end of the move is not "one 0": see Findings 2 |
| 3 `look 0 --profile snappy`, then cap 1.0 | PASS | cap 2.0 (`look 0`, intent 240.481, from joint 3.29): max command 2.000 = the cap, every non-zero at least 0.6 (min 0.600), `capped` false all run, within 3 deg +2.7 s (one coast stop at 242.25 and a 0.6 retry), error 1.4 deg. Converter restarted with `personality_max_speed_rad_s:=1.0` (Findings 3), `look -1.0 --profile snappy` (358.470): `/dome_controller/commands` max 1.000, min 0.6, `capped` true 359.00 to 361.50, within 3 deg +3.15 s, error -0.5 deg (the arbiter still asked for 2.0). Converter restored to 2.0 afterwards |
| 4 `glance --amplitude 0.6 --dwell 1` | PASS (note) | joint 5.9488 to 6.5608 (+0.612 rad) by +1.2 s, held about 1.6 s, back at 5.9488 (error 0.0 deg) by +4.0 s; `running` 419.712 to `claimed` 424.459. Run while the unlocked mood had decayed to sleepy (lazy profile, 0.6 rad/s: Findings 8) |
| 5 joystick / Foxglove mid-move | PASS | `look 2.5` (527.290), stick = `TwistStamped` `linear.x` 1.0 on `/cmd_vel_dome` at 20 Hz for 2 s: `CS source=joystick` 529.352, `dome foreign owner=joystick` 529.406 (joint 4.59 of a 3.78 target: abandoned), command +1.0 from 529.70, joint 4.59 to 6.46; release: `source=idle` 531.452, `claimed` (manual hold) then `idle` 534.449 (3.0 s after release = `after_foreign_s`), joint held 6.461 for 15 s: no resumption. Foxglove: `look 2.5` (560.342), `Twist` `angular.z` 0.5 on `/cmd_vel_dome_foxglove` (x1.5 = 0.75): `source=foxglove` 561.001, `foreign owner=foxglove` 561.002, joint 6.03 to 7.56; `source=idle` 563.300, `idle` 566.004, joint held 7.563. First joystick attempt (look 2.0 at 480.976) invalid: the probe's discovery lag put the stick 2.4 s in, when the move had nearly arrived |
| 6 claim `--override` | PASS (note) | `claim dome --owner console --lease 30 --override`: `claimed owner=console prio=200 lease=30.0` 601.992. `look 1.0` (manual 199, intent 608.254): `dropped` 1 to 2 at 608.255, no move (an earlier drop 0 to 1 at 605.2 had no intent in the bag: an idle glance). `look 1.0 --override` (intent 612.921, prio 200): `running owner=console prio=200` 612.927, joint 7.56 to 5.30 (target 5.28). Release (`--owner console`): `claimed owner=manual prio=200 lease=1.3` then `idle` 621.419. The refusal text `claimed by 'console' at priority 200` is not shown anywhere: Findings 4 |
| 7 idle glances by mood | PASS (note) | `curious --lock` (set 658.6): 7 glances in 100 s, first 9.2 s after the set, 13.0-14.1 s apart, excursions 0.53-1.29 rad (range 0.5-1.4); `calm --lock` (767.2): 7 in 170 s, 23.5-24.7 s apart (20-60), 0.33-0.63 rad (0.26-0.70); `excited --lock` (941.0): 10 in 75 s, 7.1-8.7 s apart (4-10), 0.35-0.78 rad (0.3-0.9). Every glance returned to the start heading within 2.5 deg (curious), 1.2 deg (calm), 2.4 deg (excited). Calm rarer and smaller than curious, excited quick. The spacing sits at the lower bound of `every_s` (Findings 5). Per-glance table in `transcript.txt` |
| 8 sleepy rest, return | PASS (note) | `sleepy --lock` (1025.5): rest started 1027.303, done 1029.893 at joint 5.913 (bearing 0.370, `rest_rad` 0.35: 1.1 deg), lazy 0.6 rad/s, held 45 s. `turn 0.35 --source behavior` (1076.703) moved it 0.31 rad and the rest was re-applied as that move ended (1078.121, 1.4 s): not a push (Findings 6). A real push (probe putting 0.7 rad/s on `/dome_controller/commands` for 0.5 s at about 1152, invisible to the arbiter): joint 5.913 to 6.246 (+0.333 rad, 19 deg); correction `running act=rest` at 1169.844 (17.7 s later, inside `rest_recheck_s` 30), command -0.6 (`min_speed`), done 1171.185 at 5.9198 (0.4 deg from the rest) |
| 9 body moving | PASS | `excited --lock`; `look 2.8 --profile smooth` (intent 1238.034, peak -1.0); `TwistStamped` `linear.x` 0.3 at 10 Hz on `/diff_cont/cmd_vel` for 14 s: `dome suppressed` 1238.756, command to 0 at 1238.754 (same tick). `dropped` 2 to 3 (1242.837) and 3 to 4 (1251.813): the next two glances. `look 0` manual (intent 1245.506) ran while suppressed: command 0.6 to 1.0, 0 at 1247.03, joint 5.33 to 6.30 (target 6.283). Base still (last `being_driven` 1250.616, `stopped` 1252.264): `suppressed` to `claimed` 1251.907 (after_still_s 1.0 after the last command), `idle` 1252.834. The state reads `suppressed` while the manual look runs (Findings 11) |
| 10 mood locked, bumps | PASS | `calm --lock`; two `ros2 topic pub --once /stimuli ... bump` (seen on `/stimuli` 1293.871, 1304.614): `r2_affect` logs `stimulus bump from accept i=1.00: no change ... (locked, not applied)` twice, mood `calm` throughout, dome status `mood=calm`; manual `look 0.5` (1315.013) ran: `running`, joint 5.7725 (bearing 0.51), `claimed`, `idle` 1321.891 |
| 11 kill `r2_affect` | PASS (at the limit) | SIGKILL 1350.167, last `/mood/state` 1350.080, `dome stale mood=neutral` 1353.100 (3.02 s); that move (look 2.8, 3.1 s) had ended on its own at 1352.1, so repeated with a long one: `look 0 --source behavior --profile lazy` (1407.193), SIGKILL 1407.801, last mood 1407.730, `stale` 1410.734 (3.00 s), the command went to 0 at 1410.730 with the dome 0.69 rad short (held at 5.612), LED channels stale at 1410.79. Restarts: `idle mood=calm` 1370.938 / 1430.797; first glance `running src=idle act=glance` at 1390.947 (20 s after the first restart) |
| 12 kill `dome_arbiter` mid-move | PASS | `look 2.8 --profile smooth` (1469.713), SIGKILL 1470.980 (last `/dome/cmd/personality` 1470.980): `/dome_controller/commands` -1.0 until 1471.400, 0 at 1471.450 (0.47 s, `personality_timeout_s` 0.5), joint stopped 3.987. `dome` status stopped (last 1470.84, none in the next 5 s); `fld`, `magic`, `psi`, `rld` kept 2 messages a second (`cnt.log` in the transcript). Restart (`ros2 run ... -p use_sim_time:=true`): `up` 1486.968, `offline` 1486.971, `idle` 1487.001; no `/dome/cmd/personality` message in 20 s after; counters back to 0 |
| 13 stop `dome_state_broadcaster` | PASS | `--deactivate` at 1508.66, last joint state 1510.381 (a glance was running; the arbiter stopped it at 1510.768, 0.39 s later, `idle` 1510.924). `offline` 1515.503 (5.12 s, `device_stale_s` 5). `look 0` sent while offline (about 1528): `dropped` 0 to 1 and no message on `/dome/cmd/personality` or `/dome_controller/commands`. `--activate`: first joint state 1542.410, `idle` 1542.416 (CLI call to idle 11.1 s, the CLI's own latency) |
| 14 graph check | PASS | rclpy over 34 nodes: `/dome_controller/commands` published by `/dome_twist_converter` only; `/dome/cmd/personality` by `/dome_arbiter` only; `/dome/joint_command` by nothing; `/cmd_vel_dome` `/dome_teleop_node`, `/cmd_vel_dome_foxglove` `/foxglove_bridge_sim`, `/cmd_vel_joy` `/teleop_node`, `/diff_cont/cmd_vel` `/twist_mux`, `/diff_cont/odom` and `transition_event` `/diff_cont`. No node of the personality stack publishes `/cmd_vel*`, `/diff_cont/*` or `/dome/joint_command`. Run with every test probe stopped; the probes of items 5, 8 and 9 published on those topics while they ran (Findings 9) |
| 15 `dome_pushed` | PASS (no positive control) | the bag's `/stimuli` holds 15 messages in 1493 s: `being_driven` 12 (item 9), `bump` 2 (item 10), `stopped` 1; none between 160 and 215 (item 2) and no `dome_pushed` at all across about 20 personality moves of every profile. The sim never produced a push `dome_stimulus` could see, so absence is shown but not the detector firing |
| 16 CPU on the robot | robot only | not run |

Bag: 1493 s of sim time (90.1 to 1583.3), 99 903 messages, 4.3 MiB mcap with Zstd chunks
(rewritten from 39.4 MiB; under 20 MB, to be committed): `/diff_cont/odom` 43 914,
`/expression/status` 15 327, `/dome_state_broadcaster/joint_states` 14 324, `/mood/state` 8 189,
`/dome_controller/commands` 6 216, `/dome/cmd/personality` 6 180, `/dome/cmd/status` 3 090,
`/rosout` 2 627, `/expression/intent` 21 (the manual and behavior ones; idle glances are the
arbiter's own and not published), `/stimuli` 15.

## Findings

1. **The bag outgrew 20 MB and I missed the stop.** Raw it was 39.4 MiB at 1493 s, 30 Hz
   `/diff_cont/odom` being 44 000 of the 100 000 messages. Rewritten with `rosbag2_py`
   (reader to writer, `compression: "Zstd"` in the mcap storage config) to 4.3 MiB, every
   message kept. A longer run should drop `/diff_cont/odom` from section 9's list or record with
   the Zstd config. (`ros2 bag convert` produced an empty bag from this input.)
2. **Item 2's "one 0 on the command topic at the end" is not what happens.** After the target
   the arbiter holds the move for `settle_s` 0.8 s sending 0 at 20 Hz (19 zeros on
   `/dome/cmd/personality`), and the converter forwards zeros until its source goes idle (about
   1.3 s, 25 zeros on `/dome_controller/commands`). The dome is stopped throughout; the wording
   should say "zeros, then silence". Also `IDLE` comes 5.0 s after the move, the channel being
   `claimed owner=manual lease=5.0` (`manual_hold_s`) in between; item 2 and 5 should say "after
   the hold".
3. **The speed cap is read at start.** `ros2 param set /dome_twist_converter
   personality_max_speed_rad_s 1.0` is accepted but changes nothing. The cap run killed the sim's
   converter (SIGTERM), started it by hand with the same two parameter files plus `-p
   personality_max_speed_rad_s:=1.0`, and put it back with 2.0 afterwards. A real run of item 3
   in the sim wants `personality_max_speed_rad_s:=1.0` on the launch line.
4. **A refused dome intent is silent.** Item 6 expects `claimed by 'console' at priority 200`;
   `express` is fire-and-forget and the arbiter logs only accepted intents, so the refusal shows
   as `dropped` +1 at the intent's stamp (608.254 and 608.255). Idle glances dropped by a claim,
   the body or `offline` are counted the same way, so `dropped` can move without an intent
   (605.2 in item 6).
5. **Idle glances sit at the lower bound of `every_s`.** With a locked mood (strength 1.00) the
   gap from the end of one glance to the start of the next was 8.0-9 s for curious (8-20), 20 s
   for calm (20-60) and about 4 s for excited (4-10), never longer. Consistent with "strength
   scales `every_s` toward its lower bound"; the upper half of the ranges is not exercised by
   a locked mood.
6. **`turn --source behavior` is not a push.** It is a move; the rest is re-applied as it ends
   (1.4 s). The sim has no hand, so the push was injected on `/dome_controller/commands` from a
   probe (it breaks item 14's rule for 0.5 s; the bag shows those 10 messages as 0.7 with
   `/dome/cmd/status.source` idle). The correction came 17.7 s later: the recheck ticks every
   30 s from the last rest, so the latency is 0-30 s.
7. **Item 11 is at the limit again.** Stale at 3.02 s and 3.00 s after the last
   `/mood/state` with `mood_stale_s` 3.0 (phase 1's finding 5). A move shorter than about 3 s
   ends before stale and proves nothing: the second run used the lazy profile over 2.8 rad.
8. **An unlocked forced mood slides to sleepy within minutes** (item 4 ran under sleepy and the
   lazy profile). Items 5-7 and later used `--lock`.
9. **Probes publish on the guarded topics while they run.** Items 5 and 9 publish
   `/cmd_vel_dome`, `/cmd_vel_dome_foxglove` and `/diff_cont/cmd_vel` as the script says; item 8's
   push publishes `/dome_controller/commands`. The graph check ran after all of them had exited.
   The first stick of item 5 arrived 2.1 s after the move began (a fresh rclpy publisher's
   discovery, about 1.3 s beyond the 0.8 s asked); the probe waited for the move and the
   second attempt is the one tabled.
10. **Feedback loss mid-move (item 13, by chance).** A glance was running when the broadcaster
    stopped; the arbiter sent 0 0.39 s after the last joint state (`feedback_timeout_s` 0.5)
    and the dome coasted 0.27 rad. This is the `no feedback` end of 4.4 working.
11. **`dome` reads `suppressed` while a manual move runs** (item 9: `suppressed ... act=` is
    empty in the status although `look 0` was moving the dome; the only sign is the command).
    Cosmetic, but a console cannot tell "held off" from "held off and moving".
12. **Environment.** The sim ran at about real time here (sim seconds ~ wall seconds). Every CLI
    still took 10-14 s to exit (phase 1 finding 2); `mood` prints its traceback after
    `accepted:` as before. `pkill -f <pattern>` run from a bash whose own command line contains
    the pattern kills that shell: patterns were bracketed (`[l]ib/...`) or run from a script file.
    A stale `bash -c` from 2026-10-21 (pid 21544, mentions `r2_affect` in its text) is still
    running on the container; it is not a ROS node and was left alone. At the end the launch was
    SIGINTed, `gz sim` needed a SIGKILL, and `pgrep -af "gz sim|dome_arbiter|dome_mock|ros2 launch|r2_affect"`
    shows only that shell.
