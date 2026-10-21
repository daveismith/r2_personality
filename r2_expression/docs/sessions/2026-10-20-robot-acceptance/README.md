# 2026-10-20 robot acceptance run (phase 1)

Phase 1 acceptance script (r2_dev `docs/personality-phase1-plan.md`, section 9) on the robot:
`launch_robot.launch.py` on the body Pi (`/home/pi/r2_ws`, r2_expression code as at
`49eccae`), against the **bench dome controller** as `/r2_dome`, not the droid's own dome
(it was not connected). The LEDs were watched in the dome's web GUI, which cannot run
alongside the bench's Magic Panel simulator, so the panel was absent (`link` 0, `magic`
`offline`) and the Magic Panel items were not run. David watched the GUI, typed on the dome
console and ran the kills and restarts on the Pi; Claude drove the CLIs (`mood`, `express`,
`ros2 topic pub`) from the dev container through the linked Zenoh routers. Recorded on wall
time from the container with
`ros2 bag record --include-unpublished-topics --topics /expression/intent /expression/status /mood/state /r2_dome/leds/states /r2_dome/magic/states /rosout`
(`bag/`, 23:38:04 to 01:02:35 UTC, 105 571 messages, 18.8 MiB). Times below are UTC.

| item | result | evidence |
|---|---|---|
| 1 fresh start | PASS (LEDs) | `fld`, `rld`, `psi` `idle` on `calm`; displays brightness 65, drift calm, `changed_by_owner: r2_expression`; `magic` `offline` (no panel) |
| 2 `mood set excited` | PASS (LEDs) | within 1 s: brightness 100 and drift lively on `rld`, `tfld`, `bfld`, both PSIs at 100; flourishes `flash` and `rainbow` on `fld`, each ended at its duration; David: livelier, brighter, rainbow seen on the front logic. Magic Panel flourish not run |
| 3 `mood set sleepy` | PASS | logics 35, PSIs 30, drift calm, hue 0 unchanged; no flourish started in 45 s (the tail of an excited flash ran out on time); David: dimmer, slower, no effects |
| 4 `express effect fld fire --duration 5` | PASS | `fld` `running` `manual` prio 199 5000 ms, then `claimed` 5 s, then `idle`; `rld`, `psi` stayed `idle` on sleepy; David: fire on the front logic only, the rest dim and calm. No flourish arrived during the claim (sleepy), so the drop count was not exercised (it was in the sim) |
| 5 claim `magic` | NOT RUN | no Magic Panel |
| 6 sequences | NOT RUN | no Magic Panel |
| 7 foreign LED write | PASS (second attempt) | console `leds run rld rainbow`: `rld` `foreign owner=console lease 4.4s`; `express effect rld fire` sent 0.9 s later (triggered on the status) refused, `refused=1`; one flourish dropped while foreign, `dropped=1`; after `leds stop rld`: `idle`, scared palette (85, lively). First attempt: see Findings 2 |
| 8 foreign display change | PASS | console `leds mode tfld off`: `fld` `foreign owner=console lease 4.3s`, then `idle`; `tfld` back to mode 0 (random), drift lively, brightness 90, `changed_by_owner: r2_expression` |
| 9 foreign Magic Panel | NOT RUN | no Magic Panel |
| 10 `/dome/text/line0` | PASS | `hello`: `fld` `foreign owner=text_topic`, device `fld` running `text` with `source: text_topic`; cleared with `''`: `last_end: stopped`, `idle` after the hold. With `leds rostext off` the same publish: `fld` stayed `idle`, no run |
| 10a sticky manual | PASS | `leds owner manual`: every LED channel `foreign owner=manual lease=forever`, `manual: true`, `hold_remaining_ms` 4294967295; `mood set excited --lock` left the scared 85/90; `express effect fld fire` refused (`refused` 0 to 1); `--override` (200) ran 4 s; `leds owner auto`: all `idle` |
| 10b strip off | PASS | `leds enable off`: LED channels `dark`, `strip_enabled: false`, no surface running, flourishes dropped (`psi` `dropped` 3 to 5); `leds enable on`: `idle`, excited palette back (David: looks excited) |
| 11 lock calm | PASS | `mood set calm --lock`, two `bump` stimuli: mood stayed `calm`, `locked: true`, brightness 65 calm; manual effects ran on `fld`, `rld`, `psi` |
| 12 kill `r2_affect` | PASS (at the limit) | SIGKILL on the Pi: last `/mood/state` 23:53:18.225, `stale mood=neutral` 23:53:21.320 (3.1 s: `mood_stale_s` 3.0 plus a status tick, as in the sim); `ros2 launch r2_affect affect.launch.py`: first `/mood/state` 23:53:59.266, `idle` on `calm` 5 ms later (the lock is gone after a restart, as designed) |
| 13 kill `led_arbiter` | PASS | 25 s manual `rld cycle` started 23:56:45; killed about 23:56:49: LED-channel status stopped, `magic_arbiter` kept 2 Hz; the device ran `roam`, `rainbow`, `fire` and ended `last_end: duration` at 23:57:10. Restart (`ros2 run r2_expression led_arbiter --ros-args -p device:=/r2_dome`) 23:58:57.986: `offline` until `answered list; catalogue loaded` (0.22 s), `idle` at 23:58:58.827, palette by `r2_expression` (58: calm at a lower `mood_strength` after the item 12 restart; brightness scales with strength, `palette.scale_brightness`) |
| 14 stop the device | PASS | `ros stop` on the console: last `/r2_dome/leds/states` 00:00:08.811, every LED channel `offline` 00:00:13.861 (5.05 s, `device_stale_s` 5); no `list` attempt and no error in the 72 s outage; `ros start`: first state 00:01:21.321, `idle` 00:01:21.333. The device took some seconds after `ros start` to reach the router |
| 15 graph check | PASS | `item15-graph.txt`: `/r2_dome/leds|magic/*` clients are `/led_arbiter` (5), `/magic_arbiter` (4) and the live `/foxglove_bridge` (9 of its 18); no other node |
| 16 load | **FAIL** (CPU); PASS (rate) | `ps -eo pcpu,etime,args` on the Pi: `magic_arbiter` 2.6 % over 1:29:35, `led_arbiter` 2.8 % over 1:03:02 (the `ros2 run` wrapper 0.0), together **5.4 %**, target 3 %. `/dome/joint_states` 10.0 Hz with both arbiters, 10.0 Hz with `led_arbiter` down. See Findings 1 |

## Findings

1. **Item 16 fails: the two arbiters cost 5.4 % of a Pi core, target 3 %.** Lifetime
   averages over an hour that was mostly idle, so close to the steady state. `magic_arbiter`
   costs nearly what `led_arbiter` does (2.6 against 2.8 %) with no panel to drive and its
   channel `offline`, so the cost looks fixed per node (the 2 Hz status timer, the
   `/mood/state` subscription at the affect rate, the 1 Hz dome state topics, rclpy
   executor overhead) rather than work. Phase 0's item 14 failed the same way (6.8 % for
   four processes). Not investigated here.
2. **The device hold protects a person for 5 s after their last write, not for the run.**
   First attempt at item 7: the effect was sent after David replied, well past the hold;
   `rld` still showed `foreign owner=console lease=0.0s` (the arbiter waits for a run it
   did not start), but the manual intent at 199 was sent and the device accepted it,
   replacing the console's rainbow. That follows the rules (the arbiter holds back only
   its own idle intents; the device gates by the hold), but it means a long console run is
   unprotected from an operator's `express` once the person stops typing. The second
   attempt, triggered on the `foreign` status, was refused as the item expects.
3. **The CLI is fast here.** `mood` and `express` returned in 0.6-1 s each; finding 2 of
   the sim note (10-14 s for the zenoh session close) did not reproduce, which supports
   it being environmental.
4. **`express` and `mood` are not on `PATH`**: they are under
   `install/r2_expression/lib/r2_expression/` and `install/r2_affect/lib/r2_affect/`, so
   `ros2 run r2_expression express` (as section 5.5 says) or the full path.
5. **`/foxglove_bridge` still listed clients on the nine `/dome_mock/*` services** an hour
   after the sim and its mock had stopped. Harmless (nothing serves them), likely the
   bridge not dropping clients for services that went away.
6. **Not covered by this run**: the droid's own dome (step 0's droid part: firmware,
   router, catalogue capture), the Magic Panel (items 5, 6, 9 and the panel parts of 2
   and 13), and the palettes seen on a physical strip (the web GUI only).
