# 2026-11-18 robot acceptance run on the droid's own dome (phase 1)

Phase 1 acceptance script (r2_dev `docs/personality-phase1-plan.md`, section 9) on the robot,
for the first time against **the droid's own dome and Magic Panel**: `launch_robot.launch.py`
on the body Pi (`/home/pi/r2_ws`; `r2_personality` `a191d84`, `r2_bringup` `f33a090`,
`astromech_msgs` `162a100`, all `phase2`), the dome MCU (`esp32p4`) on USB-NCM to the dome Pi
and through the slip ring to the body's router at `fd50:43e1:cede::1`. The 2026-10-20 run was
against the bench dome with no panel, so items 5, 6 and 9 and the panel parts of 2 and 13 are
new here; the rest is a re-run on the real strips.

David watched the dome from the droid's right side; Claude drove everything else from the dev
container: `mood` and `express`, the dome console (over SSH to the dome Pi and its serial
port, `con.sh`), and the kills and restarts on the body Pi. The dome was turned by
`express look` and held with a renewed `express claim dome` so the surface under test faced
David. `probe.py` logged changes (`probe.log`); the bag (`bag/`) holds `/expression/intent
/expression/status /mood/state /r2_dome/leds/states /r2_dome/magic/states /rosout
/dome/text/line0 /dome/text/line1 /stimuli`, recorded on wall time from the container,
04:06:55 to 04:50:22 UTC (46 571 messages, 8.9 MiB). Times below are UTC on 2026-11-18.

| item | result | evidence |
|---|---|---|
| 1 fresh start | PASS | `fld`, `rld`, `psi`, `magic` `idle` on `calm`; logics drift calm at 65, PSIs 65, `changed_by_owner: r2_expression`; panel link 1, fw 0.12.1, nothing running. David: logics lit and drifting slowly, PSIs lit, panel dark |
| 2 `mood set excited` | PASS | drift lively and brightness 100 on the three logics and both PSIs 1.0 s after the mood; `fld` flourish (effect) at 7.1 s for 4.0 s, `psi` flourish (clip) at 17.1 s for 1.25 s, each ended at its duration. David saw all three. Panel part: mood flourish sequences ran on the real panel (3.5 s each, 04:11:10, 04:12:58, 04:20:56) |
| 3 `mood set sleepy --lock` | PASS | logics 35, PSIs 30, drift calm, hue 0. David: dimmer, otherwise the same; the logics are still lit at 35 on the real strips |
| 4 `express effect fld fire --duration 5` | PASS | `running` `manual` 199 for 5.0 s, `claimed` 5.0 s, `idle`; under `excited --lock` one `fld` flourish was dropped during the run (`dropped` 0 to 1; 3 after the second run); `rld`, `psi` not claimed. David saw the fire once the front faced him (second run) |
| 5 claim `magic` | PASS | `claimed owner=console` 30 s; 21 s of `excited --lock` started no sequence; `express release magic` refused `no claim by 'express' on magic`; `--owner console` accepted, `idle` |
| 6 sequences | PASS | 44 Smile `--duration 3`: device `state` 1 then 2 (ended by itself after about 1 s), channel `claimed` 5 s, `idle`. 56 Random show `--duration 3`: stopped by the arbiter at 3.0 s (`state` 3). David, panel facing him: smile, blank, then a busy pattern that stopped and left the panel dark |
| 7 foreign LED write | PASS (second attempt) | console `leds run rld rainbow`: `rld` `foreign owner=console lease 4.2s`; `express effect rld fire` 40 ms after the status refused `held by console` (`refused` 0 to 1); `leds stop rld`: `idle` and the palette 4.3 s later. No `rld` flourish came up to be dropped (excited does not pick `rld`). First attempt: the effect went 16 s after the rainbow and was accepted (finding 2 of the 2026-10-20 note) |
| 8 foreign display change | PASS | console `leds mode tfld off`: `fld` and `tfld` `foreign owner=console lease 4.6s`, `idle` 4.6 s later, `tfld` back to mode 0 by `r2_expression` |
| 9 foreign Magic Panel | PASS | console `magic_panel start 44`: device `source` 1, `held_by: console`; `magic` `foreign owner=console`, `idle` 5 s after the image ended. A mood flourish had started in the same second and lost to the console |
| 10 `/dome/text/line0` | PASS | `hello`: `fld` `foreign owner=text_topic`, both front logics running; cleared with `''`: `idle` 4 s later. With `leds rostext off` the same publish changed nothing |
| 10a sticky manual | PASS | `leds owner manual`: `manual: true`, every LED channel `foreign owner=manual`, lease forever (the console lists `magic` as held too); `mood set sleepy --lock` left the excited 100/lively; `express effect fld fire` refused `held: manual` (`refused` 0 to 1); `--override` (200) ran 4 s; `leds owner auto`: all `idle` |
| 10b strip off | PASS | `leds enable off`: `strip_enabled: false`, LED channels `dark` for 28 s under `excited --lock`, no run sent; `leds enable on`: `idle`, a flourish 1.6 s later |
| 11 lock calm | PASS | `mood set calm --lock`, two `bump` stimuli: `r2_affect` logged `locked, not applied`, mood stayed `calm`; manual `fire` ran on `fld`, `rld`, `psi` and Smile on `magic` |
| 12 kill `r2_affect` | PASS (at the limit) | SIGKILL sent 04:25:43.89 (through SSH); all four channels `stale mood=neutral` 04:25:47.27 (3.4 s including the SSH round trip), `rld` to 40; restarted by hand 04:25:56.9: first `/mood/state` and `idle` on `calm` 04:25:58.98, `rld` back to 65 |
| 13 kill `led_arbiter` | PASS | 20 s manual `rld cycle` started 04:26:17.49, arbiter killed 04:26:20.9: the device ended the run at 04:26:37.7 (its duration); restart 04:26:45: `offline`, `answered list; catalogue loaded`, `idle` 0.5 s after it came up, the nine missing-clip WARNs again. Panel part: see finding 1 |
| 14 stop the device | PASS | console `ros stop` 04:28:42.0: every channel `offline` 04:28:47.7 (5.7 s), nothing logged by either arbiter in the 43 s outage; `ros start` 04:29:25.1: `idle` 04:29:27.2 |
| 15 graph check | PASS | `item15-graph.txt`: the clients of `/r2_dome/leds/*` are `/led_arbiter` and `/foxglove_bridge`, of `/r2_dome/magic/*` `/magic_arbiter` and `/foxglove_bridge`; no other node |
| 16 load | PASS (3.20 %, accepted); PASS (rate) | `/proc/<pid>/stat` on the Pi over a quiet 600 s window (04:39:38 to 04:49:38, calm, unlocked): `led_arbiter` 1.71 %, `magic_arbiter` 1.49 %, together **3.20 %** of a core against the 3 % target of the day; David made 3.20 % the budget the same day (`dome_arbiter` 3.04 %, `r2_affect` 0.77 % for reference). `/dome/joint_states` 10.0 Hz. See finding 7 |

## Findings

1. **With `magic_arbiter` dead, a looping panel show does not end by itself.** Extra to item
   13: `express sequence magic 56 --duration 15` started 04:27:06, `magic_arbiter` killed
   04:27:09. The LED side of this case ends on the device's own duration; the Magic Panel has
   no such limit for a looping sequence, so the random show ran on (iteration 1 and counting)
   for the 44 s the arbiter was down. The restarted arbiter adopted it (`active_source:
   adopted`, 30 s) and stopped it at 04:28:23.9. So the panel is tidied up when the arbiter
   returns, not before. David, 2026-11-18: fine as it is.
2. **The droid's dome has none of the palette clips.** `led_arbiter` drops nine flourish picks
   at start (`hscan`, `vscan`, `colourswap`, `flipflop`, `flipflop-alt`, `alarm`,
   `alarm-twinkle`, `fadescroll`), as the 2026-11-10 catalogue capture predicted. Calm,
   excited and sleepy, the moods judged by eye here, do not use them. Curious, playful,
   annoyed, scared and sulking were not looked at and should wait for the clips.
3. **The device hold is 5 s after the last console write**, as on the bench (item 7's first
   attempt).
4. **Item 10b: no drop was counted while dark.** In 28 s under `excited --lock` no flourish
   was counted in `dropped` on any LED channel (on the bench `psi` went 3 to 5). No run was
   sent, which is what the item asks; whether the scheduler skips or drops while dark was not
   pinned down.
5. **`ros2 service info` has no `-v` in Jazzy**, so item 15 used
   `get_client_names_and_types_by_node` from a short rclpy script.
6. **Getting to the dome.** The MCU's console is the dome Pi's
   `/dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_123456-if00` (prompt
   `esp32p4>`; user `davids` on the dome Pi, in `dialout`, `picocom` installed). The MCU has
   no network console. The dev container reaches the body's router at
   `tcp/[fd50:43e1:cede::1]:7447` (`.env`); the old `r2-body.davidiansmith.ca:7447` is
   refused because the boot-time `zenohd.service` listens only on `::1` and that address.

7. **Item 16: the two arbiters cost 3.20 % of a core against a 3 % target; accepted.** The bench run measured
   2.74 % after its fix, with no panel (`magic` `offline`). Here the panel is live, so
   `magic_arbiter` handles a real 1 Hz state and runs flourishes; it is the likely source of
   the extra 0.5 %, though that was not measured. The first attempt at this item timed the
   `bash` wrappers of the hand-restarted nodes (0 %) and was discarded; lifetime averages over
   the 12 minutes before the window, start-up and items 13 and 14 included, were 1.8 and 1.7 %.

## Not done

- The five moods whose flourishes need the missing clips, by eye.
