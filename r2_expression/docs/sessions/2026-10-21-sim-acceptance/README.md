# 2026-10-21 sim acceptance run (phase 1)

Phase 1 acceptance script (r2_dev `docs/personality-phase1-plan.md`, section 9, items 1-15;
16 is robot-only) run against the headless simulator in the dev container with the shipped
`expression.yaml` and `affect.yaml`, on `main` at `49eccae`. The real workbench dome
`/r2_dome` was on the graph throughout: nothing called `/r2_dome/*` and nothing published to
`/dome/*`; the device was the sim's `/dome_mock` (arbiters with `device:=/dome_mock`, its
text topics under `/dome_mock/dome/text/*`). An agent drove the CLIs (`mood`, `express`,
`ros2 topic pub`) and read the mock's `mock/picture`, `leds/states` and `magic/states`;
nobody drove the Foxglove panels. Recorded on sim time with
`ros2 bag record --use-sim-time --include-unpublished-topics --topics /expression/intent /expression/status /mood/state /dome_mock/leds/states /dome_mock/magic/states /rosout`.
`rosout.txt` is `/rosout` as text (sim noise filtered); `transcript.txt` is the per-item
command output and the `/expression/status` change logs (sim stamps) the table cites.

| item | result | evidence |
|---|---|---|
| 1 fresh start | PASS | `fld`, `magic`, `psi`, `rld` (and `tfld`/`bfld`) `idle`, mood `calm`; picture: all displays `mode=random`, brightness 65, drift calm (the calm palette's top) |
| 2 `mood set excited` | PASS | within 2 s: brightness 100, drift lively on `rld`, `tfld`, `bfld`, PSIs; flourishes within 15 s: `magic` `running idle/sequence 3500ms` at +12 s, `fld` rainbow 4 s, both back to `idle` at their duration (3.6 s, 4.0 s) |
| 3 `mood set sleepy` | PASS | brightness 35 logics / 30 PSIs, drift calm; one in-flight `psi` clip finished, no new flourish in 20 s; hue is never sent (rule 1; the picture has no hue) |
| 4 `express effect fld fire --duration 5` | PASS | `fld` `running owner=manual active=manual/effect 5000ms` (`tfld`/`bfld` `claimed`), `claimed lease 5.0s` after the end, `idle` 5 s later; `psi` kept styling (idle clip ran meanwhile); `dropped` 0 to 1 on `fld` in a second 20 s manual run, `rld` untouched |
| 5 claim magic | PASS | `claimed owner=console prio=199 lease=60s`; `mood set excited --lock`: panel `run_counter` unchanged (7), brightness stays 1; `release magic` refused `no claim by 'express' on magic`; `--owner console` accepted, brightness 12 four seconds later. First attempt invalid (lease 30 s lapsed in the slow CLI), repeated with 60 s |
| 6 sequences end blank | PASS | `sequence magic 44 --duration 3`: seq 44 RUNNING then COMPLETE (`end` blank) after its 1 s; `magic 56 --duration 3`: RUNNING at 682.0, STOPPED at 685.0 (the arbiter's stop at 3 s) |
| 7 foreign LED write | PASS (note) | `express dome "leds run rld rainbow"`: `rld foreign owner=console`; `express effect rld fire` (sent within the hold) refused `run rld fire refused: held by console`, `refused` 0 to 1; `leds stop rld`, 5 s hold, `rld idle` with the palette (brightness 100, lively). `dropped` on `rld` was not observed: see Findings 3 |
| 8 foreign display change | PASS | `leds mode tfld off`: `fld` and `tfld` `foreign owner=console lease 4.1s` (not `bfld`); at +4.1 s both `idle` and the picture shows `tfld mode=random` again |
| 9 foreign Magic Panel | PASS | `magic_panel start 44`: `held_by console` seen at 1155.0, `magic foreign owner=console lease 4.5s`, seq 44 `started_by=console`; `idle` at 1159.5 after the hold |
| 10 text topic | PASS | `ros2 topic pub --once /dome_mock/dome/text/line0 ... hello`: `fld` (and `tfld`, `bfld`) `foreign owner=text_topic`, picture `fld running effect=text source=text_topic`; empty `line0`: `idle` at +4.1 s of hold; after `leds rostext off` the same publish changed nothing (`fld idle`, no run) |
| 10a sticky manual | PASS | `leds owner manual`: all six LED channels and `magic` `foreign owner=manual lease=forever`; `mood set excited --lock` left the logics at 65/calm; `effect fld fire` refused `held: manual` (`refused` 1); `--override` ran 4 s; `leds owner auto`: all `idle`, `rld` brightness 100 |
| 10b strip off | PASS | `leds enable off`: `fld`, `tfld`, `bfld`, `rld`, `psi` `dark` (`magic` stays `idle`), no running surface for 25 s while `dropped` rose (`psi` 4 to 7); `leds enable on`: `idle` and a flourish ran within 4 s |
| 11 lock | PASS | `mood set calm --lock`; two `bump` stimuli logged `(locked, not applied)`, mood stays `calm`, brightness 65; manual `fld` effect, `psi` effect and `magic` sequence 44 each `running owner=manual` |
| 12 kill `r2_affect` | PASS (at the limit) | SIGKILL: last `/mood/state` at 1633.641, every channel `stale mood=neutral` at 1636.645 (3.004 s, the `mood_stale_s` of 3.0 plus a status tick); picture brightness 40 calm; restart: `calm` palette (65) back, status `idle` at 1661.5 |
| 13 kill `led_arbiter` | PASS | SIGKILL while a manual `rld cycle` of 14 s ran: the mock showed `rld running effect=cycle:rainbow` at 0.7, 1.7, 2.7, 3.7 s elapsed, then `idle last_end=duration` by the end; `/expression/status` for the LED channels stopped (6 s: `{'magic': 11}`, 4 s: `{'magic': 7}`), `magic_arbiter` kept publishing; restart: `answered list; catalogue loaded`, all channels `idle`, palette reapplied (first run: excited 100 to sleepy 35 set while it was down, 35 on the picture after the restart) |
| 14 kill the mock | PASS | SIGKILL: last mock state 1923.0, every channel `offline` at 1928.0 (`device_stale_s` 5 s); 0 `list` log lines in 22 s (the arbiters do not re-list), no errors; restart: the first state message (1968.107) and every channel `idle` in the same status tick, palette in place |
| 15 graph check | FAIL as written; PASS reworded, on recheck | the clients of `/dome_mock/leds/*`, `/dome_mock/magic/*` are the two arbiters as expected, but `/foxglove_bridge_sim` (and `/foxglove_bridge` by `ros2 node info`) also lists a client on every one of them and on all 9 `/r2_dome/leds|magic/*` services: see Findings 1. No client other than the bridges. Recheck with the reworded item and the allow-list (`item15-recheck.txt`): `/foxglove_bridge_sim` holds the 9 `/dome_mock` clients and none on `/r2_dome`; only the arbiters and the two bridges are clients |

Bag: 1997 s of sim time (54.5 to 2051.6), 50 579 messages, 8.7 MiB mcap (under 20 MB,
committed): `/mood/state` 24 220, `/expression/status` 19 771, `/dome_mock/leds/states` 1 953,
`/dome_mock/magic/states` 1 953, `/rosout` 2 670, `/expression/intent` 12 (the manual ones;
flourishes are the arbiters' own and not published).

## Findings

1. **Item 15 fails as written: both Foxglove bridges are graph clients of the dome services.**
   `ros2 node info /foxglove_bridge_sim` lists Service Clients for all 9 `/dome_mock/leds|magic/*`
   and all 9 `/r2_dome/leds|magic/*` services (it lists clients for every service on the
   graph, 391 lines), and `/foxglove_bridge` does the same. rclpy
   `get_client_names_and_types_by_node` gave: `/dome_mock/leds/{configure,list,run,say,stop}`
   -> `/foxglove_bridge_sim`, `/led_arbiter`; `/dome_mock/magic/{configure,list,start,stop}` ->
   `/foxglove_bridge_sim`, `/magic_arbiter`; `/r2_dome/leds|magic/*` -> `/foxglove_bridge_sim`
   only. Nothing called them (the arbiters' `owner: r2_expression` is the only writer seen on
   the mock, and no `/r2_dome` state changed). Almost certainly the bridge creating a generic
   client per service it can expose to a Service Call panel; the script's check needs to say
   "other than the Foxglove bridges", or the bridge needs a service allow-list
   (`service_whitelist`) that leaves the dome out. A Service Call panel pointed at
   `/r2_dome/leds/run` would be a way for the sim's bridge to reach the real dome.
   **Resolved, rechecked 2026-10-20 23:13 UTC** (`item15-recheck.txt`): item 15 reworded to
   allow the bridges, and `sim_foxglove.launch.py` gives its bridge
   `service_whitelist: ['^(?!/r2_dome/).*']` (r2_sim d1ac845; r2_bringup 375e566 fixed the
   launch file's `value_type`, which until then stopped every bridge launched with it). With
   the live `/r2_dome` and `/foxglove_bridge` on the graph, `/foxglove_bridge_sim` lists
   clients on the 9 `/dome_mock/leds|magic/*` services and none on `/r2_dome/*`; the live
   bridge still holds all 18, as the item allows.
2. **Every CLI call takes 10-14 s.** After doing its job `express` and `mood` wait for the
   rmw_zenoh session to close: `Error closing session: close operation timed out ...
   Failed to close the session (Error code: -128)`. Consequences: the 5 s device hold lapses
   between two sequential CLI calls, so items 5 (a 30 s lease) and 7 (the refusal) needed the
   second command started in the background 1 s after the first, and the lease at 60 s (the
   maximum). `mood` additionally prints a traceback and exits 1 after `accepted: ...`
   (`mood_cli.py:142`, `rclpy.try_shutdown()` raising on `context.shutdown()`), which
   `ros2 run` reports as `Process exited with failure 1`. The accepted line is correct each
   time. Workaround: concurrent invocations, snapshots taken from rclpy watcher scripts
   (`watch.py`, `mgw.py`, in the transcript's `-w`/`-m` blocks) instead of `express status`.
3. **Item 7: a flourish dropped on `rld` was not observed.** The shipped palettes pick `rld`
   only for curious, playful, annoyed and scared, at weight 1 of 5-8 (the LED arbiter draws
   among its own picks). With `rld` foreign for about 80 s under `scared --lock`
   and 40 s under `excited --lock` no `rld` flourish came up, so `rld dropped` stayed 0. The
   mechanism is shown where the draw landed: `fld dropped` 0 to 1 during the manual run
   (item 4), `magic` 5 and `psi` 8 during the sticky-manual and dark periods (items 10a, 10b).
   Not a defect; to test it deterministically a mood with a heavier `rld` pick is needed.
4. **A forced mood decays by itself within about a minute** (`mood set excited` without
   `--lock` was back to `calm` at +45 s; a later `mood clear` reported `sleepy 0.02`, the idle
   slide). Items 5, 7, 8, 10a, 11, 12, 13 used `--lock` where the label had to hold.
5. **Item 12 is at the edge**: stale reported 3.004 s after the last mood message with
   `mood_stale_s: 3.0` and a 2 Hz status tick; passes "within 3 s" only by 4 ms of rounding.
   A run on a loaded host can read 3.5 s. Consider wording "within 4 s" or a smaller
   threshold.
6. **`express status` (one shot) shows 4 channels, `--watch`/the topic shows 6**:
   `tfld` and `bfld` are published by the arbiter but the first snapshot after a quiet
   period lists only `fld`, `magic`, `psi`, `rld` (item 1 text says four channels; the topic
   has six). Cosmetic.
7. **Item 13 first attempt** killed the arbiter at 382 ms before a 4 s flourish ended (the
   status tick lags), which proves little; repeated with a 14 s manual `rld cycle`, the
   result in the table. Both attempts ended by the device's duration.
8. **Item 6's blank is read from the state topic** (`end` 1, COMPLETE/STOPPED); the picture
   has no lamp pattern for the Magic Panel, only `magic: link state sequence ...`.
9. **Item 14**: the arbiters log nothing on loss or return beyond `offline`/`idle` and
   `answered list; catalogue loaded` at start-up; after the device returns they recover from
   its state topic without a new `list`. So "no retry storm" is shown by zero `list` lines,
   not by a 5 s cadence.
10. **Environment**: `/foxglove_bridge` was on the graph at the start and gone at the end
    (no such process was running at the end; I started and stopped only `foxglove_bridge_sim`
    via the sim launch). A stale `ros2 topic echo --once /dome_mock/mock/picture` (pid 74255,
    from before this run) was killed. `Velocity command timed out. Braking.` floods `/rosout`
    from the sim's twist mux (filtered out of `rosout.txt`). The background nodes started with
    `nohup` ignored SIGINT; they were stopped with SIGTERM/SIGKILL.
