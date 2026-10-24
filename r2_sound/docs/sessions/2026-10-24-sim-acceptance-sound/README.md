# 2026-10-24 sim acceptance run (phase 3, sound channel)

Phase 3 acceptance script (r2_dev `docs/personality-phase3-plan.md`, section 9, sim column,
items 1-12 as asked, plus the sim-testable 13, 14 and 15; 16 is robot-only) run by an agent
against the headless simulator in the dev container (`ros2 launch r2_sim sim_foxglove.launch.py`,
`sound_backend:=wav`, shipped `expression.yaml`, `affect.yaml`, `clips.yaml`: 223 clips) on the
`phase3` branches. Branch heads: `r2_personality` 4f48aaf, `r2_bringup` 1de86f5,
`r2_sim` 4253d94, `astromech_msgs` 6657953 (all on `phase3`); `r2_dev` (root) 9a17768.
Router in the sidecar, `rmw_zenoh_cpp`, no Zenoh variable exported. The robot was unreachable;
the real workbench `/r2_dome` was on the graph (it subscribes `/dome/audio/level`, which in the
sim has no publisher: the player writes `/dome_mock/dome/audio/level`) and nothing called
`/r2_dome/*`. A stray `/dome_mock_t73790_1` (pid 73809, from an earlier session, not mine) was
on the graph throughout (a leftover of an earlier test run; killed after this session). `R2_PHASE3_FULL=1` is named in the plan (step 8a)
but nothing in the code, tests or launch reads it, so it was not set (no node or backend tests
were run; this is the stack only). The sim ran at a real-time factor of 0.83, and the arbiters
measure `mood_stale_s`, `device_stale_s` and the claim leases in sim time, so those read about
20 % longer on the wall clock below.

How: the stack was launched once, stopped, and launched again with the bag and a probe already
running, so item 1's startup messages are in both. `express` was the CLI for intents (it exits
in about 1 s here, not 25 s); `ros2 run r2_affect mood set <label> --lock` for the mood; one
rclpy probe (`probe.py`, wall-clock stamps) logged every `/sound/level`, `/dome_mock/dome/audio/level`
and `/sound/playing` message and every change of `/sound_player/state`, the sound channel of
`/expression/status`, the mood label and the sound nodes' `/rosout`; a second probe (`pre.py`)
published the item 6 intents the moment an idle play was running. Times below are wall epoch
seconds modulo 1000 (`832.616` is 1792859832.616; they wrap at 1000 because the items run in
order over 1 640 s, so a smaller number after a larger one is the next thousand); the WAV file
names carry the same moments in UTC (epoch 1792859832 is 16:37:12).
Bag: `ros2 bag record -o .../2026-10-24-sim-acceptance-sound-bag --include-unpublished-topics
--topics /expression/intent /expression/status /mood/state /sound_player/state /sound/level
/sound/playing /dome/audio/level /dome_mock/dome/audio/level /rosout` (section 9's list, plus
the sim's level topic; `/dome/audio/level` has no publisher in the sim, so it recorded nothing).
The player's WAVs are in `/ws/.audio/out/` (bind-mounted: `.audio/out` in the workspace on the
Mac); the stamps are UTC, the session ran from 16:35 to 17:05.

| item | result | evidence |
|---|---|---|
| 1 fresh start | PASS | at startup (741.5-741.6) `/sound/level` 0.0, `/dome_mock/dome/audio/level` 0, `/sound/playing` false, `/sound_player/state` `IDLE` `backend=wav`, `last_end 0`; `sound_player up: 223 clips ..., backend wav (/ws/.audio/out)`; `sound` offline 740.67, `idle` 741.54 with `mood=calm`. `express sounds`: 223 clips, 12 with `excluded  (manual only)` (`music-2secofsilence`, `music-cantina`, `music-march`, `padawan-theme` among them). "No stream open" has no sim meaning: the wav backend has none |
| 2 `play padawan-scream02` | PASS (audible: David's) | intent at 831.99; `running (manual)` 832.611; `/sound_player/state` `PLAYING` `owner r2_expression` `priority 199` 832.616, `elapsed_ms` 0, 200, 433, 633 ... 2033 (5 Hz, 200 ms steps), play false and `IDLE last_end 1` at 834.689 (2.073 s wall for a 2.043 s clip); `sound` `RUNNING` then `claimed owner=manual lease=5.0` 834.692, `idle` 840.704 (6.0 s wall = 5.0 sim s). WAV `20261024T163712.629Z-padawan-scream02.wav`: 2043.5 ms (clip 2043), peak -4.15 dBFS = catalogue peak -0.3 + `gain_db` -3.85, RMS -15.25 dBFS (whole file) |
| 3 level | PASS | 62 `/sound/level` and 62 dome messages in the play: median gap 34.0 ms, 29.6 Hz; level 0.000 to a maximum 0.308 following the clip (quiet lead-in, a burst, a decay); dome byte maximum 223, and the byte equals `round((20 log10 level + 40) / 34 * 255)` on all 62 pairs (max difference 0). Both end with a 0 at 834.689, the same instant as `/sound/playing` false (last real block 834.681): within 50 ms. `/sound/playing` true 832.616, false 834.689. The mock's picture (`/dome_mock/mock/picture`, 1 Hz, full-length echo) read `level=200`, `level=189`, then `level=0` |
| 4 `--pool ooh` x3, `nosuch` | PASS (notes) | three plays picked `music-85` (700 ms), `padawan-ooh07`, `padawan-ooh02`: all tagged `ooh`, all different. `express play --pool nosuch` never reaches the arbiter: `express` refuses it (`unknown tag 'nosuch'`, rc 2). A raw `ExpressionIntent` `{"pool": "nosuch"}` moved `dropped` 2 to 3; on the player `ros2 service call /sound_player/play {pool: nosuch}` answers `unknown pool`, `{pool: excited}` answers `empty pool`. `--pool excited` and `--pool question` through `express`: sent, `dropped` 0 to 1 and 1 to 2, nothing played. The arbiter logs no reason for a drop (Findings 2). Plan says 7 `ooh` clips; the catalogue has 16 (`padawan-ooh01-07` and nine `music-*`, see Findings 1) |
| 5 loudness | PASS (levels); FAIL (scream vs hum) | the plan's "three" sets are two (`R2 Padawan`, `R2 sounds and music`). Mid-level clips nearest each set's median: `padawan-sent04` (RMS -12.32 in the file) WAV `...T163949.228Z-padawan-sent04.wav` RMS -16.71 dBFS whole file / -15.27 non-silent; `music-33` (-9.41) WAV `...T163959.482Z-music-33.wav` -17.37 / -14.94: 0.7 dB apart (whole file), 0.3 dB (non-silent), inside +-2 dB. `padawan-scream02` WAV `...T164009.425Z-padawan-scream02.wav` -15.25 / -14.53 and `padawan-hum19` `...T164020.223Z-padawan-hum19.wav` (379 ms) -16.56 / -11.64: the scream is 1.3 dB louder over the whole file and 2.9 dB quieter over the non-silent part. Not "at least 6 dB". Both clips take the same set gain (-3.85) and their file RMS differs by 1.64 dB the other way (`rms_dbfs` -10.87 and -9.23), so the gain preserves the set's dynamics exactly; the plan's expectation is about the sound, not the numbers (Findings 3). Peaks both -4.15 |
| 6 pre-emption | PASS | long clip: `padawan-sent08` (3.664 s, the longest Padawan `SENT`; none is 4 s) at 54.786, then `padawan-scream02` accepted 56.164, its `PLAYING` state `last_end 4` at 56.340 (the CLI needs 0.8 s per call, so the second came 1.4 s in, not 1.0 s). WAV `...T164054.798Z-padawan-sent08.wav` is 1386.7 ms: the last 50 ms of it fall from 0.31 to 0.157, 0.056, 0.009, 0.001 in 5 ms steps, a fade of about 20 ms (`fade_ms` 20); `...T164056.304Z-padawan-scream02.wav` 2043.5 ms. Idle vs behaviour: under `excited --lock` an idle `padawan-chortle` (2656 ms, `chatter-7`) started 108.121; the behaviour intent `pool chatter`, priority 10, published at 108.124 (`pre.py`, which is why it is not an `express` call): `dropped` 3 to 4 at 108.126, the chortle ran to its end 110.815 (2.69 s) and `last_end 1`. At 145.119 another idle chortle; manual `play padawan-hum19` at 145.121: `padawan-hum19` `PLAYING` with `last_end 4` at 145.184 (63 ms after the intent), WAV `...T164225.119Z-padawan-chortle.wav` is 53 ms long (cut with the fade, peak -56 dBFS: the chortle starts quiet), `...T164225.181Z-padawan-hum19.wav` 379 ms |
| 7 duration | PASS (note) | `express play padawan-theme --source behavior`: no play, `dropped` 4 to 5 (the refusal reason is not logged; Findings 2). `express play music-44 --source behavior --duration 5`: `PLAYING` prio 100, `rem 5000` at 184.914; state `IDLE last_end 2` at 189.962 (5.048 s); WAV `...T164304.948Z-music-44.wav` exactly 5000.0 ms (cut and fade inside the 5 s; clip 10 852 ms). `express play padawan-theme` (manual, uncapped): `PLAYING` `rem 329953`, `lease=335.0` 195.185; `express stop sound` sent 204.166 (9.0 s in): state `FADING el 9000 rem 20` 204.944, `IDLE last_end 3` 205.013; WAV `...T164315.953Z-padawan-theme.wav` 9020.0 ms. The plan's "stop it at 10 s" was done at 9 s (the CLI's 0.8 s) |
| 8 chatter per mood | PASS (audible: David's) | `excited --lock`: first idle play 8.3 s after the set, 11 plays in 90 s, start-to-start gaps 7.2-8.2 s (`every_s` 6-15, sitting at the low end as in phase 2's finding 5); clips: `padawan-chortle` (laugh), `music-48`, `music-79`, `music-107`, `padawan-sent03`, `padawan-sent13` (chatter), four `voice:*` (vocalize): the fallback pools laugh + chatter + voice as the plan says. `sleepy --lock`: two plays in 270 s, at +104.8 s and +208.5 s (103.7 s apart; `every_s` 90-240), both `hum` clips (`music-136`, `padawan-hum20`), peaks 8.00 dB under the catalogue's (gain -8). `sulking --lock`: plays at +71.7, +140.9, +208.4 s (gaps 69.3 and 67.5; `every_s` 60-180), `music-96`, `music-94`, `music-36`, all from the `chatter` pool (the `sad` pool dropped with a WARN at startup). Mood gains in the WAV peaks vs the catalogue's peak + gain: excited +2.00 dB, sulking -4.00, sleepy -8.00, calm (the first minutes, unlocked) -3.00. Each `every_s` respected. For David to audition: `20261024T164140.401Z-voice-776688483`, `164148.137Z-padawan-chortle`, `164156.395Z-voice-625565803`, `164203.632Z-padawan-chortle`, `164210.629Z-voice-1196699721`, `164217.885Z-voice-86050095`, `164233.370Z-padawan-chortle`, `164240.883Z-voice-1616164203` (excited, first pass); `164404.872Z-padawan-chortle` to `164529.878Z-voice-1047790026` (excited, the timed window: `music-48`, `voice-2090342774`, `music-79`, `music-107`, `voice-949102746`, `padawan-chortle`, `padawan-sent03`, `padawan-chortle`, `padawan-sent13`, `music-48`); `164718.873Z-music-136`, `164902.625Z-padawan-hum20` (sleepy); `165130.604Z-music-96`, `165239.859Z-music-94`, `165347.359Z-music-36` (sulking). The mood pools (`excited`, `sad`, `sleepy`) are empty as the plan says: WARN "no playable clip tagged ..." for `question`, `excited`, `scared`, `sleepy`, `sad` |
| 9 claim | PASS (notes) | `claim sound --owner console --lease 120 --override`: `claimed owner=console prio=200`, but the lease is 60.0 s (the arbiter caps it; with `--lease 30` it stays 30.0). For 34 s under `excited --lock` no idle play started (chatter is due every 7-8 s) and `dropped` did not move: the chatter slots are skipped, not counted (Findings 5). `express play padawan-scream02` (199): `dropped` 6 to 7, nothing played. `play padawan-scream02 --override` (200): `running` with `owner=console prio=200`, `PLAYING` prio 200 at 996.448, WAV `...T165636.432Z-padawan-scream02.wav` 2043.5 ms. Release (second run, `--lease 40`, `release sound --owner console` at 1080.3): `idle` 1081.17, first chatter 1087.84 (6.7 s later, inside `every_s`). The first run's lease ran out before I released (a "no claim" exit 1), which is why there are two runs; that run's override scream is `...T165455.498Z-padawan-scream02.wav` |
| 10 bumps | PASS | `excited --lock`, three `bump` stimuli (intensity 1.0) at 123.6, 129.3, 134.9: `r2_affect` logs `no change -> aro=0.80 val=0.80 ... (locked, not applied)` each time, `/mood/state` `excited` throughout, the `sound` status `mood=excited`; 7 idle plays in the next 45 s (`padawan-chortle`, `music-28`, `music-119`, `music-43`, `padawan-chortle`, `padawan-chortle`) with +2.00 dB mood gain in their WAVs (`...T165847.592Z-padawan-chortle.wav` peak -2.16 vs -4.16). No sleepy or scared voice |
| 11 kill `r2_affect` | PASS (at the limit) | `excited --lock`, idle `music-44` (10.9 s, `express play music-44 --source idle`) started 190.937; SIGKILL 192.88 (the last `/mood/state` in the bag 192.72; sim stamp 1199.53); `sound stale mood=neutral` 196.584 (3.7 s after the kill, 3.86 s after the last message, about 3.2 sim s at 0.83, `mood_stale_s` 3.0: Findings 7), and in the same instant the play went `FADING` at `elapsed_ms 5667` and `IDLE last_end 3`. WAV `...T165950.933Z-music-44.wav` 5687 ms (fade included). No chatter while stale. Manual `play padawan-hum19` at 205.5 during stale: played (`mood=neutral`), WAV `...T170005.530Z-padawan-hum19.wav` peak -4.15, the catalogue's alone (no mood gain). Restart (`ros2 run r2_affect r2_affect --ros-args -r __node:=r2_affect --params-file <the launch's file>`): `/mood/state` back 238.8, `sound idle` 238.96; `excited --lock` at 246.4, first chatter 253.09 (`voice:1022938107`, WAV `...T170053.119Z-voice-1022938107.wav`) |
| 12 kill `sound_player` | PASS (note) | `music-44` manual playing from 295.05, SIGKILL 298.49: no WAV written (nothing at ~17:01:35 in `.audio/out`); the last dome byte before the kill was 0 (a quiet block, 0.0017), so the "level stays up" case was not shown: Findings 8. `sound` went `offline (manual)` 304.577, 6.08 s after the kill (`device_stale_s` 5 sim s). The launch does not respawn the player (`personality.launch.py` has no `respawn`); I restarted it by hand with the launch's parameter file (`ros2 run r2_sound sound_player --ros-args -r __node:=sound_player --params-file ...`) at 310.5: at 313.421 `/sound/level` 0.0, dome 0, `/sound/playing` false, `IDLE last_end 0`; `sound_arbiter`: `/sound_player is back after 15 s: listing again`, `sound: idle`. A second kill with the player idle (354.2): `offline` 359.775 (5.7 s), restart 364.424, same four messages. Nothing queued during the outage |
| 13 kill `sound_arbiter` | PASS (extra) | manual `music-44` started 3 s before the SIGKILL (396.63): the player kept playing; WAV `...T170313.189Z-music-44.wav` 10 851.8 ms, the clip in full (manual, uncapped). The bag has no `sound` status after the kill; `fld`, `rld`, `psi`, `magic` and `dome` kept 25-27 messages in the 14 s outage. Restart 410.6 (hand-started with the launch's file): `offline` then `idle` in 14 ms, `device /sound_player answered list; catalogue loaded` |
| 14 vocalize | PASS (audible: David's) | `express vocalize --arousal 0.9 --valence 0.8 --seed 7` twice: `...T170353.347Z-voice-7.wav` and `...T170359.917Z-voice-7.wav` are byte-identical in content (810 ms, same md5 of the samples); `--arousal 0.1 --valence 0.2 --seed 7`: `...T170406.347Z-voice-7.wav` 2368 ms (2.9 x longer), RMS -17.8 vs -16.2 dBFS, spectral centroid 418 Hz vs 1199 Hz: slower and lower. David to judge "excited" against "glum" |
| 15 graph check | PASS | rclpy over 37 nodes: clients of `/sound_player/play`, `vocalize`, `stop`: `/sound_arbiter` and `/foxglove_bridge_sim` only; `/dome_mock/dome/audio/level` (the sim's `/dome/audio/level`) published by `sound_player` only and `/dome/audio/level` by nothing; `/sound/level` and `/sound/playing` `sound_player` only. `/cmd_vel_dome` `dome_teleop_node`, `/cmd_vel_joy` `teleop_node`, `/diff_cont/cmd_vel` `twist_mux`, `/diff_cont/odom` `diff_cont`, `/dome_controller/commands` `dome_twist_converter`, `/dome/cmd/personality` `dome_arbiter`, `/dome/joint_command` none. The personality nodes (`sound_*`, `*_arbiter`, `r2_affect`, the stimulus nodes) publish only their own topics (`/expression/status`, `/mood/state`, `/stimuli`, `/dome/cmd/personality`, the sound topics). Run with my probes' publishers stopped except the graph checker itself |
| 16 robot | robot only | not run |

Bag: `2026-10-24-sim-acceptance-sound-bag/` (mcap, 6.2 MiB, 1811 s wall from 16:34:26, 45 099
messages: `/expression/status` 19 270, `/mood/state` 8 600, `/sound/level` 5 719 and
`/dome_mock/dome/audio/level` 5 719, `/sound_player/state` 2 634, `/rosout` 2 946,
`/sound/playing` 178, `/expression/intent` 33), under 20 MB, to be committed. It covers both
stack launches: the first one (stopped 16:35) is the first minutes; item 1's startup is in the
second.

## Findings

1. **`ooh` has 16 clips, not 7.** Seven are `padawan-ooh01-07`; nine `music-*` clips (`music-141`,
   `45`, `57`, `60`, `83`, `85`, `86`, `89`, `95`) carry the tag from the tag rules. One of the three
   picks was `music-85`. Section 9 item 4 should say 16, or the music clips are re-tagged.
2. **A refused sound intent is silent, as the dome's was.** Unknown pool, empty pool and
   `excluded: manual play only` appear only as `dropped` +1 in `/expression/status`; the arbiter
   logs nothing for them (only accepted intents). The reason strings exist on the player's
   service (`unknown pool`, `empty pool`) and were seen only by calling it by hand. And
   `express play --pool nosuch` is rejected by the CLI's own tag check (rc 2) before any arbiter;
   item 4 and item 7's "refused `unknown pool`" / "refused `excluded: manual play only`" read as
   if the channel answered.
3. **Item 5: two sets, not three, and the 6 dB scream is not in the files.** `padawan-scream02`
   and `padawan-hum19` have the same set gain, and `hum19` has the louder file (non-silent RMS
   -9.23 against -10.87). Across the `scream`/`hum`/`alarm`/`ooh`/`chatter` tags of the Padawan set the post-gain RMS
   runs -14.2 to -16.8 on average with up to 8 dB inside a tag. If the item means to prove
   dynamics are kept, compare two clips of the same set whose file RMS differs by 6 dB, or
   drop the number.
4. **Item 6 wants raw publishes.** The `express` CLI takes about 0.8 s from call to play, so
   "1 s in" came out 1.4 s, and an idle-play race (pre-empting a 2.6 s chortle with a
   behaviour intent) is not reachable by hand: it needs a probe publishing on seeing the
   play. No Padawan `SENT` is 4 s (the longest is `padawan-sent08`, 3.66 s).
5. **Chatter slots while claimed are skipped, not "dropped and counted"** (item 9). The
   `dropped` counter moved only for the scream (199 against the claim at 200); 34 s of
   claimed excited chatter left it at 6. The expression README says "skipped", the plan says
   "dropped and counted".
6. **A claim's lease is capped at 60 s** (requested 120, granted 60.0); the "CLAIMED 5 s" of items 2 and 6
   is `manual_hold_s` 5 in sim seconds, 6.0 s on the wall at 0.83.
7. **Item 11 is at the limit again** (phase 1 finding 5, phase 2 finding 7): 3.86 s wall from the
   last `/mood/state` is about 3.2 sim seconds at the measured 0.83 real-time factor, against
   `mood_stale_s` 3.0 (the arbiter's tick is not finer than a few hundred milliseconds). A
   chatter clip is 1-3 s, so a stale during ordinary chatter ends the clip on its own first; the
   test used a 10.9 s idle clip.
8. **Item 12's held level was 0 by luck.** The SIGKILL fell on a quiet block (byte 0); the
   second kill came with the player idle. A kill during a loud passage would leave the mock's
   `level=N` up; the plan accepts that. Not demonstrated.
9. **The launch does not respawn `sound_player`** (or any personality node). Item 12 and 13
   restarts were by hand from the launch's `/tmp/launch_params_*` files.
10. **`R2_PHASE3_FULL` appears only in the plan** (step 8a). No test or launch reads it.
11. **A mood-less calm voice starts early.** Unlocked calm played `music-48` 55 s after startup
    (within 45-120) and the unlocked mood slid to sleepy over minutes in phase 2; all items here
    used `--lock`.
12. **`r2_personality` was dirty at the end:** `r2_expression/README.md` modified, which this run
    did not touch (it was clean at the start). `r2_sound/docs/` is the new untracked directory.

## Open items

- David to audition on the Mac (route 1, `afplay`) the WAVs named in items 2, 5, 6, 7, 8, 9, 10,
  11 and 14. The ones that matter most: item 5's four (`20261024T163949.228Z-padawan-sent04`,
  `163959.482Z-music-33`, `164009.425Z-padawan-scream02`, `164020.223Z-padawan-hum19`), item 6's
  pre-emption cut (`164054.798Z-padawan-sent08`, 20 ms fade at 1.39 s), item 7's cut
  (`164304.948Z-music-44`, 5 s) and stop (`164315.953Z-padawan-theme`, 9 s), item 14's three
  `voice-7` files. For clicks at the cuts and the fades, and that the three mood voices differ.
- Decide the items 3, 4, 5, 9 wording (Findings 1, 3, 5, 6) and whether a refused sound intent
  should log its reason (Finding 2).
- Item 5's 6 dB claim FAILED as written and should not be carried to the robot column as is.
- A held dome level after a SIGKILL during a loud passage (Finding 8) and a respawn policy for
  the player (Finding 9) for the robot session.
- Items 8 on the pools David will tag (`excited`, `sad`, `sleepy`, `question`, `scared`):
  re-run item 8 with them in.

- Item 5's second half: the plan's pair was wrong, not the gain. `padawan-hum19`'s source is
  louder than `padawan-scream02`'s (-9.23 against -10.87 dBFS RMS), so with one gain per set
  the hum cannot come out 6 dB quieter. The plan now names `padawan-wolfwstl` and
  `padawan-sent18` (the set's loudest and quietest playable clips, 9.3 dB apart) for that
  check, and the pair recorded here stands as the evidence that the set's dynamics are kept
  (the orchestrator, after this run).

## Files

- `README.md`: this note.
- `bag/`: the bag (6.2 MiB mcap), as the phase 2 notes keep theirs.
- The WAVs: `/ws/.audio/out/` (87 files from this run, stamps `20261024T1637` onwards; not in
  git, `.audio/` is ignored).
- Probe, helpers and raw logs (not kept in the repo): the session scratchpad
  (`probe.py`, `pre.py`, `acc.log` with the timestamped lines this note cites).
