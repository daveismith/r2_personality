# 2026-10-18 sim acceptance run

Phase 0 acceptance script (r2_dev `docs/personality-phase0-plan.md`, section 8) run against
the headless simulator in the dev container with the shipped `affect.yaml`. David drove the
Foxglove panels (layout `r2_affect/foxglove/personality-tuning.json` on `:8766`), Claude drove
the CLIs from the container. Recorded on sim time with
`ros2 bag record --use-sim-time --include-unpublished-topics --topics /stimuli /mood/state /rosout`:
528 s, 8 stimuli, 6416 states, `/rosout` included.

| item | result |
|---|---|
| 1 fresh start | `calm` at temperament (seen again after the item 12 restart, `idle_s` counting) |
| 2 bump | arousal up, valence down, confidence down; INFO line; `idle_s` reset |
| 3 second bump within 2 s | logged `dropped: refractory (2.0 s window ...)`, no change |
| 4 leave it | label back to `calm` within 20 s of the last stimulus (arousal relaxing with tau 45 s) |
| 5 face_known dave, bearing 0.8 | `focus_subject` dave, `focus_bearing_rad` 0.8 |
| 6 force sleepy (Foxglove) | vector at the prototype, label `sleepy` at once, relaxes back afterwards |
| 7 force + lock (Foxglove) | `locked` true; `face_known` while locked logged `(locked, not applied)` |
| 8 lock from another source | refused, reason `locked by 'foxglove' until cleared`; clear from the other source accepted |
| 9 unknown kind | one WARN, nothing else |
| 10 reload | doubled bump applied after reload (valence −0.80); broken YAML refused with the old config kept (bump still −0.80); restore reloaded |
| 11 idle | `calm -> sleepy` about 150 s after the last stimulus |
| 12 kill r2_affect | `/mood/state` silent at once; controllers, teleop and the adapters unaffected; restart comes up `calm` |
| 13 pause Gazebo | 20 messages in 2 s with one frozen stamp while paused; distinct stamps again on resume |
| 14 robot CPU | not run (robot) |

`ros2 run r2_affect replay <this dir> --compare` reports a 0.40 valence mismatch at 142.6 s
and 157.0 s and a 0.20 confidence mismatch at 511.6 s. Both are expected: the first two are
the item 10 bumps applied under the temporarily doubled impulse, which the replay cannot know
about, and the last is the item 12 restart to temperament, which the replay can only
approximate as a force. Up to 142 s the trace reproduces, including the refractory drop and
the inferred clear of item 8.

Tuning observations for step 6 item 4 (no values changed yet):

- `calm` held for only about 80 s after the last stimulus before the idle rate took over and
  the label went to `sleepy` at about 150 s. For a droid at a convention that is too eager
  to doze; raise `idle.after_s` or soften the `idle` rate.
- `face_known` has a 20 s refractory, so a second sighting within that window is ignored
  entirely. Probably right for the camera, surprising from a button.
- A bump at `calm` lands on `sulking` (from `sleepy`) or `annoyed`/`scared` (from `calm`),
  which reads well. Confidence clamps to 0 after two bumps.

**Change made 2026-10-19 (first tuning change):** `idle.after_s` 30 → 180, `idle.ramp_s`
120 → 300, rate unchanged. Simulated from `calm` with nothing happening, the label now flips
to `sleepy` at about 6 min 35 s instead of 2 min 20 s, and arousal reaches −0.8 at about
8 min 13 s (3 min 20 s before). Three minutes of unchanged calm covers the normal gaps
between people at an event; a slower slide into sleep gives the phase 4 head droop something
to follow. Candidates tried (after_s/ramp_s → sleepy at): 120/180 → 4:26, 240/240 → 7:00,
300/300 → 8:35; a rate of −0.015 was rejected because its equilibrium (−0.675) sits too close
to the flip point near −0.5.
