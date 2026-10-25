# r2_sound

R2's sound device: clip player, synth, level feed. Design and steps:
`docs/personality-phase3-plan.md` in the r2_dev workspace.

**Status: steps 4 (the player) and 7 (synth v1) built.** `build_catalogue`, `sound_player`
and the synthesiser (`r2_sound/synth.py`, `vocalize`) are real; the synth awaits David's
notes on the review grid (`synth_grid`).

## Running the player

```sh
ros2 launch r2_sound sound_player.launch.py backend:=wav          # files in /ws/.audio/out
ros2 launch r2_sound sound_player.launch.py backend:=sounddevice device:=USB   # the Pi's DAC
ros2 service call /sound_player/play astromech_msgs/srv/PlaySound \
    "{name: padawan-sent01, priority: 199, owner: me}"
ros2 service call /sound_player/play astromech_msgs/srv/PlaySound "{pool: ooh, priority: 199}"
ros2 service call /sound_player/vocalize astromech_msgs/srv/Vocalize \
    "{arousal: 0.8, valence: 0.7, seed: 42, priority: 199, owner: me}"   # length_s 0: its own
ros2 service call /sound_player/stop astromech_msgs/srv/StopSound "{}"
ros2 service call /sound_player/reload std_srvs/srv/Trigger "{}"   # after build_catalogue
```

In the stack only `sound_arbiter` calls `play`, `vocalize` and `stop` (plan rule 2); the
calls above are for the bench. Parameters (every one read at startup; `ros2 param set`
is refused):

| parameter | default | |
| --- | --- | --- |
| `catalogue` | `''` | the `clips.yaml` to load; `''` is the installed one |
| `sounds_dir` | `/sounds` | the catalogue's paths are relative to it (`/home/pi/sounds` on the Pi) |
| `backend` | `sounddevice` | `sounddevice`, `wav` or `null` |
| `device` | `''` | output device name substring (`USB`); `''` the default device |
| `require_device` | false | exit 1 when no output device matches `device`, instead of running silent |
| `sample_rate` | 44100 | output rate, 8000..48000; clips at other rates are resampled (linear) |
| `cache_mb` | 64 | decode cache cap (least recently used out); a clip over half of it is decoded for each play and never cached |
| `state_hz` | 5 | `~/state` while playing; 1 Hz while idle |
| `level_hz` | 30 | the two level topics while playing |
| `stream_linger_s` | 2 | the sounddevice stream closes this long after the last play |
| `fade_ms` | 20 | the fade of a stop, a replacement and a duration cut |
| `dome_level_topic` | `/dome/audio/level` | the sim: `/dome_mock/dome/audio/level` |
| `level_floor_dbfs`, `level_ceiling_dbfs` | -40, -6 | the dome level's 0 and 255 |
| `wav_dir` | `/ws/.audio/out` | where the `wav` backend writes |
| `allow_duplicate` | false | start even when another node of the same name is on the graph |

A bad parameter, an invalid catalogue, or a sounddevice backend without PortAudio stops the
node at startup with exit 1. A sounddevice backend with no output device matching `device`
(the USB DAC unplugged) logs one ERROR naming the devices found and runs on the `null`
backend: the services answer and the state goes on, silent, with `backend` `null (no
device: USB)`. `require_device:=true` makes that exit 1 instead. PortAudio's device list is
fixed when it starts, so a DAC plugged in later needs a restart. A clip whose file is
missing is dropped with a WARN and is neither listed nor picked.

The launch file reads the parameters from `r2_sound/parameters.py`, which imports nothing
third-party: launching loads neither `soundfile` nor PortAudio (the node does, at its first
decode and its sounddevice backend).

**Threads.** The executor runs only the service callbacks; the node has no timer. A `play`
is resolved in the callback (name or pool, exclusion, gain clamp) and handed to the worker
(`r2_sound/player.py`), which never decodes and never waits on a decode. A clip already in
the decode cache (a hit) is played at once: the current play fades, the backend starts, and
the answer carries the play's exact length. A miss goes to the decoder thread (`decode.py`)
and is answered at once with the catalogue's length: the current play fades (`last_end 4`),
the state shows the new play as `PLAYING` with `elapsed_ms` 0 (at 1 Hz) so `sound_arbiter`
keeps its run, and the stream starts when the decode is done. If that decode (or the
backend's start) fails, the play ends with `last_end 5` (failed) and a WARN; a stop or
another play meanwhile drops it, and the
decoder, which reads a file in chunks of 65536 frames, stops at the next chunk, so the next
decode does not queue behind a long theme nobody wants. A new play touches neither the
play waiting on its decode nor the one playing until it is known to start: a phrase is
rendered first, a file is checked first, a hit is gained and cut first, and a refusal there
(`synth failed`, `decode failed: no such file`, `empty clip`) leaves both as they were. The
worker does all
the publishing: the level at `level_hz` and the state at `state_hz` while a play runs, the
state once a second while idle (with the stream's linger check), and at a play's end the
zeros, `playing` false and the IDLE state at once. The backend's thread (the PortAudio
callback) only renders blocks, stores the block RMS and queues the end; it never touches
rclpy. SIGINT and SIGTERM are the node's own: the play fades, the zeros and the IDLE state go
out, then rclpy shuts down.

**Plays.** One at a time. A play replaces whatever plays (the old one fades over `fade_ms`,
`last_end 4`); `duration_ms` (0: the whole clip) cuts with the fade (`last_end 2`), unless
it is within 1 ms of the clip's length (a length rounded to the millisecond): that plays
the whole clip (`last_end 1`). The arbiter sends 0, not the clip's length, when its cap
does not bite. A stop fades (`last_end 3`); a clip that runs out is `last_end 1`. A play past its length by 2 s
(a stuck output) is ended and the backend reset. The gain is the catalogue's plus the
request's, clamped so the clip's peak stays at or below -1 dBFS (rule 4: accepted, with a
counted WARN). `name` wins over `pool`; a pool pick is uniform over the tag's playable
clips, those with `clip_fraction` over 0.01 at half weight. Refusals (`reason`): `unknown
name`, `unknown pool` (not a tag of the fixed list), `empty pool` (no playable clip has the
tag; `excluded` is always empty), `excluded: manual play only` (an excluded clip by name
below priority 199, the manual priority; the arbiter applies the same rule), `closed`
(shutting down), `decode failed: no such file` (the clip's file is not there, checked
before anything is queued), `empty clip`, `synth failed: ...` (vocalize), and `backend: ...`
when the output fails. A file that is there but cannot be decoded is accepted (a miss is
answered before its decode) and then ends with `last_end 5` (failed) and a WARN
`decode failed: ...`.

## Backends

| backend | does | needs |
| --- | --- | --- |
| `sounddevice` | plays through PortAudio, 1470-frame blocks at 44.1 kHz | a sound device, or on the Mac the PulseAudio route below |
| `wav` | plays on wall time and writes what played to `<wav_dir>/<utc stamp>-<name>.wav` (16-bit mono, gain, fade and cut included) as the play ends | nothing; the sim's speaker |
| `null` | the `wav` backend without the file | nothing; the tests |

`backends.py` is the only module that imports `sounddevice`, at the first sounddevice
backend; `decode.py` imports `soundfile` at the first decode (the only other importer is the
`build_catalogue` tool). The `sounddevice` stream opens with the first play and closes
`stream_linger_s` after the last, so an idle player holds no stream. Its callback copies the
frames, applies a pending fade, computes the block RMS and hands it on; it allocates nothing.
In the container the ALSA default is the `pulse` plugin (`/etc/asound.conf`); with
`PULSE_SERVER` unset or unreachable the stream cannot open and each play is refused
`backend: ...` (use `backend:=wav`). The container has no sound device, so the
`sounddevice` path is tested around a fake stream only; latency and CPU are measured on the
robot (plan step 8b).

## Topics and services

| name | type | |
| --- | --- | --- |
| `~/list` | `astromech_msgs/srv/ListSounds` | the catalogue as loaded; `synth` true |
| `~/play` | `astromech_msgs/srv/PlaySound` | see "Plays" above |
| `~/vocalize` | `astromech_msgs/srv/Vocalize` | renders a phrase on the worker and plays it as `voice:<seed>` (see "The synth") |
| `~/stop` | `astromech_msgs/srv/StopSound` | fades; `stopped` false when nothing played |
| `~/reload` | `std_srvs/srv/Trigger` | reads the catalogue and the sounds again; a bad file keeps the old catalogue |
| `~/state` | `astromech_msgs/msg/SoundState` | at `state_hz` while playing and on every change, 1 Hz idle; `elapsed_ms`/`remaining_ms` from the backend's frame count |
| `/sound/level` | `std_msgs/msg/Float32` | block RMS 0..1 at `level_hz` while playing, then one 0 |
| `/sound/playing` | `std_msgs/msg/Bool` | agrees with `~/state`: true from the moment a play is accepted (a miss still decoding included), false when nothing is pending or playing; on change only (a replacement does not toggle it); transient local, so a late subscriber gets the current value |
| `dome_level_topic` | `std_msgs/msg/UInt8` | the level for the dome, same cadence, then one 0 |

`~` is the node name, `/sound_player` by default. At startup the node publishes one IDLE
state, a 0 on both level topics and `playing` false (the dome holds its last level forever).

## The level

Each output block (a 30th of a second) is measured as it is rendered: its RMS, linear 0..1,
goes on `/sound/level` and in the state's `level`. The dome gets
`gain.level_to_byte(20 log10(rms))`: `level_floor_dbfs` and below is 0, `level_ceiling_dbfs`
and above 255, linear in dB between. A clip at the house level (-16 dBFS) shows about 180.
The level topics run only while a play runs; at its end both get one 0, published with the
IDLE state (within 50 ms of the end).

## Listening on the Mac

- Route 1: `wav` backend, then `afplay` on the files in `.audio/out/`.
- Route 2: PulseAudio on the Mac over TCP, `sounddevice` backend.

Full instructions: "Sound on the Mac" in `r2_bringup`'s README. Nothing in the stack or the
tests needs route 2.

## The catalogue

`config/clips.yaml` lists every clip of the two sets under `/sounds` (`R2 Padawan`,
`R2 sounds and music`); the audio stays out of git. It is built by `build_catalogue` and
read by the pure `r2_sound.catalogue.load_catalogue(path, sounds_dir)`, which returns
`(Catalogue, warnings)` as frozen dataclasses and drops (with a warning, never an error) the
clips whose file is missing under `sounds_dir`. `pools(catalogue)` maps each tag to its
playable (not `excluded`) clips.

Schema (unknown keys, duplicate names, tags outside the list, a `gain_db` outside -20..+12
and a `confidence` outside 0..1 are refused with the dotted path of the key):

```yaml
house_level_dbfs: -16.0
generated: '2026-10-24'          # moves only when the content does
sets:
  R2 Padawan: {clips: 55, median_rms_dbfs: -11.87, set_gain_db: -4.13}
clips:                           # sorted by name
- name: padawan-sent01           # set prefix + stem, lower-case, unique
  set: R2 Padawan
  path: R2 Padawan/32SENT1.mp3   # relative to sounds_dir
  duration_ms: 2100
  rms_dbfs: -13.1                # over the non-silent part (frames above -50 dBFS)
  peak_dbfs: -0.4
  clip_fraction: 0.0             # share of samples with |x| >= 0.999
  set_gain_db: -4.13             # house level minus the set's median RMS
  gain_db: -4.13                 # + outlier trim + sheet override, peak-limited (plan 2.3)
  tags: [chatter]                # the effective pools (merge rule below)
  suggested_tag: chatter
  confidence: 1.0
  excluded: false                # music, speech, silence: manual play only
  label: chatter                 # from the file name, or ''
duplicates:                      # the clips left out as duplicates (the loader ignores it)
- {name: music-64, path: R2 sounds and music/64.mp3, duplicate_of: padawan-sent10}
```

The fixed tag list: `chatter, excited, sad, question, ooh, hum, alarm, scream, laugh,
annoyed, whistle, calm, scared, sleepy, greeting, yes, no, misc, excluded`. `yes` and `no` are
answer pools for behaviours that ask R2 a question (added 2026-10-25); no mood chatters from them.

**Gain** (`r2_sound/gain.py`): each set's median RMS goes to -16 dBFS; a clip more than 8 dB
from its set median is pulled halfway back; the sheet's `clip_override_db` is added; the peak
stays at or below -1 dBFS. The medians are over the clips not on the exclusion list.
`level_to_byte` maps dBFS to the dome's 0..255 level (floor -40, ceiling -6).

**Suggestions** (`r2_sound/analysis.py`): labelled clips (named `SENT`, `OOH`, `HUM`,
`ALARM`, `SCREAM`, `CHORTLE`, `ANNOYED`, `WOLFWSTL`, `MISC`, `Powerup`, `DOODOO`,
`SHORTCKT`, `PATROL`) suggest their own bucket at confidence 1.0 and are the references. Every
other clip gets the bucket of its 3 nearest references in standardised feature space
(duration, envelope shape, burst count, zero-crossing rate and its slope, spectral centroid
and its slope), with a confidence from the neighbours' agreement and distance. For buckets
with no reference there are rules at confidence 0.4: a short rising chirp is `question`, a
long descending one `sad`, a fast multi-burst high-centroid one `excited`, a long very quiet
one `sleepy`; they apply only when the neighbours are less sure than 0.6. `calm`, `scared`
and `greeting` come from the sheet only. **These are a first pass to be confirmed by ear.**
Music, speech and silence (`Theme`, `Chorus`, `Emperor`, `Cantina`, `March`, `Disco`,
`StarWars`, `Leia`, `Silence`) are `excluded`.

**The merge rule** (`catalogue.effective_tags`): a clip's tags are the sheet's `tags` if it
has any; otherwise `[suggested_tag]` if its `confidence` is at or above the threshold
(default 0.6); otherwise `[chatter]`. A mood whose pools end up empty falls back to
`chatter` in `sound_arbiter`'s palette (rule 8), so an untouched sheet still gives R2 a
voice; the player itself refuses a play of an empty pool with `empty pool`.

**Duplicates.** Some sounds are in both sets (and a few twice in one). `build_catalogue`
fingerprints each clip (log energies in 16 bands, 125 Hz to 4 kHz, over 50 ms frames of the
8 kHz mono signal, each band's mean removed, so a level difference does not count) and scores
every pair whose durations are within 25 % or 0.3 s (the music set's copies carry 5-10 % more padding than the Padawan originals), across the sets too: the lesser of the
band profiles' correlation and the time patterns' best cross-correlation within 100 ms of
lag, 0..1; the same samples score 1.0. Pairs at 0.85 or above (`--duplicate-threshold`) are
suggested. On the two sets the copies score 0.917 to 0.998 and the next pairs, variants
rather than copies, 0.818 to 0.891 (`--pairs 30` prints the best pairs). Linked pairs form
a group; the group keeps a labelled clip over an unlabelled one, then the clip whose RMS is nearest its set's median, then the shorter
name, then the first alphabetically, and every other member gets `suggested_duplicate_of`
(the kept clip), `duplicate_score` and `duplicate_timing` in the sheet. Those three columns
are refreshed every run and do nothing (`duplicate_timing` is empty when there is no suggestion); only David's `duplicate_of` acts. A clip whose `duplicate_of` names
another clip is left out of `clips` (the player never lists or picks it), listed under
`duplicates:` (name, path, duplicate_of) and not counted in its set's `clips:`; the set
medians still count it, so marking one moves no other gain. Its tags are ignored (with a
WARN). In the kit its WAV is `duplicates/<name>__of__<kept>.wav` instead of its bucket's,
so it can still be heard. A `duplicate_of` whose clip is not on disk (a `missing` row) keeps
the duplicate in the catalogue with a WARN, so the sound is not lost.

The sheet's columns: `name, set, path, duration_s, rms_dbfs, suggested_tag, confidence,
tags, clip_override_db, duplicate_of, suggested_duplicate_of, duplicate_score,
duplicate_timing, status`.
David's are `tags`, `clip_override_db` and `duplicate_of`; a sheet from before the
duplicate columns, or without `duplicate_timing`, reads fine and is written with them.

`duplicate_timing` (`analysis.timing_comparison`) compares a suggested pair, its numbers given as kept clip / this clip, on the 10 ms RMS
envelope of the 8 kHz signals: the active part is the frames within 35 dB of the loudest; it
gives lead and trail silence and active length for both (kept clip first), then a verdict.
`same recording`: active lengths within 3 % and the envelopes correlate >= 0.9 within 200 ms
of shift (so only padding or level differs). `other take`: the lengths differ and
stretching one envelope onto the other does not fit better (the syllables fall at other
times, the pitch kept). `stretched`: the lengths differ, stretching fits clearly better and
the spectral centroid is within 4 %. `resampled`: the lengths differ and the centroid moved
(slower or faster playback). `other take` pairs are the ones worth keeping for synth work:
the same phrase, the rhythm jittered by tens of ms, the pitch held. `--pairs N` prints the
timing columns beside score and names.

**Tagging and re-running.**

```sh
build_catalogue --sounds-dir /sounds --out config/clips.yaml --sheet config/tags.csv \
    --kit /ws/.audio/tagging [--threshold 0.6] [--duplicate-threshold 0.85] [--pairs 30]
```

1. Listen a bucket at a time on the Mac: `/ws/.audio/tagging/play_bucket.sh chatter` plays
   each WAV of the kit (16-bit mono 22.05 kHz, at catalogue loudness, cut to 10 s) and
   prints its name. `README.md` in the kit repeats this.
2. Recommended: the tag tool, a local web page (standard library only, 127.0.0.1 only).
   From the Mac, outside the container:
   `python3 src/r2_personality/r2_sound/r2_sound/tools/tag_tool.py --workspace ~/path/to/r2_dev`
   (`--sheet`, `--kit`, `--port 8765`, `--no-browser` as options); in the container,
   `tag_tool --workspace /ws --no-browser` and open the printed URL. Clips are listed by
   suggested bucket with the kit WAV playing; keys: space replays, Enter = Confirm and next,
   arrows = previous/next, 1-9 and 0 toggle the first ten tags, `d` = mark the suggested
   duplicate and next, `k` = play the kept clip. **Confirm** writes the suggested tag into
   `tags` (so the merge no longer depends on the confidence), **Clear** empties it again.
   **Duplicates:** a clip with a suggestion shows "Suggested duplicate of <name> (score
   0.97)" with **Play kept** (that clip's kit WAV) and **Mark duplicate of <name>**. The
   suggestion is only a suggestion: listen to both first. Any clip can be marked by hand in
   the "Duplicate of" field (a list of the clip names) and **Not a duplicate** clears it; a
   marked clip shows `dup` in the list and counts as reviewed. The tool refuses a clip as
   its own duplicate, a name not in the sheet, and chains (the kept clip must not itself be
   a duplicate). **Save** re-reads `config/tags.csv` from disk and rewrites in place only
   `tags`, `clip_override_db` and `duplicate_of` of the changed rows (so a `build_catalogue`
   run while the page was open is kept; a row gone from the file is refused; atomic,
   previous sheet kept as `tags.csv.bak`). Then go to step 3. Alternative, by hand: in
   `config/tags.csv` write pool names into `tags` only where the suggestion is wrong or
   missing (comma-separated, from the fixed list), a number of dB in `clip_override_db` for
   a clip that sits wrong in its set, and the name of the clip to keep in `duplicate_of` for
   a clip that is a duplicate (usually the `suggested_duplicate_of`, once heard). A clip of
   the exclusion list given any other tag here becomes playable; `excluded` here excludes
   any clip.
3. Run `build_catalogue` again. It reads those three columns by name and keeps them,
   refreshes every other column (the duplicate suggestions included), rewrites `clips.yaml` and the kit (a clip whose suggestion changed
   moves folder). A row belongs to the clip at its `path`, else (a file moved within its
   set, or a row with no path) to the clip of its `name`. A row whose file is gone is never
   deleted: it stays as it was with `status` `missing` (and a WARN) and finds its clip
   again when the file is back. Repeat suffixes (`-2`) follow the sorted paths; a renamed
   set directory changes its clips' paths and names, so its rows stay `missing` and their
   tags must be copied to the new rows by hand. A run with nothing changed rewrites
   identical files. A bad tag or override stops the run with the sheet's line number; so do
   two rows with the same `path`, or the same `name` (unless one is `missing`), naming both
   lines; and a `duplicate_of` that is not a clip of the sheet, is the clip itself, or names
   a clip that is itself marked a duplicate (no chains: name the clip to keep).

Run the tool from the container (`colcon build --symlink-install` first so the defaults for
`--out` and `--sheet` point into `src/`). With `--kit` left out no WAVs are written.

## The synth

`r2_sound/synth.py` (pure numpy; the model, its ranges and how to add a syllable are in its
docstring) composes a phrase of 2-8 syllables (beep, chirp up/down, warble, trill, whistle,
buzz, purr) from `(arousal, valence)` in 0..1 and a seed, and renders it at the house level
(-16 dBFS RMS, peak at most -1 dBFS). `vocalize` renders on the player's worker (synchronously: a phrase takes
milliseconds), never on a callback, and plays the phrase like a clip named `voice:<seed>`: the same state, level
topics, fade and `last_end`; the `wav` backend writes it like any play. `seed` 0 draws one and
the response returns it, so a phrase can be replayed; `length_s` 0 lets the synth choose from
the arousal (0.6-2.5 s), otherwise the phrase is that long (0.4-8 s) and is cut at the request
with the fade when asked for less than 0.4 s. `duration_ms` in the response is the play's
length. A gain over the phrase's headroom is clamped with a warning (rule 4). About 10 ms
for a 2 s phrase in the dev container.

The review grid for David: `synth_grid` renders 25 phrases (arousal and valence each 0.1 ..
0.9, seed 1) to `/ws/.audio/synth/a<A>_v<V>.wav` at 22.05 kHz with an `index.md` and a
`play_grid.sh` (afplay, in order, printing the coordinates).
