# r2_sound

R2's sound device: clip player, synth, level feed. Design and steps:
`docs/personality-phase3-plan.md` in the r2_dev workspace.

**Status: step 4 done (the player).** `build_catalogue` and `sound_player` are real;
`vocalize` answers `no synth` until step 7.

## Running the player

```sh
ros2 launch r2_sound sound_player.launch.py backend:=wav          # files in /ws/.audio/out
ros2 launch r2_sound sound_player.launch.py backend:=sounddevice device:=USB   # the Pi's DAC
ros2 service call /sound_player/play astromech_msgs/srv/PlaySound \
    "{name: padawan-sent01, priority: 199, owner: me}"
ros2 service call /sound_player/play astromech_msgs/srv/PlaySound "{pool: ooh, priority: 199}"
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
| `sample_rate` | 44100 | output rate, 8000..48000; clips at other rates are resampled (linear) |
| `cache_mb` | 64 | decode cache cap (least recently used out) |
| `state_hz` | 5 | `~/state` while playing; 1 Hz while idle |
| `level_hz` | 30 | the two level topics while playing |
| `stream_linger_s` | 2 | the sounddevice stream closes this long after the last play |
| `fade_ms` | 20 | the fade of a stop, a replacement and a duration cut |
| `dome_level_topic` | `/dome/audio/level` | the sim: `/dome_mock/dome/audio/level` |
| `level_floor_dbfs`, `level_ceiling_dbfs` | -40, -6 | the dome level's 0 and 255 |
| `wav_dir` | `/ws/.audio/out` | where the `wav` backend writes |
| `allow_duplicate` | false | start even when another node of the same name is on the graph |

A bad parameter, an invalid catalogue, or a sounddevice backend without PortAudio or
without a device matching `device` stops the node at startup with exit 1 (PortAudio's
device list is fixed when it starts, so a DAC plugged in later needs a restart). A clip whose file is missing is dropped with a WARN and is
neither listed nor picked.

**Threads.** The executor runs only the service callbacks; the node has no timer. A `play`
is resolved in the callback (name or pool, exclusion, gain clamp) and handed to the worker
(`r2_sound/player.py`), which decodes it (`decode.py`, on a cache miss), stops the current
play with the fade and waits for its end, starts the backend and answers. The worker does all
the publishing: the level at `level_hz` and the state at `state_hz` while a play runs, the
state once a second while idle (with the stream's linger check), and at a play's end the
zeros, `playing` false and the IDLE state at once. The backend's thread (the PortAudio
callback) only renders blocks, stores the block RMS and queues the end; it never touches
rclpy. SIGINT and SIGTERM are the node's own: the play fades, the zeros and the IDLE state go
out, then rclpy shuts down.

**Plays.** One at a time. A play replaces whatever plays (the old one fades over `fade_ms`,
`last_end 4`); `duration_ms` (0: the whole clip) cuts with the fade (`last_end 2`); a stop
fades (`last_end 3`); a clip that runs out is `last_end 1`. A play past its length by 2 s
(a stuck output) is ended and the backend reset. The gain is the catalogue's plus the
request's, clamped so the clip's peak stays at or below -1 dBFS (rule 4: accepted, with a
counted WARN). `name` wins over `pool`; a pool pick is uniform over the tag's playable
clips, those with `clip_fraction` over 0.01 at half weight. Refusals (`reason`): `unknown
name`, `unknown pool` (not a tag of the fixed list), `empty pool` (no playable clip has the
tag; `excluded` is always empty), `excluded` (an excluded clip below priority 199), `closed`
(shutting down), and `decode failed: ...` or `backend: ...` when the file or the output
fails.

## Backends

| backend | does | needs |
| --- | --- | --- |
| `sounddevice` | plays through PortAudio, 1470-frame blocks at 44.1 kHz | a sound device, or on the Mac the PulseAudio route below |
| `wav` | plays on wall time and writes what played to `<wav_dir>/<utc stamp>-<name>.wav` (16-bit mono, gain, fade and cut included) as the play ends | nothing; the sim's speaker |
| `null` | the `wav` backend without the file | nothing; the tests |

`backends.py` is the only module that imports `sounddevice` (and `decode.py` the only one
that imports `soundfile`). The `sounddevice` stream opens with the first play and closes
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
| `~/list` | `astromech_msgs/srv/ListSounds` | the catalogue as loaded; `synth` false until step 7 |
| `~/play` | `astromech_msgs/srv/PlaySound` | see "Plays" above |
| `~/vocalize` | `astromech_msgs/srv/Vocalize` | refused `no synth` until step 7 |
| `~/stop` | `astromech_msgs/srv/StopSound` | fades; `stopped` false when nothing played |
| `~/reload` | `std_srvs/srv/Trigger` | reads the catalogue and the sounds again; a bad file keeps the old catalogue |
| `~/state` | `astromech_msgs/msg/SoundState` | at `state_hz` while playing and on every change, 1 Hz idle; `elapsed_ms`/`remaining_ms` from the backend's frame count |
| `/sound/level` | `std_msgs/msg/Float32` | block RMS 0..1 at `level_hz` while playing, then one 0 |
| `/sound/playing` | `std_msgs/msg/Bool` | on change; transient local, so a late subscriber gets the current value |
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
```

The fixed tag list: `chatter, excited, sad, question, ooh, hum, alarm, scream, laugh,
annoyed, whistle, calm, scared, sleepy, greeting, misc, excluded`.

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
`chatter` in the player (rule 8), so an untouched sheet still gives R2 a voice.

**Tagging and re-running.**

```sh
build_catalogue --sounds-dir /sounds --out config/clips.yaml --sheet config/tags.csv \
    --kit /ws/.audio/tagging [--threshold 0.6]
```

1. Listen a bucket at a time on the Mac: `/ws/.audio/tagging/play_bucket.sh chatter` plays
   each WAV of the kit (16-bit mono 22.05 kHz, at catalogue loudness, cut to 10 s) and
   prints its name. `README.md` in the kit repeats this.
2. In `config/tags.csv` (columns `name, set, path, duration_s, rms_dbfs, suggested_tag,
   confidence, tags, clip_override_db`) write pool names into `tags` only where the suggestion
   is wrong or missing (comma-separated, from the fixed list), and a number of dB in
   `clip_override_db` for a clip that sits wrong in its set.
3. Run `build_catalogue` again. It reads those two columns by name and keeps them, refreshes
   every other column, rewrites `clips.yaml` and the kit (a clip whose suggestion changed
   moves folder). A run with nothing changed rewrites identical files. A bad tag or override
   stops the run with the sheet's line number.

Run the tool from the container (`colcon build --symlink-install` first so the defaults for
`--out` and `--sheet` point into `src/`). With `--kit` left out no WAVs are written.

## Not implemented yet

- The sound channel in `r2_expression` and the bringup (steps 5, 6)
- The synth, `vocalize` (step 7)
