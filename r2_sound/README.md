# r2_sound

R2's sound device: clip player, synth, level feed. Design and steps:
`docs/personality-phase3-plan.md` in the r2_dev workspace.

**Status: step 3 done (catalogue).** `build_catalogue` is real; `sound_player` is still a
placeholder that prints "not implemented until step 3/4" and exits.

## Backends (the player's `backend` parameter)

| backend | does | needs |
| --- | --- | --- |
| `sounddevice` | plays through PortAudio | a sound device, or on the Mac the PulseAudio route below |
| `wav` | writes what it would play to `/ws/.audio/out/` | nothing; the default for the sim and tests |
| `null` | plays nothing | nothing |

In the container the ALSA default is the `pulse` plugin (`/etc/asound.conf`). With
`PULSE_SERVER` unset or unreachable, `sounddevice` playback fails harmlessly and the player
falls back as its `backend` parameter says (not implemented yet).

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

- Decode and resample for the player (step 4; the catalogue side is done)
- The player node and its backends (step 4)
- Sound channel, launch file (steps 5, 6)
- Synth and level feed (step 7)
