# r2_sound

R2's sound device: clip player, synth, level feed. Design and steps:
`docs/personality-phase3-plan.md` in the r2_dev workspace.

**Status: skeleton (step 1).** `sound_player` and `build_catalogue` are placeholders that
print "not implemented until step 3/4" and exit.

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

## Not implemented yet

- Catalogue, gain and decode (step 3), `config/clips.yaml`
- The player node and its backends (step 4)
- Sound channel, launch file (steps 5, 6)
- Synth and level feed (step 7)
