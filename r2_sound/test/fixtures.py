# Copyright (c) 2026 David Smith
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Synthetic audio and a small sound tree for the r2_sound tests (never /sounds)."""

import os
import wave

import numpy as np

RATE = 22050


def tone(freq=1000.0, seconds=1.0, amp=0.3, rate=RATE):
    t = np.arange(int(seconds * rate)) / rate
    return amp * np.sin(2 * np.pi * freq * t)


def chirp(f0, f1, seconds=1.0, amp=0.3, rate=RATE):
    t = np.arange(int(seconds * rate)) / rate
    phase = 2 * np.pi * (f0 * t + (f1 - f0) * t ** 2 / (2 * seconds))
    return amp * np.sin(phase)


def burst_train(freq=3000.0, bursts=10, on=0.05, off=0.05, amp=0.3, rate=RATE):
    one = np.concatenate([tone(freq, on, amp, rate), np.zeros(int(off * rate))])
    return np.tile(one, bursts)


def write_wav(path, samples, rate=RATE):
    """Write mono float samples (-1..1) as a 16-bit WAV with the ``wave`` module."""
    os.makedirs(os.path.dirname(str(path)), exist_ok=True)
    data = (np.clip(samples, -1, 1) * 32767).astype('<i2')
    with wave.open(str(path), 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(data.tobytes())


def make_tree(root):
    """Two sets: labelled Padawan-style files, and numbered unlabelled ones."""
    pad = os.path.join(str(root), 'R2 Padawan')
    music = os.path.join(str(root), 'R2 sounds and music')
    write_wav(os.path.join(pad, '32SENT1.wav'), tone(1000, 1.0, 0.3))
    write_wav(os.path.join(pad, '33SENT2.wav'), tone(1200, 1.2, 0.35))
    write_wav(os.path.join(pad, '25OOH1.wav'), chirp(600, 900, 0.8, 0.3))
    write_wav(os.path.join(pad, '7PATROL1.wav'), tone(2000, 0.6, 0.25))
    write_wav(os.path.join(pad, '1SCREAM2.wav'), tone(2500, 1.0, 0.9))
    write_wav(os.path.join(pad, '9Theme.wav'), tone(300, 1.0, 0.3))
    write_wav(os.path.join(music, '1.wav'), chirp(500, 1800, 1.0, 0.5))
    write_wav(os.path.join(music, '2.wav'), chirp(1800, 400, 2.0, 0.5))
    write_wav(os.path.join(music, '3.wav'), burst_train(3000, 10, amp=0.5))
    write_wav(os.path.join(music, '4.wav'), tone(800, 1.5, 0.4))
    write_wav(os.path.join(music, 'March.wav'), tone(200, 1.0, 0.4))
    return str(root)


def make_duplicate_tree(root):
    """
    Return :func:`make_tree`'s tree with duplicates added.

    ``music-10`` is ``music-3``'s exact copy, ``padawan-sent09`` is ``music-2`` at 0.7 of
    its level behind 10 ms of silence (a near copy, cross-set), and ``music-11`` is a
    different burst train of ``music-3``'s length.
    """
    make_tree(root)
    pad = os.path.join(str(root), 'R2 Padawan')
    music = os.path.join(str(root), 'R2 sounds and music')
    write_wav(os.path.join(music, '10.wav'), burst_train(3000, 10, amp=0.5))
    near = np.concatenate([np.zeros(int(0.01 * RATE)), 0.7 * chirp(1800, 400, 2.0, 0.5)])
    write_wav(os.path.join(pad, '40SENT9.wav'), near)
    write_wav(os.path.join(music, '11.wav'), burst_train(1200, 4, on=0.15, off=0.1, amp=0.5))
    return str(root)


# --- a small catalogue for the player ------------------------------------------------------

PLAYER_RATE = 44100


def clip_entry(name, path, duration_ms, gain_db=0.0, tags=('chatter',), excluded=False,
               peak_dbfs=-10.0, clip_fraction=0.0):
    """Return one ``clips.yaml`` entry (the measurements are nominal)."""
    return {'name': name, 'set': 'Test', 'path': path, 'duration_ms': duration_ms,
            'rms_dbfs': -16.0, 'peak_dbfs': peak_dbfs, 'clip_fraction': clip_fraction,
            'set_gain_db': 0.0, 'gain_db': gain_db, 'tags': list(tags),
            'suggested_tag': tags[0], 'confidence': 1.0, 'excluded': excluded, 'label': ''}


def write_catalogue(path, clips):
    """Write a ``clips.yaml`` with the ``clip_entry`` dicts in ``clips``."""
    import yaml
    data = {'house_level_dbfs': -16.0, 'generated': '2026-10-24',
            'sets': {'Test': {'clips': len(clips), 'median_rms_dbfs': -16.0,
                              'set_gain_db': 0.0}},
            'clips': sorted(clips, key=lambda c: c['name'])}
    os.makedirs(os.path.dirname(str(path)), exist_ok=True)
    with open(str(path), 'w') as f:
        yaml.safe_dump(data, f)
    return str(path)


def make_player_tree(root):
    """
    Return ``(sounds_dir, catalogue_path)``: five clips, constant-amplitude tones.

    ``beep`` 0.5 s at 0.25 (44.1 kHz, gain -6 dB), ``low`` 0.4 s at 22.05 kHz, ``long`` 3 s,
    ``music`` excluded, ``ooh1``/``ooh2`` the ``ooh`` pool; ``scream`` is ``alarm`` only.
    """
    sounds = os.path.join(str(root), 'sounds')
    clips = []
    for name, seconds, amp, rate, gain, tags, excluded in (
            ('beep', 0.5, 0.25, PLAYER_RATE, -6.0, ('chatter',), False),
            ('low', 0.4, 0.25, RATE, 0.0, ('chatter',), False),
            ('long', 3.0, 0.25, PLAYER_RATE, 0.0, ('hum',), False),
            ('music', 0.3, 0.25, PLAYER_RATE, 0.0, ('excluded',), True),
            ('ooh1', 0.3, 0.25, PLAYER_RATE, 0.0, ('ooh',), False),
            ('ooh2', 0.3, 0.25, PLAYER_RATE, 0.0, ('ooh',), False),
            ('scream', 0.3, 0.25, PLAYER_RATE, 0.0, ('alarm',), False)):
        rel = 'Test/%s.wav' % name
        write_wav(os.path.join(sounds, rel), square(seconds, amp, rate), rate)
        clips.append(clip_entry(name, rel, int(seconds * 1000), gain, tags, excluded,
                                peak_dbfs=-12.0))
    return sounds, write_catalogue(os.path.join(str(root), 'clips.yaml'), clips)


def square(seconds, amp, rate):
    """Return a square wave at 441 Hz: its RMS is ``amp`` exactly, block for block."""
    n = int(round(seconds * rate))
    t = np.arange(n)
    return np.where((t * 441 * 2 // rate) % 2 == 0, amp, -amp).astype(np.float64)


def read_wav(path):
    """Return ``(samples float, rate)`` of a 16-bit mono WAV."""
    with wave.open(str(path), 'rb') as w:
        data = np.frombuffer(w.readframes(w.getnframes()), dtype='<i2')
        return data.astype(np.float64) / 32767.0, w.getframerate()
