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
    os.makedirs(os.path.dirname(path), exist_ok=True)
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
