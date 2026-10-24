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

"""Pure tests of the player's parts: decode, cache, fade, backends (fake stream), worker."""

import os
import random
import threading
import time
import types

import numpy as np
import pytest

from fixtures import make_player_tree, PLAYER_RATE, read_wav, square, write_wav  # noqa: I100
from r2_sound import backends, gain
from r2_sound.analysis import resample
from r2_sound.catalogue import load_catalogue, pick, pick_weights, pools, resolve_play
from r2_sound.decode import decode, DecodeCache
from r2_sound.player import (END_DURATION, END_ENDED, END_REPLACED, END_STOPPED, FADING,
                             IDLE, Player, PLAYING, PlayJob)

RATE = PLAYER_RATE
BLOCK = 1470


# --- gain, level, fade ------------------------------------------------------------------

@pytest.mark.parametrize('dbfs, byte', [(-60.0, 0), (-40.0, 0), (-23.0, 128), (-6.0, 255),
                                        (0.0, 255), (-120.0, 0)])
def test_level_mapping(dbfs, byte):
    assert gain.level_to_byte(dbfs) == byte


def test_level_mapping_follows_its_parameters():
    assert gain.level_to_byte(-30.0, floor_dbfs=-30.0, ceiling_dbfs=-10.0) == 0
    assert gain.level_to_byte(-20.0, floor_dbfs=-30.0, ceiling_dbfs=-10.0) == 128


def test_block_rms():
    assert gain.block_rms(np.zeros(10)) == 0.0
    assert gain.block_rms(np.array([])) == 0.0
    assert gain.block_rms(square(0.1, 0.5, RATE)) == pytest.approx(0.5)


def test_fade_shape():
    curve = gain.fade_out(882)
    assert curve.dtype == np.float32 and len(curve) == 882
    assert curve[0] == pytest.approx(1.0) and curve[-1] == pytest.approx(0.0, abs=1e-7)
    assert np.all(np.diff(curve) <= 0)                    # monotone down
    assert curve[441] == pytest.approx(0.5, abs=0.01)     # half way at the middle
    assert list(gain.fade_out(1)) == [0.0] and len(gain.fade_out(0)) == 0


def test_play_gain_is_clamped_to_the_peak_ceiling():
    assert gain.play_gain_db(-4.0, 3.0, -10.0) == (-1.0, False)
    total, clamped = gain.play_gain_db(-4.0, 20.0, -10.0)
    assert clamped and total == pytest.approx(9.0)        # -10 + 9 = -1 dBFS peak
    assert gain.play_gain_db(-4.0, -6.0, 0.5) == (-10.0, False)
    assert gain.play_gain_db(-4.0, float('nan'), -10.0) == (-4.0, True)
    assert gain.play_gain_db(-4.0, float('inf'), -10.0) == (9.0, True)


# --- decode ---------------------------------------------------------------------------------

def test_resample_up_is_linear():
    x = np.arange(10, dtype=np.float32)
    up = resample(x, 22050, 44100)
    assert len(up) == 20 and up[1] == pytest.approx(0.5) and up[2] == pytest.approx(1.0)
    assert up[-1] == pytest.approx(9.0)                   # held beyond the end
    assert resample(x, 44100, 44100) is x
    assert len(resample(np.zeros(22050), 22050, 44100)) == 44100


def test_resample_keeps_a_tone():
    t = np.arange(8000) / 8000.0
    out = resample(np.sin(2 * np.pi * 200 * t), 8000, 44100)
    want = np.sin(2 * np.pi * 200 * np.arange(len(out)) / 44100.0)
    assert len(out) == 44100 and np.max(np.abs(out - want)[:-10]) < 0.02


def test_decode_applies_gain_and_rate(tmp_path):
    path = str(tmp_path / 'a.wav')
    write_wav(path, square(0.5, 0.25, 22050), 22050)
    x = decode(path, RATE, -6.0)
    assert x.dtype == np.float32 and len(x) == RATE // 2
    assert not x.flags.writeable
    assert gain.block_rms(x) == pytest.approx(0.25 * 10 ** (-6 / 20), rel=0.02)


def test_cache_evicts_least_recently_used():
    loads = []

    def loader(path, rate, gain_db):
        loads.append(path)
        return np.zeros(100, dtype=np.float32)            # 400 bytes each

    cache = DecodeCache(1000, RATE, loader)
    a = cache.get('a')
    cache.get('b')
    assert cache.get('a') is a and cache.hits == 1        # a is now the most recent
    cache.get('c')                                        # 1200 bytes: b goes
    assert [k[0] for k in cache.keys()] == ['a', 'c'] and cache.bytes == 800
    cache.get('b')
    assert loads == ['a', 'b', 'c', 'b']
    cache.get('a', -3.0)                                  # another gain, another entry
    assert ('a', -3.0) in cache.keys()
    cache.clear()
    assert cache.keys() == [] and cache.bytes == 0


def test_cache_drops_a_decode_begun_before_a_clear():
    cache = None

    def loader(path, rate, gain_db):
        cache.clear()                                     # a reload while decoding
        return np.zeros(10, dtype=np.float32)

    cache = DecodeCache(1000, RATE, loader)
    cache.get('a')
    assert cache.keys() == []


def test_cache_does_not_keep_a_buffer_over_the_cap():
    cache = DecodeCache(100, RATE, lambda p, r, g: np.zeros(1000, dtype=np.float32))
    assert len(cache.get('big')) == 1000 and cache.keys() == []


# --- pools --------------------------------------------------------------------------------

@pytest.fixture
def tree(tmp_path):
    return make_player_tree(tmp_path)


def test_resolve_play_refusals(tree):
    sounds, path = tree
    catalogue, _ = load_catalogue(path, sounds)
    by_name, pool_map, rng = catalogue.by_name(), pools(catalogue), random.Random(1)
    assert resolve_play(by_name, pool_map, 'nope', '', 100, rng) == (None, 'unknown name')
    assert resolve_play(by_name, pool_map, '', 'nope', 100, rng) == (None, 'unknown pool')
    assert resolve_play(by_name, pool_map, '', 'sad', 100, rng) == (None, 'empty pool')
    assert resolve_play(by_name, pool_map, '', 'excluded', 199, rng) == (None, 'empty pool')
    assert resolve_play(by_name, pool_map, 'music', '', 100, rng) == (None, 'excluded')
    assert resolve_play(by_name, pool_map, 'music', '', 199, rng)[0].name == 'music'
    picked = {resolve_play(by_name, pool_map, '', 'ooh', 100, rng)[0].name for _ in range(50)}
    assert picked == {'ooh1', 'ooh2'}


def test_pick_weights_halve_clipped_clips(tree):
    sounds, path = tree
    catalogue, _ = load_catalogue(path, sounds)
    a, b = catalogue.by_name()['ooh1'], catalogue.by_name()['ooh2']
    b = type(b)(**{**b.__dict__, 'clip_fraction': 0.02})
    assert pick_weights([a, b]) == [1.0, 0.5]
    rng = random.Random(3)
    counts = sum(pick([a, b], rng) is b for _ in range(3000))
    assert 850 < counts < 1150                            # a third


# --- Play and the backends ------------------------------------------------------------------

def test_play_renders_copies_and_ends():
    buf = np.arange(1, 3001, dtype=np.float32) / 3000.0
    ends = []
    play = backends.Play(buf, lambda rms: None, lambda f, s: ends.append((f, s)))
    out = np.empty(BLOCK, dtype=np.float32)
    rms, count = play.render_into(out)
    assert count == BLOCK and np.array_equal(out, buf[:BLOCK]) and not play.done
    play.render_into(out)
    rms, count = play.render_into(out)
    assert count == 60 and np.all(out[60:] == 0) and play.done
    play.end()
    play.end()
    assert ends == [(3000, False)]


def test_play_fade_spans_blocks():
    buf = np.ones(10000, dtype=np.float32)
    play = backends.Play(buf, lambda rms: None, lambda f, s: None)
    out = np.empty(BLOCK, dtype=np.float32)
    play.render_into(out)
    assert play.begin_fade(2000) and not play.begin_fade(10)
    play.render_into(out)
    assert np.allclose(out, gain.fade_out(2000)[:BLOCK]) and not play.done
    play.render_into(out)
    assert np.allclose(out[:530], gain.fade_out(2000)[BLOCK:]) and np.all(out[530:] == 0)
    assert play.done and play.stopped and play.position == BLOCK + 2000


class FakeStream:
    """Stands for sounddevice.OutputStream: the test calls the callback."""

    made = []

    def __init__(self, samplerate, blocksize, channels, dtype, device, callback):
        self.kwargs = {'samplerate': samplerate, 'blocksize': blocksize,
                       'channels': channels, 'dtype': dtype, 'device': device}
        self.callback = callback
        self.started = self.closed = False
        FakeStream.made.append(self)

    def start(self):
        self.started = True

    def abort(self):
        pass

    def close(self):
        self.closed = True

    def blocks(self, count):
        """Run the callback ``count`` times; return the frames it wrote."""
        written = []
        for _ in range(count):
            out = np.full((self.kwargs['blocksize'], 1), 9.0, dtype=np.float32)
            self.callback(out, len(out), None, None)
            written.append(out[:, 0].copy())
        return np.concatenate(written)


@pytest.fixture
def fake_sd(monkeypatch):
    FakeStream.made = []
    devices = [{'name': 'HDMI 0', 'max_output_channels': 2},
               {'name': 'USB Audio mic', 'max_output_channels': 0},
               {'name': 'USB Audio DAC', 'max_output_channels': 2}]
    monkeypatch.setattr(backends, 'sounddevice', types.SimpleNamespace(
        OutputStream=FakeStream, query_devices=lambda: devices))
    return FakeStream


def test_sounddevice_backend_plays_through_a_fake_stream(fake_sd):
    now = [100.0]
    backend = backends.SoundDeviceBackend(RATE, device='usb', linger_s=2.0,
                                          clock=lambda: now[0])
    buf = np.linspace(0.1, 0.2, 5000).astype(np.float32)
    levels, ends = [], []
    backend.start(buf, levels.append, lambda f, s: ends.append((f, s)), name='x')
    stream = fake_sd.made[0]
    assert stream.started and stream.kwargs == {
        'samplerate': RATE, 'blocksize': BLOCK, 'channels': 1, 'dtype': 'float32', 'device': 2}
    frames = stream.blocks(4)                            # 5880 frames: the clip and silence
    assert np.array_equal(frames[:5000], buf) and np.all(frames[5000:] == 0)
    assert len(levels) == 4 and levels[0] == pytest.approx(gain.block_rms(buf[:BLOCK]))
    assert ends == [(5000, False)] and backend.position_frames() == 5000
    stream.blocks(1)                                      # idle: silence, nothing called
    assert len(levels) == 4
    backend.housekeeping()
    assert not stream.closed                              # lingering
    now[0] += 2.0
    backend.housekeeping()
    assert stream.closed and not backend.stream_open


def test_sounddevice_backend_fades_on_stop_and_reuses_the_stream(fake_sd):
    backend = backends.SoundDeviceBackend(RATE, linger_s=2.0)
    buf = np.ones(RATE, dtype=np.float32)
    ends = []
    backend.start(buf, lambda r: None, lambda f, s: ends.append((f, s)))
    stream = fake_sd.made[0]
    assert stream.kwargs['device'] is None
    stream.blocks(2)
    assert backend.stop(20.0)
    frames = stream.blocks(1)
    assert np.allclose(frames[:882], gain.fade_out(882)) and np.all(frames[882:] == 0)
    assert ends == [(2 * BLOCK + 882, True)]
    assert not backend.stop(20.0)
    backend.start(buf, lambda r: None, lambda f, s: ends.append((f, s)))   # still open
    assert len(fake_sd.made) == 1
    assert np.array_equal(stream.blocks(1), buf[:BLOCK])
    backend.start(buf, lambda r: None, lambda f, s: ends.append((f, s)))   # replaced at once
    assert ends[-1] == (BLOCK, True)
    backend.close()
    assert stream.closed


def test_sounddevice_backend_device_not_found(fake_sd):
    with pytest.raises(ValueError, match='HDMI 0, USB Audio DAC'):
        backends.SoundDeviceBackend(RATE, device='nope')


def test_sounddevice_missing_is_an_error(monkeypatch):
    monkeypatch.setattr(backends, 'sounddevice', None)
    with pytest.raises(RuntimeError, match='PortAudio'):
        backends.make_backend('sounddevice', RATE)


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.005)


def test_wav_backend_writes_what_played_in_wall_time(tmp_path):
    backend = backends.WavBackend(RATE, str(tmp_path / 'out'))
    buf = (square(0.3, 0.5, RATE)).astype(np.float32)
    done, levels = threading.Event(), []
    began = time.monotonic()
    backend.start(buf, levels.append, lambda f, s: done.set(), name='voice:42')
    assert done.wait(3.0)
    took = time.monotonic() - began
    assert 0.27 < took < 0.6
    assert len(levels) == 9 and levels[0] == pytest.approx(0.5, rel=1e-3)
    name = os.path.basename(backend.last_path)
    assert name.endswith('Z-voice-42.wav') and len(name) == len('20261024T101010.123Z-') + 12
    data, rate = read_wav(backend.last_path)
    assert rate == RATE and len(data) == len(buf)
    assert np.allclose(data, buf, atol=1e-4)


def test_wav_backend_cut_by_a_stop(tmp_path):
    backend = backends.WavBackend(RATE, str(tmp_path))
    buf = np.full(RATE * 2, 0.5, dtype=np.float32)
    ended = []
    backend.start(buf, lambda r: None, lambda f, s: ended.append((f, s)), name='cut')
    time.sleep(0.2)
    assert backend.stop(20.0)
    wait_for(lambda: ended)
    frames, stopped = ended[0]
    assert stopped and 0.15 * RATE < frames < 0.35 * RATE
    data, _ = read_wav(backend.last_path)
    assert len(data) == frames
    assert np.allclose(data[-882:], 0.5 * gain.fade_out(882), atol=1e-4)


def test_null_backend_writes_nothing(tmp_path):
    backend = backends.NullBackend(RATE)
    done = threading.Event()
    backend.start(np.zeros(100, dtype=np.float32), lambda r: None, lambda f, s: done.set())
    assert done.wait(1.0) and backend.last_path is None and backend.name == 'null'


def test_make_backend_refuses_an_unknown_kind():
    with pytest.raises(ValueError, match='sounddevice, wav, null'):
        backends.make_backend('alsa', RATE)


# --- the worker -----------------------------------------------------------------------------

class Sink:
    def __init__(self):
        self.states, self.levels, self.plays, self.warnings = [], [], [], []
        self.lock = threading.Lock()

    def state(self, s):
        with self.lock:
            self.states.append((time.monotonic(), s))

    def level(self, rms):
        with self.lock:
            self.levels.append((time.monotonic(), rms))

    def playing(self, value):
        self.plays.append(value)

    def warn(self, text):
        self.warnings.append(text)


@pytest.fixture
def worker(tree):
    sounds, _ = tree
    sink = Sink()
    player = Player(backends.NullBackend(RATE), DecodeCache(2 ** 24, RATE), sink, RATE)
    player.start()
    yield player, sink, sounds
    player.close()


def job(sounds, name, **kw):
    gains = {'beep': -6.0}
    return PlayJob(name, os.path.join(sounds, 'Test/%s.wav' % name), gains.get(name, 0.0),
                   kw.pop('gain_db', gains.get(name, 0.0)), **kw)


def last_state(sink):
    with sink.lock:
        return sink.states[-1][1]


def test_worker_plays_then_ends_with_zeros(worker):
    player, sink, sounds = worker
    assert player.play(job(sounds, 'beep', owner='me', priority=100)) == (True, '', 500)
    assert sink.plays == [True]
    state = last_state(sink)
    assert (state.state, state.name, state.owner, state.priority) == (PLAYING, 'beep', 'me', 100)
    wait_for(lambda: last_state(sink).state == IDLE)
    end = last_state(sink)
    assert end.last_end == END_ENDED and end.elapsed_ms == 0 and sink.plays == [True, False]
    playing = [s for _, s in sink.states if s.state == PLAYING]
    assert 2 <= len(playing) <= 6                         # 5 Hz over 0.5 s, and the changes
    assert playing[-1].elapsed_ms + playing[-1].remaining_ms == 500
    levels = [rms for _, rms in sink.levels]
    assert levels[-1] == 0.0
    assert max(levels) == pytest.approx(0.25 * 10 ** (-6 / 20), rel=1e-3)
    assert 10 <= len(levels) <= 20                        # 30 Hz


def test_worker_replace_stop_and_cut(worker):
    player, sink, sounds = worker
    player.play(job(sounds, 'long'))
    time.sleep(0.1)
    assert player.play(job(sounds, 'beep'))[0]
    assert last_state(sink).last_end == END_REPLACED and sink.plays == [True]
    assert player.stop()
    assert last_state(sink).state == FADING
    wait_for(lambda: last_state(sink).state == IDLE)
    assert last_state(sink).last_end == END_STOPPED
    assert not player.stop()
    assert player.play(job(sounds, 'long', duration_ms=200)) == (True, '', 200)
    wait_for(lambda: last_state(sink).state == IDLE)
    assert last_state(sink).last_end == END_DURATION


def test_worker_refuses_a_bad_file_and_after_close(worker, tmp_path):
    player, sink, sounds = worker
    bad = PlayJob('bad', str(tmp_path / 'missing.wav'), 0.0, 0.0)
    accepted, reason, _ = player.play(bad)
    assert not accepted and reason.startswith('decode failed')
    player.close()
    assert player.play(job(sounds, 'beep')) == (False, 'closed', 0)
    assert last_state(sink).state == IDLE and sink.levels[-1][1] == 0.0


def test_worker_close_stops_a_play(worker):
    player, sink, sounds = worker
    player.play(job(sounds, 'long'))
    player.close()
    end = last_state(sink)
    assert end.state == IDLE and end.last_end == END_STOPPED
    assert sink.levels[-1][1] == 0.0 and sink.plays[-1] is False


def test_worker_idle_state_is_one_hertz(worker):
    player, sink, sounds = worker
    time.sleep(2.2)
    assert 2 <= len(sink.states) <= 4 and not sink.levels


def test_a_request_given_up_on_is_never_served(tree):
    sounds, _ = tree
    sink = Sink()
    player = Player(backends.NullBackend(RATE), DecodeCache(2 ** 24, RATE), sink, RATE)
    assert player.play(job(sounds, 'beep'), timeout=0.05) == (False, 'timeout', 0)
    player.start()                                        # the worker comes late
    time.sleep(0.3)
    assert sink.plays == [] and all(s.state == IDLE for _, s in sink.states)
    player.close()


def test_stop_after_a_natural_end_is_false(worker):
    player, sink, sounds = worker
    player.play(job(sounds, 'ooh1'))
    wait_for(lambda: last_state(sink).state == IDLE)
    assert not player.stop() and last_state(sink).last_end == END_ENDED
