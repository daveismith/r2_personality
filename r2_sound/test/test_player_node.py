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

"""
Node tests: ``sound_player`` as a subprocess on the ``wav`` and ``null`` backends.

Everything is test-private: the node runs as ``sound_player_t<pid>_<n>`` (which moves its
services and ``~/state``), ``/sound/level`` and ``/sound/playing`` are remapped under
``/t<pid>_<n>`` and the dome level goes to ``/t<pid>_<n>/dome/audio/level``. The real
``/sound_player`` and ``/dome/audio/level`` are never touched. The catalogue is a handful of
synthetic square waves (``fixtures.make_player_tree``), never ``/sounds``.
"""

import glob
import os
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from astromech_msgs.msg import SoundState
from astromech_msgs.srv import ListSounds, PlaySound, StopSound, Vocalize
from fixtures import clip_entry, make_player_tree, read_wav, write_catalogue  # noqa: I100
import numpy as np
import pytest
from r2_sound.gain import level_to_byte, to_dbfs
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, UInt8
from std_srvs.srv import Trigger
import yaml

STARTUP_S = 40.0
CALL_S = 10.0
IDLE, PLAYING, FADING = 0, 1, 2
ENDED, DURATION, STOPPED, REPLACED = 1, 2, 3, 4
BEEP_AMP = 0.25 * 10 ** (-6 / 20)       # the beep's square wave after its catalogue gain
_counter = 0


class Proc:
    """A subprocess in its own session, its output collected."""

    def __init__(self, argv):
        self.proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, start_new_session=True)
        self.output = []
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self):
        for line in self.proc.stdout:
            self.output.append(line)

    def text(self):
        return ''.join(self.output)

    def send(self, sig):
        try:
            os.killpg(self.proc.pid, sig)
        except ProcessLookupError:
            pass

    def close(self, sig=signal.SIGINT, timeout=15.0):
        """Stop the process as a user would; return its exit code (None if it had to die)."""
        code = None
        self.send(sig)
        try:
            code = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass
        self.send(signal.SIGKILL)
        return code


def wait_until(predicate, timeout, what='condition', output=None):
    """Return the first truthy value of ``predicate()`` within ``timeout`` seconds."""
    deadline = time.monotonic() + timeout
    while True:
        value = predicate()
        if value:
            return value
        if time.monotonic() >= deadline:
            raise AssertionError('timed out on %s\n%s' % (what, output() if output else ''))
        time.sleep(0.01)


class PlayerUnderTest:
    """The player subprocess and a test node on its private names."""

    def __init__(self, tmp_path, backend='wav', extra=(), catalogue=None, wait=True):
        global _counter
        _counter += 1
        self.tag = 't%d_%d' % (os.getpid(), _counter)
        self.name = 'sound_player_%s' % self.tag
        self.sounds, self.catalogue = make_player_tree(tmp_path)
        if catalogue is not None:
            self.catalogue = catalogue(tmp_path, self.sounds, self.catalogue)
        self.wav_dir = str(tmp_path / 'out')
        self.topics = {'level': '/%s/sound/level' % self.tag,
                       'playing': '/%s/sound/playing' % self.tag,
                       'dome': '/%s/dome/audio/level' % self.tag}
        self.node = rclpy.create_node('player_test_%s' % self.tag)
        self.lock = threading.Lock()
        self.states, self.levels, self.domes, self.playing = [], [], [], []
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.node.create_subscription
        sub(SoundState, '/%s/state' % self.name, self._keep(self.states), reliable)
        sub(Float32, self.topics['level'], self._keep(self.levels), reliable)
        sub(UInt8, self.topics['dome'], self._keep(self.domes), reliable)
        sub(Bool, self.topics['playing'], self._keep(self.playing), latched)
        self.clients = {name: self.node.create_client(srv, '/%s/%s' % (self.name, name))
                        for name, srv in (('list', ListSounds), ('play', PlaySound),
                                          ('vocalize', Vocalize), ('stop', StopSound),
                                          ('reload', Trigger))}
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self._spin, daemon=True).start()
        # The executable itself, not `ros2 run`: the exit code is then the node's own (a
        # SIGTERM ends the `ros2 run` wrapper with -15 whatever the node does).
        self.proc = Proc([
            os.path.join(get_package_prefix('r2_sound'), 'lib', 'r2_sound', 'sound_player'),
            '--ros-args',
            '-r', '__node:=%s' % self.name, '-p', 'backend:=%s' % backend,
            '-p', 'catalogue:=%s' % self.catalogue, '-p', 'sounds_dir:=%s' % self.sounds,
            '-p', 'wav_dir:=%s' % self.wav_dir, '-p', 'dome_level_topic:=%s' % self.topics['dome'],
            '-r', '/sound/level:=%s' % self.topics['level'],
            '-r', '/sound/playing:=%s' % self.topics['playing'], *extra])
        if wait:
            for client in self.clients.values():
                wait_until(client.service_is_ready, STARTUP_S, 'the services', self.proc.text)
            self.wait_state(lambda s: s.state == IDLE, STARTUP_S, 'the first IDLE state')

    def _spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def _keep(self, into):
        def keep(msg):
            with self.lock:
                into.append((time.monotonic(), msg))
        return keep

    def call(self, service, **fields):
        client = self.clients[service]
        request = client.srv_type.Request()
        for key, value in fields.items():
            setattr(request, key, value)
        future = client.call_async(request)
        wait_until(future.done, CALL_S, 'the %s call' % service, self.proc.text)
        return future.result()

    def play(self, **fields):
        fields.setdefault('priority', 100)
        return self.call('play', **fields)

    def mark(self):
        """Return the message counts now, to look only at what follows."""
        with self.lock:
            return {k: len(getattr(self, k)) for k in ('states', 'levels', 'domes', 'playing')}

    def since(self, mark, what):
        with self.lock:
            return list(getattr(self, what)[mark[what]:])

    def wait_state(self, predicate, timeout=10.0, what='a state', mark=None):
        start = mark['states'] if mark else 0

        def check():
            with self.lock:
                for _, msg in self.states[start:]:
                    if predicate(msg):
                        return msg
        return wait_until(check, timeout, what, self.proc.text)

    def wait_idle(self, mark, timeout=10.0):
        return self.wait_state(lambda s: s.state == IDLE, timeout, 'IDLE', mark)

    def wavs(self):
        return sorted(glob.glob(os.path.join(self.wav_dir, '*.wav')))

    def close(self, sig=signal.SIGINT):
        code = self.proc.close(sig)
        self.executor.shutdown(timeout_sec=0)
        self.node.destroy_node()
        return code


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    try:
        rclpy.try_shutdown()
    except RuntimeError:          # a zenoh session that is slow to close is not a failure
        pass


@pytest.fixture(scope='module')
def player(tmp_path_factory):
    under_test = PlayerUnderTest(tmp_path_factory.mktemp('player'))
    yield under_test
    under_test.close()


@pytest.fixture
def fresh(tmp_path):
    made = []

    def make(**kwargs):
        made.append(PlayerUnderTest(tmp_path, **kwargs))
        return made[-1]
    yield make
    for item in made:
        item.close()


def play_and_wait(player, **fields):
    mark = player.mark()
    before = set(player.wavs())
    response = player.play(**fields)
    assert response.accepted, response.reason
    end = player.wait_idle(mark)
    new = wait_until(lambda: sorted(set(player.wavs()) - before), 5.0, 'the WAV')
    return response, end, mark, new


# --- the catalogue and the startup ------------------------------------------------------------

def test_list_matches_the_catalogue(player):
    response = player.call('list')
    with open(player.catalogue) as f:
        data = yaml.safe_load(f)
    assert [c.name for c in response.clips] == [c['name'] for c in data['clips']]
    beep = {c.name: c for c in response.clips}['beep']
    assert (list(beep.tags), beep.duration_ms, beep.gain_db, beep.excluded) == (
        ['chatter'], 500, -6.0, False)
    assert {c.name: c for c in response.clips}['music'].excluded
    assert response.catalogue == player.catalogue and response.sample_rate == 44100
    assert response.synth


def test_startup_is_idle_with_zeros(player):
    first = player.states[0][1]
    assert first.state == IDLE and first.last_end == 0 and first.backend == 'wav'
    wait_until(lambda: player.playing, 5.0, 'the latched playing')
    assert player.playing[0][1].data is False


# --- playing ------------------------------------------------------------------------------

def test_play_by_name_writes_the_wav_and_reports_the_play(player):
    response, end, mark, new = play_and_wait(player, name='beep', gain_db=-3.0, owner='me',
                                             intent_id='i1')
    assert (response.name, response.duration_ms) == ('beep', 500)
    assert end.last_end == ENDED
    data, rate = read_wav(new[0])
    assert rate == 44100 and len(data) == 22050 and new[0].endswith('-beep.wav')
    assert np.max(np.abs(data)) == pytest.approx(BEEP_AMP * 10 ** (-3 / 20), abs=1e-3)
    playing = [s for _, s in player.since(mark, 'states') if s.state == PLAYING]
    assert playing and all((s.name, s.owner, s.priority, s.intent_id) == (
        'beep', 'me', 100, 'i1') for s in playing)
    assert all(s.elapsed_ms + s.remaining_ms == 500 for s in playing)
    assert playing[-1].elapsed_ms > playing[0].elapsed_ms


def test_levels_stream_while_playing_and_end_with_a_zero(player):
    _, end, mark, _ = play_and_wait(player, name='long', duration_ms=1000)
    wait_until(lambda: player.since(mark, 'levels')[-1][1].data == 0.0, 1.0, 'the last 0')
    levels = player.since(mark, 'levels')
    domes = player.since(mark, 'domes')
    level = BEEP_AMP * 10 ** (6 / 20)                  # long has no catalogue gain
    loud = [m.data for _, m in levels if m.data > 0]
    assert 20 <= len(loud) <= 40                       # 30 Hz for a second
    assert np.median(loud) == pytest.approx(level, rel=0.01)
    assert levels[-1][1].data == 0.0 and domes[-1][1].data == 0
    byte = level_to_byte(to_dbfs(level))
    assert sum(m.data == byte for _, m in domes) >= 15
    idle_at = [t for t, s in player.since(mark, 'states') if s.state == IDLE][0]
    assert abs(levels[-1][0] - idle_at) < 0.05 and abs(domes[-1][0] - idle_at) < 0.05
    playing = player.since(mark, 'playing')
    assert [m.data for _, m in playing] == [True, False]
    states = [s for _, s in player.since(mark, 'states') if s.state == PLAYING]
    assert 4 <= len(states) <= 9                       # 5 Hz, plus the change


def test_play_by_pool_picks_within_the_pool(player):
    names = set()
    for _ in range(6):
        mark = player.mark()
        response = player.play(pool='ooh')
        assert response.accepted
        names.add(response.name)
        player.wait_idle(mark)
    assert names <= {'ooh1', 'ooh2'} and names


def test_a_second_play_replaces_the_first(player):
    mark = player.mark()
    assert player.play(name='long').accepted
    player.wait_state(lambda s: s.state == PLAYING and s.name == 'long', mark=mark)
    assert player.play(name='beep').accepted
    replaced = player.wait_state(lambda s: s.name == 'beep', mark=mark)
    assert replaced.last_end == REPLACED and replaced.state == PLAYING
    assert player.wait_idle(mark).last_end == ENDED
    assert [m.data for _, m in player.since(mark, 'playing')] == [True, False]


def test_stop_fades(player):
    mark = player.mark()
    before = set(player.wavs())
    assert player.play(name='long').accepted
    time.sleep(0.5)
    assert player.call('stop', owner='test').stopped
    assert player.wait_idle(mark).last_end == STOPPED
    assert not player.call('stop').stopped
    new = wait_until(lambda: sorted(set(player.wavs()) - before), 5.0, 'the WAV')
    data, _ = read_wav(new[0])
    assert 0.4 * 44100 < len(data) < 1.0 * 44100
    tail = np.abs(data[-882:])
    assert tail[-1] < 0.01 and tail[:100].max() > 0.2 and np.all(np.diff(tail[::50]) <= 1e-3)


def test_duration_cuts(player):
    response, end, _, new = play_and_wait(player, name='long', duration_ms=300)
    assert response.duration_ms == 300 and end.last_end == DURATION
    data, _ = read_wav(new[0])
    assert len(data) == 13230 and abs(data[-1]) < 0.01


def test_gain_is_clamped_with_a_warning(player):
    _, _, _, new = play_and_wait(player, name='ooh1', gain_db=30.0)
    data, _ = read_wav(new[0])
    assert 20 * np.log10(np.max(np.abs(data))) == pytest.approx(-1.0, abs=0.1)
    wait_until(lambda: 'clamped' in player.proc.text(), 5.0, 'the clamp warning')


def test_refusals(player):
    for fields, reason in (({'name': 'nope'}, 'unknown name'),
                           ({'pool': 'nope'}, 'unknown pool'),
                           ({'pool': 'sad'}, 'empty pool'),
                           ({'pool': 'excluded'}, 'empty pool'),
                           ({'name': 'music', 'priority': 100}, 'excluded')):
        response = player.play(**fields)
        assert (response.accepted, response.reason) == (False, reason), fields
    for fields, reason in (({'arousal': 1.5}, 'bad arousal: must be 0..1'),
                           ({'valence': float('nan')}, 'bad valence: must be 0..1'),
                           ({'length_s': -1.0}, 'bad length_s: must be >= 0')):
        response = player.call('vocalize', seed=5, priority=100, **fields)
        assert (response.accepted, response.reason, response.seed) == (False, reason, 5)


def test_an_excluded_clip_plays_at_manual_priority(player):
    response, end, _, _ = play_and_wait(player, name='music', priority=199)
    assert response.accepted and end.last_end == ENDED


def test_parameters_are_read_at_startup(player):
    result = subprocess.run(['ros2', 'param', 'set', '/%s' % player.name, 'fade_ms', '50.0'],
                            capture_output=True, text=True, timeout=30)
    assert 'read at startup' in result.stdout + result.stderr


# --- vocalize ----------------------------------------------------------------------------

def vocalize_and_wait(player, **fields):
    fields.setdefault('priority', 100)
    fields.setdefault('arousal', 0.6)
    fields.setdefault('valence', 0.7)
    mark = player.mark()
    before = set(player.wavs())
    response = player.call('vocalize', **fields)
    assert response.accepted, response.reason
    end = player.wait_idle(mark)
    new = wait_until(lambda: sorted(set(player.wavs()) - before), 5.0, 'the WAV')
    return response, end, mark, new


def test_vocalize_plays_a_phrase_as_voice_seed(player):
    response, end, mark, new = vocalize_and_wait(player, seed=42, length_s=1.0, owner='me',
                                                 intent_id='v1')
    assert response.seed == 42 and response.duration_ms == 1000
    assert end.last_end == ENDED
    playing = [s for _, s in player.since(mark, 'states') if s.state == PLAYING]
    assert playing and all((s.name, s.owner, s.priority, s.intent_id) == (
        'voice:42', 'me', 100, 'v1') for s in playing)
    assert all(s.elapsed_ms + s.remaining_ms == 1000 for s in playing)
    assert [m.data for _, m in player.since(mark, 'playing')] == [True, False]
    loud = [m.data for _, m in player.since(mark, 'levels') if m.data > 0]
    assert 15 <= len(loud) <= 40
    assert player.since(mark, 'levels')[-1][1].data == 0.0
    data, rate = read_wav(new[0])
    assert new[0].endswith('-voice-42.wav') and rate == 44100 and len(data) == 44100
    assert 20 * np.log10(np.max(np.abs(data))) <= -1.0 + 0.01


def test_vocalize_seed_0_draws_a_seed_and_a_seed_replays(player):
    response, _, _, _ = vocalize_and_wait(player, seed=0)
    assert response.seed != 0
    assert 600 <= response.duration_ms <= 2500        # the synth's own length
    first = vocalize_and_wait(player, seed=response.seed)[3][0]
    second = vocalize_and_wait(player, seed=response.seed)[3][0]
    with open(first, 'rb') as a, open(second, 'rb') as b:
        assert a.read() == b.read()


def test_vocalize_is_cut_at_the_requested_length(player):
    response, end, _, new = vocalize_and_wait(player, seed=3, length_s=0.2)
    assert response.duration_ms == 200 and end.last_end == DURATION
    data, _ = read_wav(new[0])
    assert len(data) == 8820 and abs(data[-1]) < 0.01


# --- the catalogue on disk -----------------------------------------------------------------

def with_a_missing_clip(tmp_path, sounds, path):
    with open(path) as f:
        clips = yaml.safe_load(f)['clips']
    clips.append(clip_entry('ghost', 'Test/ghost.wav', 300))
    return write_catalogue(path, clips)


def test_a_missing_file_is_dropped_and_reload_reads_the_catalogue_again(fresh):
    player = fresh(catalogue=with_a_missing_clip)
    names = [c.name for c in player.call('list').clips]
    assert 'ghost' not in names and 'beep' in names
    wait_until(lambda: 'ghost' in player.proc.text() and 'missing' in player.proc.text(),
               5.0, 'the missing-file warning')
    with open(player.catalogue) as f:
        clips = [c for c in yaml.safe_load(f)['clips'] if c['name'] != 'beep']
    write_catalogue(player.catalogue, clips + [clip_entry('extra', 'Test/low.wav', 400)])
    response = player.call('reload')
    assert response.success and '1 dropped' in response.message
    names = [c.name for c in player.call('list').clips]
    assert 'beep' not in names and 'extra' in names
    assert player.play(name='beep').reason == 'unknown name'
    with open(player.catalogue, 'w') as f:
        f.write('clips: [\n')
    assert not player.call('reload').success
    assert 'extra' in [c.name for c in player.call('list').clips]


def test_the_null_backend_writes_nothing(fresh):
    player = fresh(backend='null')
    mark = player.mark()
    assert player.play(name='low').duration_ms == 400   # 22.05 kHz, resampled
    assert player.wait_idle(mark).backend == 'null'
    assert player.wavs() == []


def test_a_bad_parameter_exits_1(fresh):
    player = fresh(extra=('-p', 'backend:=alsa'), wait=False)
    assert player.proc.proc.wait(timeout=STARTUP_S) == 1
    assert 'backend must be one of' in player.proc.text()


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_a_signal_stops_the_play_publishes_zeros_and_exits_0(fresh, sig):
    player = fresh()
    mark = player.mark()
    assert player.play(name='long').accepted
    wait_until(lambda: any(m.data > 0 for _, m in player.since(mark, 'domes')), 5.0,
               'a level')
    began = time.monotonic()
    code = player.proc.close(sig, timeout=15.0)
    assert code == 0, player.proc.text()
    assert time.monotonic() - began < 15.0
    end = player.wait_state(lambda s: s.state == IDLE, 1.0, 'the last IDLE', mark)
    assert end.last_end == STOPPED
    assert player.since(mark, 'levels')[-1][1].data == 0.0
    assert player.since(mark, 'domes')[-1][1].data == 0
    assert player.since(mark, 'playing')[-1][1].data is False
