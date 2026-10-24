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
Integration tests: ``sound_arbiter`` against a FAKE sound player, each as a subprocess.

The real ``sound_player`` (the ``r2_sound`` package) is built in parallel with this channel, so
the device here is a small fake in this file (run as ``python3 test_sound_node.py --fake``): it
serves ``~/list``, ``~/play``, ``~/vocalize`` and ``~/stop`` with the plan's refusal reasons and
publishes ``~/state`` at 5 Hz, ``elapsed_ms`` and ``remaining_ms`` advancing on wall time and
``last_end`` set on an end, a replacement or a stop (phase 3 plan 5.1). It prints one line per
request (``FAKE PLAY ...``), which the tests read. **Step 8a of the plan exercises the real
player** (``sound_player`` on the ``null`` and ``wav`` backends with this arbiter); nothing here
stands in for that.

Everything is test-private, as in ``test_nodes.py`` (whose ``Proc`` and ``wait_until`` are
reused): the fake runs as ``sound_player_t<tag>`` (which moves its services and state topic),
the arbiter takes ``device`` of that name, and every ``/expression/*`` and ``/mood/state`` name
is remapped under ``/t<tag>``. The real ``/sound_player`` and ``/sound/*`` are never touched.
"""

import json
import os
import random
import re
import signal
import sys
import threading
import time

from astromech_msgs.msg import ChannelStatus
from astromech_msgs.msg import ExpressionIntent
from astromech_msgs.msg import MoodState
from astromech_msgs.msg import SoundClip
from astromech_msgs.msg import SoundState
from astromech_msgs.srv import ClaimChannel
from astromech_msgs.srv import ListSounds
from astromech_msgs.srv import PlaySound
from astromech_msgs.srv import StopSound
from astromech_msgs.srv import Vocalize
from fixtures import sound_data
import pytest
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from test_nodes import Proc, ros_context, STARTUP_S, wait_until  # noqa: F401
from test_nodes import STATE_CLAIMED, STATE_IDLE, STATE_OFFLINE, STATE_RUNNING
from test_nodes import STATE_STALE
import yaml

CLIPS = (       # name, tags, duration_ms, excluded: the fake's six-entry catalogue
    ('sent1', ['chatter'], 1500, False), ('sent2', ['chatter'], 2500, False),
    ('yay', ['excited'], 1200, False), ('scream', ['scream'], 3000, False),
    ('hum1', ['hum'], 20000, False), ('cantina', ['excluded'], 120000, True))
EXTRA_CLIP = ('newclip', ['chatter'], 800, False)       # in the catalogue after a restart


def fake_main(argv):
    """Run the fake player until it is killed: ``--fake [--synth] [--extra] --ros-args ...``."""
    class FakePlayer(Node):

        def __init__(self, synth, extra):
            super().__init__('fake_sound_player')
            self.synth = synth
            self.clips = CLIPS + ((EXTRA_CLIP,) if extra else ())
            self.current = None
            self.last_end = 0
            self.rng = random.Random(1)
            self.create_service(ListSounds, '~/list', self.on_list)
            self.create_service(PlaySound, '~/play', self.on_play)
            self.create_service(Vocalize, '~/vocalize', self.on_vocalize)
            self.create_service(StopSound, '~/stop', self.on_stop)
            self.pub = self.create_publisher(
                SoundState, '~/state', QoSProfile(depth=10,
                                                  reliability=ReliabilityPolicy.RELIABLE))
            self.create_timer(0.2, self.publish)
            self.create_timer(0.05, self.watch)

        def on_list(self, request, response):
            print('FAKE LIST', flush=True)
            response.clips = [SoundClip(name=n, tags=t, duration_ms=d, gain_db=-2.0,
                                        excluded=x) for n, t, d, x in self.clips]
            response.catalogue, response.sample_rate, response.synth = 'fake', 44100, self.synth
            return response

        def begin(self, name, length_ms, cut, request):
            if self.current is not None:
                self.last_end = 4
            self.current = {'name': name, 'owner': request.owner, 'priority': request.priority,
                            'intent_id': request.intent_id, 'start': time.monotonic(),
                            'length_ms': length_ms, 'cut': cut}
            self.publish()

        def on_play(self, request, response):
            print('FAKE PLAY name=%s pool=%s gain=%.1f dur=%d prio=%d owner=%s id=%s' % (
                request.name or '-', request.pool or '-', request.gain_db, request.duration_ms,
                request.priority, request.owner, request.intent_id), flush=True)
            usable = [c for c in self.clips if not c[3]]
            if bool(request.name) == bool(request.pool):
                response.reason = 'unknown name'
                return response
            if request.name:
                pick = [c for c in self.clips if c[0] == request.name]
                response.reason = '' if pick else 'unknown name'
            else:
                pick = [c for c in usable if request.pool in c[1]]
                response.reason = '' if pick else (
                    'empty pool' if any(request.pool in c[1] for c in self.clips)
                    else 'unknown pool')
            if not pick:
                return response
            name, _, length_ms, _ = self.rng.choice(pick)
            cut = 0 < request.duration_ms < length_ms
            length_ms = request.duration_ms if cut else length_ms
            self.begin(name, length_ms, cut, request)
            response.accepted, response.name, response.duration_ms = True, name, length_ms
            return response

        def on_vocalize(self, request, response):
            print('FAKE VOCALIZE seed=%d length=%.1f prio=%d owner=%s id=%s' % (
                request.seed, request.length_s, request.priority, request.owner,
                request.intent_id), flush=True)
            if not self.synth:
                response.reason = 'no synth'
                return response
            length_ms = int(request.length_s * 1000) or 1000
            self.begin('voice:%d' % request.seed, length_ms, False, request)
            response.accepted, response.seed, response.duration_ms = True, request.seed, length_ms
            return response

        def on_stop(self, request, response):
            print('FAKE STOP owner=%s' % request.owner, flush=True)
            response.stopped = self.current is not None
            if self.current is not None:
                self.current, self.last_end = None, 3
                self.publish()
            return response

        def watch(self):
            cur = self.current
            if cur is not None and (time.monotonic() - cur['start']) * 1000 >= cur['length_ms']:
                self.current, self.last_end = None, 2 if cur['cut'] else 1
                self.publish()

        def publish(self):
            msg = SoundState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.backend = 'fake'
            msg.last_end = self.last_end
            cur = self.current
            if cur is not None:
                elapsed = int((time.monotonic() - cur['start']) * 1000)
                msg.state, msg.name, msg.owner = SoundState.PLAYING, cur['name'], cur['owner']
                msg.priority, msg.intent_id = cur['priority'], cur['intent_id']
                msg.elapsed_ms = elapsed
                msg.remaining_ms = max(0, cur['length_ms'] - elapsed)
            self.pub.publish(msg)

    rclpy.init(args=argv)
    node = FakePlayer(synth='--synth' in argv, extra='--extra' in argv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


OWNER = 'r2_expression'
_counter = 0


def config_text(max_duration_ms=None):
    """
    Return the test config: short staleness, a 1 s manual hold, only excited chatters.

    ``excited`` has its chatter every 1.5 s so a test sees it quickly; ``calm`` has none, so
    the plays a test sends are the only plays.
    """
    data = sound_data()
    data['defaults'].update(mood_stale_s=1.0, device_stale_s=2.0, status_hz=5.0,
                            manual_hold_s=1.0)
    if max_duration_ms is not None:
        data['defaults']['max_duration_ms'] = max_duration_ms
    del data['moods']['calm']['sound']['chatter']
    data['moods']['excited']['sound']['chatter']['every_s'] = [1.5, 1.5]
    return yaml.safe_dump(data)


def parse_log(proc, verb):
    """Return the ``FAKE <verb>`` lines of the fake's output as dicts of their key=values."""
    out = []
    for line in proc.text().splitlines():
        match = re.match(r'FAKE %s ?(.*)$' % verb, line)
        if match:
            out.append(dict(token.split('=', 1) for token in match.group(1).split()))
    return out


class SoundRig:
    """The fake player, ``sound_arbiter`` and a test node that plays everyone else."""

    def __init__(self, tmp_path, max_duration_ms=None, player_args=(), start_player=True):
        global _counter
        _counter += 1
        self.tag = 'sn%d_%d' % (os.getpid(), _counter)
        self.player_name = 'sound_player_t%s' % self.tag
        self.arbiter_name = 'sound_arbiter_t%s' % self.tag
        self.topics = {name: '/t%s%s' % (self.tag, name) for name in (
            '/expression/intent', '/mood/state', '/expression/status',
            '/expression/claim/sound')}
        self.config = str(tmp_path / ('expression_%s.yaml' % self.tag))
        with open(self.config, 'w') as f:
            f.write(config_text(max_duration_ms))
        self.player_args = player_args
        self.player = self.start_player() if start_player else None
        self.node = rclpy.create_node('sound_test_%s' % self.tag)
        self.lock = threading.Lock()
        self.status = None
        self.history = []
        self.state = None
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(ChannelStatus, self.topics['/expression/status'],
                                      self._on_status, reliable)
        self.node.create_subscription(SoundState, '/%s/state' % self.player_name,
                                      self._on_state, reliable)
        self.intents = self.node.create_publisher(
            ExpressionIntent, self.topics['/expression/intent'], reliable)
        self.mood_pub = self.node.create_publisher(
            MoodState, self.topics['/mood/state'], reliable)
        self.claims = self.node.create_client(ClaimChannel,
                                              self.topics['/expression/claim/sound'])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        self._mood = None
        self._mood_stop = threading.Event()
        self._mood_thread = None
        self.arbiter = Proc(self._arbiter_argv())
        self.wait_ready()

    # --- processes ---------------------------------------------------------------------------

    def start_player(self, *extra):
        self.player = Proc([sys.executable, os.path.abspath(__file__), '--fake',
                            *self.player_args, *extra, '--ros-args',
                            '-r', '__node:=%s' % self.player_name])
        return self.player

    def kill_player(self):
        self.player.close(signal.SIGKILL)

    def _arbiter_argv(self):
        remaps = []
        for name, target in self.topics.items():
            remaps += ['-r', '%s:=%s' % (name, target)]
        return ['ros2', 'run', 'r2_expression', 'sound_arbiter', '--ros-args',
                '-r', '__node:=%s' % self.arbiter_name, '-p', 'device:=/%s' % self.player_name,
                '-p', 'config:=%s' % self.config, *remaps]

    def close(self):
        self.stop_mood()
        self.arbiter.close()
        if self.player is not None:
            self.player.close(signal.SIGKILL)
        self.executor.shutdown()
        self.node.destroy_node()

    def output(self):
        return '--- player ---\n%s\n--- arbiter ---\n%s' % (
            self.player.text() if self.player else '', self.arbiter.text())

    # --- test-side topics --------------------------------------------------------------------

    def _on_status(self, msg):
        with self.lock:
            self.status = msg
            self.history.append((time.monotonic(), msg))

    def _on_state(self, msg):
        self.state = msg

    def wait_ready(self):
        self.wait_status(lambda s: s.state != STATE_OFFLINE, STARTUP_S, 'ready')
        for pub in (self.intents, self.mood_pub):
            wait_until(lambda pub=pub: pub.get_subscription_count() >= 1, 30.0,
                       'subscriptions matched', self.output)

    def wait_status(self, predicate, timeout=8.0, what=''):
        def check():
            with self.lock:
                msg = self.status
            return msg if msg is not None and predicate(msg) else None
        return wait_until(check, timeout, 'sound status %s' % what, self.output)

    def start_mood(self, label, strength=1.0):
        """Publish ``/mood/state`` at 5 Hz with ``label`` until :meth:`stop_mood`."""
        self._mood = (label, strength)
        if self._mood_thread is None or not self._mood_thread.is_alive():
            self._mood_stop.clear()
            self._mood_thread = threading.Thread(target=self._mood_loop, daemon=True)
            self._mood_thread.start()
        self.wait_status(lambda m: m.mood == label, 10.0, 'mood %s' % label)

    def _mood_loop(self):
        while not self._mood_stop.wait(0.2):
            msg = MoodState()
            msg.mood, msg.mood_strength = self._mood
            self.mood_pub.publish(msg)

    def stop_mood(self):
        self._mood_stop.set()
        if self._mood_thread is not None:
            self._mood_thread.join(timeout=2.0)

    def intent(self, source, priority, action, params=None, duration_ms=0, interruptible=False,
               intent_id='', channel='sound'):
        msg = ExpressionIntent()
        msg.channel, msg.source, msg.priority, msg.action = channel, source, priority, action
        msg.params = json.dumps(params) if params is not None else ''
        msg.duration_ms, msg.interruptible, msg.intent_id = duration_ms, interruptible, intent_id
        self.intents.publish(msg)

    def claim(self, owner='op', priority=150, lease_s=10.0):
        request = ClaimChannel.Request()
        request.channel, request.owner = 'sound', owner
        request.priority, request.lease_s = priority, lease_s
        future = self.claims.call_async(request)
        wait_until(future.done, 10.0, 'claim reply', self.output)
        return future.result()

    def get_dropped(self):
        with self.lock:
            return self.status.dropped if self.status is not None else 0

    def plays(self, player=None):
        return parse_log(player or self.player, 'PLAY')

    def wait_plays(self, count, timeout=8.0, what='plays', player=None):
        return wait_until(lambda: len(self.plays(player)) >= count and self.plays(player),
                          timeout, what, self.output)

    def wait_playing(self, name, timeout=8.0):
        return wait_until(lambda: self.state is not None and self.state.state == 1 and
                          self.state.name == name and self.state, timeout, 'playing ' + name,
                          self.output)


@pytest.fixture
def rig(tmp_path):
    made = []

    def make(**kwargs):
        made.append(SoundRig(tmp_path, **kwargs))
        return made[-1]
    yield make
    for item in made:
        item.close()


# --- a play runs ------------------------------------------------------------------------------

def test_a_manual_play_runs_and_shows_on_status_with_the_players_owner_and_priority(rig):
    s = rig()
    s.start_mood('calm')
    s.intent('manual', 199, 'play', {'name': 'sent2'}, intent_id='m1')
    status = s.wait_status(lambda m: m.state == STATE_RUNNING, 8.0, 'running')
    assert (status.active_source, status.active_action, status.active_intent_id) == (
        'manual', 'play', 'm1')
    assert (status.owner, status.priority) == ('manual', 199)          # the auto-claim
    (play,) = s.plays()
    assert (play['name'], play['pool'], play['prio'], play['owner'], play['id']) == (
        'sent2', '-', '199', OWNER, 'm1')
    assert play['dur'] == '0'                                          # manual: the clip's own
    state = s.wait_playing('sent2')
    assert (state.owner, state.priority, state.intent_id) == (OWNER, 199, 'm1')
    status = s.wait_status(lambda m: 0 < m.active_remaining_ms <= 2500, 4.0, 'remaining')
    assert status.active_action == 'play'
    s.wait_status(lambda m: m.state != STATE_RUNNING, 10.0, 'the play ended on the player')
    assert s.state.last_end == 1 and s.get_dropped() == 0


# --- chatter ----------------------------------------------------------------------------------

def test_chatter_under_a_locked_excited_mood_arrives_within_its_every_s_from_its_pools(rig):
    s = rig()
    started = time.monotonic()
    s.start_mood('excited')
    first = s.wait_plays(1, 6.0, 'first chatter')
    assert time.monotonic() - started < 1.5 + 4.0                     # every_s plus startup
    s.wait_plays(3, 15.0, 'three chatters')
    plays = s.plays()
    assert {p['name'] for p in plays} <= {'yay', 'sent1', 'sent2'}    # excited, chatter pools
    for play in plays:
        assert (play['prio'], play['owner'], play['pool']) == ('10', OWNER, '-')
        assert play['id'].startswith('chatter-') and float(play['gain']) == 2.0
        assert int(play['dur']) == {'yay': 1200, 'sent1': 1500, 'sent2': 2500}[play['name']]
    assert first[0]['name'] in ('yay', 'sent1', 'sent2')


# --- interruptibility -------------------------------------------------------------------------

def test_an_idle_play_is_not_cut_by_an_equal_priority_behavior_play_but_is_by_manual(rig):
    s = rig()
    s.start_mood('calm')
    s.intent('idle', 10, 'play', {'name': 'hum1'}, intent_id='idle1')
    s.wait_playing('hum1')
    s.intent('behavior', 10, 'play', {'name': 'yay'}, intent_id='beh1')
    wait_until(lambda: s.get_dropped() == 1, 8.0, 'the behaviour play dropped', s.output)
    assert [p['name'] for p in s.plays()] == ['hum1'] and s.state.name == 'hum1'
    s.intent('manual', 199, 'play', {'name': 'yay'}, intent_id='man1')
    s.wait_playing('yay')
    assert [p['name'] for p in s.plays()] == ['hum1', 'yay']
    assert s.wait_status(lambda m: m.active_intent_id == 'man1').active_source == 'manual'
    assert s.get_dropped() == 1


def test_interruptible_true_on_the_running_intent_lets_the_equal_priority_newer_win(rig):
    s = rig()
    s.start_mood('calm')
    s.intent('idle', 10, 'play', {'name': 'hum1'}, interruptible=True, intent_id='idle1')
    s.wait_playing('hum1')
    s.intent('behavior', 10, 'play', {'name': 'yay'}, intent_id='beh1')
    s.wait_playing('yay')
    status = s.wait_status(lambda m: m.active_intent_id == 'beh1', 8.0, 'the newer')
    assert (status.active_source, status.dropped) == ('behavior', 0)
    assert s.state.last_end in (0, 4)                                  # replaced, then playing


# --- the duration cap -------------------------------------------------------------------------

def test_duration_is_capped_for_idle_and_behavior_plays_and_a_manual_one_is_not(rig):
    s = rig(max_duration_ms=3000)
    s.start_mood('calm')
    s.intent('idle', 10, 'play', {'name': 'hum1'}, intent_id='i1')
    s.wait_playing('hum1')
    s.intent('behavior', 100, 'play', {'name': 'sent2'}, intent_id='b1')     # 2.5 s clip
    s.wait_playing('sent2')
    s.intent('manual', 199, 'play', {'name': 'cantina'}, intent_id='m1')
    s.wait_playing('cantina')
    assert [(p['name'], p['dur']) for p in s.plays()] == [
        ('hum1', '3000'), ('sent2', '2500'), ('cantina', '0')]
    assert s.state.remaining_ms > 100000                               # nothing cut it
    s.intent('manual', 199, 'stop', intent_id='st1')
    wait_until(lambda: s.state.state == 0, 8.0, 'stopped', s.output)
    assert parse_log(s.player, 'STOP') and s.state.last_end == 3


def test_a_clip_that_ends_before_the_cap_ends_the_run_with_the_players_word(rig):
    s = rig(max_duration_ms=1000)
    s.start_mood('calm')
    s.intent('behavior', 100, 'play', {'name': 'sent1'}, intent_id='b1')     # 1.5 s clip
    s.wait_playing('sent1')
    wait_until(lambda: s.state.state == 0, 8.0, 'cut at the cap', s.output)
    assert s.state.last_end == 2                                       # duration, not ended
    assert s.plays()[0]['dur'] == '1000'
    s.wait_status(lambda m: m.state == STATE_IDLE, 8.0, 'idle again')


# --- claims, stale, offline -------------------------------------------------------------------

def test_a_claim_stops_the_personalitys_play_and_refuses_an_idle_one(rig):
    s = rig()
    s.start_mood('calm')
    s.intent('idle', 10, 'play', {'name': 'hum1'}, intent_id='i1')
    s.wait_playing('hum1')
    assert s.claim('op', 150, 20.0).accepted
    wait_until(lambda: s.state.state == 0 and parse_log(s.player, 'STOP'), 8.0, 'stopped',
               s.output)
    assert s.wait_status(lambda m: m.state == STATE_CLAIMED, 8.0, 'claimed').owner == 'op'
    s.intent('idle', 10, 'play', {'name': 'sent1'}, intent_id='i2')
    s.intent('behavior', 100, 'play', {'name': 'sent1'}, intent_id='b2')       # below 150
    wait_until(lambda: s.get_dropped() == 2, 8.0, 'both dropped and counted', s.output)
    s.intent('manual', 199, 'play', {'name': 'yay'}, intent_id='m1')
    s.wait_playing('yay')
    assert [p['name'] for p in s.plays()] == ['hum1', 'yay']


def test_stale_stops_an_idle_play(rig):
    s = rig()
    s.start_mood('calm')
    s.intent('idle', 10, 'play', {'name': 'hum1'}, intent_id='i1')
    s.wait_playing('hum1')
    s.stop_mood()
    s.wait_status(lambda m: m.state == STATE_STALE, 10.0, 'stale')
    wait_until(lambda: s.state.state == 0 and parse_log(s.player, 'STOP'), 8.0, 'stopped',
               s.output)
    assert s.state.last_end == 3


def test_stale_silences_the_chatter(rig):
    s = rig()
    s.start_mood('excited')
    s.wait_plays(1, 8.0, 'a chatter')
    s.stop_mood()
    s.wait_status(lambda m: m.state == STATE_STALE, 10.0, 'stale')
    time.sleep(1.0)                                                    # a play may be finishing
    count = len(s.plays())
    time.sleep(5.0)                                                    # >3 slots of 1.5 s
    assert len(s.plays()) == count


def test_killing_the_player_gives_offline_and_restarting_it_lists_again(rig):
    s = rig()
    s.start_mood('calm')
    assert len(parse_log(s.player, 'LIST')) == 1
    s.kill_player()
    s.wait_status(lambda m: m.state == STATE_OFFLINE, 15.0, 'offline')
    s.intent('behavior', 100, 'play', {'name': 'sent1'}, intent_id='lost')
    wait_until(lambda: s.get_dropped() == 1, 8.0, 'dropped while offline', s.output)
    old = s.player
    s.start_player('--extra')                                           # one more clip now
    s.wait_status(lambda m: m.state != STATE_OFFLINE, 40.0, 'back')
    assert parse_log(s.player, 'LIST') or wait_until(
        lambda: parse_log(s.player, 'LIST'), 15.0, 'listed again', s.output)
    s.intent('manual', 199, 'play', {'name': 'newclip'}, intent_id='m1')   # only after the list
    s.wait_playing('newclip')
    assert s.plays(old) == [] and [p['name'] for p in s.plays()] == ['newclip']
    assert 'listing again' in s.arbiter.text()


# --- refusals ---------------------------------------------------------------------------------

def test_clip_vocalize_without_a_synth_and_unknown_names_are_refused_and_counted(rig):
    s = rig()
    s.start_mood('calm')
    s.intent('manual', 199, 'clip', {'name': 'sent1'}, intent_id='c1')            # use play
    s.intent('manual', 199, 'vocalize', {'seed': 3}, intent_id='v1')              # no synth
    s.intent('manual', 199, 'play', {'name': 'nope'}, intent_id='p1')             # unknown name
    s.intent('manual', 199, 'play', {'pool': 'nope'}, intent_id='p2')             # unknown pool
    wait_until(lambda: s.get_dropped() == 4, 10.0, 'four refusals counted', s.output)
    time.sleep(0.5)
    assert s.plays() == [] and parse_log(s.player, 'VOCALIZE') == []


def test_a_pool_play_asks_the_player_for_the_pool_and_a_vocalize_reaches_its_service(rig):
    s = rig(player_args=('--synth',))
    s.start_mood('calm')
    s.intent('manual', 199, 'play', {'pool': 'scream', 'gain_db': -4}, interruptible=True,
             intent_id='p1')
    s.wait_playing('scream')
    (play,) = s.plays()
    assert (play['name'], play['pool'], play['gain']) == ('-', 'scream', '-4.0')
    s.intent('manual', 199, 'vocalize', {'seed': 7, 'length_s': 1.5}, intent_id='v1')
    s.wait_playing('voice:7')
    (voice,) = parse_log(s.player, 'VOCALIZE')
    assert (voice['seed'], voice['length'], voice['prio'], voice['owner']) == (
        '7', '1.5', '199', OWNER)


# --- the graph --------------------------------------------------------------------------------

def _names(entries):
    return {name for name, _ in entries}


def test_the_only_client_of_the_play_vocalize_and_stop_services_is_the_arbiter(rig):
    s = rig()
    node = s.node
    dev = '/%s' % s.player_name
    wanted = {dev + '/' + x for x in ('list', 'play', 'vocalize', 'stop')}
    clients = lambda name: _names(node.get_client_names_and_types_by_node(name, '/'))  # noqa
    wait_until(lambda: clients(s.arbiter_name) == wanted, 10.0, 'arbiter clients', s.output)
    subs = _names(node.get_subscriber_names_and_types_by_node(s.arbiter_name, '/'))
    assert subs == {s.topics['/expression/intent'], s.topics['/mood/state'], dev + '/state'}
    pubs = _names(node.get_publisher_names_and_types_by_node(s.arbiter_name, '/'))
    assert pubs == {s.topics['/expression/status'], '/rosout', '/parameter_events'}
    assert s.topics['/expression/claim/sound'] in _names(
        node.get_service_names_and_types_by_node(s.arbiter_name, '/'))
    controls = {dev + '/play', dev + '/vocalize', dev + '/stop'}
    users = [name for name, ns in node.get_node_names_and_namespaces()
             if name.endswith(s.tag) and controls & clients(name)]
    assert users == [s.arbiter_name]            # not the fake, not this test, not express


if __name__ == '__main__' and '--fake' in sys.argv:
    fake_main(sys.argv)
