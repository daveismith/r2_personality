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
Integration tests: the real node as a subprocess, driven over ROS 2 topics and services.

Each test starts its own node with a small YAML (long taus unless the test is about
relaxing, short dwell) written to ``tmp_path``, and talks to it with a short-lived rclpy
node spinning in a background thread.
"""

import math
import os
import signal
import subprocess
import threading
import time

from astromech_msgs.msg import MoodState
from astromech_msgs.msg import Stimulus
from astromech_msgs.srv import SetMood
import pytest
from r2_affect.config import load_config
from r2_affect.model import AffectModel
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

CONFIG = """
temperament:      {{arousal: 0.0, valence: 0.2, curiosity: 0.4, confidence: 0.6}}
relax_tau_s:      {{arousal: {tau}, valence: {tau}, curiosity: {tau}, confidence: {tau}}}
hysteresis:       {{margin: 0.15, dwell_s: {dwell}}}
focus:            {{default_ttl_s: 5.0}}
idle:             {{after_s: 3000, ramp_s: 100}}
impulses:
  bump:           {{arousal: {bump}, valence: -0.2}}
  cheer:          {{arousal: 0.8, valence: 0.6}}
  slow:           {{arousal: 0.1, refractory_s: 100}}
  quick:          {{arousal: 0.01, refractory_s: 2.0}}
  idle:           {{arousal: -0.02}}
moods:
  calm:     {{arousal: 0.0,  valence: 0.2}}
  excited:  {{arousal: 0.8,  valence: 0.8}}
  sleepy:   {{arousal: -0.8, valence: 0.0}}
"""

STARTUP_TIMEOUT_S = 30.0
_counter = 0


def render(tau=1000.0, bump=0.3, dwell=30):
    return CONFIG.format(tau=tau, bump=bump, dwell=dwell)


def write_config(path, tau=1000.0, bump=0.3, dwell=30):
    path.write_text(render(tau, bump, dwell))
    return str(path)


class Harness:
    """A running node plus a test-side rclpy node."""

    def __init__(self, config_path, extra_args=()):
        self.config_path = config_path
        # A live stack (the sim, the robot) may have its own /r2_affect on the graph, so
        # this node gets its own name and topics.
        global _counter
        _counter += 1
        tag = '%d_%d' % (os.getpid(), _counter)
        self.node_name = 'r2_affect_t%s' % tag
        self.topics = {name: '/t%s%s' % (tag, name)
                       for name in ('/stimuli', '/mood/state', '/mood/set',
                                    '/mood/reload_config')}
        self.clock_topic = '/t%s/clock' % tag
        remaps = ['-r', '__node:=%s' % self.node_name, '-r', '/clock:=%s' % self.clock_topic]
        for name, target in self.topics.items():
            remaps += ['-r', '%s:=%s' % (name, target)]
        self.proc = subprocess.Popen(
            ['ros2', 'run', 'r2_affect', 'r2_affect', '--ros-args', '-p',
             'config:=%s' % config_path, *remaps, *extra_args],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True)
        self.output = []
        threading.Thread(target=self._drain, daemon=True).start()

        self.node = rclpy.create_node('r2_affect_test_%s' % tag)
        self.lock = threading.Lock()
        self.states = []
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(
            MoodState, self.topics['/mood/state'], self._on_state, reliable)
        self.stim_pub = self.node.create_publisher(Stimulus, self.topics['/stimuli'], reliable)
        self.clock_pub = self.node.create_publisher(Clock, self.clock_topic, 10)
        self.set_client = self.node.create_client(SetMood, self.topics['/mood/set'])
        self.reload_client = self.node.create_client(Trigger, self.topics['/mood/reload_config'])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spinner = threading.Thread(target=self.executor.spin, daemon=True)
        self.spinner.start()

    def _drain(self):
        for line in self.proc.stdout:
            self.output.append(line)

    def _on_state(self, msg):
        with self.lock:
            self.states.append(msg)

    def close(self):
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.executor.shutdown()
        self.node.destroy_node()

    # --- helpers ---------------------------------------------------------------------------

    def wait_for(self, predicate, timeout=10.0, what='condition'):
        """Return the first state message after now for which predicate is true."""
        deadline = time.monotonic() + timeout
        with self.lock:
            seen = len(self.states)
        while time.monotonic() < deadline:
            with self.lock:
                fresh = self.states[seen:]
                seen = len(self.states)
            for msg in fresh:
                if predicate(msg):
                    return msg
            time.sleep(0.01)
        raise AssertionError('timed out waiting for %s\nnode output:\n%s' % (
            what, ''.join(self.output[-30:])))

    def wait_log(self, text, timeout=5.0):
        """Wait until the node's output has a line containing ``text``; return all output."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            joined = ''.join(self.output)
            if text in joined:
                return joined
            time.sleep(0.05)
        raise AssertionError('no %r in node output:\n%s' % (text, ''.join(self.output)))

    def next_state(self, timeout=5.0):
        return self.wait_for(lambda m: True, timeout, 'a /mood/state message')

    def publish_clock(self, t):
        msg = Clock()
        msg.clock.sec = int(t)
        msg.clock.nanosec = int(round((t - int(t)) * 1e9))
        self.clock_pub.publish(msg)

    def wait_up(self, sim_t0=None):
        if sim_t0 is None:
            self.next_state(STARTUP_TIMEOUT_S)
        else:
            # A sim-time node ticks only when /clock moves; keep nudging it until it does.
            deadline = time.monotonic() + STARTUP_TIMEOUT_S
            with self.lock:
                seen = len(self.states)
            step = 0
            while time.monotonic() < deadline:
                step += 1
                self.publish_clock(sim_t0 + 0.1 * step)
                time.sleep(0.1)
                with self.lock:
                    if any(m.header.stamp.sec > 0 for m in self.states[seen:]):
                        self.sim_t = sim_t0 + 0.1 * step
                        break
            else:
                raise AssertionError('no /mood/state\n' + ''.join(self.output[-30:]))
        assert self.set_client.wait_for_service(timeout_sec=10.0)
        assert self.reload_client.wait_for_service(timeout_sec=10.0)

    def stimulate(self, kind, intensity=1.0, confidence=1.0, source='test', subject='',
                  bearing=math.nan):
        msg = Stimulus()
        msg.subject = subject
        msg.kind = kind
        msg.source = source
        msg.intensity = float(intensity)
        msg.confidence = float(confidence)
        msg.bearing_rad = bearing
        self.stim_pub.publish(msg)

    def call(self, client, request, timeout=10.0):
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        assert done.wait(timeout), 'service call timed out'
        return future.result()

    def set_mood(self, mood='', lock=False, duration=0.0, source='test'):
        request = SetMood.Request()
        request.mood = mood
        request.lock = lock
        request.duration_s = float(duration)
        request.source = source
        return self.call(self.set_client, request)

    def reload(self):
        return self.call(self.reload_client, Trigger.Request())


def vector(msg):
    return [msg.arousal, msg.valence, msg.curiosity, msg.confidence]


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def start_node(tmp_path, request):
    """Return a factory that starts a node on a fresh YAML; finalisers kill it."""
    harnesses = []

    def start(tau=1000.0, bump=0.3, extra_args=(), sim_t0=None, dwell=30):
        config = write_config(tmp_path / ('affect%d.yaml' % len(harnesses)), tau, bump, dwell)
        harness = Harness(config, extra_args)
        harnesses.append(harness)
        harness.wait_up(sim_t0)
        return harness

    yield start
    for harness in harnesses:
        harness.close()


def test_stimulus_moves_state_by_the_impulse(start_node):
    h = start_node(bump=0.3)
    before = h.next_state()
    h.stimulate('bump', intensity=0.5)
    after = h.wait_for(lambda m: m.arousal > before.arousal + 0.05, what='the bump')
    # tau 1000 s: the relax between the stimulus and this tick is below 1e-4.
    assert after.arousal - before.arousal == pytest.approx(0.15, abs=2e-3)
    assert after.valence - before.valence == pytest.approx(-0.1, abs=2e-3)
    assert after.curiosity == pytest.approx(before.curiosity, abs=1e-3)
    assert after.idle_s < before.idle_s + 1.0
    assert after.header.stamp.sec > 0


def test_state_is_published_every_tick(start_node):
    h = start_node()
    h.next_state()
    with h.lock:
        start = len(h.states)
    time.sleep(2.0)
    with h.lock:
        count = len(h.states) - start
    assert 7 <= count <= 13, 'expected about 10 messages in 2 s (5 Hz), got %d' % count


def test_set_mood_force_switches_label_at_once(start_node):
    h = start_node()
    reply = h.set_mood('excited')
    assert reply.accepted and reply.reason == ''
    # Dwell is 30 s, so only a force can have changed the label this fast.
    assert reply.result.mood == 'excited'
    assert reply.result.arousal == pytest.approx(0.8, abs=1e-3)
    assert reply.result.valence == pytest.approx(0.8, abs=1e-3)
    assert reply.result.curiosity == pytest.approx(0.4, abs=1e-3)   # unspecified axis kept
    assert not reply.result.locked
    assert h.wait_for(lambda m: m.mood == 'excited').arousal == pytest.approx(0.8, abs=1e-2)


def test_set_mood_lock_freezes_the_vector(start_node):
    h = start_node()
    reply = h.set_mood('sleepy', lock=True, source='alice')
    assert reply.accepted and reply.result.locked and reply.result.lock_source == 'alice'
    locked = h.wait_for(lambda m: m.locked)
    h.stimulate('bump', intensity=1.0)
    time.sleep(1.0)
    later = h.next_state()
    assert vector(later) == pytest.approx(vector(locked), abs=1e-6)
    assert later.locked and later.lock_source == 'alice'
    assert later.idle_s < 1.5          # the stimulus still counted as evidence of the world


def test_timed_lock_expires(start_node):
    h = start_node()
    reply = h.set_mood('', lock=True, duration=1.0, source='alice')
    assert reply.accepted and reply.result.locked
    assert reply.result.lock_until.sec > 0
    start = time.monotonic()
    h.wait_for(lambda m: not m.locked, timeout=5.0, what='the lock to expire')
    assert 0.5 < time.monotonic() - start < 3.0
    after = h.next_state()
    assert after.lock_source == '' and after.lock_until.sec == 0
    h.stimulate('bump')
    h.wait_for(lambda m: m.arousal > 0.2, what='a bump to apply after expiry')


def test_other_sources_lock_is_refused_but_clear_works(start_node):
    h = start_node()
    assert h.set_mood('sleepy', lock=True, source='alice').accepted
    refused = h.set_mood('excited', lock=True, source='bob')
    assert not refused.accepted
    assert 'alice' in refused.reason
    assert refused.result.mood == 'sleepy' and refused.result.lock_source == 'alice'
    also = h.set_mood('', lock=True, duration=5.0, source='bob')
    assert not also.accepted and 'alice' in also.reason
    cleared = h.set_mood('', lock=False, source='bob')
    assert cleared.accepted and cleared.reason == ''
    assert not cleared.result.locked and cleared.result.lock_source == ''
    h.wait_for(lambda m: not m.locked)


def test_unknown_label_is_refused(start_node):
    h = start_node()
    reply = h.set_mood('grumpy')
    assert not reply.accepted
    assert 'grumpy' in reply.reason
    assert reply.result.mood == 'calm'


def test_reload_changes_the_next_impulse_and_refuses_a_broken_file(start_node, tmp_path):
    h = start_node(bump=0.2)
    first = h.next_state()
    h.stimulate('bump')
    one = h.wait_for(lambda m: m.arousal > first.arousal + 0.05, what='the first bump')
    assert one.arousal - first.arousal == pytest.approx(0.2, abs=2e-3)

    write_config(tmp_path / 'affect0.yaml', bump=0.4)
    reply = h.reload()
    assert reply.success, reply.message
    assert h.set_mood('', lock=False).accepted
    before = h.next_state()
    h.stimulate('bump')
    two = h.wait_for(lambda m: m.arousal > before.arousal + 0.05, what='the doubled bump')
    assert two.arousal - before.arousal == pytest.approx(0.4, abs=2e-3)

    broken = render(bump=0.9).replace('valence: -0.2}', 'valencex: -0.2}')
    (tmp_path / 'affect0.yaml').write_text(broken)
    reply = h.reload()
    assert not reply.success
    assert 'valencex' in reply.message, reply.message
    before = h.next_state()
    h.stimulate('bump')
    three = h.wait_for(lambda m: m.arousal > before.arousal + 0.05, what='a bump on old config')
    assert three.arousal - before.arousal == pytest.approx(0.4, abs=2e-3)

    (tmp_path / 'affect0.yaml').write_text('{{ not yaml')
    reply = h.reload()
    assert not reply.success and 'YAML' in reply.message


def test_invalid_config_at_startup_exits_non_zero(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text(render().replace('excited', 'excitedx')
                   .replace('arousal: 0.8,', 'arousalx: 0.8,'))
    proc = subprocess.run(
        ['ros2', 'run', 'r2_affect', 'r2_affect', '--ros-args', '-p', 'config:=%s' % bad,
         '-r', '__node:=r2_affect_bad_%d' % os.getpid()],
        capture_output=True, text=True, timeout=60)
    assert proc.returncode != 0
    assert 'arousalx' in proc.stdout + proc.stderr


def test_bool_rate_parameter_is_refused_at_startup(tmp_path):
    proc = subprocess.run(
        ['ros2', 'run', 'r2_affect', 'r2_affect', '--ros-args',
         '-p', 'config:=%s' % write_config(tmp_path / 'ok.yaml'), '-p', 'rate_hz:=true',
         '-r', '__node:=r2_affect_boolrate_%d' % os.getpid()],
        capture_output=True, text=True, timeout=60)
    assert proc.returncode == 1
    assert 'rate_hz must be a number' in proc.stdout + proc.stderr


def test_sim_time_drives_the_node(start_node):
    tau = 20.0
    h = start_node(tau=tau, sim_t0=100.0, extra_args=('-p', 'use_sim_time:=true'))
    t0 = h.sim_t
    publish_clock = h.publish_clock
    reply = h.set_mood('excited')
    assert reply.accepted
    forced_at = reply.result.header.stamp.sec + reply.result.header.stamp.nanosec * 1e-9
    assert forced_at == pytest.approx(t0, abs=0.2)
    start_vector = vector(reply.result)

    # Clock stopped: 2 s of wall time must change nothing, but the heartbeat goes on.
    time.sleep(0.3)
    with h.lock:
        mark = len(h.states)
    time.sleep(2.0)
    with h.lock:
        quiet = list(h.states[mark:])
    assert len(quiet) >= 7, 'expected a steady 5 Hz heartbeat, got %d messages' % len(quiet)
    for msg in quiet:
        assert vector(msg) == pytest.approx(start_vector, abs=1e-6)
        assert msg.header.stamp == quiet[0].header.stamp

    # Advance 10 s in 100 ms steps (50 ms apart in wall time: the node ticks every 100 ms).
    with h.lock:
        mark = len(h.states)
    for i in range(1, 101):
        publish_clock(t0 + 0.1 * i)
        time.sleep(0.05)
    with h.lock:
        resumed = next(m for m in h.states[mark:]
                       if m.header.stamp != quiet[0].header.stamp)
    # The first message after the resume has relaxed by one tick at most (about 0.2 s).
    assert resumed.arousal == pytest.approx(start_vector[0], abs=0.02)
    end = time.monotonic() + 15.0
    while True:
        with h.lock:
            final = h.states[-1]
        if final.header.stamp.sec + final.header.stamp.nanosec * 1e-9 >= t0 + 9.85:
            break
        assert time.monotonic() < end, 'node did not reach the end of the sim interval'
        time.sleep(0.05)
    stamp = final.header.stamp.sec + final.header.stamp.nanosec * 1e-9
    elapsed = stamp - forced_at

    # Compare against a pure model driven with the same times, and the closed form.
    model = AffectModel(load_config(h.config_path))
    model.set_mood('excited', False, 0.0, 'test', 0.0)
    now = 0.0
    model.tick(now)
    while now < elapsed - 1e-9:
        now = min(now + 0.1, elapsed)
        model.tick(now)
    assert final.arousal == pytest.approx(model.state.arousal, abs=0.01)
    assert final.arousal == pytest.approx(0.8 * math.exp(-elapsed / tau), abs=0.02)
    assert final.arousal < 0.8 - 0.1          # relaxed for real, not just 2 s of wall time


def test_only_expected_publishers(start_node):
    h = start_node()
    deadline = time.monotonic() + 10.0
    topics = set()
    while time.monotonic() < deadline:
        pubs = h.node.get_publisher_names_and_types_by_node(h.node_name, '/')
        topics = {name for name, _ in pubs}
        if h.topics['/mood/state'] in topics:
            break
        time.sleep(0.1)
    assert topics == {h.topics['/mood/state'], '/rosout', '/parameter_events'}


def test_huge_duration_is_refused_and_the_node_survives(start_node):
    h = start_node()
    for duration in (1e10, 1e20, 86401.0):
        reply = h.set_mood('', lock=True, duration=duration)
        assert not reply.accepted and reply.reason == 'duration_s above one day'
        assert not reply.result.locked
    reply = h.set_mood('', lock=True, duration=86400.0)
    assert reply.accepted and reply.result.lock_until.sec > 0
    assert h.proc.poll() is None


def test_non_utf8_config_is_refused_on_reload(start_node, tmp_path):
    h = start_node(bump=0.2)
    (tmp_path / 'affect0.yaml').write_bytes(render().encode() + b'# stray \xb0 byte\n')
    reply = h.reload()
    assert not reply.success and 'UTF-8' in reply.message
    assert h.proc.poll() is None
    before = h.next_state()
    h.stimulate('bump')
    after = h.wait_for(lambda m: m.arousal > before.arousal + 0.05, what='a bump on old config')
    assert after.arousal - before.arousal == pytest.approx(0.2, abs=2e-3)


def test_int_rate_parameter_is_accepted(start_node):
    h = start_node(extra_args=('-p', 'rate_hz:=10'))
    with h.lock:
        start = len(h.states)
    time.sleep(1.5)
    with h.lock:
        assert len(h.states) - start >= 10


def test_startup_parameters_cannot_be_changed_later(start_node):
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.msg import ParameterValue
    from rcl_interfaces.srv import SetParameters
    h = start_node()
    client = h.node.create_client(SetParameters, '/%s/set_parameters' % h.node_name)
    assert client.wait_for_service(timeout_sec=10.0)
    attempts = {
        'rate_hz': ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=20.0),
        'min_confidence': ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                         double_value=0.5),
        'max_dt_s': ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0),
        'config': ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value='/tmp/other.yaml'),
        # Refused before TimeSource sees it: the node stays on the wall clock.
        'use_sim_time': ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True),
    }
    for name, value in attempts.items():
        request = SetParameters.Request()
        request.parameters = [ParameterMsg(name=name, value=value)]
        result = h.call(client, request).results[0]
        assert not result.successful, name
        assert 'restart the node' in result.reason, name
    # Still ticking on the wall clock (a switch to sim time with no /clock would stamp 0).
    assert h.next_state().header.stamp.sec > 0


def test_unknown_and_reserved_kinds_warn_once(start_node):
    h = start_node()
    h.next_state()
    for _ in range(3):
        h.stimulate('nonsense')
        h.stimulate('idle')
    h.stimulate('bump')               # a marker: everything before it has been handled
    h.wait_log('stimulus bump from')
    out = ''.join(h.output)
    assert out.count("unknown stimulus kind 'nonsense'") == 1
    assert out.count("stimulus kind 'idle' is reserved") == 1


def test_applied_stimulus_and_transition_are_logged_at_info(start_node):
    h = start_node(dwell=0.3)
    h.next_state()
    h.stimulate('cheer', source='unit')
    h.wait_for(lambda m: m.mood == 'excited', timeout=5.0, what='the label to change')
    out = h.wait_log('mood: calm -> excited')
    stimulus_lines = [line for line in out.splitlines() if 'stimulus cheer from unit' in line]
    assert len(stimulus_lines) == 1 and '[INFO]' in stimulus_lines[0]
    assert 'aro+0.80' in stimulus_lines[0] and 'val+0.60' in stimulus_lines[0]
    assert out.count('mood: calm -> excited') == 1


def test_refractory_drop_is_logged_once_per_window(start_node):
    h = start_node()
    h.next_state()
    h.stimulate('slow')
    for _ in range(3):
        h.stimulate('slow')
    h.stimulate('bump')
    out = h.wait_log('stimulus bump from')
    assert out.count('dropped: refractory') == 1
    assert [line for line in out.splitlines() if 'dropped: refractory' in line][0].count(
        '[INFO]') == 1


def test_first_refractory_drop_after_each_acceptance_is_logged_at_info(start_node):
    # quick has a 2 s window. Accept at 0, drop at ~1.6 (INFO), accept again at ~2.4 (a new
    # window), drop at once: INFO again, although it is within 2 s of the last INFO.
    h = start_node()
    h.next_state()
    h.stimulate('quick', source='unit')
    time.sleep(1.6)
    h.stimulate('quick', source='unit')
    h.stimulate('quick', source='unit')       # same window: DEBUG
    time.sleep(0.8)
    h.stimulate('quick', source='unit')
    h.stimulate('quick', source='unit')
    h.stimulate('bump')
    out = h.wait_log('stimulus bump from')
    applied = [line for line in out.splitlines() if 'stimulus quick from unit i=' in line]
    dropped = [line for line in out.splitlines() if 'quick from unit dropped: refractory' in line]
    assert len(applied) == 2, out
    assert len(dropped) == 2 and all('[INFO]' in line for line in dropped), out


def test_focus_updates_while_locked(start_node):
    h = start_node()
    assert h.set_mood('sleepy', lock=True).accepted
    locked = h.wait_for(lambda m: m.locked)
    h.stimulate('bump', subject='dave', bearing=0.8)
    seen = h.wait_for(lambda m: m.focus_subject == 'dave', what='the focus')
    assert seen.focus_bearing_rad == pytest.approx(0.8, abs=1e-5)
    assert vector(seen) == pytest.approx(vector(locked), abs=1e-6)
    assert seen.locked


def test_low_confidence_stimulus_is_dropped_and_idle_not_reset(start_node):
    h = start_node(extra_args=('-p', 'min_confidence:=0.5'))
    time.sleep(1.0)
    before = h.next_state()
    assert before.idle_s > 0.8
    h.stimulate('bump', confidence=0.1)
    time.sleep(0.5)
    after = h.next_state()
    assert after.idle_s > before.idle_s
    assert vector(after) == pytest.approx(vector(before), abs=1e-4)
    h.stimulate('bump', confidence=0.9)
    h.wait_for(lambda m: m.idle_s < 0.5, what='idle_s to reset on an accepted stimulus')


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_signals_stop_the_node_quietly(tmp_path, sig):
    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix('r2_affect'), 'lib', 'r2_affect', 'r2_affect')
    tag = 'sig%d_%d' % (os.getpid(), sig)
    proc = subprocess.Popen(
        [exe, '--ros-args', '-r', '__node:=r2_affect_%s' % tag,
         '-r', '/mood/state:=/%s/state' % tag, '-r', '/stimuli:=/%s/stimuli' % tag,
         '-r', '/mood/set:=/%s/set' % tag, '-r', '/mood/reload_config:=/%s/reload' % tag,
         '-p', 'config:=%s' % write_config(tmp_path / 'sig.yaml')],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(3.0)
        assert proc.poll() is None
        proc.send_signal(sig)
        out, _ = proc.communicate(timeout=10)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert proc.returncode == 0, out
    assert 'Traceback' not in out
