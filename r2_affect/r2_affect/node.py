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
The ``r2_affect`` node: ROS 2 around :class:`r2_affect.model.AffectModel`.

``/stimuli`` in, ``/mood/state`` out at ``rate_hz``, ``/mood/set`` and
``/mood/reload_config`` as services. All time is the node clock
(``use_sim_time`` aware); ``header.stamp`` of a stimulus is never used.
Everything runs in one callback group on a single-threaded executor, so the
model needs no locking.
"""

import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from astromech_msgs.msg import MoodState
from astromech_msgs.msg import Stimulus
from astromech_msgs.srv import SetMood
from builtin_interfaces.msg import Time
from r2_affect.config import AffectConfigError
from r2_affect.config import AXES
from r2_affect.config import IDLE_KIND
from r2_affect.config import load_config
from r2_affect.model import AffectModel
from r2_affect.model import DROP_LOW_CONFIDENCE
from r2_affect.model import DROP_REFRACTORY
from r2_affect.model import DROP_RESERVED_KIND
from r2_affect.model import DROP_UNKNOWN_KIND
from r2_affect.model import NOT_APPLIED_LOCKED
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_srvs.srv import Trigger


def default_config_path():
    """Return the installed ``affect.yaml``."""
    return os.path.join(get_package_share_directory('r2_affect'), 'config', 'affect.yaml')


def seconds_to_time(seconds):
    """Convert float seconds to ``builtin_interfaces/Time``."""
    sec = int(math.floor(seconds))
    return Time(sec=sec, nanosec=min(int(round((seconds - sec) * 1e9)), 999999999))


def state_to_msg(state, stamp):
    """Map an :class:`~r2_affect.model.AffectState` to a ``MoodState``."""
    msg = MoodState()
    msg.header.stamp = stamp
    msg.arousal = state.arousal
    msg.valence = state.valence
    msg.curiosity = state.curiosity
    msg.confidence = state.confidence
    msg.mood = state.mood
    msg.mood_strength = state.mood_strength
    msg.locked = state.locked
    msg.lock_source = state.lock_source
    if state.lock_until is not None:
        msg.lock_until = seconds_to_time(state.lock_until)
    msg.focus_subject = state.focus_subject
    msg.focus_bearing_rad = state.focus_bearing
    msg.idle_s = state.idle_s
    return msg


def _vector_text(state):
    return ' '.join('%s=%.2f' % (axis[:3], getattr(state, axis)) for axis in AXES)


class AffectNode(Node):
    """Run the affect model on the node clock."""

    def __init__(self):
        super().__init__('r2_affect')
        self.declare_parameter('config', default_config_path())
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('min_confidence', 0.0)
        self.declare_parameter('max_dt_s', 0.5)

        rate_hz = float(self.get_parameter('rate_hz').value)
        if not (math.isfinite(rate_hz) and rate_hz > 0.0):
            raise ValueError('rate_hz must be > 0, got %r' % rate_hz)
        self._config_path = str(self.get_parameter('config').value)
        config = load_config(self._config_path)           # AffectConfigError: main() exits
        self._model = AffectModel(
            config,
            min_confidence=float(self.get_parameter('min_confidence').value),
            max_dt_s=float(self.get_parameter('max_dt_s').value))

        self._warned_kinds = set()
        self._warned_reserved = False
        self._warned_clamp = False

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(MoodState, '/mood/state', reliable)
        self.create_subscription(
            Stimulus, '/stimuli', self._on_stimulus,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.create_service(SetMood, '/mood/set', self._on_set_mood)
        self.create_service(Trigger, '/mood/reload_config', self._on_reload)
        self.create_timer(1.0 / rate_hz, self._on_tick)

        state = self._model.state
        self.get_logger().info(
            'r2_affect up: config %s, %.1f Hz, mood %s (%s)' % (
                self._config_path, rate_hz, state.mood, _vector_text(state)))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tick(self):
        result = self._model.tick(self._now())
        log = self.get_logger()
        if result.dt_clamped and not self._warned_clamp:
            self._warned_clamp = True
            log.warning('time step %.3f s clamped to %.3f s (sim paused or clock jump); '
                        'not repeated' % (result.raw_dt, result.dt))
        if result.transition is not None:
            t = result.transition
            log.info('mood: %s -> %s (d=%.2f)' % (t.old, t.new, t.distance))
        if result.lock_expired:
            log.info('lock expired')
        self._pub.publish(state_to_msg(result.state, self.get_clock().now().to_msg()))

    def _on_stimulus(self, msg):
        result = self._model.stimulus(
            msg.kind, msg.intensity, msg.confidence, msg.subject, msg.bearing_rad,
            msg.ttl_s, self._now())
        log = self.get_logger()
        if result.lock_expired:
            log.info('lock expired')
        reason = result.reason
        if reason == DROP_LOW_CONFIDENCE:
            log.debug('stimulus %s from %s dropped: confidence %.2f below minimum %.2f' % (
                msg.kind, msg.source, msg.confidence, self._model.min_confidence))
        elif reason == DROP_REFRACTORY:
            log.debug('stimulus %s from %s dropped: refractory' % (msg.kind, msg.source))
        elif reason == DROP_UNKNOWN_KIND:
            if msg.kind not in self._warned_kinds:
                self._warned_kinds.add(msg.kind)
                log.warning('unknown stimulus kind %r from %s ignored; not repeated for this '
                            'kind' % (msg.kind, msg.source))
        elif reason == DROP_RESERVED_KIND:
            if not self._warned_reserved:
                self._warned_reserved = True
                log.warning("stimulus kind '%s' is reserved for the idle drift and is ignored; "
                            'not repeated' % IDLE_KIND)
        else:
            deltas = ' '.join('%s%+.2f' % (axis[:3], result.delta[axis])
                              for axis in AXES if result.delta[axis] != 0.0)
            note = ' (locked, not applied)' if reason == NOT_APPLIED_LOCKED else ''
            log.info('stimulus %s from %s i=%.2f: %s -> %s%s' % (
                msg.kind, msg.source or '-', result.intensity, deltas or 'no change',
                _vector_text(result.state), note))

    def _on_set_mood(self, request, response):
        before = self._model.state
        now = self._now()
        accepted, reason = self._model.set_mood(
            request.mood, request.lock, request.duration_s, request.source, now)
        after = self._model.state
        log = self.get_logger()
        source = request.source or 'unknown'
        if before.locked and not after.locked and not (not request.mood and not request.lock):
            log.info('lock held by %s expired' % before.lock_source)
        if not accepted:
            log.info('mood set refused for %s: %s' % (source, reason))
        elif request.mood:
            log.info('mood %s %s by %s%s' % (
                'forced' if not request.lock else 'forced and locked', request.mood, source,
                ' for %.1f s' % request.duration_s
                if request.lock and request.duration_s > 0.0 else ''))
        elif request.lock:
            log.info('mood locked in place by %s%s' % (
                source, ' for %.1f s' % request.duration_s if request.duration_s > 0.0 else ''))
        else:
            log.info('lock cleared by %s' % source if before.locked and not after.locked
                     else 'clear requested by %s (nothing locked)' % source)
        response.accepted = accepted
        response.reason = reason
        response.result = state_to_msg(after, self.get_clock().now().to_msg())
        return response

    def _on_reload(self, request, response):
        path = str(self.get_parameter('config').value)
        try:
            config = load_config(path)
        except AffectConfigError as exc:
            self.get_logger().error('reload refused, keeping the old config: %s' % exc)
            response.success = False
            response.message = 'reload refused, old config kept: %s' % exc
            return response
        self._model.reload(config)
        self._config_path = path
        self.get_logger().info('config reloaded from %s' % path)
        response.success = True
        response.message = 'config reloaded from %s' % path
        return response


def main(args=None):
    """Run the node; exit 1 on an invalid config at startup."""
    rclpy.init(args=args)
    try:
        try:
            node = AffectNode()
        except (AffectConfigError, ValueError) as exc:
            rclpy.logging.get_logger('r2_affect').error('cannot start: %s' % exc)
            return 1
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
