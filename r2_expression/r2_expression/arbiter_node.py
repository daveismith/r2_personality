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
The arbiter nodes: ROS 2 around :class:`r2_expression.core.Arbiter`.

One node class, two kinds. ``led_arbiter`` serves ``fld``, ``tfld``, ``bfld``, ``rld`` and
``psi`` against ``<device>/leds/*``; ``magic_arbiter`` serves ``magic`` against
``<device>/magic/*`` (plan 5.3). Each is the only ROS client of its device services.

Threads. The executor thread (``main``) runs every subscription, the claim service and the
10 Hz tick; each takes ``_lock`` around the core and never waits on the device. One worker
thread makes the device calls from a queue, in order, one request in flight at a time with a
5 s timeout; its replies go back into the core under the same lock. All time is the node
clock (``use_sim_time`` aware); the tick timer runs on the steady clock.
"""

from dataclasses import dataclass
import os
import queue
import sys
import threading
import time
from types import ModuleType
from typing import Callable
from typing import Type

from ament_index_python.packages import get_package_share_directory
from astromech_msgs.msg import ChannelStatus
from astromech_msgs.msg import ExpressionIntent
from astromech_msgs.msg import LedStateArray
from astromech_msgs.msg import MagicPanelState
from astromech_msgs.msg import MoodState
from astromech_msgs.srv import ClaimChannel
from r2_expression import led_renderer
from r2_expression import magic_renderer
from r2_expression.config import ExpressionConfigError
from r2_expression.config import load_config
from r2_expression.core import Arbiter
from r2_expression.core import CALL_MAGIC_STOP
from r2_expression.core import CALL_STOP
from r2_expression.core import held_by
from r2_expression.core import LED_CHANNELS
from r2_expression.core import MAGIC_CHANNELS
from r2_expression.core import PRIORITY_BEHAVIOR
from r2_expression.core import STATE_FOREIGN
from r2_expression.core import STATE_IDLE
from r2_expression.core import STATE_NAMES
from r2_expression.palette import led_catalogue
from r2_expression.palette import magic_catalogue
from r2_expression.render_common import RenderRefusal
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

# Read once at startup; a later `ros2 param set` of any of these is refused.
STARTUP_PARAMETERS = ('config', 'device', 'use_sim_time')

REQUEST_TIMEOUT_S = 5.0      # one device request, plan 2.1
LIST_RETRY_S = 5.0           # between attempts at the device's `list`
TICK_S = 0.1                 # the core's timers, 10 Hz
POLL_S = 0.1                 # how often a waiting worker looks at the stop flag
# Where the nodes meet. Both offer a claim service; rmw_zenoh gives a request to either
# server of a shared name, so each kind has its own (README, "The claim service").
INTENT_TOPIC = '/expression/intent'
MOOD_TOPIC = '/mood/state'
STATUS_TOPIC = '/expression/status'
CLAIM_SERVICES = {'led': '/expression/claim/led', 'magic': '/expression/claim/magic'}
# tfld and bfld are reported only while they are doing something.
QUIET_WHEN_IDLE = ('tfld', 'bfld')


@dataclass(frozen=True)
class Kind:
    """What differs between the two nodes."""

    channels: tuple
    renderer: ModuleType
    state_topic: str                 # under the device node
    state_type: Type
    on_state: str                    # the core method that takes a snapshot
    catalogue: Callable


KINDS = {
    'led': Kind(LED_CHANNELS, led_renderer, 'leds/states', LedStateArray, 'led_state',
                led_catalogue),
    'magic': Kind(MAGIC_CHANNELS, magic_renderer, 'magic/states', MagicPanelState,
                  'magic_state', magic_catalogue),
}


def _reliable(depth):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)


def default_config_path():
    """Return the installed ``expression.yaml``."""
    return os.path.join(get_package_share_directory('r2_expression'), 'config',
                        'expression.yaml')


def status_to_msg(status, stamp):
    """Map a core :class:`~r2_expression.core.Status` to a ``ChannelStatus``."""
    msg = ChannelStatus()
    msg.header.stamp = stamp
    msg.channel = status.channel
    msg.state = status.state
    msg.owner = status.owner
    msg.priority = status.priority
    msg.lease_remaining_s = float(status.lease_remaining_s)
    msg.active_source = status.active_source
    msg.active_action = status.active_action
    msg.active_intent_id = status.active_intent_id
    msg.active_remaining_ms = status.active_remaining_ms
    msg.mood = status.mood
    msg.dropped = status.dropped
    msg.refused = status.refused
    return msg


def _change_key(status):
    """Return what makes a status a change worth publishing at once (not the countdowns)."""
    return (status.state, status.owner, status.priority, status.active_source,
            status.active_action, status.active_intent_id, status.mood, status.dropped,
            status.refused)


class ArbiterNode(Node):
    """The LED or the Magic Panel arbiter, by ``kind``."""

    def __init__(self, kind):
        self._kind = KINDS[kind]
        self._kind_name = kind
        super().__init__('%s_arbiter' % kind)
        self.declare_parameter('config', default_config_path())
        self.declare_parameter('device', '/r2_dome')
        device = str(self.get_parameter('device').value).rstrip('/')
        if not device.startswith('/') or len(device) < 2:
            raise ValueError('device must be an absolute node name like /r2_dome, got %r'
                             % device)
        self._device = device
        self._config_path = str(self.get_parameter('config').value)
        config = load_config(self._config_path)     # ExpressionConfigError: main() exits
        self._core = Arbiter(config, self._kind.channels)
        self._lock = threading.Lock()
        self._queue = queue.Queue()        # (time.monotonic() when queued, Action | None)
        self._max_age_s = config.defaults.device_stale_s
        self._stale_burst = 0              # stale actions dropped since the queue last caught up
        self._stop = threading.Event()
        self._catalogue_ready = False
        self._panel = None                 # magic: (interrogations, link) of the last state
        self._last_key = {}
        self._held_logged = set()
        self._last_mood = ''
        self._last_status_at = 0.0
        self._status_period = 1.0 / config.defaults.status_hz

        self._device_clients = {}
        renderer = self._kind.renderer
        for call, (suffix, srv) in renderer.SERVICES.items():
            self._device_clients[call] = self.create_client(srv, '%s/%s' % (device, suffix))
        self._list_client = self.create_client(
            renderer.LIST[1], '%s/%s' % (device, renderer.LIST[0]))

        self._status_pub = self.create_publisher(ChannelStatus, STATUS_TOPIC, _reliable(10))
        self.create_subscription(ExpressionIntent, INTENT_TOPIC, self._on_intent, _reliable(50))
        self.create_subscription(MoodState, MOOD_TOPIC, self._on_mood, _reliable(10))
        self.create_subscription(self._kind.state_type,
                                 '%s/%s' % (device, self._kind.state_topic),
                                 self._on_state, _reliable(10))
        self.create_service(ClaimChannel, CLAIM_SERVICES[kind], self._on_claim)
        # Registered after TimeSource's own callback, so it runs first and a refused
        # use_sim_time never reaches TimeSource.
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(TICK_S, self._on_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self._worker = threading.Thread(target=self._work, name='%s_worker' % kind, daemon=True)
        self._worker.start()
        self.get_logger().info(
            '%s_arbiter up: config %s, device %s, channels %s' % (
                kind, self._config_path, device, ' '.join(self._core.channels)))

    # --- helpers ---------------------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_parameters(self, params):
        for param in params:
            if param.name in STARTUP_PARAMETERS:
                return SetParametersResult(
                    successful=False, reason='read at startup; restart the node')
        return SetParametersResult(successful=True)

    def _settle(self, now, actions=(), force=False):
        """Queue ``actions``, log core warnings, publish status changes. Hold ``_lock``."""
        for action in actions:
            self._queue.put((time.monotonic(), action))
        log = self.get_logger()
        for line in self._core.take_warnings():
            log.warning(line)
        periodic = force or time.monotonic() - self._last_status_at >= self._status_period
        if periodic:
            self._last_status_at = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        for status in self._core.statuses(now):
            key = _change_key(status)
            previous = self._last_key.get(status.channel)
            changed = key != previous
            self._last_key[status.channel] = key
            if changed:
                self._log_change(status, previous)
            quiet = status.channel in QUIET_WHEN_IDLE and status.state == STATE_IDLE
            if changed or (periodic and not quiet):
                self._status_pub.publish(status_to_msg(status, stamp))

    def _log_change(self, status, previous):
        if previous is not None and previous[0] == status.state:
            return
        if status.state != STATE_FOREIGN:     # no longer foreign: a new hold logs again
            self._held_logged -= {key for key in self._held_logged if key[0] == status.channel}
        self.get_logger().info('%s: %s%s' % (
            status.channel, STATE_NAMES[status.state],
            ' (%s)' % status.owner if status.owner else ''))

    # --- subscriptions and the claim service (executor thread) -----------------------------

    def _on_intent(self, msg):
        if msg.channel not in self._core.channels:
            return
        with self._lock:
            now = self._now()
            result = self._core.intent(msg, now)
            self._settle(now, result.actions)
        log = self.get_logger()
        if result.accepted:
            log.info('intent %s accepted: %s %s %s from %s' % (
                result.intent_id, msg.channel, msg.action, result.reason, msg.source))
        else:
            log.debug('intent %s dropped: %s' % (result.intent_id, result.reason))

    def _on_mood(self, msg):
        with self._lock:
            now = self._now()
            actions = self._core.mood(msg.mood, msg.mood_strength, now)
            self._settle(now, actions)
        if msg.mood and msg.mood != self._last_mood:
            self.get_logger().info('mood: %s -> %s' % (self._last_mood or '-', msg.mood))
            self._last_mood = msg.mood

    def _on_state(self, msg):
        if not self._catalogue_ready:
            return                     # the catalogue first, then the first state (plan 5.3)
        if self._kind_name == 'magic':
            self._watch_panel(msg)
        with self._lock:
            now = self._now()
            actions = getattr(self._core, self._kind.on_state)(msg, now)
            self._settle(now, actions)

    def _watch_panel(self, msg):
        """Ask for the catalogue again when the panel re-reads itself or comes back."""
        seen = (msg.interrogations, msg.link)
        previous, self._panel = self._panel, seen
        if previous is not None and (seen[0] != previous[0] or
                                     (seen[1] == 1 and previous[1] != 1)):
            self.get_logger().info('panel changed (interrogations %d, link %d): listing again'
                                   % seen)
            self._queue.put((time.monotonic(), None))

    def _on_claim(self, request, response):
        with self._lock:
            now = self._now()
            result = self._core.claim(request.channel, request.owner, request.priority,
                                      request.lease_s, now)
            self._settle(now, result.actions, force=True)
        response.accepted = result.accepted
        response.reason = result.reason
        if result.status is not None:
            response.status = status_to_msg(result.status, self.get_clock().now().to_msg())
        else:
            response.status.channel = request.channel
        if request.channel in self._core.channels:
            self.get_logger().info('claim %s by %s at %d for %.1f s: %s' % (
                request.channel, request.owner, request.priority, request.lease_s,
                'accepted' if result.accepted else 'refused, %s' % result.reason))
        return response

    def _on_tick(self):
        with self._lock:
            now = self._now()
            self._settle(now, self._core.tick(now))

    # --- the worker: the only thread that talks to the device ------------------------------

    def _call(self, client, request):
        """Call a service and wait up to the timeout; return the response or None."""
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + REQUEST_TIMEOUT_S
        while not done.wait(POLL_S):
            if self._stop.is_set() or time.monotonic() >= deadline:
                client.remove_pending_request(future)
                return None
        return future.result()

    def _work(self):
        self._first_catalogue()
        while not self._stop.is_set():
            try:
                queued_at, item = self._queue.get(timeout=POLL_S)
            except queue.Empty:
                self._end_stale_burst()
                continue
            try:
                if item is None:
                    self._end_stale_burst()
                    self._load_catalogue()
                elif self._is_stale(queued_at, item):
                    self._drop_stale(item)
                else:
                    self._end_stale_burst()
                    self._send(item)
            except Exception as exc:     # the worker must outlive any one bad request
                self.get_logger().error('device worker: %r' % (exc,))
                if item is not None:
                    self._reply(item, False, 'internal: %s' % exc, quiet=True)

    def _is_stale(self, queued_at, action):
        """Return True for a start or palette call that waited over ``device_stale_s``."""
        return (action.priority <= PRIORITY_BEHAVIOR and
                action.call not in (CALL_STOP, CALL_MAGIC_STOP) and
                time.monotonic() - queued_at > self._max_age_s)

    def _drop_stale(self, action):
        """Not played into a recovered device: tell the core it failed, count it, log the burst."""
        if self._stale_burst == 0:
            self.get_logger().warning(
                'device worker is behind: dropping queued personality actions older than '
                '%.0f s (manual, safety and stops are kept)' % self._max_age_s)
        self._stale_burst += 1
        self._reply(action, False, 'stale', quiet=True)

    def _end_stale_burst(self):
        if self._stale_burst:
            self.get_logger().info('dropped %d stale device actions' % self._stale_burst)
            self._stale_burst = 0

    def _load_catalogue(self):
        """Call `list` once and give the core the catalogue; return True on success."""
        response = self._call(self._list_client, self._kind.renderer.LIST[1].Request())
        if response is None:
            self.get_logger().debug('list: no answer from %s' % self._device)
            return False
        with self._lock:
            now = self._now()
            actions = self._core.set_catalogue(self._kind.catalogue(response), now)
            self._catalogue_ready = True
            self._settle(now, actions)
        return True

    def _first_catalogue(self):
        """
        Call `list` once the device is on the graph, then every LIST_RETRY_S while it fails.

        Waiting for the service to appear costs nothing on the device, so it is polled; only an
        actual call that gets no answer is followed by the pause. OFFLINE meanwhile.
        """
        log = self.get_logger()
        failures = 0
        while not self._stop.is_set():
            ready = self._list_client.service_is_ready()
            if ready and self._load_catalogue():
                log.info('device %s answered list; catalogue loaded' % self._device)
                return
            if not ready:
                if failures == 0:
                    log.warning('device %s is not on the graph; waiting for its list service'
                                % self._device)
                    failures = 1
                self._drain_offline(POLL_S)
                continue
            failures += 1
            log.warning('device %s does not answer list; trying again in %.0f s'
                        % (self._device, LIST_RETRY_S))
            self._drain_offline(LIST_RETRY_S)

    def _drain_offline(self, seconds):
        """Wait ``seconds``, refusing queued device calls: there is nobody to send them to."""
        deadline = time.monotonic() + seconds
        while not self._stop.is_set() and time.monotonic() < deadline:
            try:
                _, action = self._queue.get(timeout=POLL_S)
            except queue.Empty:
                continue
            if action is not None:
                self._reply(action, False, 'device offline')

    def _send(self, action):
        log = self.get_logger()
        try:
            request = self._kind.renderer.render(action)
        except RenderRefusal as refusal:
            log.warning('%s: %s refused locally: %s'
                        % (action.channel, action.call, refusal.reason))
            self._reply(action, False, refusal.reason, quiet=True)
            return
        except ValueError as exc:                   # a bug in the core or the renderer
            log.error('%s: %s not sent: %s' % (action.channel, action.call, exc))
            self._reply(action, False, 'internal: %s' % exc, quiet=True)
            return
        client = self._device_clients[action.call]
        if not client.service_is_ready():
            log.debug('%s: %s on %s: device not on the graph' % (
                action.channel, action.call, action.target))
            self._reply(action, False, 'device offline', quiet=True)
            return
        log.debug('%s: %s on %s (%s)'
                  % (action.channel, action.call, action.target, action.why))
        response = self._call(client, request)
        if response is None:
            if not self._stop.is_set():
                log.warning("%s: %s on %s: no answer in %.0f s (a string over the device's "
                            'buffer is never answered)'
                            % (action.channel, action.call, action.target, REQUEST_TIMEOUT_S))
                self._reply(action, False, 'timeout', quiet=True)
            return
        reason = response.reason
        if response.accepted and getattr(response, 'confirmed', True) is False:
            reason = reason or 'not confirmed'
        self._reply(action, response.accepted, reason)

    def _reply(self, action, accepted, reason, quiet=False):
        """Report the device's answer to the core and log what it says about it."""
        with self._lock:
            now = self._now()
            line = self._core.device_reply(action, accepted, reason, now)
            holder = held_by(reason) if not accepted else ''
            first = holder and (action.channel, holder) not in self._held_logged
            if first:
                self._held_logged.add((action.channel, holder))
            self._settle(now)
        if not line or quiet:
            return
        log = self.get_logger()
        if accepted:
            log.info(line)
        elif holder:
            (log.info if first else log.debug)(line)
        else:
            log.warning(line)

    def stop_worker(self):
        """Ask the worker to finish (it notices within POLL_S) and wait briefly."""
        self._stop.set()
        self._worker.join(timeout=2.0)


def _try_shutdown():
    """Shut rclpy down unless a signal handler already did."""
    try:
        rclpy.try_shutdown()
    except (RCLError, RuntimeError):     # a session that is slow to close is not a failure
        pass


def main(kind, args=None):
    """Run the ``kind`` arbiter; exit 1 on an invalid config or device at startup."""
    rclpy.init(args=args)
    try:
        try:
            node = ArbiterNode(kind)
        except (ExpressionConfigError, ValueError, TypeError, OSError, KeyError) as exc:
            rclpy.logging.get_logger('%s_arbiter' % kind).error('cannot start: %s' % exc)
            return 1
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            # spin_once with a timeout, so SIGINT/SIGTERM are noticed on an idle node.
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.2)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RCLError:
            if rclpy.ok():        # not just a signal shutting the context mid-callback
                raise
        finally:
            node.stop_worker()
            executor.shutdown(timeout_sec=0)
            node.destroy_node()
    finally:
        _try_shutdown()
    return 0


def main_led(args=None):
    """Entry point of ``led_arbiter``."""
    return main('led', args)


def main_magic(args=None):
    """Entry point of ``magic_arbiter``."""
    return main('magic', args)


if __name__ == '__main__':
    sys.exit(main_led())
