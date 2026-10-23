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

Three kinds on one base, :class:`ArbiterNode` (the core, intents, mood, the claim service,
status, the 4 Hz tick, the namesake refusal). :class:`DeviceArbiterNode`: ``led_arbiter``
serves ``fld``, ``tfld``, ``bfld``, ``rld`` and ``psi`` against ``<device>/leds/*``;
``magic_arbiter`` serves ``magic`` against ``<device>/magic/*`` (plan 5.3). Each is the only
ROS client of its device services. :class:`DomeArbiterNode`: ``dome_arbiter`` serves
``dome`` (phase 2 plan 5.4); its "device" is ``dome_twist_converter``, which it reaches by
publishing on ``command_topic`` alone (no services, no catalogue, no worker).

Threads. The executor thread (``main``) runs every subscription, the claim service and the
timers; each takes ``_lock`` around the core and never waits on a device. LED and Magic
Panel: one worker thread makes the device calls from a queue, in order, one request in
flight at a time with a 5 s timeout; its replies go back into the core under the same lock.
An idle worker sleeps on the queue until an action arrives or the queue is closed (no
polling: a wake-up costs about 0.1 % of a Pi 5 core per Hz). Dome: every velocity the core
returns is published at once, from the callback that produced it (a stop leaves the node
in the same callback); while :attr:`~r2_expression.core.Arbiter.moving`, a 20 Hz timer,
created by the first move action and destroyed when the move ends, ticks the core in place
of the 4 Hz tick (which skips its turn meanwhile). All time is the node clock
(``use_sim_time`` aware); the timers run on the steady clock.
"""

from collections import deque
from dataclasses import dataclass
import functools
import math
import os
import queue
import signal
import sys
import threading
import time
from types import ModuleType
from typing import Callable
from typing import Type

from ament_index_python.packages import get_package_share_directory
from astromech_msgs.msg import ChannelStatus
from astromech_msgs.msg import DomeCommandStatus
from astromech_msgs.msg import ExpressionIntent
from astromech_msgs.msg import LedStateArray
from astromech_msgs.msg import MagicPanelState
from astromech_msgs.msg import MoodState
from astromech_msgs.srv import ClaimChannel
from nav_msgs.msg import Odometry
from r2_expression import dome_renderer
from r2_expression import led_renderer
from r2_expression import magic_renderer
from r2_expression.config import ExpressionConfigError
from r2_expression.config import load_config
from r2_expression.core import Arbiter
from r2_expression.core import CALL_MAGIC_STOP
from r2_expression.core import CALL_STOP
from r2_expression.core import DOME_CHANNELS
from r2_expression.core import held_by
from r2_expression.core import LED_CHANNELS
from r2_expression.core import MAGIC_CHANNELS
from r2_expression.core import PRIORITY_BEHAVIOR
from r2_expression.core import RUN_CALLS
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
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

# Read once at startup; a later `ros2 param set` of any of these is refused. Every kind has
# these; the LED and Magic Panel nodes add `device`, the dome node DOME_PARAMETERS.
STARTUP_PARAMETERS = ('config', 'use_sim_time', 'allow_duplicate')
# The dome node's names (plan 5.4), parameters so a test can move them without remapping.
DOME_PARAMETERS = {
    'command_topic': '/dome/cmd/personality',
    'status_topic': '/dome/cmd/status',
    'joint_states_topic': '/dome_state_broadcaster/joint_states',
    'odom_topic': '/diff_cont/odom',
    'joint_name': 'dome_joint',
}

REQUEST_TIMEOUT_S = 5.0      # one device request, plan 2.1
LIST_RETRY_S = 5.0           # between attempts at the device's `list`
TICK_S = 0.25                # the core's timers, 4 Hz: leases, holds and durations to 0.25 s
MOVE_TICK_S = 0.05           # the dome's 20 Hz while a move runs (the converter's rate)
CLOSE_ACK_S = 0.2            # how long the dome's last 0 may take to be acknowledged
ODOM_DECIMATE = 3            # the dome reads one odometry message in 3 (rule 11)
POLL_S = 0.1                 # how often a worker in a call or offline looks at the stop flag
DUPLICATE_WAIT_S = 1.0       # discovery time before the graph is asked for a node of our name
GUARD_LOG_S = 60.0           # a failing callback is logged at most this often
# Where the nodes meet. Both offer a claim service; rmw_zenoh gives a request to either
# server of a shared name, so each kind has its own (README, "The claim service").
INTENT_TOPIC = '/expression/intent'
MOOD_TOPIC = '/mood/state'
STATUS_TOPIC = '/expression/status'
CLAIM_SERVICES = {'led': '/expression/claim/led', 'magic': '/expression/claim/magic',
                  'dome': '/expression/claim/dome'}
# tfld and bfld are reported only while they are doing something.
QUIET_WHEN_IDLE = ('tfld', 'bfld')


@dataclass(frozen=True)
class Kind:
    """What differs between the nodes; the dome has no device state topic or catalogue."""

    channels: tuple
    renderer: ModuleType
    state_topic: str = ''            # under the device node
    state_type: Type = None
    on_state: str = ''               # the core method that takes a snapshot
    catalogue: Callable = None


KINDS = {
    'led': Kind(LED_CHANNELS, led_renderer, 'leds/states', LedStateArray, 'led_state',
                led_catalogue),
    'magic': Kind(MAGIC_CHANNELS, magic_renderer, 'magic/states', MagicPanelState,
                  'magic_state', magic_catalogue),
    'dome': Kind(DOME_CHANNELS, dome_renderer),
}


class _ActionQueue:
    """A FIFO of ``(time, Action | None)`` where a newer run replaces an older queued one."""

    def __init__(self):
        self._items = deque()
        self._ready = threading.Condition()
        self._closed = False

    def put_replacing(self, item):
        """
        Queue ``item``; return the queued actions it replaces.

        A run, say or start for a (channel, target) makes any run still queued for the same
        key pointless: the device would play both in a row. Stops and configures are kept.
        """
        key = self._run_key(item)
        replaced = []
        with self._ready:
            if key is not None:
                kept = deque()
                for old in self._items:
                    (replaced if self._run_key(old) == key else kept).append(old)
                self._items = kept
            self._items.append(item)
            self._ready.notify()
        return [old[1] for old in replaced]

    def get(self, timeout=None):
        """
        Return the oldest item.

        Raise ``queue.Empty`` after ``timeout`` seconds (None: no limit), or at once when the
        queue is closed and empty.
        """
        with self._ready:
            if not self._ready.wait_for(lambda: self._items or self._closed, timeout) or \
                    not self._items:
                raise queue.Empty
            return self._items.popleft()

    def close(self):
        """Wake every waiting :meth:`get`; from now on an empty queue raises at once."""
        with self._ready:
            self._closed = True
            self._ready.notify_all()

    @staticmethod
    def _run_key(item):
        action = item[1]
        if action is None or action.call not in RUN_CALLS:
            return None
        return action.channel, action.target


def guarded(on_error=None):
    """Log an exception from a node callback (once a minute per callback) and carry on."""
    def decorate(method):
        @functools.wraps(method)
        def wrapper(self, *args):
            try:
                return method(self, *args)
            except Exception as exc:
                self._log_guarded(method.__name__, exc)
                return on_error(*args) if on_error else None
        return wrapper
    return decorate


def _internal_error(request, response):
    response.accepted = False
    response.reason = 'internal error'
    response.status.channel = request.channel
    return response


def _reliable(depth):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)


def _best_effort(depth):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)


def _steady():
    return Clock(clock_type=ClockType.STEADY_TIME)


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
    """
    What every arbiter shares; a subclass reaches its device.

    A subclass names its startup parameters, declares them in :meth:`_declare_device`
    (returning what the logs call the device), opens its topics, clients and threads in
    :meth:`_open_device`, hands the core's actions on in :meth:`_dispatch` (under
    ``_lock``) and lets go in :meth:`close`.
    """

    startup_parameters = STARTUP_PARAMETERS
    # True: main() leaves rclpy's signal handlers out and a signal ends the spin loop, so
    # the context is still up when close() runs (the dome publishes its last 0 there).
    own_signal_handlers = False

    def __init__(self, kind):
        self._kind = KINDS[kind]
        self._kind_name = kind
        super().__init__('%s_arbiter' % kind)
        self.declare_parameter('config', default_config_path())
        self.declare_parameter('allow_duplicate', False)
        self._device = self._declare_device()
        self._refuse_a_duplicate()
        self._config_path = str(self.get_parameter('config').value)
        config = load_config(self._config_path)     # ExpressionConfigError: main() exits
        self._core = Arbiter(config, self._kind.channels)
        self._lock = threading.Lock()
        self._guard_logged = {}            # callback name -> time.monotonic() of its last log
        self._last_key = {}
        self._held_logged = set()
        self._last_mood = ''
        self._last_status_at = 0.0
        self._status_period = 1.0 / config.defaults.status_hz

        self._status_pub = self.create_publisher(ChannelStatus, STATUS_TOPIC, _reliable(10))
        self.create_subscription(ExpressionIntent, INTENT_TOPIC, self._on_intent, _reliable(50))
        self.create_subscription(MoodState, MOOD_TOPIC, self._on_mood, _reliable(10))
        self.create_service(ClaimChannel, CLAIM_SERVICES[kind], self._on_claim)
        # Registered after TimeSource's own callback, so it runs first and a refused
        # use_sim_time never reaches TimeSource.
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(TICK_S, self._on_tick, clock=_steady())
        self._open_device(config)
        self.get_logger().info(
            '%s_arbiter up: config %s, device %s, channels %s' % (
                kind, self._config_path, self._device, ' '.join(self._core.channels)))

    # --- what a kind supplies ----------------------------------------------------------------

    def _declare_device(self):
        """Declare the device's parameters; return its name for the logs."""
        raise NotImplementedError

    def _open_device(self, config):
        """Create the device's topics, clients and threads."""
        raise NotImplementedError

    def _dispatch(self, actions, now):
        """Send the core's ``actions`` on (``_lock`` is held)."""
        raise NotImplementedError

    def close(self):
        """Let go of the device before the node is destroyed."""
        raise NotImplementedError

    # --- helpers ---------------------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _refuse_a_duplicate(self):
        """
        Raise ValueError when another node of this name is on the graph.

        Two arbiter pairs would fight over one device with the same owner. ``allow_duplicate``
        is for the test that wants two.
        """
        if self.get_parameter('allow_duplicate').value is True:
            return
        time.sleep(DUPLICATE_WAIT_S)      # let discovery of an older node catch up
        name, namespace = self.get_name(), self.get_namespace()
        found = [(n, ns) for n, ns in self.get_node_names_and_namespaces()
                 if n == name and ns == namespace]
        if len(found) > 1:                # one of them is this node
            raise ValueError(
                'another node named %s is already on the graph (a second %s_arbiter, perhaps '
                'on another machine?); two would fight over %s. Stop it, or pass '
                '-p allow_duplicate:=true' % (
                    self.get_fully_qualified_name(), self._kind_name, self._device))

    def _log_guarded(self, name, exc):
        """Log a callback's exception, at most once per GUARD_LOG_S for each callback."""
        now = time.monotonic()
        last = self._guard_logged.get(name)
        if last is None or now - last >= GUARD_LOG_S:
            self._guard_logged[name] = now
            self.get_logger().error('%s failed and was skipped: %s: %s' % (
                name, type(exc).__name__, str(exc)[:200]))

    def _on_parameters(self, params):
        for param in params:
            if param.name in self.startup_parameters:
                return SetParametersResult(
                    successful=False, reason='read at startup; restart the node')
        return SetParametersResult(successful=True)

    def _settle(self, now, actions=(), force=False):
        """Dispatch ``actions``, log core warnings, publish status changes. Hold ``_lock``."""
        self._dispatch(actions, now)
        log = self.get_logger()
        for line in self._core.take_warnings():
            log.warning(line)
        periodic = force or time.monotonic() - self._last_status_at >= self._status_period
        if periodic:
            self._last_status_at = time.monotonic()
        stamp = None                       # built only if something is published
        for status in self._core.statuses(now):
            key = _change_key(status)
            previous = self._last_key.get(status.channel)
            changed = key != previous
            self._last_key[status.channel] = key
            if changed:
                self._log_change(status, previous)
            quiet = status.channel in QUIET_WHEN_IDLE and status.state == STATE_IDLE
            if changed or (periodic and not quiet):
                stamp = stamp or self.get_clock().now().to_msg()
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

    @guarded()
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

    @guarded()
    def _on_mood(self, msg):
        with self._lock:
            if msg.mood and msg.mood != self._last_mood:
                # logged under the lock, so before its palette calls can reach the worker
                self.get_logger().info('mood: %s -> %s' % (self._last_mood or '-', msg.mood))
                self._last_mood = msg.mood
            now = self._now()
            actions = self._core.mood(msg.mood, msg.mood_strength, now)
            self._settle(now, actions)

    @guarded(_internal_error)
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

    @guarded()
    def _on_tick(self):
        with self._lock:
            now = self._now()
            self._settle(now, self._core.tick(now))


class DeviceArbiterNode(ArbiterNode):
    """The LED or the Magic Panel arbiter: service calls from one worker thread."""

    startup_parameters = STARTUP_PARAMETERS + ('device',)

    def _declare_device(self):
        self.declare_parameter('device', '/r2_dome')
        device = str(self.get_parameter('device').value).rstrip('/')
        if not device.startswith('/') or len(device) < 2:
            raise ValueError('device must be an absolute node name like /r2_dome, got %r'
                             % device)
        return device

    def _open_device(self, config):
        self._queue = _ActionQueue()       # (time.monotonic() when queued, Action | None)
        self._max_age_s = config.defaults.device_stale_s
        self._stale_burst = 0              # stale actions dropped since the queue last caught up
        self._stop = threading.Event()
        self._catalogue_ready = False
        self._panel = None                 # magic: (interrogations, link) of the last state
        device = self._device
        self._device_clients = {}
        renderer = self._kind.renderer
        for call, (suffix, srv) in renderer.SERVICES.items():
            self._device_clients[call] = self.create_client(srv, '%s/%s' % (device, suffix))
        self._list_client = self.create_client(
            renderer.LIST[1], '%s/%s' % (device, renderer.LIST[0]))
        self.create_subscription(self._kind.state_type,
                                 '%s/%s' % (device, self._kind.state_topic),
                                 self._on_state, _reliable(10))
        self._worker = threading.Thread(target=self._work, name='%s_worker' % self._kind_name,
                                        daemon=True)
        self._worker.start()

    def _dispatch(self, actions, now):
        for action in actions:
            for old in self._queue.put_replacing((time.monotonic(), action)):
                self._core.device_reply(old, False, 'superseded', now)

    # --- the device's state topic (executor thread) ------------------------------------------

    @guarded()
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
            self._queue.put_replacing((time.monotonic(), None))

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

    def _await_catalogue(self):
        """Run :meth:`_first_catalogue`, again after a pause if it raises."""
        while not self._stop.is_set():
            try:
                return self._first_catalogue()
            except Exception as exc:     # the worker must outlive a bad first `list` too
                self._log_guarded('device worker (first list)', exc)
                self._stop.wait(LIST_RETRY_S)

    def _safe_reply(self, action, exc):
        """Tell the core a request failed inside the worker; never raise."""
        try:
            self._reply(action, False, 'internal: %s' % exc, quiet=True)
        except Exception as again:
            self._log_guarded('device worker (reply)', again)

    def _work(self):
        self._await_catalogue()
        while not self._stop.is_set():
            try:
                # Polled only during a stale burst, so its summary is logged once the
                # queue has caught up; otherwise asleep until an action or close().
                queued_at, item = self._queue.get(timeout=POLL_S if self._stale_burst else None)
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
                self._log_guarded('device worker', exc)
                if item is not None:
                    self._safe_reply(item, exc)

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

    def close(self):
        """Ask the worker to finish (it notices within POLL_S) and wait briefly."""
        self._stop.set()
        self._queue.close()
        self._worker.join(timeout=2.0)


class DomeArbiterNode(ArbiterNode):
    """
    The dome arbiter: velocities on ``command_topic`` for ``dome_twist_converter``.

    Feedback, all fed to :meth:`~r2_expression.core.Arbiter.dome_state`: the joint
    position from ``joint_states_topic`` (depth 1, best effort), who drives the dome from
    ``status_topic`` (depth 1, reliable like the converter's publisher: a source change
    must not be lost) and the body's speed and yaw rate from ``odom_topic`` (depth 1, best
    effort, one message in :data:`ODOM_DECIMATE` deserialised).
    """

    startup_parameters = STARTUP_PARAMETERS + tuple(DOME_PARAMETERS)
    own_signal_handlers = True

    def _declare_device(self):
        for name, default in DOME_PARAMETERS.items():
            self.declare_parameter(name, default)
        self._names = {name: str(self.get_parameter(name).value) for name in DOME_PARAMETERS}
        for name, value in self._names.items():
            if not value:
                raise ValueError('%s must not be empty' % name)
        return 'dome_twist_converter via %s' % self._names['command_topic']

    def _open_device(self, config):
        dome = config.defaults.dome
        self._speed_range = (dome.min_speed_rad_s, dome.max_speed_rad_s)
        self._move_timer = None
        self._sent = 0.0                   # the last velocity published
        self._odom_count = 0
        names = self._names
        self._command_pub = self.create_publisher(
            self._kind.renderer.MESSAGE, names['command_topic'], _reliable(1))
        self.create_subscription(JointState, names['joint_states_topic'],
                                 self._on_joint_state, _best_effort(1))
        self.create_subscription(DomeCommandStatus, names['status_topic'],
                                 self._on_command_status, _reliable(1))
        self.create_subscription(Odometry, names['odom_topic'], self._on_odom,
                                 _best_effort(1), raw=True)

    def _dispatch(self, actions, now):
        for action in actions:
            self._publish(action)
        moving = self._core.moving
        if moving and self._move_timer is None:
            self._move_timer = self.create_timer(MOVE_TICK_S, self._on_move_tick,
                                                 clock=_steady())
        elif not moving and self._move_timer is not None:
            self.destroy_timer(self._move_timer)
            self._move_timer = None
            self.get_logger().info('dome: move over, coast scale %.2f for the next'
                                   % self._core.coast_scale)

    def _send(self, msg):
        """Stamp and publish one command; return its velocity."""
        msg.header.stamp = self.get_clock().now().to_msg()
        self._command_pub.publish(msg)
        self._sent = msg.twist.angular.z
        return self._sent

    def _publish(self, action):
        renderer = self._kind.renderer
        try:
            msg = renderer.render(action, *self._speed_range)
        except Exception as exc:           # a bug in the core or the renderer: stop instead
            self._log_guarded('dome renderer', exc)
            msg = renderer.MESSAGE()
        previous = self._sent
        velocity = self._send(msg)
        if action.why != 'move' or (velocity == 0.0) != (previous == 0.0):
            self.get_logger().debug('dome: %.2f rad/s (%s %s)' % (
                velocity, action.why, action.intent_id))

    def _feed(self, state):
        with self._lock:
            now = self._now()
            self._settle(now, self._core.dome_state(state, now))

    # --- feedback (executor thread) ------------------------------------------------------------

    @guarded()
    def _on_joint_state(self, msg):
        try:
            index = list(msg.name).index(self._names['joint_name'])
        except ValueError:
            return
        if index < len(msg.position):
            self._feed({'position_rad': msg.position[index]})

    @guarded()
    def _on_command_status(self, msg):
        self._feed({'source': msg.source})

    @guarded()
    def _on_odom(self, data):
        take = self._odom_count == 0       # the first, then every ODOM_DECIMATE-th
        self._odom_count = (self._odom_count + 1) % ODOM_DECIMATE
        if not take:
            return
        twist = deserialize_message(data, Odometry).twist.twist
        self._feed({'body_speed_m_s': math.hypot(twist.linear.x, twist.linear.y),
                    'body_yaw_rad_s': twist.angular.z})

    # --- timers --------------------------------------------------------------------------------

    def _on_tick(self):
        if self._move_timer is None:       # while a move runs, the 20 Hz timer ticks
            super()._on_tick()

    def _on_move_tick(self):
        super()._on_tick()

    def close(self):
        """Send one 0 if a move was running (best effort: the converter times out anyway)."""
        if not self._core.moving and self._sent == 0.0:
            return
        try:
            self._send(self._kind.renderer.MESSAGE())
            # Reliable, but the session closes next: give the 0 a moment to leave.
            self._command_pub.wait_for_all_acked(Duration(seconds=CLOSE_ACK_S))
            self.get_logger().info('dome: stopped on shutdown')
        except Exception:                  # the context may be gone already
            pass


NODES = {'led': DeviceArbiterNode, 'magic': DeviceArbiterNode, 'dome': DomeArbiterNode}


def _interrupt(signum, frame):
    """End the spin loop; a second signal (launch repeats Ctrl+C) must not break close()."""
    for each in (signal.SIGINT, signal.SIGTERM):
        signal.signal(each, signal.SIG_IGN)
    raise KeyboardInterrupt


def _try_shutdown():
    """Shut rclpy down unless a signal handler already did."""
    try:
        rclpy.try_shutdown()
    except (RCLError, RuntimeError):     # a session that is slow to close is not a failure
        pass


def main(kind, args=None):
    """Run the ``kind`` arbiter; exit 1 on an invalid config or device at startup."""
    if NODES[kind].own_signal_handlers:
        # rclpy's handlers shut the context down before the loop below ends, and then the
        # final 0 could not be published; here a signal ends the loop, then the node closes.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        for signum in (signal.SIGINT, signal.SIGTERM):     # SIGINT too: it may be ignored
            signal.signal(signum, _interrupt)
    else:
        rclpy.init(args=args)
    try:
        try:
            node = NODES[kind](kind)
        except (ExpressionConfigError, ValueError, TypeError, OSError, KeyError) as exc:
            rclpy.logging.get_logger('%s_arbiter' % kind).error('cannot start: %s' % exc)
            return 1
        except KeyboardInterrupt:          # own handlers: a signal while the node was built
            return 0
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            # spin_once with a timeout, so SIGINT/SIGTERM are noticed on an idle node.
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.5)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RCLError:
            if rclpy.ok():        # not just a signal shutting the context mid-callback
                raise
        finally:
            node.close()
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


def main_dome(args=None):
    """Entry point of ``dome_arbiter``."""
    return main('dome', args)


if __name__ == '__main__':
    sys.exit(main_led())
