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
``dome_mock``: the dome device's LED and Magic Panel ROS surface, for development without it.

The node is named ``r2_dome`` by default and every service and state topic hangs off the node
name (``~/leds/run``), so ``-r __node:=other`` moves them clear of a real dome on the graph.
The device's absolute topics (``/dome/text/line0|1``, ``/dome/audio/level``) are kept, and the
mock adds ``~/mock/console``, ``~/mock/picture`` and ``~/mock/image``. Behaviour lives in
:mod:`r2_mocks.led_model` and :mod:`r2_mocks.magic_model`; this file maps them to ROS.
"""

import os
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from astromech_msgs.msg import LedClip
from astromech_msgs.msg import LedDisplay
from astromech_msgs.msg import LedDisplayState
from astromech_msgs.msg import LedStateArray
from astromech_msgs.msg import LedSurface
from astromech_msgs.msg import LedSurfaceState
from astromech_msgs.msg import MagicPanelDescriptor
from astromech_msgs.msg import MagicPanelSequence
from astromech_msgs.msg import MagicPanelSettings
from astromech_msgs.msg import MagicPanelState
from astromech_msgs.srv import ConfigureLedDisplay
from astromech_msgs.srv import ConfigureMagicPanel
from astromech_msgs.srv import ListLeds
from astromech_msgs.srv import ListMagicPanel
from astromech_msgs.srv import RunLedEffect
from astromech_msgs.srv import SayLedText
from astromech_msgs.srv import StartMagicSequence
from astromech_msgs.srv import StopLedEffect
from astromech_msgs.srv import StopMagicSequence
from r2_mocks import picture
from r2_mocks.console import Console
from r2_mocks.holds import Manual
from r2_mocks.led_model import LedModel
from r2_mocks.led_model import load_catalogue
from r2_mocks.magic_model import MagicModel
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import UInt8
import yaml

NAME_BUFFER = 96                # a request string longer than this is never answered
LINE_BUFFER = 128               # ... and a text line longer than this
MAGIC_START_BLOCK_S = 0.01      # magic/start waits for the panel's counter to move
PICTURE_PERIOD_S = 1.0
MAX_STATE_HZ = 10.0
EXECUTOR_THREADS = 4            # up to two can sit in a deliberate no-reply wait
TOO_LONG_REASON = 'request string too long'
CONFIG_DEFAULTS = {'state_hz': 1.0, 'hold_ms': 5000.0, 'too_long_sleep_s': 5.5}


def default_config_path():
    """Return the installed ``dome_mock.yaml``."""
    return os.path.join(get_package_share_directory('r2_mocks'), 'config', 'dome_mock.yaml')


def fill(msg, values):
    """Copy the entries of ``values`` that are fields of ``msg`` onto it (nested dicts too)."""
    for key in msg.get_fields_and_field_types():
        if key in values:
            if isinstance(values[key], dict):
                fill(getattr(msg, key), values[key])
            else:
                setattr(msg, key, values[key])
    return msg


def fill_list(msg_type, items):
    """Return a list of ``msg_type`` messages built from the dicts in ``items``."""
    return [fill(msg_type(), item) for item in items]


def locked(method):
    """Run a subscription or timer callback under the node's lock."""
    def wrapper(self, *args):
        with self._lock:
            return method(self, *args)
    return wrapper


class DomeMock(Node):
    """The mock device node."""

    def __init__(self):
        super().__init__('r2_dome')
        loose = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('config', default_config_path(), loose)
        self.declare_parameter('fitted', True, loose)
        values = {}
        for name, default in CONFIG_DEFAULTS.items():
            self.declare_parameter(name, default, loose)
            value = self.get_parameter(name).value
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('%s must be a number, got %r' % (name, value))
            values[name] = float(value)
        if not 0.0 < values['state_hz'] <= MAX_STATE_HZ:
            raise ValueError('state_hz must be in (0, %g], got %r' % (
                MAX_STATE_HZ, values['state_hz']))
        if values['hold_ms'] < 0.0 or values['too_long_sleep_s'] < 0.0:
            raise ValueError('hold_ms and too_long_sleep_s must be >= 0')
        fitted = self.get_parameter('fitted').value
        if not isinstance(fitted, bool):
            raise ValueError('fitted must be a bool, got %r' % (fitted,))
        self.no_reply_s = values['too_long_sleep_s']
        self.catalogue = load_catalogue(self.get_parameter('config').value)
        manual = Manual()           # the sticky setting, one flag for both models
        self.leds = LedModel(self.catalogue, hold_ms=values['hold_ms'], manual=manual)
        self.panel = MagicModel(
            self.catalogue, hold_ms=values['hold_ms'], fitted=fitted, manual=manual)
        self.console = Console(self.leds, self.panel)
        self.panel.poll(self.now())
        self.layout = picture.Layout(self.catalogue)
        # The models are touched from up to two executor threads; this serialises them.
        self._lock = threading.RLock()
        self.stopping = threading.Event()
        self._last_colours = None
        self._last_text = None
        self._image_data = b''
        self._last_picture_at = None
        self._listing = {
            'displays': fill_list(LedDisplay, self.catalogue['displays']),
            'surfaces': fill_list(LedSurface, self.catalogue['surfaces']),
            'clips': fill_list(LedClip, self.catalogue['clips'])}

        self._make_services()
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._led_pub = self.create_publisher(LedStateArray, '~/leds/states', reliable)
        self._magic_pub = self.create_publisher(MagicPanelState, '~/magic/states', reliable)
        self._picture_pub = self.create_publisher(String, '~/mock/picture', reliable)
        self._image_pub = self.create_publisher(Image, '~/mock/image', reliable)
        self.create_subscription(String, '~/mock/console', self._on_console, reliable)
        for index in (0, 1):
            self.create_subscription(
                String, '/dome/text/line%d' % index,
                lambda msg, index=index: self._on_text(index, msg), reliable)
        self.create_subscription(
            UInt8, '/dome/audio/level', self._on_level,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0 / values['state_hz'], self._tick)
        self.create_timer(self.panel.panel['poll_period_ms'] / 1000.0, self._poll)
        self.get_logger().info(
            'dome mock up as %s (config %s, state_hz %g, hold_ms %g, magic panel %s)' % (
                self.get_fully_qualified_name(), self.get_parameter('config').value,
                values['state_hz'], values['hold_ms'], 'fitted' if fitted else 'not fitted'))

    def now(self):
        """Return the node clock in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def _make_services(self):
        group = ReentrantCallbackGroup()     # a no-reply sleep must not stall the others
        for name, srv, callback in (
                ('leds/list', ListLeds, self._list_leds),
                ('leds/run', RunLedEffect, self._run),
                ('leds/say', SayLedText, self._say),
                ('leds/stop', StopLedEffect, self._stop),
                ('leds/configure', ConfigureLedDisplay, self._configure),
                ('magic/list', ListMagicPanel, self._list_magic),
                ('magic/start', StartMagicSequence, self._start),
                ('magic/stop', StopMagicSequence, self._magic_stop),
                ('magic/configure', ConfigureMagicPanel, self._magic_configure)):
            self.create_service(srv, '~/' + name, callback, callback_group=group)

    # --- serving a request ---------------------------------------------------------------

    def _serve(self, response, what, apply, names=(), lines=(), pause_s=0.0):
        """
        Answer a request the way the device does.

        A string over its buffer is never answered by the device: the mock logs a WARN,
        waits past the client's 5 s timeout (outside the lock, so state keeps publishing)
        and replies anyway having applied nothing. Otherwise ``apply()`` fills the response
        under the lock, and ``pause_s`` is the time the device blocks after it.
        """
        if (any(len(n.encode()) > NAME_BUFFER for n in names)
                or any(len(n.encode()) > LINE_BUFFER for n in lines)):
            self.get_logger().warning(
                'request string over its buffer (%d bytes for names, %d per line): the '
                'device would never answer; waiting %g s' % (
                    NAME_BUFFER, LINE_BUFFER, self.no_reply_s))
            self.stopping.wait(self.no_reply_s)
            response.accepted, response.reason = False, TOO_LONG_REASON
            return response
        with self._lock:
            apply()
            if not response.accepted:
                self.get_logger().info('%s refused: %s' % (what, response.reason))
            else:
                self._publish_views()
        if pause_s:
            time.sleep(pause_s)
        return response

    # --- LED services -------------------------------------------------------------------

    def _list_leds(self, request, response):
        cat = self.catalogue
        response.displays = self._listing['displays']
        response.surfaces = self._listing['surfaces']
        response.effects = [e['name'] for e in cat['effects']]
        response.effect_summaries = [e['summary'] for e in cat['effects']]
        response.clips = self._listing['clips']
        response.text_motions = list(cat['text_motions'])
        response.drift_presets = list(cat['drift_presets'])
        return response

    def _run(self, request, response):
        def apply():
            response.accepted, response.reason = self.leds.run(
                self.now(), request.surface, request.effect, request.rgb, request.speed,
                request.duration_ms, request.owner, request.priority)
        return self._serve(response, 'run %s %s' % (request.surface, request.effect), apply,
                           (request.surface, request.effect, request.owner))

    def _say(self, request, response):
        def apply():
            response.accepted, response.reason = self.leds.say(
                self.now(), request.surface, list(request.lines), request.linked,
                request.motion, request.on_black, request.rgb, request.speed,
                request.duration_ms, request.owner, request.priority)
        return self._serve(response, 'say %s' % request.surface, apply,
                           (request.surface, request.owner), request.lines)

    def _stop(self, request, response):
        def apply():
            response.accepted, response.reason, response.stopped = self.leds.stop(
                self.now(), request.surface, request.priority)
        return self._serve(response, 'stop %s' % request.surface, apply, (request.surface,))

    def _configure(self, request, response):
        def apply():
            response.accepted, response.reason, state = self.leds.configure(
                self.now(), request.display, request.apply, request.mode, request.brightness,
                request.hue, request.drift, request.rgb, request.owner, request.priority)
            if state is not None:
                response.result = fill(LedDisplayState(), state)
        return self._serve(response, 'configure %s' % request.display, apply,
                           (request.display, request.drift, request.owner))

    # --- Magic Panel services -----------------------------------------------------------

    def _list_magic(self, request, response):
        with self._lock:
            descriptor, sequences = self.panel.list_catalogue(self.now())
        response.panel = fill(MagicPanelDescriptor(), descriptor)
        response.sequences = fill_list(MagicPanelSequence, sequences)
        return response

    def _start(self, request, response):
        def apply():
            reply = self.panel.start(
                self.now(), request.sequence_id, request.repeat, request.end, request.owner,
                request.priority)
            response.accepted, response.reason = reply['accepted'], reply['reason']
            response.confirmed, response.run_counter = reply['confirmed'], reply['run_counter']
        return self._serve(response, 'magic start %d' % request.sequence_id, apply,
                           pause_s=MAGIC_START_BLOCK_S)

    def _magic_stop(self, request, response):
        def apply():
            reply = self.panel.stop(self.now(), request.mode, request.priority)
            response.accepted, response.reason = reply['accepted'], reply['reason']
        return self._serve(response, 'magic stop', apply)

    def _magic_configure(self, request, response):
        def apply():
            reply = self.panel.configure(
                request.apply, request.brightness, request.default_brightness,
                request.rotated, request.legacy, request.inputs, request.resume)
            response.accepted, response.reason, response.saved = (
                reply['accepted'], reply['reason'], reply['saved'])
            response.result = fill(MagicPanelSettings(), reply['result'])
        return self._serve(response, 'magic configure', apply)

    # --- topics -------------------------------------------------------------------------

    @locked
    def _on_console(self, msg):
        ok, message = self.console.execute(self.now(), msg.data)
        log = self.get_logger().info if ok else self.get_logger().warning
        log('console %r: %s' % (msg.data, message))
        self._publish_views()

    @locked
    def _on_text(self, index, msg):
        if len(msg.data.encode()) > LINE_BUFFER:
            self.get_logger().warning('text line over %d bytes ignored' % LINE_BUFFER)
            return
        accepted, reason = self.leds.text_slot(self.now(), index, msg.data)
        if not accepted:
            self.get_logger().warning('text line%d refused: %s' % (index, reason))
        self._publish_views()

    @locked
    def _on_level(self, msg):
        self.leds.set_level(msg.data)

    @locked
    def _poll(self):
        self.panel.poll(self.now())

    @locked
    def _tick(self):
        """Publish both state topics; once per PICTURE_PERIOD_S also the views as a heartbeat."""
        now = self.now()
        led_state, panel_state = self.leds.state(now), self.panel.state(now)
        stamp = self.get_clock().now().to_msg()
        msg = LedStateArray()
        msg.header.stamp = stamp
        msg.level, msg.level_set = led_state['level'], led_state['level_set']
        msg.displays = fill_list(LedDisplayState, led_state['displays'])
        msg.surfaces = fill_list(LedSurfaceState, led_state['surfaces'])
        msg.strip_enabled, msg.manual = led_state['strip_enabled'], led_state['manual']
        self._led_pub.publish(msg)
        out = fill(MagicPanelState(), panel_state)
        out.header.stamp = stamp
        self._magic_pub.publish(out)
        beat = (self._last_picture_at is None
                or now - self._last_picture_at >= PICTURE_PERIOD_S)
        if beat:
            self._last_picture_at = now
        self._publish_views(led_state, panel_state, now, heartbeat=beat)

    def _publish_views(self, led_state=None, panel_state=None, now=None, heartbeat=False):
        """Publish the text picture and the image when they changed (or as a heartbeat)."""
        if led_state is None:
            now = self.now()
            led_state, panel_state = self.leds.state(now), self.panel.state(now)
        text = picture.picture_text(led_state, panel_state)
        if heartbeat or text != self._last_text:
            self._last_text = text
            self._picture_pub.publish(String(data=text))
        colours = self.layout.colours(led_state, now)
        changed = colours != self._last_colours
        if changed:         # the image is a function of the colours alone: render only then
            self._last_colours = colours
            self._image_data = self.layout.render(colours)
        if changed or heartbeat:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width, msg.encoding = self.layout.height, self.layout.width, 'rgb8'
            msg.step = self.layout.width * 3
            msg.data = self._image_data
            self._image_pub.publish(msg)


def _try_shutdown():
    """Shut rclpy down unless a signal handler already did."""
    try:
        rclpy.try_shutdown()
    except RCLError:
        pass


def main(args=None):
    """Run the node until SIGINT/SIGTERM; exit 1 on a bad parameter or config."""
    rclpy.init(args=args)
    try:
        try:
            node = DomeMock()
        except (ValueError, OSError, KeyError, TypeError, AttributeError,
                yaml.YAMLError) as exc:
            rclpy.logging.get_logger('dome_mock').error('cannot start: %s' % exc)
            return 1
        except KeyboardInterrupt:       # a signal while the node was still being built
            return 0
        executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
        executor.add_node(node)
        try:
            # spin_once with a timeout, so SIGINT/SIGTERM are noticed on an idle node.
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.2)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RCLError:
            if rclpy.ok():
                raise
        finally:
            node.stopping.set()         # ends a deliberate no-reply wait at once
            executor.shutdown(timeout_sec=0)
            node.destroy_node()
    finally:
        _try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
