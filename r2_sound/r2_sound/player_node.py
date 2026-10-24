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
``sound_player``: R2's sound device (plan 4.3, 5.3).

Services (under the node name, ``/sound_player`` by default): ``~/list`` (ListSounds),
``~/play`` (PlaySound), ``~/vocalize`` (Vocalize: a phrase of :mod:`r2_sound.synth`, played
as the clip ``voice:<seed>``), ``~/stop`` (StopSound), ``~/reload`` (Trigger: the catalogue
again). Topics: ``~/state`` (SoundState), ``/sound/level`` (Float32 RMS),
``/sound/playing`` (Bool, on change) and the dome's level on ``dome_level_topic`` (UInt8).

The executor runs the service callbacks only: the node has no timer. A play request is
resolved here (name or pool, exclusion, gain clamp), a vocalize request checked and given a
seed (0 asks for a random one), and handed to the worker (:class:`r2_sound.player.Player`),
which renders (or has its decoder thread decode), owns the backend and publishes; a slow
decode is answered before it is done, so a call never waits on one. A phrase is cut at the
request's ``length_s`` (rule 3); the synth renders that length, or its shortest phrase when
asked for less. The node has its own SIGINT/SIGTERM handlers, so the context is still up
when it closes: the play is stopped and the zeros and the IDLE state go out before rclpy
shuts down.
"""

import dataclasses
import functools
import math
import os
import random
import signal
import sys
import time

from ament_index_python.packages import get_package_share_directory
from astromech_msgs.msg import SoundClip as SoundClipMsg
from astromech_msgs.msg import SoundState
from astromech_msgs.srv import ListSounds
from astromech_msgs.srv import PlaySound
from astromech_msgs.srv import StopSound
from astromech_msgs.srv import Vocalize
from r2_sound.backends import check_backend, DeviceNotFound, make_backend, NullBackend
from r2_sound.catalogue import CatalogueError, load_catalogue, pools, resolve_play
from r2_sound.decode import DecodeCache
from r2_sound.gain import level_to_byte, play_gain_db, to_dbfs
from r2_sound.parameters import NODE_NAME, PARAMETERS, RANGES
from r2_sound.player import Player, PlayJob, Voice
from r2_sound.synth import LENGTH_RANGE_S, render as render_phrase
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
import yaml

LEVEL_TOPIC = '/sound/level'
PLAYING_TOPIC = '/sound/playing'
DUPLICATE_WAIT_S = 1.0       # discovery time before the graph is asked for a node of our name
CLOSE_ACK_S = 0.2            # how long the last zeros may take to be acknowledged
OWNER_BYTES = 31
GUARD_LOG_S = 60.0           # a failing callback is logged at most this often
MAX_SEED = 2 ** 31 - 1       # a random seed is drawn from 1..MAX_SEED
VOICE_PREFIX = 'voice:'


def default_catalogue_path():
    """Return the installed ``clips.yaml``."""
    return os.path.join(get_package_share_directory('r2_sound'), 'config', 'clips.yaml')


def read_parameters(node):
    """Declare and validate the parameters; return name -> value (ValueError when bad)."""
    loose = ParameterDescriptor(dynamic_typing=True)
    values = {}
    for name, default in PARAMETERS.items():
        node.declare_parameter(name, default, loose)
        value = node.get_parameter(name).value
        if isinstance(default, bool) or isinstance(default, str):
            if type(value) is not type(default):
                raise ValueError('%s must be a %s, got %r' % (
                    name, type(default).__name__, value))
        else:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('%s must be a number, got %r' % (name, value))
            low, high = RANGES[name]
            if not low <= value <= high:
                raise ValueError('%s must be in %g..%g, got %r' % (name, low, high, value))
            value = type(default)(value)
        values[name] = value
    check_backend(values['backend'])
    if values['level_floor_dbfs'] >= values['level_ceiling_dbfs']:
        raise ValueError('level_floor_dbfs must be below level_ceiling_dbfs')
    values['catalogue'] = values['catalogue'] or default_catalogue_path()
    return values


def _qos(depth, latched=False):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                                  else DurabilityPolicy.VOLATILE))


def guarded(fill):
    """Answer a service whose callback raised with ``fill(response)``; log once a minute."""
    def decorate(method):
        @functools.wraps(method)
        def wrapper(self, request, response):
            try:
                return method(self, request, response)
            except Exception as exc:
                self._log_guarded(method.__name__, exc)
                return fill(response)
        return wrapper
    return decorate


def _refused(response):
    response.accepted, response.reason = False, 'internal error'
    return response


def _unchanged(response):
    return response


def _owner(request):
    """Return the request's owner cut to :data:`OWNER_BYTES` (SoundState's field)."""
    return request.owner.encode()[:OWNER_BYTES].decode(errors='ignore')


def _vocalize_refusal(request):
    """Return why a Vocalize request cannot be rendered, or ''."""
    for field in ('arousal', 'valence'):
        value = getattr(request, field)
        if not 0.0 <= value <= 1.0:              # NaN fails it too
            return 'bad %s: must be 0..1' % field
    if not (math.isfinite(request.length_s) and request.length_s >= 0.0):
        return 'bad length_s: must be >= 0'
    return ''


class _Sink:
    """The worker's publishing, as messages (called from the worker thread)."""

    def __init__(self, node, floor_dbfs, ceiling_dbfs):
        self._node = node
        self._floor = floor_dbfs
        self._ceiling = ceiling_dbfs
        self._state_pub = node.create_publisher(SoundState, '~/state', _qos(10))
        self._level_pub = node.create_publisher(Float32, LEVEL_TOPIC, _qos(5))
        self._dome_pub = node.create_publisher(UInt8, node.params['dome_level_topic'], _qos(5))
        self._playing_pub = node.create_publisher(Bool, PLAYING_TOPIC, _qos(1, latched=True))
        self.publishers = (self._state_pub, self._level_pub, self._dome_pub,
                           self._playing_pub)

    def state(self, snapshot):
        msg = SoundState(**dataclasses.asdict(snapshot))
        msg.header.stamp = self._node.get_clock().now().to_msg()
        self._state_pub.publish(msg)

    def level(self, rms):
        rms = min(1.0, max(0.0, float(rms)))
        self._level_pub.publish(Float32(data=rms))
        self._dome_pub.publish(UInt8(data=level_to_byte(to_dbfs(rms), self._floor,
                                                        self._ceiling)))

    def playing(self, value):
        self._playing_pub.publish(Bool(data=bool(value)))

    def warn(self, text):
        self._node.get_logger().warning(text)


class SoundPlayerNode(Node):
    """The node; see the module docstring."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.params = read_parameters(self)
        self._refuse_a_duplicate()
        self._rng = random.Random()
        self._clamped = 0
        self._guard_logged = {}
        self._load()                           # CatalogueError, OSError: main() exits 1
        p = self.params
        rate = p['sample_rate']
        self._sink = _Sink(self, p['level_floor_dbfs'], p['level_ceiling_dbfs'])
        backend = self._make_backend(rate)
        self._player = Player(backend, DecodeCache(p['cache_mb'] * 2 ** 20, rate),
                              self._sink, rate, p['fade_ms'], p['state_hz'],
                              p['level_hz'], synth=render_phrase)
        for name, srv, callback in (('list', ListSounds, self._on_list),
                                    ('play', PlaySound, self._on_play),
                                    ('vocalize', Vocalize, self._on_vocalize),
                                    ('stop', StopSound, self._on_stop),
                                    ('reload', Trigger, self._on_reload)):
            self.create_service(srv, '~/' + name, callback)
        self.add_on_set_parameters_callback(self._on_parameters)
        # The dome may hold a level from before a crash: zeros and IDLE before anything else.
        self._sink.level(0.0)
        self._sink.playing(False)
        self._sink.state(self._player.snapshot())
        self._player.start()
        self.get_logger().info(
            'sound_player up: %d clips from %s (sounds %s), backend %s%s, %d Hz' % (
                len(self._view[0].clips), p['catalogue'], p['sounds_dir'],
                backend.name, ' (%s)' % p['wav_dir'] if p['backend'] == 'wav' else '', rate))

    def _make_backend(self, rate):
        """
        Return the output; a missing device is fatal only with ``require_device``.

        Without it the node logs one ERROR naming the devices found and runs silent on a
        ``null`` backend named ``null (no device: <device>)``, so the state says why.
        """
        p = self.params
        try:
            return make_backend(p['backend'], rate, p['device'], p['stream_linger_s'],
                                p['wav_dir'], self._sink.warn)
        except DeviceNotFound as exc:
            if p['require_device']:
                raise
            self.get_logger().error('%s; running silent on the null backend '
                                    '(require_device:=true to exit)' % exc)
            return NullBackend(rate, self._sink.warn, name='null (no device: %s)' % p['device'])

    # --- the catalogue ----------------------------------------------------------------------

    def _load(self):
        """Read the catalogue and the sounds; log the dropped clips."""
        path = self.params['catalogue']
        catalogue, warnings = load_catalogue(path, self.params['sounds_dir'])
        for text in warnings:
            self.get_logger().warning('catalogue: %s' % text)
        # One attribute, swapped whole: a reader sees one catalogue's three views.
        self._view = (catalogue, catalogue.by_name(), pools(catalogue))
        return catalogue, warnings

    def _refuse_a_duplicate(self):
        """Raise ValueError when another node of this name is on the graph."""
        if self.params['allow_duplicate']:
            return
        time.sleep(DUPLICATE_WAIT_S)      # let discovery of an older node catch up
        name, namespace = self.get_name(), self.get_namespace()
        found = [(n, ns) for n, ns in self.get_node_names_and_namespaces()
                 if n == name and ns == namespace]
        if len(found) > 1:                # one of them is this node
            raise ValueError(
                'another node named %s is already on the graph; two players would answer '
                'the same services. Stop it, or pass -p allow_duplicate:=true'
                % self.get_fully_qualified_name())

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
            if param.name in PARAMETERS or param.name == 'use_sim_time':
                return SetParametersResult(
                    successful=False, reason='read at startup; restart the node')
        return SetParametersResult(successful=True)

    # --- services ---------------------------------------------------------------------------

    @guarded(_unchanged)
    def _on_list(self, request, response):
        catalogue = self._view[0]
        response.clips = [SoundClipMsg(name=c.name, tags=list(c.tags),
                                       duration_ms=c.duration_ms, gain_db=c.gain_db,
                                       excluded=c.excluded) for c in catalogue.clips]
        response.catalogue = self.params['catalogue']
        response.sample_rate = self.params['sample_rate']
        response.synth = True
        return response

    @guarded(_refused)
    def _on_play(self, request, response):
        if self._player.closed:
            response.accepted, response.reason = False, 'closed'
            return response
        _, by_name, pool_map = self._view
        clip, reason = resolve_play(by_name, pool_map, request.name, request.pool,
                                    request.priority, self._rng)
        if clip is None:
            response.accepted, response.reason = False, reason
            self.get_logger().info('play %r refused: %s' % (
                request.name or 'pool ' + request.pool, reason))
            return response
        gain_db, clamped = play_gain_db(clip.gain_db, request.gain_db, clip.peak_dbfs)
        if clamped:
            self._clamped += 1
            self.get_logger().warning(
                'play %s: gain %+.1f dB clamped to %+.1f dB (peak ceiling; %d so far)' % (
                    clip.name, clip.gain_db + request.gain_db, gain_db, self._clamped))
        path = os.path.join(self.params['sounds_dir'], clip.path)
        accepted, reason, duration_ms = self._player.play(PlayJob(
            clip.name, path, clip.gain_db, gain_db, request.duration_ms, request.priority,
            _owner(request), request.intent_id, clip_ms=clip.duration_ms))
        response.accepted, response.reason = accepted, reason
        response.name, response.duration_ms = clip.name, duration_ms
        if not accepted:
            self.get_logger().warning('play %s failed: %s' % (clip.name, reason))
        return response

    @guarded(_refused)
    def _on_vocalize(self, request, response):
        response.seed = request.seed or self._rng.randint(1, MAX_SEED)
        reason = _vocalize_refusal(request)
        if self._player.closed:
            reason = 'closed'
        if reason:
            response.accepted, response.reason = False, reason
            self.get_logger().info('vocalize refused: %s' % reason)
            return response
        name = '%s%d' % (VOICE_PREFIX, response.seed)
        # The synth renders a requested length to the frame; only one under its shortest
        # phrase needs the cut (rule 3). 0: the phrase's own length.
        cut_ms = 0
        if 0.0 < request.length_s < LENGTH_RANGE_S[0]:
            cut_ms = max(1, int(round(request.length_s * 1000.0)))
        accepted, reason, duration_ms = self._player.play(PlayJob(
            name, '', 0.0, request.gain_db, cut_ms, request.priority, _owner(request),
            request.intent_id,
            voice=Voice(request.arousal, request.valence, request.length_s, response.seed)))
        response.accepted, response.reason, response.duration_ms = accepted, reason, duration_ms
        if not accepted:
            self.get_logger().warning('vocalize %s failed: %s' % (name, reason))
        return response

    @guarded(_unchanged)
    def _on_stop(self, request, response):
        response.stopped = self._player.stop()
        return response

    @guarded(_unchanged)
    def _on_reload(self, request, response):
        try:
            catalogue, warnings = self._load()
        except (CatalogueError, OSError, yaml.YAMLError) as exc:
            response.success, response.message = False, str(exc)[:300]
            self.get_logger().error('reload failed, the old catalogue stays: %s' % exc)
            return response
        self._player.cache.clear()
        response.success = True
        response.message = '%d clips from %s, %d dropped' % (
            len(catalogue.clips), self.params['catalogue'], len(warnings))
        self.get_logger().info('reload: %s' % response.message)
        return response

    # --- shutdown ---------------------------------------------------------------------------

    def close(self):
        """Stop the play, publish the zeros and IDLE, and let them leave."""
        self._player.close()
        for publisher in self._sink.publishers:
            try:
                publisher.wait_for_all_acked(Duration(seconds=CLOSE_ACK_S))
            except Exception:              # the context may be gone already
                pass


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


def main(args=None):
    """
    Run ``sound_player``; exit 1 on a bad parameter, catalogue or backend at startup.

    A missing output device is not one unless ``require_device`` (see
    :meth:`SoundPlayerNode._make_backend`).
    """
    # rclpy's own handlers would shut the context down before close() could publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, _interrupt)
    try:
        try:
            node = SoundPlayerNode()
        except (CatalogueError, ValueError, TypeError, OSError, RuntimeError,
                yaml.YAMLError) as exc:
            rclpy.logging.get_logger(NODE_NAME).error('cannot start: %s' % exc)
            return 1
        except KeyboardInterrupt:          # a signal while the node was built
            return 0
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while rclpy.ok():         # a timeout, so a signal on an idle node is noticed
                executor.spin_once(timeout_sec=1.0)
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


if __name__ == '__main__':
    sys.exit(main())
