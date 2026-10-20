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
The ``express`` operator tool: drive R2's lights by hand through the arbiters.

    express claim   <channel> [--owner NAME] [--priority 199] [--lease 30]
    express release <channel> [--owner NAME]
    express effect  <channel> <name> [--rgb ff2200] [--speed 5] [--duration 5]
    express clip    <channel> <name> [...]
    express text    <channel> "line1" ["line2"] [--motion bounce] [--on-black] [...]
    express configure <channel> [--drift calm|lively] [--brightness N]
    express sequence magic <id|name> [--repeat 1] [--end leave|blank] [--duration 5]
    express stop    <channel> [--mode blank|freeze]
    express status  [--watch]
    express dome    "<console line>"          # the sim mock's console only

Intents go out as ``source: manual`` at priority 199, which a person at the dome still
beats; ``--override`` sends 200.  Exit codes: 0 done (claim accepted), 1 refused or
unavailable, 2 bad arguments.  The argument handling (:func:`parse_args`,
:func:`build_intent`) needs no ROS.
"""

import argparse
import json
import os
import re
import sys
import time
import uuid

from r2_expression.config import (
    BRIGHTNESS_RANGE, CLIPLESS_CHANNELS, DRIFT_PRESETS, PICK_CHANNELS)
from r2_expression.core import END_VALUES, LADDER, STATE_NAMES, STOP_MODES, TEXT_MOTIONS

CHANNELS = PICK_CHANNELS
MOTIONS = TEXT_MOTIONS
DRIFTS = DRIFT_PRESETS
ENDS = tuple(END_VALUES)
STOP_MODE_NAMES = tuple(STOP_MODES)
SOURCE_PRIORITY = {'idle': LADDER['idle'], 'behavior': LADDER['behavior']}
DEFAULT_PRIORITY = 199         # one below the device's own override: a person at the dome wins
OVERRIDE_PRIORITY = LADDER['manual']
MAX_NAME = 63                  # names and text lines must stay under 64 characters
MAX_LINES = {'fld': 2}         # displays per channel; every other channel has one
LED_BRIGHTNESS = BRIGHTNESS_RANGE['fld'][1]
MAGIC_BRIGHTNESS = BRIGHTNESS_RANGE['magic'][1]

INTENT_TOPIC = '/expression/intent'
STATUS_TOPIC = '/expression/status'
CLAIM_SERVICE = '/expression/claim'
DEFAULT_DEVICE = '/r2_dome'

LISTENER_WAIT_S = 2.0
FLUSH_S = 0.3
SERVICE_WAIT_S = 5.0
SERVICE_REPLY_S = 5.0
STATUS_WINDOW_S = 1.5

_RGB = re.compile(r'(?:0x)?([0-9a-fA-F]{6})\Z')


def _is_id(ref):
    return ref.isascii() and ref.isdigit()


def _channel(value):
    if value not in CHANNELS:
        raise argparse.ArgumentTypeError(
            'unknown channel %r (one of %s)' % (value, ', '.join(CHANNELS)))
    return value


def _name(value):
    if not value or len(value) > MAX_NAME:
        raise argparse.ArgumentTypeError('must be 1 to %d characters' % MAX_NAME)
    return value


def _rgb(value):
    match = _RGB.match(value)
    if not match:
        raise argparse.ArgumentTypeError('%r is not six hex digits (rrggbb)' % value)
    return int(match.group(1), 16)


def _ranged_int(low, high):
    def parse(value):
        try:
            number = int(value)
        except ValueError:
            raise argparse.ArgumentTypeError('%r is not an integer' % value)
        if not low <= number <= high:
            raise argparse.ArgumentTypeError('%d is outside %d to %d' % (number, low, high))
        return number
    return parse


def _seconds(value):
    try:
        number = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError('%r is not a number' % value)
    if not 0 <= number <= 4294967.0:      # duration_ms is a uint32
        raise argparse.ArgumentTypeError('%s must be 0 or more seconds' % value)
    return number


def _line(value):
    if len(value) > MAX_NAME:
        raise argparse.ArgumentTypeError('a text line must be under 64 characters')
    return value


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(prog='express', description="Drive R2's lights by hand.")
    sub = parser.add_subparsers(dest='command', required=True)

    def sender(p, channel=True):
        if channel:
            p.add_argument('channel', type=_channel)
        p.add_argument('--override', action='store_true',
                       help='send at 200 (OVERRIDE_PRIORITY): beats a person at the dome')
        p.add_argument('--source', choices=sorted(SOURCE_PRIORITY),
                       help='testing: send as this source at its ladder priority (default: '
                            'manual at 199)')
        p.add_argument('--priority', type=_ranged_int(0, 255), metavar='N',
                       help='with --source only: the priority to send')
        p.add_argument('--duration', type=_seconds, default=0.0, metavar='S',
                       help="seconds (0: the action's own length, or until stopped)")
        p.add_argument('--hold', action='store_true',
                       help='not interruptible by a newer intent of the same priority')
        p.add_argument('--id', dest='intent_id', type=_name,
                       help='intent id (default: generated)')

    def look(p):
        p.add_argument('--rgb', type=_rgb, metavar='RRGGBB', help='colour (default: its own)')
        p.add_argument('--speed', type=_ranged_int(1, 9), metavar='1-9')

    for verb in ('effect', 'clip'):
        p = sub.add_parser(verb, help='run an LED effect' if verb == 'effect'
                           else 'run a named clip')
        sender(p)
        p.add_argument('name', type=_name)
        look(p)
    p = sub.add_parser('text', help='show text on an LED surface')
    sender(p)
    p.add_argument('lines', nargs='+', type=_line, metavar='line')
    p.add_argument('--motion', choices=MOTIONS)
    p.add_argument('--on-black', action='store_true')
    p.add_argument('--linked', action='store_true')
    look(p)
    p = sub.add_parser('configure', help='set drift and/or brightness (never hue)')
    sender(p)
    p.add_argument('--drift', choices=DRIFTS)
    p.add_argument('--brightness', type=_ranged_int(0, LED_BRIGHTNESS), metavar='N',
                   help='0-100; magic 0-15')
    p = sub.add_parser('sequence', help='start a Magic Panel sequence')
    sender(p)
    p.add_argument('ref', metavar='id|name')
    p.add_argument('--repeat', type=_ranged_int(0, 255), default=1)
    p.add_argument('--end', choices=ENDS)
    p = sub.add_parser('stop', help='stop what a channel is running')
    sender(p)
    p.add_argument('--mode', choices=STOP_MODE_NAMES, help='magic only')

    for verb, text in (('claim', 'hold a channel'), ('release', 'release a claim')):
        p = sub.add_parser(verb, help=text)
        p.add_argument('channel', type=_channel)
        p.add_argument('--owner', default='express', type=_name)
        p.add_argument('--claim-service', default=CLAIM_SERVICE, metavar='NAME')
        if verb == 'claim':
            p.add_argument('--priority', type=_ranged_int(0, 255), default=DEFAULT_PRIORITY)
            p.add_argument('--override', action='store_true', help='claim at 200')
            p.add_argument('--lease', type=_seconds, default=30.0, metavar='S',
                           help='seconds, renewed by calling again (the arbiter caps it)')
    p = sub.add_parser('status', help='print one line per channel from /expression/status')
    p.add_argument('--watch', action='store_true')
    p = sub.add_parser('dome', help='send a console line to the sim dome mock')
    p.add_argument('line')
    p.add_argument('--device', default=DEFAULT_DEVICE, metavar='NS')
    return parser


def parse_args(argv):
    """Parse and validate argv (without ROS arguments); exit 2 on bad input."""
    parser = build_parser()
    ns = parser.parse_args(argv)
    cmd = ns.command
    if cmd == 'claim' and ns.override:
        ns.priority = OVERRIDE_PRIORITY
    if cmd not in ('effect', 'clip', 'text', 'configure', 'sequence', 'stop'):
        return ns
    if ns.priority is not None and ns.source is None:
        parser.error('--priority needs --source idle or behavior')
    if ns.override and ns.source is not None:
        parser.error('--override only applies to the default manual source')
    magic = ns.channel == 'magic'
    if cmd == 'sequence' and not magic:
        parser.error('sequence is for magic only')
    if cmd in ('effect', 'clip', 'text') and magic:
        parser.error('%s is for the LED channels, not magic' % cmd)
    if cmd == 'text' and ns.channel == 'psi':
        parser.error('the PSIs have no text mode')
    if cmd == 'text' and len(ns.lines) > MAX_LINES.get(ns.channel, 1):
        parser.error('%s takes at most %d text line(s)' % (
            ns.channel, MAX_LINES.get(ns.channel, 1)))
    if cmd == 'clip' and ns.channel in CLIPLESS_CHANNELS:
        parser.error('%s has no clips' % ns.channel)
    if cmd == 'configure':
        if ns.drift is None and ns.brightness is None:
            parser.error('configure needs --drift and/or --brightness')
        if ns.drift and ns.channel not in ('fld', 'rld'):
            parser.error('--drift is for fld and rld')
        if ns.brightness is not None and magic and ns.brightness > MAGIC_BRIGHTNESS:
            parser.error('magic brightness is 0 to 15')
    if cmd == 'stop' and ns.mode and not magic:
        parser.error('--mode is for magic only')
    if cmd == 'sequence' and _is_id(ns.ref) and int(ns.ref) > 255:
        parser.error('a sequence id is 0 to 255')
    if cmd == 'sequence' and len(ns.ref) > MAX_NAME:
        parser.error('a sequence name must be under 64 characters')
    return ns


def build_intent(ns):
    """Return the ``ExpressionIntent`` fields (``params`` as JSON text) for a parsed sender."""
    action = ns.command
    params = {}
    if action in ('effect', 'clip'):
        params['name'] = ns.name
    if action in ('effect', 'clip', 'text'):
        if ns.rgb is not None:
            params['rgb'] = ns.rgb
        if ns.speed is not None:
            params['speed'] = ns.speed
    if action == 'text':
        params['lines'] = list(ns.lines)
        if ns.motion:
            params['motion'] = ns.motion
        if ns.on_black:
            params['on_black'] = True
        if ns.linked:
            params['linked'] = True
    elif action == 'configure':
        if ns.channel == 'magic':
            action = 'brightness'
            params['value'] = ns.brightness
        else:
            if ns.drift:
                params['drift'] = ns.drift
            if ns.brightness is not None:
                params['brightness'] = ns.brightness
    elif action == 'sequence':
        if _is_id(ns.ref):
            params['id'] = int(ns.ref)
        else:
            params['name'] = ns.ref
        params['repeat'] = ns.repeat
        if ns.end:
            params['end'] = ns.end
    elif action == 'stop' and ns.mode:
        params['mode'] = ns.mode
    if ns.source:
        source = ns.source
        priority = SOURCE_PRIORITY[source] if ns.priority is None else ns.priority
    else:
        source = 'manual'
        priority = OVERRIDE_PRIORITY if ns.override else DEFAULT_PRIORITY
    return {
        'channel': ns.channel,
        'source': source,
        'priority': priority,
        'action': action,
        'params': json.dumps(params) if params else '',
        'duration_ms': int(round(ns.duration * 1000)),
        'interruptible': not ns.hold,
        'intent_id': ns.intent_id or 'express-%s' % uuid.uuid4().hex[:8],
    }


def format_status(msg):
    """Return the one-line text for a ``ChannelStatus``."""
    state = STATE_NAMES[msg.state] if msg.state < len(STATE_NAMES) else str(msg.state)
    active = 'active=%s/%s %dms' % (msg.active_source, msg.active_action,
                                    msg.active_remaining_ms) if msg.active_action else 'active=-'
    return ('%-5s %-8s owner=%s prio=%d lease=%s %s mood=%s dropped=%d refused=%d' % (
        msg.channel, state, msg.owner or '-', msg.priority,
        'forever' if msg.lease_remaining_s < 0 else '%.1fs' % msg.lease_remaining_s, active,
        msg.mood or '-', msg.dropped, msg.refused))


def _wait_for_listener(publisher):
    end = time.monotonic() + LISTENER_WAIT_S
    while time.monotonic() < end:
        if publisher.get_subscription_count() > 0:
            return True
        time.sleep(0.05)
    return publisher.get_subscription_count() > 0


def _spin_for(executor, seconds):
    import rclpy
    end = time.monotonic() + seconds
    while time.monotonic() < end and rclpy.ok():
        executor.spin_once(timeout_sec=0.02)


def _qos():
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def run_publish(node, executor, msg_type, topic, fill, description):
    """Publish one message; return 0, or 1 if no subscriber matched (see ``express status``)."""
    publisher = node.create_publisher(msg_type, topic, _qos())
    listening = _wait_for_listener(publisher)
    if not listening:
        print('express: nobody is listening on %s (is an arbiter running?)' % topic,
              file=sys.stderr)
    msg = msg_type()
    fill(msg)
    publisher.publish(msg)
    print(description)
    _spin_for(executor, FLUSH_S)
    return 0 if listening else 1


def run_intent(node, executor, ns):
    """Publish the intent for a sender subcommand."""
    from astromech_msgs.msg import ExpressionIntent
    fields = build_intent(ns)

    def fill(msg):
        msg.header.stamp = node.get_clock().now().to_msg()
        for key, value in fields.items():
            setattr(msg, key, value)

    return run_publish(
        node, executor, ExpressionIntent, INTENT_TOPIC, fill,
        'sent %s %s %s at %d (%s) id=%s' % (
            fields['source'], fields['channel'], fields['action'], fields['priority'],
            fields['params'] or '-', fields['intent_id']))


def run_dome(node, executor, ns):
    """Publish a console line to the sim mock."""
    from std_msgs.msg import String
    topic = '%s/mock/console' % ns.device.rstrip('/')
    publisher = node.create_publisher(String, topic, _qos())
    listening = _wait_for_listener(publisher)
    if not listening:
        print('note: nobody subscribes to %s; it exists only in the sim (the real dome has its '
              'own console)' % topic, file=sys.stderr)
    publisher.publish(String(data=ns.line))
    print('sent %r to %s' % (ns.line, topic))
    _spin_for(executor, FLUSH_S)
    return 0 if listening else 1


def run_claim(node, executor, ns):
    """Call the claim service; return the exit code."""
    from astromech_msgs.srv import ClaimChannel
    client = node.create_client(ClaimChannel, ns.claim_service)
    if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
        print('express: %s not available after %.0f s (is an arbiter running?)'
              % (ns.claim_service, SERVICE_WAIT_S), file=sys.stderr)
        return 1
    request = ClaimChannel.Request()
    request.channel = ns.channel
    request.owner = ns.owner
    if ns.command == 'claim':
        request.priority = ns.priority
        request.lease_s = ns.lease
    else:
        request.priority = DEFAULT_PRIORITY
        request.lease_s = 0.0
    future = client.call_async(request)
    executor.spin_until_future_complete(future, timeout_sec=SERVICE_REPLY_S)
    if not future.done() or future.result() is None:
        print('express: no reply from %s' % ns.claim_service, file=sys.stderr)
        return 1
    response = future.result()
    print('%s: %s' % ('accepted' if response.accepted else 'refused: ' + response.reason,
                      format_status(response.status)))
    return 0 if response.accepted else 1


def run_status(node, executor, ns):
    """Print channel status lines; return the exit code."""
    import rclpy
    from astromech_msgs.msg import ChannelStatus
    latest = {}

    def on_status(msg):
        if ns.watch:
            print(format_status(msg), flush=True)
        latest[msg.channel] = msg

    node.create_subscription(ChannelStatus, STATUS_TOPIC, on_status, _qos())
    if ns.watch:
        while rclpy.ok():                 # a timeout, so SIGTERM is noticed with no traffic
            executor.spin_once(timeout_sec=0.2)
        return 0
    _spin_for(executor, STATUS_WINDOW_S)
    for channel in sorted(latest):
        print(format_status(latest[channel]))
    if not latest:
        print('express: nothing on %s in %.1f s (is an arbiter running?)'
              % (STATUS_TOPIC, STATUS_WINDOW_S), file=sys.stderr)
    return 0 if latest else 1


def _try_shutdown():
    """Shut rclpy down unless a signal handler already did."""
    import rclpy
    from rclpy._rclpy_pybind11 import RCLError
    try:
        rclpy.try_shutdown()
    except RCLError:
        pass


def main(args=None):
    """Run the express tool."""
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args

    args = sys.argv if args is None else args
    ns = parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    try:
        node = Node('express_%d' % os.getpid(), start_parameter_services=False)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            if ns.command in ('claim', 'release'):
                return run_claim(node, executor, ns)
            if ns.command == 'status':
                return run_status(node, executor, ns)
            if ns.command == 'dome':
                return run_dome(node, executor, ns)
            return run_intent(node, executor, ns)
        except (KeyboardInterrupt, ExternalShutdownException):
            return 0
        finally:
            executor.shutdown(timeout_sec=0)
            node.destroy_node()
    finally:
        _try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
