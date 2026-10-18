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
The ``mood`` command line tool: force, lock, clear and watch R2's mood.

    mood set <label> [--lock] [--duration S]
    mood lock [--duration S]
    mood clear
    mood watch

Every subcommand takes ``--source NAME`` (default ``mood_cli``); ROS arguments
such as ``--ros-args -p use_sim_time:=true`` are honoured as for any node.
"""

import argparse
import math
import sys

from astromech_msgs.msg import MoodState
from astromech_msgs.srv import SetMood
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.utilities import remove_ros_args

SERVICE_WAIT_S = 5.0
SERVICE_REPLY_S = 5.0


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(prog='mood', description="Set or watch R2's mood.")
    sub = parser.add_subparsers(dest='command', required=True)

    def common(p):
        p.add_argument('--source', default='mood_cli',
                       help='who is asking; recorded as lock_source (default mood_cli)')

    p_set = sub.add_parser('set', help='force the vector to a mood, optionally locking it')
    p_set.add_argument('label', help='a mood from affect.yaml, e.g. sleepy')
    p_set.add_argument('--lock', action='store_true', help='freeze the dynamics there')
    p_set.add_argument('--duration', type=float, default=0.0, metavar='S',
                       help='lock for S seconds (0: until cleared)')
    common(p_set)
    p_lock = sub.add_parser('lock', help='lock the vector where it is')
    p_lock.add_argument('--duration', type=float, default=0.0, metavar='S',
                        help='lock for S seconds (0: until cleared)')
    common(p_lock)
    common(sub.add_parser('clear', help='clear any force or lock'))
    common(sub.add_parser('watch', help='print every /mood/state message'))
    return parser


def format_state(msg):
    """Return the one-line text for a ``MoodState``."""
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    lock = 'locked(%s)' % msg.lock_source if msg.locked else 'unlocked'
    if msg.locked and (msg.lock_until.sec or msg.lock_until.nanosec):
        lock += ' until %.1f' % (msg.lock_until.sec + msg.lock_until.nanosec * 1e-9)
    if msg.focus_subject or not math.isnan(msg.focus_bearing_rad):
        focus = 'focus=%s@%.2f' % (msg.focus_subject or '-', msg.focus_bearing_rad)
    else:
        focus = 'focus=-'
    return ('%.2f %s (%.2f) arousal=%.2f valence=%.2f curiosity=%.2f confidence=%.2f %s %s '
            'idle=%.1fs' % (stamp, msg.mood, msg.mood_strength, msg.arousal, msg.valence,
                            msg.curiosity, msg.confidence, lock, focus, msg.idle_s))


def call_set_mood(node, request):
    """Call /mood/set; return the response or None (message printed) if unavailable."""
    client = node.create_client(SetMood, '/mood/set')
    if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
        print('mood: /mood/set not available after %.0f s (is r2_affect running?)'
              % SERVICE_WAIT_S, file=sys.stderr)
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=SERVICE_REPLY_S)
    if not future.done():
        print('mood: no reply from /mood/set', file=sys.stderr)
        return None
    return future.result()


def run_set(node, args):
    """Run set, lock and clear; return the exit code."""
    request = SetMood.Request()
    request.source = args.source
    if args.command == 'set':
        request.mood = args.label
        request.lock = args.lock
        request.duration_s = args.duration
    elif args.command == 'lock':
        request.lock = True
        request.duration_s = args.duration
    response = call_set_mood(node, request)
    if response is None:
        return 1
    result = response.result
    if response.accepted:
        print('accepted: mood %s (%.2f), %s' % (
            result.mood, result.mood_strength,
            'locked by %s' % result.lock_source if result.locked else 'unlocked'))
        return 0
    print('refused: %s (mood %s, %s)' % (
        response.reason, result.mood,
        'locked by %s' % result.lock_source if result.locked else 'unlocked'))
    return 1


def run_watch(node):
    """Print every /mood/state message until interrupted."""
    node.create_subscription(
        MoodState, '/mood/state', lambda msg: print(format_state(msg), flush=True),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    try:
        while rclpy.ok():                 # a timeout, so SIGTERM is noticed with no traffic
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    return 0


def _try_shutdown():
    """Shut rclpy down unless a signal handler already did."""
    try:
        rclpy.try_shutdown()
    except RCLError:
        pass


def main(args=None):
    """Run the mood tool."""
    args = sys.argv if args is None else args
    parsed = build_parser().parse_args(remove_ros_args(args)[1:])
    rclpy.init(args=args)
    try:
        node = Node('mood_cli_%s' % ('watch' if parsed.command == 'watch' else 'set'),
                    start_parameter_services=False)
        try:
            if parsed.command == 'watch':
                return run_watch(node)
            return run_set(node, parsed)
        finally:
            node.destroy_node()
    finally:
        _try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
