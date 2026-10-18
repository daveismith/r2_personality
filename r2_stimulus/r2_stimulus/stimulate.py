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
The ``stimulate`` command line tool: publish a Stimulus on /stimuli.

    stimulate <kind> [--intensity F] [--confidence F] [--subject S] [--bearing RAD]
                     [--ttl S] [--extra JSON] [--repeat N] [--every S] [--source NAME]

An omitted ``--bearing`` is NaN (unknown), never 0.0, which is straight ahead.
``kind`` is deliberately not checked against affect.yaml. ROS arguments such as
``--ros-args -p use_sim_time:=true`` are honoured as for any node.
"""

import argparse
import json
import math
import os
import sys
import time

NO_LISTENER_WAIT_S = 2.0
FLUSH_S = 0.3


def _build_parser():
    parser = argparse.ArgumentParser(
        prog='stimulate', description='Publish a Stimulus on /stimuli.')
    parser.add_argument('kind', help='stimulus kind, e.g. bump (not validated)')
    parser.add_argument('--intensity', type=float, default=1.0, help='0..1 (default 1)')
    parser.add_argument('--confidence', type=float, default=1.0, help='0..1 (default 1)')
    parser.add_argument('--subject', default='', help='who or what, when known')
    parser.add_argument('--bearing', type=float, default=math.nan,
                        help='radians, +ve left; omitted means unknown (NaN)')
    parser.add_argument('--ttl', type=float, default=0.0,
                        help='seconds; 0 means the affect node default')
    parser.add_argument('--extra', default='', help='JSON object with extra detail')
    parser.add_argument('--repeat', type=int, default=1, help='number of messages')
    parser.add_argument('--every', type=float, default=0.5,
                        help='seconds between repeats (default 0.5)')
    parser.add_argument('--source', default='stimulate_cli', help='source name')
    return parser


def parse_args(argv):
    """Parse and validate argv (without ROS arguments); exit 2 on bad input."""
    parser = _build_parser()
    ns = parser.parse_args(argv)
    if not ns.kind or any(c.isspace() for c in ns.kind):
        parser.error('kind must be non-empty and contain no whitespace')
    for name in ('intensity', 'confidence'):
        if not 0.0 <= getattr(ns, name) <= 1.0:
            parser.error('--%s must be within [0, 1]' % name)
    if ns.repeat < 1:
        parser.error('--repeat must be at least 1')
    if not ns.every >= 0.0:
        parser.error('--every must be >= 0')
    if ns.extra:
        try:
            parsed = json.loads(ns.extra)
        except ValueError as exc:
            parser.error('--extra is not valid JSON: %s' % exc)
        if not isinstance(parsed, dict):
            parser.error('--extra must be a JSON object')
    return ns


def build_message_fields(ns):
    """Return the Stimulus fields (except header.stamp) for parsed args."""
    return {
        'kind': ns.kind,
        'source': ns.source,
        'intensity': float(ns.intensity),
        'confidence': float(ns.confidence),
        'subject': ns.subject,
        'bearing_rad': float(ns.bearing),
        'ttl_s': float(ns.ttl),
        'extra': ns.extra,
    }


def _wait_for_listener(publisher):
    end = time.monotonic() + NO_LISTENER_WAIT_S
    while time.monotonic() < end:
        if publisher.get_subscription_count() > 0:
            return True
        time.sleep(0.05)
    return publisher.get_subscription_count() > 0


def _spin_for(node, seconds):
    import rclpy
    end = time.monotonic() + seconds
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)


def _try_shutdown():
    """Shut rclpy down unless a signal handler already did."""
    import rclpy
    from rclpy._rclpy_pybind11 import RCLError
    try:
        rclpy.try_shutdown()
    except RCLError:
        pass


def main(args=None):
    """Entry point."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from rclpy.utilities import remove_ros_args
    from astromech_msgs.msg import Stimulus

    argv = remove_ros_args(args=sys.argv if args is None else args)[1:]
    ns = parse_args(argv)
    fields = build_message_fields(ns)

    rclpy.init(args=args)
    try:
        node = Node('stimulate_%d' % os.getpid())
        publisher = node.create_publisher(
            Stimulus, '/stimuli', QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        if not _wait_for_listener(publisher):
            print('warning: nobody is listening on /stimuli', file=sys.stderr)
        for i in range(ns.repeat):
            msg = Stimulus()
            msg.header.stamp = node.get_clock().now().to_msg()
            for key, value in fields.items():
                setattr(msg, key, value)
            publisher.publish(msg)
            print('sent %s intensity=%.2f subject=%r bearing=%s' % (
                msg.kind, msg.intensity, msg.subject,
                'unknown' if math.isnan(msg.bearing_rad) else '%.2f' % msg.bearing_rad))
            if i + 1 < ns.repeat:
                _spin_for(node, ns.every)
        _spin_for(node, FLUSH_S)
        node.destroy_node()
    finally:
        _try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
