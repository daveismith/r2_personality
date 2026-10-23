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
Dome renderer: a core ``CALL_VELOCITY`` :class:`~r2_expression.core.Action` to the message.

Pure apart from the message type. :func:`render` returns the ``geometry_msgs/TwistStamped``
for ``/dome/cmd/personality``: ``twist.angular.z`` is ``velocity_rad_s`` (positive = the
positive joint direction), everything else 0; the node sets ``header.stamp``. The intent's
params became a :class:`~r2_expression.motion.Move` in the core already (bearing
conversion, named profiles), so all that is left here is the message and the checks.

No attribution. A Twist carries no owner and no priority, so ``check_attribution`` has
nothing to check: the converter knows its inputs by topic, not by owner, and its
``DomeCommandStatus`` (``source``, the velocity it sent, ``capped``) is how the arbiter
learns who drives the dome (``foreign`` on ``joystick``/``foxglove``), where the LED
device's ``held_by`` would tell it. What can be checked a second time is the velocity:
finite and 0 or in [``min_speed``, cap] (plan rule 4); the core guarantees both, so a
value outside is a bug (``ValueError``), not a refusal.
"""

import math

from geometry_msgs.msg import TwistStamped
from r2_expression.core import CALL_VELOCITY
from r2_expression.render_common import REFUSE_UNKNOWN_ACTION
from r2_expression.render_common import RenderRefusal

MESSAGE = TwistStamped
EPS = 1e-9                   # float slack on the bounds


def render(action, min_speed=0.0, max_speed=math.inf):
    """
    Return the ``TwistStamped`` for ``action``, or raise ``RenderRefusal``/``ValueError``.

    ``min_speed`` and ``max_speed`` are ``defaults.dome.min_speed_rad_s`` and
    ``max_speed_rad_s``.
    """
    if action.call != CALL_VELOCITY:
        raise RenderRefusal(REFUSE_UNKNOWN_ACTION)
    velocity = action.request.get('velocity_rad_s')
    if isinstance(velocity, bool) or not isinstance(velocity, (int, float)) or \
            not math.isfinite(velocity):
        raise ValueError('a dome velocity must be a finite number, got %r' % (velocity,))
    if velocity != 0.0 and not min_speed - EPS <= abs(velocity) <= max_speed + EPS:
        raise ValueError('a dome velocity must be 0 or %g to %g rad/s (rule 4), got %g'
                         % (min_speed, max_speed, velocity))
    msg = TwistStamped()
    msg.twist.angular.z = float(velocity)
    return msg
