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
What the LED and Magic Panel renderers share: building a request and the local refusals.

The renderers turn a core :class:`~r2_expression.core.Action` into the service request
message and refuse, before any device call, what can be checked locally, with the device's
own strings. They also check a second time what the core already guarantees (rules 6 and 9
of the phase 1 plan): every request that has ``priority`` carries 1-255, never 0, and every
request that has ``owner`` carries one.
"""

from r2_expression.core import MAX_OWNER_BYTES

MAX_NAME_BYTES = 63          # the device never answers a name of 96 bytes or more (plan 2.1)
MAX_LINE_BYTES = 63          # bytes: the device's strlen counts 64+ as `text too long`

REFUSE_OWNER = 'owner too long'
REFUSE_NAME = 'name too long'
REFUSE_TEXT = 'text too long'
REFUSE_UNKNOWN_ACTION = 'unknown action'


class RenderRefusal(Exception):
    """A request the renderer will not send; ``reason`` is the device's string for it."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


def too_long(text, limit):
    """Return True when ``text`` is longer than ``limit`` bytes in UTF-8."""
    return len(text.encode('utf-8', 'surrogatepass')) > limit


def build(request_type, fields):
    """Return a ``request_type`` with ``fields`` set; a name the service lacks is a bug."""
    msg = request_type()
    known = msg.get_fields_and_field_types()
    for name, value in fields.items():
        if name not in known:
            raise ValueError('%s has no field %r' % (request_type.__qualname__, name))
        setattr(msg, name, value)
    return msg


def check_attribution(msg):
    """Raise unless a request's ``priority`` is 1-255 and its ``owner`` is set (and short)."""
    names = msg.get_fields_and_field_types()
    if 'owner' in names:
        if not msg.owner:
            raise ValueError('a device request needs its owner (rule 6)')
        if too_long(msg.owner, MAX_OWNER_BYTES):
            raise RenderRefusal(REFUSE_OWNER)
    if 'priority' in names and not 1 <= msg.priority <= 255:
        raise ValueError('a device request needs priority 1-255, never 0 (rule 6): got %r'
                         % msg.priority)
    return msg
