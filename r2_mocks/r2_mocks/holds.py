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
The device's hold policy, shared by the LED and Magic Panel models (plan 2.4).

A ``console``, ``ui`` or ``text_topic`` write holds what it touched for ``hold_ms``; the
sticky ``manual`` setting holds everything until cleared. A ROS request at priority 1..199
is refused while the target is held; priority 0 and 200 or more never are. The sticky flag
is one :class:`Manual` object owned by the caller and handed to both models.
"""

import math

HOLD_FOREVER = 0xFFFFFFFF
PRIORITY_OVERRIDE = 200
OWNER_MAX_BYTES = 31
HOLDING_SOURCES = ('console', 'ui', 'text_topic')
REFUSE_OWNER_TOO_LONG = 'owner too long'
REFUSE_NOT_FITTED = 'not fitted'


def held_reason(held_by):
    """Return the refusal text for a hold by ``held_by``."""
    if held_by == 'manual':
        return 'held: manual'
    return 'held by ' + held_by.replace('_', ' ')


def owner_too_long(owner):
    """Return True if ``owner`` does not fit the device's 31-byte buffer."""
    return len(owner.encode()) > OWNER_MAX_BYTES


def hold_refusal(holder, source, priority):
    """Return the refusal for a request from ``source`` at ``priority`` against ``holder``."""
    if holder and source == 'ros' and 1 <= priority < PRIORITY_OVERRIDE:
        return held_reason(holder)
    return ''


class Manual:
    """The sticky manual setting (``leds owner manual|auto``) and the holds it overrides."""

    def __init__(self):
        self.on = False
        self._holds = []

    def set_on(self, on):
        """Set it; clearing it (``owner auto``) drops every timed hold as well."""
        self.on = bool(on)
        if not self.on:
            for hold in self._holds:
                hold.clear()


class Hold:
    """Who holds one thing (a display, the panel) and until when."""

    def __init__(self, manual):
        self.manual = manual
        self.held_by = ''
        self.until = 0.0
        manual._holds.append(self)

    def holder(self, now):
        """Return console, ui, text_topic, manual or '' for no hold."""
        if self.manual.on:
            return 'manual'
        return self.held_by if self.held_by and self.until > now else ''

    def remaining_ms(self, now):
        """Return the milliseconds left; HOLD_FOREVER under manual."""
        if self.manual.on:
            return HOLD_FOREVER
        return int(math.ceil((self.until - now) * 1000.0)) if self.holder(now) else 0

    def touch(self, source, now, hold_s):
        """Renew the hold if ``source`` is one that holds."""
        if source in HOLDING_SOURCES:
            self.held_by = source
            self.until = now + hold_s

    def clear(self):
        """Drop the timed hold."""
        self.held_by = ''
        self.until = 0.0
