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
LED renderer: a core :class:`~r2_expression.core.Action` to the device's LED request.

Pure apart from the message types. :func:`render` returns the request message for the
action's call, or raises :class:`~r2_expression.render_common.RenderRefusal` with the device's
own string when the request cannot be right (plan 5.2): text on a PSI, more lines than the
surface has displays, a line of 64 characters or more, a name over 63 bytes, an owner over 31.
Nothing here sets hue or saves to flash (rules 4 and 9).
"""

from astromech_msgs.srv import ConfigureLedDisplay
from astromech_msgs.srv import ListLeds
from astromech_msgs.srv import RunLedEffect
from astromech_msgs.srv import SayLedText
from astromech_msgs.srv import StopLedEffect
from r2_expression.core import APPLY_BRIGHTNESS
from r2_expression.core import APPLY_DRIFT
from r2_expression.core import APPLY_MODE
from r2_expression.core import CALL_CONFIGURE
from r2_expression.core import CALL_RUN
from r2_expression.core import CALL_SAY
from r2_expression.core import CALL_STOP
from r2_expression.render_common import build
from r2_expression.render_common import check_attribution
from r2_expression.render_common import MAX_LINE_BYTES
from r2_expression.render_common import MAX_NAME_BYTES
from r2_expression.render_common import REFUSE_NAME
from r2_expression.render_common import REFUSE_TEXT
from r2_expression.render_common import REFUSE_UNKNOWN_ACTION
from r2_expression.render_common import RenderRefusal
from r2_expression.render_common import too_long

# Service names under the device node, and the one `list` that builds the catalogue.
LIST = ('leds/list', ListLeds)
SERVICES = {
    CALL_RUN: ('leds/run', RunLedEffect),
    CALL_SAY: ('leds/say', SayLedText),
    CALL_STOP: ('leds/stop', StopLedEffect),
    CALL_CONFIGURE: ('leds/configure', ConfigureLedDisplay),
}
PSI_SURFACES = ('fpsi', 'rpsi')
ALLOWED_APPLY = APPLY_MODE | APPLY_BRIGHTNESS | APPLY_DRIFT     # never HUE, never SAVE
NO_TEXT_MODE = 'display has no text mode'
TOO_MANY_LINES = 'more lines than the surface has displays'
NO_TEXT = 'no text'


def render(action):
    """Return the service request message for ``action``, or raise ``RenderRefusal``."""
    entry = SERVICES.get(action.call)
    if entry is None:
        raise RenderRefusal(REFUSE_UNKNOWN_ACTION)
    fields = action.request
    for key in ('effect', 'surface', 'display', 'drift'):
        if too_long(str(fields.get(key, '')), MAX_NAME_BYTES):
            raise RenderRefusal(REFUSE_NAME)
    if action.call == CALL_SAY:
        _check_text(fields)
    elif action.call == CALL_CONFIGURE and fields.get('apply', 0) & ~ALLOWED_APPLY:
        raise ValueError('the personality sets only mode, brightness and drift (rules 4, 9)')
    return check_attribution(build(entry[1].Request, fields))


def _check_text(fields):
    surface, lines = fields['surface'], list(fields.get('lines', ()))
    if surface in PSI_SURFACES:
        raise RenderRefusal(NO_TEXT_MODE)
    if not lines:
        raise RenderRefusal(NO_TEXT)
    if len(lines) > (2 if surface == 'fld' else 1):
        raise RenderRefusal(TOO_MANY_LINES)
    if any(too_long(line, MAX_LINE_BYTES) for line in lines):
        raise RenderRefusal(REFUSE_TEXT)
