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
Sound renderer: a core :class:`~r2_expression.core.Action` to ``sound_player``'s request.

Same contract as :mod:`r2_expression.led_renderer`. The core has already judged the name or
pool against the player's catalogue; this refuses again, with the player's own strings, what
can be seen from the request alone (phase 3 plan 5.4): neither a name nor a pool, or both
(``unknown name``), a name over 63 bytes, an owner over 31. ``StopSound`` takes an owner and
no priority; every other request carries a priority of 1-255, never 0.
"""

from astromech_msgs.srv import ListSounds
from astromech_msgs.srv import PlaySound
from astromech_msgs.srv import StopSound
from astromech_msgs.srv import Vocalize
from r2_expression.core import CALL_PLAY
from r2_expression.core import CALL_SOUND_STOP
from r2_expression.core import CALL_VOCALIZE
from r2_expression.render_common import build
from r2_expression.render_common import check_attribution
from r2_expression.render_common import MAX_NAME_BYTES
from r2_expression.render_common import REFUSE_NAME
from r2_expression.render_common import REFUSE_UNKNOWN_ACTION
from r2_expression.render_common import RenderRefusal
from r2_expression.render_common import too_long

LIST = ('list', ListSounds)
SERVICES = {
    CALL_PLAY: ('play', PlaySound),
    CALL_VOCALIZE: ('vocalize', Vocalize),
    CALL_SOUND_STOP: ('stop', StopSound),
}
UNKNOWN_NAME = 'unknown name'


def render(action):
    """Return the service request message for ``action``, or raise ``RenderRefusal``."""
    entry = SERVICES.get(action.call)
    if entry is None:
        raise RenderRefusal(REFUSE_UNKNOWN_ACTION)
    fields = action.request
    if action.call == CALL_PLAY:
        name, pool = fields.get('name', ''), fields.get('pool', '')
        if bool(name) == bool(pool):             # a clip or a pool, exactly one
            raise RenderRefusal(UNKNOWN_NAME)
        if too_long(name or pool, MAX_NAME_BYTES):
            raise RenderRefusal(REFUSE_NAME)
    return check_attribution(build(entry[1].Request, fields))
