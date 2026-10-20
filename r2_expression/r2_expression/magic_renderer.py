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
Magic Panel renderer: a core :class:`~r2_expression.core.Action` to the panel's request.

Same contract as :mod:`r2_expression.led_renderer`. ``ConfigureMagicPanel`` takes no priority
and no owner, so the arbiter gates it on the panel's ``held_by`` (the core does); this
renderer sets only ``APPLY_BRIGHTNESS`` on it, never a save or a factory reset (rule 4).
"""

from astromech_msgs.srv import ConfigureMagicPanel
from astromech_msgs.srv import ListMagicPanel
from astromech_msgs.srv import StartMagicSequence
from astromech_msgs.srv import StopMagicSequence
from r2_expression.core import CALL_MAGIC_BRIGHTNESS
from r2_expression.core import CALL_MAGIC_STOP
from r2_expression.core import CALL_START
from r2_expression.core import MAGIC_APPLY_BRIGHTNESS
from r2_expression.render_common import build
from r2_expression.render_common import check_attribution
from r2_expression.render_common import REFUSE_UNKNOWN_ACTION
from r2_expression.render_common import RenderRefusal

LIST = ('magic/list', ListMagicPanel)
SERVICES = {
    CALL_START: ('magic/start', StartMagicSequence),
    CALL_MAGIC_STOP: ('magic/stop', StopMagicSequence),
    CALL_MAGIC_BRIGHTNESS: ('magic/configure', ConfigureMagicPanel),
}
MAX_BRIGHTNESS = 15


def render(action):
    """Return the service request message for ``action``, or raise ``RenderRefusal``."""
    entry = SERVICES.get(action.call)
    if entry is None:
        raise RenderRefusal(REFUSE_UNKNOWN_ACTION)
    fields = action.request
    if action.call == CALL_MAGIC_BRIGHTNESS:
        if fields.get('apply') != MAGIC_APPLY_BRIGHTNESS:
            raise ValueError('the personality sets only the panel brightness (rule 4)')
        if not 0 <= fields.get('brightness', 0) <= MAX_BRIGHTNESS:
            raise RenderRefusal('out of range')
    return check_attribution(build(entry[1].Request, fields))
