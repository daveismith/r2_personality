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
The dome device's Magic Panel side as a pure model (no ROS): plan 2.4 and 2.5.

``now`` is passed to every method, as in :mod:`r2_mocks.led_model`. The panel runs on its own
clock: the state a subscriber sees is the last *poll* (``poll()``, once a second on the
device), so a start shows on the state topic up to one poll late, while the reply to
``start`` carries the counter at once. The hold, ``manual`` and the controller's counters are
the controller's own and read live.
"""

import random

from r2_mocks.holds import Hold
from r2_mocks.holds import hold_refusal
from r2_mocks.holds import Manual
from r2_mocks.holds import owner_too_long
from r2_mocks.holds import REFUSE_NOT_FITTED
from r2_mocks.holds import REFUSE_OWNER_TOO_LONG

NEVER = 0xFFFFFFFF
SEQ_NONE = 255
SUB_SEQ_OFF = 254
FLAG_LOOPS = 1
FLAG_RANDOM = 2

LINK_ABSENT = 0
LINK_PRESENT = 1
LINK_UNSUPPORTED = 2

SOURCE_NONE = 0
SOURCE_I2C = 1
SOURCE_GPIO = 3

STATE_IDLE = 0
STATE_RUNNING = 1
STATE_COMPLETE = 2
STATE_STOPPED = 3

STOP_BLANK = 0
STOP_FREEZE = 1
END_BLANK = 1

APPLY_BRIGHTNESS = 1
APPLY_DEFAULT_BRIGHTNESS = 2
APPLY_ORIENTATION = 4
APPLY_LEGACY = 8
APPLY_INPUTS = 16
APPLY_RESUME = 32
APPLY_SAVE = 64
APPLY_FACTORY = 128

RANDOM_PATTERN_S = 10.0         # a random show shows a pattern this long ...
RANDOM_PAUSE_S = 60.0           # ... and is dark for about this long between patterns
RANDOM_PATTERN_IDS = tuple(range(8, 20))
FACTORY = {'brightness': 8, 'default_brightness': 8, 'rotated': False, 'legacy': True,
           'inputs': True, 'resume': False}


REFUSE_TOO_OLD = 'firmware too old'
REFUSE_UNKNOWN = 'unknown sequence'
REFUSE_RANGE = 'out of range'
REFUSE_UNSUPPORTED = 'not supported by this panel'


class _Run:
    def __init__(self, seq_id, now, repeat, end, source, started_by, owner, counter):
        self.seq_id = seq_id
        self.start = now
        self.repeat = repeat
        self.end = end
        self.source = source
        self.started_by = started_by
        self.owner = owner
        self.ours = started_by != 'other'
        self.counter = counter
        self.stopped_at = None


class MagicModel:
    """A Magic Panel behind the dome controller."""

    def __init__(self, catalogue, hold_ms=5000, fitted=True, caps=None, seed=1,
                 manual=None):
        magic = catalogue['magic']
        self.panel = dict(magic['panel'])
        self.panel.update(caps or {})
        self.sequences = {s['id']: dict(s) for s in magic['sequences']}
        if not self.panel['has_names']:
            for seq in self.sequences.values():
                seq['name'] = ''
        self.hold_s = hold_ms / 1000.0
        self.fitted = fitted
        self.manual = manual or Manual()
        self.hold = Hold(self.manual)
        self.live = dict(FACTORY)
        self.saved = dict(self.live)
        self.have_saved = False
        self.run = None
        self.run_counter = 0
        self.rotary_code = 0
        self.takeovers = 0
        self.polls = 0
        self._seed = seed
        self._snapshot = None
        self._poll_time = None
        self._read_at = None         # when the catalogue was read: the first poll

    # --- link and descriptor ------------------------------------------------------------

    def link(self):
        """Return the descriptor's link value."""
        if not self.fitted:
            return LINK_ABSENT
        return LINK_UNSUPPORTED if self.panel['unsupported'] else LINK_PRESENT

    def descriptor(self, now):
        """Return the ``MagicPanelDescriptor`` fields as a dict."""
        desc = dict(self.panel)
        desc['link'] = self.link()
        desc['catalogue_age_ms'] = (
            int((now - (self._read_at or now)) * 1000) if self.link() == LINK_PRESENT
            else NEVER)
        desc['seq_count'] = len(self.sequences)
        desc['cached'] = len(self.sequences)
        return desc

    def list_catalogue(self, now):
        """Return ``(descriptor, sequences)``; the sequences are empty unless the link is up."""
        sequences = list(self.sequences.values()) if self.link() == LINK_PRESENT else []
        return self.descriptor(now), sequences

    def set_fitted(self, fitted):
        """Plug the panel in or out (``magic_panel fit on|off``)."""
        self.fitted = bool(fitted)
        self._snapshot = None

    def sequence_id(self, token):
        """Return the id a console token names (a number or a name, any case), or None."""
        if token.isdigit():
            return int(token)
        for seq in self.sequences.values():
            if seq['name'] and seq['name'].lower() == token.lower():
                return seq['id']
        return None

    # --- holds --------------------------------------------------------------------------

    def _hold(self, source, now):
        self.hold.touch(source, now, self.hold_s)

    def _gate(self, now, owner, priority, source):
        """Return the refusal that comes before the request's own checks, or ''."""
        if owner_too_long(owner):
            return REFUSE_OWNER_TOO_LONG
        if self.link() == LINK_UNSUPPORTED:
            return REFUSE_TOO_OLD
        if self.link() != LINK_PRESENT:
            return REFUSE_NOT_FITTED
        return hold_refusal(self.hold.holder(now), source, priority)

    # --- start and stop -----------------------------------------------------------------

    def start(self, now, sequence_id, repeat=1, end=0, owner='', priority=0, source='ros'):
        """
        Start a sequence, replacing whatever runs.

        Return ``{'accepted', 'reason', 'confirmed', 'run_counter'}``; the counter moves by
        one on every accepted start and wraps at 256.
        """
        refused = {'accepted': False, 'confirmed': False, 'run_counter': 0}
        reason = self._gate(now, owner, priority, source)
        if not reason and sequence_id not in self.sequences:
            reason = REFUSE_UNKNOWN
        if not reason and end > END_BLANK:
            reason = REFUSE_RANGE
        if not reason and repeat != 1 and not self.panel['has_repeat']:
            reason = REFUSE_UNSUPPORTED
        if reason:
            return dict(refused, reason=reason)
        self._begin(now, sequence_id, repeat, end, source, owner)
        self._hold(source, now)
        return {'accepted': True, 'reason': '', 'confirmed': True,
                'run_counter': self.run_counter}

    def _begin(self, now, sequence_id, repeat, end, source, owner):
        self.run_counter = (self.run_counter + 1) & 0xFF
        gpio = source == 'gpio'
        self.run = _Run(sequence_id, now, repeat, end, SOURCE_GPIO if gpio else SOURCE_I2C,
                        'other' if gpio else source, owner if source == 'ros' else '',
                        self.run_counter)

    def stop(self, now, mode=STOP_BLANK, priority=0, source='ros'):
        """Stop whatever runs (blank or freeze); return ``{'accepted', 'reason'}``."""
        reason = self._gate(now, '', priority, source)
        if not reason and mode > STOP_FREEZE:
            reason = REFUSE_RANGE
        if reason:
            return {'accepted': False, 'reason': reason}
        if self.run is not None and self._compute(now)['state'] == STATE_RUNNING:
            self.run.stopped_at = now
        self._hold(source, now)
        return {'accepted': True, 'reason': ''}

    def rotary(self, now, code):
        """
        Turn the panel's own rotary switch to ``code``: a GPIO run we did not start.

        Return ``''`` or the reason nothing happened. The run counter moves, as on any start
        from any source, and ``takeovers`` follows it.
        """
        if self.link() != LINK_PRESENT:
            return REFUSE_NOT_FITTED
        if code not in self.sequences:
            return REFUSE_UNKNOWN
        self.rotary_code = code
        self.takeovers += 1
        self._begin(now, code, 1, 0, 'gpio', '')
        return ''

    # --- configure ----------------------------------------------------------------------

    def configure(self, apply, brightness=0, default_brightness=0, rotated=False, legacy=False,
                  inputs=False, resume=False):
        """Change settings, all checked before any applies; return a dict like the response."""
        result = {'accepted': False, 'reason': '', 'saved': False}
        reason = ''
        if self.link() != LINK_PRESENT:
            reason = REFUSE_NOT_FITTED
        elif apply & APPLY_FACTORY:
            self.live = dict(FACTORY)
            self.saved, self.have_saved = dict(FACTORY), True
            return dict(result, accepted=True, saved=True, result=dict(self.live))
        else:
            reason = self._configure_problem(apply, brightness, default_brightness)
        if reason:
            return dict(result, reason=reason, result=dict(self.live))
        wanted = {APPLY_BRIGHTNESS: ('brightness', brightness),
                  APPLY_DEFAULT_BRIGHTNESS: ('default_brightness', default_brightness),
                  APPLY_ORIENTATION: ('rotated', rotated), APPLY_LEGACY: ('legacy', legacy),
                  APPLY_INPUTS: ('inputs', inputs), APPLY_RESUME: ('resume', resume)}
        for bit, (key, value) in wanted.items():
            if apply & bit:
                self.live[key] = value
        saved = bool(apply & APPLY_SAVE)
        if saved:
            self.saved, self.have_saved = dict(self.live), True
        return dict(result, accepted=True, saved=saved, result=dict(self.live))

    def _configure_problem(self, apply, brightness, default_brightness):
        needs = ((APPLY_BRIGHTNESS, 'has_brightness'),
                 (APPLY_DEFAULT_BRIGHTNESS, 'has_eeprom_config'),
                 (APPLY_ORIENTATION, 'has_orientation'), (APPLY_LEGACY, 'has_legacy'),
                 (APPLY_SAVE, 'has_eeprom_config'))
        if any(apply & bit and not self.panel[cap] for bit, cap in needs):
            return REFUSE_UNSUPPORTED
        if apply & APPLY_BRIGHTNESS and not 0 <= brightness <= 15:
            return REFUSE_RANGE
        if apply & APPLY_DEFAULT_BRIGHTNESS and not 0 <= default_brightness <= 15:
            return REFUSE_RANGE
        return ''

    # --- state --------------------------------------------------------------------------

    def _compute(self, now):
        """Return the run-dependent fields as the panel would report them at ``now``."""
        out = self._idle()
        run = self.run
        if run is None:
            return out
        return self._running(out, run, now)

    def _idle(self):
        return {
            'sequence_id': SEQ_NONE, 'source': SOURCE_NONE, 'state': STATE_IDLE,
            'sub_seq': 0, 'in_pause': False, 'iteration': 0, 'repeat': 0, 'elapsed_ms': 0,
            'remaining_known': False, 'remaining_ms': 0, 'ours': False, 'end_known': False,
            'end': 0, 'run_counter': self.run_counter, 'started_by': '', 'owner': '',
            'takeovers': self.takeovers}

    def _running(self, out, run, now):
        seq = self.sequences[run.seq_id]
        length = seq['length_ms']
        loops = bool(seq['flags'] & FLAG_LOOPS)
        total = None if loops or run.repeat == 0 else length * run.repeat
        elapsed = int(((run.stopped_at if run.stopped_at is not None else now)
                       - run.start) * 1000)
        state = STATE_STOPPED if run.stopped_at is not None else STATE_RUNNING
        if state == STATE_RUNNING and total is not None and elapsed >= total:
            state, elapsed = STATE_COMPLETE, total
        out.update(sequence_id=run.seq_id, source=run.source, state=state, repeat=run.repeat,
                   elapsed_ms=elapsed, ours=run.ours, end_known=run.ours, end=run.end,
                   started_by=run.started_by, owner=run.owner)
        if loops:
            out['iteration'] = 0
        elif length:
            out['iteration'] = min(255, elapsed // length if state != STATE_COMPLETE
                                   else run.repeat)
        if state == STATE_RUNNING and total is not None:
            out.update(remaining_known=True, remaining_ms=total - elapsed)
        if state == STATE_RUNNING and seq['flags'] & FLAG_RANDOM and loops:
            self._random_show(out, run, elapsed / 1000.0)
        return out

    def _random_show(self, out, run, elapsed_s):
        cycle, phase = divmod(elapsed_s, RANDOM_PATTERN_S + RANDOM_PAUSE_S)
        out['iteration'] = min(255, int(cycle))
        if phase >= RANDOM_PATTERN_S:
            out.update(sub_seq=SUB_SEQ_OFF, in_pause=True)
        else:
            rng = random.Random(self._seed * 7919 + run.counter * 131 + int(cycle))
            out['sub_seq'] = rng.choice(RANDOM_PATTERN_IDS)

    def poll(self, now):
        """Read the panel (the device does this once a second); state advances only here."""
        self.polls += 1
        if self._read_at is None:
            self._read_at = now
        self._snapshot = self._compute(now) if self.link() == LINK_PRESENT else None
        self._poll_time = now

    def state(self, now):
        """Return the ``MagicPanelState`` fields (without the header) as a dict."""
        polled = self._poll_time
        snap = self._snapshot
        link = self.link()
        base = self._idle() if snap is None else snap
        have = snap is not None and link == LINK_PRESENT
        desc = self.panel
        state = dict(base)
        state.update(
            link=link, unsupported=desc['unsupported'], fw_major=desc['fw_major'],
            fw_minor=desc['fw_minor'], fw_patch=desc['fw_patch'], proto_major=desc['proto_major'],
            proto_minor=desc['proto_minor'], have_status=have,
            read_age_ms=int((now - polled) * 1000) if have else NEVER, last_read_failed=False,
            rotary_code=self.rotary_code, last_error=0, last_error_name='NONE', error_count=0,
            live=dict(self.live), saved=dict(self.saved), have_saved=self.have_saved,
            polls=self.polls, read_errors=0, interrogations=1, rejections=0, refusal_seq=0,
            refusal_confirmed=False, refusal_reason='', refusal_op='', refusal_age_ms=0,
            manual=self.manual.on, held_by=self.hold.holder(now),
            hold_remaining_ms=self.hold.remaining_ms(now))
        return state
