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

"""Tests for the arbiter core, named for the rules of plan 4.2 (no ROS, ``now`` advanced)."""

import dataclasses
import math

import pytest

from fixtures import base_data, cfg, dome_cfg, dome_data, FixedRng, held, running  # noqa: I100
from fixtures import LED_CAT, led_snap, MAGIC_CAT, magic_snap, MIN_SPEED, OWNER, PlantDouble
from fixtures import playing, SOUND_CAT, sound_cfg, SOUND_CLIPS, sound_data, sound_list
from fixtures import sound_snap
from r2_expression import motion
from r2_expression.config import parse_config
from r2_expression.core import (Action, APPLY_BRIGHTNESS, APPLY_DRIFT, APPLY_HUE, APPLY_MODE,
                                Arbiter, CALL_CONFIGURE, CALL_MAGIC_BRIGHTNESS,
                                CALL_MAGIC_STOP, CALL_PLAY, CALL_RUN, CALL_SAY,
                                CALL_SOUND_STOP, CALL_START, CALL_STOP, CALL_VELOCITY,
                                CALL_VOCALIZE, channel_of, DOME_CHANNELS, held_by, Intent,
                                LED_CHANNELS, SOUND_CHANNELS, Status)
from r2_expression.core import (STATE_CLAIMED, STATE_DARK, STATE_FOREIGN, STATE_IDLE,
                                STATE_NAMES, STATE_OFFLINE, STATE_RUNNING, STATE_STALE,
                                STATE_SUPPRESSED)
from r2_expression.palette import sound_catalogue

ALL_ACTIONS = []


def collect(actions):
    """Keep every action any test produced, for the rule 6 and rule 9 sweeps."""
    actions = list(actions)
    ALL_ACTIONS.extend(actions)
    return actions


def led(config=None, rng=None, mood='calm', strength=1.0, snap=None):
    a = Arbiter(config or cfg(), LED_CHANNELS, rng=rng or FixedRng(0.0))
    collect(a.set_catalogue(LED_CAT, 0.0))
    collect(a.led_state(snap or led_snap(), 0.0))
    if mood:
        collect(a.mood(mood, strength, 0.0))
    return a


def magic(config=None, rng=None, mood='calm', snap=None):
    a = Arbiter(config or cfg(), ('magic',), rng=rng or FixedRng(0.0))
    collect(a.set_catalogue(MAGIC_CAT, 0.0))
    collect(a.magic_state(snap or magic_snap(), 0.0))
    if mood:
        collect(a.mood(mood, 1.0, 0.0))
    return a


def step(a, t, snap=None, mood='calm'):
    """
    Live up to ``t`` as a node would; return every action on the way.

    Each second, and at ``t``: a state message, a mood message (unless ``mood`` is
    None) and a tick, so nothing goes stale between the moments a test looks at.
    """
    start = getattr(a, 'test_t', 0.0)
    times = [start + k for k in range(1, int(t - start) + 1) if start + k < t] + [t]
    out = []
    for now in times:
        if a.kind == 'led':
            out += a.led_state(snap or led_snap(), now)
        else:
            out += a.magic_state(snap or magic_snap(), now)
        if mood:
            out += a.mood(mood, 1.0, now)
        out += a.tick(now)
    a.test_t = t
    return collect(out)


def intent(a, t, channel='fld', source='idle', priority=10, action='effect', params=None,
           duration_ms=0, interruptible=True, intent_id=''):
    if params is None:
        params = {'name': 'fire'} if action == 'effect' else {}
    result = a.intent(Intent(channel, source, priority, action, params, duration_ms,
                             interruptible, intent_id), t)
    collect(result.actions)
    return result


def quiet():
    """Return the test config without flourishes."""
    data = base_data()
    for mood in data['moods'].values():
        del mood['flourish']
    return parse_config(data)


def calls(actions):
    return [(x.call, x.target) for x in actions]


def accept_all(a, actions, t):
    for x in actions:
        assert a.device_reply(x, True, '', t) == ''


# --- channels ------------------------------------------------------------------------------

def test_channel_of_maps_sub_channels_and_the_psi_fan_out():
    assert channel_of('fld').surfaces == ('fld',)
    assert channel_of('fld').displays == ('tfld', 'bfld')
    assert channel_of('tfld').parent == 'fld' and channel_of('tfld').palette_channel == 'fld'
    assert channel_of('psi').surfaces == ('fpsi', 'rpsi')
    assert channel_of('fpsi').name == channel_of('rpsi').name == 'psi'
    assert channel_of('magic').kind == 'magic'
    assert channel_of('dome').kind == 'dome' and channel_of('dome').surfaces == ('dome',)
    assert not channel_of('dome').overlaps(channel_of('fld'))
    assert channel_of('head') is None
    assert channel_of('tfld').overlaps(channel_of('fld'))
    assert not channel_of('tfld').overlaps(channel_of('bfld'))


def test_arbiter_serves_one_channel_group():
    with pytest.raises(ValueError):
        Arbiter(cfg(), ('fld', 'magic'))
    with pytest.raises(ValueError):
        Arbiter(cfg(), ('fld', 'dome'))
    with pytest.raises(ValueError):
        Arbiter(cfg(), ('magic', 'dome'))
    with pytest.raises(ValueError):
        Arbiter(cfg(), ('fld',), owner='x' * 32)


# --- rule 6 and rule 9: every request ------------------------------------------------------

def test_rule6_a_request_built_with_priority_zero_raises():
    with pytest.raises(ValueError, match='never 0'):
        Action(CALL_RUN, 'fld', 'fld', {'surface': 'fld', 'priority': 0}, OWNER, 0)
    with pytest.raises(ValueError, match='never 0'):
        Action(CALL_MAGIC_BRIGHTNESS, 'magic', 'magic', {'apply': 1, 'brightness': 3}, OWNER, 0)
    with pytest.raises(ValueError):
        Action(CALL_RUN, 'fld', 'fld', {'surface': 'fld', 'priority': 0}, OWNER, 10)
    with pytest.raises(ValueError):
        Action(CALL_RUN, 'fld', 'fld', {'surface': 'fld', 'priority': 10}, '', 10)


def test_rule9_a_request_carrying_hue_raises():
    with pytest.raises(ValueError, match='rule 9'):
        Action(CALL_CONFIGURE, 'fld', 'tfld', {'display': 'tfld', 'apply': APPLY_HUE,
                                               'priority': 10}, OWNER, 10)
    with pytest.raises(ValueError, match='rule 9'):
        Action(CALL_CONFIGURE, 'fld', 'tfld', {'display': 'tfld', 'hue': 3, 'priority': 10},
               OWNER, 10)


def test_rule9_an_intent_asking_for_hue_is_dropped():
    a = led()
    r = intent(a, 0.5, source='manual', priority=199, action='configure',
               params={'hue': 64, 'brightness': 50})
    assert not r.accepted and 'hue' in r.reason and r.actions == ()


def test_ladder_drops_a_source_above_its_slot_priority_zero_and_unknowns():
    a = led()
    for kwargs, fragment in [({'source': 'idle', 'priority': 11}, 'above the idle slot'),
                             ({'source': 'behavior', 'priority': 150}, 'above the behavior'),
                             ({'source': 'manual', 'priority': 201}, 'above the manual'),
                             ({'source': 'idle', 'priority': 0}, 'priority 0'),
                             ({'source': 'console', 'priority': 10}, 'unknown source'),
                             ({'action': 'dance'}, 'unknown action'),
                             ({'action': 'sequence'}, 'unknown action')]:
        r = intent(a, 0.5, **kwargs)
        assert not r.accepted and fragment in r.reason, (kwargs, r.reason)
    assert a.status('fld', 0.5).dropped == 7
    assert intent(a, 0.5, source='safety', priority=255).accepted


def test_intent_for_another_arbiters_channel_is_not_counted():
    a = led()
    r = intent(a, 0.5, channel='magic', action='sequence', params={'id': 3})
    assert (r.accepted, r.reason) == (False, 'not my channel')
    assert all(s.dropped == 0 for s in a.statuses(0.5))


# --- claims --------------------------------------------------------------------------------

def test_claim_accepted_refused_renewed_released_expired():
    a = led()
    r = a.claim('rld', 'console', 200, 30.0, 1.0)
    assert r.accepted and r.reason == '' and r.status.state == STATE_CLAIMED
    assert (r.status.owner, r.status.priority, r.status.lease_remaining_s) == ('console', 200,
                                                                               30.0)
    r = a.claim('rld', 'foxglove', 200, 10.0, 2.0)
    assert not r.accepted and "claimed by 'console' at priority 200" in r.reason
    r = a.claim('rld', 'test', 100, 10.0, 2.0)
    assert not r.accepted and 'console' in r.reason
    r = a.claim('rld', 'console', 200, 30.0, 10.0)               # heartbeat
    assert r.accepted and r.status.lease_remaining_s == 30.0
    step(a, 39.0)
    assert a.status('rld', 39.0).state == STATE_CLAIMED
    step(a, 40.0)                                                 # expiry releases
    assert a.status('rld', 40.0).state == STATE_IDLE
    a.claim('rld', 'console', 200, 30.0, 41.0)
    r = a.claim('rld', 'console', 200, 0.0, 42.0)                 # release
    assert r.accepted and r.status.state == STATE_IDLE and r.status.owner == ''


def test_release_by_an_owner_without_a_claim_is_refused_naming_the_channel():
    a = led()
    a.claim('rld', 'console', 200, 30.0, 1.0)
    r = a.claim('rld', 'express', 199, 0.0, 2.0)
    assert (r.accepted, r.reason) == (False, "no claim by 'express' on rld")
    assert r.status.owner == 'console' and r.status.state == STATE_CLAIMED   # the real holder
    r = a.claim('psi', 'express', 199, 0.0, 2.0)
    assert (r.accepted, r.reason) == (False, "no claim by 'express' on psi")
    assert a.claim('rld', 'console', 200, 0.0, 3.0).accepted
    r = a.claim('rld', 'console', 200, 0.0, 4.0)                  # twice: nothing left to release
    assert not r.accepted and 'no claim' in r.reason


def test_release_needs_no_priority():
    a = led()
    a.claim('fld', 'console', 200, 30.0, 1.0)
    r = a.claim('fld', 'console', 0, 0.0, 2.0)
    assert r.accepted and r.status.state == STATE_IDLE


def test_claim_lease_is_capped_at_max_lease_s():
    a = led()
    r = a.claim('fld', 'console', 200, 600.0, 1.0)
    assert r.status.lease_remaining_s == 60.0


def test_claim_at_higher_priority_layers_over_a_lower_one():
    a = led()
    a.claim('rld', 'behaviour', 100, 30.0, 1.0)
    r = a.claim('rld', 'console', 200, 5.0, 2.0)
    assert r.accepted and r.status.owner == 'console'
    step(a, 7.0)
    st = a.status('rld', 7.0)
    assert (st.state, st.owner, st.priority) == (STATE_CLAIMED, 'behaviour', 100)


def test_claim_refuses_bad_requests_and_other_channels():
    a = led()
    r = a.claim('magic', 'console', 200, 5.0, 1.0)
    assert (r.accepted, r.reason, r.status) == (False, 'not my channel', None)
    assert 'priority' in a.claim('fld', 'console', 0, 5.0, 1.0).reason
    assert 'owner' in a.claim('fld', '', 200, 5.0, 1.0).reason
    assert 'lease_s' in a.claim('fld', 'console', 200, -1.0, 1.0).reason
    assert 'lease_s' in a.claim('fld', 'console', 200, float('nan'), 1.0).reason


def test_claim_owner_manual_is_reserved_for_the_auto_claim():
    a = led()
    r = a.claim('fld', 'manual', 150, 60.0, 1.0)
    assert not r.accepted and 'reserved' in r.reason


def test_claim_preempts_and_stops_the_lower_run():
    a = led()
    r = intent(a, 1.0, channel='psi', action='clip', params={'name': 'pulse'},
               duration_ms=3000)
    assert calls(r.actions) == [(CALL_RUN, 'fpsi'), (CALL_RUN, 'rpsi')]
    accept_all(a, r.actions, 1.0)
    c = a.claim('psi', 'console', 200, 30.0, 1.5)
    assert c.accepted
    assert calls(c.actions) == [(CALL_STOP, 'fpsi'), (CALL_STOP, 'rpsi')]
    assert all(x.why == 'preempt' and x.priority == 10 for x in c.actions)
    assert c.status.state == STATE_CLAIMED and c.status.active_intent_id == ''
    r = intent(a, 2.0, channel='psi', action='clip', params={'name': 'flash'})
    assert not r.accepted and 'claimed by' in r.reason
    assert a.status('psi', 2.0).dropped == 1


def test_claim_does_not_stop_a_higher_run():
    a = led()
    r = intent(a, 1.0, channel='rld', source='manual', priority=200, duration_ms=10000)
    c = a.claim('rld', 'test', 150, 30.0, 1.5)
    assert not c.accepted          # the manual auto-claim at 200 holds the channel
    assert r.accepted and a.status('rld', 1.5).state == STATE_RUNNING


def test_tfld_claim_blocks_fld_and_is_reported_on_fld():
    a = led()
    assert a.claim('tfld', 'console', 200, 30.0, 1.0).accepted
    r = intent(a, 1.5, channel='fld')
    assert not r.accepted and "claimed by 'console'" in r.reason
    st = a.status('fld', 1.5)
    assert (st.state, st.owner, st.priority, st.dropped) == (STATE_CLAIMED, 'console', 200, 1)
    assert a.status('bfld', 1.5).state == STATE_IDLE
    assert intent(a, 1.5, channel='bfld').accepted          # bfld does not share tfld
    r = a.claim('fld', 'foxglove', 200, 30.0, 2.0)
    assert not r.accepted and 'on tfld' in r.reason


def test_fld_claim_covers_both_boards():
    a = led()
    a.claim('fld', 'console', 200, 30.0, 1.0)
    for sub in ('tfld', 'bfld'):
        assert a.status(sub, 1.5).state == STATE_CLAIMED
        assert not intent(a, 1.5, channel=sub).accepted


def test_tfld_run_replaces_the_fld_run():
    a = led()
    intent(a, 1.0, channel='fld', duration_ms=5000)
    r = intent(a, 2.0, channel='tfld', action='text', params={'lines': ['hi']},
               duration_ms=5000)
    assert calls(r.actions) == [(CALL_SAY, 'tfld')]
    assert a.status('fld', 2.0).state == STATE_IDLE
    assert a.status('tfld', 2.0).state == STATE_RUNNING


# --- manual auto-claim ---------------------------------------------------------------------

def test_manual_intent_auto_claims_until_hold_after_its_run_ends():
    a = led()
    r = intent(a, 1.0, source='manual', priority=199, duration_ms=5000)
    assert r.accepted and r.actions[0].request['duration_ms'] == 5000
    st = a.status('fld', 1.0)
    assert (st.state, st.active_source, st.owner, st.priority) == (
        STATE_RUNNING, 'manual', 'manual', 199)
    step(a, 3.0)
    assert not intent(a, 3.0).accepted            # an idle flourish is dropped
    step(a, 6.0)                                  # the run ends by its duration
    assert a.status('fld', 6.0).state == STATE_CLAIMED
    step(a, 10.9)
    assert a.status('fld', 10.9).state == STATE_CLAIMED
    out = step(a, 11.0)
    assert a.status('fld', 11.0).state == STATE_IDLE
    assert calls(out) == [(CALL_CONFIGURE, 'tfld'), (CALL_CONFIGURE, 'bfld')]  # released
    assert a.status('fld', 11.0).dropped == 2       # and the flourish due at 10


def test_manual_run_without_duration_holds_its_claim_until_it_ends_then_hold():
    a = led()
    intent(a, 1.0, channel='rld', source='manual', priority=199)      # duration 0
    step(a, 60.0)
    st = a.status('rld', 60.0)
    assert (st.state, st.owner, st.lease_remaining_s) == (STATE_RUNNING, 'manual', -1.0)
    intent(a, 60.0, channel='rld', source='manual', priority=199, action='stop')
    st = a.status('rld', 60.0)
    assert (st.state, st.lease_remaining_s) == (STATE_CLAIMED, 5.0)
    step(a, 65.0)
    assert a.status('rld', 65.0).state == STATE_IDLE


def test_manual_auto_claim_is_renewed_by_each_manual_intent():
    a = led()
    intent(a, 1.0, channel='rld', source='manual', priority=199, action='configure',
           params={'brightness': 80})
    step(a, 5.0)
    intent(a, 5.0, channel='rld', source='manual', priority=199, action='configure',
           params={'brightness': 90})
    step(a, 9.9)
    assert a.status('rld', 9.9).state == STATE_CLAIMED
    assert abs(a.status('rld', 9.9).lease_remaining_s - 0.1) < 1e-6
    step(a, 10.0)
    assert a.status('rld', 10.0).state == STATE_IDLE


def test_manual_configure_preempts_a_lower_run():
    a = led()
    intent(a, 1.0, channel='rld', duration_ms=5000)
    r = intent(a, 2.0, channel='rld', source='manual', priority=199, action='configure',
               params={'brightness': 80})
    assert calls(r.actions) == [(CALL_STOP, 'rld'), (CALL_CONFIGURE, 'rld')]
    cfg_action = r.actions[1]
    assert cfg_action.request['apply'] == APPLY_BRIGHTNESS
    assert cfg_action.priority == 199


def test_manual_intent_does_not_pass_a_higher_claim():
    a = led()
    a.claim('fld', 'console', 200, 30.0, 1.0)
    r = intent(a, 1.5, source='manual', priority=199)
    assert not r.accepted
    assert intent(a, 1.5, source='manual', priority=200).accepted


def test_manual_intent_below_another_claim_is_dropped_even_under_its_auto_claim():
    a = led()
    assert a.claim('fld', 'bt', 150, 30.0, 1.0).accepted
    assert intent(a, 1.5, source='manual', priority=200, action='configure',
                  params={'brightness': 80}).accepted
    r = intent(a, 2.0, source='manual', priority=120)
    assert not r.accepted and "claimed by 'bt' at priority 150" in r.reason
    assert intent(a, 2.5, source='manual', priority=199).accepted
    st = a.status('fld', 2.5)
    assert (st.owner, st.priority) == ('manual', 200)   # renewal never lowers the claim


# --- priority within a level ---------------------------------------------------------------

def test_equal_priority_newer_wins_unless_non_interruptible():
    a = led()
    first = intent(a, 1.0, channel='rld', source='behavior', priority=100, duration_ms=5000)
    second = intent(a, 2.0, channel='rld', source='behavior', priority=100, duration_ms=5000,
                    intent_id='b')
    assert first.accepted and second.accepted
    assert a.status('rld', 2.0).active_intent_id == 'b'
    intent(a, 3.0, channel='rld', source='behavior', priority=100, duration_ms=4000,
           interruptible=False, intent_id='keep')
    r = intent(a, 4.0, channel='rld', source='behavior', priority=100, intent_id='late')
    assert not r.accepted and 'non-interruptible' in r.reason
    assert a.status('rld', 4.0).dropped == 1
    step(a, 7.0)                                       # 'keep' ends by its duration
    assert intent(a, 7.0, channel='rld', source='behavior', priority=100).accepted


def test_a_running_higher_priority_intent_blocks_lower_ones():
    a = led()
    intent(a, 1.0, channel='rld', source='behavior', priority=100, duration_ms=5000)
    r = intent(a, 2.0, channel='rld')
    assert not r.accepted and 'running at priority 100' in r.reason
    r = intent(a, 2.0, channel='rld', action='stop')
    assert not r.accepted
    r = intent(a, 2.0, channel='rld', action='configure', params={'brightness': 30})
    assert not r.accepted                  # brightness would show through the effect


def test_drops_are_counted_per_channel():
    a = led()
    a.claim('rld', 'console', 200, 30.0, 1.0)
    for t in (1.1, 1.2, 1.3):
        intent(a, t, channel='rld')
    assert a.status('rld', 1.3).dropped == 3
    assert a.status('fld', 1.3).dropped == 0


# --- duration cap (rule 3) -----------------------------------------------------------------

@pytest.mark.parametrize('source, priority, asked, sent', [
    ('idle', 10, 0, 30000), ('idle', 10, 90000, 30000), ('idle', 10, 2000, 2000),
    ('behavior', 100, 0, 30000), ('behavior', 100, 45000, 30000),
    ('manual', 199, 0, 0), ('manual', 200, 90000, 90000)])
def test_rule3_duration_cap_for_idle_and_behaviour_not_manual(source, priority, asked, sent):
    a = led()
    r = intent(a, 1.0, channel='rld', source=source, priority=priority, duration_ms=asked)
    assert r.actions[0].request['duration_ms'] == sent


def test_rule3_duration_from_params_when_the_field_is_zero():
    a = led()
    r = intent(a, 1.0, channel='rld', params={'name': 'fire', 'duration_ms': 1500})
    assert r.actions[0].request['duration_ms'] == 1500


def test_rule3_magic_sequence_is_stopped_by_the_arbiter_at_its_duration():
    a = magic()
    r = intent(a, 1.0, channel='magic', action='sequence', params={'id': 44},
               duration_ms=3000)
    assert calls(r.actions) == [(CALL_START, 'magic')]
    assert r.actions[0].request == {'sequence_id': 44, 'repeat': 1, 'end': 1,
                                    'owner': OWNER, 'priority': 10}
    accept_all(a, r.actions, 1.0)
    run = magic_snap(state=1, started_by='ros', owner=OWNER, sequence_id=44)
    assert step(a, 3.0, snap=run) == []
    out = step(a, 4.0, snap=run)
    assert calls(out) == [(CALL_MAGIC_STOP, 'magic')]
    assert out[0].request == {'mode': 0, 'priority': 10} and out[0].why == 'duration'
    assert a.status('magic', 4.0).state == STATE_RUNNING     # until the device takes it
    a.device_reply(out[0], True, '', 4.0)
    assert a.status('magic', 4.0).state == STATE_IDLE


def test_rule3_a_refused_duration_stop_is_retried_after_the_hold():
    a = magic()
    r = intent(a, 1.0, channel='magic', action='sequence', params={'id': 27, 'repeat': 2},
               duration_ms=3000)
    accept_all(a, r.actions, 1.0)
    run = magic_snap(state=1, started_by='ros', owner=OWNER, sequence_id=27)
    stop = step(a, 4.0, snap=run)[0]
    a.device_reply(stop, False, 'held by console', 4.0)
    assert CALL_MAGIC_STOP not in [x.call for x in step(a, 8.9, snap=run)]
    assert calls(step(a, 9.0, snap=run)) == [(CALL_MAGIC_STOP, 'magic')]


def test_rule3_idle_magic_sequence_without_duration_gets_the_cap():
    a = magic()
    intent(a, 1.0, channel='magic', action='sequence', params={'name': 'Smile'})
    assert a.status('magic', 1.0).active_remaining_ms == 30000


def test_a_sequence_the_arbiter_starts_ends_blank_unless_the_intent_says_leave():
    a = magic()
    for source, priority in (('idle', 10), ('behavior', 100), ('manual', 199)):
        r = intent(a, 1.0, channel='magic', source=source, priority=priority,
                   action='sequence', params={'id': 44}, duration_ms=3000)
        assert r.actions[0].request['end'] == 1
    r = intent(a, 2.0, channel='magic', source='manual', priority=199, action='sequence',
               params={'id': 44, 'end': 'leave'}, duration_ms=3000)
    assert r.actions[0].request['end'] == 0


def test_an_unbounded_sequence_is_refused_for_idle_and_behaviour_but_not_manual():
    a = magic()
    for source, priority in (('idle', 10), ('behavior', 100)):
        for params in ({'id': 44, 'repeat': 0}, {'id': 56}, {'name': 'Random show'}):
            r = intent(a, 1.0, channel='magic', source=source, priority=priority,
                       action='sequence', params=params, duration_ms=3000)
            assert (r.accepted, r.reason) == (False, 'unbounded sequence')
    assert a.status('magic', 1.0).dropped == 6
    r = intent(a, 2.0, channel='magic', source='manual', priority=199, action='sequence',
               params={'id': 56, 'repeat': 0}, duration_ms=3000)
    assert r.accepted and r.actions[0].request['repeat'] == 0


# --- psi fan-out ---------------------------------------------------------------------------

def test_psi_fan_out_partial_hold_refusal_is_reported_and_the_other_keeps_its_run():
    a = led()
    r = intent(a, 1.0, channel='psi', source='behavior', priority=100, action='clip',
               params={'name': 'flash', 'rgb': 0xff2200}, duration_ms=1200)
    fpsi, rpsi = r.actions
    assert fpsi.request['surface'] == 'fpsi' and rpsi.request['surface'] == 'rpsi'
    line = a.device_reply(fpsi, False, 'held by console', 1.0)
    assert 'fpsi' in line and 'held by console' in line and 'rpsi keeps its run' in line
    assert a.device_reply(rpsi, True, '', 1.0) == ''
    st = a.status('psi', 1.0)
    assert (st.state, st.owner, st.refused) == (STATE_FOREIGN, 'console', 1)
    assert st.active_intent_id == r.intent_id            # rpsi still runs it


def test_psi_fan_out_other_refusal_is_a_failure_not_a_hold():
    a = led()
    r = intent(a, 1.0, channel='psi', action='clip', params={'name': 'pulse'})
    line = a.device_reply(r.actions[0], False, 'no clip for this surface', 1.0)
    assert 'failed: no clip for this surface' in line and 'rpsi keeps its run' in line
    st = a.status('psi', 1.0)
    assert (st.state, st.refused) == (STATE_RUNNING, 0)
    a.device_reply(r.actions[1], False, 'no clip for this surface', 1.0)
    assert a.status('psi', 1.0).state == STATE_IDLE


def test_text_on_psi_is_refused_with_the_devices_string():
    a = led()
    r = intent(a, 1.0, channel='psi', source='manual', priority=199, action='text',
               params={'lines': ['hi']})
    assert (r.accepted, r.reason) == (False, 'display has no text mode')
    assert a.status('psi', 1.0).state == STATE_IDLE          # a drop claims nothing


def test_text_request_fields():
    a = led()
    r = intent(a, 1.0, channel='fld', source='manual', priority=199, action='text',
               params={'lines': ['hello', 'world'], 'motion': 'left', 'on_black': True,
                       'rgb': 'ff2200'})
    assert r.actions[0].request == {'surface': 'fld', 'lines': ['hello', 'world'],
                                    'linked': False, 'motion': 2, 'on_black': True,
                                    'rgb': 0xff2200, 'speed': 0, 'duration_ms': 0,
                                    'owner': OWNER, 'priority': 199}


# --- the device's holds --------------------------------------------------------------------

@pytest.mark.parametrize('reason, holder', [('held by console', 'console'), ('held by ui', 'ui'),
                                            ('held by text topic', 'text_topic'),
                                            ('held: manual', 'manual'),
                                            ('unknown effect', '')])
def test_held_refusals_name_the_holder(reason, holder):
    assert held_by(reason) == holder


def test_hold_refusal_marks_foreign_with_the_holder_and_the_devices_hold_time():
    a = led()
    r = intent(a, 1.0, channel='rld', duration_ms=3000)
    a.device_reply(r.actions[0], False, 'held by console', 1.0)
    st = a.status('rld', 1.0)
    assert (st.state, st.owner, st.lease_remaining_s, st.refused) == (
        STATE_FOREIGN, 'console', 5.0, 1)
    assert st.active_intent_id == ''                     # the run never started
    r = intent(a, 1.2, channel='rld')
    assert not r.accepted and r.reason == 'channel is foreign'


def test_held_manual_refusal_is_forever_until_the_state_topic_says_otherwise():
    a = led()
    r = intent(a, 1.0, channel='rld', source='manual', priority=199)
    a.device_reply(r.actions[0], False, 'held: manual', 1.0)
    st = a.status('rld', 2.0)
    assert (st.state, st.owner, st.lease_remaining_s) == (STATE_FOREIGN, 'manual', -1.0)
    step(a, 60.0, snap=led_snap(manual=True))
    assert a.status('rld', 60.0).lease_remaining_s == -1.0
    step(a, 61.0)
    assert a.status('rld', 61.0).state == STATE_IDLE


def test_held_by_on_the_state_topic_marks_foreign_with_its_remaining_time():
    a = led()
    text = running('text', source='text_topic', owner='')
    step(a, 1.0, snap=led_snap(displays={'tfld': held('text_topic', 4200)},
                               surfaces={'fld': text}))
    for channel in ('fld', 'tfld'):
        st = a.status(channel, 2.0)
        assert (st.state, st.owner) == (STATE_FOREIGN, 'text_topic')
        assert abs(st.lease_remaining_s - 3.2) < 1e-6
    assert a.status('bfld', 2.0).state == STATE_FOREIGN   # the fld text run covers bfld
    assert a.status('rld', 2.0).state == STATE_IDLE


def test_a_hold_never_stops_the_persons_run():
    a = led()
    r = intent(a, 1.0, channel='rld', duration_ms=20000)
    accept_all(a, r.actions, 1.0)
    person = led_snap(displays={'rld': held('console')},
                      surfaces={'rld': running('rainbow', source='console', owner='')})
    step(a, 1.2, snap=person)                        # within the grace: our record stays
    c = a.claim('rld', 'test', 200, 30.0, 1.3)
    assert c.accepted and c.actions == ()            # the surface runs the person's effect
    step(a, 3.0, snap=person)
    assert a.status('rld', 3.0).active_intent_id == ''


def test_hold_expiry_reapplies_the_palette():
    a = led()
    out = step(a, 1.0, snap=led_snap(displays={'rld': held('console', 2000)}))
    assert out == [] and a.status('rld', 1.0).state == STATE_FOREIGN
    out = step(a, 3.5)
    assert calls(out) == [(CALL_CONFIGURE, 'rld')]
    assert out[0].request['brightness'] == 60 and out[0].request['drift'] == 'calm'
    assert out[0].why == 'palette' and out[0].priority == 10
    assert a.status('rld', 3.5).state == STATE_IDLE
    assert step(a, 4.0) == []                        # once, not every tick


def _styled(out, channel):
    return [x for x in out if x.channel == channel and x.call == CALL_CONFIGURE]


def _fresh_palette():
    a = Arbiter(cfg(), LED_CHANNELS, rng=FixedRng(0.0))
    collect(a.set_catalogue(LED_CAT, 0.0))
    collect(a.led_state(led_snap(), 0.0))
    return a, collect(a.mood('calm', 1.0, 0.0))


def test_a_failed_palette_configure_is_retried_after_the_backoff():
    a, out = _fresh_palette()
    (first,) = _styled(out, 'rld')
    assert a.device_reply(first, False, 'timeout', 0.1).startswith('rld: configure')
    assert _styled(step(a, 4.9), 'rld') == []            # device_stale_s (5 s) not over
    again = _styled(step(a, 5.2), 'rld')
    assert len(again) == 1 and again[0].why == 'palette'
    assert a.device_reply(again[0], True, '', 5.3) == ''
    assert _styled(step(a, 20.0), 'rld') == []           # accepted: not again


def test_a_palette_the_device_refuses_outright_is_not_retried():
    a, out = _fresh_palette()
    a.device_reply(_styled(out, 'rld')[0], False, 'out of range', 0.1)
    assert _styled(step(a, 30.0), 'rld') == []


def test_a_failed_palette_retry_backs_off_again():
    a, out = _fresh_palette()
    a.device_reply(_styled(out, 'rld')[0], False, 'timeout', 0.1)
    (again,) = _styled(step(a, 5.2), 'rld')
    a.device_reply(again, False, 'timeout', 5.3)
    assert _styled(step(a, 10.0), 'rld') == []
    assert len(_styled(step(a, 10.5), 'rld')) == 1


def test_a_new_label_after_a_failed_palette_does_not_wait_for_the_backoff():
    a, out = _fresh_palette()
    a.device_reply(_styled(out, 'rld')[0], False, 'timeout', 0.1)
    assert len(_styled(a.mood('excited', 1.0, 1.0), 'rld')) == 1


def test_a_palette_refused_as_held_is_not_resent_until_the_hold_ends():
    a, out = _fresh_palette()
    line = a.device_reply(_styled(out, 'rld')[0], False, 'held by console', 0.1)
    assert 'channel foreign' in line and a.status('rld', 0.1).state == STATE_FOREIGN
    person = led_snap(displays={'rld': held('console', 5000)})
    assert _styled(step(a, 9.0, snap=person), 'rld') == []   # past the backoff, still held
    assert a.status('rld', 9.0).state == STATE_FOREIGN
    assert len(_styled(step(a, 10.0), 'rld')) == 1           # the hold ended


def test_configure_drift_is_accepted_on_every_logic_display_but_not_the_psis():
    a = led()
    for channel in ('fld', 'tfld', 'bfld', 'rld'):
        r = intent(a, 1.0, channel=channel, source='manual', priority=199, action='configure',
                   params={'drift': 'lively'})
        assert r.accepted and r.actions[-1].request['apply'] & APPLY_DRIFT
        assert r.actions[-1].request['drift'] == 'lively'
    r = intent(a, 2.0, channel='psi', source='manual', priority=199, action='configure',
               params={'drift': 'lively'})
    assert not r.accepted and 'display has no drift' in r.reason


def test_after_the_hold_the_palette_waits_for_the_persons_run_to_end():
    a = led()
    run = {'rld': running('rainbow', source='console', owner='')}
    step(a, 1.0, snap=led_snap(displays={'rld': held('console', 1000)}, surfaces=run))
    out = step(a, 3.0, snap=led_snap(displays={'rld': {'mode': 5}}, surfaces=run))
    st = a.status('rld', 3.0)
    assert out == [] and (st.state, st.owner, st.lease_remaining_s) == (STATE_FOREIGN,
                                                                        'console', 0.0)
    assert not intent(a, 3.0, channel='rld').accepted
    out = step(a, 4.0)                               # the person stopped it
    assert calls(out) == [(CALL_CONFIGURE, 'rld')]


def test_after_the_hold_a_display_left_off_is_put_back_to_random():
    a = led()
    step(a, 1.0, snap=led_snap(displays={'tfld': dict(held('console', 1000), mode=4)}))
    out = step(a, 3.0, snap=led_snap(displays={'tfld': {'mode': 4}}))
    by_display = {x.target: x.request for x in out}
    assert by_display['tfld']['apply'] == APPLY_MODE | APPLY_BRIGHTNESS | APPLY_DRIFT
    assert by_display['tfld']['mode'] == 0
    assert by_display['bfld']['apply'] == APPLY_BRIGHTNESS | APPLY_DRIFT


def test_manual_setting_is_a_standing_claim_forever():
    a = led()
    step(a, 1.0, snap=led_snap(manual=True))
    for t in (1.0, 500.0):
        step(a, t, snap=led_snap(manual=True))
        for channel in LED_CHANNELS:
            st = a.status(channel, t)
            assert (st.state, st.owner, st.lease_remaining_s) == (STATE_FOREIGN, 'manual',
                                                                  -1.0)
    assert not intent(a, 500.0, channel='fld').accepted
    r = intent(a, 500.0, channel='fld', source='manual', priority=199)
    assert r.accepted                                # sent: the device refuses held: manual
    out = step(a, 501.0)                             # AUTO
    assert {x.target for x in out if x.call == CALL_CONFIGURE} == {'rld', 'fpsi', 'rpsi'}


def test_refusal_hold_gives_way_to_a_later_snapshot():
    a = led()
    r = intent(a, 1.0, channel='rld')
    a.device_reply(r.actions[0], False, 'held by ui', 1.0)
    step(a, 2.0)                                     # too soon after the refusal
    assert a.status('rld', 2.0).state == STATE_FOREIGN
    out = step(a, 2.6)
    assert a.status('rld', 2.6).state == STATE_IDLE
    assert calls(out) == [(CALL_CONFIGURE, 'rld')]


def test_a_snapshot_hold_ends_when_its_time_runs_out():
    a = led()
    step(a, 1.0, snap=led_snap(displays={'rld': held('console', 1000)}))
    assert a.status('rld', 1.5).state == STATE_FOREIGN
    out = collect(a.tick(2.0))                       # no newer snapshot yet
    assert a.status('rld', 2.0).state == STATE_IDLE
    assert calls(out) == [(CALL_CONFIGURE, 'rld')]


def test_a_refusal_hold_on_a_board_covers_the_overlapping_channels():
    a = led()
    r = intent(a, 1.0, channel='tfld', action='configure', params={'brightness': 50})
    a.device_reply(r.actions[0], False, 'held by console', 1.0)
    for channel in ('fld', 'tfld'):
        assert a.status(channel, 1.1).state == STATE_FOREIGN
    assert a.status('bfld', 1.1).state == STATE_IDLE
    assert collect(a.tick(1.1)) == []                # no re-send into the hold
    assert a.status('tfld', 1.1).refused == 1 and a.status('fld', 1.1).refused == 0


# --- dark and offline ----------------------------------------------------------------------

def test_dark_suppresses_flourishes_and_the_palette():
    a = led()
    dark = led_snap(strip=False)
    assert step(a, 1.0, snap=dark) == []
    assert all(s.state == STATE_DARK for s in a.statuses(1.0))
    out = step(a, 10.0, snap=dark)                   # the flourish falls due and is dropped
    assert out == [] and a.status('fld', 10.0).dropped == 1
    out = step(a, 11.0)
    assert {x.call for x in out} == {CALL_CONFIGURE}   # back on: re-styled
    assert len(out) == 5


def test_offline_without_catalogue_or_snapshot_and_after_device_stale_s():
    a = Arbiter(cfg(), LED_CHANNELS, rng=FixedRng(0.0))
    assert a.status('fld', 0.0).state == STATE_OFFLINE
    assert a.led_state(led_snap(), 0.0) == []
    assert a.status('fld', 0.0).state == STATE_OFFLINE       # no catalogue yet
    a.mood('calm', 1.0, 0.0)
    out = a.set_catalogue(LED_CAT, 0.5)
    assert len(out) == 5 and a.status('fld', 0.5).state == STATE_IDLE
    a.mood('calm', 1.0, 5.5)
    assert a.tick(5.5) == [] and a.status('fld', 5.5).state == STATE_OFFLINE
    a.mood('calm', 1.0, 10.0)
    out = a.tick(10.0)
    assert out == [] and a.status('fld', 10.0).dropped == 1  # the flourish is dropped
    assert not intent(a, 10.0).accepted
    out = step(a, 11.0)
    assert len(out) == 5 and a.status('fld', 11.0).state == STATE_IDLE


def test_magic_link_absent_is_offline():
    a = magic(snap=magic_snap(link=0))
    assert a.status('magic', 0.0).state == STATE_OFFLINE


# --- own runs survive a restart ------------------------------------------------------------

def test_own_runs_are_adopted_at_startup_and_bounded():
    snap = led_snap(surfaces={'fld': running('cycle:fire'), 'fpsi': running('pulse'),
                              'rld': running('fire', owner='someone')})
    a = led(config=quiet(), snap=snap)
    st = a.status('fld', 0.0)
    assert (st.state, st.active_source, st.active_remaining_ms) == (STATE_RUNNING, 'adopted',
                                                                    30000)
    assert a.status('psi', 0.0).state == STATE_RUNNING
    assert (a.status('rld', 0.0).state, a.status('rld', 0.0).owner) == (STATE_FOREIGN,
                                                                        'someone')
    later = led_snap(surfaces={'fld': running('cycle:roam'), 'fpsi': running('pulse'),
                               'rld': running('fire', owner='someone')})
    step(a, 10.0, snap=later)
    assert a.status('fld', 10.0).state == STATE_RUNNING        # cycle:<child> is still it
    out = step(a, 30.0, snap=later)
    assert sorted(calls(out)) == [(CALL_STOP, 'fld'), (CALL_STOP, 'fpsi')]
    assert all(x.why == 'duration' and x.priority == 10 for x in out)


def test_adopted_run_cleared_when_the_state_topic_shows_it_ended():
    a = led(snap=led_snap(surfaces={'rld': running('fire')}))
    step(a, 1.0)
    assert a.status('rld', 1.0).state == STATE_IDLE


def test_magic_run_is_adopted_at_startup_and_stopped_at_the_cap():
    run = magic_snap(state=1, started_by='ros', owner=OWNER, sequence_id=56)
    a = magic(snap=run)
    assert a.status('magic', 0.0).active_source == 'adopted'
    assert step(a, 29.0, snap=run) == []
    assert calls(step(a, 30.0, snap=run)) == [(CALL_MAGIC_STOP, 'magic')]


# --- watchdog ------------------------------------------------------------------------------

def test_watchdog_stale_mood_drops_to_neutral_and_back():
    a = led()
    assert step(a, 2.0, mood=None) == []
    out = step(a, 3.5, mood=None)
    assert len(out) == 5 and {x.request['brightness'] for x in out} == {40}
    assert all(s.state == STATE_STALE and s.mood == 'neutral' for s in a.statuses(3.5))
    assert not intent(a, 3.6).accepted               # idle intents wait for the mood
    assert step(a, 10.0, mood=None) == []            # no flourish while stale
    out = step(a, 11.0)
    assert len(out) == 5 and {x.request['brightness'] for x in out} == {60}
    assert a.status('fld', 11.0).mood == 'calm'


def test_watchdog_never_a_mood_is_stale_after_mood_stale_s():
    a = led(mood=None)
    assert step(a, 3.0, mood=None) == []
    assert a.status('fld', 3.0).mood == ''
    out = step(a, 3.1, mood=None)
    assert len(out) == 5 and a.status('fld', 3.1).state == STATE_STALE


def test_palette_on_label_change_once_scaled_by_strength():
    a = Arbiter(cfg(), LED_CHANNELS, rng=FixedRng(0.0))
    a.set_catalogue(LED_CAT, 0.0)
    a.led_state(led_snap(), 0.0)
    out = collect(a.mood('excited', 0.5, 0.0))
    assert {x.target: (x.request['drift'], x.request['brightness']) for x in out} == {
        'tfld': ('lively', 90), 'bfld': ('lively', 90), 'rld': ('lively', 90),
        'fpsi': ('', 90), 'rpsi': ('', 90)}
    assert {x.target: x.request['apply'] for x in out}['fpsi'] == APPLY_BRIGHTNESS
    assert a.mood('excited', 0.9, 0.1) == []                  # same label: nothing
    assert len(a.mood('calm', 0.9, 0.2)) == 5


def test_unknown_label_uses_neutral_and_warns_once():
    a = led(mood='sleepy')
    assert a.take_warnings() == ["mood 'sleepy' has no palette in expression.yaml; using "
                                 'neutral']
    assert a.status('fld', 0.0).mood == 'neutral'
    a.mood('calm', 1.0, 1.0)
    a.mood('sleepy', 1.0, 2.0)
    assert a.take_warnings() == []


# --- flourishes ----------------------------------------------------------------------------

def test_flourish_fires_within_every_s_as_an_idle_intent():
    a = led()                                        # calm: every_s [10, 20], rng 0
    assert step(a, 9.9) == []
    out = step(a, 10.0)
    assert calls(out) == [(CALL_RUN, 'fld')]
    req = out[0].request
    assert (req['effect'], req['rgb'], req['duration_ms'], req['priority'], req['owner']) == (
        'pulse', 0x2060ff, 2000, 10, OWNER)
    st = a.status('fld', 10.0)
    assert (st.state, st.active_source, st.active_action) == (STATE_RUNNING, 'idle', 'clip')
    assert st.active_intent_id.startswith('flourish-')
    assert step(a, 19.9) == []
    assert calls(step(a, 20.0)) == [(CALL_RUN, 'fld')]


def test_flourish_never_while_claimed_and_the_drop_is_counted():
    a = led()
    step(a, 5.0)
    a.claim('fld', 'console', 200, 30.0, 5.0)
    assert step(a, 10.0) == []
    assert a.status('fld', 10.0).dropped == 1


def test_flourish_never_under_a_claim_even_at_idle_priority():
    a = led()
    step(a, 5.0)
    assert a.claim('fld', 'op', 10, 30.0, 5.0).accepted
    assert step(a, 25.0) == []
    assert a.status('fld', 25.0).dropped == 2


def test_flourish_never_while_foreign():
    a = led()
    person = led_snap(displays={'tfld': held('console')})
    assert step(a, 10.0, snap=person) == []
    assert a.status('fld', 10.0).dropped == 1


def test_flourish_restarts_on_label_change():
    a = led(rng=FixedRng(0.0))
    step(a, 3.0)
    step(a, 4.0, mood='excited')                     # every_s [5, 5] from 4.0
    assert [x.call for x in step(a, 8.9, mood='excited')] == []
    out = step(a, 9.0, mood='excited')
    assert calls(out) == [(CALL_RUN, 'fld')]         # rng 0 draws the first pick


def test_flourish_pick_for_the_other_arbiter_is_ignored_here():
    a = led(rng=FixedRng(0.99))                      # draws the magic pick
    step(a, 1.0, mood='excited')
    assert step(a, 6.0, mood='excited') == []
    m = magic(rng=FixedRng(0.99))
    step(m, 1.0, mood='excited')
    assert calls(step(m, 6.0, mood='excited')) == [(CALL_START, 'magic')]


def test_listleds_pick_validation_drops_an_absent_clip_with_one_warning():
    a = Arbiter(cfg(), LED_CHANNELS, rng=FixedRng(0.0))
    from r2_expression.palette import LedCatalogue
    a.set_catalogue(LedCatalogue(effects=frozenset({'rainbow'}), clips={}), 0.0)
    warnings = a.take_warnings()
    assert len(warnings) == 1 and "clip 'pulse'" in warnings[0]
    a.reload(cfg(), 0.5)
    assert a.take_warnings() == []
    a.led_state(led_snap(), 0.0)
    a.mood('calm', 1.0, 0.0)
    for t in (10.0, 30.0):
        a.mood('calm', 1.0, t)
        a.led_state(led_snap(), t)
        assert a.tick(t) == []                       # the calm set is empty here


# --- the Magic Panel -----------------------------------------------------------------------

def test_magic_palette_brightness_follows_the_mood():
    a = Arbiter(cfg(), ('magic',), rng=FixedRng(0.0))
    a.set_catalogue(MAGIC_CAT, 0.0)
    a.magic_state(magic_snap(), 0.0)
    out = collect(a.mood('excited', 1.0, 0.0))
    assert calls(out) == [(CALL_MAGIC_BRIGHTNESS, 'magic')]
    assert out[0].request == {'apply': 1, 'brightness': 12}


def test_configure_magic_panel_gated_on_held_by():
    a = magic(mood=None)
    held_snap = magic_snap(held_by='console', hold_remaining_ms=3000)
    a.magic_state(held_snap, 0.1)
    assert a.mood('calm', 1.0, 0.2) == []            # no palette while held
    r = intent(a, 0.3, channel='magic', source='manual', priority=200, action='brightness',
               params={'value': 9})
    assert not r.accepted and 'held by console' in r.reason
    out = step(a, 3.5)
    assert calls(out) == [(CALL_MAGIC_BRIGHTNESS, 'magic')]
    r = intent(a, 3.6, channel='magic', source='manual', priority=199, action='brightness',
               params={'value': 9})
    assert calls(r.actions) == [(CALL_MAGIC_BRIGHTNESS, 'magic')]


def test_magic_run_started_by_other_is_foreign():
    a = magic()
    out = step(a, 1.0, snap=magic_snap(state=1, started_by='other', sequence_id=44))
    st = a.status('magic', 1.0)
    assert out == [] and (st.state, st.owner, st.lease_remaining_s) == (STATE_FOREIGN,
                                                                        'other', 0.0)
    assert not intent(a, 1.0, channel='magic', action='sequence', params={'id': 3}).accepted
    step(a, 2.0, snap=magic_snap(state=2, started_by='other', sequence_id=44))
    assert a.status('magic', 2.0).state == STATE_IDLE


def test_magic_console_start_is_foreign_with_its_hold():
    a = magic()
    step(a, 1.0, snap=magic_snap(state=1, started_by='console', sequence_id=44,
                                 held_by='console', hold_remaining_ms=5000))
    st = a.status('magic', 1.0)
    assert (st.state, st.owner, st.lease_remaining_s) == (STATE_FOREIGN, 'console', 5.0)


def test_magic_sequence_by_name_and_unknown_name():
    a = magic()
    r = intent(a, 1.0, channel='magic', source='manual', priority=199, action='sequence',
               params={'name': 'Smile', 'end': 'blank', 'repeat': 2})
    assert r.actions[0].request['sequence_id'] == 44
    assert r.actions[0].request['end'] == 1 and r.actions[0].request['repeat'] == 2
    r = intent(a, 1.0, channel='magic', source='manual', priority=199, action='sequence',
               params={'name': 'Frown'})
    assert (r.accepted, r.reason) == (False, 'unknown sequence')


def test_magic_stop_intent():
    a = magic()
    intent(a, 1.0, channel='magic', action='sequence', params={'id': 44}, duration_ms=3000)
    r = intent(a, 1.5, channel='magic', action='stop', params={'mode': 'freeze'})
    assert calls(r.actions) == [(CALL_MAGIC_STOP, 'magic')]
    assert r.actions[0].request == {'mode': 1, 'priority': 10}


# --- stop ----------------------------------------------------------------------------------

def test_stop_names_exactly_the_surfaces_the_arbiter_started_on():
    a = led()
    intent(a, 1.0, channel='psi', action='clip', params={'name': 'pulse'})
    r = intent(a, 2.0, channel='psi', action='stop')
    assert calls(r.actions) == [(CALL_STOP, 'fpsi'), (CALL_STOP, 'rpsi')]
    r = intent(a, 3.0, channel='psi', action='stop')
    assert r.accepted and r.actions == () and r.reason == 'nothing running'
    r = intent(a, 3.0, channel='rld', source='manual', priority=199, action='stop')
    assert calls(r.actions) == [(CALL_STOP, 'rld')]


def test_stop_on_fld_stops_our_run_on_a_board():
    a = led()
    intent(a, 1.0, channel='tfld', source='behavior', priority=100, duration_ms=5000)
    r = intent(a, 2.0, channel='fld', source='behavior', priority=100, action='stop')
    assert calls(r.actions) == [(CALL_STOP, 'tfld')]
    assert a.status('tfld', 2.0).state == STATE_IDLE


def test_rule7_an_operator_stop_never_stops_a_run_it_did_not_start():
    a = led()
    step(a, 1.0, snap=led_snap(surfaces={'rld': running('fire', owner='other_node')}))
    r = intent(a, 1.5, channel='rld', source='manual', priority=150, action='stop')
    assert r.accepted and r.actions == ()
    m = magic()
    step(m, 1.0, snap=magic_snap(state=1, started_by='other', sequence_id=44))
    r = intent(m, 1.5, channel='magic', source='manual', priority=199, action='stop')
    assert r.accepted and r.actions == ()


def test_text_with_more_lines_than_displays_is_refused_with_the_devices_string():
    a = led()
    r = intent(a, 1.0, channel='rld', source='behavior', priority=100, action='text',
               params={'lines': ['one', 'two']})
    assert (r.accepted, r.reason) == (False, 'more lines than the surface has displays')
    assert intent(a, 1.0, channel='fld', source='behavior', priority=100, action='text',
                  params={'lines': ['one', 'two']}).accepted


def test_a_refused_replacement_keeps_tracking_the_run_it_would_have_replaced():
    a = magic()
    first = intent(a, 1.0, channel='magic', source='behavior', priority=100,
                   action='sequence', params={'id': 23, 'repeat': 2}, duration_ms=10000)
    accept_all(a, first.actions, 1.0)
    second = intent(a, 2.0, channel='magic', source='behavior', priority=100,
                    action='sequence', params={'id': 44})
    a.device_reply(second.actions[0], False, 'not supported by this panel', 2.0)
    assert a.status('magic', 2.0).active_intent_id == first.intent_id
    run = magic_snap(state=1, started_by='ros', owner=OWNER, sequence_id=23)
    assert calls(step(a, 11.0, snap=run)) == [(CALL_MAGIC_STOP, 'magic')]


def test_behaviour_intent_never_replaces_a_persons_run_after_the_hold():
    a = led()
    person = led_snap(surfaces={'fld': running('fire', source='console', owner='')})
    step(a, 1.0, snap=person)
    r = intent(a, 1.5, source='behavior', priority=50, params={'name': 'rainbow'})
    assert (r.accepted, r.reason, r.actions) == (False, 'channel is foreign', ())
    assert intent(a, 1.5, source='manual', priority=199).accepted   # an operator may


def test_rule7_no_duration_stop_is_sent_into_a_hold():
    a = magic()
    r = intent(a, 1.0, channel='magic', source='behavior', priority=100, action='sequence',
               params={'id': 27, 'repeat': 2}, duration_ms=3000)
    accept_all(a, r.actions, 1.0)
    ours_held = magic_snap(state=1, started_by='ros', owner=OWNER, sequence_id=27,
                           held_by='console', hold_remaining_ms=60000)
    assert step(a, 40.0, snap=ours_held) == []
    assert a.status('magic', 40.0).refused == 0
    ours = magic_snap(state=1, started_by='ros', owner=OWNER, sequence_id=27)
    assert calls(step(a, 61.5, snap=ours))[-1:] == [(CALL_MAGIC_STOP, 'magic')]


def test_a_stop_retry_keeps_the_manual_claim():
    a = magic()
    r = intent(a, 1.0, channel='magic', source='manual', priority=150, action='sequence',
               params={'id': 27, 'repeat': 0}, duration_ms=3000)
    accept_all(a, r.actions, 1.0)
    ours = magic_snap(state=1, started_by='ros', owner=OWNER, sequence_id=27)
    stop = step(a, 4.0, snap=ours)[0]
    a.device_reply(stop, False, 'busy', 4.0)
    step(a, 8.5, snap=ours)
    assert not a.claim('magic', 'op', 100, 30.0, 8.5).accepted


def test_a_stopping_run_ends_only_when_every_surface_took_its_stop():
    both = led_snap(surfaces={'fpsi': running('pulse'), 'rpsi': running('pulse')})
    a = led(config=quiet(), snap=both)
    stops = [x for x in step(a, 30.0, snap=both) if x.call == CALL_STOP]
    assert sorted(x.target for x in stops) == ['fpsi', 'rpsi']
    by = {x.target: x for x in stops}
    a.device_reply(by['fpsi'], True, '', 30.0)
    a.device_reply(by['rpsi'], False, 'busy', 30.0)
    assert a.status('psi', 30.0).state == STATE_RUNNING


@pytest.mark.parametrize('priority', [True, 5.5])
def test_a_priority_that_is_not_an_integer_is_a_counted_drop(priority):
    a = led()
    r = a.intent(Intent('rld', 'idle', priority, 'effect', {'name': 'fire'}), 1.0)
    assert not r.accepted and 'integer 1-255' in r.reason
    assert a.status('rld', 1.0).dropped == 1


def test_release_names_the_channel_the_claim_is_on():
    a = led()
    a.claim('fld', 'op', 100, 30.0, 1.0)
    r = a.claim('tfld', 'op', 0, 0.0, 2.0)
    assert not r.accepted and "'op' holds fld" in r.reason
    assert a.claim('fld', 'op', 0, 0.0, 2.0).accepted


def test_an_unconfirmed_displaced_run_is_not_restored():
    a = led()
    first = intent(a, 1.0, source='manual', priority=200, action='text',
                   params={'lines': ['a']})
    second = intent(a, 1.1, source='manual', priority=200, action='text',
                    params={'lines': ['b']})
    a.device_reply(first.actions[0], False, 'held by console', 1.2)
    a.device_reply(second.actions[0], False, 'held by console', 1.2)
    assert a.status('fld', 1.2).active_intent_id == ''


def test_palette_waits_for_a_higher_priority_run_to_end():
    a = led()
    intent(a, 1.0, channel='rld', source='safety', priority=255, duration_ms=3000)
    assert collect(a.mood('excited', 1.0, 1.5)) and all(
        x.target != 'rld' for x in a.mood('excited', 1.0, 1.5))
    out = step(a, 4.0, mood='excited')
    assert (CALL_CONFIGURE, 'rld') in calls(out)


@pytest.mark.parametrize('params', [{'name': 'fire', 'rgb': '255'},
                                    {'name': 'fire', 'rgb': 'zz2200'},
                                    {'name': 'fire', 'duration_ms': 2 ** 32}])
def test_request_fields_out_of_their_wire_range_are_dropped(params):
    a = led()
    r = intent(a, 1.0, channel='rld', source='manual', priority=199, params=params)
    assert (r.accepted, r.reason) == (False, 'out of range')


def test_bad_stop_mode_is_a_counted_drop():
    a = magic()
    r = intent(a, 1.0, channel='magic', source='manual', priority=199, action='stop',
               params={'mode': []})
    assert (r.accepted, r.reason) == (False, 'unknown stop mode')
    assert a.status('magic', 1.0).dropped == 1


# --- device evidence -----------------------------------------------------------------------

def test_a_confirmed_run_is_cleared_once_the_state_topic_shows_it_gone():
    a = led()
    r = intent(a, 1.0, channel='rld', duration_ms=20000)
    accept_all(a, r.actions, 1.0)
    step(a, 2.0)                                     # inside the grace: maybe an old message
    assert a.status('rld', 2.0).state == STATE_RUNNING
    step(a, 3.0, snap=led_snap(surfaces={'rld': running('fire')}))
    assert a.status('rld', 3.0).state == STATE_RUNNING
    step(a, 4.0, snap=led_snap(surfaces={'rld': running('fire', owner='x')}))
    assert a.status('rld', 4.0).active_intent_id == ''


def test_accepted_not_confirmed_is_logged():
    a = magic()
    r = intent(a, 1.0, channel='magic', action='sequence', params={'id': 44})
    line = a.device_reply(r.actions[0], True, 'not confirmed', 1.0)
    assert 'not confirmed' in line
    assert a.status('magic', 1.0).state == STATE_RUNNING


# --- reload --------------------------------------------------------------------------------

def test_reload_keeps_claims_and_counts_and_restyles():
    a = led()
    a.claim('rld', 'console', 200, 30.0, 1.0)
    intent(a, 1.0, channel='rld')
    out = collect(a.reload(cfg(neutral={'fld': {'brightness': 30}}), 2.0))
    st = a.status('rld', 2.0)
    assert (st.state, st.owner, st.dropped) == (STATE_CLAIMED, 'console', 1)
    assert {x.target for x in out} == {'tfld', 'bfld', 'fpsi', 'rpsi'}


def test_reload_before_any_other_call():
    a = Arbiter(cfg(), LED_CHANNELS)
    assert a.reload(cfg()) == []
    assert a.status('fld', 0.0).state == STATE_OFFLINE


# --- status --------------------------------------------------------------------------------

def _every_state():
    """Yield (state, status) for every ChannelStatus state."""
    a = led()
    yield STATE_IDLE, a.status('rld', 0.0)
    intent(a, 0.5, channel='rld', duration_ms=3000, intent_id='x1')
    yield STATE_RUNNING, a.status('rld', 0.5)
    a.claim('fld', 'console', 200, 30.0, 0.5)
    yield STATE_CLAIMED, a.status('fld', 0.5)
    step(a, 1.0, snap=led_snap(displays={'fpsi': held('ui', 2500)}))
    yield STATE_FOREIGN, a.status('psi', 1.0)
    step(a, 4.5, mood=None)
    yield STATE_STALE, a.status('rld', 4.5)
    a.tick(20.0)
    yield STATE_OFFLINE, a.status('rld', 20.0)
    step(a, 21.0, snap=led_snap(strip=False))
    yield STATE_DARK, a.status('rld', 21.0)


def test_status_fields_complete_for_every_state():
    names = {f.name for f in dataclasses.fields(Status)}
    assert names == {'channel', 'state', 'owner', 'priority', 'lease_remaining_s',
                     'active_source', 'active_action', 'active_intent_id',
                     'active_remaining_ms', 'mood', 'dropped', 'refused'}
    seen = {}
    for expected, st in _every_state():
        assert st.state == expected
        assert isinstance(st.channel, str) and st.channel
        assert isinstance(st.owner, str) and isinstance(st.mood, str)
        assert isinstance(st.priority, int) and 0 <= st.priority <= 255
        assert isinstance(st.lease_remaining_s, float)
        assert isinstance(st.active_remaining_ms, int) and st.active_remaining_ms >= 0
        assert isinstance(st.dropped, int) and isinstance(st.refused, int)
        seen[expected] = st
    assert len(seen) == 7
    run = seen[STATE_RUNNING]
    assert (run.active_source, run.active_action, run.active_intent_id,
            run.active_remaining_ms, run.mood) == ('idle', 'effect', 'x1', 3000, 'calm')
    claimed = seen[STATE_CLAIMED]
    assert (claimed.owner, claimed.priority, claimed.lease_remaining_s) == ('console', 200,
                                                                            30.0)
    foreign = seen[STATE_FOREIGN]
    assert (foreign.owner, foreign.priority, foreign.lease_remaining_s) == ('ui', 0, 2.5)
    assert seen[STATE_STALE].mood == 'neutral'
    assert seen[STATE_IDLE].owner == '' and seen[STATE_IDLE].lease_remaining_s == 0.0


# --- the dome (phase 2 plan 4.3) ----------------------------------------------------------

DOME_ARBITERS = []               # every dome arbiter a test drove, for the rule 14 sweep
STILL = {'body_speed_m_s': 0.0, 'body_yaw_rad_s': 0.0}
DRIVING = {'body_speed_m_s': 0.3, 'body_yaw_rad_s': 0.0}


def dome(config=None, rng=None, mood='excited', position=0.0, **kwargs):
    """Return a dome arbiter that has a status, a joint state and ``mood``; an ideal plant."""
    a = Arbiter(config or dome_cfg(), DOME_CHANNELS, rng=rng or FixedRng(0.0), **kwargs)
    a.test_plant, a.test_t, a.test_cmd, a.test_log = PlantDouble(position, ideal=True), 0.0, \
        0.0, []
    DOME_ARBITERS.append(a)
    feed(a, a.dome_state({'source': 'idle', 'position_rad': position}, 0.0))
    if mood:
        feed(a, a.mood(mood, 1.0, 0.0))
    return a


def learning_cfg():
    """Return the dome config with ``coast.learn`` on, a whole lesson at a time."""
    data = dome_data()
    data.setdefault('defaults', {})['dome'] = {
        'coast': {'learn': True, 'scale_limits': [0.01, 2.0], 'learn_rate': 1.0}}
    return parse_config(data)


def feed(a, actions):
    """Collect ``actions``; the last velocity among them drives the plant from now on."""
    actions = collect(actions)
    sent = [x for x in actions if x.call == CALL_VELOCITY]
    a.test_log += sent
    if sent:
        a.test_cmd = sent[-1].request['velocity_rad_s']
    return actions


def live(a, t, source='idle', mood='excited', body=None, position=True):
    """
    Run the dome node's loop up to ``t`` at 20 Hz; return the velocities sent.

    The joint state (unless ``position`` is False) and the converter's ``source``
    (unless None) at 10 Hz, with ``body`` (odometry fields) if given; ``mood`` at
    20 Hz (unless None); a tick; the plant follows the last velocity.
    """
    out = []
    while a.test_t < t - 1e-9:
        now = round(a.test_t + 0.05, 6)
        a.test_plant.step(a.test_cmd, 0.05)
        if round(now * 20) % 2 == 0:
            state = dict(body or {})
            if position:
                state['position_rad'] = a.test_plant.reading()
            if source is not None:
                state['source'] = source
            out += feed(a, a.dome_state(state, now))
        if mood:
            out += feed(a, a.mood(mood, 1.0, now))
        out += feed(a, a.tick(now))
        a.test_t = now
    return [x for x in out if x.call == CALL_VELOCITY]


def dome_intent(a, action, params=None, source='manual', priority=199, **kwargs):
    result = intent(a, a.test_t, 'dome', source, priority, action, params, **kwargs)
    feed(a, result.actions)
    return result


def speeds(actions):
    return [x.request['velocity_rad_s'] for x in actions]


def test_dome_look_at_sends_a_velocity_closes_on_the_joint_and_ends_with_a_0():
    a = dome()
    r = dome_intent(a, 'look_at', {'bearing': 1.0, 'profile': 'smooth'})
    assert r.accepted and calls(r.actions) == [(CALL_VELOCITY, 'dome')]
    assert speeds(r.actions) == [-MIN_SPEED]                  # bearing left: joint negative
    st = a.status('dome', 0.0)
    assert (st.state, st.active_action, st.active_source, st.active_remaining_ms) == (
        STATE_RUNNING, 'look_at', 'manual', 8000)
    assert (st.owner, st.priority) == ('manual', 199)   # the manual auto-claim
    assert a.moving
    sent = live(a, 6.0)
    assert not a.moving and a.test_plant.position == pytest.approx(-1.0, abs=0.05)
    assert sent[-1].request['velocity_rad_s'] == 0.0 and sent[-1].why == 'done'
    assert max(abs(v) for v in speeds(sent)) <= 1.0     # smooth's v_max
    assert a.status('dome', 6.0).state == STATE_CLAIMED
    live(a, 12.0)
    assert a.status('dome', 12.0).state == STATE_IDLE


def test_dome_active_remaining_ms_counts_down_the_budget_to_max_move_s():
    a = dome()
    dome_intent(a, 'turn', {'delta': 6.0, 'profile': 'lazy'}, source='behavior', priority=100)
    live(a, 2.0)
    assert a.status('dome', 2.0).active_remaining_ms == 6000
    sent = live(a, 8.0)
    assert not a.moving and sent[-1].request['velocity_rad_s'] == 0.0
    assert a.status('dome', 8.0).state == STATE_IDLE


def test_dome_track_is_refused_not_in_this_phase_and_counted():
    a = dome()
    r = dome_intent(a, 'track', {'bearing': 0.2})
    assert (r.accepted, r.reason, r.actions) == (False, 'not in this phase', ())
    assert a.status('dome', 0.0).dropped == 1


@pytest.mark.parametrize('action, params, fragment', [
    ('look_at', {}, 'bearing must be a finite number'),
    ('look_at', {'bearing': 'left'}, 'bearing must be a finite number'),
    ('look_at', {'bearing': True}, 'bearing must be a finite number'),
    ('look_at', {'bearing': 1.0, 'profile': 'zippy'}, "unknown profile 'zippy'"),
    ('turn', {'delta': float('inf')}, 'delta must be a finite number'),
    ('turn', {'delta': 10 ** 400}, 'delta must be a finite number'),
    ('turn', {'delta': '1.5'}, 'delta must be a finite number'),
    ('glance', {'amplitude': 4.0}, 'amplitude must be in (0, pi]'),
    ('glance', {'side': 'up'}, 'side must be left or right'),
    ('glance', {'dwell_s': -1}, 'dwell (dwell_s, or duration_ms) must be 0 to max_move_s'),
    ('rest', {}, 'the mood has no rest heading'),
    ('dance', {}, 'unknown action'),
    ('effect', {'name': 'fire'}, 'unknown action')])
def test_dome_malformed_params_are_refused_with_bad_params(action, params, fragment):
    a = dome()
    r = dome_intent(a, action, params)
    assert not r.accepted and fragment in r.reason, r.reason
    assert r.reason.startswith('bad params: ') or 'action' in r.reason
    assert a.status('dome', 0.0).dropped == 1 and not a.moving


def test_dome_turn_is_in_the_bearing_convention_and_goes_beyond_pi():
    a = dome()
    dome_intent(a, 'turn', {'delta': -4.0, 'profile': 'snappy'})
    live(a, 7.0)
    assert a.test_plant.position == pytest.approx(4.0, abs=0.05)    # right: joint positive


def test_dome_glance_duration_ms_is_the_dwell_when_dwell_s_is_absent():
    a = dome()
    dome_intent(a, 'glance', {'amplitude': 0.5, 'side': 'right'}, duration_ms=3000)
    assert a.status('dome', 0.0).active_remaining_ms == 11000      # max_move_s + the dwell


@pytest.mark.parametrize('source', ['joystick', 'foxglove'])
def test_dome_foreign_on_joystick_or_foxglove_abandons_the_move_with_no_lease(source):
    a = dome()
    dome_intent(a, 'look_at', {'bearing': 2.0}, source='behavior', priority=100)
    live(a, 0.5)
    sent = live(a, 0.6, source=source)                 # the status arrives at 0.6
    assert speeds(sent)[-1] == 0.0 and sent[-1].why == 'foreign'
    st = a.status('dome', 0.6)
    assert (st.state, st.owner, st.lease_remaining_s, st.active_action) == (
        STATE_FOREIGN, source, 0.0, '')
    assert not a.moving
    assert live(a, 3.0, source=source) == []            # nothing while the person drives
    assert live(a, 6.0, source='idle') == []            # let go: not resumed
    assert a.status('dome', 6.0).state == STATE_IDLE


def test_dome_moves_of_every_source_are_refused_while_foreign():
    a = dome()
    live(a, 0.2, source='joystick')
    for source, priority in (('manual', 200), ('safety', 255), ('behavior', 100)):
        r = dome_intent(a, 'look_at', {'bearing': 1.0}, source=source, priority=priority)
        assert (r.accepted, r.reason) == (False, 'channel is foreign')
    r = dome_intent(a, 'stop')                         # a stop is a 0: never a fight
    assert r.accepted and speeds(r.actions) == [0.0]


def test_dome_personality_source_at_the_converter_is_not_foreign():
    a = dome()
    live(a, 0.2, source='personality')
    assert a.status('dome', 0.2).state == STATE_IDLE


def test_dome_glances_resume_only_after_after_foreign_s():
    a = dome(mood='calm')                              # rng 0: a glance at 10 s
    live(a, 8.0, mood='calm')
    live(a, 9.5, source='foxglove', mood='calm')       # idle again at 9.6
    assert live(a, 12.55, mood='calm') == []           # due at 10, held to 9.6 + 3
    sent = live(a, 12.6, mood='calm')
    assert speeds(sent) == [-MIN_SPEED] and a.status('dome', 12.6).active_action == 'glance'


def test_dome_suppressed_on_body_speed_stops_the_move_and_drops_idle_and_behaviour():
    a = dome()
    dome_intent(a, 'look_at', {'bearing': 2.0}, source='behavior', priority=100)
    live(a, 0.5)
    sent = live(a, 0.6, body=DRIVING)
    assert speeds(sent)[-1] == 0.0 and sent[-1].why == 'suppressed'
    assert a.status('dome', 0.6).state == STATE_SUPPRESSED and not a.moving
    for source, priority, action in (('idle', 10, 'glance'), ('behavior', 100, 'look_at')):
        r = dome_intent(a, action, {'bearing': 1.0}, source=source, priority=priority)
        assert (r.accepted, r.reason) == (False, 'channel is suppressed')
    assert a.status('dome', 0.6).dropped == 2


def test_dome_suppressed_on_body_yaw_too():
    a = dome()
    live(a, 0.2, body={'body_speed_m_s': 0.0, 'body_yaw_rad_s': -0.5})
    assert a.status('dome', 0.2).state == STATE_SUPPRESSED
    live(a, 0.4, body={'body_speed_m_s': -0.1, 'body_yaw_rad_s': 0.2})   # under both
    assert a.status('dome', 0.4).state == STATE_SUPPRESSED                # after_still_s


def test_dome_manual_still_runs_while_suppressed():
    a = dome()
    live(a, 0.2, body=DRIVING)
    r = dome_intent(a, 'look_at', {'bearing': 1.0})
    assert r.accepted and speeds(r.actions) == [-MIN_SPEED]
    sent = live(a, 1.0, body=DRIVING)
    assert a.moving and sent and all(x.why == 'move' for x in sent)
    st = a.status('dome', 1.0)
    assert (st.state, st.active_action) == (STATE_SUPPRESSED, 'look_at')


def test_dome_body_fields_left_out_keep_their_last_value():
    a = dome()
    live(a, 0.2, body={'body_speed_m_s': 0.5})
    live(a, 3.0, body={'body_yaw_rad_s': 0.0})               # says nothing of the speed
    assert a.status('dome', 3.0).state == STATE_SUPPRESSED


def test_dome_suppressed_clears_after_still_s_after_the_body_stops():
    a = dome()
    live(a, 1.0, body=DRIVING)
    live(a, 1.9, body=STILL)                           # still from 1.2
    assert a.status('dome', 1.9).state == STATE_SUPPRESSED
    live(a, 2.2, body=STILL)
    assert a.status('dome', 2.2).state == STATE_IDLE


def test_dome_with_body_silence_s_a_silent_drive_command_counts_as_still():
    a = dome(body_silence_s=0.5)                       # /diff_cont/cmd_vel: silent when idle
    live(a, 0.2, body=DRIVING)                         # the last command at 0.2
    live(a, 0.65)
    assert a.status('dome', 0.65).state == STATE_SUPPRESSED
    live(a, 1.65)                                      # still from 0.7, clear after_still_s on
    assert a.status('dome', 1.65).state == STATE_SUPPRESSED
    live(a, 1.8)
    assert a.status('dome', 1.8).state == STATE_IDLE
    b = dome()                                         # odometry: silence is not still
    live(b, 0.2, body=DRIVING)
    live(b, 5.0)
    assert b.status('dome', 5.0).state == STATE_SUPPRESSED


def test_dome_stale_mood_stops_a_personality_move_and_manual_still_runs():
    a = dome()
    dome_intent(a, 'turn', {'delta': 6.0, 'profile': 'lazy'}, source='behavior', priority=100)
    assert live(a, 3.0, mood=None)[-1].why == 'move'
    sent = live(a, 3.5, mood=None)
    assert speeds(sent)[-1] == 0.0 and sent[-1].why == 'stale'
    assert a.status('dome', 3.5).state == STATE_STALE and not a.moving
    assert not dome_intent(a, 'glance', source='idle', priority=10).accepted
    r = dome_intent(a, 'look_at', {'bearing': 0.0})
    assert r.accepted and a.moving
    assert live(a, 4.0, mood=None)[-1].why == 'move'


def test_dome_offline_until_both_a_joint_state_and_a_status_arrive():
    a = Arbiter(dome_cfg(), DOME_CHANNELS, rng=FixedRng(0.0))
    assert a.status('dome', 0.0).state == STATE_OFFLINE
    assert a.dome_state({'position_rad': 0.0}, 0.1) == []
    assert a.status('dome', 0.1).state == STATE_OFFLINE
    r = a.intent(Intent('dome', 'manual', 199, 'look_at', {'bearing': 1.0}), 0.1)
    assert (r.accepted, r.reason) == (False, 'channel is offline')
    a.mood('excited', 1.0, 0.2)
    assert a.dome_state({'source': 'idle'}, 0.2) == []
    assert a.status('dome', 0.2).state == STATE_IDLE


@pytest.mark.parametrize('lost', ['status', 'joint state'])
def test_dome_offline_mid_move_when_status_or_feedback_is_stale_ends_it_with_a_0(lost):
    a = dome()
    dome_intent(a, 'turn', {'delta': 6.0, 'profile': 'lazy'}, source='behavior', priority=100)
    live(a, 0.5)
    if lost == 'status':
        sent = live(a, 5.6, source=None)
        assert sent[-1].why == 'offline' and speeds(sent)[-1] == 0.0
    else:
        sent = live(a, 5.6, position=False)
        assert sent[-1].why == 'done' and speeds(sent)[-1] == 0.0    # the move's own 0.5 s
        assert a.test_log[-1].request['velocity_rad_s'] == 0.0
    assert a.status('dome', 5.6).state == STATE_OFFLINE and not a.moving
    assert live(a, 8.0, source=None, position=False) == []          # nothing sent
    live(a, 8.2)
    assert a.status('dome', 8.2).state == STATE_IDLE


def test_dome_a_claim_stops_the_move_with_a_0():
    a = dome()
    dome_intent(a, 'look_at', {'bearing': 2.0}, source='behavior', priority=100)
    live(a, 0.5)
    c = a.claim('dome', 'console', 200, 30.0, 0.5)
    feed(a, c.actions)
    assert c.accepted and speeds(c.actions) == [0.0] and c.actions[0].why == 'preempt'
    assert c.status.state == STATE_CLAIMED and not a.moving
    r = dome_intent(a, 'look_at', {'bearing': 1.0}, source='behavior', priority=100)
    assert not r.accepted and "claimed by 'console'" in r.reason


def test_dome_equal_priority_replaces_the_move_without_a_0_unless_non_interruptible():
    a = dome()
    dome_intent(a, 'look_at', {'bearing': 3.0}, source='behavior', priority=100,
                intent_id='a')
    before = live(a, 0.5)[-1].request['velocity_rad_s']
    r = dome_intent(a, 'look_at', {'bearing': 2.5}, source='behavior', priority=100,
                    intent_id='b')
    assert r.accepted and speeds(r.actions)[0] < 0.0          # same way, no 0 between
    assert abs(speeds(r.actions)[0] - before) <= 4.0 * 0.05 + 1e-9
    assert a.status('dome', 0.5).active_intent_id == 'b'
    dome_intent(a, 'turn', {'delta': 3.0}, source='behavior', priority=100,
                interruptible=False, intent_id='keep')
    r = dome_intent(a, 'look_at', {'bearing': 0.0}, source='behavior', priority=100)
    assert not r.accepted and 'non-interruptible' in r.reason
    assert a.status('dome', 0.5).active_intent_id == 'keep'
    r = dome_intent(a, 'look_at', {'bearing': 0.0}, source='manual', priority=199)
    assert r.accepted and a.status('dome', 0.5).active_source == 'manual'
    r = dome_intent(a, 'look_at', {'bearing': 0.0}, source='behavior', priority=100)
    assert not r.accepted                                    # below the running one


def test_dome_stop_ends_the_move_with_a_0_and_manual_stop_sends_one_anyway():
    a = dome()
    dome_intent(a, 'look_at', {'bearing': 2.0}, source='behavior', priority=100)
    live(a, 0.3)
    r = dome_intent(a, 'stop', source='behavior', priority=100)
    assert speeds(r.actions) == [0.0] and not a.moving
    r = dome_intent(a, 'stop', source='behavior', priority=100)
    assert (r.accepted, r.reason, r.actions) == (True, 'nothing running', ())
    assert speeds(dome_intent(a, 'stop').actions) == [0.0]


def test_dome_profile_comes_from_params_for_manual_and_from_the_mood_for_idle():
    a = dome(mood='sleepy')                                   # lazy
    live(a, 8.0, mood='sleepy')                               # the rest move first
    dome_intent(a, 'look_at', {'bearing': -2.0, 'profile': 'snappy'}, source='idle',
                priority=10)
    assert max(abs(v) for v in speeds(live(a, 9.5, mood='sleepy'))) <= 0.6
    b = dome()
    dome_intent(b, 'look_at', {'bearing': -2.0, 'profile': 'lazy'})
    assert max(abs(v) for v in speeds(live(b, 1.5))) <= 0.6
    c = dome()
    dome_intent(c, 'look_at', {'bearing': -2.0, 'profile': 'mood'})   # excited: snappy
    assert max(abs(v) for v in speeds(live(c, 1.5))) > 1.0


def test_dome_moves_are_never_given_the_duration_cap():
    a = dome()
    r = dome_intent(a, 'look_at', {'bearing': 1.0}, source='idle', priority=10,
                    duration_ms=500)
    assert r.accepted and a.status('dome', 0.0).active_remaining_ms == 8000


# --- the dome: glances and rest ------------------------------------------------------------

def test_dome_idle_glance_fires_within_every_s_and_returns_to_where_it_started():
    a = dome(mood='calm', position=0.4)                       # every_s [10, 20], strength 1
    assert live(a, 9.9, mood='calm') == []
    sent = live(a, 10.0, mood='calm')
    assert speeds(sent) == [-MIN_SPEED]                             # rng 0: left, amplitude 0.3
    st = a.status('dome', 10.0)
    assert (st.active_source, st.active_action, st.priority) == ('idle', 'glance', 0)
    assert st.active_intent_id.startswith('glance-')
    sent = live(a, 16.0, mood='calm')
    assert not a.moving and speeds(sent)[-1] == 0.0
    assert a.test_plant.position == pytest.approx(0.4, abs=0.10)


def test_dome_glance_never_while_claimed_and_the_drop_is_counted():
    a = dome(mood='calm')
    live(a, 5.0, mood='calm')
    a.claim('dome', 'op', 10, 30.0, 5.0)
    assert live(a, 10.0, mood='calm') == []
    assert a.status('dome', 10.0).dropped == 1


def test_dome_glance_never_during_a_move_and_the_cadence_restarts_after_it():
    a = dome(mood='calm')
    live(a, 9.0, mood='calm')
    dome_intent(a, 'turn', {'delta': 1.0}, source='behavior', priority=100)
    live(a, 11.0, mood='calm')
    assert a.status('dome', 11.0).active_action == 'turn'     # not replaced, not dropped
    assert a.status('dome', 11.0).dropped == 0
    while a.moving:
        live(a, a.test_t + 0.05, mood='calm')
    ended = a.test_t
    assert live(a, ended + 9.9, mood='calm') == []
    assert speeds(live(a, ended + 10.05, mood='calm'))[0] != 0.0


def test_dome_no_glances_while_stale():
    a = dome(mood='calm')
    live(a, 2.0, mood='calm')
    assert live(a, 30.0, mood=None) == []
    assert a.status('dome', 30.0).dropped == 0


def test_dome_mood_rest_is_its_palette_on_a_label_change_and_after_foreign():
    a = dome(mood='sleepy')                                   # rest_rad 0.35: joint -0.35
    assert a.status('dome', 0.0).active_action == 'rest'
    assert speeds(a.test_log) == [-MIN_SPEED]
    live(a, 8.0, mood='sleepy')
    assert not a.moving and a.test_plant.position == pytest.approx(-0.35, abs=0.09)
    live(a, 9.0, source='joystick', mood='sleepy')
    a.test_plant.position = 1.0                               # the person turned it
    sent = live(a, 9.1, mood='sleepy')
    assert sent and sent[0].request['velocity_rad_s'] < 0.0 and sent[0].why == 'intent'
    live(a, 20.0, mood='sleepy')
    assert a.test_plant.position == pytest.approx(-0.35, abs=0.09)
    a.mood('excited', 1.0, 20.0)                               # no rest: nothing sent
    assert not a.moving


def test_dome_rest_is_re_applied_when_a_manual_claim_ends():
    a = dome(mood='sleepy')
    live(a, 8.0, mood='sleepy')
    dome_intent(a, 'look_at', {'bearing': -1.0})
    live(a, 13.0, mood='sleepy')
    assert a.status('dome', 13.0).state == STATE_CLAIMED       # the auto-claim
    while a.status('dome', a.test_t).state == STATE_CLAIMED:
        live(a, a.test_t + 0.05, mood='sleepy')
    assert a.status('dome', a.test_t).active_action == 'rest'


def test_dome_rest_drift_is_corrected_at_min_speed_once_per_rest_recheck_s():
    a = dome(mood='sleepy')
    while a.moving or a.test_t < 1.0:
        live(a, a.test_t + 0.05, mood='sleepy')
    ended = a.test_t
    a.test_plant.position = 0.0                                # pushed 0.35 away
    assert live(a, ended + 29.9, mood='sleepy') == []
    sent = live(a, ended + 30.0, mood='sleepy')
    assert speeds(sent) == [-MIN_SPEED] and a.status('dome', a.test_t).active_action == 'rest'
    live(a, ended + 40.0, mood='sleepy')
    assert all(abs(v) in (0.0, MIN_SPEED) for v in speeds(a.test_log[-200:]))
    assert a.test_plant.position == pytest.approx(-0.35, abs=0.09)


def test_dome_rest_within_the_band_is_not_corrected():
    a = dome(mood='sleepy')
    while a.moving or a.test_t < 1.0:
        live(a, a.test_t + 0.05, mood='sleepy')
    assert live(a, a.test_t + 65.0, mood='sleepy') == []


# --- the dome: the arbiter's surface -------------------------------------------------------

def test_dome_arbiter_takes_only_dome_state():
    a = dome()
    with pytest.raises(TypeError):
        a.set_catalogue(LED_CAT, 0.0)
    with pytest.raises(TypeError):
        a.led_state(led_snap(), 0.0)
    with pytest.raises(TypeError):
        led().dome_state({'source': 'idle'}, 0.0)


def test_dome_state_ignores_a_position_that_is_not_finite_and_wraps_the_rest():
    a = dome()
    a.dome_state({'position_rad': math.nan}, 0.1)
    a.dome_state({'position_rad': 7.0}, 0.2)
    r = dome_intent(a, 'look_at', {'bearing': -(7.0 - 2 * math.pi)})
    assert r.accepted and r.actions[0].request['velocity_rad_s'] == 0.0   # already there


def test_dome_config_warnings_are_raised_once_by_the_dome_arbiter_only():
    data = dome_data()
    data['profiles']['snappy']['v_max'] = 3.0
    config = parse_config(data)
    a = Arbiter(config, DOME_CHANNELS)
    warnings = a.take_warnings()
    assert any('profiles.snappy.v_max' in w and 'clamped to 2' in w for w in warnings)
    a.reload(config, 1.0)
    assert a.take_warnings() == []
    assert Arbiter(config, LED_CHANNELS).take_warnings() == []


def test_dome_status_suppressed_is_state_7():
    assert STATE_SUPPRESSED == 7 and STATE_NAMES[STATE_SUPPRESSED] == 'suppressed'
    assert len(STATE_NAMES) == 8


# --- sound (phase 3 plan 4.2, rules 20-24) -------------------------------------------------

def sound(config=None, rng=None, mood='calm', strength=1.0, snap=None, catalogue=SOUND_CAT):
    a = Arbiter(config or sound_cfg(), SOUND_CHANNELS, rng=rng or FixedRng(0.0))
    collect(a.set_catalogue(catalogue, 0.0))
    collect(a.sound_state(snap or sound_snap(), 0.0))
    if mood:
        collect(a.mood(mood, strength, 0.0))
    return a


def sstep(a, t, snap=None, mood='calm', strength=1.0):
    """Live up to ``t`` as the sound node would: a state, a mood (unless None) and a tick."""
    start = getattr(a, 'test_t', 0.0)
    times = [start + k for k in range(1, int(t - start) + 1) if start + k < t] + [t]
    out = []
    for now in times:
        out += a.sound_state(snap or sound_snap(), now)
        if mood:
            out += a.mood(mood, strength, now)
        out += a.tick(now)
    a.test_t = t
    return collect(out)


def sintent(a, t, source='idle', priority=10, action='play', params=None, duration_ms=0,
            interruptible=None, intent_id=''):
    if params is None:
        params = {'name': 'sent2'} if action == 'play' else {}
    result = a.intent(Intent('sound', source, priority, action, params, duration_ms,
                             interruptible, intent_id), t)
    collect(result.actions)
    return result


def plays(actions):
    return [x for x in actions if x.call == CALL_PLAY]


def stops(actions):
    return [x for x in actions if x.call == CALL_SOUND_STOP]


def sound_channel_cfg(**defaults):
    data = sound_data()
    data['defaults'].update(defaults)
    return parse_config(data)


def test_sound_is_a_channel_of_its_own_group():
    assert channel_of('sound').kind == 'sound' and channel_of('sound').surfaces == ('sound',)
    assert not channel_of('sound').overlaps(channel_of('fld'))
    assert Arbiter(sound_cfg(), SOUND_CHANNELS).channels == ('sound',)
    with pytest.raises(ValueError):
        Arbiter(cfg(), ('sound', 'dome'))
    with pytest.raises(TypeError):
        Arbiter(sound_cfg(), SOUND_CHANNELS).set_catalogue(LED_CAT, 0.0)
    with pytest.raises(TypeError):
        Arbiter(cfg(), LED_CHANNELS).sound_state(sound_snap(), 0.0)


def test_rule20_a_play_by_name_carries_the_owner_the_priority_and_the_intent_id():
    a = sound()
    (x,) = sintent(a, 1.0, 'manual', 199, params={'name': 'sent1'}, intent_id='m1').actions
    assert (x.call, x.channel, x.target, x.priority, x.owner, x.why) == (
        CALL_PLAY, 'sound', 'sound', 199, OWNER, 'intent')
    assert dict(x.request) == {'name': 'sent1', 'pool': '', 'gain_db': 0.0, 'duration_ms': 0,
                               'priority': 199, 'owner': OWNER, 'intent_id': 'm1'}


def test_rule20_a_play_by_pool_names_the_pool_and_not_a_clip():
    a = sound()
    (x,) = sintent(a, 1.0, 'manual', 199, params={'pool': 'hum', 'gain_db': -3}).actions
    assert (x.request['name'], x.request['pool'], x.request['gain_db']) == ('', 'hum', -3.0)


def test_rule20_a_name_or_pool_the_catalogue_lacks_is_refused_and_counted():
    a = sound()
    for params, reason in [({'name': 'nope'}, 'unknown name'), ({'pool': 'nope'}, 'unknown pool'),
                           ({'pool': 'excluded'}, 'unknown pool'),
                           ({'pool': 'laugh'}, 'unknown pool')]:
        r = sintent(a, 1.0, 'manual', 199, params=params)
        assert (r.accepted, r.reason, r.actions) == (False, reason, ()), params
    assert a.status('sound', 1.0).dropped == 4


def test_rule20_a_clip_intent_is_refused_use_play():
    a = sound()
    r = sintent(a, 1.0, 'manual', 199, action='clip', params={'name': 'sent1'})
    assert (r.accepted, r.reason) == (False, 'use play')
    assert a.status('sound', 1.0).dropped == 1
    assert not sintent(a, 1.0, action='effect').accepted      # an LED verb: unknown action


@pytest.mark.parametrize('params', [{}, {'name': 'sent1', 'pool': 'hum'}, {'name': ''},
                                    {'name': 'sent1', 'gain_db': 'loud'},
                                    {'name': 'sent1', 'gain_db': 100},
                                    {'name': 'sent1', 'gain_db': float('nan')},
                                    {'name': 'sent1', 'gain_db': True}])
def test_rule20_bad_play_params_are_refused(params):
    r = sintent(sound(), 1.0, 'manual', 199, params=params)
    assert not r.accepted and r.actions == ()


def test_rule20_the_mood_gain_is_added_for_the_personality_and_not_for_a_person():
    a = sound()
    assert plays(sintent(a, 1.0, 'idle', 10, params={'name': 'sent1'}).actions)[
        0].request['gain_db'] == -3.0                       # calm
    a = sound()
    assert plays(sintent(a, 1.0, 'idle', 10, params={'name': 'sent1', 'gain_db': 1}).actions)[
        0].request['gain_db'] == -2.0
    a = sound()
    assert plays(sintent(a, 1.0, 'manual', 199, params={'name': 'sent1'}).actions)[
        0].request['gain_db'] == 0.0
    a = sound()                       # stale: neutral's gain (a behaviour play still runs)
    a.sound_state(sound_snap(), 10.0)
    r = sintent(a, 10.0, 'behavior', 100, params={'name': 'sent1'})
    assert plays(r.actions)[0].request['gain_db'] == -6.0


def test_rule20_vocalize_is_refused_without_a_synth_and_built_with_one():
    a = sound()
    r = sintent(a, 1.0, 'manual', 199, action='vocalize', params={'arousal': 0.8})
    assert (r.accepted, r.reason) == (False, 'synth not available')
    a = sound(catalogue=sound_catalogue(sound_list(synth=True)))
    (x,) = sintent(a, 1.0, 'manual', 199, action='vocalize',
                   params={'arousal': 0.8, 'valence': 0.6, 'length_s': 1.5, 'seed': 42},
                   intent_id='v1', interruptible=True).actions
    assert x.call == CALL_VOCALIZE
    assert dict(x.request) == {'arousal': 0.8, 'valence': 0.6, 'length_s': 1.5, 'seed': 42,
                               'gain_db': 0.0, 'priority': 199, 'owner': OWNER,
                               'intent_id': 'v1'}
    (x,) = sintent(a, 2.0, 'manual', 199, action='vocalize', params={},
                   interruptible=True).actions
    assert (x.request['arousal'], x.request['valence'], x.request['length_s'],
            x.request['seed']) == (0.5, 0.5, 0.0, 0)
    for params in ({'arousal': 1.5}, {'seed': 2 ** 40}, {'length_s': -1}):
        assert not sintent(a, 3.0, 'manual', 199, action='vocalize', params=params,
                           interruptible=True).accepted


def test_rule21_a_phrase_of_the_personality_is_capped_at_max_duration():
    a = sound(catalogue=sound_catalogue(sound_list(synth=True)))
    for params, duration, length in (({}, 0, 0.0), ({'length_s': 99}, 0, 30.0),
                                     ({'length_s': 1.5}, 0, 1.5), ({}, 800, 0.8),
                                     ({'length_s': 1.5}, 800, 0.8), ({}, 99000, 30.0)):
        (x,) = sintent(a, 1.0, 'behavior', 100, action='vocalize', params=params,
                       duration_ms=duration, interruptible=True).actions
        assert x.request['length_s'] == length, (params, duration)


def test_rule20_a_phrase_takes_the_moods_vector_unless_the_intent_gives_one():
    a = sound(catalogue=sound_catalogue(sound_list(synth=True)))
    collect(a.mood('calm', 1.0, 1.0, 0.6, -0.2))          # -1..1 -> 0..1
    (x,) = sintent(a, 1.0, 'manual', 199, action='vocalize', interruptible=True).actions
    assert (x.request['arousal'], x.request['valence']) == pytest.approx((0.8, 0.4))
    (x,) = sintent(a, 2.0, 'manual', 199, action='vocalize', params={'valence': 1.0},
                   interruptible=True).actions
    assert (x.request['arousal'], x.request['valence']) == pytest.approx((0.8, 1.0))
    collect(a.mood('calm', 1.0, 3.0, float('nan'), 0.0))   # a broken axis keeps its last
    collect(a.mood('calm', 1.0, 3.0, 5.0, None))
    (x,) = sintent(a, 3.0, 'manual', 199, action='vocalize', interruptible=True).actions
    assert (x.request['arousal'], x.request['valence']) == pytest.approx((1.0, 0.5))
    a.sound_state(sound_snap(), 10.0)                       # stale: the middle voice
    (x,) = sintent(a, 10.0, 'manual', 199, action='vocalize', interruptible=True).actions
    assert (x.request['arousal'], x.request['valence']) == (0.5, 0.5)


def test_rule21_a_phrase_of_its_own_length_is_capped_by_a_short_max_duration():
    data = sound_data()
    data['defaults']['max_duration_ms'] = 1000
    a = sound(config=parse_config(data), catalogue=sound_catalogue(sound_list(synth=True)))
    (x,) = sintent(a, 1.0, 'behavior', 100, action='vocalize', interruptible=True).actions
    assert x.request['length_s'] == 1.0
    (x,) = sintent(a, 2.0, 'manual', 199, action='vocalize', interruptible=True).actions
    assert x.request['length_s'] == 0.0


def test_rule21_equal_priority_waits_for_a_run_that_did_not_say_interruptible():
    a = sound()
    first = sintent(a, 1.0, 'idle', 10, intent_id='i1')                  # silent: the default
    accept_all(a, first.actions, 1.0)
    r = sintent(a, 1.5, 'behavior', 10, params={'name': 'yay'}, intent_id='b1')
    assert not r.accepted and 'non-interruptible' in r.reason and r.actions == ()
    assert a.status('sound', 1.5).dropped == 1
    assert a.status('sound', 1.5).active_intent_id == 'i1'
    r = sintent(a, 1.6, 'manual', 199, params={'name': 'yay'}, intent_id='m1')   # higher
    assert r.accepted and [x.call for x in r.actions] == [CALL_PLAY]    # the player fades i1
    assert a.status('sound', 1.6).active_intent_id == 'm1'


def test_rule21_a_lower_priority_never_replaces_a_run():
    a = sound()
    sintent(a, 1.0, 'behavior', 100, intent_id='b1', interruptible=True)
    r = sintent(a, 1.5, 'idle', 10, intent_id='i1', interruptible=True)
    assert not r.accepted and 'running at priority 100' in r.reason


def test_rule21_a_run_that_said_interruptible_is_replaced_by_the_newer_at_equal_priority():
    a = sound()
    sintent(a, 1.0, 'idle', 10, intent_id='i1', interruptible=True)
    r = sintent(a, 1.5, 'behavior', 10, params={'name': 'yay'}, intent_id='b1')
    assert r.accepted and plays(r.actions)[0].request['name'] == 'yay'
    assert a.status('sound', 1.5).active_intent_id == 'b1'


def test_rule21_the_default_for_an_intent_without_the_field_is_the_configs():
    a = sound(sound_channel_cfg(sound={'interruptible': True}))
    sintent(a, 1.0, 'idle', 10, intent_id='i1')
    assert sintent(a, 1.5, 'behavior', 10, intent_id='b1').accepted
    a = sound()
    assert sintent(a, 1.0, 'idle', 10, intent_id='i1').accepted
    assert not sintent(a, 1.5, 'behavior', 10, intent_id='b1').accepted
    # a message always has the field: false is explicit, and the other channels default true
    msg = type('Msg', (), {'channel': 'sound', 'source': 'idle', 'priority': 10,
                           'action': 'play', 'params': '{"name": "sent1"}', 'duration_ms': 0,
                           'interruptible': False, 'intent_id': 'm'})()
    assert a.intent(msg, 3.0).accepted is False                     # i1 plays on: it waits


def test_rule21_idle_and_behavior_plays_carry_min_of_clip_length_and_the_cap():
    for source, priority in (('idle', 10), ('behavior', 100)):
        for params, want in [({'name': 'sent1'}, 1500), ({'name': 'long1'}, 30000),
                             ({'pool': 'hum'}, 30000)]:
            a = sound()
            (x,) = sintent(a, 1.0, source, priority, params=params).actions
            assert x.request['duration_ms'] == want, (source, params)
    a = sound()
    (x,) = sintent(a, 1.0, 'idle', 10, params={'name': 'sent2'}, duration_ms=1000).actions
    assert x.request['duration_ms'] == 1000
    a = sound()
    (x,) = sintent(a, 1.0, 'idle', 10, params={'name': 'sent2'}, duration_ms=9000).actions
    assert x.request['duration_ms'] == 2500              # not past the clip
    a = sound(sound_channel_cfg(max_duration_ms=1000))
    (x,) = sintent(a, 1.0, 'idle', 10, params={'name': 'sent2'}).actions
    assert x.request['duration_ms'] == 1000
    assert a.status('sound', 1.0).active_remaining_ms == 1000


def test_rule21_a_manual_play_is_not_capped_and_zero_is_the_clips_length():
    a = sound()
    (x,) = sintent(a, 1.0, 'manual', 199, params={'name': 'long1'}).actions
    assert x.request['duration_ms'] == 0
    assert a.status('sound', 1.0).active_remaining_ms == 120000      # the clip's length
    a = sound()
    (x,) = sintent(a, 1.0, 'manual', 199, params={'name': 'long1'}, duration_ms=60000).actions
    assert x.request['duration_ms'] == 60000
    assert a.status('sound', 1.0).active_remaining_ms == 60000
    a = sound()
    (x,) = sintent(a, 1.0, 'manual', 199, params={'pool': 'hum'}).actions
    assert x.request['duration_ms'] == 0 and a.status('sound', 1.0).active_remaining_ms == 0


def test_rule21_the_personality_never_plays_an_excluded_clip_by_name():
    a = sound()
    r = sintent(a, 1.0, 'behavior', 100, params={'name': 'cantina'})
    assert not r.accepted and 'manual play only' in r.reason


def test_rule24_a_remaining_ms_of_zero_does_not_end_a_run_early():
    a = sound()
    r = sintent(a, 1.0, 'manual', 199, params={'name': 'long1'}, intent_id='m1')
    accept_all(a, r.actions, 1.0)
    a.sound_state(playing('long1', 'm1', 0, 199), 1.5)
    a.tick(1.6)
    assert a.status('sound', 1.6).state == STATE_RUNNING


def test_rule21_the_cantina_is_in_no_pool_and_plays_by_name_only():
    assert all('cantina' not in names for names in SOUND_CAT.pools.values())
    assert 'excluded' not in SOUND_CAT.pools


def test_rule22_chatter_is_due_within_every_s_and_draws_from_the_moods_pools():
    a = sound(rng=FixedRng(0.0), strength=0.0)                 # calm: every 10 to 20 s
    assert plays(sstep(a, 9.5, strength=0.0)) == []
    (x,) = plays(sstep(a, 10.5, strength=0.0))
    assert (x.priority, x.why, x.request['name'], x.request['gain_db']) == (
        10, 'intent', 'sent1', -3.0)
    assert x.request['duration_ms'] == 1500 and x.intent_id.startswith('chatter-')
    a = sound(rng=FixedRng(0.999), strength=0.0)               # the last of the window
    assert plays(sstep(a, 19.5, strength=0.0)) == []
    (x,) = plays(sstep(a, 20.5, strength=0.0))
    assert x.request['name'] == 'hum1'                         # the last pool, by weight


def test_rule22_mood_strength_pulls_the_window_toward_its_lower_bound():
    a = sound(rng=FixedRng(0.999), strength=1.0)
    assert plays(sstep(a, 9.5)) == [] and len(plays(sstep(a, 10.5))) == 1
    a = sound(rng=FixedRng(0.999), strength=0.5)
    assert plays(sstep(a, 14.5, strength=0.5)) == [] and len(
        plays(sstep(a, 15.5, strength=0.5))) == 1


def test_rule22_chatter_is_an_idle_intent_through_rule_4():
    a = sound(rng=FixedRng(0.0))
    chatter = plays(sstep(a, 10.5))
    assert chatter
    assert a.status('sound', 10.5).active_source == 'idle'
    assert plays(sstep(a, 10.9)) == []                      # nothing while it plays


def test_rule22_no_chatter_while_a_play_runs_and_the_slot_is_not_kept():
    a = sound(rng=FixedRng(0.0))
    sintent(a, 1.0, 'behavior', 100, params={'name': 'long1'}, intent_id='b1')
    snap = playing('long1', 'b1', 100000, 100)
    assert plays(sstep(a, 12.5, snap=snap)) == []          # the slot at 10 passed under a play
    sintent(a, 12.6, 'behavior', 100, action='stop', params={})
    assert plays(sstep(a, 19.5)) == []                      # not made up afterwards
    assert len(plays(sstep(a, 20.5))) == 1                  # the next slot, 10 s on


def test_rule22_no_chatter_while_the_player_plays_something_else():
    a = sound(rng=FixedRng(0.0))
    assert plays(sstep(a, 10.5, snap=sound_snap(1, 'x', 'someone', 50, 'y', 5, 900))) == []


def test_rule22_no_chatter_while_claimed_and_none_after_the_release():
    a = sound(rng=FixedRng(0.0))
    assert a.claim('sound', 'op', 150, 30.0, 1.0).accepted
    assert plays(sstep(a, 12.5)) == []
    assert a.status('sound', 12.5).state == STATE_CLAIMED
    a.claim('sound', 'op', 150, 0.0, 12.6)
    assert plays(sstep(a, 15.0)) == []


def test_rule22_no_chatter_while_stale_and_it_resumes_with_the_mood():
    a = sound(rng=FixedRng(0.0))
    assert plays(sstep(a, 30.0, mood=None)) == []
    assert a.status('sound', 30.0).state == STATE_STALE
    sstep(a, 31.0)
    assert len(plays(sstep(a, 41.5))) >= 1                     # a fresh window from the mood


def test_rule22_no_chatter_while_offline():
    a = sound(rng=FixedRng(0.0))
    actions = []
    for t in range(1, 30):                                      # moods and ticks, no state
        actions += a.mood('calm', 1.0, float(t)) + a.tick(float(t))
    assert plays(actions) == [] and a.status('sound', 29.0).state == STATE_OFFLINE


def test_rule22_a_voice_pick_vocalizes_the_moods_arousal_and_valence():
    data = sound_data()
    data['moods']['calm']['sound']['chatter']['pools'] = {'voice': 1}
    a = sound(config=parse_config(data), rng=FixedRng(0.0), strength=0.0,
              catalogue=sound_catalogue(sound_list(synth=True)))
    collect(a.mood('calm', 0.0, 0.0, 0.5, 0.8))
    out = sstep(a, 10.5, strength=0.0)
    assert plays(out) == []
    (x,) = [x for x in out if x.call == CALL_VOCALIZE]
    assert (x.priority, x.why, x.request['gain_db'], x.request['length_s'],
            x.request['seed']) == (10, 'intent', -3.0, 0.0, 0)           # calm's gain
    assert (x.request['arousal'], x.request['valence']) == pytest.approx((0.75, 0.9))
    assert x.intent_id.startswith('chatter-')
    assert a.status('sound', 10.5).active_source == 'idle'


def test_rule22_a_mood_without_a_voice_has_no_chatter():
    data = sound_data()
    del data['moods']['calm']['sound']
    a = sound(parse_config(data), rng=FixedRng(0.0))
    assert plays(sstep(a, 60.0)) == []


def test_rule8_a_pool_with_no_playable_clip_is_dropped_with_one_warning():
    a = sound()
    warnings = a.take_warnings()
    assert sum("tagged 'laugh'" in w for w in warnings) == 1
    assert not any('moods.calm' in w for w in warnings)
    a.set_catalogue(SOUND_CAT, 1.0)                              # listed again: not repeated
    assert a.take_warnings() == []
    a = sound(rng=FixedRng(0.0), mood='excited')                # excited: excited 4, chatter 1
    (x,) = plays(sstep(a, 5.5, mood='excited'))
    assert x.request['name'] == 'yay'


def test_rule8_a_mood_with_no_pool_left_falls_back_to_chatter_with_one_warning():
    a = sound(rng=FixedRng(0.0), mood='scared')
    warnings = a.take_warnings()
    assert sum("falling back to 'chatter'" in w for w in warnings) == 1
    assert sum('moods.scared' in w for w in warnings) == 3         # sad, ooh, the fallback
    (x,) = plays(sstep(a, 8.5, mood='scared'))
    assert x.request['name'] in ('sent1', 'sent2')


def test_rule8_a_catalogue_without_the_fallback_pool_leaves_the_mood_silent_with_a_warning():
    clips = [c for c in SOUND_CLIPS if 'chatter' not in c['tags']]
    a = sound(rng=FixedRng(0.0), mood='scared', catalogue=sound_catalogue(sound_list(
        clips=clips)))
    assert any('is silent' in w for w in a.take_warnings())
    assert plays(sstep(a, 60.0, mood='scared')) == []


def test_rule23_stale_stops_an_idle_play_but_not_a_behavior_or_manual_one():
    for source, priority, stopped in (('idle', 10, True), ('behavior', 100, False),
                                      ('manual', 199, False)):
        a = sound()
        r = sintent(a, 1.0, source, priority, params={'name': 'long1'}, intent_id='p1')
        accept_all(a, r.actions, 1.0)
        snap = playing('long1', 'p1', 100000, priority)
        out = sstep(a, 6.0, snap=snap, mood=None)
        if stopped:
            (x,) = stops(out)
            assert (x.why, x.priority, dict(x.request)) == ('stale', 10, {'owner': OWNER})
            assert a.status('sound', 6.0).active_action == ''
        else:
            assert stops(out) == [] and a.status('sound', 6.0).active_action == 'play'


def test_rule23_manual_still_plays_while_stale():
    a = sound()
    sstep(a, 10.0, mood=None)
    assert a.status('sound', 10.0).state == STATE_STALE
    assert sintent(a, 10.0, 'manual', 199, params={'name': 'sent1'}).accepted
    r = sintent(a, 10.0, 'idle', 10)
    assert not r.accepted and 'stale' in r.reason


def test_rule24_the_play_the_player_shows_is_counted_down_from_its_remaining_ms():
    a = sound()
    r = sintent(a, 1.0, 'manual', 199, params={'name': 'sent2'}, intent_id='m1')
    accept_all(a, r.actions, 1.0)
    a.sound_state(playing('sent2', 'm1', 2000, 199), 1.2)
    assert a.status('sound', 1.2).active_remaining_ms == 2000
    a.sound_state(playing('sent2', 'm1', 1500, 199), 1.7)
    status = a.status('sound', 1.7)
    assert (status.state, status.active_remaining_ms, status.owner, status.priority) == (
        STATE_RUNNING, 1500, 'manual', 199)


def test_rule24_a_play_that_ended_on_the_device_ends_the_run():
    a = sound()
    r = sintent(a, 1.0, 'manual', 199, params={'name': 'long1'}, intent_id='m1')
    accept_all(a, r.actions, 1.0)
    a.sound_state(sound_snap(0, last_end=1), 1.5)           # inside the grace: may predate it
    assert a.status('sound', 1.5).active_action == 'play'
    a.sound_state(sound_snap(0, last_end=1), 2.6)
    assert a.status('sound', 2.6).active_action == '' and a.status('sound', 2.6).state in (
        STATE_IDLE, STATE_CLAIMED)


def test_rule24_a_state_naming_another_intent_does_not_keep_our_run_alive():
    a = sound()
    r = sintent(a, 1.0, 'manual', 199, params={'name': 'long1'}, intent_id='m1')
    accept_all(a, r.actions, 1.0)
    a.sound_state(playing('long1', 'other', 5000, 199), 2.6)
    assert a.status('sound', 2.6).active_action == ''


def test_rule24_a_play_of_our_owner_found_on_the_first_state_is_adopted():
    a = Arbiter(sound_cfg(), SOUND_CHANNELS, rng=FixedRng(0.0))
    a.set_catalogue(SOUND_CAT, 0.0)
    a.sound_state(playing('sent2', 'old-id', 2000), 0.0)
    a.mood('calm', 1.0, 0.0)
    status = a.status('sound', 0.0)
    assert (status.state, status.active_source, status.priority) == (STATE_RUNNING, 'adopted', 0)
    assert status.active_remaining_ms == 2000
    a.sound_state(playing('sent2', 'old-id', 800), 1.2)
    assert a.status('sound', 1.2).active_remaining_ms == 800
    a.sound_state(sound_snap(), 3.0)
    assert a.status('sound', 3.0).state == STATE_IDLE


def test_rule24_a_play_of_another_owner_is_not_adopted():
    a = Arbiter(sound_cfg(), SOUND_CHANNELS, rng=FixedRng(0.0))
    a.set_catalogue(SOUND_CAT, 0.0)
    a.sound_state(playing('sent2', 'x', 2000, owner='someone'), 0.0)
    a.mood('calm', 1.0, 0.0)
    assert a.status('sound', 0.0).active_action == ''


def test_rule24_no_catalogue_or_no_state_is_offline_and_a_run_ends():
    a = Arbiter(sound_cfg(), SOUND_CHANNELS)
    assert a.status('sound', 0.0).state == STATE_OFFLINE
    a.set_catalogue(SOUND_CAT, 0.0)
    assert a.status('sound', 0.0).state == STATE_OFFLINE           # no state yet
    a.sound_state(sound_snap(), 0.0)
    a.mood('calm', 1.0, 0.0)
    assert a.status('sound', 0.0).state == STATE_IDLE
    r = a.intent(Intent('sound', 'manual', 199, 'play', {'name': 'long1'}), 1.0)
    assert r.accepted and a.status('sound', 1.0).state == STATE_RUNNING
    a.mood('calm', 1.0, 7.0)
    a.tick(7.0)                                                   # 6 s without a state
    assert a.status('sound', 7.0).state == STATE_OFFLINE
    r = a.intent(Intent('sound', 'behavior', 100, 'play', {'name': 'sent1'}), 7.0)
    assert not r.accepted and r.reason == 'channel is offline'
    assert a.status('sound', 7.0).dropped == 1
    a.sound_state(sound_snap(), 8.0)                              # the player is back
    status = a.status('sound', 8.0)
    assert status.active_action == '' and status.state in (STATE_IDLE, STATE_CLAIMED)


def test_claims_a_claim_stops_the_personalitys_play_and_refuses_an_idle_one():
    a = sound()
    r = sintent(a, 1.0, 'idle', 10, params={'name': 'long1'}, intent_id='i1')
    accept_all(a, r.actions, 1.0)
    result = a.claim('sound', 'op', 150, 30.0, 2.0)
    assert result.accepted
    (x,) = collect(result.actions)
    assert (x.call, x.why, x.priority, x.intent_id, dict(x.request)) == (
        CALL_SOUND_STOP, 'preempt', 10, 'i1', {'owner': OWNER})
    assert a.status('sound', 2.0).active_action == ''
    r = sintent(a, 2.5, 'idle', 10, intent_id='i2')
    assert not r.accepted and 'claimed by' in r.reason
    assert not sintent(a, 2.5, 'behavior', 100, intent_id='b2').accepted     # below 150
    assert a.status('sound', 2.5).dropped == 2
    assert sintent(a, 2.5, 'manual', 199, intent_id='m2').accepted


def test_stop_a_manual_stop_stops_the_sound_and_an_idle_one_with_nothing_running_does_not():
    a = sound()
    r = sintent(a, 1.0, 'idle', 10, intent_id='i1')
    accept_all(a, r.actions, 1.0)
    (x,) = sintent(a, 2.0, 'manual', 199, action='stop').actions
    assert (x.call, x.why, x.priority, dict(x.request)) == (
        CALL_SOUND_STOP, 'intent', 199, {'owner': OWNER})
    assert a.status('sound', 2.0).active_action == ''
    assert len(sintent(a, 4.0, 'manual', 199, action='stop').actions) == 1   # a person: always
    a = sound()
    r = sintent(a, 3.0, 'idle', 10, action='stop')
    assert r.accepted and r.actions == () and r.reason == 'nothing running'


def test_a_play_the_player_refuses_ends_the_run_without_a_hold():
    a = sound()
    r = sintent(a, 1.0, 'manual', 199, params={'pool': 'hum'}, intent_id='m1')
    (x,) = r.actions
    assert a.device_reply(x, False, 'empty pool', 1.0).endswith('failed: empty pool')
    status = a.status('sound', 1.0)
    assert status.active_action == '' and status.refused == 0


def test_a_player_never_holds_the_sound_channel():
    a = sound()
    (x,) = sintent(a, 1.0, 'manual', 199, params={'name': 'sent1'}).actions
    a.device_reply(x, False, 'held by console', 1.0)
    assert a.status('sound', 1.0).state != STATE_FOREIGN


def test_reload_keeps_the_play_and_follows_the_new_voice():
    a = sound()
    r = sintent(a, 1.0, 'behavior', 100, params={'name': 'long1'}, intent_id='m1',
                interruptible=True)
    accept_all(a, r.actions, 1.0)
    data = sound_data()
    data['moods']['calm']['sound']['gain_db'] = -9
    collect(a.reload(parse_config(data), 2.0))
    assert a.status('sound', 2.0).active_intent_id == 'm1'
    (x,) = plays(sintent(a, 2.0, 'behavior', 100, params={'name': 'sent1'},
                         interruptible=True).actions)
    assert x.request['gain_db'] == -9.0


# --- the sweeps (keep last: they read what every test above produced) -----------------------

def _approaches(sent):
    count, previous = 0, 0.0
    for v in speeds(sent):
        count += bool(v and not previous)
        previous = v
    return count


def test_dome_the_learned_coast_is_carried_to_the_next_move():
    a = dome(learning_cfg())                                           # an ideal plant: no coast
    assert a.coast_scale == 1.0
    first = list(dome_intent(a, 'look_at', {'bearing': 1.5, 'profile': 'smooth'}).actions)
    first += live(a, 6.0)
    assert not a.moving and a.coast_scale < 0.1
    second = list(dome_intent(a, 'look_at', {'bearing': 0.2, 'profile': 'smooth'}).actions)
    second += live(a, 12.0)
    assert not a.moving and a.test_plant.position == pytest.approx(-0.2, abs=0.05)
    assert (_approaches(first), _approaches(second)) == (2, 1)   # no creep the second time
    assert a.coast_scale < 0.1


def test_dome_coast_learning_is_off_by_default_and_the_scale_stays_1():
    a = dome()                                           # an ideal plant: a lesson would say 0
    dome_intent(a, 'look_at', {'bearing': 1.5, 'profile': 'smooth'})
    live(a, 6.0)
    assert not a.moving and a.coast_scale == 1.0


def test_dome_a_move_stopped_before_it_learned_leaves_the_carried_coast_alone():
    a = dome(learning_cfg())
    dome_intent(a, 'look_at', {'bearing': 1.5, 'profile': 'smooth'})
    live(a, 6.0)
    learned = a.coast_scale
    dome_intent(a, 'turn', {'delta': 3.0})
    live(a, 6.5)
    dome_intent(a, 'stop')
    assert not a.moving and a.coast_scale == pytest.approx(learned, abs=0.01)


def test_dome_the_default_config_gives_the_motion_models_default_limits():
    assert Arbiter._dome_limits(dome_cfg()) == motion.Limits()     # one set of defaults


def test_dome_a_replacement_carries_its_predecessors_learned_coast_to_the_memory():
    a = dome(learning_cfg())
    dome_intent(a, 'glance', {'amplitude': 1.0, 'dwell_s': 3.0, 'profile': 'smooth'})
    move = a._ch['dome'].run.move
    while not move.coast_learned:
        live(a, a.test_t + 0.05)
    learned = move.coast_scale
    here = -a.test_plant.position                        # bearing of where it is: there already
    dome_intent(a, 'look_at', {'bearing': here, 'profile': 'smooth'})
    live(a, a.test_t + 2.0)
    assert not a.moving and a.coast_scale == pytest.approx(learned, abs=0.05)


@pytest.mark.parametrize('how', ['claim', 'stop'])
def test_dome_a_move_interrupted_by_a_claim_or_a_stop_teaches_the_coast_nothing(how):
    a = dome(learning_cfg())                             # an ideal plant: a fresh move learns
    dome_intent(a, 'glance', {'amplitude': 1.0, 'dwell_s': 3.0, 'profile': 'smooth'})
    move = a._ch['dome'].run.move
    while not move.coast_learned:
        live(a, a.test_t + 0.05)
    assert a.moving and move.coast_scale < 0.5           # in its dwell, having learned
    if how == 'claim':
        feed(a, a.claim('dome', 'console', 250, 30.0, a.test_t).actions)
    else:
        feed(a, dome_intent(a, 'stop').actions)
    assert not a.moving and a.coast_scale == 1.0


def test_rule14_every_dome_move_ends_with_a_0_or_is_replaced():
    assert len(DOME_ARBITERS) > 20
    for a in DOME_ARBITERS:
        log = a.test_log
        for x, after in zip(log, log[1:]):
            if x.request['velocity_rad_s'] != 0.0 and after.intent_id != x.intent_id:
                assert after.request['velocity_rad_s'] == 0.0 or after.why == 'intent', (
                    x, after)
        if not a.moving and log:
            assert log[-1].request['velocity_rad_s'] == 0.0


def test_rule4_every_dome_velocity_is_0_or_between_min_speed_and_the_cap():
    for a in DOME_ARBITERS:
        for x in a.test_log:
            assert x.request['velocity_rad_s'] == 0.0 or MIN_SPEED <= abs(
                x.request['velocity_rad_s']) <= 2.0


def test_rule6_every_outgoing_request_carries_owner_and_a_nonzero_priority():
    assert len(ALL_ACTIONS) > 50
    for x in ALL_ACTIONS:
        assert x.owner == OWNER and 1 <= x.priority <= 255
        if 'owner' in x.request:
            assert x.request['owner'] == OWNER
        if x.call not in (CALL_MAGIC_BRIGHTNESS, CALL_VELOCITY, CALL_SOUND_STOP):   # none
            assert x.request['priority'] == x.priority
    assert {x.call for x in ALL_ACTIONS} >= {CALL_RUN, CALL_SAY, CALL_STOP, CALL_CONFIGURE,
                                             CALL_START, CALL_MAGIC_STOP,
                                             CALL_MAGIC_BRIGHTNESS, CALL_PLAY,
                                             CALL_SOUND_STOP}


def test_rule9_no_outgoing_request_sets_hue():
    for x in ALL_ACTIONS:
        assert 'hue' not in x.request
        if x.call == CALL_CONFIGURE:
            assert not x.request['apply'] & APPLY_HUE
            assert x.request['apply']


def test_rule3_no_personality_run_has_duration_zero():
    for x in ALL_ACTIONS:
        if x.call in (CALL_RUN, CALL_SAY, CALL_PLAY) and x.priority <= 100:
            assert x.request['duration_ms'] > 0
