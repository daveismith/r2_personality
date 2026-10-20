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

import pytest

from fixtures import base_data, cfg, FixedRng, held, LED_CAT, led_snap  # noqa: I100
from fixtures import MAGIC_CAT, magic_snap, OWNER, running
from r2_expression.config import parse_config
from r2_expression.core import (Action, APPLY_BRIGHTNESS, APPLY_DRIFT, APPLY_HUE, APPLY_MODE,
                                Arbiter, CALL_CONFIGURE, CALL_MAGIC_BRIGHTNESS,
                                CALL_MAGIC_STOP, CALL_RUN, CALL_SAY, CALL_START, CALL_STOP,
                                channel_of, held_by, Intent, LED_CHANNELS, Status)
from r2_expression.core import (STATE_CLAIMED, STATE_DARK, STATE_FOREIGN, STATE_IDLE,
                                STATE_OFFLINE, STATE_RUNNING, STATE_STALE)

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
    assert channel_of('dome') is None
    assert channel_of('tfld').overlaps(channel_of('fld'))
    assert not channel_of('tfld').overlaps(channel_of('bfld'))


def test_arbiter_serves_one_channel_group():
    with pytest.raises(ValueError):
        Arbiter(cfg(), ('fld', 'magic'))
    with pytest.raises(ValueError):
        Arbiter(cfg(), ('fld', 'dome'))
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
    assert r.actions[0].request == {'sequence_id': 44, 'repeat': 1, 'end': 0,
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
    r = intent(a, 1.0, channel='magic', action='sequence', params={'id': 27, 'repeat': 0},
               duration_ms=3000)
    accept_all(a, r.actions, 1.0)
    run = magic_snap(state=1, started_by='ros', owner=OWNER, sequence_id=27)
    stop = step(a, 4.0, snap=run)[0]
    a.device_reply(stop, False, 'held by console', 4.0)
    assert CALL_MAGIC_STOP not in [x.call for x in step(a, 8.9, snap=run)]
    assert calls(step(a, 9.0, snap=run)) == [(CALL_MAGIC_STOP, 'magic')]


def test_rule3_idle_magic_sequence_without_duration_gets_the_cap():
    a = magic()
    intent(a, 1.0, channel='magic', action='sequence', params={'name': 'Random show'})
    assert a.status('magic', 1.0).active_remaining_ms == 30000


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
                   action='sequence', params={'id': 23, 'repeat': 0}, duration_ms=10000)
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


# --- the sweeps (keep last: they read what every test above produced) -----------------------

def test_rule6_every_outgoing_request_carries_owner_and_a_nonzero_priority():
    assert len(ALL_ACTIONS) > 50
    for x in ALL_ACTIONS:
        assert x.owner == OWNER and 1 <= x.priority <= 255
        if 'owner' in x.request:
            assert x.request['owner'] == OWNER
        if x.call not in (CALL_MAGIC_BRIGHTNESS,):
            assert x.request['priority'] == x.priority
    assert {x.call for x in ALL_ACTIONS} >= {CALL_RUN, CALL_SAY, CALL_STOP, CALL_CONFIGURE,
                                             CALL_START, CALL_MAGIC_STOP,
                                             CALL_MAGIC_BRIGHTNESS}


def test_rule9_no_outgoing_request_sets_hue():
    for x in ALL_ACTIONS:
        assert 'hue' not in x.request
        if x.call == CALL_CONFIGURE:
            assert not x.request['apply'] & APPLY_HUE
            assert x.request['apply']


def test_rule3_no_personality_run_has_duration_zero():
    for x in ALL_ACTIONS:
        if x.call in (CALL_RUN, CALL_SAY) and x.priority <= 100:
            assert x.request['duration_ms'] > 0
