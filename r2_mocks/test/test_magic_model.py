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

"""magic_model against plan 2.4-2.5: one test per rule, refusal strings quoted verbatim."""

import copy
import os

import pytest
from r2_mocks import magic_model as mg
from r2_mocks.holds import HOLD_FOREVER
from r2_mocks.led_model import load_catalogue
from r2_mocks.magic_model import MagicModel

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config', 'dome_mock.yaml')
CATALOGUE = load_catalogue(CONFIG)


def make(**kwargs):
    return MagicModel(copy.deepcopy(CATALOGUE), **kwargs)


@pytest.fixture
def p():
    return make()


def polled(model, now):
    model.poll(now)
    return model.state(now)


# --- catalogue --------------------------------------------------------------------------

def test_catalogue_has_the_58_sequences_of_v012_1_with_names(p):
    descriptor, sequences = p.list_catalogue(5.0)
    assert [s['id'] for s in sequences] == list(range(58))
    assert sequences[44]['name'] == 'Smile' and sequences[0]['name'] == 'All off'
    assert (descriptor['link'], descriptor['seq_count'], descriptor['has_repeat']) == (1, 58, True)
    assert (descriptor['fw_major'], descriptor['fw_minor'], descriptor['fw_patch']) == (0, 12, 1)


def test_catalogue_is_empty_and_never_aged_when_the_panel_is_not_fitted():
    descriptor, sequences = make(fitted=False).list_catalogue(5.0)
    assert sequences == [] and descriptor['link'] == mg.LINK_ABSENT
    assert descriptor['catalogue_age_ms'] == mg.NEVER


def test_names_are_empty_without_cap_names():
    _, sequences = make(caps={'has_names': False}).list_catalogue(0)
    assert {s['name'] for s in sequences} == {''}


# --- start ------------------------------------------------------------------------------

def test_start_confirms_and_moves_run_counter_by_one(p):
    first = p.start(0, 44, owner='r2_expression', priority=10)
    assert first == {'accepted': True, 'reason': '', 'confirmed': True, 'run_counter': 1}
    assert p.start(1, 46)['run_counter'] == 2


def test_run_counter_wraps_at_256(p):
    for i in range(256):
        reply = p.start(i, 0)
    assert reply['run_counter'] == 0
    assert p.start(300, 0)['run_counter'] == 1


def test_start_replaces_a_running_sequence_without_refusing(p):
    p.start(0, 1, repeat=0)
    assert p.start(1, 3)['accepted']
    assert polled(p, 2)['sequence_id'] == 3


def test_unknown_sequence(p):
    assert p.start(0, 99) == {'accepted': False, 'reason': 'unknown sequence',
                              'confirmed': False, 'run_counter': 0}


def test_out_of_range_end(p):
    assert p.start(0, 3, end=2)['reason'] == 'out of range'


def test_repeat_without_cap_repeat_is_not_supported_by_this_panel():
    old = make(caps={'has_repeat': False})
    assert old.start(0, 3, repeat=2)['reason'] == 'not supported by this panel'
    assert old.start(0, 3, repeat=0)['reason'] == 'not supported by this panel'
    assert old.start(0, 3, repeat=1)['accepted']


def test_start_when_the_panel_is_not_fitted():
    assert make(fitted=False).start(0, 3)['reason'] == 'not fitted'


def test_start_when_the_firmware_is_too_old():
    old = make(caps={'unsupported': 1})
    assert old.start(0, 3)['reason'] == 'firmware too old'
    assert old.list_catalogue(0)[1] == []


def test_checks_run_in_order_link_then_sequence_then_range_then_support():
    assert make(fitted=False).start(0, 99, end=5)['reason'] == 'not fitted'
    assert make().start(0, 99, end=5)['reason'] == 'unknown sequence'
    assert make(caps={'has_repeat': False}).start(0, 3, repeat=3, end=5)['reason'] == \
        'out of range'


def test_owner_too_long(p):
    assert p.start(0, 3, owner='x' * 32)['reason'] == 'owner too long'
    assert p.start(0, 3, owner='x' * 31)['accepted']


# --- running, ending ---------------------------------------------------------------------

def test_idle_panel_reports_seq_none(p):
    s = polled(p, 1)
    assert (s['state'], s['sequence_id'], s['source'], s['started_by']) == (
        mg.STATE_IDLE, mg.SEQ_NONE, mg.SOURCE_NONE, '')


def test_timed_sequence_ends_by_its_length_with_state_complete(p):
    p.start(0, 3, repeat=1)                         # On 5s, 5015 ms
    s = polled(p, 2)
    assert (s['state'], s['elapsed_ms'], s['remaining_known'], s['remaining_ms']) == (
        mg.STATE_RUNNING, 2000, True, 3015)
    s = polled(p, 6)
    assert (s['state'], s['elapsed_ms'], s['remaining_known']) == (mg.STATE_COMPLETE, 5015, False)


def test_repeat_multiplies_the_length_and_iterations_count(p):
    p.start(0, 3, repeat=3)
    s = polled(p, 7)
    assert (s['state'], s['iteration'], s['repeat'], s['remaining_ms']) == (
        mg.STATE_RUNNING, 1, 3, 3 * 5015 - 7000)
    assert polled(p, 16)['state'] == mg.STATE_COMPLETE
    assert polled(p, 16)['iteration'] == 3


def test_repeat_zero_runs_forever_with_no_remaining(p):
    p.start(0, 3, repeat=0)
    s = polled(p, 3600)
    assert (s['state'], s['remaining_known']) == (mg.STATE_RUNNING, False)


def test_loop_sequences_run_until_stopped_and_ignore_repeat(p):
    p.start(0, 56, repeat=2)
    assert polled(p, 10000)['state'] == mg.STATE_RUNNING
    p.start(0, 57, repeat=1)
    assert polled(p, 10000)['state'] == mg.STATE_RUNNING


def test_random_show_alternates_patterns_and_dark_pauses(p):
    p.start(0, 56)
    s = polled(p, 5)
    assert s['in_pause'] is False and s['sub_seq'] != mg.SUB_SEQ_OFF
    assert s['state'] == mg.STATE_RUNNING and s['remaining_known'] is False
    s = polled(p, 30)
    assert (s['in_pause'], s['sub_seq'], s['state']) == (True, mg.SUB_SEQ_OFF, mg.STATE_RUNNING)
    s = polled(p, 75)
    assert s['in_pause'] is False and s['sub_seq'] != mg.SUB_SEQ_OFF
    assert polled(p, 120)['in_pause'] is True            # the pause is about a minute long


def test_the_dark_pause_lasts_about_a_minute(p):
    p.start(0, 57)
    dark = [t for t in range(0, 70) if polled(p, t)['in_pause']]
    assert dark[0] == 10 and len(dark) == 60


def test_state_advances_only_on_poll(p):
    p.start(0, 3)
    assert p.state(3)['state'] == mg.STATE_IDLE           # not polled since the start
    s = polled(p, 3)
    assert s['state'] == mg.STATE_RUNNING and s['elapsed_ms'] == 3000
    s = p.state(4.5)                                      # same reading, older
    assert s['elapsed_ms'] == 3000 and s['read_age_ms'] == 1500


def test_poll_counter_moves_and_have_status_needs_a_poll(p):
    p.poll(0)
    p.poll(1)
    assert p.state(1)['polls'] == 2 and p.state(1)['have_status'] is True


def test_not_fitted_state_has_no_status():
    q = make(fitted=False)
    q.poll(0)
    s = q.state(0)
    assert (s['link'], s['have_status'], s['read_age_ms']) == (0, False, mg.NEVER)


# --- stop -------------------------------------------------------------------------------

def test_stop_blank_and_freeze_give_state_stopped_and_keep_attribution(p):
    p.start(0, 1, repeat=0, owner='r2_expression', priority=10)
    assert p.stop(2, mg.STOP_BLANK, priority=10) == {'accepted': True, 'reason': ''}
    s = polled(p, 9)
    assert (s['state'], s['elapsed_ms'], s['started_by'], s['owner'], s['ours']) == (
        mg.STATE_STOPPED, 2000, 'ros', 'r2_expression', True)
    p.start(10, 1, repeat=0)
    assert p.stop(12, mg.STOP_FREEZE)['accepted']
    assert polled(p, 20)['state'] == mg.STATE_STOPPED


def test_stopping_an_idle_panel_is_accepted(p):
    assert p.stop(0) == {'accepted': True, 'reason': ''}
    assert polled(p, 1)['state'] == mg.STATE_IDLE


def test_stop_mode_out_of_range(p):
    assert p.stop(0, 2)['reason'] == 'out of range'


def test_stopping_a_completed_run_leaves_it_complete(p):
    p.start(0, 3)
    p.stop(10)
    assert polled(p, 11)['state'] == mg.STATE_COMPLETE


# --- attribution ------------------------------------------------------------------------

def test_ros_start_reads_back_started_by_ros_owner_ours_source_and_end(p):
    p.start(0, 44, end=1, owner='r2_expression', priority=10)
    s = polled(p, 1)
    assert (s['started_by'], s['owner'], s['ours'], s['source'], s['sequence_id']) == (
        'ros', 'r2_expression', True, mg.SOURCE_I2C, 44)
    assert (s['end_known'], s['end'], s['run_counter'], s['takeovers']) == (True, 1, 1, 0)


@pytest.mark.parametrize('source', ['console', 'ui'])
def test_console_and_ui_starts_are_attributed_and_have_no_owner(p, source):
    p.start(0, 44, source=source, owner='ignored')
    s = polled(p, 1)
    assert (s['started_by'], s['owner'], s['ours']) == (source, '', True)


def test_rotary_is_a_gpio_run_the_controller_did_not_start(p):
    p.start(0, 3, owner='r2_expression')
    assert p.rotary(1, 5) == ''
    s = polled(p, 2)
    assert (s['source'], s['started_by'], s['owner'], s['ours'], s['end_known']) == (
        mg.SOURCE_GPIO, 'other', '', False, False)
    assert (s['takeovers'], s['run_counter'], s['rotary_code'], s['sequence_id']) == (1, 2, 5, 5)


def test_takeovers_show_on_the_next_poll_only(p):
    p.poll(0)
    p.rotary(0.5, 3)
    assert p.state(0.6)['takeovers'] == 0
    assert polled(p, 1)['takeovers'] == 1


# --- holds ------------------------------------------------------------------------------

@pytest.mark.parametrize('source', ['console', 'ui'])
def test_console_and_ui_start_hold_the_panel_for_hold_ms(p, source):
    p.start(0, 3, source=source)
    s = p.state(1)
    assert (s['held_by'], s['hold_remaining_ms'], s['manual']) == (source, 4000, False)
    assert p.start(2, 4, priority=10)['reason'] == 'held by ' + source
    assert p.stop(2, priority=10)['reason'] == 'held by ' + source
    assert p.start(5, 4, priority=10)['accepted']
    assert p.state(5)['held_by'] == ''


def test_console_stop_holds_and_renews(p):
    p.start(0, 1, repeat=0)
    p.stop(1, source='console')
    p.stop(4, source='ui')
    s = p.state(6)
    assert (s['held_by'], s['hold_remaining_ms']) == ('ui', 3000)


def test_priority_zero_and_200_plus_override_the_hold(p):
    p.start(0, 3, source='console')
    for priority in (0, 200, 255):
        assert p.start(1, 4, priority=priority)['accepted']
        assert p.stop(1, priority=priority)['accepted']


def test_ros_never_holds_and_never_ends_a_hold(p):
    p.start(0, 3, priority=10)
    assert p.state(0)['held_by'] == ''
    p.start(1, 3, source='console')
    p.start(2, 4, priority=250)
    assert p.state(2)['hold_remaining_ms'] == 4000


def test_hold_check_precedes_unknown_sequence_but_follows_not_fitted(p):
    p.start(0, 3, source='console')
    assert p.start(1, 99, priority=10)['reason'] == 'held by console'
    q = make(fitted=False)
    q.manual.set_on(True)
    assert q.start(0, 3, priority=10)['reason'] == 'not fitted'


def test_sticky_manual_holds_the_panel_forever(p):
    p.manual.set_on(True)
    s = p.state(1000)
    assert (s['manual'], s['held_by'], s['hold_remaining_ms']) == (True, 'manual', HOLD_FOREVER)
    assert p.start(1000, 3, priority=10)['reason'] == 'held: manual'
    assert p.stop(1000, priority=10)['reason'] == 'held: manual'
    assert p.start(1000, 3, priority=0)['accepted']
    p.manual.set_on(False)
    assert p.state(1000)['held_by'] == '' and p.start(1000, 3, priority=10)['accepted']


# --- configure --------------------------------------------------------------------------

def test_configure_brightness_live_and_range(p):
    reply = p.configure(mg.APPLY_BRIGHTNESS, brightness=3)
    assert reply['accepted'] and reply['result']['brightness'] == 3
    bad = p.configure(mg.APPLY_BRIGHTNESS | mg.APPLY_ORIENTATION, brightness=16, rotated=True)
    assert (bad['accepted'], bad['reason']) == (False, 'out of range')
    assert p.live['brightness'] == 3 and p.live['rotated'] is False


def test_configure_needs_the_capability():
    q = make(caps={'has_brightness': False})
    assert q.configure(mg.APPLY_BRIGHTNESS, brightness=3)['reason'] == \
        'not supported by this panel'


def test_configure_save_copies_live_to_saved_and_factory_restores(p):
    p.configure(mg.APPLY_BRIGHTNESS | mg.APPLY_SAVE, brightness=2)
    assert p.state(0)['saved']['brightness'] == 2 and p.state(0)['have_saved'] is True
    reply = p.configure(mg.APPLY_FACTORY, brightness=5)
    assert reply['saved'] and p.live == mg.FACTORY


def test_configure_is_not_gated_by_a_hold(p):
    p.manual.set_on(True)
    assert p.configure(mg.APPLY_BRIGHTNESS, brightness=1)['accepted']


def test_sequence_lookup_by_id_and_by_name(p):
    assert p.sequence_id('44') == 44 and p.sequence_id('smile') == 44
    assert p.sequence_id('Trace up line') == 9 and p.sequence_id('nope') is None


def test_catalogue_age_counts_from_the_first_poll_not_from_the_epoch(p):
    p.poll(1000.0)
    assert p.descriptor(1002.5)['catalogue_age_ms'] == 2500


def test_only_the_random_shows_have_patterns_and_pauses(p):
    p.start(0, 24, repeat=0)                     # Fade out/in: flagged random, not a show
    s = polled(p, 30)
    assert (s['sub_seq'], s['in_pause'], s['state']) == (0, False, mg.STATE_RUNNING)


def test_rotary_says_why_nothing_happened():
    q = make(fitted=False)
    assert q.rotary(0, 3) == 'not fitted'
    r = make()
    assert r.rotary(0, 99) == 'unknown sequence' and r.state(0)['rotary_code'] == 0
