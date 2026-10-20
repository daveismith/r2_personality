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

"""led_model against plan 2.2-2.4: one test per rule, refusal strings quoted verbatim."""

import copy
import os

import pytest
from r2_mocks import led_model as led
from r2_mocks.holds import HOLD_FOREVER
from r2_mocks.led_model import LedModel

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config', 'dome_mock.yaml')
CATALOGUE = led.load_catalogue(CONFIG)


@pytest.fixture
def m():
    return LedModel(copy.deepcopy(CATALOGUE))


def surface(model, name, now):
    return next(s for s in model.surface_states(now) if s['name'] == name)


def display(model, name, now):
    return next(d for d in model.display_states(now) if d['name'] == name)


# --- catalogue -------------------------------------------------------------------------

def test_catalogue_has_five_displays_six_surfaces_and_the_clips():
    assert [d['name'] for d in CATALOGUE['displays']] == ['rpsi', 'rld', 'bfld', 'tfld', 'fpsi']
    assert [s['name'] for s in CATALOGUE['surfaces']] == [
        'rpsi', 'rld', 'fld', 'bfld', 'tfld', 'fpsi']
    assert len(CATALOGUE['effects']) == 14 and len(CATALOGUE['clips']) == 48
    assert CATALOGUE['text_motions'] == ['bounce', 'loop', 'left', 'right', 'up']
    assert CATALOGUE['drift_presets'] == ['calm', 'lively']


# --- run semantics (2.3) -----------------------------------------------------------------

def test_run_starts_and_reports_attribution(m):
    assert m.run(0, 'fld', 'fire', owner='r2_expression', priority=10) == (True, '')
    s = surface(m, 'fld', 1)
    assert (s['running'], s['effect'], s['source'], s['owner']) == (
        True, 'fire', 'ros', 'r2_expression')
    assert s['elapsed_ms'] == 1000
    assert display(m, 'tfld', 1)['mode'] == led.MODE_EFFECT


def test_run_replaces_across_overlapping_displays_fld_and_tfld(m):
    m.run(0, 'fld', 'fire')
    m.run(1, 'tfld', 'rainbow')
    old = surface(m, 'fld', 1)
    assert not old['running'] and old['last_end'] == 'replaced' and old['last_effect'] == 'fire'
    m.run(2, 'fld', 'roam')
    assert not surface(m, 'tfld', 2)['running']
    assert surface(m, 'tfld', 2)['last_end'] == 'replaced'
    assert surface(m, 'fld', 2)['effect'] == 'roam'


def test_run_never_refuses_because_busy_and_leaves_other_surfaces_alone(m):
    m.run(0, 'rld', 'fire')
    assert m.run(1, 'fld', 'fire')[0]
    assert surface(m, 'rld', 1)['running']


def test_stop_is_exact_a_tfld_stop_leaves_an_fld_run(m):
    m.run(0, 'fld', 'fire')
    assert m.stop(1, 'tfld') == (True, '', [])
    assert surface(m, 'fld', 1)['running']
    assert m.stop(1, 'fld') == (True, '', ['fld'])
    assert surface(m, 'fld', 1)['last_end'] == 'stopped'


def test_stop_all_stops_everything_and_idle_stop_is_accepted(m):
    m.run(0, 'all', 'fire')
    accepted, _, stopped = m.stop(1, 'all')
    assert accepted and sorted(stopped) == ['fld', 'fpsi', 'rld', 'rpsi']
    assert m.stop(2, 'all') == (True, '', [])


def test_all_runs_on_four_surfaces(m):
    assert m.run(0, 'all', 'fire') == (True, '')
    running = [s['name'] for s in m.surface_states(1) if s['running']]
    assert running == ['rpsi', 'rld', 'fld', 'fpsi']


def test_all_is_accepted_only_if_every_surface_accepts_and_the_others_keep_running(m):
    cat = copy.deepcopy(CATALOGUE)
    cat['clips'] = [c for c in cat['clips'] if not (c['name'] == 'pulse' and c['target'] == 'rld')]
    partial = LedModel(cat)
    accepted, reason = partial.run(0, 'all', 'pulse')
    assert (accepted, reason) == (False, 'no clip for this surface')
    assert [s['name'] for s in partial.surface_states(1) if s['running']] == [
        'rpsi', 'fld', 'fpsi']


def test_all_text_is_partial_the_psis_have_no_text_mode(m):
    accepted, reason = m.say(0, 'all', ['hi'])
    assert (accepted, reason) == (False, 'display has no text mode')
    assert [s['name'] for s in m.surface_states(1) if s['running']] == ['rld', 'fld']


def test_duration_ends_the_run_with_last_end_duration(m):
    m.run(0, 'fld', 'fire', duration_ms=2000)
    assert surface(m, 'fld', 1.9)['running']
    s = surface(m, 'fld', 2.0)
    assert not s['running'] and s['last_end'] == 'duration' and s['last_effect'] == 'fire'


def test_wipe_ends_by_itself(m):
    m.run(0, 'rld', 'wipe')
    assert surface(m, 'rld', 1)['running']
    s = surface(m, 'rld', 4)
    assert not s['running'] and s['last_end'] == 'ended'


def test_non_looping_clips_end_by_themselves_looping_ones_do_not(m):
    m.run(0, 'fld', 'failure')
    m.run(0, 'rld', 'march')
    m.run(0, 'fpsi', 'pulse')
    assert not surface(m, 'fld', 18.0)['running'] and surface(m, 'fld', 18)['last_end'] == 'ended'
    assert surface(m, 'rld', 48.2)['running'] and not surface(m, 'rld', 48.3)['running']
    assert surface(m, 'fpsi', 3600)['running']


def test_lightsout_holds_forever(m):
    m.run(0, 'fld', 'lightsout')
    assert surface(m, 'fld', 100000)['running']


def test_text_left_right_up_end_by_themselves_bounce_and_loop_do_not(m):
    for motion, ends in ((0, False), (1, False), (2, True), (3, True), (4, True)):
        model = LedModel(copy.deepcopy(CATALOGUE))
        model.say(0, 'fld', ['hi', 'there'], motion=motion)
        assert surface(model, 'fld', 3600)['running'] is (not ends), motion
        if ends:
            assert surface(model, 'fld', 3600)['last_end'] == 'ended'


def test_cycle_reports_cycle_child_and_changes_each_ten_seconds(m):
    m.run(0, 'fld', 'cycle')
    names = [surface(m, 'fld', 10.0 * k + 1)['effect'] for k in range(6)]
    assert all(n.startswith('cycle:') and n[6:] in led.CYCLE_CHILDREN for n in names)
    assert all(a != b for a, b in zip(names, names[1:]))
    assert surface(m, 'fld', 1)['effect'] == surface(m, 'fld', 9.9)['effect']


def test_ending_restores_mode_and_picture(m):
    m.configure(0, 'tfld', led.APPLY_MODE, mode=led.MODE_SOLID, rgb=0x112233)
    m.configure(0, 'bfld', led.APPLY_MODE, mode=led.MODE_OFF)
    m.run(1, 'fld', 'fire', duration_ms=1000)
    assert display(m, 'tfld', 1.5)['mode'] == led.MODE_EFFECT
    assert display(m, 'tfld', 3)['mode'] == led.MODE_SOLID
    assert display(m, 'tfld', 3)['rgb'] == 0x112233
    assert display(m, 'bfld', 3)['mode'] == led.MODE_OFF
    assert display(m, 'rld', 3)['mode'] == led.MODE_RANDOM


def test_stop_restores_too(m):
    m.configure(0, 'rld', led.APPLY_MODE, mode=led.MODE_OFF)
    m.run(1, 'rld', 'fire')
    m.stop(2, 'rld')
    assert display(m, 'rld', 2)['mode'] == led.MODE_OFF


def test_last_end_last_effect_and_sources_survive_a_new_run(m):
    m.run(0, 'fld', 'fire', owner='first')
    m.stop(1, 'fld')
    m.run(2, 'fld', 'rainbow', owner='second')
    s = surface(m, 'fld', 3)
    assert (s['effect'], s['owner']) == ('rainbow', 'second')
    assert (s['last_end'], s['last_effect'], s['last_source'], s['last_owner']) == (
        'stopped', 'fire', 'ros', 'first')


def test_fresh_surface_reports_last_end_none(m):
    s = surface(m, 'fld', 0)
    assert (s['last_end'], s['last_effect'], s['running'], s['source']) == ('none', '', False, '')


# --- refusals (2.3) ---------------------------------------------------------------------

@pytest.mark.parametrize('surface_name, effect, speed, rgb, reason', [
    ('nowhere', 'fire', 0, 0, 'unknown surface'),
    ('fld', 'nosuch', 0, 0, 'unknown effect'),
    ('tfld', 'pulse', 0, 0, 'no clip for this surface'),
    ('fld', 'fire', 10, 0, 'out of range'),
    ('fld', 'fire', 0, 0x1000000, 'out of range'),
])
def test_run_refusal_strings(m, surface_name, effect, speed, rgb, reason):
    assert m.run(0, surface_name, effect, speed=speed, rgb=rgb) == (False, reason)
    assert not any(s['running'] for s in m.surface_states(1))


@pytest.mark.parametrize('surface_name, lines, kwargs, reason', [
    ('fld', [''], {}, 'no text'),
    ('fld', [], {}, 'no text'),
    ('rld', ['a', 'b'], {}, 'more lines than the surface has displays'),
    ('fld', ['x' * 64], {}, 'text too long'),
    ('fpsi', ['hi'], {}, 'display has no text mode'),
    ('fld', ['hi'], {'motion': 5}, 'unknown text motion'),
    ('fld', ['hi'], {'speed': 10}, 'out of range'),
    ('nowhere', ['hi'], {}, 'unknown surface'),
])
def test_say_refusal_strings(m, surface_name, lines, kwargs, reason):
    assert m.say(0, surface_name, lines, **kwargs) == (False, reason)


def test_text_length_is_counted_in_utf_8_bytes(m):
    assert m.say(0, 'fld', ['\u00e9' * 40]) == (False, 'text too long')    # 40 chars, 80 bytes
    assert m.say(0, 'fld', ['\u00e9' * 31 + 'x']) == (True, '')            # 63 bytes


def test_text_of_63_characters_is_accepted(m):
    assert m.say(0, 'fld', ['x' * 63]) == (True, '')


@pytest.mark.parametrize('kwargs, reason', [
    ({'apply': led.APPLY_MODE, 'mode': led.MODE_TEXT}, 'unknown mode'),
    ({'apply': led.APPLY_MODE, 'mode': 9}, 'unknown mode'),
    ({'apply': led.APPLY_BRIGHTNESS, 'brightness': 101}, 'out of range'),
    ({'apply': led.APPLY_DRIFT, 'drift': 'frantic'}, 'unknown drift preset'),
])
def test_configure_refusal_strings(m, kwargs, reason):
    accepted, got, _ = m.configure(0, 'tfld', **kwargs)
    assert (accepted, got) == (False, reason)


def test_configure_unknown_display(m):
    assert m.configure(0, 'zzz', led.APPLY_MODE)[:2] == (False, 'unknown display')


def test_a_refused_configure_changes_nothing(m):
    accepted, reason, _ = m.configure(
        0, 'tfld', led.APPLY_MODE | led.APPLY_BRIGHTNESS | led.APPLY_DRIFT,
        mode=led.MODE_OFF, brightness=90, drift='frantic')
    assert not accepted
    d = display(m, 'tfld', 0)
    assert (d['mode'], d['brightness'], d['drift']) == (led.MODE_RANDOM, 40, 'calm')


# --- configure (2.3) --------------------------------------------------------------------

def test_apply_mode_releases_a_run_at_once(m):
    m.run(0, 'fld', 'fire')
    accepted, _, state = m.configure(1, 'tfld', led.APPLY_MODE, mode=led.MODE_OFF)
    assert accepted and state['mode'] == led.MODE_OFF
    s = surface(m, 'fld', 1)
    assert not s['running'] and s['last_end'] == 'released'
    assert display(m, 'tfld', 1)['mode'] == led.MODE_OFF
    assert display(m, 'bfld', 1)['mode'] == led.MODE_RANDOM        # the other board restored


def test_apply_brightness_applies_through_a_running_effect(m):
    m.run(0, 'fld', 'fire')
    m.configure(1, 'tfld', led.APPLY_BRIGHTNESS, brightness=70)
    assert display(m, 'tfld', 1)['brightness'] == 70 and surface(m, 'fld', 1)['running']


def test_solid_with_rgb_zero_keeps_the_previous_colour(m):
    m.configure(0, 'rld', led.APPLY_MODE, mode=led.MODE_SOLID, rgb=0xAA0000)
    m.configure(1, 'rld', led.APPLY_MODE, mode=led.MODE_OFF)
    m.configure(2, 'rld', led.APPLY_MODE, mode=led.MODE_SOLID, rgb=0)
    assert display(m, 'rld', 2)['rgb'] == 0xAA0000


def test_drift_applies_to_logic_displays(m):
    m.configure(0, 'rld', led.APPLY_DRIFT, drift='lively')
    assert display(m, 'rld', 0)['drift'] == 'lively'


def test_drift_on_a_psi_named_alone_is_refused(m):
    assert m.configure(0, 'fpsi', led.APPLY_DRIFT, drift='calm')[:2] == (
        False, 'display has no drift')


def test_drift_under_all_skips_the_psis(m):
    accepted, _, state = m.configure(0, 'all', led.APPLY_DRIFT, drift='lively')
    assert accepted and state['name'] == ''
    assert [display(m, n, 0)['drift'] for n in ('rpsi', 'fpsi', 'rld', 'tfld')] == [
        '', '', 'lively', 'lively']


def test_brightness_under_all_reaches_every_display(m):
    m.configure(0, 'all', led.APPLY_BRIGHTNESS, brightness=15)
    assert {d['brightness'] for d in m.display_states(0)} == {15}


def test_save_is_accepted_and_never_persists(m):
    accepted, _, _ = m.configure(0, 'tfld', led.APPLY_BRIGHTNESS | led.APPLY_SAVE, brightness=20)
    assert accepted
    assert not hasattr(m, 'saved')
    fresh = LedModel(copy.deepcopy(CATALOGUE))
    assert display(fresh, 'tfld', 0)['brightness'] == 40


def test_hue_is_stored_and_a_psi_takes_it(m):
    m.configure(0, 'fpsi', led.APPLY_HUE, hue=-64)
    assert display(m, 'fpsi', 0)['hue'] == -64


# --- attribution (2.4) -------------------------------------------------------------------

def test_ros_configure_sets_changed_by_ros_and_owner(m):
    m.configure(0, 'tfld', led.APPLY_BRIGHTNESS, brightness=20, owner='r2_expression',
                priority=10)
    d = display(m, 'tfld', 0)
    assert (d['changed_by'], d['changed_by_owner']) == ('ros', 'r2_expression')
    assert display(m, 'bfld', 0)['changed_by'] == ''


def test_console_write_is_attributed_console_and_clears_the_owner(m):
    m.configure(0, 'tfld', led.APPLY_BRIGHTNESS, brightness=20, owner='x')
    m.configure(1, 'tfld', led.APPLY_BRIGHTNESS, brightness=30, source='console')
    d = display(m, 'tfld', 1)
    assert (d['changed_by'], d['changed_by_owner']) == ('console', '')


def test_a_console_run_reads_source_console(m):
    m.run(0, 'fld', 'fire', source='console')
    s = surface(m, 'fld', 1)
    assert (s['source'], s['owner']) == ('console', '')


# --- holds (2.4) -------------------------------------------------------------------------

@pytest.mark.parametrize('source, reason', [
    ('console', 'held by console'), ('ui', 'held by ui'),
    ('text_topic', 'held by text topic')])
def test_ros_priority_in_1_to_199_is_refused_on_a_held_display(m, source, reason):
    m.configure(0, 'tfld', led.APPLY_BRIGHTNESS, brightness=20, source=source)
    assert m.run(1, 'fld', 'fire', priority=10) == (False, reason)
    assert m.say(1, 'fld', ['hi'], priority=199) == (False, reason)
    assert m.stop(1, 'fld', priority=1) == (False, reason, [])
    assert m.configure(1, 'tfld', led.APPLY_BRIGHTNESS, brightness=1, priority=50)[:2] == (
        False, reason)


@pytest.mark.parametrize('priority', [0, 200, 255])
def test_priority_zero_and_200_plus_are_never_refused(m, priority):
    m.run(0, 'fld', 'fire', source='console')
    assert m.run(1, 'fld', 'rainbow', priority=priority) == (True, '')
    assert m.configure(1, 'tfld', led.APPLY_BRIGHTNESS, brightness=5, priority=priority)[0]
    assert m.stop(1, 'fld', priority=priority)[0]


def test_the_hold_lasts_hold_ms_and_counts_down(m):
    m.run(0, 'fld', 'fire', source='console')
    d = display(m, 'tfld', 1)
    assert (d['held_by'], d['hold_remaining_ms']) == ('console', 4000)
    assert display(m, 'rld', 1)['held_by'] == ''
    assert m.run(4.9, 'fld', 'roam', priority=10)[0] is False
    assert m.run(5.0, 'fld', 'roam', priority=10) == (True, '')
    assert display(m, 'tfld', 5.0)['held_by'] == ''


def test_each_further_write_renews_the_hold(m):
    m.run(0, 'fld', 'fire', source='console')
    m.configure(4, 'tfld', led.APPLY_BRIGHTNESS, brightness=9, source='ui')
    d = display(m, 'tfld', 8)
    assert (d['held_by'], d['hold_remaining_ms']) == ('ui', 1000)
    assert display(m, 'bfld', 8)['held_by'] == ''     # only tfld was touched at 4 s


def test_hold_ms_is_configurable():
    model = LedModel(copy.deepcopy(CATALOGUE), hold_ms=1000)
    model.run(0, 'fld', 'fire', source='console')
    assert model.run(1.0, 'fld', 'roam', priority=10) == (True, '')


def test_ros_writes_never_hold_and_never_end_a_hold(m):
    m.run(0, 'fld', 'fire', priority=10)
    assert display(m, 'tfld', 0)['held_by'] == ''
    m.configure(0, 'tfld', led.APPLY_BRIGHTNESS, brightness=20, source='console')
    m.run(1, 'fld', 'roam', priority=250)
    d = display(m, 'tfld', 1)
    assert d['held_by'] == 'console' and d['hold_remaining_ms'] == 4000


def test_hold_check_is_all_or_nothing_under_all(m):
    m.configure(0, 'fpsi', led.APPLY_BRIGHTNESS, brightness=30, source='console')
    assert m.run(1, 'all', 'fire', priority=10) == (False, 'held by console')
    assert not any(s['running'] for s in m.surface_states(1))
    assert m.configure(1, 'all', led.APPLY_BRIGHTNESS, brightness=1, priority=10)[:2] == (
        False, 'held by console')
    assert display(m, 'rld', 1)['brightness'] == 40


def test_a_hold_on_one_board_refuses_the_whole_fld_surface(m):
    m.configure(0, 'bfld', led.APPLY_BRIGHTNESS, brightness=30, source='console')
    assert m.run(1, 'fld', 'fire', priority=10) == (False, 'held by console')
    assert m.run(1, 'tfld', 'fire', priority=10) == (True, '')


def test_hold_is_checked_before_unknown_effect_and_out_of_range(m):
    m.configure(0, 'tfld', led.APPLY_BRIGHTNESS, brightness=30, source='console')
    assert m.run(1, 'fld', 'nosuch', speed=99, priority=10) == (False, 'held by console')


def test_unknown_surface_comes_before_the_hold(m):
    m.manual.set_on(True)
    assert m.run(0, 'nowhere', 'fire', priority=10) == (False, 'unknown surface')


def test_not_fitted_comes_before_the_hold(m):
    m.manual.set_on(True)
    m.fitted = False
    assert m.run(0, 'fld', 'fire', priority=10) == (False, 'not fitted')


# --- manual and enable ------------------------------------------------------------------

def test_sticky_manual_holds_every_display_forever(m):
    m.manual.set_on(True)
    assert m.run(1000, 'fld', 'fire', priority=10) == (False, 'held: manual')
    assert m.stop(1000, 'all', priority=10)[:2] == (False, 'held: manual')
    d = display(m, 'rld', 1000)
    assert (d['held_by'], d['hold_remaining_ms']) == ('manual', HOLD_FOREVER)
    assert m.state(1000)['manual'] is True
    assert m.run(1000, 'fld', 'fire', priority=0) == (True, '')


def test_owner_auto_drops_manual_and_every_timed_hold(m):
    m.configure(0, 'tfld', led.APPLY_BRIGHTNESS, brightness=30, source='console')
    m.manual.set_on(True)
    m.manual.set_on(False)
    assert m.state(1)['manual'] is False
    assert display(m, 'tfld', 1)['held_by'] == ''
    assert m.run(1, 'fld', 'fire', priority=10) == (True, '')


def test_a_strip_that_is_not_wired_refuses_everything_as_not_fitted(m):
    m.fitted = False
    for result in (m.run(0, 'fld', 'fire'), m.say(0, 'fld', ['x'])):
        assert result == (False, 'not fitted')
    assert m.configure(0, 'tfld', led.APPLY_MODE, mode=0)[:2] == (False, 'not fitted')
    assert m.stop(0, 'fld')[:2] == (False, 'not fitted')


def test_enable_off_darkens_the_strip_but_accepts_and_tracks_writes(m):
    m.set_enabled(False)
    assert m.state(0)['strip_enabled'] is False
    assert m.run(0, 'fld', 'fire') == (True, '')
    assert next(s for s in m.surface_states(0) if s['name'] == 'fld')['running']
    assert m.configure(0, 'tfld', led.APPLY_BRIGHTNESS, brightness=33)[:2] == (True, '')
    assert m.stop(0, 'fld')[:2] == (True, '')
    assert m.state(0)['strip_enabled'] is False
    m.set_enabled(True)
    assert m.state(0)['strip_enabled'] is True


def test_owner_of_32_bytes_is_too_long_31_is_fine(m):
    assert m.run(0, 'fld', 'fire', owner='x' * 32) == (False, 'owner too long')
    assert m.say(0, 'fld', ['a'], owner='x' * 32) == (False, 'owner too long')
    assert m.configure(0, 'tfld', 0, owner='x' * 32)[:2] == (False, 'owner too long')
    assert m.run(0, 'fld', 'fire', owner='x' * 31) == (True, '')
    assert m.run(0, 'fld', 'fire', owner='é' * 16) == (False, 'owner too long')     # bytes


# --- text slots -------------------------------------------------------------------------

def test_a_text_slot_message_starts_say_fld_bounce_both_lines_no_duration(m):
    m.text_slot(0, 0, 'hello')
    s = surface(m, 'fld', 1)
    assert (s['running'], s['effect'], s['source'], s['duration_ms']) == (
        True, 'text', 'text_topic', 0)
    assert surface(m, 'fld', 10000)['running']          # bounce never ends
    m.text_slot(2, 1, 'world')
    assert surface(m, 'fld', 3)['elapsed_ms'] == 1000   # a new run, started at 2 s


def test_a_text_slot_write_replaces_a_ros_run_and_holds_fld(m):
    m.run(0, 'fld', 'fire', owner='r2_expression', priority=10)
    m.text_slot(1, 0, 'hi')
    s = surface(m, 'fld', 2)
    assert (s['source'], s['last_end'], s['last_source'], s['last_owner']) == (
        'text_topic', 'replaced', 'ros', 'r2_expression')
    assert display(m, 'tfld', 2)['held_by'] == 'text_topic'
    assert m.run(2, 'fld', 'fire', priority=10) == (False, 'held by text topic')


def test_both_slots_empty_stops_the_text_run_and_is_itself_a_write(m):
    m.text_slot(0, 0, 'hi')
    m.text_slot(3, 0, '')
    s = surface(m, 'fld', 3)
    assert not s['running'] and s['last_end'] == 'stopped'
    assert display(m, 'tfld', 3)['hold_remaining_ms'] == 5000


def test_one_slot_cleared_while_the_other_has_text_keeps_a_run(m):
    m.text_slot(0, 0, 'a')
    m.text_slot(1, 1, 'b')
    m.text_slot(2, 0, '')
    assert surface(m, 'fld', 3)['running']


def test_clearing_the_slots_does_not_stop_a_run_that_is_not_text(m):
    m.run(0, 'fld', 'fire')
    m.text_slot(1, 0, '')
    assert surface(m, 'fld', 2)['running']


def test_rostext_off_makes_the_slots_inert(m):
    m.rostext = False
    m.text_slot(0, 0, 'hi')
    assert not surface(m, 'fld', 1)['running'] and display(m, 'tfld', 1)['held_by'] == ''


def test_a_long_text_slot_line_is_refused(m):
    assert m.text_slot(0, 0, 'x' * 64) == (False, 'text too long')


# --- level and state --------------------------------------------------------------------

def test_level_is_stored_and_level_set_flips(m):
    assert (m.state(0)['level'], m.state(0)['level_set']) == (0, False)
    m.set_level(200)
    assert (m.state(0)['level'], m.state(0)['level_set']) == (200, True)
    m.run(0, 'fld', 'micbright')
    assert m.state(1)['level'] == 200


def test_console_text_and_manual_modes_are_not_ros_modes(m):
    assert m.configure(0, 'tfld', led.APPLY_MODE, mode=led.MODE_TEXT, text='x',
                       source='console')[0]
    assert display(m, 'tfld', 0)['text'] == 'x'
    assert m.configure(0, 'tfld', led.APPLY_MODE, mode=led.MODE_MANUAL, source='console')[0]
    assert m.configure(0, 'fpsi', led.APPLY_MODE, mode=led.MODE_TEXT, text='x',
                       source='console')[:2] == (False, 'display has no text mode')


def test_a_refused_text_slot_message_does_not_poison_the_other_line(m):
    assert m.text_slot(0, 0, 'x' * 64) == (False, 'text too long')
    assert m.text_slot(1, 1, 'hi')[0] is True
    assert m.slots == ['', 'hi']
