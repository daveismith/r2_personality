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

"""Tests for expression.yaml loading and validation (plan 4.3)."""

import copy
import dataclasses
import math
import os
import re

import pytest
import yaml

from fixtures import AFFECT_YAML, base_data, cfg, dome_data, LEDS_LIST, SHIPPED  # noqa: I100
from fixtures import capture_names, MAGIC_LIST
from r2_expression.config import (DEFAULT_PROFILE, ExpressionConfigError, load_config,
                                  load_config_string, missing_moods, parse_config)
from r2_expression.motion import Coast, Heading, Profile
from r2_expression.palette import LedCatalogue, MagicCatalogue, validate_picks


def rejects(data, fragment):
    with pytest.raises(ExpressionConfigError) as info:
        parse_config(data)
    assert fragment in str(info.value), str(info.value)
    return str(info.value)


def with_mood(**mood):
    data = base_data()
    data['moods']['calm'] = mood
    return data


def with_pick(**pick):
    data = base_data()
    data['moods']['calm']['flourish']['picks'] = [pick]
    return data


# --- the shipped file ----------------------------------------------------------------------

def test_shipped_expression_yaml_has_every_mood_of_affect_yaml():
    if not os.path.exists(AFFECT_YAML):
        pytest.skip('r2_affect is not beside this package')
    with open(AFFECT_YAML) as f:
        labels = list(yaml.safe_load(f)['moods'])
    config = load_config(SHIPPED)
    assert missing_moods(config, labels) == []
    assert list(config.moods) == labels


def test_shipped_expression_yaml_round_trips_through_the_dataclasses():
    config = load_config(SHIPPED)
    assert parse_config(config.to_dict()) == config


def test_shipped_defaults_are_the_plan_values():
    d = load_config(SHIPPED).defaults
    assert (d.manual_hold_s, d.mood_stale_s, d.max_duration_ms, d.status_hz, d.max_lease_s,
            d.device_stale_s) == (5.0, 3.0, 30000, 2.0, 60.0, 5.0)


def test_shipped_brightness_respects_the_floors_of_section_2_6():
    config = load_config(SHIPPED)
    for name, mood in config.moods.items():
        for channel in ('fld', 'rld'):
            assert mood.styles[channel].brightness[0] >= 25, (name, channel)
        assert mood.styles['psi'].brightness[0] >= 20, name
    neutral = config.neutral.styles
    assert neutral['fld'].brightness == neutral['rld'].brightness == (40, 40)
    assert neutral['psi'].brightness == (40, 40)
    assert 'magic' not in neutral          # the Magic Panel is untouched while neutral


def test_shipped_picks_all_exist_in_the_workbench_catalogue():
    if not os.path.exists(LEDS_LIST):
        pytest.skip('docs/dome is not beside this package')
    effects, clips, sequences = capture_names()
    led = LedCatalogue(effects=frozenset(effects),
                       clips={k: frozenset(v) for k, v in clips.items()})
    picks, warnings = validate_picks(load_config(SHIPPED), led=led,
                                     magic=MagicCatalogue(sequences=sequences))
    assert warnings == []
    assert all(picks[name] for name in picks)


def test_no_shipped_pick_names_a_looping_sequence_of_the_workbench_catalogue():
    if not os.path.exists(MAGIC_LIST):
        pytest.skip('docs/dome is not beside this package')
    with open(MAGIC_LIST) as f:
        looping = {int(i) for i, flags in re.findall(
            r"MagicPanelSequence\(id=(\d+), name='[^']*', flags=(\d+)", f.read())
            if int(flags) & 1}
    assert {56, 57} <= looping
    for mood in load_config(SHIPPED).moods.values():
        for pick in mood.flourish.picks:
            assert pick.channel != 'magic' or pick.name not in looping


def test_shipped_flourishes_never_carry_duration_zero_and_lightsout_is_timed():
    for mood in load_config(SHIPPED).moods.values():
        for pick in mood.flourish.picks:
            assert pick.duration_ms and pick.duration_ms <= 30000
            if pick.name == 'lightsout':
                assert pick.duration_ms


def _picked(mood):
    return {(p.channel, p.name) for p in mood.flourish.picks}


def test_shipped_dome_defaults_are_the_plan_4_5_values():
    d = load_config(SHIPPED).defaults.dome
    assert (d.min_speed_rad_s, d.max_speed_rad_s, d.max_accel_rad_s2, d.max_move_s,
            d.settle_s, d.rest_recheck_s) == (0.5, 2.0, 4.0, 8.0, 0.8, 30.0)
    assert d.coast == Coast(0.37, 0.07) and d.heading == Heading(-1, 0.0)
    assert (d.body_speed_m_s, d.body_yaw_rad_s, d.after_foreign_s, d.after_still_s) == (
        0.15, 0.3, 3.0, 1.0)


def test_shipped_every_mood_has_a_dome_entry_and_profiles_within_the_caps():
    config = load_config(SHIPPED)
    assert config.dome_warnings == ()
    assert config.profiles['snappy'] == Profile(2.0, 4.0, 3.0, 0.05)
    assert config.profiles['smooth'] == Profile(1.0, 1.5, 1.5, 0.05)
    assert config.profiles['lazy'] == Profile(0.6, 0.8, 1.0, 0.09)
    assert config.neutral.dome.profile == 'smooth'
    assert config.neutral.dome.glance is None and config.neutral.dome.rest_rad is None
    for name, mood in config.moods.items():
        assert mood.dome is not None and mood.dome.profile in config.profiles, name
        assert mood.dome.glance is not None, name
    moods = config.moods
    assert moods['calm'].dome.glance.every_s == (20, 60)
    assert moods['curious'].dome.glance.amplitude_rad == (0.5, 1.4)
    assert moods['excited'].dome.glance.dwell_s == (0.2, 0.6)
    assert (moods['sleepy'].dome.profile, moods['sleepy'].dome.rest_rad) == ('lazy', 0.35)
    assert moods['sulking'].dome.rest_rad == 2.6             # rests looking away
    assert [n for n, m in moods.items() if m.dome.rest_rad is not None] == ['sleepy',
                                                                            'sulking']


def test_shipped_moods_have_the_character_of_plan_4_3():
    moods = load_config(SHIPPED).moods
    for name, mood in moods.items():
        assert mood.styles['magic'].brightness is not None, name
        assert mood.flourish is not None and mood.flourish.picks, name
    for name in ('excited', 'playful'):
        assert moods[name].styles['fld'].drift == moods[name].styles['rld'].drift == 'lively'
    for name in ('sleepy', 'sulking', 'calm'):
        assert moods[name].styles['fld'].drift == moods[name].styles['rld'].drift == 'calm'
    assert moods['excited'].flourish.every_s == (6, 15)
    assert moods['calm'].flourish.every_s == (40, 90)
    assert moods['sleepy'].flourish.every_s == (120, 300)
    assert ('fld', 'pulse') in _picked(moods['calm'])
    assert {('fld', 'roam'), ('rld', 'cycle')} <= _picked(moods['curious'])
    for name, reds in (('annoyed', {'alarm', 'flash', 'redalert'}),
                       ('scared', {'alarm-twinkle', 'flash', 'redalert'})):
        assert reds <= {n for _, n in _picked(moods[name])}, name
    assert ('magic', 6) in _picked(moods['annoyed'])
    assert ('magic', 7) in _picked(moods['scared'])
    assert max(s.brightness[1] for s in moods['sleepy'].styles.values()) <= 35
    assert min(moods['excited'].styles['fld'].brightness) >= 80


# --- shape ---------------------------------------------------------------------------------

def test_defaults_take_the_plan_values_when_absent():
    data = base_data()
    del data['defaults']
    d = parse_config(data).defaults
    assert (d.manual_hold_s, d.mood_stale_s, d.max_duration_ms, d.status_hz, d.max_lease_s,
            d.device_stale_s) == (5.0, 3.0, 30000, 2.0, 60.0, 5.0)


def test_brightness_number_and_range_both_load():
    config = cfg()
    assert config.neutral.styles['fld'].brightness == (40, 40)
    assert config.moods['calm'].styles['fld'].brightness == (50, 60)
    assert config.moods['calm'].styles['magic'].brightness == (5, 5)


def test_config_is_immutable():
    config = cfg()
    with pytest.raises(dataclasses.FrozenInstanceError):
        config.defaults.manual_hold_s = 1.0
    with pytest.raises(TypeError):
        config.moods['new'] = None
    with pytest.raises(TypeError):
        config.moods['calm'].styles['fld'] = None


def test_mood_may_leave_channels_out():
    config = parse_config(with_mood(fld={'drift': 'lively'}))
    assert list(config.moods['calm'].styles) == ['fld']
    assert config.moods['calm'].flourish is None


def test_pick_duration_above_the_cap_is_capped():
    data = with_pick(channel='fld', effect='fire', duration_ms=90000)
    data['defaults']['max_duration_ms'] = 20000
    assert parse_config(data).moods['calm'].flourish.picks[0].duration_ms == 20000


def test_sequence_pick_by_id_or_name():
    picks = parse_config(with_pick(channel='magic', sequence='Smile')).moods['calm']
    assert picks.flourish.picks[0].name == 'Smile'
    picks = parse_config(with_pick(channel='magic', sequence=44, repeat=2, end='blank'))
    pick = picks.moods['calm'].flourish.picks[0]
    assert (pick.name, pick.repeat, pick.end) == (44, 2, 'blank')


def test_missing_moods_lists_labels_without_an_entry():
    assert missing_moods(cfg(), ['calm', 'sleepy', 'excited']) == ['sleepy']


# --- the dome (phase 2 plan 4.5) ----------------------------------------------------------

def dome_rejects(change, fragment):
    data = dome_data()
    change(data)
    return rejects(data, fragment)


def test_dome_loads_and_round_trips():
    config = parse_config(dome_data())
    assert parse_config(config.to_dict()) == config
    assert config.mood_dome('sleepy').rest_rad == 0.35
    assert config.mood_dome('sleepy', stale=True) == config.neutral.dome
    assert config.mood_dome('unknown') == config.neutral.dome
    assert config.profile('lazy').v_max == 0.6


def test_dome_without_any_dome_config_moves_with_the_default_profile_and_warns_per_mood():
    config = cfg()
    assert config.profile('') == DEFAULT_PROFILE and config.neutral.dome.profile == ''
    assert config.mood_dome('calm').profile == ''
    assert config.dome_warnings == (
        'moods.calm: no dome entry; the dome moves as neutral, with no glances',
        'moods.excited: no dome entry; the dome moves as neutral, with no glances')
    data = base_data()
    data['defaults']['dome'] = {'max_speed_rad_s': 0.8}
    assert parse_config(data).profile('').v_max == 0.8          # within the cap too
    data['defaults']['dome'] = {'min_speed_rad_s': 1.2}
    assert parse_config(data).profile('').v_max == 1.2          # and never under the floor


def test_dome_v_max_and_accel_above_the_cap_are_clamped_with_a_warning():
    data = dome_data()
    data['profiles']['snappy'].update(v_max=3.0, accel=6.0)
    config = parse_config(data)
    assert (config.profiles['snappy'].v_max, config.profiles['snappy'].accel) == (2.0, 4.0)
    assert config.dome_warnings == (
        'profiles.snappy.v_max: 3 is above defaults.dome.max_speed_rad_s; clamped to 2',
        'profiles.snappy.accel: 6 is above defaults.dome.max_accel_rad_s2; clamped to 4')
    assert parse_config(config.to_dict()) == config              # warnings do not compare


def test_dome_defaults_partial_blocks_keep_the_rest():
    data = dome_data()
    data['defaults']['dome'] = {'coast': {'c2_rad': 0.0}, 'heading': {'sign': 1}}
    d = parse_config(data).defaults.dome
    assert d.coast == Coast(0.37, 0.0) and d.heading == Heading(1, 0.0)
    assert d.min_speed_rad_s == 0.5


@pytest.mark.parametrize('dome, fragment', [
    ({'tick_hz': 20}, 'defaults.dome.tick_hz: unknown key'),
    ({'min_speed_rad_s': 0}, 'defaults.dome.min_speed_rad_s: must be > 0'),
    ({'min_speed_rad_s': 2.5}, 'defaults.dome.min_speed_rad_s: 2.5 is above max_speed'),
    ({'coast': {'c1_s': -0.1}}, 'defaults.dome.coast.c1_s: must be > 0'),
    ({'coast': {'c2_rad': -0.1}}, 'defaults.dome.coast.c2_rad: must be >= 0'),
    ({'coast': {'c3': 1}}, 'defaults.dome.coast.c3: unknown key'),
    ({'heading': {'sign': 0}}, 'defaults.dome.heading.sign: must be -1 or 1'),
    ({'heading': {'sign': True}}, 'defaults.dome.heading.sign: must be -1 or 1'),
    ({'heading': {'offset_rad': 4.0}}, 'defaults.dome.heading.offset_rad: 4 is outside'),
    ({'heading': {'offset_rad': -math.pi}}, 'outside (-pi, pi]'),
    ({'after_foreign_s': 'soon'}, 'defaults.dome.after_foreign_s: must be a number'),
    ('fast', 'defaults.dome: must be a mapping')])
def test_dome_rejects_bad_defaults(dome, fragment):
    def change(data):
        data['defaults']['dome'] = dome
    dome_rejects(change, fragment)


@pytest.mark.parametrize('profile, fragment', [
    ({'v_max': 1.0, 'accel': 1.0, 'k': 1.0}, 'profiles.p.settle_band_rad: required key'),
    ({'v_max': 1.0, 'accel': 1.0, 'k': 0, 'settle_band_rad': 0.05}, 'profiles.p.k: must be > 0'),
    ({'v_max': 0.4, 'accel': 1.0, 'k': 1.0, 'settle_band_rad': 0.05},
     'profiles.p.v_max: 0.4 is below defaults.dome.min_speed_rad_s'),
    ({'v_max': 1.0, 'accel': 1.0, 'k': 1.0, 'settle_band_rad': 4.0}, 'is above pi'),
    ({'v_max': 1.0, 'accel': 1.0, 'k': 1.0, 'settle_band_rad': 0.05, 'ki': 1},
     'profiles.p.ki: unknown key'),
    ('smooth', 'profiles.p: must be a mapping')])
def test_dome_rejects_bad_profiles(profile, fragment):
    def change(data):
        data['profiles']['p'] = profile
    dome_rejects(change, fragment)


def test_dome_profile_mood_is_reserved():
    def change(data):
        data['profiles']['mood'] = dict(data['profiles']['smooth'])
    dome_rejects(change, "profiles.mood: 'mood' is reserved")


@pytest.mark.parametrize('dome, fragment', [
    ({}, 'moods.calm.dome.profile: required key missing'),
    ({'profile': 'zippy'}, "moods.calm.dome.profile: unknown profile 'zippy'"),
    ({'profile': 'smooth', 'colour': 1}, 'moods.calm.dome.colour: unknown key'),
    ({'profile': 'smooth', 'rest_rad': 3.5}, 'moods.calm.dome.rest_rad: 3.5 is outside'),
    ({'profile': 'smooth', 'glance': {'every_s': [5, 10], 'dwell_s': [1, 2]}},
     'moods.calm.dome.glance.amplitude_rad: required key missing'),
    ({'profile': 'smooth', 'glance': {'every_s': [5, 10], 'amplitude_rad': [0.2, 3.5],
                                      'dwell_s': [1, 2]}},
     'moods.calm.dome.glance.amplitude_rad: max 3.5 is above 3.14159'),
    ({'profile': 'smooth', 'glance': {'every_s': [10, 5], 'amplitude_rad': [0.2, 0.5],
                                      'dwell_s': [1, 2]}},
     'moods.calm.dome.glance.every_s: min 10 is above max 5'),
    ({'profile': 'smooth', 'glance': {'every_s': [5, 10], 'amplitude_rad': [0, 0.5],
                                      'dwell_s': [1, 2]}},
     'moods.calm.dome.glance.amplitude_rad[0]: must be > 0'),
    ({'profile': 'smooth', 'glance': {'every_s': 5, 'amplitude_rad': [0.2, 0.5],
                                      'dwell_s': [1, 2]}},
     'moods.calm.dome.glance.every_s: must be [min, max]'),
    ({'profile': 'smooth', 'glance': {'every_s': [5, 10], 'amplitude_rad': [0.2, 0.5],
                                      'dwell_s': [1, 9]}},
     'moods.calm.dome.glance.dwell_s: max 9 is above 8')])
def test_dome_rejects_bad_mood_entries(dome, fragment):
    def change(data):
        data['moods']['calm']['dome'] = dome
    dome_rejects(change, fragment)


@pytest.mark.parametrize('key', ['glance', 'rest_rad'])
def test_dome_neutral_has_a_profile_only(key):
    def change(data):
        data['neutral']['dome'][key] = 0.3
    dome_rejects(change, 'neutral.dome.%s: neutral has no glances and no rest' % key)


# --- rule 9: no hue anywhere ---------------------------------------------------------------

@pytest.mark.parametrize('where', ['top', 'defaults', 'neutral', 'style', 'pick', 'flourish'])
def test_rule9_rejects_hue_anywhere(where):
    data = base_data()
    if where == 'top':
        data['hue'] = 0
        path = 'hue'
    elif where == 'defaults':
        data['defaults']['hue'] = 0
        path = 'defaults.hue'
    elif where == 'neutral':
        data['neutral']['fld']['hue'] = 64
        path = 'neutral.fld.hue'
    elif where == 'style':
        data['moods']['calm']['rld']['hue'] = 64
        path = 'moods.calm.rld.hue'
    elif where == 'pick':
        data['moods']['calm']['flourish']['picks'][0]['hue'] = 64
        path = 'moods.calm.flourish.picks[0].hue'
    else:
        data['moods']['calm']['flourish']['hue'] = 64
        path = 'moods.calm.flourish.hue'
    message = rejects(data, 'rule 9')
    assert message.startswith(path + ':')


# --- rejections ----------------------------------------------------------------------------

def test_rejects_unknown_top_level_key():
    rejects(base_data(palette={}), 'palette: unknown top-level key')


def test_rejects_unknown_default():
    data = base_data()
    data['defaults']['tick_hz'] = 10
    rejects(data, 'defaults.tick_hz: unknown key')


@pytest.mark.parametrize('key, value', [('manual_hold_s', 0), ('mood_stale_s', -1),
                                        ('max_duration_ms', 0), ('max_duration_ms', 1.5),
                                        ('status_hz', 'fast'), ('max_lease_s', float('inf'))])
def test_rejects_bad_defaults(key, value):
    data = base_data()
    data['defaults'][key] = value
    rejects(data, 'defaults.%s' % key)


@pytest.mark.parametrize('key', ['neutral', 'moods'])
def test_rejects_missing_required_section(key):
    data = base_data()
    del data[key]
    rejects(data, '%s: required key missing' % key)


def test_rejects_empty_moods():
    rejects(base_data(moods={}), 'moods: must name at least one mood')


@pytest.mark.parametrize('channel', ['tfld', 'bfld', 'head', 'fpsi'])
def test_rejects_unknown_style_channel(channel):
    data = base_data()
    data['moods']['calm'][channel] = {'brightness': 50}
    rejects(data, 'moods.calm.%s: unknown channel' % channel)


def test_rejects_unknown_style_key():
    data = base_data()
    data['moods']['calm']['fld']['speed'] = 3
    rejects(data, 'moods.calm.fld.speed: unknown key')


@pytest.mark.parametrize('channel, value', [('fld', 101), ('rld', -1), ('psi', [10, 120]),
                                            ('magic', 16), ('magic', [0, 20])])
def test_rejects_brightness_out_of_range(channel, value):
    data = base_data()
    data['moods']['calm'][channel] = {'brightness': value}
    rejects(data, 'out of range')


def test_rejects_brightness_range_upside_down_or_fractional():
    data = base_data()
    data['moods']['calm']['fld']['brightness'] = [70, 50]
    rejects(data, 'moods.calm.fld.brightness: min 70 is above max 50')
    data['moods']['calm']['fld']['brightness'] = 55.5
    rejects(data, 'must be an integer')
    data['moods']['calm']['fld']['brightness'] = [50, 60, 70]
    rejects(data, 'a range is [min, max]')


@pytest.mark.parametrize('channel', ['psi', 'magic'])
def test_rejects_drift_where_there_is_none(channel):
    data = base_data()
    data['moods']['calm'][channel]['drift'] = 'calm'
    rejects(data, 'moods.calm.%s.drift: display has no drift' % channel)


def test_rejects_unknown_drift_preset():
    data = base_data()
    data['moods']['calm']['fld']['drift'] = 'custom'
    rejects(data, 'moods.calm.fld.drift: unknown drift preset')


def test_rejects_empty_style():
    data = base_data()
    data['moods']['calm']['fld'] = {}
    rejects(data, 'moods.calm.fld: a style needs')


def test_rejects_neutral_range_and_neutral_flourish():
    data = base_data()
    data['neutral']['fld']['brightness'] = [30, 50]
    rejects(data, 'neutral.fld.brightness: must be a single number')
    data = base_data()
    data['neutral']['flourish'] = copy.deepcopy(data['moods']['calm']['flourish'])
    rejects(data, 'neutral.flourish: neutral has no flourishes')


def test_rejects_pick_with_no_kind_or_two_kinds():
    rejects(with_pick(channel='fld'), 'exactly one of effect, clip, sequence, got none')
    rejects(with_pick(channel='fld', effect='fire', clip='pulse'),
            'exactly one of effect, clip, sequence, got effect, clip')


def test_rejects_pick_on_unknown_channel_or_without_one():
    rejects(with_pick(channel='dome', effect='fire'), 'picks[0].channel: unknown channel')
    rejects(with_pick(effect='fire'), 'picks[0].channel: required key missing')


@pytest.mark.parametrize('pick, fragment', [
    ({'channel': 'tfld', 'clip': 'pulse'}, 'no clip for this surface'),
    ({'channel': 'fld', 'sequence': 3}, 'fld picks name an effect or a clip'),
    ({'channel': 'magic', 'effect': 'fire'}, 'magic picks name a sequence'),
    ({'channel': 'magic', 'sequence': 3, 'rgb': 0xff0000}, 'picks[0].rgb: not a Magic Panel'),
    ({'channel': 'fld', 'effect': 'fire', 'repeat': 2}, 'picks[0].repeat: not a LED'),
    ({'channel': 'fld', 'effect': 'fire', 'speed': 10}, 'picks[0].speed'),
    ({'channel': 'fld', 'effect': 'fire', 'rgb': 0x1000000}, 'picks[0].rgb'),
    ({'channel': 'fld', 'effect': 'fire', 'weight': 0}, 'picks[0].weight: must be > 0'),
    ({'channel': 'magic', 'sequence': 256}, 'picks[0].sequence'),
    ({'channel': 'magic', 'sequence': 3, 'end': 'freeze'}, 'picks[0].end'),
    ({'channel': 'magic', 'sequence': 3, 'repeat': 0}, 'picks[0].repeat'),
    ({'channel': 'fld', 'effect': ''}, 'must be a non-empty name'),
    ({'channel': 'fld', 'effect': 'fire', 'colour': 1}, 'picks[0].colour: unknown key'),
])
def test_rejects_bad_picks(pick, fragment):
    rejects(with_pick(**pick), fragment)


def test_rule3_rejects_duration_zero_in_a_pick():
    rejects(with_pick(channel='fld', effect='fire', duration_ms=0), 'rule 3')


def test_rejects_every_s_min_above_max_or_not_positive():
    data = base_data()
    data['moods']['calm']['flourish']['every_s'] = [30, 10]
    rejects(data, 'moods.calm.flourish.every_s: min 30 is above max 10')
    data['moods']['calm']['flourish']['every_s'] = [0, 10]
    rejects(data, 'every_s[0]: must be > 0')
    data['moods']['calm']['flourish']['every_s'] = 10
    rejects(data, 'must be [min, max]')


def test_rejects_empty_picks():
    data = base_data()
    data['moods']['calm']['flourish']['picks'] = []
    rejects(data, 'picks: must be a non-empty list')


def test_yaml_merge_keys_share_a_style_and_explicit_keys_override():
    text = (
        'neutral: {fld: {drift: calm, brightness: 40}}\n'
        'moods:\n'
        '  calm:\n'
        '    fld: &soft {drift: calm, brightness: [40, 50]}\n'
        '    rld: {<<: *soft, drift: lively}\n')
    rld = load_config_string(text).moods['calm'].styles['rld']
    assert (rld.drift, rld.brightness) == ('lively', (40, 50))


def test_rejects_a_huge_integer_as_out_of_range():
    data = base_data()
    data['moods']['calm']['fld']['brightness'] = 10 ** 400
    rejects(data, 'moods.calm.fld.brightness: out of range')


def test_rejects_a_complex_key():
    with pytest.raises(ExpressionConfigError, match='a key must be a plain name'):
        load_config_string('? [a, b]\n: 1\n')


def test_rejects_duplicate_keys():
    with pytest.raises(ExpressionConfigError, match='duplicate key'):
        load_config_string('neutral: {}\nneutral: {}\nmoods: {}\n')


def test_rejects_invalid_yaml_and_non_mapping():
    with pytest.raises(ExpressionConfigError, match='invalid YAML'):
        load_config_string('moods: [')
    with pytest.raises(ExpressionConfigError, match='must be a mapping'):
        load_config_string('')


def test_rejects_unreadable_file(tmp_path):
    with pytest.raises(ExpressionConfigError, match='cannot read'):
        load_config(str(tmp_path / 'absent.yaml'))


def test_error_type_is_a_value_error():
    assert issubclass(ExpressionConfigError, ValueError)
