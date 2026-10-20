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
import os

import pytest
import yaml

from fixtures import AFFECT_YAML, base_data, cfg, LEDS_LIST, SHIPPED  # noqa: I100
from fixtures import capture_names
from r2_expression.config import (ExpressionConfigError, load_config, load_config_string,
                                  missing_moods, parse_config)
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


def test_shipped_flourishes_never_carry_duration_zero_and_lightsout_is_timed():
    for mood in load_config(SHIPPED).moods.values():
        for pick in mood.flourish.picks:
            assert pick.duration_ms and pick.duration_ms <= 30000
            if pick.name == 'lightsout':
                assert pick.duration_ms


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


@pytest.mark.parametrize('channel', ['tfld', 'bfld', 'dome', 'fpsi'])
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
