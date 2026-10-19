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

"""Unit tests for the affect.yaml loader and validator (no ROS graph)."""

import copy
import os
import sys

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from r2_affect.config import AffectConfigError, AXES  # noqa: E402
from r2_affect.config import load_config, load_config_string, parse_config  # noqa: E402
import yaml  # noqa: E402

SHIPPED = os.path.join(PKG_ROOT, 'config', 'affect.yaml')

# Plan section 5.4, transcribed: what the shipped affect.yaml must contain.
SPEC_5_4 = {
    'temperament': {'arousal': 0.0, 'valence': 0.2, 'curiosity': 0.4, 'confidence': 0.6},
    'relax_tau_s': {'arousal': 45, 'valence': 120, 'curiosity': 60, 'confidence': 300},
    'distance_weights': {'arousal': 1.0, 'valence': 1.0, 'curiosity': 1.0, 'confidence': 1.0},
    'strength_radius': 0.5,
    'hysteresis': {'margin': 0.15, 'dwell_s': 1.5},
    'focus': {'default_ttl_s': 5.0},
    'idle': {'after_s': 180, 'ramp_s': 300},
    'impulses': {
        'button_sequence': {'arousal': 0.2, 'valence': 0.2, 'refractory_s': 1.0},
        'being_driven': {'arousal': 0.3, 'valence': 0.2, 'confidence': 0.1,
                         'refractory_s': 2.0},
        'stopped': {'arousal': -0.1},
        'dome_pushed': {'arousal': 0.4, 'valence': -0.2, 'confidence': -0.2,
                        'refractory_s': 2.0},
        'bump': {'arousal': 0.6, 'valence': -0.4, 'confidence': -0.3, 'refractory_s': 2.0},
        'loud_sound': {'arousal': 0.4, 'curiosity': 0.4},
        'face_known': {'arousal': 0.5, 'valence': 0.6, 'curiosity': 0.3, 'refractory_s': 20},
        'face_unknown': {'arousal': 0.3, 'curiosity': 0.6, 'confidence': -0.2},
        'touched': {'valence': 0.4, 'confidence': 0.1},
        'low_battery': {'arousal': -0.4, 'valence': -0.3},
        'idle': {'arousal': -0.02},
    },
    'moods': {
        'calm': {'arousal': 0.0, 'valence': 0.2},
        'curious': {'arousal': 0.3, 'valence': 0.2, 'curiosity': 0.8},
        'excited': {'arousal': 0.8, 'valence': 0.8},
        'playful': {'arousal': 0.5, 'valence': 0.6, 'confidence': 0.8},
        'annoyed': {'arousal': 0.4, 'valence': -0.6},
        'scared': {'arousal': 0.7, 'valence': -0.6, 'confidence': 0.1},
        'sleepy': {'arousal': -0.8, 'valence': 0.0},
        'sulking': {'arousal': -0.3, 'valence': -0.5},
    },
}

MINIMAL = {
    'temperament': {'arousal': 0.0, 'valence': 0.0, 'curiosity': 0.5, 'confidence': 0.5},
    'relax_tau_s': {'arousal': 1.0, 'valence': 1.0, 'curiosity': 1.0, 'confidence': 1.0},
    'impulses': {'idle': {'arousal': -0.1}, 'poke': {'arousal': 0.5, 'refractory_s': 1.0}},
    'moods': {'calm': {'arousal': 0.0}},
}


def minimal(**changes):
    data = copy.deepcopy(MINIMAL)
    data.update(copy.deepcopy(changes))
    return data


def rejected(data, *fragments):
    """Assert parse_config refuses ``data`` with a message containing ``fragments``."""
    with pytest.raises(AffectConfigError) as info:
        parse_config(data)
    for fragment in fragments:
        assert fragment in str(info.value), str(info.value)
    return str(info.value)


# --- the shipped file -------------------------------------------------------------------

def test_shipped_affect_yaml_matches_plan_section_5_4():
    with open(SHIPPED, encoding='utf-8') as stream:
        assert yaml.safe_load(stream) == SPEC_5_4


def test_shipped_affect_yaml_round_trips_through_the_dataclasses():
    config = load_config(SHIPPED)
    assert config.to_dict() == SPEC_5_4
    assert parse_config(config.to_dict()) == config
    assert parse_config(yaml.safe_load(yaml.safe_dump(config.to_dict()))) == config


def test_shipped_affect_yaml_keeps_mood_file_order():
    config = load_config(SHIPPED)
    assert list(config.moods) == list(SPEC_5_4['moods'])
    assert config.impulses['face_known'].refractory_s == 20.0
    assert config.impulses['stopped'].refractory_s is None
    assert dict(config.impulses['bump'].deltas) == {
        'arousal': 0.6, 'valence': -0.4, 'confidence': -0.3}


def test_config_is_immutable():
    config = load_config(SHIPPED)
    with pytest.raises(AttributeError):
        config.strength_radius = 1.0
    with pytest.raises(TypeError):
        config.temperament['arousal'] = 1.0
    with pytest.raises(TypeError):
        config.moods['calm'].center['arousal'] = 1.0


# --- defaults ---------------------------------------------------------------------------

def test_optional_sections_take_their_defaults_when_absent():
    config = parse_config(minimal())
    assert dict(config.distance_weights) == {axis: 1.0 for axis in AXES}
    assert config.strength_radius == 0.5
    assert config.focus.default_ttl_s == 5.0
    assert (config.idle.after_s, config.idle.ramp_s) == (30.0, 120.0)
    assert (config.hysteresis.margin, config.hysteresis.dwell_s) == (0.15, 1.5)


def test_partly_given_sections_default_the_rest():
    config = parse_config(minimal(distance_weights={'curiosity': 2.0},
                                  hysteresis={'dwell_s': 3.0}, idle={'ramp_s': 10}))
    assert dict(config.distance_weights) == {
        'arousal': 1.0, 'valence': 1.0, 'curiosity': 2.0, 'confidence': 1.0}
    assert (config.hysteresis.margin, config.hysteresis.dwell_s) == (0.15, 3.0)
    assert (config.idle.after_s, config.idle.ramp_s) == (30.0, 10.0)


def test_prototype_may_specify_a_single_axis():
    config = parse_config(minimal(moods={'bold': {'confidence': 1.0}}))
    assert dict(config.moods['bold'].center) == {'confidence': 1.0}


def test_zero_refractory_is_legal():
    data = minimal()
    data['impulses']['poke']['refractory_s'] = 0
    assert parse_config(data).impulses['poke'].refractory_s == 0.0


# --- rejections (plan 5.4) --------------------------------------------------------------

def test_rejects_unknown_top_level_key():
    rejected(minimal(temprament={}), 'temprament', 'unknown top-level key')


@pytest.mark.parametrize('section', ['temperament', 'relax_tau_s', 'distance_weights',
                                     'impulses', 'moods'])
def test_rejects_unknown_axis_anywhere(section):
    data = minimal()
    if section in ('impulses', 'moods'):
        entry = 'poke' if section == 'impulses' else 'calm'
        data[section][entry]['arousl'] = 0.1
        path = '%s.%s.arousl' % (section, entry)
    else:
        data[section] = dict(data.get(section, {}), arousl=0.1)
        path = '%s.arousl' % section
    rejected(data, path, 'unknown axis')


@pytest.mark.parametrize('axis,value', [('arousal', 1.5), ('valence', -1.01),
                                        ('curiosity', -0.1), ('confidence', 1.2)])
def test_rejects_temperament_out_of_range(axis, value):
    data = minimal()
    data['temperament'][axis] = value
    rejected(data, 'temperament.%s' % axis, 'outside')


@pytest.mark.parametrize('axis,value', [('arousal', -1.5), ('curiosity', 1.1)])
def test_rejects_prototype_value_out_of_range(axis, value):
    rejected(minimal(moods={'calm': {'arousal': 0.0}, 'odd': {axis: value}}),
             'moods.odd.%s' % axis, 'outside')


@pytest.mark.parametrize('tau', [0, -1.0])
def test_rejects_non_positive_tau(tau):
    data = minimal()
    data['relax_tau_s']['valence'] = tau
    rejected(data, 'relax_tau_s.valence', '> 0')


def test_rejects_negative_refractory():
    data = minimal()
    data['impulses']['poke']['refractory_s'] = -0.5
    rejected(data, 'impulses.poke.refractory_s', '>= 0')


def test_rejects_empty_moods_table():
    rejected(minimal(moods={}), 'moods', 'at least one')


def test_rejects_missing_idle_impulse():
    rejected(minimal(impulses={'poke': {'arousal': 0.5}}), 'impulses.idle', 'missing')


@pytest.mark.parametrize('key', ['temperament', 'relax_tau_s', 'impulses', 'moods'])
def test_rejects_missing_required_section(key):
    data = minimal()
    del data[key]
    rejected(data, key, 'required')


@pytest.mark.parametrize('section', ['temperament', 'relax_tau_s'])
def test_rejects_missing_axis_in_temperament_or_tau(section):
    data = minimal()
    del data[section]['curiosity']
    rejected(data, '%s.curiosity' % section, 'required axis')


@pytest.mark.parametrize('value', ['fast', True, None, float('nan'), float('inf')])
def test_rejects_non_numeric_or_non_finite_value(value):
    data = minimal()
    data['impulses']['poke']['arousal'] = value
    rejected(data, 'impulses.poke.arousal')


def test_rejects_unknown_key_in_a_section():
    rejected(minimal(hysteresis={'margin': 0.1, 'dwel_s': 1.0}), 'hysteresis.dwel_s',
             'unknown key')


def test_rejects_section_that_is_not_a_mapping():
    rejected(minimal(focus=5.0), 'focus', 'mapping')
    rejected(minimal(impulses={'idle': {'arousal': -0.1}, 'poke': 0.5}), 'impulses.poke',
             'mapping')


def test_rejects_prototype_with_no_axes():
    rejected(minimal(moods={'calm': {'arousal': 0.0}, 'blank': {}}), 'moods.blank',
             'at least one axis')


@pytest.mark.parametrize('key,value,fragment', [
    ('strength_radius', 0.0, '> 0'),
    ('hysteresis', {'margin': -0.1}, '>= 0'),
    ('focus', {'default_ttl_s': 0.0}, '> 0'),
    ('idle', {'ramp_s': 0.0}, '> 0'),
])
def test_rejects_values_the_model_cannot_use(key, value, fragment):
    rejected(minimal(**{key: value}), key, fragment)


@pytest.mark.parametrize('weight', [0.0, -1.0])
def test_rejects_non_positive_distance_weight(weight):
    rejected(minimal(distance_weights={'arousal': 1.0, 'curiosity': weight}),
             'distance_weights.curiosity', '> 0')


def test_rejects_duplicate_keys():
    text = yaml.safe_dump(MINIMAL, sort_keys=False) + 'moods: {calm: {arousal: 0.0}}\n'
    with pytest.raises(AffectConfigError, match="'moods': duplicate key"):
        load_config_string(text)


def test_rejects_invalid_yaml_and_empty_file():
    with pytest.raises(AffectConfigError, match='invalid YAML'):
        load_config_string('moods: [unclosed\n')
    with pytest.raises(AffectConfigError, match='must be a mapping'):
        load_config_string('')


def test_rejects_unreadable_file(tmp_path):
    missing = tmp_path / 'nope.yaml'
    with pytest.raises(AffectConfigError, match='nope.yaml'):
        load_config(str(missing))


def test_error_type_is_a_value_error():
    assert issubclass(AffectConfigError, ValueError)
