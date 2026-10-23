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

"""Tests for the palette helpers: styles by label and strength, pick validation, scheduling."""

import json
import math

import pytest

from fixtures import base_data, cfg, dome_cfg, FixedRng, LED_CAT, MAGIC_CAT  # noqa: I100
from r2_expression.config import parse_config
from r2_expression.palette import (choose, FlourishScheduler, glance_intent_fields,
                                   GlancePick, GlanceScheduler, led_catalogue,
                                   magic_catalogue, pick_intent_fields, resolve_styles,
                                   scale_brightness, unit_strength, validate_picks)


@pytest.mark.parametrize('strength, expected', [(0.0, 50), (1.0, 60), (0.5, 55), (0.26, 53),
                                                (2.0, 60), (-1.0, 50), (math.nan, 50),
                                                (None, 50)])
def test_palette_strength_scales_brightness_between_min_and_max(strength, expected):
    assert scale_brightness((50, 60), strength) == expected


def test_palette_selection_by_label():
    styles = resolve_styles(cfg(), 'excited', 1.0)
    assert styles['fld'].drift == 'lively' and styles['fld'].brightness == 100
    assert styles['psi'].drift is None and styles['psi'].brightness == 100
    assert styles['magic'].brightness == 12
    calm = resolve_styles(cfg(), 'calm', 0.0)
    assert (calm['rld'].drift, calm['rld'].brightness) == ('calm', 50)


def test_palette_unknown_label_and_stale_are_neutral():
    for styles in (resolve_styles(cfg(), 'sleepy', 1.0),
                   resolve_styles(cfg(), 'excited', 1.0, stale=True)):
        assert styles['fld'].brightness == 40 and styles['fld'].drift == 'calm'
        assert styles['psi'].brightness == 40
        assert 'magic' not in styles       # neutral leaves the Magic Panel untouched


def test_palette_channel_left_out_of_a_mood_takes_neutral():
    data = base_data()
    del data['moods']['excited']['rld']
    styles = resolve_styles(parse_config(data), 'excited', 1.0)
    assert (styles['rld'].drift, styles['rld'].brightness) == ('calm', 40)


def test_catalogues_build_from_list_responses():
    led = led_catalogue({'effects': ['fire'], 'clips': [
        {'name': 'pulse', 'target': 'fld'}, {'name': 'pulse', 'target': 'psi'}]})
    assert led.has('effect', 'fire', 'fld') and not led.has('effect', 'pulse', 'fld')
    assert led.has('clip', 'pulse', 'psi') and not led.has('clip', 'pulse', 'rld')
    assert not led.has('clip', 'pulse', 'tfld')
    magic = magic_catalogue({'sequences': [{'id': 44, 'name': 'Smile'}, {'id': 3, 'name': ''}]})
    assert magic.resolve('Smile') == 44 and magic.resolve(3) == 3
    assert magic.resolve('') is None and magic.resolve(9) is None
    assert magic.unbounded == frozenset()
    flagged = magic_catalogue({'sequences': [
        {'id': 56, 'name': 'Random show', 'flags': 3}, {'id': 25, 'name': 'Fade', 'flags': 2},
        {'id': 7, 'name': 'Loop', 'flags': 1}, {'id': 44, 'name': 'Smile', 'flags': 0}]})
    assert flagged.unbounded == frozenset({56, 7})    # RANDOM alone ends by its length


def test_listleds_validation_drops_a_pick_the_device_lacks_with_one_warning():
    data = base_data()
    data['moods']['calm']['flourish']['picks'] = [
        {'channel': 'fld', 'clip': 'pulse'}, {'channel': 'psi', 'clip': 'sparkle'},
        {'channel': 'rld', 'effect': 'nope'}, {'channel': 'magic', 'sequence': 99}]
    picks, warnings = validate_picks(parse_config(data), led=LED_CAT)
    assert [p.name for p in picks['calm']] == ['pulse', 99]    # magic is not judged here
    assert len(warnings) == 2
    assert 'moods.calm.flourish.picks[1]' in warnings[0][1] and 'sparkle' in warnings[0][1]
    picks, warnings = validate_picks(parse_config(data), magic=MAGIC_CAT)
    assert [p.name for p in picks['calm']] == ['pulse', 'sparkle', 'nope']
    assert 'sequence 99' in warnings[0][1]


def test_flourish_scheduler_fires_within_every_s():
    mood = cfg().moods['calm']                     # every_s [10, 20]
    rng = FixedRng(0.5)
    sched = FlourishScheduler(rng)
    sched.restart(mood.flourish, 100.0)
    assert sched.next_due == 115.0
    assert sched.poll(mood.flourish.picks, 114.9) is None
    assert sched.poll(mood.flourish.picks, 115.0) is mood.flourish.picks[0]
    assert sched.next_due == 130.0
    sched.restart(None, 130.0)
    assert sched.next_due is None and sched.poll(mood.flourish.picks, 1e9) is None


def test_flourish_choice_is_weighted_and_deterministic():
    picks = cfg().moods['excited'].flourish.picks  # weights 1, 1
    assert choose(picks, FixedRng(0.0)) is picks[0]
    assert choose(picks, FixedRng(0.49)) is picks[0]
    assert choose(picks, FixedRng(0.5)) is picks[1]
    assert choose((), FixedRng(0.5)) is None


def test_pick_becomes_an_idle_intent_with_its_duration():
    calm, excited = cfg().moods['calm'], cfg().moods['excited']
    fields = pick_intent_fields(calm.flourish.picks[0], 'flourish-1')
    assert fields['source'] == 'idle' and fields['priority'] == 10
    assert fields['action'] == 'clip' and fields['duration_ms'] == 2000
    assert json.loads(fields['params']) == {'name': 'pulse', 'rgb': 0x2060ff}
    fields = pick_intent_fields(excited.flourish.picks[1], 'flourish-2')
    assert fields['action'] == 'sequence' and json.loads(fields['params']) == {'id': 27}


# --- idle glances (phase 2 plan 4.5) -------------------------------------------------------

class Sequence:
    """An RNG that returns ``values`` in turn."""

    def __init__(self, *values):
        self.values = list(values)

    def random(self):
        return self.values.pop(0)


@pytest.mark.parametrize('strength, expected', [(0.3, 0.3), (-1.0, 0.0), (7.0, 1.0),
                                                (math.nan, 0.0), ('x', 0.0)])
def test_unit_strength_clamps_to_0_1(strength, expected):
    assert unit_strength(strength) == expected


def test_glance_scheduler_fires_within_every_s_scaled_toward_the_lower_bound_by_strength():
    glance = dome_cfg().moods['calm'].dome.glance         # every_s [10, 20]
    sched = GlanceScheduler(FixedRng(0.5))
    sched.restart(glance, 100.0, strength=0.0)
    assert sched.next_due == 115.0                        # the whole window
    sched.restart(glance, 100.0, strength=0.5)
    assert sched.next_due == 112.5                        # [10, 15]
    sched.restart(glance, 100.0, strength=1.0)
    assert sched.next_due == 110.0                        # always the lower bound
    assert sched.poll(109.9) is None
    pick = sched.poll(110.0, strength=0.0)
    assert sched.next_due == 125.0                        # the next drawn at strength 0
    assert pick == GlancePick(amplitude_rad=0.4, dwell_s=0.75, side='right')
    sched.stop()
    assert sched.poll(1e9) is None
    sched.restart(None, 0.0)
    assert sched.next_due is None


def test_glance_draws_amplitude_dwell_and_side_from_the_mood():
    glance = dome_cfg().moods['calm'].dome.glance         # amplitude [0.3, 0.5], dwell [0.5, 1]
    sched = GlanceScheduler(Sequence(0.0, 0.0, 1.0, 0.0, 0.49))   # due, next, the draw
    sched.restart(glance, 0.0, strength=1.0)
    assert sched.poll(10.0) == GlancePick(amplitude_rad=0.5, dwell_s=0.5, side='left')


def test_glance_becomes_an_idle_dome_intent():
    fields = glance_intent_fields(GlancePick(0.4, 1.5, 'left'), 'glance-3')
    assert (fields['channel'], fields['source'], fields['priority'], fields['action']) == (
        'dome', 'idle', 10, 'glance')
    assert json.loads(fields['params']) == {'amplitude': 0.4, 'dwell_s': 1.5, 'side': 'left'}
    assert fields['intent_id'] == 'glance-3' and fields['interruptible']
