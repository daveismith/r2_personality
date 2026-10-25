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

"""Tests for loading, validating and pooling clips.yaml (plan 2.1-2.3, rules 7 and 8)."""

import copy

import pytest
import yaml

from fixtures import tone, write_wav  # noqa: I100
from r2_sound.catalogue import (CatalogueError, effective_tags, load_catalogue, parse_catalogue,
                                pools, TAGS)


def clip(name, **over):
    entry = {'name': name, 'set': 'S', 'path': f'S/{name}.wav', 'duration_ms': 1000,
             'rms_dbfs': -12.0, 'peak_dbfs': -3.0, 'clip_fraction': 0.0, 'set_gain_db': -4.0,
             'gain_db': -4.0, 'tags': ['chatter'], 'suggested_tag': 'chatter',
             'confidence': 1.0, 'excluded': False, 'label': 'chatter'}
    entry.update(over)
    return entry


def data(*clips):
    return {'house_level_dbfs': -16.0, 'generated': '2026-10-24',
            'sets': {'S': {'clips': len(clips), 'median_rms_dbfs': -12.0,
                           'set_gain_db': -4.0}},
            'clips': list(clips) or [clip('a')]}


def rejects(d, fragment):
    with pytest.raises(CatalogueError) as info:
        parse_catalogue(d)
    assert fragment in str(info.value), str(info.value)
    return str(info.value)


def test_the_fixed_tag_list_is_the_plans_nineteen():
    assert TAGS == ('chatter', 'excited', 'sad', 'question', 'ooh', 'hum', 'alarm', 'scream',
                    'laugh', 'annoyed', 'whistle', 'calm', 'scared', 'sleepy', 'greeting',
                    'yes', 'no', 'misc', 'excluded')
    assert len(set(TAGS)) == 19


def test_a_valid_catalogue_parses_into_frozen_dataclasses():
    catalogue, warnings = parse_catalogue(data(clip('a'), clip('b', tags=['ooh', 'hum'])))
    assert warnings == []
    assert [c.name for c in catalogue.clips] == ['a', 'b']
    assert catalogue.clips[1].tags == ('ooh', 'hum')
    assert catalogue.house_level_dbfs == -16.0
    with pytest.raises(AttributeError):
        catalogue.clips[0].gain_db = 3.0


def test_round_trip_through_to_dict():
    catalogue, _ = parse_catalogue(data(clip('a'), clip('b', tags=['sad'])))
    assert parse_catalogue(catalogue.to_dict())[0] == catalogue


def test_duplicate_names_are_refused():
    message = rejects(data(clip('a'), clip('a')), 'duplicate name')
    assert message.startswith('clips[1].name')


def test_unknown_keys_are_refused_with_their_path():
    d = data()
    d['clips'][0]['volume'] = 1
    assert rejects(d, 'unknown key').startswith('clips[0].volume')
    d = data()
    d['extra'] = 1
    assert rejects(d, 'unknown key').startswith('extra')
    d = data()
    d['sets']['S']['loud'] = 1
    assert rejects(d, 'unknown key').startswith('sets.S.loud')


def test_missing_required_keys_are_refused():
    d = data()
    del d['clips'][0]['gain_db']
    assert rejects(d, 'is required').startswith('clips[0].gain_db')
    d = data()
    del d['generated']
    rejects(d, 'is required')


def test_tags_outside_the_fixed_list_are_refused():
    assert rejects(data(clip('a', tags=['chatter', 'grumpy'])), 'grumpy').startswith(
        'clips[0].tags[1]')
    rejects(data(clip('a', suggested_tag='nope')), 'suggested_tag')
    rejects(data(clip('a', tags=[])), 'non-empty')


@pytest.mark.parametrize('gain_db, ok', [(-20.0, True), (12.0, True), (-20.5, False),
                                         (12.5, False)])
def test_gain_db_must_be_within_minus_20_and_plus_12(gain_db, ok):
    d = data(clip('a', gain_db=gain_db))
    if ok:
        parse_catalogue(d)
    else:
        rejects(d, 'gain_db')


@pytest.mark.parametrize('confidence', [-0.1, 1.01])
def test_confidence_must_be_within_0_and_1(confidence):
    rejects(data(clip('a', confidence=confidence)), 'confidence')


def test_the_excluded_tag_requires_the_excluded_flag():
    rejects(data(clip('a', tags=['excluded'], excluded=False)), 'excluded')


def test_names_must_be_lower_case_and_other_types_are_checked():
    rejects(data(clip('Loud')), 'lower-case')
    rejects(data(clip('a', excluded='no')), 'excluded')
    rejects(data(clip('a', duration_ms=1.5)), 'duration_ms')
    rejects(data(clip('a', clip_fraction=2.0)), 'clip_fraction')
    d = data()
    d['clips'][0]['set'] = 'Other'
    rejects(d, 'not in sets')


def test_missing_files_are_dropped_with_a_warning_not_an_error(tmp_path):
    write_wav(tmp_path / 'S' / 'a.wav', tone())
    catalogue, warnings = parse_catalogue(data(clip('a'), clip('b')), str(tmp_path))
    assert [c.name for c in catalogue.clips] == ['a']
    assert len(warnings) == 1 and 'b' in warnings[0] and 'S/b.wav' in warnings[0]


def test_every_file_missing_gives_an_empty_catalogue_and_warnings(tmp_path):
    catalogue, warnings = parse_catalogue(data(clip('a'), clip('b')), str(tmp_path))
    assert catalogue.clips == () and len(warnings) == 2


def test_load_catalogue_reads_a_file_and_names_it_in_errors(tmp_path):
    path = tmp_path / 'clips.yaml'
    path.write_text(yaml.safe_dump(data(clip('a'))))
    catalogue, warnings = load_catalogue(str(path), str(tmp_path))
    assert catalogue.clips == () and len(warnings) == 1
    bad = copy.deepcopy(data())
    bad['clips'][0]['confidence'] = 7
    path.write_text(yaml.safe_dump(bad))
    with pytest.raises(CatalogueError, match='clips.yaml: clips\\[0\\].confidence'):
        load_catalogue(str(path))


# --- the merge rule ------------------------------------------------------------------------

def test_sheet_tags_win_over_a_confident_suggestion():
    assert effective_tags(('sad', 'calm'), 'chatter', 1.0) == ('sad', 'calm')


def test_a_confident_suggestion_is_used_without_sheet_tags():
    assert effective_tags((), 'ooh', 0.6) == ('ooh',)        # at the threshold counts
    assert effective_tags((), 'ooh', 0.95, 0.9) == ('ooh',)


def test_a_weak_suggestion_falls_to_chatter():
    assert effective_tags((), 'ooh', 0.59) == ('chatter',)
    assert effective_tags((), 'question', 0.4) == ('chatter',)
    assert effective_tags((), 'ooh', 0.7, threshold=0.8) == ('chatter',)


# --- pools ---------------------------------------------------------------------------------

def test_pools_map_each_tag_to_its_playable_clips():
    catalogue, _ = parse_catalogue(data(
        clip('a', tags=['chatter']),
        clip('b', tags=['chatter', 'ooh']),
        clip('c', tags=['ooh'], excluded=True),
        clip('d', tags=['excluded'], excluded=True)))
    p = pools(catalogue)
    assert [c.name for c in p['chatter']] == ['a', 'b']
    assert [c.name for c in p['ooh']] == ['b']
    assert p['excluded'] == ()
    assert p['sad'] == ()
    assert set(p) == set(TAGS)
