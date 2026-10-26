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

"""Tests for the catalogue builder tool, on synthetic WAV trees (never /sounds)."""

import csv
import os
import types
import wave

import numpy as np
import pytest
import soundfile
import yaml

from fixtures import (burst_train, chirp, make_duplicate_tree, make_tree,  # noqa: I100
                      RATE, tone, write_wav)
from r2_sound import analysis, gain
from r2_sound.catalogue import CatalogueError, load_catalogue, pools
from r2_sound.tools import build_catalogue as bc


@pytest.fixture
def tree(tmp_path):
    return make_tree(tmp_path / 'sounds')


def run(tree, tmp_path, **kw):
    out = str(tmp_path / 'config' / 'clips.yaml')
    sheet = str(tmp_path / 'config' / 'tags.csv')
    kit = str(tmp_path / 'kit')
    kw.setdefault('today', '2026-10-24')
    catalogue, warnings = bc.build(tree, out, sheet, kit, **kw)
    return catalogue, warnings, out, sheet, kit


def by_name(catalogue):
    return catalogue.by_name()


def snapshot(tmp_path):
    files = {}
    for root, _, names in os.walk(str(tmp_path)):
        if os.path.relpath(root, str(tmp_path)).startswith('sounds'):
            continue
        for name in names:
            path = os.path.join(root, name)
            with open(path, 'rb') as f:
                files[os.path.relpath(path, str(tmp_path))] = f.read()
    return files


def read_sheet_rows(sheet):
    with open(sheet, newline='') as f:
        return list(csv.DictReader(f))


# --- names and labels ----------------------------------------------------------------------

@pytest.mark.parametrize('set_name, rel, name', [
    ('R2 Padawan', '32SENT1.mp3', 'padawan-sent01'),
    ('R2 Padawan', '40SENT19.mp3', 'padawan-sent19'),
    ('R2 Padawan', '1SCREAM2.mp3', 'padawan-scream02'),
    ('R2 Padawan', '3DOODOO.mp3', 'padawan-doodoo'),
    ('R2 Padawan', 'clean Leia/5LEIA.mp3', 'padawan-leia'),
    ('R2 Padawan', 'leia-hologram-cleaned-amped-up.mp3', 'padawan-leia-hologram-cleaned-amped-up'),
    ('R2 sounds and music', '140.mp3', 'music-140'),
    ('R2 sounds and music', '2SecOfSilence.mp3', 'music-2secofsilence'),
    ('R2 sounds and music', 'Scream.mp3', 'music-scream'),
    ('mp3-trigger', '001 gen-1.mp3', 'trigger-001-gen-1'),
    ('mp3-trigger', '185 birthday.mp3', 'trigger-185-birthday'),    # 192 BIRTHDAY differs
    ('mp3-trigger', '192 BIRTHDAY.mp3', 'trigger-192-birthday'),
    ('Some Other Set', 'Hello World.mp3', 'some-other-set-hello-world')])
def test_name_derivation(set_name, rel, name):
    assert bc.base_name(set_name, rel) == name


def test_repeated_names_get_a_numeric_suffix(tmp_path):
    root = tmp_path / 's'
    write_wav(str(root / 'R2 Padawan' / 'a' / '5LEIA.wav'), tone())
    write_wav(str(root / 'R2 Padawan' / 'b' / '5LEIA.wav'), tone())
    names = [i.name for i in bc._unique_names(bc._scan(str(root)))]
    assert names == ['padawan-leia', 'padawan-leia-2']


def test_repeat_suffixes_follow_the_sorted_paths(tmp_path):
    root = tmp_path / 's'
    write_wav(str(root / 'R2 Padawan' / '5LEIA.wav'), tone())       # os.walk: this first
    write_wav(str(root / 'R2 Padawan' / '0' / '5LEIA.wav'), tone())
    items = bc._unique_names(bc._scan(str(root)))
    assert [(i.path, i.name) for i in items] == [
        ('R2 Padawan/0/5LEIA.wav', 'padawan-leia'), ('R2 Padawan/5LEIA.wav', 'padawan-leia-2')]


@pytest.mark.parametrize('rel, label', [
    ('32SENT1.mp3', 'chatter'), ('25OOH3.mp3', 'ooh'), ('52HUM19.mp3', 'hum'),
    ('13ALARM3.mp3', 'alarm'), ('1SCREAM2.mp3', 'scream'), ('2CHORTLE.mp3', 'laugh'),
    ('8ANNOYED.mp3', 'annoyed'), ('4WOLFWSTL.mp3', 'whistle'), ('17MISC3.mp3', 'misc'),
    ('Powerup.mp3', 'misc'), ('3DOODOO.mp3', 'misc'), ('6SHORTCKT.mp3', 'misc'),
    ('7PATROL1.mp3', 'misc'), ('Scream.mp3', 'scream'), ('140.mp3', ''), ('Hello.mp3', ''),
    ('001 gen-1.mp3', 'chatter'), ('026 chat-1.mp3', 'chatter'), ('051 happy-1.mp3', 'excited'),
    ('076 sad-1.mp3', 'sad'), ('101 whis-1.mp3', 'whistle'), ('126 screa-1.mp3', 'scream'),
    ('255 start.mp3', '')])
def test_label_mapping(rel, label):
    assert bc.label_of(rel) == label


@pytest.mark.parametrize('rel', [
    '9Theme.mp3', '12Chorus.mp3', '11Emperor.mp3', '10Cantina.mp3', 'Cantina.mp3', 'March.mp3',
    'SWDiscoShort.mp3', 'StarWars.mp3', '5LEIA.mp3', 'clean Leia/x.mp3',
    'leia-hologram-cleaned-amped-up.mp3', '2SecOfSilence.mp3', '151 leia-1.mp3',
    '176 mus-1.mp3', '180 cantina song.mp3', '182 gangnamstyle.mp3', '185 birthday.mp3',
    '186 harlemshake.mp3', '187 in love.mp3', '188 macho man.mp3',
    '189 Peanut Butter Jelly Time.mp3', '190 UptownFunk.mp3', '191 Celebration.mp3',
    '193 R2TANGO.mp3', '194 R2THEME.mp3', '195 R2THRONE.mp3'])
def test_the_exclusion_list(rel):
    assert bc.label_of(rel) == 'excluded'


# --- the run -------------------------------------------------------------------------------

def test_build_measures_names_labels_and_excludes(tree, tmp_path):
    catalogue, warnings, out, sheet, kit = run(tree, tmp_path)
    assert warnings == []
    clips = by_name(catalogue)
    assert sorted(clips) == sorted(
        ['padawan-sent01', 'padawan-sent02', 'padawan-ooh01', 'padawan-patrol01',
         'padawan-scream02', 'padawan-theme', 'music-1', 'music-2', 'music-3', 'music-4',
         'music-march'])
    assert [c.name for c in catalogue.clips] == sorted(clips)
    assert clips['padawan-theme'].excluded and clips['music-march'].excluded
    assert clips['padawan-sent01'].label == 'chatter'
    assert clips['padawan-sent01'].suggested_tag == 'chatter'
    assert clips['padawan-sent01'].confidence == 1.0
    assert clips['padawan-patrol01'].tags == ('misc',)
    assert clips['padawan-theme'].tags == ('excluded',)
    sent = clips['padawan-sent01']
    assert sent.duration_ms == 1000
    assert sent.rms_dbfs == pytest.approx(-13.52, abs=0.1)         # 0.3 sine
    assert sent.path == 'R2 Padawan/32SENT1.wav'
    assert not clips['music-4'].label


def test_sets_are_aligned_to_the_house_level_each(tree, tmp_path):
    catalogue, *_ = run(tree, tmp_path)
    assert catalogue.house_level_dbfs == -16.0
    for name, info in catalogue.sets.items():
        playable = [c.rms_dbfs for c in catalogue.clips if c.set_name == name and not c.excluded]
        assert info.median_rms_dbfs == pytest.approx(gain.median(playable), abs=0.01)
        assert info.median_rms_dbfs + info.set_gain_db == pytest.approx(-16.0, abs=0.02)
    assert catalogue.sets['R2 Padawan'].set_gain_db != catalogue.sets[
        'R2 sounds and music'].set_gain_db


def test_gain_keeps_the_peak_under_the_ceiling(tree, tmp_path):
    catalogue, *_ = run(tree, tmp_path)
    for c in catalogue.clips:
        assert c.peak_dbfs + c.gain_db <= -1.0 + 0.01, c.name


def test_unlabelled_clips_get_a_suggestion_and_the_rules_fire(tree, tmp_path):
    catalogue, *_ = run(tree, tmp_path)
    clips = by_name(catalogue)
    assert clips['music-1'].suggested_tag == 'question'
    assert clips['music-2'].suggested_tag == 'sad'
    assert clips['music-3'].suggested_tag == 'excited'
    for name in ('music-1', 'music-2', 'music-3'):
        assert clips[name].confidence <= 0.5
        assert clips[name].tags == ('chatter',)         # below the 0.6 threshold


def test_the_threshold_decides_between_suggestion_and_chatter(tree, tmp_path):
    low, *_ = run(tree, tmp_path, threshold=0.0)
    assert by_name(low)['music-1'].tags == ('question',)
    high, *_ = run(tree, tmp_path, threshold=1.01)
    assert by_name(high)['music-1'].tags == ('chatter',)
    assert by_name(high)['padawan-ooh01'].tags == ('chatter',)    # even a labelled one


def test_the_sheet_has_every_clip_and_the_plan_columns(tree, tmp_path):
    catalogue, _, _, sheet, _ = run(tree, tmp_path)
    with open(sheet) as f:
        assert f.readline().strip() == ('name,set,path,duration_s,rms_dbfs,suggested_tag,'
                                        'confidence,tags,clip_override_db,duplicate_of,'
                                        'suggested_duplicate_of,duplicate_score,'
                                        'duplicate_timing,status')
    rows = read_sheet_rows(sheet)
    assert [r['name'] for r in rows] == [c.name for c in catalogue.clips]
    assert all(r['status'] == '' for r in rows)
    assert all(r['suggested_tag'] and r['confidence'] for r in rows)
    assert all(r['tags'] == '' and r['clip_override_db'] == '' for r in rows)
    assert rows[0]['path'].count('/') == 1


def edit_sheet(sheet, **edits):
    rows = read_sheet_rows(sheet)
    for row in rows:
        row.update(edits.get(row['name'], {}))
    with open(sheet, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def test_the_sheet_merges_tags_and_overrides_and_keeps_them(tree, tmp_path):
    first, _, out, sheet, _ = run(tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': 'Calm, sleepy', 'clip_override_db': '3.5'},
                         'music-1': {'tags': 'greeting'},
                         'padawan-sent01': {'tags': 'excluded'}})
    second, *_ = run(tree, tmp_path)
    a, b = by_name(first)['music-4'], by_name(second)['music-4']
    assert b.tags == ('calm', 'sleepy')
    assert b.gain_db == pytest.approx(min(a.gain_db + 3.5, -1.0 - a.peak_dbfs), abs=0.02)
    assert by_name(second)['music-1'].tags == ('greeting',)
    assert by_name(second)['music-1'].suggested_tag == 'question'     # the suggestion stays
    assert by_name(second)['padawan-sent01'].excluded
    assert pools(second)['calm'] == (b,)
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    assert rows['music-4']['tags'] == 'calm,sleepy'
    assert rows['music-4']['clip_override_db'] == '3.5'
    run(tree, tmp_path)
    assert {r['name']: r for r in read_sheet_rows(sheet)} == rows


def test_a_bad_tag_or_override_in_the_sheet_is_refused_with_its_line(tree, tmp_path):
    _, _, _, sheet, _ = run(tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': 'grumpy'}})
    with pytest.raises(CatalogueError, match=r'tags.csv:\d+: tags: .grumpy.'):
        run(tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': '', 'clip_override_db': 'loud'}})
    with pytest.raises(CatalogueError, match='clip_override_db'):
        run(tree, tmp_path)


def write_sheet_rows(sheet, rows):
    with open(sheet, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


@pytest.mark.parametrize('column', ['name', 'path'])
def test_two_sheet_rows_with_one_name_or_path_are_refused_with_both_lines(
        tree, tmp_path, column):
    _, _, _, sheet, _ = run(tree, tmp_path)
    rows = read_sheet_rows(sheet)
    first = next(i for i, r in enumerate(rows) if r['name'] == 'music-4')
    copy = dict(rows[1 if first != 1 else 2])
    copy[column] = rows[first][column]
    copy['name' if column == 'path' else 'path'] += '-x'   # only the one column is shared
    rows.append(copy)
    write_sheet_rows(sheet, rows)
    line = len(rows) + 1                                   # the header is line 1
    with pytest.raises(CatalogueError, match=r'tags.csv:%d: %s: .%s. is also on line %d' % (
            line, column, rows[first][column].replace('/', '.'), first + 2)):
        run(tree, tmp_path)


def test_a_missing_row_may_share_a_name(tmp_path):
    sheet = str(tmp_path / 'tags.csv')
    empty = {'tags': '', 'clip_override_db': ''}
    write_sheet_rows(sheet, [
        {'name': 'a', 'path': 'S/x/a.wav', **empty, 'status': 'missing'},
        {'name': 'a', 'path': 'S/y/a.wav', **empty, 'status': ''}])
    assert [r['path'] for r in bc.read_sheet(sheet)] == ['S/x/a.wav', 'S/y/a.wav']


def test_a_sheet_row_for_a_vanished_clip_is_kept_and_its_tag_comes_back(tree, tmp_path):
    _, _, _, sheet, _ = run(tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': 'calm', 'clip_override_db': '2'}})
    wav = os.path.join(tree, 'R2 sounds and music', '4.wav')
    with open(wav, 'rb') as f:
        audio = f.read()
    os.remove(wav)
    catalogue, warnings, *_ = run(tree, tmp_path)
    assert 'music-4' not in by_name(catalogue)
    assert any('music-4' in w and 'missing' in w for w in warnings)
    row = {r['name']: r for r in read_sheet_rows(sheet)}['music-4']
    assert (row['tags'], row['clip_override_db'], row['status']) == ('calm', '2', 'missing')
    assert row['path'] == 'R2 sounds and music/4.wav' and row['rms_dbfs']
    run(tree, tmp_path)                                    # still gone: still kept
    assert {r['name']: r for r in read_sheet_rows(sheet)}['music-4'] == row
    with open(wav, 'wb') as f:
        f.write(audio)
    catalogue, warnings, *_ = run(tree, tmp_path)
    assert warnings == [] and by_name(catalogue)['music-4'].tags == ('calm',)
    row = {r['name']: r for r in read_sheet_rows(sheet)}['music-4']
    assert (row['tags'], row['status']) == ('calm', '')


def test_sheet_rows_follow_their_path_when_names_shift(tmp_path):
    root = tmp_path / 'sounds'
    a = root / 'R2 Padawan' / 'a' / '5LEIA.wav'
    write_wav(str(a), tone(1000))
    write_wav(str(root / 'R2 Padawan' / 'b' / '5LEIA.wav'), tone(1500))
    _, _, _, sheet, _ = run(str(root), tmp_path)
    edit_sheet(sheet, **{'padawan-leia': {'tags': 'hum'}, 'padawan-leia-2': {'tags': 'sad'}})
    os.remove(str(a))                                      # b is now padawan-leia
    catalogue, *_ = run(str(root), tmp_path)
    assert by_name(catalogue)['padawan-leia'].tags == ('sad',)
    rows = read_sheet_rows(sheet)
    assert [(r['name'], r['path'], r['tags'], r['status']) for r in rows] == [
        ('padawan-leia', 'R2 Padawan/a/5LEIA.wav', 'hum', 'missing'),
        ('padawan-leia', 'R2 Padawan/b/5LEIA.wav', 'sad', '')]
    write_wav(str(a), tone(1000))
    catalogue, *_ = run(str(root), tmp_path)
    assert by_name(catalogue)['padawan-leia'].tags == ('hum',)
    assert by_name(catalogue)['padawan-leia-2'].tags == ('sad',)


def test_a_row_without_a_path_or_with_a_moved_file_is_matched_by_name(tree, tmp_path):
    _, _, _, sheet, _ = run(tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': 'calm'},
                         'music-1': {'tags': 'greeting', 'path': ''}})
    sub = os.path.join(tree, 'R2 sounds and music', 'moved')
    os.makedirs(sub)
    os.rename(os.path.join(tree, 'R2 sounds and music', '4.wav'), os.path.join(sub, '4.wav'))
    catalogue, warnings, *_ = run(tree, tmp_path)
    assert warnings == []
    assert by_name(catalogue)['music-4'].tags == ('calm',)
    assert by_name(catalogue)['music-4'].path == 'R2 sounds and music/moved/4.wav'
    assert by_name(catalogue)['music-1'].tags == ('greeting',)


def test_the_sheet_wins_over_the_exclusion_list(tree, tmp_path):
    first, _, _, sheet, _ = run(tree, tmp_path)
    assert by_name(first)['padawan-theme'].excluded
    edit_sheet(sheet, **{'padawan-theme': {'tags': 'hum'},
                         'music-march': {'tags': 'excluded'}})
    second, *_ = run(tree, tmp_path)
    theme = by_name(second)['padawan-theme']
    assert not theme.excluded and theme.tags == ('hum',) and theme in pools(second)['hum']
    assert theme.suggested_tag == 'excluded'
    assert by_name(second)['music-march'].excluded


def test_a_file_that_cannot_be_decoded_stops_the_run_and_keeps_the_sheet(tree, tmp_path):
    _, _, _, sheet, _ = run(tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': 'calm'}})
    before = snapshot(tmp_path)
    with open(os.path.join(tree, 'R2 sounds and music', '5.wav'), 'wb') as f:
        f.write(b'not audio')
    with pytest.raises(CatalogueError, match='5.wav: cannot decode'):
        run(tree, tmp_path)
    assert snapshot(tmp_path) == before


def test_a_byte_order_mark_or_a_missing_header_in_the_sheet_is_handled(tree, tmp_path):
    _, _, _, sheet, _ = run(tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': 'calm'}})
    with open(sheet, encoding='utf-8') as f:
        text = f.read()
    with open(sheet, 'w', encoding='utf-8-sig') as f:
        f.write(text)
    catalogue, *_ = run(tree, tmp_path)
    assert by_name(catalogue)['music-4'].tags == ('calm',)
    with open(sheet, 'w') as f:
        f.write('name,set\nmusic-4,x\n')
    with pytest.raises(CatalogueError, match='lacks'):
        run(tree, tmp_path)


def test_a_non_finite_override_is_refused(tree, tmp_path):
    _, _, _, sheet, _ = run(tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'clip_override_db': 'nan'}})
    with pytest.raises(CatalogueError, match='clip_override_db'):
        run(tree, tmp_path)


def test_the_kit_prune_leaves_foreign_folders_alone(tree, tmp_path):
    _, _, _, _, kit = run(tree, tmp_path)
    foreign = os.path.join(kit, 'out')
    write_wav(os.path.join(foreign, 'keep.wav'), tone())
    run(tree, tmp_path)
    assert os.path.exists(os.path.join(foreign, 'keep.wav'))


def test_an_empty_sounds_dir_is_an_error(tmp_path):
    (tmp_path / 'empty').mkdir()
    with pytest.raises(CatalogueError, match='no audio files'):
        bc.build(str(tmp_path / 'empty'), str(tmp_path / 'clips.yaml'))


def test_the_written_catalogue_loads_and_drops_what_is_missing(tree, tmp_path):
    _, _, out, *_ = run(tree, tmp_path)
    catalogue, warnings = load_catalogue(out, tree)
    assert warnings == [] and len(catalogue.clips) == 11
    os.remove(os.path.join(tree, 'R2 Padawan', '32SENT1.wav'))
    catalogue, warnings = load_catalogue(out, tree)
    assert len(catalogue.clips) == 10 and 'padawan-sent01' in warnings[0]


def test_the_pools_of_a_built_catalogue(tree, tmp_path):
    catalogue, *_ = run(tree, tmp_path)
    p = pools(catalogue)
    assert {c.name for c in p['chatter']} >= {'padawan-sent01', 'padawan-sent02', 'music-4'}
    assert [c.name for c in p['ooh']] == ['padawan-ooh01']
    assert all(not c.excluded for clips in p.values() for c in clips)


# --- idempotence and the kit ---------------------------------------------------------------

def test_a_second_run_changes_nothing_not_even_on_another_day(tree, tmp_path):
    run(tree, tmp_path, today='2026-10-24')
    before = snapshot(tmp_path)
    run(tree, tmp_path, today='2026-11-02')
    assert snapshot(tmp_path) == before
    assert "generated: '2026-10-24'" in before[os.path.join('config', 'clips.yaml')].decode()


def test_the_generated_date_moves_when_the_content_does(tree, tmp_path):
    run(tree, tmp_path, today='2026-10-24')
    write_wav(os.path.join(tree, 'R2 sounds and music', '6.wav'), tone(900, 1.0, 0.4))
    catalogue, *_ = run(tree, tmp_path, today='2026-11-02')
    assert catalogue.generated == '2026-11-02'


def test_the_kit_groups_wavs_by_suggested_bucket(tree, tmp_path):
    catalogue, _, _, _, kit = run(tree, tmp_path)
    for c in catalogue.clips:
        path = os.path.join(kit, c.suggested_tag, f'{c.name}.wav')
        info = soundfile.info(path)
        assert (info.samplerate, info.channels, info.subtype) == (22050, 1, 'PCM_16')
    script = os.path.join(kit, 'play_bucket.sh')
    assert os.access(script, os.X_OK)
    text = open(script).read()
    assert 'afplay' in text and 'basename' in text
    readme = open(os.path.join(kit, 'README.md')).read()
    assert 'chatter, excited, sad, question' in readme and 'tags.csv' in readme


def test_long_clips_are_cut_to_ten_seconds_in_the_kit(tree, tmp_path):
    write_wav(os.path.join(tree, 'R2 sounds and music', '7.wav'), tone(500, 12.0, 0.3), 8000)
    _, _, _, _, kit = run(tree, tmp_path)
    found = [f for d in os.listdir(kit) if os.path.isdir(os.path.join(kit, d))
             for f in [os.path.join(kit, d, 'music-7.wav')] if os.path.exists(f)]
    assert len(found) == 1
    info = soundfile.info(found[0])
    assert info.samplerate == 22050 and info.duration == pytest.approx(10.0, abs=0.01)


def test_a_changed_bucket_moves_the_kit_file(tree, tmp_path):
    _, _, _, _, kit = run(tree, tmp_path, threshold=0.0)
    assert os.path.exists(os.path.join(kit, 'question', 'music-1.wav'))
    os.remove(os.path.join(tree, 'R2 sounds and music', '1.wav'))
    run(tree, tmp_path)
    assert not os.path.exists(os.path.join(kit, 'question', 'music-1.wav'))


def test_main_runs_the_tool_and_prints_the_headline(tree, tmp_path, capsys):
    out = str(tmp_path / 'c' / 'clips.yaml')
    code = bc.main(['--sounds-dir', tree, '--out', out, '--sheet', str(tmp_path / 't.csv'),
                    '--kit', str(tmp_path / 'kit')])
    assert code == 0
    printed = capsys.readouterr().out
    assert '11 clips, 2 excluded' in printed and 'set_gain_db' in printed
    assert bc.main(['--sounds-dir', str(tmp_path / 'nope'), '--out', out]) == 1


def test_wave_module_files_have_the_expected_rate(tree):
    with wave.open(os.path.join(tree, 'R2 Padawan', '32SENT1.wav')) as w:
        assert w.getframerate() == RATE


# --- duplicates ----------------------------------------------------------------------------

def test_duplicate_score_exact_near_and_different():
    burst = burst_train(3000, 10, amp=0.5)
    exact = analysis.fingerprint(burst[:, None], RATE)
    assert analysis.duplicate_score(exact, analysis.fingerprint(burst.copy(), RATE)) == 1.0
    sweep = chirp(1800, 400, 2.0, 0.5)
    near = np.concatenate([np.zeros(int(0.03 * RATE)), 0.5 * sweep])     # -6 dB, 30 ms late
    a, b = analysis.fingerprint(sweep, RATE), analysis.fingerprint(near, RATE)
    assert not analysis.exact_match(a, b)
    assert analysis.duplicate_score(a, b) >= analysis.DUPLICATE_THRESHOLD
    other = analysis.fingerprint(burst_train(1200, 4, on=0.15, off=0.1, amp=0.5), RATE)
    assert analysis.duplicate_score(exact, other) < 0.5
    steady = [analysis.fingerprint(tone(f, 1.0, 0.3), RATE) for f in (300, 1000, 2500)]
    assert analysis.duplicate_score(steady[0], steady[1]) < 0.5
    assert analysis.duplicate_score(steady[1], steady[2]) < 0.5
    assert analysis.duplicate_score(steady[0], exact) == 0.0          # steady vs not
    assert analysis.durations_close(1.0, 1.2) and analysis.durations_close(0.5, 0.75)
    assert not analysis.durations_close(1.0, 1.4)


@pytest.fixture
def dup_tree(tmp_path):
    return make_duplicate_tree(tmp_path / 'sounds')


def test_duplicates_are_suggested_and_the_kept_clip_follows_the_rule(dup_tree, tmp_path):
    pairs = []
    catalogue, warnings, _, sheet, _ = run(dup_tree, tmp_path, pairs_out=pairs)
    assert warnings == [] and len(catalogue.clips) == 13    # music-10 is identical: marked
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    # the exact copy: same RMS, so the shorter name is kept
    assert rows['music-10']['suggested_duplicate_of'] == 'music-3'
    assert rows['music-10']['duplicate_score'] == '1.000'
    assert rows['music-3']['suggested_duplicate_of'] == ''
    # the near copy, across the sets: the clip nearer its set median wins over the shorter name
    clips = by_name(catalogue)
    sets = catalogue.sets
    off = {n: abs(clips[n].rms_dbfs - sets[clips[n].set_name].median_rms_dbfs)
           for n in ('music-2', 'padawan-sent09')}
    assert off['padawan-sent09'] < off['music-2']
    assert rows['music-2']['suggested_duplicate_of'] == 'padawan-sent09'
    assert float(rows['music-2']['duplicate_score']) >= analysis.DUPLICATE_THRESHOLD
    assert rows['padawan-sent09']['suggested_duplicate_of'] == ''
    # the same length, another sound: scored, not suggested
    assert rows['music-11']['suggested_duplicate_of'] == ''
    assert rows['music-11']['duplicate_score'] == ''
    score = {frozenset((a, b)): s for s, a, b in pairs}
    assert score[frozenset(('music-3', 'music-11'))] < analysis.DUPLICATE_THRESHOLD
    assert frozenset(('music-1', 'music-2')) not in score            # 1 s vs 2 s: not scored
    assert {n for n, r in rows.items() if r['duplicate_of']} == {'music-10'}


@pytest.mark.parametrize('rel, rank', [
    ('140.mp3', 0), ('001 gen-1.mp3', 1), ('176 mus-1.mp3', 1), ('32SENT1.mp3', 2),
    ('026 chat-1.mp3', 2), ('17MISC3.mp3', 2), ('052 happy-2.mp3', 3),
    ('176 cantina beep.mp3', 3), ('25OOH3.mp3', 3), ('2SecOfSilence.mp3', 3),
    ('clean Leia/5LEIA.mp3', 3)])
def test_name_rank(rel, rank):
    assert bc.name_rank(rel) == rank


def test_identical_clips_are_marked_keeping_the_descriptive_name(tmp_path):
    root = tmp_path / 's'
    same = tone()
    for rel in ('mp3-trigger/001 gen-1.wav', 'mp3-trigger/052 happy-2.wav',
                'R2 sounds and music/140.wav'):
        write_wav(str(root / rel), same)
    write_wav(str(root / 'R2 sounds and music' / '141.wav'), tone(seconds=2.0))
    marks = []
    catalogue, warnings, out, sheet, kit = run(str(root), tmp_path, marks_out=marks)
    assert warnings == []
    assert sorted(marks) == [('music-140', 'trigger-052-happy-2'),
                             ('trigger-001-gen-1', 'trigger-052-happy-2')]
    assert sorted(by_name(catalogue)) == ['music-141', 'trigger-052-happy-2']
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    assert rows['music-140']['duplicate_of'] == 'trigger-052-happy-2'
    assert rows['trigger-001-gen-1']['duplicate_of'] == 'trigger-052-happy-2'
    assert rows['trigger-052-happy-2']['duplicate_of'] == ''
    assert os.path.exists(os.path.join(kit, 'duplicates',
                                       'music-140__of__trigger-052-happy-2.wav'))
    before = snapshot(tmp_path)
    run(str(root), tmp_path, today='2026-11-02')                    # idempotent
    assert snapshot(tmp_path) == before
    # cleared by hand and tagged: marked again (the same samples), the tags kept but ignored
    edit_sheet(sheet, **{'music-140': {'duplicate_of': '', 'tags': 'calm'}})
    marks = []
    catalogue, warnings, *_ = run(str(root), tmp_path, marks_out=marks)
    assert marks == [('music-140', 'trigger-052-happy-2')]
    assert len(warnings) == 1 and 'music-140' in warnings[0] and 'ignored' in warnings[0]
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    assert (rows['music-140']['duplicate_of'], rows['music-140']['tags']) == (
        'trigger-052-happy-2', 'calm')
    # the kept clip of a hand mark is never auto-marked (no chains)
    edit_sheet(sheet, **{'music-140': {'duplicate_of': ''},
                         'music-141': {'duplicate_of': 'music-140'}})
    marks = []
    catalogue, warnings, *_ = run(str(root), tmp_path, marks_out=marks)
    assert marks == [] and len(warnings) == 1 and 'no chains' in warnings[0]
    assert sorted(by_name(catalogue)) == ['music-140', 'trigger-052-happy-2']


def test_the_kept_rule_tie_breaks():
    class Item:
        def __init__(self, name, rms, set_name='S', label='', reviewed=False, marked=False,
                     descriptive=0):
            self.name, self.set, self.label = name, set_name, label
            self.reviewed, self.marked, self.descriptive = reviewed, marked, descriptive
            self.measurement = types.SimpleNamespace(rms_dbfs=rms)

    medians = {'S': -10.0, 'R2 Padawan': -10.0, 'mp3-trigger': -10.0}
    items = [Item('b-long', -10.5), Item('a-long', -9.5), Item('zz', -11.0)]
    assert min(items, key=lambda i: bc.keep_key(i, medians)).name == 'a-long'
    items = [Item('b-long', -10.5), Item('a-long', -9.5), Item('zz', -10.5)]
    assert min(items, key=lambda i: bc.keep_key(i, medians)).name == 'zz'
    items = [Item('a-long', -10.0), Item('zz-far', -14.0, label='chatter')]   # a label wins
    assert min(items, key=lambda i: bc.keep_key(i, medians)).name == 'zz-far'
    items.append(Item('zzz-tagged', -15.0, reviewed=True))            # sheet tags win over it
    assert min(items, key=lambda i: bc.keep_key(i, medians)).name == 'zzz-tagged'
    items[-1].marked = True                               # but a marked duplicate never keeps
    assert min(items, key=lambda i: bc.keep_key(i, medians)).name == 'zz-far'
    items.append(Item('zzzz-named', -16.0, descriptive=3))      # a descriptive name over a label
    assert min(items, key=lambda i: bc.keep_key(i, medians)).name == 'zzzz-named'
    items = [Item('trigger-001-gen-1', -10.0, 'mp3-trigger', label='chatter'),
             Item('padawan-sent01', -12.0, 'R2 Padawan', label='chatter')]
    assert min(items, key=lambda i: bc.keep_key(i, medians)).name == 'padawan-sent01'
    assert bc.set_rank('R2 Padawan') < bc.set_rank('mp3-trigger') < bc.set_rank('Other')


def old_format(sheet):
    """Rewrite a sheet without the three duplicate columns (the format before them)."""
    rows = read_sheet_rows(sheet)
    for row in rows:
        for column in ('duplicate_of', 'suggested_duplicate_of', 'duplicate_score'):
            del row[column]
    write_sheet_rows(sheet, rows)


def test_an_old_format_sheet_reads_and_comes_out_with_the_columns(dup_tree, tmp_path):
    _, _, _, sheet, _ = run(dup_tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': 'calm', 'clip_override_db': '2'}})
    old_format(sheet)
    with open(sheet) as f:
        assert 'duplicate_of' not in f.readline()
    catalogue, warnings, *_ = run(dup_tree, tmp_path)
    assert warnings == [] and by_name(catalogue)['music-4'].tags == ('calm',)
    with open(sheet) as f:
        assert f.readline().strip().split(',') == list(bc.SHEET_COLUMNS)
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    assert (rows['music-4']['tags'], rows['music-4']['clip_override_db']) == ('calm', '2')
    assert rows['music-10']['suggested_duplicate_of'] == 'music-3'
    assert rows['music-4']['duplicate_of'] == ''


def test_the_sheet_has_the_timing_column_after_the_score(dup_tree, tmp_path):
    _, _, _, sheet, _ = run(dup_tree, tmp_path)
    columns = list(bc.SHEET_COLUMNS)
    assert columns[columns.index('duplicate_score') + 1] == 'duplicate_timing'
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    assert rows['music-10']['duplicate_timing'].startswith('same recording; lead ')
    assert rows['music-3']['duplicate_timing'] == ''
    assert rows['music-11']['duplicate_timing'] == ''


def test_a_sheet_without_the_timing_column_reads_and_gets_it(dup_tree, tmp_path):
    _, _, _, sheet, _ = run(dup_tree, tmp_path)
    edit_sheet(sheet, **{'music-4': {'tags': 'calm', 'clip_override_db': '2'}})
    rows = read_sheet_rows(sheet)
    for row in rows:
        del row['duplicate_timing']
    write_sheet_rows(sheet, rows)
    with open(sheet) as f:
        assert 'duplicate_timing' not in f.readline()
    catalogue, warnings, *_ = run(dup_tree, tmp_path)
    assert warnings == [] and by_name(catalogue)['music-4'].tags == ('calm',)
    with open(sheet) as f:
        assert f.readline().strip().split(',') == list(bc.SHEET_COLUMNS)
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    assert rows['music-4']['clip_override_db'] == '2'
    assert rows['music-10']['duplicate_timing'].startswith('same recording')


def test_pairs_report_prints_the_timing_columns(dup_tree, tmp_path, capsys):
    out = str(tmp_path / 'c' / 'clips.yaml')
    assert bc.main(['--sounds-dir', dup_tree, '--out', out, '--pairs', '3',
                    '--sheet', str(tmp_path / 't.csv')]) == 0      # never the real sheet
    printed = capsys.readouterr().out
    assert 'verdict' in printed and 'same recording' in printed


def _print_of(signal):
    return analysis.fingerprint(np.asarray(signal, dtype=np.float32), RATE)


def _bursts(gaps, freq=1500.0, on_s=0.1, lead_s=0.1, tail_s=0.1):
    """Return Hann-shaped tone bursts of ``on_s`` separated by the ``gaps`` (s)."""
    parts = [np.zeros(int(lead_s * RATE))]
    for i in range(len(gaps) + 1):
        parts.append(tone(freq, on_s, 0.4) * np.hanning(int(on_s * RATE)))
        if i < len(gaps):
            parts.append(np.zeros(int(gaps[i] * RATE)))
    parts.append(np.zeros(int(tail_s * RATE)))
    return np.concatenate(parts)


def test_timing_same_recording_with_extra_trailing_silence():
    base = _bursts((0.2, 0.2))
    longer = np.concatenate([base, np.zeros(int(0.3 * RATE))])
    t = analysis.timing_comparison(_print_of(base), _print_of(longer))
    assert t.verdict == 'same recording'
    assert t.trail_b_s == pytest.approx(t.trail_a_s + 0.3, abs=0.02)
    assert t.active_a_s == pytest.approx(t.active_b_s, abs=0.02)
    assert t.summary().startswith('same recording; lead ') and len(t.summary()) < 90


def test_timing_other_take_has_shortened_gaps_and_the_same_pitch():
    # three 100 ms bursts, each gap 40 ms shorter: the bursts do not move by one stretch
    a, b = _print_of(_bursts((0.08, 0.12))), _print_of(_bursts((0.04, 0.08)))
    t = analysis.timing_comparison(a, b)
    assert t.verdict == 'other take'
    assert t.ratio < 0.97 and t.pitch_ratio == pytest.approx(1.0, abs=0.04)
    assert t.pitch_ratio == pytest.approx(1.0, abs=0.04)
    assert t.summary().startswith('other take; active ') and len(t.summary()) < 90


def test_timing_uniform_stretch_by_resampling_moves_the_pitch():
    base = _bursts((0.2, 0.2))
    n = int(len(base) * 1.25)
    slow = np.interp(np.linspace(0, len(base) - 1, n), np.arange(len(base)), base)
    t = analysis.timing_comparison(_print_of(base), _print_of(slow))
    assert t.verdict == 'resampled'
    assert t.ratio == pytest.approx(1.25, abs=0.03) and t.pitch_ratio < 0.9


def line_of(sheet, name):
    return 2 + [r['name'] for r in read_sheet_rows(sheet)].index(name)


@pytest.mark.parametrize('edits, message', [
    ({'music-10': 'music-99'}, r"tags.csv:{music-10}: duplicate_of: 'music-99' is not a clip"),
    ({'music-10': 'music-10'}, r"tags.csv:{music-10}: duplicate_of: 'music-10' is the clip "
                               'itself'),
    ({'music-10': 'music-3', 'music-3': 'music-4'},
     r"tags.csv:{music-10}: duplicate_of: 'music-3' is itself marked duplicate_of "
     r"'music-4' on line {music-3}"),
])
def test_bad_duplicate_of_is_refused_with_its_line(dup_tree, tmp_path, edits, message):
    _, _, _, sheet, _ = run(dup_tree, tmp_path)
    before = snapshot(tmp_path)
    edit_sheet(sheet, **{n: {'duplicate_of': t} for n, t in edits.items()})
    edited = snapshot(tmp_path)
    lines = {n: line_of(sheet, n) for n in ('music-10', 'music-3')}
    with pytest.raises(CatalogueError, match=message.format(**lines)):
        run(dup_tree, tmp_path)
    assert snapshot(tmp_path) == edited != before                 # nothing written


def test_a_marked_duplicate_leaves_the_catalogue_and_moves_in_the_kit(dup_tree, tmp_path):
    first, _, out, sheet, kit = run(dup_tree, tmp_path)
    dup_dir = os.path.join(kit, 'duplicates')
    assert os.listdir(dup_dir) == ['music-10__of__music-3.wav']   # identical: marked by the run
    edit_sheet(sheet, **{'music-10': {'duplicate_of': 'music-3', 'tags': 'alarm'},
                         'music-2': {'duplicate_of': 'padawan-sent09'}})
    catalogue, warnings, *_ = run(dup_tree, tmp_path)
    assert len(warnings) == 1 and 'music-10' in warnings[0] and 'tags' in warnings[0]
    names = set(by_name(catalogue))
    assert 'music-10' not in names and 'music-2' not in names and len(names) == 12
    assert catalogue.sets['R2 sounds and music'].clips == 5       # 7 clips, 2 duplicates
    assert catalogue.sets['R2 Padawan'].clips == 7
    assert catalogue.sets['R2 sounds and music'].median_rms_dbfs == \
        first.sets['R2 sounds and music'].median_rms_dbfs        # no other gain moves
    with open(out) as f:
        data = yaml.safe_load(f)
    assert data['duplicates'] == [
        {'name': 'music-10', 'path': 'R2 sounds and music/10.wav', 'duplicate_of': 'music-3'},
        {'name': 'music-2', 'path': 'R2 sounds and music/2.wav',
         'duplicate_of': 'padawan-sent09'}]
    loaded, load_warnings = load_catalogue(out, dup_tree)
    assert load_warnings == [] and set(by_name(loaded)) == names
    assert all('music-10' != c.name for p in pools(loaded).values() for c in p)
    dup_dir = os.path.join(kit, 'duplicates')
    assert sorted(os.listdir(dup_dir)) == ['music-10__of__music-3.wav',
                                           'music-2__of__padawan-sent09.wav']
    for bucket in os.listdir(kit):
        if bucket != 'duplicates' and os.path.isdir(os.path.join(kit, bucket)):
            assert 'music-10.wav' not in os.listdir(os.path.join(kit, bucket))
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    assert (rows['music-10']['duplicate_of'], rows['music-10']['tags']) == ('music-3', 'alarm')
    assert rows['music-10']['suggested_duplicate_of'] == 'music-3'  # still refreshed
    before = snapshot(tmp_path)
    run(dup_tree, tmp_path, today='2026-11-02')                    # idempotent
    assert snapshot(tmp_path) == before
    edit_sheet(sheet, **{'music-10': {'duplicate_of': ''}, 'music-2': {'duplicate_of': ''}})
    # music-10 and music-3 are identical, so one is marked again; music-10 has tags now
    # (reviewed), so it is the kept one this time
    catalogue, *_ = run(dup_tree, tmp_path)
    assert {'music-2', 'music-10'} <= set(by_name(catalogue)) and len(catalogue.clips) == 13
    assert os.listdir(dup_dir) == ['music-3__of__music-10.wav']
    with open(out) as f:
        assert yaml.safe_load(f)['duplicates'] == [
            {'name': 'music-3', 'path': 'R2 sounds and music/3.wav', 'duplicate_of': 'music-10'}]


def test_a_duplicate_of_a_vanished_clip_stays_in_the_catalogue(dup_tree, tmp_path):
    _, _, _, sheet, _ = run(dup_tree, tmp_path)
    edit_sheet(sheet, **{'music-10': {'duplicate_of': 'music-3'}})
    os.remove(os.path.join(dup_tree, 'R2 sounds and music', '3.wav'))
    catalogue, warnings, *_ = run(dup_tree, tmp_path)
    assert 'music-10' in by_name(catalogue)
    assert any('music-10' in w and 'no clip on disk' in w for w in warnings)
    rows = {r['name']: r for r in read_sheet_rows(sheet)}
    assert rows['music-10']['duplicate_of'] == 'music-3'            # David's column is kept
    assert rows['music-3']['status'] == 'missing'


def test_the_loader_tolerates_the_duplicates_key_and_its_absence(dup_tree, tmp_path):
    _, _, out, *_ = run(dup_tree, tmp_path)
    with open(out) as f:
        data = yaml.safe_load(f)
    assert 'duplicates' in data
    catalogue, _ = load_catalogue(out)
    del data['duplicates']
    with open(out, 'w') as f:
        yaml.safe_dump(data, f)
    assert load_catalogue(out)[0] == catalogue
