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
import wave

import pytest
import soundfile

from fixtures import make_tree, RATE, tone, write_wav  # noqa: I100
from r2_sound import gain
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
    ('7PATROL1.mp3', 'misc'), ('Scream.mp3', 'scream'), ('140.mp3', ''), ('Hello.mp3', '')])
def test_label_mapping(rel, label):
    assert bc.label_of(rel) == label


@pytest.mark.parametrize('rel', [
    '9Theme.mp3', '12Chorus.mp3', '11Emperor.mp3', '10Cantina.mp3', 'Cantina.mp3', 'March.mp3',
    'SWDiscoShort.mp3', 'StarWars.mp3', '5LEIA.mp3', 'clean Leia/x.mp3',
    'leia-hologram-cleaned-amped-up.mp3', '2SecOfSilence.mp3'])
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
                                        'confidence,tags,clip_override_db,status')
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
