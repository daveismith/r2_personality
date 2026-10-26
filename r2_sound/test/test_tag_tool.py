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

"""Tests for the tag tool's sheet round-trip, kit lookup and server (standard library)."""

import json
import threading
import urllib.error
import urllib.request

import pytest
from r2_sound.tools import tag_tool

HEADER = ('name,set,path,duration_s,rms_dbfs,suggested_tag,confidence,tags,'
          'clip_override_db,status\n')
ROWS = [
    'a-1,Set A,Set A/1.mp3,1.00,-10.0,chatter,0.423,,,\n',
    'a-2,"Set, B",Set A/2.mp3,2.50,-9.0,alarm,0.900,hum,3,\n',
    'a-3,Set A,Set A/3.mp3,0.50,-8.0,laugh,1.000,,,missing\n',
]


@pytest.fixture
def sheet(tmp_path):
    path = tmp_path / 'tags.csv'
    path.write_text(HEADER + ''.join(ROWS), encoding='utf-8')
    return path


def test_save_changes_only_two_columns(sheet):
    n = tag_tool.save_changes(sheet, {'a-1': {'tags': ['alarm', 'calm'],
                                              'clip_override_db': '-2.5'}})
    assert n == 1
    lines = sheet.read_text().splitlines(keepends=True)
    assert lines[0] == HEADER
    assert lines[1] == 'a-1,Set A,Set A/1.mp3,1.00,-10.0,chatter,0.423,"alarm,calm",-2.5,\n'
    assert lines[2:] == ROWS[1:]
    assert (sheet.parent / 'tags.csv.bak').read_text() == HEADER + ''.join(ROWS)


def test_save_clear_and_number_override(sheet):
    tag_tool.save_changes(sheet, {'a-2': {'tags': [], 'clip_override_db': ''}})
    assert sheet.read_text().splitlines()[2] == 'a-2,"Set, B",Set A/2.mp3,2.50,-9.0,alarm,0.900,,,'
    tag_tool.save_changes(sheet, {'a-2': {'tags': ['hum'], 'clip_override_db': 4}})
    assert sheet.read_text().splitlines()[2].endswith('0.900,hum,4,')


def test_save_rejects_bad_input_and_writes_nothing(sheet):
    before = sheet.read_text()
    for bad in ({'a-1': {'tags': ['nope']}},
                {'a-1': {'tags': [], 'clip_override_db': 'loud'}},
                {'a-1': {'tags': [], 'clip_override_db': 'nan'}},
                {'zzz': {'tags': []}}):
        with pytest.raises(tag_tool.SheetError):
            tag_tool.save_changes(sheet, bad)
    assert sheet.read_text() == before
    assert not (sheet.parent / 'tags.csv.bak').exists()


def test_kit_lookup(tmp_path):
    kit = tmp_path / 'kit'
    (kit / 'chatter').mkdir(parents=True)
    (kit / 'alarm').mkdir()
    (kit / 'chatter' / 'a-1.wav').write_bytes(b'RIFF')
    (kit / 'chatter' / 'a-2.wav').write_bytes(b'RIFF')
    (kit / 'alarm' / 'a-2.wav').write_bytes(b'RIFF')
    index = tag_tool.index_kit(kit)
    assert tag_tool.kit_path(index, 'a-2', 'alarm') == kit / 'alarm' / 'a-2.wav'
    assert tag_tool.kit_path(index, 'a-2', 'sad') is not None
    assert tag_tool.kit_path(index, 'a-3', 'laugh') is None
    assert tag_tool.index_kit(tmp_path / 'absent') == {}


def test_server_serves_clips_and_audio(sheet, tmp_path):
    kit = tmp_path / 'kit'
    (kit / 'chatter').mkdir(parents=True)
    (kit / 'chatter' / 'a-1.wav').write_bytes(b'RIFFdata')
    server = tag_tool.make_server(sheet, kit, 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{server.server_address[1]}'
    try:
        data = json.load(urllib.request.urlopen(base + '/api/clips'))
        assert [c['name'] for c in data['clips']] == ['a-1', 'a-2', 'a-3']
        assert [c['has_audio'] for c in data['clips']] == [True, False, False]
        resp = urllib.request.urlopen(base + '/audio/a-1.wav')
        assert resp.headers['Content-Type'] == 'audio/wav'
        assert resp.read() == b'RIFFdata'
        req = urllib.request.Request(base + '/api/save', method='POST',
                                     data=json.dumps({'a-1': {'tags': ['x']}}).encode())
        with pytest.raises(urllib.error.HTTPError) as err:
            urllib.request.urlopen(req)
        assert err.value.code == 400
    finally:
        server.shutdown()
        server.server_close()


# --- duplicates ----------------------------------------------------------------------------

NEW_HEADER = ('name,set,path,duration_s,rms_dbfs,suggested_tag,confidence,tags,'
              'clip_override_db,duplicate_of,suggested_duplicate_of,duplicate_score,status\n')
NEW_ROWS = [
    'a-1,Set A,Set A/1.mp3,1.00,-10.0,chatter,0.423,,,,a-2,0.970,\n',
    'a-2,Set A,Set A/2.mp3,1.01,-9.0,alarm,0.900,hum,3,,,,\n',
    'a-3,Set A,Set A/3.mp3,0.50,-8.0,laugh,1.000,,,,,,\n',
]


@pytest.fixture
def new_sheet(tmp_path):
    (tmp_path / 'new').mkdir()
    path = tmp_path / 'new' / 'tags.csv'
    path.write_text(NEW_HEADER + ''.join(NEW_ROWS), encoding='utf-8')
    return path


def test_save_duplicate_of_changes_only_davids_columns(new_sheet):
    n = tag_tool.save_changes(new_sheet, {'a-1': {'tags': [], 'clip_override_db': '',
                                                  'duplicate_of': 'a-2'}})
    assert n == 1
    lines = new_sheet.read_text().splitlines(keepends=True)
    assert lines[0] == NEW_HEADER
    assert lines[1] == 'a-1,Set A,Set A/1.mp3,1.00,-10.0,chatter,0.423,,,a-2,a-2,0.970,\n'
    assert lines[2:] == NEW_ROWS[1:]
    tag_tool.save_changes(new_sheet, {'a-1': {'tags': ['sad'], 'clip_override_db': ''}})
    assert new_sheet.read_text().splitlines()[1].endswith(',sad,,a-2,a-2,0.970,')  # kept
    tag_tool.save_changes(new_sheet, {'a-1': {'tags': [], 'duplicate_of': ''}})
    assert new_sheet.read_text() == NEW_HEADER + ''.join(NEW_ROWS)


@pytest.mark.parametrize('changes, message', [
    ({'a-1': {'duplicate_of': 'a-1'}}, 'itself'),
    ({'a-1': {'duplicate_of': 'zzz'}}, 'not a clip'),
    ({'a-1': {'duplicate_of': 3}}, 'clip name'),
    ({'a-1': {'duplicate_of': 'a-2'}, 'a-2': {'duplicate_of': 'a-3'}}, 'no chains'),
])
def test_save_refuses_a_bad_duplicate_of(new_sheet, changes, message):
    with pytest.raises(tag_tool.SheetError, match=message):
        tag_tool.save_changes(new_sheet, changes)
    assert new_sheet.read_text() == NEW_HEADER + ''.join(NEW_ROWS)


def test_save_refuses_a_chain_with_a_row_already_on_disk(new_sheet):
    tag_tool.save_changes(new_sheet, {'a-1': {'duplicate_of': 'a-2'}})
    with pytest.raises(tag_tool.SheetError, match='no chains'):
        tag_tool.save_changes(new_sheet, {'a-2': {'duplicate_of': 'a-3'}})   # a-1 keeps a-2
    with pytest.raises(tag_tool.SheetError, match='no chains'):
        tag_tool.save_changes(new_sheet, {'a-3': {'duplicate_of': 'a-1'}})   # a-1 is one


def test_an_old_sheet_gets_the_column_when_a_duplicate_is_saved(sheet):
    tag_tool.save_changes(sheet, {'a-1': {'tags': ['alarm']}})
    assert sheet.read_text().splitlines()[0] + '\n' == HEADER          # not needed yet
    tag_tool.save_changes(sheet, {'a-3': {'tags': [], 'duplicate_of': 'a-2'}})
    lines = sheet.read_text().splitlines()
    assert lines[0] == ('name,set,path,duration_s,rms_dbfs,suggested_tag,confidence,tags,'
                        'clip_override_db,duplicate_of,status')
    assert lines[1] == 'a-1,Set A,Set A/1.mp3,1.00,-10.0,chatter,0.423,alarm,,,'
    assert lines[3] == 'a-3,Set A,Set A/3.mp3,0.50,-8.0,laugh,1.000,,,a-2,missing'


def test_payload_has_the_duplicate_fields_even_for_an_old_sheet(sheet, new_sheet, tmp_path):
    old = tag_tool.clips_payload(sheet, {})['clips'][0]
    assert {k: old[k] for k in tag_tool.DUPLICATE_FIELDS} == dict.fromkeys(
        tag_tool.DUPLICATE_FIELDS, '')
    new = tag_tool.clips_payload(new_sheet, {})['clips'][0]
    assert (new['duplicate_of'], new['suggested_duplicate_of'], new['duplicate_score']) == (
        '', 'a-2', '0.970')


def test_a_save_applies_onto_the_sheet_as_it_is_on_disk_now(new_sheet, tmp_path):
    kit = tmp_path / 'kit'
    (kit / 'duplicates').mkdir(parents=True)
    (kit / 'chatter').mkdir()
    (kit / 'duplicates' / 'a-1__of__a-2.wav').write_bytes(b'RIFFdup')
    (kit / 'chatter' / 'a-2.wav').write_bytes(b'RIFFkept')
    server = tag_tool.make_server(new_sheet, kit, 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{server.server_address[1]}'

    def post(body):
        req = urllib.request.Request(base + '/api/save', method='POST',
                                     data=json.dumps(body).encode())
        return json.load(urllib.request.urlopen(req))

    try:
        data = json.load(urllib.request.urlopen(base + '/api/clips'))
        assert data['clips'][0]['suggested_duplicate_of'] == 'a-2'
        assert urllib.request.urlopen(base + '/audio/a-1.wav').read() == b'RIFFdup'
        assert urllib.request.urlopen(base + '/audio/a-2.wav').read() == b'RIFFkept'
        # build_catalogue rewrites the sheet while the page is open: new suggestions, an
        # extra column, a row more
        changed = ('name,set,path,duration_s,rms_dbfs,suggested_tag,confidence,tags,'
                   'clip_override_db,duplicate_of,suggested_duplicate_of,duplicate_score,'
                   'extra,status\n'
                   'a-1,Set A,Set A/1.mp3,1.00,-10.0,sad,0.700,,,,a-2,0.990,x,\n'
                   'a-2,Set A,Set A/2.mp3,1.01,-9.0,alarm,0.900,hum,3,,,,y,\n'
                   'a-3,Set A,Set A/3.mp3,0.50,-8.0,laugh,1.000,,,,,,z,\n'
                   'a-4,Set A,Set A/4.mp3,0.50,-8.0,ooh,1.000,,,,,,w,\n')
        new_sheet.write_text(changed, encoding='utf-8')
        assert post({'a-1': {'tags': [], 'clip_override_db': '', 'duplicate_of': 'a-2'}}) == {
            'saved': 1}
        lines = new_sheet.read_text().splitlines(keepends=True)
        assert lines[1] == 'a-1,Set A,Set A/1.mp3,1.00,-10.0,sad,0.700,,,a-2,a-2,0.990,x,\n'
        assert ''.join(lines[2:]) == ''.join(changed.splitlines(keepends=True)[2:])
        # a row the page knows is gone from the file: refused, nothing written
        new_sheet.write_text(changed.replace(
            'a-3,Set A,Set A/3.mp3,0.50,-8.0,laugh,1.000,,,,,,z,\n', ''), encoding='utf-8')
        before = new_sheet.read_text()
        with pytest.raises(urllib.error.HTTPError) as err:
            post({'a-3': {'tags': ['sad']}})
        assert err.value.code == 400 and 'a-3' in json.load(err.value)['error']
        assert new_sheet.read_text() == before
    finally:
        server.shutdown()
        server.server_close()


def test_kit_index_reads_the_duplicates_folder(tmp_path):
    kit = tmp_path / 'kit'
    (kit / 'duplicates').mkdir(parents=True)
    (kit / 'duplicates' / 'music-64__of__padawan-sent10.wav').write_bytes(b'RIFF')
    index = tag_tool.index_kit(kit)
    assert tag_tool.kit_path(index, 'music-64', 'chatter') == (
        kit / 'duplicates' / 'music-64__of__padawan-sent10.wav')


def test_payload_passes_the_duplicate_timing_through(tmp_path):
    path = tmp_path / 'tags.csv'
    path.write_text(
        'name,set,path,tags,clip_override_db,duplicate_of,suggested_duplicate_of,'
        'duplicate_score,duplicate_timing,status\n'
        'a-1,S,S/1.mp3,,,,a-2,0.970,"other take; active 0.78/0.70 s (ratio 0.90), '
        'pitch x0.99",\n'
        'a-2,S,S/2.mp3,,,,,,,\n', encoding='utf-8')
    clips = tag_tool.clips_payload(path, {})['clips']
    assert clips[0]['duplicate_timing'].startswith('other take; active 0.78/0.70 s')
    assert clips[1]['duplicate_timing'] == ''
    assert 'duplicate_timing' in tag_tool.PAGE and 'other take' in tag_tool.PAGE
    assert 'Collapse all' in tag_tool.PAGE and 'r2tag.collapsed' in tag_tool.PAGE
