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
