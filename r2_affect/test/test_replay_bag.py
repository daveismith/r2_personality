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

"""The replay tool's bag layer and exit codes, on small bags written here (no ROS graph)."""

import os
import sys

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEST_DIR = os.path.dirname(os.path.abspath(__file__))
for path in (PKG_ROOT, TEST_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

rosbag2_py = pytest.importorskip('rosbag2_py')
msgs = pytest.importorskip('astromech_msgs.msg')

from r2_affect.config import load_config  # noqa: E402
from r2_affect.replay import main, read_bag  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from test_replay import fake_node, SCENARIO, SHIPPED  # noqa: E402

NS = 1000000000


def to_time(seconds, msg_time):
    msg_time.sec = int(seconds // 1)
    msg_time.nanosec = int(round((seconds - msg_time.sec) * 1e9)) % NS


def state_msg(rec):
    m = msgs.MoodState()
    to_time(rec.stamp, m.header.stamp)
    for name in ('arousal', 'valence', 'curiosity', 'confidence', 'mood', 'mood_strength',
                 'locked', 'lock_source', 'idle_s', 'focus_subject'):
        setattr(m, name, getattr(rec, name))
    m.focus_bearing_rad = rec.focus_bearing
    if rec.lock_until is not None:
        to_time(rec.lock_until, m.lock_until)
    return m


def stimulus_msg(s, header_stamp=12345.0):
    m = msgs.Stimulus()
    to_time(header_stamp, m.header.stamp)      # never used: the replay uses receive times
    m.kind, m.source, m.subject, m.extra = s.kind, s.source, s.subject, ''
    m.intensity, m.confidence, m.bearing_rad, m.ttl_s = (
        s.intensity, s.confidence, s.bearing_rad, s.ttl_s)
    return m


def write_bag(path, entries, storage='mcap', topics=None):
    """Write ``(topic, type, msg, receive_time_s)`` entries to a new bag; return its path."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    declared = dict(topics or {})
    for topic, type_name, _, _ in entries:
        declared.setdefault(topic, type_name)
    for i, (topic, type_name) in enumerate(declared.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(i, topic, type_name, 'cdr'))
    for topic, _, msg, t in sorted(entries, key=lambda e: e[3]):
        writer.write(topic, serialize_message(msg), int(round(t * NS)))
    del writer                                  # closes the bag
    return str(path)


def session(stimuli=SCENARIO, wall0=1.7e9):
    """Return a matching session: the fake node's states and stimuli at wall receive times."""
    recorded = fake_node(load_config(SHIPPED), stimuli, -10.0, 16.0, 0.03, rtf=0.956)
    entries = [('/mood/state', 'astromech_msgs/msg/MoodState', state_msg(r), wall0 + r.t)
               for r in recorded]
    entries += [('/stimuli', 'astromech_msgs/msg/Stimulus', stimulus_msg(s), wall0 + s.t)
                for s in stimuli]
    log = Log(name='r2_affect', msg='noise')
    entries.append(('/rosout', 'rcl_interfaces/msg/Log', log, wall0 + 1.0))
    return recorded, entries


@pytest.mark.parametrize('storage', ['mcap', 'sqlite3'])
def test_read_bag_uses_receive_times_and_skips_other_topics(tmp_path, storage):
    recorded, entries = session()
    bag = write_bag(tmp_path / 'bag', entries, storage)
    stimuli, states = read_bag(bag)
    assert [s.kind for s in stimuli] == ['bump', 'face_known', 'touched']
    assert stimuli[0].t == pytest.approx(1.7e9 + SCENARIO[0].t, abs=1e-6)   # not 12345
    assert stimuli[1].subject == 'dave' and stimuli[1].bearing_rad == pytest.approx(0.8)
    assert len(states) == len(recorded)
    assert states[0].t == pytest.approx(1.7e9 + recorded[0].t, abs=1e-6)
    assert states[0].stamp == pytest.approx(recorded[0].stamp, abs=1e-6)
    assert states[0].mood == recorded[0].mood


@pytest.mark.parametrize('storage', ['mcap', 'sqlite3'])
def test_main_matching_run_exits_0(tmp_path, storage, capsys):
    _, entries = session()
    bag = write_bag(tmp_path / 'bag', entries, storage)
    csv_path = tmp_path / 'out.csv'
    assert main([bag, '--compare', '--config', SHIPPED, '--csv', str(csv_path)]) == 0
    out = capsys.readouterr().out
    assert 'OK: max diff' in out
    assert csv_path.read_text().startswith('t_rel,arousal')


def test_main_changed_config_exits_1(tmp_path, capsys):
    _, entries = session()
    bag = write_bag(tmp_path / 'bag', entries)
    changed = tmp_path / 'affect.yaml'
    with open(SHIPPED) as f:
        changed.write_text(f.read().replace(
            'bump:            {arousal: +0.6, valence: -0.4',
            'bump:            {arousal: +0.3, valence: -0.2'))
    assert main([bag, '--compare', '--config', str(changed)]) == 1
    assert 'MISMATCH' in capsys.readouterr().out


def test_main_empty_bag_exits_2(tmp_path, capsys):
    bag = write_bag(tmp_path / 'bag', [], topics={
        '/stimuli': 'astromech_msgs/msg/Stimulus', '/mood/state': 'astromech_msgs/msg/MoodState'})
    assert main([bag, '--config', SHIPPED]) == 2
    assert 'nothing to replay' in capsys.readouterr().err


def test_main_missing_stimuli_topic_exits_2(tmp_path, capsys):
    _, entries = session()
    bag = write_bag(tmp_path / 'bag', [e for e in entries if e[0] != '/stimuli'])
    assert main([bag, '--compare', '--config', SHIPPED]) == 2
    assert 'no /stimuli topic' in capsys.readouterr().err


def test_main_wrong_type_on_stimuli_exits_2(tmp_path, capsys):
    _, entries = session()
    entries = [e for e in entries if e[0] != '/stimuli']
    entries.append(('/stimuli', 'std_msgs/msg/String', String(data='bump'), 1.7e9 + 2.0))
    bag = write_bag(tmp_path / 'bag', entries)
    assert main([bag, '--config', SHIPPED]) == 2
    assert 'expected astromech_msgs/msg/Stimulus' in capsys.readouterr().err


def test_main_directory_that_is_not_a_bag_exits_2(tmp_path, capsys):
    (tmp_path / 'notabag').mkdir()
    (tmp_path / 'notabag' / 'README').write_text('hello')
    assert main([str(tmp_path / 'notabag'), '--config', SHIPPED]) == 2
    assert main([str(tmp_path / 'missing'), '--config', SHIPPED]) == 2
    err = capsys.readouterr().err
    assert 'cannot open' in err and 'no bag at' in err


def test_main_compare_without_states_exits_2(tmp_path, capsys):
    _, entries = session()
    bag = write_bag(tmp_path / 'bag', [e for e in entries if e[0] != '/mood/state'])
    assert main([bag, '--config', SHIPPED]) == 0                    # grid replay is fine
    assert main([bag, '--compare', '--config', SHIPPED]) == 2
    assert '--compare needs /mood/state' in capsys.readouterr().err
