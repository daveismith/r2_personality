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

"""The device queue: a newer run replaces an older queued one for the same target."""

import queue

import pytest
from r2_expression.arbiter_node import _ActionQueue
from r2_expression.core import Action, CALL_CONFIGURE, CALL_RUN, CALL_STOP


def act(call, target='rld', **request):
    request.setdefault('priority', 10)
    return Action(call, 'rld', target, request, 'r2_expression', request['priority'])


def test_newest_run_per_target_wins_and_stops_configures_and_relists_are_kept():
    q = _ActionQueue()
    runs = [act(CALL_RUN, effect=name) for name in 'abc']
    other = act(CALL_RUN, target='fld')
    stop, cfg = act(CALL_STOP), act(CALL_CONFIGURE)
    assert q.put_replacing((1, runs[0])) == []
    assert q.put_replacing((2, runs[1])) == [runs[0]]
    assert q.put_replacing((3, stop)) == []
    assert q.put_replacing((4, other)) == []
    assert q.put_replacing((5, cfg)) == []
    assert q.put_replacing((6, None)) == []              # a re-list request
    assert q.put_replacing((7, runs[2])) == [runs[1]]
    assert q.put_replacing((8, None)) == []
    got = [q.get(0.1)[1] for _ in range(6)]
    assert got == [stop, other, cfg, None, runs[2], None]
    with pytest.raises(queue.Empty):
        q.get(0.05)
