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

"""Shared fixtures for the r2_expression tests: configs, catalogues, device snapshots."""

import copy
import os
import re
import sys

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from r2_expression.config import parse_config  # noqa: E402
from r2_expression.palette import LedCatalogue, MagicCatalogue  # noqa: E402

SHIPPED = os.path.join(PKG_ROOT, 'config', 'expression.yaml')
AFFECT_YAML = os.path.join(PKG_ROOT, '..', 'r2_affect', 'config', 'affect.yaml')
DOME_DOCS = os.path.join(PKG_ROOT, '..', 'docs', 'dome')
LEDS_LIST = os.path.join(DOME_DOCS, '2026-10-20-workbench-leds-list.txt')
MAGIC_LIST = os.path.join(DOME_DOCS, '2026-10-20-workbench-magic-list.txt')

OWNER = 'r2_expression'
DISPLAYS = ('rpsi', 'rld', 'bfld', 'tfld', 'fpsi')
SURFACES = ('rpsi', 'rld', 'fld', 'bfld', 'tfld', 'fpsi')

BASE = {
    'defaults': {'manual_hold_s': 5.0, 'mood_stale_s': 3.0, 'max_duration_ms': 30000,
                 'status_hz': 2.0, 'max_lease_s': 60.0, 'device_stale_s': 5.0},
    'neutral': {'fld': {'drift': 'calm', 'brightness': 40},
                'rld': {'drift': 'calm', 'brightness': 40},
                'psi': {'brightness': 40}},
    'moods': {
        'calm': {
            'fld': {'drift': 'calm', 'brightness': [50, 60]},
            'rld': {'drift': 'calm', 'brightness': [50, 60]},
            'psi': {'brightness': [50, 60]},
            'magic': {'brightness': 5},
            'flourish': {'every_s': [10, 20], 'picks': [
                {'channel': 'fld', 'clip': 'pulse', 'rgb': 0x2060ff, 'duration_ms': 2000}]},
        },
        'excited': {
            'fld': {'drift': 'lively', 'brightness': [80, 100]},
            'rld': {'drift': 'lively', 'brightness': [80, 100]},
            'psi': {'brightness': [80, 100]},
            'magic': {'brightness': 12},
            'flourish': {'every_s': [5, 5], 'picks': [
                {'channel': 'fld', 'effect': 'rainbow', 'weight': 1, 'duration_ms': 4000},
                {'channel': 'magic', 'sequence': 27, 'weight': 1, 'duration_ms': 3000}]},
        },
    },
}

LED_CAT = LedCatalogue(
    effects=frozenset({'rainbow', 'fire', 'lightsout', 'wipe', 'cycle', 'roam'}),
    clips={name: frozenset({'fld', 'psi', 'rld'})
           for name in ('pulse', 'flash', 'hscan', 'alarm')})
MAGIC_CAT = MagicCatalogue(sequences={0: 'All off', 23: 'Eye scan', 27: 'Flash halves',
                                      44: 'Smile', 56: 'Random show'})


def base_data(**changes):
    """Return a deep copy of BASE; top-level keys in ``changes`` replace BASE's."""
    data = copy.deepcopy(BASE)
    data.update(copy.deepcopy(changes))
    return data


def cfg(**changes):
    return parse_config(base_data(**changes))


def led_snap(strip=True, manual=False, displays=None, surfaces=None):
    """Return a LedStateArray as a dict: every display random and unheld, every surface idle."""
    disp = {n: {'name': n, 'mode': 0, 'held_by': '', 'hold_remaining_ms': 0,
                'changed_by': '', 'changed_by_owner': '', 'brightness': 40, 'drift': 'calm'}
            for n in DISPLAYS}
    for name, fields in (displays or {}).items():
        disp[name].update(fields)
    surf = {n: {'name': n, 'running': False, 'effect': '', 'source': '', 'owner': '',
                'last_end': 'none'} for n in SURFACES}
    for name, fields in (surfaces or {}).items():
        surf[name].update(fields)
    return {'displays': list(disp.values()), 'surfaces': list(surf.values()),
            'strip_enabled': strip, 'manual': manual}


def running(effect, source='ros', owner=OWNER):
    return {'running': True, 'effect': effect, 'source': source, 'owner': owner}


def held(by, remaining_ms=5000):
    return {'held_by': by, 'hold_remaining_ms': remaining_ms}


def magic_snap(**fields):
    """Return a MagicPanelState as a dict: panel present and idle."""
    snap = {'link': 1, 'state': 0, 'started_by': '', 'owner': '', 'run_counter': 0,
            'held_by': '', 'hold_remaining_ms': 0, 'manual': False, 'sequence_id': 255,
            'remaining_ms': 0}
    snap.update(fields)
    return snap


class FixedRng:
    """An RNG whose random() is always ``value``."""

    def __init__(self, value=0.0):
        self.value = value

    def random(self):
        return self.value


def capture_names():
    """Effects, clips (name -> targets) and sequence ids/names from the workbench captures."""
    with open(LEDS_LIST) as f:
        leds = f.read()
    effects = re.search(r'effects=\[([^\]]*)\]', leds).group(1)
    effects = set(re.findall(r"'([^']+)'", effects))
    clips = {}
    for name, target in re.findall(r"LedClip\(name='([^']+)', target='([^']+)'", leds):
        clips.setdefault(name, set()).add(target)
    with open(MAGIC_LIST) as f:
        magic = f.read()
    sequences = {int(i): n for i, n in
                 re.findall(r"MagicPanelSequence\(id=(\d+), name='([^']*)'", magic)}
    return effects, clips, sequences
