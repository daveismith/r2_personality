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

from r2_expression.config import DomeDefaults, parse_config  # noqa: E402
from r2_expression.palette import LedCatalogue, MagicCatalogue  # noqa: E402
from r2_mocks.dome_plant import DomePlant  # noqa: E402

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
MAGIC_CAT = MagicCatalogue(
    sequences={0: 'All off', 23: 'Eye scan', 27: 'Flash halves', 44: 'Smile',
               56: 'Random show'},
    unbounded=frozenset({56}))


PROFILES = {'snappy': {'v_max': 2.0, 'accel': 4.0, 'k': 3.0, 'settle_band_rad': 0.05},
            'smooth': {'v_max': 1.0, 'accel': 1.5, 'k': 1.5, 'settle_band_rad': 0.05},
            'lazy': {'v_max': 0.6, 'accel': 0.8, 'k': 1.0, 'settle_band_rad': 0.09}}
MIN_SPEED = DomeDefaults().min_speed_rad_s    # every dome command is 0 or at least this
CALM_GLANCE = {'every_s': [10, 20], 'amplitude_rad': [0.3, 0.5], 'dwell_s': [0.5, 1.0]}


def dome_data():
    """Return BASE with the dome: profiles, calm glancing, excited snappy, sleepy resting."""
    data = base_data(profiles=PROFILES)
    data['neutral']['dome'] = {'profile': 'smooth'}
    data['moods']['calm']['dome'] = {'profile': 'smooth', 'glance': CALM_GLANCE}
    data['moods']['excited']['dome'] = {'profile': 'snappy'}
    data['moods']['sleepy'] = {'dome': {'profile': 'lazy', 'rest_rad': 0.35}}
    return data


def base_data(**changes):
    """Return a deep copy of BASE; top-level keys in ``changes`` replace BASE's."""
    data = copy.deepcopy(BASE)
    data.update(copy.deepcopy(changes))
    return data


def cfg(**changes):
    return parse_config(base_data(**changes))


def dome_cfg():
    return parse_config(dome_data())


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


def shutdown_rclpy():
    """Shut rclpy down; a zenoh session that is slow to close (RuntimeError) is not a failure."""
    import rclpy
    try:
        rclpy.try_shutdown()
    except RuntimeError:
        pass


# --- the dome plant ----------------------------------------------------------------------

class PlantDouble(DomePlant):
    """
    ``r2_mocks``' plant of plan 2.1, with the tests' names and a hand to push it.

    ``position`` is the true angle (unwrapped; settable: a push), :meth:`reading` the
    joint state's. ``quirks`` is the firmware's 9-degree crossing of 0 (off by default).
    """

    def __init__(self, position=0.0, ideal=False, quirks=False):
        super().__init__(ideal=ideal, firmware_quirks=quirks, initial_rad=position)

    @property
    def position(self):
        return self.position_rad

    @position.setter
    def position(self, value):
        self._position, self._velocity, self._coast_decel = float(value), 0.0, 0.0

    def reading(self):
        return self.reported_rad


def simulate(move, plant, t0=0.0, horizon_s=20.0, physics_hz=60, step_hz=20, state_hz=10):
    """
    Run ``move`` against ``plant`` as the node would; return ``(reason, log)``.

    The plant runs at ``physics_hz``, the move is stepped at ``step_hz`` with the latest
    joint state, published at ``state_hz`` (with its time). ``log`` is
    ``[(t, command, true position)]``, one per step; the command holds between steps.
    """
    physics_dt = 1.0 / physics_hz
    per_step, per_state = physics_hz // step_hz, physics_hz // state_hz
    log, command = [], 0.0
    sample, sample_at = plant.reading(), t0
    for n in range(int(horizon_s * physics_hz) + 1):
        t = t0 + n * physics_dt
        if n % per_state == 0:
            sample, sample_at = plant.reading(), t
        if n % per_step == 0:
            command, done, reason = move.step(sample, t, sample_at)
            log.append((t, command, plant.position))
            if done:
                for _ in range(int(3.0 * physics_hz)):     # let the coast finish
                    plant.step(0.0, physics_dt)
                return reason, log
        plant.step(command, physics_dt)
    return 'still running', log
