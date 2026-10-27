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

"""
Render the synth's review grid: 5 arousal x 5 valence phrases at one seed (plan step 7).

    synth_grid [--out /ws/.audio/synth] [--seed 1] [--rate 22050] [--medley N]

Writes ``a<A>_v<V>.wav`` (16-bit mono) for arousal and valence each in 0.1, 0.3, 0.5, 0.7,
0.9, an ``index.md`` with the grid (contour, syllables, length, level, render time) and
``play_grid.sh``, which plays them in order with ``afplay`` (the Mac, route 1), printing the
coordinates first. ``--medley N`` also writes ``medley/<mood>-<seed>.wav`` for N seeds of
each of :data:`MOODS` (the voices the moods use, several phrases each, as chatter is heard)
with its own ``play_medley.sh``. Prints one line a phrase.
"""

import argparse
import os
import stat
import sys
import time

from r2_sound import synth
from r2_sound.backends import write_wav

GRID = (0.1, 0.3, 0.5, 0.7, 0.9)
MOODS = (('sad', 0.15, 0.15), ('calm', 0.2, 0.75), ('neutral', 0.5, 0.5),
         ('annoyed', 0.8, 0.2), ('excited', 0.9, 0.8))
DEFAULT_OUT = '/ws/.audio/synth'
PLAYER = """#!/bin/sh
# Play the synth grid in order: arousal rows, valence columns (index.md has the table).
# Usage: ./play_grid.sh            (afplay on the Mac; PLAYER=aplay ./play_grid.sh elsewhere)
cd "$(dirname "$0")" || exit 1
PLAYER=${PLAYER:-afplay}
"""


def file_name(arousal, valence):
    """Return the grid file name of one cell."""
    return 'a%.1f_v%.1f.wav' % (arousal, valence)


def render_grid(out, seed=1, rate=22050):
    """Render the grid into ``out``; return ``[(arousal, valence, info, render_ms)]``."""
    os.makedirs(out, exist_ok=True)
    synth.render(0.5, 0.5, 0.0, seed, rate)           # the first call pays numpy's warm-up
    cells = []
    for arousal in GRID:
        for valence in GRID:
            began = time.perf_counter()
            samples, info = synth.render(arousal, valence, 0.0, seed, rate)
            render_ms = (time.perf_counter() - began) * 1000.0
            write_wav(os.path.join(out, file_name(arousal, valence)), samples, rate)
            cells.append((arousal, valence, info, render_ms))
    return cells


def index_text(cells, seed, rate):
    """Return ``index.md`` for the rendered ``cells``."""
    lines = ['# Synth v2 review grid', '',
             'Seed %d, %d Hz, 16-bit mono; each phrase at the house level (-16 dBFS RMS, '
             'peak at most -1 dBFS). Rows: arousal; columns: valence. Play them in order with '
             '`./play_grid.sh`. Notes for the iteration go to the step 7 PR.' % (seed, rate),
             '', '| arousal \\ valence | ' + ' | '.join('%.1f' % v for v in GRID) + ' |',
             '|---|' + '---|' * len(GRID)]
    by_cell = {(a, v): info for a, v, info, _ in cells}
    for a in GRID:
        row = ['`%s`<br>%.2f s, %d syl.' % (file_name(a, v), by_cell[a, v].length_s,
                                            len(by_cell[a, v].phrase.syllables)) for v in GRID]
        lines.append('| **%.1f** | %s |' % (a, ' | '.join(row)))
    lines += ['', '## Each phrase', '', 'Syllables are shown in their groups, `|` between '
              'groups.', '', '| file | contour | syllables | length s | RMS dBFS | peak dBFS '
              '| render ms |', '|---|---|---|---|---|---|---|']
    for a, v, info, ms in cells:
        lines.append('| `%s` | %s | %s | %.2f | %.1f | %.1f | %.1f |' % (
            file_name(a, v), info.phrase.contour, grouped(info.phrase), info.length_s,
            info.rms_dbfs, info.peak_dbfs, ms))
    return '\n'.join(lines) + '\n'


def grouped(phrase):
    """Return the phrase's syllable kinds with ``|`` between its groups."""
    out, i = [], 0
    for size in phrase.groups:
        out.append(' '.join(phrase.kinds[i:i + size]))
        i += size
    return ' | '.join(out)


def render_medley(out, seeds, rate=22050):
    """Render ``seeds`` phrases of each mood of :data:`MOODS` into ``out/medley``."""
    folder = os.path.join(out, 'medley')
    os.makedirs(folder, exist_ok=True)
    lines = [PLAYER]
    for name, arousal, valence in MOODS:
        for seed in range(1, seeds + 1):
            samples, info = synth.render(arousal, valence, 0.0, seed, rate)
            file = '%s-%d.wav' % (name, seed)
            write_wav(os.path.join(folder, file), samples, rate)
            lines.append('echo "%s (a %.2f v %.2f) seed %d: %s, %s"' % (
                name, arousal, valence, seed, info.phrase.contour, grouped(info.phrase)))
            lines.append('"$PLAYER" %s' % file)
            print('%s-%d  %.2f s  %s  %s' % (
                name, seed, info.length_s, info.phrase.contour, grouped(info.phrase)))
    script = os.path.join(folder, 'play_medley.sh')
    with open(script, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    os.chmod(script, os.stat(script).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def player_text(cells):
    """Return ``play_grid.sh`` for the rendered ``cells``."""
    lines = [PLAYER]
    for a, v, info, _ in cells:
        lines.append('echo "arousal %.1f valence %.1f: %s, %s"' % (
            a, v, info.phrase.contour, grouped(info.phrase)))
        lines.append('"$PLAYER" %s' % file_name(a, v))
    return '\n'.join(lines) + '\n'


def main(args=None):
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1].strip())
    parser.add_argument('--out', default=DEFAULT_OUT)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--rate', type=int, default=22050)
    parser.add_argument('--medley', type=int, default=0, metavar='N',
                        help='also render N seeds of each mood voice into medley/')
    options = parser.parse_args(args)
    cells = render_grid(options.out, options.seed, options.rate)
    if options.medley > 0:
        render_medley(options.out, options.medley, options.rate)
    with open(os.path.join(options.out, 'index.md'), 'w') as f:
        f.write(index_text(cells, options.seed, options.rate))
    script = os.path.join(options.out, 'play_grid.sh')
    with open(script, 'w') as f:
        f.write(player_text(cells))
    os.chmod(script, os.stat(script).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    for a, v, info, ms in cells:
        print('a%.1f v%.1f  %.2f s  %d syl  rms %.1f  peak %.1f dBFS  %.1f ms' % (
            a, v, info.length_s, len(info.phrase.syllables), info.rms_dbfs, info.peak_dbfs, ms))
    print('wrote %d phrases, index.md and play_grid.sh to %s' % (len(cells), options.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
