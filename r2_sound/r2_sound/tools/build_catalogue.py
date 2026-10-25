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
Build ``config/clips.yaml``, ``config/tags.csv`` and the tagging kit from the sound sets.

    build_catalogue --sounds-dir /sounds --out config/clips.yaml --sheet config/tags.csv \
        --kit /ws/.audio/tagging [--threshold 0.6] [--duplicate-threshold 0.85]

Walks the set directories under ``--sounds-dir`` (one level of sets, any depth of files),
decodes each file with ``soundfile``, measures it (:mod:`r2_sound.gain`), names it, labels
it from its file name, suggests a bucket for every unlabelled clip from its acoustics
(:mod:`r2_sound.analysis`), merges David's sheet and writes the three outputs. The
suggestions are a first pass to be confirmed by ear.

* Name: ``<set prefix>-<stem>``, lower-case, other characters to ``-``, a number at the
  end padded to two digits, a ``-2``, ``-3`` suffix for a repeat, given in the sorted order
  of the files' paths (so stable from run to run). In the Padawan set the file's leading
  index is dropped (``32SENT1.mp3`` is ``padawan-sent01``); in the music set the stem stays
  (``140.mp3`` is ``music-140``). A renamed set directory changes its clips' names and paths.
* Label: from the file name (:data:`LABELS`); ``SENT`` is ``chatter`` and so on. Music,
  speech and silence (:data:`EXCLUSIONS`) are ``excluded``. A labelled clip is its own
  suggestion at confidence 1.0 and is a reference for the others.
* Duplicates: every pair of clips of close duration (either set) is scored
  (:func:`r2_sound.analysis.duplicate_pairs`); the pairs at or above
  ``--duplicate-threshold`` join groups (connected components), and each group keeps one
  clip: a labelled one over an unlabelled one (the label is the reference that makes its
  suggestion confident), then the one whose RMS is nearest its set's median, then the shorter
  name, then the first alphabetically. Every other member is suggested
  ``suggested_duplicate_of`` the kept one, with ``duplicate_score`` its score against it and
  ``duplicate_timing`` a one-line :func:`r2_sound.analysis.timing_comparison` of the two
  (lead, trail and active lengths; the verdict). Only the sheet's ``duplicate_of`` acts.
* Sheet (``tags.csv``): the tool fills every column but ``tags``, ``clip_override_db`` and
  ``duplicate_of``, which are David's and are read back by column name and kept (a sheet
  without ``duplicate_of`` reads as all empty and is written with it). A row belongs to the clip at
  its ``path`` (relative to ``--sounds-dir``), else to the clip of its ``name`` (a file moved
  inside its set). A row whose clip is not on disk is never deleted: it stays, as read, with
  ``status`` ``missing`` (and a warning), and gets its clip back when the file returns.
  Rows are sorted by name.
* ``clips.yaml``: ``tags`` are the merged ones (:func:`r2_sound.catalogue.effective_tags`);
  ``excluded`` is true when they hold ``excluded``. A clip whose ``duplicate_of`` is set is
  left out of ``clips`` and of its set's count and listed under ``duplicates`` (name, path,
  duplicate_of); the set medians still count it, so marking one moves no other gain. The
  sheet wins over the exclusion list: a listed clip is suggested ``excluded`` at confidence
  1.0, but other tags in the sheet make it playable.
* Kit: ``<kit>/<suggested bucket>/<name>.wav`` (16-bit mono 22.05 kHz, at the clip's
  catalogue gain, cut to 10 s), a marked duplicate as
  ``<kit>/duplicates/<name>__of__<kept>.wav``, ``play_bucket.sh`` and ``README.md``.

Re-running with nothing changed rewrites identical files (``generated`` keeps its date
while the content is the same).
"""

import argparse
import csv
import datetime
import io
import math
import os
import re
import shutil
import stat
import sys

import numpy as np
from r2_sound import analysis, gain
from r2_sound.catalogue import (Catalogue, CatalogueError, DEFAULT_THRESHOLD, effective_tags,
                                EXCLUDED_TAG, parse_catalogue, SetInfo, SoundClip, TAGS)
import soundfile
import yaml

AUDIO_EXTENSIONS = ('.mp3', '.wav', '.flac', '.ogg')
SET_PREFIXES = {'R2 Padawan': ('padawan', True), 'R2 sounds and music': ('music', False)}
LABELS = (('sent', 'chatter'), ('ooh', 'ooh'), ('hum', 'hum'), ('alarm', 'alarm'),
          ('scream', 'scream'), ('chortle', 'laugh'), ('annoyed', 'annoyed'),
          ('wolfwstl', 'whistle'), ('misc', 'misc'), ('powerup', 'misc'),
          ('doodoo', 'misc'), ('shortckt', 'misc'), ('patrol', 'misc'))
EXCLUSIONS = ('theme', 'chorus', 'emperor', 'cantina', 'march', 'disco', 'starwars', 'leia',
              'silence')
SHEET_COLUMNS = ('name', 'set', 'path', 'duration_s', 'rms_dbfs', 'suggested_tag',
                 'confidence', 'tags', 'clip_override_db', 'duplicate_of',
                 'suggested_duplicate_of', 'duplicate_score', 'duplicate_timing',
                 'status')
MISSING = 'missing'              # the status of a sheet row whose clip is not on disk
DUPLICATES_DIR = 'duplicates'    # the kit folder of the marked duplicates
DUPLICATE_SEPARATOR = '__of__'   # <name>__of__<kept>.wav
KIT_SAMPLE_RATE = 22050
KIT_MAX_S = 10.0
_INDEX = re.compile(r'^\d+(?=[^\W\d_])')
_TRAILING_NUMBER = re.compile(r'^([a-z]+)(\d+)$')


def _slug(text):
    return re.sub(r'[^a-z0-9]+', '-', text.lower()).strip('-')


def stem_of(rel_path):
    """Return the file name without extension of a path inside a set."""
    return os.path.splitext(os.path.basename(rel_path))[0]


def base_name(set_name, rel_path):
    """Return the name for a file before repeats are resolved."""
    prefix, drop_index = SET_PREFIXES.get(set_name, (_slug(set_name), False))
    stem = stem_of(rel_path)
    if drop_index:
        stem = _INDEX.sub('', stem)
    slug = _slug(stem)
    match = _TRAILING_NUMBER.match(slug)
    if match:
        slug = f'{match.group(1)}{int(match.group(2)):02d}'
    return f'{prefix}-{slug}'


def label_of(rel_path):
    """Return a file's label from its name: a bucket, ``excluded`` or ``''``."""
    inside = rel_path.lower()
    if any(word in inside for word in EXCLUSIONS):
        return EXCLUDED_TAG
    stem = _INDEX.sub('', stem_of(inside))
    for prefix, bucket in LABELS:
        if stem.startswith(prefix):
            return bucket
    return ''


def read_sheet(path):
    """
    Return the rows of an existing sheet, in file order, each a dict.

    ``name``, ``path``, ``tags`` (a tuple, validated), ``override`` (the
    ``clip_override_db`` text, validated), ``duplicate_of`` (validated: the name of another
    row of the sheet whose own ``duplicate_of`` is empty, so no chains; the column may be
    absent), ``line`` and ``fields`` (the row as read, for a row kept as ``missing``).
    Two rows with the same ``path`` (when not empty), or two rows with the same ``name``
    neither of which is ``missing``, are refused with both line numbers, like a bad tag. A
    ``missing`` row may share its name: the tool writes one when a repeat suffix moves
    (``-2`` becomes the name of a clip whose sibling is gone).
    """
    if not path or not os.path.isfile(path):
        return []
    rows = []
    seen = {'name': {}, 'path': {}}       # value -> the line it was first on
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        names = reader.fieldnames or ()
        missing = [c for c in ('name', 'tags', 'clip_override_db') if c not in names]
        if missing:
            raise CatalogueError(f'{path}: header lacks the column(s) {", ".join(missing)}')
        for number, row in enumerate(reader, start=2):
            name = (row.get('name') or '').strip()
            if not name:
                continue
            tags = tuple(t for t in re.split(r'[,;\s]+', (row.get('tags') or '').lower()) if t)
            for tag in tags:
                if tag not in TAGS:
                    raise CatalogueError(f'{path}:{number}: tags: {tag!r} is not one of '
                                         f'{", ".join(TAGS)}')
            override = (row.get('clip_override_db') or '').strip()
            if override:
                try:
                    if not math.isfinite(float(override)):
                        raise ValueError(override)
                except ValueError:
                    raise CatalogueError(f'{path}:{number}: clip_override_db: {override!r} '
                                         'is not a number') from None
            clip_path = (row.get('path') or '').strip()
            kept = (row.get('status') or '').strip() == MISSING
            for column, value in (('name', '' if kept else name), ('path', clip_path)):
                if not value:
                    continue
                first = seen[column].setdefault(value, number)
                if first != number:
                    raise CatalogueError(f'{path}:{number}: {column}: {value!r} is also on '
                                         f'line {first}')
            rows.append({'name': name, 'path': clip_path, 'tags': tags,
                         'override': override,
                         'duplicate_of': (row.get('duplicate_of') or '').strip(),
                         'line': number, 'fields': row})
    _check_duplicates(path, rows)
    return rows


def _check_duplicates(path, rows):
    """Refuse a ``duplicate_of`` that names no row, its own row, or another duplicate."""
    lines = {}
    for row in rows:
        lines.setdefault(row['name'], row)
    for row in rows:
        target = row['duplicate_of']
        if not target:
            continue
        where = f'{path}:{row["line"]}: duplicate_of: {target!r}'
        if target == row['name']:
            raise CatalogueError(f'{where} is the clip itself')
        kept = lines.get(target)
        if kept is None:
            raise CatalogueError(f'{where} is not a clip of the sheet')
        if kept['duplicate_of']:
            raise CatalogueError(f'{where} is itself marked duplicate_of '
                                 f'{kept["duplicate_of"]!r} on line {kept["line"]}; name the '
                                 'clip to keep (no chains)')


def match_sheet(items, rows):
    """
    Return ``(clip name -> row, orphan rows)``: each row's clip, by path first, then by name.

    A name only claims a row whose path is empty or not on disk; a row no clip claims is an
    orphan (kept in the sheet as ``missing``).
    """
    matched = {}
    unclaimed = dict(enumerate(rows))     # popped as claimed: the orphans are what is left
    by_path = {row['path']: index for index, row in unclaimed.items() if row['path']}
    for item in items:
        index = by_path.pop(item.path, None)
        if index is not None:
            matched[item.name] = unclaimed.pop(index)
    paths = {i.path for i in items}
    by_name = {}
    for index, row in unclaimed.items():
        if row['path'] not in paths:
            by_name.setdefault(row['name'], index)
    for item in items:
        if item.name not in matched:
            index = by_name.pop(item.name, None)
            if index is not None:
                matched[item.name] = unclaimed.pop(index)
    return matched, list(unclaimed.values())


def _sheet_fields(row):
    """Return ``(tags, override, duplicate_of)`` of a sheet row, or empty ones without one."""
    return (row['tags'], row['override'], row['duplicate_of']) if row else ((), '', '')


def _scan(sounds_dir):
    """Return sorted (set name, path relative to the set, absolute path) per audio file."""
    if not os.path.isdir(sounds_dir):
        raise CatalogueError(f'{sounds_dir}: not a directory')
    found = []
    for set_name in sorted(os.listdir(sounds_dir)):
        set_dir = os.path.join(sounds_dir, set_name)
        if not os.path.isdir(set_dir):
            continue
        for root, dirs, files in os.walk(set_dir):
            dirs.sort()
            for file in sorted(files):
                if file.lower().endswith(AUDIO_EXTENSIONS):
                    absolute = os.path.join(root, file)
                    found.append((set_name, os.path.relpath(absolute, set_dir), absolute))
    # os.walk gives a directory's files before its subdirectories: sort by the whole path,
    # so the repeat suffixes follow it alone.
    return sorted(found, key=lambda f: _rel_path(f[0], f[1]))


def _rel_path(set_name, rel):
    """Return a file's path inside ``sounds_dir`` as the catalogue writes it."""
    return f'{set_name}/{rel}'.replace(os.sep, '/')


class _Item:
    """A clip while it is being built."""

    def __init__(self, name, set_name, rel, absolute, label):
        self.name, self.set, self.rel, self.absolute, self.label = (
            name, set_name, rel, absolute, label)
        self.path = _rel_path(set_name, rel)
        self.listed = label == EXCLUDED_TAG
        self.measurement = self.features = self.kit_audio = self.suggestion = None
        self.fingerprint = None
        self.duplicate = ('', None, '')     # (suggested_duplicate_of, score, timing summary)
        self.timing = None              # the Timing of this clip against the kept one


def _unique_names(found):
    items, used = [], set()
    for set_name, rel, absolute in found:
        base = base_name(set_name, rel)
        name, n = base, 1
        while name in used:
            n += 1
            name = f'{base}-{n}'
        used.add(name)
        items.append(_Item(name, set_name, rel, absolute, label_of(rel)))
    return items


def _round(value, digits=2):
    return round(float(value), digits)


def _analyse(items, want_kit):
    """Decode and measure every item; a file that fails to decode stops the run."""
    good = []
    for item in items:
        try:
            samples, rate = soundfile.read(item.absolute, dtype='float32', always_2d=True)
        except (RuntimeError, OSError) as error:
            raise CatalogueError(f'{item.path}: cannot decode ({error}); nothing written, '
                                 'so the sheet keeps your tags') from None
        item.measurement = gain.measure(samples, rate)
        if want_kit:
            item.kit_audio = analysis.resample(
                analysis.to_mono(samples[:int(KIT_MAX_S * rate)]), rate, KIT_SAMPLE_RATE)
        item.features = analysis.extract_features(samples, rate)
        item.fingerprint = analysis.fingerprint(samples, rate)
        good.append(item)
    return good


def _suggest(items):
    """Fill ``suggestion`` (tag, confidence) on every item."""
    players = [i for i in items if not i.listed]
    labelled = [(i.features, i.label) for i in players if i.label]
    references = analysis.build_references(labelled, [i.features for i in players])
    for item in items:
        if item.listed:
            item.suggestion = (EXCLUDED_TAG, 1.0)
        elif item.label:
            item.suggestion = (item.label, 1.0)
        else:
            s = analysis.suggest(item.features, references)
            item.suggestion = (s.tag, s.confidence)


def _medians(items):
    """Return set name -> median RMS over its clips not on the exclusion list."""
    medians = {}
    for set_name in sorted({i.set for i in items}):
        rms = [i.measurement.rms_dbfs for i in items if i.set == set_name and not i.listed]
        medians[set_name] = gain.median(rms or [i.measurement.rms_dbfs for i in items
                                                if i.set == set_name])
    return medians


def keep_key(item, medians):
    """Return the sort key of the clip a duplicate group keeps (the smallest wins)."""
    return (not item.label, abs(item.measurement.rms_dbfs - medians[item.set]),
            len(item.name), item.name)


def _suggest_duplicates(items, threshold):
    """Fill ``duplicate`` on every item; return the scored pairs ``[(score, a, b)]``."""
    pairs = analysis.duplicate_pairs([i.fingerprint for i in items])
    parent = list(range(len(items)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    scores = {}
    for score, a, b in pairs:
        scores[(a, b)] = score
        if score >= threshold:
            parent[root(a)] = root(b)
    groups = {}
    for i in range(len(items)):
        groups.setdefault(root(i), []).append(i)
    medians = _medians(items)
    for members in groups.values():
        if len(members) < 2:
            continue
        kept = min(members, key=lambda i: keep_key(items[i], medians))
        for i in members:
            if i != kept:
                key = (min(i, kept), max(i, kept))
                score = scores.get(key)
                if score is None:
                    score = round(analysis.duplicate_score(items[i].fingerprint,
                                                           items[kept].fingerprint), 3)
                timing = analysis.timing_comparison(items[kept].fingerprint,
                                                    items[i].fingerprint)
                items[i].timing = timing
                items[i].duplicate = (items[kept].name, score, timing.summary())
    return [(score, items[a], items[b]) for score, a, b in pairs]


def _clips(items, sheet, threshold, house_level, duplicate_of=None):
    """Return ``(every clip, sets)``; the set counts leave out ``duplicate_of``'s keys."""
    duplicate_of = duplicate_of or {}
    medians = _medians(items)
    clips = []
    for item in sorted(items, key=lambda i: i.name):
        m, median = item.measurement, medians[item.set]
        set_gain = gain.set_gain_db(median, house_level)
        tags, override, _ = _sheet_fields(sheet.get(item.name))
        total = gain.clip_gain_db(set_gain, gain.outlier_trim_db(m.rms_dbfs, median),
                                  float(override or 0.0), m.peak_dbfs)
        suggested, confidence = item.suggestion
        merged = effective_tags(tags, suggested, confidence, threshold)
        clips.append(SoundClip(
            name=item.name, set_name=item.set, path=item.path,
            duration_ms=int(round(m.duration_s * 1000)), rms_dbfs=_round(m.rms_dbfs),
            peak_dbfs=_round(m.peak_dbfs), clip_fraction=_round(m.clip_fraction, 6),
            set_gain_db=_round(set_gain), gain_db=_round(total), tags=merged,
            suggested_tag=suggested, confidence=_round(confidence, 3),
            excluded=EXCLUDED_TAG in merged, label=item.label))
    sets = {}
    for set_name, median in medians.items():
        count = sum(1 for c in clips if c.set_name == set_name and c.name not in duplicate_of)
        sets[set_name] = SetInfo(count, _round(median),
                                 _round(gain.set_gain_db(median, house_level)))
    return clips, sets


def _write_text(path, text):
    """Write ``text`` unless the file already holds exactly it."""
    if os.path.isfile(path):
        with open(path, newline='') as f:
            if f.read() == text:
                return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='') as f:
        f.write(text)


def _write_bytes(path, data):
    if os.path.isfile(path):
        with open(path, 'rb') as f:
            if f.read() == data:
                return
    with open(path, 'wb') as f:
        f.write(data)


def _dump_yaml(catalogue, duplicates=()):
    data = catalogue.to_dict()
    data['duplicates'] = list(duplicates)
    return yaml.safe_dump(data, sort_keys=False, allow_unicode=True,
                          default_flow_style=None, width=100)


def _keep_generated(path, catalogue, duplicates=()):
    """Reuse the existing file's ``generated`` date when nothing else changed."""
    try:
        with open(path) as f:
            old = yaml.safe_load(f)
        before, _ = parse_catalogue(old)
        old_duplicates = old.get('duplicates') or []
    except (OSError, yaml.YAMLError, CatalogueError, TypeError, AttributeError):
        return catalogue
    if (old_duplicates == list(duplicates) and before.to_dict() | {'generated': ''}
            == catalogue.to_dict() | {'generated': ''}):
        return Catalogue(catalogue.house_level_dbfs, before.generated, catalogue.sets,
                         catalogue.clips)
    return catalogue


def _sheet_text(clips, matched, orphans, suggested_duplicates):
    lines = []
    for clip in clips:
        tags, override, duplicate_of = _sheet_fields(matched.get(clip.name))
        partner, score, timing = suggested_duplicates.get(clip.name, ('', None, ''))
        lines.append([clip.name, clip.set_name, clip.path, f'{clip.duration_ms / 1000:.2f}',
                      f'{clip.rms_dbfs:.2f}', clip.suggested_tag, f'{clip.confidence:.3f}',
                      ','.join(tags), override, duplicate_of, partner,
                      '' if score is None else f'{score:.3f}', timing, ''])
    for row in orphans:
        fields = row['fields']
        lines.append([row['name']] + [(fields.get(c) or '') for c in SHEET_COLUMNS[1:7]]
                     + [','.join(row['tags']), row['override'], row['duplicate_of']]
                     + [(fields.get(c) or '') for c in SHEET_COLUMNS[10:13]] + [MISSING])
    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator='\n')
    writer.writerow(SHEET_COLUMNS)
    writer.writerows(sorted(lines, key=lambda line: (line[0], line[2])))
    return buffer.getvalue()


PLAY_SCRIPT = """#!/bin/sh
# Play every clip of one bucket with afplay (macOS), printing its name first.
#   ./play_bucket.sh <bucket>        e.g. ./play_bucket.sh chatter
#   ./play_bucket.sh                 lists the buckets
cd "$(dirname "$0")" || exit 1
if [ -z "$1" ] || [ ! -d "$1" ]; then
    echo "usage: $0 <bucket>; buckets:" >&2
    for d in */; do echo "  ${d%/} ($(ls "$d" | wc -l | tr -d ' ') clips)" >&2; done
    exit 1
fi
for f in "$1"/*.wav; do
    echo "$(basename "$f" .wav)"
    afplay "$f"
done
"""

KIT_README = """# Tagging kit

Generated by `build_catalogue`; do not edit, it is rewritten on every run.

Each folder is a **suggested bucket**; each file is one clip, named as in the sheet, at the
catalogue's loudness, cut to 10 s. The suggestions are a first pass (by file name for the
labelled Padawan clips, by acoustic similarity or a few rules for the rest) and are to be
confirmed by ear.

    ./play_bucket.sh chatter        # plays each clip, printing its name first

The easiest way is the `tag_tool` web page (`tag_tool.py --workspace <r2_dev>`, see the
r2_sound README). Or open `config/tags.csv` in the `r2_sound` package and,
only where the suggestion is
wrong or missing, write the correct pools in the `tags` column (several allowed,
comma-separated) from this fixed list:

    {tags}

Leave `tags` empty to accept the suggestion when its `confidence` is at least {threshold},
otherwise the clip goes in `chatter`. `clip_override_db` (a number) raises or lowers one
clip that sits wrong in its set; the catalogue still keeps the peak at -1 dBFS.
`excluded` keeps a clip out of every pool (manual play only); any other tag on a clip of
the exclusion list makes it playable. Re-run `build_catalogue` to merge the sheet into
`clips.yaml`; it never touches your three columns (`tags`, `clip_override_db`,
`duplicate_of`), and a row whose file is gone stays, marked
`missing` in `status`, until the file is back.

Duplicates: `suggested_duplicate_of` names the clip that sounds the same (score
`duplicate_score`, 1.0 for the same samples) and is the one to keep; it is only a
suggestion. `duplicate_timing` compares the two: the silence before and after the sound, the
length of the sound itself, and a verdict: `same recording` (same length and rhythm; any
difference is padding or level), `other take` (the same phrase with its rhythm moved by tens
of ms and the pitch held), `stretched` (the same sound played longer or shorter, pitch held)
or `resampled` (slower or faster playback, the pitch moved with it). `other take` pairs are
the ones worth keeping both of for synth work. Listen to both, and to mark a duplicate write
the kept clip's name in
`duplicate_of` (or press `d` in the tag tool). A marked duplicate leaves `clips.yaml` (the
player never lists or picks it; it is listed under `duplicates:`) and its WAV moves here to
`duplicates/<name>__of__<kept>.wav`, so it can still be heard. `duplicate_of` must name
another clip of the sheet that is not itself marked a duplicate.
"""


def _write_kit(kit, items, clips, threshold, duplicate_of=None):
    by_name = {i.name: i for i in items}
    duplicate_of = duplicate_of or {}
    wanted = {}
    for clip in clips:
        kept = duplicate_of.get(clip.name)
        if kept:
            rel = os.path.join(DUPLICATES_DIR, f'{clip.name}{DUPLICATE_SEPARATOR}{kept}.wav')
        else:
            rel = os.path.join(clip.suggested_tag, f'{clip.name}.wav')
        wanted[rel] = clip
    for bucket in os.listdir(kit) if os.path.isdir(kit) else ():
        folder = os.path.join(kit, bucket)
        if (bucket in TAGS or bucket == DUPLICATES_DIR) and os.path.isdir(folder):
            for file in os.listdir(folder):
                if file.endswith('.wav') and os.path.join(bucket, file) not in wanted:
                    os.remove(os.path.join(folder, file))
            if not os.listdir(folder):
                shutil.rmtree(folder)
    for rel, clip in sorted(wanted.items()):
        target = os.path.join(kit, rel)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        audio = by_name[clip.name].kit_audio * 10.0 ** (clip.gain_db / 20.0)
        encoded = io.BytesIO()
        soundfile.write(encoded, np.clip(audio, -1.0, 1.0), KIT_SAMPLE_RATE, format='WAV',
                        subtype='PCM_16')
        _write_bytes(target, encoded.getvalue())
    script = os.path.join(kit, 'play_bucket.sh')
    _write_text(script, PLAY_SCRIPT)
    os.chmod(script, os.stat(script).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    _write_text(os.path.join(kit, 'README.md'),
                KIT_README.format(tags=', '.join(TAGS), threshold=threshold))


def _active_duplicates(sheet, items, matched, warnings):
    """
    Return ``{name: kept}`` for the clips on disk whose ``duplicate_of`` acts.

    A duplicate of a clip that is not on disk stays in the catalogue (with a warning), so
    the sound is not lost; its ``duplicate_of`` stays in the sheet. Tags on a duplicate are
    ignored with a warning.
    """
    on_disk = {i.name for i in items}
    active = {}
    for name in sorted(matched):
        row = matched[name]
        kept = row['duplicate_of']
        if not kept:
            continue
        if kept not in on_disk:
            warnings.append(f'{sheet}:{row["line"]}: {name} is duplicate_of {kept!r}, which '
                            f'has no clip on disk; {name} stays in the catalogue')
            continue
        if row['tags']:
            warnings.append(f'{sheet}:{row["line"]}: {name} has tags and duplicate_of; the '
                            'tags are ignored while it is a duplicate')
        active[name] = kept
    return active


def build(sounds_dir, out, sheet=None, kit=None, threshold=DEFAULT_THRESHOLD,
          house_level=gain.HOUSE_LEVEL_DBFS, today=None,
          duplicate_threshold=analysis.DUPLICATE_THRESHOLD, pairs_out=None,
          timings_out=None, timings_top=0):
    """
    Build the catalogue, the sheet and the kit; return ``(catalogue, warnings)``.

    ``sheet`` is read if it exists and always (re)written; ``kit`` is optional. A list
    given as ``pairs_out`` receives the scored pairs ``(score, clip a, clip b)``, best first.
    """
    warnings = []
    found = _scan(sounds_dir)
    if not found:
        raise CatalogueError(f'{sounds_dir}: no audio files found in any set directory')
    rows = read_sheet(sheet)                      # a bad sheet fails before the decoding
    items = _analyse(_unique_names(found), bool(kit))
    matched, orphans = match_sheet(items, rows)
    for row in orphans:
        warnings.append(f'{sheet}: row {row["name"]!r} ({row["path"] or "no path"}) has no '
                        f'clip on disk; kept as {MISSING}')
    duplicate_of = _active_duplicates(sheet, items, matched, warnings)
    _suggest(items)
    scored = _suggest_duplicates(items, duplicate_threshold)
    if pairs_out is not None:
        pairs_out.extend((score, a.name, b.name) for score, a, b in scored)
    if timings_out is not None:
        for _, a, b in scored[:timings_top]:
            timings_out[(a.name, b.name)] = analysis.timing_comparison(a.fingerprint,
                                                                       b.fingerprint)
    every, sets = _clips(items, matched, threshold, house_level, duplicate_of)
    clips = [c for c in every if c.name not in duplicate_of]
    duplicates = [{'name': c.name, 'path': c.path, 'duplicate_of': duplicate_of[c.name]}
                  for c in every if c.name in duplicate_of]
    today = today or datetime.date.today().isoformat()
    catalogue = _keep_generated(out, Catalogue(house_level, today, sets, tuple(clips)),
                                duplicates)
    _write_text(out, _dump_yaml(catalogue, duplicates))
    if sheet:
        suggested = {i.name: i.duplicate for i in items if i.duplicate[0]}
        _write_text(sheet, _sheet_text(every, matched, orphans, suggested))
    if kit:
        _write_kit(kit, items, every, threshold, duplicate_of)
    return catalogue, warnings


def summary(catalogue):
    """Return the headline numbers as printable lines."""
    clips = catalogue.clips
    lines = [f'{len(clips)} clips, {sum(c.excluded for c in clips)} excluded']
    for name, info in catalogue.sets.items():
        lines.append(f'  {name}: {info.clips} clips, median {info.median_rms_dbfs} dBFS, '
                     f'set_gain_db {info.set_gain_db}')
    counts = {}
    for c in clips:
        counts[c.suggested_tag] = counts.get(c.suggested_tag, 0) + 1
    lines.append('suggested buckets: ' + ', '.join(f'{t} {counts[t]}' for t in sorted(counts)))
    unlabelled = [c for c in clips if not c.label]
    lines.append(f'{len(unlabelled)} unlabelled clips, '
                 f'{sum(c.confidence >= DEFAULT_THRESHOLD for c in unlabelled)} at confidence '
                 f'>= {DEFAULT_THRESHOLD}')
    return lines


def main(args=None):
    """Command-line entry point."""
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'config')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1].strip())
    parser.add_argument('--sounds-dir', default='/sounds')
    parser.add_argument('--out', default=os.path.join(here, 'clips.yaml'))
    parser.add_argument('--sheet', default=os.path.join(here, 'tags.csv'))
    parser.add_argument('--kit', default=None, help='tagging kit directory (omit for none)')
    parser.add_argument('--threshold', type=float, default=DEFAULT_THRESHOLD)
    parser.add_argument('--duplicate-threshold', type=float,
                        default=analysis.DUPLICATE_THRESHOLD,
                        help='score (0..1) at which a pair is suggested as duplicates')
    parser.add_argument('--pairs', type=int, default=0, metavar='N',
                        help='print the N best-scoring duplicate pairs')
    parser.add_argument('--house-level', type=float, default=gain.HOUSE_LEVEL_DBFS)
    options = parser.parse_args(args)
    pairs = []
    timings = {}
    try:
        catalogue, warnings = build(options.sounds_dir, options.out, options.sheet,
                                    options.kit, options.threshold, options.house_level,
                                    duplicate_threshold=options.duplicate_threshold,
                                    pairs_out=pairs, timings_out=timings,
                                    timings_top=options.pairs)
    except CatalogueError as error:
        print(f'build_catalogue: {error}', file=sys.stderr)
        return 1
    for warning in warnings:
        print(f'WARN {warning}', file=sys.stderr)
    print('\n'.join(summary(catalogue)))
    print(f'duplicates: {len(catalogue_duplicates(options.out))} marked; '
          f'{sum(p[0] >= options.duplicate_threshold for p in pairs)} pairs at score >= '
          f'{options.duplicate_threshold}')
    if options.pairs > 0:
        print('  score  lead a/b s  trail a/b s  active a/b s  ratio  verdict  (clip a  clip b)')
    for score, a, b in pairs[:options.pairs]:
        t = timings[(a, b)]
        print(f'  {score:.3f}  {t.lead_a_s:.2f}/{t.lead_b_s:.2f}  '
              f'{t.trail_a_s:.2f}/{t.trail_b_s:.2f}  {t.active_a_s:.2f}/{t.active_b_s:.2f}  '
              f'{t.ratio:.2f}  {t.verdict}  ({a}  {b})')
    return 0


def catalogue_duplicates(path):
    """Return the ``duplicates`` list of a written ``clips.yaml`` (``[]`` if none)."""
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get('duplicates') or []


if __name__ == '__main__':
    sys.exit(main())
