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
Local web tool to tag the R2 sound clips (step 2 of the catalogue workflow).

Standard library only, so it runs on the Mac with just the workspace checkout::

    python3 src/r2_personality/r2_sound/r2_sound/tools/tag_tool.py --workspace ~/r2_dev

It serves one page on 127.0.0.1 that lists the clips of ``config/tags.csv`` by suggested
bucket (each group collapsible, the choice kept in the browser; a filter by review state,
bucket or set), plays the kit WAVs of ``<workspace>/.audio/tagging`` and writes David's three
columns, ``tags``, ``clip_override_db`` and ``duplicate_of``, back into the sheet
(atomically, keeping one ``.bak``). A save re-reads the sheet from disk and changes only
those columns of the rows it names, so a ``build_catalogue`` run while the page was open
is not undone. Afterwards run ``build_catalogue`` in the container to merge the sheet.
"""

import argparse
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import webbrowser

try:
    from r2_sound.catalogue import TAGS as SOUND_TAGS
except ImportError:
    # Mirrors r2_sound.catalogue.TAGS (not importable on the Mac); keep in step.
    SOUND_TAGS = ('chatter', 'excited', 'sad', 'question', 'ooh', 'hum', 'alarm', 'scream',
                  'laugh', 'annoyed', 'whistle', 'calm', 'scared', 'sleepy', 'greeting',
                  'yes', 'no', 'misc', 'excluded')

THRESHOLD = 0.6
DUPLICATE_FIELDS = ('duplicate_of', 'suggested_duplicate_of', 'duplicate_score',
                    'duplicate_timing')
DUPLICATES_DIR = 'duplicates'      # build_catalogue's kit folder: <name>__of__<kept>.wav
DUPLICATE_SEPARATOR = '__of__'
SHEET_REL = Path('src/r2_personality/r2_sound/config/tags.csv')
KIT_REL = Path('.audio/tagging')


class SheetError(ValueError):
    """A save request or the sheet itself is not acceptable."""


def read_rows(sheet):
    """Return ``(fieldnames, rows)`` of the sheet, each row a dict of strings."""
    with open(sheet, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        fields = list(reader.fieldnames or ())
        for column in ('name', 'tags', 'clip_override_db'):
            if column not in fields:
                raise SheetError(f'{sheet}: header lacks the column {column}')
        return fields, list(reader)


def index_kit(kit):
    """
    Return ``{clip name: {bucket: path}}`` for every ``<kit>/<bucket>/<name>.wav``.

    A marked duplicate's ``duplicates/<name>__of__<kept>.wav`` is indexed under ``name``.
    """
    found = {}
    kit = Path(kit)
    if kit.is_dir():
        for bucket in sorted(p for p in kit.iterdir() if p.is_dir()):
            for wav in sorted(bucket.glob('*.wav')):
                name = wav.stem
                if bucket.name == DUPLICATES_DIR:
                    name = name.split(DUPLICATE_SEPARATOR, 1)[0]
                found.setdefault(name, {})[bucket.name] = wav
    return found


def kit_path(index, name, suggested):
    """Return the kit WAV of a clip: its suggested bucket's, else any bucket's, else None."""
    buckets = index.get(name)
    if not buckets:
        return None
    return buckets.get(suggested) or next(iter(buckets.values()))


def clips_payload(sheet, index):
    """Return the ``/api/clips`` document: tags list, threshold and the rows with audio."""
    _, rows = read_rows(sheet)
    clips = []
    for row in rows:
        if not (row.get('name') or '').strip():
            continue
        clip = {k: (v or '') for k, v in row.items() if k is not None}
        for field in DUPLICATE_FIELDS:
            clip.setdefault(field, '')
        clip['has_audio'] = kit_path(index, clip['name'], clip.get('suggested_tag', '')) \
            is not None
        clips.append(clip)
    return {'tags': list(SOUND_TAGS), 'threshold': THRESHOLD, 'clips': clips}


def validate_changes(changes, known):
    """
    Return ``{name: (tags list, override text, duplicate_of)}`` or raise SheetError.

    ``duplicate_of`` is None when the change does not carry it (the column is left alone),
    else the name of a known clip other than ``name``, or ``''``.
    """
    if not isinstance(changes, dict):
        raise SheetError('body must be an object {name: {tags, clip_override_db, '
                         'duplicate_of}}')
    out = {}
    for name, change in changes.items():
        if name not in known:
            raise SheetError(f'unknown clip {name!r}')
        if not isinstance(change, dict):
            raise SheetError(f'{name}: expected an object')
        tags = change.get('tags', [])
        if not isinstance(tags, list):
            raise SheetError(f'{name}: tags must be a list')
        clean = []
        for tag in tags:
            if tag not in SOUND_TAGS:
                raise SheetError(f'{name}: {tag!r} is not one of {", ".join(SOUND_TAGS)}')
            if tag not in clean:
                clean.append(tag)
        override = change.get('clip_override_db', '')
        if override is None or (isinstance(override, str) and not override.strip()):
            text = ''
        elif isinstance(override, bool):
            raise SheetError(f'{name}: clip_override_db {override!r} is not a number')
        else:
            try:
                value = float(override)
                if not math.isfinite(value):
                    raise ValueError(override)
            except (TypeError, ValueError):
                raise SheetError(f'{name}: clip_override_db {override!r} is not a number') \
                    from None
            text = override.strip() if isinstance(override, str) else repr(override)
        duplicate_of = change.get('duplicate_of')
        if duplicate_of is not None:
            if not isinstance(duplicate_of, str):
                raise SheetError(f'{name}: duplicate_of must be a clip name')
            duplicate_of = duplicate_of.strip()
            if duplicate_of == name:
                raise SheetError(f'{name}: duplicate_of names the clip itself')
            if duplicate_of and duplicate_of not in known:
                raise SheetError(f'{name}: duplicate_of {duplicate_of!r} is not a clip of '
                                 'the sheet')
        out[name] = (clean, text, duplicate_of)
    return out


def check_chains(rows):
    """Raise SheetError when a row's ``duplicate_of`` names a row that is a duplicate too."""
    marked = {}
    for row in rows:
        name = (row.get('name') or '').strip()
        marked.setdefault(name, (row.get('duplicate_of') or '').strip())
    for name, kept in marked.items():
        if kept and marked.get(kept):
            raise SheetError(f'{name}: duplicate_of {kept!r}, which is itself a duplicate of '
                             f'{marked[kept]!r}; name the clip to keep (no chains)')


def save_changes(sheet, changes):
    """
    Write ``tags``, ``clip_override_db`` and ``duplicate_of`` of the named rows into the sheet.

    The sheet is read from disk now (not when the page loaded) and the changes are applied
    by name onto those rows: every other column and row, and the row order, stay as the
    file has them; a name not in the file is refused (nothing written). A sheet without a
    ``duplicate_of`` column gets one after ``clip_override_db`` when a change sets one. The
    previous sheet is kept as ``<sheet>.bak`` and the new one replaces it atomically.
    Returns the row count changed.
    """
    sheet = Path(sheet)
    fields, rows = read_rows(sheet)
    known = {(r.get('name') or '').strip() for r in rows}
    valid = validate_changes(changes, known)
    if 'duplicate_of' not in fields and any(v[2] for v in valid.values()):
        fields.insert(fields.index('clip_override_db') + 1, 'duplicate_of')
    count = 0
    for row in rows:
        name = (row.get('name') or '').strip()
        if name in valid:
            tags, override, duplicate_of = valid[name]
            row['tags'] = ','.join(tags)
            row['clip_override_db'] = override
            if duplicate_of is not None and 'duplicate_of' in fields:
                row['duplicate_of'] = duplicate_of
            count += 1
    check_chains(rows)
    fd, tmp = tempfile.mkstemp(dir=sheet.parent, prefix=sheet.name + '.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fields, lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)
        shutil.copyfile(sheet, str(sheet) + '.bak')
        os.replace(tmp, sheet)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return count


def make_server(sheet, kit, port=8765):
    """Return a ThreadingHTTPServer bound to 127.0.0.1 serving the tool."""
    sheet = Path(sheet)
    index = index_kit(kit)
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype):
            if isinstance(body, str):
                body = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj), 'application/json')

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            try:
                if path in ('/', '/index.html'):
                    self._send(200, PAGE, 'text/html; charset=utf-8')
                elif path == '/api/clips':
                    with lock:
                        self._json(200, clips_payload(sheet, index))
                elif path.startswith('/audio/') and path.endswith('.wav'):
                    from urllib.parse import unquote
                    name = unquote(path[len('/audio/'):-len('.wav')])
                    buckets = index.get(name)
                    wav = next(iter(buckets.values())) if buckets else None
                    if wav is None:
                        self._json(404, {'error': 'no such clip'})
                        return
                    with lock:
                        suggested = ''
                        try:
                            _, rows = read_rows(sheet)
                            for row in rows:
                                if row.get('name') == name:
                                    suggested = row.get('suggested_tag') or ''
                        except (OSError, SheetError):
                            pass
                    wav = kit_path(index, name, suggested)
                    self._send(200, wav.read_bytes(), 'audio/wav')
                else:
                    self._json(404, {'error': 'not found'})
            except (OSError, SheetError) as e:
                self._json(500, {'error': str(e)})

        def do_POST(self):
            if self.path.split('?', 1)[0] != '/api/save':
                self._json(404, {'error': 'not found'})
                return
            try:
                length = int(self.headers.get('Content-Length') or 0)
                changes = json.loads(self.rfile.read(length) or b'{}')
                with lock:
                    count = save_changes(sheet, changes)
            except (SheetError, ValueError) as e:
                self._json(400, {'error': str(e)})
            except OSError as e:
                self._json(500, {'error': str(e)})
            else:
                self._json(200, {'saved': count})

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main(argv=None):
    """Run the tool."""
    p = argparse.ArgumentParser(description='Local web tool to tag the R2 sound clips.')
    p.add_argument('--workspace', default='.', help='the r2_dev checkout (default: .)')
    p.add_argument('--sheet', default=None, help='tags.csv (default: in the workspace)')
    p.add_argument('--kit', default=None, help='tagging kit (default: <workspace>/.audio/tagging)')
    p.add_argument('--port', type=int, default=8765)
    p.add_argument('--no-browser', action='store_true', help='do not open the page')
    args = p.parse_args(argv)
    ws = Path(args.workspace).expanduser()
    sheet = Path(args.sheet).expanduser() if args.sheet else ws / SHEET_REL
    kit = Path(args.kit).expanduser() if args.kit else ws / KIT_REL
    if not sheet.is_file():
        print(f'tag_tool: no sheet at {sheet}', file=sys.stderr)
        return 2
    server = make_server(sheet, kit, args.port)
    url = f'http://127.0.0.1:{server.server_address[1]}/'
    print(f'tag_tool: {url}  (sheet {sheet}, kit {kit}); Ctrl-C to stop', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>R2 tag tool</title>
<style>
:root { color-scheme: light dark; --bg:#fafafa; --fg:#1b1b1b; --mut:#666; --card:#fff;
  --line:#d4d4d4; --acc:#1a5fb4; --on:#1a5fb4; --onfg:#fff; --warn:#a15c00; }
@media (prefers-color-scheme: dark) { :root { --bg:#16181b; --fg:#e6e6e6; --mut:#9aa0a6;
  --card:#1f2226; --line:#3a3f45; --acc:#78aeed; --on:#78aeed; --onfg:#111; --warn:#e5a50a; } }
body { margin:0; background:var(--bg); color:var(--fg); font:15px/1.4 system-ui,sans-serif; }
#note { padding:6px 12px; border-bottom:1px solid var(--line); color:var(--mut); font-size:13px; }
#bar { display:flex; gap:12px; align-items:center; padding:8px 12px;
  border-bottom:1px solid var(--line); flex-wrap:wrap; }
#main { display:flex; gap:12px; padding:12px; align-items:flex-start; }
#list { width:300px; max-height:calc(100vh - 120px); overflow:auto; background:var(--card);
  border:1px solid var(--line); border-radius:6px; flex:none; }
#list h3 { margin:0; padding:4px 8px; font-size:13px; background:var(--bg);
  position:sticky; top:0; border-bottom:1px solid var(--line); cursor:pointer;
  user-select:none; display:flex; justify-content:space-between; }
#list h3.cur { color:var(--acc); }
#list h3 .arrow { display:inline-block; width:1em; }
.item { padding:3px 8px; cursor:pointer; display:flex; justify-content:space-between; }
.item.cur { background:var(--on); color:var(--onfg); }
.item.done .n::before { content:"\2713 "; }
.item.dirty .n::after { content:" *"; }
.dupmark { font-size:11px; border:1px solid currentColor; border-radius:3px; padding:0 3px;
  margin-left:4px; }
#dup { margin-top:12px; padding-top:8px; border-top:1px solid var(--line); }
input[type=text] { font:inherit; width:14em; padding:3px; background:var(--card);
  color:var(--fg); border:1px solid var(--line); border-radius:4px; }
#panel { flex:1; min-width:0; background:var(--card); border:1px solid var(--line);
  border-radius:6px; padding:12px; }
#panel h2 { margin:0 0 4px; }
.mut { color:var(--mut); } .warn { color:var(--warn); }
#tags { display:flex; flex-wrap:wrap; gap:6px; margin:12px 0; }
button { font:inherit; padding:5px 10px; border:1px solid var(--line); border-radius:5px;
  background:var(--card); color:var(--fg); cursor:pointer; }
button.on { background:var(--on); color:var(--onfg); border-color:var(--on); }
button kbd { opacity:.7; margin-right:4px; font-size:12px; }
button.primary { border-color:var(--acc); }
input[type=number] { font:inherit; width:6em; padding:3px; background:var(--card);
  color:var(--fg); border:1px solid var(--line); border-radius:4px; }
audio { width:100%; margin:8px 0; }
.row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; margin-top:10px; }
#msg { min-height:1.3em; }
</style></head><body>
<div id="note">After saving, run <code>build_catalogue</code> in the container (command in the
r2_sound README, &ldquo;Tagging and re-running&rdquo;) to merge the sheet into clips.yaml.</div>
<div id="bar">
  <label>Filter <select id="filter"></select></label>
  <span id="progress"></span>
  <span class="mut">Keys: space replay, Enter confirm+next, arrows prev/next,
    1-9,0 toggle tags, d mark suggested duplicate+next, k play kept, c collapse/expand
    the group</span>
  <button id="collapse">Collapse all</button><button id="expand">Expand all</button>
  <button id="save" class="primary">Save</button><span id="dirty" class="warn"></span>
  <span id="msg"></span>
</div>
<div id="main">
  <div id="list"></div>
  <div id="panel"><p class="mut">Loading&hellip;</p></div>
</div>
<datalist id="names"></datalist>
<script>
"use strict";
let TAGS = [], THR = 0.6, clips = [], byName = {}, orig = {}, cur = null, filt = 'all';
let collapsed = new Set();            // the collapsed groups (bucket names), kept per browser
try { collapsed = new Set(JSON.parse(localStorage.getItem('r2tag.collapsed') || '[]')); }
catch (e) { /* no storage: the groups start open */ }
const remember = () => { try { localStorage.setItem('r2tag.collapsed',
  JSON.stringify([...collapsed])); } catch (e) { /* ignore */ } };
const $ = id => document.getElementById(id);
const el = (t, props, ...kids) => { const e = document.createElement(t);
  Object.assign(e, props || {}); kids.forEach(k => e.append(k)); return e; };
const split = s => (s || '').toLowerCase().split(/[,;\s]+/).filter(Boolean);
const state = c => ({tags: c.tags_list.slice(), ov: c.ov, dup: c.dup});
const dirty = c => { const o = orig[c.name];
  return o.tags.join(',') !== c.tags_list.join(',') || o.ov !== c.ov || o.dup !== c.dup; };
const reviewed = c => c.tags_list.length > 0 || !!c.dup;
const audioSrc = n => '/audio/' + encodeURIComponent(n) + '.wav';
const dirtyCount = () => clips.filter(dirty).length;

async function load() {
  const r = await fetch('/api/clips'); const d = await r.json();
  TAGS = d.tags; THR = d.threshold; clips = d.clips; byName = {}; orig = {};
  clips.forEach(c => { c.tags_list = split(c.tags); c.ov = (c.clip_override_db || '').trim();
    c.dup = (c.duplicate_of || '').trim();
    byName[c.name] = c; orig[c.name] = state(c); });
  const N = $('names'); N.innerHTML = '';
  clips.forEach(c => N.append(el('option', {value: c.name})));
  const buckets = [...new Set(clips.map(bucketOf))].sort();
  const sets = [...new Set(clips.map(c => 'set: ' + (c.set || '')))].sort();
  const f = $('filter'); f.innerHTML = '';
  ['all', 'unreviewed'].concat(buckets, sets).forEach(
    b => f.append(el('option', {value: b, textContent: b})));
  f.onchange = () => { filt = f.value; renderList(); };
  f.onkeydown = e => e.stopPropagation();
  cur = clips.length ? clips[0].name : null;
  renderAll();
}
const bucketOf = c => c.suggested_tag || '(none)';
const visible = c => filt === 'all' ? true : filt === 'unreviewed' ? !reviewed(c)
  : filt.startsWith('set: ') ? (c.set || '') === filt.slice(5) : bucketOf(c) === filt;
const shown = c => visible(c) && !collapsed.has(bucketOf(c));   // listed and navigable
function toggleGroup(b) { if (collapsed.has(b)) collapsed.delete(b); else collapsed.add(b);
  remember(); renderList(); }
function collapseAll(on) { clips.map(bucketOf).forEach(
  b => on ? collapsed.add(b) : collapsed.delete(b)); remember(); renderList(); }

function renderAll() { renderList(); renderPanel(true); renderStatus(); }
function renderStatus() {
  const n = dirtyCount(), done = clips.filter(reviewed).length;
  $('progress').textContent = `reviewed ${done} / ${clips.length}`;
  $('dirty').textContent = n ? `unsaved ${n}` : '';
}
function renderList() {
  const L = $('list'); L.innerHTML = '';
  const groups = {};
  clips.forEach(c => (groups[bucketOf(c)] = groups[bucketOf(c)] || []).push(c));
  Object.keys(groups).sort().forEach(b => {
    const all = groups[b], shown = all.filter(visible);
    if (!shown.length) return;
    const nrev = all.filter(reviewed).length, closed = collapsed.has(b);
    const h = el('h3', {className: closed && all.some(c => c.name === cur) ? 'cur' : '',
        title: closed ? 'click to expand' : 'click to collapse'},
      el('span', {}, el('span', {className: 'arrow', textContent: closed ? '\u25b8' : '\u25be'}),
        b),
      el('span', {className: 'mut', textContent: `${nrev}/${all.length} reviewed`}));
    h.onclick = () => toggleGroup(b); L.append(h);
    if (closed) return;
    shown.forEach(c => { const d = el('div', {className: 'item' + (c.name === cur ? ' cur' : '') +
        (reviewed(c) ? ' done' : '') + (dirty(c) ? ' dirty' : '')},
        el('span', {}, el('span', {className: 'n', textContent: c.name}),
          c.dup ? el('span', {className: 'dupmark', title: 'duplicate of ' + c.dup,
                              textContent: 'dup'}) : ''),
        el('span', {className: 'mut', textContent: c.confidence || ''}));
      d.dataset.name = c.name; d.onclick = () => select(c.name); L.append(d); });
  });
}
function renderPanel(play) {
  const P = $('panel'); P.innerHTML = '';
  const c = byName[cur]; if (!c) { P.textContent = 'No clips.'; return; }
  const conf = parseFloat(c.confidence);
  const eff = c.tags_list.length ? 'tags set: the suggestion is not used'
    : (conf >= THR ? `takes effect (confidence >= ${THR})`
      : `confidence < ${THR}: falls to chatter`);
  P.append(el('h2', {textContent: c.name}),
    el('div', {className: 'mut', textContent:
      `${c.set} | ${c.duration_s} s | RMS ${c.rms_dbfs} dBFS | status ${c.status || 'ok'}`}),
    el('div', {textContent: `Suggestion: ${c.suggested_tag || '(none)'} ` +
      `(confidence ${c.confidence || '?'}) `},
      el('span', {className: 'warn', textContent: eff})));
  if (c.has_audio) { const a = el('audio', {controls: true, autoplay: !!play,
      src: '/audio/' + encodeURIComponent(c.name) + '.wav'}); a.id = 'player'; P.append(a); }
  else P.append(el('p', {className: 'warn', textContent: 'No kit WAV found for this clip.'}));
  const T = el('div', {id: 'tags'});
  TAGS.forEach((t, i) => { const b = el('button', {className: c.tags_list.includes(t) ? 'on' : ''},
      el('kbd', {textContent: i < 10 ? String((i + 1) % 10) : ''}), t);
    b.onclick = () => toggle(t); T.append(b); });
  P.append(T);
  const ov = el('input', {type: 'number', step: 'any', value: c.ov});
  ov.oninput = () => { c.ov = ov.value.trim(); renderList(); renderStatus(); };
  ov.onkeydown = e => e.stopPropagation();
  P.append(el('label', {}, 'clip_override_db ', ov));
  const row = el('div', {className: 'row'});
  [['Previous', () => step(-1)], ['Confirm', () => confirmClip()], ['Clear', () => clear()],
   ['Next', () => step(1)]].forEach(
    ([n, f]) => row.append(el('button', {textContent: n, onclick: f})));
  P.append(row);
  renderDuplicate(P, c);
}
function renderDuplicate(P, c) {
  const D = el('div', {id: 'dup'});
  const sug = c.suggested_duplicate_of;
  if (c.dup) D.append(el('div', {className: 'warn', textContent:
    `Marked duplicate of ${c.dup}: left out of clips.yaml (tags ignored)`}));
  if (sug) D.append(el('div', {textContent:
    `Suggested duplicate of ${sug} (score ${c.duplicate_score || '?'})`}));
  if (sug && c.duplicate_timing) D.append(el('div', {className: 'mut',
    textContent: c.duplicate_timing}));
  if (sug && (c.duplicate_timing || '').startsWith('other take')) D.append(el('div',
    {className: 'mut', textContent:
    'Same phrase, different timing: worth keeping both for the synth'}));
  const keptName = c.dup || sug;
  if (keptName) {
    const k = el('audio', {controls: true, src: audioSrc(keptName)}); k.id = 'kept';
    const r = el('div', {className: 'row'});
    r.append(el('button', {onclick: playKept}, el('kbd', {textContent: 'k'}),
      `Play kept (${keptName})`));
    if (sug && sug !== c.dup) r.append(el('button', {onclick: () => markDup(sug)},
      el('kbd', {textContent: 'd'}), `Mark duplicate of ${sug}`));
    D.append(r, k);
  }
  const inp = el('input', {type: 'text', value: c.dup, placeholder: 'clip to keep'});
  inp.setAttribute('list', 'names');
  inp.onkeydown = e => { e.stopPropagation(); if (e.key === 'Enter') markDup(inp.value.trim()); };
  inp.onchange = () => markDup(inp.value.trim());
  const r2 = el('div', {className: 'row'});
  r2.append(el('label', {}, 'Duplicate of ', inp),
    el('button', {textContent: 'Not a duplicate', onclick: () => markDup('')}));
  D.append(r2);
  P.append(D);
}
function markDup(target) {
  const c = byName[cur]; if (!c) return false;
  if (target) {
    let why = '';
    if (target === c.name) why = 'a clip cannot be a duplicate of itself';
    else if (!byName[target]) why = `${target} is not a clip of the sheet`;
    else if (byName[target].dup) why = `${target} is itself marked duplicate of ` +
      `${byName[target].dup}; mark the clip to keep`;
    else { const kids = clips.filter(o => o.dup === c.name).map(o => o.name);
      if (kids.length) why = `${kids.join(', ')} already name ${c.name} as the clip to keep`; }
    if (why) { $('msg').textContent = 'Not marked: ' + why; return false; }
  }
  c.dup = target; $('msg').textContent = '';
  renderList(); renderPanel(false); renderStatus(); return true;
}
function playKept() { const k = $('kept'); if (!k) return;
  const a = $('player'); if (a) a.pause(); k.currentTime = 0; k.play(); }
function select(name, play) { const k = $('kept'); if (k) k.pause();
  cur = name; renderList(); renderPanel(play !== false);
  const e = document.querySelector('.item.cur'); if (e) e.scrollIntoView({block: 'nearest'}); }
function step(d) { const v = clips.filter(shown); if (!v.length) return;
  let i = v.findIndex(c => c.name === cur);
  i = i < 0 ? 0 : Math.max(0, Math.min(v.length - 1, i + d)); select(v[i].name); }
function toggle(t) { const c = byName[cur], i = c.tags_list.indexOf(t);
  if (i < 0) c.tags_list.push(t); else c.tags_list.splice(i, 1);
  renderList(); renderPanel(false); renderStatus(); }
function confirmClip() { const c = byName[cur]; if (!c.suggested_tag) return;
  if (!c.tags_list.includes(c.suggested_tag)) c.tags_list.push(c.suggested_tag);
  renderList(); renderPanel(false); renderStatus(); }
function clear() { const c = byName[cur]; c.tags_list = []; c.ov = '';
  renderList(); renderPanel(false); renderStatus(); }
async function save() {
  const ch = {};
  clips.filter(dirty).forEach(
    c => ch[c.name] = {tags: c.tags_list, clip_override_db: c.ov, duplicate_of: c.dup});
  if (!Object.keys(ch).length) { $('msg').textContent = 'nothing to save'; return; }
  const r = await fetch('/api/save', {method: 'POST', body: JSON.stringify(ch)});
  const d = await r.json();
  if (!r.ok) { $('msg').textContent = 'Not saved: ' + d.error; return; }
  clips.forEach(c => orig[c.name] = state(c));
  $('msg').textContent = `saved ${d.saved} rows`; renderList(); renderStatus();
}
$('save').onclick = save;
$('collapse').onclick = () => collapseAll(true);
$('expand').onclick = () => collapseAll(false);
window.addEventListener('beforeunload', e => {
  if (dirtyCount()) { e.preventDefault(); e.returnValue = ''; } });
document.addEventListener('keydown', e => {
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  if (e.key === ' ') { e.preventDefault(); const a = $('player');
    if (a) { a.currentTime = 0; a.play(); } }
  else if (e.key === 'Enter') { e.preventDefault(); confirmClip(); step(1); renderStatus(); }
  else if (e.key === 'ArrowRight' || e.key === 'ArrowDown') { e.preventDefault(); step(1); }
  else if (e.key === 'ArrowLeft' || e.key === 'ArrowUp') { e.preventDefault(); step(-1); }
  else if (e.key === 'd') { e.preventDefault(); const c = byName[cur];
    if (c && c.suggested_duplicate_of && markDup(c.suggested_duplicate_of)) step(1); }
  else if (e.key === 'k') { e.preventDefault(); playKept(); }
  else if (e.key === 'c') { e.preventDefault(); const c = byName[cur];
    if (c) toggleGroup(bucketOf(c)); }
  else if (/^[0-9]$/.test(e.key)) { const i = (parseInt(e.key, 10) + 9) % 10;
    if (i < TAGS.length) toggle(TAGS[i]); }
});
load();
</script></body></html>
"""


if __name__ == '__main__':
    sys.exit(main())
