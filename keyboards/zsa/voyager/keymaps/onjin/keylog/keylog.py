#!/usr/bin/env python3
"""Measure QWERTY typing and Taipo chords on the base layer, pick combo terms.

Needs a KEYLOG=yes firmware (see README.md). Python 3 stdlib only.

  keylog.py record LOG          log normal typing until Ctrl+C
  keylog.py chords LOG          practise base-layer Taipo chords (silent mode)
  keylog.py analyze LOG...      report collisions and suggested combo terms
"""

import argparse
import glob
import math
import os
import random
import re
import sys
import time

# ---------- Layout -----------------------------------------------------------
# Base layer keycodes of the Taipo finger keys, as numbers (QMK encoding).
MOD = {'LCTL': 0x01, 'LSFT': 0x02, 'LALT': 0x04, 'LGUI': 0x08,
       'RCTL': 0x11, 'RSFT': 0x12, 'RALT': 0x14, 'RGUI': 0x18}


def kc(name):
    if len(name) == 1:
        return 0x04 + ord(name) - ord('A')
    return {'SCLN': 0x33, 'SPC': 0x2C, 'BSPC': 0x2A, 'ENT': 0x28}[name]


def MT(mod, key):
    return 0x2000 | ((MOD[mod] & 0x1F) << 8) | kc(key)


def LT(layer, key):
    return 0x4000 | ((layer & 0xF) << 8) | kc(key)


# Taipo key -> (base keycode, QWERTY label). OT outer thumb, IT inner thumb.
HANDS = {
    'left': {
        'R': (kc('Q'), 'Q'), 'S': (kc('W'), 'W'), 'N': (kc('E'), 'E'), 'I': (kc('R'), 'R'),
        'A': (MT('LGUI', 'A'), 'A'), 'O': (MT('LALT', 'S'), 'S'),
        'T': (MT('LCTL', 'D'), 'D'), 'E': (MT('LSFT', 'F'), 'F'),
        'OT': (LT(4, 'SPC'), 'Space'), 'IT': (kc('BSPC'), 'Bspc'),
    },
    'right': {
        'R': (kc('P'), 'P'), 'S': (kc('O'), 'O'), 'N': (kc('I'), 'I'), 'I': (kc('U'), 'U'),
        'A': (MT('RGUI', 'SCLN'), ';'), 'O': (MT('LALT', 'L'), 'L'),
        'T': (MT('RCTL', 'K'), 'K'), 'E': (MT('RSFT', 'J'), 'J'),
        'OT': (LT(2, 'SPC'), 'Space'), 'IT': (LT(3, 'ENT'), 'Enter'),
    },
}
MO5 = 0x5225  # MO(5), the silent mode toggle keys
LABEL = {code: lbl for hand in HANDS.values() for code, lbl in hand.values()}

# chords[] from taipo.c: fingers -> outputs (alone, + outer thumb, + inner thumb).
TAIPO = [
    ('R', 'r R >'), ('S', 's S }'), ('N', 'n N ]'), ('I', 'i I )'),
    ('A', 'a A <'), ('O', 'o O {'), ('T', 't T ['), ('E', 'e E ('),
    ('EO', 'c C 1'), ('TO', 'u U 2'), ('TA', 'q Q 3'), ('OA', 'l L 4'),
    ('IN', 'y Y 5'), ('IS', 'f F 6'), ('NS', 'p P 7'), ('NR', 'z Z 8'),
    ('SR', 'b B 9'), ('ET', 'h H 0'), ('EA', 'd D @'), ('IR', 'g G #'),
    ('TR', 'x X ^'), ('IO', 'k K +'), ('ES', 'v V *'), ('NA', 'j J ='),
    ('ER', 'm M $'), ('IA', 'w W &'), ('TS', '/ \\ |'), ('NO', '- _ %'),
    ('IT', '? ! -'), ('EN', ', . ~'), ('OR', '; : -'), ('SA', "' \" `"),
    ('INS', 'Tab Del FN'), ('ETO', 'Enter Esc AltGr'),
    ('AR', 'Gui Right PgUp'), ('OS', 'Alt Up Home'), ('TN', 'Ctrl Down End'), ('EI', 'Shift Left PgDn'),
]
THUMBS = ['', 'OT', 'IT']
LETTER_CHORDS = {f for f, outs in TAIPO if outs.split()[0].isalpha() and len(outs.split()[0]) == 1}
PAIRS = {'AR', 'OS', 'TN', 'EI'}


class Chord:
    def __init__(self, hand, fingers, thumb, out):
        self.hand, self.fingers, self.thumb, self.out = hand, fingers, thumb, out
        h = HANDS[hand]
        self.keys = frozenset(h[f][0] for f in fingers) | ({h[thumb][0]} if thumb else set())
        self.qwerty = '+'.join([h[f][1] for f in fingers] + ([h[thumb][1]] if thumb else []))
        self.name = f'{hand[0].upper()} {fingers}{"+" + thumb if thumb else ""}'

    @property
    def group(self):
        if self.thumb:
            return self.thumb
        return 'none3' if len(self.fingers) >= 3 else 'none2'

    @property
    def in_keymap(self):
        """Mapped in keymap.c."""
        if self.thumb == 'IT':
            return self.fingers not in PAIRS | {'INS'} and self.out != '-'
        if self.thumb == 'OT':
            return self.fingers in {'TS', 'NO', 'IT', 'EN', 'OR', 'SA', 'INS', 'ETO'}
        return self.fingers in {'ETO', 'INS'}

    @property
    def fw_term(self):
        """get_combo_term() in keymap.c."""
        return 70 if self.thumb == 'IT' else 50


def all_chords():
    out = []
    for hand in HANDS:
        for fingers, outs in TAIPO:
            for thumb, o in zip(THUMBS, outs.split()):
                # Single-finger letters on the base layer are plain keys.
                if not thumb and len(fingers) == 1:
                    continue
                out.append(Chord(hand, fingers, thumb, o))
    return out


GROUP_NAME = {'IT': 'fingers + inner thumb (Bspc/Enter)', 'OT': 'fingers + outer thumb (Space)',
              'none3': '3 fingers, no thumb', 'none2': '2 fingers, no thumb'}

# ---------- Console ----------------------------------------------------------
ZSA_VID = '00003297'
CONSOLE_USAGE = bytes.fromhex('0631ff0974')  # usage page 0xFF31, usage 0x74


def find_console():
    for d in sorted(glob.glob('/sys/class/hidraw/hidraw*')):
        try:
            with open(f'{d}/device/uevent') as f:
                uevent = f.read()
            with open(f'{d}/device/report_descriptor', 'rb') as f:
                desc = f.read()
        except OSError:
            continue
        if ZSA_VID in uevent.upper() and desc.startswith(CONSOLE_USAGE):
            return '/dev/' + os.path.basename(d)
    sys.exit('No QMK console found. Is a KEYLOG=yes firmware flashed?')


def console_lines():
    dev = find_console()
    print(f'Reading {dev}', file=sys.stderr)
    fd = os.open(dev, os.O_RDONLY)
    buf = b''
    while True:
        buf += os.read(fd, 64).rstrip(b'\0')
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            yield line.decode('ascii', 'replace').strip()


KL = re.compile(r'^KL (\d+) ([du]) (\d+) (\d+) ([0-9A-F]{4}) ([0-9A-F]+)$')


class Unwrap:
    """Firmware time is 16 bit ms, make it monotonic."""

    def __init__(self):
        self.last, self.off = None, 0

    def __call__(self, t):
        if self.last is not None and t + self.off < self.last - 30000:
            self.off += 65536
        self.last = t + self.off
        return self.last


def parse(lines):
    """Yield (time, pressed, keycode, layer_state) or ('mode', name)."""
    unwrap = Unwrap()
    for line in lines:
        if line.startswith('KL mode '):
            yield ('mode', line.split()[2])
            continue
        m = KL.match(line)
        if m:
            yield (unwrap(int(m[1])), m[2] == 'd', int(m[5], 16), int(m[6], 16))


# ---------- record -----------------------------------------------------------
def cmd_record(args):
    n = 0
    with open(args.log, 'a') as out:
        out.write(f'# record {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
        try:
            for line in console_lines():
                if not line.startswith('KL '):
                    continue
                out.write(line + '\n')
                out.flush()
                if line.split()[2] == 'd':
                    n += 1
                    if n % 200 == 0:
                        print(f'\r{n} presses', end='', file=sys.stderr)
        except KeyboardInterrupt:
            pass
    print(f'\n{n} presses saved to {args.log}', file=sys.stderr)


# ---------- chords -----------------------------------------------------------
def practice_set():
    """The chords keymap.c maps on the base layer."""
    return [c for c in all_chords() if c.in_keymap]


def attempts(events):
    """Group key events into attempts: first press until all keys are up.

    Yields ('mode', name) unchanged and (t0, t_last_press, keys) per attempt.
    """
    held, t0, last, keys = set(), None, None, set()
    for ev in events:
        if ev[0] == 'mode':
            held, t0, keys = set(), None, set()
            yield ev
            continue
        t, down, code, _ = ev
        if down:
            if t0 is None:
                t0, keys = t, set()
            keys.add(code)
            last = t
            held.add(code)
        elif code in held:
            held.discard(code)
            if t0 is not None and not held:
                yield (t0, last, frozenset(keys))
                t0 = None


def cmd_chords(args):
    chords = practice_set()
    if args.hand:
        chords = [c for c in chords if c.hand == args.hand]
    todo = [c for c in chords for _ in range(args.reps)]
    random.shuffle(todo)
    print('Press both bottom outer corner keys (MO 5) to enter silent mode.\n'
          'In silent mode the keyboard sends nothing to the computer, so chords\n'
          'cannot type or trigger shortcuts. Press both corners again to pause\n'
          'and get the keyboard back. Press each chord at once, as you would use it.\n')
    out = open(args.log, 'a')
    out.write(f'# chords {time.strftime("%Y-%m-%d %H:%M:%S")}\n')

    def tee():
        for line in console_lines():
            if line.startswith('KL '):
                out.write(line + '\n')
                out.flush()
                yield line

    silent = False
    i = 0
    try:
        for a in attempts(parse(tee())):
            if a[0] == 'mode':
                silent = a[1] == 'silent'
                if silent:
                    print('-- silent mode --')
                    show(todo[i], i, len(todo))
                else:
                    print('-- paused, the keyboard works again. Corners to go on, Ctrl+C to quit --')
                continue
            t0, last, keys = a
            if not silent or keys == {MO5}:
                continue
            c = todo[i]
            if keys == c.keys:
                print(f'   ok, {last - t0} ms')
                i += 1
                if i == len(todo):
                    print('\nDone. Press both corners to get the keyboard back.')
                    break
                show(todo[i], i, len(todo))
            else:
                got = '+'.join(LABEL.get(k, f'{k:04X}') for k in keys)
                print(f'   got {got}, again')
    except KeyboardInterrupt:
        print()
    out.close()
    print(f'Saved to {args.log}')


def show(c, i, n):
    hand = 'left ' if c.hand == 'left' else 'right'
    print(f'[{i + 1}/{n}] {hand}  {c.qwerty:<16} → {c.out}')


# ---------- analyze ----------------------------------------------------------
def load(path):
    """Return (typing sessions, chord sessions): lists of event lists."""
    typing, chords, cur = [], [], None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('# record'):
                cur = typing
                cur.append([])
            elif line.startswith('# chords'):
                cur = chords
                cur.append([])
            elif line.startswith('KL ') and cur is not None:
                cur[-1].append(line)
    return [list(parse(s)) for s in typing], [list(parse(s)) for s in chords]


def overlaps(events, chords):
    """Every time all keys of a chord were held together.

    Mirrors QMK combos: a chord starts at the press of one of its keys, any
    press of a key outside the chord or a release of one of its keys cancels
    it. Only events on layer 0 with no other layer active count.
    Returns {chord: [(spread ms, idle ms before the first key, press indexes)]}.
    """
    res = {c: [] for c in chords}
    state = {c: None for c in chords}
    last_press = None
    silent = False
    for i, ev in enumerate(events):
        if ev[0] == 'mode':
            silent = ev[1] == 'silent'
            state = {c: None for c in chords}
            continue
        t, down, code, layers = ev
        if silent:
            continue
        if down and layers & ~1:
            state = {c: None for c in chords}
            last_press = t
            continue
        for c in chords:
            s = state[c]
            if down:
                if code in c.keys:
                    if s is None:
                        idle = t - last_press if last_press is not None else 10**6
                        s = state[c] = {'t0': t, 'got': {code}, 'idle': idle, 'idx': []}
                    else:
                        s['got'].add(code)
                    s['idx'].append(i)
                    if s['got'] == c.keys:
                        res[c].append((t - s['t0'], s['idle'], frozenset(s['idx'])))
                        state[c] = None
                else:
                    state[c] = None
            elif s is not None and code in c.keys:
                state[c] = None
        if down:
            last_press = t
    return res


SIMULTANEOUS = 8  # ms, rolls are never this tight
UNDO = {kc('BSPC')}


def split_uses(events, occ):
    """Separate combos the firmware fired from overlaps of other chords.

    A keymap chord held together within its term fired, so it counts as a use
    (bigger chords first), and its presses do not count for sub-chords.
    Returns (uses {chord: [(spread, idle, undone)]}, collisions like occ).
    """
    fired = sorted(((c, o) for c, os in occ.items() if c.in_keymap for o in os if o[0] <= c.fw_term),
                   key=lambda co: -len(co[0].keys))
    used_idx = set()
    uses = {c: [] for c in occ}
    presses = [i for i, e in enumerate(events) if e[0] != 'mode' and e[1]]
    for c, (spread, idle, idx) in fired:
        if idx & used_idx:
            continue
        used_idx |= idx
        # Undone: the next press after the chord is Backspace.
        after = [i for i in presses if i > max(idx) and events[i][2] not in c.keys]
        undone = bool(after) and events[after[0]][2] in UNDO
        uses[c].append((spread, idle, undone))
    coll = {c: [o for o in os if not o[2] & used_idx] for c, os in occ.items()}
    # Keys pressed in the same few ms are not a roll but a deliberate chord
    # (an old combo from muscle memory, a missed key of a chord).
    deliberate = {c: [o for o in os if o[0] < SIMULTANEOUS] for c, os in coll.items()}
    coll = {c: [o for o in os if o[0] >= SIMULTANEOUS] for c, os in coll.items()}
    return uses, coll, deliberate


def deliberate(events, chords):
    """Chord practice: spread of each correct attempt, keyed by chord."""
    by_keys = {c.keys: c for c in chords}
    res = {c: [] for c in chords}
    silent = False
    for a in attempts(events):
        if a[0] == 'mode':
            silent = a[1] == 'silent'
            continue
        t0, last, keys = a
        c = by_keys.get(keys)
        if silent and c:
            res[c].append(last - t0)
    return res


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, max(0, math.ceil(p / 100 * len(xs)) - 1))] if xs else None


def cmd_analyze(args):
    chords = all_chords()
    typing, practice = [], []
    for path in args.logs:
        t, p = load(path)
        typing += t
        practice += p

    hours = 0.0
    presses = 0
    seen = set()
    for s in typing:
        evs = [e for e in s if e[0] != 'mode']
        presses += sum(1 for e in evs if e[1])
        seen |= {e[2] for e in evs}
        # Active time: gaps over 5 s do not count.
        ms = sum(min(b[0] - a[0], 5000) for a, b in zip(evs, evs[1:]))
        hours += ms / 3_600_000
    missing = [f'{h} {lbl}' for h, keys in HANDS.items() for code, lbl in keys.values() if code not in seen]
    print(f'Typing: {presses} presses, {hours * 60:.1f} min active')
    if typing and missing:
        print(f'  never pressed: {", ".join(missing)} (layout changed?)')

    coll = {c: [] for c in chords}
    uses = {c: [] for c in chords}
    simul = {c: [] for c in chords}
    for s in typing:
        u, col, sim = split_uses(s, overlaps(s, chords))
        for c in chords:
            coll[c] += col[c]
            uses[c] += u[c]
            simul[c] += sim[c]
    deli = {c: [] for c in chords}
    for s in practice:
        for c, v in deliberate(s, chords).items():
            deli[c] += v

    # Typing rhythm: gaps between consecutive presses, overlapping or not.
    gaps = []
    for s in typing:
        downs = [e[0] for e in s if e[0] != 'mode' and e[1] and not e[3] & ~1]
        gaps += [b - a for a, b in zip(downs, downs[1:]) if b - a < 1000]
    if gaps:
        print(f'  gap between presses: p5 {pct(gaps, 5)} ms, median {pct(gaps, 50)} ms')

    print()
    terms = {}
    for g in ['IT', 'OT', 'none3', 'none2']:
        gc = [c for c in chords if c.group == g]
        d = [x for c in gc for x in deli[c]]
        col = sorted(x[0] for c in gc for x in coll[c])
        if d:
            term = min(100, max(25, pct(d, 99) + args.margin))
        else:
            term = None
        terms[g] = term
        print(f'== {GROUP_NAME[g]} ==')
        if d:
            print(f'  deliberate spread: n {len(d)}, median {pct(d, 50)}, p95 {pct(d, 95)}, '
                  f'p99 {pct(d, 99)}, max {max(d)} ms')
        else:
            print('  deliberate spread: no practice data')
        if col:
            print(f'  collisions while typing: n {len(col)}, spread min {col[0]}, '
                  f'p5 {pct(col, 5)}, median {pct(col, 50)} ms')
        if term:
            hit = sum(1 for x in col if x <= term)
            print(f'  suggested term {term} ms (p99 + {args.margin}): {hit} collisions'
                  f'{f", {hit / hours:.1f}/h" if hours else ""}')
        print()

    used = sorted(((c, u) for c, u in uses.items() if u), key=lambda cu: -len(cu[1]))
    if used:
        print(f'Keymap combos fired while typing (within their term; undone = next key was Backspace,'
              ' a likely misfire):')
        print(f'  {"chord":<12} {"qwerty":<16} {"out":<6} {"uses":>5} {"undone":>6} {"spread med":>10} {"idle med":>8}')
        for c, u in used:
            print(f'  {c.name:<12} {c.qwerty:<16} {c.out:<6} {len(u):>5} {sum(x[2] for x in u):>6} '
                  f'{pct([x[0] for x in u], 50):>10} {pct([x[1] for x in u], 50):>8}')
        print()

    sim = sorted(((c, v) for c, v in simul.items() if v), key=lambda cv: -len(cv[1]))
    if sim:
        print(f'Pressed together on purpose (spread < {SIMULTANEOUS} ms) but not a keymap combo,'
              ' e.g. old combos from muscle memory or a chord with a missed key:')
        print('  ' + ', '.join(f'{c.qwerty} {len(v)}x' for c, v in sim))
        print()

    print(f'Chords that would fire while typing (term per group as above, '
          f'idle = pause before the first key, prior-idle {args.idle} ms would block idle < {args.idle}):')
    rows = []
    for c in chords:
        term = terms[c.group] or args.default_term
        hits = [x for x in coll[c] if x[0] <= term]
        if not hits and not c.in_keymap:
            continue
        blocked = sum(1 for x in hits if x[1] < args.idle)
        rows.append((len(hits), c, hits, blocked, term))
    rows.sort(key=lambda r: (-r[0], r[1].name))
    print(f'  {"chord":<12} {"qwerty":<16} {"out":<6} {"map":<4} {"term":>4} {"hits":>5} {"/h":>5} '
          f'{"idle med":>8} {"left w/ idle":>12}')
    for n, c, hits, blocked, term in rows:
        if n == 0 and not args.all:
            continue
        idle = pct([x[1] for x in hits], 50) if hits else ''
        per_h = f'{n / hours:.1f}' if hours else '-'
        print(f'  {c.name:<12} {c.qwerty:<16} {c.out:<6} {"yes" if c.in_keymap else "":<4} {term:>4} {n:>5} '
              f'{per_h:>5} {idle:>8} {n - blocked:>12}')
    if not any(r[0] for r in rows):
        print('  none')

    few = [GROUP_NAME[g] for g in ['IT', 'OT'] if sum(len(deli[c]) for c in chords if c.group == g) < 50]
    if hours * 60 < 15 or few:
        print('\nNot enough data for reliable terms:')
        if hours * 60 < 15:
            print(f'  {hours * 60:.1f} min of typing, record at least 15-30 min')
        for g in few:
            print(f'  under 50 practised chords for {g}, run "chords" to the end')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record', help='log normal typing until Ctrl+C')
    r.add_argument('log')
    c = sub.add_parser('chords', help='practise base-layer Taipo chords')
    c.add_argument('log')
    c.add_argument('--reps', type=int, default=3)
    c.add_argument('--hand', choices=['left', 'right'])
    a = sub.add_parser('analyze', help='report collisions and suggested terms')
    a.add_argument('logs', nargs='+')
    a.add_argument('--margin', type=int, default=10, help='ms added to deliberate p99')
    a.add_argument('--idle', type=int, default=150, help='prior-idle threshold to evaluate')
    a.add_argument('--default-term', type=int, default=50, help='term for groups without practice data')
    a.add_argument('--all', action='store_true', help='also list keymap chords without hits')
    args = ap.parse_args()
    {'record': cmd_record, 'chords': cmd_chords, 'analyze': cmd_analyze}[args.cmd](args)


if __name__ == '__main__':
    main()
