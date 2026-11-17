# Typing measurement

Measures how you type QWERTY and how you press Taipo chords on the base layer,
then suggests combo terms (`get_combo_term()` in `keymap.c`) that catch the
chords but not your rolls.

## Privacy

The typing log has every key you press, so text and passwords can be read
back from it. Keep logs out of this repo (for example in
`~/.local/state/keylog/`), stop `record` with Ctrl+C before typing a
password, and delete the logs once you have the numbers.

## 1. Flash the measurement firmware

```sh
cd ~/qmk_firmware
nix shell nixpkgs#qmk -c qmk flash -kb zsa/voyager -km onjin -e KEYLOG=yes
```

(or `qmk compile ... -e KEYLOG=yes` and flash `zsa_voyager_onjin.bin` with
Keymapp). The keyboard works as usual and also writes every key event to the
HID console (`keylog.c`).

## 2. Record normal typing (20-30 min)

```sh
mkdir -p ~/.local/state/keylog
./keylog.py record ~/.local/state/keylog/typing.log
```

Type as usual: code, chat, mail, in English and Polish. Ctrl+C stops. You can
record several times into the same file.

## 3. Practise the chords (about 5 min)

```sh
./keylog.py chords ~/.local/state/keylog/chords.log
```

Press both bottom outer corner keys (MO 5) to enter silent mode: the keyboard
sends nothing to the computer, so chords cannot type or fire shortcuts. The
script shows a chord (QWERTY keys and what it gives), press it at once as you
would use it. Wrong keys are shown and asked again. Both corners again pause
and give the keyboard back; after the last chord do that to finish.

`--reps N` sets repetitions per chord (default 3), `--hand left|right` limits
the set.

## 4. Analyze

```sh
./keylog.py analyze ~/.local/state/keylog/*.log
```

Per group (inner thumb, Space, 3 fingers, 2 fingers) it shows:

- deliberate spread: time from the first to the last key of a practised chord
- collisions: how often all keys of a chord were held together while typing,
  and their spread
- suggested term: p99 of deliberate spread + 10 ms (`--margin`), and how many
  collisions would still fire

The table lists chords that would fire while typing, how often per hour, the
pause before their first key (`idle med`), and how many hits would remain with
a prior-idle check of 150 ms (`--idle`).

## 5. Back to the normal firmware

```sh
nix shell nixpkgs#qmk -c qmk flash -kb zsa/voyager -km onjin
```
