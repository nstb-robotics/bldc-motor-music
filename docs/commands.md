# Commands

Type commands in the Serial Monitor (line ending "Newline").
`help` lists all commands; `st` (or `s`) shows the current settings.

## How to type

- The motor boards and the LED board each have their own USB connection and Serial Monitor. Commands entered on the LED board are listed in [LED board](#led-board-type-on-the-led-board).
- Name, a space, then the value: `mvol 50`. Commands without a value are the name only: `st`.
- Several commands in one line, separated by `;`: `mvol 50; avol 60`.
- A value outside the range is set to the nearest end of the range: `imax 5` sets 2.5.
- After a setting is changed, the related settings are printed.
- An unknown command with a value prints `? unknown: <name>`. An unknown command without a value prints `need value (or unknown): <name>` (`need value: <name>` on the LED board). Neither changes anything.

## Saving

Settings change at once. `save` stores every setting that can be saved, at its current value; they
are used again after power-up and stay after you upload the firmware again. This includes
values you only changed to test, so check with `st` first.

Not saved — back to the default at power-up: the sound choice (`patch`; all channels start as
epiano), `tone`, `hmode`, `bias`, `calb`, `hallmon`, `cpu`.

`cal` saves its own result. `factory` erases the saved settings but keeps the `cal` result;
power-cycle the board to use the defaults.

"Default" in the tables below is the value when nothing has been saved.

## show

| Command | Shows |
|---|---|
| `st` | All current settings |
| `help` | All commands |
| `id` | Board name (Motor 1 / Motor 2), firmware version, driver, `POLE_PAIRS`, `R_EFF`, and `crash=1` if there is a crash report |
| `plist` | The values of the 7 sounds and the drums |
| `spd` | Rotation: mode, target and measured speed, field voltage, lag |
| `hall` | The hall sensor state now (A, B, C) |
| `htab` | The hall angles found by `cal` |
| `stat` | A record since `statclr`: audio input, interrupt time, limiter, peak voltage and calculated current, stereo link, LED data. See [Reading stat](#reading-stat) |
| `statclr` | Clears the `stat` record |
| `cpu 1` / `cpu 0` | Starts measuring the processing load / stops and prints the result |
| `crash` | The crash report from the previous run, if there was one |

## do

| Command | Does |
|---|---|
| `save` | Stores the settings |
| `factory` | Erases the saved settings (the `cal` result is kept). Power-cycle to use the defaults |
| `panic` | Stops all notes |
| `cal` | Finds the phase order and the hall sensor angles. The wheel turns slowly forward and back; it must be free to turn. Saves its own result |

## midi

| Command | Value | Default | Saved | What it does |
|---|---|---|---|---|
| `mvol` | 0–100 | 72.5 | yes | MIDI volume. −10 dB per 25, 0 = off. At 100, a full-scale sound reaches `imax` (calculated with `R_EFF`) |
| `poly` | 4–16 | 16 | yes | Most notes at once. When all are in use, the quietest note gives way to the new one |
| `patch` | 1–7 | — | no | Plays every channel with this sound (until a program change, if `pc 1`), and chooses the sound that `atk` … `p3` change. After power-up all channels are epiano |
| `pc` | 0 / 1 | 1 | yes | 1: program changes in the song choose the sound for each channel. 0: they are ignored |
| `a4` | 400–480 Hz | 440 | yes | Pitch of A4 |
| `vcurve` | 0.2–4 | 1 | yes | How velocity sets loudness. 1: in proportion. Above 1: soft notes get softer |
| `bend` | 0–24 semitones | 2 | yes | Pitch-bend range. A song can change it per channel (RPN 0) |
| `lfo` | 0.1–20 Hz | 5.5 | yes | Vibrato speed |
| `vdep` | 0–2 semitones | 1 | yes | Vibrato depth with the mod wheel at full |
| `drum` | 0 / 1 | 1 | yes | 1: channel 10 plays drums. 0: channel 10 is silent |
| `dlev` | 0–4 | 1.5 | yes | Level of all drums |

### Shaping a sound

`patch <1-7>` chooses the sound; these commands then change it. The settings of all 7 sounds are saved with `save`; the choice made with `patch` is not.
`plist` shows the values of every sound.

| Command | Value | What it does |
|---|---|---|
| `atk` | 0.0005–1 s | Attack: time to rise to full level |
| `dec` | 0.02–20 s | Decay: after the attack, the level falls toward `sus`; in this time it falls about two-thirds of the way |
| `sus` | 0–1 | Level kept while the key is held. 0: the note fades out even while held |
| `rel` | 0.01–5 s | Release: after the key is let go, the level falls about two-thirds of the way in this time |
| `plev` | 0–4 | Level of this sound |
| `p1` `p2` `p3` | 0–20 (see below) | Depend on the sound (table below) |

| Sound | `p1` | `p2` | `p3` | Defaults: atk / dec / sus / rel / plev / p1 / p2 / p3 |
|---|---|---|---|---|
| 1 epiano | Level of the overtones at the start | How fast the overtones fade [s] | 3rd harmonic, relative to the 2nd | 0.001 / 2.0 / 0 / 0.15 / 1.0 / 0.4 / 0.4 / 0.5 |
| 2 fm-epiano | FM depth at the start | How fast the FM depth fades [s] | Modulator frequency ratio | 0.001 / 2.5 / 0 / 0.2 / 0.8 / 2.0 / 0.25 / 1.0 |
| 3 organ | Level of the 2nd harmonic | Level of the 3rd harmonic | Level of the 4th harmonic | 0.01 / 1.0 / 1 / 0.08 / 0.7 / 0.6 / 0.4 / 0.2 |
| 4 guitar | Clipping (above 1: rounder, louder tops) | — | — | 0.002 / 3.0 / 0 / 0.2 / 0.7 / 4 / — / — |
| 5 saw | — | — | — | 0.001 / 1.0 / 1 / 0.03 / 0.5 / — / — / — |
| 6 square | Pulse width, 0.05–0.95 | — | — | 0.001 / 1.0 / 1 / 0.03 / 0.5 / 0.5 / — / — |
| 7 triangle | Number of steps, 0–32 (below 2: smooth) | — | — | 0.001 / 1.0 / 1 / 0.03 / 0.8 / 16 / — / — |

## drum

The drum commands take a note number: any note of that drum works.

| Drum | Notes |
|---|---|
| kick | 35, 36 |
| snare | 37–40 |
| hi-hat | 42, 44 |
| open hi-hat | 46, 51, 53, 59 |
| cymbal | 49, 52, 55, 57 |
| tom | 41, 43, 45, 47, 48, 50 |

| Command | Value | What it does |
|---|---|---|
| `dvol <note> <x>` | 0–4 | Level of that drum |
| `ddec <note> <s>` | 0.01–5 s | How fast it fades: in this time it falls about two-thirds of the way |
| `dfreq <note> <Hz>` | kick, tom: 20–1000. Others: 100–40000 | Kick, tom: the pitch it settles to (tom: at note 41; higher notes are higher). Others: how fast the noise changes; lower is darker |
| `dsweep <note> <x>` | 0–10 | Kick, tom only: the pitch starts at `dfreq` × (1 + this) and falls |
| `dstime <note> <s>` | 0.005–1 s | Kick, tom only: how fast the pitch falls |

All saved. Defaults:

| Drum | `dvol` | `ddec` | `dfreq` | `dsweep` | `dstime` |
|---|---|---|---|---|---|
| kick | 2.0 | 0.15 | 60 | 2.5 | 0.04 |
| snare | 0.35 | 0.10 | 12000 | — | — |
| hi-hat | 0.2 | 0.03 | 40000 | — | — |
| open hi-hat | 0.2 | 0.25 | 40000 | — | — |
| cymbal | 0.2 | 0.8 | 25000 | — | — |
| tom | 0.9 | 0.2 | 90 | 0.6 | 0.06 |

## eq

MIDI and PC audio each have their own EQ: commands starting with `m` are for MIDI, `a` for audio
(for example `mbass` and `abass`). All saved; both start with the same defaults.

| Command | Value | Default | What it does |
|---|---|---|---|
| `mhpf` / `ahpf` | 0, 20–20000 Hz | 60 | Cuts below this frequency. 0 = off |
| `mlpf` / `alpf` | 0, 20–20000 Hz | 9265 | Cuts above this frequency. 0 = off |
| `mbass` / `abass` | −30 to +30 dB | 0 | Bass shelf. 0 = off |
| `mbfreq` / `abfreq` | 20–20000 Hz | 120 | Where the bass shelf works |
| `mtreble` / `atreble` | −30 to +30 dB | 0 | Treble shelf. 0 = off |
| `mtfreq` / `atfreq` | 20–20000 Hz | 2500 | Where the treble shelf works |
| `mmid` / `amid <1-4> <Hz> <dB> [Q]` | 20–20000 Hz, −30 to +30 dB, Q 0.3–10 | 250, 500, 1000, 3000 Hz; 0 dB; Q 2 | Up to 4 peaking filters. 0 dB = off. Larger Q is narrower |

## audio

| Command | Value | Default | Saved | What it does |
|---|---|---|---|---|
| `avol` | 0–100 | 92.5 | yes | PC audio volume. Same scale as `mvol` |
| `usbvol` | 0 / 1 | 1 | yes | 1: the PC's volume slider changes the level. 0: the slider is ignored, and the level is as if the slider were at 100 % |
| `role` | 0–2 | `MOTOR_NUMBER` | yes | 0: one motor. 1: plays right, sends left to Motor 2. 2: plays the left from Motor 1 |
| `src` | 0–2 | 0 | yes | With `role 0`: 0 left + right mixed, 1 left, 2 right |
| `tone` | 0–10000 Hz | 0 | no | Plays a sine wave instead of the PC audio, for checking. 0 = off |

## spin

Rotation needs the hall sensors and `cal`. Without them, `smode 1`–`3` prints `smode: run cal first`.

| Command | Value | Default | Saved | What it does |
|---|---|---|---|---|
| `smode` | 0–3 | 1 | yes | 0: no rotation. 1: constant speed (`orpm`). 2: follows how many notes are played. 3: follows the loudness |
| `orpm` | 0–120 rpm | 4 | yes | Speed for `smode 1` |
| `ostr` | 0–0.85 V | 0.5 | yes | Strength of the rotating field. Too weak and the wheel can stall; stronger takes more of the sound's limit |
| `hold` | 0–30 s | 1 | yes | After the sound stops, the wheel keeps turning this long |
| `sdir` | 1 / −1 | 1 | yes | Direction |
| `tsync` | 0 / 1 | 0 | yes | `smode 1` only. 1: follows the tempo of the MIDI clock: speed = `orpm` × (BPM ÷ `tbase`) ^ `tgain`, kept within `tmin`–`tmax` |
| `tbase` | 20–300 BPM | 120 | yes | Tempo at which the wheel turns at `orpm` |
| `tgain` | 0.5–5 | 2 | yes | How strongly the tempo changes the speed |
| `tmin` | 0–30 rpm | 3 | yes | Lowest speed (`tsync`, `smode 3`) |
| `tmax` | 1–60 rpm | 40 | yes | Highest speed (`tsync`, `smode 2`, `smode 3`) |
| `nbase` | 0–30 rpm | 3 | yes | `smode 2`: speed with no notes |
| `nk` | 0–20 rpm | 5 | yes | `smode 2`: speed added per note per second |
| `ntau` | 0.05–5 s | 0.5 | yes | `smode 2`: how long the note count is averaged |

## hall

Usually not needed. `cal` sets everything here.

| Command | Value | Default | Saved | What it does |
|---|---|---|---|---|
| `hmode` | 0 / 1 | 1 after `cal`, else 0 | no | 1: uses the hall sensors (needs `cal`). 0: does not; a small voltage (`bias`) holds the rotor in one place |
| `bias` | 0–3.6 V | 0.24 (0 after `cal`) | no | Voltage that holds the rotor when the hall sensors are not used |
| `calb` | 0–1.8 V | 0.6 | no | Voltage used during `cal` |
| `toff` | −180 to 180° | −15 | yes | Correction to the angle at which the sound is applied, in electrical degrees |
| `hallmon` | 0 / 1 | 0 | no | 1: prints the hall state each time it changes |

## drive

| Command | Value | Default | Saved | What it does |
|---|---|---|---|---|
| `vbus` | 8–30 V | 12 | yes | Your motor supply voltage. Set it to the real value |
| `imax` | 0.2 A – `IMAX_LIMIT` | 2.5 | yes | Current limit. The firmware cannot measure the current; it limits the current calculated with `R_EFF` |
| `hfield` | 0–0.85 V | 0.5 | yes | Weak field that holds the rotor still when the wheel is not turning; keeps the sound clean. Uses some current, like rotation |
| `pwmf` | 22059, 14706, 11029, 8824 Hz | 22059 | yes | PWM frequency. Other values are set to the nearest of these |
| `lmode` | 0 / 1 | 0 | yes | Limiter. 0: turns the whole sound down when it would pass the limit. 1: rounds only the tops: louder, with some distortion |
| `limrel` | 0.01–2 s | 0.15 | yes | `lmode 0`: how fast the level comes back after the limiter |
| `lmodel` | 50–5000 Hz | 1000 | yes | Corner frequency of the coil model the limiter uses; the model's inductance is `R_EFF` ÷ (2π × `lmodel`). The default is not measured |
| `vmax` | 0–15 V | 0 | yes | Highest voltage for the sound. 0, or a value above 45 % of `vbus`: 45 % of `vbus` |
| `dtm` | 0 / 1 | 1 | yes | Dead-time compensation on / off |
| `dtns` | 0–1500 ns | 270 | yes | Dead time of the driver, for the compensation |
| `dti` | 0.02–1 A | 0.2 | yes | Below about this current, the compensation is made smaller |

## led (type on the motor board)

The LEDs are set in two places. The **motor board** decides what it sends to the LED board:
notes always, and for PC audio, the kind of display (`spec`). The **LED board** decides where
and how the LEDs light: see [LED board](#led-board-type-on-the-led-board).

The spectrum is sent only while PC audio plays.

| Command | Value | Default | Saved | What it does |
|---|---|---|---|---|
| `spec` | 0–3 | 2 | yes | 0: no spectrum. 1: frequency bands (60 Hz – 8 kHz). 2: note names. 3: level meter |
| `sjoin` | 0 / 1 | 1 | yes | With two motors and `spec 1` or `2`: Motor 1 analyses left + right together and drives both rows |
| `ssplit` | 36–96 | 60 | yes | With `sjoin 1` and `spec 2`: note number that divides the low row from the high row |
| `sonset` | 0 / 1 | 0 | yes | `spec 1`, `2`: 1 lights only when a note starts |
| `sharm` | 0 / 1 | 0 | yes | `spec 2`: 1 removes overtones from the display |
| `sgain` | −30 to 40 dB | 0 | yes | Sensitivity |
| `srange` | 10–80 dB | 30 | yes | Range from dark to full brightness |
| `stilt` | 0–6 dB/octave | 3 | yes | `spec 1`: lifts the higher bands |

## misc

| Command | Value | Default | Saved | What it does |
|---|---|---|---|---|
| `echo` | 0 / 1 | 0 | yes | 1: prints every note on and off |
| `mon` | 0 / 1 | 0 | yes | 1: prints program changes, controllers, pitch-bend range, MIDI clock, and a summary each second |

## LED board (type on the LED board)

Connect the LED board's USB and open its own Serial Monitor. These commands set where and how
the LEDs light. What is sent to them (notes, spectrum) is set on the motor boards: see
[led](#led-type-on-the-motor-board). `mon` here is not the same as `mon` on the motor board:
it prints what the LED board receives.

`help` lists the commands; `st` shows the settings.

| Command | Value | Default | Saved | What it does |
|---|---|---|---|---|
| `pos1` / `pos2` | 1–64 | 61 / 34 | yes | LED number (cable end = 1) of the C of Motor 1's / Motor 2's row |
| `bright` | 0–1 | 0.5 | yes | Brightness |
| `bgain` | 0.5–4 | 1 | yes | Extra brightness gain |
| `fade` | 0.03–2 s | 0.15 | yes | After a note ends, its LED goes dark in about this time |
| `hdec` | 0, 0.1–10 s | 2 | yes | While a note is held, its LED dims with this time. 0: stays lit |
| `clo` / `chi` | `clo` 0–126, `chi` 1–127; `clo` must be lower than `chi` | 36 / 84 | yes | Lowest and highest notes of the colour scale (the colour shows the octave) |
| `sdec` | 0.03–2 s | 0.15 | yes | How fast the spectrum and meter fall |
| `phold` | 0–3 s | 0.6 | yes | Meter: how long the peak is held |
| `pfade` | 0.05–2 s | 0.3 | yes | Meter: how fast the peak fades |
| `mhue0` / `mhue1` | −360 to 720° | 120 / 0 | yes | Meter colour at the outer end / at the center (hue) |
| `mcurve` | 0.2–5 | 1 | yes | Meter: how the colour changes from end to center |
| `test` | 0 / 1 | 0 | no | 1: lights each row in 12 colours, to check `pos1` and `pos2` |
| `mon` | 0 / 1 | 0 | no | 1: prints what the board receives |
| `off` | — | — | — | Turns all LEDs off |
| `save`, `st`, `help`, `crash` | — | — | — | As on the motor board |

## Diagnostics

### Reading stat

| Line | Meaning | Normal |
|---|---|---|
| `blocks` `updates` `overrun` `underrun` | Audio received from the PC. `underrun` counts gaps | `underrun` does not keep rising |
| `fill min` `max` | How full the audio buffer was | — |
| `isr max` `avg` `budget` | Interrupt time: longest, average, and the time available (22.67 µs) | `max` stays below `budget` |
| `late` | Interrupts that started late | 0 |
| `shed` | Notes released to keep up | 0 |
| `peak in` | Loudest PC audio received [dBFS] | — |
| `limiter max reduction` | Most the limiter turned the sound down [dB] | Large: the sound often hits the limit |
| `peak V` `peak I(calc)` | Highest voltage, and highest current as calculated with `R_EFF` | `I` near `imax`: the sound hits the limit |
| `link tx` `txdrop` `rx` `bad` `gap` | Stereo link. Motor 1 counts `tx`, Motor 2 counts `rx` | `txdrop`, `bad`, `gap` do not keep rising |
| `led notes dropped` | Notes that could not be sent to the LED board | 0 |
| `spec frames` `drop` `fft max` | Spectrum frames sent, dropped, and the longest FFT time | — |

### cpu

`cpu 1` starts, `cpu 0` stops and prints a summary: interrupt time (average, longest, share of the
time available), times over budget, `shed`, the number of notes at the longest time, the longest
loop time, and the time per number of notes and per stage.

### Messages

| Message | Meaning |
|---|---|
| `HALL ERROR (hall=…): rotation stopped. check hall 5V / GND / wiring` | The hall signals are invalid; rotation stops. `hall ok: rotation enabled` when they return |
| `WARNING: config.h says MOTOR_NUMBER … but the saved role is …` | The saved `role` does not match `MOTOR_NUMBER`. Check `MOTOR_NUMBER`, or set `role` and `save` |
| `WARNING: the last run stopped with a crash (type crash to show the report)` | Shown at power-up when the previous run ended with a crash. `crash` shows the report |
| `cal: not done yet (type cal with the wheel free to turn)` | Shown at power-up until `cal` has been done |
| `hmode: run cal first` | Using the hall sensors needs `cal` |
| `smode: run cal first` | Rotation needs `cal` |

## Notes

- Windows remembers each board by its USB port. Keep each board on the same port; moving it can mix up the names.
