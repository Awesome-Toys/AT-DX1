# AT-DX1 CW mode

AT-DX1's CW mode is designed so that the **radio performs the CW work** and the Android application acts primarily as the user interface/terminal.

## Onboard receive decoder

In the current RP2350/RP2354A firmware, core 1 runs the CW receive DSP. It scans approximately 300 to 900 Hz, acquires a dominant plausible CW tone, tracks that tone, derives mark/space timing and estimates receive speed automatically.

The current noise-guard logic requires stable/dominant spectral acquisition before tone lock, rejects short pre-lock impulses, and withholds decoded text until the timing is coherent. The timing lock is cleared when the tracked signal disappears so that a new station can be acquired cleanly.

The decoder reports decoded text plus useful status such as receive WPM, tracked tone, frequency error and quality.

## Onboard transmit encoder/keyer

Entering CW terminal mode moves the CDC port from normal CAT parsing into the CW terminal. Printable text is queued for transmission and the RP2354A creates the Morse timing.

The current keyer uses 5 ms raised-cosine edges on TXSW rather than hard switching the carrier instantaneously.

Current terminal commands include:

| Command | Function |
|---|---|
| `%CW` | Enter CW terminal mode |
| ordinary text | Queue text for CW transmission |
| `%F14060000` | Set actual CW carrier to 14.060000 MHz |
| `%W20` | Set transmit speed to 20 WPM |
| `%T700` | Set receive/BFO tone target to 700 Hz |
| `%R` | Reacquire receive tone |
| `%S` or `%?` | Status/help |
| `%Q` | Exit CW terminal and return to CAT/FT8 mode |

During CW receive the RF LO is offset by the selected receive tone; `%F` continues to represent the actual on-air CW carrier frequency.

## Android companion app

The companion app UI is intended to expose the radio's CW functions in a field-friendly interface. The supplied development screenshot shows:

- USB connection/disconnection state;
- phone RX-audio option;
- signal level indication;
- decoded RX CW text;
- Clear / Reacquire / Status controls;
- live text entry for immediate CW transmission;
- 40 m / 30 m / 20 m / 17 m selection;
- fine tuning in 1 / 10 / 100 Hz steps;
- TX WPM control;
- CW receive/BFO tone control;
- a CW waterfall around the selected tone;
- receive status including WPM, tracked tone and decoder quality.

The Android application source was not included in the supplied files used to assemble this repository snapshot, so this repository currently contains its documentation and screenshot rather than a buildable Android Studio project.
