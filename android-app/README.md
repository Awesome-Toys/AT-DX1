# AT-DX1 Android CW companion app

The current installable Android package is:

[`app-debug.apk`](app-debug.apk)

This APK is intended to work with the 2026-09-16 centred-CW AT-DX1 firmware. The radio handles RF tuning, band limits, USB-sideband receive and Morse TX timing; the phone handles the CW receive presentation/DSP.

## Current CW receive architecture

The app sends only the actual wanted RF using `%F<Hz>`. AT-DX1 validates that frequency and internally sets:

`RX LO = wanted RF - 10,005 Hz`

The wanted CW signal therefore appears at **10.005 kHz audio**.

The app receive chain is:

`AT-DX1 USB audio -> noise blanker -> centred 10.005 kHz channel -> 300 Hz CW filter -> decoder + AGC -> 300..800 Hz phone BFO`

## Current UI/DSP functions

- USB CDC control of AT-DX1.
- 40/30/20/17 m band selection and fine tuning.
- 1/10/100 Hz tuning steps using the radio's smooth same-band retune path.
- 10 Hz to 20 kHz waterfall.
- Wanted RF fixed at the waterfall centre.
- Centre-channel S-meter measured before AGC.
- 300 Hz total CW filter bandwidth (approximately ±150 Hz).
- Noise blanker retained before channel filtering.
- AGC retained after the narrow filter for phone listening.
- Adjustable **300-800 Hz** phone listening BFO; default 600 Hz.
- BFO changes audio pitch only and never retunes the radio.
- CW decoder works from the same centred narrow channel.
- Phone speaker outputs only the selected wanted CW channel.
- Live text-to-CW transmit via the radio-side keyer.
- TX WPM control.

## Installation

Download `app-debug.apk` to the Android phone and install it. Android may ask for permission to install apps from the browser/file-manager used to open the APK.

The Android Studio source project is not currently included in this repository. The packaged APK is provided for normal AT-DX1 use.

## USB protocol used by the app

- `%CW` — enter CW terminal mode
- `%F<Hz>` — set actual wanted RF / TX carrier
- `%W<WPM>` — set TX speed
- `%S` — request CW status
- `%Q` — exit CW terminal mode

The app does **not** send a BFO command; BFO/filter/NB/AGC/decoder operation is phone-side.

See [../docs/CW_MODE.md](../docs/CW_MODE.md) and [../docs/COMMAND_REFERENCE.md](../docs/COMMAND_REFERENCE.md).