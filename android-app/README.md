# AT-DX1 Android companion app

The AT-DX1 companion app is intended to be a lightweight portable CW front end for the radio rather than moving all radio DSP into the phone.

The current design has the RP2354A perform CW receive decoding and Morse transmit timing. The Android app presents the information and sends commands/text over USB.

## Intended/current UI functions

- Connect/disconnect AT-DX1 over USB CDC
- Display connected radio/callsign information
- Optional phone monitoring of RX audio
- Signal-level display
- Display decoded CW text from the radio
- Clear/reacquire/status controls
- Live type-to-transmit CW text entry
- 40/30/20/17 m band selection
- Fine frequency tuning
- TX WPM setting
- Receive/BFO tone setting
- CW waterfall centred on the selected receive tone
- Display decoder WPM, tracked tone and quality/status information

The development screenshot is stored at:

`../docs/images/at-dx1-cw-app-screenshot.jpg`

## Source status

The Android application source files were **not included in the supplied material used to assemble this repository snapshot**. This directory is therefore a documented placeholder for the Android Studio project rather than a fabricated/invented app implementation.

When the app source is added, original AT-DX1 app code is intended to use the MIT License as described in `../LICENSE.md`.
