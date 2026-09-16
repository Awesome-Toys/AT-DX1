# Changelog

## 2026-09-16 — I/Q receiver, calibration, smooth tuning and Android CW v1.23

- Updated firmware to the current RP2354A true-I/Q build.
- Added 48 ksps/channel I/Q sampling and 511-tap Hilbert USB/LSB sideband selection.
- Added automatic persistent I/Q polarity, gain and quadrature calibration using a 14.075000 MHz ~-60 dBm bench carrier.
- Added same-band click-free PLLA retuning without CLK0 mute or PLL soft reset.
- Updated CW architecture so wanted RF is kept as the actual TX carrier and RX LO is internally wanted RF - 10.005 kHz.
- Updated Android companion source to v1.23.
- Android waterfall now spans 10 Hz to 20 kHz with wanted RF fixed at the centre.
- Added centre-channel S-meter, 300 Hz CW filter, retained noise blanker/AGC and a phone-only 300-800 Hz BFO.
- Added the Android Studio v1.23 source project under `android-app/source/`.
- Added I/Q calibration, operating and command-reference documentation.
- Corrected older repository text that described Q-only receive and radio-side CW receive decoding.

## Repository snapshot — 2026-09-14

- Added RP2354A V1.0 schematic, Gerbers and BOM.
- Added PCB render and assembled-prototype photograph.
- Added AT-DX1 logo/app-icon assets and Android CW companion-app screenshot.
- Added then-current RP2350/RP2354A firmware snapshot.
- Documented USB Audio Class + TS-480-style CAT integration for WSJT-X and compatible FT8/FT4 applications.
- Added CERN-OHL-P-2.0 licensing for original AT-DX1 hardware and MIT licensing for original AT-DX1 software.
- Added separate branding/artwork notice.
- Expanded acknowledgement of WB2CBA / TinyDX and the 74ACT244 HF PA/driver inspiration.
