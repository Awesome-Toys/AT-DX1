# AT-DX1 firmware

The `source/` directory contains the latest RP2350/RP2354A firmware file available when this repository snapshot was prepared:

`AT-DX1_RP2350_CW_terminal_AUTO_300-900Hz_KEYSHAPED_NOISEGUARD_2026-09-12.ino`

## Current feature set

The source includes:

- RP2350A/RP2354A-only hardware profile;
- 40/30/20/17 m control and indicators;
- 27 MHz Si5351 reference configuration;
- Q-only ADC receive audio to USB (I channel reserved);
- 48 kHz USB Audio Class 2;
- Kenwood TS-480-style CAT for FT8/FT4 host applications;
- Hamlib/WSJT-X-oriented CAT behaviour;
- audio/VOX-governed normal digital-mode TX;
- FT8 tone measurement and RF retuning;
- receive clock strategy using fixed even integer MS0 dividers by band;
- TX clock strategy with fixed 864 MHz PLLB and MS1 retuning;
- onboard CW receive DSP on core 1;
- automatic CW tone acquisition/tracking and receive-speed estimation;
- CW blank-channel noise guard;
- CW text terminal over the CDC port;
- onboard Morse transmit encoder/keyer;
- 5 ms raised-cosine CW key shaping;
- serial bench-test mode and diagnostics.

## Build environment

The sketch includes/depends on:

- Arduino framework
- Earle Philhower Arduino-Pico core with RP2350 support
- Adafruit TinyUSB
- Arduino Audio Tools
- Pico SDK hardware headers supplied through the RP2350 Arduino core

Select an **RP2350A-compatible target** corresponding to the RP2354A/QFN-60 hardware. The source intentionally rejects RP2040 and RP2350B builds.

`core1_separate_stack = true` is intentionally enabled because core 1 runs the CW DSP in addition to the main multicore work.

## Important

This is active-development radio firmware. Compile configuration, USB libraries and RP2350 core versions can affect behaviour. Test updates into a dummy load and verify frequency/spectrum before on-air use.

## Licence

Original AT-DX1 software is intended to be distributed under the MIT License, subject to any third-party portions retaining their original terms. See `../LICENSE.md` and `../THIRD_PARTY.md`.
