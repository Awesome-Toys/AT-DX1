# AT-DX1 firmware

The current firmware is:

[`AT-DX1.ino`](AT-DX1.ino)

It targets the RP2350A/RP2354A hardware used on the AT-DX1 V1.0 PCB.

## Current feature set

- 40/30/20/17 m band control and indicators.
- 27 MHz Si5351 reference configuration.
- True-I/Q receive: GPIO26/ADC0 Q_OUT + GPIO27/ADC1 I_OUT.
- ADC round-robin at 96 ksample/s total = 48 ksample/s per channel.
- 511-tap Hilbert/phasing USB/LSB receiver with interleaved-ADC timing correction.
- Built-in persistent I/Q polarity/gain/quadrature calibration.
- 48 kHz, 16-bit stereo USB Audio Class 2; demodulated mono duplicated to L/R.
- Kenwood TS-480-style CAT for FT8/FT4 hosts including Hamlib/WSJT-X.
- Same-band RX tuning without CLK0 mute or PLLA soft reset.
- FT8 tone measurement and RF retuning with safety checks.
- Radio-side CW TX queue/keyer with 5 ms raised-cosine shaping.
- CW receive LO fixed 10.005 kHz below the requested wanted RF; USB is forced in CW terminal mode.
- Android app owns CW noise blanking, 300 Hz filtering, AGC, S-meter, waterfall, BFO and Morse decode.
- Serial bench-test and diagnostic commands.

## Arduino IDE build environment

AT-DX1 uses the **Earle F. Philhower Arduino-Pico core** with RP2350 support.

Add this Boards Manager URL in Arduino IDE under **File -> Preferences -> Additional Boards Manager URLs**:

`https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`

Then install **Raspberry Pi Pico/RP2040/RP2350 by Earle F. Philhower** from Boards Manager.

The sketch also depends on:

- Adafruit TinyUSB
- Arduino Audio Tools
- Pico SDK hardware headers supplied by the Arduino-Pico core

## Exact Arduino IDE board settings

Use these settings for the AT-DX1 RP2354A PCB:

| Arduino IDE option | Selection |
|---|---|
| Board | **Generic RP2350** |
| Chip Variant | **RP2350A** |
| CPU Architecture | **ARM** |
| CPU Speed | **150 MHz** |
| Flash Size | **2MB (no FS)** |
| PSRAM Size | **0 MB** |
| PSRAM CS | **None** |
| USB Stack | **Adafruit TinyUSB** |
| Upload Method | **Default (UF2)** |

The AT-DX1 uses the **RP2354A**, which is the RP2350A/QFN-60 variant with 2 MB stacked flash. Do not select RP2040, RP2350B, RISC-V, or a Pico/Pico 2 board profile for this firmware.

The source contains compile-time checks and intentionally rejects RP2040 and RP2350B builds.

`core1_separate_stack = true` is intentionally enabled and should be left unchanged.

## Compiling and uploading

1. Open [`AT-DX1.ino`](AT-DX1.ino) in Arduino IDE.
2. Select **Generic RP2350** and apply the exact settings above.
3. Install any missing libraries if Arduino IDE reports them during compilation.
4. Click **Verify** to compile.
5. For UF2 programming, place the AT-DX1 into BOOTSEL mode so the RP2354A appears as a USB mass-storage device.
6. Use the Arduino IDE upload process or copy the generated UF2 file to the RP2354A BOOTSEL drive.

If the board is already running AT-DX1 firmware and normal USB upload is not available, BOOTSEL/UF2 programming is the most reliable recovery method.

## CW frequency semantics

The app/terminal always sends the **actual wanted RF / CW TX carrier**. For example:

`%F14060000`

means 14.060000 MHz on-air CW. The radio validates 14.060000 MHz against the supported amateur-band limits and keeps that value for transmit.

For CW receive only:

`RX LO = wanted RF - 10,005 Hz`

so the wanted RF appears at 10.005 kHz in the USB audio stream. BFO pitch is entirely app-side and does not retune the radio.

## I/Q calibration

Use an unmodulated signal generator at **14.075000 MHz, about -60 dBm** and send:

`IQCAL;`

The firmware temporarily receives at 14.074000 MHz USB, measures I and Q, solves polarity/gain/quadrature correction, stores the result in flash-backed EEPROM emulation, and restores the previous receive state.

Status:

`IQCAL?;`

Reset to default coefficients:

`IQCAL RESET;`

See [../docs/IQ_CALIBRATION.md](../docs/IQ_CALIBRATION.md).

## Commands

See [../docs/COMMAND_REFERENCE.md](../docs/COMMAND_REFERENCE.md) for the useful CAT, CW, calibration and bench commands implemented by the current sketch.

## Important

This is active-development radio firmware. Test updates into a dummy load and verify frequency, output level and spectrum before on-air use.

## Licence

Original AT-DX1 software is distributed under the MIT License, subject to third-party portions retaining their original terms. See `../LICENSE.md` and `../THIRD_PARTY.md`.