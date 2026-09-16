# USB, CAT and FT8 / FT4 operation

The current AT-DX1 RP2350/RP2354A firmware behaves like a USB-connected radio plus sound card so established digital-mode applications can be used without a proprietary FT8 decoder.

## USB audio

Current format:

- USB Audio Class 2
- 48 kHz sample rate
- 16-bit samples
- stereo host format, with demodulated mono RX duplicated to L/R
- bidirectional audio for receive and transmit

Current USB identity strings:

- Manufacturer: `AT-DX1`
- Product: `AT-DX1 RP2350 FT8 CW USB Audio CAT`
- Serial: `AT-DX1-RP2350-1`

## True-I/Q receive

The current firmware samples:

- Q_OUT: GPIO26 / ADC0
- I_OUT: GPIO27 / ADC1

ADC round-robin runs at 96 ksample/s total, giving 48 ksample/s per channel. A 511-tap Hilbert/phasing receiver performs radio-side USB/LSB selection before the mono audio is sent over USB.

The I/Q calibration stored by `IQCAL` corrects physical polarity, relative channel gain and residual quadrature error.

## CAT control

The default CDC interface uses a Kenwood TS-480-style subset intended to work with Hamlib applications including WSJT-X. It implements the frequency/PTT/status transactions required by the intended digital-mode workflow and tolerates additional status polling commands.

Useful commands include:

- `FA;` / `FAxxxxxxxxxxx;`
- `IF;`
- `ID;`
- `MD;`, `MD1;`, `MD2;`
- `TX;` / `TX1;`
- `RX;`

See [COMMAND_REFERENCE.md](COMMAND_REFERENCE.md).

## Smooth receive tuning

For a same-band frequency change, the firmware keeps CLK0 enabled, RX connected and USB audio live while updating PLLA. It does not soft-reset PLLA for normal 1/10/100 Hz tuning. The fixed even integer MS0 divider is only rebuilt when the required band/profile changes.

## Normal FT8 / FT4 transmit path

During normal digital-mode operation, AT-DX1 uses both CAT state and incoming USB transmit audio. CAT PTT alone does not intentionally create a normal FT8 carrier. Valid audio is required before the normal modulation path keys RF.

The firmware measures the incoming digital-mode tone, derives the audio frequency, and retunes the TX synthesizer so resulting RF is dial frequency + audio tone.

Safeguards include:

- FT8 audio-frequency safety window;
- noisy-crossing rejection;
- small retune deadband;
- controlled RX/TX switching;
- continuous QSD/USB receive processing while the receive RF FET is disconnected during TX.

## WSJT-X setup concept

Configure approximately as:

- **Rig/CAT:** Kenwood TS-480 / compatible Hamlib profile using AT-DX1 CDC serial.
- **Input audio:** AT-DX1 USB audio device.
- **Output audio:** AT-DX1 USB audio device.
- **Receive mode:** USB for normal FT8/FT4.

Exact port/device names depend on the operating system.

Before first RF operation, test into a 50-ohm dummy load and verify dial frequency, output level and spectrum.
