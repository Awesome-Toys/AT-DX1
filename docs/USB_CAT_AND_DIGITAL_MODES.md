# USB, CAT and FT8 / FT4 operation

The current AT-DX1 RP2350/RP2354A firmware is designed to behave like a USB-connected radio plus sound card so that established digital-mode applications can be used without an AT-DX1-specific FT8 decoder.

## USB audio

The current firmware configures:

- USB Audio Class 2
- 48 kHz sample rate
- 16-bit audio
- two host channels, with the current Q-only receive signal duplicated L/R
- bidirectional audio for receive and transmit

Current USB identity strings in the firmware are:

- Manufacturer: `AT-DX1`
- Product: `AT-DX1 RP2350 FT8 CW USB Audio CAT`
- Serial: `AT-DX1-RP2350-1`

The receive path currently digitizes `Q_OUT` on GPIO26 / ADC0. `I_OUT` on GPIO27 / ADC1 is reserved for future true-I/Q firmware.

## CAT control

The default CDC interface uses a Kenwood TS-480-style CAT command set intended to work with Hamlib-based applications including WSJT-X. The firmware implements the frequency/PTT/status transactions needed by the intended digital-mode workflow and tolerates additional polling commands used by radio-control software.

The firmware deliberately ignores unsupported CAT queries rather than returning a stale error reply that could be consumed by the next pipelined transaction.

## Normal FT8 / FT4 transmit path

During normal digital-mode operation, AT-DX1 uses both CAT state and the incoming USB transmit audio. CAT PTT alone does not create a normal FT8 carrier. Valid audio is required before the normal modulation path keys RF.

The current firmware measures the incoming digital-mode tone using positive zero crossings, derives the audio frequency, and retunes the Si5351 TX MultiSynth so the resulting RF is at dial frequency + audio tone.

Safeguards in the current firmware include:

- an FT8 audio-frequency safety window;
- noisy-crossing rejection;
- a small retune deadband;
- controlled RX/TX switching;
- continuous QSD/USB receive audio while the receive RF FET itself is disconnected during TX.

## WSJT-X setup concept

AT-DX1 is intended to be configured in WSJT-X as:

- **Rig/CAT:** a compatible Kenwood TS-480/Hamlib profile using the AT-DX1 CDC serial port;
- **Input audio:** AT-DX1 USB audio device;
- **Output audio:** AT-DX1 USB audio device.

Exact port/device names depend on the operating system.

Before first RF operation, test into a 50-ohm dummy load and verify the selected dial frequency, output level and spectrum.
