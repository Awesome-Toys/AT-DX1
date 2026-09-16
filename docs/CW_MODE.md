# AT-DX1 CW mode

AT-DX1 CW mode deliberately divides the work between the radio and the Android phone.

## What runs where

### AT-DX1 / RP2354A

- validates the requested wanted RF against the supported 40/30/20/17 m bands;
- keeps the requested wanted RF as the CW TX carrier;
- derives CW RX LO = wanted RF - 10.005 kHz;
- forces radio-side USB demodulation for CW;
- supplies continuous 48 kHz USB audio;
- performs Morse TX queue/key timing;
- applies approximately 5 ms raised-cosine CW key shaping.

### Android app v1.23

- 10 Hz to 20 kHz waterfall;
- wanted RF fixed at the 10.005 kHz audio centre;
- impulse noise blanker;
- centre-channel S-meter;
- 300 Hz total CW filter bandwidth;
- AGC for phone listening;
- adjustable 300-800 Hz listening BFO;
- Morse decoding;
- phone speaker monitoring of only the selected CW channel;
- band/frequency tuning and CW text terminal.

## Frequency model

If the app selects:

`14.060000 MHz`

then AT-DX1 treats 14.060000 MHz as the real wanted RF and CW TX carrier. For receive it uses:

`14.060000 MHz - 10.005 kHz = 14.049995 MHz RX LO`

With USB demodulation, the selected 14.060000 MHz carrier appears at exactly **10.005 kHz** in the USB audio.

The app displays audio from 10 Hz to 20 kHz, giving the exact RF span:

`wanted RF - 9.995 kHz` to `wanted RF + 9.995 kHz`

The selected wanted RF therefore remains fixed at the waterfall centre.

## BFO and phone audio

The BFO slider is a phone-only listening control from **300 to 800 Hz**. It digitally translates the centred CW channel to the selected pitch after the RF frequency has already been selected.

Changing the BFO does not change RF tuning, waterfall centre, S-meter target or decoder RF target.

The speaker and decoder use a **300 Hz** centred CW channel. Noise blanking is performed before that channel filter; listener AGC is applied after it.

## S-meter

The S-meter measures the centred wanted channel before AGC rather than measuring total waterfall power. A strong station elsewhere in the 20 kHz display should therefore not become the selected station's S-meter reading.

## Smooth tuning

1 Hz, 10 Hz and 100 Hz same-band steps use the firmware's fast RX PLLA retune. CLK0 remains enabled, RX remains connected, ADC/IQ/Hilbert history continues and USB audio is not intentionally blanked. This avoids the large clicks and horizontal waterfall lines caused by the earlier break-before-make tuning path.

A full rebuild/blank is still used for a real band/profile change.

## CW terminal commands

| Command | Function |
|---|---|
| `%CW` | Enter CW terminal mode from CAT mode. |
| ordinary text | Queue text for Morse TX. |
| `%F14060000` | Set actual wanted RF / CW TX carrier to 14.060000 MHz. |
| `%W20` | Set TX speed to 20 WPM. |
| `%S` or `%F` | Show current CW status. |
| `%?` | Show help. |
| `%Q` | Exit CW terminal and restore CAT/FT8 operation. |

The old `%T` radio-BFO command is no longer used; firmware explicitly reports that BFO is app-side. The old `%C<frequency>,<bfo>` form is accepted for compatibility but its second value is ignored.

See [COMMAND_REFERENCE.md](COMMAND_REFERENCE.md) for the complete basic command summary.
