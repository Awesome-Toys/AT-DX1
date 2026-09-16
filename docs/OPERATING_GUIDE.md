# AT-DX1 operating guide

This page describes the basic current workflow. AT-DX1 remains an experimental QRPp radio; verify RF output into a 50-ohm dummy load before on-air use.

## 1. First setup

1. Flash the current [`firmware/AT-DX1.ino`](../firmware/AT-DX1.ino).
2. Connect the radio by USB.
3. Run the built-in I/Q calibration once using the procedure in [IQ_CALIBRATION.md](IQ_CALIBRATION.md).
4. Confirm the supported band LEDs and CAT frequency control work.
5. Verify TX frequency, output level and spectrum into a dummy load.

## 2. FT8 / FT4 with a PC

AT-DX1 presents:

- a 48 kHz / 16-bit USB audio device;
- a USB CDC CAT port using a TS-480-style command subset.

Typical WSJT-X concept:

- Rig: Kenwood TS-480 / compatible Hamlib profile.
- CAT port: AT-DX1 USB CDC serial port.
- Audio input: AT-DX1 USB audio.
- Audio output: AT-DX1 USB audio.
- Normal receive mode: USB for FT8/FT4.

CAT PTT alone does not intentionally create a normal FT8 carrier. The firmware also requires valid incoming digital-mode audio for the normal transmit path.

## 3. CW with the Android app

Install [`android-app/app-debug.apk`](../android-app/app-debug.apk) on the Android phone, then connect the AT-DX1 by USB.

The app sends the actual wanted RF frequency. If the display says 14.060000 MHz, then:

- wanted RF / CW TX carrier = **14.060000 MHz**;
- AT-DX1 validates that wanted RF against its supported band limits;
- CW RX physical LO = **14.049995 MHz**;
- CW receive mode = **USB**;
- the wanted 14.060000 MHz signal appears at **10.005 kHz audio**.

### Waterfall

The Android waterfall displays audio from **10 Hz to 20 kHz**. Because the wanted signal is at 10.005 kHz, the exact displayed RF span is:

`wanted RF - 9.995 kHz` through `wanted RF + 9.995 kHz`

The wanted RF remains at the fixed centre marker while the spectrum moves as the main tuning frequency changes.

### S-meter

The S-meter follows the narrow channel centred on the wanted RF, not total 20 kHz waterfall energy. Its measurement is taken before listener AGC so strong/weak signals remain distinguishable.

### Noise blanker, filter and AGC

The phone receive chain is:

`USB audio -> noise blanker -> centred channel -> 300 Hz CW filter -> decoder + AGC -> speaker`

The CW filter is approximately **300 Hz total bandwidth (±150 Hz)**. This is narrow enough to reject nearby signals while remaining practical for normal CW tuning and keying sidebands.

### BFO / listening pitch

The BFO slider is **300 to 800 Hz**, default 600 Hz. It is local to the phone.

Changing BFO:

- changes the pitch from the phone speaker;
- does not change the selected RF frequency;
- does not move the waterfall centre;
- does not retune the Si5351;
- does not change the S-meter target;
- does not change the decoder's RF target.

The phone speaker outputs only the selected narrow CW channel.

### Smooth tuning

Same-band 1 Hz, 10 Hz and 100 Hz tuning steps update the Si5351 PLLA while CLK0, RXSW, ADC, I/Q/Hilbert DSP and USB audio remain live. There is no intentional audio blanking or PLL soft reset for those steps. A short disturbance is still expected for an actual band/profile change.

## 4. CW transmit

The radio remains responsible for Morse timing. Text entered in the app is sent to the CW terminal and queued by the RP2354A. The radio keys the actual selected wanted RF and applies approximately 5 ms raised-cosine RF key shaping.

TX speed can be changed with `%W<WPM>`; valid range is 5 to 50 WPM.

## 5. Useful checks

- `FA;` — read normal CAT frequency.
- `IF;` — read CAT status.
- `IQCAL?;` — show stored I/Q calibration/status.
- `%S` in CW terminal — show CW frequency/WPM/centre/buffer status.

See [COMMAND_REFERENCE.md](COMMAND_REFERENCE.md) for the command table.