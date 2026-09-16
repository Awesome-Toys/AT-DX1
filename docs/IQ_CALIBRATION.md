# AT-DX1 I/Q calibration

AT-DX1 has a built-in automatic calibration for the two QSD baseband/ADC paths. It learns:

- physical I/Q polarity;
- relative Q-channel gain correction;
- residual quadrature/phase-mix correction;
- achieved opposite-sideband/image rejection.

The calibration is stored persistently in the RP2350's program flash using **EEPROM emulation**. There is no separate EEPROM IC on the AT-DX1 PCB.

## What you need

- AT-DX1 running the current I/Q firmware.
- RF signal generator connected to the AT-DX1 antenna input.
- Generator set to **14.075000 MHz**.
- Unmodulated carrier at approximately **-60 dBm**.
- USB serial/CAT terminal connected to AT-DX1.
- Normal radio software stopped/idle so it cannot retune or transmit during calibration.

The level is not extremely critical; approximately -60 dBm is intended to give a clean measurable tone without approaching ADC clipping.

## Calibration procedure

1. Connect the signal generator to the AT-DX1 antenna input.
2. Set it to **14.075000 MHz**, approximately **-60 dBm**, unmodulated.
3. Make sure AT-DX1 is in normal CAT/RX mode, not CW terminal mode, bench-test mode or transmit.
4. Optionally query the current values:

   `IQCAL?;`

5. Start calibration:

   `IQCAL;`

6. The radio automatically switches to **14.074000 MHz USB**, waits for the receiver to settle, then measures I and Q for approximately one second.
7. Query status if required:

   `IQCAL?;`

8. On success, the correction is stored automatically and the previous frequency/mode is restored.
9. Power-cycle the radio and send `IQCAL?;` again to confirm the stored values reload correctly.

## Example successful result

`IQCAL OK POL=- QG=0.98892 QM=+0.01250 IR=42.6 R=137.8;`

The exact values will vary from board to board.

### Result fields

| Field | Meaning |
|---|---|
| `POL` | Learned physical quadrature polarity. `+` or `-` is valid; it is not a pass/fail quality grade. |
| `QG` | Q-channel amplitude multiplier used to match I and Q. Values close to 1.0 indicate small gain mismatch. |
| `QM` | Residual quadrature/phase-mix correction applied to Q. Values close to zero indicate small residual phase error. |
| `IR` | Calculated opposite-sideband/image rejection in dB after correction. Higher is better. Firmware requires at least 15 dB before accepting/saving a new calibration. |
| `R` | Calibration signal RMS amplitude in ADC counts. Used as a signal-level sanity check. |

The calibration directly feeds the radio-side USB/LSB phasing receiver, so the same correction is used by FT8/FT4 receive and by CW receive.

## Reset calibration

To return to unity/default coefficients and save that state:

`IQCAL RESET;`

Then run `IQCAL;` again when the correct signal-generator setup is available.

## Common failure messages

| Message | Likely meaning / action |
|---|---|
| `IQCAL BUSY TX/TEST/CW;` | Leave CW/test mode and stop TX before calibration. |
| `IQCAL FAIL LOW-SIG;` | Generator level is too low, disconnected, or not reaching the receiver. |
| `IQCAL FAIL SIDE/POL;` | The expected +1 kHz calibration relationship was not coherent enough; confirm 14.075000 MHz generator and 14.074000 MHz calibration RX setup. |
| `IQCAL FAIL TUNE;` | Something retuned/changed the receiver during the measurement; close CAT applications and retry. |
| `IQCAL FAIL MATH;` | Invalid/degenerate measurement; check the generator and retry. |
| `IQCAL FAIL RANGE;` | Required correction is outside the allowed range; inspect the I/Q analogue paths. |
| `IQCAL FAIL REJECT;` | Solved image rejection is below the firmware acceptance limit. |
| `IQCAL FAIL SAVE;` | Calibration solved but could not be saved to flash-backed storage. |

## When to recalibrate

A successful calibration is normally persistent. Recalibrate after changes that can materially alter the I/Q analogue path, ADC scaling or firmware calibration model, or if measured opposite-sideband rejection has noticeably degraded.
