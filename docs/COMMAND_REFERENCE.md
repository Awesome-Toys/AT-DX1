# AT-DX1 basic command reference

AT-DX1 uses the USB CDC port in two modes:

1. normal **TS-480-style CAT mode** for FT8/FT4 and general radio control;
2. **CW terminal mode**, entered with `%CW`.

Normal CAT commands are terminated with `;`. CW terminal control commands begin with `%` and may be terminated by Enter/CR/LF or `;`.

## Normal CAT mode

### Frequency and status

| Command | Function / reply |
|---|---|
| `FA;` | Read VFO A/dial frequency. Reply is `FA` plus 11 frequency digits. |
| `FA00014074000;` | Set dial frequency to 14.074000 MHz. Successful SET is silent. Frequency must be inside a supported AT-DX1 band. |
| `FB;` / `FBxxxxxxxxxxx;` | VFO B-compatible alias; AT-DX1 uses the same dial-frequency state. |
| `IF;` | Read TS-480-style frequency/mode/TX status. |
| `ID;` | Returns `ID020;` (TS-480 identity used for compatibility). |
| `MD;` | Read receive sideband mode (`MD1;` LSB or `MD2;` USB). |
| `MD1;` | Select LSB receive. Successful SET is silent. |
| `MD2;` | Select USB receive. Successful SET is silent. |

### PTT

| Command | Function |
|---|---|
| `TX;`, `TX0;`, `TX1;`, `TX2;` | Request TX. Successful SET is silent. Normal FT8/FT4 RF still depends on valid USB transmit audio. |
| `RX;` | Return CAT PTT state to receive. Successful SET is silent. |

### I/Q calibration

| Command | Function |
|---|---|
| `IQCAL;` | Start automatic I/Q calibration. Use 14.075000 MHz, approximately -60 dBm, unmodulated generator input. |
| `IQCAL?;` | Report calibration state/result and stored coefficients. |
| `IQCAL RESET;` | Restore/save default unity calibration values. |

See [IQ_CALIBRATION.md](IQ_CALIBRATION.md).

## CW terminal mode

Enter from normal CAT mode with:

`%CW;`

The radio forces USB receive and internally uses `wanted RF - 10,005 Hz` as the physical RX LO.

| Command | Function |
|---|---|
| ordinary text | Queue text for Morse transmission using the radio-side keyer. |
| `%F14060000` | Set actual wanted RF / CW TX carrier to 14.060000 MHz. AT-DX1 applies the real band limits to this wanted RF. |
| `%F` | Report current CW status. |
| `%W20` | Set CW TX speed to 20 WPM. Valid range 5-50 WPM. |
| `%S` | Report CW status. |
| `%?` | Show CW terminal help. |
| `%Q` | Exit CW terminal and return to normal CAT/FT8 mode. |
| `%T...` | Legacy command only. Current firmware replies that BFO is app-side; it does not retune RF. |
| `%C<Hz>,<value>` | Legacy compatibility form. Frequency is accepted; second old-BFO value is ignored. Current app should use `%F<Hz>`. |

Typical CW status resembles:

`[CW F=14060000 Hz TX=20 WPM SHAPE=5ms RX=USB CENTER=10005Hz BUF=0/1023]`

The current Android app does not send a radio BFO command. Its 300-800 Hz BFO, 300 Hz CW filter, noise blanker, AGC, S-meter and decoder are phone-side.

## Bench-test mode

Bench-test commands are intended for controlled testing, preferably into a dummy load.

| Command | Function |
|---|---|
| `TEST?;` | Report current test state. |
| `TEST 10 OFF;` | Select exact 10.000000 MHz RX test; all four band LEDs on. |
| `TEST 10 ON;` | Enable fixed 10.000000 MHz test carrier. |
| `TEST 40 OFF;` / `TEST 40 ON;` | Select 7.074000 MHz RX / fixed carrier. |
| `TEST 30 OFF;` / `TEST 30 ON;` | Select 10.136000 MHz RX / fixed carrier. |
| `TEST 20 OFF;` / `TEST 20 ON;` | Select 14.074000 MHz RX / fixed carrier. |
| `TEST 17 OFF;` / `TEST 17 ON;` | Select 18.100000 MHz RX / fixed carrier. |
| `TEST ON;` / `TEST OFF;` | Enable/disable currently selected test carrier. |
| `TEST EXIT;` | Leave bench mode and restore normal receiver operation. |

## Compatibility/status queries

The firmware also accepts/responds to a number of benign TS-480 status commands used by Hamlib, including `AI`, `RM`, `SM`, `PC`, `TY`, `BY`, `AG0`, `RG`, `SQ0`, `GT`, `PA`, `RA`, `ML`, `NL`, `RL`, `LK`, `UL`, `PS`, `FR` and `FT`. Where AT-DX1 has no matching hardware meter/control, a fixed syntactically valid value is returned or the SET is harmlessly ignored.

## Diagnostics

Advanced diagnostic commands implemented by the current firmware:

- `DBG;` — diagnostic state
- `DBGC;` — counters
- `DBGE;` — error diagnostics
- `DBGW;` — watchdog diagnostics
- `DBGR;` — reset reliability diagnostics/counters

These are primarily development/service commands rather than normal operating controls.
