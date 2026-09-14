# AT-DX1 hardware notes — RP2354A V1.0

This document summarises the architecture represented by the supplied 2026-09-14 hardware files.

## Bands

- 40 m
- 30 m
- 20 m
- 17 m

## Controller and synthesis

- RP2354A (RP2350A family, QFN-60)
- 27 MHz reference used by the current firmware
- Si5351A synthesizer
- I2C on GPIO0 / GPIO1 in the current firmware

## Receive path

The receiver is a quadrature-sampling design using the SN74CBTLV3253 switching device. A 74LVC74 clock divider/phasing stage is driven from Si5351 CLK0 at 4× the wanted receive frequency to generate final-frequency quadrature clocks.

The present firmware uses Q_OUT on GPIO26 / ADC0 as the USB receive audio source. I_OUT on GPIO27 / ADC1 is present/reserved for future I/Q processing.

## Transmit path

The compact TX stage uses two SN74ACT244 devices, driven as complementary RF paths. The Si5351 CLK1 output is generated at 2× wanted RF and divided by the TX flip-flop stage to produce 0°/180° drive. A 4:8 transformer combines/couples the transmitter to the output network.

The design deliberately operates at QRPp power levels. Measure each build rather than assuming a fixed output power.

## Output filtering

The V1.0 board includes an elliptic low-pass-filter network for the 40 m to 17 m design range. Always verify harmonic attenuation on the assembled board before on-air use.

## Indicators / control

Current firmware pin use includes:

- GPIO14: 40 m LED
- GPIO13: 30 m LED
- GPIO12: 20 m LED
- GPIO11: 17 m LED
- GPIO15: TX LED
- GPIO19: TXSW
- GPIO20: RXSW

## USB

USB-C carries device connection for firmware control/CAT and USB audio. The current firmware presents itself as an AT-DX1 USB audio/CAT device.
