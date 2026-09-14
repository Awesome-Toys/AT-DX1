# Build and test notes

## Hardware assembly

Use the schematic, BOM and Gerber package in `hardware/` as a matched V1.0 set. The hardware is experimental and should be checked carefully after assembly before RF operation.

Recommended first checks:

1. Inspect for shorts, reversed components and solder bridges before applying power.
2. Verify the 5 V / 3.3 V / local regulated rails against the schematic.
3. Confirm the RP2354A can enumerate over USB and can enter the boot/programming mode.
4. Verify the 27 MHz reference and Si5351 operation.
5. Verify RX clocks and receive switching before enabling the TX drivers.
6. Use a 50-ohm dummy load for first carrier tests.
7. Verify RF output on each supported band and check harmonic/spurious performance before connecting an antenna.

## Firmware

The current firmware source is under `firmware/source/` and targets RP2350A/RP2354A hardware only.

The source uses:

- Arduino framework / Earle Philhower Arduino-Pico core with RP2350 support;
- Adafruit TinyUSB;
- Arduino Audio Tools;
- RP2350 Pico SDK hardware headers exposed by the Arduino-Pico environment.

The source deliberately sets:

`bool core1_separate_stack = true;`

because core 1 also runs the CW receive DSP.

See `firmware/README.md` for more detail.

## RF test mode

The current firmware contains a bench test mode over the CDC serial/CAT port. It can select RX or a fixed carrier at the supported-band reference frequencies and at 10 MHz.

Use test mode only into a suitable dummy load and test equipment. Do not assume the output is spectrally compliant until it has been measured on the actual assembled unit.
