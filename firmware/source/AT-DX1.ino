/*
  TinyDX FT8/FT4 + CW terminal transceiver - RP2350A/RP2354A only
  Revised 2026-09-15 RP2350-only I/Q USB-receiver + CW-terminal build:

  I/Q receive architecture update:
    - ADC0/Q_OUT and ADC1/I_OUT are sampled continuously at 48 ksps each
    - a 511-tap phasing/Hilbert sideband selector runs in the RP2350
    - USB is the default receive sideband after every boot
    - Kenwood MD1/MD2 select LSB/USB; CW terminal forces USB because its LO is carrier-BFO
    - the USB sound-card format stays 48 kHz / 16-bit / stereo with mono RX duplicated L/R
    - AT-DX1 retains CW carrier generation, TX queue/keying and 5 ms raised-cosine shaping
    - CW-specific receive DSP/decoding remains in the Android companion app
    - app performs noise blanking, AGC, 50 Hz BFO filtering, waterfall and Morse decode
    - built-in I/Q calibration uses a 14.075000 MHz ~-60 dBm test carrier with
      the receiver automatically placed at 14.074000 MHz USB
    - calibration corrects relative Q-channel gain and residual quadrature error,
      stores the coefficients in emulated EEPROM, then restores the prior RX setup

    RP2354A PCB: Schematic_FT8-FT4_RP2354A_2026-09-10(1).svg
       - 27.000 MHz TCXO reference
       - GPIO14/13/12/11 = 40/30/20/17 m LEDs
       - GPIO15 = TX LED, GPIO19 = TXSW, GPIO20 = RXSW

  This source intentionally supports only the RP2350A/QFN-60 hardware profile.

  Common radio behaviour retained from the proven build:
    - true-I/Q RX: GPIO26/ADC0 Q_OUT + GPIO27/ADC1 I_OUT
    - RP2350 converts I/Q to ordinary USB/LSB mono audio before USB Audio
    - 48 kHz USB Audio Class 2 interface; demodulated mono duplicated L/R
    - Kenwood TS-480 style CAT used by FT8CN / FT8TW / WSJT-X
    - Hamlib-compatible TS-480 CAT: successful SET commands are silent
      (TX/TX0/TX1/TX2/RX); read commands such as FA/IF/ID return answers
    - Tolerant TS-480 meter/status support (RM/SWR, SM, PC, TY, etc.)
    - Unknown CAT commands are counted and ignored rather than emitting a
      stale '?;' reply that can poison the next Hamlib transaction
    - CAT + TinyDX-style audio VOX PTT: valid USB FT8 audio remains the
      authority for actual RF; CAT PTT alone can never create a carrier
    - Persistent diagnostics plus watchdog-reset breadcrumbs available with DBG; / DBGC; / DBGE; / DBGW; / DBGR;
    - four-positive-crossing / three-cycle TX tone measurement
    - FT8 +/-50 Hz TX audio safety window
    - 0.20 Hz TX retune deadband and noisy-crossing rejection
    - Continuous QSD/USB receive audio during TX; RX RF FET remains disconnected

  Common RF clock plan:
    RECEIVE:
      Si5351 CLK0 = 4 x dial frequency -> U1 74LVC74 divide-by-4
      U1 generates final-frequency 0/90 degree LO_I / LO_Q clocks.
      CLK1 is disabled during RX.

    TRANSMIT:
      Si5351 CLK1 = 2 x (dial frequency + measured FT8 audio frequency)
      -> U9 74LVC74 divide-by-2 -> TX1 / TX2 complementary outputs.
      TX1/TX2 therefore emerge at wanted RF and 180 degrees apart.
      CLK0 remains enabled so U1 keeps the QSD quadrature clocks running and
      I/Q sampling and the receive DSP continue while the receive RF FET is disabled.

  RX spur-reduction method:
    - MS0 uses a fixed EVEN INTEGER divider for each whole amateur band.
    - MS0 integer mode is enabled.
    - Tuning within a band changes PLLA, not MS0.
    - This avoids changing MultiSynth divider topology while receiving.

  TX frequency-generation method:
    - PLLB is held at an exact integer x32 of the 27 MHz TCXO:
         27 MHz x 32 = 864 MHz
    - Real-time FT8 modulation changes only MS1.

  Safe boot state:
    TX drivers disabled, RX path disconnected, TX LED OFF and Si5351 outputs
    disabled/LOW until the appropriate clock has been configured.

  Default band/frequency: 20 m FT8, 14.074000 MHz.

  Bench test mode (USB serial / CAT CDC port):
    TEST 10 OFF -> exact 10.000000 MHz RX; all four band LEDs ON
    TEST 10 ON  -> exact 10.000000 MHz carrier; all four band LEDs ON
    TEST 40 OFF/ON -> 7.074000 MHz RX / fixed carrier
    TEST 30 OFF/ON -> 10.136000 MHz RX / fixed carrier
    TEST 20 OFF/ON -> 14.074000 MHz RX / fixed carrier
    TEST 17 OFF/ON -> 18.100000 MHz RX / fixed carrier
    TEST ON / TEST OFF -> enable/disable the currently selected test carrier
    TEST?           -> report current test state
    TEST EXIT       -> leave test mode and restore normal receiver operation

  Test mode bypasses CAT/VOX modulation deliberately. The fixed test carrier
  still uses CLK1=2*RF followed by the hardware divide-by-2 stage.

  I/Q calibration (USB serial / CAT CDC port; run with normal radio software idle):
    IQCAL;          -> start automatic calibration
                       Set signal generator to 14.075000 MHz, about -60 dBm.
                       Firmware temporarily uses 14.074000 MHz USB.
    IQCAL?;         -> report RUN / OK / FAIL and stored coefficients
    IQCAL RESET;    -> restore unity/default I/Q correction and save it
  A valid calibration is saved in RP2350 flash-backed EEPROM and survives reboot.

  CW terminal mode (USB serial / CAT CDC port):
    %CW              -> leave CAT parser and enter CW terminal
    normal text      -> queue immediately for Morse transmission
    %F14060000       -> set actual CW carrier to 14.060000 MHz
    %W20             -> set TX speed to 20 WPM (RX decode is handled by the Android app)
    %T700            -> set receive BFO offset to 700 Hz (100..7000 Hz)
    %C14060000,700    -> atomically set carrier + BFO (waterfall drag/select)
    %S / %?          -> status / help (CW RX DSP/decode is in the Android app)
    %Q               -> exit CW terminal and restore CAT/FT8 operation

  During CW receive, CLK0 is tuned (carrier - receive tone) and the I/Q DSP
  is forced to USB. Therefore %F always means the actual on-air CW carrier
  frequency, not the offset RX LO.
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <EEPROM.h>
#include "AudioTools.h"
#include "AudioTools/Communication/USB/USBAudioStream.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/structs/watchdog.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <climits>

using namespace audio_tools;

// Arduino-Pico multicore: give core 1 its own 8 KB stack instead of splitting
// the default stack between core 0 and core 1. Core 1 remains responsible for
// FT8 playback-tone tracking while ADC DMA continues independently.
bool core1_separate_stack = true;

// Cross-core memory ordering for the RP2350 multicore build.
static inline void tinyDxMemoryBarrier() {
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

// ---------------------------------------------------------------------------
// RP2350A / RP2354A hardware profile
// ---------------------------------------------------------------------------
static constexpr uint8_t Q_AUDIO_PIN = 26;    // ADC0 = Q_OUT; 0-degree QSD clock path
static constexpr uint8_t Q_ADC_CHANNEL = 0;
static constexpr uint8_t I_AUDIO_PIN = 27;    // ADC1 = I_OUT; 90-degree delayed QSD clock path
static constexpr uint8_t I_ADC_CHANNEL = 1;

static constexpr uint8_t I2C_SDA_PIN = 0;
static constexpr uint8_t I2C_SCL_PIN = 1;
static constexpr uint32_t I2C_CLOCK_HZ = 400000;

// ---------------------------------------------------------------------------
// Front-panel LED brightness
// Set DIM_LEDS to 1 to PWM the band and TX LEDs at the percentage below.
// Set DIM_LEDS to 0 to restore the original full-brightness ON/OFF behaviour.
// ---------------------------------------------------------------------------
#define DIM_LEDS 1
#define LED_BRIGHTNESS_PERCENT 12
static constexpr uint32_t LED_PWM_FREQUENCY_HZ = 20000;  // above audio range

static_assert(LED_BRIGHTNESS_PERCENT >= 0 && LED_BRIGHTNESS_PERCENT <= 100,
              "LED_BRIGHTNESS_PERCENT must be between 0 and 100");
static constexpr uint8_t LED_PWM_LEVEL = static_cast<uint8_t>(
    (255UL * LED_BRIGHTNESS_PERCENT + 50UL) / 100UL);

#if !defined(PICO_RP2350)
  #error "This AT-DX1 firmware is RP2350-only. Select an RP2350 target."
#endif

// The current AT-DX1/RP2354A PCB uses the RP2350A/QFN-60 GPIO layout.
#if !defined(PICO_RP2350A) || !(PICO_RP2350A)
  #error "AT-DX1 RP2354A requires RP2350A/QFN-60, not RP2350B."
#endif

static constexpr const char *TINYDX_HARDWARE_NAME = "RP2354A";
static constexpr uint8_t BAND_40_LED_PIN = 14;
static constexpr uint8_t BAND_30_LED_PIN = 13;
static constexpr uint8_t BAND_20_LED_PIN = 12;
static constexpr uint8_t BAND_17_LED_PIN = 11;
static constexpr uint8_t TX_LED_PIN      = 15;
static constexpr uint8_t TXSW_PIN        = 19;
static constexpr uint8_t RXSW_PIN        = 20;
static constexpr double SI5351_REFERENCE_HZ = 27000000.0;
static constexpr int32_t SI5351_CALIBRATION_PPB = 0;

// TXSW drives active-low OE inputs on the ACT244 TX drivers.
static constexpr uint8_t TXSW_DISABLED_LEVEL = HIGH;
static constexpr uint8_t TXSW_ENABLED_LEVEL  = LOW;

// CW key shaping uses the RP-series hardware PWM block on TXSW.  The carrier
// PWM is intentionally much faster than the 5 ms envelope.  TXSW is active
// LOW, therefore a compare value of TX_CONTROL_PWM_COUNTS means fully OFF
// (pin continuously HIGH), while compare 0 means fully ON (pin continuously
// LOW).  Intermediate values are used only during the rise/fall of a CW mark.
//
// TOP=124 gives 125 duty steps. The divider is computed from the RP2350
// system clock so TXSW PWM runs very close to 1 MHz.
static constexpr uint32_t TX_CONTROL_PWM_HZ = 1000000UL;
static constexpr uint16_t TX_CONTROL_PWM_WRAP = 124;
static constexpr uint16_t TX_CONTROL_PWM_COUNTS = TX_CONTROL_PWM_WRAP + 1;
static constexpr uint32_t CW_KEY_SHAPE_US = 5000UL;
static constexpr int64_t CW_ENVELOPE_TIMER_US = -100;  // 100 us = 50 steps / 5 ms
static constexpr uint8_t CW_ENVELOPE_STEPS = 50;

// Half-cosine / raised-cosine edge, precomputed as
// 0.5 - 0.5*cos(pi*x), x=0..1, scaled to 0..125.  Keeping the table integer
// avoids floating point inside the 10 kHz timer callback.
static constexpr uint8_t CW_RAISED_COSINE[CW_ENVELOPE_STEPS + 1] = {
    0, 0, 0, 1, 2, 3, 4, 6, 8, 10, 12, 14, 17, 20, 23, 26, 29,
    32, 36, 39, 43, 47, 51, 55, 59, 62, 66, 70, 74, 78, 82, 86, 89,
    93, 96, 99, 102, 105, 108, 111, 113, 115, 117, 119, 121, 122,
    123, 124, 125, 125, 125};

// RXSW drives the receive RF FET gate.
static constexpr uint8_t RXSW_DISABLED_LEVEL = LOW;
static constexpr uint8_t RXSW_ENABLED_LEVEL  = HIGH;

static constexpr uint32_t RF_SWITCH_SETTLE_US = 20;
static constexpr uint32_t RX_AUDIO_RECOVERY_US = 2000;

// ---------------------------------------------------------------------------
// USB audio and detector settings -- retained from the proven build
// ---------------------------------------------------------------------------
static constexpr uint32_t SAMPLE_RATE = 48000;
static constexpr uint8_t CHANNELS = 2;
static constexpr uint8_t BITS_PER_SAMPLE = 16;
static constexpr size_t FRAMES_PER_BLOCK = 48;  // one USB millisecond

// The RP2350 has one ADC.  Round-robin ADC0/ADC1 sampling therefore runs at
// 96 ksps total so each physical I/Q channel is sampled at 48 ksps.
static constexpr uint8_t RX_ADC_CHANNEL_COUNT = 2;
static constexpr uint32_t RX_ADC_TOTAL_SAMPLE_RATE = SAMPLE_RATE * RX_ADC_CHANNEL_COUNT;
static constexpr size_t ADC_SAMPLES_PER_BLOCK = FRAMES_PER_BLOCK * RX_ADC_CHANNEL_COUNT;

// TS-480 mode numbers used by CAT.  AT-DX1 intentionally boots in USB.
enum RxSidebandMode : uint8_t {
  RX_MODE_LSB = 1,
  RX_MODE_USB = 2
};

// Explicit prototype: prevents the Arduino sketch preprocessor from creating
// a prototype before RxSidebandMode has been declared.
void setRxSidebandMode(RxSidebandMode mode);

RxSidebandMode rxSidebandMode = RX_MODE_USB;
RxSidebandMode rxModeBeforeCw = RX_MODE_USB;

static constexpr double MIN_AUDIO_FREQUENCY_HZ = 100.0;
static constexpr double MAX_AUDIO_FREQUENCY_HZ = 3000.0;

// Standard FT8's eight tones span 43.75 Hz. This guard rejects unrelated USB
// audio or a bad crossing estimate from causing a wide RF excursion.
static constexpr double TX_AUDIO_WINDOW_HALF_WIDTH_HZ = 50.0;
static constexpr double TX_RETUNE_DEADBAND_HZ = 0.20;
static constexpr double TX_MAX_THREE_CYCLE_SPREAD_HZ = 10.0;

static constexpr int32_t VOX_SAMPLE_THRESHOLD = 300;
static constexpr int32_t ZERO_CROSS_HYSTERESIS = 80;
static constexpr uint32_t VOX_HANG_SAMPLES =
    (SAMPLE_RATE * 40UL) / 1000UL;
static constexpr uint32_t USB_PACKET_TIMEOUT_US = 100000;

// ---------------------------------------------------------------------------
// System lockup protection
// The RP2350 hardware watchdog resets the whole MCU if core 0
// stops making progress (USB stack stall, I2C stall, spinlock deadlock, etc.).
// Core 1 also publishes a heartbeat so a silent failure of the audio core can
// deliberately stop watchdog feeding and force a clean whole-chip reset.
// ---------------------------------------------------------------------------
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 4000;
static constexpr uint32_t CORE1_HEARTBEAT_TIMEOUT_MS = 2000;
static constexpr uint32_t I2C_TRANSACTION_TIMEOUT_MS = 20;

volatile uint32_t core1HeartbeatMs = 0;
uint8_t bootResetReason = 0;

// Watchdog-reset breadcrumbs. Scratch register 4 is used by the Pico SDK
// watchdog implementation, so TinyDX deliberately uses only
// scratch 0 and 1. These registers survive a watchdog reset.
static constexpr uint32_t WATCHDOG_BREADCRUMB_MAGIC = 0xA7D10000UL;
static constexpr uint32_t WATCHDOG_BREADCRUMB_MASK  = 0xFFFF0000UL;

enum WatchdogStage : uint16_t {
  WD_STAGE_NONE = 0,
  WD_STAGE_SETUP_I2C = 1,
  WD_STAGE_SETUP_USB = 2,
  WD_STAGE_LOOP_START = 10,
  WD_STAGE_USB_TASK_1 = 11,
  WD_STAGE_CAT_1 = 12,
  WD_STAGE_CLOCK_1 = 13,
  WD_STAGE_USB_AUDIO_RX = 14,
  WD_STAGE_USB_TASK_2 = 15,
  WD_STAGE_CAT_2 = 16,
  WD_STAGE_CLOCK_2 = 17,
  WD_STAGE_USB_AUDIO_TX = 18,
  WD_STAGE_USB_TASK_3 = 19,
  WD_STAGE_CAT_3 = 20,
  WD_STAGE_CLOCK_3 = 21,
  WD_STAGE_CDC_FLUSH = 22,
  WD_STAGE_YIELD = 23,
  WD_STAGE_I2C_PROBE = 30,
  WD_STAGE_I2C_REG = 31,
  WD_STAGE_I2C_BLOCK = 32,
  WD_STAGE_ADC_LOCK_1 = 40,
  WD_STAGE_ADC_LOCK_2 = 41,
  WD_STAGE_USB_READ = 50,
  WD_STAGE_USB_WRITE = 51,
  WD_STAGE_CORE1_STALE = 60
};

uint32_t previousWatchdogBreadcrumb = 0;
uint32_t previousWatchdogAux = 0;

static inline void setWatchdogBreadcrumb(WatchdogStage stage, uint32_t aux = 0) {
  watchdog_hw->scratch[0] = WATCHDOG_BREADCRUMB_MAGIC | static_cast<uint32_t>(stage);
  watchdog_hw->scratch[1] = aux;
}

const char *watchdogStageText(uint16_t stage) {
  switch (stage) {
    case WD_STAGE_SETUP_I2C: return "SETI2C";
    case WD_STAGE_SETUP_USB: return "SETUSB";
    case WD_STAGE_LOOP_START: return "LOOP";
    case WD_STAGE_USB_TASK_1: return "USB1";
    case WD_STAGE_CAT_1: return "CAT1";
    case WD_STAGE_CLOCK_1: return "CLK1";
    case WD_STAGE_USB_AUDIO_RX: return "AUDRX";
    case WD_STAGE_USB_TASK_2: return "USB2";
    case WD_STAGE_CAT_2: return "CAT2";
    case WD_STAGE_CLOCK_2: return "CLK2";
    case WD_STAGE_USB_AUDIO_TX: return "AUDTX";
    case WD_STAGE_USB_TASK_3: return "USB3";
    case WD_STAGE_CAT_3: return "CAT3";
    case WD_STAGE_CLOCK_3: return "CLK3";
    case WD_STAGE_CDC_FLUSH: return "FLUSH";
    case WD_STAGE_YIELD: return "YIELD";
    case WD_STAGE_I2C_PROBE: return "I2CP";
    case WD_STAGE_I2C_REG: return "I2CR";
    case WD_STAGE_I2C_BLOCK: return "I2CB";
    case WD_STAGE_ADC_LOCK_1: return "LOCK1";
    case WD_STAGE_ADC_LOCK_2: return "LOCK2";
    case WD_STAGE_USB_READ: return "UREAD";
    case WD_STAGE_USB_WRITE: return "UWRITE";
    case WD_STAGE_CORE1_STALE: return "C1STAL";
    default: return "NONE";
  }
}

static inline uint16_t previousWatchdogStage() {
  if ((previousWatchdogBreadcrumb & WATCHDOG_BREADCRUMB_MASK) != WATCHDOG_BREADCRUMB_MAGIC) return 0;
  return static_cast<uint16_t>(previousWatchdogBreadcrumb & 0xFFFFu);
}

// ---------------------------------------------------------------------------
// Supported bands and fixed RX integer MultiSynth dividers
// ---------------------------------------------------------------------------
struct BandPlan {
  uint32_t minHz;
  uint32_t maxHz;
  uint32_t ft8Hz;
  uint8_t ledPin;
  uint8_t rxMs0Divider;
  const char *name;
};

// CLK0 = 4*F.  These fixed even dividers keep PLLA between 600 and 900 MHz
// throughout each entire amateur band:
//   40 m: MS0=30 -> PLLA 840.0..876.0 MHz
//   30 m: MS0=20 -> PLLA 808.0..812.0 MHz
//   20 m: MS0=14 -> PLLA 784.0..803.6 MHz
//   17 m: MS0=12 -> PLLA 867.264..872.064 MHz
static constexpr BandPlan BAND_PLANS[] = {
    { 7000000UL,  7300000UL,  7074000UL, BAND_40_LED_PIN, 30, "40m"},
    {10100000UL, 10150000UL, 10136000UL, BAND_30_LED_PIN, 20, "30m"},
    {14000000UL, 14350000UL, 14074000UL, BAND_20_LED_PIN, 14, "20m"},
    {18068000UL, 18168000UL, 18100000UL, BAND_17_LED_PIN, 12, "17m"},
};
static constexpr size_t BAND_COUNT = sizeof(BAND_PLANS) / sizeof(BAND_PLANS[0]);
static constexpr uint8_t DEFAULT_BAND_INDEX = 2;  // 20 m
static constexpr uint32_t DEFAULT_DIAL_FREQUENCY_HZ = 14074000UL;

int bandIndexForFrequency(uint32_t frequencyHz) {
  for (size_t i = 0; i < BAND_COUNT; ++i) {
    if (frequencyHz >= BAND_PLANS[i].minHz &&
        frequencyHz <= BAND_PLANS[i].maxHz) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

static constexpr bool CAT_DEBUG_REPORTS = false;
uint32_t dialFrequencyHz = DEFAULT_DIAL_FREQUENCY_HZ;
uint32_t dialFrequencyVersion = 1;
bool catTxRequested = false;

// ---------------------------------------------------------------------------
// Reliability diagnostics
// These counters are intentionally persistent until reset/power-cycle so an
// intermittent failure can be inspected after the FT8 program has been closed.
// Query with DBG; (state/reasons), DBGC; (CAT counters), DBGE; (errors), DBGW; (watchdog/reset).
// ---------------------------------------------------------------------------
enum VoxStopReason : uint8_t {
  VOX_STOP_NONE = 0,
  VOX_STOP_SILENCE,
  VOX_STOP_USB_TIMEOUT,
  VOX_STOP_STREAM_ENDED
};

enum RfStopReason : uint8_t {
  RF_STOP_NONE = 0,
  RF_STOP_REQUEST_ENDED,
  RF_STOP_AUDIO_INVALID,
  RF_STOP_TX_CONFIG_FAIL,
  RF_STOP_CLK_ENABLE_FAIL,
  RF_STOP_FAST_RETUNE_FAIL,
  RF_STOP_TEST_MODE
};

volatile VoxStopReason lastVoxStopReason = VOX_STOP_NONE;
volatile RfStopReason lastRfStopReason = RF_STOP_NONE;

volatile uint32_t catCommandCount = 0;
volatile uint32_t catUnknownCount = 0;
volatile uint32_t catMeterQueryCount = 0;
volatile uint32_t catTxCommandCount = 0;
volatile uint32_t catRxCommandCount = 0;
volatile uint32_t catShortWriteCount = 0;
volatile uint32_t i2cFailureCount = 0;
volatile uint32_t rfStopCount = 0;
volatile uint32_t audioInvalidRfStopCount = 0;
volatile uint32_t txClockFailureCount = 0;
volatile uint32_t fastRetuneFailureCount = 0;

uint8_t catSelectedMeter = 1;  // TS-480 RM1 = SWR

// ---------------------------------------------------------------------------
// Bench-test state. This is intentionally separate from CAT/VOX state so a
// fixed carrier can never be mistaken for normal FT8 modulation.
// ---------------------------------------------------------------------------
static constexpr double TEST_10MHZ_RF_HZ = 10000000.0;
bool benchTestModeActive = false;
bool benchTestCarrierRequested = false;
double benchTestRfFrequencyHz = TEST_10MHZ_RF_HZ;
int8_t benchTestBandIndex = -1;  // -1 = special 10 MHz test; 0..3 = BAND_PLANS
uint32_t benchTestRequestVersion = 1;

// ---------------------------------------------------------------------------
// Si5351 configuration
// ---------------------------------------------------------------------------
static constexpr uint8_t SI5351_ADDRESS = 0x60;

// SI5351_REFERENCE_HZ and calibration are selected above with the hardware
// profile: 27 MHz on the RP2354A PCB.
static constexpr double SI5351_PLL_MIN_HZ = 600000000.0;
static constexpr double SI5351_PLL_MAX_HZ = 900000000.0;

// Exact integer PLLB on the RP2354A board: 27 MHz * 32 = 864 MHz.
// Real-time FT8 modulation therefore changes only MS1, never PLLB.
static constexpr double SI5351_TX_PLL_HZ = SI5351_REFERENCE_HZ * 32.0;

// Each Si5351 output now drives only one 74LVC74 clock input. A lower RX drive
// reduces edge-current noise; TX uses 4 mA for comfortable margin at 2*RF.
static constexpr uint8_t SI5351_RX_DRIVE_MA = 2;
static constexpr uint8_t SI5351_TX_DRIVE_MA = 4;

// Register 183. Keep the existing 8 pF setting used on both schematics.
// 0x92 = XTAL_CL 8 pF with the required reserved-bit pattern.
static constexpr uint8_t SI5351_CRYSTAL_LOAD_REGISTER = 0x92;

static constexpr uint8_t SI5351_REG_OUTPUT_ENABLE = 3;
static constexpr uint8_t SI5351_REG_CLK0_CONTROL = 16;
static constexpr uint8_t SI5351_REG_CLK1_CONTROL = 17;
static constexpr uint8_t SI5351_REG_CLK2_CONTROL = 18;
static constexpr uint8_t SI5351_REG_CLK_DISABLE_STATE = 24;
static constexpr uint8_t SI5351_REG_PLLA = 26;
static constexpr uint8_t SI5351_REG_PLLB = 34;
static constexpr uint8_t SI5351_REG_MS0 = 42;
static constexpr uint8_t SI5351_REG_MS1 = 50;
static constexpr uint8_t SI5351_REG_PLL_RESET = 177;
static constexpr uint8_t SI5351_REG_CRYSTAL_LOAD = 183;

bool si5351Present = false;
bool si5351Ready = false;
bool rxClockEnabled = false;
bool txClockEnabled = false;
bool rxClockConfigured = false;
bool txClockConfigured = false;
bool rxRetunePending = true;
uint8_t currentRxBandIndex = 0xFF;
uint8_t currentRxMs0Divider = 0;
uint8_t si5351OutputEnableShadow = 0xFF;

double currentAudioFrequencyHz = 0.0;
double currentTxFrequencyHz = 0.0;       // actual RF after U9 divide-by-2
double appliedTxFrequencyHz = 0.0;       // actual RF after U9 divide-by-2
bool txFrequencyReady = false;

// ---------------------------------------------------------------------------
// Shared core-0/core-1 state
// ---------------------------------------------------------------------------
volatile bool usbPlaybackStreaming = false;
volatile uint32_t lastUsbPacketMicros = 0;
volatile bool requestedVoxTx = false;
volatile int32_t requestedAudioMilliHz = 0;
volatile uint32_t frequencyRequestVersion = 0;
volatile uint32_t trackerOverruns = 0;
volatile bool trackerCoreReady = false;

double txAudioWindowReferenceHz = 0.0;
bool txAudioWindowLocked = false;
int32_t lastPublishedAudioMilliHz = 0;

// Receiver USB audio is blanked only around receiver-clock reconfiguration and
// initial analogue recovery. It deliberately remains live throughout TX.
volatile bool rxAudioBlankingActive = true;
volatile uint32_t rxAudioUnblankAtUs = 0;

// ---------------------------------------------------------------------------
// CW terminal mode
//
// The USB CDC port remains TS-480 CAT by default.  The CAT command "%CW"
// enters a raw CW terminal mode.  In that mode ordinary printable characters
// are queued immediately for transmission, while lines beginning with '%'
// are local control commands.
//
// CW dial semantics:
//   dialFrequencyHz = actual CW carrier frequency.
//   RX CLK0 is tuned cwRxToneHz below the carrier (default 700 Hz).
//   CW receive audio is sent unprocessed over USB; the Android app performs
//   BFO filtering, noise blanking, AGC, waterfall processing and CW decode.
// ---------------------------------------------------------------------------
static constexpr uint16_t CW_DEFAULT_WPM = 20;
static constexpr uint16_t CW_MIN_WPM = 5;
static constexpr uint16_t CW_MAX_WPM = 50;
static constexpr uint16_t CW_DEFAULT_RX_TONE_HZ = 700;
static constexpr uint16_t CW_MIN_RX_TONE_HZ = 100;
static constexpr uint16_t CW_MAX_RX_TONE_HZ = 7000;

volatile bool cwTerminalMode = false;
volatile bool cwReceiveRfActive = false;
uint16_t cwWpm = CW_DEFAULT_WPM;
volatile uint16_t cwRxToneHz = CW_DEFAULT_RX_TONE_HZ;
uint32_t cwRfRequestVersion = 1;

struct CwSymbol {
  char character;
  const char *pattern;
};

static constexpr CwSymbol CW_SYMBOLS[] = {
    {'A', ".-"},     {'B', "-..."},   {'C', "-.-."},   {'D', "-.."},
    {'E', "."},      {'F', "..-."},   {'G', "--."},    {'H', "...."},
    {'I', ".."},     {'J', ".---"},   {'K', "-.-"},    {'L', ".-.."},
    {'M', "--"},     {'N', "-."},     {'O', "---"},    {'P', ".--."},
    {'Q', "--.-"},   {'R', ".-."},    {'S', "..."},    {'T', "-"},
    {'U', "..-"},    {'V', "...-"},   {'W', ".--"},    {'X', "-..-"},
    {'Y', "-.--"},   {'Z', "--.."},
    {'0', "-----"},  {'1', ".----"},  {'2', "..---"},  {'3', "...--"},
    {'4', "....-"},  {'5', "....."},  {'6', "-...."},  {'7', "--..."},
    {'8', "---.."},  {'9', "----."},
    {'.', ".-.-.-"}, {',', "--..--"}, {'?', "..--.."}, {'/', "-..-."},
    {'=', "-...-"},  {'+', ".-.-."},  {'-', "-....-"}, {'@', ".--.-."},
    {'\'', ".----."}, {'(', "-.--."}, {')', "-.--.-"}, {':', "---..."},
    {';', "-.-.-."}, {'!', "-.-.--"}, {'&', ".-..."}
};
static constexpr size_t CW_SYMBOL_COUNT =
    sizeof(CW_SYMBOLS) / sizeof(CW_SYMBOLS[0]);

const char *cwPatternForCharacter(char c) {
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  for (size_t i = 0; i < CW_SYMBOL_COUNT; ++i) {
    if (CW_SYMBOLS[i].character == c) return CW_SYMBOLS[i].pattern;
  }
  return nullptr;
}


static constexpr uint16_t CW_TX_QUEUE_SIZE = 1024;
static constexpr uint16_t CW_TX_QUEUE_MASK = CW_TX_QUEUE_SIZE - 1;
static_assert((CW_TX_QUEUE_SIZE & CW_TX_QUEUE_MASK) == 0,
              "CW_TX_QUEUE_SIZE must be a power of two");

char cwTxQueue[CW_TX_QUEUE_SIZE];
uint16_t cwTxHead = 0;
uint16_t cwTxTail = 0;

bool cwTxQueueEmpty() {
  return cwTxHead == cwTxTail;
}

bool cwTxQueuePush(char c) {
  const uint16_t next = static_cast<uint16_t>((cwTxHead + 1u) & CW_TX_QUEUE_MASK);
  if (next == cwTxTail) return false;
  cwTxQueue[cwTxHead] = c;
  cwTxHead = next;
  return true;
}

bool cwTxQueuePeek(char &c) {
  if (cwTxQueueEmpty()) return false;
  c = cwTxQueue[cwTxTail];
  return true;
}

bool cwTxQueuePop(char &c) {
  if (!cwTxQueuePeek(c)) return false;
  cwTxTail = static_cast<uint16_t>((cwTxTail + 1u) & CW_TX_QUEUE_MASK);
  return true;
}

bool cwTxQueueBackspace() {
  if (cwTxQueueEmpty()) return false;
  cwTxHead = static_cast<uint16_t>((cwTxHead - 1u) & CW_TX_QUEUE_MASK);
  return true;
}

enum CwKeyPhase : uint8_t {
  CW_KEY_IDLE = 0,
  CW_KEY_MARK,
  CW_KEY_ELEMENT_GAP
};

CwKeyPhase cwKeyPhase = CW_KEY_IDLE;
const char *cwCurrentPattern = nullptr;
uint8_t cwCurrentElement = 0;
bool cwKeyDown = false;

// Dedicated mark timestamps consumed by the hardware-PWM envelope timer.
// The timer turns TXSW on gently during the first 5 ms and off gently during
// the last 5 ms, so the shaping time is INSIDE the normal dit/dah duration.
volatile bool cwEnvelopeMarkActive = false;
volatile uint32_t cwEnvelopeMarkStartUs = 0;
volatile uint32_t cwEnvelopeMarkEndUs = 0;
volatile bool cwTxEnvelopeHardwareReady = false;

uint32_t cwKeyDeadlineUs = 0;
uint32_t cwLastMarkEndUs = 0;
uint32_t cwNextCharacterUs = 0;
uint32_t cwSessionHoldUntilUs = 0;
bool cwHaveSentMark = false;

static inline bool cwTimeReached(uint32_t nowUs, uint32_t deadlineUs) {
  return static_cast<int32_t>(nowUs - deadlineUs) >= 0;
}

uint32_t cwDotUs() {
  return 1200000UL / static_cast<uint32_t>(cwWpm);
}

void resetCwKeyer(bool clearQueue = true) {
  cwKeyPhase = CW_KEY_IDLE;
  cwCurrentPattern = nullptr;
  cwCurrentElement = 0;
  cwKeyDown = false;
  cwEnvelopeMarkActive = false;
  cwEnvelopeMarkStartUs = 0;
  cwEnvelopeMarkEndUs = 0;
  cwKeyDeadlineUs = 0;
  cwLastMarkEndUs = 0;
  cwNextCharacterUs = 0;
  cwSessionHoldUntilUs = 0;
  cwHaveSentMark = false;
  if (clearQueue) {
    cwTxHead = 0;
    cwTxTail = 0;
  }
}

bool cwKeyerSessionRequested() {
  if (!cwTerminalMode) return false;
  const uint32_t nowUs = micros();
  if (cwKeyPhase != CW_KEY_IDLE || !cwTxQueueEmpty()) return true;
  return cwHaveSentMark && !cwTimeReached(nowUs, cwSessionHoldUntilUs);
}

void serviceCwKeyer() {
  if (!cwTerminalMode) return;

  uint32_t nowUs = micros();
  const uint32_t dotUs = cwDotUs();

  // Bound the number of state transitions per call.  This prevents a long
  // delayed loop pass from monopolising core 0 while still catching up cleanly.
  for (uint8_t transition = 0; transition < 6; ++transition) {
    if (cwKeyPhase == CW_KEY_MARK) {
      if (!cwTimeReached(nowUs, cwKeyDeadlineUs)) return;

      const uint32_t markEndUs = cwKeyDeadlineUs;
      cwKeyDown = false;
      cwEnvelopeMarkActive = false;
      cwLastMarkEndUs = markEndUs;
      cwHaveSentMark = true;

      ++cwCurrentElement;
      if (cwCurrentPattern != nullptr &&
          cwCurrentPattern[cwCurrentElement] != '\0') {
        cwKeyPhase = CW_KEY_ELEMENT_GAP;
        cwKeyDeadlineUs = markEndUs + dotUs;
        cwSessionHoldUntilUs = cwKeyDeadlineUs;
      } else {
        cwKeyPhase = CW_KEY_IDLE;
        cwCurrentPattern = nullptr;
        cwCurrentElement = 0;
        cwNextCharacterUs = markEndUs + 3UL * dotUs;
        cwSessionHoldUntilUs = cwNextCharacterUs;
      }
      nowUs = micros();
      continue;
    }

    if (cwKeyPhase == CW_KEY_ELEMENT_GAP) {
      if (!cwTimeReached(nowUs, cwKeyDeadlineUs)) return;
      if (!cwTxEnvelopeHardwareReady) return;

      const uint32_t markStartUs = cwKeyDeadlineUs;
      const char element = cwCurrentPattern[cwCurrentElement];
      const uint32_t markUnits = (element == '-') ? 3UL : 1UL;
      const uint32_t markEndUs = markStartUs + markUnits * dotUs;
      cwEnvelopeMarkStartUs = markStartUs;
      cwEnvelopeMarkEndUs = markEndUs;
      tinyDxMemoryBarrier();
      cwEnvelopeMarkActive = true;
      cwKeyDown = true;
      cwKeyPhase = CW_KEY_MARK;
      cwKeyDeadlineUs = markEndUs;
      cwSessionHoldUntilUs = cwKeyDeadlineUs;
      nowUs = micros();
      continue;
    }

    // IDLE: consume spaces, unsupported characters, or start the next symbol.
    char queued = '\0';
    if (!cwTxQueuePeek(queued)) return;

    if (queued == ' ') {
      cwTxQueuePop(queued);
      if (cwHaveSentMark) {
        cwNextCharacterUs = cwLastMarkEndUs + 7UL * dotUs;
        cwSessionHoldUntilUs = cwNextCharacterUs;
      }
      continue;
    }

    const char *pattern = cwPatternForCharacter(queued);
    if (pattern == nullptr) {
      cwTxQueuePop(queued);  // silently discard unsupported terminal characters
      continue;
    }

    if (cwHaveSentMark && !cwTimeReached(nowUs, cwNextCharacterUs)) return;
    if (!cwTxEnvelopeHardwareReady) return;

    cwTxQueuePop(queued);
    cwCurrentPattern = pattern;
    cwCurrentElement = 0;
    const uint32_t markUnits = (pattern[0] == '-') ? 3UL : 1UL;
    const uint32_t markEndUs = nowUs + markUnits * dotUs;
    cwEnvelopeMarkStartUs = nowUs;
    cwEnvelopeMarkEndUs = markEndUs;
    tinyDxMemoryBarrier();
    cwEnvelopeMarkActive = true;
    cwKeyDown = true;
    cwKeyPhase = CW_KEY_MARK;
    cwKeyDeadlineUs = markEndUs;
    cwSessionHoldUntilUs = cwKeyDeadlineUs;
    return;
  }
}

uint32_t cwReceiveLoFrequencyHz() {
  if (dialFrequencyHz <= cwRxToneHz) return 0;
  return dialFrequencyHz - static_cast<uint32_t>(cwRxToneHz);
}

// Raw CW terminal output.  The normal main-loop CDC flush handles delivery.
void cwTerminalWriteRaw(const char *text);

// ---------------------------------------------------------------------------
// CW receive DSP is intentionally host-side.
//
// In CW terminal mode the RP2350 continues to stream ordinary USB-demodulated
// mono audio over USB Audio Class 2.  Generic I/Q sideband selection is radio-
// side; the Android companion app still owns the CW noise blanker, AGC, fixed
// 50 Hz BFO filter, waterfall and Morse decoder.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Front-panel / RF switching helpers
// ---------------------------------------------------------------------------
uint txSwPwmSlice = 0;
uint txSwPwmChannel = 0;
uint txLedPwmSlice = 0;
uint txLedPwmChannel = 0;
bool txPwmSharedSlice = false;
bool txPwmHardwareConfigured = false;
volatile uint16_t txSwHighLevelShadow = TX_CONTROL_PWM_COUNTS;
volatile uint16_t txLedLevelShadow = 0;
repeating_timer_t cwEnvelopeTimer;
bool cwEnvelopeTimerStarted = false;

static inline void writeTxPwmLevelsUnsafe(uint16_t txSwHighLevel,
                                          uint16_t txLedLevel) {
  if (!txPwmHardwareConfigured) return;

  if (txPwmSharedSlice) {
    uint16_t levelA = 0;
    uint16_t levelB = 0;
    if (txSwPwmChannel == PWM_CHAN_A) levelA = txSwHighLevel;
    else levelB = txSwHighLevel;
    if (txLedPwmChannel == PWM_CHAN_A) levelA = txLedLevel;
    else levelB = txLedLevel;
    pwm_set_both_levels(txSwPwmSlice, levelA, levelB);
  } else {
    pwm_set_chan_level(txSwPwmSlice, txSwPwmChannel, txSwHighLevel);
    pwm_set_chan_level(txLedPwmSlice, txLedPwmChannel, txLedLevel);
  }
}

static inline void setCwTxEnvelopeLevelFromTimer(uint16_t rfEnableLevel) {
  if (rfEnableLevel > TX_CONTROL_PWM_COUNTS) {
    rfEnableLevel = TX_CONTROL_PWM_COUNTS;
  }
  // Active-low OE: more RF enable means less HIGH time on TXSW.
  txSwHighLevelShadow =
      static_cast<uint16_t>(TX_CONTROL_PWM_COUNTS - rfEnableLevel);
  writeTxPwmLevelsUnsafe(txSwHighLevelShadow, txLedLevelShadow);
}

bool cwEnvelopeTimerCallback(repeating_timer_t *) {
  // Never touch TXSW from this ISR outside an armed CW TX session.  FT8 and
  // bench-test modes retain their ordinary static full-ON/full-OFF behaviour.
  if (!cwTerminalMode || !cwTxEnvelopeHardwareReady) return true;

  uint16_t rfEnableLevel = 0;
  const bool markActive = cwEnvelopeMarkActive;
  const uint32_t markStartUs = cwEnvelopeMarkStartUs;
  const uint32_t markEndUs = cwEnvelopeMarkEndUs;
  tinyDxMemoryBarrier();

  if (markActive) {
    const uint32_t nowUs = time_us_32();
    if (static_cast<int32_t>(nowUs - markStartUs) >= 0 &&
        static_cast<int32_t>(markEndUs - nowUs) > 0) {
      const uint32_t elapsedUs = nowUs - markStartUs;
      const uint32_t remainingUs = markEndUs - nowUs;

      if (elapsedUs < CW_KEY_SHAPE_US) {
        uint32_t index =
            (elapsedUs * CW_ENVELOPE_STEPS) / CW_KEY_SHAPE_US;
        if (index > CW_ENVELOPE_STEPS) index = CW_ENVELOPE_STEPS;
        rfEnableLevel = CW_RAISED_COSINE[index];
      } else if (remainingUs < CW_KEY_SHAPE_US) {
        uint32_t index =
            (remainingUs * CW_ENVELOPE_STEPS) / CW_KEY_SHAPE_US;
        if (index > CW_ENVELOPE_STEPS) index = CW_ENVELOPE_STEPS;
        rfEnableLevel = CW_RAISED_COSINE[index];
      } else {
        rfEnableLevel = TX_CONTROL_PWM_COUNTS;
      }
    }
  }

  setCwTxEnvelopeLevelFromTimer(rfEnableLevel);
  return true;
}

void beginTxPwmHardware() {
  // Hold safe levels with SIO until the PWM slices have been fully configured.
  pinMode(TXSW_PIN, OUTPUT);
  digitalWrite(TXSW_PIN, TXSW_DISABLED_LEVEL);
  pinMode(TX_LED_PIN, OUTPUT);
  digitalWrite(TX_LED_PIN, LOW);

  txSwPwmSlice = pwm_gpio_to_slice_num(TXSW_PIN);
  txSwPwmChannel = pwm_gpio_to_channel(TXSW_PIN);
  txLedPwmSlice = pwm_gpio_to_slice_num(TX_LED_PIN);
  txLedPwmChannel = pwm_gpio_to_channel(TX_LED_PIN);
  txPwmSharedSlice = (txSwPwmSlice == txLedPwmSlice);

  const float divider = static_cast<float>(clock_get_hz(clk_sys)) /
      (static_cast<float>(TX_CONTROL_PWM_HZ) *
       static_cast<float>(TX_CONTROL_PWM_COUNTS));

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_wrap(&cfg, TX_CONTROL_PWM_WRAP);
  pwm_config_set_clkdiv(&cfg, divider < 1.0f ? 1.0f : divider);

  pwm_init(txSwPwmSlice, &cfg, false);
  if (!txPwmSharedSlice) pwm_init(txLedPwmSlice, &cfg, false);

  txSwHighLevelShadow = TX_CONTROL_PWM_COUNTS;  // TXSW continuously HIGH = OFF
  txLedLevelShadow = 0;
  txPwmHardwareConfigured = true;
  writeTxPwmLevelsUnsafe(txSwHighLevelShadow, txLedLevelShadow);

  gpio_set_function(TXSW_PIN, GPIO_FUNC_PWM);
  gpio_set_function(TX_LED_PIN, GPIO_FUNC_PWM);

  pwm_set_enabled(txSwPwmSlice, true);
  if (!txPwmSharedSlice) pwm_set_enabled(txLedPwmSlice, true);

  cwEnvelopeTimerStarted = add_repeating_timer_us(
      CW_ENVELOPE_TIMER_US, cwEnvelopeTimerCallback, nullptr, &cwEnvelopeTimer);
}

void setTxDriversEnabled(bool enabled) {
  // Static full-ON/full-OFF is used by FT8 and test mode.  Because compare=0
  // or compare=TOP+1 produces a constant logic level, there is NO 1 MHz PWM
  // on TXSW except during the 5 ms CW shaping edges.
  const uint32_t irqState = save_and_disable_interrupts();
  txSwHighLevelShadow = enabled ? 0 : TX_CONTROL_PWM_COUNTS;
  writeTxPwmLevelsUnsafe(txSwHighLevelShadow, txLedLevelShadow);
  restore_interrupts(irqState);
}

void setRxPathEnabled(bool enabled) {
  digitalWrite(RXSW_PIN, enabled ? RXSW_ENABLED_LEVEL : RXSW_DISABLED_LEVEL);
}

void setPanelLed(uint8_t pin, bool enabled) {
#if DIM_LEDS
  // Arduino-Pico analogWrite() is retained for the BAND LEDs only.
  analogWrite(pin, enabled ? LED_PWM_LEVEL : 0);
#else
  digitalWrite(pin, enabled ? HIGH : LOW);
#endif
}

void setTxLed(bool enabled) {
  const uint16_t onLevel =
#if DIM_LEDS
      static_cast<uint16_t>((TX_CONTROL_PWM_COUNTS *
                             LED_BRIGHTNESS_PERCENT + 50UL) / 100UL);
#else
      TX_CONTROL_PWM_COUNTS;
#endif
  const uint32_t irqState = save_and_disable_interrupts();
  txLedLevelShadow = enabled ? onLevel : 0;
  writeTxPwmLevelsUnsafe(txSwHighLevelShadow, txLedLevelShadow);
  restore_interrupts(irqState);
}

void applyBandOutputs(uint32_t frequencyHz) {
  const int selected = bandIndexForFrequency(frequencyHz);
  for (size_t i = 0; i < BAND_COUNT; ++i) {
    setPanelLed(BAND_PLANS[i].ledPin,
                static_cast<int>(i) == selected);
  }
}

void applyBenchTestLedOutputs() {
  if (!benchTestModeActive) {
    applyBandOutputs(dialFrequencyHz);
    return;
  }

  // The exact 10 MHz calibration/test mode lights ALL band LEDs so it is
  // unmistakable and cannot be confused with a normal amateur-band setting.
  if (benchTestBandIndex < 0) {
    setPanelLed(BAND_40_LED_PIN, true);
    setPanelLed(BAND_30_LED_PIN, true);
    setPanelLed(BAND_20_LED_PIN, true);
    setPanelLed(BAND_17_LED_PIN, true);
    return;
  }

  // A band-specific carrier test lights only the selected band LED.
  for (size_t i = 0; i < BAND_COUNT; ++i) {
    setPanelLed(BAND_PLANS[i].ledPin,
                static_cast<int>(i) == benchTestBandIndex);
  }
}

void beginRfAndLedHardware() {
  pinMode(BAND_40_LED_PIN, OUTPUT);
  pinMode(BAND_30_LED_PIN, OUTPUT);
  pinMode(BAND_20_LED_PIN, OUTPUT);
  pinMode(BAND_17_LED_PIN, OUTPUT);
  pinMode(RXSW_PIN, OUTPUT);

#if DIM_LEDS
  // Use a fixed 8-bit PWM range so LED_BRIGHTNESS_PERCENT maps predictably.
  // 20 kHz keeps the LED PWM above the normal audio range.
  analogWriteFreq(LED_PWM_FREQUENCY_HZ);
  analogWriteRange(255);
#endif

  // TXSW and TX LED use direct Pico-SDK PWM so CW can have a controlled RF
  // envelope without Arduino analogWrite() reprogramming their PWM slice.
  beginTxPwmHardware();

  // Safe hardware state before the Si5351 is touched.
  setTxDriversEnabled(false);
  setRxPathEnabled(false);
  setTxLed(false);
  applyBandOutputs(DEFAULT_DIAL_FREQUENCY_HZ);  // 20 m LED ON at boot

  rxAudioBlankingActive = true;
  rxAudioUnblankAtUs = 0;
}

// ---------------------------------------------------------------------------
// Low-level I2C and Si5351 helpers
// ---------------------------------------------------------------------------
bool i2cDevicePresent(uint8_t address) {
  setWatchdogBreadcrumb(WD_STAGE_I2C_PROBE, static_cast<uint32_t>(address));
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value) {
  const uint32_t aux = (static_cast<uint32_t>(address) << 24) |
                       (static_cast<uint32_t>(reg) << 16) |
                       static_cast<uint32_t>(value);
  setWatchdogBreadcrumb(WD_STAGE_I2C_REG, aux);
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  const bool ok = Wire.endTransmission() == 0;
  if (!ok) ++i2cFailureCount;
  return ok;
}

bool i2cWriteBlock(uint8_t address, uint8_t startRegister,
                   const uint8_t *data, size_t length) {
  const uint32_t aux = (static_cast<uint32_t>(address) << 24) |
                       (static_cast<uint32_t>(startRegister) << 16) |
                       static_cast<uint32_t>(length & 0xFFFFu);
  setWatchdogBreadcrumb(WD_STAGE_I2C_BLOCK, aux);
  Wire.beginTransmission(address);
  Wire.write(startRegister);
  Wire.write(data, length);
  const bool ok = Wire.endTransmission() == 0;
  if (!ok) ++i2cFailureCount;
  return ok;
}

uint8_t si5351DriveCode(uint8_t driveMa) {
  if (driveMa >= 8) return 3;
  if (driveMa >= 6) return 2;
  if (driveMa >= 4) return 1;
  return 0;
}

double correctedReferenceHz() {
  return SI5351_REFERENCE_HZ *
      (1.0 + static_cast<double>(SI5351_CALIBRATION_PPB) * 1.0e-9);
}

// Converts a + b/c divider into the Si5351 P1/P2/P3 representation.
void si5351CalculateParams(double ratio,
                           uint32_t &p1, uint32_t &p2, uint32_t &p3) {
  uint32_t a = static_cast<uint32_t>(floor(ratio));
  const double fractional = ratio - static_cast<double>(a);
  const uint32_t maximumDenominator = 1048575UL;

  uint32_t b = static_cast<uint32_t>(
      llround(fractional * static_cast<double>(maximumDenominator)));
  uint32_t c = maximumDenominator;

  if (b >= c) {
    ++a;
    b = 0;
  }
  if (b == 0) c = 1;

  const uint32_t floor128bc = static_cast<uint32_t>(
      (static_cast<uint64_t>(128) * b) / c);

  p1 = 128UL * a + floor128bc - 512UL;
  p2 = 128UL * b - c * floor128bc;
  p3 = c;
}

bool si5351WriteSynth(uint8_t baseRegister,
                      uint32_t p1, uint32_t p2, uint32_t p3,
                      uint8_t rDividerCode = 0) {
  uint8_t data[8];
  data[0] = static_cast<uint8_t>((p3 >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(p3 & 0xFF);
  data[2] = static_cast<uint8_t>(((rDividerCode & 0x07) << 4) |
                                 ((p1 >> 16) & 0x03));
  data[3] = static_cast<uint8_t>((p1 >> 8) & 0xFF);
  data[4] = static_cast<uint8_t>(p1 & 0xFF);
  data[5] = static_cast<uint8_t>(((p3 >> 12) & 0xF0) |
                                 ((p2 >> 16) & 0x0F));
  data[6] = static_cast<uint8_t>((p2 >> 8) & 0xFF);
  data[7] = static_cast<uint8_t>(p2 & 0xFF);

  return i2cWriteBlock(SI5351_ADDRESS, baseRegister, data, sizeof(data));
}

bool si5351SetOutputMask(uint8_t clearBits, uint8_t setBits) {
  si5351OutputEnableShadow &= static_cast<uint8_t>(~clearBits);
  si5351OutputEnableShadow |= setBits;
  return i2cWriteRegister(SI5351_ADDRESS,
                          SI5351_REG_OUTPUT_ENABLE,
                          si5351OutputEnableShadow);
}

bool si5351EnableClk0(bool enabled) {
  if (!si5351Ready) return false;
  if (enabled == rxClockEnabled) return true;
  const bool ok = enabled
      ? si5351SetOutputMask(0x01, 0x00)
      : si5351SetOutputMask(0x00, 0x01);
  if (ok) rxClockEnabled = enabled;
  return ok;
}

bool si5351EnableClk1(bool enabled) {
  if (!si5351Ready) return false;
  if (enabled == txClockEnabled) return true;
  const bool ok = enabled
      ? si5351SetOutputMask(0x02, 0x00)
      : si5351SetOutputMask(0x00, 0x02);
  if (ok) txClockEnabled = enabled;
  return ok;
}

// Configure CLK0 at 4*F using a caller-selected fixed EVEN integer MS0 divider.
// profileTag identifies the normal band (0..3) or a special bench profile.
bool si5351ConfigureRxClockWithDivider(uint32_t frequencyHz,
                                       uint8_t ms0Divider,
                                       uint8_t profileTag) {
  if (!si5351Ready) return false;
  if ((ms0Divider & 1u) != 0u || ms0Divider < 8) return false;

  const double clk0Hz = 4.0 * static_cast<double>(frequencyHz);
  const double pllAHz = clk0Hz * static_cast<double>(ms0Divider);
  if (pllAHz < SI5351_PLL_MIN_HZ || pllAHz > SI5351_PLL_MAX_HZ) {
    rxClockConfigured = false;
    return false;
  }

  si5351EnableClk0(false);
  uint32_t p1, p2, p3;

  // Keep MS0 fixed and in integer mode. Rewrite it only when the profile/divider
  // actually changes. Frequency movement is produced by PLLA instead.
  if (!rxClockConfigured || currentRxBandIndex != profileTag ||
      currentRxMs0Divider != ms0Divider) {
    si5351CalculateParams(static_cast<double>(ms0Divider), p1, p2, p3);
    if (!si5351WriteSynth(SI5351_REG_MS0, p1, p2, p3)) return false;

    // 0x4C: MS0 integer mode, PLLA source, CLK0 sourced from MS0, not inverted.
    const uint8_t clk0Control =
        static_cast<uint8_t>(0x4C | si5351DriveCode(SI5351_RX_DRIVE_MA));
    if (!i2cWriteRegister(SI5351_ADDRESS,
                          SI5351_REG_CLK0_CONTROL,
                          clk0Control)) return false;
  }

  si5351CalculateParams(pllAHz / correctedReferenceHz(), p1, p2, p3);
  if (!si5351WriteSynth(SI5351_REG_PLLA, p1, p2, p3)) return false;
  if (!i2cWriteRegister(SI5351_ADDRESS, SI5351_REG_PLL_RESET, 0x20)) return false;

  currentRxBandIndex = profileTag;
  currentRxMs0Divider = ms0Divider;
  rxClockConfigured = true;
  rxRetunePending = false;
  return true;
}

// Normal receive setup: use the fixed divider assigned to the selected band.
bool si5351ConfigureRxClock(uint32_t frequencyHz) {
  const int bandIndex = bandIndexForFrequency(frequencyHz);
  if (bandIndex < 0) {
    rxClockConfigured = false;
    return false;
  }
  const BandPlan &band = BAND_PLANS[bandIndex];
  return si5351ConfigureRxClockWithDivider(
      frequencyHz, band.rxMs0Divider, static_cast<uint8_t>(bandIndex));
}

// Bench RX: special 10.000 MHz uses CLK0=40 MHz and fixed MS0=/20, so PLLA
// is exactly 800 MHz. Band tests reuse their normal fixed-divider RX plans.
bool si5351ConfigureBenchRxClock() {
  if (benchTestBandIndex < 0) {
    return si5351ConfigureRxClockWithDivider(10000000UL, 20, 0xFE);
  }
  const BandPlan &band = BAND_PLANS[benchTestBandIndex];
  return si5351ConfigureRxClockWithDivider(
      static_cast<uint32_t>(llround(benchTestRfFrequencyHz)),
      band.rxMs0Divider, static_cast<uint8_t>(benchTestBandIndex));
}

bool si5351ConfigureTxPll() {
  uint32_t p1, p2, p3;
  si5351CalculateParams(SI5351_TX_PLL_HZ / correctedReferenceHz(), p1, p2, p3);
  return si5351WriteSynth(SI5351_REG_PLLB, p1, p2, p3);
}

// CLK1 must be 2*RF because U9 divides it by two to make TX1/TX2.
bool si5351ConfigureTxClock(double rfFrequencyHz) {
  if (!si5351Ready || rfFrequencyHz <= 0.0) return false;

  const double clk1Hz = 2.0 * rfFrequencyHz;
  const double divider = SI5351_TX_PLL_HZ / clk1Hz;
  if (divider < 8.0 || divider > 900.0) return false;

  si5351EnableClk1(false);

  uint32_t p1, p2, p3;
  si5351CalculateParams(divider, p1, p2, p3);
  if (!si5351WriteSynth(SI5351_REG_MS1, p1, p2, p3)) return false;

  // 0x2C: fractional MS1, PLLB source, CLK1 sourced from MS1, non-inverted.
  const uint8_t clk1Control =
      static_cast<uint8_t>(0x2C | si5351DriveCode(SI5351_TX_DRIVE_MA));
  if (!i2cWriteRegister(SI5351_ADDRESS,
                        SI5351_REG_CLK1_CONTROL,
                        clk1Control)) return false;

  appliedTxFrequencyHz = rfFrequencyHz;
  txClockConfigured = true;
  return true;
}

// Fast FT8 modulation update: keep PLLB fixed and change only MS1. The software
// target is doubled before the U9 divide-by-two, so the final RF follows the
// measured USB FT8 tone with the correct frequency deviation.
bool si5351SetTxFrequencyFast(double rfFrequencyHz) {
  if (!si5351Ready || !txClockConfigured || rfFrequencyHz <= 0.0) return false;

  const double clk1Hz = 2.0 * rfFrequencyHz;
  const double divider = SI5351_TX_PLL_HZ / clk1Hz;
  if (divider < 8.0 || divider > 900.0) return false;

  uint32_t p1, p2, p3;
  si5351CalculateParams(divider, p1, p2, p3);
  if (!si5351WriteSynth(SI5351_REG_MS1, p1, p2, p3)) return false;

  appliedTxFrequencyHz = rfFrequencyHz;
  return true;
}

bool beginSi5351() {
  if (!si5351Present) return false;

  si5351OutputEnableShadow = 0xFF;
  if (!i2cWriteRegister(SI5351_ADDRESS,
                        SI5351_REG_OUTPUT_ENABLE,
                        0xFF)) return false;

  // Disabled CLK0/1/2 outputs are forced LOW, giving deterministic inputs to
  // both 74LVC74 divider sections during boot and switching.
  if (!i2cWriteRegister(SI5351_ADDRESS,
                        SI5351_REG_CLK_DISABLE_STATE,
                        0x00)) return false;

  if (!i2cWriteRegister(SI5351_ADDRESS, SI5351_REG_CLK0_CONTROL, 0x80)) return false;
  if (!i2cWriteRegister(SI5351_ADDRESS, SI5351_REG_CLK1_CONTROL, 0x80)) return false;
  if (!i2cWriteRegister(SI5351_ADDRESS, SI5351_REG_CLK2_CONTROL, 0x80)) return false;
  if (!i2cWriteRegister(SI5351_ADDRESS,
                        SI5351_REG_CRYSTAL_LOAD,
                        SI5351_CRYSTAL_LOAD_REGISTER)) return false;

  // Configure fixed PLLB at an exact x32 of the fitted reference:
  // 864 MHz on RP2354A hardware. FT8 modulation
  // never changes or resets PLLB; only MS1 is updated on each tone movement.
  if (!si5351ConfigureTxPll()) return false;
  if (!i2cWriteRegister(SI5351_ADDRESS, SI5351_REG_PLL_RESET, 0x80)) return false;

  si5351Ready = true;
  rxClockEnabled = false;
  txClockEnabled = false;
  rxClockConfigured = false;
  txClockConfigured = false;

  // Default to 20 m RX. The receiver RF path is still disconnected here.
  if (!si5351ConfigureRxClock(DEFAULT_DIAL_FREQUENCY_HZ)) {
    si5351Ready = false;
    return false;
  }
  if (!si5351EnableClk0(true)) {
    si5351Ready = false;
    return false;
  }

  delayMicroseconds(RF_SWITCH_SETTLE_US);
  setRxPathEnabled(true);
  rxAudioBlankingActive = false;
  return true;
}

void beginI2cAndSi5351() {
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  // Never allow a stuck SDA/SCL or RF-corrupted transaction to block core 0
  // indefinitely. Arduino-Pico can reset the I2C peripheral on timeout.
  Wire.setTimeout(I2C_TRANSACTION_TIMEOUT_MS, true);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.clearTimeoutFlag();

  si5351Present = i2cDevicePresent(SI5351_ADDRESS);
  if (si5351Present) si5351Ready = beginSi5351();
}

// ---------------------------------------------------------------------------
// USB Audio Class 2
// ---------------------------------------------------------------------------
USBAudioStream usbAudio;
int16_t usbInputBlock[FRAMES_PER_BLOCK * CHANNELS];
int16_t usbOutputBlock[FRAMES_PER_BLOCK * CHANNELS];

// Forward declarations used by the RX blanking path below.
bool audioTxRequested();
bool combinedTxRequested();

bool beginUsbAudio() {
  AudioInfo format(SAMPLE_RATE, CHANNELS, BITS_PER_SAMPLE);
  auto cfg = usbAudio.defaultConfig(RXTX_MODE);
  cfg.copyFrom(format);

  // These values preserve compatibility with the Android FT8 applications
  // used by the earlier version of this project.
  cfg.vid = 0x2E8A;
  cfg.pid = 0x0005;
  cfg.manufacturer = "AT-DX1";
  cfg.product = "AT-DX1 RP2350 FT8 CW USB Audio CAT";
  cfg.serial = "AT-DX1-RP2350-1";
  cfg.self_powered = false;
  cfg.max_power_ma = 100;
  cfg.fifo_packets = 32;
  cfg.enable_multi_sample_rate = false;
  cfg.enable_ep_in_flow_control = true;
  cfg.volume_active = false;

  if (!usbAudio.begin(cfg)) return false;

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(20);  // setup-only descriptor reload
    TinyUSBDevice.attach();
  }
  return true;
}


// ---------------------------------------------------------------------------
// Persistent I/Q balance calibration
// ---------------------------------------------------------------------------
// Calibration fixture:
//   AT-DX1 dial / RX LO: 14.074000 MHz, USB
//   Signal generator:    14.075000 MHz, approximately -60 dBm
// This produces a clean +1 kHz upper-sideband tone. The sign of the measured
// Q versus H(I) correlation identifies the board's physical quadrature sense;
// the calibration then solves a least-squares correction that makes the
// time-aligned Q path match the correctly signed H(I) path.  I is the amplitude reference; Q gets a relative
// gain plus a small I cross-term that corrects residual quadrature/phase error.
// The correction is physical-I/Q compensation and therefore applies to both
// USB and LSB receive modes.
static constexpr uint32_t IQCAL_RX_FREQUENCY_HZ = 14074000UL;
static constexpr uint32_t IQCAL_SIGNAL_FREQUENCY_HZ = 14075000UL;
static constexpr uint32_t IQCAL_ARM_DELAY_MS = 300UL;
static constexpr uint32_t IQCAL_SAMPLE_TARGET = SAMPLE_RATE;  // 1 second
static constexpr float IQCAL_MIN_RMS_ADC_COUNTS = 2.0f;
static constexpr float IQCAL_MIN_USB_CORRELATION = 0.50f;
static constexpr float IQCAL_MIN_Q_GAIN = 0.50f;
static constexpr float IQCAL_MAX_Q_GAIN = 2.00f;
static constexpr float IQCAL_MAX_ABS_Q_MIX = 0.75f;
static constexpr float IQCAL_MIN_IMAGE_REJECTION_DB = 15.0f;

// Arduino-Pico EEPROM API is flash-backed emulation on RP2350; there is no
// physical EEPROM device on the AT-DX1 PCB.
static constexpr uint32_t IQCAL_FLASH_MAGIC = 0x31435149UL;  // "IQC1"
static constexpr uint16_t IQCAL_FLASH_VERSION = 2;
static constexpr size_t IQCAL_FLASH_STORAGE_BYTES = 256;
static constexpr int IQCAL_FLASH_STORAGE_ADDRESS = 0;

struct IqCalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t bytes;
  float qGain;
  float qPhaseMix;
  float usbPolarity;
  float imageRejectionDb;
  float signalRmsAdcCounts;
  uint32_t checksum;
};

// Explicit prototype: prevents Arduino from auto-generating this declaration
// ahead of IqCalibrationRecord.
uint32_t iqCalibrationChecksum(const IqCalibrationRecord &record);

static_assert(sizeof(IqCalibrationRecord) <= IQCAL_FLASH_STORAGE_BYTES,
              "I/Q calibration record must fit in flash-backed EEPROM-emulation allocation");

float rxIqQGain = 1.0f;
float rxIqQPhaseMix = 0.0f;
// Runtime multiplier for the physical quadrature sense. +1 keeps the original
// PCB convention; -1 swaps USB/LSB Hilbert polarity. IQCAL learns this from
// the fixed +1 kHz calibration carrier.
float rxIqUsbPolarity = 1.0f;
float iqCalLastImageRejectionDb = 0.0f;
float iqCalLastSignalRmsAdcCounts = 0.0f;
bool iqCalibrationStored = false;

uint32_t iqCalibrationChecksum(const IqCalibrationRecord &record) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  const size_t count = offsetof(IqCalibrationRecord, checksum);
  uint32_t hash = 2166136261UL;  // FNV-1a
  for (size_t i = 0; i < count; ++i) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool iqCalibrationValuesSane(float qGain, float qMix, float usbPolarity) {
  return isfinite(qGain) && isfinite(qMix) && isfinite(usbPolarity) &&
         qGain >= IQCAL_MIN_Q_GAIN && qGain <= IQCAL_MAX_Q_GAIN &&
         fabsf(qMix) <= IQCAL_MAX_ABS_Q_MIX &&
         fabsf(fabsf(usbPolarity) - 1.0f) < 0.01f;
}

void setDefaultIqCalibration() {
  rxIqQGain = 1.0f;
  rxIqQPhaseMix = 0.0f;
  rxIqUsbPolarity = 1.0f;
  iqCalLastImageRejectionDb = 0.0f;
  iqCalLastSignalRmsAdcCounts = 0.0f;
  iqCalibrationStored = false;
}

void beginIqCalibrationStorage() {
  EEPROM.begin(IQCAL_FLASH_STORAGE_BYTES);

  IqCalibrationRecord record{};
  EEPROM.get(IQCAL_FLASH_STORAGE_ADDRESS, record);
  const bool valid =
      record.magic == IQCAL_FLASH_MAGIC &&
      record.version == IQCAL_FLASH_VERSION &&
      record.bytes == sizeof(IqCalibrationRecord) &&
      record.checksum == iqCalibrationChecksum(record) &&
      iqCalibrationValuesSane(record.qGain, record.qPhaseMix, record.usbPolarity) &&
      isfinite(record.imageRejectionDb) &&
      isfinite(record.signalRmsAdcCounts);

  if (!valid) {
    setDefaultIqCalibration();
    return;
  }

  rxIqQGain = record.qGain;
  rxIqQPhaseMix = record.qPhaseMix;
  rxIqUsbPolarity = record.usbPolarity;
  iqCalLastImageRejectionDb = record.imageRejectionDb;
  iqCalLastSignalRmsAdcCounts = record.signalRmsAdcCounts;
  iqCalibrationStored = true;
}

bool saveIqCalibrationToFlash() {
  IqCalibrationRecord record{};
  record.magic = IQCAL_FLASH_MAGIC;
  record.version = IQCAL_FLASH_VERSION;
  record.bytes = sizeof(IqCalibrationRecord);
  record.qGain = rxIqQGain;
  record.qPhaseMix = rxIqQPhaseMix;
  record.usbPolarity = rxIqUsbPolarity;
  record.imageRejectionDb = iqCalLastImageRejectionDb;
  record.signalRmsAdcCounts = iqCalLastSignalRmsAdcCounts;
  record.checksum = iqCalibrationChecksum(record);

  EEPROM.put(IQCAL_FLASH_STORAGE_ADDRESS, record);
  watchdog_update();
  const bool ok = EEPROM.commit();
  watchdog_update();
  iqCalibrationStored = ok;
  return ok;
}

enum IqCalibrationState : uint8_t {
  IQCAL_IDLE = 0,
  IQCAL_ARMING,
  IQCAL_CAPTURING,
  IQCAL_SOLVE_PENDING,
  IQCAL_DONE_OK,
  IQCAL_DONE_FAIL
};

enum IqCalibrationFailReason : uint8_t {
  IQCAL_FAIL_NONE = 0,
  IQCAL_FAIL_BUSY,
  IQCAL_FAIL_TUNE_CHANGED,
  IQCAL_FAIL_SIGNAL_LOW,
  IQCAL_FAIL_WRONG_SIDEBAND,
  IQCAL_FAIL_MATH,
  IQCAL_FAIL_RANGE,
  IQCAL_FAIL_REJECTION,
  IQCAL_FAIL_SAVE
};

// Explicit prototypes for functions whose signatures use the enum.  Without
// these, Arduino can emit prototypes before IqCalibrationFailReason exists.
const char *iqCalibrationFailText(IqCalibrationFailReason reason);
void failIqCalibration(IqCalibrationFailReason reason);

IqCalibrationState iqCalState = IQCAL_IDLE;
IqCalibrationFailReason iqCalFailReason = IQCAL_FAIL_NONE;
uint32_t iqCalArmAtMs = 0;
uint32_t iqCalSamples = 0;
uint32_t iqCalSavedDialFrequencyHz = DEFAULT_DIAL_FREQUENCY_HZ;
RxSidebandMode iqCalSavedSidebandMode = RX_MODE_USB;

// Normalized second-order statistics accumulated from one second of the +1 kHz
// calibration tone.  Normalizing to ADC full scale keeps the float sums small.
float iqCalSumQQ = 0.0f;
float iqCalSumII = 0.0f;
float iqCalSumHH = 0.0f;
float iqCalSumQI = 0.0f;
float iqCalSumQH = 0.0f;
float iqCalSumIH = 0.0f;

void resetIqCalibrationAccumulator() {
  iqCalSamples = 0;
  iqCalSumQQ = 0.0f;
  iqCalSumII = 0.0f;
  iqCalSumHH = 0.0f;
  iqCalSumQI = 0.0f;
  iqCalSumQH = 0.0f;
  iqCalSumIH = 0.0f;
}

bool iqCalibrationActive() {
  return iqCalState == IQCAL_ARMING ||
         iqCalState == IQCAL_CAPTURING ||
         iqCalState == IQCAL_SOLVE_PENDING;
}

void armIqCalibrationCaptureIfReady() {
  if (iqCalState != IQCAL_ARMING || rxAudioBlankingActive) return;
  if (static_cast<int32_t>(millis() - iqCalArmAtMs) < 0) return;
  resetIqCalibrationAccumulator();
  iqCalState = IQCAL_CAPTURING;
}

void accumulateIqCalibrationSample(float iDelayed, float qDelayed,
                                   float hilbertIUsbSense) {
  if (iqCalState != IQCAL_CAPTURING) return;

  static constexpr float SCALE = 1.0f / 32768.0f;
  const float i = iDelayed * SCALE;
  const float q = qDelayed * SCALE;
  const float h = hilbertIUsbSense * SCALE;

  iqCalSumQQ += q * q;
  iqCalSumII += i * i;
  iqCalSumHH += h * h;
  iqCalSumQI += q * i;
  iqCalSumQH += q * h;
  iqCalSumIH += i * h;

  ++iqCalSamples;
  if (iqCalSamples >= IQCAL_SAMPLE_TARGET) {
    iqCalState = IQCAL_SOLVE_PENDING;
  }
}

// ---------------------------------------------------------------------------
// I/Q receive sideband selector
// ---------------------------------------------------------------------------
// The QSD PCB mapping is Q_OUT from the undelayed clock and I_OUT from the
// 90-degree-delayed clock.  With that physical convention an RF signal above
// the LO is selected by Q(delayed) + Hilbert(I).  If a future PCB revision
// reverses quadrature sense, changing this one constant to -1 swaps the
// physical USB/LSB sense without touching the DSP.
static constexpr float RX_USB_HILBERT_SIGN = +1.0f;

// 511-tap Hamming-windowed Hilbert transformer at 48 ksps.  Its group delay is
// 255 samples (5.3125 ms).  Anti-symmetry and every-other zero coefficient
// reduce the implementation to only 128 multiply-accumulates per audio sample.
// It is essentially flat through the useful AT-DX1 digital/CW audio range;
// the very-low-frequency transition is deliberately traded for practical CPU
// cost and much stronger opposite-sideband rejection from ~200 Hz upward.
static constexpr size_t RX_HILBERT_TAPS = 511;
static constexpr size_t RX_HILBERT_DELAY = (RX_HILBERT_TAPS - 1) / 2;
static constexpr size_t RX_IQ_HISTORY_SIZE = 512;
static constexpr size_t RX_IQ_HISTORY_MASK = RX_IQ_HISTORY_SIZE - 1;
static_assert((RX_IQ_HISTORY_SIZE & RX_IQ_HISTORY_MASK) == 0,
              "RX_IQ_HISTORY_SIZE must be a power of two");

static constexpr float RX_HILBERT_COEFF[128] = {
    -0.0001997239f, -0.0002016540f, -0.0002043231f, -0.0002077482f, -0.0002119465f, -0.0002169355f, -0.0002227327f, -0.0002293558f,
    -0.0002368226f, -0.0002451510f, -0.0002543592f, -0.0002644654f, -0.0002754882f, -0.0002874461f, -0.0003003582f, -0.0003142434f,
    -0.0003291210f, -0.0003450107f, -0.0003619323f, -0.0003799057f, -0.0003989515f, -0.0004190903f, -0.0004403432f, -0.0004627315f,
    -0.0004862771f, -0.0005110020f, -0.0005369290f, -0.0005640810f, -0.0005924817f, -0.0006221551f, -0.0006531258f, -0.0006854193f,
    -0.0007190612f, -0.0007540782f, -0.0007904977f, -0.0008283476f, -0.0008676569f, -0.0009084554f, -0.0009507739f, -0.0009946441f,
    -0.0010400989f, -0.0010871723f, -0.0011358996f, -0.0011863174f, -0.0012384637f, -0.0012923781f, -0.0013481017f, -0.0014056774f,
    -0.0014651500f, -0.0015265664f, -0.0015899755f, -0.0016554285f, -0.0017229792f, -0.0017926840f, -0.0018646022f, -0.0019387960f,
    -0.0020153312f, -0.0020942769f, -0.0021757061f, -0.0022596961f, -0.0023463284f, -0.0024356896f, -0.0025278713f, -0.0026229707f,
    -0.0027210912f, -0.0028223429f, -0.0029268427f, -0.0030347157f, -0.0031460952f, -0.0032611236f, -0.0033799537f, -0.0035027487f,
    -0.0036296842f, -0.0037609487f, -0.0038967447f, -0.0040372909f, -0.0041828228f, -0.0043335950f, -0.0044898831f, -0.0046519857f,
    -0.0048202269f, -0.0049949592f, -0.0051765663f, -0.0053654673f, -0.0055621200f, -0.0057670262f, -0.0059807363f, -0.0062038561f,
    -0.0064370532f, -0.0066810655f, -0.0069367103f, -0.0072048954f, -0.0074866317f, -0.0077830484f, -0.0080954105f, -0.0084251398f,
    -0.0087738399f, -0.0091433256f, -0.0095356596f, -0.0099531951f, -0.0103986291f, -0.0108750675f, -0.0113861046f, -0.0119359238f,
    -0.0125294220f, -0.0131723688f, -0.0138716094f, -0.0146353267f, -0.0154733847f, -0.0163977826f, -0.0174232650f, -0.0185681541f,
    -0.0198555033f, -0.0213147291f, -0.0229839676f, -0.0249135627f, -0.0271713736f, -0.0298511153f, -0.0330859674f, -0.0370717885f,
    -0.0421089019f, -0.0486824534f, -0.0576304318f, -0.0705357167f, -0.0907902083f, -0.1272128683f, -0.2121399256f, -0.6365975484f,
};

float rxIHistory[RX_IQ_HISTORY_SIZE] = {};
float rxQHistory[RX_IQ_HISTORY_SIZE] = {};
size_t rxIqHistoryWrite = 0;
uint16_t rxIqHistoryPrimed = 0;

void resetRxSidebandDsp() {
  memset(rxIHistory, 0, sizeof(rxIHistory));
  memset(rxQHistory, 0, sizeof(rxQHistory));
  rxIqHistoryWrite = 0;
  rxIqHistoryPrimed = 0;
}

void setRxSidebandMode(RxSidebandMode mode) {
  if (mode != RX_MODE_USB && mode != RX_MODE_LSB) return;
  if (rxSidebandMode == mode) return;
  rxSidebandMode = mode;
  resetRxSidebandDsp();
}

float demodulateIqSideband(float iSample, float qSample) {
  const size_t w = rxIqHistoryWrite;
  rxIHistory[w] = iSample;
  rxQHistory[w] = qSample;

  float hilbertI = 0.0f;
  for (size_t pair = 0; pair < 128; ++pair) {
    const size_t m = pair * 2;
    const size_t nearIndex = (w - m) & RX_IQ_HISTORY_MASK;
    const size_t farIndex =
        (w - ((RX_HILBERT_TAPS - 1) - m)) & RX_IQ_HISTORY_MASK;
    hilbertI += RX_HILBERT_COEFF[pair] *
                (rxIHistory[nearIndex] - rxIHistory[farIndex]);
  }

  // The single RP2350 ADC samples Q then I, so the I sample occurs half of an
  // audio-frame later (1/96 kHz = 10.4167 us).  The Hilbert output therefore
  // represents physical time (n-delay+0.5)T.  Align Q to that same instant with
  // a 4-point half-sample cubic interpolation.  This is essentially flat over
  // the AT-DX1 audio range and avoids the frequency-dependent image rejection
  // loss that a simple Q/I pair would otherwise suffer from ADC time skew.
  const size_t q0 = (w - RX_HILBERT_DELAY) & RX_IQ_HISTORY_MASK;
  const float qDelayed =
      (-rxQHistory[(q0 - 1) & RX_IQ_HISTORY_MASK]
       + 9.0f * rxQHistory[q0]
       + 9.0f * rxQHistory[(q0 + 1) & RX_IQ_HISTORY_MASK]
       - rxQHistory[(q0 + 2) & RX_IQ_HISTORY_MASK]) * (1.0f / 16.0f);

  // I was physically sampled half a frame after Q.  The ordinary centre tap
  // of the I history is therefore already at the same physical instant as the
  // half-sample-interpolated Q above.  It is also the orthogonal basis used by
  // the calibration cross-term to remove residual analogue phase error.
  const float iDelayed = rxIHistory[q0];

  rxIqHistoryWrite = (w + 1) & RX_IQ_HISTORY_MASK;
  if (rxIqHistoryPrimed < RX_HILBERT_TAPS) ++rxIqHistoryPrimed;

  // Mute the short startup/retune fill interval instead of passing a partly
  // populated Hilbert history as a click into the USB sound-card stream.
  if (rxIqHistoryPrimed < RX_HILBERT_TAPS) return 0.0f;

  // The calibration always evaluates the physical USB relationship, regardless
  // of the currently selected receive mode.  With the specified +1 kHz test
  // carrier, qDelayed and this USB-sense H(I) waveform should be in phase.
  const float effectiveUsbHilbertSign = RX_USB_HILBERT_SIGN * rxIqUsbPolarity;
  const float hilbertIUsbSense = effectiveUsbHilbertSign * hilbertI;
  accumulateIqCalibrationSample(iDelayed, qDelayed, hilbertIUsbSense);

  // I is the calibration reference.  qGain corrects relative amplitude;
  // qPhaseMix rotates the Q path slightly using the orthogonal I basis.
  const float qCorrected = rxIqQGain * qDelayed +
                           rxIqQPhaseMix * iDelayed;
  const float sign =
      (rxSidebandMode == RX_MODE_USB ? effectiveUsbHilbertSign
                                     : -effectiveUsbHilbertSign);

  // 0.5 keeps a correctly phased wanted signal at approximately the same USB
  // level as the former single-Q path: corrected Q and H(I) add coherently.
  return 0.5f * (qCorrected + sign * hilbertI);
}

// ---------------------------------------------------------------------------
// ADC capture: core 1 DMA -> core 0 USB input
// ---------------------------------------------------------------------------
enum AdcBufferState : uint8_t {
  ADC_EMPTY = 0,
  ADC_FILLING,
  ADC_READY,
  ADC_COPYING
};

static constexpr uint8_t ADC_BUFFER_COUNT = 3;
alignas(4) uint16_t adcBuffer[ADC_BUFFER_COUNT][ADC_SAMPLES_PER_BLOCK];
volatile uint8_t adcState[ADC_BUFFER_COUNT] = {
    ADC_EMPTY, ADC_EMPTY, ADC_EMPTY};
volatile uint32_t adcReadySequence[ADC_BUFFER_COUNT] = {0, 0, 0};
volatile uint32_t adcSequenceCounter = 0;
volatile uint32_t adcDroppedBlocks = 0;
volatile uint8_t adcDmaIndex = 0;
volatile bool adcStarted = false;

int adcDmaChannel = -1;
spin_lock_t *adcStateLock = nullptr;

uint8_t chooseNextAdcDmaBufferLocked() {
  for (uint8_t i = 0; i < ADC_BUFFER_COUNT; ++i) {
    if (adcState[i] == ADC_EMPTY) return i;
  }

  int selected = -1;
  uint32_t oldestSequence = UINT32_MAX;
  for (uint8_t i = 0; i < ADC_BUFFER_COUNT; ++i) {
    if (adcState[i] == ADC_READY &&
        adcReadySequence[i] < oldestSequence) {
      selected = i;
      oldestSequence = adcReadySequence[i];
    }
  }

  if (selected >= 0) {
    ++adcDroppedBlocks;
    return static_cast<uint8_t>(selected);
  }

  ++adcDroppedBlocks;
  return adcDmaIndex;
}

void __isr adcDmaHandler() {
  const uint32_t mask = 1u << adcDmaChannel;
  if ((dma_hw->ints1 & mask) == 0) return;
  dma_hw->ints1 = mask;

  const uint32_t irqState = spin_lock_blocking(adcStateLock);
  const uint8_t completed = adcDmaIndex;
  adcState[completed] = ADC_READY;
  adcReadySequence[completed] = ++adcSequenceCounter;

  const uint8_t next = chooseNextAdcDmaBufferLocked();
  adcDmaIndex = next;
  adcState[next] = ADC_FILLING;
  spin_unlock(adcStateLock, irqState);

  dma_channel_set_write_addr(adcDmaChannel, adcBuffer[next], false);
  dma_channel_set_trans_count(adcDmaChannel, ADC_SAMPLES_PER_BLOCK, true);
}

void beginAdcOnCore1() {
  const int lockNumber = spin_lock_claim_unused(true);
  adcStateLock = spin_lock_instance(static_cast<uint>(lockNumber));

  const uint32_t state = spin_lock_blocking(adcStateLock);
  for (uint8_t i = 0; i < ADC_BUFFER_COUNT; ++i) {
    adcState[i] = ADC_EMPTY;
  }
  adcDmaIndex = 0;
  adcState[0] = ADC_FILLING;
  spin_unlock(adcStateLock, state);

  adc_init();
  adc_gpio_init(Q_AUDIO_PIN);
  adc_gpio_init(I_AUDIO_PIN);

  // The first conversion is Q/ADC0.  RP2350 round-robin then advances to
  // I/ADC1 and alternates Q,I,Q,I... continuously.
  adc_select_input(Q_ADC_CHANNEL);
  adc_set_round_robin((1u << Q_ADC_CHANNEL) | (1u << I_ADC_CHANNEL));
  adc_fifo_setup(true, true, 1, false, false);
  adc_fifo_drain();

  // 96 ksps total -> 48 ksps Q + 48 ksps I.  The RP-series ADC runs from an
  // independent 48 MHz clock, and 96 ksps is well below its conversion limit.
  adc_set_clkdiv((48000000.0f / static_cast<float>(RX_ADC_TOTAL_SAMPLE_RATE)) - 1.0f);

  adcDmaChannel = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(adcDmaChannel);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_dreq(&cfg, DREQ_ADC);

  dma_channel_configure(adcDmaChannel,
                        &cfg,
                        adcBuffer[0],
                        &adc_hw->fifo,
                        ADC_SAMPLES_PER_BLOCK,
                        false);

  dma_channel_set_irq1_enabled(adcDmaChannel, true);
  irq_set_exclusive_handler(DMA_IRQ_1, adcDmaHandler);
  irq_set_enabled(DMA_IRQ_1, true);

  adcStarted = true;
  tinyDxMemoryBarrier();
  dma_channel_start(adcDmaChannel);
  adc_run(true);
}

bool getAdcBlock(uint16_t *destination) {
  if (!adcStarted || adcStateLock == nullptr) return false;

  int selected = -1;
  uint32_t oldestSequence = UINT32_MAX;

  setWatchdogBreadcrumb(WD_STAGE_ADC_LOCK_1);
  uint32_t irqState = spin_lock_blocking(adcStateLock);
  for (uint8_t i = 0; i < ADC_BUFFER_COUNT; ++i) {
    if (adcState[i] == ADC_READY &&
        adcReadySequence[i] < oldestSequence) {
      selected = i;
      oldestSequence = adcReadySequence[i];
    }
  }
  if (selected >= 0) adcState[selected] = ADC_COPYING;
  spin_unlock(adcStateLock, irqState);

  if (selected < 0) return false;

  memcpy(destination, adcBuffer[selected], sizeof(adcBuffer[selected]));

  setWatchdogBreadcrumb(WD_STAGE_ADC_LOCK_2);
  irqState = spin_lock_blocking(adcStateLock);
  adcState[selected] = ADC_EMPTY;
  spin_unlock(adcStateLock, irqState);
  return true;
}

// Bound USB-audio work on core 0 so CAT can never be starved by an audio
// endpoint that remains continuously active. Two 48-frame blocks give plenty
// of catch-up margin at 48 kHz while returning to CAT every few milliseconds.
static constexpr uint8_t USB_AUDIO_BLOCKS_PER_PASS = 2;

void sendAdcToUsb() {
  static uint16_t raw[ADC_SAMPLES_PER_BLOCK];

  // Each analogue baseband path gets its own slow DC servo.  Keeping the DC
  // removal before the Hilbert transform avoids turning independent op-amp/ADC
  // offsets into a false quadrature component.
  static int32_t dcEstimateQ16 = 2048L << 16;
  static int32_t dcEstimateI16 = 2048L << 16;
  static bool wasBlanking = true;

  for (uint8_t block = 0; block < USB_AUDIO_BLOCKS_PER_PASS; ++block) {
    if (!getAdcBlock(raw)) break;

    const bool usbCaptureStreaming = usbAudio.isStreamingActiveTx();

    // Reset the phasing history once per RX-clock blanking interval.  This
    // prevents old pre-retune I/Q samples from being combined with the new LO.
    if (rxAudioBlankingActive) {
      if (!wasBlanking) resetRxSidebandDsp();
      wasBlanking = true;
      if (usbCaptureStreaming) {
        memset(usbInputBlock, 0, sizeof(usbInputBlock));
        setWatchdogBreadcrumb(WD_STAGE_USB_WRITE, sizeof(usbInputBlock));
        usbAudio.write(reinterpret_cast<const uint8_t *>(usbInputBlock),
                       sizeof(usbInputBlock));
      }
      continue;
    }
    wasBlanking = false;
    armIqCalibrationCaptureIfReady();

    for (size_t frame = 0; frame < FRAMES_PER_BLOCK; ++frame) {
      // beginAdcOnCore1() establishes the repeating FIFO order Q then I.
      const int32_t rawQ12 = static_cast<int32_t>(raw[frame * 2] & 0x0FFFu);
      const int32_t rawI12 = static_cast<int32_t>(raw[frame * 2 + 1] & 0x0FFFu);

      const int32_t rawQ16 = rawQ12 << 16;
      const int32_t rawI16 = rawI12 << 16;
      dcEstimateQ16 += (rawQ16 - dcEstimateQ16) >> 12;
      dcEstimateI16 += (rawI16 - dcEstimateI16) >> 12;

      const int32_t centredQ = (rawQ16 - dcEstimateQ16) >> 16;
      const int32_t centredI = (rawI16 - dcEstimateI16) >> 16;

      // Preserve the old Q-only gain ahead of the 0.5 coherent I/Q combine.
      const float q = static_cast<float>(centredQ * 16);
      const float i = static_cast<float>(centredI * 16);
      float demod = demodulateIqSideband(i, q);

      if (demod > 32767.0f) demod = 32767.0f;
      if (demod < -32768.0f) demod = -32768.0f;
      const int16_t sample = static_cast<int16_t>(lrintf(demod));

      if (usbCaptureStreaming) {
        // Keep the existing USB descriptor and host behaviour: ordinary mono
        // receive audio duplicated into stereo L/R, never raw I/Q over USB.
        usbInputBlock[frame * 2] = sample;
        usbInputBlock[frame * 2 + 1] = sample;
      }
    }

    if (usbCaptureStreaming) {
      setWatchdogBreadcrumb(WD_STAGE_USB_WRITE, sizeof(usbInputBlock));
      usbAudio.write(reinterpret_cast<const uint8_t *>(usbInputBlock),
                     sizeof(usbInputBlock));
    }
  }
}

// ---------------------------------------------------------------------------
// Core-to-core USB-playback sample ring
// ---------------------------------------------------------------------------
static constexpr uint32_t TRACKER_RING_SIZE = 4096;
static constexpr uint32_t TRACKER_RING_MASK = TRACKER_RING_SIZE - 1;
// Never let core 1 remain inside one ring-drain loop indefinitely while the
// USB producer keeps adding samples. At 48 kHz, 256 samples are only 5.33 ms.
static constexpr uint16_t CORE1_MAX_SAMPLES_PER_PASS = 256;
static constexpr uint16_t CORE1_HEARTBEAT_SAMPLE_INTERVAL = 64;
static_assert((TRACKER_RING_SIZE & TRACKER_RING_MASK) == 0,
              "TRACKER_RING_SIZE must be a power of two");

int16_t trackerRing[TRACKER_RING_SIZE];
volatile uint32_t trackerHead = 0;
volatile uint32_t trackerTail = 0;

void trackerPush(int16_t sample) {
  const uint32_t head = trackerHead;
  if ((head - trackerTail) >= TRACKER_RING_SIZE) {
    ++trackerOverruns;
    return;
  }

  trackerRing[head & TRACKER_RING_MASK] = sample;
  tinyDxMemoryBarrier();
  trackerHead = head + 1;
}

bool trackerPop(int16_t &sample) {
  const uint32_t tail = trackerTail;
  if (tail == trackerHead) return false;

  sample = trackerRing[tail & TRACKER_RING_MASK];
  tinyDxMemoryBarrier();
  trackerTail = tail + 1;
  return true;
}

void trackerFlush() {
  trackerTail = trackerHead;
  tinyDxMemoryBarrier();
}

// ---------------------------------------------------------------------------
// TinyDX-style four-crossing / three-cycle detector on core 1
// ---------------------------------------------------------------------------
struct ThreeCycleDetectorState {
  int16_t previousSample = 0;
  bool zeroCrossArmed = false;
  bool voxActive = false;
  uint32_t sampleNumber = 0;
  uint32_t lastLoudSample = 0;
  uint32_t crossingsQ16[4] = {0, 0, 0, 0};
  uint8_t crossingCount = 0;
};

ThreeCycleDetectorState detector;

int32_t absoluteSample(int16_t value) {
  const int32_t expanded = static_cast<int32_t>(value);
  return expanded < 0 ? -expanded : expanded;
}

void resetTxAudioGuard() {
  txAudioWindowReferenceHz = 0.0;
  txAudioWindowLocked = false;
  lastPublishedAudioMilliHz = 0;
}

void publishAudioFrequency(double frequencyHz) {
  int32_t milliHz = 0;

  if (frequencyHz >= MIN_AUDIO_FREQUENCY_HZ &&
      frequencyHz <= MAX_AUDIO_FREQUENCY_HZ) {
    // The first clean estimate establishes the allowed FT8 audio neighbourhood
    // for this transmission. Any standard FT8 tone is within 43.75 Hz of it.
    if (!txAudioWindowLocked) {
      txAudioWindowReferenceHz = frequencyHz;
      txAudioWindowLocked = true;
    } else if (fabs(frequencyHz - txAudioWindowReferenceHz) >
               TX_AUDIO_WINDOW_HALF_WIDTH_HZ) {
      return;  // Reject unrelated/wild audio rather than widening the RF TX.
    }

    milliHz = static_cast<int32_t>(llround(frequencyHz * 1000.0));

    // Avoid hundreds of unnecessary MS0 rewrites per second while a nominally
    // constant FT8 tone is only moving by estimator dither. During a real GFSK
    // transition, cumulative motion quickly exceeds this small deadband.
    if (lastPublishedAudioMilliHz != 0 &&
        llabs(static_cast<long long>(milliHz) -
              static_cast<long long>(lastPublishedAudioMilliHz)) <
            static_cast<long long>(llround(TX_RETUNE_DEADBAND_HZ * 1000.0))) {
      return;
    }
  } else {
    // Zero is the deliberate 'no valid TX audio' command. Other out-of-range
    // measurements are also converted to zero by callers only when TX ends.
    milliHz = 0;
  }

  if (milliHz == lastPublishedAudioMilliHz) return;

  lastPublishedAudioMilliHz = milliHz;
  requestedAudioMilliHz = milliHz;
  tinyDxMemoryBarrier();
  ++frequencyRequestVersion;
}

void clearThreeCycleMeasurement() {
  detector.zeroCrossArmed = false;
  detector.crossingCount = 0;
}

void startVox() {
  if (detector.voxActive) return;
  detector.voxActive = true;
  requestedVoxTx = true;
  resetTxAudioGuard();
  clearThreeCycleMeasurement();
  tinyDxMemoryBarrier();
}

void stopVox(VoxStopReason reason = VOX_STOP_NONE) {
  if (!detector.voxActive) return;
  detector.voxActive = false;
  requestedVoxTx = false;
  lastVoxStopReason = reason;
  publishAudioFrequency(0.0);
  resetTxAudioGuard();
  clearThreeCycleMeasurement();
  tinyDxMemoryBarrier();
}

bool periodIsValid(uint32_t periodQ16) {
  const double minimumPeriodQ16 =
      (static_cast<double>(SAMPLE_RATE) * 65536.0) /
      MAX_AUDIO_FREQUENCY_HZ;
  const double maximumPeriodQ16 =
      (static_cast<double>(SAMPLE_RATE) * 65536.0) /
      MIN_AUDIO_FREQUENCY_HZ;

  const double period = static_cast<double>(periodQ16);
  return period >= minimumPeriodQ16 && period <= maximumPeriodQ16;
}

void processPlaybackSample(int16_t sample) {
  const uint32_t currentSampleNumber = detector.sampleNumber++;
  const int32_t magnitude = absoluteSample(sample);

  if (magnitude >= VOX_SAMPLE_THRESHOLD) {
    detector.lastLoudSample = currentSampleNumber;
    if (!detector.voxActive) startVox();
  }

  if (detector.voxActive &&
      static_cast<uint32_t>(currentSampleNumber - detector.lastLoudSample) >
          VOX_HANG_SAMPLES) {
    stopVox(VOX_STOP_SILENCE);
    detector.previousSample = sample;
    return;
  }

  if (!detector.voxActive) {
    detector.previousSample = sample;
    return;
  }

  // Schmitt-style arming prevents small noise around zero from creating
  // extra crossings. Interpolation still uses the actual samples around zero.
  if (sample <= -ZERO_CROSS_HYSTERESIS) {
    detector.zeroCrossArmed = true;
  }

  if (detector.zeroCrossArmed &&
      detector.previousSample <= 0 &&
      sample > 0 &&
      currentSampleNumber > 0) {

    const int32_t denominator =
        static_cast<int32_t>(sample) -
        static_cast<int32_t>(detector.previousSample);

    if (denominator > 0) {
      const uint32_t numerator = static_cast<uint32_t>(
          -static_cast<int32_t>(detector.previousSample));
      const uint32_t fractionQ16 = static_cast<uint32_t>(
          (static_cast<uint64_t>(numerator) << 16) /
          static_cast<uint32_t>(denominator));

      const uint32_t crossingQ16 =
          ((currentSampleNumber - 1u) << 16) + fractionQ16;

      detector.crossingsQ16[detector.crossingCount++] = crossingQ16;

      if (detector.crossingCount == 4) {
        const uint32_t period1 =
            detector.crossingsQ16[1] - detector.crossingsQ16[0];
        const uint32_t period2 =
            detector.crossingsQ16[2] - detector.crossingsQ16[1];
        const uint32_t period3 =
            detector.crossingsQ16[3] - detector.crossingsQ16[2];

        if (periodIsValid(period1) &&
            periodIsValid(period2) &&
            periodIsValid(period3)) {
          const double scale = static_cast<double>(SAMPLE_RATE) * 65536.0;
          const double frequency1 = scale / static_cast<double>(period1);
          const double frequency2 = scale / static_cast<double>(period2);
          const double frequency3 = scale / static_cast<double>(period3);
          const double averageFrequency =
              (frequency1 + frequency2 + frequency3) / 3.0;
          const double minimumFrequency =
              fmin(frequency1, fmin(frequency2, frequency3));
          const double maximumFrequency =
              fmax(frequency1, fmax(frequency2, frequency3));

          // Reject a crossing set that is internally inconsistent. This stops
          // one noisy/false zero crossing from briefly commanding an off-tone
          // RF frequency. The 10 Hz allowance is deliberately much larger than
          // normal GFSK movement over only three audio cycles.
          if ((maximumFrequency - minimumFrequency) <=
              TX_MAX_THREE_CYCLE_SPREAD_HZ) {
            publishAudioFrequency(averageFrequency);
          }
        }

        // Reuse the fourth crossing as the first crossing of the next result.
        detector.crossingsQ16[0] = detector.crossingsQ16[3];
        detector.crossingCount = 1;
      }
    }

    detector.zeroCrossArmed = false;
  }

  detector.previousSample = sample;
}

// ---------------------------------------------------------------------------
// USB playback receiver: core 0 reads packets, core 1 measures frequency
// ---------------------------------------------------------------------------
void receiveUsbPlayback() {
  const bool streaming = usbAudio.isStreamingActiveRx();

  // CW terminal mode owns transmit control.  Keep the USB playback endpoint
  // serviced so a host cannot wedge it, but never allow playback audio to arm
  // the FT8 VOX detector while CW mode is selected.
  if (cwTerminalMode) {
    usbPlaybackStreaming = false;
    if (!streaming) return;

    const size_t bytesPerFrame = CHANNELS * sizeof(int16_t);
    for (uint8_t block = 0; block < USB_AUDIO_BLOCKS_PER_PASS; ++block) {
      const int available = usbAudio.available();
      if (available < static_cast<int>(bytesPerFrame)) break;
      size_t wanted = sizeof(usbOutputBlock);
      if (static_cast<size_t>(available) < wanted) {
        wanted = static_cast<size_t>(available);
      }
      wanted -= wanted % bytesPerFrame;
      if (wanted == 0) break;
      usbAudio.readBytes(reinterpret_cast<uint8_t *>(usbOutputBlock), wanted);
    }
    return;
  }

  usbPlaybackStreaming = streaming;
  if (!streaming) return;

  const size_t bytesPerFrame = CHANNELS * sizeof(int16_t);

  // Do only a bounded amount of playback work per pass. This is deliberately
  // symmetrical with sendAdcToUsb(): CAT/PTT must get CPU time even if the PC
  // is continuously streaming audio in both directions.
  for (uint8_t block = 0; block < USB_AUDIO_BLOCKS_PER_PASS; ++block) {
    const int available = usbAudio.available();
    if (available < static_cast<int>(bytesPerFrame)) break;

    size_t wanted = sizeof(usbOutputBlock);
    if (static_cast<size_t>(available) < wanted) {
      wanted = static_cast<size_t>(available);
    }
    wanted -= wanted % bytesPerFrame;
    if (wanted == 0) break;

    setWatchdogBreadcrumb(WD_STAGE_USB_READ, static_cast<uint32_t>(wanted));
    const size_t received = usbAudio.readBytes(
        reinterpret_cast<uint8_t *>(usbOutputBlock), wanted);
    if (received == 0) break;

    lastUsbPacketMicros = micros();
    const size_t frames = received / bytesPerFrame;

    for (size_t i = 0; i < frames; ++i) {
      trackerPush(usbOutputBlock[i * 2]);  // left channel only
    }
  }
}


// ---------------------------------------------------------------------------
// TX/RX clock and frequency service on core 0
// ---------------------------------------------------------------------------
bool audioTxRequested() {
  return requestedVoxTx && usbPlaybackStreaming;
}

bool combinedTxRequested() {
  // CAT PTT is retained for host compatibility/status, but it is deliberately
  // NOT sufficient to create RF. Actual RF follows valid outgoing USB audio,
  // TinyDX-style, via audioTxRequested().
  return catTxRequested || audioTxRequested();
}

void applyDialFrequency(uint32_t frequencyHz) {
  if (bandIndexForFrequency(frequencyHz) < 0) return;

  dialFrequencyHz = frequencyHz;
  ++dialFrequencyVersion;
  rxRetunePending = true;
  if (!benchTestModeActive) applyBandOutputs(frequencyHz);
}

void serviceClockGenerator() {
  static bool txModeActive = false;
  static bool txDriversActive = false;
  static uint32_t appliedAudioVersion = 0;
  static uint32_t appliedDialVersion = 0;
  static bool benchTestApplied = false;
  static uint32_t appliedBenchTestVersion = 0;
  static bool cwModeApplied = false;
  static bool cwSessionApplied = false;
  static bool cwClockActive = false;
  static bool cwCarrierActive = false;
  static uint32_t appliedCwRfVersion = 0;

  serviceCwKeyer();

  const uint32_t audioVersion = frequencyRequestVersion;
  const uint32_t dialVersion = dialFrequencyVersion;
  tinyDxMemoryBarrier();

  const double audioHz = static_cast<double>(requestedAudioMilliHz) / 1000.0;
  const bool audioValid =
      audioHz >= MIN_AUDIO_FREQUENCY_HZ && audioHz <= MAX_AUDIO_FREQUENCY_HZ;
  const bool txNow = audioTxRequested();

  // -----------------------------------------------------------------------
  // BENCH TEST MODE
  // -----------------------------------------------------------------------
  // Test mode owns the RF hardware completely. Normal CAT/VOX requests may
  // still be received, but they cannot key or modulate RF until TEST EXIT.
  if (benchTestModeActive) {
    if (!benchTestApplied) {
      // Break safely away from whatever normal RX/TX state was active.
      rxAudioBlankingActive = true;
      rxAudioUnblankAtUs = 0;
      tinyDxMemoryBarrier();

      si5351EnableClk0(false);
      si5351EnableClk1(false);
      setRxPathEnabled(false);
      setTxDriversEnabled(false);
      setTxLed(false);

      txModeActive = false;
      txDriversActive = false;
      txFrequencyReady = false;
      currentAudioFrequencyHz = 0.0;
      currentTxFrequencyHz = 0.0;
      benchTestApplied = true;
      appliedBenchTestVersion = 0;  // force application below
      delayMicroseconds(RF_SWITCH_SETTLE_US);
    }

    if (appliedBenchTestVersion != benchTestRequestVersion) {
      // Every selection or ON/OFF change is break-before-make. Blank audio only
      // while the selected 4*F receive clock is being rebuilt.
      rxAudioBlankingActive = true;
      rxAudioUnblankAtUs = 0;
      tinyDxMemoryBarrier();

      si5351EnableClk0(false);
      si5351EnableClk1(false);
      setRxPathEnabled(false);
      setTxLed(false);
      setTxDriversEnabled(false);
      txDriversActive = false;
      txFrequencyReady = false;
      currentAudioFrequencyHz = 0.0;
      currentTxFrequencyHz = 0.0;
      delayMicroseconds(RF_SWITCH_SETTLE_US);

      applyBenchTestLedOutputs();

      // Keep the QSD running at the selected test frequency in both RX and TX.
      // RXSW remains disabled below whenever the fixed carrier is active.
      const bool testRxReady = si5351ConfigureBenchRxClock();
      const bool testQsdRunning = testRxReady && si5351EnableClk0(true);

      if (benchTestCarrierRequested) {
        // si5351ConfigureTxClock() takes FINAL RF. It programs CLK1 to 2*RF,
        // and U9 divides that by two to create the actual test carrier.
        txFrequencyReady = si5351ConfigureTxClock(benchTestRfFrequencyHz);
        if (txFrequencyReady) {
          setTxDriversEnabled(true);
          delayMicroseconds(RF_SWITCH_SETTLE_US);
          if (si5351EnableClk1(true)) {
            setTxLed(true);
            txDriversActive = true;
            currentTxFrequencyHz = benchTestRfFrequencyHz;
          } else {
            setTxDriversEnabled(false);
            setTxLed(false);
            txDriversActive = false;
            txFrequencyReady = false;
          }
        }

        // QSD/USB audio remains live during fixed-carrier TX, but the receive
        // RF input FET stays disconnected from the antenna.
        setRxPathEnabled(false);
        if (testQsdRunning) {
          rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
        }
      } else {
        // TX OFF in test mode reconnects the receiver at the selected test
        // frequency. The special 10 MHz profile uses CLK0=40 MHz divided by 4.
        if (testQsdRunning) {
          delayMicroseconds(RF_SWITCH_SETTLE_US);
          setRxPathEnabled(true);
          rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
        } else {
          setRxPathEnabled(false);
          rxAudioUnblankAtUs = 0;
        }
      }

      appliedBenchTestVersion = benchTestRequestVersion;
    }

    // Release QSD/USB audio after the clock-settling interval in both RX and
    // fixed-carrier TX. RXSW itself remains off throughout carrier transmission.
    if (rxAudioBlankingActive && rxClockConfigured && rxClockEnabled &&
        rxAudioUnblankAtUs != 0 &&
        static_cast<int32_t>(micros() - rxAudioUnblankAtUs) >= 0) {
      rxAudioBlankingActive = false;
      rxAudioUnblankAtUs = 0;
      tinyDxMemoryBarrier();
    }
    return;
  }

  // TEST EXIT: shut down the fixed carrier first, restore the saved normal
  // dial-band LEDs, rebuild the 4*F RX clock, reconnect RX, then retain the
  // existing 2 ms USB-audio recovery blanking.
  if (benchTestApplied) {
    si5351EnableClk1(false);
    setTxLed(false);
    setTxDriversEnabled(false);
    setRxPathEnabled(false);
    txDriversActive = false;
    txModeActive = false;
    txFrequencyReady = false;
    currentAudioFrequencyHz = 0.0;
    currentTxFrequencyHz = 0.0;
    delayMicroseconds(RF_SWITCH_SETTLE_US);

    applyBandOutputs(dialFrequencyHz);
    rxAudioBlankingActive = true;
    rxAudioUnblankAtUs = 0;
    rxRetunePending = true;
    tinyDxMemoryBarrier();

    const bool rxReady = si5351ConfigureRxClock(dialFrequencyHz);
    if (rxReady && si5351EnableClk0(true)) {
      delayMicroseconds(RF_SWITCH_SETTLE_US);
      setRxPathEnabled(true);
      rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
    } else {
      setRxPathEnabled(false);
      rxAudioUnblankAtUs = 0;
    }

    benchTestApplied = false;
    appliedBenchTestVersion = benchTestRequestVersion;
    tinyDxMemoryBarrier();
    return;
  }

  // -----------------------------------------------------------------------
  // CW TERMINAL MODE
  // -----------------------------------------------------------------------
  // CW mode owns RF while selected.  FT8 USB playback is ignored by the VOX
  // path, the receive LO sits cwRxToneHz below the displayed/carrier frequency,
  // and CLK1/TXSW are keyed only during Morse marks.
  if (cwTerminalMode) {
    if (!cwModeApplied) {
      rxAudioBlankingActive = true;
      rxAudioUnblankAtUs = 0;
      tinyDxMemoryBarrier();

      cwTxEnvelopeHardwareReady = false;
      cwEnvelopeMarkActive = false;
      tinyDxMemoryBarrier();
      si5351EnableClk1(false);
      setTxLed(false);
      setTxDriversEnabled(false);
      setRxPathEnabled(false);
      cwReceiveRfActive = false;

      txModeActive = false;
      txDriversActive = false;
      txFrequencyReady = false;
      currentAudioFrequencyHz = 0.0;
      currentTxFrequencyHz = 0.0;
      cwClockActive = false;
      cwCarrierActive = false;
      cwSessionApplied = false;
      cwModeApplied = true;
      appliedCwRfVersion = 0;
      rxRetunePending = true;
      delayMicroseconds(RF_SWITCH_SETTLE_US);
    }

    const bool cwSessionNow = cwKeyerSessionRequested();
    const bool cwRfChanged =
        appliedCwRfVersion != cwRfRequestVersion ||
        appliedDialVersion != dialVersion;

    if (cwRfChanged) {
      // Break before changing either RX or TX frequency.
      cwTxEnvelopeHardwareReady = false;
      cwEnvelopeMarkActive = false;
      tinyDxMemoryBarrier();
      si5351EnableClk1(false);
      setTxLed(false);
      setTxDriversEnabled(false);
      setRxPathEnabled(false);
      cwReceiveRfActive = false;
      si5351EnableClk0(false);
      cwClockActive = false;
      cwCarrierActive = false;
      cwSessionApplied = false;
      txDriversActive = false;
      txFrequencyReady = false;
      currentTxFrequencyHz = 0.0;

      rxAudioBlankingActive = true;
      rxAudioUnblankAtUs = 0;
      tinyDxMemoryBarrier();
      delayMicroseconds(RF_SWITCH_SETTLE_US);

      const uint32_t cwRxLoHz = cwReceiveLoFrequencyHz();
      const bool rxReady =
          cwRxLoHz != 0 && si5351ConfigureRxClock(cwRxLoHz);
      const bool qsdRunning = rxReady && si5351EnableClk0(true);

      // Pre-program the fixed CW carrier while the TX drivers are disabled.
      // Marks then require only TXSW/CLK1 gating, not an I2C frequency write.
      txFrequencyReady =
          si5351ConfigureTxClock(static_cast<double>(dialFrequencyHz));

      if (qsdRunning && !cwSessionNow) {
        delayMicroseconds(RF_SWITCH_SETTLE_US);
        setRxPathEnabled(true);
        cwReceiveRfActive = true;
        rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
      } else {
        setRxPathEnabled(false);
        cwReceiveRfActive = false;
        if (qsdRunning) {
          rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
        }
      }

      appliedCwRfVersion = cwRfRequestVersion;
      appliedDialVersion = dialVersion;
      rxRetunePending = false;
    }

    // Enter a CW transmit session before the first mark.  CLK1 runs
    // continuously behind the disabled ACT244 outputs for the duration of the
    // message; the actual Morse envelope is keyed only with TXSW.  This keeps
    // dit/dah timing independent of I2C transaction latency.
    if (cwSessionNow && !cwSessionApplied) {
      rxAudioBlankingActive = true;
      rxAudioUnblankAtUs = 0;
      tinyDxMemoryBarrier();

      setRxPathEnabled(false);
      cwReceiveRfActive = false;
      setTxDriversEnabled(false);
      setTxLed(false);

      if (!txFrequencyReady) {
        si5351EnableClk1(false);
        txFrequencyReady =
            si5351ConfigureTxClock(static_cast<double>(dialFrequencyHz));
      }

      if (txFrequencyReady && si5351EnableClk1(true)) {
        cwClockActive = true;
        cwTxEnvelopeHardwareReady = true;
        tinyDxMemoryBarrier();
      } else {
        cwClockActive = false;
        cwTxEnvelopeHardwareReady = false;
        txFrequencyReady = false;
      }

      delayMicroseconds(RF_SWITCH_SETTLE_US);
      cwSessionApplied = true;
    }

    // CW RF amplitude is owned by the 10 kHz envelope timer.  It PWM-controls
    // the active-low TXSW OE only during the first/last 5 ms of each mark; the
    // middle of the mark is static LOW (fully enabled), and gaps are static
    // HIGH (fully disabled).  Main-loop state here is therefore LED/status only.
    if (cwKeyDown && cwClockActive) {
      if (!cwCarrierActive) {
        setTxLed(true);
        cwCarrierActive = true;
        txDriversActive = true;
        currentTxFrequencyHz = static_cast<double>(dialFrequencyHz);
      }
    } else if (cwCarrierActive) {
      setTxLed(false);
      cwCarrierActive = false;
      txDriversActive = false;
      currentTxFrequencyHz = 0.0;
    }

    // Once the keyer has observed the required trailing character/word gap and
    // has no queued text, return to receive.
    if (!cwSessionNow && cwSessionApplied) {
      cwTxEnvelopeHardwareReady = false;
      cwEnvelopeMarkActive = false;
      tinyDxMemoryBarrier();
      setTxDriversEnabled(false);
      if (cwCarrierActive) {
        setTxLed(false);
        cwCarrierActive = false;
        txDriversActive = false;
      }
      if (cwClockActive) {
        si5351EnableClk1(false);
        cwClockActive = false;
      }

      delayMicroseconds(RF_SWITCH_SETTLE_US);
      if (rxClockConfigured && rxClockEnabled) {
        setRxPathEnabled(true);
        cwReceiveRfActive = true;
        rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
      } else {
        cwReceiveRfActive = false;
      }
      cwSessionApplied = false;
    }

    if (rxAudioBlankingActive && rxClockConfigured && rxClockEnabled &&
        rxAudioUnblankAtUs != 0 &&
        static_cast<int32_t>(micros() - rxAudioUnblankAtUs) >= 0) {
      rxAudioBlankingActive = false;
      rxAudioUnblankAtUs = 0;
      tinyDxMemoryBarrier();
    }
    return;
  }

  // CW -> normal CAT/FT8 transition.
  if (cwModeApplied) {
    cwTxEnvelopeHardwareReady = false;
    cwEnvelopeMarkActive = false;
    tinyDxMemoryBarrier();
    si5351EnableClk1(false);
    setTxLed(false);
    setTxDriversEnabled(false);
    setRxPathEnabled(false);
    cwReceiveRfActive = false;
    cwClockActive = false;
    cwCarrierActive = false;
    cwSessionApplied = false;
    txDriversActive = false;
    txModeActive = false;
    txFrequencyReady = false;
    currentAudioFrequencyHz = 0.0;
    currentTxFrequencyHz = 0.0;
    delayMicroseconds(RF_SWITCH_SETTLE_US);

    rxAudioBlankingActive = true;
    rxAudioUnblankAtUs = 0;
    rxRetunePending = true;
    tinyDxMemoryBarrier();

    const bool rxReady = si5351ConfigureRxClock(dialFrequencyHz);
    if (rxReady && si5351EnableClk0(true)) {
      delayMicroseconds(RF_SWITCH_SETTLE_US);
      setRxPathEnabled(true);
      rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
    } else {
      setRxPathEnabled(false);
      rxAudioUnblankAtUs = 0;
    }

    cwModeApplied = false;
    appliedDialVersion = dialVersion;
    tinyDxMemoryBarrier();
    return;
  }

  if (dialVersion != appliedDialVersion) {
    appliedDialVersion = dialVersion;
    rxRetunePending = true;
  }

  // TinyDX-style RF authority: only outgoing USB audio can enter TX hardware
  // state. CAT PTT is acknowledged/reported for host compatibility, but cannot
  // disconnect RX or create a carrier by itself.
  if (txNow && !txModeActive) {
    // Keep the QSD clock and ADC-to-USB audio path live during TX. Only the
    // receive RF FET is opened to isolate the QSD input from the antenna.
    rxAudioBlankingActive = false;
    rxAudioUnblankAtUs = 0;
    tinyDxMemoryBarrier();

    setRxPathEnabled(false);
    delayMicroseconds(RF_SWITCH_SETTLE_US);

    setTxDriversEnabled(false);
    setTxLed(false);
    si5351EnableClk1(false);
    txFrequencyReady = false;
    txDriversActive = false;
    txModeActive = true;
  }

  if (txNow) {
    if (!audioValid) {
      // VOX has detected outgoing audio but the frequency estimator has not yet
      // produced a valid FT8-range tone (or has just lost it). Never emit a
      // dial-frequency carrier. Record this only if RF had actually been active.
      if (txDriversActive || txFrequencyReady) {
        lastRfStopReason = RF_STOP_AUDIO_INVALID;
        ++rfStopCount;
        ++audioInvalidRfStopCount;
      }
      si5351EnableClk1(false);
      setTxDriversEnabled(false);
      setTxLed(false);
      txDriversActive = false;
      txFrequencyReady = false;
      currentAudioFrequencyHz = 0.0;
      currentTxFrequencyHz = 0.0;
      appliedAudioVersion = audioVersion;
      return;
    }

    const double requiredRfHz =
        static_cast<double>(dialFrequencyHz) + audioHz;

    if (!txClockConfigured || !txDriversActive) {
      // Program CLK1 while the ACT244s are disabled, then enable the hardware
      // only after the 2*RF clock is ready and held LOW by output-disable.
      si5351EnableClk1(false);
      txFrequencyReady = si5351ConfigureTxClock(requiredRfHz);

      if (txFrequencyReady) {
        setTxDriversEnabled(true);
        delayMicroseconds(RF_SWITCH_SETTLE_US);
        if (si5351EnableClk1(true)) {
          setTxLed(true);  // LED means actual RF TX clock is active
          txDriversActive = true;
        } else {
          lastRfStopReason = RF_STOP_CLK_ENABLE_FAIL;
          ++rfStopCount;
          ++txClockFailureCount;
          setTxDriversEnabled(false);
          setTxLed(false);
          txDriversActive = false;
          txFrequencyReady = false;
        }
      } else {
        lastRfStopReason = RF_STOP_TX_CONFIG_FAIL;
        ++rfStopCount;
        ++txClockFailureCount;
      }
    } else if (audioVersion != appliedAudioVersion ||
               fabs(requiredRfHz - appliedTxFrequencyHz) > 0.001) {
      // Normal FT8 modulation path: one MS1 register-block update only.
      txFrequencyReady = si5351SetTxFrequencyFast(requiredRfHz);
      if (!txFrequencyReady) {
        lastRfStopReason = RF_STOP_FAST_RETUNE_FAIL;
        ++rfStopCount;
        ++fastRetuneFailureCount;
        si5351EnableClk1(false);
        setTxDriversEnabled(false);
        setTxLed(false);
        txDriversActive = false;
      }
    }

    appliedAudioVersion = audioVersion;
    if (txFrequencyReady && txDriversActive) {
      currentAudioFrequencyHz = audioHz;
      currentTxFrequencyHz = requiredRfHz;
    }
    return;
  }

  // TX -> RX transition. Stop RF first, disable ACT244s, and then reconnect the
  // receive RF FET. CLK0 and the QSD have continued running throughout TX. The
  // PLLA/MS0 RX plan is rebuilt only if CAT changed the dial during transmission.
  if (txModeActive) {
    if (txDriversActive) {
      lastRfStopReason = RF_STOP_REQUEST_ENDED;
      ++rfStopCount;
    }
    si5351EnableClk1(false);
    setTxLed(false);
    setTxDriversEnabled(false);
    txDriversActive = false;
    txModeActive = false;
    txFrequencyReady = false;
    currentAudioFrequencyHz = 0.0;
    currentTxFrequencyHz = 0.0;

    delayMicroseconds(RF_SWITCH_SETTLE_US);

    bool rxReady = rxClockConfigured && !rxRetunePending;
    if (!rxReady) rxReady = si5351ConfigureRxClock(dialFrequencyHz);

    if (rxReady && si5351EnableClk0(true)) {
      delayMicroseconds(RF_SWITCH_SETTLE_US);
      setRxPathEnabled(true);
      rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
    } else {
      setRxPathEnabled(false);
      rxAudioUnblankAtUs = 0;
    }
    tinyDxMemoryBarrier();
  }

  // CAT dial change while already receiving. Disconnect and blank RX around the
  // PLLA retune so the PLL reset / divider transition cannot appear in FT8CN.
  if (!txModeActive && rxRetunePending) {
    rxAudioBlankingActive = true;
    rxAudioUnblankAtUs = 0;
    tinyDxMemoryBarrier();

    setRxPathEnabled(false);
    si5351EnableClk0(false);
    delayMicroseconds(RF_SWITCH_SETTLE_US);

    if (si5351ConfigureRxClock(dialFrequencyHz) && si5351EnableClk0(true)) {
      delayMicroseconds(RF_SWITCH_SETTLE_US);
      setRxPathEnabled(true);
      rxAudioUnblankAtUs = micros() + RX_AUDIO_RECOVERY_US;
    } else {
      setRxPathEnabled(false);
      rxAudioUnblankAtUs = 0;
    }
  }

  // Release USB RX audio only after the valid 4*F CLK0 has been divided to the
  // QSD quadrature clocks, RXSW is back on, and the short analogue recovery time
  // has elapsed. Signed subtraction is safe across micros() wraparound.
  if (!txNow && rxAudioBlankingActive && rxClockConfigured && rxClockEnabled &&
      rxAudioUnblankAtUs != 0 &&
      static_cast<int32_t>(micros() - rxAudioUnblankAtUs) >= 0) {
    rxAudioBlankingActive = false;
    rxAudioUnblankAtUs = 0;
    tinyDxMemoryBarrier();
  }
}

// ---------------------------------------------------------------------------
// I/Q calibration control and solver
// ---------------------------------------------------------------------------
const char *iqCalibrationFailText(IqCalibrationFailReason reason) {
  switch (reason) {
    case IQCAL_FAIL_BUSY: return "BUSY";
    case IQCAL_FAIL_TUNE_CHANGED: return "TUNE";
    case IQCAL_FAIL_SIGNAL_LOW: return "LOW-SIG";
    case IQCAL_FAIL_WRONG_SIDEBAND: return "SIDE/POL";
    case IQCAL_FAIL_MATH: return "MATH";
    case IQCAL_FAIL_RANGE: return "RANGE";
    case IQCAL_FAIL_REJECTION: return "REJECT";
    case IQCAL_FAIL_SAVE: return "SAVE";
    default: return "NONE";
  }
}

void restoreIqCalibrationRxSetup() {
  setRxSidebandMode(iqCalSavedSidebandMode);
  applyDialFrequency(iqCalSavedDialFrequencyHz);
}

void failIqCalibration(IqCalibrationFailReason reason) {
  iqCalFailReason = reason;
  iqCalState = IQCAL_DONE_FAIL;
  restoreIqCalibrationRxSetup();
}

void startIqCalibration() {
  iqCalSavedDialFrequencyHz = dialFrequencyHz;
  iqCalSavedSidebandMode = rxSidebandMode;
  iqCalFailReason = IQCAL_FAIL_NONE;
  resetIqCalibrationAccumulator();

  // The fixture is intentionally fixed so there is no opportunity to calibrate
  // against the wrong sideband by typing an arbitrary test frequency.
  setRxSidebandMode(RX_MODE_USB);
  applyDialFrequency(IQCAL_RX_FREQUENCY_HZ);
  resetRxSidebandDsp();
  iqCalArmAtMs = millis() + IQCAL_ARM_DELAY_MS;
  iqCalState = IQCAL_ARMING;
}

void solveIqCalibration() {
  if (iqCalState != IQCAL_SOLVE_PENDING || iqCalSamples == 0) return;

  const float n = static_cast<float>(iqCalSamples);
  const float qq = iqCalSumQQ / n;
  const float ii = iqCalSumII / n;
  const float hh = iqCalSumHH / n;
  const float qi = iqCalSumQI / n;
  const float qh = iqCalSumQH / n;
  const float ih = iqCalSumIH / n;

  const float signalRmsAdcCounts = sqrtf(fmaxf(hh, 0.0f)) * 2048.0f;
  if (!isfinite(signalRmsAdcCounts) ||
      signalRmsAdcCounts < IQCAL_MIN_RMS_ADC_COUNTS) {
    failIqCalibration(IQCAL_FAIL_SIGNAL_LOW);
    return;
  }

  const float corrDen = sqrtf(fmaxf(qq * hh, 0.0f));
  if (!(corrDen > 1.0e-12f)) {
    failIqCalibration(IQCAL_FAIL_MATH);
    return;
  }
  const float rawUsbCorrelation = qh / corrDen;
  if (!isfinite(rawUsbCorrelation) ||
      fabsf(rawUsbCorrelation) < IQCAL_MIN_USB_CORRELATION) {
    // Weak correlation means the expected 1 kHz calibration carrier is missing,
    // too noisy, or not producing a clean quadrature relationship. A negative
    // correlation by itself is NOT an error: with the fixed +1 kHz fixture it
    // simply tells us that this board's physical I/Q rotation sense is opposite.
    failIqCalibration(IQCAL_FAIL_WRONG_SIDEBAND);
    return;
  }

  // Learn physical quadrature polarity from the fixed ABOVE-LO (+1 kHz) carrier.
  // qh/ih were accumulated using the currently selected polarity. If correlation
  // is negative, flip the Hilbert target for this solve and remember that flip
  // so normal USB/LSB demodulation uses the correct physical sense afterwards.
  const float polarityFlip = (rawUsbCorrelation >= 0.0f) ? 1.0f : -1.0f;
  const float solvedUsbPolarity = rxIqUsbPolarity * polarityFlip;
  const float qhUsb = qh * polarityFlip;
  const float ihUsb = ih * polarityFlip;

  // Least-squares solution for H(I)_USB ~= a*q + b*i.
  // a is the relative Q gain correction and b is the residual phase/quadrature
  // correction. Solving from second-order statistics avoids storing a second
  // of raw 48 kHz samples in RAM.
  const float det = qq * ii - qi * qi;
  if (!isfinite(det) || det <= 1.0e-12f) {
    failIqCalibration(IQCAL_FAIL_MATH);
    return;
  }

  const float qGain = (qhUsb * ii - ihUsb * qi) / det;
  const float qMix = (ihUsb * qq - qhUsb * qi) / det;
  if (!iqCalibrationValuesSane(qGain, qMix, solvedUsbPolarity)) {
    failIqCalibration(IQCAL_FAIL_RANGE);
    return;
  }

  // Predict wanted USB and rejected LSB power from the same captured data.
  const float common = qGain * qGain * qq + qMix * qMix * ii + hh +
                       2.0f * qGain * qMix * qi;
  const float cross = 2.0f * qGain * qhUsb + 2.0f * qMix * ihUsb;
  float wantedPower = 0.25f * (common + cross);
  float imagePower = 0.25f * (common - cross);
  wantedPower = fmaxf(wantedPower, 1.0e-15f);
  imagePower = fmaxf(imagePower, 1.0e-15f);
  float imageRejectionDb = 10.0f * log10f(wantedPower / imagePower);
  if (!isfinite(imageRejectionDb)) {
    failIqCalibration(IQCAL_FAIL_MATH);
    return;
  }
  if (imageRejectionDb > 120.0f) imageRejectionDb = 120.0f;
  if (imageRejectionDb < IQCAL_MIN_IMAGE_REJECTION_DB) {
    failIqCalibration(IQCAL_FAIL_REJECTION);
    return;
  }

  rxIqQGain = qGain;
  rxIqQPhaseMix = qMix;
  rxIqUsbPolarity = solvedUsbPolarity;
  iqCalLastImageRejectionDb = imageRejectionDb;
  iqCalLastSignalRmsAdcCounts = signalRmsAdcCounts;
  resetRxSidebandDsp();

  if (!saveIqCalibrationToFlash()) {
    iqCalFailReason = IQCAL_FAIL_SAVE;
    iqCalState = IQCAL_DONE_FAIL;
  } else {
    iqCalFailReason = IQCAL_FAIL_NONE;
    iqCalState = IQCAL_DONE_OK;
  }
  restoreIqCalibrationRxSetup();
}

void serviceIqCalibration() {
  if (iqCalState == IQCAL_ARMING || iqCalState == IQCAL_CAPTURING) {
    // Do not save a calibration if anything changes the controlled fixture
    // while the one-second measurement is in progress.
    if (benchTestModeActive || cwTerminalMode || combinedTxRequested()) {
      failIqCalibration(IQCAL_FAIL_BUSY);
      return;
    }
    if (dialFrequencyHz != IQCAL_RX_FREQUENCY_HZ ||
        rxSidebandMode != RX_MODE_USB) {
      failIqCalibration(IQCAL_FAIL_TUNE_CHANGED);
      return;
    }
  }

  if (iqCalState == IQCAL_SOLVE_PENDING) solveIqCalibration();
}

// ---------------------------------------------------------------------------
// Minimal Kenwood TS-480 CAT control
// ---------------------------------------------------------------------------
static constexpr uint8_t CAT_CDC_INSTANCE = 0;
static constexpr size_t CAT_COMMAND_BUFFER_SIZE = 32;
char catCommandBuffer[CAT_COMMAND_BUFFER_SIZE];
size_t catCommandLength = 0;
char catReplyTerminator = ';';

bool commandIs(const char *command, size_t length, const char *text) {
  const size_t textLength = strlen(text);
  return length == textLength &&
         memcmp(command, text, textLength) == 0;
}

bool parseCatFrequency11(const char *digits, uint32_t &frequencyHz) {
  uint64_t value = 0;
  for (size_t i = 0; i < 11; ++i) {
    const char c = digits[i];
    if (c < '0' || c > '9') return false;
    value = value * 10ULL + static_cast<uint64_t>(c - '0');
  }

  if (value > 0xFFFFFFFFULL) return false;

  const uint32_t candidateHz = static_cast<uint32_t>(value);
  if (bandIndexForFrequency(candidateHz) < 0) return false;

  frequencyHz = candidateHz;
  return true;
}

void catWriteRaw(const char *text) {
  const size_t length = strlen(text);
  if (length == 0 || length >= 64) return;

  char reply[64];
  memcpy(reply, text, length + 1);
  if (reply[length - 1] == ';') {
    reply[length - 1] = catReplyTerminator;
  }

  const uint32_t written = tud_cdc_n_write(
      CAT_CDC_INSTANCE,
      reinterpret_cast<const uint8_t *>(reply),
      length);
  if (written != length) ++catShortWriteCount;
  tud_cdc_n_write_flush(CAT_CDC_INSTANCE);
  TinyUSB_Device_FlushCDC();
}

void cwTerminalWriteRaw(const char *text) {
  if (text == nullptr) return;
  const size_t length = strlen(text);
  if (length == 0) return;
  tud_cdc_n_write(
      CAT_CDC_INSTANCE,
      reinterpret_cast<const uint8_t *>(text),
      static_cast<uint32_t>(length));
}

bool parseUnsignedDecimal(const char *text, uint32_t &value) {
  if (text == nullptr) return false;
  while (*text == ' ' || *text == '\t') ++text;
  if (*text == '\0') return false;

  uint64_t accumulated = 0;
  bool haveDigit = false;
  while (*text >= '0' && *text <= '9') {
    haveDigit = true;
    accumulated = accumulated * 10ULL +
                  static_cast<uint64_t>(*text - '0');
    if (accumulated > 0xFFFFFFFFULL) return false;
    ++text;
  }

  while (*text == ' ' || *text == '\t') ++text;
  if (*text != '\0' || !haveDigit) return false;

  value = static_cast<uint32_t>(accumulated);
  return true;
}

// Parse the app's atomic CW selection command payload: <carrierHz>,<bfoHz>.
// Keeping both values in one CDC command lets the firmware apply them in one
// RF service pass, avoiding a transient RX-LO step while the user releases the
// draggable waterfall filter window.
bool parseUnsignedDecimalPair(const char *text, uint32_t &first, uint32_t &second) {
  if (text == nullptr) return false;
  while (*text == ' ' || *text == '\t') ++text;

  uint64_t a = 0;
  bool haveA = false;
  while (*text >= '0' && *text <= '9') {
    haveA = true;
    a = a * 10ULL + static_cast<uint64_t>(*text - '0');
    if (a > 0xFFFFFFFFULL) return false;
    ++text;
  }
  while (*text == ' ' || *text == '\t') ++text;
  if (!haveA || *text != ',') return false;
  ++text;
  while (*text == ' ' || *text == '\t') ++text;

  uint64_t b = 0;
  bool haveB = false;
  while (*text >= '0' && *text <= '9') {
    haveB = true;
    b = b * 10ULL + static_cast<uint64_t>(*text - '0');
    if (b > 0xFFFFFFFFULL) return false;
    ++text;
  }
  while (*text == ' ' || *text == '\t') ++text;
  if (!haveB || *text != '\0') return false;

  first = static_cast<uint32_t>(a);
  second = static_cast<uint32_t>(b);
  return true;
}

void sendIqCalibrationStatus() {
  char response[96];
  if (iqCalState == IQCAL_ARMING) {
    snprintf(response, sizeof(response), "IQCAL ARM 14074000/14075000;");
  } else if (iqCalState == IQCAL_CAPTURING) {
    snprintf(response, sizeof(response), "IQCAL RUN %lu/%lu;",
             static_cast<unsigned long>(iqCalSamples),
             static_cast<unsigned long>(IQCAL_SAMPLE_TARGET));
  } else if (iqCalState == IQCAL_SOLVE_PENDING) {
    snprintf(response, sizeof(response), "IQCAL SOLVE;");
  } else if (iqCalState == IQCAL_DONE_FAIL) {
    snprintf(response, sizeof(response), "IQCAL FAIL %s;",
             iqCalibrationFailText(iqCalFailReason));
  } else if ((iqCalibrationStored || iqCalState == IQCAL_DONE_OK) &&
             iqCalLastImageRejectionDb > 0.0f) {
    snprintf(response, sizeof(response),
             "IQCAL OK POL=%c QG=%.5f QM=%+.5f IR=%.1f R=%.1f;",
             (rxIqUsbPolarity >= 0.0f ? '+' : '-'),
             static_cast<double>(rxIqQGain),
             static_cast<double>(rxIqQPhaseMix),
             static_cast<double>(iqCalLastImageRejectionDb),
             static_cast<double>(iqCalLastSignalRmsAdcCounts));
  } else {
    snprintf(response, sizeof(response),
             "IQCAL DEFAULT POL=+ QG=1.00000 QM=+0.00000;");
  }
  catWriteRaw(response);
}

bool handleIqCalibrationCommand(const char *command, size_t length) {
  if (commandIs(command, length, "IQCAL?")) {
    sendIqCalibrationStatus();
    return true;
  }

  if (commandIs(command, length, "IQCAL RESET")) {
    if (iqCalibrationActive()) {
      catWriteRaw("IQCAL BUSY;");
      return true;
    }
    setDefaultIqCalibration();
    resetRxSidebandDsp();
    iqCalFailReason = IQCAL_FAIL_NONE;
    iqCalState = IQCAL_IDLE;
    if (!saveIqCalibrationToFlash()) {
      iqCalFailReason = IQCAL_FAIL_SAVE;
      iqCalState = IQCAL_DONE_FAIL;
    }
    sendIqCalibrationStatus();
    return true;
  }

  if (commandIs(command, length, "IQCAL")) {
    if (iqCalibrationActive()) {
      sendIqCalibrationStatus();
      return true;
    }
    if (benchTestModeActive || cwTerminalMode || combinedTxRequested()) {
      catWriteRaw("IQCAL BUSY TX/TEST/CW;");
      return true;
    }
    startIqCalibration();
    catWriteRaw("IQCAL START SIG=14075000Hz ~-60dBm;");
    return true;
  }

  if (length >= 5 && memcmp(command, "IQCAL", 5) == 0) {
    catWriteRaw("IQCAL ?;");
    return true;
  }
  return false;
}

static constexpr size_t CW_COMMAND_BUFFER_SIZE = 32;
char cwCommandBuffer[CW_COMMAND_BUFFER_SIZE] = {};
size_t cwCommandLength = 0;
bool cwCommandActive = false;
bool cwPreviousByteWasCr = false;

void sendCwTerminalStatus() {
  char response[192];
  const uint16_t queued = static_cast<uint16_t>(
      (cwTxHead - cwTxTail) & CW_TX_QUEUE_MASK);

  snprintf(response, sizeof(response),
           "\r\n[CW F=%lu Hz TX=%u WPM SHAPE=5ms BFO=%u Hz RX=USB BUF=%u/%u]\r\n",
           static_cast<unsigned long>(dialFrequencyHz),
           static_cast<unsigned>(cwWpm),
           static_cast<unsigned>(cwRxToneHz),
           static_cast<unsigned>(queued),
           static_cast<unsigned>(CW_TX_QUEUE_SIZE - 1));
  cwTerminalWriteRaw(response);
}

void sendCwTerminalHelp() {
  cwTerminalWriteRaw(
      "\r\n[CW terminal: text=TX  %F<Hz> carrier  %T<Hz> BFO(100..7000)  %C<Hz>,<BFO> atomic select  %W<WPM>  %S=status  %Q=exit; RX DSP/decode runs in app]\r\n");
}

void enterCwTerminalMode() {
  // If bench mode was active, request a safe TEST EXIT first.  The RF service
  // performs that break-before-make transition before it applies CW mode.
  if (benchTestModeActive) {
    benchTestCarrierRequested = false;
    benchTestModeActive = false;
    ++benchTestRequestVersion;
  }

  catTxRequested = false;
  rxModeBeforeCw = rxSidebandMode;
  setRxSidebandMode(RX_MODE_USB);  // CW LO is carrier-BFO, so wanted CW is the USB side.
  cwTerminalMode = true;
  cwCommandActive = false;
  cwCommandLength = 0;
  cwCommandBuffer[0] = '\0';
  // If %CW was terminated by CR/LF, suppress the LF half of a CRLF pair after
  // the parser switches modes. A standalone LF is harmless: the flag clears
  // as soon as the next non-LF terminal character arrives.
  cwPreviousByteWasCr = (catReplyTerminator == '\r');
  resetCwKeyer(true);
  ++cwRfRequestVersion;
  tinyDxMemoryBarrier();

  cwTerminalWriteRaw("\r\n[CW TERMINAL ON]\r\n");
  sendCwTerminalStatus();
  sendCwTerminalHelp();
}

void exitCwTerminalMode() {
  resetCwKeyer(true);
  cwTerminalMode = false;
  setRxSidebandMode(rxModeBeforeCw);
  ++cwRfRequestVersion;
  tinyDxMemoryBarrier();
  cwTerminalWriteRaw("\r\n[CW TERMINAL OFF - CAT/FT8 MODE]\r\n");
}

void executeCwTerminalCommand(const char *command) {
  if (command == nullptr || command[0] == '\0') {
    sendCwTerminalHelp();
    return;
  }

  if (strcmp(command, "?") == 0) {
    sendCwTerminalHelp();
    return;
  }

  if (strcmp(command, "S") == 0) {
    sendCwTerminalStatus();
    return;
  }


  if (strcmp(command, "Q") == 0) {
    exitCwTerminalMode();
    return;
  }

  if (command[0] == 'C') {
    if (command[1] == '\0') {
      sendCwTerminalStatus();
      return;
    }

    uint32_t frequencyHz = 0;
    uint32_t requestedToneHz = 0;
    if (!parseUnsignedDecimalPair(command + 1, frequencyHz, requestedToneHz) ||
        requestedToneHz < CW_MIN_RX_TONE_HZ ||
        requestedToneHz > CW_MAX_RX_TONE_HZ ||
        bandIndexForFrequency(frequencyHz) < 0 ||
        frequencyHz <= requestedToneHz ||
        bandIndexForFrequency(
            frequencyHz - static_cast<uint32_t>(requestedToneHz)) < 0) {
      cwTerminalWriteRaw("\r\n[CW ?C - use %C<carrierHz>,<BFO 100..7000>; TX and RX LO must remain in 40/30/20/17 m]\r\n");
      return;
    }

    // Apply carrier and BFO as one logical RF change.  The Android waterfall
    // normally changes both by the same delta, which keeps RX LO = F-BFO
    // unchanged while TX follows the selected CW trace.
    cwRxToneHz = static_cast<uint16_t>(requestedToneHz);
    applyDialFrequency(frequencyHz);
    ++cwRfRequestVersion;
    sendCwTerminalStatus();
    return;
  }

  if (command[0] == 'F') {
    if (command[1] == '\0') {
      sendCwTerminalStatus();
      return;
    }

    uint32_t frequencyHz = 0;
    if (!parseUnsignedDecimal(command + 1, frequencyHz) ||
        bandIndexForFrequency(frequencyHz) < 0 ||
        frequencyHz <= static_cast<uint32_t>(cwRxToneHz) ||
        bandIndexForFrequency(
            frequencyHz - static_cast<uint32_t>(cwRxToneHz)) < 0) {
      cwTerminalWriteRaw("\r\n[CW ?F - frequency must keep TX and RX LO inside 40/30/20/17 m]\r\n");
      return;
    }

    applyDialFrequency(frequencyHz);
    ++cwRfRequestVersion;
    sendCwTerminalStatus();
    return;
  }

  if (command[0] == 'W') {
    if (command[1] == '\0') {
      sendCwTerminalStatus();
      return;
    }

    uint32_t requestedWpm = 0;
    if (!parseUnsignedDecimal(command + 1, requestedWpm) ||
        requestedWpm < CW_MIN_WPM || requestedWpm > CW_MAX_WPM) {
      cwTerminalWriteRaw("\r\n[CW ?W - use 5..50 WPM]\r\n");
      return;
    }

    cwWpm = static_cast<uint16_t>(requestedWpm);
    sendCwTerminalStatus();
    return;
  }

  if (command[0] == 'T') {
    if (command[1] == '\0') {
      sendCwTerminalStatus();
      return;
    }

    uint32_t requestedToneHz = 0;
    if (!parseUnsignedDecimal(command + 1, requestedToneHz) ||
        requestedToneHz < CW_MIN_RX_TONE_HZ ||
        requestedToneHz > CW_MAX_RX_TONE_HZ ||
        dialFrequencyHz <= requestedToneHz ||
        bandIndexForFrequency(
            dialFrequencyHz - static_cast<uint32_t>(requestedToneHz)) < 0) {
      cwTerminalWriteRaw("\r\n[CW ?T - use 100..7000 Hz and keep RX LO inside the band]\r\n");
      return;
    }

    cwRxToneHz = static_cast<uint16_t>(requestedToneHz);
    ++cwRfRequestVersion;
    sendCwTerminalStatus();
    return;
  }

  cwTerminalWriteRaw("\r\n[CW ? - unknown command]\r\n");
  sendCwTerminalHelp();
}

void feedCwTerminalByte(char c) {
  // Most serial terminals send CR+LF.  Treat that pair as one logical Enter so
  // the LF after a local % command cannot become an accidental transmitted space.
  if (cwPreviousByteWasCr && c == '\n') {
    cwPreviousByteWasCr = false;
    return;
  }
  cwPreviousByteWasCr = (c == '\r');

  if (!cwCommandActive) {
    if (c == '%') {
      cwCommandActive = true;
      cwCommandLength = 0;
      cwCommandBuffer[0] = '\0';
      return;
    }

    if (c == '\b' || c == 0x7F) {
      cwTxQueueBackspace();
      return;
    }

    if (c == '\r' || c == '\n' || c == '\t') {
      // Treat terminal line breaks/tabs as a word separator.  Multiple spaces
      // collapse naturally to one Morse 7-dot word gap in the keyer.
      cwTxQueuePush(' ');
      return;
    }

    if (c >= 32 && c <= 126) {
      if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
      if (c == ' ' || cwPatternForCharacter(c) != nullptr) {
        if (!cwTxQueuePush(c)) {
          cwTerminalWriteRaw("\r\n[CW TX QUEUE FULL]\r\n");
        }
      }
    }
    return;
  }

  // A CW control command is terminated by Enter or ';'.  Commands are local
  // only; nothing between '%' and the terminator is transmitted as Morse.
  if (c == '\r' || c == '\n' || c == ';') {
    cwCommandBuffer[cwCommandLength] = '\0';
    executeCwTerminalCommand(cwCommandBuffer);
    cwCommandActive = false;
    cwCommandLength = 0;
    cwCommandBuffer[0] = '\0';
    return;
  }

  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');

  if (cwCommandLength < CW_COMMAND_BUFFER_SIZE - 1) {
    cwCommandBuffer[cwCommandLength++] = c;
    cwCommandBuffer[cwCommandLength] = '\0';
  } else {
    cwCommandActive = false;
    cwCommandLength = 0;
    cwCommandBuffer[0] = '\0';
    cwTerminalWriteRaw("\r\n[CW COMMAND TOO LONG]\r\n");
  }
}

void sendCatFrequency(char vfoLetter = 'A') {
  char response[16];
  snprintf(response, sizeof(response),
           "F%c%011lu;",
           vfoLetter,
           static_cast<unsigned long>(dialFrequencyHz));
  catWriteRaw(response);
}

void sendCatIfStatus() {
  char response[39];
  memset(response, '0', sizeof(response));

  response[0] = 'I';
  response[1] = 'F';

  char frequency[12];
  snprintf(frequency, sizeof(frequency), "%011lu",
           static_cast<unsigned long>(dialFrequencyHz));
  memcpy(response + 2, frequency, 11);

  memset(response + 13, ' ', 5);
  memcpy(response + 18, "+0000", 5);
  response[23] = '0';  // RIT off
  response[24] = '0';  // XIT off
  response[25] = '0';  // memory bank
  response[26] = '0';
  response[27] = '0';  // memory channel 00
  const bool statusTx = benchTestModeActive
      ? benchTestCarrierRequested
      : combinedTxRequested();
  response[28] = statusTx ? '1' : '0';
  response[29] = (rxSidebandMode == RX_MODE_LSB) ? '1' : '2';
  response[30] = '0';  // VFO A
  response[31] = '0';  // scan off
  response[32] = '0';  // simplex
  response[33] = '0';  // tone off
  response[34] = '0';
  response[35] = '0';  // tone number 00
  response[36] = ' ';
  response[37] = ';';
  response[38] = '\0';

  catWriteRaw(response);
}

const char *voxStopReasonText(uint8_t reason) {
  switch (reason) {
    case VOX_STOP_SILENCE: return "SILENCE";
    case VOX_STOP_USB_TIMEOUT: return "USBTO";
    case VOX_STOP_STREAM_ENDED: return "STREAM";
    default: return "NONE";
  }
}

const char *rfStopReasonText(uint8_t reason) {
  switch (reason) {
    case RF_STOP_REQUEST_ENDED: return "END";
    case RF_STOP_AUDIO_INVALID: return "AUDIO";
    case RF_STOP_TX_CONFIG_FAIL: return "TXCFG";
    case RF_STOP_CLK_ENABLE_FAIL: return "CLKEN";
    case RF_STOP_FAST_RETUNE_FAIL: return "RETUNE";
    case RF_STOP_TEST_MODE: return "TEST";
    default: return "NONE";
  }
}

const char *bootResetReasonText(uint8_t reason) {
  // RP2350-only build uses the Pico SDK watchdog directly.  Keep the diagnostic
  // compact: 4 means the previous reset was watchdog-caused, 1 means a normal
  // non-watchdog boot/reset.
  return (reason == 4) ? "WDT" : "PWR";
}

void sendDiagnosticState() {
  char response[64];
  snprintf(response, sizeof(response),
           "DBG VR=%s RR=%s CAT=%u VOX=%u AUD=%u I2C=%lu;",
           voxStopReasonText(lastVoxStopReason),
           rfStopReasonText(lastRfStopReason),
           catTxRequested ? 1u : 0u,
           audioTxRequested() ? 1u : 0u,
           requestedAudioMilliHz != 0 ? 1u : 0u,
           static_cast<unsigned long>(i2cFailureCount));
  catWriteRaw(response);
}

void sendDiagnosticCounters() {
  char response[64];
  snprintf(response, sizeof(response),
           "DBGC C=%lu U=%lu M=%lu T=%lu R=%lu F=%lu;",
           static_cast<unsigned long>(catCommandCount),
           static_cast<unsigned long>(catUnknownCount),
           static_cast<unsigned long>(catMeterQueryCount),
           static_cast<unsigned long>(catTxCommandCount),
           static_cast<unsigned long>(catRxCommandCount),
           static_cast<unsigned long>(rfStopCount));
  catWriteRaw(response);
}

void sendDiagnosticErrors() {
  char response[64];
  snprintf(response, sizeof(response),
           "DBGE I=%lu A=%lu C=%lu F=%lu W=%lu;",
           static_cast<unsigned long>(i2cFailureCount),
           static_cast<unsigned long>(audioInvalidRfStopCount),
           static_cast<unsigned long>(txClockFailureCount),
           static_cast<unsigned long>(fastRetuneFailureCount),
           static_cast<unsigned long>(catShortWriteCount));
  catWriteRaw(response);
}

void sendWatchdogDiagnostics() {
  const uint32_t now = millis();
  const uint32_t age = trackerCoreReady
      ? static_cast<uint32_t>(now - core1HeartbeatMs)
      : 0xFFFFFFFFUL;
  char response[64];
  const uint16_t oldStage = previousWatchdogStage();
  snprintf(response, sizeof(response),
           "DBGW B=%s S=%s X=%08lX H=%lu R=%u;",
           bootResetReasonText(bootResetReason),
           watchdogStageText(oldStage),
           static_cast<unsigned long>(previousWatchdogAux),
           static_cast<unsigned long>(age),
           trackerCoreReady ? 1u : 0u);
  catWriteRaw(response);
}

void setBenchTestSelection(int8_t bandIndex, double rfFrequencyHz,
                           bool carrierOn) {
  benchTestBandIndex = bandIndex;
  benchTestRfFrequencyHz = rfFrequencyHz;
  benchTestCarrierRequested = carrierOn;
  benchTestModeActive = true;

  // Do not preserve a stale CAT PTT request across bench testing. Test mode
  // itself owns TX; CAT/VOX modulation is intentionally ignored while active.
  catTxRequested = false;

  // Blank RX immediately, before serviceClockGenerator() performs the hardware
  // transition on this same core.
  rxAudioBlankingActive = true;
  rxAudioUnblankAtUs = 0;
  ++benchTestRequestVersion;
  tinyDxMemoryBarrier();
}

void sendBenchTestStatus() {
  char response[64];
  if (!benchTestModeActive) {
    snprintf(response, sizeof(response), "TEST NORMAL;");
  } else if (benchTestBandIndex < 0) {
    snprintf(response, sizeof(response), "TEST 10 %s 10000000Hz;",
             benchTestCarrierRequested ? "ON" : "OFF");
  } else {
    snprintf(response, sizeof(response), "TEST %s %s %luHz;",
             BAND_PLANS[benchTestBandIndex].name,
             benchTestCarrierRequested ? "ON" : "OFF",
             static_cast<unsigned long>(llround(benchTestRfFrequencyHz)));
  }
  catWriteRaw(response);
}

bool handleBenchTestCommand(const char *command, size_t length) {
  if (commandIs(command, length, "TEST?") ||
      commandIs(command, length, "TEST")) {
    sendBenchTestStatus();
    return true;
  }

  if (commandIs(command, length, "TEST EXIT")) {
    benchTestCarrierRequested = false;
    benchTestModeActive = false;
    ++benchTestRequestVersion;
    tinyDxMemoryBarrier();
    catWriteRaw("TEST EXIT OK;");
    return true;
  }

  if (commandIs(command, length, "TEST ON")) {
    if (!benchTestModeActive) {
      catWriteRaw("TEST SELECT FIRST;");
      return true;
    }
    benchTestCarrierRequested = true;
    ++benchTestRequestVersion;
    tinyDxMemoryBarrier();
    sendBenchTestStatus();
    return true;
  }

  if (commandIs(command, length, "TEST OFF")) {
    if (!benchTestModeActive) {
      catWriteRaw("TEST NORMAL;");
      return true;
    }
    benchTestCarrierRequested = false;
    ++benchTestRequestVersion;
    tinyDxMemoryBarrier();
    sendBenchTestStatus();
    return true;
  }

  struct TestCommandChoice {
    const char *token;
    int8_t bandIndex;
    double rfHz;
  };
  static constexpr TestCommandChoice choices[] = {
      {"10", -1, TEST_10MHZ_RF_HZ},
      {"40",  0, static_cast<double>(BAND_PLANS[0].ft8Hz)},
      {"30",  1, static_cast<double>(BAND_PLANS[1].ft8Hz)},
      {"20",  2, static_cast<double>(BAND_PLANS[2].ft8Hz)},
      {"17",  3, static_cast<double>(BAND_PLANS[3].ft8Hz)},
  };

  for (const auto &choice : choices) {
    char selectOnly[16];
    char selectOn[20];
    char selectOff[20];
    snprintf(selectOnly, sizeof(selectOnly), "TEST %s", choice.token);
    snprintf(selectOn, sizeof(selectOn), "TEST %s ON", choice.token);
    snprintf(selectOff, sizeof(selectOff), "TEST %s OFF", choice.token);

    if (commandIs(command, length, selectOnly) ||
        commandIs(command, length, selectOff)) {
      setBenchTestSelection(choice.bandIndex, choice.rfHz, false);
      sendBenchTestStatus();
      return true;
    }
    if (commandIs(command, length, selectOn)) {
      setBenchTestSelection(choice.bandIndex, choice.rfHz, true);
      sendBenchTestStatus();
      return true;
    }
  }

  // It begins with TEST but was not a valid test command. Consume it here so
  // it is not mistaken for a Kenwood command.
  if (length >= 4 && memcmp(command, "TEST", 4) == 0) {
    catWriteRaw("TEST ?;");
    return true;
  }
  return false;
}

void resetReliabilityDiagnostics() {
  lastVoxStopReason = VOX_STOP_NONE;
  lastRfStopReason = RF_STOP_NONE;
  catCommandCount = 0;
  catUnknownCount = 0;
  catMeterQueryCount = 0;
  catTxCommandCount = 0;
  catRxCommandCount = 0;
  catShortWriteCount = 0;
  i2cFailureCount = 0;
  rfStopCount = 0;
  audioInvalidRfStopCount = 0;
  txClockFailureCount = 0;
  fastRetuneFailureCount = 0;
  tinyDxMemoryBarrier();
}

void executeCatCommand(const char *command, size_t length) {
  ++catCommandCount;

  if (commandIs(command, length, "%CW")) {
    enterCwTerminalMode();
    return;
  }

  if (commandIs(command, length, "DBG")) {
    sendDiagnosticState();
    return;
  }
  if (commandIs(command, length, "DBGC")) {
    sendDiagnosticCounters();
    return;
  }
  if (commandIs(command, length, "DBGE")) {
    sendDiagnosticErrors();
    return;
  }
  if (commandIs(command, length, "DBGW")) {
    sendWatchdogDiagnostics();
    return;
  }
  if (commandIs(command, length, "DBGR")) {
    resetReliabilityDiagnostics();
    catWriteRaw("DBGR OK;");
    return;
  }

  if (handleIqCalibrationCommand(command, length)) return;
  if (handleBenchTestCommand(command, length)) return;
  if (commandIs(command, length, "FA")) {
    sendCatFrequency('A');
    return;
  }

  if (commandIs(command, length, "FB")) {
    sendCatFrequency('B');
    return;
  }

  if (length == 13 &&
      (command[0] == 'F') &&
      (command[1] == 'A' || command[1] == 'B')) {
    uint32_t newFrequencyHz = 0;
    if (parseCatFrequency11(command + 2, newFrequencyHz)) {
      applyDialFrequency(newFrequencyHz);
      return;  // Kenwood set commands do not reply.
    }
    catWriteRaw("?;");
    return;
  }

  // TS-480 PTT SET commands.
  //
  // IMPORTANT for Hamlib compatibility:
  // A successful TX SET command is silent. Hamlib can send TX1; and then
  // immediately issue ID; or IF; to verify the radio. If TinyDX sends TX0;
  // here, that stale response is consumed as the answer to the following
  // command and Hamlib reports a protocol error / PTT failure.
  //
  // TX; defaults to P1=0. TinyDX accepts TX0/TX1/TX2 as the same PTT-ON
  // request because it has one transmit path. TX1 is commonly used by Hamlib
  // for data-mode PTT.
  if (commandIs(command, length, "TX") ||
      commandIs(command, length, "TX0") ||
      commandIs(command, length, "TX1") ||
      commandIs(command, length, "TX2")) {
    if (benchTestModeActive) {
      catWriteRaw("?;");
      return;
    }
    catTxRequested = true;
    ++catTxCommandCount;
    tinyDxMemoryBarrier();
    return;  // Successful Kenwood SET command: no reply.
  }

  // RX; returns the radio to receive. Successful execution is also silent.
  // The host obtains RX/TX status using IF;.
  if (commandIs(command, length, "RX")) {
    if (benchTestModeActive) {
      catWriteRaw("?;");
      return;
    }
    catTxRequested = false;
    ++catRxCommandCount;
    tinyDxMemoryBarrier();
    return;  // Successful Kenwood SET command: no reply.
  }

  if (commandIs(command, length, "IF")) {
    sendCatIfStatus();
    return;
  }

  if (commandIs(command, length, "ID")) {
    catWriteRaw("ID020;");  // Kenwood TS-480
    return;
  }

  if (commandIs(command, length, "MD")) {
    catWriteRaw(rxSidebandMode == RX_MODE_LSB ? "MD1;" : "MD2;");
    return;
  }
  if (commandIs(command, length, "MD1")) {
    setRxSidebandMode(RX_MODE_LSB);
    return;  // Successful Kenwood SET command: no reply.
  }
  if (commandIs(command, length, "MD2")) {
    setRxSidebandMode(RX_MODE_USB);
    return;  // Successful Kenwood SET command: no reply.
  }

  if (commandIs(command, length, "AI")) {
    catWriteRaw("AI0;");
    return;
  }
  if (commandIs(command, length, "AI0") ||
      commandIs(command, length, "AI1") ||
      commandIs(command, length, "AI2")) return;

  // -----------------------------------------------------------------------
  // Tolerant TS-480 status/meter subset.
  //
  // Hamlib's current TS-480 backend advertises SWR/COMP/ALC meter support and
  // reads all three at once by sending RM; and expecting exactly:
  //   RM1xxxx;RM2xxxx;RM3xxxx;
  // TinyDX has no directional coupler, compressor meter or ALC detector, so
  // report zero rather than returning ?; (which can provoke retries/errors).
  // -----------------------------------------------------------------------
  if (commandIs(command, length, "RM")) {
    ++catMeterQueryCount;
    catWriteRaw("RM10000;RM20000;RM30000;");
    return;
  }
  if (commandIs(command, length, "RM1") ||
      commandIs(command, length, "RM2") ||
      commandIs(command, length, "RM3")) {
    catSelectedMeter = static_cast<uint8_t>(command[2] - '0');
    return;  // Successful meter-selection SET command is silent.
  }

  // TS-480 S-meter read. The AT-DX1 does not expose an RSSI/S-meter, so S0.
  if (commandIs(command, length, "SM") ||
      commandIs(command, length, "SM0")) {
    ++catMeterQueryCount;
    catWriteRaw("SM00000;");
    return;
  }

  // Fixed harmless values for occasional Hamlib/application status queries.
  // PC005 is the lowest legal TS-480 power setting; it is only a CAT fiction.
  if (commandIs(command, length, "PC")) {
    catWriteRaw("PC005;");
    return;
  }
  if (length == 5 && command[0] == 'P' && command[1] == 'C' &&
      command[2] >= '0' && command[2] <= '9' &&
      command[3] >= '0' && command[3] <= '9' &&
      command[4] >= '0' && command[4] <= '9') {
    return;  // Accept but ignore requested TS-480 power setting.
  }

  // Firmware/model-type read. The final '1' identifies the SAT-type variant
  // to Hamlib; no AT-DX1 behaviour depends on this fictional model detail.
  if (commandIs(command, length, "TY")) {
    catWriteRaw("TY001;");
    return;
  }

  // Additional benign TS-480 reads. These are not needed for FT8 itself, but
  // returning syntactically valid fixed values prevents UI/status polling from
  // turning into a CAT timeout. Corresponding SET commands may be ignored.
  if (commandIs(command, length, "BY")) {
    catWriteRaw("BY00;");       // not busy
    return;
  }
  if (commandIs(command, length, "AG0")) {
    catWriteRaw("AG0000;");     // AF gain = 0 (fictional/status only)
    return;
  }
  if (commandIs(command, length, "RG")) {
    catWriteRaw("RG100;");      // RF gain 100%
    return;
  }
  if (commandIs(command, length, "SQ0")) {
    catWriteRaw("SQ0000;");     // squelch 0
    return;
  }
  if (commandIs(command, length, "GT")) {
    catWriteRaw("GT002;");      // AGC slow
    return;
  }
  if (commandIs(command, length, "PA")) {
    catWriteRaw("PA00;");       // preamp off
    return;
  }
  if (commandIs(command, length, "RA")) {
    catWriteRaw("RA0000;");     // attenuator off
    return;
  }
  if (commandIs(command, length, "ML")) {
    catWriteRaw("ML000;");      // TX monitor off
    return;
  }
  if (commandIs(command, length, "NL")) {
    catWriteRaw("NL000;");      // noise blanker level 0
    return;
  }
  if (commandIs(command, length, "RL")) {
    catWriteRaw("RL00;");       // noise reduction level 0
    return;
  }
  if (commandIs(command, length, "LK")) {
    catWriteRaw("LK00;");       // unlocked
    return;
  }
  if (commandIs(command, length, "UL")) {
    catWriteRaw("UL0;");        // PLL locked
    return;
  }

  if (commandIs(command, length, "PS")) {
    catWriteRaw("PS1;");
    return;
  }

  if (commandIs(command, length, "FR")) {
    catWriteRaw("FR0;");
    return;
  }
  if (commandIs(command, length, "FR0")) return;

  if (commandIs(command, length, "FT")) {
    catWriteRaw("FT0;");
    return;
  }
  if (commandIs(command, length, "FT0")) return;

  // Unknown/irrelevant commands are deliberately ignored rather than answered
  // with ?;. A stale error reply can be consumed as the answer to the next
  // Hamlib command when applications pipeline transactions. Count it so DBG
  // diagnostics can still reveal that the host asked for something unsupported.
  ++catUnknownCount;
}

void feedCatByte(char c) {
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');

  if (c == ';' || c == '\r' || c == '\n') {
    if (catCommandLength > 0) {
      catReplyTerminator = (c == ';') ? ';' : '\r';
      executeCatCommand(catCommandBuffer, catCommandLength);
      catCommandLength = 0;
      catCommandBuffer[0] = '\0';
    }
    return;
  }

  if (catCommandLength < CAT_COMMAND_BUFFER_SIZE - 1) {
    catCommandBuffer[catCommandLength++] = c;
    catCommandBuffer[catCommandLength] = '\0';
  } else {
    // Drop an overlength command cleanly rather than leaving a partial command
    // in the parser. The next terminator starts us fresh.
    catCommandLength = 0;
    catCommandBuffer[0] = '\0';
  }
}

void serviceCatControl() {
  // The earlier proven TinyDX CAT implementation read CDC in blocks. Restore
  // that behaviour: it handles a complete Hamlib command in one service call
  // instead of making progress one byte at a time across several loop passes.
  uint8_t buffer[64];

  while (tud_cdc_n_available(CAT_CDC_INSTANCE) > 0) {
    const uint32_t count = tud_cdc_n_read(
        CAT_CDC_INSTANCE, buffer, sizeof(buffer));
    if (count == 0) break;

    for (uint32_t i = 0; i < count; ++i) {
      if (cwTerminalMode) {
        feedCwTerminalByte(static_cast<char>(buffer[i]));
      } else {
        feedCatByte(static_cast<char>(buffer[i]));
      }
    }
  }
}


// ---------------------------------------------------------------------------
// Optional serial diagnostics. Disabled while CAT occupies the same CDC port.
// ---------------------------------------------------------------------------
void serviceDebugReports() {
  if (!CAT_DEBUG_REPORTS) return;

  static uint32_t lastReportMs = 0;
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastReportMs) < 1000) return;
  lastReportMs = now;

  Serial.print("HW=");
  Serial.print(TINYDX_HARDWARE_NAME);
  Serial.print(" Ref=");
  Serial.print(static_cast<unsigned long>(SI5351_REFERENCE_HZ));
  Serial.print(" Dial=");
  Serial.print(dialFrequencyHz);
  Serial.print(" RXclk=");
  Serial.print(static_cast<unsigned long>(dialFrequencyHz * 4UL));
  Serial.print(" RXms0=");
  Serial.print(currentRxMs0Divider);
  Serial.print(" Audio=");
  Serial.print(currentAudioFrequencyHz, 3);
  Serial.print(" TXrf=");
  Serial.print(currentTxFrequencyHz, 3);
  Serial.print(" TXclk=");
  Serial.print(currentTxFrequencyHz * 2.0, 3);
  Serial.print(" Test=");
  Serial.print(benchTestModeActive ? "YES" : "NO");
  if (benchTestModeActive) {
    Serial.print(benchTestCarrierRequested ? "/ON@" : "/OFF@");
    Serial.print(benchTestRfFrequencyHz, 0);
  }
  Serial.print(" overruns=");
  Serial.println(trackerOverruns);
}

// ---------------------------------------------------------------------------
// Whole-chip lockup watchdog
// Feed only from core 0. If core 0 blocks anywhere, the hardware watchdog
// resets the entire chip. If core 1 stops updating its heartbeat, make the RF
// hardware safe and deliberately stop feeding so the same reset occurs.
// ---------------------------------------------------------------------------
void serviceSystemWatchdog() {
  if (trackerCoreReady) {
    // Snapshot the heartbeat first, then read the clock. FIX4/FIX5 used an
    // unsigned expression millis() - core1HeartbeatMs directly. Because core 1
    // can update the volatile heartbeat between those two operand reads, a
    // heartbeat 1 ms newer than the sampled millis() value produced 0xFFFFFFFF
    // and falsely looked billions of milliseconds stale. A signed delta makes
    // a slightly-future concurrent heartbeat negative instead of stale.
    const uint32_t heartbeat = core1HeartbeatMs;
    tinyDxMemoryBarrier();
    const uint32_t now = millis();
    const int32_t ageMs = static_cast<int32_t>(now - heartbeat);

    if (ageMs > static_cast<int32_t>(CORE1_HEARTBEAT_TIMEOUT_MS)) {
      // These are GPIO-only operations: do not touch I2C while recovering from
      // a possible bus/deadlock fault. The watchdog will reset the full chip.
      setTxDriversEnabled(false);
      setRxPathEnabled(false);
      setTxLed(false);
      setWatchdogBreadcrumb(WD_STAGE_CORE1_STALE,
                            static_cast<uint32_t>(ageMs));
      while (true) tight_loop_contents();
    }
  }
  watchdog_update();
}

// ---------------------------------------------------------------------------
// Arduino multicore entry points
// ---------------------------------------------------------------------------
void setup() {
  beginRfAndLedHardware();

  // Capture reset reason and the last pre-reset breadcrumb BEFORE beginning
  // a fresh watchdog period or writing any new breadcrumb. Scratch 0/1 survive
  // a watchdog reset; scratch 4 is intentionally left to the Pico SDK.
  bootResetReason = watchdog_caused_reboot() ? 4u : 1u;
  previousWatchdogBreadcrumb = watchdog_hw->scratch[0];
  previousWatchdogAux = watchdog_hw->scratch[1];
  watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
  watchdog_update();

  // Load saved I/Q balance before the first receive samples are demodulated.
  beginIqCalibrationStorage();
  watchdog_update();

  setWatchdogBreadcrumb(WD_STAGE_SETUP_I2C);
  beginI2cAndSi5351();
  watchdog_update();

  setWatchdogBreadcrumb(WD_STAGE_SETUP_USB);
  Serial.begin(115200);
  if (!TinyUSBDevice.isInitialized()) TinyUSBDevice.begin(0);

  if (!beginUsbAudio()) {
    // Leave the RF hardware safe. Illuminate all four band LEDs as a visible
    // startup fault indication; TX LED remains OFF and TX drivers stay disabled.
    setTxDriversEnabled(false);
    setRxPathEnabled(false);
    setTxLed(false);
    setPanelLed(BAND_40_LED_PIN, true);
    setPanelLed(BAND_30_LED_PIN, true);
    setPanelLed(BAND_20_LED_PIN, true);
    setPanelLed(BAND_17_LED_PIN, true);
    // Do not feed the watchdog here. A startup failure will automatically
    // recover with a whole-chip reset instead of requiring a power cycle.
    while (true) tight_loop_contents();
  }
  watchdog_update();
}

void loop() {
  setWatchdogBreadcrumb(WD_STAGE_LOOP_START);
  serviceSystemWatchdog();

  // CAT/PTT gets priority over continuous audio servicing. Hamlib uses a short
  // command timeout (about half a second in the observed WSJT-X test), so never
  // let a continuously active USB-audio endpoint monopolise core 0.
  setWatchdogBreadcrumb(WD_STAGE_USB_TASK_1);
  TinyUSB_Device_Task();
  setWatchdogBreadcrumb(WD_STAGE_CAT_1);
  serviceCatControl();
  setWatchdogBreadcrumb(WD_STAGE_CLOCK_1);
  serviceClockGenerator();

  setWatchdogBreadcrumb(WD_STAGE_USB_AUDIO_RX);
  receiveUsbPlayback();
  setWatchdogBreadcrumb(WD_STAGE_USB_TASK_2);
  TinyUSB_Device_Task();
  setWatchdogBreadcrumb(WD_STAGE_CAT_2);
  serviceCatControl();
  setWatchdogBreadcrumb(WD_STAGE_CLOCK_2);
  serviceClockGenerator();

  setWatchdogBreadcrumb(WD_STAGE_USB_AUDIO_TX);
  sendAdcToUsb();
  setWatchdogBreadcrumb(WD_STAGE_USB_TASK_3);
  TinyUSB_Device_Task();
  setWatchdogBreadcrumb(WD_STAGE_CAT_3);
  serviceCatControl();
  setWatchdogBreadcrumb(WD_STAGE_CLOCK_3);
  serviceClockGenerator();

  // Solve/save calibration outside the per-sample audio loop.  EEPROM.commit()
  // briefly idles the other core while flash is written, so it must never run
  // from the ADC/DMA path itself.
  serviceIqCalibration();

  serviceDebugReports();
  setWatchdogBreadcrumb(WD_STAGE_CDC_FLUSH);
  TinyUSB_Device_FlushCDC();
  serviceSystemWatchdog();
  setWatchdogBreadcrumb(WD_STAGE_YIELD);
  yield();
}

void setup1() {
  detector = ThreeCycleDetectorState{};
  beginAdcOnCore1();
  core1HeartbeatMs = millis();
  trackerCoreReady = true;
  tinyDxMemoryBarrier();
}

void loop1() {
  core1HeartbeatMs = millis();
  tinyDxMemoryBarrier();

  // CW terminal mode does not run receive DSP on the RP2350. ADC DMA and the
  // core-0 USB-audio path continue streaming Q_OUT to the Android app, while
  // FT8 playback tracking is explicitly stopped so it cannot arm FT8 VOX.
  if (cwTerminalMode) {
    if (detector.voxActive) stopVox(VOX_STOP_STREAM_ENDED);
    trackerFlush();
    core1HeartbeatMs = millis();
    tinyDxMemoryBarrier();
    tight_loop_contents();
    return;
  }

  const uint32_t nowUs = micros();
  const bool packetTimedOut =
      static_cast<uint32_t>(nowUs - lastUsbPacketMicros) >
      USB_PACKET_TIMEOUT_US;

  if (!usbPlaybackStreaming || packetTimedOut) {
    if (detector.voxActive) {
      stopVox(usbPlaybackStreaming ? VOX_STOP_USB_TIMEOUT : VOX_STOP_STREAM_ENDED);
    }
    trackerFlush();
    core1HeartbeatMs = millis();
    tinyDxMemoryBarrier();
    tight_loop_contents();
    return;
  }

  int16_t sample = 0;
  uint16_t processed = 0;

  // IMPORTANT: this loop is deliberately bounded.  Continuous USB playback
  // must not prevent the core-1 heartbeat from being refreshed.
  while (processed < CORE1_MAX_SAMPLES_PER_PASS && trackerPop(sample)) {
    processPlaybackSample(sample);
    ++processed;

    if ((processed % CORE1_HEARTBEAT_SAMPLE_INTERVAL) == 0) {
      core1HeartbeatMs = millis();
      tinyDxMemoryBarrier();
    }
  }

  core1HeartbeatMs = millis();
  tinyDxMemoryBarrier();

  if (processed == 0) tight_loop_contents();
}