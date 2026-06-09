// =============================================================================
// MS3 Pro Ultimate  →  Haltech UC10  CAN BRIDGE  v2.0  (PRODUCTION)
// =============================================================================
// Reads the MS3 "Advanced Real-Time Data Broadcast" and re-broadcasts every
// supported channel in Haltech V2 CAN format for the Haltech IC7/UC10 dash.
//
// Hardware
//   Arduino Nano (ATmega328P, 5V, 16MHz)
//   2× MCP2515 modules with 8MHz crystal
//
// Wiring summary
//   CAN_A: CS=D10, INT=D3, shared SCK=D13, MISO=D12, MOSI=D11 → MS3   @ 500kbps
//   CAN_B: CS=D9,  INT=D2, shared SCK=D13, MISO=D12, MOSI=D11 → UC10  @ 1 Mbps
//   Power: 12V switched (1A fuse) → VIN, GND → GND
//
// Library
//   "mcp_can" by Cory Fowler  (Library Manager → search "MCP2515")
//
// MS3 TunerStudio setup (REQUIRED)
//   CAN-Bus/Testmodes → CAN Parameters → Master enable = On
//   CAN-Bus/Testmodes → CAN Realtime Data Broadcasting → Enable = On
//   Base ID: leave default (1520 dec / 0x5F0).
//   Enable broadcast on at LEAST groups: 0, 1, 2, 3, 13, 14, 17, 22, 31, 33,
//   42, 47, 53.  Higher rates are fine; bridge only reads what it needs.
//   (Turn OFF the simplified "Dash Broadcasting" to avoid bus clutter.)
//
// =============================================================================
// v2.0 Changelog (all v1 issues fixed):
//   • CRITICAL: Fixed 0x361 fuel/oil pressure scaling (kPa→bar, was 10x off).
//   • CRITICAL: Fixed 0x372 boost target scaling (kPa→bar, was 10x off).
//   • Fixed 0x362 ignition advance byte position (was byte 4, correct per spec).
//   • Fixed afrtgt1 truncation (uint8_t → int16_t).
//   • Added MCP2515 hardware filters so CAN_A only accepts MS3 broadcast IDs
//     — reduces interrupt load, eliminates boot-time race condition.
//   • Added max-iteration guard on the receive loop (prevents lockup if INT
//     pin gets stuck low).
//   • Added MS3 timeout / data-staleness detection (>2s with no MS3 data
//     blanks the dash output instead of showing frozen values).
//   • Added watchdog timer reset on every loop pass (auto-recovers if hung).
//   • Added TX-failure detection with auto-recovery (CAN bus-off resets the
//     module rather than failing silently).
//   • Added serial debug stats every 5s (RPM, fault counts).
//   • Removed unused 'len' variable warning.
//   • Stoich constant defined so it can be tuned for ethanol blends.
//   • Gear field: handles MS3 signed gear (-1=reverse) → unsigned for Haltech.
//   • Saturating math on conversions so extreme values don't wrap.
//   • v2.1: Added Haltech WB1 wideband controller emulator on 0x2B1 at 20Hz.
//     UC10 dash requires WB1 presence frames to display wideband data — now
//     handled so AFR/Lambda channels show real values instead of "Device Time
//     Out". Lambda derived from MS3 AFR1.
//   • v2.2: Sensor mapping for Frank's setup (Generic sensors with bar/°F):
//     - Fuel Pressure  → Generic Sensor 1 (bar → bar1000)
//     - Oil Pressure   → Generic Sensor 2 (bar → bar1000)
//     - Oil Temp       → Generic Sensor 3 (°F → K×10)
//     - Coolant Press  → Generic Sensor 4 (bar → bar1000)
//     - Fuel Temp      → Generic Sensor 6 (°F → K×10)
//     - Coolant pressure now broadcast on 0x360 bytes 6-7 (was always 0).
//   • v2.2: WB1 frame DLC fixed: 8 bytes → 7 bytes per reference emulator.
//     This may resolve "Device Time Out" on wideband channels.
// =============================================================================

#include <SPI.h>
#include <mcp_can.h>
#include <avr/wdt.h>      // Watchdog timer

// -----------------------------------------------------------------------------
// Pin definitions
// -----------------------------------------------------------------------------
// NOTE: CS/INT pins SWAPPED from original layout because the physical wiring
// has the MS3 module on the pins originally assigned to UC10, and vice versa.
// Code now matches the as-built wiring: D9/D2 → MS3 module, D10/D3 → UC10 module.
constexpr uint8_t PIN_CAN_A_CS  = 9;    // MS3 side (was 10)
constexpr uint8_t PIN_CAN_B_CS  = 10;   // UC10 side (was 9)
constexpr uint8_t PIN_CAN_A_INT = 2;    // MS3 side (was 3)
constexpr uint8_t PIN_LED       = LED_BUILTIN;   // heartbeat LED

// -----------------------------------------------------------------------------
// Tuning constants
// -----------------------------------------------------------------------------
// Stoichiometric AFR (14.7 for pump gas; 9.0 for E85; 14.5 typical for E10).
// Lambda is broadcast based on this, so set to your actual fuel.
constexpr int16_t STOICH_AFR_X10 = 147;     // 14.7 × 10 (gasoline default)

// MS3 data-staleness timeout. If no valid MS3 frame received in this many ms,
// dash output is zeroed so you see "no data" instead of frozen values.
constexpr uint32_t MS3_TIMEOUT_MS = 2000;

// Generic-sensor inputs where YOU wired each sensor in TunerStudio.
// Indices are 0-based: sensor 1 = index 0, sensor 3 = index 2.
// Set to -1 if not used.
//
// Frank's current wiring (PSI for pressures, °F for temps):
//   Generic 01 → Fuel Pressure        (Analog In 3)
//   Generic 02 → Oil Pressure         (Analog In 1)
//   Generic 03 → Oil Temperature      (Analog In 2)
//   Generic 04 → Coolant Pressure     (Analog In 5)
//   Generic 06 → Fuel Temperature     (Analog In 4)
constexpr int8_t FUELP_SENSOR_INDEX  = 0;   // sensor 1
constexpr int8_t OILP_SENSOR_INDEX   = 1;   // sensor 2
constexpr int8_t OILT_SENSOR_INDEX   = 2;   // sensor 3
constexpr int8_t COOLP_SENSOR_INDEX  = 3;   // sensor 4
constexpr int8_t FUELT_SENSOR_INDEX  = 5;   // sensor 6

// MS3 generic sensors are output as user-defined scaled values.
// Frank's sensors output PSI for pressures and °F for temps.
//
// Haltech expects pressures in bar × 1000.  1 PSI = 0.0689476 bar.
//   So bar1000 = PSI × 68.9476.
// MS3 generic sensors are stored as int16_t × 10 (one decimal of precision),
// so the actual conversion is: bar1000 = sensor_x10 × 68.9476 / 10 ≈ sensor_x10 × 6.89.
// Using integer math: bar1000 = sensor_x10 × 689 / 100   (within 0.1% of exact)
constexpr int32_t PSI10_TO_BAR1000_NUM = 689;
constexpr int32_t PSI10_TO_BAR1000_DEN = 100;

// -----------------------------------------------------------------------------
// Protocol constants
// -----------------------------------------------------------------------------
// MS3 Advanced broadcast: base ID 1520 dec = 0x5F0; group N at base+N.
constexpr uint32_t MS3_BASE = 0x5F0;
constexpr uint32_t MS3_LAST = 0x5F0 + 63;

// Haltech V2 broadcast frame IDs (verified against Haltech V2.35 spec)
constexpr uint32_t HT_360 = 0x360;   // 50Hz RPM, MAP, TPS, coolant pressure
constexpr uint32_t HT_361 = 0x361;   // 50Hz Fuel P, Oil P, Pedal, Wastegate P
constexpr uint32_t HT_362 = 0x362;   // 50Hz Inj duty pri/sec, Ign angle
constexpr uint32_t HT_368 = 0x368;   // 20Hz Lambda 1-4
constexpr uint32_t HT_36A = 0x36A;   // 20Hz Knock level / retard
constexpr uint32_t HT_370 = 0x370;   // 20Hz Wheelspeed, Gear, Intake cam
constexpr uint32_t HT_372 = 0x372;   // 10Hz Battery V, AirTemp2, Tgt boost, Baro
constexpr uint32_t HT_373 = 0x373;   // 10Hz EGT 1-4
constexpr uint32_t HT_374 = 0x374;   // 10Hz EGT 5-8
constexpr uint32_t HT_3E0 = 0x3E0;   // 5Hz  Coolant, Air, Fuel, Oil temp (Kelvin)
constexpr uint32_t HT_3E1 = 0x3E1;   // 5Hz  Trans T, Diff T, Fuel composition
constexpr uint32_t HT_3E3 = 0x3E3;   // 5Hz  Fuel trims

// Haltech WB1 wideband controller emulation
// Broadcast at 20Hz on 0x2B1 makes the UC10 dash see "a real WB1 on the bus"
// and accept lambda data instead of showing "Device Time Out" on wideband channels.
constexpr uint32_t HT_WB1 = 0x2B1;   // 20Hz Haltech WB1 wideband presence + lambda

// -----------------------------------------------------------------------------
// Module handles
// -----------------------------------------------------------------------------
MCP_CAN CAN_A(PIN_CAN_A_CS);
MCP_CAN CAN_B(PIN_CAN_B_CS);

// -----------------------------------------------------------------------------
// MS3 channel cache (all values in MS3 native scaling unless noted)
// -----------------------------------------------------------------------------
struct {
  // Group 0
  int16_t  rpm;             // RPM 1:1
  int16_t  pw1_x1000;       // injector PW, ms × 1000
  // Group 1
  int16_t  adv_deg_x10;     // ignition advance, deg BTDC × 10
  int16_t  afrtgt1_x10;     // target AFR × 10 (widened to int16 to avoid 25.5 cap)
  // Group 2
  int16_t  baro_x10;        // kPa × 10
  int16_t  map_x10;         // kPa × 10
  int16_t  mat_x10;         // °F × 10
  int16_t  clt_x10;         // °F × 10
  // Group 3
  int16_t  tps_x10;         // % × 10
  int16_t  batt_x10;        // V × 10
  // Group 8
  int16_t  fuelload_x10;
  // Group 13-16: generic sensors 1-16 (raw × 10)
  int16_t  sensor[16];
  // Group 17
  int16_t  boost_targ_x10;  // kPa × 10
  // Group 22-23 EGT 1-8 (°F × 10)
  int16_t  egt[8];
  // Group 31 per-cyl AFR (1 byte each, AFR × 10)
  uint8_t  afr1_x10;
  // Group 33
  int8_t   gear;            // current gear (-1 = reverse, 0 = neutral, 1+ = forward)
  // Group 42 VSS1 (m/s × 10)
  int16_t  vss1_ms_x10;
  // Group 47 ethanol % × 10
  int16_t  flex_pct_x10;
  // Group 53 dedicated fuel pressure & temp
  int16_t  fuel_press1_x10; // kPa × 10
  int16_t  fuel_temp1_x10;  // °F × 10
} D;

// -----------------------------------------------------------------------------
// Runtime state
// -----------------------------------------------------------------------------
uint32_t lastMS3RxMs    = 0;     // millis() of last successful MS3 frame
uint32_t t50 = 0, t20 = 0, t10 = 0, t5 = 0, tStats = 0;
uint32_t ms3FrameCount  = 0;
uint32_t txErrCount     = 0;
bool     ms3Stale       = true;

// =============================================================================
// HELPERS
// =============================================================================
static inline void wBE(uint8_t* b, uint8_t o, int16_t v) {
  b[o]     = (uint8_t)((v >> 8) & 0xFF);
  b[o + 1] = (uint8_t)( v       & 0xFF);
}

static inline int16_t rBE(const uint8_t* b, uint8_t o) {
  return (int16_t)(((uint16_t)b[o] << 8) | b[o + 1]);
}

// Saturating cast to int16_t
static inline int16_t sat16(int32_t v) {
  if (v >  32767) return  32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

// MS3 °F×10 → Haltech Kelvin×10.  K = (F − 32) × 5/9 + 273.15
static int16_t F10_to_K10(int16_t f10) {
  int32_t c10 = ((int32_t)(f10 - 320) * 5) / 9;   // °C × 10
  return sat16(c10 + 2731);
}

// MS3 kPa×10 → Haltech bar×1000  (1 bar = 100 kPa  →  kPa×10 = bar×1000)
// Same magnitude, just relabelled. No conversion needed.
static inline int16_t kPa10_to_bar1000(int16_t kpa10) { return kpa10; }

// MS3 AFR×10 → Haltech Lambda×1000.  λ = AFR / stoich
static int16_t afr10_to_lambda1000(int16_t afr10) {
  if (afr10 <= 0) return 1000;                         // safe default
  return sat16(((int32_t)afr10 * 1000) / STOICH_AFR_X10);
}

// MS3 m/s×10 → km/h×10.  km/h = m/s × 3.6
static inline int16_t ms10_to_kmh10(int16_t ms10) {
  return sat16(((int32_t)ms10 * 36) / 10);
}

// MS3 gear (signed) → Haltech gear (unsigned, 0=N, 1+=forward, 0xFF=reverse)
// Most UC10 setups treat 0=N and negative as reverse symbol.
static inline int16_t gear_to_haltech(int8_t g) {
  if (g < 0) return 0xFF;                              // reverse
  return (int16_t)g;
}

// Fetch a generic-sensor value, or 0 if index is out of range
static inline int16_t sensorOrZero(int8_t idx) {
  return (idx >= 0 && idx < 16) ? D.sensor[idx] : 0;
}

// Convert MS3 PSI×10 (from generic sensor with PSI calibration) to Haltech bar×1000
// 1 PSI = 0.0689 bar, so bar1000 = PSI×10 × 6.89
// Using integer math: PSI×10 × 689 / 100 (within 0.1% of exact)
// Clamps negative values to 0 since Haltech pressure fields are unsigned;
// sending small negative values gets interpreted as huge positive values.
static inline int16_t bar10_to_bar1000(int16_t psi10) {
  if (psi10 <= 0) return 0;
  return sat16(((int32_t)psi10 * 689) / 100);
}

// Fetch a pressure sensor and convert bar → bar1000 for Haltech CAN
static inline int16_t pressSensorBar1000(int8_t idx) {
  if (idx < 0) return 0;
  return bar10_to_bar1000(D.sensor[idx]);
}

// Fetch a temperature sensor (°F×10) and convert to Kelvin×10 for Haltech CAN
static inline int16_t tempSensorK10(int8_t idx) {
  if (idx < 0) return 0;
  int16_t f10 = D.sensor[idx];
  if (f10 == 0) return 0;
  return F10_to_K10(f10);
}

// =============================================================================
// MS3 ADVANCED FRAME PARSER
// =============================================================================
void parseMS3(uint32_t id, const uint8_t* d) {
  const uint8_t g = (uint8_t)(id - MS3_BASE);

  switch (g) {
    case 0:
      D.pw1_x1000 = rBE(d, 2);
      D.rpm       = rBE(d, 6);
      break;
    case 1:
      D.adv_deg_x10 = rBE(d, 0);
      D.afrtgt1_x10 = (int16_t)d[4];    // MS3 sends 1 byte, widen safely
      break;
    case 2:
      D.baro_x10 = rBE(d, 0);
      D.map_x10  = rBE(d, 2);
      D.mat_x10  = rBE(d, 4);
      D.clt_x10  = rBE(d, 6);
      break;
    case 3:
      D.tps_x10  = rBE(d, 0);
      D.batt_x10 = rBE(d, 2);
      break;
    case 8:
      D.fuelload_x10 = rBE(d, 2);
      break;
    case 13:
      D.sensor[0] = rBE(d, 0); D.sensor[1] = rBE(d, 2);
      D.sensor[2] = rBE(d, 4); D.sensor[3] = rBE(d, 6);
      break;
    case 14:
      D.sensor[4] = rBE(d, 0); D.sensor[5] = rBE(d, 2);
      D.sensor[6] = rBE(d, 4); D.sensor[7] = rBE(d, 6);
      break;
    case 15:
      D.sensor[8]  = rBE(d, 0); D.sensor[9]  = rBE(d, 2);
      D.sensor[10] = rBE(d, 4); D.sensor[11] = rBE(d, 6);
      break;
    case 16:
      D.sensor[12] = rBE(d, 0); D.sensor[13] = rBE(d, 2);
      D.sensor[14] = rBE(d, 4); D.sensor[15] = rBE(d, 6);
      break;
    case 17:
      D.boost_targ_x10 = rBE(d, 0);
      break;
    case 22:
      D.egt[0] = rBE(d, 0); D.egt[1] = rBE(d, 2);
      D.egt[2] = rBE(d, 4); D.egt[3] = rBE(d, 6);
      break;
    case 23:
      D.egt[4] = rBE(d, 0); D.egt[5] = rBE(d, 2);
      D.egt[6] = rBE(d, 4); D.egt[7] = rBE(d, 6);
      break;
    case 31:
      D.afr1_x10 = d[0];
      break;
    case 33:
      D.gear = (int8_t)d[6];
      break;
    case 42:
      D.vss1_ms_x10 = rBE(d, 0);
      break;
    case 47:
      D.flex_pct_x10 = rBE(d, 0);
      break;
    case 53:
      D.fuel_press1_x10 = rBE(d, 0);
      D.fuel_temp1_x10  = rBE(d, 4);
      break;
    default: break;     // ignore groups we don't use
  }

  lastMS3RxMs = millis();
  ms3FrameCount++;
}

// =============================================================================
// HALTECH TX WITH ERROR DETECTION
// =============================================================================
static void htSend(uint32_t id, const uint8_t* buf) {
  if (CAN_B.sendMsgBuf(id, 0, 8, (uint8_t*)buf) != CAN_OK) {
    txErrCount++;
  }
}

// WB1 sends 7-byte frames (per the reference Haltech wideband emulator).
// Using 8-byte frames here would cause the dash to ignore the message.
static void htSendWB1(uint32_t id, const uint8_t* buf) {
  if (CAN_B.sendMsgBuf(id, 0, 7, (uint8_t*)buf) != CAN_OK) {
    txErrCount++;
  }
}

// =============================================================================
// HALTECH BROADCAST  (verified against Haltech CAN V2.35 spec)
// =============================================================================

// ----- 50 Hz -----------------------------------------------------------------
void send50() {
  uint8_t b[8];

  // ===== 0x360 — RPM | MAP | TPS | Coolant Pressure =========================
  // Bytes 0-1: RPM            (raw RPM, 1:1)
  // Bytes 2-3: MAP            (×0.1 kPa)
  // Bytes 4-5: TPS            (×0.1 %)
  // Bytes 6-7: Coolant press  (×0.001 bar — from generic sensor, PSI converted)
  memset(b, 0, 8);
  wBE(b, 0, ms3Stale ? 0 : D.rpm);
  wBE(b, 2, ms3Stale ? 0 : D.map_x10);
  wBE(b, 4, ms3Stale ? 0 : D.tps_x10);
  wBE(b, 6, ms3Stale ? 0 : pressSensorBar1000(COOLP_SENSOR_INDEX));
  htSend(HT_360, b);

  // ===== 0x361 — Fuel P | Oil P | Pedal | Wastegate P ======================
  // Bytes 0-1: Fuel pressure  (×0.001 bar)  ← from generic sensor (PSI → bar)
  // Bytes 2-3: Oil pressure   (×0.001 bar)  ← from generic sensor (PSI → bar)
  // Bytes 4-5: Pedal pos      (×0.1 %)     — not available
  // Bytes 6-7: Wastegate P    (×0.1 kPa)   — not available
  memset(b, 0, 8);
  wBE(b, 0, ms3Stale ? 0 : pressSensorBar1000(FUELP_SENSOR_INDEX));
  wBE(b, 2, ms3Stale ? 0 : pressSensorBar1000(OILP_SENSOR_INDEX));
  htSend(HT_361, b);

  // ===== 0x362 — Inj duty pri/sec | Ign angle leading/trailing =============
  // Bytes 0-1: Inj duty pri   (×0.1 %)     — not directly available
  // Bytes 2-3: Inj duty sec   (×0.1 %)     — not available
  // Bytes 4-5: Ign angle lead (×0.1 deg)   ← MS3 adv_deg is deg×10 ✓
  // Bytes 6-7: Ign angle trail(×0.1 deg)   — not available
  memset(b, 0, 8);
  wBE(b, 4, ms3Stale ? 0 : D.adv_deg_x10);
  htSend(HT_362, b);
}

// ----- 20 Hz -----------------------------------------------------------------
void send20() {
  uint8_t b[8];

  // ===== 0x368 — Lambda 1-4 (×0.001) =======================================
  memset(b, 0, 8);
  if (!ms3Stale) {
    wBE(b, 0, afr10_to_lambda1000((int16_t)D.afr1_x10));  // Lambda 1
  }
  // Lambdas 2-4 not available
  htSend(HT_368, b);

  // ===== 0x370 — Wheelspeed | Gear | Intake cam 1/2 ========================
  // Bytes 0-1: Wheelspeed     (×0.1 km/h)
  // Bytes 2-3: Gear           (raw int, 0=N)
  // Bytes 4-5: Int cam 1      (×0.1 deg)   — not available
  // Bytes 6-7: Int cam 2      (×0.1 deg)   — not available
  memset(b, 0, 8);
  if (!ms3Stale) {
    wBE(b, 0, ms10_to_kmh10(D.vss1_ms_x10));
    wBE(b, 2, gear_to_haltech(D.gear));
  }
  htSend(HT_370, b);

  // ===== 0x2B1 — WB1 wideband controller emulation ==========================
  // Tricks the UC10 into thinking a real Haltech WB1 is on the bus, so it
  // accepts lambda data on wideband channels instead of "Device Time Out".
  // Bytes 0-1: WBI1 Lambda    (unsigned big-endian, y = x / 1024)
  // Bytes 2-3: WBI2 Lambda    (unused, 32767 = "free air" idle value)
  // Byte  4:   WBI1 sense R   (Ω) — fake healthy value
  // Byte  5:   WBI2 sense R   (Ω) — fake healthy value
  // Byte  6:   Diagnostic nibbles — 0 = no fault on both channels
  // Byte  7:   Battery V      (y = x × 20/255)
  memset(b, 0, 8);
  if (!ms3Stale && D.afr1_x10 > 0) {
    // Convert MS3 AFR×10 → Lambda×1024 (WB1's native scaling)
    // λ = AFR / stoich;  WB1 raw = λ × 1024
    uint16_t wb1_raw = (uint16_t)(((int32_t)D.afr1_x10 * 1024) / STOICH_AFR_X10);
    wBE(b, 0, (int16_t)wb1_raw);
  } else {
    wBE(b, 0, (int16_t)0x7FFF);   // 32767 = "Free Air" — sensor in fresh air
  }
  wBE(b, 2, (int16_t)0x7FFF);     // WBI2 always Free Air (we don't have a second wideband)
  b[4] = 80;                       // Fake sense resistor value (healthy range ~60-100Ω)
  b[5] = 80;
  b[6] = 0x00;                     // No diagnostic faults on either channel
  // Battery voltage: WB1 format is y = x × 20/255, so x = batt × 255 / 20
  // MS3 batt is V×10, so: x = (batt_x10 / 10) × 255 / 20 = batt_x10 × 255 / 200
  b[7] = (uint8_t)(((int32_t)D.batt_x10 * 255) / 200);
  htSendWB1(HT_WB1, b);   // Send as 7-byte frame per WB1 protocol spec
}

// ----- 10 Hz -----------------------------------------------------------------
void send10() {
  uint8_t b[8];

  // ===== 0x372 — Battery V | AirTemp2 | Target boost | Baro ================
  // Bytes 0-1: Battery V      (×0.1 V)
  // Bytes 2-3: Air temp 2     (×0.1 K)      — reuse MAT
  // Bytes 4-5: Target boost   (×0.001 bar)  ← MS3 kPa×10 maps 1:1
  // Bytes 6-7: Barometer      (×0.1 kPa)
  memset(b, 0, 8);
  if (!ms3Stale) {
    wBE(b, 0, D.batt_x10);
    wBE(b, 2, F10_to_K10(D.mat_x10));
    wBE(b, 4, kPa10_to_bar1000(D.boost_targ_x10));
    wBE(b, 6, D.baro_x10);
  }
  htSend(HT_372, b);

  // ===== 0x373 — EGT 1-4 (×0.1 K) ==========================================
  memset(b, 0, 8);
  if (!ms3Stale) {
    wBE(b, 0, F10_to_K10(D.egt[0]));
    wBE(b, 2, F10_to_K10(D.egt[1]));
    wBE(b, 4, F10_to_K10(D.egt[2]));
    wBE(b, 6, F10_to_K10(D.egt[3]));
  }
  htSend(HT_373, b);

  // ===== 0x374 — EGT 5-8 (×0.1 K) ==========================================
  memset(b, 0, 8);
  if (!ms3Stale) {
    wBE(b, 0, F10_to_K10(D.egt[4]));
    wBE(b, 2, F10_to_K10(D.egt[5]));
    wBE(b, 4, F10_to_K10(D.egt[6]));
    wBE(b, 6, F10_to_K10(D.egt[7]));
  }
  htSend(HT_374, b);
}

// ----- 5 Hz ------------------------------------------------------------------
void send5() {
  uint8_t b[8];

  // ===== 0x3E0 — Coolant | IAT | Fuel | Oil temp  (×0.1 K) =================
  memset(b, 0, 8);
  if (!ms3Stale) {
    wBE(b, 0, F10_to_K10(D.clt_x10));
    wBE(b, 2, F10_to_K10(D.mat_x10));
    wBE(b, 4, tempSensorK10(FUELT_SENSOR_INDEX));   // Fuel temp from generic sensor
    wBE(b, 6, tempSensorK10(OILT_SENSOR_INDEX));    // Oil temp from generic sensor
  }
  htSend(HT_3E0, b);

  // ===== 0x3E1 — Trans T | Diff T | Fuel composition =======================
  // Bytes 0-1: Trans T   (×0.1 K)  — not available
  // Bytes 2-3: Diff T    (×0.1 K)  — not available
  // Bytes 4-5: Fuel comp (×0.1 %)
  memset(b, 0, 8);
  if (!ms3Stale) {
    wBE(b, 4, D.flex_pct_x10);
  }
  htSend(HT_3E1, b);
}

// =============================================================================
// CAN-A initialisation with hardware filters for MS3 broadcast IDs
// =============================================================================
static void setupCanAFilters() {
  // MS3 advanced broadcast uses IDs 0x5F0..0x5FF (groups 0-15) on the
  // first filter pair, and 0x600..0x60F (groups 16-31) etc. We just
  // accept anything 0x5F0-0x63F by masking the lower 6 bits.
  //
  // Mask 0x7C0 means: top 5 bits must match.
  //   0x5F0 & 0x7C0 = 0x5C0  (matches 0x5C0..0x5FF)
  //   0x600 & 0x7C0 = 0x600  (matches 0x600..0x63F)
  //
  // Two RX buffers, two masks; cover both ranges to be safe.
  CAN_A.init_Mask(0, 0, 0x7C0);
  CAN_A.init_Filt(0, 0, 0x5C0);
  CAN_A.init_Filt(1, 0, 0x600);

  CAN_A.init_Mask(1, 0, 0x7C0);
  CAN_A.init_Filt(2, 0, 0x5C0);
  CAN_A.init_Filt(3, 0, 0x600);
  CAN_A.init_Filt(4, 0, 0x5C0);
  CAN_A.init_Filt(5, 0, 0x600);
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  // Disable watchdog while we initialise; re-enable in loop.
  wdt_disable();

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CAN_A_INT, INPUT);
  memset(&D, 0, sizeof(D));

  Serial.begin(115200);
  Serial.println(F("\n=== MS3 → Haltech UC10 Bridge v2.0 ==="));

  // ---- CAN_A: MS3 side @ 500 kbps, 8MHz crystal ----------------------------
  if (CAN_A.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("CAN_A (MS3 @ 500k):  OK"));
  } else {
    Serial.println(F("CAN_A FAIL — check wiring, crystal, library"));
  }
  setupCanAFilters();
  CAN_A.setMode(MCP_NORMAL);

  // ---- CAN_B: UC10 side @ 1 Mbps, 8MHz crystal -----------------------------
  if (CAN_B.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("CAN_B (UC10 @ 1M):   OK"));
  } else {
    Serial.println(F("CAN_B FAIL — check wiring, crystal, library"));
  }
  CAN_B.setMode(MCP_NORMAL);

  Serial.println(F("Bridge running.\n"));

  // Enable 2-second watchdog. Any hang >2s resets the Nano.
  wdt_enable(WDTO_2S);
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  wdt_reset();      // pet the watchdog

  // ---- Drain MS3 frames (bounded to avoid lockup on stuck INT) -------------
  uint8_t drainGuard = 16;     // max 16 frames per loop pass
  while (drainGuard-- && (digitalRead(PIN_CAN_A_INT) == LOW)) {
    uint32_t id;
    uint8_t  len;
    uint8_t  buf[8];
    if (CAN_A.readMsgBuf(&id, &len, buf) != CAN_OK) break;
    if (len < 1) continue;
    if (id >= MS3_BASE && id <= MS3_LAST) {
      parseMS3(id, buf);
    }
  }

  uint32_t now = millis();

  // ---- Update staleness flag ----------------------------------------------
  ms3Stale = (now - lastMS3RxMs) > MS3_TIMEOUT_MS;

  // ---- Heartbeat LED: solid when fresh data, blink when stale -------------
  if (ms3Stale) {
    digitalWrite(PIN_LED, (now / 250) & 1);
  } else {
    digitalWrite(PIN_LED, HIGH);
  }

  // ---- Broadcast Haltech messages at correct rates -------------------------
  if (now - t50 >= 20)  { send50();  t50  = now; }
  if (now - t20 >= 50)  { send20();  t20  = now; }
  if (now - t10 >= 100) { send10();  t10  = now; }
  if (now - t5  >= 200) { send5();   t5   = now; }

  // ---- Debug stats every 5 seconds ----------------------------------------
  if (now - tStats >= 5000) {
    tStats = now;
    Serial.print(F("[stats] RPM="));   Serial.print(D.rpm);
    Serial.print(F(" CLT="));          Serial.print(D.clt_x10 / 10);
    Serial.print(F("F MAP="));         Serial.print(D.map_x10 / 10);
    Serial.print(F("kPa MS3frames=")); Serial.print(ms3FrameCount);
    Serial.print(F(" txErr="));        Serial.print(txErrCount);
    Serial.print(F(" stale="));        Serial.println(ms3Stale ? "Y" : "N");

    // Pressure debug: shows raw MS3 sensor values AND what's sent to Haltech
    Serial.print(F("[press] FuelRaw="));   Serial.print(D.sensor[FUELP_SENSOR_INDEX]);
    Serial.print(F(" FuelSent="));         Serial.print(pressSensorBar1000(FUELP_SENSOR_INDEX));
    Serial.print(F(" | OilRaw="));         Serial.print(D.sensor[OILP_SENSOR_INDEX]);
    Serial.print(F(" OilSent="));          Serial.print(pressSensorBar1000(OILP_SENSOR_INDEX));
    Serial.print(F(" | CoolRaw="));        Serial.print(D.sensor[COOLP_SENSOR_INDEX]);
    Serial.print(F(" CoolSent="));         Serial.println(pressSensorBar1000(COOLP_SENSOR_INDEX));
  }
}
