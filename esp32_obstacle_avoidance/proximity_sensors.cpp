// =============================================================================
//  proximity_sensors.cpp  --  Implementation of the sensor-reading layer.
// =============================================================================

#include "proximity_sensors.h"

#if !USE_FAKE_SENSORS
// ---- Real hardware path ----------------------------------------------------
#include <Wire.h>
#include <VL53L0X.h> // Pololu VL53L0X library (Library Manager / lib_deps = pololu/VL53L0X)

// One driver object per mux channel. Pololu staff note: use a SEPARATE object
// per sensor, otherwise every channel reports the same value.
static VL53L0X g_sensor[NUM_SENSORS];

// Is a sensor physically present on this mux channel? Ring channels come from
// RING_SENSOR_FITTED, the up channel from UP_SENSOR_FITTED (both in config.h).
static bool isFitted(uint8_t ch) {
  if (ch == UP_SENSOR_CHANNEL) return UP_SENSOR_FITTED;
  return (ch < NUM_RING_SENSORS) ? RING_SENSOR_FITTED[ch] : false;
}

// Select a single downstream channel on the TCA9548A. The control register is a
// bitmask, so writing (1 << ch) enables exactly that one channel and disables
// the rest. Call this before every transaction with the sensor on that channel.
static void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
#if TCA_SETTLE_US > 0
  delayMicroseconds(TCA_SETTLE_US);
#endif
}
#endif // !USE_FAKE_SENSORS

// -----------------------------------------------------------------------------
// Full bring-up of one channel: select, init, timing budget, continuous mode.
// Shared by begin() (boot) and maintain() (self-healing re-init), so the two
// paths can never configure a sensor differently.
bool ProximitySensors::initChannel(uint8_t ch) {
#if USE_FAKE_SENSORS
  (void)ch;
  return true;
#else
  tcaselect(ch);
  g_sensor[ch].setTimeout(SENSOR_TIMEOUT_MS);
  if (!g_sensor[ch].init()) return false;

  // Timing budget MUST be set before startContinuous() or it has no effect.
  g_sensor[ch].setMeasurementTimingBudget(SENSOR_TIMING_BUDGET_US);

#if VL53L0X_LONG_RANGE
  // Longer reach (~2 m) at the cost of speed / low-IR requirement.
  g_sensor[ch].setSignalRateLimit(0.1);
  g_sensor[ch].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  g_sensor[ch].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

  // Back-to-back continuous ranging. All sensors integrate in parallel; the
  // mux only time-shares who we can *read*, not who is *measuring*.
  g_sensor[ch].startContinuous();
  return true;
#endif
}

// -----------------------------------------------------------------------------
bool ProximitySensors::begin() {
#if USE_FAKE_SENSORS
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) _ok[ch] = isFitted(ch);
  DEBUG_SERIAL.println(F("[sensors] FAKE mode: no hardware in use."));
  return true;
#else
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  bool all_ok = true;
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    // Channels declared empty in config.h are skipped entirely: probing one
    // costs about a second of boot time waiting for a timeout, and reports a
    // "FAILED" that hides the failures that matter. _ok stays false, so
    // readAll() keeps returning SENSOR_MM_ERROR for it and the sector goes
    // out as "unknown", which is the correct thing to tell an autopilot
    // about a direction we cannot see.
    if (!isFitted(ch)) {
      _ok[ch] = false;
      DEBUG_SERIAL.printf("[sensors] ch%u: not fitted (config.h), sector "
                          "reported as unknown\n", ch);
      continue;
    }
    _ok[ch] = initChannel(ch);
    if (_ok[ch]) {
      DEBUG_SERIAL.printf("[sensors] ch%u: VL53L0X OK\n", ch);
    } else {
      all_ok = false;
      DEBUG_SERIAL.printf("[sensors] ch%u: VL53L0X init FAILED (check "
                          "wiring/mux); will keep retrying every %u ms\n",
                          ch, (unsigned)SENSOR_RETRY_PERIOD_MS);
    }
  }
  return all_ok;
#endif
}

// -----------------------------------------------------------------------------
void ProximitySensors::readAll(uint16_t out_mm[NUM_SENSORS]) {
#if USE_FAKE_SENSORS
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) out_mm[ch] = FAKE_CLEAR_MM;
  if (FAKE_VARY_CHANNEL < NUM_SENSORS) out_mm[FAKE_VARY_CHANNEL] = FAKE_VARY_MM;
#else
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    if (!_ok[ch]) {
      out_mm[ch] = SENSOR_MM_ERROR;
      continue;
    }
    tcaselect(ch);
    uint16_t mm = g_sensor[ch].readRangeContinuousMillimeters();
    // A read timeout means the transaction failed (dead sensor / bus glitch);
    // distinguish it from a valid "no target" reading (large mm value).
    if (g_sensor[ch].timeoutOccurred()) {
      out_mm[ch] = SENSOR_MM_ERROR;
      // Enough consecutive timeouts = the sensor's continuous mode is gone
      // (brown-out or contact bounce); flag it for maintain() to re-init.
      if (_err[ch] < 255) _err[ch]++;
    } else {
      out_mm[ch] = mm;
      _err[ch] = 0;
    }
  }
#endif
}

// -----------------------------------------------------------------------------
void ProximitySensors::maintain() {
#if !USE_FAKE_SENSORS
  uint32_t now = millis();
  if (now - _last_retry_ms < SENSOR_RETRY_PERIOD_MS) return;

  // One attempt per period, round-robin, so a permanently dead chip (ch2)
  // costs one bounded init attempt every few periods, never a tight loop.
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint8_t ch = _retry_ch;
    _retry_ch = (uint8_t)((_retry_ch + 1) % NUM_SENSORS);
    if (!isFitted(ch)) continue;
    bool needs = !_ok[ch] || _err[ch] >= SENSOR_RETRY_ERR_READS;
    if (!needs) continue;

    _last_retry_ms = now;      // only a real attempt consumes the period
    bool was_ok = _ok[ch];
    if (initChannel(ch)) {
      _ok[ch] = true;
      _err[ch] = 0;
      DEBUG_SERIAL.printf("[sensors] ch%u: re-init OK%s\n", ch,
                          was_ok ? " (was timing out)" : " (was dead)");
    } else {
      _ok[ch] = false;
      DEBUG_SERIAL.printf("[sensors] ch%u: re-init failed, next try in "
                          "%u ms\n", ch, (unsigned)SENSOR_RETRY_PERIOD_MS);
    }
    return;
  }
#endif
}
