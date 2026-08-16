// =============================================================================
//  proximity_sensors.h  --  Sensor-reading layer (hardware-facing).
//
//  Knows about the TCA9548A mux and the 7x VL53L0X ToF sensors (channels 0..5
//  = horizontal ring, channel 6 = pointing up; identical handling here). Knows
//  NOTHING
//  about MAVLink. Its only job is to hand back the latest distance (in mm) for
//  each of the NUM_SENSORS channels.
//
//  Returned buffer conventions (millimetres):
//    * SENSOR_MM_ERROR (0xFFFF)      -> I2C timeout / sensor absent (no data)
//    * value >= VL53L0X_MAX_VALID_MM -> valid read but no target in range (clear)
//    * anything else                 -> a real distance in mm
//
//  When USE_FAKE_SENSORS == 1 this class touches no hardware and returns the
//  simulated values configured in config.h.
// =============================================================================

#pragma once
#include <Arduino.h>
#include "config.h"

class ProximitySensors {
 public:
  // Initialise the mux + sensors (or the fake generator). Returns true only if
  // every sensor came up; individual failures are tracked per channel and that
  // channel reports SENSOR_MM_ERROR until maintain() manages to revive it.
  bool begin();

  // Fill out_mm[0..NUM_SENSORS-1] with the latest distance per channel (mm).
  void readAll(uint16_t out_mm[NUM_SENSORS]);

  // SELF-HEALING (added 2026-08-16). Call every loop(); internally
  // rate-limited to one re-init attempt per SENSOR_RETRY_PERIOD_MS.
  //
  // WHY: begin() used to be the only init that ever ran, so a channel that
  // failed at boot, or glitched mid-session, was dead until the next power
  // cycle. That is precisely the observed farm behaviour: the ring only ever
  // LOST sectors across a powered afternoon (2 dead -> 3 -> 4) and a cold
  // boot brought them back. A VL53L0X that browns out or drops its bus
  // contact stops its continuous-ranging mode, after which every read times
  // out even though the chip answers its address; only init() +
  // startContinuous() bring it back, and nothing called them.
  //
  // Scope: revives boot-init failures and runtime dropouts (the ch5-class
  // faults). It cannot revive genuinely dead hardware (ch2: ACKs its
  // address, never completes init, at every bus speed). A failed attempt
  // leaves the channel exactly as it was: reporting SENSOR_MM_ERROR, which
  // reaches ArduPilot as "unknown", never as "clear".
  void maintain();

  // Whether channel `ch` is currently initialised and reading.
  bool sensorOk(uint8_t ch) const {
    return (ch < NUM_SENSORS) ? _ok[ch] : false;
  }

  // Currently-healthy channel count out of the fitted ring channels (for the
  // health STATUSTEXT). "Healthy" = initialised and not streaming errors.
  uint8_t ringHealthy() const;
  uint8_t ringFitted() const;

 private:
  bool initChannel(uint8_t ch);        // shared by begin() and maintain()
  bool channelHealthy(uint8_t ch) const {
    return _ok[ch] && _err[ch] < SENSOR_RETRY_ERR_READS;
  }
  bool _ok[NUM_SENSORS] = {false};
  // Consecutive failed reads per channel; SENSOR_RETRY_ERR_READS in a row
  // marks an _ok channel as needing re-init (its continuous mode is gone).
  uint8_t _err[NUM_SENSORS] = {0};
  uint32_t _last_retry_ms = 0;
  uint32_t _last_bus_clear_ms = 0;
  uint8_t _retry_ch = 0;               // round-robin cursor
};
