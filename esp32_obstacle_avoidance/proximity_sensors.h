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
  // channel then always reports SENSOR_MM_ERROR.
  bool begin();

  // Fill out_mm[0..NUM_SENSORS-1] with the latest distance per channel (mm).
  void readAll(uint16_t out_mm[NUM_SENSORS]);

  // Whether channel `ch` initialised successfully.
  bool sensorOk(uint8_t ch) const {
    return (ch < NUM_SENSORS) ? _ok[ch] : false;
  }

 private:
  bool _ok[NUM_SENSORS] = {false};
};
