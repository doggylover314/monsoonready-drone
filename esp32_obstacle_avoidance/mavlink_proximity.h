// =============================================================================
//  mavlink_proximity.h  --  MAVLink layer (protocol-facing).
//
//  Turns an array of per-channel distances (mm, from the sensor layer) into
//  MAVLink v2 messages and sends them over a Stream (Serial2 -> Pixhawk):
//    * the 6 ring sensors  -> OBSTACLE_DISTANCE (id 330)
//    * the upward sensor   -> DISTANCE_SENSOR (id 132), orientation "up"
//  Knows NOTHING about the mux or the VL53L0X.
//
//  Sector mapping: with 6 ring sensors the angular increment is 60 deg, so
//  ArduPilot reads exactly distances[0..5]; mux channel N (via
//  SECTOR_FOR_CHANNEL) fills sector N. Sector 0 sits at
//  SENSOR_ANGLE_OFFSET_DEG clockwise from vehicle forward (sent as
//  angle_offset). Unused sectors (6..71) are set to "unknown".
// =============================================================================

#pragma once
#include <Arduino.h>
#include "config.h"

class MavlinkProximity {
 public:
  // Number of elements in the MAVLink OBSTACLE_DISTANCE distances[] field.
  static const uint16_t DISTANCES_LEN = 72;

  // Sentinel cm values written into the distances[] array. ArduPilot ignores
  // BOTH of these (as well as anything outside [min,max]), so they both read as
  // "this sector has no obstacle to avoid". They are kept distinct for the
  // benefit of ground stations / other consumers that honour the spec meanings.
  static const uint16_t SECTOR_NO_DATA = 65535;               // UINT16_MAX = unknown
  static const uint16_t SECTOR_NO_OBSTACLE = RANGE_MAX_CM + 1; // spec "no obstacle present"

  // Bind the output stream (e.g. Serial2). Call once in setup().
  void begin(Stream &out) { _out = &out; }

  // Build and transmit one OBSTACLE_DISTANCE message from the mm readings.
  // Only the ring channels (0..NUM_RING_SENSORS-1) are used.
  // Returns the number of bytes written to the stream.
  uint16_t sendObstacleDistance(const uint16_t sensor_mm[NUM_SENSORS]);

  // Build and transmit one DISTANCE_SENSOR message for the UPWARD sensor from
  // its raw mm reading. Returns bytes written (0 if the reading was a sensor
  // error, in which case nothing is sent this cycle).
  uint16_t sendDistanceSensorUp(uint16_t mm);

  // Emit a low-rate HEARTBEAT so the autopilot/GCS recognises this component.
  // Returns bytes written.
  uint16_t sendHeartbeat();

  // Emit a STATUSTEXT (severity INFO). ArduPilot forwards it to every GCS
  // and records it in the .BIN log, which makes it the one channel that
  // reaches the field operator without the debug USB. Max 49 chars used.
  uint16_t sendStatusText(const char *text);

  // Convert one raw sensor reading (mm) into the cm value that goes into the
  // distances[] array (or a SECTOR_NO_* sentinel). Exposed static so the debug
  // log can show exactly what each sector will report. Pure function.
  static uint16_t classifyCm(uint16_t mm);

  // Fill a caller-provided distances[DISTANCES_LEN] array (cm). Exposed for
  // testing / debugging; sendObstacleDistance() uses it internally.
  void buildDistanceArray(const uint16_t sensor_mm[NUM_SENSORS],
                          uint16_t out_cm[DISTANCES_LEN]);

 private:
  Stream *_out = nullptr;
};
