// =============================================================================
//  mavlink_proximity.cpp  --  Implementation of the MAVLink layer.
// =============================================================================

#include "mavlink_proximity.h"

// Official MAVLink v2 C library. Installed via:
//   PlatformIO : lib_deps = okalachev/MAVLink
//   Arduino IDE: Library Manager -> "MAVLink" (by Oleg Kalachev)
// OBSTACLE_DISTANCE (msg 330) lives in the "common" dialect, which <MAVLink.h>
// pulls in. The generated pack function handles the v2 framing, the field
// re-ordering, and the message CRC (CRC_EXTRA = 23) for us.
//
// If you instead drop the raw mavlink/c_library_v2 headers into your libraries
// folder, change this include to <common/mavlink.h>.
#include <MAVLink.h>

// -----------------------------------------------------------------------------
uint16_t MavlinkProximity::classifyCm(uint16_t mm) {
  if (mm == SENSOR_MM_ERROR) return SECTOR_NO_DATA;      // read failed -> unknown
  if (mm == 0) return SECTOR_NO_DATA;                    // 0 mm = VL53L0X signal-fail, not a
                                                         // real touch -> unknown (don't fake a
                                                         // 5 cm obstacle out of a glitch)
  if (mm >= VL53L0X_MAX_VALID_MM) return SECTOR_NO_OBSTACLE; // nothing in range -> clear

  uint16_t cm = mm / 10;
  if (cm < RANGE_MIN_CM) cm = RANGE_MIN_CM;   // genuinely very close (<5 cm): clamp up so it stays valid
  if (cm > RANGE_MAX_CM) return SECTOR_NO_OBSTACLE; // real but beyond trusted range -> clear
  return cm;                                   // a real obstacle distance in cm
}

// -----------------------------------------------------------------------------
void MavlinkProximity::buildDistanceArray(const uint16_t sensor_mm[NUM_SENSORS],
                                          uint16_t out_cm[DISTANCES_LEN]) {
  // Every sector defaults to "unknown"; we only have real data for 6 of them.
  for (uint16_t i = 0; i < DISTANCES_LEN; i++) out_cm[i] = SECTOR_NO_DATA;

  for (uint8_t ch = 0; ch < NUM_RING_SENSORS; ch++) {
    uint8_t sector = SECTOR_FOR_CHANNEL[ch];
    if (sector < DISTANCES_LEN) out_cm[sector] = classifyCm(sensor_mm[ch]);
  }
}

// -----------------------------------------------------------------------------
uint16_t MavlinkProximity::sendObstacleDistance(const uint16_t sensor_mm[NUM_SENSORS]) {
  uint16_t distances[DISTANCES_LEN];
  buildDistanceArray(sensor_mm, distances);

  mavlink_message_t msg;
  static uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_obstacle_distance_pack(
      MAV_SYSID, MAV_COMPID, &msg,
      (uint64_t)micros(),                        // time_usec (monotonic us)
      MAV_DISTANCE_SENSOR_LASER,                 // sensor_type = 0 (laser/ToF)
      distances,                                 // distances[72] in cm
      (uint8_t)(SECTOR_INCREMENT_DEG + 0.5f),    // increment (deg, integer)
      (uint16_t)RANGE_MIN_CM,                    // min_distance (cm)
      (uint16_t)RANGE_MAX_CM,                    // max_distance (cm)
      (float)SECTOR_INCREMENT_DEG,               // increment_f (deg) - ArduPilot prefers this
      (float)SENSOR_ANGLE_OFFSET_DEG,            // angle_offset: bearing of sector 0
      MAV_FRAME_BODY_FRD);                       // body-fixed frame (front-right-down)

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (_out) _out->write(buf, len);
  return len;
}

// -----------------------------------------------------------------------------
uint16_t MavlinkProximity::sendDistanceSensorUp(uint16_t mm) {
  uint16_t cm = classifyCm(mm);
  if (cm == SECTOR_NO_DATA) return 0; // read failed -> send nothing this cycle

  // A "clear" reading is RANGE_MAX_CM + 1, which lies outside [min,max], so
  // ArduPilot treats it as "nothing overhead". Real distances pass through.
  mavlink_message_t msg;
  static uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_distance_sensor_pack(
      MAV_SYSID, MAV_COMPID, &msg,
      millis(),                     // time_boot_ms
      (uint16_t)RANGE_MIN_CM,       // min_distance (cm)
      (uint16_t)RANGE_MAX_CM,       // max_distance (cm)
      cm,                           // current_distance (cm)
      MAV_DISTANCE_SENSOR_LASER,    // type = laser/ToF
      UP_SENSOR_CHANNEL,            // id: any stable per-sensor value
      MAV_SENSOR_ROTATION_PITCH_90, // orientation 24 = pointing straight UP
      UINT8_MAX,                    // covariance: unknown
      0.0f, 0.0f,                   // horizontal/vertical FOV: unknown
      NULL,                         // quaternion: unused for a fixed orientation
      0);                           // signal_quality: 0 = unknown

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (_out) _out->write(buf, len);
  return len;
}

// -----------------------------------------------------------------------------
uint16_t MavlinkProximity::sendStatusText(const char *text, uint8_t severity) {
  // The header hard-codes these (MAVLink.h is not visible there); if the
  // library enum ever disagreed, fail the build instead of mislabeling.
  static_assert(MavlinkProximity::SEV_WARNING == MAV_SEVERITY_WARNING,
                "SEV_WARNING out of sync with MAVLink enum");
  static_assert(MavlinkProximity::SEV_INFO == MAV_SEVERITY_INFO,
                "SEV_INFO out of sync with MAVLink enum");

  mavlink_message_t msg;
  static uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STATUSTEXT carries exactly 50 chars; pack() reads that many, so feed it
  // a bounded, NUL-padded copy rather than trusting the caller's length.
  char padded[50] = {0};
  strncpy(padded, text, sizeof(padded) - 1);

  mavlink_msg_statustext_pack(
      MAV_SYSID, MAV_COMPID, &msg,
      severity,
      padded,
      0, 0);                       // id/chunk_seq: single-chunk message

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (_out) _out->write(buf, len);
  return len;
}

// -----------------------------------------------------------------------------
uint16_t MavlinkProximity::sendHeartbeat() {
  mavlink_message_t msg;
  static uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
      MAV_SYSID, MAV_COMPID, &msg,
      MAV_TYPE_ONBOARD_CONTROLLER, // we are a companion/onboard controller
      MAV_AUTOPILOT_INVALID,       // not an autopilot
      0,                           // base_mode
      0,                           // custom_mode
      MAV_STATE_ACTIVE);           // system_status

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (_out) _out->write(buf, len);
  return len;
}
