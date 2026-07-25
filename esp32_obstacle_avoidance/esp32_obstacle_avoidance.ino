// =============================================================================
//  esp32_obstacle_avoidance.ino
//
//  Obstacle-avoidance sensor module for an ArduPilot Pixhawk.
//
//  Hardware : ESP32-WROOM-32
//             7x VL53L0X time-of-flight sensors on a TCA9548A I2C multiplexer:
//             mux channels 0..5 in a horizontal ring (one per hexa arm, 60 deg
//             apart) and channel 6 pointing straight UP.
//             ESP32 Serial2 wired to the Pixhawk (TELEM2 / SERIAL2).
//
//  Function : read all 7 sensors as fast as practical, send the 6 ring readings
//             as a MAVLink v2 OBSTACLE_DISTANCE message (cm, one 60-deg sector
//             per sensor) and the up reading as DISTANCE_SENSOR (orientation
//             up), both over Serial2 at 10 Hz. Prints all distances plus a TX
//             confirmation over USB.
//
//  Layers   : proximity_sensors.*  -> hardware (mux + VL53L0X)  [no MAVLink]
//             mavlink_proximity.*   -> MAVLink OBSTACLE_DISTANCE [no hardware]
//             this file             -> wiring of the two + scheduling + debug
//
//  See config.h for all pins, baud rates, ranges, and the USE_FAKE_SENSORS
//  test flag. See README.md for the ArduPilot parameters to set on the Pixhawk.
// =============================================================================

#include "config.h"
#include "proximity_sensors.h"
#include "mavlink_proximity.h"

ProximitySensors sensors;
MavlinkProximity mav;

static uint16_t latest_mm[NUM_SENSORS]; // freshest reading per channel (mm)
static uint32_t last_tx_ms = 0;
static uint32_t last_hb_ms = 0;
static uint32_t tx_count = 0;

// -----------------------------------------------------------------------------
static void printBanner() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================================"));
  DEBUG_SERIAL.println(F(" ESP32 Obstacle Avoidance (6 ring + 1 up) -> ArduPilot (MAVLink)"));
  DEBUG_SERIAL.println(F("========================================================"));
#if USE_FAKE_SENSORS
  DEBUG_SERIAL.println(F(" MODE          : FAKE SENSORS (no hardware required)"));
  DEBUG_SERIAL.printf (" FAKE          : all %u mm, channel %u = %u mm\n",
                       (unsigned)FAKE_CLEAR_MM, (unsigned)FAKE_VARY_CHANNEL,
                       (unsigned)FAKE_VARY_MM);
  DEBUG_SERIAL.printf (" TX GUARD      : MAVLink TX only while GPIO%u is jumpered to GND\n",
                       (unsigned)FAKE_TX_ENABLE_PIN);
#else
  DEBUG_SERIAL.println(F(" MODE          : REAL VL53L0X via TCA9548A mux"));
#endif
  DEBUG_SERIAL.printf (" SENSORS       : %u ring @ %.0f deg spacing + 1 UP (ch%u)\n",
                       (unsigned)NUM_RING_SENSORS, (double)SECTOR_INCREMENT_DEG,
                       (unsigned)UP_SENSOR_CHANNEL);
  DEBUG_SERIAL.printf (" REPORT RANGE  : %u..%u cm (OBSTACLE_DISTANCE min/max)\n",
                       (unsigned)RANGE_MIN_CM, (unsigned)RANGE_MAX_CM);
  DEBUG_SERIAL.printf (" PIXHAWK LINK  : Serial2 @ %lu baud (TX=GPIO%u, RX=GPIO%u)\n",
                       (unsigned long)PIXHAWK_BAUD, (unsigned)PIXHAWK_TX_PIN,
                       (unsigned)PIXHAWK_RX_PIN);
  DEBUG_SERIAL.printf (" MAVLINK ID    : sysid %u, compid %u\n",
                       (unsigned)MAV_SYSID, (unsigned)MAV_COMPID);
  DEBUG_SERIAL.printf (" TX RATE       : %u Hz OBSTACLE_DISTANCE + DISTANCE_SENSOR(up)\n",
                       (unsigned)TX_RATE_HZ);
  DEBUG_SERIAL.printf (" SECTORS       : bearing = %.0f + 60*s deg clockwise from nose\n",
                       (double)SENSOR_ANGLE_OFFSET_DEG);
  DEBUG_SERIAL.println(F("--------------------------------------------------------"));
}

// -----------------------------------------------------------------------------
// One human-readable line: the 6 ring sectors (as they will appear in the
// MAVLink message) plus the UP sensor, followed by the TX confirmation.
static void printReadings(uint32_t now_ms, const uint16_t mm[NUM_SENSORS],
                          uint16_t tx_bytes) {
  DEBUG_SERIAL.printf("[%8lu ms] ", (unsigned long)now_ms);
  for (uint8_t ch = 0; ch < NUM_RING_SENSORS; ch++) {
    uint8_t sector = SECTOR_FOR_CHANNEL[ch];
    uint16_t v = MavlinkProximity::classifyCm(mm[ch]);
    unsigned bearing = (unsigned)(SENSOR_ANGLE_OFFSET_DEG +
                                  sector * SECTOR_INCREMENT_DEG + 0.5f) % 360u;
    DEBUG_SERIAL.printf("s%u[%3u]:", (unsigned)sector, bearing);
    if (v == MavlinkProximity::SECTOR_NO_DATA)          DEBUG_SERIAL.print("ERR   ");
    else if (v == MavlinkProximity::SECTOR_NO_OBSTACLE) DEBUG_SERIAL.print("CLEAR ");
    else                                                DEBUG_SERIAL.printf("%3ucm ", (unsigned)v);
  }
  uint16_t up = MavlinkProximity::classifyCm(mm[UP_SENSOR_CHANNEL]);
  DEBUG_SERIAL.print("UP:");
  if (up == MavlinkProximity::SECTOR_NO_DATA)          DEBUG_SERIAL.print("ERR   ");
  else if (up == MavlinkProximity::SECTOR_NO_OBSTACLE) DEBUG_SERIAL.print("CLEAR ");
  else                                                 DEBUG_SERIAL.printf("%3ucm ", (unsigned)up);
  DEBUG_SERIAL.printf("| TX %u B  (#%lu)\n",
                      (unsigned)tx_bytes, (unsigned long)tx_count);
}

// -----------------------------------------------------------------------------
void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(300);
#if USE_FAKE_SENSORS
  pinMode(FAKE_TX_ENABLE_PIN, INPUT_PULLUP); // bench jumper to GND enables TX
#endif
  printBanner();

  // Serial2 -> Pixhawk. Explicit pins so it works regardless of core defaults.
  PIXHAWK_SERIAL.begin(PIXHAWK_BAUD, SERIAL_8N1, PIXHAWK_RX_PIN, PIXHAWK_TX_PIN);
  mav.begin(PIXHAWK_SERIAL);

  if (!sensors.begin()) {
    DEBUG_SERIAL.println(F("[setup] WARNING: not all sensors initialised; "
                           "those channels will report ERR/CLEAR."));
  }
  DEBUG_SERIAL.println(F("[setup] running.\n"));

  uint32_t now = millis();
  last_tx_ms = now;
  last_hb_ms = now;
}

// -----------------------------------------------------------------------------
void loop() {
  // Read every iteration -> the buffer always holds the freshest sample when
  // we send. The VL53L0X units keep ranging continuously in the background.
  sensors.readAll(latest_mm);

  uint32_t now = millis();

  // Fake-mode flight guard: without the bench jumper, fake data never
  // reaches the Pixhawk (a wrong-firmware flash then fails safe in flight).
#if USE_FAKE_SENSORS
  bool tx_allowed = (digitalRead(FAKE_TX_ENABLE_PIN) == LOW);
#else
  const bool tx_allowed = true;
#endif

  // --- OBSTACLE_DISTANCE (ring) + DISTANCE_SENSOR (up) at 10 Hz ---
  if (now - last_tx_ms >= TX_PERIOD_MS) {
    last_tx_ms = now;
    uint16_t sent = 0;
    if (tx_allowed) {
      sent = mav.sendObstacleDistance(latest_mm);
      sent += mav.sendDistanceSensorUp(latest_mm[UP_SENSOR_CHANNEL]);
      tx_count++;
    }
    printReadings(now, latest_mm, sent);
  }

#if SEND_HEARTBEAT
  // --- HEARTBEAT at 1 Hz (helps the autopilot/GCS see this component) ---
  if (now - last_hb_ms >= HEARTBEAT_PERIOD_MS) {
    last_hb_ms = now;
    if (tx_allowed) mav.sendHeartbeat();
#if USE_FAKE_SENSORS
    else DEBUG_SERIAL.printf("[guard] FAKE mode, TX BLOCKED: jumper GPIO%u to GND to enable\n",
                             (unsigned)FAKE_TX_ENABLE_PIN);
#endif
  }
#endif
}
