# 1 "/tmp/tmppf7pdr_j"
#include <Arduino.h>
# 1 "/media/sleuther/Stuff/Robu AI Challenge/esp32_obstacle_avoidance/esp32_obstacle_avoidance.ino"
# 26 "/media/sleuther/Stuff/Robu AI Challenge/esp32_obstacle_avoidance/esp32_obstacle_avoidance.ino"
#include "config.h"
#include "proximity_sensors.h"
#include "mavlink_proximity.h"
#include "i2c_diag.h"

ProximitySensors sensors;
MavlinkProximity mav;

static uint16_t latest_mm[NUM_SENSORS];
static uint32_t last_tx_ms = 0;
static uint32_t last_hb_ms = 0;
static uint32_t tx_count = 0;
static void printBanner();
static void printReadings(uint32_t now_ms, const uint16_t mm[NUM_SENSORS],
                          uint16_t tx_bytes);
void setup();
void loop();
#line 40 "/media/sleuther/Stuff/Robu AI Challenge/esp32_obstacle_avoidance/esp32_obstacle_avoidance.ino"
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
  {



    unsigned fitted = 0;
    DEBUG_SERIAL.printf(" SENSORS       : ring %.0f deg spacing + UP (ch%u). Fitted:",
                        (double)SECTOR_INCREMENT_DEG,
                        (unsigned)UP_SENSOR_CHANNEL);
    for (uint8_t ch = 0; ch < NUM_RING_SENSORS; ch++) {
      if (!RING_SENSOR_FITTED[ch]) continue;
      fitted++;
      DEBUG_SERIAL.printf(" ch%u(%.0fdeg)", ch,
                          (double)(SENSOR_ANGLE_OFFSET_DEG +
                                   SECTOR_FOR_CHANNEL[ch] * SECTOR_INCREMENT_DEG));
    }
    DEBUG_SERIAL.printf("\n                 %u of %u ring sensors; the rest are "
                        "reported as UNKNOWN (never as clear)\n",
                        fitted, (unsigned)NUM_RING_SENSORS);
  }
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




static void printReadings(uint32_t now_ms, const uint16_t mm[NUM_SENSORS],
                          uint16_t tx_bytes) {
  DEBUG_SERIAL.printf("[%8lu ms] ", (unsigned long)now_ms);
  for (uint8_t ch = 0; ch < NUM_RING_SENSORS; ch++) {
    uint8_t sector = SECTOR_FOR_CHANNEL[ch];
    uint16_t v = MavlinkProximity::classifyCm(mm[ch]);
    unsigned bearing = (unsigned)(SENSOR_ANGLE_OFFSET_DEG +
                                  sector * SECTOR_INCREMENT_DEG + 0.5f) % 360u;
    DEBUG_SERIAL.printf("s%u[%3u]:", (unsigned)sector, bearing);


    if (!RING_SENSOR_FITTED[ch]) DEBUG_SERIAL.print("----  ");
    else if (v == MavlinkProximity::SECTOR_NO_DATA) DEBUG_SERIAL.print("ERR   ");
    else if (v == MavlinkProximity::SECTOR_NO_OBSTACLE) DEBUG_SERIAL.print("CLEAR ");
    else DEBUG_SERIAL.printf("%3ucm ", (unsigned)v);
  }
  uint16_t up = MavlinkProximity::classifyCm(mm[UP_SENSOR_CHANNEL]);
  DEBUG_SERIAL.print("UP:");
  if (up == MavlinkProximity::SECTOR_NO_DATA) DEBUG_SERIAL.print("ERR   ");
  else if (up == MavlinkProximity::SECTOR_NO_OBSTACLE) DEBUG_SERIAL.print("CLEAR ");
  else DEBUG_SERIAL.printf("%3ucm ", (unsigned)up);
  DEBUG_SERIAL.printf("| TX %u B  (#%lu)\n",
                      (unsigned)tx_bytes, (unsigned long)tx_count);
}


void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(300);
#if RUN_I2C_DIAG


  runI2cDiag();
  return;
#endif
#if USE_FAKE_SENSORS
  pinMode(FAKE_TX_ENABLE_PIN, INPUT_PULLUP);
#endif
  printBanner();


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


void loop() {
#if RUN_I2C_DIAG


  delay(1000);
  return;
#else


  sensors.readAll(latest_mm);

  uint32_t now = millis();



#if USE_FAKE_SENSORS
  bool tx_allowed = (digitalRead(FAKE_TX_ENABLE_PIN) == LOW);
#else
  const bool tx_allowed = true;
#endif


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

  if (now - last_hb_ms >= HEARTBEAT_PERIOD_MS) {
    last_hb_ms = now;
    if (tx_allowed) mav.sendHeartbeat();
#if USE_FAKE_SENSORS
    else DEBUG_SERIAL.printf("[guard] FAKE mode, TX BLOCKED: jumper GPIO%u to GND to enable\n",
                             (unsigned)FAKE_TX_ENABLE_PIN);
#endif
  }
#endif
#endif
}