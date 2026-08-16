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
#include "i2c_diag.h"

ProximitySensors sensors;
MavlinkProximity mav;

static uint16_t latest_mm[NUM_SENSORS]; // freshest reading per channel (mm)
static uint32_t last_tx_ms = 0;
static uint32_t last_hb_ms = 0;
static uint32_t last_health_ms = 0;
static uint32_t last_event_ms = 0;
static uint16_t health_prev = 0;        // bitmask snapshot for change detection
static uint32_t tx_count = 0;

// -----------------------------------------------------------------------------
// One bit per healthy channel (fitted ring channels + up). Comparing two
// snapshots tells us exactly which sensor was lost or revived, so the GCS
// hears about a mid-flight dropout the moment it happens instead of up to
// 15 s later in the periodic summary.
static uint16_t healthMask() {
  uint16_t m = 0;
  for (uint8_t ch = 0; ch < NUM_RING_SENSORS; ch++)
    if (RING_SENSOR_FITTED[ch] && sensors.channelHealthy(ch)) m |= (uint16_t)1u << ch;
  if (sensors.channelHealthy(UP_SENSOR_CHANNEL)) m |= (uint16_t)1u << UP_SENSOR_CHANNEL;
  return m;
}

// "prx LOST ch4 BACK up | ring 4/6" -- only the sections that apply.
static void formatHealthEvent(uint16_t lost, uint16_t back, char *out, size_t n) {
  size_t p = (size_t)snprintf(out, n, "prx");
  for (int pass = 0; pass < 2; pass++) {
    uint16_t bits = pass == 0 ? lost : back;
    if (!bits || p >= n) continue;
    p += (size_t)snprintf(out + p, n - p, pass == 0 ? " LOST" : " BACK");
    for (uint8_t ch = 0; ch < NUM_SENSORS && p < n; ch++) {
      if (!(bits & ((uint16_t)1u << ch))) continue;
      if (ch == UP_SENSOR_CHANNEL) p += (size_t)snprintf(out + p, n - p, " up");
      else                         p += (size_t)snprintf(out + p, n - p, " ch%u", ch);
    }
  }
  if (p < n)
    snprintf(out + p, n - p, " | ring %u/%u",
             (unsigned)sensors.ringHealthy(), (unsigned)sensors.ringFitted());
}

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
  {
    // Spell out which ring channels are fitted and which bearings are
    // therefore unreported: flying with a partial ring is fine, but only if
    // nobody is under the impression it covers 360 degrees.
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
    // "----" = no sensor fitted here, which is a configuration fact, not a
    // fault. Distinguished from ERR so a real failure still stands out.
    if (!RING_SENSOR_FITTED[ch])                        DEBUG_SERIAL.print("----  ");
    else if (v == MavlinkProximity::SECTOR_NO_DATA)     DEBUG_SERIAL.print("ERR   ");
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
#if RUN_I2C_DIAG
  // Bench mode: report on the I2C bus and stop. Deliberately before the
  // Pixhawk link is opened, so a diagnostic build cannot transmit anything.
  runI2cDiag();
  return;
#endif
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
  // Baseline for change detection: channels dead at boot are covered by the
  // banner and the begin() warning, not announced as a "change". Backdate the
  // periodic timer so the FIRST health summary goes out on the first loop
  // pass; the GCS learns the boot state immediately, not 15 s in.
  health_prev = healthMask();
  last_health_ms = now - HEALTH_TEXT_PERIOD_MS;
}

// -----------------------------------------------------------------------------
void loop() {
#if RUN_I2C_DIAG
  // Diagnostic build: the report already ran in setup(). Idle rather than
  // repeat it, so the output stays readable in the serial monitor.
  delay(1000);
  return;
#else
  // Read every iteration -> the buffer always holds the freshest sample when
  // we send. The VL53L0X units keep ranging continuously in the background.
  sensors.readAll(latest_mm);

  // Self-healing: re-init channels that failed at boot or stopped answering
  // (rate-limited inside; at most one bounded attempt per 5 s), and clear
  // the whole I2C bus if every fitted channel dies at once. Before this, a
  // mid-session dropout was dead until the next power cycle.
  sensors.maintain();

  // The Pixhawk streams its own telemetry at us on this port (SERIAL1 is
  // MAVLink2). Nothing here reads it, so drain it and keep the RX buffer
  // from sitting permanently full.
  while (PIXHAWK_SERIAL.available()) PIXHAWK_SERIAL.read();

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
#endif  // SEND_HEARTBEAT

  // --- Ring health, two mechanisms (2026-08-16, ESP32 reliability pass):
  //
  // 1. EVENT: any sensor lost or revived is announced IMMEDIATELY, severity
  //    WARNING on a loss (pops in the GCS) and INFO on a recovery. Serial
  //    always prints the event even when MAVLink TX is jumper-blocked.
  // 2. PERIODIC: every 15 s a summary STATUSTEXT plus a per-channel serial
  //    line with the live error counters, so a slow decay is visible from
  //    the serial monitor before it costs a sector.
  {
    uint16_t health_now = healthMask();
    if (health_now != health_prev &&
        now - last_event_ms >= HEALTH_EVENT_MIN_GAP_MS) {
      uint16_t lost = health_prev & ~health_now;
      uint16_t back = health_now & ~health_prev;
      char text[50];
      formatHealthEvent(lost, back, text, sizeof(text));
      if (tx_allowed)
        mav.sendStatusText(text, lost ? MavlinkProximity::SEV_WARNING
                                      : MavlinkProximity::SEV_INFO);
      DEBUG_SERIAL.printf("[health] EVENT %s%s\n", text,
                          tx_allowed ? "" : " (TX blocked)");
      health_prev = health_now;
      last_event_ms = now;
    }
  }

  if (now - last_health_ms >= HEALTH_TEXT_PERIOD_MS) {
    last_health_ms = now;
    char text[50];
    snprintf(text, sizeof(text), "prx ring %u/%u up:%s",
             (unsigned)sensors.ringHealthy(),
             (unsigned)sensors.ringFitted(),
             (latest_mm[UP_SENSOR_CHANNEL] == SENSOR_MM_ERROR
              || !sensors.channelHealthy(UP_SENSOR_CHANNEL)) ? "ERR" : "ok");
    if (tx_allowed) mav.sendStatusText(text);
    DEBUG_SERIAL.printf("[health] %s%s | per-channel:",
                        text, tx_allowed ? "" : " (TX blocked)");
    for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
      char label[8];
      if (ch == UP_SENSOR_CHANNEL) snprintf(label, sizeof(label), "up");
      else                         snprintf(label, sizeof(label), "ch%u", ch);
      if (ch < NUM_RING_SENSORS && !RING_SENSOR_FITTED[ch])
        DEBUG_SERIAL.printf(" %s:--", label);          // not fitted
      else if (sensors.channelHealthy(ch))
        DEBUG_SERIAL.printf(" %s:ok", label);
      else
        DEBUG_SERIAL.printf(" %s:DOWN(e%u)", label,
                            (unsigned)sensors.errCount(ch));
    }
    DEBUG_SERIAL.println();
  }
#endif  // RUN_I2C_DIAG
}
