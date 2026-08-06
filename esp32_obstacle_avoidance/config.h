// =============================================================================
//  config.h  --  Central configuration for the ESP32 obstacle avoidance
//                module (7x VL53L0X -> TCA9548A -> ESP32 -> Pixhawk):
//                6 sensors in a horizontal ring (between the arms) + 1 UP.
//
//  Everything you are likely to change lives here: pins, baud rates, the
//  fake-sensor test knobs, the reported range, and the MAVLink identity.
//
//  --------------------------------------------------------------------------
//  WIRING (ESP32-WROOM-32 dev board)
//  --------------------------------------------------------------------------
//
//   ESP32            TCA9548A mux            7x VL53L0X (one per mux channel)
//   -----            ------------            --------------------------------
//   GPIO21 (SDA) --- SDA (mux upstream)
//   GPIO22 (SCL) --- SCL (mux upstream)
//   3V3          --- VIN                     VIN of every sensor
//   GND          --- GND                     GND of every sensor
//                    SD0/SC0 -------------->  sensor 0  (bearing OFFSET +   0 deg)
//                    SD1/SC1 -------------->  sensor 1  (bearing OFFSET +  60 deg)
//                    SD2/SC2 -------------->  sensor 2  (bearing OFFSET + 120 deg)
//                    SD3/SC3 -------------->  sensor 3  (bearing OFFSET + 180 deg)
//                    SD4/SC4 -------------->  sensor 4  (bearing OFFSET + 240 deg)
//                    SD5/SC5 -------------->  sensor 5  (bearing OFFSET + 300 deg)
//                    SD6/SC6 -------------->  sensor 6  (points straight UP)
//
//   OFFSET = SENSOR_ANGLE_OFFSET_DEG below. THIS BUILD USES 0: the sensors are
//   mounted BETWEEN the arms with sensor 0 facing straight out the nose
//   (measured on the airframe 2026-08-06). Use 30 only if the ring is instead
//   mounted ON the arms of a hexa X. Angles clockwise from above.
//
//   All seven VL53L0X keep their default I2C address (0x29); the mux gates the
//   bus so only one is visible at a time -- no XSHUT address juggling needed.
//   Angles are measured CLOCKWISE from the vehicle's forward direction, viewed
//   from above -- this matches ArduPilot's OBSTACLE_DISTANCE convention exactly.
//
//   ESP32                         Pixhawk TELEM2 (= SERIAL2)
//   -----                         -------------------------
//   GPIO17 (TX2) --------------->  RX   (Pixhawk receives our data)
//   GPIO16 (RX2) <---------------  TX   (optional; lets us hear the Pixhawk)
//   GND          ---------------  GND  (common ground - REQUIRED)
//   (do NOT power the ESP32 from TELEM2 5V unless you know the board's current
//    limit; power the ESP32 from USB or its own 5V and just share GND.)
//
//   USB serial (GPIO1/GPIO3, UART0) is used for the human-readable debug log.
// =============================================================================

#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
// COMPILE-TIME MODE
// -----------------------------------------------------------------------------
// Set to 1 to run with NO hardware attached: the sensor layer returns simulated
// distances so you can verify the MAVLink output on the wire (or in Mission
// Planner) before you have any sensors wired. Set to 0 for real VL53L0X reads.
// 2026-08-03: set to 0 for the real-mode-first flash (ring is fully wired;
// see PROJECT_STATE TODO 4). Flip back to 1 only to bisect link-vs-ring, and
// remember fake mode then needs the GPIO4 jumper to transmit at all.
#define USE_FAKE_SENSORS 0

// Fake-mode flight guard: while USE_FAKE_SENSORS is 1, MAVLink is only
// transmitted if this pin is jumpered to GND (bench-only jumper). If a
// fake-mode build is ever flashed on the drone by mistake, no phantom
// obstacles reach the Pixhawk. Has no effect when USE_FAKE_SENSORS is 0.
#define FAKE_TX_ENABLE_PIN 4

// -----------------------------------------------------------------------------
// GEOMETRY  (6 ring sensors, between the arms -> 60 deg/sector, + 1 UP sensor)
// -----------------------------------------------------------------------------
#define NUM_SENSORS 7        // total mux channels in use (0..5 = ring, 6 = up)
#define NUM_RING_SENSORS 6   // horizontal ring only (goes into OBSTACLE_DISTANCE)
#define UP_SENSOR_CHANNEL 6  // mux channel of the upward sensor (DISTANCE_SENSOR)
#define SECTOR_INCREMENT_DEG (360.0f / NUM_RING_SENSORS) // 60.0 deg

// Bearing of ring sensor 0, in degrees CLOCKWISE from vehicle forward. Sent as
// the OBSTACLE_DISTANCE angle_offset, so ArduPilot rotates the ring for us and
// no remapping is needed here.
//
// CORRECTED 2026-08-06 (30.0 -> 0.0) once the ring was physically built: the
// sensors are NOT on the arms, they sit BETWEEN them, with sensor 0 pointing
// straight out the nose. So the bearings are 0/60/120/180/240/300, not the
// 30/90/... that hexa-X arm mounting would give. The old value was written
// when the mounting was still assumed to be arm-mounted.
//
// This is not cosmetic: angle_offset is what tells ArduPilot which way an
// obstacle lies. Leaving it at 30 with this build reports every obstacle 30
// degrees clockwise of where it really is, so avoidance would slide the
// aircraft along the wrong vector.
#define SENSOR_ANGLE_OFFSET_DEG 0.0f

// Map each physical ring mux channel to an OBSTACLE_DISTANCE sector index.
// Sector s covers a bearing of (SENSOR_ANGLE_OFFSET_DEG + s * 60) degrees,
// measured CLOCKWISE from vehicle forward. Default is the identity map
// (channel N -> sector N). If your physical build wires the channels in a
// different order, remap here instead of rewiring.
static const uint8_t SECTOR_FOR_CHANNEL[NUM_RING_SENSORS] __attribute__((unused)) =
    {0, 1, 2, 3, 4, 5};

// -----------------------------------------------------------------------------
// I2C  (ESP32 <-> TCA9548A <-> VL53L0X)
// -----------------------------------------------------------------------------
#define I2C_SDA_PIN 21       // ESP32-WROOM default SDA
#define I2C_SCL_PIN 22       // ESP32-WROOM default SCL
#define I2C_CLOCK_HZ 400000  // 400 kHz Fast-mode; drop to 100000 if bus is flaky
#define TCA9548A_ADDR 0x70   // default mux address (A0/A1/A2 all low)
#define TCA_SETTLE_US 0      // microseconds to wait after switching channel
                             // (raise to ~50 only if you see bus errors)

// -----------------------------------------------------------------------------
// VL53L0X behaviour
// -----------------------------------------------------------------------------
#define SENSOR_TIMING_BUDGET_US 20000 // 20 ms = high-speed profile (fastest usable)
#define SENSOR_TIMEOUT_MS 50          // per-read abort so a dead sensor cannot hang us
#define VL53L0X_LONG_RANGE 0          // 1 = trade speed for ~2 m reach (needs low IR)
// A reading at or above this many mm is treated as "no target in range" (clear).
// The VL53L0X reports ~8190 mm when it sees nothing; keep this comfortably above
// RANGE_MAX_CM*10 so real readings are never mistaken for "clear". Readings that
// are real but beyond RANGE_MAX_CM are handled by the max clamp in classifyCm().
#define VL53L0X_MAX_VALID_MM 8000
// Sentinel returned by the sensor layer for an I2C timeout / absent sensor.
#define SENSOR_MM_ERROR 0xFFFF

// -----------------------------------------------------------------------------
// REPORTED RANGE (centimetres) -- goes into OBSTACLE_DISTANCE min/max_distance
// -----------------------------------------------------------------------------
// VL53L0X practical range is ~4 cm (min) to ~120 cm reliably (up to ~200 cm in
// long-range mode / low light). ArduPilot ignores any sector reading that falls
// outside [RANGE_MIN_CM, RANGE_MAX_CM], so these bound what it will act on.
#define RANGE_MIN_CM 5
#define RANGE_MAX_CM 200

// -----------------------------------------------------------------------------
// SERIAL PORTS
// -----------------------------------------------------------------------------
#define PIXHAWK_SERIAL Serial2 // hardware UART2 -> Pixhawk TELEM1 (user wired
                               // TELEM1<->TELEM2 opposite to plan, 2026-08-02)
#define PIXHAWK_TX_PIN 17      // ESP32 TX2 -> Pixhawk TELEM1 RX
#define PIXHAWK_RX_PIN 16      // ESP32 RX2 <- Pixhawk TELEM1 TX
#define PIXHAWK_BAUD 115200    // MUST equal ArduPilot SERIAL1_BAUD (115 = 115200)

#define DEBUG_SERIAL Serial    // USB serial for the human-readable log
#define DEBUG_BAUD 115200

// -----------------------------------------------------------------------------
// TX SCHEDULING
// -----------------------------------------------------------------------------
#define TX_RATE_HZ 10                       // OBSTACLE_DISTANCE + up DISTANCE_SENSOR rate
#define TX_PERIOD_MS (1000 / TX_RATE_HZ)    // 100 ms
#define SEND_HEARTBEAT 1                    // also emit a 1 Hz MAVLink HEARTBEAT
#define HEARTBEAT_PERIOD_MS 1000            // (helps GCS/ArduPilot recognise the link)

// -----------------------------------------------------------------------------
// MAVLINK IDENTITY
// -----------------------------------------------------------------------------
// System ID should match the Pixhawk's SYSID_THISMAV (default 1) so ArduPilot
// associates our proximity data with that vehicle. Component ID 195 is
// MAV_COMP_ID_PATHPLANNER, the value ArduPilot's own avoidance docs recommend.
#define MAV_SYSID 1
#define MAV_COMPID 195

// -----------------------------------------------------------------------------
// FAKE-SENSOR TEST KNOBS (only used when USE_FAKE_SENSORS == 1)
// -----------------------------------------------------------------------------
// All sensors report FAKE_CLEAR_MM ("all 8 m" = 8000 mm, i.e. beyond range =>
// reported as CLEAR), except one channel you can vary to inject an obstacle.
#define FAKE_CLEAR_MM 8000    // 8 m -> beyond RANGE_MAX_CM -> shows up as CLEAR
#define FAKE_VARY_CHANNEL 2   // which mux channel (0..6) to override; 6 = UP sensor
#define FAKE_VARY_MM 500      // that channel's simulated distance (500 mm = 50 cm)
