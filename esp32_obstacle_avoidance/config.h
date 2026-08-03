// =============================================================================
//  config.h  --  Central configuration for the ESP32 obstacle avoidance
//                module (7x VL53L0X -> TCA9548A -> ESP32 -> Pixhawk):
//                6 sensors in a horizontal ring (one per arm) + 1 pointing UP.
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
//                    SD0/SC0 -------------->  sensor 0  (arm at OFFSET +   0 deg)
//                    SD1/SC1 -------------->  sensor 1  (arm at OFFSET +  60 deg)
//                    SD2/SC2 -------------->  sensor 2  (arm at OFFSET + 120 deg)
//                    SD3/SC3 -------------->  sensor 3  (arm at OFFSET + 180 deg)
//                    SD4/SC4 -------------->  sensor 4  (arm at OFFSET + 240 deg)
//                    SD5/SC5 -------------->  sensor 5  (arm at OFFSET + 300 deg)
//                    SD6/SC6 -------------->  sensor 6  (points straight UP)
//
//   OFFSET = SENSOR_ANGLE_OFFSET_DEG below: 30 for a hexa X layout (two arms
//   at the front, ring sensor 0 on the front-right arm), 0 for hexa Plus
//   (one arm pointing straight forward). Angles clockwise from above.
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
#define USE_FAKE_SENSORS 1

// Fake-mode flight guard: while USE_FAKE_SENSORS is 1, MAVLink is only
// transmitted if this pin is jumpered to GND (bench-only jumper). If a
// fake-mode build is ever flashed on the drone by mistake, no phantom
// obstacles reach the Pixhawk. Has no effect when USE_FAKE_SENSORS is 0.
#define FAKE_TX_ENABLE_PIN 4

// -----------------------------------------------------------------------------
// GEOMETRY  (6 ring sensors, one per arm -> 60 deg per sector, + 1 UP sensor)
// -----------------------------------------------------------------------------
#define NUM_SENSORS 7        // total mux channels in use (0..5 = ring, 6 = up)
#define NUM_RING_SENSORS 6   // horizontal ring only (goes into OBSTACLE_DISTANCE)
#define UP_SENSOR_CHANNEL 6  // mux channel of the upward sensor (DISTANCE_SENSOR)
#define SECTOR_INCREMENT_DEG (360.0f / NUM_RING_SENSORS) // 60.0 deg

// Bearing of ring sensor 0, in degrees CLOCKWISE from vehicle forward. The ring
// sensors sit on the hexa arms: a hexa X frame (two arms at the front) has arms
// at 30/90/150/210/270/330 deg -> use 30. A hexa Plus frame (one arm straight
// forward) has arms at 0/60/... -> use 0. This is sent as the OBSTACLE_DISTANCE
// angle_offset, so ArduPilot rotates the ring for us; no remapping needed here.
#define SENSOR_ANGLE_OFFSET_DEG 30.0f

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
