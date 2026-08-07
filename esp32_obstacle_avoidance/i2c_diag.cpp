// =============================================================================
//  i2c_diag.cpp  --  see i2c_diag.h for what this is and why it exists.
// =============================================================================

#include "config.h"

#if RUN_I2C_DIAG

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "i2c_diag.h"

// DEBUG_SERIAL / DEBUG_BAUD come from config.h; do not redefine them here.

static const uint8_t VL53L0X_ADDR = 0x29;

// Clock/settle combinations to sweep, slowest last so the report reads as
// "does relaxing the bus help?".
struct BusCfg { uint32_t hz; uint16_t settle_us; const char *label; };
static const BusCfg CONFIGS[] = {
    {400000, 0,   "400 kHz, no settle delay  (current config.h settings)"},
    {400000, 50,  "400 kHz, 50 us settle"},
    {100000, 50,  "100 kHz, 50 us settle"},
    {50000,  200, "50 kHz, 200 us settle    (slowest, most forgiving)"},
};
static const uint8_t NUM_CONFIGS = sizeof(CONFIGS) / sizeof(CONFIGS[0]);

// Per-channel result across the sweep, so the summary can say whether a
// channel is dead everywhere or only at speed. Sized from CONFIGS itself so
// adding a row to the sweep cannot silently overflow it.
static bool ok_at[NUM_SENSORS][sizeof(CONFIGS) / sizeof(CONFIGS[0])];

static bool muxSelect(uint8_t ch, uint16_t settle_us) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << ch);
  bool acked = (Wire.endTransmission() == 0);
  if (settle_us) delayMicroseconds(settle_us);
  return acked;
}

static void muxCloseAll() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// Reads the mux's control register back. If the value does not match what we
// just wrote, the mux is not actually switching and every "sensor" fault is
// really one mux fault.
static bool muxReadback(uint8_t expect) {
  if (Wire.requestFrom((uint8_t)TCA9548A_ADDR, (uint8_t)1) != 1) return false;
  return Wire.read() == expect;
}

// Idle line levels with a channel open. Both should read HIGH (pull-ups).
// A LOW line is a short or a device clamping the bus, which no amount of
// retrying will fix and which also explains why its NEIGHBOURS may be fine:
// the mux isolates the fault to the selected channel.
static void reportLines(uint8_t ch) {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(200);
  int sda = digitalRead(I2C_SDA_PIN);
  int scl = digitalRead(I2C_SCL_PIN);
  DEBUG_SERIAL.printf("      idle lines: SDA=%s SCL=%s%s\n",
                      sda ? "HIGH" : "LOW ", scl ? "HIGH" : "LOW ",
                      (sda && scl) ? "" : "   <-- STUCK LOW, suspect a short "
                                          "or a half-connected sensor");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
}

// Every address that ACKs on the currently selected channel.
static uint8_t scanChannel() {
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    if (addr == TCA9548A_ADDR) continue;   // upstream, not downstream
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      DEBUG_SERIAL.printf("      device at 0x%02X%s\n", addr,
                          addr == VL53L0X_ADDR ? "  (VL53L0X)" : "");
      found++;
    }
  }
  return found;
}

static void runOneConfig(uint8_t cfg_i) {
  const BusCfg &cfg = CONFIGS[cfg_i];
  DEBUG_SERIAL.printf("\n--- %s ---\n", cfg.label);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(cfg.hz);

  // The mux itself first: if this fails nothing downstream can be judged.
  Wire.beginTransmission(TCA9548A_ADDR);
  if (Wire.endTransmission() != 0) {
    DEBUG_SERIAL.printf("  MUX at 0x%02X DID NOT ACK. Everything below is "
                        "meaningless until the mux answers: check its VIN, "
                        "GND, SDA, SCL and address straps.\n", TCA9548A_ADDR);
    for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) ok_at[ch][cfg_i] = false;
    return;
  }
  DEBUG_SERIAL.printf("  mux 0x%02X ACK\n", TCA9548A_ADDR);

  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    DEBUG_SERIAL.printf("  ch%u%s:\n", ch,
                        ch == UP_SENSOR_CHANNEL ? " (up)" : "");
    bool sel = muxSelect(ch, cfg.settle_us);
    bool rb = muxReadback(1 << ch);
    DEBUG_SERIAL.printf("      select %s, readback %s\n",
                        sel ? "ACK" : "NACK",
                        rb ? "matches" : "MISMATCH (mux not switching!)");

    uint8_t found = scanChannel();
    if (found == 0) {
      DEBUG_SERIAL.println("      nothing answers on this channel");
      reportLines(ch);
      Wire.setClock(cfg.hz);
      muxSelect(ch, cfg.settle_us);
    }

    // Only worth attempting a driver init if something is actually there.
    bool ok = false;
    if (found) {
      VL53L0X s;
      s.setTimeout(SENSOR_TIMEOUT_MS);
      ok = s.init();
      DEBUG_SERIAL.printf("      VL53L0X init %s\n", ok ? "OK" : "FAILED");
      if (ok) {
        s.startContinuous();
        delay(60);
        uint16_t mm = s.readRangeContinuousMillimeters();
        DEBUG_SERIAL.printf("      one reading: %u mm%s\n", mm,
                            s.timeoutOccurred() ? "  (TIMEOUT)" : "");
        s.stopContinuous();
      }
    }
    ok_at[ch][cfg_i] = ok;
  }
  muxCloseAll();
}

void runI2cDiag() {
  DEBUG_SERIAL.println(F("\n\n================ I2C DIAGNOSTIC ================"));
  DEBUG_SERIAL.printf("SDA=GPIO%u SCL=GPIO%u, mux 0x%02X, %u channels "
                      "(ch%u = up)\n",
                      I2C_SDA_PIN, I2C_SCL_PIN, TCA9548A_ADDR, NUM_SENSORS,
                      UP_SENSOR_CHANNEL);
  DEBUG_SERIAL.println(F("Nothing is transmitted to the Pixhawk in this mode."));

  for (uint8_t i = 0; i < NUM_CONFIGS; i++) runOneConfig(i);

  DEBUG_SERIAL.println(F("\n================ SUMMARY ================"));
  DEBUG_SERIAL.println(F("channel   400k/0   400k/50  100k/50  50k/200"));
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    DEBUG_SERIAL.printf("  ch%u%s     ", ch, ch == UP_SENSOR_CHANNEL ? "(up)" : "    ");
    for (uint8_t i = 0; i < NUM_CONFIGS; i++)
      DEBUG_SERIAL.printf("%-9s", ok_at[ch][i] ? "OK" : "fail");
    DEBUG_SERIAL.println();
  }
  DEBUG_SERIAL.println(F(
      "\nHOW TO READ THIS:\n"
      "  fail everywhere, nothing at 0x29   -> that sensor has no power, a\n"
      "     broken wire, or is dead. Meter ITS OWN VIN and GND at the sensor\n"
      "     end, not at the hub.\n"
      "  fail everywhere, SDA or SCL LOW    -> short or a sensor clamping the\n"
      "     bus on that channel. Unplug it: the line should go HIGH again.\n"
      "  OK only at the slower speeds       -> signal integrity, not wiring.\n"
      "     Set I2C_CLOCK_HZ / TCA_SETTLE_US in config.h to the slowest\n"
      "     column that passes, and prefer shorter leads.\n"
      "  0x29 present but init FAILED       -> sensor answers but comms are\n"
      "     marginal; try the slower configs and check for XSHUT held low.\n"
      "  every channel fails                -> the mux or its own supply,\n"
      "     not the sensors.\n"));
  DEBUG_SERIAL.println(F("Set RUN_I2C_DIAG back to 0 in config.h to fly."));
}

#endif  // RUN_I2C_DIAG
