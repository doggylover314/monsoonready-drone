// =============================================================================
//  i2c_diag.h  --  Bench diagnostic for the TCA9548A + VL53L0X ring.
//
//  Built only when RUN_I2C_DIAG is 1 in config.h. It replaces the normal
//  mission behaviour with a one-shot report, so nothing is transmitted to the
//  Pixhawk while it runs.
//
//  It exists because "ch1: VL53L0X init FAILED" says a channel did not answer
//  but not WHY, and the candidate causes need different fixes:
//
//    nothing answers at 0x29 ............ sensor unpowered / wire off / dead
//    SDA or SCL stuck low .............. short to GND, or a sensor holding
//                                        the bus (often a half-connected one)
//    answers at 0x29 but init fails .... sensor present, comms marginal
//    works at 100 kHz but not 400 ...... signal integrity: wire length,
//                                        capacitance, weak pull-ups
//    works with a settle delay .......... mux switching too fast for the wiring
//
//  The last two are why the diagnostic sweeps clock speed and settle time
//  instead of testing one configuration: config.h already carries the knobs
//  (I2C_CLOCK_HZ, TCA_SETTLE_US) and their comments say to change them "if the
//  bus is flaky", which is exactly the hypothesis under test.
// =============================================================================

#pragma once

// Runs the full report over the serial monitor. Never returns anything; read
// the output. Safe to call repeatedly.
void runI2cDiag();
