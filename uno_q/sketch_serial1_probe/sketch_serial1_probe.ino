/*
 * ============================================================================
 *  SUPERSEDED. DO NOT FLASH THIS FOR A MISSION.
 *
 *  The flying firmware is ../sketch_mav_shovel/. This probe answers only the
 *  question it was built for (2026-08-13: is Serial1 ours? YES) and it does
 *  NOT provide mav_read / mav_write / mav_stats, so with this sketch on the
 *  STM32 the Linux side cannot reach the Pixhawk at all: mav_shovel_pump.py
 *  and run_mission.py will both fail to find their RPC methods, and the whole
 *  autonomy chain is dead with no obvious cause.
 *
 *  Keep it: it is the minimal test to re-run after any core upgrade, since it
 *  transmits a heartbeat with no dependency on the Bridge whatsoever.
 *  Just never fly it.  (Warning added in review 2026-08-15: this folder was
 *  worked on for weeks and the shovel was written the night before the shoot,
 *  so reaching for the familiar name is the easy mistake.)
 * ============================================================================
 *
 * serial1_probe - can a sketch on the UNO Q's STM32 talk to the Pixhawk on D0/D1?
 *
 * WHY THIS EXISTS (TODO 7, the blocker for autonomous flight):
 * The UNO Q docs contradict each other. The user manual says D0/D1 are Serial1
 * on the JDIGITAL header, and ALSO says the router claims Serial1 and you must
 * not open it in your own code. The Arduino_RouterBridge README says the Bridge
 * uses a core-routed UART, "falling back to Serial1 if the core does not
 * provide it". Those cannot both be acted on and no amount of reading settles
 * it. Only the board settles it.
 *
 * REWRITTEN 2026-08-11. The previous version reported its verdict ONLY on
 * LED_BUILTIN. That was a reasonable call when the alternative was a serial
 * monitor possibly routed through the peripheral under test, but the user has
 * since confirmed THE BOARD IS SEALED INSIDE THE AIRFRAME, so nobody can see
 * the LED. A verdict you cannot read is not a verdict.
 *
 * SO IT NOW ANSWERS FROM THE OTHER END OF THE WIRE. The sketch TRANSMITS a real
 * MAVLink heartbeat on Serial1 as MAV_COMP_ID_ONBOARD_COMPUTER (191). If the
 * link works, the PIXHAWK sees a new MAVLink node appear, and that is readable
 * from the laptop over USB, with the aircraft shut:
 *
 *     ./python tools/bench.py nodes
 *
 * A row for "sys 1 comp 191" IS the pass. No LED, no screen on the board, no
 * guessing. The ESP32 already proves this works as a channel: it shows up the
 * same way at compid 195.
 *
 * TRANSMITTING IS SAFE HERE, AND THIS IS NOT THE RETRACTED LOOPBACK IDEA. D1 is
 * the UNO Q's TX and it feeds the Pixhawk's RX5. That is what the wiring is
 * FOR. The hazard recorded earlier was a JUMPER tying D1 to D0, which shorts
 * the UNO Q's TX to the Pixhawk's TX5 output: two push-pull drivers fighting.
 * Nothing here does that. Do not fit that jumper.
 *
 * THE LED IS STILL DRIVEN, as a free second channel for when the airframe is
 * open, and it now reports the RECEIVE direction specifically:
 *   SOLID ON         = MAVLink framing byte arriving from the Pixhawk. RX works.
 *   FAST BLINK (5Hz) = bytes arriving but no MAVLink framing among them. The
 *                      UART and wire work; suspect SERIAL5_BAUD (want 115200)
 *                      or SERIAL5_PROTOCOL (want 2 = MAVLink2). Wrong baud
 *                      reads as exactly this: real traffic, garbled framing.
 *   SLOW BLINK (1Hz) = zero bytes. Either the router holds Serial1, or SERIAL5
 *                      is unconfigured, or TX5 is not landing on D0.
 *
 * READING THE TWO CHANNELS TOGETHER IS THE WHOLE POINT, because they fail
 * independently and that is what localises the fault:
 *   nodes PASS + LED solid  = both directions work. TODO 7 is SOLVED.
 *   nodes PASS + LED blinks = our TX reaches the Pixhawk but its TX5 does not
 *                             reach us. One broken wire (TX5->D0), not a
 *                             software problem. Check that conductor.
 *   nodes FAIL + LED solid  = we hear the Pixhawk but it does not hear us.
 *                             D1->RX5 is the suspect conductor.
 *   nodes FAIL + LED slow   = Serial1 is not ours, or SERIAL5 is off. Check the
 *                             params first, it costs nothing.
 *
 * IF OPENING Serial1 KILLS THE LINUX BRIDGE, that is not a crash, it is THE
 * ANSWER: it means the Bridge really did fall back to Serial1, the two cannot
 * coexist, and the byte-shovel architecture (Linux -> Bridge -> STM32 ->
 * Serial1 -> Pixhawk) is impossible as drawn. SSH is unaffected either way:
 * it runs over the network on the Linux side and does not touch this UART.
 *
 * WHAT A PASS DOES NOT PROVE: that the link is good enough to FLY on. That
 * needs the byte-shovel and a soak against dropped and garbled frames. This
 * proves the pins are usable, which is the question that has blocked for weeks.
 */

static const unsigned long BAUD = 115200;    // must match SERIAL5_BAUD
static const unsigned long HEARTBEAT_MS = 1000;
static const unsigned long QUIET_MS = 2000;  // no framing for this long = stale

// MAVLink frame start bytes. v2 opens with 0xFD, v1 with 0xFE. Accept either:
// seeing v1 still proves the pins and the wire, and only means SERIAL5_PROTOCOL
// wants a look.
static const uint8_t MAVLINK_V2_MAGIC = 0xFD;
static const uint8_t MAVLINK_V1_MAGIC = 0xFE;

/*
 * A real MAVLink2 HEARTBEAT, sys 1 comp 191 (MAV_COMP_ID_ONBOARD_COMPUTER),
 * type MAV_TYPE_ONBOARD_CONTROLLER, autopilot MAV_AUTOPILOT_INVALID.
 *
 * NOT hand-written. Generated on the laptop by pymavlink and pasted here, so
 * the bytes come from a real MAVLink implementation rather than from memory of
 * the wire format. Byte 4 is the sequence number and is bumped per send, which
 * means the CRC has to be recomputed: a frame with a stale CRC is dropped
 * silently by the receiver and would look exactly like a dead wire.
 */
static const uint8_t HEARTBEAT_TEMPLATE[] = {
  0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0xBF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x12, 0x08, 0x00, 0x04, 0x03, 0xAE, 0xC6,
};
static const uint8_t HEARTBEAT_LEN = sizeof(HEARTBEAT_TEMPLATE);
static const uint8_t HEARTBEAT_CRC_EXTRA = 50;   // read from pymavlink, not guessed

static uint8_t  txSeq = 0;
static unsigned long lastBeat = 0;
static unsigned long lastMagic = 0;
static unsigned long lastByte = 0;

/*
 * CRC-16/MCRF4XX, the checksum MAVLink uses. This C was verified on the laptop
 * against pymavlink's own output before being written here: it reproduces the
 * reference frame's CRC (0xC6AE) exactly, and seq-bumped frames it produces
 * parse cleanly through a real MAVLink parser.
 */
static uint16_t mavlinkCrc(const uint8_t *data, uint16_t len, uint8_t extra) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i <= len; i++) {
    uint8_t b = (i < len) ? data[i] : extra;   // crc_extra is fed in last
    uint8_t tmp = b ^ (uint8_t)(crc & 0xFF);
    tmp ^= (uint8_t)(tmp << 4);
    crc = (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3)
          ^ ((uint16_t)tmp >> 4);
  }
  return crc;
}

static void sendHeartbeat() {
  uint8_t f[sizeof(HEARTBEAT_TEMPLATE)];
  memcpy(f, HEARTBEAT_TEMPLATE, HEARTBEAT_LEN);
  f[4] = txSeq++;                              // wraps at 255, which is correct
  // CRC covers everything after the 0xFD magic, excluding the CRC itself.
  uint16_t crc = mavlinkCrc(&f[1], HEARTBEAT_LEN - 3, HEARTBEAT_CRC_EXTRA);
  f[HEARTBEAT_LEN - 2] = (uint8_t)(crc & 0xFF);
  f[HEARTBEAT_LEN - 1] = (uint8_t)(crc >> 8);
  Serial1.write(f, HEARTBEAT_LEN);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial1.begin(BAUD);
  delay(200);                 // let the peripheral settle before trusting it
}

void loop() {
  unsigned long now = millis();

  // Drain whatever the Pixhawk is streaming and note what kind of bytes.
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    lastByte = now;
    if (b == MAVLINK_V2_MAGIC || b == MAVLINK_V1_MAGIC) lastMagic = now;
  }

  if (now - lastBeat >= HEARTBEAT_MS) {
    lastBeat = now;
    sendHeartbeat();
  }

  // LED reports the RECEIVE side only. The transmit side is answered by
  // tools/bench.py nodes on the laptop, which works with the airframe shut.
  bool framing = lastMagic && (now - lastMagic < QUIET_MS);
  bool anyBytes = lastByte && (now - lastByte < QUIET_MS);
  if (framing) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    unsigned long period = anyBytes ? 100 : 500;   // 5Hz vs 1Hz
    digitalWrite(LED_BUILTIN, (now / period) % 2 ? HIGH : LOW);
  }
}
