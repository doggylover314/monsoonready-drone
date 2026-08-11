/*
 * serial1_probe - can a sketch on the UNO Q's STM32 hear the Pixhawk on D0/D1?
 *
 * WHY THIS EXISTS (TODO 7, the blocker for autonomous flight):
 * The UNO Q docs contradict each other. The user manual says D0/D1 are Serial1
 * on the JDIGITAL header, and ALSO says the router claims Serial1 and you must
 * not open it in your own code. The Arduino_RouterBridge README says the Bridge
 * uses a core-routed UART, "falling back to Serial1 if the core does not
 * provide it". Those cannot both be acted on and no amount of reading settles
 * it. Only the board settles it.
 *
 * THIS IS A LISTEN-ONLY TEST AND THAT IS DELIBERATE. D0/D1 are already wired to
 * the Pixhawk's SERIAL5 (TX5 -> D0, D1 -> RX5, per the SERIAL MAP). ArduPilot
 * streams heartbeats on any MAVLink-configured serial port without being asked,
 * so the data is already arriving and the sketch has to do nothing but read it.
 *
 * DO NOT LOOPBACK-JUMPER D1 TO D0 WHILE THE PIXHAWK IS CONNECTED. That ties the
 * Pixhawk's TX5 output and the UNO Q's TX output onto one net: two push-pull
 * drivers fighting whenever they disagree. An earlier version of this file told
 * you to do exactly that, before it was known the pins were already wired.
 * This version never transmits at all, so it cannot contend with TX5.
 *
 * VERDICT IS ON THE LED, not a serial monitor, because the monitor may itself be
 * routed through the peripheral under test:
 *   SOLID ON         = MAVLink framing byte seen. Serial1 is ours, the wiring is
 *                      good, the Pixhawk is talking, and the baud is right.
 *                      The D0/D1 plan LIVES and the byte-shovel is next.
 *   FAST BLINK (5Hz) = bytes ARE arriving but no MAVLink framing byte among them.
 *                      The UART and the wire work; something else is off. Check
 *                      SERIAL5_PROTOCOL (want 2 = MAVLink2) and SERIAL5_BAUD
 *                      (want 115200) with tools/parameters.py get. Wrong baud
 *                      reads as exactly this: real traffic, garbled framing.
 *   SLOW BLINK (1Hz) = ZERO bytes in the whole window. Either the router is
 *                      holding Serial1, or SERIAL5 is not configured, or TX5 is
 *                      not landing on D0. Those are three different problems and
 *                      this test cannot tell them apart; check the params first
 *                      because that costs nothing.
 *
 * WHAT A PASS DOES NOT PROVE: that the link is good enough to FLY on. That needs
 * the byte-shovel and a soak against dropped/garbled frames. This proves the
 * pins are usable, which is the question that has been blocking for two weeks.
 */

static const unsigned long BAUD = 115200;   // must match SERIAL5_BAUD
static const unsigned long LISTEN_MS = 3000;

// MAVLink frame start bytes. v2 frames open with 0xFD, v1 with 0xFE. Accept
// either: seeing v1 still proves the pins and the wire, and only means the
// protocol setting wants a look.
static const uint8_t MAVLINK_V2_MAGIC = 0xFD;
static const uint8_t MAVLINK_V1_MAGIC = 0xFE;

enum Verdict { NO_BYTES, BYTES_NO_FRAME, FRAMED };
static Verdict verdict = NO_BYTES;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial1.begin(BAUD);
  delay(200);                 // let the peripheral settle before trusting it

  unsigned long deadline = millis() + LISTEN_MS;
  bool sawAnyByte = false;
  bool sawMagic = false;

  // Listen only. Never write: TX5 is driving the other end of this pair.
  while (millis() < deadline) {
    while (Serial1.available()) {
      uint8_t b = (uint8_t)Serial1.read();
      sawAnyByte = true;
      if (b == MAVLINK_V2_MAGIC || b == MAVLINK_V1_MAGIC) {
        sawMagic = true;
      }
    }
  }

  if (sawMagic)        verdict = FRAMED;
  else if (sawAnyByte) verdict = BYTES_NO_FRAME;
  else                 verdict = NO_BYTES;
}

void loop() {
  switch (verdict) {
    case FRAMED:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case BYTES_NO_FRAME:
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW);  delay(100);
      break;
    default:
      digitalWrite(LED_BUILTIN, HIGH); delay(500);
      digitalWrite(LED_BUILTIN, LOW);  delay(500);
      break;
  }
}
