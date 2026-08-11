/*
 * serial1_probe - does the UNO Q's STM32 actually give a sketch Serial1 (D0/D1)?
 *
 * WHY THIS EXISTS (TODO 7, the blocker for autonomous flight):
 * The UNO Q docs contradict each other. The user manual says D0/D1 are Serial1
 * on the JDIGITAL header, and ALSO says the router claims Serial1 and you must
 * not open it in your own code. The Arduino_RouterBridge README says the Bridge
 * uses a core-routed UART, "falling back to Serial1 if the core does not
 * provide it". Those cannot both be acted on, and no amount of reading settles
 * it. Only the board settles it.
 *
 * THE TEST: physical loopback. Jumper D1 (TX) to D0 (RX), write a known byte
 * on Serial1, see whether it comes back. This needs NO extra parts, which is
 * the whole point: the board has one USB port, the splitter has one USB-A
 * socket, and the camera owns it.
 *
 * THE VERDICT IS ON THE LED, deliberately, so this test does not depend on a
 * serial monitor that may itself be routed through the thing under test:
 *   SOLID ON          = loopback works. Serial1 is ours. D0/D1 wiring plan LIVES.
 *   FAST BLINK (5Hz)  = Serial1 opened but nothing came back. Either the router
 *                       is holding the pins, or the jumper is not making contact.
 *                       RE-SEAT THE JUMPER BEFORE BELIEVING THIS.
 *   SLOW BLINK (1Hz)  = sketch is running but never reached the test. Should not
 *                       happen; means setup() did not complete.
 *
 * WHAT A PASS DOES AND DOES NOT PROVE. It proves the UART peripheral is
 * reachable from sketch code and is not being held by the router. It does NOT
 * prove the Pixhawk link works: that needs the byte-shovel plus the real
 * SERIAL5 wiring, and is the next step, not this one.
 *
 * IF IT FAILS: the D0/D1 plan is dead and the fallback is a USB-serial adapter,
 * which on this build ALSO needs a hub downstream of the splitter because the
 * camera holds the only A socket. Two unverified parts. Know this before Saturday.
 */

static const unsigned long BAUD = 115200;
static const uint8_t PROBE_BYTE = 0x5A;   // 0b01011010, alternating bits: a
                                          // shorted or stuck-low line cannot
                                          // fake this the way 0x00 or 0xFF can.
static const unsigned long REPLY_TIMEOUT_MS = 200;

enum Verdict { RUNNING, LOOPED_BACK, NO_REPLY };
static Verdict verdict = RUNNING;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial1.begin(BAUD);
  delay(200);                 // let the peripheral settle before trusting it

  // Drain anything already sitting in the buffer, so a stale byte from boot
  // noise cannot be mistaken for our own echo. This matters: SERIAL5 boot noise
  // is a known thing on this project's other UART.
  while (Serial1.available()) {
    Serial1.read();
  }

  Serial1.write(PROBE_BYTE);
  Serial1.flush();            // block until the byte is actually clocked out

  unsigned long deadline = millis() + REPLY_TIMEOUT_MS;
  verdict = NO_REPLY;
  while (millis() < deadline) {
    if (Serial1.available()) {
      if (Serial1.read() == PROBE_BYTE) {
        verdict = LOOPED_BACK;
      }
      break;                  // one byte in, one byte out; anything else is a fail
    }
  }
}

void loop() {
  switch (verdict) {
    case LOOPED_BACK:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case NO_REPLY:
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW);  delay(100);
      break;
    default:
      digitalWrite(LED_BUILTIN, HIGH); delay(500);
      digitalWrite(LED_BUILTIN, LOW);  delay(500);
      break;
  }
}
