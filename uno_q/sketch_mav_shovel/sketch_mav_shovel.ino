/* MAVLink byte-shovel: Bridge RPC <-> Serial1 (D0/D1 -> Pixhawk SERIAL5).

   REPLACES sketch_serial1_probe as the flying firmware. The probe proved the
   STM32 -> Pixhawk TX direction with a hardcoded comp-191 heartbeat; this
   sketch carries REAL MAVLink both ways for the Linux side, which is where
   the detector and mission logic live. Once uno_q/mav_shovel_pump.py runs,
   the comp-191 heartbeats on the Pixhawk bus come from pymavlink on Linux
   through this shovel, not from firmware.

   ARCHITECTURE (each piece verified, nothing guessed):
     Linux (pymavlink) -> mav_shovel_pump.py -> unix socket
       /var/run/arduino-router.sock -> arduino-router daemon -> /dev/ttyHS1
       -> Bridge (its own devicetree UART: the core defines
       ARDUINO_ROUTER_SERIAL via ARDUINO_ROUTER_PHANDLE, zephyrSerial.h:152,
       so the Bridge does NOT touch Serial1) -> these RPC methods -> Serial1
       -> D0/D1 -> Pixhawk SERIAL5 at 115200.

   BINARY SAFETY: payloads cross the RPC as BASE64 IN A STRING. MAVLink is
   raw bytes; msgpack-python emits `bin` for bytes while the MCU-side MsgPack
   library binds callbacks on `str`, and that mismatch is exactly the kind of
   cross-stack subtlety that eats a night. Base64 costs 4/3 inflation on a
   link whose serial leg moves at most ~11.5 KB/s; correctness wins.

   RX PATH: loop() continuously drains Serial1 into a 4 KiB ring so nothing
   is lost between Linux polls (the Zephyr core's own Serial1 RX buffer is
   not sized by us and must not be trusted to hold a poll interval's worth).
   The ring is shared with the Bridge's RPC thread (begin() spawns it), so
   access is guarded by a Zephyr k_mutex, same primitive bridge.h itself
   uses. On overflow the OLDEST bytes drop and a counter records it: MAVLink
   resynchronises on the next magic byte, and mav_stats makes the loss
   visible instead of silent.
*/

#include <Arduino_RouterBridge.h>

static const uint32_t PIXHAWK_BAUD = 115200;

static uint8_t ring[4096];
static size_t ring_head = 0;   // written by loop()
static size_t ring_tail = 0;   // read by mav_read (Bridge RPC thread)
static struct k_mutex ring_mtx;

static uint32_t rx_total = 0;   // bytes taken off Serial1
static uint32_t rx_dropped = 0; // bytes lost to ring overflow
static uint32_t tx_total = 0;   // bytes written to Serial1

/* ---- base64 ---- */

static const char B64A[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static String b64encode(const uint8_t *d, size_t n) {
    String out;
    out.reserve(((n + 2) / 3) * 4);
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)d[i] << 16;
        if (i + 1 < n) v |= (uint32_t)d[i + 1] << 8;
        if (i + 2 < n) v |= d[i + 2];
        out += B64A[(v >> 18) & 63];
        out += B64A[(v >> 12) & 63];
        out += (i + 1 < n) ? B64A[(v >> 6) & 63] : '=';
        out += (i + 2 < n) ? B64A[v & 63] : '=';
    }
    return out;
}

static int b64val(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;  // '=' and anything else end the decode
}

static size_t b64decode(const String &s, uint8_t *out, size_t maxlen) {
    size_t n = 0;
    uint32_t acc = 0;
    int bits = 0;
    for (unsigned int i = 0; i < s.length() && n < maxlen; i++) {
        int v = b64val(s[i]);
        if (v < 0) break;
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out[n++] = (uint8_t)((acc >> bits) & 0xFF);
        }
    }
    return n;
}

/* ---- RPC methods (run on the Bridge's update thread) ---- */

// Return up to 768 raw bytes (1024 b64 chars) of Pixhawk output per call.
// Sized well under any router message limit; the pump polls fast enough
// that the ring never holds more than a poll interval of traffic anyway.
String mav_read() {
    uint8_t tmp[768];
    size_t n = 0;
    k_mutex_lock(&ring_mtx, K_FOREVER);
    while (n < sizeof(tmp) && ring_tail != ring_head) {
        tmp[n++] = ring[ring_tail];
        ring_tail = (ring_tail + 1) % sizeof(ring);
    }
    k_mutex_unlock(&ring_mtx);
    return b64encode(tmp, n);
}

// Write base64-decoded bytes to the Pixhawk. Returns raw bytes written.
int mav_write(String b64) {
    uint8_t buf[768];
    size_t n = b64decode(b64, buf, sizeof(buf));
    size_t w = Serial1.write(buf, n);
    tx_total += w;
    return (int)w;
}

// "rx_total,rx_dropped,tx_total" - dropped > 0 means the pump polls too
// slowly or stalled; MAVLink survives it but telemetry gets lossy.
String mav_stats() {
    return String(rx_total) + "," + String(rx_dropped) + "," + String(tx_total);
}

void setup() {
    k_mutex_init(&ring_mtx);
    Serial1.begin(PIXHAWK_BAUD);
    Bridge.begin();
    Bridge.provide("mav_read", mav_read);
    Bridge.provide("mav_write", mav_write);
    Bridge.provide("mav_stats", mav_stats);
}

void loop() {
    // Drain Serial1 in small batches so the mutex hold stays microseconds.
    uint8_t tmp[64];
    size_t n = 0;
    while (n < sizeof(tmp) && Serial1.available() > 0) {
        int c = Serial1.read();
        if (c < 0) break;
        tmp[n++] = (uint8_t)c;
    }
    if (n > 0) {
        k_mutex_lock(&ring_mtx, K_FOREVER);
        for (size_t i = 0; i < n; i++) {
            size_t nh = (ring_head + 1) % sizeof(ring);
            if (nh == ring_tail) {           // full: drop the OLDEST byte
                ring_tail = (ring_tail + 1) % sizeof(ring);
                rx_dropped++;
            }
            ring[ring_head] = tmp[i];
            ring_head = nh;
            rx_total++;
        }
        k_mutex_unlock(&ring_mtx);
    } else {
        delay(1);  // idle: yield to the Bridge thread
    }
}
