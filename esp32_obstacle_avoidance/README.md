# ESP32 Obstacle-Avoidance Module → ArduPilot (MAVLink)

Firmware for an **ESP32-WROOM-32** that reads **7× VL53L0X** time-of-flight
sensors through a **TCA9548A** I²C multiplexer and streams them to an
**ArduPilot Pixhawk** over `Serial2` at **10 Hz**:

- **6 ring sensors** (one per hexa arm, 60° apart) → **`OBSTACLE_DISTANCE`**
  for horizontal obstacle avoidance;
- **1 upward sensor** (mux channel 6) → **`DISTANCE_SENSOR`** with
  orientation "up", for overhead obstacles.

The reading layer (mux + VL53L0X) and the MAVLink layer are cleanly separated,
and a `USE_FAKE_SENSORS` flag lets you verify the MAVLink output with **no
hardware attached**.

---

## 1. Files

| File | Purpose |
|------|---------|
| `esp32_obstacle_avoidance.ino` | Main sketch: setup, 10 Hz scheduling, debug log |
| `config.h` | **All** pins, baud rates, ranges, IDs, and the fake-sensor knobs |
| `proximity_sensors.h/.cpp` | Sensor layer — TCA9548A + VL53L0X. No MAVLink. |
| `mavlink_proximity.h/.cpp` | MAVLink layer — builds/sends `OBSTACLE_DISTANCE`. No hardware. |
| `platformio.ini` | PlatformIO build (also opens as an Arduino sketch) |
| `ardupilot_proximity.param` | Loadable Pixhawk parameter file (Mission Planner) |

---

## 2. Wiring

### I²C: ESP32 → TCA9548A → 7× VL53L0X

```
ESP32                TCA9548A               VL53L0X ×7 (one per mux channel)
-----                --------               --------------------------------
GPIO21 (SDA) ─────── SDA (upstream)
GPIO22 (SCL) ─────── SCL (upstream)
3V3          ─────── VIN                    VIN of every sensor
GND          ─────── GND                    GND of every sensor
                     SD0/SC0 ────────────►  sensor 0   arm at OFFSET + 0°
                     SD1/SC1 ────────────►  sensor 1   arm at OFFSET + 60°
                     SD2/SC2 ────────────►  sensor 2   arm at OFFSET + 120°
                     SD3/SC3 ────────────►  sensor 3   arm at OFFSET + 180°
                     SD4/SC4 ────────────►  sensor 4   arm at OFFSET + 240°
                     SD5/SC5 ────────────►  sensor 5   arm at OFFSET + 300°
                     SD6/SC6 ────────────►  sensor 6   pointing straight UP
```

- **OFFSET = `SENSOR_ANGLE_OFFSET_DEG`** in `config.h`: `0` on this airframe,
  whose ring is mounted **between** the arms with sensor 0 facing straight out
  the nose. Use `30` only for a ring mounted **on** hexa X arms. It is sent as
  the `OBSTACLE_DISTANCE` `angle_offset`, so ArduPilot rotates the ring
  automatically.
- All seven VL53L0X keep the **default address `0x29`** — the mux gates the bus
  so only one is visible at a time (no XSHUT address-swapping needed).
- **Angles are clockwise from vehicle forward, seen from above** — this matches
  ArduPilot's `OBSTACLE_DISTANCE` convention, so mux channel N → sector N with
  no correction needed. If your build wires channels in a different physical
  order, remap `SECTOR_FOR_CHANNEL[]` in `config.h`.
- TCA9548A address is `0x70` (A0/A1/A2 low). Most breakouts include I²C
  pull-ups.

### UART: ESP32 `Serial2` → Pixhawk TELEM2 (`SERIAL2`)

```
ESP32                     Pixhawk TELEM2 (= SERIAL2)
-----                     --------------------------
GPIO17 (TX2) ───────────► RX     (Pixhawk receives our OBSTACLE_DISTANCE)
GPIO16 (RX2) ◄─────────── TX     (optional — lets the ESP32 hear the Pixhawk)
GND          ──────────── GND    (common ground — REQUIRED)
```

- **Cross the lines:** ESP32 **TX**→Pixhawk **RX**, ESP32 **RX**→Pixhawk **TX**.
- Share **GND**. Don't back-power the ESP32 from TELEM2's 5 V unless you know the
  port's current budget — power the ESP32 from USB or its own 5 V regulator.
- On **ESP32-WROVER**, GPIO16/17 are used by PSRAM — remap `PIXHAWK_TX_PIN` /
  `PIXHAWK_RX_PIN` in `config.h` to free pins. WROOM-32 is fine as-is.
- USB serial (UART0) is used separately for the debug log.

---

## 3. Build & flash

### Option A — PlatformIO (recommended)

```bash
cd "esp32_obstacle_avoidance"
pio run -t upload            # build + flash
pio device monitor           # watch the debug log @115200
```

`platformio.ini` pulls the two libraries automatically (`okalachev/MAVLink`,
`pololu/VL53L0X`).

### Option B — Arduino IDE

1. Install the **ESP32 board package** (Boards Manager → “esp32” by Espressif).
2. Install libraries (Sketch → Include Library → Manage Libraries):
   - **MAVLink** by *Oleg Kalachev*
   - **VL53L0X** by *Pololu* (only needed for real hardware, not fake mode)
3. Open `esp32_obstacle_avoidance.ino` (keep all files in one folder).
4. Select board **“ESP32 Dev Module”**, pick the port, and Upload.
5. Open Serial Monitor at **115200**.

> The MAVLink include is `#include <MAVLink.h>` (the Kalachev package). If you
> instead drop the raw `mavlink/c_library_v2` headers into your libraries
> folder, change it to `#include <common/mavlink.h>` in `mavlink_proximity.cpp`.

---

## 4. Test with no hardware (fake mode)

`config.h` ships with `USE_FAKE_SENSORS 1`. **Flight guard:** in fake mode the
MAVLink output is only transmitted while **GPIO4 is jumpered to GND** (bench
jumper). Without the jumper the debug log still runs but shows `TX 0 B`, so a
fake-mode build accidentally left on the drone can never feed phantom
obstacles to the Pixhawk. Real-sensor mode (`USE_FAKE_SENSORS 0`) ignores the
jumper entirely.

Flash it, fit the GPIO4-GND jumper, open the monitor, and you get one line
every 100 ms:

```
[    1234 ms] s0[ 30]:CLEAR s1[ 90]:CLEAR s2[150]: 50cm s3[210]:CLEAR s4[270]:CLEAR s5[330]:CLEAR UP:CLEAR | TX 205 B  (#123)
```

- All sectors report **CLEAR** (fake distance 8000 mm = 8 m is beyond
  `RANGE_MAX_CM`, so ArduPilot treats it as “no obstacle”).
- **Sector 2** shows **50 cm** — that's the injected obstacle. Change which
  channel and how far via `FAKE_VARY_CHANNEL` / `FAKE_VARY_MM` in `config.h`
  (channel 6 injects it on the UP sensor instead).
- `TX 205 B` confirms both messages (`OBSTACLE_DISTANCE` + `DISTANCE_SENSOR`)
  were packed and written to `Serial2` (exact count can vary slightly with
  MAVLink v2 zero-truncation). `TX 0 B` in fake mode means the GPIO4 guard
  jumper is not fitted.

To see the actual MAVLink bytes decoded, wire `Serial2` to a USB-UART adapter
and point **Mission Planner** or **MAVProxy** at it; the Proximity/Radar view
will show the ring with the one obstacle. Set `USE_FAKE_SENSORS 0` for real
sensors.

---

## 5. ArduPilot parameters (Pixhawk side)

Set these on the flight controller, then **reboot**. Load
`ardupilot_proximity.param` directly, or set them by hand. Values are for
**Copter 4.5/4.6** (verified against ArduPilot source). **Please still verify
against the ArduPilot docs for your exact firmware — you said you would.**

### Must-set

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SERIAL2_PROTOCOL` | `2` | MAVLink2 on TELEM2 (so the port parses our messages) |
| `SERIAL2_BAUD` | `115` | 115200 baud — **must equal** the ESP32's `PIXHAWK_BAUD` |
| `PRX1_TYPE` | `2` | Proximity backend = **MAVLink** (consumes `OBSTACLE_DISTANCE`) |
| `AVOID_ENABLE` | `7` | Avoidance sources bitmask; **bit 1 (value 2) = proximity** must be set. `3` or `7` both include it. |
| `RNGFND2_TYPE` | `10` | Upward sensor: rangefinder backend = **MAVLink** `DISTANCE_SENSOR`. RNGFND1 stays reserved for the downward TF-Luna. **Verify value on the ArduPilot rangefinder wiki.** |
| `RNGFND2_ORIENT` | `24` | 24 = pointing **up**. **Verify.** |

> Reboot after changing `SERIAL2_PROTOCOL` and `PRX1_TYPE` — they gate their
> sub-parameters and only take effect / appear after a restart.

### Proximity orientation (defaults are fine for a normal mount)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `PRX1_ORIENT` | `0` | 0 = normal, 1 = upside-down. Leave 0. |
| `PRX1_YAW_CORR` | `0` | Degrees to rotate the ring if sector 0 isn't the nose. |
| `PRX1_MIN` / `PRX1_MAX` | `0` | m expected range; 0 = use sensor's own min/max. |

### Avoidance tuning

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `AVOID_MARGIN` | `2.0` | m stand-off from objects in GPS modes (Loiter/PosHold). |
| `AVOID_BEHAVE` | `0` | 0 = Slide around, 1 = Stop at margin (Copter default 0). |
| `AVOID_DIST_MAX` | `5.0` | m at which avoidance kicks in for non-GPS modes (AltHold). |
| `AVOID_ACCEL_MAX` | `3.0` | m/s² max accel used while avoiding (smoothing). |
| `AVOID_BACKUP_SPD` | `0.75` | m/s max horizontal back-away speed. |
| `AVOID_BACKUP_DZ` | `0.10` | m deadzone before backing away (raise if it oscillates). |

### Optional (mission path-planning, not needed for simple avoidance)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `OA_TYPE` | `0` | 0=Disabled, 1=BendyRuler, 2=Dijkstra, 3=Dijkstra+BendyRuler. Set 1 only if you want AUTO/GUIDED/RTL to reroute around obstacles. |

**Naming notes (current firmware):** per-sensor params use the `PRX1_` prefix
(`PRX1_TYPE`, `PRX1_ORIENT`, …); the shared/filter params keep the plain `PRX_`
prefix (`PRX_FILT` default 0.25 Hz, `PRX_IGN_GND`, `PRX_ALT_MIN` default 1.0 m).
There is **no** `PRX1_FILT`. The pre-4.1 single-instance names (`PRX_TYPE` etc.)
no longer exist.

### Verify it's working
- Mission Planner → **Ctrl-F → Proximity**, or the **Radar/Proximity** view,
  should show the 6-sector ring updating. In QGC, **Analyze Tools → MAVLink
  Inspector** should show `OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` arriving
  at ~10 Hz from component 195.
- If it doesn't: confirm the TX/RX aren't swapped, the bauds match, you rebooted,
  and `SYSID_THISMAV` on the Pixhawk equals the ESP32's `MAV_SYSID` (default 1).

---

## 6. How the mapping works (design notes)

- **6 ring sensors ⇒ 60° increment.** ArduPilot reads `round(360 / increment)`
  = **6** sectors, i.e. `distances[0..5]`. Ring sensor on mux channel N fills
  sector N.
- **Sector 0 sits at `SENSOR_ANGLE_OFFSET_DEG` clockwise from the nose** (0°
  on this build, the ring being between the arms), sent as `angle_offset`,
  with `frame = MAV_FRAME_BODY_FRD`.
- **The upward sensor (channel 6) goes out as a separate `DISTANCE_SENSOR`**
  message with `orientation = MAV_SENSOR_ROTATION_PITCH_90` (24 = up). A failed
  read sends nothing that cycle; a "clear" read sends `RANGE_MAX_CM + 1`, which
  is outside `[min,max]` and therefore ignored as "nothing overhead".
- **Units:** `distances[]`, `min_distance`, `max_distance` are all **cm**;
  `sensor_type = MAV_DISTANCE_SENSOR_LASER (0)`.
- **`increment_f` (float) is set to 60.0** — ArduPilot prefers the float field;
  the integer `increment` is also set to 60 for other consumers. (If *both* were
  zero, ArduPilot would discard the whole message.)
- **Sentinels.** ArduPilot ignores a sector reading of `0`, `65535`, or anything
  outside `[min,max]`. So:
  - real obstacle in range → its distance in cm;
  - nothing in range → `RANGE_MAX_CM + 1` (“no obstacle”, shown as `CLEAR`);
  - read error / absent sensor → `65535` (“unknown”, shown as `ERR`);
  - unused sectors `8..71` → `65535`.
  All non-obstacle cases collapse to “nothing to avoid here” in ArduPilot.
- **The MAVLink C library** handles the v2 framing, the payload field-reordering,
  and the message CRC (`CRC_EXTRA = 23`) — the firmware never touches checksums.

---

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| `ch#: VL53L0X init FAILED` | Mux wiring/channel, sensor power, or wrong `TCA9548A_ADDR`. Check 3V3 + pull-ups. Try `I2C_CLOCK_HZ 100000`. |
| All sectors `ERR` on real hardware | `Wire` pins wrong, mux not powered, or address clash. Set `TCA_SETTLE_US` to ~50 on long/noisy wiring. |
| Nothing in Mission Planner Proximity view | Reboot Pixhawk; check `SERIAL2_PROTOCOL=2`, `PRX1_TYPE=2`, baud match, TX/RX not swapped, common GND, matching `SYSID`. |
| Readings beyond ~1.2 m unstable | Normal for VL53L0X in bright light. Set `VL53L0X_LONG_RANGE 1` for ~2 m (slower, needs low IR) and lower `RANGE_MAX_CM` if needed. |
| Compile: `MAVLink.h: No such file` | Install the **MAVLink** (Kalachev) library, or switch the include to `<common/mavlink.h>` for a raw c_library_v2 drop-in. |
| Loop stutters when a sensor dies | Expected: a dead channel waits up to `SENSOR_TIMEOUT_MS` (50 ms) before erroring. Lower it, or unplug/disable that channel. |

---

## 8. Quick tuning cheatsheet (`config.h`)

- `USE_FAKE_SENSORS` — `1` to test with no hardware, `0` for real sensors.
- `SENSOR_ANGLE_OFFSET_DEG` — `0` for a ring between the arms (this build), `30` for one on hexa X arms.
- `PIXHAWK_BAUD` — keep in sync with `SERIAL2_BAUD` (115200 ↔ 115).
- `RANGE_MIN_CM` / `RANGE_MAX_CM` — the window ArduPilot will act on.
- `SECTOR_FOR_CHANNEL[]` — remap if your physical channel order differs.
- `FAKE_VARY_CHANNEL` / `FAKE_VARY_MM` — the sector and distance to simulate.
- `VL53L0X_LONG_RANGE` — `1` trades speed for ~2 m reach.
