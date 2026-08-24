# The obstacle ring

Firmware for an ESP32-WROOM-32 that reads seven VL53L0X time-of-flight sensors
through a TCA9548A multiplexer and streams them to the Pixhawk over `Serial2` at
10 Hz. Six ring sensors, one per hexa arm and 60 degrees apart, go out as
`OBSTACLE_DISTANCE`. The seventh points up on mux channel 6 and goes out as a
separate `DISTANCE_SENSOR` with orientation up.

The reading layer and the MAVLink layer are separate, and a `USE_FAKE_SENSORS`
flag verifies the MAVLink output with no hardware attached.

Read this alongside the note in `docs/README.md` about what the ring currently
does on this aircraft, which is report distances and steer nothing.

## Files

| File | Purpose |
|------|---------|
| `esp32_obstacle_avoidance.ino` | Main sketch: setup, 10 Hz scheduling, debug log |
| `config.h` | Every pin, baud rate, range, ID and fake-sensor knob |
| `proximity_sensors.h/.cpp` | Sensor layer, TCA9548A and VL53L0X. No MAVLink. |
| `mavlink_proximity.h/.cpp` | MAVLink layer, builds and sends the messages. No hardware. |
| `platformio.ini` | PlatformIO build. Also opens as an Arduino sketch. |
| `ardupilot_proximity.param` | Historic standalone parameter file. See the warning below. |

## Wiring

```
ESP32                TCA9548A               VL53L0X x7 (one per mux channel)
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

`SENSOR_ANGLE_OFFSET_DEG` in `config.h` is 0 on this airframe, whose ring is
mounted between the arms with sensor 0 facing straight out the nose. Use 30 only
for a ring mounted on hexa X arms. It is sent as the `OBSTACLE_DISTANCE`
`angle_offset`, so ArduPilot rotates the ring itself.

All seven sensors keep the default address `0x29`, because the mux gates the bus
so only one is visible at a time and no XSHUT address-swapping is needed. The
TCA9548A sits at `0x70` with A0, A1 and A2 low, and most breakouts include the
I2C pull-ups. Angles run clockwise from vehicle forward seen from above, which
matches ArduPilot's convention, so mux channel N fills sector N with no
correction. A build that wires channels in a different physical order remaps
`SECTOR_FOR_CHANNEL[]` in `config.h`.

The serial link runs from the ESP32's `Serial2` to a Pixhawk telemetry port. On
this aircraft that is TELEM1, because TELEM2 carries the SiK radio.

```
ESP32                     Pixhawk TELEM1
-----                     --------------
GPIO17 (TX2) ───────────► RX     (Pixhawk receives our OBSTACLE_DISTANCE)
GPIO16 (RX2) ◄─────────── TX     (optional, lets the ESP32 hear the Pixhawk)
GND          ──────────── GND    (common ground, required)
```

Cross the lines, ESP32 TX to Pixhawk RX and ESP32 RX to Pixhawk TX, and share
ground. Do not back-power the ESP32 from the port's 5 V unless you know that
port's current budget; power it from USB or its own regulator. On an
ESP32-WROVER, GPIO16 and 17 belong to PSRAM, so remap `PIXHAWK_TX_PIN` and
`PIXHAWK_RX_PIN`. WROOM-32 is fine as shipped. UART0 over USB carries the debug
log separately.

## Build and flash

PlatformIO is the easier route. `platformio.ini` pulls both libraries
automatically, `okalachev/MAVLink` and `pololu/VL53L0X`.

```bash
pio run -t upload -d esp32_obstacle_avoidance
```

```bash
pio device monitor -d esp32_obstacle_avoidance
```

From the Arduino IDE instead: install the Espressif ESP32 board package, add the
MAVLink library by Oleg Kalachev and the VL53L0X library by Pololu, open the
`.ino` with all files in one folder, select "ESP32 Dev Module", upload, and open
the Serial Monitor at 115200. The include is `<MAVLink.h>` from the Kalachev
package. Dropping raw `c_library_v2` headers into the libraries folder instead
means changing it to `<common/mavlink.h>` in `mavlink_proximity.cpp`.

## Testing with no hardware

`config.h` ships with `USE_FAKE_SENSORS 1`. In fake mode the MAVLink output only
transmits while GPIO4 is jumpered to ground. Without that jumper the debug log
still runs but shows `TX 0 B`, so a fake-mode build left on the drone by accident
can never feed phantom obstacles to the Pixhawk. Real-sensor mode ignores the
jumper entirely.

Flash it, fit the jumper, open the monitor, and one line arrives every 100 ms:

```
[    1234 ms] s0[ 30]:CLEAR s1[ 90]:CLEAR s2[150]: 50cm s3[210]:CLEAR s4[270]:CLEAR s5[330]:CLEAR UP:CLEAR | TX 205 B  (#123)
```

Every sector reads CLEAR because the fake distance of 8000 mm is beyond
`RANGE_MAX_CM`, which ArduPilot treats as no obstacle. Sector 2 shows the
injected obstacle, and `FAKE_VARY_CHANNEL` and `FAKE_VARY_MM` in `config.h`
change which channel and how far, with channel 6 injecting on the upward sensor
instead. `TX 205 B` confirms both messages were packed and written to `Serial2`,
and the exact count varies a little with MAVLink v2 zero-truncation.

To see the bytes decoded, wire `Serial2` to a USB-UART adapter and point Mission
Planner or MAVProxy at it. QGC's MAVLink Inspector should show
`OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` arriving at about 10 Hz from component
195.

## Pixhawk parameters

Do not load `ardupilot_proximity.param`. It is kept for history and it is stale:
it assumes TELEM2, it sets `AVOID_ENABLE,7` and `RNGFND2_TYPE,10`, and this
aircraft now runs the ring on TELEM1 with `AVOID_ENABLE,0`, `OA_TYPE,0` and
`RNGFND2_TYPE,0` after channel 6 died on 2026-08-21. Its values were also
checked against Copter 4.5 and 4.6, while the aircraft flies 4.7.0.

`param_dumps/pixhawk_full_setup.param` is the single maintained config for this
aircraft and it carries the current proximity block with the reasoning beside
each value. Push it with `tools/parameters.py`, which acknowledges every write,
rather than a ground station's bulk load, which drops them silently. Reboot after
changing a `SERIALx_PROTOCOL` or `PRX1_TYPE`, because both gate sub-parameters
that only appear after a restart.

## How the mapping works

Six ring sensors mean a 60 degree increment, so ArduPilot reads
`round(360 / increment)` as 6 sectors and the sensor on mux channel N fills
sector N. Sector 0 sits at `SENSOR_ANGLE_OFFSET_DEG` clockwise from the nose,
sent as `angle_offset` with `frame = MAV_FRAME_BODY_FRD`. The upward sensor goes
out separately with `orientation = MAV_SENSOR_ROTATION_PITCH_90`; a failed read
sends nothing that cycle, and a clear read sends `RANGE_MAX_CM + 1`, which falls
outside the valid window and is ignored as nothing overhead.

Distances are all in cm and `sensor_type` is `MAV_DISTANCE_SENSOR_LASER`.
`increment_f` is set to 60.0 because ArduPilot prefers the float field, and the
integer `increment` is also set to 60 for other consumers. Were both zero,
ArduPilot would discard the whole message.

ArduPilot ignores a sector reading of 0, of 65535, or of anything outside the
min and max. The firmware uses that: a real obstacle in range sends its distance
in cm, nothing in range sends `RANGE_MAX_CM + 1` and shows as CLEAR, a read
error or absent sensor sends 65535 and shows as ERR, and unused sectors 8 to 71
send 65535. Every non-obstacle case collapses to nothing to avoid here. The
MAVLink C library handles v2 framing, payload field reordering and the message
CRC, so the firmware never touches checksums.

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `ch#: VL53L0X init FAILED` | Mux wiring or channel, sensor power, or wrong `TCA9548A_ADDR`. Check 3V3 and pull-ups, try `I2C_CLOCK_HZ 100000`. |
| All sectors `ERR` on real hardware | `Wire` pins wrong, mux not powered, or an address clash. Set `TCA_SETTLE_US` to about 50 on long or noisy wiring. |
| Nothing in the Proximity view | Reboot the Pixhawk, then check the port protocol and baud, `PRX1_TYPE=2`, TX and RX not swapped, common ground, and matching `SYSID`. |
| Readings beyond about 1.2 m unstable | Normal for VL53L0X in bright light, and the reason the ring steers nothing on this aircraft. `VL53L0X_LONG_RANGE 1` reaches about 2 m, slower and only in low IR. |
| Compile fails on `MAVLink.h` | Install the Kalachev MAVLink library, or switch the include to `<common/mavlink.h>`. |
| Loop stutters when a sensor dies | Expected. A dead channel waits up to `SENSOR_TIMEOUT_MS` before erroring. This is what caused months of arming refusals, and the fix is to mark absent channels absent. |

## config.h cheatsheet

`USE_FAKE_SENSORS` is 1 to test with no hardware and 0 for real sensors.
`SENSOR_ANGLE_OFFSET_DEG` is 0 for a ring between the arms, which is this build,
and 30 for one on hexa X arms. `PIXHAWK_BAUD` has to stay in sync with the
Pixhawk's `SERIALx_BAUD`, where 115200 is written as 115. `RANGE_MIN_CM` and
`RANGE_MAX_CM` set the window ArduPilot will act on. `SECTOR_FOR_CHANNEL[]`
remaps a different physical channel order. `FAKE_VARY_CHANNEL` and
`FAKE_VARY_MM` pick the simulated sector and distance. `VL53L0X_LONG_RANGE`
trades speed for about 2 m of reach.
