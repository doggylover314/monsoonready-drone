# Field checklist — SATURDAY 2026-08-15: SHOOT + SUBMISSION DAY

The goal: the FULL AUTONOMOUS LOOP (survey, detect, drop) on camera, one
continuous take, uploaded and submitted the same day. 2-3 attempts exist; a
failure costs a recharge, not the day. The one open blocker is Linux ->
Pixhawk (byte-shovel over the Bridge); if it is not working by morning, shoot
the piloted flight + ground AI demo and say on camera what is and is not
autonomous. Honest beats overclaimed, and an overclaim judges catch is fatal.

Abort rule, memorised before you leave: **flip to STABILIZE, never disarm.**
Disarm airborne only for an imminent person-strike or an unrecoverable flyaway.

**REHEARSE BEFORE YOU RECORD.** Nothing has flown since the ESP32 ring went
live, the ring params changed (PRX1_TYPE=2), and the gate got new endpoints
(560/1760). The first flight of the day is a REHEARSAL, not the take.

---

## TONIGHT, AT HOME

- [ ] **Charge everything**: flight pack(s), transmitter, MacBook, both
      phones, the camera you will film with. A power bank for the field.
- [ ] **SD CARD SEATED IN THE PIXHAWK** (standing item, nearly forgotten
      2026-08-10 because it was still in the reader). No card, no log.
- [ ] **Phone hotspot test**: hotspot on, UNO Q joins it, SSH in once, open
      https://drone.reysen.net and see the dashboard. Failure is free at home
      and unfixable at the field.
- [ ] **MacBook ready**: repo pulled, `.venv` present (`./python
      tools/wiring_check.py` complains about a missing serial device, not a
      missing module), QGC opens, SiK radio appears as /dev/cu.* when plugged
      in. The Linux laptop is NOT going; the MacBook is the field machine.
- [ ] **Write the running-order card** (bottom of this file) and tape it
      somewhere the narrator can read it.
- [ ] **Gate check with the new endpoints — PROPS OFF, hopper EMPTY**:
      `./python tools/wiring_check.py --wiggle` and watch the gate: closed
      560, open 1760, correct direction, full throw. Your eyes are the test.

## TAKE — aircraft

- [ ] The aircraft (closed: UNO Q, ESP32 ring, lidars, hopper all inside)
- [ ] **GPS puck and RC receiver reconnected** (they were off for bench work)
- [ ] Flight pack(s), FULL. If there is only one pack, the next item is what
      makes attempts 2 and 3 exist:
- [ ] **The charger + its power supply**, and a plan for where it plugs in
      between attempts
- [ ] Transmitter (check its battery)
- [ ] **Hopper payload** (Bti granules, or the stand-in you have been bench
      testing with) + a funnel or scoop + enough to reload for every attempt

## TAKE — ground station

- [ ] MacBook + **USB-C to USB-A adapter/hub** (SiK radio and Pixhawk cable
      are both USB-A; no adapter = no telemetry, no checks, no log pull)
- [ ] MacBook charger
- [ ] SiK telemetry radio
- [ ] USB cable for the Pixhawk (log pull) + SD card reader (backup route)
- [ ] Phone with hotspot and data: the board's internet (dashboard, and the
      opening Google-search shot needs internet at the field)

## TAKE — the demo target

- [ ] **Dark tray, ~60 cm** — the guaranteed puddle. Detection over grass
      needs a target that is certain to exist.
- [ ] **More water than you think** (carry cans/bottles; the tray must read
      as water from altitude, wind evaporates shallow fills)

## TAKE — filming

- [ ] The camera (or phone) you will film with: charged, empty enough for a
      10+ minute continuous take, plus its tripod/gimbal if one exists
- [ ] The running-order card (taped up)
- [ ] Second phone or power bank as camera backup

## TAKE — repairs and safety

- [ ] **PLIERS** (permanent line: motor nuts cannot be tightened by hand,
      and a nut backing off was crash #1)
- [ ] Hex drivers, Loctite, spare props, spare prop nuts
- [ ] Multimeter (pack voltage vs what the FC claims)
- [ ] Zip ties, electrical tape
- [ ] First aid kit. Six props at hover RPM cut.
- [ ] LiPo-safe bag for charged/hot packs
- [ ] Headlamp or torch (attempts can run late)
- [ ] Water and snacks for the humans, mosquito repellent (you are standing
      next to engineered mosquito habitat all afternoon)

## AT THE SITE — order of operations

- [ ] 1. **Transmitter ON before anything.** RC FAIL with the TX off is the
      check working, not a fault.
- [ ] 2. **Prop nuts with the pliers, every one.**
- [ ] 3. Power up, **do not arm for 2-5 min**:
      `./python tools/bench.py gps --seconds 300` until READY (10+ sats,
      HDOP < 1.5, 3D fix).
- [ ] 4. Full check over the radio: `./python tools/wiring_check.py` —
      expect FC / GPS / COMPASS / TF-LUNA / ESP32 / RC all PASS.
- [ ] 5. Hotspot up, board on it, dashboard loads on the phone.
- [ ] 6. Load the hopper. Place and fill the tray where the survey will pass.
- [ ] 7. **REHEARSAL FLIGHT, unrecorded.** Fly exactly what the take will be.
      Land, pull the log (`./python tools/check_log.py <log>.BIN`), fix what
      it shows, recharge if needed.
- [ ] 8. Reset everything to its mark, reload the hopper, refill the tray,
      THEN record.

## STOP AND GO HOME IF

- Any accelerometer clipping rise, or VibeZ median >= 15 on any IMU
- Any EKF variance message, or `Crash: Disarming` while genuinely airborne
- Learned hover throttle above 0.5 with the payload fitted
- Anything about the sound or the feel that you cannot explain

## KNOWN-GOOD — do not re-debug at the field

FC, GPS driver, compass, TF-Luna, SiK link: PASS (2026-08-02 wiring check).
UNO Q -> Pixhawk over D0/D1: comp 191 heard, TX direction proven 2026-08-13.
ESP32 ring: comp 195 heard 2026-08-14 after the TX/RX swap was fixed; ch0/1/
3/4 solid, ch5 intermittent, ch2 dead chip (known, accepted). Endurance is
CLOSED: 16.7 min to empty, 13.3 min to 20% reserve at 28.8 A true hover; if
the day needs a number, use that one. TF-Luna over water is a permanent no;
descend-BESIDE is the design.

---

# Video plan — the take itself

Requirements: **5-10 min, ONE continuous unedited take, publicly viewable,
opens with a Google search for the date on screen, shows functionality +
assembly + components.** Two people, one camera. Upload + submission the same
day — budget an hour at home on real wifi for the upload, not hotspot data.

Running order (~8 min):

1. **0:00-0:30 — Date proof.** Camera on a screen, Google "today's date",
   result visible. Project name and who you are while it is on screen.
2. **0:30-2:00 — Components.** Walk the airframe: Pixhawk, GPS mast, SiK,
   UNO Q + B525 camera, TF-Luna, ESP32 ring, hopper + MG90 gate, battery.
3. **2:00-3:00 — Assembly evidence.** Plate layout, wiring bay, splice work;
   what was made vs bought.
4. **3:00-4:00 — The AI, on the MacBook.** Detector live over the tray or
   over saved field frames, boxes on real puddles. Say the model, dataset
   size, and on-board inference time. Have it ALREADY RUNNING before 0:00.
5. **4:00-6:30 — The flight.** Pre-arm, take off, the survey, the detect,
   the drop over the tray. Camera never stops.
6. **6:30-8:00 — Result and honesty.** The treated target, the dashboard
   map, then state plainly what was autonomous and what was piloted.

Stage a charged spare pack (if one exists) where the pilot can reach it
without leaving frame. Single takes fail on forgetting, not on flying.
