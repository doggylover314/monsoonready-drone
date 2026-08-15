# Farm checklist — SATURDAY 2026-08-15: SHOOT DAY

**Tomorrow is the ONLY farm access; the submission upload is about a week
later (user, 2026-08-14), from home wifi.** So tomorrow's job is the
FOOTAGE, and the footage is unrepeatable. Back the video file up to a laptop
AND to Drive the same evening — a phone lost or wiped in the following week
must not be able to take the submission with it.

**VENUE IS A FARM, not the grass field: dry dirt, much bigger area.** Prop
wash on dirt is a dust cloud into motors, the B525 lens and the lidar ring,
so takeoff happens from a weighted cloth sheet. There is a marketing office
with mains power (charger between attempts), a verandah with tables, a water
pump (tray refills), a kitchen (salt refills), and a farm first aid kit.
BOTH laptops go: Linux laptop primary (every tool proven on it, native
USB-A), MacBook backup.

The goal: the FULL AUTONOMOUS LOOP (survey, detect, drop) on camera, one
continuous take. ONE pack only, so each attempt costs a recharge at the
marketing office; budget 2-3 attempts across the day. If the Linux ->
Pixhawk link is not working by morning, shoot the piloted flight + ground AI
demo and say on camera what is and is not autonomous. Honest beats
overclaimed, and an overclaim judges catch is fatal.

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
- [ ] **Both laptops ready**: repo pulled on BOTH; Linux laptop is primary
      (all tools proven there, native USB-A); MacBook is backup (`.venv`
      works, QGC opens, SiK appears as /dev/cu.* when plugged in).
- [ ] **Write the running-order card** (bottom of this file) and tape it
      somewhere the narrator can read it.
- [ ] **MEASURE THE CAMERA FOV. 10 minutes, and it decides whether the drop
      lands on the target or several metres from it.** Nobody has ever
      measured it, so `--hfov-deg` has never been passed, so every detection
      so far would resolve to NADIR: the mission flies to where the aircraft
      was standing when it saw the water, not to the water. At 15 m that
      error can be half a camera footprint.
      Point the camera square at a wall a measured distance away, mark where
      the left and right edges of the picture fall, measure between the
      marks, then:
      `./python uno_q/calibrate_camera.py --distance 2.0 --width <measured>`
      Write the number it prints on the checklist and **pass `--hfov-deg
      <that number>` on every run_mission command.**
- [ ] **Make the target BIG, not tray-sized. The arithmetic decides this,
      not taste.** A target spans `target_m / footprint_m * 640` pixels in
      the model's input, and YOLO is unreliable below roughly 40 px. A 0.6 m
      tray at 15 m comes out around 13-25 px depending on the true FOV,
      which is a coin flip at best. The farm has a water pump and dirt: wet
      a patch **2-3 m across** and the same sum gives 80-150 px, plus it
      looks far more like the training data (real water on ground) than a
      plastic tray does. Keep the tray as a backup target.
      `calibrate_camera.py` prints this pixel figure once the FOV is known.
- [ ] **Push params FIRST, then gate check — PROPS OFF, hopper EMPTY**:
      `./python tools/parameters.py push` (delivers SERVO9 MIN 500 / MAX
      1800 / TRIM 560 and PRX1_TYPE=2), then
      `./python tools/wiring_check.py --wiggle` and watch AND listen: closed
      560, open 1760, full throw, and NO buzzing at closed (buzzing = servo
      stalled against the end stop; back the closed value off to ~580-600 and
      tell the assistant). The wiggle ends on a close, which shuts the gate
      that servo_jog left open. Your eyes are the test.
- [ ] **Power-cycle the aircraft after the push** and confirm the gate SITS
      CLOSED at boot, before arming, with nothing commanding it. That is
      SERVO9_TRIM doing its job; if it boots open anyway, say so.

## PACK LIST — tick each item as it goes in the vehicle

Status from the 2026-08-14 night audit is in brackets; PACKED still gets
ticked when it physically enters the car, because "packed at home" and "in
the vehicle" have different failure modes.

Aircraft:
- [ ] The aircraft (UNO Q, ESP32 ring, lidars, hopper inside; no rattles —
      shaken and confirmed; battery lives strapped inside it)
- [ ] GPS puck + RC receiver ON it (reconnected, wiring check passed)
- [ ] The pack — the only one — FULL (charging overnight/early morning)
- [ ] Charger + its supply (mains at the marketing office between attempts)
- [ ] Transmitter (charged) + 1 set spare AAs
- [ ] Starter salt in a bag (top up from the farm kitchen as needed)

Ground station:
- [ ] Linux laptop + charger (primary field machine)
- [ ] MacBook + charger (backup)
- [ ] USB-C to USB-A adapter (packed — MacBook's only route to the radio)
- [ ] SiK telemetry radio (in the MacBook bag)
- [ ] Known-good DATA USB cable for the Pixhawk (not a charge-only one)
- [ ] SD card reader
- [ ] Hotspot phone, charged, with data (board already joined the hotspot)

Target:
- [ ] The dark tray
- [ ] A water can to carry pump water to the tray at the plot

Filming:
- [ ] Vivo X300 FE: charged, storage cleared for a 10+ min take
- [ ] Printed/written shooting script (see VIDEO_SCRIPT.md) — one copy per
      person
- [ ] Power bank
- [ ] Tripod OPTIONAL: handheld is fine for the walking take; a stand only
      helps the two screen close-ups if hands get tired

Repairs and safety:
- [ ] PLIERS (permanent line: a nut backing off was crash #1)
- [ ] Hex drivers + Loctite
- [ ] Every spare prop owned (few exist — fly conservative, no showing off)
- [ ] Multimeter
- [ ] Zip ties, electrical tape
- [ ] Cloth sheets (landing pad; weigh the corners with stones or the dust
      cloud problem comes back with a flapping sheet on top)
- [ ] Lens cloth (packed)
- [ ] Torches (attempts can run late; farm has first aid already)
- [ ] Human water + snacks + mosquito repellent

At home, NOT in the car:
- [ ] SD card seated in the Pixhawk (confirmed 2026-08-14, re-check at the
      door anyway — it has nearly been lost to the reader before)

## AT THE SITE — order of operations

- [ ] 1. **Transmitter ON before anything.** RC FAIL with the TX off is the
      check working, not a fault.
- [ ] 2. **Prop nuts with the pliers, every one.**
- [ ] 3. Power up, **do not arm for 2-5 min**:
      `./python tools/bench.py gps --seconds 300` until READY (10+ sats,
      HDOP < 1.5, 3D fix).
- [ ] 3b. **FIRST laptop action: `./python tools/parameters.py push`** —
      this DELIVERS AVOID_ENABLE=0, which never reached the board on Friday
      night (the drone left for Raghav's before the push ran). Until this
      runs, the degrading ring still steers the aircraft.
- [ ] 4. Full check over the radio: `./python tools/wiring_check.py` —
      expect FC / GPS / COMPASS / TF-LUNA / ESP32 / RC all PASS.
      **RC is a HARD GATE: with the TX on it must PASS before arming — an
      RC failsafe mid-mission is an RTL straight through the take.**
      RING/UP-SENSOR FAILs are expected and accepted (avoidance is off).
- [ ] 4b. **Confirm comp 191 on the bus**: `./python tools/bench.py nodes`
      while the pump runs (see mission start below) — the last box to tick
      on the Linux->Pixhawk link.
- [ ] 4c. **POWER-CYCLE THE AIRCRAFT SHORTLY BEFORE THE TAKE, and check the
      ring afterwards.** The ESP32 probes each lidar channel exactly ONCE at
      boot and latches a failed channel dead for the entire session; there is
      no retry. That is why the sector count only ever fell last night (2 dead
      -> 3 -> 4) across a board that stayed powered for hours: those were
      RUNTIME read timeouts on a hot, sealed frame, and nothing re-initialises
      them while the power stays on. A cold boot re-runs init and may well
      bring channels back. It costs 30 seconds and it is the only lever left
      without opening the airframe.
- [ ] 5. Hotspot up, board on it, dashboard loads on the phone.
- [ ] 5b. **Dashboard needs flask on the board.** If it errors with
      ModuleNotFoundError: `~/venv/bin/pip install flask msgpack`
      (board_setup.sh now installs both, but the board was set up before
      that fix landed).
- [ ] 6. Load the hopper (salt from the kitchen if the bag runs out). Place
      the tray where the survey will pass, fill it from the pump.
- [ ] 6b. **Mission start (BOARD over SSH, two terminals — the pump is a
      server and gets its own). All from ~/monsoonready-drone, `git pull`
      first.**
      terminal 1: `~/venv/bin/python uno_q/mav_shovel_pump.py`
      terminal 2, sanity first (read-only):
      `~/venv/bin/python uno_q/test_mission_link.py`
- [ ] 6c. **Make the survey (no waypoint file exists — it is made HERE):**
      carry the aircraft to the corner of the plot where the survey should
      START, point its NOSE along the row direction, then (terminal 2, with
      3D fix): `~/venv/bin/python uno_q/make_waypoints.py --out wp_farm.txt`
      Default = 3 rows x 20 m, 5 m apart, starting at the aircraft,
      extending dead ahead. Put the tray under the MIDDLE row.
- [ ] 6d. **Fly it** (terminal 2):
      `~/venv/bin/python uno_q/run_mission.py --conn udpin:127.0.0.1:14555
      --waypoints wp_farm.txt --hfov-deg <measured> --camera <N>`
      `--model` now defaults to the repo's own models/best.onnx, so it no
      longer has to be passed. **`--hfov-deg` DOES**: without it every
      detection resolves to nadir (see the FOV item above).
      `--camera <N>`: check with `v4l2-ctl --list-devices` first; the
      default of 1 is unverified on the reflashed image.
      Add `--no-drop` for the rehearsal if you want the full loop flown
      with nothing to clean up.
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
assembly + components.** Two people, one camera. Upload happens ~a week
later from home wifi; TONIGHT'S job after the shoot is only to BACK UP the
file (laptop + Drive) so no phone accident can lose it.

**The full shooting script with lines to say is VIDEO_SCRIPT.md.** The
running order below is the skeleton; the script is what you print.

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
