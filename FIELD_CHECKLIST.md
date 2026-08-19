# Field checklist — the NEARBY FIELD shoot (rewritten 2026-08-16 after the
# farm day; the farm items it replaced are in git history)

**The submission upload is about a week after the shoot, from home wifi.**
The shoot's job is the FOOTAGE, and the footage is unrepeatable. Back the
video file up to a laptop AND to Drive the same evening — a phone lost or
wiped in the following week must not be able to take the submission with it.

**WHAT CHANGED SINCE THE FARM (2026-08-16):** the Pixhawk now talks to the
UNO Q over its own USB cable into a hub on the board's USB-A port (byte
shovel deleted, SERIAL5 wires out); the camera is opened BY NAME so the
farm's index-shuffle failure is impossible; every program logs to ~/logs/;
the dashboard is light-mode, starts the mission, runs the self-test, and
takes photos. Grass field, not dirt: the dust-cloud rule relaxes, the
weighted sheet is still the cleanest pad.

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

## THE NIGHT BEFORE, AT HOME

- [ ] **Charge everything**: flight pack, transmitter, MacBook, both
      phones, the filming camera, BOTH power banks (one films, one keeps
      the hotspot phone alive).
- [ ] **SD CARD SEATED IN THE PIXHAWK** (standing item, nearly forgotten
      2026-08-10 because it was still in the reader). No card, no log.
- [ ] **PUSH THE PARAMS — this delivery now carries four changes:**
      `./python tools/parameters.py push` delivers **SERIAL5_PROTOCOL,-1**
      (the shovel port, now dead), the **FENCE block** now including
      **FENCE_TYPE,7** (altitude + circle + the hand-drawn POLYGON), and
      re-asserts AVOID_ENABLE=0. Pixhawk on the laptop by USB, or pull the
      Pixhawk plug from the hub and use it. **After the push, power-cycle
      and confirm it still ARMS on the bench.**
- [ ] **Draw the geofence at home if you already know the field's shape**
      (dashboard map -> Draw fence -> click the corners -> Save -> Push
      fence). It is stored inside the Pixhawk and survives reboots, so this
      can be done once and re-checked on site. IT IS NOT OPTIONAL AND THE
      AIRCRAFT WILL NOT TELL YOU: with FENCE_TYPE,7 and no polygon loaded,
      Copter 4.7 arms perfectly happily and simply has no polygon boundary
      (tested in SITL 2026-08-16). The dashboard's **fence** self-test line
      is the thing that catches it — treat a red 'fence' row as a no-fly.
- [ ] **Board network: PHONE HOTSPOT DIRECTLY (user, 2026-08-16 — the
      MR3020 kept failing to associate in WISP mode, so the router is OUT
      of the plan).** Turn the hotspot on at home, then join the board to it
      from the dashboard: **Settings panel -> Scan -> pick the hotspot ->
      password -> Connect**. Do it at home so the profile is SAVED; at the
      field the board then joins it on its own at boot. iPhone: Maximize
      Compatibility ON (2.4 GHz). Note the board's new IP from the hotspot's
      client list — the dashboard URL changes with the network.
- [ ] **Full-system test at home, exactly as the field will run:** board on,
      dashboard up (`--enable-control`), press **Test everything** on the
      dashboard, all green except GPS indoors. This exercises the camera
      by-name, the Pixhawk USB link, battery, Luna and ring in one button.
- [ ] **Gate check — PROPS OFF, hopper EMPTY**:
      `./python tools/wiring_check.py --wiggle`, watch AND listen: closed
      560, open 1760, full throw, NO buzzing at closed. Power-cycle after
      and confirm the gate SITS CLOSED at boot (SERVO9_TRIM doing its job).
- [ ] **Tape / strain-relieve the hub plugs** (camera + Pixhawk + hub into
      the board). The leading theory for the farm camera failure is a plug
      walking out under flight vibration; tape is the one-rupee fix.
- [ ] **Both laptops ready**: repo pulled on BOTH; Linux laptop primary.
- [ ] **Write the running-order card** (bottom of this file).
- [ ] FOV is DONE: 56.2 deg measured 2026-08-15, baked into camera_geom;
      no flag needed. Re-measure ONLY if the camera or housing changes.
- [ ] **Target stays BIG: wet a patch 2-3 m across** (80-150 px in the
      model input vs a coin-flip 13-25 px for the 0.6 m tray). Tray = backup.

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
- [ ] USB-C to USB-A adapter (MacBook's only route to the radio)
- [ ] SiK telemetry radio (in the MacBook bag)
- [ ] Known-good DATA USB cable for the Pixhawk (not a charge-only one)
- [ ] SD card reader
- [ ] **Hotspot phone, charged, with DATA — this is now the whole network**
      (the MR3020 is OUT: WISP mode kept failing to associate, user
      2026-08-16). Board and laptop both join the phone directly.
- [ ] **Power bank for the hotspot phone** — it will run all day serving
      two clients and filming nothing; a dead phone is now a dead network.

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
- [ ] 3. **Network up first — PHONE HOTSPOT, no router:** hotspot ON
      (Maximize Compatibility if iPhone), laptop joins it, board joins it on
      power-up because the profile was saved at home. If the board does not
      appear: plug the laptop into the board over USB/SSH on the old
      network, or use the dashboard **Settings -> Scan -> Connect** from
      whichever network still reaches it. Internet through the phone means
      the board's clock NTP-syncs and every log timestamp is true.
- [ ] 4. Power the aircraft, **do not arm for 2-5 min**. Meanwhile, from
      the laptop, SSH in and start the dashboard with the one command:
      `bash ~/monsoonready-drone/uno_q/start_dashboard.sh`
      Open `http://<board-ip>:8080` (IP from the hotspot's client list;
      `http://arduino-drone.local:8080` if mDNS cooperates).
- [ ] 5. **Press TEST EVERYTHING on the dashboard.** 30 s, no motors, no
      servos: camera by name, Pixhawk USB heartbeat, GPS vs the arming
      rules (10+ sats, HDOP <= 1.5, 3D), battery voltage, TF-Luna, ring,
      **fence** (is the drawn polygon actually inside the Pixhawk?) and
      **prearm** (the exact reason it would refuse to arm). Names stay on
      screen and tick off live. Every failure prints its reason in the panel
      and in ~/logs/test_everything.log. Do not arm until GPS goes green.
- [ ] 5a. **Fence: draw it on the map for THIS field** if the shape changed
      (map header -> Draw fence -> click corners -> Save -> Push fence), then
      press **Check arming** (5 s) to confirm the aircraft holds it. Nothing
      else will warn you if it does not.
- [ ] 5b. **RC is a HARD GATE**: `./python tools/wiring_check.py` over the
      radio must show RC PASS with the TX on — an RC failsafe mid-mission
      is an RTL straight through the take. (Ring FAILs stay accepted.)
- [ ] 5c. **POWER-CYCLE THE AIRCRAFT SHORTLY BEFORE THE TAKE.** The ESP32
      probes each lidar channel exactly ONCE at boot and latches failures
      for the whole session (runtime timeouts never recover while powered).
      A cold boot re-runs init; 30 seconds, and it is the only lever.
- [ ] 6. Load the hopper. Place the tray / wet the patch where the survey
      will pass.
- [ ] 6b. **Make the survey (no waypoint file exists — it is made HERE):**
      carry the aircraft to the corner where the survey should START, point
      its NOSE along the row direction, then over SSH (3D fix needed):
      `~/venv/bin/python uno_q/make_waypoints.py --out wp_field.txt`
      Default = 3 rows x 20 m, 5 m apart, starting at the aircraft,
      extending dead ahead. Target under the MIDDLE row. (Run it BEFORE the
      mission; they share the one serial port.)
- [ ] 6c. **Fly it FROM THE DASHBOARD: Arm controls -> START MISSION.**
      The mission launches detached on the board (SSH dropping cannot kill
      it), the map follows it live, and ~/logs/run_mission.log carries the
      1 Hz telemetry line plus every command and ack.
      **STOP (RTL) on the dashboard is the deliberate abort**; it is
      SIGTERM, the graceful wind-up. Every default is baked: conn auto,
      camera auto, HFOV 56.2, model from the repo. `--no-drop` rehearsal =
      start the dashboard with `--no-drop`, or tick nothing and let the
      gate fire (hopper is the rehearsal variable, not the code).
- [ ] 7. **REHEARSAL FLIGHT, unrecorded.** Fly exactly what the take will
      be. Land, pull the SD log (`./python tools/check_log.py <log>.BIN`),
      read ~/logs/run_mission.log, fix what they show, recharge.
- [ ] 8. Reset everything to its mark, reload the hopper, refill the
      target, THEN record.

## STOP AND GO HOME IF

- Any accelerometer clipping rise, or VibeZ median >= 15 on any IMU
- Any EKF variance message, or `Crash: Disarming` while genuinely airborne
- Learned hover throttle above 0.5 with the payload fitted
- Anything about the sound or the feel that you cannot explain

## KNOWN-GOOD — do not re-debug at the field

FC, GPS driver, compass*, TF-Luna, SiK link: PASS (2026-08-02 wiring check).
*Compass caveat from the farm logs: two prearm "Check mag field" failures
(1038 and 1058 vs an 875 ceiling) across power cycles — if prearm complains
about mag field or compass variance at the field, move the aircraft away
from the car/phones/power bank and re-check before touching calibration.
UNO Q -> Pixhawk over USB through the hub: PROVEN 2026-08-16, 8/8 heartbeats
x3 while the camera streamed 600 frames beside it. The D0/D1 + SERIAL5
shovel is DELETED and its wires are out.
Camera: opened BY NAME since 2026-08-16 (the farm failure was the camera
losing the /dev/video0 race to the video codecs after a replug; impossible
now). ESP32 ring: comp 195 heard 2026-08-14; ch0/1/3/4 solid, ch5
intermittent, ch2 dead chip (known, accepted). Endurance is CLOSED: 16.7 min
to empty, 13.3 min to 20% reserve at 28.8 A true hover. TF-Luna over water
is a permanent no; descend-BESIDE is the design.

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

THERE IS NO SPARE PACK (user, repeated, last 2026-08-18): one 3S pack,
~13 min usable. Plan every rehearsal against the take's energy budget.
Single takes fail on forgetting, not on flying.
