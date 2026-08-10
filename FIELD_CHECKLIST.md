# Field checklist — session of Mon 2026-08-10

One pack. Objective: **a real endurance number**, plus prove the parts that
have never been proven. Everything else is a bonus.

Abort rule, memorised before you leave: **flip to STABILIZE, never disarm.**
Disarm airborne only for an imminent person-strike or an unrecoverable flyaway.

---

## BEFORE YOU LEAVE (at the bench, ~20 min)

- [ ] **1. Gate travel — PROPS OFF, hopper EMPTY.** The gate moves (confirmed
      2026-08-10) but opened the WRONG WAY, because wiring_check had its own
      hard-coded 1900/1000 and ignored the dropper's values. Fixed: it now
      reads them from `dropper.py`, so the banner should read
      `closed 1600us -> open 1000us, about 60 deg counter-clockwise`.
      `./python tools/wiring_check.py --wiggle`
      **Watch the gate.** If the throw is not ~60 deg, tune without editing
      code: `--servo-open-us` / `--servo-closed-us`. If it still turns the
      wrong way, swap those two numbers, then tell me so the defaults in
      `dropper.py` are corrected. An ACCEPTED ack proves only that the command
      arrived; your eyes prove the direction.
- [ ] **2. Salt flow test — still on the bench.** Load the hopper, trigger the
      gate, catch the salt, weigh it, time it. Write down **grams per second**.
      This is now load-bearing, not optional: the dropper does VARIABLE doses
      (gate held open longer for a bigger puddle, 0.3-3.0 s), and without a
      flow rate those seconds are proportional to nothing. Measure it at two
      dwells if you can, e.g. 0.5 s and 2 s, so you know whether flow is
      actually linear in time or whether the gate takes a moment to get going.
- [ ] **3. Phone hotspot to the UNO Q.** Not set up yet, and without it there
      is no in-flight detection recording. Do it now, on your home wifi, where
      failure is free. Turn the hotspot on, join the board to it, note the IP
      from the phone's connected-devices list, and SSH in once to prove it.
- [ ] **4. Pack a tray and water.** It just rained but the field is grass. A
      dark tray of water roughly 60 cm across is visible from 3-5 m and gives
      the camera something real to detect. Bring more water than you think.
- [ ] **5. Charge state**: note the pack's resting voltage before you go, so
      the endurance figure has a starting point.

## TAKE

Aircraft, the pack, charger, TX, MacBook + USB cable, SiK radio, phone,
**Loctite + hex drivers**, spare props, tray + water, salt, something to
restrain the aircraft for the motor test, notebook.

## AT THE SITE — pre-arm

- [ ] **5b. TRANSMITTER ON before you run anything.** Every bench run so far
      has reported `FAIL RC ... receiver ABSENT/UNHEALTHY`, which is the check
      doing its job with the TX switched off. It must read PASS before you arm.
- [ ] **6. Prop nuts by hand, every one.** C1 was a nut backing off.
- [ ] **7. Power up, DO NOT ARM for 2-5 minutes.**
      `./python tools/bench.py gps --seconds 300`
      Wait for `READY` (10+ sats, HDOP < 1.5, 3D fix). Indoors on 2026-08-10 it
      already reached fix 3 with 8 sats, so outdoors this should come quickly.
- [ ] **8. Full wiring check over the radio.**
      `./python tools/wiring_check.py`
      Expect FC / GPS / COMPASS / TF-LUNA / RC / SiK PASS. The rangefinder
      reading below 0.20 m on the legs is now reported as expected, not FAIL.
- [ ] **9. Start the detection worker** (only if step 3 worked):
      `~/venv/bin/python ~/uno_q/detect_worker.py --model ~/best.onnx --camera 2 --save-dir ~/field_$(date +%H%M)`
      Put the tray of water where the aircraft will hover over it.

## THE ENDURANCE FLIGHT (the one that matters)

Fly **one continuous hover** rather than several hops: interrupted flights
make the mAh-per-minute figure meaningless.

- [ ] **10.** Take off in **Stabilize**, climb to about 2 m, settle.
- [ ] **11.** Switch to **AltHold** and hold station over the tray. Start a
      stopwatch. Note the time at each of: first low-battery warning, and any
      change in how it flies.
- [ ] **12. Land as soon as the low-battery warning sounds.** Do not fly to
      the failsafe deliberately — you already know it triggers RTL and climbs
      to 15 m, and a LiPo taken below ~3.3 V per cell resting is damaged.
- [ ] **13.** Disarm. Note the stopwatch time and the pack's resting voltage
      after a couple of minutes.

## AFTER LANDING, BEFORE ANY SECOND FLIGHT

- [ ] **14.** Pull the log **by USB cable or SD card** — never over the radio,
      a `.bin` at a few kB/s takes hours.
      `./python tools/check_log.py <log>.BIN`
      Exit code is the verdict now: 0 cleared, 1 a gate failed, 2 unjudgeable.
      Read the **burn rate** line: that is your real endurance.
- [ ] **15.** Compare the log's consumed mAh against what the charger puts
      back in. This is the only independent check on the current sensor, and
      it decides whether ~105 A is real or `BATT_AMP_PERVLT` is mis-scaled.
- [ ] **16.** Pull the detection frames if the worker ran:
      `scp -r arduino@<hotspot-ip>:~/field_* /tmp/`

## KNOWN-GOOD AS OF 2026-08-10 (do not re-debug these)

FC, GPS driver, compass, TF-Luna, and the SiK link all PASS. The aircraft has
flown. The obstacle ring is parked and unplugged by choice. TF-Luna over water
is settled as a permanent no, so descend-BESIDE is the only design and there is
no basin test to run.

## STOP AND GO HOME IF

- Any accelerometer clipping rise (flight 37 had +16, watch whether it grows)
- VibeZ median ≥ 15 on any IMU
- Any EKF variance message, or a `Crash: Disarming` while genuinely airborne
- Learned hover throttle above 0.5 with the payload fitted
- Anything about the sound or the feel that you cannot explain

---

# Weekend video plan

**DATE CONFLICT, RESOLVE THIS BEFORE PLANNING ANYTHING ELSE:** today is Monday
2026-08-10, which PROJECT_STATE records as the footage freeze (~Aug 10). The
plan to shoot at the farm "on the weekend" puts the recording on the 15th-16th,
five days the wrong side of that date. Either the freeze is softer than
recorded, or the video has to be shot far sooner than the weekend. Check the
actual submission deadline before committing to a weekend shoot.

Requirements that constrain everything: **5-10 min, ONE continuous unedited
take, publicly viewable, opens with a Google search for the date on screen,
must show functionality + assembly + components.** Two people, one camera.
AI demo on a laptop screen on the ground.

**The single-take problem**: you cannot stop, so the running order has to be
physically walkable and every prop must already be in place. Rehearse the
walk without recording first.

Suggested running order (roughly 8 min):

1. **0:00-0:30 — Date proof.** Camera on the laptop screen, Google "today's
   date", show the result. Say the project name and who you are while it is on
   screen. Get this right; it is a disqualification criterion.
2. **0:30-2:00 — Components, on the bench.** Camera operator walks the
   airframe with the pilot narrating: Pixhawk, GPS mast, SiK radio, the UNO Q
   and B525 camera underneath, TF-Luna, the hopper and MG90 gate, the battery.
   Pick each up where possible. This is the "components used" requirement.
3. **2:00-3:00 — Assembly evidence.** You cannot rebuild it on camera, so show
   the build: the plate layout, the wiring bay, the splice work, and say what
   was made versus bought.
4. **3:00-4:00 — The AI, on the laptop.** Camera on the screen: run the
   detector over the saved field frames or live over the tray, showing boxes
   drawn on real puddles. Say the model, the dataset size, and the board's
   inference time (511 ms on the UNO Q). **Have this already running and
   tested before you start recording** — a laptop waking up on camera is a
   minute of dead air you cannot cut.
5. **4:00-6:30 — The flight.** Walk to the aircraft, pre-arm, take off, hover,
   demonstrate control, then the drop over the tray. Camera operator keeps the
   aircraft in frame and does not stop recording.
6. **6:30-8:00 — Result and honesty.** Show the treated target, then back to
   the laptop for the base-station map if it is running. Close by stating
   plainly what is autonomous today and what is piloted. **Judges reward a
   clear-eyed limitation more than an overclaim, and an overclaim they catch
   is fatal.**

**Autonomy decision**: the full autonomous loop needs the UNO Q talking to the
Pixhawk, which is still blocked. You chose to test D0/D1 on the board first —
do that at home, not at the farm, and set a hard cutoff: if the link is not
proven by Friday night, shoot the piloted version. A piloted flight plus a
convincing ground AI demo is a complete, honest submission. A failed
autonomous attempt mid-take is an unusable recording.

**Two things to prepare that cost nothing**: a charged spare pack staged where
the pilot can reach it without leaving frame, and a written running order taped
where the narrator can see it. Single takes fail on forgetting, not on flying.
