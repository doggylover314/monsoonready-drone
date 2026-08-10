# MonsoonReady → Autonomous Larvicide Drone

An **F550 hexacopter** that surveys for standing water after monsoon rain,
detects it with a **YOLO model running onboard an Arduino UNO Q**, descends
over the target on a **TF-Luna** rangefinder, and drops **granular larvicide**
into it. Detection, decision and action all happen on the aircraft; no ground
station and no cloud are in the loop.

Arduino Physical AI Challenge India, 2026. Two-person team.

---

## 1. The problem

*Aedes aegypti*, the mosquito carrying dengue and chikungunya, breeds in small
bodies of standing water that appear everywhere after monsoon rain: flat
rooftops, construction sites, blocked drains, tarpaulins, unused tanks. The
larval stage is where the mosquito is least mobile and most vulnerable.
Municipal vector control relies on ground crews walking sites, which is slow
and misses what a person cannot reach or does not know about.

**The design insight is that larval control does not need spraying.** Granular
*Bti* only has to **land in the water**. That removes the tank, the pump, the
nozzle, the spray drift and most of the payload weight, and reduces the problem
to: find the water, get above it, drop a measured dose.

---

## 2. The mission loop

| Step | Actor | Action |
|------|-------|--------|
| 1 | Pixhawk | Fly survey pattern at survey altitude (`survey_alt_m`, default 15 m) |
| 2 | UNO Q | Run detector on downward stills; **onboard**, no link required |
| 3 | UNO Q | On detection, **latch** target lat/lon at survey altitude |
| 4 | UNO Q → Pixhawk | Guided reposition over target, then descend on rangefinder |
| 5 | UNO Q | At drop height, open the servo gate; granules fall |
| 6 | UNO Q → Pixhawk | Climb to survey altitude, resume the pattern |
| 7 | UNO Q | On landing, switch role to base station: heatmap and report |

Demonstration flights dispense **inert salt**, not larvicide, so that no part
of the demonstration is a pesticide application. See `05_compliance_narrative.md`.

---

## 3. What is claimed, and what is not

The model detects **standing-water candidates**. It cannot detect stagnation,
because stagnation is a property of time and not of a single frame: a fresh
puddle and a two-week-old breeding site are identical from 15 m.

Stagnation is therefore established the way it actually can be, by **the same
candidate persisting across repeated passes on different days**, plus operator
confirmation. A single flight produces candidates. A survey programme produces
breeding sites.

This limitation is stated rather than absorbed into the claim, because the
narrow honest version survives questioning and the broad version does not.

---

## 4. Hardware

| Subsystem | Part | Rationale |
|-----------|------|-----------|
| Frame | **F550** hexacopter, X | Six motors: payload margin and motor-failure tolerance. Replaced the S550 destroyed in crash 3; centre plates no longer available. |
| Flight controller | **Pixhawk 2.4.8**, ArduCopter **4.7.0** (Pixhawk1-bdshot) | Mature guided-mode MAVLink interface, strong logging for post-incident analysis |
| Motors / props | 6× **DJI A2212 920KV**, DJI-style **1045** | Changed from EMAX MT2213 when matching propellers proved unavailable in India |
| ESCs | 6× **45A BLHeli_32** | Rated far above the A2212's draw, so they never run hot |
| Battery | **3S 8000mAh** LiPo, XT60 | Survey endurance |
| AI compute | **Arduino UNO Q, 4GB** | Runs the detector onboard. The physical-AI premise of the project. |
| Camera | **Logitech B525**, 720p UVC | Already owned, and UVC works on the UNO Q today, which outweighed any spec gain from a new part |
| Height / descent | **Benewake TF-Luna**, serial, downward | Native ArduPilot support; true height above the water surface, and puddle size for dosing |
| Obstacle ring | 7× **VL53L0X** on a **TCA9548A** mux, read by an **ESP32** | Cheapest proximity ring the flight controller understands natively |
| Dispenser | **MG90** servo gate on a tube | Metal gears, no tank, no pump, no nozzle |
| Status display | 1.3in I²C **OLED** (SH1106) | Prearm pass/fail, satellites, EKF, mode, battery: field-readable with no laptop |
| Telemetry | **433MHz SiK** | Ground monitoring during tests |
| RC | **FlySky FS-i6X / FS-iA10B**, iBUS | 10 channels; dedicated arm and kill switches |

### 4.1 Serial allocation

Every subsystem speaks to the flight controller in a protocol it already
understands. Nothing here requires a firmware fork.

```
Pixhawk 2.4.8
  SERIAL1 ──── SiK 433MHz telemetry
  SERIAL2 ──── ESP32 obstacle module      MAVLink2 115200, compid 195
  SERIAL3 ──── NEO-M8N GPS + compass mast
  SERIAL4 ──── UNO Q mission computer     MAVLink2 115200, compid 191
  SERIAL5 ──── TF-Luna rangefinder        serial, 115200
```

SERIAL4 and SERIAL5 share one 6-pin DF13 split cable; pin 1 (5 V) feeds the
TF-Luna only. The UNO Q's `D0`/`D1` are STM32 `USART1` at 3.3 V, bridged to the
Linux side by RPC.

- The **ESP32** emits standard `OBSTACLE_DISTANCE` (6-sector ring) and
  `DISTANCE_SENSOR` (upward), so ArduPilot's existing avoidance consumes them
  unmodified. Full detail in `esp32_obstacle_avoidance/README.md`.
- The **UNO Q** commands the aircraft with standard guided-mode messages as
  component 191. Full detail in `uno_q/README.md`.

---

## 5. The detection model

### 5.1 Task framing

Single class, `puddle`. Every source dataset is collapsed to that one class by
`training/merge_datasets.py`. Multi-class sets keep only water-named classes;
other boxes are dropped, and an image left with no boxes **stays in as a
negative**, which is free hard-negative data.

Classes named `pool` or `water tank` are **deliberately not kept**. They are
genuine breeding sites, but they are not drop targets for this aircraft, and
learning them would produce confident detections the mission logic would then
have to suppress. Not learning them is the better design.

### 5.2 Model choice

**`yolo26n`**, 640 px, trained on the RTX 3050 laptop.

| Candidate | Verdict |
|-----------|---------|
| `yolov8n` | Run-1 baseline. Superseded. |
| **`yolo26n`** | **Adopted.** Better accuracy at similar size, roughly 2× faster CPU ONNX, and an **NMS-free export** that removes a postprocessing stage from the UNO Q's CPU. |
| YOLO v12+ attention models | Rejected: too slow on an A53-class CPU for what they add |
| Cloud inference | Rejected: see 5.5 |

Training augmentation includes `degrees=180` and `flipud=0.5`, because a nadir
drone view has no canonical "up".

### 5.3 Training data

| Dataset | Train | Val |
|---------|-------|-----|
| v1 | 11,725 | 3,069 |
| v2 (merged 2026-07-25) | ~21,700 | ~4,500 |

Licences and full attribution in `03_dataset_citations.md`. All public sets are
CC BY 4.0, which makes attribution a licence obligation.

### 5.4 Results

Run 1: dataset v1, `yolov8n` baseline, stopped around epoch 160 of 200 after
plateauing.

| Metric | Run 1 | v2 / `yolo26n` |
|--------|-------|----------------|
| Precision | 0.79 | TBD |
| Recall | 0.72 | TBD |
| mAP50 | 0.789 | TBD |
| mAP50-95 | 0.474 | TBD |
| UNO Q inference | not benchmarked | TBD |

**Known failure modes**, found by inspecting run-1 predictions image by image
rather than by reading the metrics:

| Failure | Description | v2 response |
|---------|-------------|-------------|
| Sheet water | A thin film across a wide surface, with no puddle-like outline | First-party nadir photographs at survey height |
| Glare | Specular sun on the water surface | Same, shot across times of day |
| Close range | Frames where water fills most of the frame are unreliable | Handled in software: target latching, section 6 |

The close-range finding is the one that changed the architecture rather than
the dataset.

### 5.5 Why inference is onboard

1. A drone over a construction site has **no dependable link**.
2. Cloud round-trip latency does not fit a **descent decision loop**.
3. The challenge is about **physical AI at the edge**; moving the model to a
   datacentre answers a different question.

The model runs on the aircraft or the project has failed.

---

## 6. Mission logic and safety

Implementation and state diagram in `uno_q/README.md`. Three behaviours are
worth calling out here, because each exists in response to something that has
already gone wrong or is known to be physically risky.

### 6.1 Target latching

Target coordinates are locked on **first detection at survey altitude**, and
detections are ignored for the rest of the manoeuvre.

The reason is the run-1 finding above: close-range frames are exactly where the
model is least reliable. A re-detecting implementation would let the least
trustworthy frames steer the aircraft. Latching means the **most informative
view, the wide one from altitude, is the one that decides**.

### 6.2 Descent aborts upward

The TF-Luna uses **850 nm infrared**, and still water at that wavelength
behaves close to a mirror. Specular dropout over the exact thing being
descended toward is **expected, not hypothetical**.

A naive "no reading means abort" rule would abort every descent, because the
sensor cannot see the ground from 15 m at all: **the first part of every
descent is legitimately blind**. The implemented rule distinguishes the cases:

| Condition | Interpretation | Action |
|-----------|----------------|--------|
| No reading, above `rng_expect_m` | Normal. Out of sensor range. | Continue descent |
| No reading, below `rng_expect_m` | Should see ground by now, does not | **Abort upward** |
| Reading acquired, then lost | Dropout | **Abort upward** |
| EKF below drop height, rangefinder never confirmed | Altitude sources disagree | **Abort upward** |
| Valid reading ≤ `drop_alt_m` | Confirmed at drop height | Drop |

A missed puddle costs nothing. A blind descent costs the aircraft. After two
crashes caused by a corrupted altitude estimate (`02_crash_postmortems.md`), no
single altitude source is allowed to act alone.

### 6.3 The pilot always wins

If the flight mode changes away from `GUIDED` for any reason, the mission code
**stands down** and stops commanding the aircraft. It never fights the human on
the sticks.

### 6.4 Simulation evidence

`uno_q/sitl_test.py` runs two scripted scenarios against ArduPilot SITL with a
simulated hexacopter and rangefinder:

| Scenario | Pass criteria | Result 2026-07-26 |
|----------|---------------|-------------------|
| Nominal | Exactly one drop, survey completes, RTL, no abort | **PASS** (drop at rng 2.98 m) |
| Rangefinder dropout | Zero drops, abort upward, survey still completes | **PASS** |

This is the functionality evidence for the parts of the loop not yet flown.

---

## 7. Base station mode

After landing, the UNO Q stops being a mission computer and becomes a report
server: a heatmap of detections over the survey area, the treated sites, and
the image that triggered each drop. A survey then produces a municipal work
product rather than just a flight, and repeated surveys are what turn
"candidate" into "confirmed breeding site" per section 3.

Status: **not implemented** (`PROJECT_STATE.md` TODO 13).

---

## 8. What is not done

| Item | State |
|------|-------|
| Airframe | Rebuild in progress after crash 3 |
| **Vibration** | **The open blocker.** Median ~20.6 against a gate of 15. Until an unloaded hover clears the gate, no altitude-holding mode is trustworthy on this aircraft. |
| ESP32 obstacle module | Compiles clean in both modes; **never flashed** |
| Detect-descend-treat loop | Proven in simulation; not yet flown |
| Base station | Not implemented |
| Digital Sky registration | Parked; portal blocks self-registration. See `05`. |

**Descope ladder**, in order, if time runs short:

1. Loiter + onboard detection + drop → the minimum judgeable demonstration
2. \+ base-station report
3. \+ obstacle array
4. \+ fully automatic guided descent

---

## 9. Reproducibility

The project is one git repository. Every decision is in a dated, author-tagged,
append-only log in `PROJECT_STATE.md`, because two people on two machines built
it.

| Task | Command |
|------|---------|
| Merge datasets | `.venv/bin/python training/merge_datasets.py` |
| Train | `.venv/bin/python training/train.py` (checkpoints per epoch, resumes from `last.pt`) |
| Export ONNX | `.venv/bin/python training/export.py` |
| Push Pixhawk params | `python tools/parameters.py push` (per-write ack) |
| Dump Pixhawk params | `python tools/parameters.py pull` |
| Mission tests | `.venv/bin/python uno_q/sitl_test.py` |

Ground-station **bulk parameter load is not used**: it was found to silently
drop writes, which is why `tools/parameters.py` acknowledges every individual
write. The mission code runs unchanged against the simulator and the aircraft;
only the connection string differs.

---

## 10. AI assistance

Disclosed in full in `08_ai_authorship_disclosure.md`.
