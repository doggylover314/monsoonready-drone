# MonsoonReady PROJECT_STATE (machine-oriented; terse > readable)
# PURPOSE: single source of truth; append-only decision log at bottom; update on every new fact. Supersedes handoff v3 (2026-07-20) where they conflict.
# RULES(user,standing): logs-first troubleshooting (pymavlink on .bin); NO em-dashes; no invented specs/prices/URLs/param values; India sourcing Robu.in/Amazon.in/FabToLab/FlyRobo/IndiaMART, avoid zbotic+hitechxyz; don't re-suggest rejected; Build Log.txt user-maintained, don't edit/propose unless asked; RESPONSE_DEFAULTS(direct, no preamble/filler/summaries, plain prose/tight lists, Haiku-note for trivial, 15+msg offer fresh-chat summary once, corrections->note edit-last-message saves tokens)
# ENV: user OS=Zorin OS 18 (Linux). GCS=QGroundControl ONLY (AppImage at /media/sleuther/Stuff/). NOT Mission Planner. Laptop GPU=RTX3050 4GB. Project root=/media/sleuther/Stuff/Robu AI Challenge. GIT: private remote github.com/<github-account>/monsoonready-drone; STANDING RULE (user 2026-07-25): commit+push after every change; gitignore excludes datasets/runs/venvs/zips/param dumps+generated param files (pixhawk_full_setup.param IS tracked = hand-maintained config). UNO Q commands: give BARE commands, no ssh prefix (user keeps a terminal open).
# PARAM STATUS 2026-07-25 FINAL: all 55 setup params PUSHED+VERIFIED on board via tools/push_params.py (pymavlink, per-write ack; QGC bulk-load silently drops some writes, never trust it for loading). RNGFND1_GNDCLEAR renamed RNGFND1_GNDCLR (meters) in 4.7. Board exposes 1054 params, full backup in pixhawk_every_param.param (regen: tools/pull_params.py, QGC closed).

## COMPETITION
Arduino Physical AI Challenge India 2026. Deadline 2026-08-15. Judges Arduino/Qualcomm/Robu.in. Criteria: innovation, functionality, documentation, presentation. Edge AI rewarded, cloud discouraged. GPS+manual flight allowed, private-property demo OK.
AI-authorship: not penalized; risk=comprehension in Q&A; disclose as "AI-assisted, architecture/testing/debug mine"; keep dataset+bench evidence photos.
CONCEPT: hexacopter surveys, UNO Q onboard ML detects stagnant water in downward stills, descend over puddle, drop granular Bti (demo=inert salt), on landing UNO Q=base station (heatmap/report). Judged loop detect->descend->treat runs onboard.
FRAMING(decided): model detects "standing-water candidates"; stagnation confirmed by persistence across passes/operator; document honestly.
REGULATORY: Digital Sky parked (portal blocks self-reg); >2kg AUW=Small category; compliance gap acknowledged deliberately in docs; salt avoids pesticide law. User will write narrative.

## AIRCRAFT
Frame F550 (ex-S550 destroyed crash3; S550 empty weight WAS 1.85kg, hover 30% throttle @1.85kg [user: calc done, do not re-litigate]; F550 lighter). Hexa X layout (CONFIRMED)-> FRAME_CLASS 2, FRAME_TYPE 1, SENSOR_ANGLE_OFFSET_DEG 30 (firmware default, correct).
Pixhawk 2.4.8, ArduCopter 4.6.3. Motors 6x DJI A2212 920KV (ex-EMAX MT2213, prop availability). Props DJI-style 1045, handed M6 nutcaps (black CW silver CCW). ESC 6x 45A BLHeli_32. Batt 3S 8000mAh XT60. Charger HTRC B6 V2 (changed from SkyRC S65, noted 2026-07-25). GPS NEO-M8N+compass mast. RC FS-i6X+FS-iA10B iBUS. Telem 433MHz SiK. OLED SH1106 NTF_DISPLAY_TYPE=2 on compass I2C, 5V UBEC, works, splices need soldering, mount glanceable top plate.
Payload budget ~650g; AI payload ~230-340g. B525 stripped weight: still to weigh.
Switches: ch5 arm, ch8 kill, ch7 3pos Stab/AltHold/Loiter, spare 2pos RTL, Simple Mode per-mode checkbox.
PARTS: ALL ordered items ARRIVED incl 7x VL53L0X + TCA9548A (2026-07-25). Frame assembly STARTED. VectoBac G sourcing status: unclear/pending.

## POWER MAP
Power module (new, XT60) -> Pixhawk; calibrate V/I in QGC vs multimeter on install (UBEC bench V readings w/o sense pins = floating artifacts).
UBEC 5V/5A -> UNO Q 5V pin (NOT VIN; GPIO 3.3V; USB-C free for camera) + OLED.
XY-3606 buck #2 (set 5.00V w/ meter BEFORE connecting) -> SG90 servo AND ESP32 (decided 2026-07-25, same BEC as servo). ESP32 3V3 reg feeds mux+7x VL53L0X.
Servo signal from UNO Q GPIO 3.3V (known-marginal-but-usually-fine; watch for glitches; salvage 470-1000uF cap ONLY if servo misbehaves, never buy).
Common grounds everywhere. XT60 ~60A continuous rating: full-throttle 6-motor bursts may touch it (user aware, unlikely to matter).

## SERIAL MAP (FINAL, option B decided)
SERIAL1(TELEM1)=SiK radio. SERIAL2(TELEM2)=ESP32 MAVLink2 115200. SERIAL3=GPS. SERIAL4=UNO Q MAVLink2. SERIAL5=TF-Luna serial (rangefinder).
SERIAL4/5 share one 6pin DF13: pin1 5V->TF-Luna only; pin2 TX4->UNOQ D0(RX); pin3 RX4<-UNOQ D1(TX); pin4 TX5->TFLuna RXD; pin5 RX5<-TFLuna TXD; pin6 GND->both. Custom split cable from ordered DF13 pigtail. TF-Luna pin5 float=UART mode (verify datasheet pinout per batch). TF-Luna UART logic=3.3V ok.
PENDING TEST: SERIAL5 boot-noise (ESP32-as-USB-serial trick: EN->GND, Pixhawk TX5->ESP32 TX0 pin, monitor 57600+115200 across power cycles; also confirm SERIAL5_ params exist). If noisy -> fallback TF-Luna I2C (addr 0x10, mode pin low, ArduPilot Benewake I2C type, verify wiki).
UNO Q D0/D1=STM32 USART1 PB7/PB6 3.3V. Linux<->STM32 via Bridge RPC. Plan: STM32 sketch shovels raw bytes<->Bridge, pymavlink unchanged on Linux; measure latency in SITL. UNO Q STM32 exists for precise timing (user point vs RPi5).

## ESP32 OBSTACLE MODULE (esp32_obstacle_avoidance/)
7x VL53L0X via TCA9548A(0x70): ch0-5 ring one/arm 60deg (OBSTACLE_DISTANCE, angle_offset=30 hexaX), ch6 up (DISTANCE_SENSOR orient 24). 10Hz + 1Hz heartbeat. sysid1 compid195. ESP32 GPIO21/22 I2C, GPIO17->PixRX GPIO16<-PixTX Serial2 115200.
COMPILE STATUS: VERIFIED CLEAN 2026-07-25, both modes, PlatformIO esp32dev, MAVLink@2.0.31(okalachev) VL53L0X@1.3.1(pololu), RAM7% flash22%. Pin these versions if updating platformio.ini. NOT yet flashed/bench-tested on hardware.
FAKE-MODE FLIGHT GUARD (added 2026-07-25): USE_FAKE_SENSORS=1 only transmits MAVLink while GPIO4 jumpered to GND; unjumpered=TX 0 B + guard warning; real mode unaffected. Preflight: banner must say REAL mode.
Array is REINSTATED PoC (never call rejected; don't dwell sunlight). TF-Luna adds range; geofence=precaution atop sensors.
Test sequence: (1) fake+jumper, monitor 115200, 6 sectors+UP+TX bytes /100ms; (2) FAKE_VARY_CHANNEL sweep incl 6; (3) wire TELEM2+params+reboot; (4) QGC MAVLink Inspector: OBSTACLE_DISTANCE+DISTANCE_SENSOR ~10Hz from comp195; (5) USE_FAKE_SENSORS 0, hand-wave. Mount check: each ring sensor 25deg FOV must not see legs/props.
PRX gotcha: PRX1_TYPE=2 makes ESP32 an ARMING DEPENDENCY (prearm fails without feed). Field recovery: PRX1_TYPE=0 + reboot.

## PIXHAWK PARAMS
Canonical file: pixhawk_full_setup.param (project root, created 2026-07-25; supersedes esp32_obstacle_avoidance/ardupilot_proximity.param). VERIFY-marked lines need checking against 4.6.3 onboard list (RNGFND m-vs-cm names, RNGFND1_TYPE 20 for TF-Luna serial, SERIAL5_PROTOCOL 9, BRD_SER2_RTSCTS, GUID_TIMEOUT).
fmuv2/fmuv3 CHECK (pending): QGC boot messages show build name; if fmuv2 (feature-trimmed 1MB build; 2.4.8 boards actually have 2MB) reflash fmuv3 from firmware.ardupilot.org Copter stable fmuv3 arducopter.apj via QGC custom firmware file. Then confirm PRX1_TYPE/AVOID_ENABLE/RNGFND2_TYPE exist.
Failsafes: RC-loss RTL + low-batt RTL CONFIGURED (user-confirmed 2026-07-25). Crash3 disarm was DELIBERATE pilot choice (drift to unrecoverable area), not failsafe gap; do not recharacterize. Re-verify failsafe bench test after rebuild (TX off, props off). FENCE: not configured, user will do later (commented block in param file).
Notch: throttle mode placeholder; set REF=learned MOT_THST_HOVER from LOADED hover, FREQ/BW from FFT (INS_LOG_BAT_MASK 1 during tuning). MOT_HOVER_LEARN 2.

## NOTCH STRATEGY (settled 2026-07-25 post-flash)
Firmware = Pixhawk1-bdshot 4.7.0 stable, FLASHED OK, prior params survived (hexa X, 8000mAh, modes intact). Boot msg "Chan 13 to 14, PWM: failed, no DMA" = EXPECTED on this build, harmless with motors on MAIN.
bdshot RPM notch: DEAD END for hexa on this board (hwdef fmuv3-bdshot.inc verified: AUX5-6 NODMA = no DShot; only AUX1-4 DShot, only AUX2-3 BIDIR). Motors STAY on MAIN PWM. Do not re-propose the AUX move.
ADOPTED PATH: in-flight gyro FFT notch, build has HAL_GYROFFT_ENABLED (features.txt verified): FFT_ENABLE 1, FFT_MINHZ/MAXHZ ~50/200, INS_HNTCH_MODE 4, REF 1; commented block in pixhawk_full_setup.param; first hovers on throttle mode, then switch. Tracks payload-mass changes automatically (better than throttle mode for drop mission). BLHeli_32 value = ESC quality + passthrough config only; RPM telemetry unused.
QGC "Missing params: 1:ARMING_CHECK" dialog = 4.7.0 REPLACED ARMING_CHECK with ARMING_SKIPCHK (source-verified, bitmask of checks to SKIP, 0=run all; set 0). Latest QGC (github appimage, user reinstalled) still lags; popup is PERMANENT-cosmetic until QGC updates; no param file can fix it; ignore per boot.
Post-flash summary showed Batt1 low/crt failsafe = Land/Land (pre-existing, not policy RTL): fixed by loading pixhawk_full_setup.param (low=RTL 2, crt=Land 1).

## CRASH LESSONS (permanent)
C1 prop nut backed off->vibration->alt corrupt->prop departed (fix: handedness+Loctite). C2 AltHold wind drift into tree. C3 armed seconds after power-on HDOP65-99, vibration broke EKF, phantom-fall climb 7->47m full throttle, PILOT DISARMED at 47m deliberately (drift to unrecoverable area); frame destroyed; GPS healthy; procedural.
Rules: abort=flip Stabilize NOT disarm (disarm only imminent person-strike/unreachable flyaway). GPS modes need 10+sats HDOP<1.5 no-EKF-complaints, wait 2-5min. Hover VibeZ median<15 gate before AltHold/Loiter. Sat count!=quality. Insulate bare connectors pre-power. Smoke stopper first power-up of repaired wiring.
VIBRATION: unsolved blocker pre-rebuild (median 20.6 peak 60+, gate<15). Rebuild plan: bin bad props, rigid motors, service loops, FC foam near CG, baro covered, GPS cable tied, notch, compass recal. Bench: all 6 motors/ESCs/GPS/Pixhawk/telem/RX PASS; power module+buzzer died, replacements arrived. Wreck stripped. Loaded-hover verification of VibeZ+MOT_THST_HOVER still required (payload changes both).

## CAMERA / ML
Cam Logitech B525 720p UVC, strip housing, face down, weigh. Lock focus via v4l2-ctl (AF hunts). Route USB away from GPS mast (USB2 noise in GPS band); test sat count cam-on vs off. RPi CSI rejected (no carrier support).
TESTS DUE (user: "tomorrow" = 2026-07-26): UNO Q enumerates B525 host-mode while powered via 5V pin; v4l2 capture; model runtime on UNO Q (was Edge Impulse aarch64, now = export of local model); hopper build+flow test w/ salt (bridging/clumping; Bti granule size differs).
TRAINING (decided 2026-07-25): LOCAL on laptop (RTX3050 4GB), NOT Edge Impulse cloud. Ultralytics YOLO (yolov8n/11n), imgsz 640 batch<=8-16 for 4GB, ckpt every epoch, Ctrl+C pausable, auto-resume from last.pt. Pipeline: training/ dir (venv, merge_datasets.py collapses all classes->single "puddle", train.py auto-resume, export ONNX/TFLite for UNO Q). Deploy: onnx/tflite on UNO Q Linux. On-device story intact for judges.
DATASETS (Roboflow Universe, user screenshots 2026-07-25): USE: mosquito/luis-augusto-silva-bq4bv/mosquito-suh0p VERIFIED 2026-07-25: 2850 imgs OD, CC BY 4.0, classes bottle/tire/pool/bucket/puddle/water-tanks, drone-view breeding sites ex SMT Lab UFRJ = most mission-relevant, USE puddle class only (merge script filters), CITE in docs (BibTeX on its Universe page); siblings for later: aedes-total 18.5k, Aedes_uni 5.19k, MBG-mosquito 4.8k. ALSO USE: puddle_Detect/puddledetect 4.93k OD (core); "puddle detection" hanyang 1.5k OD; "puddle" HanYang 1.5k OD (dedupe vs other hanyang sets, likely overlap); water YINJIA 1.01k; yinjia-huang-water-part2. INSPECT-THEN-USE: 水たまり検出ver1; Puddles FumigationRobot 108 (nadir pavement); hanyang segment sets (seg->bbox convert). DOWNLOAD FORMAT: "YOLOv8" (YOLOv8 PyTorch TXT) zips -> training/exports/<name>/. merge_datasets.py (run with .venv python, needs yaml) keeps single-class sets whole, multi-class sets filtered to puddle/water-named classes, others dropped (images become negatives). SKIP: ObstacleD(chairs), Battleground(cards), Personnes_Malvoyantes, kirovka, DirtDetectionMeter, Merged_Sematics, Offroad-II both, OffRoadForest, black-ice, Spillage(50), zahra ObjectDetection. Check licenses per set. Domain gap: add own nadir phone photos from height + negatives (shadows/tarps/wet-ground/rooftops/plastic).
Predictive-model stub framework for judges: still to build.

## PARAM SYSTEM (2026-07-25 evening)
User factory-RESET the board (no prereset backup saved; old switch/mode config reconstructed from pre-reset QGC summary screenshot + handoff into setup file: FLTMODE_CH 7, FLTMODE1-6 = 0,0,2,2,5,5, RC5_OPTION 153 arm, RC8_OPTION 31 kill, RC6_OPTION 4 RTL commented until spare-switch channel confirmed by user).
QGC LOAD BUG SOLVED: QGC only loads its own TAB format (vid cid name value type); Mission-Planner NAME,VALUE files parse as ZERO params -> "no differences" message. tools/make_complete_params.py converts: dump + pixhawk_full_setup.param overrides -> pixhawk_complete.params (973 params, 40 overridden). tools/pull_params.py (pymavlink puller, appeared 2026-07-25, not from this session) outputs MP format: fine for backup, NOT loadable in QGC.
4.7 RENAMES source/dump-verified: ARMING_CHECK->ARMING_SKIPCHK; RTL_ALT->RTL_ALT_M (METERS not cm), RTL_ALT_FINAL->RTL_ALT_FINAL_M; unit-suffix wave (_M/_MS) likely hits RNGFND too, stage-2 dump will tell. CONFIRMED existing: BRD_SER2_RTSCTS, SERIAL5_PROTOCOL (default -1), GUID_TIMEOUT, FS_THR_ENABLE(=1)/FS_THR_VALUE(=975), FFT_ENABLE. FLTMODE_CH default 5 would collide with ch5 arm switch: we set 7.
STAGE FLOW: dump1 (973 defaults) -> stage1 load -> dump_stage2 (1005 params, gates BATT_/INS_HNTCH_/RNGFND2_ opened). FINDING 2026-07-25: QGC bulk load PARTIALLY applied: persisted BATT_MONITOR/INS_HNTCH_ENABLE/RNGFND2_TYPE/SERIAL2_/SERIAL5_/FRAME_/AVOID_ etc, but SILENTLY DROPPED at least PRX1_TYPE, RNGFND1_TYPE, SERIAL4_PROTOCOL, SERIAL4_BAUD, BRD_SER2_RTSCTS, FLTMODE3, RC5_OPTION, INS_LOG_BAT_MASK, RNGFND2_ORIENT. pixhawk_complete.params regenerated off stage2 (1005 rows, 49 overrides). [PENDING user] load again, reboot, dump as param_dump_stage3.params, regen; iterate until make_complete_params prints zero diffs+zero missing (PRX1_ORIENT/YAW_CORR + RNGFND1_MIN/MAX/ORIENT/GNDCLEAR still to appear). If stage3 still drops writes: use pymavlink direct param_set with ack verification over USB (QGC closed), extend tools/pull_params.py.
2026-07-25: mosquito v1 (367M, 6 classes incl puddle, 5069 imgs) replaces excluded v4 (v2 too large, user got v1); merge filters to puddle-only. Full dataset now 11725 train / 3069 val. Training RESTARTED from scratch on it (old run deleted at epoch 6, ~8min, mAP50 was 0.42 on 5-set data).

## TRAINING RUN (2026-07-25)
Datasets extracted by user to training/exports/: puddle_Detect.v2i (275M), puddle-detection.v32-640pure (115M), puddle.v2i (83M), water.v2i (100M), yinjia-part2 (37M) = all single-class 'puddle'. mosquito.v4i (258M) EXCLUDED -> training/excluded/: v4 has NO puddle class (bucket/pool/tire/water-tanks only); would train water scenes as negatives = poison. FIX: download/fork mosquito VERSION 2 (6 classes incl puddle) at 640px, then re-merge + resume/retrain.
Merged: 7262 train / 2463 val, single class puddle. Training STARTED detached (setsid nohup, training/train.log, yolov8n 640px batch 8, RTX3050): pause = kill the train.py process (checkpoint per epoch), resume = rerun train.py. Monitor: runs/puddle/results.csv.

## UNO Q ACCESS
ssh arduino@<tailnet-ip> (user-provided 2026-07-25). Key auth NOT yet set up (password-only; user to run ssh-copy-id, never give Claude the password). Camera currently disconnected. Pending on-board setup: v4l-utils, onnxruntime, opencv-python-headless, inference benchmark script, camera enumeration+focus-lock test.
MOSQUITO DATASET SIZE: full-res export is 14GB; do NOT download raw and do NOT train on Roboflow hosted (weights not downloadable; sequential training = catastrophic forgetting). Use/generate a 640x640-resized version (Fork Dataset if none of the 4 versions is resized), export YOLOv8, expect a few hundred MB. All other datasets: downloaded+unzipped by user 2026-07-25.
FULL PARAM DEFINITION WORKFLOW: user wants EVERY firmware param defined. Method: QGC Parameters>Tools>Save to file -> param_dump_4.7.0.params in project root; Claude merges pixhawk_full_setup.param decisions over the dump -> complete file; re-dump after each calibration session as git-versioned config backup. PENDING: user has not saved dump yet.

## MISSION SW (to build)
UNO Q: guided cmds via pymavlink (SITL first), descent-abort reading DISTANCE_SENSOR (TF-Luna via Pixhawk; dropout=abort UPWARD never continue), servo drop state machine, base-station heatmap/report, MAV_CMD_SET_MESSAGE_INTERVAL at startup for DISTANCE_SENSOR+GLOBAL_POSITION_INT on SERIAL4, compid 191 (not 195). GUID_TIMEOUT low. Pilot ch7->Stabilize = drilled abort.
TF-Luna-over-water risk: 850nm IR vs still water absorb/specular; bench test on arrival basin indoor/outdoor nadir+angle. FALLBACK (user-approved): hover+descend NEXT TO puddle over ground.
SITL: full install pending; rehearse survey, guided, param file load to catch name errors, latency test. FPV Freerider (line-of-sight) for nose-in skill; own cable=/dev/input/js0 verified.

## TEST/DO CHECKLIST (live)
NOW-POSSIBLE: fmuv2/v3 check; SERIAL5 boot-noise; flash fake-mode+jumper bench test; SITL install; dataset export+merge+train; UNO Q camera tests; hopper build (user: 2026-07-26).
AFTER-ASSEMBLY (user: basic, will do): accel+level cal, ESC cal, motor order/direction, FRAME params, compass cal, radio cal, RC failsafe bench re-verify, power module calibrate, XY-3606 set 5.00V, sensor-FOV self-detection check, GPS-vs-USB-cam interference test.
GATES: bench GPS check -> UNLOADED hover VibeZ<15 -> Loiter -> LOADED hover (VibeZ + relearn MOT_THST_HOVER, set INS_HNTCH_REF) -> payload integration -> mission -> footage by ~Aug 10 -> docs.
DESCOPE LADDER: (a) Loiter+onboard detect+drop = minimum judgeable; (b) +base station report; (c) +obstacle array PoC; (d) +guided auto descent.
CONFORMAL COAT LAST (mask baro/USB/connectors).

## DECISION LOG (append here)
2026-07-19 servo cap skipped; salvage only if servo misbehaves.
2026-07-20 handoff v3; ESP32 fw updated 6@60deg+up; sensors 7x plan.
2026-07-24 serial option B (split SERIAL4/5 cable) chosen over TF-Luna I2C; hexa X confirmed.
2026-07-25 project moved to /media/sleuther/Stuff/Robu AI Challenge; all parts arrived; frame assembly started; ESP32 powered from servo BEC (XY-3606); charger=HTRC B6 V2; fw compiles clean both modes (libs pinned above); fake-mode GPIO4 TX guard added; README TX-bytes fixed (205); pixhawk_full_setup.param created; local YOLO training adopted (pausable, resume) replacing Edge Impulse cloud path (env verified: ultralytics 8.4.105, torch 2.13.0 cu130, RTX3050 detected); standing-water framing adopted; AVOID_MARGIN 1 / AVOID_DIST_MAX 1.5 sized to sensor range; Build Log fixes verified (battery reason, backup removed); failsafes RC+battery RTL confirmed configured; FENCE deferred.
2026-07-25 (later) firmware upgraded via QGC to Pixhawk1-bdshot 4.7.0 stable (from 4.6.3); flash verified, params survived. bdshot RPM notch then found IMPOSSIBLE for hexa (AUX5-6 NODMA per hwdef); motor-to-AUX plan cancelled same day; adopted in-flight FFT notch instead (HAL_GYROFFT_ENABLED confirmed in build). Serial map option B unchanged throughout. TF-Luna stays SERIAL5.
