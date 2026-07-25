# MonsoonReady PROJECT_STATE (machine-oriented; terse > readable)
# PURPOSE: single source of truth; TODO reflects only REMAINING work; append-only decision log at bottom; update on every new fact; commit+push after every change.
# RULES(user,standing): logs-first troubleshooting (pymavlink on .bin); NO em-dashes; no invented specs/prices/URLs/param values; India sourcing Robu.in/Amazon.in/FabToLab/FlyRobo/IndiaMART, avoid zbotic+hitechxyz; don't re-suggest rejected; Build Log.txt user-maintained, don't edit/propose unless asked; UNO Q commands BARE, no ssh prefix; RESPONSE_DEFAULTS(direct, no preamble/filler/summaries, plain prose/tight lists, Haiku-note for trivial, 15+msg offer fresh-chat summary once, corrections->note edit-last-message saves tokens)
# ENV: Zorin OS 18. GCS=QGroundControl ONLY (AppImage /media/sleuther/Stuff/; its "Missing params: 1:ARMING_CHECK" popup = permanent-cosmetic metadata lag, ignore). Laptop RTX3050 4GB. Project root=/media/sleuther/Stuff/Robu AI Challenge. Git: private github.com/<github-account>/monsoonready-drone, identity sleuther/<email>. UNO Q: ssh arduino@<tailnet-ip>, key auth WORKS, ~/venv has onnxruntime 1.28 + cv2 5.0, v4l-utils installed, 2.2G free.
# DEADLINE: 2026-08-15 (Arduino Physical AI Challenge India; judges Arduino/Qualcomm/Robu.in; innovation/functionality/documentation/presentation; edge AI rewarded). Footage freeze target ~Aug 10.

## CONCEPT + FRAMING
Hexacopter surveys; UNO Q onboard ML detects standing water in downward stills; descend over (or beside, see TF-Luna risk) puddle; drop granular Bti (demo=inert salt); on landing UNO Q=base station (heatmap/report). Judged loop detect->descend->treat runs onboard.
Framing: model detects "standing-water candidates", stagnation confirmed by persistence across passes/operator; document honestly. AI-authorship: disclose "AI-assisted; architecture/testing/debug mine"; keep dataset+bench evidence; be able to explain every design decision in Q&A.
REGULATORY: Digital Sky parked (portal blocks self-reg); >2kg AUW=Small category; compliance gap acknowledged deliberately; salt avoids pesticide law; narrative to write (user).

## AIRCRAFT (fixed facts)
F550 hexa X (FRAME_CLASS 2 TYPE 1; SENSOR_ANGLE_OFFSET_DEG 30). S550 predecessor was 1.85kg, hovered 30% throttle (user: settled, do not re-litigate); F550 lighter. Payload budget ~650g; AI payload ~230-340g.
Pixhawk 2.4.8 = Pixhawk1-bdshot 4.7.0 stable (flashed 2026-07-25). Motors 6x DJI A2212 920KV, DJI-style 1045 props, handed M6 nutcaps (black CW silver CCW). 6x 45A BLHeli_32 (RPM notch impossible on this board: AUX5-6 NODMA; motors stay MAIN PWM; ESCs = quality+passthrough only). 3S 8000mAh XT60. Charger HTRC B6 V2. NEO-M8N+compass mast. FS-i6X/FS-iA10B iBUS. 433MHz SiK. OLED SH1106 (NTF_DISPLAY_TYPE 2, compass I2C, 5V UBEC; splices need soldering; glanceable top-plate mount).
Switches: ch5 arm (RC5_OPTION 153), ch8 kill (RC8_OPTION 31), ch7 3pos Stab/AltHold/Loiter (FLTMODE_CH 7), spare 2pos=RTL (CHANNEL UNCONFIRMED: ask user, then uncomment RC6_OPTION,4 in setup file + push). Simple Mode per-mode checkbox (not restored post-reset; reconfigure if wanted).
Parts: ALL arrived. Frame assembly in progress.

## POWER MAP
Power module (new) -> Pixhawk (calibrate V/I vs multimeter on install; UBEC bench V readings w/o sense pins = floating artifacts). UBEC 5V/5A -> UNO Q 5V pin (NOT VIN; GPIO 3.3V; USB-C free for camera) + OLED. XY-3606 (set 5.00V with meter BEFORE connecting) -> SG90 servo + ESP32 (ESP32 3V3 reg feeds mux + 7x VL53L0X). Servo signal = UNO Q GPIO 3.3V (marginal-but-usually-fine; salvage 470-1000uF cap ONLY if servo misbehaves). Common grounds. XT60 ~60A continuous (full-punch bursts may touch it; known).

## SERIAL MAP (final)
SERIAL1=SiK. SERIAL2(TELEM2)=ESP32 MAVLink2 115200. SERIAL3=GPS. SERIAL4=UNO Q MAVLink2 115200. SERIAL5=TF-Luna serial rangefinder 115200.
SERIAL4/5 split cable (one 6pin DF13): pin1 5V->TF-Luna only; pin2 TX4->UNOQ D0(RX); pin3 RX4<-UNOQ D1(TX); pin4 TX5->TFLuna RXD; pin5 RX5<-TFLuna TXD; pin6 GND->both. TF-Luna pin5 float=UART (verify pinout per batch); its UART is 3.3V logic.
UNO Q D0/D1 = STM32 USART1 (PB7/PB6, 3.3V). Linux<->STM32 via Bridge RPC; plan = STM32 byte-shovel sketch, pymavlink unchanged on Linux; measure latency in SITL.

## PIXHAWK CONFIG (done state)
All 55 chosen params PUSHED+VERIFIED on board (tools/push_params.py, pymavlink per-write ack). Board exposes 1054 params; full backup pixhawk_every_param.param (regen: tools/pull_params.py, QGC closed). Canonical hand-maintained config = pixhawk_full_setup.param (tracked in git; dumps gitignored).
TOOLING RULES: NEVER trust QGC bulk load (silently drops writes; also only reads its own tab format). Writes = push_params.py. Dumps = pull_params.py or QGC save. make_complete_params.py builds QGC-format complete file from a QGC dump (float32-aware).
4.7 renames encountered: ARMING_CHECK->ARMING_SKIPCHK (0=all checks); RTL_ALT->RTL_ALT_M (m); RTL_ALT_FINAL->RTL_ALT_FINAL_M; RNGFND1_GNDCLEAR->RNGFND1_GNDCLR (m, set 0.13 placeholder, MEASURE on airframe).
Notch: first hovers throttle mode (loaded: INS_HNTCH_REF from learned MOT_THST_HOVER, FREQ/BW from log FFT, INS_LOG_BAT_MASK=1 active); then adopt FFT mode (FFT_ENABLE 1, FFT_MINHZ/MAXHZ ~50/200, INS_HNTCH_MODE 4, REF 1; commented block in setup file; HAL_GYROFFT_ENABLED confirmed in build). "Chan 13 to 14 PWM failed no DMA" boot msg = expected, harmless.
Failsafes on board: FS_THR_ENABLE 1 (RTL), BATT low RTL / crit Land, ARMING_SKIPCHK 0. FENCE: deferred by user (commented block ready).
PRX gotcha: PRX1_TYPE=2 makes ESP32 an ARMING DEPENDENCY. Field recovery: PRX1_TYPE=0 + reboot.

## ESP32 OBSTACLE MODULE (esp32_obstacle_avoidance/)
7x VL53L0X via TCA9548A 0x70: ch0-5 ring 60deg (OBSTACLE_DISTANCE angle_offset 30), ch6 up (DISTANCE_SENSOR orient 24). 10Hz + heartbeat, sysid1 compid195. GPIO21/22 I2C; GPIO17->PixRX, GPIO16<-PixTX. Compiles clean both modes (MAVLink@2.0.31 okalachev, VL53L0X@1.3.1 pololu; pin versions). NEVER FLASHED yet.
Fake-mode flight guard: USE_FAKE_SENSORS=1 transmits only with GPIO4 jumpered to GND; else TX 0 B + warning. Preflight: banner must say REAL.
Array = reinstated PoC (never call rejected; don't dwell sunlight). Mount check: 25deg FOV must not see legs/props.

## CRASH LESSONS (permanent)
C1 prop nut backed off -> vibration -> alt corrupt -> prop departed (fix: handedness+Loctite). C2 AltHold wind drift into tree. C3 armed seconds post-power-on HDOP 65-99, vibration broke EKF, phantom-fall climb to 47m, pilot disarmed DELIBERATELY (drift to unrecoverable area); procedural, GPS healthy.
Rules: abort=flip Stabilize NOT disarm (disarm only imminent person-strike/unreachable flyaway). GPS modes: 10+ sats, HDOP<1.5, no EKF complaints, wait 2-5min. VibeZ median<15 hover gate before AltHold/Loiter. Insulate bare connectors; smoke stopper on repaired wiring first power-up.
VIBRATION = THE unsolved blocker (was median 20.6, peaks 60+; gate <15). Rebuild countermeasures: good props only, rigid motors, service loops, FC foam near CG, baro covered, GPS cable tied, notch, compass recal.

## ML / TRAINING
Pipeline (training/): merge_datasets.py (single class "puddle"; single-class sets kept whole; multi-class filtered to puddle/water-named; dropped boxes -> negatives; run with .venv python) -> train.py (yolov8n 640 batch8, ckpt/epoch, Ctrl+C pause, auto-resume last.pt) -> export.py (ONNX for UNO Q onnxruntime).
Dataset v1 = 11725 train / 3069 val from: mosquito v1 (5069, puddle class only, CC BY 4.0, CITE BibTeX from its Universe page), puddle_Detect 4.93k, hanyang puddle-detection 1.5k + puddle 1.5k, water 1k, yinjia-part2. mosquito v4 EXCLUDED (no puddle class = poison; in training/excluded/). Optional later: aedes-total 18.5k, Aedes_uni 5.19k, MBG-mosquito 4.8k, 水たまり検出ver1, Puddles-108.
RUN1 (2026-07-25, in progress ~e160/200): P 0.79 R 0.72 mAP50 0.789 mAP50-95 0.474, plateaued.
Camera: B525 720p UVC; strip housing, weigh; lock focus v4l2-ctl; route USB away from GPS mast; test sat count cam-on vs off.
TF-Luna-over-water risk: 850nm IR vs still water = dropout/specular; dropout policy = abort UPWARD, never continue. Fallback (user-approved): hover+descend NEXT TO puddle.

## TODO (live; ordered by dependency)
NOW (bench, no flight):
1. Training run1 finishes -> export.py -> scp ONNX to UNO Q -> benchmark fps + spot-check predictions on val images.
2. Camera on UNO Q (cam currently disconnected): lsusb/v4l2 enumeration, focus lock, capture, full image->ONNX loop. Verify enumeration while board powered via 5V pin (not USB-C PSU).
3. Hopper build + salt flow test (user slot 2026-07-26); watch bridging/clumping (Bti granules differ; monsoon humidity).
4. ESP32: flash fake-mode + GPIO4 jumper, verify serial lines; FAKE_VARY_CHANNEL sweep incl ch6.
5. SERIAL5 boot-noise test (ESP32 EN->GND as USB-serial; Pixhawk TX5->ESP32 TX0; watch 57600+115200 across power cycles). If noisy -> TF-Luna to I2C fallback (addr 0x10, pin5 low, Benewake I2C type, verify wiki).
6. TF-Luna bench over water basin, indoor+outdoor, nadir+angle -> decides descend-over vs descend-beside.
7. ESP32->TELEM2 wire-up: OBSTACLE_DISTANCE+DISTANCE_SENSOR ~10Hz from comp195 in QGC MAVLink Inspector; then real sensors (USE_FAKE_SENSORS 0), hand-wave each direction.
8. User: confirm spare RTL switch channel -> RC6_OPTION,4 push. User: VectoBac G sourcing (photo suffices for docs). User: Simple Mode reconfig if wanted.
9. Phone nadir photos from height + negatives (shadows/tarps/wet ground/rooftops/plastic) -> dataset v2.
SOFTWARE (parallel, laptop):
10. SITL install; rehearse survey mission + guided.
11. UNO Q mission code: pymavlink guided cmds, descent-abort (DISTANCE_SENSOR, dropout=abort up), servo drop state machine, MAV_CMD_SET_MESSAGE_INTERVAL for DISTANCE_SENSOR+GLOBAL_POSITION_INT on SERIAL4, compid 191. Test against SITL.
12. STM32 Bridge byte-shovel sketch + latency measurement.
13. Base-station heatmap/report mode. 14. Predictive-model stub for judges.
ASSEMBLY (in progress) then CALIBRATIONS:
15. Finish frame: ESCs soldered to PCB plate + zip-tied, motors rigid, FC foam, GPS mast, service loops, power module install + V/I calibration vs multimeter, XY-3606 to 5.00V, SERIAL4/5 split cable build, OLED splices soldered, buzzer, TF-Luna + camera mounts (weigh stripped B525), ESP32+mux+7 sensors (FOV check), hopper mount.
16. Calibrations: accel+level, compass, radio, ESC cal, motor order/direction test, RC failsafe bench test (TX off, props off), battery calibration. Re-dump params after (pull_params.py).
17. Conformal coat LAST (mask baro/USB/connectors).
FLIGHT GATES (strict order):
18. Bench GPS check -> unloaded hover VibeZ<15 (THE gate) -> notch FFT switch + log verify -> Loiter -> loaded hover w/ ballast (relearn MOT_THST_HOVER, VibeZ recheck) -> payload integration -> mission flights -> demo footage (freeze ~Aug 10).
DOCUMENTATION (start now, judged category):
19. Write-up, crash post-mortems, dataset citations, demo video storyboard, compliance narrative, Q&A prep, training/bench evidence photos.
DESCOPE LADDER if time runs out: (a) Loiter + onboard detect + drop = minimum judgeable; (b) +base station report; (c) +obstacle array PoC; (d) +guided auto descent.

## DECISION LOG (append here)
2026-07-19 servo cap skipped; salvage only if servo misbehaves.
2026-07-20 handoff v3; ESP32 fw updated 6@60deg+up; sensors 7x plan.
2026-07-24 serial option B (split SERIAL4/5 cable) chosen over TF-Luna I2C; hexa X confirmed.
2026-07-25 project moved to /media/sleuther/Stuff/Robu AI Challenge; all parts arrived; frame assembly started; ESP32 powered from servo BEC (XY-3606); charger=HTRC B6 V2; fw compiles clean both modes; fake-mode GPIO4 TX guard added; pixhawk_full_setup.param created; local YOLO training adopted over Edge Impulse cloud; standing-water framing adopted; AVOID_MARGIN 1 / AVOID_DIST_MAX 1.5 sized to sensor range; failsafes confirmed; FENCE deferred.
2026-07-25 firmware upgraded to Pixhawk1-bdshot 4.7.0 stable; bdshot RPM notch found impossible for hexa (AUX5-6 NODMA); adopted in-flight FFT notch; motors stay MAIN.
2026-07-25 board factory-reset by user (no prereset backup; switches/modes reconstructed from screenshot+handoff). QGC bulk-load found to silently drop writes -> pymavlink push_params.py with per-write ack became the standard; all 55 params verified; RNGFND1_GNDCLR rename found. Full-param backup workflow live (pull_params.py).
2026-07-25 datasets: mosquito v4 excluded (no puddle class), v1 adopted (2.9k-image set, 5069 imgs incl splits, CC BY 4.0); merged 11725/3069; run1 started, restarted once to include mosquito (8min lost). Private git repo created (<github-account>/monsoonready-drone), commit+push-every-change rule adopted; generated param files + datasets gitignored, pixhawk_full_setup.param tracked.
2026-07-25 PROJECT_STATE restructured: done-work pruned into this log, TODO section now reflects only remaining work.
