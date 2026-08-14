Write the running-order card (bottom of this file) and tape it somewhere the narrator can read it.
Push params FIRST, then gate check — PROPS OFF, hopper EMPTY: ./python tools/parameters.py push (delivers SERVO9 MIN 500 / MAX 1800 / TRIM 560 and PRX1_TYPE=2), then ./python tools/wiring_check.py --wiggle and watch AND listen: closed 560, open 1760, full throw, and NO buzzing at closed (buzzing = servo stalled against the end stop; back the closed value off to ~580-600 and tell the assistant). The wiggle ends on a close, which shuts the gate that servo_jog left open. Your eyes are the test.
Power-cycle the aircraft after the push and confirm the gate SITS CLOSED at boot, before arming, with nothing commanding it. That is SERVO9_TRIM doing its job; if it boots open anyway, say so.

Aircraft:
Transmitter (charged) + 1 set spare AAs

Macbook + Charger
Linux laptop + charger (primary field machine)

SiK telemetry radio (in the MacBook bag)
Known-good DATA USB cable for the Pixhawk (not a charge-only one)
SD card reader

The dark tray
A water can to carry pump water to the tray at the plot
Filming:

Vivo X300 FE: charged, storage cleared for a 10+ min take
Printed/written shooting script (see VIDEO_SCRIPT.md) — one copy per person
Power bank
Tripod OPTIONAL: handheld is fine for the walking take; a stand only helps the two screen close-ups if hands get tired
Repairs and safety:

Cloth sheets (landing pad; weigh the corners with stones or the dust cloud problem comes back with a flapping sheet on top)

Torches (attempts can run late; farm has first aid already)
Human water + snacks + mosquito repellent
At home, NOT in the car:

