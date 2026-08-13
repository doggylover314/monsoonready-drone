#!/usr/bin/env python3
"""Which UART on the UNO Q's LINUX side, if any, can hear the Pixhawk?

    python3 ~/find_pixhawk_uart.py

RUN THIS BEFORE WRITING ANY STM32 SKETCH. The recorded plan for TODO 7 is
Linux -> Bridge RPC -> STM32 sketch -> Serial1 (D0/D1) -> Pixhawk SERIAL5,
which needs a byte-shovel sketch, a working Bridge, and Serial1 to be unclaimed
by the router. That is three unknowns stacked on each other.

But the Linux side exposes /dev/ttyS0..S3, and if the Pixhawk's SERIAL5 TX
happens to land on one of them, ALL THREE UNKNOWNS VANISH: pymavlink opens the
port directly and the STM32 is not involved at all. That would be a strictly
simpler, strictly more reliable architecture than the recorded plan, and it
costs one read-only test to find out.

The recorded claim is "UNO Q D0/D1 = STM32 USART1 (PB7/PB6)", which if true
means no ttyS can see the Pixhawk and the sketch route is the only one. That
claim has never been verified against the board. This settles it either way.

STRICTLY READ-ONLY. It opens each port, listens, and closes. It never writes a
byte, so it cannot contend with the Pixhawk's TX5 and cannot corrupt anything
that may be using a port for something else.

WHAT THE RESULT MEANS:
  MAVLINK on some port -> use it directly. Skip the sketch entirely.
  BYTES but no MAVLink -> something is talking at another baud, or that port
                          is the Linux<->STM32 Bridge. Not the Pixhawk.
  SILENCE everywhere   -> the recorded claim holds. The Pixhawk is only
                          reachable through the STM32, so the sketch route is
                          the real one.
"""

import glob
import sys
import time

BAUDS = [115200, 57600]          # SERIAL5_BAUD is 115200; 57600 as a courtesy
LISTEN_S = 3.0
MAGIC = (0xFD, 0xFE)             # MAVLink v2, v1

try:
    import serial                # pyserial
except ImportError:
    sys.exit("pyserial is missing on this board. Install it into whichever\n"
             "python you are running, e.g.  ~/venv/bin/pip install pyserial\n"
             "and then run this with  ~/venv/bin/python")


def listen(port, baud):
    """Return (n_bytes, n_magic, sample) after LISTEN_S on this port."""
    try:
        with serial.Serial(port, baud, timeout=0.2) as s:
            # Some ports need a moment before they produce anything sane.
            time.sleep(0.2)
            s.reset_input_buffer()
            data = bytearray()
            end = time.time() + LISTEN_S
            while time.time() < end:
                chunk = s.read(256)
                if chunk:
                    data.extend(chunk)
            return len(data), sum(data.count(m) for m in MAGIC), bytes(data[:16])
    except (OSError, serial.SerialException) as e:
        return None, None, str(e)


def main():
    ports = sorted(glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyUSB*')
                   + glob.glob('/dev/ttyACM*'))
    if not ports:
        sys.exit("no serial ports at all on this board")

    print(f"listening {LISTEN_S:.0f}s on each of {len(ports)} port(s) "
          f"at {BAUDS} baud. Read-only, nothing is transmitted.\n")
    hits = []
    unopenable = 0
    for port in ports:
        for baud in BAUDS:
            n, magic, sample = listen(port, baud)
            if n is None:
                unopenable += 1
                print(f"  {port:<16} {baud:>6}  cannot open: {sample}")
                continue
            note = ''
            if magic:
                # A framing byte can occur by chance in random data, so report
                # the count and let the density speak rather than calling a
                # single 0xFD a pass.
                note = f"  <-- {magic} MAVLink framing byte(s)"
                hits.append((port, baud, n, magic))
            print(f"  {port:<16} {baud:>6}  {n:6d} bytes  "
                  f"{sample.hex(' ') if n else ''}{note}")

    print()
    # A port that REFUSED TO OPEN proves nothing about MAVLink, and conflating
    # the two is how this script lied on 2026-08-13: all four ttyS returned
    # "Input/output error" on configure and it reported "NO MAVLINK ... that
    # CONFIRMS the recorded claim", which was a conclusion drawn from a failure
    # mode it had never distinguished. Silence is evidence; an unopenable port
    # is the absence of evidence.
    if unopenable and not hits:
        print(f"INCONCLUSIVE. {unopenable} of {len(ports) * len(BAUDS)} "
              f"attempts could not even OPEN the port.")
        print("  These ttyS nodes exist in /dev but are not usable UARTs from "
              "userspace: no hardware behind them, or not muxed to pins. That "
              "says NOTHING about whether the Pixhawk is reachable, so it "
              "neither confirms nor refutes the D0/D1-belongs-to-the-STM32 "
              "claim.")
        print("  It DOES close this shortcut: Linux cannot use these ports, so "
              "the STM32 sketch route is the one that remains.")
        return
    if hits:
        best = max(hits, key=lambda h: h[3])
        print("MAVLINK FOUND. The Linux side can reach the Pixhawk DIRECTLY:")
        for port, baud, n, magic in hits:
            print(f"  {port} at {baud}: {magic} framing bytes in {n}")
        print(f"\nUse {best[0]} at {best[1]}. This makes the STM32 sketch, the "
              f"byte-shovel and the whole Bridge question UNNECESSARY: point "
              f"pymavlink at that device and the companion link is done.\n"
              f"Confirm with:\n"
              f"  ~/venv/bin/python -c \"from pymavlink import mavutil; "
              f"m=mavutil.mavlink_connection('{best[0]}', baud={best[1]}); "
              f"print(m.recv_match(type='HEARTBEAT', blocking=True, "
              f"timeout=10))\"")
    else:
        print("NO MAVLINK on any Linux-side UART, and every port OPENED fine.")
        print("  That CONFIRMS the recorded claim that D0/D1 belong to the "
              "STM32 (USART1), not to Linux, so the Pixhawk is only reachable "
              "through a sketch on the MCU. The byte-shovel route is the real "
              "one and the next step is building/uploading the probe sketch.")
        print("  Before accepting that: check the Pixhawk is POWERED and that "
              "SERIAL5_PROTOCOL=2 / SERIAL5_BAUD=115200, because an "
              "unconfigured SERIAL5 also produces exactly this silence.")


if __name__ == '__main__':
    main()
