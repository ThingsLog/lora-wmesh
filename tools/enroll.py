#!/usr/bin/env python3
# W-Mesh enrolment helper — drives the boot console over USB-UART.
#
# The one thing this script does better than a human with a terminal:
# it sends `set time <epoch>` with the CURRENT UTC second, automatically,
# LAST — right before `save`/`boot` — so the RTC error at sealing is
# bounded by serial latency (milliseconds), not by typing speed.
#
# Usage (reset the node right after starting the script):
#   python3 tools/enroll.py /dev/tty.usbserial-XXX \
#       --profile relay --depth 1 --phases 300,600 \
#       --slot 3 --whitelist 31,32,33 --window 12:05 --id 21 --boot
#
#   python3 tools/enroll.py /dev/tty.usbserial-XXX --show   # inspect only
#
# Requires: pip install pyserial
# SPDX-License-Identifier: MIT
import argparse
import sys
import time

BAUD = 115200
BANNER = "enrolment console"


def build_commands(a):
    """Ordered `set` commands from the CLI arguments; time is added by run()."""
    cmds = []
    if a.id is not None:        cmds.append(f"set id {a.id}")
    if a.profile:               cmds.append(f"set profile {a.profile}")
    if a.phases:                cmds.append(f"set phases {a.phases}")
    if a.depth is not None:     cmds.append(f"set depth {a.depth}")
    if a.ports is not None:     cmds.append(f"set ports {a.ports}")
    if a.window:                cmds.append(f"set window {a.window}")
    if a.preroll is not None:   cmds.append(f"set preroll {a.preroll}")
    if a.psub is not None:      cmds.append(f"set psub {a.psub}")
    if a.ppsub is not None:     cmds.append(f"set ppsub {a.ppsub}")
    if a.slot is not None:      cmds.append(f"set slot {a.slot}")
    if a.slotdur is not None:   cmds.append(f"set slotdur {a.slotdur}")
    if a.guard is not None:     cmds.append(f"set guard {a.guard}")
    if a.acq is not None:       cmds.append(f"set acq {a.acq}")
    if a.drift is not None:     cmds.append(f"set drift {a.drift}")
    if a.freerun is not None:   cmds.append(f"set freerun {a.freerun}")
    if a.freq is not None:      cmds.append(f"set freq {a.freq}")
    if a.bfreq is not None:     cmds.append(f"set bfreq {a.bfreq}")
    if a.whitelist:             cmds.append(f"set whitelist {a.whitelist}")
    if a.prefixless is not None: cmds.append(f"set prefixless {a.prefixless}")
    if a.gnssbk is not None:    cmds.append(f"set gnssbk {a.gnssbk}")
    if a.devaddr:               cmds.append(f"set devaddr {a.devaddr}")
    if a.nwkskey:               cmds.append(f"set nwkskey {a.nwkskey}")
    if a.appskey:               cmds.append(f"set appskey {a.appskey}")
    if a.fcnt is not None:      cmds.append(f"set fcnt {a.fcnt}")
    return cmds


def exchange(ser, line, timeout=2.0):
    """Send one line, collect the reply, fail hard on ERR."""
    ser.reset_input_buffer()
    ser.write((line + "\n").encode())
    reply, deadline = [], time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline().decode(errors="replace").rstrip()
        if raw:
            reply.append(raw)
            deadline = time.time() + 0.3   # drain multi-line replies (show/help)
    text = "\n".join(reply)
    print(f"> {line}\n{text}")
    if text.startswith("ERR"):
        sys.exit(f"ABORT: node rejected '{line}' — nothing was saved.")
    return text


def run():
    p = argparse.ArgumentParser(description="W-Mesh enrolment over the boot console")
    p.add_argument("port", help="serial device, e.g. /dev/tty.usbserial-XXX")
    p.add_argument("--show", action="store_true", help="print provisioning and exit")
    p.add_argument("--id", type=int)
    p.add_argument("--profile", choices=["leaf", "relay", "stake"])
    p.add_argument("--depth", type=int)
    p.add_argument("--ports", type=int)
    p.add_argument("--phases", help="comma list, deepest first (count defines H)")
    p.add_argument("--window", help="HH:MM UTC — the BEACON time, not power-on")
    p.add_argument("--preroll", type=int)
    p.add_argument("--psub", type=int, help="relay's beacon sub-slot within its tier")
    p.add_argument("--ppsub", type=int, help="beacon parent's sub-slot to listen at")
    p.add_argument("--slot", type=int)
    p.add_argument("--slotdur", type=int)
    p.add_argument("--guard", type=int)
    p.add_argument("--acq", type=int)
    p.add_argument("--drift", type=int)
    p.add_argument("--freerun", type=int)
    p.add_argument("--freq", type=int)
    p.add_argument("--bfreq", type=int)
    p.add_argument("--whitelist", help="comma list of child ids, or - to clear")
    p.add_argument("--prefixless", type=int, choices=[0, 1])
    p.add_argument("--gnssbk", type=int, choices=[0, 1],
                   help="GNSS backup clock (relay/leaf): powered only on a missed beacon")
    p.add_argument("--devaddr", help="LoRaWAN ABP DevAddr, 8 hex (MSB first); 0 clears")
    p.add_argument("--nwkskey", help="LoRaWAN NwkSKey, 32 hex")
    p.add_argument("--appskey", help="LoRaWAN AppSKey, 32 hex")
    p.add_argument("--fcnt", type=int, help="LoRaWAN FCntUp start (ABP reset)")
    p.add_argument("--no-time", action="store_true",
                   help="skip the automatic `set time` (bench experiments only)")
    p.add_argument("--boot", action="store_true", help="leave the console after save")
    a = p.parse_args()

    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial missing: pip install pyserial")

    ser = serial.Serial(a.port, BAUD, timeout=0.5)
    print(f"Waiting for the boot console on {a.port} — reset the node now...")
    deadline = time.time() + 30
    ser.write(b"\n")                        # the keypress that opens the console
    while time.time() < deadline:
        raw = ser.readline().decode(errors="replace")
        if BANNER in raw.lower():
            break
        ser.write(b"\n")
    else:
        sys.exit("No console banner within 30 s — is the node in reset? wiring?")
    print("Console is up.")

    if a.show:
        exchange(ser, "show")
        return

    for cmd in build_commands(a):
        exchange(ser, cmd)

    if not a.no_time:
        # LAST, right before save: bound the sealing-time RTC error to ms.
        exchange(ser, f"set time {int(time.time())}")

    exchange(ser, "show")
    text = exchange(ser, "save")
    if "OK" not in text:
        sys.exit("ABORT: save did not confirm.")
    if a.boot:
        exchange(ser, "boot")
        print("Node is running its schedule. Seal it.")
    else:
        print("Saved. Send `boot` (or reset without a keypress) when ready.")


if __name__ == "__main__":
    run()
