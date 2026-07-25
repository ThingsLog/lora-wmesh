#!/usr/bin/env python3
# W-Mesh hardware-in-the-loop smoke test — exercises the boot console of a
# REAL node over USB serial: command round-trips, validation rejects, flash
# save, and (optionally, with a manual reset) persistence across power-up.
#
# Run it on a bench/fresh node BEFORE field provisioning: the test leaves the
# node provisioned as a bare-leaf bench config with the id you pass. For the
# three-node XIAO bench fleet:
#   python3 tools/hiltest.py /dev/cu.usbmodemXXX --id 51
#   python3 tools/hiltest.py /dev/cu.usbmodemYYY --id 52 --reset-check
#
# Reset the node right after starting the script (same as enroll.py).
# Requires: pip install pyserial
# SPDX-License-Identifier: MIT
import argparse
import sys
import time

BAUD = 115200
BANNER = "enrolment console"

passed = 0
failed = []


def check(cond, what):
    global passed
    if cond:
        passed += 1
        print(f"  PASS  {what}")
    else:
        failed.append(what)
        print(f"  FAIL  {what}")


def exchange(ser, line, timeout=2.0):
    """Send one line, return the reply text (local echo stripped)."""
    ser.reset_input_buffer()
    ser.write((line + "\n").encode())
    reply, deadline = [], time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline().decode(errors="replace").rstrip()
        if raw:
            reply.append(raw)
            deadline = time.time() + 0.3
    if reply and reply[0].strip() == line.strip():
        reply = reply[1:]                     # drop the console's echo
    return "\n".join(reply)


def wait_console(ser, hint):
    """Catch the boot banner, or probe with `help` if the console is already
    up from an earlier session (the banner prints only once)."""
    print(f"Waiting for the boot console — {hint}...")
    deadline = time.time() + 30
    ser.write(b"\n")
    while time.time() < deadline:
        raw = ser.readline().decode(errors="replace")
        if BANNER in raw.lower() or "save | boot" in raw:
            print("Console is up.")
            return True
        ser.write(b"\nhelp\n")
    return False


def open_port(serial_mod, port):
    deadline = time.time() + 20
    while time.time() < deadline:
        try:
            return serial_mod.Serial(port, BAUD, timeout=0.5)
        except (OSError, serial_mod.SerialException):
            time.sleep(0.5)      # USB CDC re-enumerating after a reset
    sys.exit(f"Cannot open {port}")


def run():
    p = argparse.ArgumentParser(description="W-Mesh boot-console smoke test")
    p.add_argument("port", help="serial device, e.g. /dev/cu.usbmodem101")
    p.add_argument("--id", type=int, default=250,
                   help="node id the test leaves provisioned (default 250)")
    p.add_argument("--reset-check", action="store_true",
                   help="verify flash persistence across a manual reset")
    p.add_argument("--boot", action="store_true",
                   help="leave the console (start the schedule) when done")
    a = p.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("pyserial missing: pip install pyserial")

    ser = open_port(serial, a.port)
    if not wait_console(ser, "reset the node now"):
        sys.exit("No console banner within 30 s.")

    # --- command round-trips ------------------------------------------------
    check("show" in exchange(ser, "help"), "help lists commands")
    check("id" in exchange(ser, "show"), "show prints provisioning")
    check(exchange(ser, f"set id {a.id}").startswith("OK"), "set id")
    check(exchange(ser, "set profile leaf").startswith("OK"), "set profile leaf")
    check(exchange(ser, "set ports 1").startswith("OK"), "set ports 1 (battery only)")
    check(exchange(ser, "set phases 900").startswith("OK"), "set phases")
    check(exchange(ser, "set depth 1").startswith("OK"), "set depth")
    check(exchange(ser, "set slot 2").startswith("OK"), "set slot")

    # --- the console rejects what it must ------------------------------------
    check(exchange(ser, "set window 25:00").startswith("ERR"),
          "rejects window 25:00")
    check(exchange(ser, "set nonsense 1").startswith("ERR"),
          "rejects unknown key")
    check(exchange(ser, "frobnicate").startswith("ERR"),
          "rejects unknown command")
    # validation runs at save: an out-of-range field must block the write
    check(exchange(ser, "set ports 9").startswith("OK"), "set accepts raw value")
    check(exchange(ser, "save").startswith("ERR"), "save rejects ports 9")
    check(exchange(ser, "set ports 1").startswith("OK"), "restore ports 1")

    # --- time, save, verify ---------------------------------------------------
    check(exchange(ser, f"set time {int(time.time())}").startswith("OK"),
          "set time (epoch)")
    check("OK" in exchange(ser, "save"), "save writes the flash page")
    shown = exchange(ser, "show")
    check(f"id {a.id}" in shown, "saved id visible in show")
    check("profile leaf" in shown, "saved profile visible in show")
    check("ports 1" in shown, "saved ports visible in show")

    # --- persistence across reset (manual) ------------------------------------
    if a.reset_check:
        print("\nPress the RESET button on the node once, now.")
        ser.close()
        time.sleep(2.0)
        ser = open_port(serial, a.port)
        if not wait_console(ser, "waiting for the fresh boot"):
            check(False, "console after reset")
        else:
            shown = exchange(ser, "show")
            check(f"id {a.id}" in shown, "config persisted across reset")

    if a.boot:
        exchange(ser, "boot")
        print("Node left running its schedule.")

    print(f"\n{passed} passed, {len(failed)} failed")
    if failed:
        for f in failed:
            print(f"  - {f}")
        sys.exit(1)


if __name__ == "__main__":
    run()
