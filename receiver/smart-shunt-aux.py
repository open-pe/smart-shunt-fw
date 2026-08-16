#!/usr/bin/env python3
"""Flip the smart shunt's aux switch (GPIO 10) by editing the control file.

This tool touches no BLE. Only one central may hold the link, so a tool that connected itself would
be unusable whenever smart-shunt-ble-client.py is running -- instead the collector watches this file
and pushes any change over the connection it already has. That also means:

  * with no collector running, nothing happens until one starts (then it reconciles);
  * the device's own NVS is the source of truth for the pin, not this file. `status` reports what
    was *requested* here, which is not necessarily what the pin is doing. For that, read the
    `aux` field on the smart_shunt_meta point, or `aux` on the device's serial console.

Usage:
    ./smart-shunt-aux.py on|off|toggle|status [--device ADDR|NAME] [--file PATH]

`--device` defaults to "*", i.e. every shunt without an entry of its own.
"""
import argparse
import json
import os
import sys
import tempfile

DEFAULT_FILE = os.environ.get(
    'SMART_SHUNT_AUX_FILE',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'aux-state.json'))


def load(path):
    try:
        with open(path) as f:
            data = json.load(f)
    except FileNotFoundError:
        return {}
    if not isinstance(data, dict):
        raise SystemExit(f"{path}: expected a JSON object, got {type(data).__name__}")
    return data


def save(path, data):
    # Atomic: the collector polls this file, and a half-written one would read as corrupt. It would
    # correctly refuse to act on that -- but there is no reason to make it decide.
    d = os.path.dirname(os.path.abspath(path)) or '.'
    fd, tmp = tempfile.mkstemp(dir=d, prefix='.aux-state-', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(data, f, indent=2, sort_keys=True)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def main():
    ap = argparse.ArgumentParser(description="Set the smart shunt aux switch via the control file")
    ap.add_argument("action", choices=["on", "off", "toggle", "status"])
    ap.add_argument("-d", "--device", default="*",
                    help='BLE address or name; default "*" (every shunt without its own entry)')
    ap.add_argument("-f", "--file", default=DEFAULT_FILE, help="control file path")
    args = ap.parse_args()

    data = load(args.file)

    if args.action == "status":
        if not data:
            print(f"{args.file}: no entries (no switch is being commanded)")
            return 0
        print(f"{args.file}:")
        for k, v in sorted(data.items()):
            mark = "" if isinstance(v, bool) else "   <- not true/false, the collector will ignore it"
            print(f"  {k:24} {v}{mark}")
        return 0

    if args.action == "toggle":
        cur = data.get(args.device)
        if not isinstance(cur, bool):
            print(f"cannot toggle {args.device!r}: no current value in {args.file}. "
                  f"Set it explicitly with `on` or `off` first.", file=sys.stderr)
            return 1
        want = not cur
    else:
        want = args.action == "on"

    data[args.device] = want
    save(args.file, data)
    print(f"{args.device} -> {'ON' if want else 'off'}  ({args.file})")
    print("the collector will push this to the device on its next pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
