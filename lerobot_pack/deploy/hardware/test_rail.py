#!/usr/bin/env python3
"""Test the rail ALONE before connecting it to anything else.
   python hardware/test_rail.py --port /dev/ttyUSB0 status
   python hardware/test_rail.py --port /dev/ttyUSB0 pct 50        # move to 50 % of travel
   python hardware/test_rail.py --port /dev/ttyUSB0 mm 120
   python hardware/test_rail.py --port /dev/ttyUSB0 cycle         # 0 -> 50 -> 100 -> 50 -> 0 %, prints the position each time
Opening the port resets the ESP32; it then WAITS and does not move. `home`, `pct`, `mm` and `cycle` make it home first
(full-length travel to both switches, 1-2 min) and then move. Keep a hand on the power switch / e-stop."""
import argparse, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from panel_core import SerialRail

ap = argparse.ArgumentParser()
ap.add_argument("--port", required=True)
ap.add_argument("cmd", choices=["status", "pct", "mm", "cycle", "home"])
ap.add_argument("value", nargs="?", type=float)
a = ap.parse_args()
cfg = {"driver": "esp32", "port": a.port, "unit": "mm" if a.cmd == "mm" else "pct", "positions": {"x": a.value or 0},
       "move_timeout_s": 90, "boot_timeout_s": 15, "home_timeout_s": 180}
r = SerialRail(cfg, print)
r.connect()
if a.cmd == "status":
    print("position mm:", r.position())
elif a.cmd == "home":
    r.home()
elif a.cmd in ("pct", "mm"):
    r.move_to("x"); print("arrived, position mm:", r.position())
else:
    for p in (0, 50, 100, 50, 0):
        r.positions["x"] = p; t = time.time(); r.move_to("x")
        print(f"{p:>3} %  -> {r.position():7.2f} mm  ({time.time()-t:.1f}s)")
r.close()
