#!/usr/bin/env python3
"""Load test for the 3 vertical rails: front-left (X), front-right (B) and rear (Y) together.

    python3 rail_test.py                         # 20 mm up/down strokes for 30 s at full speed
    python3 rail_test.py --amp 50 --seconds 60   # longer strokes, longer run
    python3 rail_test.py --sim                   # dry run, no hardware

Each stroke goes UP first and then back DOWN to where it started, so it never drives below
the start position. All three move together, so the barrel stays level (pitch 0) and the
crossbar stays square.
Every reversal is a full decel + accel, which is where the drives pull the most current.
Speed is capped by GRBL's $110/$111/$114 (1200 mm/min = 20 mm/s), so it can't over-speed the screws.

Ctrl-C stops immediately (jog cancel). Piston (Z) and bogie (A) are never commanded
(except the 0.05 mm out-and-back wake jog on the bogie that ends a drive reset).
If a rail stalls (drive ALM fires), the test stops, reports it and resets the drives,
so no power cycle is needed. Close miniterm / teleop first: only one program can hold
the serial port.
"""
from __future__ import annotations

import argparse
import math
import time

import config as C
from teleop import recover_drives


class Stalled(RuntimeError):
    pass


def show(st, start, strokes):
    pos = "  ".join(f"{C.AXIS_LABELS[i]} {st.mpos[i] - start[i]:7.2f}" for i in C.RAILS)
    print(f"\r{st.state:8} | {pos} mm from start | strokes {strokes}", end="", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=C.PORT)
    ap.add_argument("--baud", type=int, default=C.BAUD)
    ap.add_argument("--amp", type=float, default=20.0, help="stroke length in mm, up then back down (max 100)")
    ap.add_argument("--seconds", type=float, default=30.0, help="how long to keep stroking")
    ap.add_argument("--rate", type=float, default=1200.0,
                    help="per-axis speed in mm/min (GRBL clamps to $110/$111/$114 anyway)")
    ap.add_argument("--sim", action="store_true", help="no hardware, simulated motion")
    args = ap.parse_args()
    if not 0 < args.amp <= 100:
        ap.error("--amp must be between 0 and 100 mm")

    if args.sim:
        from sim_link import SimLink
        link = SimLink(letters=C.AXIS_LETTERS, log=print)
    else:
        from grbl_link import GrblLink
        print(f"connecting to GRBL on {args.port} (the Mega resets on connect)...")
        link = GrblLink(args.port, args.baud, C.STATUS_HZ, letters=C.AXIS_LETTERS, log=print,
                        alm_conducts_on_fault=C.ALM_CONDUCTS_ON_FAULT, grbl_order=C.GRBL_AXIS_ORDER)

    rails = C.RAILS
    feed = args.rate * math.sqrt(len(rails))  # vector feed so each rail runs at --rate
    start = None
    strokes = 0
    going_up = True                           # next leg to send
    try:
        t0 = time.monotonic()                 # wait for a status report with buffer info
        while not link.planner_total or link.status.planner_free is None:
            if time.monotonic() - t0 > 3.0:
                raise RuntimeError("no status from GRBL (is $10=3 set?)")
            time.sleep(0.05)
        st = link.status
        if st.drive_alarm or st.state.startswith(("Alarm", "Sleep")):
            print(f"GRBL {st.state}{', drive ALM active' if st.drive_alarm else ''}: resetting drives")
            ok, msg = recover_drives(link)
            if not ok:
                raise RuntimeError(f"drive reset failed: {msg}. Power-cycle the drives and retry.")
        start = link.status.mpos
        resets = link.reset_count

        print(f"stroking FL+FR+rear {args.amp:g} mm up/down at {args.rate:g} mm/min per axis "
              f"for {args.seconds:g} s. Ctrl-C to stop.")
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end or not going_up:     # always finish on a DOWN leg
            if link.failed:
                raise RuntimeError(link.failed)
            st = link.status
            if st.drive_alarm or st.state.startswith("Alarm") or link.reset_count != resets:
                raise Stalled("drive ALM fired" if st.drive_alarm else f"GRBL reset / {st.state}")
            queued = link.planner_total - st.planner_free
            if queued < 2:                                   # keep at most ~2 legs queued
                d = args.amp if going_up else -args.amp
                link.jog([d if i in rails else 0.0 for i in range(C.N_AXES)], feed)
                if not going_up:
                    strokes += 1
                going_up = not going_up
                time.sleep(0.1)                              # let the status catch up
            show(st, start, strokes)
            time.sleep(0.05)

        t0 = time.monotonic()                                # let the last leg finish
        while link.status.state != "Idle" and time.monotonic() - t0 < 30:
            st = link.status
            if st.drive_alarm or st.state.startswith("Alarm") or link.reset_count != resets:
                raise Stalled("drive ALM fired" if st.drive_alarm else f"GRBL reset / {st.state}")
            show(st, start, strokes)
            time.sleep(0.05)
        print(f"\ndone: {strokes} strokes, back at start.")
    except Stalled as e:
        print(f"\nSTALL after {strokes} strokes ({e}). The rails stopped mid-stroke.")
        print("resetting drives (motors go limp for a moment)...")
        ok, msg = recover_drives(link)
        print(f"drive reset {'OK' if ok else 'FAILED'}: {msg}")
    except KeyboardInterrupt:
        link.jog_cancel()
        print("\nCtrl-C: jog cancelled (rails stop somewhere between start and start + amp).")
    finally:
        try:
            link.jog_cancel()
            t0 = time.monotonic()
            while link.status.state not in ("Idle", "Alarm") and time.monotonic() - t0 < 2.0:
                time.sleep(0.05)
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    main()
