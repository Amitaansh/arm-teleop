#!/usr/bin/env python3
"""Apply grbl_settings.txt to GRBL and read the settings back to verify.

    python3 setup_grbl.py                 # apply + verify
    python3 setup_grbl.py --show          # just print build info and current settings
"""
from __future__ import annotations

import argparse
import os

import config as C
from grbl_link import GrblLink

HERE = os.path.dirname(os.path.abspath(__file__))


def load_settings(path):
    wanted = {}
    with open(path) as f:
        for raw in f:
            line = raw.split(";", 1)[0].strip()
            if not line:
                continue
            key, _, val = line.partition("=")
            wanted[key.strip()] = val.strip()
    return wanted


def read_settings(link):
    lines, result = link.query("$$")
    current = {}
    for ln in lines:
        if ln.startswith("$") and "=" in ln:
            k, _, v = ln.partition("=")
            current[k] = v.split()[0]
    return current, result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=C.PORT)
    ap.add_argument("--baud", type=int, default=C.BAUD)
    ap.add_argument("--file", default=os.path.join(HERE, "grbl_settings.txt"))
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    link = GrblLink(args.port, args.baud, letters=C.AXIS_LETTERS)
    try:
        info, _ = link.query("$I")
        print("build info:", *info, sep="\n  ")
        if args.show:
            cur, _ = read_settings(link)
            for k in sorted(cur, key=lambda s: int(s[1:])):
                print(f"{k}={cur[k]}")
            return

        wanted = load_settings(args.file)
        for k, v in wanted.items():
            res = link.send_line_wait(f"{k}={v}")
            print(f"{k}={v:<10} {res}")

        cur, _ = read_settings(link)
        bad = [k for k, v in wanted.items() if k not in cur or abs(float(cur[k]) - float(v)) > 1e-6]
        if bad:
            print("\nMISMATCH:", ", ".join(f"{k} want {wanted[k]} got {cur.get(k, 'missing')}" for k in bad))
            if any(k in ("$103", "$113", "$123") and k not in cur for k in bad):
                print("4th-axis settings missing: the firmware was not built with 4 axes.")
        else:
            print("\nall settings verified.")
    finally:
        link.close()


if __name__ == "__main__":
    main()
