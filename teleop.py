#!/usr/bin/env python3
"""Keyboard teleop for the arm's linear axes (front pair, rear, piston, bogie).

    python3 teleop.py            # real hardware (GRBL on the Mega)
    python3 teleop.py --sim      # no hardware: simulated motion, same keys and limits

Keys (see config.py):  arrows height | I/K pitch | W/S front | E/D rear | U/O piston
                       J/L bogie | 1/2/3 gear | Space stop | hold Z zero | C reset drives | Esc quit
                       (--sim only: F12 simulates a drive stall)

How motion works: while motion keys are held, short jog increments (JOG_DT of motion each)
are streamed to GRBL, with at most MAX_QUEUED_BLOCKS queued. When the held-key set changes
(including release), a jog cancel is sent: GRBL decelerates and flushes, and once it reports
Idle the new motion (if any) starts. Every increment is clipped against travel and pitch
limits relative to the zero pose.

Drive alarms: a stalled drive's ALM output hits GRBL's abort input, which stops all motion.
The controller then resets the drives by itself (MF frees the motors briefly, which clears
the drive, then they are energised again), drops the zero and waits for every motion key
to be released. C does the same reset by hand.
"""
from __future__ import annotations

import argparse
import math
import shutil
import signal
import sys
import threading
import time

import config as C
from limits import clip_increment, pitch_deg

N = 4
ZERO_VEC = (0, 0, 0, 0)


def recover_drives(link, log=print):
    """Reset the drives: MF attempts from config, then the optional power-cycle relay.

    Blocks for one to a few seconds. Returns (ok, message).
    """
    ok, msg = False, "no reset attempted"
    for free_s in C.DRIVE_FREE_S:
        ok, msg = link.reset_drives(free_s, C.WAKE_AXIS, C.WAKE_MM, C.DRIVE_SETTLE_S)
        if ok:
            return ok, msg
        log(f"drive reset with MF held {free_s:g} s failed: {msg}")
    if C.POWER_CYCLE_OFF_CMD:
        log("power-cycling the drives through the relay")
        ok, msg = link.power_cycle_drives(C.POWER_CYCLE_OFF_CMD, C.POWER_CYCLE_ON_CMD,
                                          C.POWER_CYCLE_OFF_S, C.POWER_CYCLE_BOOT_S,
                                          C.WAKE_AXIS, C.WAKE_MM, C.DRIVE_SETTLE_S)
    return ok, msg


class Controller:
    """All teleop logic. No I/O of its own: fed with key events/held keys, talks to a link.

    Modes: IDLE -> JOG -> CANCEL -> IDLE for motion. Any drive alarm or GRBL alarm goes to
    RESET (drives being reset in a worker thread), then IDLE, or FAULT if the reset failed
    or auto reset is used up. FAULT waits for C.
    """

    def __init__(self, link, log=print):
        self.link = link
        self.log = log
        self.gear = C.DEFAULT_GEAR
        self.zero = None                  # MPos at the zero pose, or None
        self.mode = "IDLE"
        self.vec = ZERO_VEC
        self.target = None                # relative position where queued motion ends
        self.next_send = 0.0
        self.cancel_t = 0.0
        self.cancel_warned = False
        self.stop_latch = False
        self.quit = False
        self.limit_hits = set()
        self.z_down_t = None
        self.reset_seen = link.reset_count
        self.error_seen = link.error_count
        self.fault_msg = ""
        self._auto_resets = []            # times of recent automatic resets
        self._worker = None
        self._result = None               # (ok, msg) from the reset worker

    # ------------------------------------------------------------- helpers
    @property
    def zeroed(self) -> bool:
        return self.zero is not None

    def rel(self, mpos):
        if self.zero is None:
            return tuple(mpos)
        return tuple(m - z for m, z in zip(mpos, self.zero))

    @staticmethod
    def motion_vector(held) -> tuple:
        v = [0, 0, 0, 0]
        for k in held:
            kv = C.MOTION_KEYS.get(k)
            if kv:
                v = [a + b for a, b in zip(v, kv)]
        return tuple(max(-1, min(1, a)) for a in v)

    def active_gear(self) -> int:
        return self.gear if self.zeroed else C.UNZEROED_GEAR

    def velocity(self, vec):
        frac = C.GEARS[self.active_gear()]
        return [vec[i] * frac * C.VMAX[i] for i in range(N)]

    def drop_zero(self, why: str):
        if self.zero is not None:
            self.zero = None
            self.log(f"ZERO LOST ({why}): limits off, fine gear only. Re-zero with Z.")

    def fault_reason(self, st):
        if st.drive_alarm:
            return "drive alarm (ALM): a drive stalled or faulted"
        if st.state.startswith("Alarm"):
            alarm = getattr(self.link, "last_alarm", None)
            return f"GRBL alarm ({alarm[1]})" if alarm else "GRBL alarm"
        if st.state.startswith("Sleep"):
            return "GRBL asleep (motors free)"
        return None

    # ------------------------------------------------------------- events
    def on_event(self, kind: str, key: str, now: float):
        if kind == "disconnected":
            self.log(f"keyboard disconnected: {key}")
            return
        if kind == "connected":
            self.log(f"keyboard: {key}")
            return
        cmd = C.COMMAND_KEYS.get(key)
        if cmd is None:
            return
        if kind == "up":
            if cmd == "zero":
                self.z_down_t = None
            return
        if cmd.startswith("gear"):
            self.gear = int(cmd[-1])
            if not self.zeroed and self.gear != C.UNZEROED_GEAR:
                self.log(f"gear {self.gear} set; fine gear is forced until zeroed")
        elif cmd == "stop":
            self.stop_latch = True
        elif cmd == "quit":
            self.quit = True
        elif cmd == "zero":
            self.z_down_t = now
        elif cmd == "clear_alarm":
            if self.mode == "RESET":
                self.log("drive reset already running")
            else:
                self._auto_resets = []
                self.stop_latch = True
                self.drop_zero("drive reset")
                self._start_reset("C pressed")
        elif cmd == "sim_fault":
            inject = getattr(self.link, "inject_fault", None)
            if inject:
                inject()
            else:
                self.log("F12 stall simulation only works with --sim")

    # ------------------------------------------------------------- drive reset
    def _start_reset(self, why: str):
        self.mode = "RESET"
        self._result = None
        self.log(f"resetting drives ({why}); motors go limp for a moment")
        self._worker = threading.Thread(target=self._reset_worker, daemon=True)
        self._worker.start()

    def _reset_worker(self):
        try:
            self._result = recover_drives(self.link, self.log)
        except Exception as e:                   # never leave the controller stuck in RESET
            self._result = (False, f"reset error: {e}")

    def _poll_reset(self):
        if self._result is None:
            return
        ok, msg = self._result
        self._result, self._worker = None, None
        self.reset_seen = self.link.reset_count  # the reset's own GRBL resets/errors are expected
        self.error_seen = self.link.error_count
        self.limit_hits = set()
        if ok:
            self.mode = "IDLE"
            self.fault_msg = ""
            self.log(f"drives reset OK ({msg}). Release all keys to move again.")
        else:
            self.mode = "FAULT"
            self.fault_msg = msg
            self.log(f"DRIVE RESET FAILED: {msg}. If a drive LED is still red: cut drive power "
                     "(E-stop), restore it, then press C.")

    def _on_fault(self, why: str, now: float):
        self.log(f"FAULT: {why}")
        self.drop_zero("fault")
        self.stop_latch = True                   # no motion until every motion key is released
        self.z_down_t = None
        self.limit_hits = set()
        self.fault_msg = why
        self._auto_resets = [t for t in self._auto_resets if now - t < C.AUTO_RESET_WINDOW_S]
        if not C.AUTO_RESET:
            self.mode = "FAULT"
            self.log("auto reset is off: press C to reset the drives")
        elif len(self._auto_resets) >= C.AUTO_RESET_MAX:
            self.mode = "FAULT"
            self.log(f"{len(self._auto_resets)} auto resets in {C.AUTO_RESET_WINDOW_S:.0f} s: "
                     "check for a jam or overload, then press C")
        else:
            self._auto_resets.append(now)
            self._start_reset("auto")

    def wait_reset(self, timeout: float):
        """Let a running drive reset finish (so we never quit with the motors left free)."""
        w = self._worker
        if w is not None:
            w.join(timeout)

    # ------------------------------------------------------------- main step
    def update(self, held, now: float):
        link, st = self.link, self.link.status

        if self.mode == "RESET":
            self._poll_reset()
            return

        if link.reset_count != self.reset_seen:          # Mega reset: zero is no longer trusted
            self.reset_seen = link.reset_count
            self.drop_zero("GRBL reset")
            self.stop_latch = True                       # GRBL dropped the jog: release keys first
            if self.mode != "FAULT":
                self.mode = "IDLE"
        if link.error_count != self.error_seen:          # a jog line was rejected
            self.error_seen = link.error_count
            self.log(f"GRBL rejected {link.last_error[0]!r}: {link.last_error[1]}")
            if self.mode == "JOG":
                self._cancel(now)

        if self.mode == "FAULT":                         # wait for C
            return
        why = self.fault_reason(st)
        if why:
            self._on_fault(why, now)
            return

        # zero: hold Z, only while stopped
        if self.z_down_t is not None and now - self.z_down_t >= C.ZERO_HOLD_S:
            self.z_down_t = None
            if self.mode == "IDLE" and st.state == "Idle":
                self.zero = tuple(st.mpos)
                self.log("ZERO SET at current pose. Limits active.")
            else:
                self.log("zero refused: stop moving first")

        vec = self.motion_vector(held)
        if self.stop_latch:
            if vec == ZERO_VEC:
                self.stop_latch = False
            vec = ZERO_VEC

        fresh = now - st.t < 0.3
        if self.mode == "IDLE":
            if vec != ZERO_VEC and st.state == "Idle" and fresh:
                self.vec = vec
                self.target = list(self.rel(st.mpos))
                self.mode = "JOG"
                self.next_send = now
                for _ in range(C.PREFILL):
                    self._send_increment(now, prefill=True)
        elif self.mode == "JOG":
            if vec != self.vec or st.state not in ("Idle", "Jog"):
                self._cancel(now)
            elif now >= self.next_send:
                self._send_increment(now)
        elif self.mode == "CANCEL":
            if st.state == "Idle" and st.t > self.cancel_t + 0.02:
                self.mode = "IDLE"
                self.limit_hits = set()
            elif now - self.cancel_t > 3.0 and not self.cancel_warned:
                self.cancel_warned = True
                self.log(f"still waiting for GRBL to stop (state {st.state})")

    def _cancel(self, now: float):
        self.link.jog_cancel()
        self.mode = "CANCEL"
        self.cancel_t = now
        self.cancel_warned = False

    def _send_increment(self, now: float, prefill: bool = False):
        st = self.link.status
        if not prefill and self.link.planner_total and st.planner_free is not None:
            if self.link.planner_total - st.planner_free >= C.MAX_QUEUED_BLOCKS:
                self.next_send = now + 0.01               # GRBL is behind: wait
                return
        d = [v * C.JOG_DT for v in self.velocity(self.vec)]
        if self.zeroed:
            d, self.limit_hits = clip_increment(self.target, d)
        if max(abs(x) for x in d) < 0.001:               # pinned at a limit
            self.next_send = now + C.JOG_DT
            return
        speed = math.sqrt(sum(x * x for x in d)) / C.JOG_DT    # mm/s along the clipped move
        self.link.jog(d, speed * 60.0)
        self.target = [t + x for t, x in zip(self.target, d)]
        self.next_send = max(self.next_send + C.JOG_DT, now) if not prefill else now + C.JOG_DT

    # ------------------------------------------------------------- display
    def status_line(self) -> str:
        st = self.link.status
        p = self.rel(st.mpos)
        pos = " ".join(f"{n[0].upper()}{v:8.2f}" for n, v in zip(C.AXIS_NAMES, p))
        if self.zeroed:
            tag = f"pitch {pitch_deg(p[C.FRONT], p[C.REAR]):+6.1f}deg"
        else:
            tag = "UNZEROED: limits OFF, fine only, hold Z at zero pose"
        alm = " | DRIVE ALM" if st.drive_alarm else ""
        fault = " | FAULT: press C" if self.mode == "FAULT" else ""
        lim = f" | LIMIT {','.join(sorted(self.limit_hits))}" if self.limit_hits else ""
        stop = " | STOP (release keys)" if self.stop_latch else ""
        return (f"[{self.mode:6}] gear {self.active_gear()} | {st.state:7} | {pos} | "
                f"{tag}{alm}{fault}{lim}{stop}")


# ----------------------------------------------------------------- runner
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=C.PORT)
    ap.add_argument("--baud", type=int, default=C.BAUD)
    ap.add_argument("--keyboard", default=C.KEYBOARD_DEVICE, help="evdev path, default auto-detect")
    ap.add_argument("--no-grab", action="store_true", help="do not grab the keyboard exclusively")
    ap.add_argument("--sim", action="store_true", help="simulated motion, no serial port")
    args = ap.parse_args()

    def log(msg):
        print("\r\033[K" + msg, flush=True)

    from keyboard_in import Keyboard

    if args.sim:
        from sim_link import SimLink
        link = SimLink(letters=C.AXIS_LETTERS, log=log)
    else:
        from grbl_link import GrblLink
        log(f"connecting to GRBL on {args.port} (the Mega resets on connect)...")
        link = GrblLink(args.port, args.baud, C.STATUS_HZ, letters=C.AXIS_LETTERS, log=log,
                        alm_conducts_on_fault=C.ALM_CONDUCTS_ON_FAULT)

    kb = Keyboard(args.keyboard, grab=not args.no_grab, log=log)
    ctl = Controller(link, log=log)

    def on_signal(*_):
        ctl.quit = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    log("teleop running. Esc to quit. Jog to the zero pose, then hold Z for 1 s.")
    period, next_disp = 1.0 / C.LOOP_HZ, 0.0
    try:
        while not ctl.quit:
            now = time.monotonic()
            if link.failed:
                log(f"LINK FAILED: {link.failed}")
                break
            while not kb.events.empty():
                kind, key = kb.events.get_nowait()
                ctl.on_event(kind, key, now)
            ctl.update(kb.held_keys() if kb.connected else frozenset(), now)
            if now >= next_disp:
                next_disp = now + 1.0 / C.DISPLAY_HZ
                width = shutil.get_terminal_size().columns - 1    # a wrapped line can't be redrawn in place
                sys.stdout.write("\r\033[K" + ctl.status_line()[:width])
                sys.stdout.flush()
            time.sleep(period)
    finally:
        try:
            if ctl.mode == "RESET":
                log("waiting for the drive reset to finish...")
                ctl.wait_reset(8.0)
            link.jog_cancel()
            t0 = time.monotonic()
            while link.status.state not in ("Idle", "Alarm", "Sleep") and time.monotonic() - t0 < 2.0:
                time.sleep(0.05)
        except Exception:
            pass
        kb.close()
        link.close()
        print("\nstopped.")


if __name__ == "__main__":
    main()
