#!/usr/bin/env python3
"""Teleop for the arm's linear axes: front-left, front-right, rear rails, piston, bogie.

    python3 teleop.py                  # real hardware (GRBL on the Mega)
    python3 teleop.py --home           # same, and home everything as soon as GRBL is ready
    python3 teleop.py --sim            # no hardware: simulated motion, same keys and limits
    python3 teleop.py --listen 0.0.0.0 # also take keys from other machines (C2 laptop)

Keys come from a keyboard plugged into the Jetson and/or from remote clients: a web page
(http://localhost:8080 on the Jetson's desktop through AnyDesk, or http://<jetson>:8080 from
the laptop with --listen 0.0.0.0; a gamepad works there too), or UDP from a joystick / GUI
bridge. See inputs.py. The full key menu is on screen.

How motion works: while motion keys are held, short jog increments (JOG_DT of motion each)
are streamed to GRBL, with at most MAX_QUEUED_BLOCKS queued. When the held-key set changes
(including release), a jog cancel is sent: GRBL decelerates and flushes, and once it reports
Idle the new motion (if any) starts. Every increment is clipped against travel, front-pair
squareness (rack) and pitch limits, for the axes that have a zero.

Drive alarms: a stalled drive's ALM output hits GRBL's abort input, which stops all motion.
The controller then resets the drives by itself (MF frees the motors briefly, which clears
the drive, then they are energised again), drops the zero and waits for every motion key
to be released. C does the same reset by hand.

Homing (hold H, or F1/F2/F3 for rails/piston/bogie only): every motor is driven into its
hard stops until its drive stalls; that sets its zero and measures its travel. See homing.py.
"""
from __future__ import annotations

import argparse
import collections
import math
import os
import shutil
import signal
import sys
import threading
import time

import config as C
from homing import PARTS, Homer, HomeResult, HomingAborted, HomingError, check_config
from limits import clip_increment, front_height, pitch_deg

N = C.N_AXES
FL, FR, REAR, PISTON, BOGIE = C.FL, C.FR, C.REAR, C.PISTON, C.BOGIE
ZERO_VEC = (0,) * N
HOLD_S = {"zero": C.ZERO_HOLD_S}


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


def hold_time(cmd: str) -> float:
    if cmd.startswith("home:"):
        return C.HOME_HOLD_S
    return HOLD_S.get(cmd, 0.0)


class Controller:
    """All teleop logic. No I/O of its own: fed with key events/held keys, talks to a link.

    Modes: IDLE -> JOG -> CANCEL -> IDLE for motion. Any drive alarm or GRBL alarm goes to
    RESET (drives being reset in a worker thread), then IDLE, or FAULT if the reset failed
    or auto reset is used up. FAULT waits for C. HOME runs a homing job in a worker thread
    (it handles its own stalls), then IDLE, or FAULT if a drive reset failed.
    """

    def __init__(self, link, log=print):
        self.link = link
        self.log = log
        self.gear = C.DEFAULT_GEAR
        self.zero = [None] * N            # per axis: MPos at its zero, or None
        self.travel = list(C.TRAVEL)      # per axis; homing replaces it with the measured stops
        self.home_pending = None          # job to start as soon as GRBL is Idle (--home, keys)
        self.holds = {}                   # hold-to-act command -> time its key went down
        self.mode = "IDLE"
        self.vec = ZERO_VEC
        self.target = None                # relative position where queued motion ends
        self.next_send = 0.0
        self.cancel_t = 0.0
        self.cancel_warned = False
        self.stop_latch = True            # nothing moves until every motion key has been released
        self.quit = False
        self.limit_hits = set()
        self.reset_seen = link.reset_count
        self.error_seen = link.error_count
        self.fault_msg = ""
        self._auto_resets = []            # times of recent automatic resets
        self._worker = None
        self._result = None               # (ok, msg) from the reset worker
        self._homer = None
        self._home_job = None
        self._home_result = None          # HomeResult or HomingError from the homing worker
        self._home_abort = threading.Event()

    # ------------------------------------------------------------- helpers
    @property
    def referenced(self) -> list:
        return [z is not None for z in self.zero]

    @property
    def zeroed(self) -> bool:
        return all(self.referenced)

    def rel(self, mpos):
        return tuple(m - z if z is not None else m for m, z in zip(mpos, self.zero))

    @staticmethod
    def motion_vector(held) -> tuple:
        v = [0] * N
        for k in held:
            kv = C.MOTION_KEYS.get(k)
            if kv:
                v = [a + b for a, b in zip(v, kv)]
        return tuple(max(-1, min(1, a)) for a in v)

    def active_gear(self) -> int:
        return self.gear if self.zeroed else C.UNZEROED_GEAR

    def velocity(self, vec):
        frac = C.GEARS[self.active_gear()]
        v = [vec[i] * frac * C.VMAX[i] for i in range(N)]
        if vec[FL] != vec[FR]:                     # one front rail without the other: trim
            for i in C.FRONT:
                v[i] = max(-C.TRIM_SPEED, min(C.TRIM_SPEED, v[i]))
        return v

    def drop_zero(self, why: str, axes=None):
        axes = range(N) if axes is None else axes
        lost = [C.AXIS_LABELS[i] for i in axes if self.zero[i] is not None]
        for i in axes:
            self.zero[i] = None
            self.travel[i] = C.TRAVEL[i]
        if lost:
            self.log(f"ZERO LOST ({why}): {', '.join(lost)}. Limits off for those, fine gear "
                     "only. Home (H) or re-zero (Z).")

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
    def on_event(self, kind: str, key: str, now: float, source: str = "keyboard"):
        if kind == "disconnected":
            self.log(f"input lost: {key}")
            return
        if kind == "connected":
            self.log(f"input: {key}")
            return
        cmd = C.COMMAND_KEYS.get(key)
        if cmd is None:
            return
        if kind == "up":
            self.holds.pop(cmd, None)
            return
        if cmd == "quit" and source != "keyboard":
            return
        if hold_time(cmd) > 0:
            if cmd.startswith("home:") and self.mode in ("FAULT", "RESET", "HOME"):
                self.log("homing: clear the fault first (C)" if self.mode != "HOME"
                         else "homing is already running (Space aborts)")
                return
            self.holds.setdefault(cmd, now)
        elif cmd.startswith("gear"):
            self.gear = int(cmd[-1])
            if not self.zeroed and self.gear != C.UNZEROED_GEAR:
                self.log(f"gear {self.gear} set; fine gear is forced until every axis is zeroed")
        elif cmd == "stop":
            self.stop_latch = True
            self.home_pending = None
            self.abort_home()
        elif cmd == "quit":
            self.quit = True
            self.abort_home()
        elif cmd == "clear_alarm":
            if self.mode == "HOME":
                self.log("homing is running: Space aborts it")
            elif self.mode == "RESET":
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

    def _held_action(self, cmd: str, st):
        """A hold-to-act key has been held long enough."""
        if cmd == "zero":
            if self.mode == "IDLE" and st.state == "Idle":
                self.zero = list(st.mpos)
                self.travel = list(C.TRAVEL)
                self.log("ZERO SET at current pose for every axis. Limits active.")
            else:
                self.log("zero refused: stop moving first")
        elif cmd.startswith("home:"):
            job = cmd.split(":", 1)[1]
            if self.mode == "IDLE" and st.state == "Idle":
                self.home_pending = job
            else:
                self.log("homing refused: stop moving first")

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
        self.link.jog_cancel()                   # GRBL may not have aborted (ALM polarity False)
        self.log(f"FAULT: {why}")
        self.drop_zero("fault")
        self.stop_latch = True                   # no motion until every motion key is released
        self.holds = {}
        self.home_pending = None
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

    def wait_worker(self, timeout: float):
        """Let a running drive reset or homing finish (never quit with the motors left free)."""
        w = self._worker
        if w is not None:
            w.join(timeout)

    # ------------------------------------------------------------- homing
    def _start_home(self, job: str):
        try:
            check_config(job)
        except HomingError as e:
            self.log(f"HOMING NOT STARTED: {e}")
            return
        axes = sorted({a for part in C.HOME_JOBS[job] for a in PARTS[part]})
        self.mode = "HOME"
        self._home_job = job
        for i in axes:                           # these get a new zero from their stops
            self.zero[i] = None
            self.travel[i] = C.TRAVEL[i]
        self.limit_hits = set()
        self._home_result = None
        self._home_abort.clear()
        self._homer = Homer(self.link, recover_drives, self.log, self._home_abort)
        names = ", ".join(C.AXIS_LABELS[i] for i in axes)
        self.log(f"HOMING ({job}: {names}): each motor is driven into its hard stops until its "
                 "drive stalls. Space aborts.")
        self._worker = threading.Thread(target=self._home_worker, args=(job,), daemon=True)
        self._worker.start()

    def _home_worker(self, job: str):
        try:
            self._home_result = self._homer.run(job)
        except HomingError as e:
            self._home_result = e
        except Exception as e:                   # never leave the controller stuck in HOME
            self._home_result = HomingError(f"homing error: {e}")

    def abort_home(self):
        if self.mode == "HOME" and not self._home_abort.is_set():
            self._home_abort.set()
            self.log("aborting homing...")

    def _poll_home(self):
        res = self._home_result
        if res is None:
            return
        self._home_result, self._worker = None, None
        self.reset_seen = self.link.reset_count  # homing's own stalls and resets are expected
        self.error_seen = self.link.error_count
        self.limit_hits = set()
        self.stop_latch = True
        if isinstance(res, HomeResult):
            for i, z in res.zero.items():
                self.zero[i] = z
                self.travel[i] = res.travel[i]
            self.mode = "IDLE"
            rest = [C.AXIS_LABELS[i] for i in range(N) if self.zero[i] is None]
            self.log(f"HOMED {', '.join(res.homed)} ({self._homer.stalls} stalls): zero set, "
                     "limits active." + (f" Still unzeroed: {', '.join(rest)}." if rest else "")
                     + " Release all keys to move.")
        elif isinstance(res, HomingAborted):
            self.mode = "IDLE"
            self.log(f"homing {res}. The axes it was homing are unzeroed.")
        else:
            if res.drive_fault:
                self.mode = "FAULT"
                self.fault_msg = str(res)
                self.drop_zero("drive reset failed during homing")
            else:
                self.mode = "IDLE"
            self.log(f"HOMING FAILED: {res}")

    # ------------------------------------------------------------- main step
    def update(self, held, now: float):
        link, st = self.link, self.link.status

        if self.mode == "RESET":
            self._poll_reset()
            return
        if self.mode == "HOME":
            self._poll_home()
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
            self.home_pending = None                     # no surprise homing after a fault
            self.holds = {k: t for k, t in self.holds.items() if not k.startswith("home:")}
            return
        why = self.fault_reason(st)
        if why:
            self._on_fault(why, now)
            return

        for cmd, t in list(self.holds.items()):          # hold-to-act keys (Z, H, F1-F3)
            if now - t >= hold_time(cmd):
                del self.holds[cmd]
                self._held_action(cmd, st)
        if self.home_pending and self.mode == "IDLE" and st.state == "Idle" and now - st.t < 0.3:
            job, self.home_pending = self.home_pending, None
            self._start_home(job)
            return

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
        d, self.limit_hits = clip_increment(self.target, d, self.travel, self.referenced)
        if max(abs(x) for x in d) < 0.001:               # pinned at a limit
            self.next_send = now + C.JOG_DT
            return
        speed = math.sqrt(sum(x * x for x in d)) / C.JOG_DT    # mm/s along the clipped move
        self.link.jog(d, speed * 60.0)
        self.target = [t + x for t, x in zip(self.target, d)]
        self.next_send = max(self.next_send + C.JOG_DT, now) if not prefill else now + C.JOG_DT

    # ------------------------------------------------------------- display
    def flags(self) -> list:
        st = self.link.status
        out = []
        if self.mode == "HOME" and self._homer is not None:
            out.append(f"HOMING {self._home_job}: {self._homer.step} (Space aborts)")
        if self.home_pending:
            out.append(f"homing {self.home_pending} starts when GRBL is idle")
        for cmd, t in self.holds.items():
            left = hold_time(cmd) - (time.monotonic() - t)
            if left > 0:
                out.append(f"hold {cmd} {left:.1f} s")
        if st.drive_alarm:
            out.append("DRIVE ALM")
        if self.mode == "FAULT":
            out.append(f"FAULT: {self.fault_msg} -> press C")
        if self.limit_hits:
            out.append(f"LIMIT {','.join(sorted(self.limit_hits))}")
        if self.stop_latch:
            out.append("STOP (release keys)")
        return out

    def pose_text(self, p) -> str:
        ref = self.referenced
        bits = []
        if ref[FL] and ref[FR]:
            bits.append(f"rack {p[FL] - p[FR]:+.2f} mm")
        if ref[FL] and ref[FR] and ref[REAR]:
            bits.append(f"pitch {pitch_deg(front_height(p), p[REAR]):+.1f} deg")
        return "  ".join(bits)

    def status_line(self) -> str:
        """One line: for --plain and for remote clients."""
        st = self.link.status
        p = self.rel(st.mpos)
        ref = self.referenced
        pos = " ".join(f"{C.AXIS_LABELS[i]}{'' if ref[i] else '?'} {p[i]:.1f}" for i in range(N))
        tail = self.pose_text(p) or "UNZEROED: fine gear only, hold H to home"
        flags = " | ".join(self.flags())
        return (f"[{self.mode}] gear {self.active_gear()} | {st.state} | {pos} | {tail}"
                + (f" | {flags}" if flags else ""))

    def screen_lines(self) -> list:
        """The top of the full-screen display."""
        st = self.link.status
        p = self.rel(st.mpos)
        ref = self.referenced
        w = 10
        lines = [
            f"mode {self.mode:<6}  gear {self.active_gear()}"
            + ("" if self.zeroed else f" (set {self.gear}, fine until zeroed)")
            + f"   GRBL {st.state}",
            "          " + "".join(f"{C.AXIS_LABELS[i]:>{w}}" for i in range(N)),
            "position  " + "".join(f"{p[i]:>{w}.2f}" for i in range(N)) + "   mm",
            "zeroed    " + "".join(f"{'yes' if ref[i] else 'NO':>{w}}" for i in range(N)),
            "limits    " + "".join(
                f"{(f'{self.travel[i][0]:.0f}..{self.travel[i][1]:.0f}' if ref[i] else 'off'):>{w}}"
                for i in range(N)),
        ]
        pose = self.pose_text(p)
        lines.append(pose if pose else "UNZEROED: limits off where it says NO, fine gear only. "
                                       "Hold H to home everything.")
        lines.append(" | ".join(self.flags()) or "no faults")
        return lines


# ----------------------------------------------------------------- display
KEY_LABELS = {"KEY_UP": "Up", "KEY_DOWN": "Down", "KEY_LEFT": "Left", "KEY_RIGHT": "Right",
              "KEY_SPACE": "Space", "KEY_ESC": "Esc"}


def key_label(name: str) -> str:
    return KEY_LABELS.get(name, name[4:] if name.startswith("KEY_") else name)


def help_entries() -> list:
    return [f"{'/'.join(key_label(k) for k in keys):>10}  {text}" for keys, text in C.KEY_HELP]


def unlisted_keys() -> list:
    listed = {k for keys, _ in C.KEY_HELP for k in keys}
    return sorted((set(C.MOTION_KEYS) | set(C.COMMAND_KEYS)) - listed)


class Display:
    """Full-screen terminal display: status, key menu, recent log. Also writes the log file.
    plain=True: one redrawn status line with the log scrolling above it (old style)."""

    def __init__(self, plain: bool, log_path: str = None):
        self.plain = plain
        self.lines = collections.deque(maxlen=200)
        self.lock = threading.Lock()
        self.file = None
        if log_path:
            try:
                os.makedirs(os.path.dirname(log_path), exist_ok=True)
                self.file = open(log_path, "a", buffering=1)
            except OSError as e:
                print(f"no log file ({e})")
        if not plain:
            sys.stdout.write("\033[?25l\033[2J")     # hide cursor, clear screen

    def log(self, msg: str):
        stamp = time.strftime("%H:%M:%S")
        with self.lock:
            self.lines.append(f"{stamp} {msg}")
            if self.file:
                try:
                    self.file.write(f"{time.strftime('%Y-%m-%d')} {stamp} {msg}\n")
                except OSError:
                    pass
        if self.plain:
            print("\r\033[K" + msg, flush=True)

    def recent(self, n: int) -> list:
        with self.lock:
            return list(self.lines)[-n:]

    def draw(self, ctl, hub):
        size = shutil.get_terminal_size()
        width = size.columns - 1                       # a wrapped line can't be redrawn in place
        if self.plain:
            sys.stdout.write("\r\033[K" + ctl.status_line()[:width])
            sys.stdout.flush()
            return
        out = ["arm-teleop   " + hub.describe()]
        out += ctl.screen_lines()
        entries = help_entries()
        colw = max(len(e) for e in entries) + 3
        cols = max(1, min(3, width // colw))
        rows = (len(entries) + cols - 1) // cols
        out.append("-- keys " + "-" * max(0, width - 8))
        for r in range(rows):
            out.append("".join(entries[c * rows + r].ljust(colw)
                               for c in range(cols) if c * rows + r < len(entries)))
        out.append("-- log " + "-" * max(0, width - 7))
        room = max(1, size.lines - len(out) - 1)
        out += self.recent(min(room, max(C.LOG_LINES, room)))
        out = out[: size.lines - 1]
        sys.stdout.write("\033[H" + "".join(line[:width] + "\033[K\n" for line in out) + "\033[J")
        sys.stdout.flush()

    def close(self):
        if not self.plain:
            sys.stdout.write("\033[?25h\n")
            sys.stdout.flush()
        if self.file:
            self.file.close()


# ----------------------------------------------------------------- runner
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=C.PORT)
    ap.add_argument("--baud", type=int, default=C.BAUD)
    ap.add_argument("--keyboard", default=C.KEYBOARD_DEVICE, help="evdev path, default auto-detect")
    ap.add_argument("--no-grab", action="store_true", help="do not grab the keyboard exclusively")
    ap.add_argument("--no-keyboard", action="store_true", help="no local keyboard (remote input only)")
    ap.add_argument("--listen", default=C.NET_LISTEN,
                    help="address for remote input, e.g. 0.0.0.0 for other machines (default %(default)s)")
    ap.add_argument("--web-port", type=int, default=C.WEB_PORT, help="web page port (default %(default)s)")
    ap.add_argument("--net-port", type=int, default=C.NET_PORT, help="UDP port (default %(default)s)")
    ap.add_argument("--no-net", action="store_true", help="no remote input")
    ap.add_argument("--sim", action="store_true", help="simulated motion, no serial port")
    ap.add_argument("--home", nargs="?", const="all", choices=sorted(C.HOME_JOBS),
                    help="home as soon as GRBL is ready (default: all)")
    ap.add_argument("--plain", action="store_true", help="one status line instead of the full screen")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    log_path = None
    if C.LOG_DIR:
        log_path = os.path.join(here, C.LOG_DIR, time.strftime("teleop-%Y%m%d-%H%M%S.log"))
    disp = Display(plain=args.plain or not sys.stdout.isatty(), log_path=log_path)
    log = disp.log
    for k in unlisted_keys():
        log(f"config: {k} is mapped but not in KEY_HELP, so it is missing from the menu")

    link = kb = net = None
    try:
        if args.sim:
            from sim_link import SimLink
            link = SimLink(letters=C.AXIS_LETTERS, log=log)
        else:
            from grbl_link import GrblLink
            log(f"connecting to GRBL on {args.port} (the Mega resets on connect)...")
            link = GrblLink(args.port, args.baud, C.STATUS_HZ, letters=C.AXIS_LETTERS, log=log,
                            alm_conducts_on_fault=C.ALM_CONDUCTS_ON_FAULT,
                            grbl_order=C.GRBL_AXIS_ORDER)

        if C.USE_KEYBOARD and not args.no_keyboard:
            try:
                from keyboard_in import Keyboard
                kb = Keyboard(args.keyboard, grab=not args.no_grab, log=log)
            except RuntimeError as e:
                log(f"local keyboard off: {e}")
        ctl_ref = {}
        if args.listen and not args.no_net and (args.web_port or args.net_port):
            from inputs import RemoteInput

            def status_for_clients():
                c = ctl_ref.get("ctl")
                return (c.status_line() if c else "starting", disp.recent(C.LOG_LINES))
            try:
                net = RemoteInput(args.listen, args.web_port, args.net_port, C.NET_TOKEN,
                                  C.NET_TIMEOUT_S, log, status_for_clients)
                log(f"remote input: {net.address}")
            except OSError as e:
                log(f"remote input off: cannot listen on {args.listen} ({e})")
        from inputs import InputHub
        hub = InputHub(kb, net)
        ctl = Controller(link, log=log)
        ctl_ref["ctl"] = ctl
        ctl.home_pending = args.home

        def on_signal(*_):
            ctl.quit = True
            ctl.abort_home()
        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        if log_path:
            log(f"log file: {log_path}")
        log("teleop running. Hold H to home, or jog to the zero pose and hold Z.")
        period, next_disp = 1.0 / C.LOOP_HZ, 0.0
        try:
            while not ctl.quit:
                now = time.monotonic()
                if link.failed:
                    log(f"LINK FAILED: {link.failed}")
                    break
                for kind, key, src in hub.events():
                    ctl.on_event(kind, key, now, src)
                ctl.update(hub.held_keys(), now)
                if now >= next_disp:
                    next_disp = now + 1.0 / C.DISPLAY_HZ
                    disp.draw(ctl, hub)
                time.sleep(period)
        finally:
            try:
                if ctl.mode in ("RESET", "HOME"):
                    ctl.abort_home()
                    log("waiting for the drive reset / homing to stop...")
                    ctl.wait_worker(30.0)
                link.jog_cancel()
                t0 = time.monotonic()
                while link.status.state not in ("Idle", "Alarm", "Sleep") and time.monotonic() - t0 < 2.0:
                    time.sleep(0.05)
            except Exception:
                pass
    finally:
        for thing in (kb, net, link):
            if thing is not None:
                try:
                    thing.close()
                except Exception:
                    pass
        disp.close()
        tail = disp.recent(5)
        print("\n".join(tail))
        print("stopped.")


if __name__ == "__main__":
    main()
