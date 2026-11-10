"""Stand-in for GrblLink with no hardware: same interface, simulated motion.

Use it to check the key map, gears, limits and homing on the Jetson before motors are wired:
    python3 teleop.py --sim
Moves are executed at their feed rate with no acceleration.

Each axis has hard stops (SIM_STOPS) and a closed-loop drive: it follows the steps until its
carriage is blocked by a stop, and once it falls more than SIM_TRIP_MM behind it stalls:
ALM fires, the motor goes free (later steps are ignored) and, if no other drive was already
in alarm, GRBL aborts (ALARM:3 if moving), just like the wired-together ALM lines on A9.
GRBL's count (MPos) starts at 0 wherever the axes happen to be, as on power-up.

The two front rails carry one crossbar: if they get more than SIM_RACK_BIND_MM out of
square, the crossbar binds and the rail that is moving is blocked (and stalls).

inject_fault() (F12 in teleop --sim) stalls the front-left drive on demand.
clear_needs_free_s > the MF times in config simulates a drive that only clears on a power cycle.
time_scale > 1 runs motion and resets that many times faster (for tests).
alm_visible=False: ALM still aborts GRBL but never shows up as Pn:R (for tests).
"""
from __future__ import annotations

import collections
import math
import threading
import time

from grbl_link import ALARM_TEXT, Status

PLANNER_SIZE = 31

# Hard stops per axis (FL, FR, rear, piston, bogie), in mm from where each axis sits at
# power-up. The front rails start 120 / 126 mm above their bottom stops (crossbar 6 mm out of
# square), the rear 80 mm (barrel slightly pitched), the piston 200 mm out, the bogie 150 mm
# right of its left stop.
SIM_STOPS = ((-120.0, 440.0), (-126.0, 434.0), (-80.0, 475.0), (-200.0, 310.0), (-150.0, 298.0))
SIM_TRIP_MM = 2.0                   # following error at which a blocked drive alarms
SIM_RACK_BIND_MM = 25.0             # front rails this far out of square: the crossbar binds
SIM_FRONT = (0, 1)                  # which axes are the two front rails
SIM_MAX_STEP_MM = 0.1               # motion is integrated in steps no longer than this


class SimLink:
    def __init__(self, letters=("X", "B", "Y", "Z", "A"), log=print, clear_needs_free_s=0.0,
                 stops=SIM_STOPS, trip_mm=SIM_TRIP_MM, rack_bind_mm=SIM_RACK_BIND_MM,
                 time_scale=1.0, alm_visible=True, **_):
        self.letters = tuple(letters)
        self.n_axes = len(self.letters)
        self.log = log
        self.planner_total = PLANNER_SIZE
        self.reset_count = 1
        self.last_error = None
        self.error_count = 0
        self.last_alarm = None
        self.alarm_count = 0
        self.failed = None
        self.clear_needs_free_s = clear_needs_free_s
        self.stops = stops or ((-math.inf, math.inf),) * self.n_axes
        self.trip_mm = trip_mm
        self.rack_bind_mm = rack_bind_mm
        self.time_scale = time_scale
        self.alm_visible = alm_visible    # False: ALM aborts GRBL but never shows as Pn:R (test)
        self.lines = []                   # every jog line "sent", for tests
        self._pos = [0.0] * self.n_axes   # GRBL's step count (MPos)
        self.phys = [0.0] * self.n_axes   # where each carriage really is (same frame at power-up)
        self._cmd = [0.0] * self.n_axes   # where each drive has been told to be
        self._faulted = set()             # axes whose drive is in alarm (motor free)
        self._queue = collections.deque()  # (remaining delta list, speed mm/s)
        self._alarm = False               # GRBL alarm lock
        self._lock = threading.Lock()
        self._alive = True
        self.status = self._snapshot()
        threading.Thread(target=self._run, daemon=True).start()

    def _snapshot(self):
        if self._alarm:
            state = "Alarm"
        else:
            state = "Jog" if self._queue else "Idle"
        alm = bool(self._faulted) and self.alm_visible
        return Status(state, tuple(self._pos), PLANNER_SIZE - len(self._queue), 127, time.monotonic(),
                      pins="R" if alm else "", drive_alarm=alm)

    # ------------------------------------------------------------- mechanics (lock held)
    def _step(self, d):
        """GRBL sends d; each healthy drive follows until blocked. Returns an alarm message
        if a drive stalled and that aborted GRBL."""
        self._pos = [p + x for p, x in zip(self._pos, d)]
        msg = None
        moved = []
        for i, x in enumerate(d):
            if i in self._faulted or not x:
                continue
            lo, hi = self.stops[i]
            self._cmd[i] += x
            self.phys[i] = min(max(self._cmd[i], lo), hi)
            moved.append(i)
        a, b = SIM_FRONT
        if self.rack_bind_mm and max(a, b) < self.n_axes:
            rack = self.phys[a] - self.phys[b]
            if abs(rack) > self.rack_bind_mm:           # crossbar binds: the mover is blocked
                lim = math.copysign(self.rack_bind_mm, rack)
                if a in moved:
                    self.phys[a] = self.phys[b] + lim
                else:
                    self.phys[b] = self.phys[a] - lim
        for i in moved:
            if abs(self._cmd[i] - self.phys[i]) > self.trip_mm:
                msg = self._fault(i) or msg
        return msg

    def _fault(self, axis):
        """Drive `axis` alarms. ALM lines are wired together: only the first one aborts GRBL."""
        first = not self._faulted
        self._faulted.add(axis)
        if not first:
            return None
        moving = bool(self._queue)
        self._queue.clear()
        self.reset_count += 1             # GRBL soft-resets on the abort input
        if moving:
            self._alarm = True
            self.last_alarm = (3, ALARM_TEXT[3])
            self.alarm_count += 1
            return f"GRBL ALARM:3: {ALARM_TEXT[3]}"
        return None

    def _run(self):
        dt = 0.005
        while self._alive:
            msg = None
            with self._lock:
                budget = None
                while self._queue:
                    rem, speed = self._queue[0]
                    dist = math.sqrt(sum(r * r for r in rem))
                    if budget is None:
                        budget = speed * dt * self.time_scale
                    chunk = min(budget, SIM_MAX_STEP_MM)     # small steps: stalls land precisely
                    if dist <= chunk:
                        self._queue.popleft()
                        budget -= dist
                        step = rem
                    else:
                        f = chunk / dist
                        self._queue[0] = ([r * (1 - f) for r in rem], speed)
                        budget -= chunk
                        step = [r * f for r in rem]
                    msg = self._step(step)
                    if msg or budget <= 1e-9:
                        break
                self.status = self._snapshot()
            if msg:
                self.log(msg)
            time.sleep(dt)

    # ------------------------------------------------------------- link interface
    def jog(self, delta, feed_mm_min):
        words = " ".join(f"{L}{d:.3f}" for L, d in zip(self.letters, delta) if abs(d) >= 0.0005)
        if not words:
            return
        line = f"$J=G91 G21 {words} F{feed_mm_min:.1f}"
        self.lines.append(line)
        with self._lock:
            if self._alarm:               # GRBL rejects jogs while alarm-locked
                self.last_error = (line, "error:8")
                self.error_count += 1
                return
            self._queue.append(([round(d, 3) for d in delta], feed_mm_min / 60.0))

    def jog_cancel(self):
        with self._lock:
            self._queue.clear()
            self.status = self._snapshot()

    def feed_hold(self):
        self.jog_cancel()

    def unlock(self):
        with self._lock:
            self._alarm = False
            self.status = self._snapshot()
        return "ok"

    def inject_fault(self, axis=0):
        """Simulate a drive stall (front-left by default): ALM pulls GRBL's abort input low."""
        with self._lock:
            msg = self._fault(axis)
            self.status = self._snapshot()
        if msg:
            self.log(msg)

    def _finish_reset(self, cleared, settle_s):
        with self._lock:
            if cleared:
                self._faulted.clear()
                self._cmd = list(self.phys)   # a cleared drive takes its encoder as its position
            self._alarm = False
            self.reset_count += 1
            self.status = self._snapshot()
        time.sleep(settle_s / self.time_scale)
        if self._faulted:
            return False, "ALM still active"
        return True, "drives energised, ALM clear"

    def reset_drives(self, free_s=0.3, wake_axis=-1, wake_mm=0.05, settle_s=0.3):
        self.jog_cancel()
        time.sleep((free_s + 0.5) / self.time_scale)   # about as long as the real sequence
        return self._finish_reset(free_s >= self.clear_needs_free_s, settle_s)

    def power_cycle_drives(self, off_cmd, on_cmd, off_s=1.0, boot_s=1.5, wake_axis=-1, wake_mm=0.05,
                           settle_s=0.3):
        self.jog_cancel()
        time.sleep((off_s + boot_s) / self.time_scale)
        return self._finish_reset(True, settle_s)

    def close(self):
        self._alive = False
