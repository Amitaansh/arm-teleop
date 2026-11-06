"""Stand-in for GrblLink with no hardware: same interface, simulated motion.

Use it to check the key map, gears and limits on the Jetson before motors are wired:
    python3 teleop.py --sim
Moves are executed at their feed rate with no acceleration.

Drive alarms can be simulated too: inject_fault() (F12 in teleop --sim) acts like a drive
stalling: ALM fires, GRBL aborts (ALARM:3 if moving) and reports the alarm until the drives
are reset. clear_needs_free_s > the MF times in config simulates a drive that only clears
on a power cycle.
"""
from __future__ import annotations

import collections
import math
import threading
import time

from grbl_link import ALARM_TEXT, Status

PLANNER_SIZE = 31


class SimLink:
    def __init__(self, letters=("X", "Y", "Z", "A"), log=print, clear_needs_free_s=0.0, **_):
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
        self.lines = []                   # every jog line "sent", for tests
        self._pos = [0.0] * self.n_axes
        self._queue = collections.deque()  # (remaining delta list, speed mm/s)
        self._alarm = False               # GRBL alarm lock
        self._drive_fault = False         # a drive's ALM is active
        self._lock = threading.Lock()
        self._alive = True
        self.status = self._snapshot()
        threading.Thread(target=self._run, daemon=True).start()

    def _snapshot(self):
        if self._alarm:
            state = "Alarm"
        else:
            state = "Jog" if self._queue else "Idle"
        return Status(state, tuple(self._pos), PLANNER_SIZE - len(self._queue), 127, time.monotonic(),
                      pins="R" if self._drive_fault else "", drive_alarm=self._drive_fault)

    def _run(self):
        dt = 0.005
        while self._alive:
            with self._lock:
                budget = None
                while self._queue:
                    rem, speed = self._queue[0]
                    dist = math.sqrt(sum(r * r for r in rem))
                    if budget is None:
                        budget = speed * dt
                    if dist <= budget:
                        self._pos = [p + r for p, r in zip(self._pos, rem)]
                        budget -= dist
                        self._queue.popleft()
                    else:
                        f = budget / dist
                        self._pos = [p + r * f for p, r in zip(self._pos, rem)]
                        self._queue[0] = ([r * (1 - f) for r in rem], speed)
                        break
                self.status = self._snapshot()
            time.sleep(dt)

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

    def inject_fault(self):
        """Simulate a drive stall: ALM pulls GRBL's reset/abort input low."""
        with self._lock:
            moving = bool(self._queue)
            self._queue.clear()
            self._drive_fault = True
            self.reset_count += 1         # GRBL soft-resets on the abort input
            if moving:
                self._alarm = True
                self.last_alarm = (3, ALARM_TEXT[3])
                self.alarm_count += 1
            self.status = self._snapshot()
        if moving:
            self.log(f"GRBL ALARM:3: {ALARM_TEXT[3]}")

    def _finish_reset(self, cleared, settle_s):
        with self._lock:
            if cleared:
                self._drive_fault = False
            self._alarm = False
            self.reset_count += 1
            self.status = self._snapshot()
        time.sleep(settle_s)
        if self._drive_fault:
            return False, "ALM still active"
        return True, "drives energised, ALM clear"

    def reset_drives(self, free_s=0.3, wake_axis=-1, wake_mm=0.05, settle_s=0.3):
        self.jog_cancel()
        time.sleep(free_s + 0.5)          # about as long as the real sequence
        return self._finish_reset(free_s >= self.clear_needs_free_s, settle_s)

    def power_cycle_drives(self, off_cmd, on_cmd, off_s=1.0, boot_s=1.5, wake_axis=-1, wake_mm=0.05,
                           settle_s=0.3):
        self.jog_cancel()
        time.sleep(off_s + boot_s)
        return self._finish_reset(True, settle_s)

    def close(self):
        self._alive = False
