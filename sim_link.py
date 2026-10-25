"""Stand-in for GrblLink with no hardware: same interface, simulated motion.

Use it to check the key map, gears and limits on the Jetson before motors are wired:
    python3 teleop.py --sim
Moves are executed at their feed rate with no acceleration.
"""
from __future__ import annotations

import collections
import math
import threading
import time

from grbl_link import Status

PLANNER_SIZE = 31


class SimLink:
    def __init__(self, letters=("X", "Y", "Z", "A"), log=print, **_):
        self.letters = tuple(letters)
        self.n_axes = len(self.letters)
        self.log = log
        self.planner_total = PLANNER_SIZE
        self.reset_count = 1
        self.last_error = None
        self.error_count = 0
        self.failed = None
        self.lines = []                   # every jog line "sent", for tests
        self._pos = [0.0] * self.n_axes
        self._queue = collections.deque()  # (remaining delta list, speed mm/s)
        self._lock = threading.Lock()
        self._alive = True
        self.status = self._snapshot()
        threading.Thread(target=self._run, daemon=True).start()

    def _snapshot(self):
        busy = bool(self._queue)
        return Status("Jog" if busy else "Idle", tuple(self._pos),
                      PLANNER_SIZE - len(self._queue), 127, time.monotonic())

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
            self._queue.append(([round(d, 3) for d in delta], feed_mm_min / 60.0))

    def jog_cancel(self):
        with self._lock:
            self._queue.clear()
            self.status = self._snapshot()

    def feed_hold(self):
        self.jog_cancel()

    def unlock(self):
        return "ok"

    def close(self):
        self._alive = False
