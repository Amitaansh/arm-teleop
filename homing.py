"""Auto homing on the hard stops: every motor finds its own stops.

There are no home switches. A blocked closed-loop drive falls behind the commanded position,
trips its following-error alarm (ALM) and frees its motor, and ALM aborts GRBL through A9.
That stall is the home signal. After every stall the drives are reset (recover_drives).
The ALM lines are wired together, so a stall only says *some* drive stalled. Homing makes
it unambiguous by ending every stop with a seek where only one motor moves.

Parts (config.HOME_JOBS picks which; config.HOME_ENDS picks which stops, in order):

  rails   for each end (top, then bottom), to keep the crossbar square and the pitch sane:
            1. all three rails together until the first stall
            2. front pair together until the first stall
            3. front-left alone until it stalls     (at most HOME_SQUARE_SEARCH_MM)
            4. front-right alone until it stalls    (at most HOME_SQUARE_SEARCH_MM)
            5. rear alone until it stalls
          Steps 3-5 give each motor its own stop. Moving together first means one rail never
          travels far on its own: that would rack the crossbar (front pair) or pitch the
          barrel past its limits (rear). After step 2, moving one rail only brings things
          back toward square / level.
  piston  in until it stalls (and out first, if HOME_ENDS["piston"] has "max").
  bogie   right stop, then left stop: zero midway between them (HOME_ZERO "center").

Zero: an axis with HOME_ZERO "min" is zeroed at its min stop (minus HOME_MIN_STOP_MM).
The zero must come from that motor's last stall: a drive only trips after falling some
distance behind (the trip distance), and GRBL has already counted those steps, so every
stall leaves GRBL's count off by the trip distance. Taking the zero from the last stall
cancels that out, which is why "min" comes last in HOME_ENDS.

Limits: the measured stops, HOME_TOP_MARGIN_MM inside (HOME_BOTTOM_MARGIN_MM above a min-stop
zero), intersected with config.TRAVEL.

Bounds on moves where one side travels without the other: a front rail alone at most
HOME_SQUARE_SEARCH_MM (crossbar rack), the front pair or the rear alone at most
HOME_ALONE_MAX_MM (barrel pitch).

A stall is GRBL aborting (reset banner / ALARM:3) while the motor moves. If ALM does not
show up in the status report (Pn:R), it might be noise on A9, so the same seek runs again
for at most HOME_CONFIRM_MM: a motor that is really at its stop stalls again at once.

Uses only the link interface (status, reset_count, error_count, last_error, failed, jog,
jog_cancel), so it runs against SimLink too.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import config as C

N = C.N_AXES
FL, FR, REAR, PISTON, BOGIE = C.FL, C.FR, C.REAR, C.PISTON, C.BOGIE
PARTS = {"rails": (FL, FR, REAR), "piston": (PISTON,), "bogie": (BOGIE,)}
MOVING = ("Jog", "Run", "Hold")
UP_WORD = {FL: "up", FR: "up", REAR: "up", PISTON: "out", BOGIE: "right"}
DOWN_WORD = {FL: "down", FR: "down", REAR: "down", PISTON: "in", BOGIE: "left"}


class HomingError(RuntimeError):
    def __init__(self, msg: str, drive_fault: bool = False):
        super().__init__(msg)
        self.drive_fault = drive_fault        # True: the drives could not be reset


class HomingAborted(HomingError):
    pass


@dataclass
class HomeResult:
    zero: dict = field(default_factory=dict)      # axis -> MPos of its zero
    travel: dict = field(default_factory=dict)    # axis -> (lo, hi) relative to that zero
    stops: dict = field(default_factory=dict)     # axis -> {"min"/"max": MPos at its stall}

    @property
    def homed(self) -> tuple:
        return tuple(C.AXIS_LABELS[i] for i in sorted(self.zero))


def check_config(job: str = None) -> None:
    """Raise HomingError if the homing settings can't work."""
    jobs = [job] if job else list(C.HOME_JOBS)
    for j in jobs:
        if j not in C.HOME_JOBS:
            raise HomingError(f"unknown homing job {j!r}; HOME_JOBS has {list(C.HOME_JOBS)}")
        for part in C.HOME_JOBS[j]:
            if part not in PARTS:
                raise HomingError(f"HOME_JOBS[{j!r}]: unknown part {part!r}; valid: {list(PARTS)}")
    for part, ends in C.HOME_ENDS.items():
        if part not in PARTS:
            raise HomingError(f"HOME_ENDS: unknown part {part!r}")
        if not ends or any(e not in ("min", "max") for e in ends) or len(set(ends)) != len(ends):
            raise HomingError(f"HOME_ENDS[{part!r}] = {ends}: use 'max' and/or 'min', once each")
        for axis in PARTS[part]:
            mode = C.HOME_ZERO[axis]
            if mode not in ("min", "center"):
                raise HomingError(f"HOME_ZERO for {C.AXIS_NAMES[axis]}: 'min' or 'center'")
            if "min" not in ends:
                raise HomingError(f"HOME_ENDS[{part!r}] needs 'min': the zero is set from it")
            if mode == "min" and ends[-1] != "min":
                raise HomingError(f"HOME_ENDS[{part!r}]: 'min' must come last "
                                  "(the zero has to come from the last stall)")


class Homer:
    """Runs a homing job on a link. Blocking: run it in a worker thread.

    recover: function(link, log) -> (ok, msg) that resets the drives after a stall.
    abort:   threading.Event; set it to stop homing (motion is cancelled, drives left healthy).
    step:    what it is doing right now, for the status display.
    """

    def __init__(self, link, recover, log=print, abort: threading.Event = None):
        self.link, self.recover, self.log = link, recover, log
        self.abort = abort if abort is not None else threading.Event()
        self.step = "starting"
        self.stalls = 0
        # how far a full seek may go before giving up: the whole travel plus some
        self.search = [hi - lo + C.HOME_SEARCH_EXTRA_MM for lo, hi in C.TRAVEL]
        self.search[BOGIE] = (max(C.TRAVEL[BOGIE][1], -C.HOME_MIN_STOP_MM[BOGIE])
                              - min(C.TRAVEL[BOGIE][0], C.HOME_MIN_STOP_MM[BOGIE])
                              + C.HOME_SEARCH_EXTRA_MM)

    # ------------------------------------------------------------- job
    def run(self, job: str = "all") -> HomeResult:
        check_config(job)
        st = self._idle()
        if st.drive_alarm:
            raise HomingError("a drive alarm is active: press C first")
        res = HomeResult()
        for part in C.HOME_JOBS[job]:
            axes = PARTS[part]
            stops = {i: {} for i in axes}
            for end in C.HOME_ENDS[part]:
                d = +1 if end == "max" else -1
                if part == "rails":
                    self._rails(d, end, stops)
                else:
                    axis = axes[0]
                    word = UP_WORD[axis] if d > 0 else DOWN_WORD[axis]
                    label = f"{C.AXIS_LABELS[axis]} {word}"
                    st = self._seek(label, {axis: d}, self.search[axis])
                    stops[axis][end] = st.mpos[axis]
                    self._reset(label)
            for axis in axes:
                zero, travel = self._reference(axis, stops[axis])
                res.zero[axis], res.travel[axis], res.stops[axis] = zero, travel, stops[axis]
            for axis in axes:
                park = C.HOME_PARK_MM[axis]
                if park is not None:
                    st = self._idle()
                    self._move(f"{C.AXIS_LABELS[axis]} to {park:+.0f} mm", axis,
                               park - (st.mpos[axis] - res.zero[axis]))
        self.step = "done"
        return res

    def _rails(self, d: int, end: str, stops: dict) -> None:
        word = "up" if d > 0 else "down"
        full = max(self.search[i] for i in C.RAILS)
        self._seek(f"rails {word} together", {FL: d, FR: d, REAR: d}, full)
        self._reset(f"rails {word} together")
        alone = C.HOME_ALONE_MAX_MM
        hint = (f"more than {alone:g} mm without the other side would pitch the barrel too far. "
                "Did the first stall hit something other than a stop? Start nearer level, or "
                "raise HOME_ALONE_MAX_MM")
        self._seek(f"front pair {word}", {FL: d, FR: d}, alone, hint=hint)
        self._reset(f"front pair {word}")
        for axis in (FL, FR):
            label = f"{C.AXIS_LABELS[axis]} {word} alone (squaring)"
            st = self._seek(label, {axis: d}, C.HOME_SQUARE_SEARCH_MM,
                            hint=f"the front rails are more than {C.HOME_SQUARE_SEARCH_MM:g} mm "
                                 "out of square (trim with R/F or T/G first), or this drive's "
                                 "ALM / STEP wiring is off")
            stops[axis][end] = st.mpos[axis]
            self._reset(label)
        label = f"rear {word} alone"
        st = self._seek(label, {REAR: d}, alone, hint=hint)
        stops[REAR][end] = st.mpos[REAR]
        self._reset(label)

    def _reference(self, axis: int, s: dict):
        """Zero and limits for one axis from its stall positions (GRBL MPos)."""
        name, m = C.AXIS_LABELS[axis], C.HOME_TOP_MARGIN_MM
        lo_cfg, hi_cfg = C.TRAVEL[axis]
        if C.HOME_ZERO[axis] == "center" and "max" in s:
            zero = (s["min"] + s["max"]) / 2.0
            half = (s["max"] - s["min"]) / 2.0
            if half <= m:
                raise HomingError(f"{name}: only {2 * half:.1f} mm between its stops: "
                                  "it stalled on something on the way?")
            travel = (max(lo_cfg, -half + m), min(hi_cfg, half - m))
            self.log(f"homing: {name}: stops {2 * half:.1f} mm apart, zero midway; "
                     f"limits {travel[0]:+.1f} .. {travel[1]:+.1f} mm")
        else:
            zero = s["min"] - C.HOME_MIN_STOP_MM[axis]
            lo = max(lo_cfg, C.HOME_MIN_STOP_MM[axis] + C.HOME_BOTTOM_MARGIN_MM)
            hi = hi_cfg
            if "max" in s:
                measured = s["max"] - s["min"]
                top = s["max"] - zero
                if top - m <= lo:
                    raise HomingError(f"{name}: only {measured:.1f} mm between its stops: "
                                      "it stalled on something on the way?")
                hi = min(hi_cfg, top - m)
                self.log(f"homing: {name}: {measured:.1f} mm stop to stop; "
                         f"max {hi:.1f} mm (TRAVEL says {hi_cfg:g})")
            travel = (lo, hi)
        return zero, travel

    # ------------------------------------------------------------- primitives
    def _seek(self, label: str, dirs: dict, search_mm: float, hint: str = None):
        """Jog the axes in dirs ({axis: +1 or -1}) until a drive stalls, for at most
        search_mm. Returns the first status report after the stall (GRBL stopped)."""
        self.step = label
        speed = min(C.HOME_SPEED[i] for i in dirs)
        feed = speed * math.sqrt(len(dirs)) * 60.0       # vector feed: each axis runs at `speed`
        remaining, travelled, confirming = search_mm, 0.0, False
        while True:
            start = self._idle()
            dist = C.HOME_CONFIRM_MM if confirming else remaining
            delta = [0.0] * N
            for i, d in dirs.items():
                delta[i] = d * dist
            stalled, st = self._jog(label, delta, feed, dist / speed)
            moved = max(abs(st.mpos[i] - start.mpos[i]) for i in dirs)
            travelled += moved
            remaining -= moved
            if not stalled:
                if confirming:
                    self.log(f"homing: {label}: no second stall within {C.HOME_CONFIRM_MM:g} mm: "
                             "the abort was not a stall (noise on A9?), seeking on. If this repeats "
                             "at a real stop, raise HOME_CONFIRM_MM")
                    confirming = False
                if remaining > 0.5:
                    continue
                raise HomingError(f"{label}: moved {travelled:.0f} mm without a stall. "
                                  + (hint or "Is there a hard stop, and does the drive's ALM "
                                             "reach A9?"))
            if st.drive_alarm or confirming:
                self.stalls += 1
                self.log(f"homing: {label}: stalled after {travelled:.1f} mm")
                return st
            self.log(f"homing: {label}: GRBL aborted after {travelled:.1f} mm but ALM is not "
                     "showing in the status (Pn:R); checking with a short second seek")
            self._reset(label)
            confirming = True

    def _move(self, label: str, axis: int, mm: float) -> None:
        """A plain move that must not stall."""
        if abs(mm) < 0.001:
            return
        self.step = label
        self._idle()
        delta = [0.0] * N
        delta[axis] = mm
        v = C.HOME_SPEED[axis]
        stalled, _ = self._jog(label, delta, v * 60.0, abs(mm) / v)
        if stalled:
            self._reset(label)
            raise HomingError(f"{label}: stalled on the way")

    def _jog(self, label, delta, feed_mm_min, duration):
        """Send one jog and watch it. Returns (stalled, status).

        stalled = GRBL reset (A9 abort), GRBL alarm, or a drive alarm. Otherwise it returns
        once GRBL is back to Idle after the move.
        """
        link = self.link
        self._check_abort(label)
        resets, errors = link.reset_count, link.error_count
        t0 = time.monotonic()
        link.jog(delta, feed_mm_min)
        moving = False
        while True:
            if link.failed:
                raise HomingError(f"serial link failed: {link.failed}")
            st, now = link.status, time.monotonic()
            fresh = st.t > t0
            if (link.reset_count != resets or st.drive_alarm
                    or (fresh and st.state.startswith("Alarm"))):
                # checked before abort, so a stall is always followed by a drive reset
                link.jog_cancel()     # in case ALM does not abort GRBL (ALM_CONDUCTS_ON_FAULT False)
                return True, self._settled()
            if self.abort.is_set():
                self._stop()
                raise HomingAborted(f"aborted during '{label}'")
            if link.error_count != errors:
                raise HomingError(f"{label}: GRBL rejected the jog ({link.last_error[1]})")
            if fresh:
                if st.state.startswith(MOVING):
                    moving = True
                elif st.state == "Idle" and (moving or now - t0 > duration + 1.0):
                    return False, st
            if now - t0 > duration * 1.5 + 10.0:
                self._stop()
                raise HomingError(f"{label}: GRBL still {st.state} after {now - t0:.0f} s")
            time.sleep(0.005)

    def _reset(self, label: str):
        """Reset the drives after a stall."""
        self.step = f"{label}: resetting drives"
        ok, msg = self.recover(self.link, self.log)
        if not ok:
            raise HomingError(f"drive reset after '{label}' failed: {msg}", drive_fault=True)

    def _settled(self):
        st = self._wait(lambda s: not s.state.startswith(MOVING), 3.0, after=time.monotonic())
        if st is None:
            raise HomingError(f"GRBL did not stop after a stall (state {self.link.status.state})")
        return st

    def _idle(self):
        st = self._wait(lambda s: s.state == "Idle", 3.0, after=time.monotonic())
        if st is None:
            raise HomingError(f"GRBL not Idle (state {self.link.status.state})")
        return st

    def _stop(self) -> None:
        self.link.jog_cancel()
        self._wait(lambda s: not s.state.startswith(MOVING), 3.0, after=time.monotonic())

    def _check_abort(self, label: str) -> None:
        if self.abort.is_set():
            raise HomingAborted(f"aborted before '{label}'")

    def _wait(self, pred, timeout: float, after: float):
        """First status report received after `after` that satisfies pred, or None."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.link.failed:
                raise HomingError(f"serial link failed: {self.link.failed}")
            st = self.link.status
            if st.t > after and pred(st):
                return st
            time.sleep(0.01)
        return None
