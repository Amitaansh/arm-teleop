"""Serial link to GRBL 1.1 (grbl-Mega-5X). The ONLY file that knows GRBL exists.

Swapping GRBL for custom firmware later means rewriting this file behind the same
interface: status, planner_total, reset_count, failed, jog(), jog_cancel(), unlock(),
reset_drives(), power_cycle_drives(), close().

Axes are always in teleop's own order (letters=...). GRBL reports MPos in its own axis
order (X Y Z A B for grbl-Mega-5X with N_AXIS 5); this file maps between the two.

Protocol notes (GRBL 1.1):
- Lines get 'ok' or 'error:N'. Character-counting flow control: unacknowledged bytes
  must never exceed GRBL's 128-byte RX buffer.
- Realtime bytes skip the buffer: '?' status, 0x85 jog cancel, '!' hold, 0x18 reset.
- Status needs $10=3 (MPos + buffer state):  <Jog|MPos:1.000,...|Bf:31,127|FS:...|Pn:R>

Drive wiring this file relies on:
- ALM: the five drives' ALM+ are joined onto A9, GRBL's reset/abort input (4.7k pull-up),
  ALM- to GND. A faulted drive (stall = position following error, over-current, ...) pulls
  A9 low. GRBL then aborts at once (ALARM:3 if it was moving) and soft-resets, and every
  status report carries 'R' in its Pn: field for as long as the drive holds the alarm.
  That 'R' is how the drive alarm is seen here (Status.drive_alarm).
- MF: each drive's MF+ is on its axis's GRBL enable pin, MF- to GND. GRBL drives those pins
  HIGH (= motor freed) only when it disables the steppers ($4=0, $1=255): on an alarm and
  in sleep ($SLP). reset_drives() uses sleep to hold MF on, which is what clears the drive.
"""
from __future__ import annotations

import collections
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import serial

RT_STATUS = b"?"
RT_HOLD = b"!"
RT_RESUME = b"~"
RT_RESET = b"\x18"
RT_JOG_CANCEL = b"\x85"
RX_BUFFER = 127

IDLE_LOCK_S = 0.30      # GRBL waits $1 ms (255) after entering sleep before the enable pins go HIGH
WAKE_FEED = 30.0        # mm/min for the tiny out-and-back jog that re-energises the drives

ALARM_TEXT = {
    1: "hard limit",
    2: "soft limit",
    3: "reset while moving (drive ALM fired, or reset input)",
    4: "probe fail", 5: "probe fail",
    6: "homing fail", 7: "homing fail", 8: "homing fail", 9: "homing fail", 10: "homing fail",
}

# Status messages a drive reset produces on purpose; not worth printing while it runs.
_QUIET_MSGS = ("[MSG:Sleeping]", "[MSG:'$H'|'$X' to unlock]", "[MSG:Caution: Unlocked]")


class GrblError(RuntimeError):
    pass


@dataclass
class Status:
    state: str = "Unknown"                       # Idle, Jog, Run, Hold:0, Alarm, Sleep, ...
    mpos: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)
    planner_free: Optional[int] = None
    rx_free: Optional[int] = None
    t: float = 0.0                               # time.monotonic() when received
    pins: str = ""                               # Pn: letters; 'R' = reset/abort input (A9) active
    drive_alarm: bool = False                    # a drive's ALM output says it is faulted


class _Pending:
    __slots__ = ("line", "nbytes", "event", "result")

    def __init__(self, line: str, nbytes: int):
        self.line, self.nbytes = line, nbytes
        self.event = threading.Event()
        self.result: Optional[str] = None


class GrblLink:
    def __init__(self, port: str, baud: int = 115200, status_hz: float = 20.0,
                 letters=("X", "Y", "Z", "A"), log=print, alm_conducts_on_fault: bool = True,
                 grbl_order: str = "XYZABC"):
        """letters: GRBL letter of each of our axes, in our axis order (e.g. X B Y Z A).
        grbl_order: the order GRBL itself reports MPos in (its axis 1, 2, 3, ...)."""
        self.letters = tuple(letters)
        self.n_axes = len(self.letters)
        missing = [L for L in self.letters if L not in grbl_order]
        if missing:
            raise GrblError(f"axis letters {missing} not in GRBL axis order {grbl_order!r}")
        self._mpos_index = [grbl_order.index(L) for L in self.letters]
        self._short_mpos_warned = False
        self.log = log
        self.alm_conducts_on_fault = alm_conducts_on_fault
        self.status = Status(mpos=(0.0,) * self.n_axes)
        self.planner_total = 0            # largest Bf planner count seen = buffer size
        self.reset_count = 0              # increments on every GRBL welcome banner
        self.last_error: Optional[Tuple[str, str]] = None
        self.error_count = 0
        self.last_alarm: Optional[Tuple[int, str]] = None    # (code, text) of the latest ALARM:N
        self.alarm_count = 0
        self.failed: Optional[str] = None

        self._cv = threading.Condition()
        self._wlock = threading.Lock()
        self._pending = collections.deque()
        self._pending_bytes = 0
        self._capture: Optional[List[str]] = None
        self._welcome = threading.Event()
        self._quiet = 0                   # >0 while a drive reset runs
        self._alive = True

        # Opening the port toggles DTR, which resets the Mega (bootloader ~1-2 s).
        self._ser = serial.Serial(port, baud, timeout=0.05)
        threading.Thread(target=self._reader, daemon=True).start()
        if not self._welcome.wait(4.0):
            self.realtime(RT_RESET)
            if not self._welcome.wait(3.0):
                self.close()
                raise GrblError(f"no GRBL banner on {port}")
        time.sleep(0.1)
        self.send_line_wait("$10=3")      # MPos + buffer state in status reports
        threading.Thread(target=self._poller, args=(status_hz,), daemon=True).start()

    # ------------------------------------------------------------- low level
    def _write(self, data: bytes) -> None:
        with self._wlock:
            self._ser.write(data)

    def realtime(self, byte: bytes) -> None:
        self._write(byte)

    def send_line(self, line: str, timeout: float = 2.0) -> _Pending:
        """Queue a line without waiting for its 'ok' (character counting)."""
        data = (line.strip() + "\n").encode()
        if len(data) > RX_BUFFER:
            raise GrblError(f"line too long: {line}")
        p = _Pending(line.strip(), len(data))
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._pending_bytes + len(data) > RX_BUFFER:
                left = deadline - time.monotonic()
                if left <= 0 or self.failed:
                    raise GrblError("GRBL RX buffer stuck (no 'ok' coming back)")
                self._cv.wait(left)
            self._pending.append(p)
            self._pending_bytes += len(data)
            self._write(data)             # under the lock so queue order == wire order
        return p

    def send_line_wait(self, line: str, timeout: float = 5.0) -> str:
        p = self.send_line(line, timeout)
        if not p.event.wait(timeout):
            raise GrblError(f"no reply to {line!r}")
        return p.result

    def query(self, line: str, timeout: float = 5.0):
        """Send a $ command and collect its text reply lines (e.g. '$$', '$I')."""
        self._capture = []
        try:
            result = self.send_line_wait(line, timeout)
            return list(self._capture), result
        finally:
            self._capture = None

    # ------------------------------------------------------------- threads
    def _reader(self) -> None:
        buf = b""
        while self._alive:
            try:
                chunk = self._ser.read(256)
            except (serial.SerialException, OSError) as e:
                self._fail(f"serial error: {e}")
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self._handle(line)

    def _poller(self, hz: float) -> None:
        while self._alive and not self.failed:
            try:
                self.realtime(RT_STATUS)
            except (serial.SerialException, OSError) as e:
                self._fail(f"serial error: {e}")
                return
            time.sleep(1.0 / hz)

    def _fail(self, msg: str) -> None:
        if self._alive and not self.failed:
            self.failed = msg
            with self._cv:
                self._cv.notify_all()

    def _handle(self, line: str) -> None:
        if line.startswith("<") and line.endswith(">"):
            self._parse_status(line)
        elif line == "ok" or line.startswith("error:"):
            with self._cv:
                if self._pending:
                    p = self._pending.popleft()
                    self._pending_bytes -= p.nbytes
                    p.result = line
                    p.event.set()
                    if line != "ok":
                        self.last_error = (p.line, line)
                        self.error_count += 1
                self._cv.notify_all()
        elif line.startswith("Grbl ") or line.startswith("GrblHAL "):
            with self._cv:                # reset: GRBL dropped everything it had
                for p in self._pending:
                    p.result = "reset"
                    p.event.set()
                self._pending.clear()
                self._pending_bytes = 0
                self.reset_count += 1
                self._cv.notify_all()
            self._welcome.set()
        elif line.startswith("ALARM:"):
            try:
                code = int(line[6:])
            except ValueError:
                code = -1
            self.last_alarm = (code, ALARM_TEXT.get(code, "unknown"))
            self.alarm_count += 1
            self.log(f"GRBL {line}: {self.last_alarm[1]}")
        elif self._capture is not None:
            self._capture.append(line)
        elif self._quiet and line in _QUIET_MSGS:
            pass
        else:
            self.log(f"GRBL {line}")

    def _parse_status(self, line: str) -> None:
        fields = line[1:-1].split("|")
        mpos, pf, rf, pins = self.status.mpos, None, None, ""
        for f in fields[1:]:
            key, _, val = f.partition(":")
            if key == "MPos":
                vals = [float(x) for x in val.split(",")]
                need = max(self._mpos_index) + 1
                if len(vals) < need:
                    if not self._short_mpos_warned:
                        self._short_mpos_warned = True
                        self.log(f"GRBL reports {len(vals)} axes but teleop needs {need} "
                                 f"({''.join(self.letters)}): rebuild grbl-Mega-5X with N_AXIS {need}")
                    vals += [0.0] * (need - len(vals))
                mpos = tuple(vals[k] for k in self._mpos_index)
            elif key == "Bf":
                a, _, b = val.partition(",")
                pf, rf = int(a), int(b)
                self.planner_total = max(self.planner_total, pf)
            elif key == "Pn":
                pins = val
        drive_alarm = ("R" in pins) == self.alm_conducts_on_fault
        self.status = Status(fields[0], mpos, pf, rf, time.monotonic(), pins, drive_alarm)

    # ------------------------------------------------------------- commands
    def jog(self, delta, feed_mm_min: float) -> None:
        """Relative jog. delta per axis in mm; feed is the vector speed in mm/min."""
        words = " ".join(f"{L}{d:.3f}" for L, d in zip(self.letters, delta) if abs(d) >= 0.0005)
        if words:
            self.send_line(f"$J=G91 G21 {words} F{feed_mm_min:.1f}")

    def jog_cancel(self) -> None:
        self.realtime(RT_JOG_CANCEL)

    def feed_hold(self) -> None:
        self.realtime(RT_HOLD)

    def unlock(self) -> str:
        return self.send_line_wait("$X")

    # ------------------------------------------------------------- drive alarm recovery
    def _wait(self, pred: Callable[[Status], bool], timeout: float, after: float = 0.0) -> Optional[Status]:
        """Wait for a status report received after `after` that satisfies pred."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.failed:
                raise GrblError(self.failed)
            st = self.status
            if st.t > after and pred(st):
                return st
            time.sleep(0.01)
        return None

    def _stop_motion(self) -> None:
        self.jog_cancel()
        if not self._wait(lambda s: not s.state.startswith(("Jog", "Run", "Hold")), 3.0):
            raise GrblError(f"GRBL did not stop (state {self.status.state})")

    def _soft_reset(self) -> float:
        """0x18 and wait for the banner. Returns the time the banner was seen."""
        resets = self.reset_count
        self.realtime(RT_RESET)
        deadline = time.monotonic() + 2.0
        while self.reset_count == resets:
            if time.monotonic() > deadline:
                raise GrblError("no GRBL banner after soft reset")
            time.sleep(0.01)
        return time.monotonic()

    def _unlock_if_alarm(self, after: float) -> None:
        st = self._wait(lambda s: True, 1.0, after=after)
        if st is None:
            raise GrblError("no status report from GRBL")
        if st.state.startswith("Sleep"):              # sleeping GRBL ignores lines until a reset
            st = self._wait(lambda s: True, 1.0, after=self._soft_reset())
            if st is None:
                raise GrblError("no status report from GRBL")
        if st.state.startswith("Alarm"):
            res = self.send_line_wait("$X")
            if res != "ok":
                raise GrblError(f"$X refused: {res}")
        if not self._wait(lambda s: s.state == "Idle", 1.5, after=time.monotonic()):
            raise GrblError(f"GRBL not Idle after unlock (state {self.status.state})")

    def _wake(self, axis: int, mm: float) -> None:
        """GRBL re-enables the steppers only when it starts a move: one tiny jog out and back."""
        L = self.letters[axis]
        for d in (mm, -mm):
            res = self.send_line_wait(f"$J=G91 G21 {L}{d:.3f} F{WAKE_FEED:.1f}")
            if res == "reset":
                raise GrblError("drive faulted again as soon as it was re-energised")
            if res != "ok":
                raise GrblError(f"wake jog refused: {res}")
        if not self._wait(lambda s: s.state == "Idle", 3.0, after=time.monotonic()):
            raise GrblError(f"wake jog did not finish (state {self.status.state})")

    def _check_clear(self, settle_s: float, resets_before: int) -> Tuple[bool, str]:
        time.sleep(settle_s)
        t0 = time.monotonic()
        seen = []
        while len(seen) < 3 and time.monotonic() - t0 < 1.0:     # a few fresh reports
            st = self._wait(lambda s: True, 0.5, after=seen[-1].t if seen else t0)
            if st is None:
                break
            seen.append(st)
        if not seen:
            return False, "no status report from GRBL"
        if any(s.drive_alarm for s in seen):
            return False, "ALM still active"
        if self.reset_count != resets_before or not seen[-1].state == "Idle":
            return False, f"drive faulted again while re-enabling (GRBL {seen[-1].state})"
        return True, "drives energised, ALM clear"

    def reset_drives(self, free_s: float = 0.3, wake_axis: int = -1, wake_mm: float = 0.05,
                     settle_s: float = 0.3) -> Tuple[bool, str]:
        """Clear a drive alarm and any GRBL alarm. Blocks for about free_s + 1.5 s.

        1. stop any jog
        2. $SLP: GRBL drives every enable pin HIGH, so MF frees every motor; hold it free_s
        3. soft reset (0x18) to leave sleep, then $X to clear GRBL's alarm lock
        4. a tiny out-and-back jog on wake_axis re-enables the steppers (MF released)
        5. check that ALM has gone inactive and GRBL is Idle
        Returns (ok, message). GRBL keeps MPos through a soft reset, but treat the position
        as lost: the stalled axis slipped, and every motor was free for free_s.
        """
        self._quiet += 1
        try:
            self._stop_motion()
            if self.status.state.startswith("Sleep"):     # already asleep: motors already free
                time.sleep(free_s)
            else:
                res = self.send_line_wait("$SLP")
                if res == "ok" and self._wait(lambda s: s.state.startswith("Sleep"), 2.0):
                    time.sleep(IDLE_LOCK_S + free_s)
                else:
                    self.log(f"$SLP did not work ({res}, state {self.status.state}): "
                             "motors were not freed, so the drive alarm will likely stay")
            t_reset = self._soft_reset()
            resets = self.reset_count
            self._unlock_if_alarm(t_reset)
            self._wake(wake_axis, wake_mm)
            return self._check_clear(settle_s, resets)
        except GrblError as e:
            return False, str(e)
        finally:
            self._quiet -= 1

    def power_cycle_drives(self, off_cmd: str, on_cmd: str, off_s: float = 1.0, boot_s: float = 1.5,
                           wake_axis: int = -1, wake_mm: float = 0.05,
                           settle_s: float = 0.3) -> Tuple[bool, str]:
        """Cut and restore drive power through a relay on a GRBL output (e.g. coolant M8/M9).

        For faults the drive only clears on power-down (ESS manual: stall Err5, over-current
        Err1) when freeing the motor with MF is not enough.
        """
        self._quiet += 1
        try:
            self._stop_motion()
            self._unlock_if_alarm(time.monotonic())
            res = self.send_line_wait(off_cmd)
            if res != "ok":
                return False, f"{off_cmd} refused: {res}"
            time.sleep(off_s)
            res = self.send_line_wait(on_cmd)
            if res not in ("ok", "reset"):    # a drive booting can glitch ALM and reset GRBL
                return False, f"{on_cmd} refused: {res}"
            time.sleep(boot_s)
            resets = self.reset_count
            self._unlock_if_alarm(time.monotonic())
            self._wake(wake_axis, wake_mm)
            return self._check_clear(settle_s, resets)
        except GrblError as e:
            return False, str(e)
        finally:
            self._quiet -= 1

    def close(self) -> None:
        self._alive = False
        time.sleep(0.1)
        try:
            self._ser.close()
        except Exception:
            pass
