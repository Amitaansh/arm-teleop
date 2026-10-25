"""Serial link to GRBL 1.1 (grbl-Mega-5X). The ONLY file that knows GRBL exists.

Swapping GRBL for custom firmware later means rewriting this file behind the same
interface: status, planner_total, reset_count, failed, jog(), jog_cancel(), unlock(), close().

Protocol notes (GRBL 1.1):
- Lines get 'ok' or 'error:N'. Character-counting flow control: unacknowledged bytes
  must never exceed GRBL's 128-byte RX buffer.
- Realtime bytes skip the buffer: '?' status, 0x85 jog cancel, '!' hold, 0x18 reset.
- Status needs $10=3 (MPos + buffer state):  <Jog|MPos:1.000,...|Bf:31,127|FS:...>
"""
from __future__ import annotations

import collections
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import serial

RT_STATUS = b"?"
RT_HOLD = b"!"
RT_RESUME = b"~"
RT_RESET = b"\x18"
RT_JOG_CANCEL = b"\x85"
RX_BUFFER = 127


class GrblError(RuntimeError):
    pass


@dataclass
class Status:
    state: str = "Unknown"                       # Idle, Jog, Run, Hold:0, Alarm, ...
    mpos: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)
    planner_free: Optional[int] = None
    rx_free: Optional[int] = None
    t: float = 0.0                               # time.monotonic() when received


class _Pending:
    __slots__ = ("line", "nbytes", "event", "result")

    def __init__(self, line: str, nbytes: int):
        self.line, self.nbytes = line, nbytes
        self.event = threading.Event()
        self.result: Optional[str] = None


class GrblLink:
    def __init__(self, port: str, baud: int = 115200, status_hz: float = 20.0,
                 letters=("X", "Y", "Z", "A"), log=print):
        self.letters = tuple(letters)
        self.n_axes = len(self.letters)
        self.log = log
        self.status = Status(mpos=(0.0,) * self.n_axes)
        self.planner_total = 0            # largest Bf planner count seen = buffer size
        self.reset_count = 0              # increments on every GRBL welcome banner
        self.last_error: Optional[Tuple[str, str]] = None
        self.error_count = 0
        self.failed: Optional[str] = None

        self._cv = threading.Condition()
        self._wlock = threading.Lock()
        self._pending = collections.deque()
        self._pending_bytes = 0
        self._capture: Optional[List[str]] = None
        self._welcome = threading.Event()
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
            self.log(f"GRBL {line}")
        elif self._capture is not None:
            self._capture.append(line)
        else:
            self.log(f"GRBL {line}")

    def _parse_status(self, line: str) -> None:
        fields = line[1:-1].split("|")
        mpos, pf, rf = self.status.mpos, None, None
        for f in fields[1:]:
            key, _, val = f.partition(":")
            if key == "MPos":
                vals = [float(x) for x in val.split(",")]
                vals += [0.0] * (self.n_axes - len(vals))
                mpos = tuple(vals[: self.n_axes])
            elif key == "Bf":
                a, _, b = val.partition(",")
                pf, rf = int(a), int(b)
                self.planner_total = max(self.planner_total, pf)
        self.status = Status(fields[0], mpos, pf, rf, time.monotonic())

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

    def close(self) -> None:
        self._alive = False
        time.sleep(0.1)
        try:
            self._ser.close()
        except Exception:
            pass
