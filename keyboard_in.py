"""Keyboard input via Linux evdev: real key-down / key-up events, several keys at once.

Reads /dev/input directly, so it works without a GUI and regardless of window focus.
Needs read access to /dev/input/event*:  sudo usermod -aG input $USER  (then log out/in).
If the keyboard is unplugged, all keys count as released and it reconnects when plugged back.
"""
from __future__ import annotations

import queue
import select
import threading
import time

try:
    import evdev
    from evdev import ecodes
except ImportError:          # lets the rest of the package import on non-Linux machines
    evdev = None
    ecodes = None


def _key_name(code: int) -> str:
    name = ecodes.KEY.get(code, f"KEY_{code}")
    return name[0] if isinstance(name, (list, tuple)) else name


class Keyboard:
    def __init__(self, path=None, grab=True, log=print):
        if evdev is None:
            raise RuntimeError("python-evdev not installed (pip install evdev)")
        self.path, self.grab, self.log = path, grab, log
        self.events = queue.Queue()      # ("down"|"up", key) and ("connected"|"disconnected", info)
        self.connected = False
        self.name = None
        self._held = set()
        self._lock = threading.Lock()
        self._alive = True
        self._dev = None
        threading.Thread(target=self._run, daemon=True).start()

    def held_keys(self) -> frozenset:
        with self._lock:
            return frozenset(self._held)

    def _find(self):
        if self.path:
            return evdev.InputDevice(self.path)
        for p in evdev.list_devices():
            try:
                d = evdev.InputDevice(p)
            except OSError:
                continue
            keys = d.capabilities().get(ecodes.EV_KEY, [])
            if ecodes.KEY_A in keys and ecodes.KEY_SPACE in keys and ecodes.KEY_ESC in keys:
                return d
            d.close()
        return None

    def _release_all(self):
        with self._lock:
            released = list(self._held)
            self._held.clear()
        for k in released:
            self.events.put(("up", k))

    def _run(self):
        warned = False
        while self._alive:
            try:
                dev = self._find()
            except PermissionError:
                if not warned:
                    self.log("keyboard: permission denied on /dev/input. "
                             "Run: sudo usermod -aG input $USER, then log out and back in")
                    warned = True
                time.sleep(1.0)
                continue
            if dev is None:
                if not warned:
                    self.log("keyboard: no keyboard found, waiting...")
                    warned = True
                time.sleep(1.0)
                continue
            warned = False
            try:
                if self.grab:
                    dev.grab()
                self._dev = dev
                self.name = dev.name
                self.connected = True
                self.events.put(("connected", dev.name))
                while self._alive:
                    r, _, _ = select.select([dev.fd], [], [], 0.2)
                    if not r:
                        continue
                    for ev in dev.read():
                        if ev.type != ecodes.EV_KEY or ev.value == 2:   # 2 = autorepeat
                            continue
                        k = _key_name(ev.code)
                        with self._lock:
                            if ev.value == 1:
                                self._held.add(k)
                            else:
                                self._held.discard(k)
                        self.events.put(("down" if ev.value == 1 else "up", k))
            except OSError as e:
                self.connected = False
                self._release_all()
                self.events.put(("disconnected", str(e)))
                time.sleep(1.0)
            finally:
                self._dev = None

    def close(self):
        self._alive = False
        dev = self._dev
        if dev is not None:
            try:
                if self.grab:
                    dev.ungrab()
            except OSError:
                pass
