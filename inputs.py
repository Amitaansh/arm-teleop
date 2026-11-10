"""Where key presses come from: the Jetson's own keyboard, and remote clients.

Every source reports the set of keys it is holding. InputHub adds the sets together and
turns changes into ("down" | "up", key, source) events for the controller.

Remote clients (RemoteInput) talk one of two ways. Both carry the same thing, the full set of
keys held right now, by their evdev names (see config.py), about 20 times a second:

  Web page   GET  http://<jetson>:WEB_PORT/         the page (web_keys.html): type into it, or
                                                   use a gamepad plugged into that computer
             POST http://<jetson>:WEB_PORT/held     body {"id": "...", "name": "...",
                                                          "token": "...", "held": [...]}
                                                   reply {"status": "...", "log": [...]}
  UDP        datagram to <jetson>:NET_PORT        {"v": 1, "name": "joystick", "token": "...",
                                                    "held": ["KEY_UP", "KEY_2"]}
             teleop replies about 5 times a second with {"status": "...", "log": [...]}

Rules for both:
  - Send the set also when it is empty. It is state, not events, so a lost packet costs
    nothing.
  - If nothing comes from a client for NET_TIMEOUT_S, all its keys count as released (radio
    dropout, closed tab, crashed bridge). That is the deadman.
  - "token" is only checked if config.NET_TOKEN is set.
  - Unknown keys are ignored. KEY_ESC (quit) is ignored: only the Jetson's keyboard can quit.
"""
from __future__ import annotations

import json
import os
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import config as C

KNOWN_KEYS = set(C.MOTION_KEYS) | set(C.COMMAND_KEYS)
LOCAL_ONLY = {k for k, cmd in C.COMMAND_KEYS.items() if cmd == "quit"}
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_keys.html")


class RemoteInput:
    """Web page + UDP remote clients, with one shared client table."""

    def __init__(self, host: str, web_port=None, udp_port=None, token=None, timeout_s: float = 0.5,
                 log=print, status_fn=None):
        self.log, self.token, self.timeout_s = log, token, timeout_s
        self.status_fn = status_fn            # () -> (status_line, [log lines]) for replies
        self.events = queue.Queue()           # ("down" | "up", key, source) / ("connected", ...)
        self.clients = {}                     # client key -> {"name", "src", "held", "t", "reply_t"}
        self._lock = threading.Lock()
        self._alive = True
        self._warned = set()
        self.addresses = []
        self.sock = self.http = None
        if udp_port:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((host, udp_port))
            self.sock.settimeout(0.05)
            self.addresses.append(f"udp {host}:{udp_port}")
            threading.Thread(target=self._udp_loop, daemon=True).start()
        if web_port:
            try:
                self.http = _QuietServer((host, web_port), _handler(self))
            except OSError:
                self.close()
                raise
            self.http.daemon_threads = True
            shown = "localhost" if host in ("127.0.0.1", "0.0.0.0") else host
            self.addresses.append(f"http://{shown}:{web_port}")
            threading.Thread(target=self.http.serve_forever, kwargs={"poll_interval": 0.2},
                             daemon=True).start()
        threading.Thread(target=self._deadman_loop, daemon=True).start()

    @property
    def address(self) -> str:
        return ", ".join(self.addresses)

    # ------------------------------------------------------------- client table
    def held_keys(self) -> frozenset:
        with self._lock:
            out = set()
            for c in self.clients.values():
                out |= c["held"]
            return frozenset(out)

    def summary(self) -> list:
        """[(name, seconds since last message, held keys)] for the display."""
        now = time.monotonic()
        with self._lock:
            return [(c["name"], now - c["t"], sorted(c["held"])) for c in self.clients.values()]

    def _set_held(self, c, held):
        old = c["held"]
        for k in sorted(old - held):
            self.events.put(("up", k, c["src"]))
        for k in sorted(held - old):
            self.events.put(("down", k, c["src"]))
        c["held"] = held

    def update(self, key, name: str, src: str, msg: dict, peer: str):
        """One message from a client. Returns the reply dict, or None (rate-limited / refused).
        Raises ValueError for a malformed message."""
        held = msg.get("held", [])
        if not isinstance(held, list):
            raise ValueError("'held' must be a list")
        if self.token is not None and msg.get("token") != self.token:
            self._warn(peer, f"remote input: wrong token from {peer}, ignored")
            raise PermissionError("wrong token")
        keys = frozenset(k for k in held if isinstance(k, str) and k in KNOWN_KEYS and k not in LOCAL_ONLY)
        now = time.monotonic()
        with self._lock:
            c = self.clients.get(key)
            if c is None:
                c = self.clients[key] = {"name": name, "src": src, "held": frozenset(), "t": now,
                                         "reply_t": 0.0}
                self.events.put(("connected", f"remote {src}", "net"))
            c["t"] = now
            self._set_held(c, keys)
            reply = now - c["reply_t"] >= 0.2
            if reply:
                c["reply_t"] = now
        return self._reply() if reply else None

    def _reply(self):
        if self.status_fn is None:
            return {}
        try:
            line, logs = self.status_fn()
            return {"status": line, "log": logs[-C.LOG_LINES:]}
        except Exception:
            return {}

    def _deadman_loop(self):
        while self._alive:
            now = time.monotonic()
            with self._lock:
                for k, c in list(self.clients.items()):
                    if now - c["t"] > self.timeout_s:
                        self._set_held(c, frozenset())
                        del self.clients[k]
                        self.events.put(("disconnected", f"remote {c['src']} (nothing for "
                                         f"{self.timeout_s:g} s)", "net"))
            time.sleep(0.05)

    def _warn(self, peer, msg):
        if peer not in self._warned:
            self._warned.add(peer)
            self.log(msg)

    # ------------------------------------------------------------- UDP
    def _udp_loop(self):
        while self._alive:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._alive:
                    self.log("remote input: UDP socket error")
                return
            try:
                msg = json.loads(data.decode())
                if not isinstance(msg, dict):
                    raise ValueError("not a JSON object")
                name = str(msg.get("name", "udp"))[:24]
                out = self.update(("udp", addr), name, f"{name}@{addr[0]}", msg, addr[0])
            except PermissionError:
                continue
            except (ValueError, UnicodeDecodeError) as e:
                self._warn(addr[0], f"remote input: bad UDP packet from {addr[0]} ({e})")
                continue
            if out:
                try:
                    self.sock.sendto(json.dumps(out).encode()[:8000], addr)
                except OSError:
                    pass

    def close(self):
        self._alive = False
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        if self.http is not None:
            self.http.shutdown()
            self.http.server_close()


class _QuietServer(ThreadingHTTPServer):
    def handle_error(self, request, client_address):
        pass                               # a dropped browser connection must not print over the display


def _page_config() -> dict:
    return {
        "help": [[list(keys), text] for keys, text in C.KEY_HELP],
        "keys": sorted(KNOWN_KEYS - LOCAL_ONLY),
        "gamepad": {
            "deadzone": C.GAMEPAD_DEADZONE,
            "axes": {str(a): list(v) for a, v in C.GAMEPAD_AXES.items()},
            "buttons": {str(b): k for b, k in C.GAMEPAD_BUTTONS.items()},
        },
        "token_required": C.NET_TOKEN is not None,
        "send_ms": 50,
    }


def _handler(remote: RemoteInput):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):      # keep the terminal display clean
            pass

        def _send(self, code: int, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.split("?")[0] not in ("/", "/index.html"):
                self._send(404, b"not found", "text/plain")
                return
            try:
                with open(PAGE, encoding="utf-8") as f:
                    page = f.read()
            except OSError as e:
                self._send(500, f"web_keys.html missing: {e}".encode(), "text/plain")
                return
            page = page.replace("/*CONFIG*/null", json.dumps(_page_config()))
            self._send(200, page.encode(), "text/html; charset=utf-8")

        def do_POST(self):
            if self.path != "/held":
                self._send(404, b"not found", "text/plain")
                return
            try:
                n = int(self.headers.get("Content-Length", "0"))
                if not 0 <= n <= 4096:
                    raise ValueError("bad length")
                msg = json.loads(self.rfile.read(n).decode())
                if not isinstance(msg, dict):
                    raise ValueError("not a JSON object")
                cid = str(msg.get("id", ""))[:40] or self.client_address[0]
                name = str(msg.get("name", "browser"))[:24]
                peer = self.client_address[0]
                out = remote.update(("web", cid), name, f"{name}@{peer}", msg, peer)
            except PermissionError:
                self._send(403, b'{"error": "wrong token"}', "application/json")
                return
            except (ValueError, UnicodeDecodeError) as e:
                self._send(400, json.dumps({"error": str(e)}).encode(), "application/json")
                return
            self._send(200, json.dumps(out or {}).encode(), "application/json")

    return Handler


class InputHub:
    """The local keyboard (optional) plus remote input (optional), as one source."""

    def __init__(self, keyboard=None, remote: RemoteInput = None):
        self.keyboard, self.remote = keyboard, remote

    def held_keys(self) -> frozenset:
        held = set()
        if self.keyboard is not None and self.keyboard.connected:
            held |= self.keyboard.held_keys()
        if self.remote is not None:
            held |= self.remote.held_keys()
        return frozenset(held)

    def events(self):
        """Drain pending events: yields (kind, key_or_info, source)."""
        if self.keyboard is not None:
            while True:
                try:
                    kind, key = self.keyboard.events.get_nowait()
                except queue.Empty:
                    break
                yield kind, key, "keyboard"
        if self.remote is not None:
            while True:
                try:
                    yield self.remote.events.get_nowait()
                except queue.Empty:
                    break

    def describe(self) -> str:
        parts = []
        if self.keyboard is not None:
            parts.append(f"keyboard: {self.keyboard.name}" if self.keyboard.connected
                         else "keyboard: none")
        if self.remote is not None:
            clients = self.remote.summary()
            if clients:
                parts.append("remote: " + ", ".join(f"{n} ({age * 1000:.0f} ms)" for n, age, _ in clients))
            else:
                parts.append(f"remote: {self.remote.address}")
        return " | ".join(parts) or "no inputs"

    def close(self):
        if self.keyboard is not None:
            self.keyboard.close()
        if self.remote is not None:
            self.remote.close()
