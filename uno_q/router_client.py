"""Minimal MessagePack-RPC client for the UNO Q's arduino-router unix socket.

The reflashed image has the router daemon (/usr/bin/arduino-router
--unix-port /var/run/arduino-router.sock) but NOT the App Lab python runtime
(`arduino.app_utils` is not installed anywhere on the board, checked
2026-08-14), so this file is the Linux side of the Bridge, written against
the documented protocol rather than the missing library.

Wire protocol (arduino-router repo, github.com/arduino/arduino-router):
    REQUEST  [0, msgid, "method", [params]]
    RESPONSE [1, msgid, error, result]      error is None on success
    NOTIFY   [2, "method", [params]]
The router forwards a request to whichever client registered the method
(the MCU sketch's Bridge.provide() does that registration) and routes the
response back. We only CALL, never provide, so registration is not
implemented here.

Dependency: msgpack (`~/venv/bin/pip install msgpack`). Everything else is
stdlib.
"""

import itertools
import socket

import msgpack

SOCKET_PATH = "/var/run/arduino-router.sock"

MSG_REQUEST = 0
MSG_RESPONSE = 1
MSG_NOTIFY = 2


class RouterError(RuntimeError):
    """The router or the remote method returned an error."""


class RouterClient:
    def __init__(self, path=SOCKET_PATH, timeout=2.0):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect(path)
        self._unpacker = msgpack.Unpacker(raw=False)
        self._msgid = itertools.count(1)

    def call(self, method, *params, timeout=2.0):
        """Call a method, block for its response, return the result.

        Raises RouterError on an error response, a closed socket, or an
        unparseable reply. NOTIFY messages and stray responses that arrive
        while waiting are discarded: nothing in this project subscribes to
        notifications, and a stray response can only come from a msgid this
        client never issued.
        """
        mid = next(self._msgid)
        self.sock.settimeout(timeout)
        self.sock.sendall(msgpack.packb([MSG_REQUEST, mid, method,
                                         list(params)]))
        while True:
            for msg in self._unpacker:
                if (isinstance(msg, (list, tuple)) and len(msg) == 4
                        and msg[0] == MSG_RESPONSE and msg[1] == mid):
                    err, result = msg[2], msg[3]
                    if err is not None:
                        raise RouterError(f"{method}: {err!r}")
                    return result
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                raise RouterError(
                    f"{method}: no response in {timeout}s (is the shovel "
                    f"sketch flashed and did Bridge.provide run?)") from None
            if not data:
                raise RouterError("router closed the socket")
            self._unpacker.feed(data)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
