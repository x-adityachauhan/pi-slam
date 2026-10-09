"""Laptop receiver: keeps the newest frame, preserves the Pi capture clock."""
import socket
import struct
import threading
import time

import cv2
import numpy as np

HEADER = struct.Struct("!4sQQIHHI")


def read_exact(sock, size):
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("Pi camera stream disconnected")
        chunks.extend(chunk)
    return bytes(chunks)


class PiStream:
    def __init__(self, host="127.0.0.1", port=8765):
        self.socket = socket.create_connection((host, port), timeout=10)
        self.socket.settimeout(10)
        self.condition = threading.Condition()
        self.frame = None
        self.error = None
        self.closed = False
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def _receive(self):
        previous = -1
        try:
            while not self.closed:
                magic, sequence, stamp, size, width, height, flags = HEADER.unpack(
                    read_exact(self.socket, HEADER.size))
                if magic != b"PSL1" or not 0 < size <= 16 * 1024 * 1024:
                    raise ValueError("Invalid Pi stream header")
                if stamp <= previous or (flags & 0xE000) != 0x2000:
                    raise ValueError("Non-monotonic or unsupported capture timestamp")
                jpeg = read_exact(self.socket, size)
                image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None or image.shape[:2] != (height, width):
                    raise ValueError("Invalid camera frame")
                previous = stamp
                with self.condition:
                    self.frame = (sequence, stamp, image, jpeg, time.monotonic())
                    self.condition.notify_all()
        except Exception as exc:
            if not self.closed:
                with self.condition:
                    self.error = exc
                    self.condition.notify_all()

    def read(self, after=-1, timeout=10):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: self.error or (self.frame is not None and self.frame[0] != after), timeout)
            if self.error:
                raise self.error
            if not ready:
                raise TimeoutError("No new camera frame")
            return self.frame

    def close(self):
        self.closed = True
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()
        self.thread.join(timeout=2)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
