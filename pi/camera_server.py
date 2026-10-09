#!/usr/bin/env python3
"""Stream hardware-encoded JPEG frames and V4L2 timestamps over a local socket.

No third-party Python packages are required on the Pi. Use an SSH tunnel to
reach the loopback-only listener. Supports 64-bit Linux single-planar V4L2.
"""
import argparse
import ctypes as C
import errno
import fcntl
import mmap
import os
import select
import signal
import socket
import struct
import subprocess
import threading

U32 = C.c_uint32


class Timeval(C.Structure):
    _fields_ = [("sec", C.c_long), ("usec", C.c_long)]


class Timecode(C.Structure):
    _fields_ = [("type", U32), ("flags", U32), ("parts", C.c_uint8 * 8)]


class BufferMemory(C.Union):
    _fields_ = [("offset", U32), ("userptr", C.c_ulong), ("planes", C.c_void_p), ("fd", C.c_int)]


class Buffer(C.Structure):
    _fields_ = [("index", U32), ("type", U32), ("bytesused", U32),
                ("flags", U32), ("field", U32), ("timestamp", Timeval),
                ("timecode", Timecode), ("sequence", U32), ("memory", U32),
                ("m", BufferMemory), ("length", U32), ("reserved2", U32),
                ("request_fd", C.c_int)]


class RequestBuffers(C.Structure):
    _fields_ = [("count", U32), ("type", U32), ("memory", U32),
                ("capabilities", U32), ("flags", C.c_uint8), ("reserved", C.c_uint8 * 3)]


class PixFormat(C.Structure):
    _fields_ = [(name, U32) for name in (
        "width", "height", "pixelformat", "field", "bytesperline", "sizeimage",
        "colorspace", "priv", "flags", "ycbcr_enc", "quantization", "xfer_func")]


class FormatUnion(C.Union):
    _fields_ = [("pix", PixFormat), ("raw", C.c_uint8 * 200), ("alignment", C.c_uint64)]


class Format(C.Structure):
    _fields_ = [("type", U32), ("fmt", FormatUnion)]


def ioctl(fd, number, arg, direction=3):
    request = (direction << 30) | (C.sizeof(arg) << 16) | (ord("V") << 8) | number
    data = bytearray(C.string_at(C.addressof(arg), C.sizeof(arg)))
    fcntl.ioctl(fd, request, data, True)
    C.memmove(C.addressof(arg), bytes(data), len(data))


class Camera:
    def __init__(self, device, width, height, fps, quality):
        if C.sizeof(Buffer) != 88 or C.sizeof(Format) != 208:
            raise RuntimeError("This capture helper requires a 64-bit Linux ABI")
        self.fd = -1
        self.buffers = []
        self.streaming = False
        # Configure with v4l-utils, already present on this Pi.
        subprocess.run(["v4l2-ctl", "-d", device, "--set-parm", str(fps),
                        "--set-ctrl", f"compression_quality={quality},exposure_dynamic_framerate=0"],
                       check=True)
        try:
            self.fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
            fmt = Format(type=1)
            fmt.fmt.pix.width, fmt.fmt.pix.height = width, height
            fmt.fmt.pix.pixelformat = int.from_bytes(b"MJPG", "little")
            fmt.fmt.pix.field = 1
            ioctl(self.fd, 5, fmt)  # VIDIOC_S_FMT
            self.width, self.height = fmt.fmt.pix.width, fmt.fmt.pix.height
            if fmt.fmt.pix.pixelformat != int.from_bytes(b"MJPG", "little"):
                raise RuntimeError("Camera did not accept Motion-JPEG capture")
            req = RequestBuffers(count=3, type=1, memory=1)
            ioctl(self.fd, 8, req)
            if req.count < 2:
                raise RuntimeError("Camera did not allocate sufficient capture buffers")
            for index in range(req.count):
                buf = Buffer(index=index, type=1, memory=1)
                ioctl(self.fd, 9, buf)
                self.buffers.append(mmap.mmap(self.fd, buf.length,
                    flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset))
                ioctl(self.fd, 15, buf)
            ioctl(self.fd, 18, U32(1), direction=1)
            self.streaming = True
        except BaseException:
            self.close()
            raise

    def read(self):
        if not select.select([self.fd], [], [], 5)[0]:
            raise TimeoutError("No camera frame for five seconds")
        buf = Buffer(type=1, memory=1)
        try:
            ioctl(self.fd, 17, buf)
        except OSError as exc:
            if exc.errno == errno.EAGAIN:
                return None
            raise
        try:
            if buf.flags & 0x40:  # V4L2_BUF_FLAG_ERROR
                return None
            timestamp_ns = buf.timestamp.sec * 1_000_000_000 + buf.timestamp.usec * 1000
            if timestamp_ns <= 0 or (buf.flags & 0xE000) != 0x2000:
                raise RuntimeError("Driver did not provide a valid monotonic capture timestamp")
            if not 0 < buf.bytesused <= len(self.buffers[buf.index]):
                raise RuntimeError("Invalid V4L2 frame size")
            jpeg = self.buffers[buf.index][:buf.bytesused]
            if not jpeg.startswith(b"\xff\xd8"):
                raise RuntimeError("Camera returned invalid JPEG data")
            return (buf.sequence, timestamp_ns, jpeg, buf.flags)
        finally:
            ioctl(self.fd, 15, buf)

    def close(self):
        if self.streaming:
            ioctl(self.fd, 19, U32(1), direction=1)
            self.streaming = False
        for buffer in self.buffers:
            buffer.close()
        self.buffers.clear()
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--quality", type=int, default=90)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    camera = Camera(args.device, args.width, args.height, args.fps, args.quality)
    latest = [None]
    errors = []
    condition = threading.Condition()
    stop = threading.Event()

    def capture():
        try:
            while not stop.is_set():
                frame = camera.read()
                if frame is not None:
                    with condition:
                        latest[0] = frame
                        condition.notify_all()
        except Exception as exc:
            with condition:
                errors.append(exc)
                condition.notify_all()

    thread = threading.Thread(target=capture, daemon=True)
    thread.start()
    try:
        with socket.socket() as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", args.port))
            listener.listen(2)
            listener.settimeout(1)
            print(f"Camera {camera.width}x{camera.height}; localhost:{args.port}; V4L2 timestamps", flush=True)
            while not errors:
                try:
                    client, _ = listener.accept()
                except socket.timeout:
                    continue
                with client:
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    client.settimeout(2)
                    previous = -1
                    try:
                        while True:
                            with condition:
                                condition.wait_for(lambda: errors or (
                                    latest[0] is not None and latest[0][0] != previous), timeout=5)
                                if errors:
                                    raise errors[0]
                                frame = latest[0]
                            if frame is None or frame[0] == previous:
                                raise TimeoutError("Capture stopped")
                            sequence, stamp, jpeg, flags = frame
                            header = struct.pack("!4sQQIHHI", b"PSL1", sequence, stamp,
                                                 len(jpeg), camera.width, camera.height, flags)
                            client.sendall(header + jpeg)
                            previous = sequence
                    except (BrokenPipeError, ConnectionResetError, socket.timeout):
                        pass
            if errors:
                raise errors[0]
    finally:
        stop.set()
        thread.join(timeout=6)
        camera.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
