"""Replay recorded camera bytes with original timestamps; no camera required."""
import argparse
import csv
from pathlib import Path
import socket
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--port", type=int, default=8776)
    parser.add_argument("--speed", type=float, default=1.)
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("speed must be positive")
    with (args.run / "frames.csv").open() as source:
        rows = list(csv.DictReader(source))
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", args.port))
        server.listen(1)
        print(f"Replay ready on {args.port}: {len(rows)} frames", flush=True)
        with server.accept()[0] as client:
            start = time.monotonic()
            first = int(rows[0]["capture_timestamp_ns"])
            for row in rows:
                stamp = int(row["capture_timestamp_ns"])
                wait = (stamp-first)/1e9/args.speed - (time.monotonic()-start)
                if wait > 0:
                    time.sleep(wait)
                jpeg = (args.run / row["image"]).read_bytes()
                try:
                    client.sendall(struct.pack("!4sQQIHHI", b"PSL1", int(row["sequence"]),
                                               stamp, len(jpeg), 640, 480, 0x2000)+jpeg)
                except (BrokenPipeError, ConnectionResetError):
                    break
            time.sleep(.5)


if __name__ == "__main__":
    main()
