#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

import cv2
from stream import PiStream


def main():
    parser = argparse.ArgumentParser(description="Preview or verify the live Pi camera")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--snapshot", type=Path)
    args = parser.parse_args()
    stamps, sequences, sizes = [], [], []
    start = time.monotonic()
    with PiStream(port=args.port) as stream:
        sequence = -1
        while not args.frames or len(stamps) < args.frames:
            sequence, stamp, image, jpeg, received = stream.read(after=sequence)
            stamps.append(stamp)
            sequences.append(sequence)
            sizes.append(len(jpeg))
            if args.snapshot and len(stamps) == 1:
                args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                args.snapshot.write_bytes(jpeg)
            if not args.headless:
                display = image.copy()
                fps = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0
                cv2.putText(display, f"Pi camera | {fps:.1f} fps | Q: quit", (12, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("Pi camera", display)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    cv2.destroyAllWindows()
    elapsed = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0
    print(json.dumps({"frames": len(stamps), "dimensions": list(image.shape[:2][::-1]),
        "capture_fps": (len(stamps) - 1) / elapsed if elapsed else 0,
        "sequence_gaps": sum(max(0, b-a-1) for a,b in zip(sequences, sequences[1:])),
        "mean_jpeg_bytes": sum(sizes) / len(sizes),
        "timestamp_source": "Pi V4L2 monotonic", "wall_seconds": time.monotonic()-start}, indent=2))


if __name__ == "__main__":
    main()
