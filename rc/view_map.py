#!/usr/bin/env python3
"""Serve one saved independent RC-car 3D map in a local browser."""
import argparse
import csv
import json
import signal
import sys
import threading
import webbrowser
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from viewer.server import make_server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--map", help="Independent map name, e.g. map_0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--no-open", action="store_true")
    args = parser.parse_args()
    if not (args.run/"dense/stats.json").is_file():
        raise SystemExit("No reconstructed map report. Complete a translated SLAM recording and reconstruction first.")
    report = json.loads((args.run/"dense/stats.json").read_text())
    maps = [entry for entry in report["maps"] if entry.get("points", 0)]
    if args.map:
        maps = [entry for entry in maps if entry["map"] == args.map]
    if not maps:
        raise SystemExit("No matching reconstructed map; inspect dense/stats.json")
    chosen = max(maps, key=lambda entry: entry["points"])
    path = (args.run/chosen["file"]).resolve()
    if not path.is_relative_to(args.run.resolve()):
        raise SystemExit("Map file escaped run directory")
    with path.open() as source:
        for skip, line in enumerate(source, 1):
            if line.strip() == "end_header":
                break
        else:
            raise SystemExit("Invalid PLY header")
    cloud = np.loadtxt(path, skiprows=skip, ndmin=2)
    step = max(1, int(np.ceil(len(cloud)/16000)))
    cloud = cloud[::step]
    poses_file = args.run/(chosen["map"]+"_keyframes.tum")
    if not poses_file.exists():
        poses_file = args.run/"trajectory.tum"
    poses = np.loadtxt(poses_file, comments="#", ndmin=2)
    with (args.run/"frames.csv").open() as source:
        last_image = list(csv.DictReader(source))[-1]["image"]
    image_path = (args.run/last_image).resolve()
    if not image_path.is_relative_to(args.run.resolve()):
        raise SystemExit("Image path escaped run directory")
    state = {"lock": threading.Lock(), "jpeg": image_path.read_bytes(), "map": {
        "status": f"Saved RC-car 3D surfaces — {chosen['map']} — relative scale",
        "saved": True, "total_points": chosen["points"], "metric_scale": False,
        "points": cloud[:, :3].tolist(), "colors": cloud[:, 3:6].astype(int).tolist(),
        "trajectory": poses[:, 1:4].tolist()}}
    server = make_server(state, args.port)
    # Shutdown must run off the serving thread to avoid HTTPServer's deadlock.
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown).start())
    print(f"http://127.0.0.1:{server.server_port} — one map only; Ctrl+C closes viewer", flush=True)
    if not args.no_open:
        webbrowser.open(f"http://127.0.0.1:{server.server_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
