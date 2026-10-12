#!/usr/bin/env python3
"""View separate reconstructed maps with a calibrated reference camera."""
import argparse
import csv
from functools import lru_cache
import json
import signal
import sys
import threading
import webbrowser
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from viewer.server import make_server
from rc.geometry import pose_matrix


def safe_path(run, value):
    path = (run/value).resolve()
    if not path.is_relative_to(run.resolve()):
        raise ValueError("Map asset escaped run directory")
    return path


def load_cloud(path, maximum):
    with path.open() as source:
        for skip, line in enumerate(source, 1):
            if line.strip() == "end_header":
                break
        else:
            raise ValueError("Invalid PLY header")
    cloud = np.loadtxt(path, skiprows=skip, ndmin=2)
    if cloud.shape[1] == 3:
        cloud = np.column_stack((cloud, np.tile([120, 210, 240], (len(cloud), 1))))
    if cloud.shape[1] < 6 or not np.isfinite(cloud).all():
        raise ValueError("Expected finite XYZ/RGB map points")
    step = max(1, int(np.ceil(len(cloud)/maximum)))
    return cloud[::step]


def load_faces(path, vertex_count):
    with path.open() as source:
        for skip, line in enumerate(source, 1):
            if line.strip() == "end_header":
                break
        else:
            raise ValueError("Invalid mesh header")
    faces = np.loadtxt(path, skiprows=skip+vertex_count, dtype=np.int32, ndmin=2)
    if faces.shape[1] != 4 or not np.all(faces[:, 0] == 3):
        raise ValueError("Only triangular faces supported")
    faces = faces[:, 1:]
    if np.any(faces < 0) or np.any(faces >= vertex_count):
        raise ValueError("Invalid mesh vertex index")
    return faces


def make_state(run, reconstruction=None, selected=None, max_points=200000, sparse=False):
    run = Path(run).resolve()
    if max_points < 1:
        raise ValueError("Positive display point budget required")
    if reconstruction is None:
        reconstruction = "dense_v2" if (run/"dense_v2/stats.json").exists() else "dense"
    if sparse:
        entries = []
        for path in sorted(run.glob("map_*.ply")):
            with path.open() as source:
                count = 0
                for line in source:
                    if line.startswith("element vertex "):
                        count = int(line.split()[2])
                    if line.strip() == "end_header":
                        break
            if count:
                entries.append({"map": path.stem, "points": count, "file": path.name})
        report = {"maps": entries, "method": "ORB-SLAM3 sparse landmarks; not a dense room surface or verified metric map."}
    else:
        report = json.loads(safe_path(run, reconstruction+"/stats.json").read_text())
    maps = {e["map"]: e for e in report["maps"] if e.get("points", 0) and e.get("file")}
    if not maps:
        raise ValueError("No reconstructed maps; inspect the reconstruction report")
    selected = selected or max(maps, key=lambda name: maps[name]["points"])
    if selected not in maps:
        raise ValueError("Requested map is not in this reconstruction")
    calib_path = run/"intrinsics.json"
    calibration = json.loads(calib_path.read_text()) if calib_path.exists() else None

    @lru_cache(maxsize=3)
    def load(name):
        if name not in maps:
            raise ValueError("Unknown map")
        chosen = maps[name]
        cloud = load_cloud(safe_path(run, chosen["file"]), max_points)
        faces = []
        if chosen.get("mesh_file") and len(cloud) == chosen["points"]:
            faces = load_faces(safe_path(run, chosen["mesh_file"]), len(cloud)).tolist()
        poses_file = run/(name+"_keyframes.tum")
        if not poses_file.exists():
            poses_file = run/"trajectory.tum"
        poses = np.loadtxt(poses_file, comments="#", ndmin=2)
        reference = chosen.get("reference_view")
        if reference is None:
            with (run/"frames.csv").open() as source:
                rows = list(csv.DictReader(source))
            pose = poses[len(poses)//2]
            row = min(rows, key=lambda row: abs(float(row["time_s"])-pose[0]))
            reference = {"image": row["image"], "time_s": float(row["time_s"]),
                         "T_world_camera": pose_matrix(pose[1:]).tolist()}
        im = cv2.imread(str(safe_path(run, reference["image"])))
        if im is None:
            raise ValueError("Reference camera image is missing")
        if calibration:
            im = cv2.undistort(im, np.array(calibration["K"]), np.array(calibration["distortion"]))
        ok, jpeg = cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise ValueError("Cannot encode reference image")
        data = {"status": f"Saved {'sparse 3D map' if sparse else '3D reconstruction'} · {name} · relative scale",
            "sparse": sparse,
            "saved": True, "map_id": name, "total_points": chosen["points"], "metric_scale": False,
            "points": cloud[:, :3].tolist(), "colors": cloud[:, 3:6].astype(int).tolist(), "faces": faces,
            "trajectory": poses[:, 1:4].tolist(), "reference": reference,
            "camera": {k: calibration[k] for k in ("K", "width", "height")} if calibration else None,
            "maps": [{"id": key, "points": value["points"]} for key, value in maps.items()],
            "method": report["method"], "reference_label": f"Calibrated keyframe · {reference['time_s']:.2f} s",
            "quality": "Sparse tracking landmarks only" if sparse else "Depth verified in three views" if "min_consistent_other_views" in chosen else "Pairwise reconstruction"}
        return data, jpeg.tobytes()

    data, jpeg = load(selected)
    return {"lock": threading.Lock(), "map": data, "jpeg": jpeg,
            "load_map": load, "default_map": selected,
            "map_files": {name: safe_path(run, entry.get("mesh_file", entry["file"])) for name, entry in maps.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--map", help="Independent map name")
    parser.add_argument("--reconstruction", help="Output subdirectory; defaults to dense_v2, then dense")
    parser.add_argument("--max-points", type=int, default=200000)
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--no-open", action="store_true")
    parser.add_argument("--sparse", action="store_true", help="Show saved SLAM landmarks, not dense surfaces")
    args = parser.parse_args()
    try:
        state = make_state(args.run, args.reconstruction, args.map, args.max_points, sparse=args.sparse)
    except (ValueError, OSError) as exc:
        raise SystemExit(str(exc))
    server = make_server(state, args.port)
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown).start())
    print(f"http://127.0.0.1:{server.server_port} — Ctrl+C closes viewer", flush=True)
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
