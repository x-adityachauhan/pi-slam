#!/usr/bin/env python3
"""Reconstruct colored 3D surfaces from recorded views and final SLAM poses.

No learned depth, fabricated scale, vehicle commands, or free-space guarantees.
Each independent ORB map is reconstructed separately after loop optimization.
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rc.cloud import Cloud, write_ply
from rc.geometry import pose_matrix, reconstruct_pair


def read_ply_xyz(path):
    if not path.exists():
        return np.empty((0, 3))
    with path.open() as source:
        for lines, line in enumerate(source, 1):
            if line.strip() == "end_header":
                break
        else:
            raise ValueError("Invalid PLY header")
    with np.errstate(all="ignore"):
        return np.loadtxt(path, skiprows=lines, usecols=(0, 1, 2), ndmin=2)


def load_views(run, poses_file):
    index = {}
    with (run/"frames.csv").open() as source:
        for row in csv.DictReader(source):
            stamp = round(float(row["time_s"])*1e9)
            image = (run/row["image"]).resolve()
            if not image.is_relative_to(run.resolve()):
                raise ValueError("Recorded image path escaped the run directory")
            index[stamp] = image
    poses = np.loadtxt(poses_file, comments="#", ndmin=2)
    if poses.size == 0 or poses.shape[1] != 8 or not np.isfinite(poses).all():
        raise ValueError("Expected timestamp x y z qx qy qz qw poses")
    views = []
    for row in poses:
        stamp = round(row[0]*1e9)
        image = index.get(stamp)
        if image is None:
            continue
        views.append({"time": row[0], "image": image, "T": pose_matrix(row[1:])})
    views.sort(key=lambda item: item["time"])
    # DPVO returns every processed pose; bound pair-search cost by time sampling.
    selected = []
    for view in views:
        if not selected or view["time"]-selected[-1]["time"] >= .15:
            selected.append(view)
    return selected


def scene_depth(points, T):
    if len(points):
        camera = (points-T[:3, 3]) @ T[:3, :3]
        depth = camera[:, 2]
        depth = depth[np.isfinite(depth) & (depth > 0)]
        if len(depth) >= 10:
            return float(np.median(depth))
    raise ValueError("Not enough sparse landmarks to choose map-scale depth bounds")


def candidate_pairs(views, depth):
    used = set()
    for i, first in enumerate(views):
        candidates = []
        for j in range(i+1, min(i+35, len(views))):
            second = views[j]
            baseline = np.linalg.norm(first["T"][:3, 3]-second["T"][:3, 3])/depth
            angle = np.arccos(np.clip((np.trace(first["T"][:3, :3].T @
                                              second["T"][:3, :3])-1)/2, -1, 1))
            if .02 <= baseline <= .25 and angle <= np.deg2rad(25):
                relative_t = first["T"][:3, :3].T @ (second["T"][:3, 3]-first["T"][:3, 3])
                lateral = abs(relative_t[0])/max(np.linalg.norm(relative_t), 1e-8)
                candidates.append((abs(baseline-.08)+(1-lateral)*.06, j))
        for _, j in sorted(candidates)[:2]:
            if (i, j) not in used:
                used.add((i, j))
                yield i, j


def reconstruct_pairwise(run, calibration_path, max_pairs=40, voxel_size=None, disparities=128, min_support=2):
    if max_pairs < 1 or min_support < 1:
        raise ValueError("Positive pair count and support threshold required")
    calibration = json.loads(calibration_path.read_text())
    if calibration.get("validated") is not True:
        raise ValueError("Validated intrinsics required")
    K = np.asarray(calibration["K"], dtype=np.float64)
    distortion = np.asarray(calibration["distortion"], dtype=np.float64)
    files = sorted(run.glob("map_*_keyframes.tum"))
    if not files and (run/"trajectory.tum").exists():
        files = [run/"trajectory.tum"]
    if not files:
        raise ValueError("No optimized map poses: SLAM may not have initialized. "
                         "Translate through a bright textured scene; recorded images are preserved.")
    if not (run/"frames.csv").exists():
        raise ValueError("No recorded frames. Use rc/run.sh for a new session")
    destination = run/"dense"
    destination.mkdir(exist_ok=True)
    report = {"metric_scale": False, "flight_ready": False, "vehicle_control": False,
              "method": "calibrated multi-view SGBM, left-right consistency, voxel fusion",
              "poses": "final optimized per-map poses", "maps": []}
    for poses_file in files:
        started = time.monotonic()
        name = poses_file.name.replace("_keyframes.tum", "").replace("trajectory.tum", "map")
        entry = {"map": name, "attempted_pairs": 0, "successful_pairs": 0, "points": 0,
                 "rejected": [], "independent_coordinate_system": True}
        report["maps"].append(entry)
        try:
            views = load_views(run, poses_file)
            if len(views) < 2:
                raise ValueError("Fewer than two recorded optimized views")
            depth = scene_depth(read_ply_xyz(run/(name+".ply")), views[0]["T"])
            cloud = Cloud(voxel_size if voxel_size is not None else depth/200)
            entry["voxel_size_map_units"] = cloud.voxel_size
            for i, j in candidate_pairs(views, depth):
                if entry["attempted_pairs"] >= max_pairs:
                    break
                entry["attempted_pairs"] += 1
                try:
                    images = [cv2.imread(str(views[k]["image"])) for k in (i, j)]
                    if any(image is None for image in images):
                        raise ValueError("Recorded image is missing")
                    if images[0].shape[:2] != (calibration["height"], calibration["width"]):
                        raise ValueError("Recorded dimensions differ from calibration")
                    points, colors, _ = reconstruct_pair(*images, K, distortion,
                        views[i]["T"], views[j]["T"], disparities=disparities, depth_limit=depth*8)
                    if len(points) < 100:
                        raise ValueError("Insufficient consistent textured correspondences")
                    cloud.add(points, colors)
                    entry["successful_pairs"] += 1
                except (ValueError, cv2.error) as exc:
                    entry["rejected"].append(str(exc))
                print(f"{name}: pair {entry['attempted_pairs']}/{max_pairs}, "
                      f"{len(cloud.cells)} fused surface voxels", flush=True)
            points, colors = cloud.arrays(min_support)
            entry.update(points=len(points), capacity_dropped_samples=cloud.dropped,
                         min_pair_support=min_support, candidate_voxels=len(cloud.cells))
            if not len(points):
                raise ValueError("No dense reconstruction: use translation, texture and overlapping views")
            output = destination/(name+"_dense.ply")
            write_ply(output, points, colors)
            entry["file"] = str(output.relative_to(run))
        except (ValueError, OSError, cv2.error) as exc:
            entry["error"] = str(exc)
        entry["reconstruction_seconds"] = time.monotonic()-started
    (destination/"stats.json").write_text(json.dumps(report, indent=2))
    if not any(entry["points"] for entry in report["maps"]):
        raise ValueError("No map produced dense surfaces; see dense/stats.json")
    return report


def reconstruct(run, calibration_path, max_pairs=40, voxel_size=None, disparities=128,
                min_support=2, method="multiview", output_name="dense_v2"):
    if method == "pairwise":
        return reconstruct_pairwise(run, calibration_path, max_pairs, voxel_size, disparities, min_support)
    if method != "multiview":
        raise ValueError("Unknown reconstruction method")
    from rc.multiview import reconstruct_multiview
    return reconstruct_multiview(run, calibration_path, max_pairs, voxel_size,
                                 disparities, min_support, output_name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--calibration", type=Path, default=Path("calibration/intrinsics.json"))
    parser.add_argument("--max-pairs", type=int, default=40)
    parser.add_argument("--voxel-size", type=float)
    parser.add_argument("--disparities", type=int, default=128)
    parser.add_argument("--min-support", type=int, default=2, help="Minimum other views agreeing with each depth pixel (multiview)")
    parser.add_argument("--method", choices=("multiview", "pairwise"), default="multiview")
    parser.add_argument("--output-name", default="dense_v2", help="Multiview output subdirectory; preserves the earlier dense output")
    parser.add_argument("--threads", type=int, default=2, help="OpenCV CPU thread budget")
    args = parser.parse_args()
    if args.max_pairs < 1 or args.min_support < 1 or args.threads < 1:
        parser.error("Pair, support and thread counts must be positive")
    cv2.setNumThreads(args.threads)
    try:
        print(json.dumps(reconstruct(args.run, args.calibration, args.max_pairs,
                                     args.voxel_size, args.disparities, args.min_support,
                                     args.method, args.output_name), indent=2))
    except (ValueError, OSError) as exc:
        raise SystemExit(str(exc))


if __name__ == "__main__":
    main()
