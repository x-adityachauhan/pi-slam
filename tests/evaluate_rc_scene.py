#!/usr/bin/env python3
"""Surface error on procedural billboards; NOT real camera/metric accuracy."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rc.dense_reconstruct import read_ply_xyz

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("run", type=Path)
parser.add_argument("--frames", type=int, default=400)
parser.add_argument("--reconstruction", default="dense", help="Reconstruction subdirectory to evaluate")
parser.add_argument("--mesh", action="store_true", help="Evaluate triangle centroids rather than cloud vertices")
args = parser.parse_args()
poses = np.loadtxt(args.run/"map_0_keyframes.tum", comments="#")
phase = poses[:, 0]/((args.frames-1)*.05)*2*np.pi
truth = np.column_stack((.6*np.sin(phase), .13*np.sin(phase*2), .15*(1-np.cos(phase))))
x, y = poses[:, 1:4]-poses[:, 1:4].mean(axis=0), truth-truth.mean(axis=0)
U, singular, Vt = np.linalg.svd(y.T @ x / len(x))
D = np.eye(3)
D[2, 2] = np.linalg.det(U @ Vt)
rotation = U @ D @ Vt
scale = np.sum(singular*np.diag(D))/np.mean(np.sum(x*x, axis=1))
translation = truth.mean(axis=0)-scale*rotation @ poses[:, 1:4].mean(axis=0)
rng = np.random.default_rng(17)
centers = np.column_stack((rng.uniform(-2.7, 2.7, 220), rng.uniform(-1.9, 1.9, 220), rng.uniform(3.2, 7, 220)))
cloud = read_ply_xyz(args.run/args.reconstruction/"map_0_dense.ply")
if not len(cloud):
    raise SystemExit("No reconstructed points to evaluate")
if args.mesh:
    from rc.view_map import load_faces
    faces = load_faces(args.run/args.reconstruction/"map_0_mesh.ply", len(cloud))
    cloud = cloud[faces].mean(axis=1)
distances = []
for start in range(0, len(cloud), 2048):
    points = scale*cloud[start:start+2048] @ rotation.T+translation
    offset = np.abs(points[:, None, :]-centers[None, :, :])
    offset[:, :, :2] = np.maximum(offset[:, :, :2]-.12, 0)
    distances.extend(np.min(np.linalg.norm(offset, axis=2), axis=1))
distances = np.asarray(distances)
result = {"points": len(cloud), "alignment": "Sim(3), scale intentionally removed",
    "evaluated": "triangle_centroids" if args.mesh else "cloud_points",
    "physical_accuracy_measurement": False, "units": "procedural scene units",
    "median_nearest_surface_error": float(np.median(distances)),
    "p90_nearest_surface_error": float(np.percentile(distances, 90)),
    "fraction_within_0_1_scene_units": float(np.mean(distances < .1))}
print(json.dumps(result, indent=2))
