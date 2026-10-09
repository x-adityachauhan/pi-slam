#!/usr/bin/env python3
"""Similarity-aligned position error for synthetic_stream.py (not real metres)."""
import argparse
import json
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("poses", type=Path)
parser.add_argument("--frames", type=int, default=240)
args = parser.parse_args()
poses = np.loadtxt(args.poses, comments="#")
phase = poses[:, 0] / ((args.frames-1)*.05) * 2*np.pi
truth = np.column_stack((.6*np.sin(phase), .13*np.sin(2*phase), .15*(1-np.cos(phase))))
estimate = poses[:, 1:4]
x, y = estimate-estimate.mean(axis=0), truth-truth.mean(axis=0)
U, singular, Vt = np.linalg.svd(y.T @ x / len(x))
D = np.eye(3)
D[2, 2] = np.linalg.det(U @ Vt)
rotation = U @ D @ Vt
scale = np.sum(singular*np.diag(D)) / np.mean(np.sum(x*x, axis=1))
aligned = scale*x @ rotation.T + truth.mean(axis=0)
rmse = np.sqrt(np.mean(np.sum((aligned-truth)**2, axis=1)))
diameter = np.sqrt(np.max(np.sum((truth[:, None]-truth[None, :])**2, axis=2)))
result = {"poses": len(poses), "position_rmse_synthetic_units": float(rmse),
          "trajectory_diameter_synthetic_units": float(diameter),
          "relative_position_rmse": float(rmse/diameter), "alignment": "Sim(3)",
          "physical_camera_accuracy_measurement": False}
print(json.dumps(result, indent=2))
if result["relative_position_rmse"] > .04:
    raise SystemExit("Synthetic position error exceeded the 4% smoke-test threshold")
