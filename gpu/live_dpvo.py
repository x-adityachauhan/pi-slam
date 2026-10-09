#!/usr/bin/env python3
"""Live DPV-SLAM frontend for the same timestamped Pi camera stream."""
import argparse
import json
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
import torch
from stream import PiStream
from viewer.server import make_server


def write_ply(path, points, colors):
    with path.open("w") as out:
        out.write(f"ply\nformat ascii 1.0\ncomment monocular arbitrary scale\nelement vertex {len(points)}\n"
                  "property float x\nproperty float y\nproperty float z\nproperty uchar red\n"
                  "property uchar green\nproperty uchar blue\nend_header\n")
        for point, color in zip(points, colors):
            out.write(" ".join(map(str, [*point.tolist(), *color.tolist()])) + "\n")


@torch.no_grad()
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--network", default="/opt/weights/dpvo.pth")
    parser.add_argument("--output", type=Path, default=Path("/data/runs"))
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--viewer-port", type=int, default=8090)
    parser.add_argument("--seconds", type=float, default=0)
    parser.add_argument("--buffer", type=int, default=2048)
    parser.add_argument("--record-frames", action="store_true", help="RC-car mode: record images and reconstruct surfaces after saving")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    if not torch.cuda.is_available():
        parser.error("This backend requires the RTX laptop's CUDA GPU")
    calibration = json.loads(args.calibration.read_text())
    if not calibration.get("validated"):
        parser.error("A validated calibration is required; use calibrate.py on the first laptop")
    width, height = calibration["width"], calibration["height"]
    K = np.asarray(calibration["K"], dtype=np.float64)
    d = np.asarray(calibration["distortion"], dtype=np.float64)
    map_x, map_y = cv2.initUndistortRectifyMap(K, d, None, K, (width, height), cv2.CV_32FC1)
    intrinsics = torch.tensor([K[0,0], K[1,1], K[0,2], K[1,2]], dtype=torch.float32, device="cuda")
    from dpvo.config import cfg
    from dpvo.dpvo import DPVO
    from dpvo.lietorch import SE3

    config = cfg.clone()
    config.BUFFER_SIZE = args.buffer
    config.PATCHES_PER_FRAME = 96
    config.OPTIMIZATION_WINDOW = 12
    config.REMOVAL_WINDOW = 22
    config.PATCH_LIFETIME = 13
    config.MIXED_PRECISION = True
    config.LOOP_CLOSURE = True
    config.MAX_EDGE_AGE = 512
    config.CLASSIC_LOOP_CLOSURE = False
    slam = DPVO(config, args.network, ht=height-height%16, wd=width-width%16, viz=False)
    args.output.mkdir(parents=True, exist_ok=True)
    run = args.output / (time.strftime("dpv-%Y%m%d-%H%M%S") + f"-{time.time_ns()%1000000:06d}")
    run.mkdir()
    (run / "intrinsics.json").write_text(json.dumps(calibration, indent=2))
    frame_index = None
    if args.record_frames:
        (run/"frames").mkdir()
        frame_index = (run/"frames.csv").open("w")
        frame_index.write("time_s,capture_timestamp_ns,sequence,image\n")
    state = {"lock": threading.Lock(), "jpeg": b"", "map": {"status": "Waiting for camera", "points": [], "trajectory": []}}
    server = make_server(state, args.viewer_port)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Live camera and 3D map: http://127.0.0.1:{args.viewer_port}")
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    processed, dropped, last, origin, elapsed_ms = 0, 0, -1, None, 0.0
    start = time.monotonic()
    reason = "stopped"
    failure = None
    try:
        with PiStream(port=args.port) as stream:
            while (not stopped.is_set() and (not args.max_frames or processed < args.max_frames)
                   and (not args.seconds or time.monotonic()-start < args.seconds)):
                sequence, stamp, image, jpeg, _ = stream.read(after=last)
                if image.shape[:2] != (height, width):
                    raise ValueError("Capture dimensions differ from calibration")
                if origin is None:
                    origin = stamp
                if last >= 0:
                    dropped += max(0, sequence-last-1)
                last = sequence
                if slam.n >= args.buffer-8:
                    reason = "keyframe capacity reached; start a new run or increase --buffer"
                    print(reason)
                    break
                if frame_index is not None:
                    name = f"frames/{sequence}.jpg"
                    (run/name).write_bytes(jpeg)
                    frame_index.write(f"{(stamp-origin)/1e9:.9f},{stamp},{sequence},{name}\n")
                image = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)
                image = image[:height-height%16, :width-width%16]
                tensor = torch.from_numpy(np.ascontiguousarray(image)).permute(2,0,1).cuda()
                before = time.monotonic()
                slam((stamp-origin)/1e9, tensor, intrinsics)
                torch.cuda.synchronize()
                elapsed_ms += (time.monotonic()-before)*1000
                processed += 1
                if processed % 10 == 0:
                    poses = SE3(slam.pg.poses_[:slam.n]).inv().data.cpu().numpy()
                    points = slam.pg.points_[:slam.m].cpu().numpy()
                    points = points[np.isfinite(points).all(axis=1)]
                    step = max(1, len(points)//4000)
                    status = "Tracking" if slam.is_initialized else "Move camera sideways to initialize"
                    snapshot = {"status": status, "points": points[::step].tolist(),
                        "trajectory": poses[:, :3].tolist(), "frames": processed, "skipped": dropped,
                        "mean_ms": round(elapsed_ms/processed, 1), "metric_scale": False}
                    with state["lock"]:
                        state["jpeg"], state["map"] = jpeg, snapshot
                    print(f"{status} | frames {processed} | {slam.n} keyframes | {snapshot['mean_ms']}ms")
    except Exception as exc:
        reason = str(exc)
        failure = exc
        print(reason)
    finally:
        server.shutdown()
        server.server_close()
        if frame_index is not None:
            frame_index.close()
    initialized = slam.is_initialized
    if initialized:
        poses, stamps = slam.terminate()
        np.savetxt(run / "trajectory.tum", np.column_stack((stamps, poses)), fmt="%.9f")
        points = slam.pg.points_[:slam.m].cpu().numpy()
        colors = slam.pg.colors_.view(-1,3)[:slam.m].cpu().numpy()
        finite = np.isfinite(points).all(axis=1)
        write_ply(run / "map.ply", points[finite], colors[finite])
    stats = {"frames": processed, "skipped_frames": dropped, "initialized": initialized,
             "mean_tracking_ms": elapsed_ms/max(1,processed), "metric_scale": False,
             "reason": reason, "capture_clock_origin_ns": origin,
             "configuration": config.dump(), "gpu": torch.cuda.get_device_name()}
    (run / "stats.json").write_text(json.dumps(stats, indent=2))
    print(f"Saved session: {run}")
    if failure is not None:
        raise SystemExit(f"Session ended with an error: {reason}")
    if not initialized:
        raise SystemExit("No map initialized. Use translation through a textured, well-lit static scene.")
    if args.record_frames:
        from rc.dense_reconstruct import reconstruct
        print("Reconstructing RC-car colored 3D surfaces from final poses...")
        try:
            reconstruct(run, run/"intrinsics.json")
        except (ValueError, OSError) as exc:
            raise SystemExit(f"SLAM saved, but surface reconstruction failed: {exc}")


if __name__ == "__main__":
    main()
