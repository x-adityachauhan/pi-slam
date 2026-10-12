"""Summarize actual delivery and tracking separately from tracking-call speed."""
import argparse
import json
from pathlib import Path
import numpy as np


def summarize(run):
    rows = [json.loads(line) for line in (Path(run)/"pose_events.jsonl").read_text().splitlines()]
    if not rows:
        return {"frames": 0}
    n = len(rows)
    capture = np.array([r["capture_timestamp_ns"] for r in rows], dtype=np.float64)/1e9
    result = np.array([r["result_monotonic_ns"] for r in rows], dtype=np.float64)/1e9
    return {"frames": n, "tracked_pct": 100*sum(r["tracking_state"] == 2 for r in rows)/n,
            "delivered_fps": (n-1)/max(result[-1]-result[0], 1e-9),
            "map_ids": sorted(set(r["map_id"] for r in rows)),
            "keypoints_mean": float(np.mean([r["detected_features"] for r in rows])),
            "capture_gap_max_ms": float(np.diff(capture).max()*1000) if n > 1 else 0,
            "receipt_to_result_p95_ms": float(np.percentile([r["receipt_to_result_ms"] for r in rows], 95)),
            "metric_scale": False, "end_to_end_latency_ms": None}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    print(json.dumps(summarize(parser.parse_args().run), indent=2))
