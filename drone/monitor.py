#!/usr/bin/env python3
"""Watch an atomic latest-pose file, with a watchdog even if the stream stops."""
import argparse
import json
import time
from pathlib import Path
from drone.health import PoseGate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    gate = PoseGate()
    last_stamp = None
    latest = None
    last_report = None
    try:
        while True:
            try:
                event = json.loads((args.run/"latest_pose.json").read_text())
                if event.get("capture_timestamp_ns") != last_stamp:
                    latest = gate.check(event)
                    last_stamp = event.get("capture_timestamp_ns")
                elif (time.monotonic_ns()-int(event["result_monotonic_ns"]))/1e9 > gate.max_age:
                    latest = gate.check(event)
            except (OSError, ValueError, KeyError, TypeError):
                latest = {"flight_output_enabled": False, "research_gate_passed": False,
                          "reasons": ["pose_file_missing_or_invalid"]}
            report = json.dumps(latest, sort_keys=True)
            if report != last_report:
                print(report, flush=True)
                last_report = report
            time.sleep(.05)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
