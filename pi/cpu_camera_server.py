#!/usr/bin/env python3
"""CPU capture profile; restore exposure/ISO controls when this process exits."""
import subprocess
import sys
import camera_server

CONTROLS = ("auto_exposure", "exposure_time_absolute", "iso_sensitivity_auto", "brightness")


def main():
    # Camera intrinsics and 640x480 crop remain unchanged. Manual 10 ms exposure
    # targets motion blur; ISO remains automatic. Check brightness before mapping.
    saved = {}
    for control in CONTROLS:
        value = subprocess.check_output(["v4l2-ctl", "-d", "/dev/video0", "--get-ctrl", control], text=True)
        saved[control] = int(value.rsplit(":", 1)[1].strip().split()[0])
    try:
        subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl",
                        "auto_exposure=1,exposure_time_absolute=100,iso_sensitivity_auto=1,brightness=50"], check=True)
        print("CPU profile: requested exposure 10 ms, auto ISO, brightness 50, JPEG quality 75", flush=True)
        print("Previous controls:", saved, flush=True)
        sys.argv = [sys.argv[0], "--width", "640", "--height", "480", "--fps", "20", "--quality", "75"]
        camera_server.main()
    finally:
        # Restore manual exposure value before restoring automatic mode.
        order = ("exposure_time_absolute", "iso_sensitivity_auto", "brightness", "auto_exposure")
        for control in order:
            subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--set-ctrl", f"{control}={saved[control]}"], check=False)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
