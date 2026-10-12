"""Opt-in compiled-runner regression: PI_SLAM_NATIVE_TEST=1 bash run-python.sh -m unittest discover -s tests -p test_native_stream.py."""
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time
import unittest

import cv2
import numpy as np

from calibrate import settings_text

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get("PI_SLAM_NATIVE_TEST") == "1", "requires compiled native runner")
class NativeStreamTests(unittest.TestCase):
    def test_fragment_timeout_and_disconnect_save_summary(self):
        with tempfile.TemporaryDirectory() as directory, socket.socket() as server:
            folder = Path(directory)
            config = folder / "camera.yaml"
            config.write_text(settings_text(np.array([[520., 0, 320], [0, 520., 240], [0, 0, 1]]),
                                           np.zeros(5), 640, 480))
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(40)
            errors = []

            def producer():
                try:
                    with server.accept()[0] as client:
                        client.settimeout(5)
                        _, jpeg = cv2.imencode(".jpg", np.full((480, 640, 3), 120, np.uint8))
                        for i in range(3):
                            packet = struct.pack("!4sQQIHHI", b"PSL1", i,
                                                 1_000_000_000+i*100_000_000,
                                                 len(jpeg), 640, 480, 0x2000)+jpeg.tobytes()
                            # Interrupt a header across the one-second recv timeout.
                            client.sendall(packet[:13])
                            if i == 1:
                                time.sleep(1.3)
                            client.sendall(packet[13:])
                            time.sleep(.2)
                except Exception as exc:
                    errors.append(exc)

            worker = threading.Thread(target=producer, daemon=True)
            worker.start()
            env = os.environ.copy()
            env["LD_LIBRARY_PATH"] = ":".join(str(ROOT / p) for p in (
                "env/lib", "vendor/ORB_SLAM3/lib", "vendor/ORB_SLAM3/Thirdparty/DBoW2/lib",
                "vendor/ORB_SLAM3/Thirdparty/g2o/lib"))
            result = subprocess.run([
                str(ROOT / "build/pi_slam_live"), str(ROOT / "vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
                str(config), str(folder / "run"), "--headless", "--pose-events", "--port",
                str(server.getsockname()[1])], env=env, capture_output=True, text=True, timeout=50)
            worker.join(timeout=5)
            self.assertFalse(errors)
            self.assertEqual(result.returncode, 3, result.stdout + result.stderr)
            stats = json.loads((folder / "run/stats.json").read_text())
            self.assertEqual(stats["processed_frames"], 3)
            self.assertTrue(stats["stream_failed"])
            self.assertFalse(stats["initialized"])
            self.assertEqual(stats["tracked_frames"], 0)
            self.assertIn("closed", (folder / "run/capture_error.txt").read_text())
            events = [json.loads(line) for line in (folder / "run/pose_events.jsonl").read_text().splitlines()]
            self.assertTrue(all("detected_features" in event for event in events))
