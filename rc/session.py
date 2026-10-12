"""One-command CPU session with interactive SSH authentication and owned cleanup."""
import argparse
import importlib.util
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rc.live_view import make_server


def target_settings():
    fields = (ROOT/".pi-target").read_text().split() if (ROOT/".pi-target").exists() else ["ras@10.32.62.210"]
    target = os.environ.get("PI_TARGET") or fields[0]
    alias = os.environ.get("PI_HOST_KEY_ALIAS", "" if os.environ.get("PI_TARGET") else (fields[1] if len(fields)>1 else ""))
    if target.startswith("-") or any(c.isspace() for c in target):
        raise ValueError("Invalid Pi SSH target")
    return target, alias


def stop_process(process, graceful=signal.SIGTERM):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, graceful)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def choose_server(run, port):
    for candidate in ([port] if port else [8093, 8095, 8096, 0]):
        try:
            return make_server(run, candidate)
        except OSError:
            if port:
                raise RuntimeError(f"Port {port} is busy; choose --port NUMBER or omit it")
    raise RuntimeError("Cannot start browser server")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=int, default=60)
    parser.add_argument("--no-depth", action="store_true")
    parser.add_argument("--no-open", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--exit-after-test", action="store_true", help="Close browser server when processing finishes")
    args = parser.parse_args()
    if args.seconds <= 0 or not 0 <= args.port <= 65535:
        parser.error("seconds must be positive and port must be 0..65535")
    for required in ("build/pi_slam_live", "calibration/camera.yaml", "calibration/intrinsics.json"):
        if not (ROOT/required).exists():
            raise RuntimeError(f"Missing {required}")
    if not args.no_depth:
        if not all((ROOT/"models/cpu"/name).exists() for name in ("midas-small.xml", "midas-small.bin")) or any(importlib.util.find_spec(m) is None for m in ("openvino", "skimage")):
            raise RuntimeError("CPU depth dependencies missing. Run bash rc/setup-depth.sh or use --no-depth")
    with socket.socket() as probe:
        try:
            probe.bind(("127.0.0.1", 8765))
        except OSError:
            raise RuntimeError("Camera port 8765 is already in use. Stop the previous camera launcher first.")
    target, alias = target_settings()
    (ROOT/"runs").mkdir(exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix=time.strftime("rc-%Y%m%d-%H%M%S-"), dir=ROOT/"runs"))
    server = choose_server(run, args.port)
    camera = tracker = depth = None
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
    with tempfile.TemporaryDirectory(prefix="pi-slam-ssh-") as control_dir:
        control = str(Path(control_dir)/"master")
        base = ["ssh", "-F", "/dev/null", "-S", control, "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3"]
        if alias:
            base += ["-o", f"HostKeyAlias={alias}"]
        try:
            print("Connecting to Pi. Enter its SSH password if prompted (not stored).", flush=True)
            # Foreground authentication, then only this private master owns the tunnel.
            subprocess.run(base+["-M", "-fNT", "-o", "ExitOnForwardFailure=yes", "-L", "127.0.0.1:8765:127.0.0.1:8765", target], check=True)
            if interrupted.is_set(): return 130
            with (run/"camera.log").open("w") as log:
                camera = subprocess.Popen(base+["-o", "BatchMode=yes", "-tt", target,
                    "exec python3 -u /home/ras/pi-slam/cpu_camera_server.py"], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+20
            while "V4L2 timestamps" not in (run/"camera.log").read_text():
                if interrupted.wait(.1): return 130
                if camera.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError(f"Pi camera did not start; inspect {run/'camera.log'}. Deploy with bash rc/start-camera.sh --deploy if needed.")
            threading.Thread(target=server.serve_forever, daemon=True).start()
            url = f"http://localhost:{server.server_port}"
            print(f"\nRun: {run}\nLive camera, 3D and top-down: {url}\nQ stops capture. Ctrl+C stops capture gracefully; press again after saving to close.\n", flush=True)
            if not args.no_open: webbrowser.open(url)
            env = dict(os.environ, CPU_SESSION_DIR=str(run))
            command = ["bash", str(ROOT/"rc/run.sh"), "--seconds", str(args.seconds)]
            if args.headless: command.append("--headless")
            tracker = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True)
            while not (run/"intrinsics.json").exists() and tracker.poll() is None:
                time.sleep(.05)
            if not args.no_depth and tracker.poll() is None:
                with (run/"depth.log").open("w") as log:
                    depth = subprocess.Popen(["bash", str(ROOT/"run-python.sh"), "rc/depth_worker.py", str(run)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            stop_requested = False
            while tracker.poll() is None:
                if interrupted.wait(.2):
                    interrupted.clear()
                    if not stop_requested:
                        os.killpg(tracker.pid, signal.SIGINT)
                        stop_requested = True
                        print("Saving capture and post-processing; please wait.", flush=True)
                if camera.poll() is not None and not stop_requested:
                    print("Camera process stopped; saving partial run.", flush=True)
                    os.killpg(tracker.pid, signal.SIGINT); stop_requested = True
                if depth is not None and depth.poll() is not None:
                    print(f"Depth worker stopped; see {run/'depth.log'}. Sparse SLAM continues.", flush=True)
                    depth = None
            stop_process(depth); depth = None
            stop_process(camera, signal.SIGINT); camera = None
            print(f"Test finished (status {tracker.returncode}). Files: {run}", flush=True)
            if args.exit_after_test: return tracker.returncode
            print(f"{url} now shows the last snapshot, labelled stopped/stale.", flush=True)
            print("Browser remains available. Press Ctrl+C to close the session.", flush=True)
            interrupted.clear()
            while not interrupted.wait(.5): pass
            return tracker.returncode
        finally:
            stop_process(tracker, signal.SIGINT)
            stop_process(depth)
            stop_process(camera, signal.SIGINT)
            subprocess.run(base+["-O", "exit", target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
            # shutdown only if serve_forever started (otherwise it would block).
            if 'url' in locals(): server.shutdown()
            server.server_close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, ValueError, subprocess.SubprocessError, OSError) as exc:
        print(f"SLAM session: {exc}", file=sys.stderr)
        sys.exit(1)
