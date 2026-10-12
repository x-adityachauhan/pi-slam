"""CPU-only live snapshot server. Never substitutes an older run's map."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import time
import webbrowser


def make_server(run, port):
    run = Path(run)
    assets = Path(__file__).parent/"web"

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route = self.path.split("?", 1)[0]
            try:
                if route == "/api":
                    try:
                        state = json.loads((run/"live_map.json").read_text())
                        age = (time.monotonic_ns()-state["updated_ns"])/1e9
                        state["stale"] = age > 3
                        if state["stale"]:
                            state["status"] = "STOPPED / STALE · last live snapshot · relative scale"
                    except FileNotFoundError:
                        state = {"points": [], "trajectory": [], "saved": False,
                                 "status": "Waiting for this CPU run to publish frames", "stale": True}
                    body, mime = json.dumps(state, allow_nan=False).encode(), "application/json"
                elif route == "/frame.jpg":
                    body, mime = (run/"live_frame.jpg").read_bytes(), "image/jpeg"
                elif route in ("/", "/viewer.js"):
                    body = (assets/("index.html" if route == "/" else "viewer.js")).read_bytes()
                    mime = "text/html" if route == "/" else "text/javascript"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            except (OSError, ValueError):
                self.send_error(503, "Snapshot unavailable")

        def log_message(self, *_):
            pass

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--port", type=int, default=8093)
    parser.add_argument("--no-open", action="store_true")
    args = parser.parse_args()
    server = make_server(args.run, args.port)
    print(f"CPU live map: http://localhost:{server.server_port}", flush=True)
    if not args.no_open:
        webbrowser.open(f"http://localhost:{server.server_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
