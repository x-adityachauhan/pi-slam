import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


def make_server(state, port=8090):
    page = (Path(__file__).parent / "index.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route = self.path.split("?", 1)[0]
            if route == "/":
                body, mime = page, "text/html; charset=utf-8"
            elif route == "/api":
                with state["lock"]:
                    body = json.dumps(state["map"], allow_nan=False).encode()
                mime = "application/json"
            elif route == "/frame.jpg":
                with state["lock"]:
                    body = state["jpeg"]
                mime = "image/jpeg"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)
