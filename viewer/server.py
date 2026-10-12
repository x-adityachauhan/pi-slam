import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs


def make_server(state, port=8090):
    page = (Path(__file__).parent / "index.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            url = urlsplit(self.path)
            route = url.path
            selected = parse_qs(url.query).get("map", [state.get("default_map")])[0]
            if route in ("/api", "/frame.jpg", "/download") and "load_map" in state:
                try:
                    selected_map, selected_jpeg = state["load_map"](selected)
                except (ValueError, OSError):
                    self.send_error(404, "Unknown or unavailable map")
                    return
            else:
                selected_map, selected_jpeg = None, None
            if route == "/":
                body, mime = page, "text/html; charset=utf-8"
            elif route == "/viewer.js":
                body, mime = (Path(__file__).parent/"viewer.js").read_bytes(), "text/javascript; charset=utf-8"
            elif route == "/api":
                with state["lock"]:
                    body = json.dumps(selected_map if selected_map is not None else state["map"], allow_nan=False).encode()
                mime = "application/json"
            elif route == "/frame.jpg":
                with state["lock"]:
                    body = selected_jpeg if selected_jpeg is not None else state["jpeg"]
                mime = "image/jpeg"
            elif route == "/download" and selected in state.get("map_files", {}):
                body, mime = state["map_files"][selected].read_bytes(), "application/octet-stream"
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
