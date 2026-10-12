import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.request import urlopen
from rc.live_view import make_server


class CPULiveTests(unittest.TestCase):
    def test_wait_live_and_stale_are_distinct(self):
        with tempfile.TemporaryDirectory() as temp:
            server = make_server(temp, 0)
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            url = f"http://127.0.0.1:{server.server_port}"
            try:
                def get():
                    with urlopen(url+"/api") as response:
                        return json.load(response)
                self.assertEqual(get()["points"], [])
                state = {"points": [[1,2,3]], "trajectory": [], "saved": False,
                         "updated_ns": time.monotonic_ns(), "status": "TRACKING"}
                p = Path(temp)/"live_map.json"
                p.write_text(json.dumps(state))
                self.assertFalse(get()["stale"])
                self.assertEqual(get()["points"], [[1,2,3]])
                state["updated_ns"] -= 10_000_000_000
                p.write_text(json.dumps(state))
                self.assertTrue(get()["stale"])
                with urlopen(url) as response:
                    self.assertIn(b'topdown', response.read())
            finally:
                server.shutdown()
                server.server_close()
                worker.join()
