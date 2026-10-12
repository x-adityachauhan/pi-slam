import os
import signal
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from rc.session import target_settings, choose_server, stop_process


class SessionTests(unittest.TestCase):
    def test_target_override_does_not_reuse_old_alias(self):
        with tempfile.TemporaryDirectory() as tmp, patch("rc.session.ROOT", Path(tmp)):
            (Path(tmp)/".pi-target").write_text("ras@old old-key\n")
            with patch.dict(os.environ, {"PI_TARGET":"ras@new"}, clear=True):
                self.assertEqual(target_settings(), ("ras@new", ""))
            with patch.dict(os.environ, {}, clear=True):
                self.assertEqual(target_settings(), ("ras@old", "old-key"))

    def test_explicit_busy_port_preserves_owner(self):
        with tempfile.TemporaryDirectory() as tmp:
            server = choose_server(tmp, 0)
            try:
                with self.assertRaises(RuntimeError):
                    choose_server(tmp, server.server_port)
            finally:
                server.server_close()

    def test_cleanup_only_owned_process_group(self):
        process = subprocess.Popen(["sleep", "30"], start_new_session=True)
        try:
            stop_process(process, signal.SIGTERM)
            self.assertIsNotNone(process.poll())
            stop_process(process)
        finally:
            if process.poll() is None:
                process.kill(); process.wait()
