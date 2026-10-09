import importlib.util
import json
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from stream import PiStream, HEADER
from calibrate import solve, settings_text
from viewer.server import make_server


class PipelineTests(unittest.TestCase):
    def test_orb_settings_have_required_numeric_types(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"camera.yaml"
            path.write_text(settings_text(np.array([[530.,0,320],[0,525.,240],[0,0,1]]),np.zeros(5),640,480))
            settings=cv2.FileStorage(str(path),cv2.FILE_STORAGE_READ)
            self.assertTrue(settings.getNode("Camera.fps").isInt())
            self.assertTrue(settings.getNode("Camera1.fx").isReal())
            self.assertEqual(settings.getNode("Camera.RGB").real(),0)
            settings.release()

    def test_fragmented_stream_preserves_capture_timestamp_and_pixels(self):
        image = np.full((48, 64, 3), (30, 100, 160), np.uint8)
        _, jpeg = cv2.imencode(".jpg", image)
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        port = listener.getsockname()[1]
        finish = threading.Event()

        def send():
            with listener, listener.accept()[0] as client:
                payload = HEADER.pack(b"PSL1", 7, 123456789000, len(jpeg), 64, 48, 0x2000) + jpeg.tobytes()
                for begin in range(0, len(payload), 19):
                    client.sendall(payload[begin:begin+19])
                finish.wait(2)

        worker = threading.Thread(target=send)
        worker.start()
        try:
            with PiStream(port=port) as stream:
                sequence, stamp, decoded, _, _ = stream.read()
                self.assertEqual((sequence, stamp), (7, 123456789000))
                self.assertEqual(decoded.shape, image.shape)
                self.assertLess(np.abs(decoded.astype(float)-image).mean(), 3)
        finally:
            finish.set()
            worker.join()

    def test_non_monotonic_timestamp_is_rejected(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        _, jpeg = cv2.imencode(".jpg", np.zeros((48,64,3), np.uint8))

        def send():
            with listener, listener.accept()[0] as client:
                for i in range(2):
                    client.sendall(HEADER.pack(b"PSL1", i, 1000000000, len(jpeg),64,48,0x2000)+jpeg.tobytes())
                time.sleep(0.1)

        worker = threading.Thread(target=send)
        worker.start()
        with PiStream(port=listener.getsockname()[1]) as stream:
            time.sleep(0.2)
            with self.assertRaisesRegex(ValueError, "timestamp"):
                stream.read()
        worker.join()

    def test_v4l2_native_layout(self):
        import ctypes
        spec = importlib.util.spec_from_file_location("camera_server", ROOT/"pi/camera_server.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(ctypes.sizeof(module.Buffer), 88)
        self.assertEqual(module.Buffer.timestamp.offset, 24)
        self.assertEqual(ctypes.sizeof(module.Format), 208)

    def test_calibration_rejects_missing_views(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, "25"):
                solve(Path(folder), Path(folder)/"out", (9,6), 20, 0.8)

    def test_calibration_recovers_known_synthetic_intrinsics(self):
        K = np.array([[530.,0,320.],[0,525.,240.],[0,0,1.]])
        board = np.full((700,1000,3), 255, np.uint8)
        for y in range(7):
            for x in range(10):
                if (x+y)%2 == 0:
                    board[y*100:(y+1)*100,x*100:(x+1)*100] = 0
        outer = np.array([[-20.,-20,0],[180,-20,0],[180,120,0],[-20,120,0]])
        source = np.array([[0,0],[1000,0],[1000,700],[0,700]], np.float32)
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            for i in range(40):
                row, col = (i//5)%2, i%5
                rotation = np.array([(-1 if i%2 else 1)*(.12+.04*(i%4)), .22*np.sin(i), .06*np.cos(i)])
                R,_ = cv2.Rodrigues(rotation)
                depth = 480.+35*(i%4)
                center = np.array([195.+55*col, 140.+200*row])
                xy = (center-np.array([320,240]))*depth/np.array([530,525])
                translation = np.array([*xy,depth])-R@np.array([80.,50.,0])
                projected,_=cv2.projectPoints(outer,rotation,translation,K,None)
                H=cv2.getPerspectiveTransform(source,projected.reshape(4,2).astype(np.float32))
                image=cv2.warpPerspective(board,H,(640,480),borderValue=(150,150,150))
                cv2.imwrite(str(folder/f"view_{i:03}.jpg"),image)
            solve(folder,folder/"out",(9,6),20,0.8)
            result=json.loads((folder/"out/intrinsics.json").read_text())
            estimated=np.array(result["K"])
            self.assertLess(abs(estimated[0,0]-530)/530,.05)
            self.assertLess(abs(estimated[1,1]-525)/525,.05)
            self.assertLess(result["held_out_rms_px"],.8)

    def test_local_viewer_serves_map_and_image(self):
        from urllib.request import urlopen
        state={"lock":threading.Lock(),"jpeg":b"test-jpeg", "map":{"points":[[1,2,3]],"trajectory":[]}}
        server=make_server(state,0)
        worker=threading.Thread(target=server.serve_forever)
        worker.start()
        try:
            base=f"http://127.0.0.1:{server.server_port}"
            self.assertEqual(json.loads(urlopen(base+"/api").read())["points"],[[1,2,3]])
            self.assertEqual(urlopen(base+"/frame.jpg").read(),b"test-jpeg")
            self.assertIn(b"Sparse 3D map",urlopen(base).read())
        finally:
            server.shutdown()
            server.server_close()
            worker.join()


if __name__ == "__main__":
    unittest.main()
