import unittest
from pathlib import Path
from rc.cpu_settings import configure


class CPUProfileTests(unittest.TestCase):
    def test_profile_preserves_calibration_and_is_idempotent(self):
        source = (Path(__file__).resolve().parents[1]/"calibration/camera.yaml").read_text()
        output = configure(source, "candidate")
        self.assertEqual(output, configure(output, "candidate"))
        for line in source.splitlines():
            if line.startswith("Camera") or line.startswith("Calibration"):
                self.assertIn(line, output)
        self.assertIn("ORBextractor.nFeatures: 2000", output)
        self.assertIn("CPU.VisualRecoverySeconds: 15.0", output)
        self.assertIn("ORBextractor.scaleFactor: 1.2", output)
        self.assertIn("ORBextractor.nLevels: 8", output)

    def test_missing_setting_rejected(self):
        with self.assertRaises(ValueError):
            configure("", "candidate")
