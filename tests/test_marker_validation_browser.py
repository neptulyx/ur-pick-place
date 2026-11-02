"""Exercise the real frontend and HTTP workers in an isolated headless browser."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest

import vision_sim  # noqa: F401 -- skip before importing optional vision code


class ValidationBrowserTests(unittest.TestCase):
    def run_browser(self, motion_model):
        script = Path(__file__).with_name('marker_validation_browser.cjs')
        result = subprocess.run(['node', str(script)], capture_output=True, text=True, timeout=110,
            env=dict(os.environ, PYTHON=sys.executable, CHROME_BINARY=shutil.which('google-chrome'),
                     MARKER_VALIDATION_MOTION_MODEL=motion_model))
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn('Browser verified:', result.stdout)

    @unittest.skipUnless(shutil.which('node') and shutil.which('google-chrome'),
                         'Node and Google Chrome are needed for browser integration')
    def test_step_five_with_real_http_and_capture_workers(self):
        self.run_browser('planar')

    @unittest.skipUnless(shutil.which('node') and shutil.which('google-chrome'),
                         'Node and Google Chrome are needed for browser integration')
    def test_full_pose_with_z_roll_pitch_and_yaw_changes(self):
        self.run_browser('full')


if __name__ == '__main__':
    unittest.main()
