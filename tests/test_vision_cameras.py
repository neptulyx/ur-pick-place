"""Calibrated Basler images and acquisition using pylon's hardware-free emulator."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import unittest
from types import SimpleNamespace

from vision_sim import INTRINSICS, calibration_dict
import numpy as np

from ur_pick_place.vision.config import Camera
from ur_pick_place.vision.errors import VisionError
from ur_pick_place.vision.handeye import parse_handeye
from ur_pick_place.vision.observe import intrinsics_checker


class BaslerCalibrationTests(unittest.TestCase):
    def test_uses_calibrated_lens_model_and_requires_matching_resolution(self):
        calibration = parse_handeye(calibration_dict())
        checker = intrinsics_checker(calibration, Camera(provider='basler'))
        frame = SimpleNamespace(bgr=np.zeros((720, 1280), np.uint8), intrinsics=None)
        self.assertEqual(checker(frame), INTRINSICS)
        frame.bgr = np.zeros((480, 640), np.uint8)
        with self.assertRaisesRegex(VisionError, 'calibrated'):
            checker(frame)

    def test_factory_intrinsics_cannot_be_selected_for_basler(self):
        with self.assertRaisesRegex(ValueError, 'intrinsics from calibration'):
            Camera(provider='basler', intrinsics='camera')

    @unittest.skipUnless(importlib.util.find_spec('pypylon'), 'pypylon optional extra not installed')
    def test_software_trigger_and_cleanup_with_official_emulator(self):
        code = '''
from pypylon import pylon
from ur_pick_place.vision.cameras import BaslerCamera
from ur_pick_place.vision.config import Camera
emulated = [d for d in pylon.TlFactory.GetInstance().EnumerateDevices()
            if d.GetDeviceClass() == 'BaslerCamEmu'][0]
settings = Camera(provider='basler', serial=emulated.GetSerialNumber(),
                  width=640, height=480, exposure_time_us=5000)
for _ in range(2):
    camera = BaslerCamera(settings)
    try:
        a, b = camera.read(), camera.read()
        assert a.bgr.shape == (480, 640)
        assert a.bgr.dtype.name == 'uint8'
        assert a.intrinsics is None
        assert a.frame_number != b.frame_number
        assert b.timestamp_ms >= a.timestamp_ms
    finally:
        camera.close()
'''
        result = subprocess.run([sys.executable, '-c', code], env=dict(os.environ, PYLON_CAMEMU='1'),
            cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
