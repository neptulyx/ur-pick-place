"""Camera adapters. Images keep their native pixels; calibration supplies the model."""
from types import SimpleNamespace
import time

import numpy as np

from .errors import VisionError


class BaslerCamera:
    """Mono8 acquisition for the a2A2448-23gmBAS through pypylon.

    Software-trigger each image so a post-move capture cannot reuse buffered images.
    Basler does not supply calibrated lens intrinsics; the observer uses its file.
    """

    def __init__(self, config):
        try:
            from pypylon import genicam, pylon
        except ImportError as exc:
            raise VisionError('Basler capture needs pypylon: pip install -e ".[basler]"') from exc
        self._pylon, self._camera = pylon, None
        self._frame_number = 0
        try:
            factory = pylon.TlFactory.GetInstance()
            devices = factory.EnumerateDevices()
            matches = [d for d in devices if config.serial is None or d.GetSerialNumber() == config.serial]
            if len(matches) != 1:
                raise VisionError('Select a Basler camera serial; expected exactly one matching camera')
            self.serial = matches[0].GetSerialNumber()
            camera = self._camera = pylon.InstantCamera(factory.CreateDevice(matches[0]))
            camera.Open()
            # Reset geometry that changes the intrinsics before selecting the profile.
            for name, value in (('OffsetX', 0), ('OffsetY', 0), ('BinningHorizontal', 1),
                                ('BinningVertical', 1), ('DecimationHorizontal', 1), ('DecimationVertical', 1)):
                try:
                    node = camera.GetNodeMap().GetNode(name)
                except genicam.LogicalErrorException:
                    continue  # Not every model supports binning/decimation.
                if genicam.IsWritable(node):
                    node.SetValue(value)
                elif genicam.IsReadable(node) and node.GetValue() != value:
                    raise VisionError(f'Basler {name} must be {value} for the calibrated profile')
            camera.Width.Value, camera.Height.Value = config.width, config.height
            camera.PixelFormat.Value = 'Mono8'
            camera.GainAuto.Value = 'Off'
            camera.Gain.Value = 0.0
            camera.ExposureAuto.Value = 'Off'
            if config.exposure_time_us is not None:
                camera.ExposureTime.Value = config.exposure_time_us
            camera.AcquisitionFrameRateEnable.Value = True
            camera.AcquisitionFrameRate.Value = config.fps
            camera.AcquisitionMode.Value = 'Continuous'
            camera.TriggerSelector.Value = 'FrameStart'
            camera.TriggerMode.Value = 'On'
            camera.TriggerSource.Value = 'Software'
            camera.StartGrabbing(pylon.GrabStrategy_OneByOne)
        except Exception:
            self.close()
            raise

    def read(self):
        pylon, camera = self._pylon, self._camera
        camera.WaitForFrameTriggerReady(5000, pylon.TimeoutHandling_ThrowException)
        camera.ExecuteSoftwareTrigger()
        result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        try:
            if not result.GrabSucceeded():
                raise VisionError(f'Basler image grab failed: {result.GetErrorDescription()}')
            self._frame_number += 1
            block = int(result.GetBlockID())
            if block in (0, 2**64 - 1):
                block = self._frame_number  # Some transports/emulators do not supply block IDs.
            return SimpleNamespace(bgr=np.ascontiguousarray(result.Array).copy(), intrinsics=None,
                                   frame_number=block,
                                   timestamp_ms=time.monotonic() * 1000)
        finally:
            result.Release()

    def lock_exposure(self):
        self._camera.ExposureAuto.Value = 'Off'
        self._camera.GainAuto.Value = 'Off'

    def close(self):
        if self._camera is not None:
            try:
                if self._camera.IsGrabbing():
                    self._camera.StopGrabbing()
            finally:
                self._camera.Close()
