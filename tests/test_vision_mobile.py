"""Mobile-base capture and the UI's fresh vision -> existing sequence integration."""
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from vision_sim import (CAMERA_IN_FLANGE, TCP_OFFSET, FakeGripper, handeye_error,
                        matrix_to_pose, recipe, rtde_robot, vision_config, INTRINSICS, FakeCamera, calibration_dict)
from test_vision_pipeline import World, PLACE_TCP
from test_manual import wait_for
import numpy as np

from ur_pick_place.manual import ManualService, ManualSettings
from ur_pick_place.poses import PoseStore
from ur_pick_place.vision.config import VisionConfig
from ur_pick_place.vision.grasp import teach, write_taught, load_taught, target_pose
from ur_pick_place.vision.handeye import load_handeye
from ur_pick_place.vision.observe import Observer, RobotMover
from ur_pick_place.vision.robot_state import ControlPoseSource
from ur_pick_place.vision.transforms import rotation_angle


class MobileVisionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.world = World()
        # Nominal 5 MP / 16 mm Basler optical geometry, with rendered image noise.
        intrinsics = dict(INTRINSICS, width=2448, height=2048, fx=16/.00274, fy=16/.00274,
                          ppx=1223.5, ppy=1023.5, coeffs=[0.0]*5)
        self.world.camera = FakeCamera(self.world.rtde, self.world.camera.scene, intrinsics=intrinsics)
        self.path = vision_config(self.root, place=False, refine=True,
                                  camera_in_flange=CAMERA_IN_FLANGE @ handeye_error())
        (self.root/'handeye.json').write_text(json.dumps(calibration_dict(
            CAMERA_IN_FLANGE @ handeye_error(), intrinsics=intrinsics)))
        raw = json.loads(self.path.read_text())
        raw['camera'] = dict(provider='basler', width=2448, height=2048, fps=20)
        raw['pick']['refine']['distance_m'] = .3
        self.path.write_text(json.dumps(raw))
        self.config = VisionConfig.load(self.path)
        self.calibration = load_handeye(self.root / 'handeye.json')
        self.source = ControlPoseSource(self.world.rtde, self.world.rtde)
        self.robot = rtde_robot(recipe(), self.world.rtde)
        self.observer = Observer(self.config, self.calibration, self.world.camera, self.source,
                                 mover=RobotMover(self.robot))
        observed = self.observer.look('pick', allow_motion=True)
        # User's 300 mm marker-to-pick lever arm; true TCP points downward.
        self.marker_tcp = np.eye(4)
        self.marker_tcp[:3, 3] = [.3, 0, .02]
        record = teach(observed, matrix_to_pose(self.world.pick @ self.marker_tcp), role='pick',
                       calibration=self.calibration, limits=self.config.pick.limits, tcp_offset=TCP_OFFSET)
        write_taught(self.root / 'grasp_pick.json', record)
        store = PoseStore(self.root / 'poses.json')
        home = store.save('home', recipe().pick_pose, recipe().home_joints, robot_ip='fake', demo=False)
        place = store.save('place', matrix_to_pose(self.world.place @ PLACE_TCP), recipe().home_joints, robot_ip='fake', demo=False)
        settings = ManualSettings(robot_ip='fake')
        self.service = ManualService(settings, store, recipe=recipe(),
            factory=lambda host: (self.world.rtde, self.world.rtde), sequence_path=self.root/'sequence.json')
        self.addCleanup(self.service.close)
        self.service.submit('connect')
        wait_for(lambda: self.service.state()['connected'] and not self.service.state()['pending'])
        self.stop_heartbeat = threading.Event()
        def heartbeat():
            while not self.stop_heartbeat.wait(.1):
                self.service.heartbeat()
        self.thread = threading.Thread(target=heartbeat, daemon=True)
        self.thread.start()
        self.addCleanup(lambda: (self.stop_heartbeat.set(), self.thread.join(timeout=1)))
        self.payload = dict(home_id=home['id'], place_id=place['id'], vision_file=str(self.path))
        self.camera_patch = patch('ur_pick_place.vision.cli.Runtime.camera', return_value=self.world.camera)
        self.camera_patch.start()
        self.addCleanup(self.camera_patch.stop)
        self.gripper = FakeGripper(self.world.rtde)
        gripper_patch = patch('ur_pick_place.grippers.adapters.connect_gripper', return_value=self.gripper)
        gripper_patch.start()
        self.addCleanup(gripper_patch.stop)
        self.world.rtde.events.clear()

    def preview(self):
        result = self.service.preview_sequence(self.payload)
        wait_for(lambda: not self.service.state()['pending'])
        return result

    def run_preview(self, preview):
        self.service.submit('run_sequence', {'preview_id': preview['id']})
        wait_for(lambda: not self.service.state()['pending'], timeout=15)
        return self.service.state()

    def test_different_initial_pose_refines_and_picks_with_300mm_offset(self):
        # Relative marker shift models a different AGV parking pose.
        self.world.move_pick(12, -8, 3)
        self.world.rtde.moveJ([q + .015 for q in self.world.rtde.q], .1, .1)
        self.world.rtde.events.clear()
        preview = self.preview()
        self.assertFalse(self.world.rtde.events)  # Preview has no motion.
        result = self.run_preview(preview)
        self.assertEqual(result['sequence']['status'], 'completed', result)
        truth = self.world.pick @ self.marker_tcp
        error_mm = np.linalg.norm(self.gripper.grip_tcp[:3, 3] - truth[:3, 3]) * 1000
        self.assertLess(error_mm, 2)
        self.assertLess(math.degrees(rotation_angle(self.gripper.grip_tcp, truth)), .3)
        saved = json.loads((self.root/'latest_vision_pick.json').read_text())
        self.assertTrue(saved['observations']['pick']['refined'])
        self.assertIn('pick_repeatability', saved['targets']['pick'])
        # Exactly one refine move occurs before the controller's home move.
        events = [e[0] for e in self.world.rtde.events]
        self.assertEqual(events[:2], ['moveL', 'moveJ'])

    def test_localization_across_full_pose_parking_variations_at_300mm(self):
        from vision_sim import OBSERVE_PICK
        from ur_pick_place.vision.transforms import rotvec_to_matrix
        model = load_taught(self.root/'grasp_pick.json')
        initial_marker = self.world.pick.copy()
        observer = Observer(self.config, self.calibration, self.world.camera, self.source,
            mover=RobotMover(self.robot), views={'pick': model.taught_view})
        changes = ((-15, 10, 12, 2, -3, -5, -.02), (12, -8, -10, -2, 1, 3, .015),
                   (0, 0, 0, 0, 0, 0, .03))
        for dx, dy, dz, roll, pitch, yaw, joint_delta in changes:
            with self.subTest(dx=dx, dy=dy, dz=dz, roll=roll, pitch=pitch, yaw=yaw):
                self.world.pick = initial_marker.copy()
                self.world.move_pick(dx, dy, yaw)
                self.world.pick[2, 3] += dz / 1000
                self.world.pick[:3, :3] = rotvec_to_matrix(np.radians([roll, pitch, 0])) @ self.world.pick[:3, :3]
                self.world.rtde.moveJ([q+joint_delta for q in OBSERVE_PICK], .1, .1)
                observed = observer.look('pick', allow_motion=True, from_current=True)
                pose, _ = target_pose(model, observed, self.config.pick,
                    calibration_sha256=self.calibration.sha256, tcp_offset=TCP_OFFSET,
                    target=observer.target('pick'))
                truth = self.world.pick @ self.marker_tcp
                self.assertLess(np.linalg.norm(pose[:3, 3]-truth[:3, 3])*1000, 2)
                self.assertLess(math.degrees(rotation_angle(pose, truth)), .3)

    def test_failed_refinement_blocks_the_pick(self):
        preview = self.preview()
        self.world.move_pick(12, 0, 0)
        original = self.world.rtde.moveL
        def wrong_move(pose, *args):
            result = original(pose, *args)
            self.world.move_pick(20, 0, 0)  # A changed scene after the move must fail the view check.
            return result
        self.world.rtde.moveL = wrong_move
        try:
            result = self.run_preview(preview)
        finally:
            self.world.rtde.moveL = original
        self.assertEqual(result['sequence']['status'], 'failed')
        self.assertIn('did not converge', result['error'])
        self.assertNotIn('grip', self.gripper.events)
        self.assertFalse(any(e[0] == 'forceMode' for e in self.world.rtde.events))

    def test_changed_calibration_or_expired_preview_blocks_motion(self):
        calibration_path = self.root/'handeye.json'
        original_calibration = calibration_path.read_text()
        for change in ('calibration', 'age'):
            with self.subTest(change=change):
                preview = self.preview()
                if change == 'calibration':
                    path = self.root/'handeye.json'
                    path.write_text(path.read_text()+'\n')
                else:
                    self.service._preview['vision_record']['created_at'] = time.time()-10000
                result = self.run_preview(preview)
                self.assertEqual(result['sequence']['status'], 'failed')
                self.assertFalse(self.world.rtde.events)
                self.assertNotIn('grip', self.gripper.events)
                calibration_path.write_text(original_calibration)

    def test_stop_during_capture_blocks_refine_and_pick(self):
        preview = self.preview()
        read = self.world.camera.read
        def stop_after_read():
            frame = read()
            self.service.stop()
            return frame
        with patch.object(self.world.camera, 'read', side_effect=stop_after_read):
            result = self.run_preview(preview)
        self.assertEqual(result['sequence']['status'], 'stopped')
        self.assertFalse(self.world.rtde.events)
        self.assertNotIn('grip', self.gripper.events)
