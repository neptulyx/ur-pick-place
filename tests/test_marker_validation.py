"""Validation uses simulated RTDE only; no hardware connections are opened."""
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace

from vision_sim import TCP_OFFSET, calibration_dict
from ur_pick_place.demo import DemoRobot
from ur_pick_place.vision.teach_ui import TeachingService
from ur_pick_place.vision.teaching import TeachingProject, digest, read_json


class ValidationRobot(DemoRobot):
    def __init__(self):
        super().__init__()
        self.moves = []
        self.joint_moves = []
        self.joint_stops = 0
        self.fk_calls = []
        self.stops = 0
        self.tcp = list(TCP_OFFSET)
        self._duration = .15

    def getTCPOffset(self):
        return self.tcp

    def moveL(self, *args):
        self.moves.append(args)
        return super().moveL(*args)

    def stopL(self, *args):
        self.stops += 1
        return super().stopL(*args)

    def moveJ(self, *args):
        self.joint_moves.append(args)
        return super().moveJ(*args)

    def stopJ(self, *args):
        self.joint_stops += 1
        return super().stopJ(*args)

    def getForwardKinematics(self, joints, tcp_offset=None):
        self.fk_calls.append((list(joints), tcp_offset))
        if tcp_offset is None:
            # Reproduce the live SDK/controller response when TCP is omitted.
            return [.462, .152, 2.818, -1.401, -2.190, 1.649]
        return super().getForwardKinematics(joints)


class ValidationTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        calibration = self.root / 'calibration.json'
        calibration.write_text(json.dumps(calibration_dict()))
        self.settings = dict(project=str(self.root/'project'), calibration=str(calibration), robot_ip='fake',
            tcp=list(TCP_OFFSET), motion_model='planar', approach_mm=100,
            target=dict(kind='aruco', marker_id=7, dictionary='DICT_5X5_100', marker_length_mm=30))
        self.project = TeachingProject(self.settings)
        (self.project.directory / 'grasp_pick.json').write_text('{}')
        self.project.flow['taught'] = True
        self.robot = ValidationRobot()
        self.project.flow['captured'] = True
        self.project.observation = SimpleNamespace(robot={'joints_rad': [v+.02 for v in self.robot.joints]}, summary=lambda: {})
        (self.project.directory/'observation.json').write_text('{}')
        self.record = dict(id='test-capture', status='captured', timestamp=time.time(),
            capture_robot=dict(tcp_pose_m_rad=list(self.robot.pose), joints_rad=list(self.robot.joints)),
            grasp_sha256=digest(self.project.directory/'grasp_pick.json'),
            validation_mode='fixed_z',
            validation_tcp_pose_m_rad=[.42, -.12, self.robot.pose[2], .1, math.pi, 0],
            approach_tcp_pose_m_rad=[.42, -.12, .4, .1, math.pi, 0])
        self.project.validation = copy.deepcopy(self.record)
        self.service = TeachingService(self.root, motion_factory=lambda _: (self.robot, self.robot))
        self.service.project = self.project
        self.service.heartbeat()
        self.addCleanup(self.service.close)
        self.body = dict(capture_id='test-capture', path_confirmed=True, speed_mm_s=10,
                         workspace_min_m=[-.9, -.9, .02], workspace_max_m=[.9, .9, 1.2])
        self.return_body = dict(path_confirmed=True, joint_speed_deg_s=3,
            workspace_min_m=[-.9, -.9, .02], workspace_max_m=[.9, .9, 1.2])

    def wait_done(self, heartbeat=True):
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            if heartbeat:
                self.service.heartbeat()
            if not self.service.state()['busy']:
                return
            time.sleep(.01)
        self.fail('Validation worker did not finish')

    def test_motion_is_single_use_and_records_measured_error_and_visual_review(self):
        self.service.move_validation(self.body)
        self.wait_done()
        self.assertEqual(self.service.error, '')
        result = self.project.validation
        self.assertEqual(result['status'], 'completed')
        self.assertLess(result['error']['position_mm'], 1)
        self.assertLess(result['error']['rotation_deg'], .287)
        for actual, expected in zip(self.robot.moves[0][0], self.record['validation_tcp_pose_m_rad']):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.robot.moves[0][0][2], self.record['capture_robot']['tcp_pose_m_rad'][2])
        self.assertNotEqual(self.robot.moves[0][0][2], self.record['approach_tcp_pose_m_rad'][2])
        self.assertEqual(self.robot.moves[0][1:3], (.01, .05))
        self.assertFalse(self.robot.connected)
        with self.assertRaisesRegex(ValueError, 'fresh validation'):
            self.service.move_validation(self.body)
        self.service.review_validation(dict(capture_id=self.record['id'], result='aligned'))
        saved = read_json(self.project.directory/'validation_history.json')
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]['visual_result'], 'aligned')

    def test_old_approach_capture_requires_recapture(self):
        self.project.validation.pop('validation_mode')
        self.project.validation.pop('validation_tcp_pose_m_rad')
        with self.assertRaisesRegex(ValueError, 'fixed-height'):
            self.service.move_validation(self.body)
        self.assertIsNone(self.service.motion_thread)
        self.assertFalse(self.robot.moves)

    def test_return_uses_saved_joints_and_invalidates_capture_without_changing_teaching(self):
        self.service.return_observation(self.return_body)
        self.wait_done()
        self.assertEqual(self.service.error, '')
        self.assertEqual(self.service.completed, 'return_observation')
        self.assertEqual(self.service.return_result['status'], 'completed')
        self.assertLess(self.service.return_result['joint_error_deg'], .1)
        self.assertEqual(self.robot.joint_moves[0][0], self.project.observation.robot['joints_rad'])
        self.assertEqual(self.robot.fk_calls, [(self.project.observation.robot['joints_rad'], self.settings['tcp'])])
        self.assertAlmostEqual(self.robot.joint_moves[0][1], math.radians(3))
        self.assertFalse(self.robot.moves)
        self.assertIsNone(self.project.validation)
        self.assertEqual((self.project.directory/'observation.json').read_text(), '{}')
        self.assertFalse(self.robot.connected)
        self.assertFalse(self.project.validation_history)

    def test_return_needs_reference_path_confirmation_valid_speed_and_heartbeat(self):
        for values in (dict(path_confirmed=False), dict(joint_speed_deg_s=0),
                       dict(joint_speed_deg_s=7), dict(workspace_min_m=[1, 2])):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.service.return_observation(dict(self.return_body, **values))
        self.service.heartbeat_at = time.monotonic()-3
        with self.assertRaisesRegex(ValueError, 'heartbeat'):
            self.service.return_observation(self.return_body)
        self.service.heartbeat()
        self.project.observation = None
        with self.assertRaisesRegex(ValueError, 'step 2'):
            self.service.return_observation(self.return_body)
        self.assertIsNone(self.service.motion_thread)

    def test_return_refuses_tcp_workspace_joint_safety_and_changed_reference(self):
        for problem in ('tcp', 'workspace', 'joints', 'pose', 'reference'):
            with self.subTest(problem=problem):
                self.robot = ValidationRobot()
                body = dict(self.return_body)
                if problem == 'tcp':
                    self.robot.tcp = [0]*6
                elif problem == 'workspace':
                    body['workspace_max_m'] = [.3, .9, 1.2]
                elif problem == 'joints':
                    self.robot.isJointsWithinSafetyLimits = lambda _: False
                elif problem == 'pose':
                    self.robot.isPoseWithinSafetyLimits = lambda _: False
                else:
                    def changed(_):
                        (self.project.directory/'observation.json').write_text('{"changed":true}')
                        return True
                    self.robot.isPoseWithinSafetyLimits = changed
                self.service.heartbeat()
                self.service.return_observation(body)
                self.wait_done()
                self.assertTrue(self.service.error)
                self.assertFalse(self.robot.joint_moves)
                self.assertEqual(self.service.return_result['status'], 'failed')

    def test_return_stop_and_heartbeat_loss_use_joint_stop(self):
        for reason in ('stop', 'heartbeat'):
            with self.subTest(reason=reason):
                self.robot = ValidationRobot()
                self.robot._duration = 10
                self.service.heartbeat()
                self.service.return_observation(self.return_body)
                deadline = time.monotonic()+2
                while not self.robot.joint_moves and time.monotonic() < deadline:
                    self.service.heartbeat()
                    time.sleep(.01)
                self.assertTrue(self.robot.joint_moves)
                if reason == 'stop':
                    self.service.stop()
                else:
                    self.service.heartbeat_at = time.monotonic()-3
                self.wait_done(heartbeat=False)
                self.assertGreater(self.robot.joint_stops, 0)
                self.assertNotEqual(self.service.return_result['status'], 'completed')

    def test_return_stop_during_preflight_never_dispatches(self):
        checking, release = threading.Event(), threading.Event()
        def safety_check(_):
            checking.set()
            release.wait(2)
            return True
        self.robot.isJointsWithinSafetyLimits = safety_check
        self.service.return_observation(self.return_body)
        self.assertTrue(checking.wait(1))
        self.service.stop()
        release.set()
        self.wait_done()
        self.assertFalse(self.robot.joint_moves)

    def test_return_checks_joint_completion(self):
        original = self.robot.moveJ
        def wrong_move(target, *args):
            wrong = list(target)
            wrong[0] += .02
            return original(wrong, *args)
        self.robot.moveJ = wrong_move
        self.service.return_observation(self.return_body)
        self.wait_done()
        self.assertEqual(self.service.return_result['status'], 'failed')
        self.assertIn('saved observation joints', self.service.error)

    def test_validation_refuses_a_target_with_changed_height(self):
        self.project.validation['validation_tcp_pose_m_rad'][2] += .1
        self.service.move_validation(self.body)
        self.wait_done()
        self.assertIn('captured TCP height', self.service.error)
        self.assertFalse(self.robot.moves)

    def test_expired_changed_or_unreviewed_capture_never_connects(self):
        cases = [dict(path_confirmed=False), dict(capture_id='old'), dict(speed_mm_s=21),
                 dict(workspace_min_m=[1, 2]), dict(workspace_min_m=[1, 2, 3])]
        for values in cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.service.move_validation(dict(self.body, **values))
        self.project.validation['timestamp'] -= 61
        with self.assertRaisesRegex(ValueError, 'expired'):
            self.service.move_validation(self.body)
        self.project.validation = copy.deepcopy(self.record)
        (self.project.directory/'grasp_pick.json').write_text('{"changed":true}')
        with self.assertRaisesRegex(ValueError, 'relationship changed'):
            self.service.move_validation(self.body)
        self.assertIsNone(self.service.motion_thread)
        self.assertFalse(self.robot.moves)

    def test_wrong_tcp_robot_shift_and_workspace_refuse_motion(self):
        for problem in ('tcp', 'joints', 'workspace', 'ik', 'safety', 'calibration'):
            with self.subTest(problem=problem):
                self.robot = ValidationRobot()
                self.project.validation = copy.deepcopy(self.record)
                body = dict(self.body)
                if problem == 'tcp':
                    self.robot.tcp = [0]*6
                elif problem == 'joints':
                    self.robot.joints[0] += .1
                elif problem == 'workspace':
                    body['workspace_max_m'] = [.41, .9, 1.2]
                elif problem == 'ik':
                    self.robot.getInverseKinematicsHasSolution = lambda *args: False
                elif problem == 'safety':
                    self.robot.isPoseWithinSafetyLimits = lambda *args: False
                elif problem == 'calibration':
                    Path(self.settings['calibration']).write_text('{}')
                    with self.assertRaisesRegex(ValueError, 'Calibration changed'):
                        self.service.move_validation(body)
                    continue
                self.service.heartbeat()
                self.service.move_validation(body)
                self.wait_done()
                self.assertTrue(self.service.error)
                self.assertFalse(self.robot.moves)
                self.assertEqual(self.project.validation['status'], 'failed')

    def test_stop_cancels_connection_before_move_and_requires_recapture(self):
        connecting, release = threading.Event(), threading.Event()
        def factory(_):
            connecting.set()
            release.wait(2)
            return self.robot, self.robot
        self.service.motion_factory = factory
        self.service.move_validation(self.body)
        self.assertTrue(connecting.wait(1))
        self.service.stop()
        release.set()
        self.wait_done()
        self.assertFalse(self.robot.moves)
        self.assertEqual(self.project.validation['status'], 'stopped')

    def test_stop_during_robot_preflight_never_starts_a_move(self):
        checking, release = threading.Event(), threading.Event()

        def safety_check(_):
            checking.set()
            release.wait(2)
            return True

        self.robot.isPoseWithinSafetyLimits = safety_check
        self.service.move_validation(self.body)
        self.assertTrue(checking.wait(1))
        self.service.stop()
        release.set()
        self.wait_done()
        self.assertFalse(self.robot.moves)
        self.assertEqual(self.project.validation['status'], 'stopped')

    def test_robot_change_during_preflight_never_starts_a_move(self):
        for problem in ('safety', 'joints', 'tcp'):
            with self.subTest(problem=problem):
                checking, release = threading.Event(), threading.Event()
                self.robot = ValidationRobot()
                self.project.validation = copy.deepcopy(self.record)

                def safety_check(_):
                    checking.set()
                    release.wait(2)
                    return True

                self.robot.isPoseWithinSafetyLimits = safety_check
                self.service.heartbeat()
                self.service.move_validation(self.body)
                self.assertTrue(checking.wait(1))
                if problem == 'safety':
                    self.robot.getSafetyMode = lambda: 7
                elif problem == 'joints':
                    self.robot.joints[0] += .01
                else:
                    self.robot.tcp = [0]*6
                release.set()
                self.wait_done()
                self.assertFalse(self.robot.moves)
                self.assertEqual(self.project.validation['status'], 'failed')

    def test_heartbeat_loss_and_explicit_stop_interrupt_motion(self):
        for reason in ('stop', 'heartbeat'):
            with self.subTest(reason=reason):
                self.robot = ValidationRobot()
                self.robot._duration = 10
                self.project.validation = copy.deepcopy(self.record)
                self.service.heartbeat()
                self.service.move_validation(self.body)
                deadline = time.monotonic()+2
                while not self.robot.moves and time.monotonic() < deadline:
                    self.service.heartbeat()
                    time.sleep(.01)
                self.assertTrue(self.robot.moves)
                if reason == 'stop':
                    self.service.stop()
                else:
                    self.service.heartbeat_at = time.monotonic()-3
                self.wait_done(heartbeat=False)
                self.assertGreater(self.robot.stops, 0)
                self.assertNotEqual(self.project.validation['status'], 'completed')
                self.assertFalse(self.robot.connected)

    def test_inaccurate_completion_fails_and_cannot_be_marked_aligned(self):
        def wrong_move(pose, *args):
            self.robot.moves.append((pose, *args))
            target = list(pose)
            target[0] += .01
            return DemoRobot.moveL(self.robot, target, *args)
        self.robot.moveL = wrong_move
        self.service.move_validation(self.body)
        self.wait_done()
        self.assertEqual(self.project.validation['status'], 'failed')
        self.assertGreater(self.project.validation['error']['position_mm'], 9)
        with self.assertRaises(ValueError):
            self.service.review_validation(dict(capture_id=self.record['id'], result='aligned'))

    def test_independent_xyz_error_is_recorded_separately_from_motion_completion(self):
        self.project.validation = dict(self.record, status='completed')
        self.service.review_validation(dict(capture_id=self.record['id'], result='aligned',
                                            measured_error_mm=[.5, -.3, .8]))
        measured = self.project.validation
        self.assertAlmostEqual(measured['measured_position_error_mm'], math.sqrt(.98))
        self.assertTrue(measured['within_2mm'])
        self.service.review_validation(dict(capture_id=self.record['id'], result='misaligned',
                                            measured_error_mm=[0, 0, 2.1]))
        self.assertFalse(self.project.validation['within_2mm'])

    def test_tcp_change_or_rejected_stop_during_motion_is_reported(self):
        for problem in ('tcp', 'stop'):
            with self.subTest(problem=problem):
                self.robot = ValidationRobot()
                self.robot._duration = 10
                self.project.validation = copy.deepcopy(self.record)
                self.service.heartbeat()
                self.service.move_validation(self.body)
                deadline = time.monotonic()+2
                while not self.robot.moves and time.monotonic() < deadline:
                    self.service.heartbeat()
                    time.sleep(.01)
                self.assertTrue(self.robot.moves)
                if problem == 'tcp':
                    self.robot.tcp = [0]*6
                else:
                    self.robot.stopL = lambda *args: False
                    self.service.stop()
                self.wait_done()
                self.assertEqual(self.project.validation['status'], 'failed')
                self.assertIn('TCP' if problem == 'tcp' else 'rejected the validation stop', self.service.error)
                self.assertFalse(self.robot.connected)


if __name__ == '__main__':
    unittest.main()
