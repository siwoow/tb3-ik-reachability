"""Camera-relative stop/observe/approach; no odometry target or subscription."""

import argparse
from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, JointState, LaserScan
from geometry_msgs.msg import PointStamped, Twist
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
import yaml

from .bridge import rotation
from .planner_adapter import load_model, native_query, screen_native_path
from .safety import SafetyError, check_scan


class ViewUnavailable(SafetyError):
    """Request a fresh view without accepting an invalid grasp target."""


class VisualServo(Node):
    """Reuse perception geometry, native candidate IK/FCL and pick executor."""

    def __init__(self, options):
        super().__init__('tb3_camera_servo')
        self.options = options
        self.root = Path(options.output)
        self.root.mkdir(parents=True, exist_ok=False)
        self.rgb, self.depth = deque(maxlen=12), deque(maxlen=12)
        self.info = self.joints = self.scan = None
        self.fault = None
        self.moving = False
        self.closed = threading.Event()
        self.last_command = time.monotonic()
        self.last_stop = self.get_clock().now().nanoseconds
        self.steps = 0
        self.nominal_distance = 0.
        self.spin_error = None
        self.view_state = None
        self.view_pose_tried = False
        self.home_view_tried = False
        self.reverse_view_baseline = None
        self.view_pose_history = set()
        self.view_goal = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        for kind, topic, callback in [
            (Image, '/camera/camera/color/image_raw', self.rgb.append),
            (Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth.append),
            (CameraInfo, '/camera/camera/color/camera_info', lambda m: setattr(self, 'info', m)),
            (JointState, '/joint_states', lambda m: setattr(self, 'joints', m)),
            (LaserScan, '/scan', lambda m: setattr(self, 'scan', m)),
        ]:
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1) if options.execute_motion else None
        self.target_pub = self.create_publisher(PointStamped, '/visual_servo/pick_target', 1)
        self.pick = self.create_client(Trigger, '/visual_servo/execute_pick')
        self.view_arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.spin_executor = SingleThreadedExecutor()
        self.spin_executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.spin, daemon=True)
        self.spin_thread.start()
        self.watch_thread = threading.Thread(target=self.watch, daemon=True)
        self.watch_thread.start()
        self.model = load_model(options.planner_root)
        from cleanup_perception.camera_calibration import CameraCalibration
        cfg = yaml.safe_load(Path(options.calibration).read_text())['scan_perception_node']['ros__parameters']
        self.calibration = CameraCalibration(cfg['grasp_camera_translation'],
                                             cfg['grasp_camera_rotation'], cfg['grasp_camera_frame'])
        self.align_floor = bool(cfg.get('align_grasp_floor', False))
        from ultralytics import YOLO, SAM
        self.yolo = YOLO(options.yolo)
        self._sam_model = SAM(options.sam)
        self._device = ''
        self._sam_error = ''
        self._use_sam_refinement = True
        # Warm model kernels before accepting an acquisition timestamp.
        dummy = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.yolo.predict(source=dummy, verbose=False, conf=.45)
        self._sam_model.predict(source=dummy, bboxes=[[400, 200, 800, 500]], verbose=False, save=False)
        self.record('START', mode='camera_relative', execute_motion=options.execute_motion,
                    execute_grasp=options.execute_grasp, speed_mps=.02,
                    nominal_step_m=.02, acquisition_age_limit_s=4., odometry_used=False)
        self.record('LIDAR_MODE', isolated_no_lidar_guards=options.isolated_no_lidar_guards)

    def spin(self):
        try:
            self.spin_executor.spin()
        except Exception as error:
            self.spin_error = str(error)

    def record(self, event, **values):
        row = dict(event=event, wall_time=time.time(), **values)
        with (self.root / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(row, default=lambda a: a.tolist()) + '\n')
        if event == 'OBSERVATION':
            print(f"관측 {values['index']}: Depth={values['optical_m'][2]:.3f}m "
                  f"IK={values['native_ik']} FCL={values['fcl']}", flush=True)
        else:
            print(json.dumps(row, default=lambda a: a.tolist()), flush=True)

    def stamp(self, msg):
        return Time.from_msg(msg.header.stamp).nanoseconds

    def fresh(self, msg, maximum):
        if msg is None or self.stamp(msg) <= 0:
            raise SafetyError('Missing sensor message')
        age = (self.get_clock().now().nanoseconds - self.stamp(msg))*1e-9
        if not -.05 <= age <= maximum:
            raise SafetyError(f'Stale/future {type(msg).__name__}: {age:.3f}s')

    def transform(self, target, source, stamp):
        result = self.tf.lookup_transform(target, source, Time.from_msg(stamp),
                                          timeout=Duration(seconds=.2)).transform
        r = rotation(result.rotation)
        t = result.translation
        return r, np.array([t.x, t.y, t.z])

    def guard(self, turning=False, stationary_arm=True, reverse=False):
        if self.fault or self.spin_error:
            raise SafetyError(self.fault or self.spin_error)
        self.fresh(self.scan, .50)
        self.fresh(self.joints, .30)
        if not self.options.isolated_no_lidar_guards:
            scan = self.scan
            valid = sum(math.isfinite(d) and scan.range_min <= d <= scan.range_max for d in scan.ranges)
            if not scan.ranges or valid/len(scan.ranges) < .70:
                raise SafetyError('Full-scan valid LiDAR coverage below 70%')
            r, t = self.transform('base_link', scan.header.frame_id, scan.header.stamp)
            if reverse:
                rear = np.diag([-1., -1., 1.])
                r, t = rear @ r, rear @ t
            check_scan(scan.ranges, scan.angle_min, scan.angle_increment,
                       scan.range_min, scan.range_max, r, t, turning=turning)
        msg = self.joints
        p = dict(zip(msg.name, msg.position))
        v = dict(zip(msg.name, msg.velocity))
        q = np.array([p['joint'+str(i)] for i in range(1, 5)])
        if not np.isfinite(q).all():
            raise SafetyError('Nonfinite arm feedback')
        if stationary_arm and any(abs(v.get('joint'+str(i), math.inf)) > .02 for i in range(1, 5)):
            raise SafetyError('Arm not stationary during camera/base operation')
        opening = p['gripper_left_joint']
        if not math.isfinite(opening) or not self.model.gripper_limits[0] <= opening <= self.model.gripper_limits[1]:
            raise SafetyError(f'Gripper feedback outside unchanged model limits: {opening}')
        if self.pub is not None:
            other = [i.node_name for i in self.get_publishers_info_by_topic('/cmd_vel')
                     if i.node_name != self.get_name()]
            if other:
                raise SafetyError('Other cmd_vel publishers: '+str(other))
        return q, opening

    def zero(self):
        if self.pub is not None:
            self.pub.publish(Twist())
        self.moving = False

    def stop(self):
        self.zero()
        for _ in range(5):
            time.sleep(.04)
            self.zero()
        self.last_stop = self.get_clock().now().nanoseconds

    def watch(self):
        while not self.closed.wait(.02):
            if not self.moving:
                continue
            try:
                self.fresh(self.scan, .50)
                self.fresh(self.joints, .30)
                if self.spin_error or time.monotonic()-self.last_command > .15:
                    raise SafetyError('Command/callback heartbeat lost')
            except Exception as error:
                self.fault = str(error)
                self.zero()

    def frame(self):
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            for rgb in reversed(list(self.rgb)):
                if self.stamp(rgb) <= self.last_stop:
                    continue
                depths = [d for d in list(self.depth) if self.stamp(d) > self.last_stop]
                if not depths:
                    continue
                depth = min(depths, key=lambda d: abs(self.stamp(d)-self.stamp(rgb)))
                if abs(self.stamp(depth)-self.stamp(rgb)) > 150_000_000:
                    continue
                if self.info is None or self.info.width != rgb.width or self.info.height != rgb.height:
                    continue
                self.fresh(rgb, .50)
                self.fresh(depth, .50)
                if rgb.header.frame_id != self.calibration.camera_frame or depth.header.frame_id != rgb.header.frame_id:
                    raise SafetyError('Aligned RGB-D/calibrated optical frame mismatch')
                if (rgb.width, rgb.height) != (depth.width, depth.height):
                    raise SafetyError('RGB-depth size mismatch')
                return rgb, depth, self.info
            time.sleep(.02)
        raise SafetyError('No genuinely new synchronized RGB-D after stop')

    def observe(self, index):
        from cleanup_perception.perception_node import ScanPerceptionNode
        from cleanup_perception.depth_geometry import visible_body_mask
        from cleanup_perception.grasp_candidates import body_candidates, choose_candidate, NoFeasibleCandidate
        from cleanup_perception.floor_alignment import align_candidate_heights
        self.guard()
        rgb, depth_msg, info = self.frame()
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(rgb, 'bgr8')
        depth = bridge.imgmsg_to_cv2(depth_msg)
        if depth_msg.encoding not in ('16UC1', '32FC1'):
            raise SafetyError('Unknown depth unit')
        k = np.asarray(info.k).reshape(3, 3)
        # Match existing perception: optical->link5 calibration followed by the
        # acquisition-time link5->link1 TF. Never use the generic fixed camera TF.
        rlw, tlw = self.transform('link1', 'link5', depth_msg.header.stamp)
        rlb, tlb = self.transform('link1', 'base_link', depth_msg.header.stamp)
        rc = rlw @ self.calibration.rotation
        tc = tlw + rlw @ self.calibration.translation
        rf, tf = self.transform('link1', 'base_footprint', depth_msg.header.stamp)
        if np.linalg.norm(rf-np.eye(3)) > .001:
            raise SafetyError('Floor requires parallel base_footprint/link1 axes')
        floor = float(tf[2])
        q, opening = self.guard()
        results = self.yolo.predict(source=image, conf=.45, iou=.45, verbose=False)
        boxes = []
        if results and results[0].boxes is not None:
            for b in results[0].boxes:
                name = self.yolo.names[int(b.cls[0])]
                if str(name).lower() == 'banana':
                    boxes.append(b.xyxy[0].detach().cpu().numpy())
        if len(boxes) != 1:
            if not boxes:
                self.view_state = {'detected': False, 'stamp_ns': self.stamp(depth_msg)}
                raise ViewUnavailable('Banana outside current camera view')
            raise SafetyError(f'Expected one banana, detected {len(boxes)}')
        masks = ScanPerceptionNode._run_sam_masks(self, image, boxes)
        if len(masks) != 1:
            raise SafetyError('No valid SAM banana mask')
        mask = masks[0]
        x1, y1, x2, y2 = np.rint(boxes[0]).astype(int)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(image.shape[1], x2), min(image.shape[0], y2)
        bounded = np.zeros_like(mask)
        bounded[y1:y2, x1:x2] = mask[y1:y2, x1:x2]
        mask = bounded
        metres = depth.astype(float)*(.001 if depth_msg.encoding == '16UC1' else 1.)
        supported = mask & np.isfinite(metres) & (metres > .1) & (metres < 2.5)
        self.view_state = dict(detected=True, stamp_ns=self.stamp(depth_msg),
                               center_v=(y1+y2)/2., bottom_margin_px=image.shape[0]-y2,
                               supported_pixels=int(supported.sum()))
        if supported.sum() >= 30:
            vv, uu = np.nonzero(supported)
            rays = np.column_stack((uu, vv, np.ones(len(uu)))) @ np.linalg.inv(k).T
            self.view_state['optical_point'] = np.median(rays*metres[vv, uu, None], axis=0).tolist()
        clipped = bool(mask[:3].any() or mask[-3:].any() or mask[:, :3].any() or mask[:, -3:].any())
        if clipped:
            try:
                mask = visible_body_mask(mask, depth, k, .1, 2.5)
            except ValueError as error:
                raise ViewUnavailable(str(error)) from error
        candidates = body_candidates(mask, depth, k, rc, tc, .030)
        floor_report = align_candidate_heights(candidates, mask, depth, k, rc, tc, floor) if self.align_floor else {}
        if not candidates:
            raise ViewUnavailable('No body section satisfies unchanged depth/width/end-margin constraints')
        planner = Path('/home/user/turtlebot3_ws/install/pick_and_place/lib/pick_and_place/candidate_grasp_plan')
        try:
            selected, evaluated = choose_candidate(candidates, floor, q, planner, .004)
        except NoFeasibleCandidate as error:
            evaluated = error.evaluated
            selected = min(evaluated, key=lambda c: abs(c['body_percentile']-50))
        passed = None
        feasible = sorted([c for c in evaluated if c['plan'].get('feasible')],
                          key=lambda c: c.get('score', 0), reverse=True)
        for c in feasible:
            plan = native_query(self.options.native_probe, c['target_link1'], floor, q, .004)
            c['execution_ik'] = bool(plan.get('feasible'))
            c['execution_pitch_degrees'] = plan.get('pitch_degrees')
            if plan.get('feasible'):
                waypoints = [plan['pregrasp'], *plan['descent'], *plan['lift']]
                poses = self.model.fk(np.asarray(waypoints), .018)
                actual_tcp = (np.linalg.inv(poses['link1']) @ poses['end_effector_link'])[:, :3, 3]
                xy_error = float(np.max(np.linalg.norm(actual_tcp[:, :2]-np.asarray(c['target_link1'])[:2], axis=1)))
                bottom = len(plan['descent'])
                z_error = abs(float(actual_tcp[bottom, 2])-(c['target_link1'][2]-plan['body_depth_m']))
                offset = np.asarray(c['target_link1'])-np.asarray(c['observed_link1'])
                opening_gap = float(self.model.zero_gap+2*.018)
                c['tcp_check'] = dict(xy_error_m=xy_error, z_error_m=z_error,
                                      forward_offset_m=offset.tolist(), available_opening_m=opening_gap)
                if (xy_error > .002 or z_error > .002 or
                        not np.allclose(offset, [.030, 0., 0.], atol=1e-6)):
                    raise SafetyError('TCP/target/frame offset mismatch; base motion cannot correct this')
                if c['width_m']+.004 > opening_gap:
                    continue
            # Screen the existing executor's return-home path too, before any grasp.
            checked = deepcopy(plan)
            if checked.get('feasible'):
                checked['lift'].append([0., -.523, -.523, 1.5707])
            c['fcl'] = screen_native_path(self.model, checked, q, opening)
            if c['fcl']:
                selected, passed = c, plan
                break
        yy, xx = np.nonzero(mask & np.isfinite(metres) & (metres > .1) & (metres < 2.5))
        rays = np.column_stack([xx, yy, np.ones(len(xx))]) @ np.linalg.inv(k).T
        optical = np.median(rays*metres[yy, xx, None], axis=0)
        observed = np.array(selected['observed_link1'])
        base_point = rlb.T @ (observed-tlb)
        row = dict(index=index, stamp_ns=self.stamp(depth_msg), rgb_stamp_ns=self.stamp(rgb),
                   optical_m=optical.tolist(), observed_link1=observed.tolist(),
                   base_point=base_point.tolist(), selected=selected, candidates=evaluated,
                   floor_link1=floor, floor_alignment=floor_report,
                   native_ik=any(c.get('execution_ik', False) for c in evaluated),
                   fcl=passed is not None, clipped=clipped,
                   relative_distance_m=float(np.linalg.norm(base_point[:2])),
                   arm_joints=q.tolist())
        self.record('OBSERVATION', **row)
        self.fresh(depth_msg, 4.)
        if np.max(np.abs(self.guard()[0]-q)) > .01:
            raise SafetyError('Arm moved during RGB-D geometry/IK')
        return row, passed, depth_msg.header.stamp

    def pulse(self, angular=0., reverse=False):
        if not self.options.execute_motion:
            raise SafetyError('Motion disabled')
        if reverse and angular:
            raise SafetyError('View recovery uses straight reverse motion only')
        self.fresh(self.rgb[-1] if self.rgb else None, .50)
        self.fresh(self.depth[-1] if self.depth else None, .50)
        turning = angular != 0
        self.guard(turning=turning, reverse=reverse)
        duration = abs(angular)/.1 if turning else 1.
        end = time.monotonic()+duration
        self.steps += 1
        self.record('PULSE', step=self.steps, angular_rad=angular,
                    nominal_translation_m=0. if turning else (-.020 if reverse else .020))
        self.last_command = time.monotonic()
        self.moving = True
        try:
            while time.monotonic() < end:
                self.guard(turning=turning, reverse=reverse)
                msg = Twist()
                msg.angular.z = math.copysign(.1, angular) if turning else 0.
                msg.linear.x = 0. if turning else (-.020 if reverse else .020)
                self.last_command = time.monotonic()
                self.pub.publish(msg)
                time.sleep(.04)
        finally:
            self.stop()
        if not turning:
            self.nominal_distance += .020
        time.sleep(.4)
        self.last_stop = self.get_clock().now().nanoseconds

    def move_observation_pose(self, pose, event):
        """Use the existing trajectory action after unchanged full-path FCL."""
        q, opening = self.guard()
        if not self.options.execute_grasp:
            raise SafetyError('Observation arm motion requires --execute-grasp')
        path = {'feasible': True, 'pregrasp': list(pose), 'descent': [], 'lift': []}
        if not screen_native_path(self.model, path, q, opening):
            return False
        if not self.view_arm.wait_for_server(timeout_sec=2):
            raise SafetyError('Existing arm action unavailable for observation')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint(positions=list(map(float, pose)))
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        accepted = self.view_arm.send_goal_async(goal)
        deadline = time.monotonic()+3
        while not accepted.done() and time.monotonic() < deadline:
            self.fresh(self.joints, .30)
            time.sleep(.02)
        if not accepted.done() or not accepted.result().accepted:
            raise SafetyError('Observation trajectory was not accepted')
        self.view_goal = accepted.result()
        result = self.view_goal.get_result_async()
        deadline = time.monotonic()+6
        try:
            while not result.done():
                self.fresh(self.joints, .30)
                if time.monotonic() > deadline:
                    raise SafetyError('Observation trajectory response timeout')
                time.sleep(.02)
            if result.result().status != 4 or result.result().result.error_code != 0:
                raise SafetyError('Observation trajectory did not complete')
        except BaseException:
            self.view_goal.cancel_goal_async()
            raise
        finally:
            if result.done():
                self.view_goal = None
        self.record(event, joints=list(map(float, pose)))
        time.sleep(.3)
        self.stop()
        return True

    def recover_view(self, reason):
        """Reuse floor/home views, then rear-checked pulses if view improves."""
        self.stop()
        q, opening = self.guard()
        view = self.view_state or {'detected': False}
        if view.get('optical_point') is not None:
            forward = self.calibration.rotation[:, 2]
            direction = self.calibration.rotation @ np.asarray(view['optical_point'])
            f, d = forward[[0, 2]], direction[[0, 2]]
            error = math.atan2(f[1]*d[0]-f[0]*d[1], float(f @ d))
            pose = q.copy()
            pose[3] = np.clip(q[3]+np.clip(error, -.20, .20),
                              self.model.lower[3]+.025, self.model.upper[3]-.025)
            key = tuple(np.round(pose/.02).astype(int))
            if abs(pose[3]-q[3]) > .015 and key not in self.view_pose_history:
                self.view_pose_history.add(key)
                if self.move_observation_pose(pose, 'ADAPTIVE_WRIST_VIEW'):
                    return
        choice = None
        if not self.view_pose_tried:
            self.view_pose_tried = True
            choice = ([0., -.523, -.523, 1.7707], '/visual_servo/observe_floor', 'FLOOR_OBSERVATION_POSE')
        elif not self.home_view_tried:
            self.home_view_tried = True
            choice = ([0., -.523, -.523, 1.5707], '/visual_servo/park_arm', 'HOME_OBSERVATION_POSE')
        if choice is not None:
            pose, service, event = choice
            if np.max(np.abs(q-np.asarray(pose))) > .01:
                if not self.options.execute_grasp:
                    raise SafetyError('Floor observation arm motion requires --execute-grasp')
                path = {'feasible': True, 'pregrasp': pose, 'descent': [], 'lift': []}
                if not screen_native_path(self.model, path, q, opening):
                    raise SafetyError('Existing floor observation pose fails FCL')
                client = self.create_client(Trigger, service)
                if not client.wait_for_service(timeout_sec=2):
                    raise SafetyError('Existing floor observation service unavailable')
                future = client.call_async(Trigger.Request())
                deadline = time.monotonic()+7
                while not future.done():
                    self.fresh(self.joints, .30)
                    if time.monotonic() > deadline:
                        raise SafetyError('Floor observation response timeout')
                    time.sleep(.03)
                if not future.result().success:
                    raise SafetyError('Floor observation: '+future.result().message)
                self.record(event, success=True, reason=reason)
                time.sleep(.3)
            self.stop()
            return
        baseline = self.reverse_view_baseline
        if baseline is not None:
            if not view.get('detected'):
                raise SafetyError('Banana still outside view after reverse; no visual recovery evidence')
            if baseline.get('detected'):
                improved = (view['center_v'] < baseline['center_v']-2 or
                            view['bottom_margin_px'] > baseline['bottom_margin_px']+2 or
                            view['supported_pixels'] > baseline['supported_pixels']*1.05)
                if not improved:
                    raise SafetyError('New RGB-D shows no camera-view improvement after reverse')
        if not self.options.execute_motion:
            raise SafetyError('View still unavailable after floor pose; base motion disabled')
        self.record('VIEW_RECOVERY_REVERSE', reason=reason, nominal_translation_m=-.020)
        self.reverse_view_baseline = dict(view)
        self.pulse(reverse=True)

    def grasp(self, obs, stamp):
        from action_msgs.srv import CancelGoal
        self.guard()
        if not self.pick.wait_for_service(timeout_sec=2):
            raise SafetyError('Camera-relative pick executor unavailable')
        target = PointStamped()
        target.header.frame_id = 'base_footprint'
        target.header.stamp = stamp
        r, t = self.transform('base_footprint', 'link1', stamp)
        p = r @ np.asarray(obs['observed_link1']) + t
        target.point.x, target.point.y, target.point.z = map(float, p)
        self.fresh(target, 4.)
        self.target_pub.publish(target)
        time.sleep(.10)
        if self.count_subscribers('/visual_servo/pick_target') != 1:
            raise SafetyError('Expected exactly one isolated target consumer')
        future = self.pick.call_async(Trigger.Request())
        self.record('PICK_REQUEST', target_base_footprint_m=p.tolist())
        deadline = time.monotonic()+75
        try:
            while not future.done():
                if time.monotonic() > deadline:
                    raise SafetyError('Pick execution timeout')
                self.guard(stationary_arm=False)
                time.sleep(.04)
            result = future.result()
            self.record('PICK_RESULT', success=result.success, message=result.message)
            if not result.success:
                raise SafetyError('Pick executor: '+result.message)
        except BaseException:
            for name in ('/arm_controller/follow_joint_trajectory', '/gripper_controller/gripper_cmd'):
                client = self.create_client(CancelGoal, name+'/_action/cancel_goal')
                if client.wait_for_service(timeout_sec=.5):
                    pending = client.call_async(CancelGoal.Request())
                    until = time.monotonic()+1
                    while not pending.done() and time.monotonic() < until:
                        time.sleep(.02)
            raise
    def run(self):
        deadline = time.monotonic()+12
        while self.scan is None or self.joints is None or self.info is None:
            if time.monotonic() > deadline:
                raise SafetyError('Sensor discovery timeout')
            time.sleep(.05)
        self.stop()
        range_anchor = bearing_anchor = None
        translation_window = rotation_window = 0.
        view_unavailable_since = None
        index = 0
        while True:
            try:
                obs, plan, stamp = self.observe(index)
            except ViewUnavailable as error:
                self.stop()
                if view_unavailable_since is None:
                    view_unavailable_since = time.monotonic()
                if time.monotonic()-view_unavailable_since < 4.:
                    continue
                self.recover_view(str(error))
                view_unavailable_since = None
                range_anchor = bearing_anchor = None
                translation_window = rotation_window = 0.
                continue
            except SafetyError as error:
                if str(error).startswith('Expected one banana, detected '):
                    self.stop()
                    self.record('REDETECT', reason=str(error))
                    continue
                raise
            view_unavailable_since = None
            if plan is None and obs['clipped']:
                self.recover_view('Clipped body view without a verified grasp path')
                range_anchor = bearing_anchor = None
                translation_window = rotation_window = 0.
                continue
            if plan is not None:
                self.record('REACHABLE', native_ik=True, fcl=True, steps=self.steps,
                            nominal_commanded_translation_m=self.nominal_distance)
                if self.options.execute_grasp:
                    self.grasp(obs, stamp)
                return
            self.view_pose_tried = False
            self.home_view_tried = False
            self.reverse_view_baseline = None
            self.view_pose_history.clear()
            if not self.options.execute_motion:
                return
            target = np.asarray(obs['base_point'])
            distance = float(np.linalg.norm(target[:2]))
            angle = math.atan2(target[1], target[0])
            if target[0] <= 0:
                raise SafetyError('Observed target is behind the base; no forward approach')
            if range_anchor is not None and translation_window:
                progress = range_anchor-distance
                self.record('VISUAL_PROGRESS', relative_range_reduction_m=progress,
                            nominal_command_window_m=translation_window)
                if progress > .005:
                    range_anchor, translation_window = distance, 0.
                elif translation_window >= .060-1e-9:
                    raise SafetyError('Persistent command/camera mismatch: no range reduction beyond 5mm noise')
            if bearing_anchor is not None and rotation_window:
                if bearing_anchor-abs(angle) > math.radians(1):
                    bearing_anchor, rotation_window = abs(angle), 0.
                elif rotation_window >= math.radians(15)-1e-9:
                    raise SafetyError('Persistent command/camera heading mismatch')
            if abs(angle) > math.radians(5):
                amount = max(-math.radians(5), min(math.radians(5), angle))
                if bearing_anchor is None:
                    bearing_anchor = abs(angle)
                self.pulse(angular=amount)
                rotation_window += abs(amount)
                range_anchor, translation_window = None, 0.
            else:
                if range_anchor is None:
                    range_anchor = distance
                self.pulse()
                translation_window += .020
                bearing_anchor, rotation_window = None, 0.
            index += 1

    def close(self):
        self.stop()
        if self.view_goal is not None:
            self.view_goal.cancel_goal_async()
        self.closed.set()
        self.watch_thread.join(timeout=1)
        self.spin_executor.shutdown(timeout_sec=2)
        self.spin_thread.join(timeout=2)
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    runtime = Path(__file__).resolve().parents[1]
    parser.add_argument('--execute-motion', action='store_true')
    parser.add_argument('--execute-grasp', action='store_true')
    parser.add_argument('--isolated-no-lidar-guards', action='store_true',
                        help='Only for an operator-confirmed clear isolated space; normal default keeps LiDAR guards')
    parser.add_argument('--output', required=True)
    parser.add_argument('--planner-root', default=str(runtime/'planner_runtime'))
    parser.add_argument('--native-probe', default=str(runtime/'native_probe_45'))
    parser.add_argument('--calibration', default='/home/user/turtlebot3_ws/install/cleanup_perception/share/cleanup_perception/config/grasp_camera_20260906.yaml')
    parser.add_argument('--yolo', default='/home/user/turtlebot3_ws/src/segmentation/best.pt')
    parser.add_argument('--sam', default='/home/user/turtlebot3_ws/src/segmentation/sam2_t.pt')
    options = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = VisualServo(options)
        node.run()
    except BaseException as error:
        if node is not None:
            node.record('STOP', reason=str(error), steps=node.steps,
                        nominal_commanded_translation_m=node.nominal_distance)
        raise
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
